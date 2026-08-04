#!/usr/bin/env python3
#
# Livox LiDAR stats dashboard.
#
# Run in a SEPARATE terminal from the driver. It subscribes to the
# /livox/lidar_stats topic published by livox_ros_driver and redraws the
# table in place (clears the screen each update), so it stays pinned and
# is completely isolated from the driver's scrolling log output.
#
#   python3 livox_stats_monitor.py
#   # or, after a catkin build with the install rule:
#   rosrun livox_ros_driver livox_stats_monitor.py
#
import argparse
import json
import math
import os
import re
import sqlite3
import sys
import textwrap
import threading
import time
from contextlib import closing
from pathlib import Path

try:
    import rospy
    from std_msgs.msg import String
except ImportError:  # Keep validation/render helpers testable without ROS.
    rospy = None
    String = None


_lock = threading.Lock()
_render_lock = threading.Lock()
_stats_text = "Waiting for /livox/lidar_stats ...\n"
_stats_received_mono = None
_power_status = {}
_source_received_mono = {
    "driver_stats": None,
    "power_status": None,
    "power_heartbeat": None,
}
_layout = "compact"
_history_shutdown_requested = False

_WIRE_SCHEMA_VERSION = 1
_DRIVER_STALE_SECONDS = 5.0  # Driver publishes at 1 Hz.
_MANAGER_STALE_SECONDS = 30.0  # Manager heartbeat publishes every 10 s.
_MAX_STATS_BYTES = 1024 * 1024
_MAX_POWER_JSON_BYTES = 64 * 1024
_RELAY_HISTORY_LIMIT = 5
_DEFAULT_RELAY_HISTORY_DB = os.path.expanduser(
    "~/.local/state/livox-power-cycle-manager/state.sqlite3"
)
_BROADCAST_CODE_RE = re.compile(r"^[A-Za-z0-9]{15}$")
_POWER_GROUP_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]{0,63}$")
_STATE_RE = re.compile(r"^[A-Z][A-Z0-9_]{0,63}$")
_SEVERITIES = {"INFO", "WARN", "ERROR", "CRITICAL"}
_SEVERITY_RANK = {"INFO": 0, "WARN": 1, "ERROR": 2, "CRITICAL": 3}
_LAYOUTS = ("compact", "full", "history")


def _finite_number(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    try:
        number = float(value)
    except (TypeError, ValueError, OverflowError):
        return None
    return number if math.isfinite(number) else None


def _required_text(payload, name, maximum, allow_empty=True):
    value = payload.get(name)
    if not isinstance(value, str) or len(value) > maximum:
        return None
    if not allow_empty and not value:
        return None
    return value


def _safe_inline(value):
    """Remove terminal control characters and collapse detail to one line."""
    return " ".join(
        "".join(char if char.isprintable() else " " for char in value).split()
    )


def _decode_power_payload(data, received_mono, source):
    """Return one validated cache row, or None for any malformed message."""
    if source not in {"power_status", "power_heartbeat"}:
        return None
    if not isinstance(data, str) or not data or len(data) > _MAX_POWER_JSON_BYTES:
        return None
    received = _finite_number(received_mono)
    if received is None:
        return None
    try:
        payload = json.loads(data)
    except (TypeError, ValueError, json.JSONDecodeError, RecursionError):
        return None
    if not isinstance(payload, dict):
        return None
    schema_version = payload.get("schema_version")
    if (
        isinstance(schema_version, bool)
        or not isinstance(schema_version, int)
        or schema_version != _WIRE_SCHEMA_VERSION
        or payload.get("type") != "POWER_CYCLE_STATUS"
    ):
        return None
    timestamp = _finite_number(payload.get("timestamp"))
    if timestamp is None or timestamp < 0:
        return None

    state = _required_text(payload, "state", 64, allow_empty=False)
    severity = _required_text(payload, "severity", 16, allow_empty=False)
    event_id = _required_text(payload, "event_id", 512)
    broadcast_code = _required_text(payload, "broadcast_code", 64)
    power_group = _required_text(payload, "power_group", 64)
    label = _required_text(payload, "label", 256)
    detail = _required_text(payload, "detail", 4096)
    members = payload.get("members")
    relay_channels = payload.get("relay_channels", [])
    if (
        state is None
        or _STATE_RE.fullmatch(state) is None
        or severity not in _SEVERITIES
        or event_id is None
        or broadcast_code is None
        or power_group is None
        or label is None
        or detail is None
        or not isinstance(members, list)
        or len(members) > 64
        or not isinstance(relay_channels, list)
        or len(relay_channels) > 4
    ):
        return None
    if broadcast_code and _BROADCAST_CODE_RE.fullmatch(broadcast_code) is None:
        return None
    if power_group and _POWER_GROUP_RE.fullmatch(power_group) is None:
        return None
    if any(
        not isinstance(member, str)
        or _BROADCAST_CODE_RE.fullmatch(member) is None
        for member in members
    ):
        return None
    if any(
        isinstance(channel, bool)
        or not isinstance(channel, int)
        or channel < 1
        or channel > 4
        for channel in relay_channels
    ) or len(relay_channels) != len(set(relay_channels)):
        return None
    # A true manager row has neither a group nor a trigger. An empty group
    # with a trigger is deliberately retained as an UNMAPPED device event.
    if not power_group and not broadcast_code and state not in {
        "MANAGER_HEARTBEAT",
        "MANAGER_READY",
    }:
        return None

    row = {
        "timestamp": timestamp,
        "state": state,
        "severity": severity,
        "event_id": event_id,
        "broadcast_code": broadcast_code,
        "power_group": power_group,
        "members": list(members),
        "relay_channels": sorted(relay_channels),
        "label": label,
        "detail": detail,
        "_received_mono": received,
        "_source": source,
    }
    return row


def _power_row_key(row):
    power_group = row["power_group"]
    broadcast_code = row["broadcast_code"]
    if power_group:
        return "group:%s" % power_group
    if broadcast_code:
        return "unmapped:%s" % broadcast_code
    return "__manager__"


def _elapsed_seconds(received_mono, now_mono):
    received = _finite_number(received_mono)
    now = _finite_number(now_mono)
    if received is None or now is None:
        return None
    return max(0.0, now - received)


def _age_text_from_seconds(age):
    if age is None or not math.isfinite(age):
        return "--"
    age = max(0.0, age)
    if age < 60:
        return "%ds" % int(age)
    if age < 3600:
        return "%dm%02ds" % (int(age) // 60, int(age) % 60)
    return "%dh%02dm" % (int(age) // 3600, (int(age) % 3600) // 60)


def _append_detail(lines, detail):
    safe_detail = _safe_inline(detail)
    if not safe_detail:
        return
    lines.extend(
        textwrap.wrap(
            safe_detail,
            width=132,
            initial_indent="    detail: ",
            subsequent_indent="            ",
        )
    )


def _relay_history_db_path():
    override = os.environ.get("LIVOX_POWER_CYCLE_STATE_DB")
    return os.path.abspath(os.path.expanduser(override or _DEFAULT_RELAY_HISTORY_DB))


def _finite_timestamp(value):
    number = _finite_number(value)
    if number is None or number < 0:
        return None
    return number


def _read_relay_history(path=None, limit=_RELAY_HISTORY_LIMIT):
    """Read a bounded, validated history snapshot without creating the DB."""
    db_path = os.path.abspath(os.path.expanduser(path or _relay_history_db_path()))
    if not os.path.isfile(db_path):
        return [], None
    if isinstance(limit, bool) or not isinstance(limit, int) or not 1 <= limit <= 20:
        return [], "invalid relay history limit"
    try:
        uri = Path(db_path).resolve().as_uri() + "?mode=ro"
        with closing(sqlite3.connect(uri, uri=True, timeout=0.2)) as db:
            db.execute("PRAGMA query_only=ON")
            rows = db.execute(
                "SELECT c.id,c.started_at,c.trigger_bcode,c.group_id,"
                "c.off_confirmed_at,c.on_confirmed_at,c.outcome,c.detail,"
                "COALESCE(e.recovery_reason,'UNKNOWN') "
                "FROM power_cycles AS c LEFT JOIN power_events AS e "
                "ON e.event_id=c.event_id ORDER BY c.id DESC LIMIT ?",
                (limit,),
            ).fetchall()
    except (OSError, sqlite3.Error) as exc:
        return [], _safe_inline(str(exc))[:240] or "unknown SQLite error"

    history = []
    for row in rows:
        if not isinstance(row, tuple) or len(row) != 9:
            return [], "relay history row has an invalid shape"
        (
            cycle_id,
            started_at,
            trigger,
            group_id,
            off_confirmed_at,
            on_confirmed_at,
            outcome,
            detail,
            reason,
        ) = row
        if (
            isinstance(cycle_id, bool)
            or not isinstance(cycle_id, int)
            or cycle_id <= 0
            or _finite_timestamp(started_at) is None
            or not isinstance(trigger, str)
            or _BROADCAST_CODE_RE.fullmatch(trigger) is None
            or not isinstance(group_id, str)
            or _POWER_GROUP_RE.fullmatch(group_id) is None
            or not isinstance(outcome, str)
            or _STATE_RE.fullmatch(outcome) is None
            or not isinstance(detail, str)
            or len(detail) > 1000
            or not isinstance(reason, str)
            or (reason != "UNKNOWN" and _STATE_RE.fullmatch(reason) is None)
        ):
            return [], "relay history contains an invalid value"
        off_time = None if off_confirmed_at is None else _finite_timestamp(off_confirmed_at)
        on_time = None if on_confirmed_at is None else _finite_timestamp(on_confirmed_at)
        if (off_confirmed_at is not None and off_time is None) or (
            on_confirmed_at is not None and on_time is None
        ):
            return [], "relay history contains an invalid confirmation time"
        history.append(
            {
                "id": cycle_id,
                "started_at": float(started_at),
                "trigger": trigger,
                "group_id": group_id,
                "off_confirmed_at": off_time,
                "on_confirmed_at": on_time,
                "outcome": outcome,
                "detail": detail,
                "reason": reason,
            }
        )
    return history, None


def _wall_time_text(timestamp):
    try:
        return time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(timestamp))
    except (ValueError, OverflowError, OSError):
        return "invalid-time"


def _append_relay_history(lines, history, error):
    lines.extend(
        [
            "",
            "==================== RELAY HISTORY ==================",
            "  (latest 5 persisted cycles; survives Driver/monitor restart)",
        ]
    )
    if error:
        lines.append("  unavailable: %s" % _safe_inline(error))
        return
    if not history:
        lines.append("  no relay cycle has been recorded")
        return
    for item in history:
        lines.append(
            "  %s  trigger=%s  reason=%s  group=%s"
            % (
                _wall_time_text(item["started_at"]),
                item["trigger"],
                item["reason"],
                item["group_id"],
            )
        )
        lines.append(
            "    OFF=%s  ON=%s  outcome=%s"
            % (
                "YES" if item["off_confirmed_at"] is not None else "--",
                "YES" if item["on_confirmed_at"] is not None else "--",
                item["outcome"],
            )
        )
        detail = _safe_inline(item["detail"])
        if detail:
            lines.extend(
                textwrap.wrap(
                    detail,
                    width=132,
                    initial_indent="    detail: ",
                    subsequent_indent="            ",
                )
            )


def _split_driver_sections(stats_text):
    """Split the Driver's human-readable snapshot into named sections."""
    sections = {}
    order = []
    preamble = []
    current = None
    for line in (stats_text or "").rstrip("\n").splitlines():
        if line.startswith("==================== "):
            current = line.strip().strip("=").strip()
            if current not in sections:
                sections[current] = []
                order.append(current)
            continue
        if current is None:
            preamble.append(line)
        else:
            sections[current].append(line)
    return preamble, sections, order


def _fit_cell(value, width):
    text = _safe_inline(str(value))
    if len(text) <= width:
        return text
    if width <= 1:
        return text[:width]
    return text[: width - 1] + "~"


def _parse_device_rows(lines):
    rows = []
    for line in lines:
        fields = line.split()
        if (
            len(fields) >= 8
            and fields[0].isdigit()
            and _BROADCAST_CODE_RE.fullmatch(fields[1]) is not None
        ):
            rows.append(
                {
                    "id": fields[0],
                    "broadcast_code": fields[1],
                    "state": fields[2],
                    "assess": fields[3],
                    "points": fields[4],
                    "hardware": fields[5],
                    "connected": fields[6],
                    "disconnects": fields[7],
                }
            )
    return rows


def _parse_recent_rows(lines):
    rows = {}
    for line in lines:
        fields = line.split()
        if (
            len(fields) >= 5
            and fields[0].isdigit()
            and _BROADCAST_CODE_RE.fullmatch(fields[1]) is not None
        ):
            rows[fields[1]] = {
                "loss": fields[2],
                "queue_drop": fields[3],
                "handshake": fields[4],
            }
    return rows


def _parse_measurement_rows(lines):
    rows = {}
    current = None
    for line in lines:
        header = re.match(r"^\s*L(\d+)\s+([A-Za-z0-9]{15})\s*$", line)
        if header:
            current = {
                "id": header.group(1),
                "broadcast_code": header.group(2),
                "session": "--",
                "error_reboots": "--",
                "point_cloud": "--",
                "last_recovery": "--",
                "first_data": "",
                "next": "--",
            }
            rows[current["broadcast_code"]] = current
            continue
        if current is None:
            continue
        stripped = line.strip()
        if stripped.startswith("MEASUREMENT SESSION:"):
            current["session"] = stripped.split(":", 1)[1].strip()
        elif stripped.startswith("ERROR REBOOTS:"):
            current["error_reboots"] = stripped.split(":", 1)[1].strip()
        elif stripped.startswith("POINT-CLOUD:"):
            current["point_cloud"] = stripped.split(":", 1)[1].strip()
        elif stripped.startswith("LAST RECOVERY:"):
            current["last_recovery"] = stripped.split(":", 1)[1].strip()
        elif "first data returned=" in stripped:
            current["first_data"] = stripped
        elif stripped.startswith("NEXT ESCALATION:"):
            current["next"] = stripped.split(":", 1)[1].strip()
    return rows


def _parse_alert_rows(lines):
    rows = {}
    for line in lines:
        match = re.match(
            r"^\s*\[(CRIT|ALERT|RECOVER)\]\s+L(\d+)\s+"
            r"([A-Za-z0-9]{15})\s+(.*)$",
            line,
        )
        if not match:
            continue
        rows.setdefault(match.group(3), []).append(
            "[%s] %s" % (match.group(1), _safe_inline(match.group(4)))
        )
    return rows


def _parse_history_blocks(lines):
    blocks = {}
    current = None
    for line in lines:
        header = re.match(r"^\s*L(\d+)\s+([A-Za-z0-9]{15}):\s*$", line)
        if header:
            current = {
                "id": header.group(1),
                "broadcast_code": header.group(2),
                "lines": [],
            }
            blocks[current["broadcast_code"]] = current
            continue
        if current is not None and line.strip():
            current["lines"].append(line.strip())
    return blocks


def _history_summary(block):
    if not block:
        return ""
    text = "\n".join(block.get("lines", []))
    tokens = []
    patterns = (
        ("disc", r"disconnect episodes=(\d+)"),
        ("pc", r"point-cloud outages=(\d+)"),
        ("hs", r"handshake failure episodes: stuck=(\d+)"),
        ("wake", r"wake dropout episodes=(\d+)"),
        ("normal", r"normal-dropout episodes=(\d+)"),
        ("power", r"POWER_CYCLE_REQUIRED: episodes=(\d+)"),
        ("fault", r"hardware fault episodes=(\d+)"),
        ("reboot", r"automatic reboot actions=(\d+)"),
        ("mode", r"mode failures: total=(\d+)"),
        ("relay", r"planned shared power cycles=(\d+)"),
    )
    for label, pattern in patterns:
        match = re.search(pattern, text)
        if match and int(match.group(1)) != 0:
            tokens.append("%s=%s" % (label, match.group(1)))
    if "point-cloud outages=" in text and "current=ACTIVE" in text:
        tokens.append("pc=ACTIVE")
    return "hist " + ",".join(tokens) if tokens else "history present"


def _compact_measurement(row):
    if not row:
        return {
            "session": "--",
            "error": "--",
            "point": "--",
            "last": "--",
            "next": "--",
        }
    session = row.get("session", "--").split()[0]
    error_match = re.search(r"\b\d+/\d+\b", row.get("error_reboots", ""))
    error = error_match.group(0) if error_match else "--"
    point = row.get("point_cloud", "--").split(";", 1)[0].strip()
    last_text = row.get("last_recovery", "--")
    if last_text.startswith("none"):
        last = "--"
    else:
        last = last_text.split()[0] if last_text else "--"
        first_match = re.search(
            r"first data returned=\d{4}-\d{2}-\d{2} (\d{2}:\d{2}:\d{2})",
            row.get("first_data", ""),
        )
        if first_match and last != "--":
            last = "%s@%s" % (last, first_match.group(1))
    next_text = row.get("next", "")
    reboot_match = re.search(r"soft reboot (\d+/\d+)", next_text)
    if "already active" in next_text and "POWER_CYCLE_REQUIRED" in next_text:
        next_action = "POWER ACTIVE"
    elif "POWER_CYCLE_REQUIRED" in next_text:
        next_action = "POWER next"
    elif "starts a session" in next_text:
        next_action = "start->1/3"
    elif reboot_match:
        next_action = "soft %s" % reboot_match.group(1)
    else:
        next_action = next_text or "--"
    return {
        "session": session,
        "error": error,
        "point": point,
        "last": last,
        "next": next_action,
    }


def _software_summary(sections):
    line = next(
        (
            item.strip()
            for item in sections.get("SOFTWARE", [])
            if "Driver commit=" in item
        ),
        "",
    )
    driver = re.search(r"Driver commit=([^\s]+)", line)
    sdk = re.search(r"paired SDK commit=([^\s]+)", line)
    compatibility = re.search(r"compatibility=([^\s]+)", line)
    return {
        "driver": (driver.group(1)[:7] if driver else "unknown"),
        "sdk": (sdk.group(1)[:7] if sdk else "unknown"),
        "compatibility": compatibility.group(1) if compatibility else "--",
    }


def _manager_compact_summary(rows, now_mono):
    manager = next(
        (
            row
            for row in rows
            if not row["power_group"] and not row["broadcast_code"]
        ),
        None,
    )
    if manager is None:
        return {
            "state": "NOT_SEEN",
            "age": "--",
            "detail": "manager disabled, starting, or failed",
        }
    age = _elapsed_seconds(manager["_received_mono"], now_mono)
    if age is None or age > _MANAGER_STALE_SECONDS:
        return {
            "state": "STALE",
            "age": _age_text_from_seconds(age),
            "detail": "heartbeat missing severity=CRITICAL",
        }
    detail = _safe_inline(manager.get("detail", ""))
    mode = re.search(r"\bmode=([A-Za-z0-9_-]+)", detail)
    state = mode.group(1).upper() if mode else manager["state"]
    return {
        "state": state,
        "age": _age_text_from_seconds(age),
        "detail": detail or manager["state"],
    }


def _compact_power_lines(rows, now_mono):
    manager = _manager_compact_summary(rows, now_mono)
    lines = [
        _fit_cell(
            "POWER-MGR: state=%s age=%s %s"
            % (manager["state"], manager["age"], manager["detail"]),
            138,
        )
    ]
    events = [
        row
        for row in rows
        if row["power_group"] or row["broadcast_code"]
    ]
    if not events:
        lines.append("POWER-EVENT: none")
        return lines
    events.sort(
        key=lambda item: (
            _SEVERITY_RANK.get(item["severity"], -1),
            item["_received_mono"],
        ),
        reverse=True,
    )
    event = events[0]
    age = _age_text_from_seconds(
        _elapsed_seconds(event["_received_mono"], now_mono)
    )
    group = event["power_group"] or "UNMAPPED"
    relay = ",".join(str(item) for item in event["relay_channels"]) or "-"
    event_line = (
        "POWER-EVENT: group=%s state=%s severity=%s age=%s trigger=%s relay=%s"
        % (
            group,
            event["state"],
            event["severity"],
            age,
            event["broadcast_code"] or "-",
            relay,
        )
    )
    if len(events) > 1:
        event_line += " +%d more" % (len(events) - 1)
    lines.append(_fit_cell(event_line, 138))
    return lines


def _compact_relay_line(history, error):
    if error:
        return _fit_cell("RELAY: unavailable: %s" % error, 138)
    if not history:
        return "RELAY: last cycle=none"
    item = history[0]
    return _fit_cell(
        "RELAY: last=%s reason=%s outcome=%s OFF=%s ON=%s trigger=%s"
        % (
            _wall_time_text(item["started_at"]),
            item["reason"],
            item["outcome"],
            "YES" if item["off_confirmed_at"] is not None else "--",
            "YES" if item["on_confirmed_at"] is not None else "--",
            item["trigger"],
        ),
        138,
    )


def _compose_compact_dashboard(
    stats_text,
    stats_received_mono,
    rows,
    now_mono,
    relay_history=(),
    relay_history_error=None,
):
    driver_age = _elapsed_seconds(stats_received_mono, now_mono)
    if stats_received_mono is None:
        driver_state = "WAITING"
    elif driver_age is None or driver_age > _DRIVER_STALE_SECONDS:
        driver_state = "STALE"
    else:
        driver_state = "LIVE"

    _preamble, sections, _order = _split_driver_sections(stats_text)
    devices = _parse_device_rows(sections.get("CURRENT DEVICES", []))
    recent = _parse_recent_rows(sections.get("RECENT 60 SECONDS", []))
    measurement = _parse_measurement_rows(
        sections.get("MEASUREMENT RECOVERY", [])
    )
    alerts = _parse_alert_rows(sections.get("CURRENT ALERTS", []))
    history = _parse_history_blocks(sections.get("PROCESS HISTORY", []))
    software = _software_summary(sections)
    manager = _manager_compact_summary(rows, now_mono)
    alert_count = sum(len(items) for items in alerts.values())

    lines = [
        _fit_cell(
            "LIVOX | Driver=%s(%s) | PowerMgr=%s(%s) | Driver=%s SDK=%s %s "
            "| devices=%d alerts=%d"
            % (
                driver_state,
                _age_text_from_seconds(driver_age),
                manager["state"],
                manager["age"],
                software["driver"],
                software["sdk"],
                software["compatibility"],
                len(devices),
                alert_count,
            ),
            138,
        ),
        "==================== CURRENT DEVICES ==================",
        "ID   broadcast_code   CURRENT               ASSESS       points/s  HW          connected  disc",
    ]
    if not devices:
        lines.append("--   waiting for a valid Driver snapshot")
    for device in devices:
        lines.append(
            "{:<3}  {:<15}  {:<20}  {:<10}  {:>8}  {:<10}  {:>9}  {:>4}".format(
                _fit_cell(device["id"], 3),
                _fit_cell(device["broadcast_code"], 15),
                _fit_cell(device["state"], 20),
                _fit_cell(device["assess"], 10),
                _fit_cell(device["points"], 8),
                _fit_cell(device["hardware"], 10),
                _fit_cell(device["connected"], 9),
                _fit_cell(device["disconnects"], 4),
            )
        )

    lines.extend(
        [
            "==================== RECOVERY / RECENT =================",
            "ID   net_loss  queue_drop  handshake  session   Error    point-cloud   last-recovery       next-error",
        ]
    )
    for device in devices:
        code = device["broadcast_code"]
        recent_row = recent.get(
            code, {"loss": "--", "queue_drop": "--", "handshake": "--"}
        )
        recovery = _compact_measurement(measurement.get(code))
        lines.append(
            "{:<3}  {:>8}  {:>10}  {:>9}  {:<8}  {:>7}  {:<12}  {:<18}  {:<12}".format(
                _fit_cell(device["id"], 3),
                _fit_cell(recent_row["loss"], 8),
                _fit_cell(recent_row["queue_drop"], 10),
                _fit_cell(recent_row["handshake"], 9),
                _fit_cell(recovery["session"], 8),
                _fit_cell(recovery["error"], 7),
                _fit_cell(recovery["point"], 12),
                _fit_cell(recovery["last"], 18),
                _fit_cell(recovery["next"], 12),
            )
        )

    lines.extend(
        [
            "==================== ACTION / HISTORY ==================",
            "ID   broadcast_code   current action / process history",
        ]
    )
    seen_codes = set()
    for device in devices:
        code = device["broadcast_code"]
        seen_codes.add(code)
        action_parts = list(alerts.get(code, []))
        summary = _history_summary(history.get(code))
        if summary:
            action_parts.append(summary)
        action = " | ".join(action_parts) if action_parts else "none"
        lines.append(
            "{:<3}  {:<15}  {}".format(
                _fit_cell(device["id"], 3),
                code,
                _fit_cell(action, 112),
            )
        )
    for code, alert_rows in alerts.items():
        if code in seen_codes:
            continue
        lines.append(
            "---  {:<15}  {}".format(code, _fit_cell(" | ".join(alert_rows), 112))
        )

    lines.extend(_compact_power_lines(rows, now_mono))
    lines.extend(
        [
            _compact_relay_line(relay_history, relay_history_error),
            "ASSESS: ACTIVE=fault RECOVERING=repair IDLE=low-power "
            "OBSERVE=<10m STABLE=>=10m WATCH/UNSTABLE=trend",
            "RECOVERY: last=duration@first-data; handshake=SDK attempts/60s; "
            "details: --layout full; one-shot history: --layout history",
        ]
    )
    return "\n".join(lines) + "\n"


def _history_stats_text(stats_text):
    _preamble, sections, _order = _split_driver_sections(stats_text)
    selected = []
    for name in (
        "SOFTWARE",
        "CURRENT ALERTS",
        "MEASUREMENT RECOVERY",
        "PROCESS HISTORY",
    ):
        selected.append("==================== %s ====================" % name)
        content = sections.get(name, [])
        if content:
            selected.extend(content)
        elif name == "PROCESS HISTORY":
            selected.append("  none in this Driver process")
        else:
            selected.append("  unavailable")
    return "\n".join(selected) + "\n"


def _compose_full_dashboard(
    stats_text,
    stats_received_mono,
    rows,
    now_mono,
    relay_history=(),
    relay_history_error=None,
):
    """Pure formatter. All liveness ages use the caller's monotonic clock."""
    driver_age = _elapsed_seconds(stats_received_mono, now_mono)
    if stats_received_mono is None:
        driver_state = "WAITING"
        driver_severity = "WARN"
    elif driver_age is None or driver_age > _DRIVER_STALE_SECONDS:
        driver_state = "DRIVER_STALE"
        driver_severity = "CRITICAL"
    else:
        driver_state = "LIVE"
        driver_severity = "INFO"

    lines = [
        "==================== DATA SOURCE ====================",
        "  DRIVER   NOW=%s  severity=%s  driver_age=%s  expected=1Hz stale>5s"
        % (
            driver_state,
            driver_severity,
            _age_text_from_seconds(driver_age),
        ),
        "  LIVE=realtime; DRIVER_STALE=the sections below are the last snapshot",
    ]
    manager_row = next(
        (
            row
            for row in rows
            if not row["power_group"] and not row["broadcast_code"]
        ),
        None,
    )
    if manager_row is not None:
        manager_age = _elapsed_seconds(
            manager_row["_received_mono"], now_mono
        )
        manager_state = manager_row["state"]
        manager_severity = manager_row["severity"]
        if manager_age is None or manager_age > _MANAGER_STALE_SECONDS:
            manager_state = "MANAGER_STALE"
            manager_severity = "CRITICAL"
        lines.append(
            "  POWER-MGR NOW=%s  severity=%s  manager_age=%s  heartbeat=10s stale>30s"
            % (
                manager_state,
                manager_severity,
                _age_text_from_seconds(manager_age),
            )
        )
    else:
        lines.append(
            "  POWER-MGR NOW=NOT_SEEN  severity=WARN  manager_age=--  "
            "disabled, starting, or failed before first heartbeat"
        )
    lines.append("")
    if isinstance(stats_text, str) and stats_text:
        lines.extend(stats_text.rstrip("\n").splitlines())
    else:
        lines.append("Waiting for /livox/lidar_stats ...")

    if rows:
        lines.extend(
            [
                "",
                "==================== POWER RECOVERY ================",
                "  (shared relay; separate manager process)",
            ]
        )
        sorted_rows = sorted(
            rows,
            key=lambda item: (
                item["power_group"],
                item["broadcast_code"],
                item["state"],
            ),
        )
        for row in sorted_rows:
            power_group = row["power_group"]
            trigger = row["broadcast_code"] or "-"
            age = _elapsed_seconds(row["_received_mono"], now_mono)
            age_text = _age_text_from_seconds(age)
            state = row["state"]
            severity = row["severity"]
            detail = row["detail"]
            if not power_group and not row["broadcast_code"]:
                if age is None or age > _MANAGER_STALE_SECONDS:
                    state = "MANAGER_STALE"
                    severity = "CRITICAL"
                    detail = "manager heartbeat missing; " + detail
                lines.append(
                    "  MANAGER   NOW=%s  severity=%s  manager_age=%s"
                    % (state, severity, age_text)
                )
            elif not power_group:
                lines.append("  UNMAPPED  trigger=%s" % trigger)
                lines.append(
                    "    NOW=%s  severity=%s  rx_age=%s"
                    % (state, severity, age_text)
                )
            else:
                lines.append("  GROUP %s" % power_group)
                lines.append(
                    "    NOW=%s  severity=%s  rx_age=%s  trigger=%s  members=%d  "
                    "relay=%s"
                    % (
                        state,
                        severity,
                        age_text,
                        trigger,
                        len(row["members"]),
                        ",".join(
                            str(channel) for channel in row["relay_channels"]
                        )
                        or "-",
                    )
                )
            _append_detail(lines, detail)
    _append_relay_history(lines, relay_history, relay_history_error)
    lines.extend(["", "(local refresh; liveness ages use monotonic time)"])
    return "\n".join(lines) + "\n"


def _compose_dashboard(
    stats_text,
    stats_received_mono,
    rows,
    now_mono,
    relay_history=(),
    relay_history_error=None,
    layout="compact",
):
    if layout not in _LAYOUTS:
        raise ValueError("unknown dashboard layout: %s" % layout)
    if layout == "compact":
        return _compose_compact_dashboard(
            stats_text,
            stats_received_mono,
            rows,
            now_mono,
            relay_history,
            relay_history_error,
        )
    if layout == "history":
        stats_text = _history_stats_text(stats_text)
    return _compose_full_dashboard(
        stats_text,
        stats_received_mono,
        rows,
        now_mono,
        relay_history,
        relay_history_error,
    )


def _render():
    with _render_lock:
        now_mono = time.monotonic()
        with _lock:
            stats_text = _stats_text
            stats_received_mono = _stats_received_mono
            rows = [dict(item) for item in _power_status.values()]
        relay_history, relay_history_error = _read_relay_history()
        output = _compose_dashboard(
            stats_text,
            stats_received_mono,
            rows,
            now_mono,
            relay_history,
            relay_history_error,
            layout=_layout,
        )
        # ESC[2J = clear screen, ESC[H = cursor to home (top-left)
        sys.stdout.write("\033[2J\033[H")
        sys.stdout.write(output)
        sys.stdout.flush()


def cb(msg):
    global _stats_text, _stats_received_mono, _history_shutdown_requested
    data = getattr(msg, "data", None)
    if not isinstance(data, str) or not data or len(data) > _MAX_STATS_BYTES:
        return False
    received_mono = time.monotonic()
    with _lock:
        _stats_text = data
        _stats_received_mono = received_mono
        _source_received_mono["driver_stats"] = received_mono
    _render()
    if (
        _layout == "history"
        and not _history_shutdown_requested
        and rospy is not None
        and hasattr(rospy, "signal_shutdown")
    ):
        _history_shutdown_requested = True
        rospy.signal_shutdown("one-shot history rendered")
    return True


def _accept_power_message(msg, source):
    received_mono = time.monotonic()
    row = _decode_power_payload(
        getattr(msg, "data", None), received_mono, source
    )
    if row is None:
        return False
    key = _power_row_key(row)
    with _lock:
        _power_status[key] = row
        _source_received_mono[source] = received_mono
    _render()
    return True


def power_cb(msg):
    """Compatibility callback for the status topic."""
    return _accept_power_message(msg, "power_status")


def power_heartbeat_cb(msg):
    return _accept_power_message(msg, "power_heartbeat")


def _refresh_cb(_event):
    # This local timer keeps stale indicators moving even if every publisher
    # has stopped and therefore no ROS subscription callback is firing.
    _render()


def _parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Livox fixed-height operations dashboard"
    )
    parser.add_argument(
        "--layout",
        choices=_LAYOUTS,
        default="compact",
        help=(
            "compact=fixed-height live view (default); full=all diagnostics; "
            "history=one-shot recovery/process/relay history"
        ),
    )
    return parser.parse_args(argv)


def main(argv=None):
    global _layout, _history_shutdown_requested
    if rospy is None or String is None:
        raise RuntimeError(
            "ROS Python modules unavailable; source the ROS environment first"
        )
    if argv is None:
        ros_argv = (
            rospy.myargv(argv=sys.argv)
            if hasattr(rospy, "myargv")
            else [sys.argv[0]]
        )
        argv = ros_argv[1:]
    args = _parse_args(argv)
    _layout = args.layout
    _history_shutdown_requested = False
    rospy.init_node("livox_stats_monitor", anonymous=True)
    rospy.Subscriber("livox/lidar_stats", String, cb, queue_size=1)
    rospy.Subscriber(
        "livox/power_cycle_status", String, power_cb, queue_size=32
    )
    rospy.Subscriber(
        "livox/power_cycle_heartbeat",
        String,
        power_heartbeat_cb,
        queue_size=8,
    )
    refresh_timer = rospy.Timer(rospy.Duration(1.0), _refresh_cb)
    _render()
    try:
        rospy.spin()
    finally:
        refresh_timer.shutdown()


if __name__ == "__main__":
    main()
