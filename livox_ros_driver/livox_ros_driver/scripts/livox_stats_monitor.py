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
import json
import math
import re
import sys
import textwrap
import threading
import time

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

_WIRE_SCHEMA_VERSION = 1
_DRIVER_STALE_SECONDS = 5.0  # Driver publishes at 1 Hz.
_MANAGER_STALE_SECONDS = 30.0  # Manager heartbeat publishes every 10 s.
_MAX_STATS_BYTES = 1024 * 1024
_MAX_POWER_JSON_BYTES = 64 * 1024
_BROADCAST_CODE_RE = re.compile(r"^[A-Za-z0-9]{15}$")
_POWER_GROUP_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]{0,63}$")
_STATE_RE = re.compile(r"^[A-Z][A-Z0-9_]{0,63}$")
_SEVERITIES = {"INFO", "WARN", "ERROR", "CRITICAL"}


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


def _compose_dashboard(stats_text, stats_received_mono, rows, now_mono):
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
        "==================== SOURCE HEALTH ==================",
        "  DRIVER   NOW=%s  severity=%s  driver_age=%s  expected=1Hz stale>5s"
        % (
            driver_state,
            driver_severity,
            _age_text_from_seconds(driver_age),
        ),
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
    lines.extend(["", "(local refresh; liveness ages use monotonic time)"])
    return "\n".join(lines) + "\n"


def _render():
    with _render_lock:
        now_mono = time.monotonic()
        with _lock:
            stats_text = _stats_text
            stats_received_mono = _stats_received_mono
            rows = [dict(item) for item in _power_status.values()]
        output = _compose_dashboard(
            stats_text, stats_received_mono, rows, now_mono
        )
        # ESC[2J = clear screen, ESC[H = cursor to home (top-left)
        sys.stdout.write("\033[2J\033[H")
        sys.stdout.write(output)
        sys.stdout.flush()


def cb(msg):
    global _stats_text, _stats_received_mono
    data = getattr(msg, "data", None)
    if not isinstance(data, str) or not data or len(data) > _MAX_STATS_BYTES:
        return False
    received_mono = time.monotonic()
    with _lock:
        _stats_text = data
        _stats_received_mono = received_mono
        _source_received_mono["driver_stats"] = received_mono
    _render()
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


def main():
    if rospy is None or String is None:
        raise RuntimeError(
            "ROS Python modules unavailable; source the ROS environment first"
        )
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
