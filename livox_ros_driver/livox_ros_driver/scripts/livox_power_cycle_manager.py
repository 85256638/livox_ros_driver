#!/usr/bin/env python3
"""Fail-safe CORX relay power-cycle manager for Livox LiDAR recovery.

The Livox driver publishes a POWER_CYCLE_REQUIRED event only after its bounded
local handshake recovery is exhausted.  This independent ROS node validates an
explicit power-group whitelist (one or more broadcast codes sharing one relay
channel), applies persistent group-level rate limits, confirms every relay
state transition, and verifies that every group member resumes point-cloud
publication after power is restored.

No third-party Python package is required.  ROS imports are deliberately kept
inside ``run_ros`` so configuration, relay and persistence tests can run on a
plain Python 3.8 installation.
"""

from __future__ import annotations

import argparse
from collections import deque
from contextlib import contextmanager
import hashlib
import ipaddress
import json
import math
import os
import queue
import re
import socket
import sqlite3
import sys
import threading
import time
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any, Callable, Dict, List, Mapping, Optional, Sequence, Tuple


WIRE_SCHEMA_VERSION = 1
CONFIG_SCHEMA_VERSION = 2
STATE_DB_SCHEMA_VERSION = 3
REQUEST_TYPE = "POWER_CYCLE_REQUIRED"
STATE_TYPE = "LIDAR_RECOVERY_STATE"
STATUS_TYPE = "POWER_CYCLE_STATUS"
TERMINAL_EVENT_STATES = {
    "RECOVERY_VERIFIED",
    "RECOVERY_TIMEOUT",
    "UNMAPPED",
    "TARGET_DISABLED",
    "STALE_OR_RECOVERED",
    "SUPPRESSED_COOLDOWN",
    "SUPPRESSED_DAILY_LIMIT",
    "PRECHECK_FAILED",
    "MAPPING_MISMATCH",
    "POWER_CYCLE_FAILED",
    "POWER_ON_UNCONFIRMED",
    "MAPPING_CHANGED",
    "CYCLE_ALREADY_RECORDED",
    "RECOVERY_UNVERIFIED_AFTER_RESTART",
    "NON_TARGET_STATE_CHANGED",
}
ACTIVE_ALARM_STATES = {
    "UNMAPPED",
    "TARGET_DISABLED",
    "RECOVERY_TIMEOUT",
    "PRECHECK_FAILED",
    "MAPPING_MISMATCH",
    "POWER_CYCLE_FAILED",
    "POWER_ON_UNCONFIRMED",
    "MAPPING_CHANGED",
    "CYCLE_ALREADY_RECORDED",
    "RECOVERY_UNVERIFIED_AFTER_RESTART",
    "SUPPRESSED_DAILY_LIMIT",
    "SUPPRESSED_COOLDOWN",
    "NON_TARGET_STATE_CHANGED",
}
_BROADCAST_CODE_RE = re.compile(r"^[A-Za-z0-9]{15}$")
_POWER_GROUP_ID_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]{0,63}$")
_LEGACY_COMMAND_HEADER = b"\xCC\xDD"
_LEGACY_STATUS_HEADER = b"\xAA\xBB\xB0"


class ConfigurationError(ValueError):
    """The manager configuration is unsafe or malformed."""


class RelayProtocolError(RuntimeError):
    """The relay returned a malformed or unverifiable response."""


@dataclass(frozen=True)
class Policy:
    off_seconds: float = 10.0
    boot_timeout_seconds: float = 180.0
    healthy_seconds: float = 10.0
    minimum_cycle_interval_seconds: float = 1800.0
    max_cycles_per_24_hours: int = 3
    event_max_age_seconds: float = 600.0
    status_stale_seconds: float = 5.0
    connect_timeout_seconds: float = 3.0
    command_timeout_seconds: float = 3.0
    command_retries: int = 3
    restore_retries: int = 5
    restore_deadline_seconds: float = 90.0
    ensure_on_retry_seconds: float = 30.0
    precheck_retry_seconds: float = 60.0
    precheck_max_attempts: int = 5


@dataclass(frozen=True)
class PowerGroup:
    group_id: str
    label: str
    enabled: bool
    members: Tuple[str, ...]
    host: str
    port: int
    channel: int
    address: int = 1
    allow_omitted_status_checksum: bool = False

    @property
    def power_key(self) -> str:
        """Stable physical identity used for rate limits and endpoint locks."""

        return "legacy_tcp|%s|%d|%d|%d" % (
            self.host,
            self.port,
            self.address,
            self.channel,
        )


@dataclass(frozen=True)
class ManagerConfig:
    mode: str
    state_db: str
    request_topic: str
    state_topic: str
    status_topic: str
    heartbeat_topic: str
    policy: Policy
    power_groups: Mapping[str, PowerGroup]
    member_to_group: Mapping[str, str]

    def group_for(self, broadcast_code: str) -> Optional[PowerGroup]:
        group_id = self.member_to_group.get(broadcast_code)
        return self.power_groups.get(group_id) if group_id is not None else None


@dataclass(frozen=True)
class PowerCycleRequest:
    event_id: str
    broadcast_code: str
    timestamp: float
    detected_at: float
    driver_instance: int
    handle: int
    episode_count: int

    @classmethod
    def from_payload(cls, payload: Mapping[str, Any]) -> "PowerCycleRequest":
        if payload.get("schema_version") != WIRE_SCHEMA_VERSION:
            raise ValueError("unsupported request schema_version")
        if payload.get("type") != REQUEST_TYPE:
            raise ValueError("not a POWER_CYCLE_REQUIRED request")
        event_id = _required_text(payload, "event_id", maximum=240)
        broadcast_code = _broadcast_code(payload.get("broadcast_code"))
        timestamp = _number(payload, "timestamp", minimum=0)
        detected_at = _number(payload, "detected_at", minimum=0)
        driver_instance = _integer(payload, "driver_instance", minimum=0)
        handle = _integer(payload, "handle", minimum=0, maximum=255)
        episode_count = _integer(payload, "episode_count", minimum=1)
        expected_event_id = "%s:%d:%d:%d" % (
            broadcast_code,
            driver_instance,
            int(detected_at),
            episode_count,
        )
        if event_id != expected_event_id:
            raise ValueError("event_id does not match request identity fields")
        return cls(
            event_id=event_id,
            broadcast_code=broadcast_code,
            timestamp=timestamp,
            detected_at=detected_at,
            driver_instance=driver_instance,
            handle=handle,
            episode_count=episode_count,
        )

    @classmethod
    def from_state(cls, payload: Mapping[str, Any]) -> "PowerCycleRequest":
        broadcast_code = _broadcast_code(payload.get("broadcast_code"))
        driver_instance = _integer(payload, "driver_instance", minimum=0)
        detected_at = _number(payload, "power_cycle_required_at", minimum=0)
        episode_count = _integer(
            payload, "power_cycle_required_count", minimum=1
        )
        handle = _integer(payload, "handle", minimum=0, maximum=255)
        event_id = "%s:%d:%d:%d" % (
            broadcast_code,
            driver_instance,
            int(detected_at),
            episode_count,
        )
        return cls(
            event_id=event_id,
            broadcast_code=broadcast_code,
            timestamp=time.time(),
            detected_at=detected_at,
            driver_instance=driver_instance,
            handle=handle,
            episode_count=episode_count,
        )


def _required_text(
    data: Mapping[str, Any], name: str, *, maximum: int = 1024
) -> str:
    value = data.get(name)
    if not isinstance(value, str) or not value.strip():
        raise ConfigurationError("%s must be a non-empty string" % name)
    value = value.strip()
    if len(value) > maximum:
        raise ConfigurationError("%s is too long" % name)
    return value


def _reject_unknown(
    data: Mapping[str, Any], allowed: Sequence[str], scope: str
) -> None:
    unknown = sorted(set(data) - set(allowed))
    if unknown:
        raise ConfigurationError(
            "%s contains unknown key(s): %s" % (scope, ", ".join(unknown))
        )


def _boolean(data: Mapping[str, Any], name: str, default: bool) -> bool:
    value = data.get(name, default)
    if not isinstance(value, bool):
        raise ConfigurationError("%s must be true or false" % name)
    return value


def _number(
    data: Mapping[str, Any],
    name: str,
    *,
    default: Optional[float] = None,
    minimum: Optional[float] = None,
    maximum: Optional[float] = None,
) -> float:
    value = data.get(name, default)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ConfigurationError("%s must be a number" % name)
    result = float(value)
    if not math.isfinite(result):
        raise ConfigurationError("%s must be finite" % name)
    if minimum is not None and result < minimum:
        raise ConfigurationError("%s must be >= %s" % (name, minimum))
    if maximum is not None and result > maximum:
        raise ConfigurationError("%s must be <= %s" % (name, maximum))
    return result


def _integer(
    data: Mapping[str, Any],
    name: str,
    *,
    default: Optional[int] = None,
    minimum: Optional[int] = None,
    maximum: Optional[int] = None,
) -> int:
    value = data.get(name, default)
    if isinstance(value, bool) or not isinstance(value, int):
        raise ConfigurationError("%s must be an integer" % name)
    if minimum is not None and value < minimum:
        raise ConfigurationError("%s must be >= %s" % (name, minimum))
    if maximum is not None and value > maximum:
        raise ConfigurationError("%s must be <= %s" % (name, maximum))
    return value


def _broadcast_code(value: Any) -> str:
    if not isinstance(value, str) or not _BROADCAST_CODE_RE.fullmatch(value):
        raise ConfigurationError(
            "broadcast_code must be exactly 15 ASCII letters/digits"
        )
    return value


def _power_group_id(value: Any) -> str:
    if not isinstance(value, str) or not _POWER_GROUP_ID_RE.fullmatch(value):
        raise ConfigurationError(
            "power_group id must be 1-64 safe ASCII letters/digits/._-"
        )
    return value


def _topic(data: Mapping[str, Any], name: str, default: str) -> str:
    value = data.get(name, default)
    if not isinstance(value, str) or not value.startswith("/") or " " in value:
        raise ConfigurationError("%s must be an absolute ROS topic" % name)
    return value


def _policy_from_json(data: Mapping[str, Any]) -> Policy:
    if not isinstance(data, Mapping):
        raise ConfigurationError("policy must be an object")
    _reject_unknown(
        data,
        (
            "off_seconds",
            "boot_timeout_seconds",
            "healthy_seconds",
            "minimum_cycle_interval_seconds",
            "max_cycles_per_24_hours",
            "event_max_age_seconds",
            "status_stale_seconds",
            "connect_timeout_seconds",
            "command_timeout_seconds",
            "command_retries",
            "restore_retries",
            "restore_deadline_seconds",
            "ensure_on_retry_seconds",
            "precheck_retry_seconds",
            "precheck_max_attempts",
        ),
        "policy",
    )
    policy = Policy(
        off_seconds=_number(
            data, "off_seconds", default=10, minimum=2, maximum=120
        ),
        boot_timeout_seconds=_number(
            data, "boot_timeout_seconds", default=180, minimum=30, maximum=900
        ),
        healthy_seconds=_number(
            data, "healthy_seconds", default=10, minimum=3, maximum=60
        ),
        minimum_cycle_interval_seconds=_number(
            data,
            "minimum_cycle_interval_seconds",
            default=1800,
            minimum=60,
            maximum=86400,
        ),
        max_cycles_per_24_hours=_integer(
            data, "max_cycles_per_24_hours", default=3, minimum=1, maximum=12
        ),
        event_max_age_seconds=_number(
            data, "event_max_age_seconds", default=600, minimum=30, maximum=86400
        ),
        status_stale_seconds=_number(
            data, "status_stale_seconds", default=5, minimum=2, maximum=30
        ),
        connect_timeout_seconds=_number(
            data, "connect_timeout_seconds", default=3, minimum=0.2, maximum=5
        ),
        command_timeout_seconds=_number(
            data, "command_timeout_seconds", default=3, minimum=0.2, maximum=5
        ),
        command_retries=_integer(
            data, "command_retries", default=3, minimum=1, maximum=10
        ),
        restore_retries=_integer(
            data, "restore_retries", default=5, minimum=1, maximum=20
        ),
        restore_deadline_seconds=_number(
            data,
            "restore_deadline_seconds",
            default=90,
            minimum=30,
            maximum=100,
        ),
        ensure_on_retry_seconds=_number(
            data,
            "ensure_on_retry_seconds",
            default=30,
            minimum=5,
            maximum=3600,
        ),
        precheck_retry_seconds=_number(
            data,
            "precheck_retry_seconds",
            default=60,
            minimum=5,
            maximum=3600,
        ),
        precheck_max_attempts=_integer(
            data, "precheck_max_attempts", default=5, minimum=1, maximum=20
        ),
    )
    if policy.status_stale_seconds >= policy.healthy_seconds:
        raise ConfigurationError(
            "status_stale_seconds must be less than healthy_seconds so one "
            "cached state message cannot satisfy sustained recovery"
        )
    return policy


def load_config(path: str) -> ManagerConfig:
    config_path = Path(os.path.expandvars(os.path.expanduser(path)))
    try:
        with config_path.open("r", encoding="utf-8") as stream:
            raw = json.load(stream)
    except FileNotFoundError as exc:
        raise ConfigurationError("configuration not found: %s" % config_path) from exc
    except (OSError, json.JSONDecodeError) as exc:
        raise ConfigurationError("cannot read configuration: %s" % exc) from exc
    if not isinstance(raw, Mapping):
        raise ConfigurationError("configuration root must be an object")
    _reject_unknown(
        raw,
        (
            "schema_version",
            "mode",
            "state_db",
            "topics",
            "policy",
            "power_groups",
        ),
        "configuration",
    )
    if raw.get("schema_version") != CONFIG_SCHEMA_VERSION:
        raise ConfigurationError("configuration schema_version must be 2")
    mode = raw.get("mode", "observe")
    if mode not in {"observe", "armed"}:
        raise ConfigurationError("mode must be 'observe' or 'armed'")
    raw_state_db = raw.get(
        "state_db",
        "~/.local/state/livox-power-cycle-manager/state.sqlite3",
    )
    if not isinstance(raw_state_db, str) or not raw_state_db.strip():
        raise ConfigurationError("state_db must be a non-empty path string")
    state_db = os.path.abspath(
        os.path.expandvars(
            os.path.expanduser(raw_state_db.strip())
        )
    )
    topics = raw.get("topics", {})
    if not isinstance(topics, Mapping):
        raise ConfigurationError("topics must be an object")
    _reject_unknown(
        topics, ("request", "state", "status", "heartbeat"), "topics"
    )
    policy = _policy_from_json(raw.get("policy", {}))
    group_rows = raw.get("power_groups", {})
    if not isinstance(group_rows, Mapping):
        raise ConfigurationError("power_groups must be an object keyed by group id")
    power_groups: Dict[str, PowerGroup] = {}
    member_to_group: Dict[str, str] = {}
    occupied: Dict[Tuple[str, int, int], str] = {}
    for key, row in group_rows.items():
        group_id = _power_group_id(key)
        if not isinstance(row, Mapping):
            raise ConfigurationError("power_groups.%s must be an object" % group_id)
        _reject_unknown(
            row,
            ("label", "enabled", "members", "relay"),
            "power_groups.%s" % group_id,
        )
        enabled = _boolean(row, "enabled", False)
        label = row.get("label", group_id)
        if not isinstance(label, str) or not label.strip() or len(label) > 80:
            raise ConfigurationError("power_groups.%s.label is invalid" % group_id)
        member_rows = row.get("members")
        if not isinstance(member_rows, list) or len(member_rows) != 4:
            raise ConfigurationError(
                "power_groups.%s.members must contain exactly 4 broadcast codes"
                % group_id
            )
        members: List[str] = []
        for raw_member in member_rows:
            code = _broadcast_code(raw_member)
            if code in members:
                raise ConfigurationError(
                    "power_groups.%s contains duplicate member %s"
                    % (group_id, code)
                )
            previous_group = member_to_group.get(code)
            if previous_group is not None:
                raise ConfigurationError(
                    "LiDAR %s belongs to both power groups %s and %s"
                    % (code, previous_group, group_id)
                )
            members.append(code)
            member_to_group[code] = group_id
        relay = row.get("relay")
        if not isinstance(relay, Mapping):
            raise ConfigurationError(
                "power_groups.%s.relay must be an object" % group_id
            )
        _reject_unknown(
            relay,
            (
                "protocol",
                "host",
                "port",
                "channel",
                "address",
                "allow_omitted_status_checksum",
            ),
            "power_groups.%s.relay" % group_id,
        )
        protocol = relay.get("protocol", "legacy_tcp")
        if protocol != "legacy_tcp":
            raise ConfigurationError(
                "power_groups.%s supports only legacy_tcp automation" % group_id
            )
        host = _required_text(relay, "host", maximum=64)
        try:
            ip = ipaddress.ip_address(host)
        except ValueError as exc:
            raise ConfigurationError(
                "power_groups.%s.relay.host must be a fixed IP address"
                % group_id
            ) from exc
        if ip.is_unspecified or ip.is_multicast:
            raise ConfigurationError(
                "power_groups.%s.relay.host is unsafe" % group_id
            )
        port = _integer(relay, "port", default=50000, minimum=1, maximum=65535)
        channel = _integer(relay, "channel", minimum=1, maximum=4)
        address = _integer(relay, "address", default=1, minimum=0, maximum=255)
        allow_omitted = _boolean(
            relay, "allow_omitted_status_checksum", False
        )
        target = PowerGroup(
            group_id=group_id,
            label=label.strip(),
            enabled=enabled,
            members=tuple(members),
            host=str(ip),
            port=port,
            channel=channel,
            address=address,
            allow_omitted_status_checksum=allow_omitted,
        )
        if enabled:
            endpoint = (target.host, target.port, target.channel)
            previous = occupied.get(endpoint)
            if previous is not None:
                raise ConfigurationError(
                    "enabled power groups %s and %s share relay %s:%d channel %d"
                    % (
                        previous,
                        group_id,
                        target.host,
                        target.port,
                        target.channel,
                    )
                )
            occupied[endpoint] = group_id
        power_groups[group_id] = target
    if mode == "armed" and not any(
        group.enabled for group in power_groups.values()
    ):
        raise ConfigurationError(
            "mode=armed requires at least one explicitly enabled power group"
        )
    return ManagerConfig(
        mode=mode,
        state_db=state_db,
        request_topic=_topic(
            topics, "request", "/livox/power_cycle_request"
        ),
        state_topic=_topic(
            topics, "state", "/livox/lidar_recovery_state"
        ),
        status_topic=_topic(
            topics, "status", "/livox/power_cycle_status"
        ),
        heartbeat_topic=_topic(
            topics, "heartbeat", "/livox/power_cycle_heartbeat"
        ),
        policy=policy,
        power_groups=power_groups,
        member_to_group=member_to_group,
    )


class StateStoreError(RuntimeError):
    """The durable safety state is unknown, incompatible, or inconsistent."""


class StateStore:
    """Crash-consistent deduplication, rate-limit and restore obligations."""

    _EXPECTED_COLUMNS = {
        "power_events": (
            "event_id",
            "trigger_bcode",
            "group_id",
            "power_key",
            "status",
            "attempts",
            "next_attempt",
            "first_seen",
            "last_update",
            "detail",
        ),
        "power_cycles": (
            "id",
            "event_id",
            "trigger_bcode",
            "group_id",
            "power_key",
            "started_at",
            "safety_started_at",
            "off_confirmed_at",
            "on_confirmed_at",
            "outcome",
            "detail",
            "budget_charged",
        ),
        "power_obligations": (
            "power_key",
            "group_id",
            "event_id",
            "trigger_bcode",
            "members_json",
            "host",
            "port",
            "channel",
            "address",
            "allow_omitted_checksum",
            "label",
            "created_at",
            "last_attempt",
        ),
        "power_group_bindings": (
            "group_id",
            "power_key",
            "host",
            "port",
            "channel",
            "address",
            "first_seen",
            "last_seen",
        ),
        "power_alarms": (
            "power_key",
            "event_id",
            "trigger_bcode",
            "group_id",
            "state",
            "detail",
            "updated_at",
        ),
        "safety_clock": (
            "id",
            "logical_seconds",
            "wall_observed",
            "mono_observed",
        ),
    }

    def __init__(self, path: str) -> None:
        if not isinstance(path, str) or not path:
            raise StateStoreError("state database path must be a non-empty string")
        self.path = os.path.abspath(path)
        self._wall_anchor = time.time()
        self._mono_anchor = time.monotonic()
        Path(self.path).parent.mkdir(parents=True, exist_ok=True)
        with self._db() as db:
            db.execute("PRAGMA journal_mode=WAL")
            db.execute("PRAGMA synchronous=FULL")
            self._initialize_schema(db)
            self._initialize_safety_clock(db)
        self._reconcile_incomplete_cycles()

    def _guarded_wall_time(self) -> float:
        now_wall = time.time()
        expected_wall = self._wall_anchor + (
            time.monotonic() - self._mono_anchor
        )
        if abs(now_wall - expected_wall) > 30:
            raise StateStoreError(
                "system wall clock jumped by more than 30 seconds; refusing "
                "to evaluate cooldown/daily limits until manager restart"
            )
        return now_wall

    def _initialize_safety_clock(self, db: sqlite3.Connection) -> None:
        row = db.execute(
            "SELECT logical_seconds FROM safety_clock WHERE id=1"
        ).fetchone()
        if row is None:
            logical = 0.0
            db.execute(
                "INSERT INTO safety_clock(id,logical_seconds,wall_observed,"
                "mono_observed) VALUES(1,?,?,?)",
                (logical, time.time(), time.monotonic()),
            )
        else:
            logical = float(row[0])
            if not math.isfinite(logical) or logical < 0:
                raise StateStoreError("persisted safety clock is invalid")
        self._safety_base = logical
        self._safety_mono_base = time.monotonic()

    def _safety_now(self) -> float:
        self._guarded_wall_time()
        return self._safety_base + (
            time.monotonic() - self._safety_mono_base
        )

    def checkpoint_clock(self) -> None:
        logical = self._safety_now()
        with self._db() as db:
            db.execute(
                "UPDATE safety_clock SET logical_seconds=?,wall_observed=?,"
                "mono_observed=? WHERE id=1",
                (logical, time.time(), time.monotonic()),
            )

    def _initialize_schema(self, db: sqlite3.Connection) -> None:
        if db.in_transaction:
            raise StateStoreError(
                "state schema initialization unexpectedly entered with an "
                "active transaction"
            )
        # Keep every DDL change, the schema version, and the initial safety
        # clock in one transaction.  In particular, do not use executescript()
        # here: Python's sqlite3 commits a pending transaction before running
        # it, which can strand a database halfway through a migration.
        db.execute("BEGIN IMMEDIATE")
        version = int(db.execute("PRAGMA user_version").fetchone()[0])
        tables = {
            str(row[0])
            for row in db.execute(
                "SELECT name FROM sqlite_master WHERE type='table'"
            ).fetchall()
            if not str(row[0]).startswith("sqlite_")
        }
        legacy = tables.intersection({"events", "cycles", "obligations"})
        if legacy:
            raise StateStoreError(
                "legacy single-LiDAR state tables detected (%s); automatic "
                "migration is refused because a must-ON obligation cannot be "
                "safely inferred. Keep the old manager available to confirm "
                "every legacy obligation ON, then archive the database."
                % ",".join(sorted(legacy))
            )
        unknown = tables - set(self._EXPECTED_COLUMNS)
        if unknown:
            raise StateStoreError(
                "unexpected table(s) in dedicated state database: %s"
                % ",".join(sorted(unknown))
            )
        if version not in {0, 2, STATE_DB_SCHEMA_VERSION}:
            raise StateStoreError(
                "unsupported state database schema version %d (expected 2 or %d)"
                % (version, STATE_DB_SCHEMA_VERSION)
            )
        if version == 2:
            expected_v2 = tuple(
                column
                for column in self._EXPECTED_COLUMNS["power_cycles"]
                if column != "budget_charged"
            )
            column_rows = db.execute(
                "PRAGMA table_info(power_cycles)"
            ).fetchall()
            actual_v2 = tuple(str(row[1]) for row in column_rows)
            expected_v3 = self._EXPECTED_COLUMNS["power_cycles"]
            if actual_v2 not in {expected_v2, expected_v3}:
                raise StateStoreError(
                    "state table power_cycles cannot be migrated from schema 2: %s"
                    % ",".join(actual_v2)
                )
            if actual_v2 == expected_v2:
                db.execute(
                    "ALTER TABLE power_cycles ADD COLUMN budget_charged INTEGER "
                    "NOT NULL DEFAULT 1 CHECK(budget_charged IN (0,1))"
                )
            else:
                # Recover the only safe half-migration shape produced by the
                # previous implementation: version 2 with the exact v3 column
                # appended.  Reject weaker hand-made columns fail closed.
                budget = column_rows[-1]
                table_sql_row = db.execute(
                    "SELECT sql FROM sqlite_master WHERE type='table' "
                    "AND name='power_cycles'"
                ).fetchone()
                table_sql = str(table_sql_row[0]) if table_sql_row else ""
                has_check = re.search(
                    r"CHECK\s*\(\s*budget_charged\s+IN\s*"
                    r"\(\s*0\s*,\s*1\s*\)\s*\)",
                    table_sql,
                    re.IGNORECASE,
                )
                if not (
                    str(budget[2]).upper() == "INTEGER"
                    and int(budget[3]) == 1
                    and str(budget[4]).strip("()'\"") == "1"
                    and int(budget[5]) == 0
                    and has_check is not None
                ):
                    raise StateStoreError(
                        "schema 2 half-migration has an unsafe "
                        "budget_charged definition"
                    )
            # A version-2 row predates the only code path allowed to release a
            # reservation.  Charge every row conservatively, including rows in
            # a recovered half-migration.
            db.execute("UPDATE power_cycles SET budget_charged=1")

        schema_statements = (
            """
            CREATE TABLE IF NOT EXISTS power_events (
              event_id TEXT PRIMARY KEY,
              trigger_bcode TEXT NOT NULL,
              group_id TEXT NOT NULL,
              power_key TEXT NOT NULL,
              status TEXT NOT NULL,
              attempts INTEGER NOT NULL DEFAULT 0,
              next_attempt REAL NOT NULL DEFAULT 0,
              first_seen REAL NOT NULL,
              last_update REAL NOT NULL,
              detail TEXT NOT NULL DEFAULT ''
            )
            """,
            """
            CREATE TABLE IF NOT EXISTS power_cycles (
              id INTEGER PRIMARY KEY AUTOINCREMENT,
              event_id TEXT NOT NULL UNIQUE,
              trigger_bcode TEXT NOT NULL,
              group_id TEXT NOT NULL,
              power_key TEXT NOT NULL,
              started_at REAL NOT NULL,
              safety_started_at REAL NOT NULL,
              off_confirmed_at REAL,
              on_confirmed_at REAL,
              outcome TEXT NOT NULL DEFAULT 'STARTED',
              detail TEXT NOT NULL DEFAULT '',
              budget_charged INTEGER NOT NULL DEFAULT 1
                CHECK(budget_charged IN (0,1))
            )
            """,
            """
            CREATE INDEX IF NOT EXISTS power_cycles_key_started
              ON power_cycles(power_key, safety_started_at)
            """,
            """
            CREATE INDEX IF NOT EXISTS power_cycles_budget_key_started
              ON power_cycles(power_key, budget_charged, safety_started_at)
            """,
            """
            CREATE TABLE IF NOT EXISTS power_obligations (
              power_key TEXT PRIMARY KEY,
              group_id TEXT NOT NULL,
              event_id TEXT NOT NULL,
              trigger_bcode TEXT NOT NULL,
              members_json TEXT NOT NULL,
              host TEXT NOT NULL,
              port INTEGER NOT NULL,
              channel INTEGER NOT NULL,
              address INTEGER NOT NULL,
              allow_omitted_checksum INTEGER NOT NULL,
              label TEXT NOT NULL,
              created_at REAL NOT NULL,
              last_attempt REAL NOT NULL DEFAULT 0
            )
            """,
            """
            CREATE TABLE IF NOT EXISTS power_group_bindings (
              group_id TEXT PRIMARY KEY,
              power_key TEXT NOT NULL UNIQUE,
              host TEXT NOT NULL,
              port INTEGER NOT NULL,
              channel INTEGER NOT NULL,
              address INTEGER NOT NULL,
              first_seen REAL NOT NULL,
              last_seen REAL NOT NULL
            )
            """,
            """
            CREATE TABLE IF NOT EXISTS power_alarms (
              power_key TEXT PRIMARY KEY,
              event_id TEXT NOT NULL,
              trigger_bcode TEXT NOT NULL,
              group_id TEXT NOT NULL,
              state TEXT NOT NULL,
              detail TEXT NOT NULL,
              updated_at REAL NOT NULL
            )
            """,
            """
            CREATE TABLE IF NOT EXISTS safety_clock (
              id INTEGER PRIMARY KEY CHECK(id=1),
              logical_seconds REAL NOT NULL,
              wall_observed REAL NOT NULL,
              mono_observed REAL NOT NULL
            )
            """,
        )
        for statement in schema_statements:
            db.execute(statement)
        for table, expected in self._EXPECTED_COLUMNS.items():
            actual = tuple(
                str(row[1])
                for row in db.execute("PRAGMA table_info(%s)" % table).fetchall()
            )
            if actual != expected:
                raise StateStoreError(
                    "state table %s has incompatible columns: %s"
                    % (table, ",".join(actual))
                )
        db.execute("PRAGMA user_version=%d" % STATE_DB_SCHEMA_VERSION)

    def _reconcile_incomplete_cycles(self) -> None:
        with self._db() as db:
            rows = db.execute(
                "SELECT id,event_id,trigger_bcode,group_id,power_key,"
                "off_confirmed_at,on_confirmed_at FROM power_cycles "
                "WHERE outcome IN ('STARTED','POWER_ON_UNCONFIRMED')"
            ).fetchall()
            for row in rows:
                cycle_id, event_id, _trigger, _group_id, power_key, off_at, on_at = row
                obligation = db.execute(
                    "SELECT 1 FROM power_obligations WHERE power_key=? "
                    "AND event_id=?",
                    (power_key, event_id),
                ).fetchone()
                if obligation is not None:
                    continue
                if off_at is not None and on_at is None:
                    raise StateStoreError(
                        "cycle %s recorded OFF without ON and has no persisted "
                        "restore obligation; refusing automatic operation"
                        % event_id
                    )
                if off_at is None:
                    outcome = "INTERRUPTED_BEFORE_OFF"
                    status = "POWER_CYCLE_FAILED"
                    detail = "manager restarted before OFF was confirmed"
                else:
                    outcome = "RECOVERY_UNVERIFIED_AFTER_RESTART"
                    status = "RECOVERY_UNVERIFIED_AFTER_RESTART"
                    detail = (
                        "relay ON was recorded before restart, but group health "
                        "verification did not complete; no repeat cycle"
                    )
                db.execute(
                    "UPDATE power_cycles SET outcome=?,detail=? WHERE id=?",
                    (outcome, detail, cycle_id),
                )
                db.execute(
                    "UPDATE power_events SET status=?,last_update=?,detail=? "
                    "WHERE event_id=? AND status NOT IN (%s)"
                    % ",".join("?" for _ in TERMINAL_EVENT_STATES),
                    (status, time.time(), detail, event_id)
                    + tuple(TERMINAL_EVENT_STATES),
                )
                self._update_alarm(db, str(event_id), status, detail, time.time())

    def _connect(self) -> sqlite3.Connection:
        db = sqlite3.connect(self.path, timeout=10.0)
        db.execute("PRAGMA busy_timeout=10000")
        db.execute("PRAGMA synchronous=FULL")
        return db

    @contextmanager
    def _db(self):
        db = self._connect()
        try:
            with db:
                yield db
        finally:
            db.close()

    def start_event(
        self,
        request: PowerCycleRequest,
        group_id: str,
        power_key: str,
        retry_seconds: float,
        allow_observed: bool = False,
    ) -> Tuple[bool, int, str]:
        now = time.time()
        with self._db() as db:
            db.execute(
                "INSERT OR IGNORE INTO power_events"
                "(event_id,trigger_bcode,group_id,power_key,status,first_seen,"
                "last_update) VALUES(?,?,?,?,?,?,?)",
                (
                    request.event_id,
                    request.broadcast_code,
                    group_id,
                    power_key,
                    "NEW",
                    now,
                    now,
                ),
            )
            row = db.execute(
                "SELECT status,attempts,next_attempt,last_update,group_id,power_key "
                "FROM power_events "
                "WHERE event_id=?",
                (request.event_id,),
            ).fetchone()
            assert row is not None
            status, attempts, next_attempt, last_update, stored_group, stored_key = row
            if status in TERMINAL_EVENT_STATES:
                return False, int(attempts), "terminal"
            if status == "OBSERVED" and not allow_observed:
                return False, int(attempts), "observed"
            if stored_group != group_id or stored_key != power_key:
                if status == "OBSERVED" and allow_observed:
                    db.execute(
                        "UPDATE power_events SET group_id=?,power_key=?,"
                        "status='NEW',last_update=?,detail='' WHERE event_id=?",
                        (group_id, power_key, now, request.event_id),
                    )
                    status = "NEW"
                    stored_group = group_id
                    stored_key = power_key
                else:
                    detail = (
                        "event identity mapping changed from %s/%s to %s/%s; "
                        "automatic power action refused"
                        % (stored_group, stored_key, group_id, power_key)
                    )
                    db.execute(
                        "UPDATE power_events SET status='MAPPING_CHANGED',"
                        "last_update=?,detail=? WHERE event_id=?",
                        (now, detail[:1000], request.event_id),
                    )
                    self._update_alarm(
                        db, request.event_id, "MAPPING_CHANGED", detail, now
                    )
                    return False, int(attempts), "mapping_changed"
            retry_remaining = float(next_attempt) - now
            if status == "RETRY" and 0 < retry_remaining <= max(
                retry_seconds * 2, 300
            ):
                return False, int(attempts), "retry_wait"
            processing_age = now - float(last_update)
            if status == "PROCESSING" and 0 <= processing_age < retry_seconds:
                return False, int(attempts), "processing"
            attempts = int(attempts) + 1
            db.execute(
                "UPDATE power_events SET status='PROCESSING',attempts=?,last_update=?,"
                "next_attempt=0 WHERE event_id=?",
                (attempts, now, request.event_id),
            )
            return True, attempts, "ready"

    def observe_event(self, event_id: str, detail: str) -> None:
        with self._db() as db:
            db.execute(
                "UPDATE power_events SET status='OBSERVED',attempts=0,last_update=?,"
                "detail=? WHERE event_id=?",
                (time.time(), detail[:1000], event_id),
            )

    def retry_event(self, event_id: str, delay: float, detail: str) -> None:
        now = time.time()
        excluded = tuple(TERMINAL_EVENT_STATES) + ("OBSERVED",)
        with self._db() as db:
            db.execute(
                "UPDATE power_events SET status='RETRY',next_attempt=?,last_update=?,"
                "detail=? WHERE event_id=? AND status NOT IN (%s)"
                % ",".join("?" for _ in excluded),
                (now + delay, now, detail[:1000], event_id) + excluded,
            )

    def _update_alarm(
        self,
        db: sqlite3.Connection,
        event_id: str,
        status: str,
        detail: str,
        now: float,
    ) -> None:
        row = db.execute(
            "SELECT trigger_bcode,group_id,power_key FROM power_events "
            "WHERE event_id=?",
            (event_id,),
        ).fetchone()
        if row is None:
            return
        trigger, group_id, power_key = row
        if status in ACTIVE_ALARM_STATES:
            db.execute(
                "INSERT OR REPLACE INTO power_alarms(power_key,event_id,"
                "trigger_bcode,group_id,state,detail,updated_at) "
                "VALUES(?,?,?,?,?,?,?)",
                (
                    power_key,
                    event_id,
                    trigger,
                    group_id,
                    status,
                    detail[:1000],
                    now,
                ),
            )
        elif status == "RECOVERY_VERIFIED":
            db.execute(
                "DELETE FROM power_alarms WHERE power_key=?", (power_key,)
            )

    def finish_event(self, event_id: str, status: str, detail: str) -> None:
        if status not in TERMINAL_EVENT_STATES:
            raise ValueError("event status is not terminal: %s" % status)
        now = time.time()
        with self._db() as db:
            db.execute(
                "UPDATE power_events SET status=?,last_update=?,detail=? "
                "WHERE event_id=?",
                (status, now, detail[:1000], event_id),
            )
            self._update_alarm(db, event_id, status, detail, now)

    def cycle_limit(
        self, power_key: str, policy: Policy
    ) -> Tuple[bool, str, float]:
        now = self._safety_now()
        with self._db() as db:
            last = db.execute(
                "SELECT MAX(safety_started_at) FROM power_cycles "
                "WHERE power_key=? AND budget_charged=1",
                (power_key,),
            ).fetchone()[0]
            count = db.execute(
                "SELECT COUNT(*) FROM power_cycles WHERE power_key=? "
                "AND budget_charged=1 AND safety_started_at>=?",
                (power_key, now - 86400),
            ).fetchone()[0]
        if last is not None:
            remaining = policy.minimum_cycle_interval_seconds - max(
                now - float(last), 0
            )
            if remaining > 0:
                return False, "cooldown", remaining
        if int(count) >= policy.max_cycles_per_24_hours:
            return False, "daily_limit", 0
        return True, "ok", 0

    def reserve_cycle(
        self, request: PowerCycleRequest, target: PowerGroup, policy: Policy
    ) -> Tuple[Optional[int], str, float]:
        now_wall = self._guarded_wall_time()
        now = self._safety_now()
        with self._db() as db:
            existing = db.execute(
                "SELECT id FROM power_cycles WHERE event_id=?",
                (request.event_id,),
            ).fetchone()
            if existing is not None:
                return None, "existing_event", 0
            last = db.execute(
                "SELECT MAX(safety_started_at) FROM power_cycles "
                "WHERE power_key=? AND budget_charged=1",
                (target.power_key,),
            ).fetchone()[0]
            count = int(
                db.execute(
                    "SELECT COUNT(*) FROM power_cycles WHERE power_key=? "
                    "AND budget_charged=1 AND safety_started_at>=?",
                    (target.power_key, now - 86400),
                ).fetchone()[0]
            )
            if last is not None:
                age = now - float(last)
                remaining = policy.minimum_cycle_interval_seconds - max(age, 0)
                if remaining > 0:
                    return None, "cooldown", remaining
            if count >= policy.max_cycles_per_24_hours:
                return None, "daily_limit", 0
            db.execute(
                "INSERT INTO power_cycles"
                "(event_id,trigger_bcode,group_id,power_key,started_at,"
                "safety_started_at) VALUES(?,?,?,?,?,?)",
                (
                    request.event_id,
                    request.broadcast_code,
                    target.group_id,
                    target.power_key,
                    now_wall,
                    now,
                ),
            )
            db.execute(
                "UPDATE safety_clock SET logical_seconds=?,wall_observed=?,"
                "mono_observed=? WHERE id=1",
                (now, now_wall, time.monotonic()),
            )
            row = db.execute(
                "SELECT id FROM power_cycles WHERE event_id=?", (request.event_id,)
            ).fetchone()
            assert row is not None
            return int(row[0]), "ok", 0

    def cancel_cycle_before_off(
        self, cycle_id: int, outcome: str, detail: str
    ) -> None:
        """Release a reservation only while OFF is durably known not to occur.

        Reservations start charged so a crash anywhere after reserve_cycle()
        remains conservative.  The live worker calls this method only before
        dispatching its first OFF command.  The off_confirmed_at predicate is
        a final durable guard against releasing a completed power-cycle budget.
        """

        with self._db() as db:
            updated = db.execute(
                "UPDATE power_cycles SET budget_charged=0,outcome=?,detail=? "
                "WHERE id=? AND off_confirmed_at IS NULL AND budget_charged=1",
                (outcome, detail[:1000], cycle_id),
            )
            if updated.rowcount != 1:
                raise StateStoreError(
                    "cycle cannot be cancelled because it is missing or OFF "
                    "was already confirmed"
                )

    def confirm_on_and_cancel_before_off(
        self,
        cycle_id: int,
        power_key: str,
        event_id: str,
        outcome: str,
        detail: str,
    ) -> None:
        """Atomically archive proven ON and release an unused reservation.

        The caller must first confirm the physical target channel ON via B0.
        If either the cycle update or obligation delete fails, SQLite rolls
        both changes back: the must-ON obligation remains and the safety budget
        stays conservatively charged.
        """

        now = time.time()
        with self._db() as db:
            updated = db.execute(
                "UPDATE power_cycles SET on_confirmed_at=?,budget_charged=0,"
                "outcome=?,detail=? WHERE id=? AND event_id=? "
                "AND off_confirmed_at IS NULL AND budget_charged=1",
                (now, outcome, detail[:1000], cycle_id, event_id),
            )
            if updated.rowcount != 1:
                raise StateStoreError(
                    "cycle cannot be cancelled because it is missing or OFF "
                    "was already confirmed"
                )
            deleted = db.execute(
                "DELETE FROM power_obligations WHERE power_key=? AND event_id=?",
                (power_key, event_id),
            )
            if deleted.rowcount != 1:
                raise StateStoreError(
                    "must-be-ON obligation disappeared before no-OFF archival"
                )

    def mark_cycle(self, cycle_id: int, field: str) -> None:
        if field not in {"off_confirmed_at", "on_confirmed_at"}:
            raise ValueError("invalid cycle timestamp field")
        with self._db() as db:
            db.execute(
                "UPDATE power_cycles SET %s=? WHERE id=?" % field,
                (time.time(), cycle_id),
            )

    def finish_cycle(self, cycle_id: int, outcome: str, detail: str) -> None:
        with self._db() as db:
            db.execute(
                "UPDATE power_cycles SET outcome=?,detail=? WHERE id=?",
                (outcome, detail[:1000], cycle_id),
            )

    def set_obligation(
        self, request: PowerCycleRequest, target: PowerGroup
    ) -> None:
        with self._db() as db:
            db.execute(
                "INSERT INTO power_obligations"
                "(power_key,group_id,event_id,trigger_bcode,members_json,host,"
                "port,channel,address,allow_omitted_checksum,label,created_at,"
                "last_attempt) VALUES(?,?,?,?,?,?,?,?,?,?,?,?,0)",
                (
                    target.power_key,
                    target.group_id,
                    request.event_id,
                    request.broadcast_code,
                    json.dumps(list(target.members), separators=(",", ":")),
                    target.host,
                    target.port,
                    target.channel,
                    target.address,
                    1 if target.allow_omitted_status_checksum else 0,
                    target.label,
                    time.time(),
                ),
            )

    def clear_obligation(self, power_key: str, event_id: str) -> None:
        with self._db() as db:
            db.execute(
                "DELETE FROM power_obligations WHERE power_key=? AND event_id=?",
                (power_key, event_id),
            )

    def confirm_on_and_clear_obligation(
        self, cycle_id: int, power_key: str, event_id: str
    ) -> None:
        """Atomically archive ON confirmation and discharge its obligation."""

        now = time.time()
        with self._db() as db:
            updated = db.execute(
                "UPDATE power_cycles SET on_confirmed_at=? WHERE id=? "
                "AND event_id=?",
                (now, cycle_id, event_id),
            )
            if updated.rowcount != 1:
                raise StateStoreError("cycle disappeared before ON archival")
            deleted = db.execute(
                "DELETE FROM power_obligations WHERE power_key=? AND event_id=?",
                (power_key, event_id),
            )
            if deleted.rowcount != 1:
                raise StateStoreError(
                    "must-be-ON obligation disappeared before ON archival"
                )

    def obligations(self) -> List[Tuple[PowerGroup, str, str, float]]:
        with self._db() as db:
            rows = db.execute(
                "SELECT group_id,event_id,trigger_bcode,members_json,host,port,"
                "channel,address,allow_omitted_checksum,label,last_attempt "
                "FROM power_obligations"
            ).fetchall()
        result: List[Tuple[PowerGroup, str, str, float]] = []
        for row in rows:
            try:
                raw_members = json.loads(str(row[3]))
                if not isinstance(raw_members, list) or len(raw_members) != 4:
                    raise ValueError("members snapshot must contain exactly four")
                members = tuple(_broadcast_code(item) for item in raw_members)
            except (ValueError, TypeError, json.JSONDecodeError) as exc:
                raise StateStoreError(
                    "invalid members snapshot in persisted ON obligation: %s" % exc
                ) from exc
            target = PowerGroup(
                group_id=str(row[0]),
                label=str(row[9]),
                enabled=True,
                members=members,
                host=str(row[4]),
                port=int(row[5]),
                channel=int(row[6]),
                address=int(row[7]),
                allow_omitted_status_checksum=bool(row[8]),
            )
            result.append((target, str(row[1]), str(row[2]), float(row[10])))
        return result

    def has_obligations(self) -> bool:
        with self._db() as db:
            return db.execute(
                "SELECT 1 FROM power_obligations LIMIT 1"
            ).fetchone() is not None

    def obligation_count(self) -> int:
        with self._db() as db:
            return int(db.execute("SELECT COUNT(*) FROM power_obligations").fetchone()[0])

    def touch_obligation(self, power_key: str) -> None:
        with self._db() as db:
            db.execute(
                "UPDATE power_obligations SET last_attempt=? WHERE power_key=?",
                (time.time(), power_key),
            )

    def complete_repaired_obligation(
        self, power_key: str, event_id: str
    ) -> Tuple[Optional[str], str]:
        now = time.time()
        reported_status: Optional[str] = None
        reported_detail = "startup/watchdog confirmed relay channel ON"
        with self._db() as db:
            cycle = db.execute(
                "SELECT id,off_confirmed_at FROM power_cycles WHERE event_id=?",
                (event_id,),
            ).fetchone()
            if cycle is not None:
                cycle_id, off_at = cycle
                if off_at is None:
                    outcome = "INTERRUPTED_BEFORE_OFF"
                    status = "POWER_CYCLE_FAILED"
                    detail = "startup repair confirmed ON; OFF was never recorded"
                else:
                    outcome = "RECOVERY_UNVERIFIED_AFTER_RESTART"
                    status = "RECOVERY_UNVERIFIED_AFTER_RESTART"
                    detail = (
                        "startup repair confirmed ON; post-power group health "
                        "could not be verified across the restart"
                    )
                db.execute(
                    "UPDATE power_cycles SET on_confirmed_at=COALESCE("
                    "on_confirmed_at,?),outcome=?,detail=? WHERE id=?",
                    (now, outcome, detail, cycle_id),
                )
                db.execute(
                    "UPDATE power_events SET status=?,last_update=?,detail=? "
                    "WHERE event_id=?",
                    (status, now, detail, event_id),
                )
                self._update_alarm(db, event_id, status, detail, now)
                reported_status = status
                reported_detail = detail
            db.execute(
                "DELETE FROM power_obligations WHERE power_key=? AND event_id=?",
                (power_key, event_id),
            )
        return reported_status, reported_detail

    def bind_power_groups(self, groups: Sequence[PowerGroup]) -> None:
        now = time.time()
        with self._db() as db:
            for target in groups:
                by_id = db.execute(
                    "SELECT power_key FROM power_group_bindings WHERE group_id=?",
                    (target.group_id,),
                ).fetchone()
                by_key = db.execute(
                    "SELECT group_id FROM power_group_bindings WHERE power_key=?",
                    (target.power_key,),
                ).fetchone()
                if by_id is not None and str(by_id[0]) != target.power_key:
                    raise StateStoreError(
                        "power group %s was previously bound to another physical "
                        "relay endpoint; refusing to reset its safety budget"
                        % target.group_id
                    )
                if by_key is not None and str(by_key[0]) != target.group_id:
                    raise StateStoreError(
                        "relay endpoint %s was previously bound as group %s; "
                        "renaming a group cannot reset its safety budget"
                        % (target.power_key, by_key[0])
                    )
                if by_id is None:
                    db.execute(
                        "INSERT INTO power_group_bindings(group_id,power_key,host,"
                        "port,channel,address,first_seen,last_seen) "
                        "VALUES(?,?,?,?,?,?,?,?)",
                        (
                            target.group_id,
                            target.power_key,
                            target.host,
                            target.port,
                            target.channel,
                            target.address,
                            now,
                            now,
                        ),
                    )
                else:
                    db.execute(
                        "UPDATE power_group_bindings SET last_seen=? WHERE group_id=?",
                        (now, target.group_id),
                    )

    def current_alerts(self) -> List[Tuple[str, str, str, str, str]]:
        """Return the durable still-actionable alarm snapshot per endpoint."""

        with self._db() as db:
            rows = db.execute(
                "SELECT event_id,trigger_bcode,group_id,state,detail "
                "FROM power_alarms ORDER BY updated_at"
            ).fetchall()
        return [
            (
                str(event_id),
                str(trigger),
                str(group_id),
                str(status),
                str(detail),
            )
            for event_id, trigger, group_id, status, detail in rows
        ]


def _double_checksum(body: bytes) -> bytes:
    first = sum(body) & 0xFF
    return bytes((first, (first * 2) & 0xFF))


class CorxLegacyTcpClient:
    """Minimal verified single-channel client; never exposes an all-off call."""

    def __init__(self, target: PowerGroup, policy: Policy) -> None:
        self.target = target
        self.policy = policy

    def _query_frame(self) -> bytes:
        body = bytes((0xB0, self.target.address, 0x00, 0x00, 0x0D))
        return _LEGACY_COMMAND_HEADER + body + _double_checksum(body)

    def _set_frame(self, state: bool) -> bytes:
        enable_mask = 1 << (self.target.channel - 1)
        control_mask = enable_mask if state else 0
        body = bytes((0xA1, self.target.address))
        body += control_mask.to_bytes(2, "big")
        body += enable_mask.to_bytes(2, "big")
        return _LEGACY_COMMAND_HEADER + body + _double_checksum(body)

    def _exchange(self, payload: bytes, expect: str) -> bytes:
        deadline = time.monotonic() + self.policy.command_timeout_seconds
        with socket.create_connection(
            (self.target.host, self.target.port),
            timeout=self.policy.connect_timeout_seconds,
        ) as sock:
            sock.settimeout(self.policy.command_timeout_seconds)
            sock.sendall(payload)
            buffer = bytearray()
            while time.monotonic() < deadline:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                sock.settimeout(remaining)
                try:
                    chunk = sock.recv(4096)
                except socket.timeout:
                    break
                if not chunk:
                    break
                buffer.extend(chunk)
                if expect == "ack" and b"OK!" in buffer:
                    return b"OK!"
                start = buffer.find(_LEGACY_STATUS_HEADER)
                if start >= 0 and len(buffer) >= start + 9:
                    return bytes(buffer[start : start + 9])
                if len(buffer) > 8192:
                    raise RelayProtocolError("relay response exceeded 8192 bytes")
        raise TimeoutError("relay did not return a complete %s response" % expect)

    def query(self) -> Tuple[Tuple[bool, bool, bool, bool], Optional[str]]:
        frame = self._exchange(self._query_frame(), "status")
        if len(frame) != 9 or not frame.startswith(_LEGACY_STATUS_HEADER):
            raise RelayProtocolError("invalid CORX B0 status frame")
        if frame[3] != self.target.address or frame[6] != 0x0D:
            raise RelayProtocolError("CORX B0 status address/end marker mismatch")
        expected = _double_checksum(frame[2:7])
        warning: Optional[str] = None
        if frame[7:9] != expected:
            if not (
                self.target.allow_omitted_status_checksum
                and frame[7:9] == b"\x00\x00"
            ):
                raise RelayProtocolError("CORX B0 status checksum mismatch")
            warning = "accepted explicitly allowed omitted B0 checksum (00 00)"
        mask = int.from_bytes(frame[4:6], "big")
        if mask & ~0x0F:
            raise RelayProtocolError("CORX B0 status contains bits outside 4 channels")
        return tuple(bool(mask & (1 << bit)) for bit in range(4)), warning  # type: ignore[return-value]

    def _send_set(self, state: bool) -> None:
        try:
            response = self._exchange(self._set_frame(state), "ack")
        except TimeoutError:
            # Some firmware revisions apply A1 but omit the textual ACK.  The
            # following independent B0 query remains the authority.
            return
        if response != b"OK!" and not response.startswith(_LEGACY_STATUS_HEADER):
            raise RelayProtocolError("unexpected CORX A1 acknowledgement")

    def ensure_state(
        self,
        state: bool,
        *,
        retries: Optional[int] = None,
        deadline_seconds: Optional[float] = None,
    ) -> Optional[str]:
        attempts = self.policy.command_retries if retries is None else retries
        deadline = (
            time.monotonic() + deadline_seconds
            if deadline_seconds is not None
            else None
        )
        last_error: Optional[BaseException] = None
        attempts_done = 0
        for attempt in range(1, attempts + 1):
            if deadline is not None and attempt > 1 and time.monotonic() >= deadline:
                break
            attempts_done = attempt
            try:
                states, warning = self.query()
                if states[self.target.channel - 1] is state:
                    return warning
                self._send_set(state)
                time.sleep(0.2)
                states, warning = self.query()
                if states[self.target.channel - 1] is state:
                    return warning
                last_error = RelayProtocolError(
                    "relay channel %d remained %s"
                    % (self.target.channel, "ON" if not state else "OFF")
                )
            except (OSError, TimeoutError, RelayProtocolError) as exc:
                last_error = exc
            if attempt < attempts:
                delay = min(0.5 * attempt, 2.0)
                if deadline is not None:
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        break
                    delay = min(delay, remaining)
                time.sleep(delay)
        assert last_error is not None
        raise RelayProtocolError(
            "cannot confirm channel %d %s after %d attempt(s): %s"
            % (
                self.target.channel,
                "ON" if state else "OFF",
                attempts_done,
                last_error,
            )
        )


class PowerCycleManagerCore:
    """Single-writer state machine; ROS callbacks only enqueue and observe."""

    def __init__(
        self,
        config: ManagerConfig,
        store: StateStore,
        emit: Callable[[Mapping[str, Any]], None],
        *,
        relay_factory: Callable[[PowerGroup, Policy], Any] = CorxLegacyTcpClient,
    ) -> None:
        self.config = config
        self.store = store
        self.emit = emit
        self.relay_factory = relay_factory
        self.store.bind_power_groups(
            tuple(
                group
                for group in self.config.power_groups.values()
                if group.enabled
            )
        )
        self.stop_event = threading.Event()
        self._condition = threading.Condition()
        self._latest: Dict[
            str, Tuple[Mapping[str, Any], float, int]
        ] = {}
        self._state_sequence = 0
        self._queue: "queue.Queue[PowerCycleRequest]" = queue.Queue(maxsize=128)
        self._queued_ids: set = set()
        self._completed_ids: set = set()
        self._completed_order = deque()  # type: ignore[var-annotated]
        self._completed_limit = 4096
        self._obligation_first_pass = True
        self._obligation_attempt_mono: Dict[str, float] = {}
        self._thread = threading.Thread(
            target=self._worker, name="livox-power-cycle-worker", daemon=False
        )

    def start(self) -> None:
        for event_id, trigger, _group_id, status, detail in self.store.current_alerts():
            request = PowerCycleRequest(
                event_id=event_id,
                broadcast_code=trigger,
                timestamp=time.time(),
                detected_at=time.time(),
                driver_instance=0,
                handle=255,
                episode_count=1,
            )
            self._emit(status, request, "CRITICAL", detail)
            self._mark_completed(event_id)
        self._thread.start()

    def is_alive(self) -> bool:
        return self._thread.is_alive()

    def queue_size(self) -> int:
        return self._queue.qsize()

    def stop(self) -> None:
        self.stop_event.set()
        with self._condition:
            self._condition.notify_all()
        if self._thread.is_alive() and threading.current_thread() is not self._thread:
            # Never release process/endpoint locks while the worker may still
            # be restoring an OFF channel. Network operations and the restore
            # deadline are bounded; systemd is configured not to SIGKILL this
            # safety-critical shutdown path.
            self._thread.join()

    def accept_request_payload(self, payload: Mapping[str, Any]) -> None:
        request = PowerCycleRequest.from_payload(payload)
        age = time.time() - request.timestamp
        if age > self.config.policy.event_max_age_seconds or age < -60:
            self._emit(
                "STALE_REQUEST_IGNORED",
                request,
                "WARN",
                "latched request age %.1fs is outside the accepted window; "
                "a live 1 Hz POWER_CYCLE_REQUIRED state can still re-offer it"
                % age,
            )
            return
        self._enqueue(request)

    def accept_state_payload(self, payload: Mapping[str, Any]) -> None:
        if payload.get("schema_version") != WIRE_SCHEMA_VERSION:
            raise ValueError("unsupported recovery-state schema")
        if payload.get("type") != STATE_TYPE:
            raise ValueError("not a LIDAR_RECOVERY_STATE message")
        code = _broadcast_code(payload.get("broadcast_code"))
        state_timestamp = _number(payload, "timestamp", minimum=0)
        state_age = time.time() - state_timestamp
        maximum_age = max(
            self.config.policy.status_stale_seconds + 2.0, 5.0
        )
        if state_age > maximum_age or state_age < -2.0:
            raise ValueError(
                "recovery-state timestamp age %.1fs is outside the live window"
                % state_age
            )
        _integer(payload, "driver_instance", minimum=0)
        _integer(payload, "handle", minimum=0, maximum=255)
        _integer(payload, "power_cycle_required_count", minimum=0)
        _number(payload, "power_cycle_required_at", minimum=0)
        for boolean_name in (
            "connected",
            "broadcast_fresh",
            "publishing",
        ):
            if not isinstance(payload.get(boolean_name), bool):
                raise ValueError(
                    "%s must be a boolean in recovery state" % boolean_name
                )
        connect_state = _required_text(payload, "connect_state", maximum=32)
        handshake_state = _required_text(
            payload, "handshake_state", maximum=64
        )
        required = handshake_state == REQUEST_TYPE
        if required and not (
            payload.get("connected") is False
            and connect_state == "Off"
            and payload.get("publishing") is False
            and payload.get("broadcast_fresh") is True
            and payload.get("power_cycle_required_count", 0) > 0
        ):
            raise ValueError(
                "POWER_CYCLE_REQUIRED state is internally inconsistent"
            )
        request = PowerCycleRequest.from_state(payload) if required else None
        if self.config.group_for(code) is not None:
            with self._condition:
                self._state_sequence += 1
                self._latest[code] = (
                    dict(payload),
                    time.monotonic(),
                    self._state_sequence,
                )
                self._condition.notify_all()
        if request is not None:
            self._enqueue(request)

    def _enqueue(self, request: PowerCycleRequest) -> None:
        with self._condition:
            if (
                request.event_id in self._queued_ids
                or request.event_id in self._completed_ids
            ):
                return
            try:
                self._queue.put_nowait(request)
            except queue.Full:
                self._emit(
                    "QUEUE_FULL",
                    request,
                    "CRITICAL",
                    "request queue full; event will be offered again by 1 Hz state",
                )
                return
            self._queued_ids.add(request.event_id)

    def _mark_completed(self, event_id: str) -> None:
        with self._condition:
            if event_id in self._completed_ids:
                return
            while len(self._completed_order) >= self._completed_limit:
                expired = self._completed_order.popleft()
                self._completed_ids.discard(expired)
            self._completed_order.append(event_id)
            self._completed_ids.add(event_id)

    def _worker(self) -> None:
        while not self.stop_event.is_set():
            self._repair_obligations()
            if self.store.has_obligations():
                # A known must-be-ON channel takes priority over every new OFF,
                # including requests for other groups.
                self.stop_event.wait(1.0)
                continue
            try:
                request = self._queue.get(timeout=1.0)
            except queue.Empty:
                continue
            try:
                group = self.config.group_for(request.broadcast_code)
                group_id = (
                    group.group_id
                    if group is not None
                    else "unmapped.%s" % request.broadcast_code
                )
                power_key = (
                    group.power_key
                    if group is not None
                    else "unmapped|%s" % request.broadcast_code
                )
                ready, attempt, reason = self.store.start_event(
                    request,
                    group_id,
                    power_key,
                    self.config.policy.precheck_retry_seconds,
                    allow_observed=self.config.mode == "armed",
                )
                if not ready:
                    if reason in {"terminal", "observed", "mapping_changed"}:
                        self._mark_completed(request.event_id)
                    if reason == "mapping_changed":
                        self._emit(
                            "MAPPING_CHANGED",
                            request,
                            "CRITICAL",
                            "persisted event mapping changed; automatic power "
                            "action refused",
                        )
                    continue
                self._process(request, attempt)
            except Exception as exc:
                try:
                    self._emit(
                        "MANAGER_INTERNAL_ERROR", request, "CRITICAL", repr(exc)
                    )
                except Exception:
                    pass
                try:
                    self.store.retry_event(
                        request.event_id,
                        self.config.policy.precheck_retry_seconds,
                        "internal error: %r" % (exc,),
                    )
                except Exception:
                    pass
            finally:
                with self._condition:
                    self._queued_ids.discard(request.event_id)
                self._queue.task_done()

    def _process(self, request: PowerCycleRequest, attempt: int) -> None:
        policy = self.config.policy
        target = self.config.group_for(request.broadcast_code)
        if target is None:
            if self.config.mode == "observe":
                detail = (
                    "observed unmapped event; mode=observe so no relay was touched"
                )
                self.store.observe_event(request.event_id, detail)
                self._mark_completed(request.event_id)
                self._emit("OBSERVE_UNMAPPED", request, "WARN", detail)
                return
            self._terminal(
                request, "UNMAPPED", "ERROR", "no power-group whitelist mapping"
            )
            return
        if self.config.mode == "observe":
            detail = "validated event; mode=observe so relay was not touched"
            self.store.observe_event(request.event_id, detail)
            self._mark_completed(request.event_id)
            self._emit(
                "OBSERVE_ONLY", request, "WARN", detail
            )
            return
        if not target.enabled:
            self._terminal(
                request,
                "TARGET_DISABLED",
                "WARN",
                "shared power group is explicitly disabled",
            )
            return
        precheck, precheck_detail = self._wait_trigger_required(request, 5.0)
        if precheck is False:
            self._terminal(
                request,
                "STALE_OR_RECOVERED",
                "INFO",
                precheck_detail,
            )
            return
        if precheck is None:
            self._retry_precheck(request, attempt, precheck_detail)
            return
        relay = self.relay_factory(target, policy)
        try:
            states, warning = relay.query()
        except (OSError, TimeoutError, RelayProtocolError) as exc:
            self._retry_precheck(request, attempt, "relay precheck: %s" % exc)
            return
        if warning:
            self._emit("RELAY_PROTOCOL_WARNING", request, "WARN", warning)
        if not states[target.channel - 1]:
            self._terminal(
                request,
                "MAPPING_MISMATCH",
                "CRITICAL",
                "shared channel is already OFF while trigger LiDAR broadcast is "
                "fresh; refusing to energize an unverified power group",
            )
            return

        # A single cached 1 Hz frame must never authorize every pre-OFF gate.
        # Capture the trigger's receive sequence only after the physical relay
        # precheck, then require a newer frame for the exact same episode.
        with self._condition:
            trigger_row = self._latest.get(request.broadcast_code)
            relay_precheck_sequence = (
                trigger_row[2] if trigger_row is not None else self._state_sequence
            )
        final_precheck, final_detail = self._wait_trigger_required(
            request,
            policy.status_stale_seconds,
            after_sequence=relay_precheck_sequence,
        )
        if final_precheck is False:
            self._terminal(
                request, "STALE_OR_RECOVERED", "INFO", final_detail
            )
            return
        if final_precheck is None:
            self._retry_precheck(
                request,
                attempt,
                "final pre-OFF trigger state changed: %s" % final_detail,
            )
            return

        cycle_id, reason, remaining = self.store.reserve_cycle(
            request, target, policy
        )
        if cycle_id is None:
            if reason == "cooldown":
                self._terminal(
                    request,
                    "SUPPRESSED_COOLDOWN",
                    "ERROR",
                    "physical power endpoint cooldown has %.0fs remaining"
                    % remaining,
                )
            elif reason == "daily_limit":
                self._terminal(
                    request,
                    "SUPPRESSED_DAILY_LIMIT",
                    "CRITICAL",
                    "power endpoint for group %s reached maximum %d cycles per 24h"
                    % (target.group_id, policy.max_cycles_per_24_hours),
                )
            else:
                self._terminal(
                    request,
                    "CYCLE_ALREADY_RECORDED",
                    "CRITICAL",
                    "this event already owns a durable cycle record; repeat OFF "
                    "is forbidden after an interrupted run",
                )
            return
        def changed_non_target_channels(current_states: Sequence[bool]) -> List[int]:
            return [
                index + 1
                for index in range(4)
                if index != target.channel - 1
                and bool(current_states[index]) != bool(states[index])
            ]

        # Commit the must-be-ON obligation before the first OFF command.  A
        # crash at any later instruction is repaired on process restart.
        try:
            self.store.set_obligation(request, target)
        except (sqlite3.Error, StateStoreError) as exc:
            detail = "could not persist must-be-ON obligation: %s" % exc
            try:
                self.store.cancel_cycle_before_off(
                    cycle_id, "POWER_CYCLE_FAILED", detail
                )
            except (sqlite3.Error, StateStoreError) as cancel_exc:
                detail += (
                    "; could not durably release the unused safety budget: %s; "
                    "reservation remains conservatively charged" % cancel_exc
                )
                self.store.finish_cycle(
                    cycle_id, "POWER_CYCLE_FAILED", detail
                )
            self._terminal(request, "POWER_CYCLE_FAILED", "CRITICAL", detail)
            return
        final_trigger, final_trigger_detail = self._wait_trigger_required(
            request, 0.0
        )
        if final_trigger is not True:
            try:
                restore = self.relay_factory(target, policy)
                warning = restore.ensure_state(
                    True,
                    retries=policy.restore_retries,
                    deadline_seconds=policy.restore_deadline_seconds,
                )
                if warning:
                    self._emit(
                        "RELAY_PROTOCOL_WARNING", request, "WARN", warning
                    )
                restored_states, restored_warning = restore.query()
                if restored_warning:
                    self._emit(
                        "RELAY_PROTOCOL_WARNING",
                        request,
                        "WARN",
                        restored_warning,
                    )
                if not restored_states[target.channel - 1]:
                    raise RelayProtocolError(
                        "latest B0 query no longer confirms target channel ON"
                    )
                changed = changed_non_target_channels(restored_states)
                if changed:
                    detail = (
                        "trigger state changed before OFF and non-target relay "
                        "channel(s) changed: %s; target remains ON"
                        % ",".join(str(item) for item in changed)
                    )
                    state = "NON_TARGET_STATE_CHANGED"
                    severity = "CRITICAL"
                else:
                    detail = (
                        "%s after durable reservation; no OFF was sent and "
                        "target channel is confirmed ON" % final_trigger_detail
                    )
                    if final_trigger is False:
                        state = "STALE_OR_RECOVERED"
                        severity = "INFO"
                    else:
                        state = "PRECHECK_FAILED"
                        severity = "ERROR"
                self.store.confirm_on_and_cancel_before_off(
                    cycle_id,
                    target.power_key,
                    request.event_id,
                    state,
                    detail,
                )
                self._terminal(request, state, severity, detail)
            except Exception as exc:
                detail = (
                    "trigger state changed before OFF; ON archival also failed: "
                    "%s; persistent obligation retained" % exc
                )
                self.store.finish_cycle(
                    cycle_id, "POWER_ON_UNCONFIRMED", detail
                )
                self._terminal(
                    request, "POWER_ON_UNCONFIRMED", "CRITICAL", detail
                )
            return
        off_confirmed = False
        off_phase_ok = False
        on_archived = False
        phase_error = ""
        non_target_error = ""

        try:
            self._emit(
                "POWER_OFF_COMMAND",
                request,
                "WARN",
                "%s %s:%d channel %d"
                % (target.label, target.host, target.port, target.channel)
                + " powers %d LiDARs" % len(target.members),
            )
            warning = relay.ensure_state(False)
            if warning:
                self._emit("RELAY_PROTOCOL_WARNING", request, "WARN", warning)
            off_confirmed = True
            self.store.mark_cycle(cycle_id, "off_confirmed_at")
            off_states, off_warning = relay.query()
            if off_warning:
                self._emit(
                    "RELAY_PROTOCOL_WARNING", request, "WARN", off_warning
                )
            if off_states[target.channel - 1]:
                raise RelayProtocolError(
                    "latest B0 query no longer confirms target channel OFF"
                )
            changed = changed_non_target_channels(off_states)
            if changed:
                non_target_error = (
                    "non-target relay channel(s) changed during OFF: %s"
                    % ",".join(str(item) for item in changed)
                )
                raise RelayProtocolError(non_target_error)
            self._emit(
                "POWER_OFF_CONFIRMED",
                request,
                "WARN",
                "channel OFF confirmed; holding %.1fs" % policy.off_seconds,
            )
            if self.stop_event.wait(policy.off_seconds):
                raise RelayProtocolError(
                    "manager shutdown interrupted the OFF hold; restoring ON"
                )
            off_phase_ok = True
        except Exception as exc:
            phase_error = str(exc)
            self._emit("POWER_OFF_FAILED", request, "ERROR", str(exc))
        finally:
            try:
                # Use a fresh TCP client so restoration does not depend on the
                # socket used before/during the off interval.
                restore = self.relay_factory(target, policy)
                warning = restore.ensure_state(
                    True,
                    retries=policy.restore_retries,
                    deadline_seconds=policy.restore_deadline_seconds,
                )
                if warning:
                    self._emit("RELAY_PROTOCOL_WARNING", request, "WARN", warning)
                on_states, on_warning = restore.query()
                if on_warning:
                    self._emit(
                        "RELAY_PROTOCOL_WARNING", request, "WARN", on_warning
                    )
                if not on_states[target.channel - 1]:
                    raise RelayProtocolError(
                        "latest B0 query no longer confirms target channel ON"
                    )
                changed = changed_non_target_channels(on_states)
                if changed:
                    non_target_error = (
                        "non-target relay channel(s) changed during ON restore: %s"
                        % ",".join(str(item) for item in changed)
                    )
                self.store.confirm_on_and_clear_obligation(
                    cycle_id, target.power_key, request.event_id
                )
                on_archived = True
                self._emit(
                    "POWER_ON_CONFIRMED",
                    request,
                    "INFO",
                    "shared channel ON confirmed; waiting for all %d members"
                    % len(target.members),
                )
            except Exception as exc:
                self._emit(
                    "POWER_ON_UNCONFIRMED",
                    request,
                    "CRITICAL",
                    "%s; persistent ON obligation retained" % exc,
                )

        if not on_archived:
            detail = (
                "relay ON and its durable archival could not both be confirmed"
            )
            self.store.finish_cycle(cycle_id, "POWER_ON_UNCONFIRMED", detail)
            self._terminal(
                request, "POWER_ON_UNCONFIRMED", "CRITICAL", detail
            )
            return
        if non_target_error:
            detail = non_target_error + "; target channel is confirmed ON"
            self.store.finish_cycle(
                cycle_id, "NON_TARGET_STATE_CHANGED", detail
            )
            self._terminal(
                request, "NON_TARGET_STATE_CHANGED", "CRITICAL", detail
            )
            return
        if not off_confirmed or not off_phase_ok:
            detail = (
                "%s; target channel is confirmed ON"
                % (phase_error or "OFF phase did not complete")
            )
            self.store.finish_cycle(cycle_id, "POWER_CYCLE_FAILED", detail)
            self._terminal(request, "POWER_CYCLE_FAILED", "ERROR", detail)
            return

        recovered, unhealthy_members = self._wait_group_healthy(
            target,
            request.driver_instance,
            policy.boot_timeout_seconds,
            policy.healthy_seconds,
        )
        if recovered:
            detail = (
                "all %d power-group members sustained Normal/Sampling point "
                "publication for %.1fs"
                % (len(target.members), policy.healthy_seconds)
            )
            self.store.finish_cycle(cycle_id, "RECOVERY_VERIFIED", detail)
            self._terminal(request, "RECOVERY_VERIFIED", "INFO", detail)
        else:
            detail = (
                "shared relay is ON but group members [%s] did not sustain "
                "healthy point publication within %.1fs; no immediate second cycle"
                % (", ".join(unhealthy_members), policy.boot_timeout_seconds)
            )
            self.store.finish_cycle(cycle_id, "RECOVERY_TIMEOUT", detail)
            self._terminal(request, "RECOVERY_TIMEOUT", "CRITICAL", detail)

    def _retry_precheck(
        self, request: PowerCycleRequest, attempt: int, detail: str
    ) -> None:
        policy = self.config.policy
        if attempt >= policy.precheck_max_attempts:
            self._terminal(request, "PRECHECK_FAILED", "CRITICAL", detail)
            return
        self.store.retry_event(
            request.event_id, policy.precheck_retry_seconds, detail
        )
        self._emit(
            "PRECHECK_RETRY",
            request,
            "ERROR",
            "%s; retry %d/%d in %.0fs"
            % (
                detail,
                attempt,
                policy.precheck_max_attempts,
                policy.precheck_retry_seconds,
            ),
        )

    def _wait_trigger_required(
        self,
        request: PowerCycleRequest,
        timeout: float,
        *,
        after_sequence: Optional[int] = None,
    ) -> Tuple[Optional[bool], str]:
        deadline = time.monotonic() + timeout
        last_detail = (
            "waiting for a newer state after the relay precheck"
            if after_sequence is not None
            else "waiting for a fresh state from the triggering LiDAR"
        )
        with self._condition:
            while not self.stop_event.is_set():
                now = time.monotonic()
                trigger_row = self._latest.get(request.broadcast_code)
                if trigger_row is not None:
                    trigger, received, sequence = trigger_row
                    if (
                        (after_sequence is None or sequence > after_sequence)
                        and now - received
                        <= self.config.policy.status_stale_seconds
                    ):
                        try:
                            exact_event = bool(
                                trigger.get("handshake_state") == REQUEST_TYPE
                                and trigger.get("broadcast_fresh") is True
                                and trigger.get("driver_instance")
                                == request.driver_instance
                                and trigger.get("power_cycle_required_count")
                                == request.episode_count
                                and int(
                                    trigger.get("power_cycle_required_at", -1)
                                )
                                == int(request.detected_at)
                            )
                        except (TypeError, ValueError):
                            exact_event = False
                        if not exact_event:
                            return (
                                False,
                                "current trigger state no longer matches this "
                                "POWER_CYCLE_REQUIRED event identity",
                            )
                        return (
                            True,
                            "triggering LiDAR still matches the exact live "
                            "POWER_CYCLE_REQUIRED event",
                        )
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None, last_detail
                self._condition.wait(min(remaining, 1.0))
        return None, "manager stopping before trigger-state precheck completed"

    def _wait_group_healthy(
        self,
        group: PowerGroup,
        driver_instance: int,
        timeout: float,
        healthy_seconds: float,
    ) -> Tuple[bool, List[str]]:
        verification_started = time.monotonic()
        deadline = time.monotonic() + timeout
        healthy_since: Optional[float] = None
        unhealthy_members = list(group.members)
        with self._condition:
            while not self.stop_event.is_set():
                now = time.monotonic()
                unhealthy_members = []
                for bcode in group.members:
                    row = self._latest.get(bcode)
                    member_healthy = False
                    if row is not None:
                        payload, received, _sequence = row
                        member_healthy = bool(
                            received > verification_started
                            and now - received
                            <= self.config.policy.status_stale_seconds
                            and payload.get("driver_instance") == driver_instance
                            and payload.get("connected") is True
                            and payload.get("connect_state") == "Sampling"
                            and payload.get("lidar_state") == "Normal"
                            and payload.get("handshake_state") == "IDLE"
                            and payload.get("publishing") is True
                        )
                    if not member_healthy:
                        unhealthy_members.append(bcode)
                if not unhealthy_members:
                    if healthy_since is None:
                        healthy_since = now
                    if now - healthy_since >= healthy_seconds:
                        return True, []
                else:
                    healthy_since = None
                remaining = deadline - now
                if remaining <= 0:
                    # If every member is healthy right at the deadline but the
                    # group has not held that state for healthy_seconds, all
                    # members still fail the group-level sustained-health gate.
                    if not unhealthy_members:
                        unhealthy_members = list(group.members)
                    return False, unhealthy_members
                self._condition.wait(min(remaining, 1.0))
        if not unhealthy_members:
            unhealthy_members = list(group.members)
        return False, unhealthy_members

    def _repair_obligations(self) -> None:
        now = time.time()
        now_mono = time.monotonic()
        policy = self.config.policy
        for target, event_id, trigger_bcode, last_attempt in self.store.obligations():
            previous_mono = self._obligation_attempt_mono.get(target.power_key)
            if not self._obligation_first_pass:
                if previous_mono is not None:
                    if now_mono - previous_mono < policy.ensure_on_retry_seconds:
                        continue
                else:
                    wall_age = now - last_attempt
                    if 0 <= wall_age < policy.ensure_on_retry_seconds:
                        continue
            self.store.touch_obligation(target.power_key)
            request = PowerCycleRequest(
                event_id=event_id,
                broadcast_code=trigger_bcode,
                timestamp=now,
                detected_at=now,
                driver_instance=0,
                handle=255,
                episode_count=1,
            )
            try:
                relay = self.relay_factory(target, policy)
                relay.ensure_state(
                    True,
                    retries=policy.restore_retries,
                    deadline_seconds=policy.restore_deadline_seconds,
                )
                repaired_status, repaired_detail = self.store.complete_repaired_obligation(
                    target.power_key, event_id
                )
                self._obligation_attempt_mono.pop(target.power_key, None)
                if repaired_status is None:
                    self._emit(
                        "PERSISTED_ON_RESTORED",
                        request,
                        "INFO",
                        repaired_detail,
                        target,
                    )
                else:
                    self._emit(
                        repaired_status,
                        request,
                        "CRITICAL",
                        repaired_detail,
                        target,
                    )
            except Exception as exc:
                self._obligation_attempt_mono[target.power_key] = time.monotonic()
                self._emit(
                    "PERSISTED_ON_RETRY",
                    request,
                    "CRITICAL",
                    "still cannot confirm ON: %s" % exc,
                    target,
                )
        self._obligation_first_pass = False

    def _terminal(
        self,
        request: PowerCycleRequest,
        state: str,
        severity: str,
        detail: str,
    ) -> None:
        self.store.finish_event(request.event_id, state, detail)
        self._mark_completed(request.event_id)
        self._emit(state, request, severity, detail)

    def _emit(
        self,
        state: str,
        request: PowerCycleRequest,
        severity: str,
        detail: str,
        target: Optional[PowerGroup] = None,
    ) -> None:
        if target is None:
            target = self.config.group_for(request.broadcast_code)
        payload = {
            "schema_version": WIRE_SCHEMA_VERSION,
            "type": STATUS_TYPE,
            "timestamp": time.time(),
            "state": state,
            "severity": severity,
            "event_id": request.event_id,
            "broadcast_code": request.broadcast_code,
            "power_group": target.group_id if target else "",
            "members": list(target.members) if target else [],
            "label": target.label if target else request.broadcast_code,
            "detail": detail,
        }
        try:
            self.emit(payload)
        except Exception as exc:
            # Telemetry failure must never interrupt the OFF->ON safety path.
            print(
                "[LivoxPowerCycle] status emit failed for %s: %s"
                % (state, exc),
                file=sys.stderr,
            )


def check_relays(config: ManagerConfig) -> int:
    failures = 0
    for group_id, target in sorted(config.power_groups.items()):
        if not target.enabled:
            print(
                "SKIP group=%s (%s) members=%d: disabled"
                % (group_id, target.label, len(target.members))
            )
            continue
        try:
            states, warning = CorxLegacyTcpClient(target, config.policy).query()
            suffix = " WARNING=%s" % warning if warning else ""
            print(
                "OK group=%s (%s) members=%d %s:%d channel=%d states=%s%s"
                % (
                    group_id,
                    target.label,
                    len(target.members),
                    target.host,
                    target.port,
                    target.channel,
                    ",".join("ON" if value else "OFF" for value in states),
                    suffix,
                )
            )
        except (OSError, TimeoutError, RelayProtocolError) as exc:
            failures += 1
            print(
                "FAIL group=%s (%s): %s" % (group_id, target.label, exc),
                file=sys.stderr,
            )
    return 1 if failures else 0


def _acquire_file_lock(path: str, description: str):
    import fcntl  # Linux/Ubuntu only; deliberately not imported in test paths.

    Path(path).parent.mkdir(parents=True, exist_ok=True)
    descriptor = os.open(path, os.O_RDWR | os.O_CREAT, 0o600)
    stream = os.fdopen(descriptor, "r+")
    try:
        fcntl.flock(stream.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        stream.seek(0)
        stream.truncate()
        stream.write(
            "pid=%d started=%d resource=%s\n"
            % (os.getpid(), int(time.time()), description)
        )
        stream.flush()
        return stream
    except BaseException:
        stream.close()
        raise


def _acquire_singleton_lock(state_db: str):
    """Hold one process-wide lock for the lifetime of the ROS manager."""

    return _acquire_file_lock(state_db + ".manager.lock", "state-db-manager")


def _endpoint_lock_root() -> str:
    """Return one per-user lock namespace, independent of state DB paths."""

    home = os.environ.get("HOME") or str(Path.home())
    return os.path.abspath(
        os.path.join(
            os.path.expandvars(os.path.expanduser(home)),
            ".local",
            "state",
            "livox-power-cycle-manager",
            "endpoint-locks",
        )
    )


def _acquire_endpoint_locks(
    targets: Sequence[PowerGroup], _state_db: str
) -> List[Any]:
    """Lock canonical physical endpoints, independent of config/DB filename."""

    lock_root = _endpoint_lock_root()
    unique = {target.power_key: target for target in targets}
    streams: List[Any] = []
    try:
        for power_key in sorted(unique):
            digest = hashlib.sha256(power_key.encode("utf-8")).hexdigest()
            path = os.path.join(lock_root, "endpoint-%s.lock" % digest)
            streams.append(_acquire_file_lock(path, power_key))
        return streams
    except BaseException:
        for stream in reversed(streams):
            stream.close()
        raise


def repair_obligations_without_config(state_db: str) -> int:
    """Emergency ON repair used by systemd before parsing the site config."""

    singleton_lock = _acquire_singleton_lock(state_db)
    locks: List[Any] = []
    failures = 0
    policy = Policy()
    try:
        store = StateStore(state_db)
        obligations = store.obligations()
        if not obligations:
            print("No persisted must-be-ON obligations.")
            return 0
        locks = _acquire_endpoint_locks(
            [target for target, _event, _trigger, _last in obligations],
            state_db,
        )
        for target, event_id, trigger, _last_attempt in obligations:
            store.touch_obligation(target.power_key)
            try:
                warning = CorxLegacyTcpClient(target, policy).ensure_state(
                    True,
                    retries=policy.restore_retries,
                    deadline_seconds=policy.restore_deadline_seconds,
                )
                repaired_status, repaired_detail = store.complete_repaired_obligation(
                    target.power_key, event_id
                )
                suffix = " warning=%s" % warning if warning else ""
                print(
                    "RESTORED_ON group=%s trigger=%s endpoint=%s outcome=%s "
                    "detail=%s%s"
                    % (
                        target.group_id,
                        trigger,
                        target.power_key,
                        repaired_status or "ON_ONLY",
                        repaired_detail,
                        suffix,
                    )
                )
            except (OSError, TimeoutError, RelayProtocolError, sqlite3.Error) as exc:
                failures += 1
                print(
                    "RESTORE_ON_FAILED group=%s trigger=%s endpoint=%s: %s"
                    % (target.group_id, trigger, target.power_key, exc),
                    file=sys.stderr,
                )
    finally:
        for stream in reversed(locks):
            stream.close()
        singleton_lock.close()
    return 1 if failures else 0


def run_ros(config: ManagerConfig) -> int:
    try:
        import rospy
        from std_msgs.msg import String
    except ImportError as exc:
        print(
            "ROS Python modules are unavailable; source /opt/ros/noetic/setup.bash "
            "and the catkin workspace first: %s" % exc,
            file=sys.stderr,
        )
        return 2

    rospy.init_node("livox_power_cycle_manager", anonymous=False)
    publisher = rospy.Publisher(config.status_topic, String, queue_size=32, latch=True)
    heartbeat_publisher = rospy.Publisher(
        config.heartbeat_topic, String, queue_size=8, latch=False
    )

    def emit(payload: Mapping[str, Any]) -> None:
        text = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        publisher.publish(String(data=text))
        severity = payload.get("severity")
        line = "[LivoxPowerCycle] %s %s %s: %s" % (
            payload.get("state"),
            payload.get("power_group")
            or payload.get("broadcast_code")
            or "-",
            payload.get("label") or "-",
            payload.get("detail") or "",
        )
        if severity == "CRITICAL" or severity == "ERROR":
            rospy.logerr(line)
        elif severity == "WARN":
            rospy.logwarn(line)
        else:
            rospy.loginfo(line)

    try:
        singleton_lock = _acquire_singleton_lock(config.state_db)
    except (OSError, BlockingIOError) as exc:
        rospy.logfatal(
            "Another livox_power_cycle_manager owns %s.manager.lock: %s",
            config.state_db,
            exc,
        )
        return 2
    try:
        store = StateStore(config.state_db)
    except (OSError, sqlite3.Error, StateStoreError) as exc:
        rospy.logfatal("Cannot open durable state database %s: %s", config.state_db, exc)
        singleton_lock.close()
        return 2
    endpoint_locks: List[Any] = []
    try:
        configured_targets = [
            group for group in config.power_groups.values() if group.enabled
        ]
        obligation_targets = [
            target for target, _event, _trigger, _last in store.obligations()
        ]
        endpoint_locks = _acquire_endpoint_locks(
            configured_targets + obligation_targets, config.state_db
        )
        core = PowerCycleManagerCore(config, store, emit)
    except (OSError, BlockingIOError, sqlite3.Error, StateStoreError) as exc:
        rospy.logfatal("Cannot acquire durable relay safety ownership: %s", exc)
        for stream in reversed(endpoint_locks):
            stream.close()
        singleton_lock.close()
        return 2

    def request_cb(message: Any) -> None:
        try:
            payload = json.loads(message.data)
            if not isinstance(payload, Mapping):
                raise ValueError("request JSON root is not an object")
            core.accept_request_payload(payload)
        except (ValueError, ConfigurationError, json.JSONDecodeError) as exc:
            rospy.logwarn_throttle(30, "Invalid power-cycle request ignored: %s", exc)

    def state_cb(message: Any) -> None:
        try:
            payload = json.loads(message.data)
            if not isinstance(payload, Mapping):
                raise ValueError("state JSON root is not an object")
            core.accept_state_payload(payload)
        except (ValueError, ConfigurationError, json.JSONDecodeError) as exc:
            rospy.logwarn_throttle(30, "Invalid recovery state ignored: %s", exc)

    def health_cb(_event: Any) -> None:
        if not core.is_alive() and not rospy.is_shutdown():
            rospy.logfatal("Livox power-cycle worker thread stopped unexpectedly")
            rospy.signal_shutdown("power-cycle worker stopped")
            return
        try:
            store.checkpoint_clock()
        except (OSError, sqlite3.Error, StateStoreError) as exc:
            rospy.logfatal("Cannot checkpoint durable safety clock: %s", exc)
            rospy.signal_shutdown("safety clock checkpoint failed")
            return
        # Do not overwrite the latched terminal alarm with a heartbeat. Worker
        # liveness remains enforced here and by systemd; the last actionable
        # status stays available to a dashboard that reconnects later.
        rospy.loginfo_throttle(
            60,
            "Livox power-cycle worker alive: mode=%s queue=%d obligations=%d",
            config.mode,
            core.queue_size(),
            store.obligation_count(),
        )
        heartbeat = {
            "schema_version": WIRE_SCHEMA_VERSION,
            "type": STATUS_TYPE,
            "timestamp": time.time(),
            "state": "MANAGER_HEARTBEAT",
            "severity": "WARN" if config.mode == "observe" else "INFO",
            "event_id": "",
            "broadcast_code": "",
            "power_group": "",
            "members": [],
            "label": "",
            "detail": "mode=%s worker=alive queue=%d obligations=%d"
            % (config.mode, core.queue_size(), store.obligation_count()),
        }
        heartbeat_publisher.publish(
            String(
                data=json.dumps(
                    heartbeat, ensure_ascii=False, separators=(",", ":")
                )
            )
        )

    health_timer = None
    try:
        rospy.Subscriber(config.request_topic, String, request_cb, queue_size=32)
        rospy.Subscriber(config.state_topic, String, state_cb, queue_size=64)
        rospy.on_shutdown(core.stop)
        enabled_groups = sum(
            1 for item in config.power_groups.values() if item.enabled
        )
        enabled_members = sum(
            len(item.members)
            for item in config.power_groups.values()
            if item.enabled
        )
        startup = {
            "schema_version": WIRE_SCHEMA_VERSION,
            "type": STATUS_TYPE,
            "timestamp": time.time(),
            "state": "MANAGER_READY",
            "severity": "WARN" if config.mode == "observe" else "INFO",
            "event_id": "",
            "broadcast_code": "",
            "power_group": "",
            "members": [],
            "label": "",
            "detail": "mode=%s enabled_groups=%d enabled_members=%d "
            "obligations=%d state_db=%s"
            % (
                config.mode,
                enabled_groups,
                enabled_members,
                store.obligation_count(),
                config.state_db,
            ),
        }
        emit(startup)
        # Re-publish any persisted active group alarm only after READY so the
        # latched status remains the actionable alarm, not a startup banner.
        core.start()
        rospy.loginfo(
            "Livox power-cycle manager started: mode=%s request=%s state=%s",
            config.mode,
            config.request_topic,
            config.state_topic,
        )
        health_timer = rospy.Timer(rospy.Duration(10.0), health_cb)
        rospy.spin()
    finally:
        if health_timer is not None:
            health_timer.shutdown()
        core.stop()
        try:
            store.checkpoint_clock()
        except (OSError, sqlite3.Error, StateStoreError) as exc:
            rospy.logerr("Final safety clock checkpoint failed: %s", exc)
        for stream in reversed(endpoint_locks):
            stream.close()
        singleton_lock.close()
    return 0


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Fail-safe Livox -> CORX relay power-cycle manager"
    )
    parser.add_argument(
        "--config",
        default="~/.config/livox/power_cycle.json",
        help="validated JSON configuration path",
    )
    parser.add_argument(
        "--state-db",
        default=None,
        help="authoritative state DB path (required by --repair-before-start)",
    )
    parser.add_argument(
        "--mode",
        choices=("observe", "armed"),
        default=None,
        help="override the JSON mode (ROS launch uses this as its sole daily switch)",
    )
    parser.add_argument(
        "--repair-before-start",
        action="store_true",
        help=(
            "repair every persisted must-be-ON obligation using the explicit "
            "--state-db before reading the site JSON or starting ROS"
        ),
    )
    parser.add_argument(
        "--repair-obligations",
        action="store_true",
        help="confirm every persisted relay obligation ON without parsing site config/ROS",
    )
    parser.add_argument(
        "--validate-config",
        action="store_true",
        help="validate configuration without ROS or network access",
    )
    parser.add_argument(
        "--check-relays",
        action="store_true",
        help="query each enabled power group once; never changes relay outputs",
    )
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_argument_parser().parse_args(argv)
    state_db_override = None
    if args.state_db:
        state_db_override = os.path.abspath(
            os.path.expandvars(os.path.expanduser(args.state_db))
        )
    if args.repair_obligations:
        state_db = state_db_override or os.path.abspath(
            os.path.expanduser(
                "~/.local/state/livox-power-cycle-manager/state.sqlite3"
            )
        )
        try:
            return repair_obligations_without_config(state_db)
        except (OSError, sqlite3.Error, StateStoreError) as exc:
            print("Emergency ON repair failed: %s" % exc, file=sys.stderr)
            return 2
    if args.repair_before_start:
        if state_db_override is None:
            print(
                "Configuration error: --repair-before-start requires an "
                "explicit --state-db; refusing to guess the safety database",
                file=sys.stderr,
            )
            return 2
        try:
            repair_result = repair_obligations_without_config(
                state_db_override
            )
        except (OSError, sqlite3.Error, StateStoreError) as exc:
            print("Startup ON repair failed: %s" % exc, file=sys.stderr)
            return 2
        if repair_result != 0:
            print(
                "Startup ON repair failed; refusing to read configuration or "
                "permit a new OFF",
                file=sys.stderr,
            )
            return 2
    try:
        config = load_config(args.config)
    except ConfigurationError as exc:
        print("Configuration error: %s" % exc, file=sys.stderr)
        return 2
    if state_db_override is not None:
        configured_db = os.path.normcase(os.path.realpath(config.state_db))
        override_db = os.path.normcase(os.path.realpath(state_db_override))
        if configured_db != override_db:
            print(
                "Configuration error: --state-db resolves to %s but config "
                "state_db resolves to %s; refusing split safety state"
                % (state_db_override, config.state_db),
                file=sys.stderr,
            )
            return 2
        config = replace(config, state_db=state_db_override)
    if args.mode is not None:
        config = replace(config, mode=args.mode)
    if config.mode == "armed" and not any(
        item.enabled for item in config.power_groups.values()
    ):
        print(
            "Configuration error: mode=armed requires at least one explicitly "
            "enabled power group",
            file=sys.stderr,
        )
        return 2
    if args.validate_config:
        enabled_groups = sum(
            1 for item in config.power_groups.values() if item.enabled
        )
        total_members = sum(
            len(item.members) for item in config.power_groups.values()
        )
        print(
            "Configuration valid: mode=%s groups=%d enabled_groups=%d "
            "members=%d state_db=%s"
            % (
                config.mode,
                len(config.power_groups),
                enabled_groups,
                total_members,
                config.state_db,
            )
        )
        return 0
    if args.check_relays:
        return check_relays(config)
    return run_ros(config)


if __name__ == "__main__":
    raise SystemExit(main())
