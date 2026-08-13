#!/usr/bin/env python3
"""Fail-safe CORX relay power-cycle manager for Livox LiDAR recovery.

The Livox driver publishes a POWER_CYCLE_REQUIRED event only after a bounded
soft-recovery/attribution path reaches one of five strict causes: a live-
broadcast handshake stall, an explicit low-power wake dropout, a sustained
dropout after healthy Normal publication, or a configured member missing at
startup.  This independent ROS node validates cause-specific live evidence and an explicit power-group
whitelist (one or more broadcast codes sharing a configured relay-channel set), applies
persistent group-level rate limits, confirms every relay state transition, and
verifies that every group member resumes point-cloud publication after power is
restored.

No third-party Python package is required.  ROS imports are deliberately kept
inside ``run_ros`` so configuration, relay and persistence tests can run on a
plain Python 3.8 installation.
"""

from __future__ import annotations

import argparse
from collections import deque
from contextlib import contextmanager
import errno
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
import uuid
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any, Callable, Dict, List, Mapping, Optional, Sequence, Tuple


WIRE_SCHEMA_VERSION = 1
CONFIG_SCHEMA_VERSION = 2
STATE_DB_SCHEMA_VERSION = 7
REQUEST_TYPE = "POWER_CYCLE_REQUIRED"
STATE_TYPE = "LIDAR_RECOVERY_STATE"
STATUS_TYPE = "POWER_CYCLE_STATUS"
INTENT_TYPE = "GROUP_POWER_CYCLE_INTENT"
INTENT_CANCEL_TYPE = "GROUP_POWER_CYCLE_CANCEL"
INTENT_ACK_TYPE = "GROUP_POWER_CYCLE_INTENT_ACK"
RECOVERY_STATE_IDLE = "IDLE"
RECOVERY_STATE_REQUIRED = REQUEST_TYPE
RECOVERY_REASON_NONE = "NONE"
RECOVERY_REASON_HANDSHAKE = "HANDSHAKE_STUCK"
RECOVERY_REASON_WAKE_DROPOUT = "WAKE_DROPOUT"
RECOVERY_REASON_NORMAL_DROPOUT = "NORMAL_DROPOUT"
RECOVERY_REASON_STARTUP_MISSING = "STARTUP_MISSING"
RECOVERY_REASON_ERROR_REBOOT_EXHAUSTED = "ERROR_REBOOT_EXHAUSTED"
RECOVERY_REASONS = {
    RECOVERY_REASON_HANDSHAKE,
    RECOVERY_REASON_WAKE_DROPOUT,
    RECOVERY_REASON_NORMAL_DROPOUT,
    RECOVERY_REASON_STARTUP_MISSING,
    RECOVERY_REASON_ERROR_REBOOT_EXHAUSTED,
}
WAKE_STATE_REQUIRED = REQUEST_TYPE
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
    "DRIVER_INTENT_ACK_TIMEOUT",
    "DRIVER_INTENT_REJECTED",
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
    "NON_TARGET_STATE_CHANGED",
    "DRIVER_INTENT_ACK_TIMEOUT",
    "DRIVER_INTENT_REJECTED",
}
RESOLVED_ALARM_STATES = {
    "RECOVERY_VERIFIED",
    "STALE_OR_RECOVERED",
    "SUPPRESSED_COOLDOWN",  # schema <=6 compatibility only
}
ACTIVE_WORKFLOW_STATES = {
    "DEFERRED_COOLDOWN",
    "DRIVER_INTENT_ACKED",
    "DRIVER_INTENT_ACK_RETRY",
    "MANAGER_INTERNAL_ERROR",
    "PERSISTED_ON_RETRY",
    "POWER_OFF_COMMAND",
    "POWER_OFF_CONFIRMED",
    "POWER_OFF_FAILED",
    "POWER_ON_CONFIRMED",
    "PRECHECK_RETRY",
    "QUEUE_FULL",
}
_ALARM_SEVERITIES = {"INFO", "WARN", "ERROR", "CRITICAL"}
_BROADCAST_CODE_RE = re.compile(r"^[A-Za-z0-9]{15}$")
_POWER_GROUP_ID_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]{0,63}$")
_LEGACY_COMMAND_HEADER = b"\xCC\xDD"
_LEGACY_STATUS_HEADER = b"\xAA\xBB\xB0"
_LEGACY_VERSION_BANNER = b"v1.0"
# A field CX-5104E-L either appends the ninth B0 checksum byte shortly after
# the first eight bytes or omits it completely.  Briefly wait for a genuine
# TCP tail fragment before treating a strictly verified eight-byte prefix as
# the complete single-checksum firmware variant.
_LEGACY_STATUS_TAIL_GRACE_SECONDS = 0.1
_DEFAULT_OFF_SECONDS = 10.0
_MINIMUM_OFF_SECONDS = 5.0
_WAKE_OBSERVATION_MAX_SECONDS = 60.0
_WAKE_DROPOUT_CONFIRM_MIN_SECONDS = 10.0
_NORMAL_HEALTHY_ARM_MIN_SECONDS = 30.0
_NORMAL_DROPOUT_CONFIRM_MIN_SECONDS = 5.0
_STARTUP_MISSING_GRACE_MIN_SECONDS = 30.0
_ERROR_CONFIRM_MIN_SECONDS = 3.0
_ERROR_REBOOT_ATTEMPTS_REQUIRED = 3
_EVENT_TIME_FUTURE_TOLERANCE_SECONDS = 2.0


class ConfigurationError(ValueError):
    """The manager configuration is unsafe or malformed."""


class RelayProtocolError(RuntimeError):
    """The relay returned a malformed or unverifiable response."""


@dataclass(frozen=True)
class Policy:
    # Preserve the pre-fast-recovery value for callers and site configs that
    # omit off_seconds.  New deployments opt in to five seconds explicitly.
    off_seconds: float = _DEFAULT_OFF_SECONDS
    boot_timeout_seconds: float = 180.0
    healthy_seconds: float = 10.0
    minimum_cycle_interval_seconds: float = 1800.0
    max_cycles_per_24_hours: int = 3
    event_max_age_seconds: float = 600.0
    status_stale_seconds: float = 5.0
    connect_timeout_seconds: float = 3.0
    connection_refused_retry_seconds: float = 1.0
    connection_refused_deadline_seconds: float = 3.0
    command_timeout_seconds: float = 3.0
    command_retries: int = 3
    restore_retries: int = 5
    restore_deadline_seconds: float = 90.0
    ensure_on_retry_seconds: float = 30.0
    precheck_retry_seconds: float = 60.0
    precheck_max_attempts: int = 5
    intent_ack_timeout_seconds: float = 3.0
    intent_ack_attempts: int = 3
    intent_retry_seconds: float = 0.5
    intent_valid_for_seconds: float = 15.0


@dataclass(frozen=True)
class PowerGroup:
    group_id: str
    label: str
    enabled: bool
    members: Tuple[str, ...]
    host: str
    port: int
    channels: Tuple[int, ...]
    address: int = 1
    allow_omitted_status_checksum: bool = False

    def __post_init__(self) -> None:
        if (
            not self.channels
            or len(self.channels) > 4
            or any(
                isinstance(channel, bool)
                or not isinstance(channel, int)
                or channel < 1
                or channel > 4
                for channel in self.channels
            )
            or tuple(sorted(set(self.channels))) != self.channels
        ):
            raise ValueError(
                "relay channels must be unique ascending integers from 1 to 4"
            )

    @property
    def power_key(self) -> str:
        """Stable selected-output identity used for rate limits and bindings."""

        if len(self.channels) == 1:
            # Preserve the deployed single-channel identity exactly so an
            # upgrade cannot reset its cooldown, daily budget, or must-be-ON
            # obligation merely because multi-channel support was added.
            channel_identity = str(self.channels[0])
        else:
            channel_identity = "channels=" + ",".join(
                str(channel) for channel in self.channels
            )
        return "legacy_tcp|%s|%d|%d|%s" % (
            self.host,
            self.port,
            self.address,
            channel_identity,
        )

    @property
    def channel_mask(self) -> int:
        return sum(1 << (channel - 1) for channel in self.channels)

    @property
    def persisted_channel(self) -> int:
        """Backward-compatible SQLite field; multi-channel rows store a mask."""

        return self.channels[0] if len(self.channels) == 1 else self.channel_mask

    @property
    def channels_text(self) -> str:
        return ",".join(str(channel) for channel in self.channels)

    @property
    def relay_endpoint_key(self) -> str:
        """Physical controller identity used to serialize every A1 writer."""

        return "legacy_tcp|%s|%d|%d" % (
            self.host,
            self.port,
            self.address,
        )


def _channels_from_persisted(power_key: str, stored: int) -> Tuple[int, ...]:
    """Decode the v5 SQLite channel field without changing old identities."""

    marker = "|channels="
    if marker not in power_key:
        if stored < 1 or stored > 4:
            raise StateStoreError("persisted single relay channel is invalid")
        return (stored,)
    if stored < 1 or stored > 0x0F:
        raise StateStoreError("persisted relay channel mask is invalid")
    channels = tuple(
        channel for channel in range(1, 5) if stored & (1 << (channel - 1))
    )
    encoded = power_key.split(marker, 1)[1]
    if encoded != ",".join(str(channel) for channel in channels):
        raise StateStoreError(
            "persisted relay channel mask does not match its power identity"
        )
    return channels


def _unexpected_selected_channels(
    target: PowerGroup, states: Sequence[bool], expected: bool
) -> List[int]:
    return [
        channel
        for channel in target.channels
        if bool(states[channel - 1]) is not expected
    ]


def _changed_non_target_channels(
    target: PowerGroup,
    baseline: Sequence[bool],
    current: Sequence[bool],
) -> List[int]:
    selected = set(target.channels)
    return [
        index + 1
        for index in range(4)
        if index + 1 not in selected
        and bool(current[index]) != bool(baseline[index])
    ]


@dataclass(frozen=True)
class ManagerConfig:
    mode: str
    state_db: str
    request_topic: str
    state_topic: str
    status_topic: str
    heartbeat_topic: str
    intent_topic: str
    intent_ack_topic: str
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
    recovery_reason: str = RECOVERY_REASON_HANDSHAKE
    wake_request_id: int = 0
    wake_connection_generation: int = 0
    wake_dropout_generation: int = 0
    wake_started_at: float = 0.0
    wake_dropout_at: float = 0.0
    wake_silence_at: float = 0.0
    normal_connection_generation: int = 0
    normal_dropout_generation: int = 0
    normal_healthy_since_at: float = 0.0
    normal_dropout_at: float = 0.0
    normal_silence_at: float = 0.0
    startup_missing_since: float = 0.0
    measurement_session_id: int = 0
    error_reboot_attempts: int = 0
    error_since_at: float = 0.0

    @property
    def identity(self) -> Tuple[Any, ...]:
        """Exact live-state identity, including the recovery cause.

        ``event_id`` intentionally retains the original schema-1 four-field
        format.  The cause and wake evidence are therefore part of every live
        pre-OFF comparison instead of being inferred from that legacy id.
        """

        return (
            self.event_id,
            self.broadcast_code,
            self.driver_instance,
            self.handle,
            self.episode_count,
            int(self.detected_at),
            self.recovery_reason,
            self.wake_request_id,
            self.wake_connection_generation,
            self.wake_dropout_generation,
            int(self.wake_started_at),
            int(self.wake_dropout_at),
            int(self.wake_silence_at),
            self.normal_connection_generation,
            self.normal_dropout_generation,
            int(self.normal_healthy_since_at),
            int(self.normal_dropout_at),
            int(self.normal_silence_at),
            int(self.startup_missing_since),
            self.measurement_session_id,
            self.error_reboot_attempts,
            int(self.error_since_at),
        )

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
        recovery_reason = _recovery_reason(
            payload, legacy_default=RECOVERY_REASON_HANDSHAKE
        )
        if recovery_reason not in RECOVERY_REASONS:
            raise ValueError("POWER_CYCLE_REQUIRED request has no valid cause")
        (
            wake_request_id,
            wake_connection_generation,
            wake_dropout_generation,
            wake_started_at,
            wake_dropout_at,
            wake_silence_at,
            normal_connection_generation,
            normal_dropout_generation,
            normal_healthy_since_at,
            normal_dropout_at,
            normal_silence_at,
            startup_missing_since,
        ) = _recovery_evidence(payload, recovery_reason)
        (
            measurement_session_id,
            error_reboot_attempts,
            error_since_at,
        ) = _error_recovery_evidence(payload, recovery_reason)
        session_reset_attempts = _integer(
            payload, "session_reset_attempts", default=0, minimum=0, maximum=255
        )
        if (
            recovery_reason == RECOVERY_REASON_HANDSHAKE
            and session_reset_attempts != 1
        ):
            raise ValueError(
                "HANDSHAKE_STUCK request requires exactly one session reset"
            )
        if (
            recovery_reason != RECOVERY_REASON_HANDSHAKE
            and session_reset_attempts != 0
        ):
            raise ValueError(
                "%s request contains unrelated session-reset evidence"
                % recovery_reason
            )
        if (
            recovery_reason == RECOVERY_REASON_WAKE_DROPOUT
            and not _wake_timing_evidence_valid(
                wake_started_at,
                wake_dropout_at,
                wake_silence_at,
                detected_at,
            )
        ):
            raise ValueError(
                "WAKE_DROPOUT request violates the 60s attribution window "
                "or 10s dropout confirmation"
            )
        if (
            recovery_reason == RECOVERY_REASON_NORMAL_DROPOUT
            and not _normal_timing_evidence_valid(
                normal_healthy_since_at,
                normal_dropout_at,
                normal_silence_at,
                detected_at,
            )
        ):
            raise ValueError(
                "NORMAL_DROPOUT request violates the 30s healthy arm or "
                "5s silence confirmation"
            )
        if (
            recovery_reason == RECOVERY_REASON_STARTUP_MISSING
            and not _startup_timing_evidence_valid(
                startup_missing_since, detected_at
            )
        ):
            raise ValueError(
                "STARTUP_MISSING request violates the 30s startup grace"
            )
        if (
            recovery_reason == RECOVERY_REASON_ERROR_REBOOT_EXHAUSTED
            and detected_at - error_since_at < _ERROR_CONFIRM_MIN_SECONDS
        ):
            raise ValueError(
                "ERROR_REBOOT_EXHAUSTED request violates the 3s Error confirmation"
            )
        if detected_at > timestamp + _EVENT_TIME_FUTURE_TOLERANCE_SECONDS:
            raise ValueError("power-cycle detection follows request timestamp")
        broadcast_fresh = payload.get("broadcast_fresh")
        if not isinstance(broadcast_fresh, bool):
            raise ValueError("broadcast_fresh must be a boolean in request")
        expected_broadcast_fresh = recovery_reason == RECOVERY_REASON_HANDSHAKE
        if (
            recovery_reason != RECOVERY_REASON_ERROR_REBOOT_EXHAUSTED
            and broadcast_fresh is not expected_broadcast_fresh
        ):
            raise ValueError(
                "%s request has inconsistent broadcast_fresh"
                % recovery_reason
            )
        if recovery_reason == RECOVERY_REASON_STARTUP_MISSING and handle != 255:
            raise ValueError("STARTUP_MISSING request must use synthetic handle 255")
        if recovery_reason != RECOVERY_REASON_STARTUP_MISSING and handle == 255:
            raise ValueError("only STARTUP_MISSING may use synthetic handle 255")
        recovery_state = payload.get("recovery_state")
        if (
            recovery_state is not None
            and recovery_state != RECOVERY_STATE_REQUIRED
        ):
            raise ValueError(
                "POWER_CYCLE_REQUIRED request has inconsistent recovery_state"
            )
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
            recovery_reason=recovery_reason,
            wake_request_id=wake_request_id,
            wake_connection_generation=wake_connection_generation,
            wake_dropout_generation=wake_dropout_generation,
            wake_started_at=wake_started_at,
            wake_dropout_at=wake_dropout_at,
            wake_silence_at=wake_silence_at,
            normal_connection_generation=normal_connection_generation,
            normal_dropout_generation=normal_dropout_generation,
            normal_healthy_since_at=normal_healthy_since_at,
            normal_dropout_at=normal_dropout_at,
            normal_silence_at=normal_silence_at,
            startup_missing_since=startup_missing_since,
            measurement_session_id=measurement_session_id,
            error_reboot_attempts=error_reboot_attempts,
            error_since_at=error_since_at,
        )

    @classmethod
    def from_state(cls, payload: Mapping[str, Any]) -> "PowerCycleRequest":
        request = _request_from_live_recovery_state(payload)
        if request is None:
            raise ValueError("recovery state does not require a power cycle")
        return request


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


def _recovery_reason(
    payload: Mapping[str, Any], *, legacy_default: str
) -> str:
    """Return a normalized recovery cause without widening legacy behavior."""

    value = payload.get("recovery_reason", legacy_default)
    if not isinstance(value, str) or value not in (
        RECOVERY_REASON_NONE,
        RECOVERY_REASON_HANDSHAKE,
        RECOVERY_REASON_WAKE_DROPOUT,
        RECOVERY_REASON_NORMAL_DROPOUT,
        RECOVERY_REASON_STARTUP_MISSING,
        RECOVERY_REASON_ERROR_REBOOT_EXHAUSTED,
    ):
        raise ValueError(
            "recovery_reason must be NONE, HANDSHAKE_STUCK, WAKE_DROPOUT, "
            "NORMAL_DROPOUT, STARTUP_MISSING, or ERROR_REBOOT_EXHAUSTED"
        )
    return value


def _recovery_evidence(
    payload: Mapping[str, Any], recovery_reason: str
) -> Tuple[
    int,
    int,
    int,
    float,
    float,
    float,
    int,
    int,
    float,
    float,
    float,
    float,
]:
    """Validate cause-specific evidence carried by an active event.

    A wake dropout is intentionally impossible to infer from a generic
    disconnected row: the Driver must identify the explicit wake request, the
    matching arm/dropout connection generations, the attributed disconnect,
    and the current continuous-silence edge. Handshake requests must not
    smuggle wake evidence into an otherwise valid legacy event.
    """

    wake_request_id = _integer(
        payload, "wake_request_id", default=0, minimum=0
    )
    wake_connection_generation = _integer(
        payload, "wake_connection_generation", default=0, minimum=0
    )
    wake_dropout_generation = _integer(
        payload, "wake_dropout_generation", default=0, minimum=0
    )
    wake_started_at = _number(
        payload, "wake_started_at", default=0, minimum=0
    )
    wake_dropout_at = _number(
        payload, "wake_dropout_at", default=0, minimum=0
    )
    wake_silence_at = _number(
        payload, "wake_silence_at", default=0, minimum=0
    )
    normal_connection_generation = _integer(
        payload, "normal_connection_generation", default=0, minimum=0
    )
    normal_dropout_generation = _integer(
        payload, "normal_dropout_generation", default=0, minimum=0
    )
    normal_healthy_since_at = _number(
        payload, "normal_healthy_since_at", default=0, minimum=0
    )
    normal_dropout_at = _number(
        payload, "normal_dropout_at", default=0, minimum=0
    )
    normal_silence_at = _number(
        payload, "normal_silence_at", default=0, minimum=0
    )
    startup_missing_since = _number(
        payload, "startup_missing_since", default=0, minimum=0
    )
    wake_present = (
        wake_request_id != 0
        or wake_connection_generation != 0
        or wake_dropout_generation != 0
        or wake_started_at != 0
        or wake_dropout_at != 0
        or wake_silence_at != 0
    )
    normal_present = (
        normal_connection_generation != 0
        or normal_dropout_generation != 0
        or normal_healthy_since_at != 0
        or normal_dropout_at != 0
        or normal_silence_at != 0
    )
    if recovery_reason == RECOVERY_REASON_WAKE_DROPOUT:
        if (
            wake_request_id <= 0
            or wake_connection_generation <= 0
            or wake_dropout_generation <= 0
            or wake_connection_generation != wake_dropout_generation
            or wake_started_at <= 0
            or wake_dropout_at <= 0
            or wake_silence_at <= 0
            or wake_started_at > wake_dropout_at
            or wake_dropout_at > wake_silence_at
        ):
            raise ValueError(
                "WAKE_DROPOUT requires ordered positive wake evidence"
            )
        if normal_present or startup_missing_since != 0:
            raise ValueError("WAKE_DROPOUT contains unrelated recovery evidence")
    elif recovery_reason == RECOVERY_REASON_NORMAL_DROPOUT:
        if (
            normal_connection_generation <= 0
            or normal_dropout_generation <= 0
            or normal_connection_generation != normal_dropout_generation
            or normal_healthy_since_at <= 0
            or normal_dropout_at <= 0
            or normal_silence_at <= 0
            or normal_healthy_since_at > normal_dropout_at
            or normal_dropout_at > normal_silence_at
        ):
            raise ValueError(
                "NORMAL_DROPOUT requires ordered positive normal evidence"
            )
        if wake_present or startup_missing_since != 0:
            raise ValueError("NORMAL_DROPOUT contains unrelated recovery evidence")
    elif recovery_reason == RECOVERY_REASON_STARTUP_MISSING:
        if startup_missing_since <= 0:
            raise ValueError("STARTUP_MISSING requires a positive startup timestamp")
        if wake_present or normal_present:
            raise ValueError("STARTUP_MISSING contains unrelated recovery evidence")
    elif wake_present or normal_present or startup_missing_since != 0:
        raise ValueError(
            "HANDSHAKE_STUCK request contains unrelated recovery evidence"
        )
    return (
        wake_request_id,
        wake_connection_generation,
        wake_dropout_generation,
        wake_started_at,
        wake_dropout_at,
        wake_silence_at,
        normal_connection_generation,
        normal_dropout_generation,
        normal_healthy_since_at,
        normal_dropout_at,
        normal_silence_at,
        startup_missing_since,
    )


def _error_recovery_evidence(
    payload: Mapping[str, Any], recovery_reason: str
) -> Tuple[int, int, float]:
    measurement_session_id = _integer(
        payload, "measurement_session_id", default=0, minimum=0
    )
    error_reboot_attempts = _integer(
        payload, "error_reboot_attempts", default=0, minimum=0, maximum=255
    )
    error_since_at = _number(
        payload, "error_since_at", default=0, minimum=0
    )
    if recovery_reason == RECOVERY_REASON_ERROR_REBOOT_EXHAUSTED:
        if (
            measurement_session_id <= 0
            or error_reboot_attempts != _ERROR_REBOOT_ATTEMPTS_REQUIRED
            or error_since_at <= 0
        ):
            raise ValueError(
                "ERROR_REBOOT_EXHAUSTED requires a positive measurement "
                "session, exactly three accepted soft reboots, and a positive "
                "Error onset timestamp"
            )
    elif (
        measurement_session_id != 0
        or error_reboot_attempts != 0
        or error_since_at != 0
    ):
        raise ValueError(
            "%s contains unrelated measurement Error evidence" % recovery_reason
        )
    return measurement_session_id, error_reboot_attempts, error_since_at


def _wake_timing_evidence_valid(
    wake_started_at: float,
    wake_dropout_at: float,
    wake_silence_at: float,
    detected_at: float,
) -> bool:
    """Apply the Driver's wake attribution and confirmation bounds again."""

    attribution_seconds = wake_dropout_at - wake_started_at
    confirmation_seconds = detected_at - wake_silence_at
    return (
        0 <= attribution_seconds <= _WAKE_OBSERVATION_MAX_SECONDS
        and confirmation_seconds >= _WAKE_DROPOUT_CONFIRM_MIN_SECONDS
    )


def _normal_timing_evidence_valid(
    normal_healthy_since_at: float,
    normal_dropout_at: float,
    normal_silence_at: float,
    detected_at: float,
) -> bool:
    """Recheck sustained pre-fault health and current continuous silence."""

    return (
        normal_dropout_at - normal_healthy_since_at
        >= _NORMAL_HEALTHY_ARM_MIN_SECONDS
        and detected_at - normal_silence_at
        >= _NORMAL_DROPOUT_CONFIRM_MIN_SECONDS
    )


def _startup_timing_evidence_valid(
    startup_missing_since: float, detected_at: float
) -> bool:
    return (
        detected_at - startup_missing_since
        >= _STARTUP_MISSING_GRACE_MIN_SECONDS
    )


def _request_from_live_recovery_state(
    payload: Mapping[str, Any],
) -> Optional[PowerCycleRequest]:
    """Validate and normalize one live recovery-state row.

    This is the single authority used both when a ROS state is cached and at
    every pre-OFF recheck.  Legacy schema-1 handshake rows did not include the
    generic recovery fields; only their existing
    ``handshake_state=POWER_CYCLE_REQUIRED`` shape is inferred.
    """

    broadcast_code = _broadcast_code(payload.get("broadcast_code"))
    state_timestamp = _number(payload, "timestamp", minimum=0)
    driver_instance = _integer(payload, "driver_instance", minimum=0)
    handle = _integer(payload, "handle", minimum=0, maximum=255)
    connected = payload.get("connected")
    broadcast_fresh = payload.get("broadcast_fresh")
    publishing = payload.get("publishing")
    for name, value in (
        ("connected", connected),
        ("broadcast_fresh", broadcast_fresh),
        ("publishing", publishing),
    ):
        if not isinstance(value, bool):
            raise ValueError("%s must be a boolean in recovery state" % name)

    connect_state = _required_text(payload, "connect_state", maximum=32)
    lidar_state = _required_text(payload, "lidar_state", maximum=32)
    handshake_state = _required_text(
        payload, "handshake_state", maximum=64
    )
    explicit_recovery_state = payload.get("recovery_state")
    if explicit_recovery_state is None:
        recovery_state = (
            RECOVERY_STATE_REQUIRED
            if handshake_state == REQUEST_TYPE
            else RECOVERY_STATE_IDLE
        )
    else:
        if explicit_recovery_state not in (
            RECOVERY_STATE_IDLE,
            RECOVERY_STATE_REQUIRED,
        ):
            raise ValueError(
                "recovery_state must be IDLE or POWER_CYCLE_REQUIRED"
            )
        recovery_state = explicit_recovery_state

    legacy_reason = (
        RECOVERY_REASON_HANDSHAKE
        if recovery_state == RECOVERY_STATE_REQUIRED
        and handshake_state == REQUEST_TYPE
        else RECOVERY_REASON_NONE
    )
    recovery_reason = _recovery_reason(
        payload, legacy_default=legacy_reason
    )
    wake_state = payload.get("wake_state", RECOVERY_STATE_IDLE)
    if not isinstance(wake_state, str) or not wake_state:
        raise ValueError("wake_state must be a non-empty string")
    normal_state = payload.get("normal_state", RECOVERY_STATE_IDLE)
    if not isinstance(normal_state, str) or not normal_state:
        raise ValueError("normal_state must be a non-empty string")
    startup_state = payload.get("startup_state", RECOVERY_STATE_IDLE)
    if not isinstance(startup_state, str) or not startup_state:
        raise ValueError("startup_state must be a non-empty string")

    if recovery_state == RECOVERY_STATE_IDLE:
        if (
            recovery_reason != RECOVERY_REASON_NONE
            or handshake_state == REQUEST_TYPE
            or wake_state == WAKE_STATE_REQUIRED
            or normal_state == RECOVERY_STATE_REQUIRED
            or startup_state == RECOVERY_STATE_REQUIRED
        ):
            raise ValueError(
                "recovery state is internally inconsistent: IDLE has an "
                "active recovery cause"
            )
        return None

    if recovery_reason not in RECOVERY_REASONS:
        raise ValueError(
            "recovery state is internally inconsistent: active state has no "
            "valid cause"
        )
    episode_count = _integer(
        payload, "power_cycle_required_count", minimum=1
    )
    detected_at = _number(
        payload, "power_cycle_required_at", minimum=0
    )
    if detected_at <= 0:
        raise ValueError(
            "recovery state is internally inconsistent: detected time is zero"
        )
    if (
        detected_at
        > state_timestamp + _EVENT_TIME_FUTURE_TOLERANCE_SECONDS
    ):
        raise ValueError(
            "recovery state is internally inconsistent: power-cycle "
            "detection follows state timestamp"
        )
    if recovery_reason == RECOVERY_REASON_ERROR_REBOOT_EXHAUSTED:
        if (
            connected is not True
            or connect_state == "Off"
            or lidar_state != "Error"
            or publishing is not False
            or handle == 255
        ):
            raise ValueError(
                "recovery state is internally inconsistent: "
                "ERROR_REBOOT_EXHAUSTED requires a connected real handle in "
                "Error and no point publication"
            )
    elif (
        connected is not False
        or connect_state != "Off"
        or publishing is not False
    ):
        raise ValueError(
            "recovery state is internally inconsistent: power-cycle target "
            "must be disconnected Off and not publishing"
        )

    if recovery_reason == RECOVERY_REASON_HANDSHAKE:
        if (
            handshake_state != REQUEST_TYPE
            or broadcast_fresh is not True
            or wake_state != RECOVERY_STATE_IDLE
            or normal_state != RECOVERY_STATE_IDLE
            or startup_state != RECOVERY_STATE_IDLE
        ):
            raise ValueError(
                "recovery state is internally inconsistent: "
                "HANDSHAKE_STUCK requires a fresh broadcast"
            )
    elif recovery_reason == RECOVERY_REASON_WAKE_DROPOUT:
        if (
            handshake_state != RECOVERY_STATE_IDLE
            or wake_state != WAKE_STATE_REQUIRED
            or broadcast_fresh is not False
            or normal_state != RECOVERY_STATE_IDLE
            or startup_state != RECOVERY_STATE_IDLE
        ):
            raise ValueError(
                "recovery state is internally inconsistent: WAKE_DROPOUT "
                "requires IDLE handshake and absent broadcast"
            )
    elif recovery_reason == RECOVERY_REASON_NORMAL_DROPOUT:
        if (
            handshake_state != RECOVERY_STATE_IDLE
            or wake_state != RECOVERY_STATE_IDLE
            or normal_state != RECOVERY_STATE_REQUIRED
            or startup_state != RECOVERY_STATE_IDLE
            or broadcast_fresh is not False
            or handle == 255
        ):
            raise ValueError(
                "recovery state is internally inconsistent: NORMAL_DROPOUT "
                "requires IDLE handshake/wake, absent broadcast, and a real handle"
            )
    elif recovery_reason == RECOVERY_REASON_STARTUP_MISSING:
        if (
            handle != 255
            or handshake_state != RECOVERY_STATE_IDLE
            or wake_state != RECOVERY_STATE_IDLE
            or normal_state != RECOVERY_STATE_IDLE
            or startup_state != RECOVERY_STATE_REQUIRED
            or broadcast_fresh is not False
        ):
            raise ValueError(
                "recovery state is internally inconsistent: STARTUP_MISSING "
                "requires synthetic handle 255 and no live link evidence"
            )
    else:
        if (
            handshake_state != RECOVERY_STATE_IDLE
            or wake_state != RECOVERY_STATE_IDLE
            or normal_state != RECOVERY_STATE_IDLE
            or startup_state != RECOVERY_STATE_IDLE
            or handle == 255
        ):
            raise ValueError(
                "recovery state is internally inconsistent: "
                "ERROR_REBOOT_EXHAUSTED requires no competing recovery cause"
            )

    (
        wake_request_id,
        wake_connection_generation,
        wake_dropout_generation,
        wake_started_at,
        wake_dropout_at,
        wake_silence_at,
        normal_connection_generation,
        normal_dropout_generation,
        normal_healthy_since_at,
        normal_dropout_at,
        normal_silence_at,
        startup_missing_since,
    ) = _recovery_evidence(payload, recovery_reason)
    (
        measurement_session_id,
        error_reboot_attempts,
        error_since_at,
    ) = _error_recovery_evidence(payload, recovery_reason)
    if (
        recovery_reason == RECOVERY_REASON_WAKE_DROPOUT
        and not _wake_timing_evidence_valid(
            wake_started_at,
            wake_dropout_at,
            wake_silence_at,
            detected_at,
        )
    ):
        raise ValueError(
            "recovery state is internally inconsistent: WAKE_DROPOUT "
            "violates the 60s attribution window or 10s confirmation"
        )
    if (
        recovery_reason == RECOVERY_REASON_NORMAL_DROPOUT
        and not _normal_timing_evidence_valid(
            normal_healthy_since_at,
            normal_dropout_at,
            normal_silence_at,
            detected_at,
        )
    ):
        raise ValueError(
            "recovery state is internally inconsistent: NORMAL_DROPOUT "
            "violates the 30s healthy arm or 5s silence confirmation"
        )
    if (
        recovery_reason == RECOVERY_REASON_STARTUP_MISSING
        and not _startup_timing_evidence_valid(
            startup_missing_since, detected_at
        )
    ):
        raise ValueError(
            "recovery state is internally inconsistent: STARTUP_MISSING "
            "violates the 30s startup grace"
        )
    if (
        recovery_reason == RECOVERY_REASON_ERROR_REBOOT_EXHAUSTED
        and detected_at - error_since_at < _ERROR_CONFIRM_MIN_SECONDS
    ):
        raise ValueError(
            "recovery state is internally inconsistent: "
            "ERROR_REBOOT_EXHAUSTED violates the 3s Error confirmation"
        )
    event_id = "%s:%d:%d:%d" % (
        broadcast_code,
        driver_instance,
        int(detected_at),
        episode_count,
    )
    return PowerCycleRequest(
        event_id=event_id,
        broadcast_code=broadcast_code,
        timestamp=time.time(),
        detected_at=detected_at,
        driver_instance=driver_instance,
        handle=handle,
        episode_count=episode_count,
        recovery_reason=recovery_reason,
        wake_request_id=wake_request_id,
        wake_connection_generation=wake_connection_generation,
        wake_dropout_generation=wake_dropout_generation,
        wake_started_at=wake_started_at,
        wake_dropout_at=wake_dropout_at,
        wake_silence_at=wake_silence_at,
        normal_connection_generation=normal_connection_generation,
        normal_dropout_generation=normal_dropout_generation,
        normal_healthy_since_at=normal_healthy_since_at,
        normal_dropout_at=normal_dropout_at,
        normal_silence_at=normal_silence_at,
        startup_missing_since=startup_missing_since,
        measurement_session_id=measurement_session_id,
        error_reboot_attempts=error_reboot_attempts,
        error_since_at=error_since_at,
    )


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
            "connection_refused_retry_seconds",
            "connection_refused_deadline_seconds",
            "command_timeout_seconds",
            "command_retries",
            "restore_retries",
            "restore_deadline_seconds",
            "ensure_on_retry_seconds",
            "precheck_retry_seconds",
            "precheck_max_attempts",
            "intent_ack_timeout_seconds",
            "intent_ack_attempts",
            "intent_retry_seconds",
            "intent_valid_for_seconds",
        ),
        "policy",
    )
    policy = Policy(
        off_seconds=_number(
            data,
            "off_seconds",
            default=_DEFAULT_OFF_SECONDS,
            minimum=_MINIMUM_OFF_SECONDS,
            maximum=120,
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
        connection_refused_retry_seconds=_number(
            data,
            "connection_refused_retry_seconds",
            default=1,
            minimum=0.2,
            maximum=5,
        ),
        connection_refused_deadline_seconds=_number(
            data,
            "connection_refused_deadline_seconds",
            default=3,
            minimum=1,
            maximum=15,
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
        intent_ack_timeout_seconds=_number(
            data,
            "intent_ack_timeout_seconds",
            default=3,
            minimum=1,
            maximum=10,
        ),
        intent_ack_attempts=_integer(
            data,
            "intent_ack_attempts",
            default=3,
            minimum=1,
            maximum=5,
        ),
        intent_retry_seconds=_number(
            data,
            "intent_retry_seconds",
            default=0.5,
            minimum=0,
            maximum=5,
        ),
        intent_valid_for_seconds=_number(
            data,
            "intent_valid_for_seconds",
            default=15,
            minimum=8,
            maximum=60,
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
        topics,
        ("request", "state", "status", "heartbeat", "intent", "intent_ack"),
        "topics",
    )
    policy = _policy_from_json(raw.get("policy", {}))
    group_rows = raw.get("power_groups", {})
    if not isinstance(group_rows, Mapping):
        raise ConfigurationError("power_groups must be an object keyed by group id")
    power_groups: Dict[str, PowerGroup] = {}
    member_to_group: Dict[str, str] = {}
    occupied: Dict[Tuple[str, int, int, int], str] = {}
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
                "channels",
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
        has_channel = "channel" in relay
        has_channels = "channels" in relay
        if has_channel == has_channels:
            raise ConfigurationError(
                "power_groups.%s.relay must define exactly one of channel or "
                "channels" % group_id
            )
        if has_channel:
            channels = (_integer(relay, "channel", minimum=1, maximum=4),)
        else:
            raw_channels = relay.get("channels")
            if not isinstance(raw_channels, list) or not raw_channels:
                raise ConfigurationError(
                    "power_groups.%s.relay.channels must be a non-empty list"
                    % group_id
                )
            parsed_channels: List[int] = []
            for index, raw_channel in enumerate(raw_channels):
                if isinstance(raw_channel, bool) or not isinstance(raw_channel, int):
                    raise ConfigurationError(
                        "power_groups.%s.relay.channels[%d] must be an integer"
                        % (group_id, index)
                    )
                if raw_channel < 1 or raw_channel > 4:
                    raise ConfigurationError(
                        "power_groups.%s.relay.channels[%d] must be between 1 and 4"
                        % (group_id, index)
                    )
                if raw_channel in parsed_channels:
                    raise ConfigurationError(
                        "power_groups.%s.relay.channels contains duplicate %d"
                        % (group_id, raw_channel)
                    )
                parsed_channels.append(raw_channel)
            channels = tuple(sorted(parsed_channels))
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
            channels=channels,
            address=address,
            allow_omitted_status_checksum=allow_omitted,
        )
        if enabled:
            for channel in target.channels:
                endpoint = (target.host, target.port, target.address, channel)
                previous = occupied.get(endpoint)
                if previous is not None:
                    raise ConfigurationError(
                        "enabled power groups %s and %s share relay %s:%d "
                        "address %d channel %d"
                        % (
                            previous,
                            group_id,
                            target.host,
                            target.port,
                            target.address,
                            channel,
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
        intent_topic=_topic(
            topics, "intent", "/livox/group_power_cycle_intent"
        ),
        intent_ack_topic=_topic(
            topics, "intent_ack", "/livox/group_power_cycle_ack"
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
            "recovery_reason",
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
            "recovery_reason",
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
            "severity",
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
        if version not in {0, 2, 3, 4, 5, 6, STATE_DB_SCHEMA_VERSION}:
            raise StateStoreError(
                "unsupported state database schema version %d "
                "(expected 2, 3, 4, 5, 6, or %d)"
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

        if version in {2, 3}:
            # Every schema-2/3 row predates WAKE_DROPOUT, so
            # HANDSHAKE_STUCK is the only safe and exact migration default.
            for table in ("power_events", "power_obligations"):
                # Older databases may legitimately have been created before
                # either table was first needed.  CREATE TABLE below will
                # install the complete v4 shape; only existing tables need an
                # in-place compatibility check and ALTER.
                if table not in tables:
                    continue
                expected_previous = tuple(
                    column
                    for column in self._EXPECTED_COLUMNS[table]
                    if column != "recovery_reason"
                )
                actual_previous = tuple(
                    str(row[1])
                    for row in db.execute(
                        "PRAGMA table_info(%s)" % table
                    ).fetchall()
                )
                if actual_previous != expected_previous:
                    raise StateStoreError(
                        "state table %s cannot be migrated to schema 6: %s"
                        % (table, ",".join(actual_previous))
                    )
                db.execute(
                    "ALTER TABLE %s ADD COLUMN recovery_reason TEXT "
                    "NOT NULL DEFAULT 'HANDSHAKE_STUCK' "
                    "CHECK(recovery_reason IN "
                    "('HANDSHAKE_STUCK','WAKE_DROPOUT','NORMAL_DROPOUT',"
                    "'STARTUP_MISSING','ERROR_REBOOT_EXHAUSTED'))" % table
                )

        if version in {4, 5}:
            # SQLite cannot widen a column CHECK constraint with ALTER TABLE.
            # Rebuild both reason-bearing tables inside this existing IMMEDIATE
            # transaction so a crash leaves either the complete old database or
            # the complete v6 database, never one table of each.
            for table in ("power_events", "power_obligations"):
                if table not in tables:
                    continue
                actual = tuple(
                    str(row[1])
                    for row in db.execute(
                        "PRAGMA table_info(%s)" % table
                    ).fetchall()
                )
                if actual != self._EXPECTED_COLUMNS[table]:
                    raise StateStoreError(
                        "state table %s cannot be migrated from schema %d: %s"
                        % (table, version, ",".join(actual))
                    )
            if "power_events" in tables:
                db.execute(
                    """
                    CREATE TABLE power_events_v6 (
                      event_id TEXT PRIMARY KEY,
                      trigger_bcode TEXT NOT NULL,
                      group_id TEXT NOT NULL,
                      power_key TEXT NOT NULL,
                      status TEXT NOT NULL,
                      attempts INTEGER NOT NULL DEFAULT 0,
                      next_attempt REAL NOT NULL DEFAULT 0,
                      first_seen REAL NOT NULL,
                      last_update REAL NOT NULL,
                      detail TEXT NOT NULL DEFAULT '',
                      recovery_reason TEXT NOT NULL CHECK(recovery_reason IN
                        ('HANDSHAKE_STUCK','WAKE_DROPOUT','NORMAL_DROPOUT',
                         'STARTUP_MISSING','ERROR_REBOOT_EXHAUSTED'))
                    )
                    """
                )
                db.execute(
                    "INSERT INTO power_events_v6 SELECT * FROM power_events"
                )
                db.execute("DROP TABLE power_events")
                db.execute("ALTER TABLE power_events_v6 RENAME TO power_events")
            if "power_obligations" in tables:
                db.execute(
                    """
                    CREATE TABLE power_obligations_v6 (
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
                      last_attempt REAL NOT NULL DEFAULT 0,
                      recovery_reason TEXT NOT NULL CHECK(recovery_reason IN
                        ('HANDSHAKE_STUCK','WAKE_DROPOUT','NORMAL_DROPOUT',
                         'STARTUP_MISSING','ERROR_REBOOT_EXHAUSTED'))
                    )
                    """
                )
                db.execute(
                    "INSERT INTO power_obligations_v6 "
                    "SELECT * FROM power_obligations"
                )
                db.execute("DROP TABLE power_obligations")
                db.execute(
                    "ALTER TABLE power_obligations_v6 "
                    "RENAME TO power_obligations"
                )

        if version in {2, 3, 4, 5, 6} and "power_alarms" in tables:
            expected_v6_alarm = tuple(
                column
                for column in self._EXPECTED_COLUMNS["power_alarms"]
                if column != "severity"
            )
            actual_alarm = tuple(
                str(row[1])
                for row in db.execute(
                    "PRAGMA table_info(power_alarms)"
                ).fetchall()
            )
            expected_v7_alarm = self._EXPECTED_COLUMNS["power_alarms"]
            if actual_alarm not in {expected_v6_alarm, expected_v7_alarm}:
                raise StateStoreError(
                    "state table power_alarms cannot be migrated to schema 7: %s"
                    % ",".join(actual_alarm)
                )
            if actual_alarm == expected_v6_alarm:
                db.execute(
                    "ALTER TABLE power_alarms ADD COLUMN severity TEXT NOT NULL "
                    "DEFAULT 'CRITICAL' CHECK(severity IN "
                    "('INFO','WARN','ERROR','CRITICAL'))"
                )
            # Schema <=6 replayed every alarm as CRITICAL and incorrectly kept
            # cooldown suppression forever. Preserve durable safety alarms,
            # but infer their least-surprising original severity and archive
            # obsolete cooldown rows instead of deleting the event history.
            db.execute(
                "UPDATE power_alarms SET severity=CASE state "
                "WHEN 'TARGET_DISABLED' THEN 'WARN' "
                "WHEN 'UNMAPPED' THEN 'ERROR' "
                "WHEN 'PRECHECK_FAILED' THEN 'ERROR' "
                "WHEN 'POWER_CYCLE_FAILED' THEN 'ERROR' "
                "ELSE 'CRITICAL' END"
            )
            db.execute(
                "DELETE FROM power_alarms WHERE state='SUPPRESSED_COOLDOWN'"
            )

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
              detail TEXT NOT NULL DEFAULT '',
              recovery_reason TEXT NOT NULL
                CHECK(recovery_reason IN
                  ('HANDSHAKE_STUCK','WAKE_DROPOUT','NORMAL_DROPOUT',
                   'STARTUP_MISSING','ERROR_REBOOT_EXHAUSTED'))
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
              last_attempt REAL NOT NULL DEFAULT 0,
              recovery_reason TEXT NOT NULL
                CHECK(recovery_reason IN
                  ('HANDSHAKE_STUCK','WAKE_DROPOUT','NORMAL_DROPOUT',
                   'STARTUP_MISSING','ERROR_REBOOT_EXHAUSTED'))
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
              updated_at REAL NOT NULL,
              severity TEXT NOT NULL
                CHECK(severity IN ('INFO','WARN','ERROR','CRITICAL'))
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
                "last_update,recovery_reason) VALUES(?,?,?,?,?,?,?,?)",
                (
                    request.event_id,
                    request.broadcast_code,
                    group_id,
                    power_key,
                    "NEW",
                    now,
                    now,
                    request.recovery_reason,
                ),
            )
            row = db.execute(
                "SELECT status,attempts,next_attempt,last_update,group_id,"
                "power_key,recovery_reason "
                "FROM power_events "
                "WHERE event_id=?",
                (request.event_id,),
            ).fetchone()
            assert row is not None
            (
                status,
                attempts,
                next_attempt,
                last_update,
                stored_group,
                stored_key,
                stored_reason,
            ) = row
            if status in TERMINAL_EVENT_STATES:
                return False, int(attempts), "terminal"
            if status == "OBSERVED" and not allow_observed:
                return False, int(attempts), "observed"
            mapping_changed = (
                stored_group != group_id or stored_key != power_key
            )
            reason_changed = stored_reason != request.recovery_reason
            if mapping_changed or reason_changed:
                # Observe mode may safely adopt a newly armed whitelist
                # mapping, but it must never reinterpret an existing legacy
                # event_id as a different recovery cause.  The schema-1 event
                # id does not include recovery_reason, so cause changes are an
                # identity collision and must fail closed.
                if (
                    status == "OBSERVED"
                    and allow_observed
                    and not reason_changed
                ):
                    db.execute(
                        "UPDATE power_events SET group_id=?,power_key=?,"
                        "status='NEW',last_update=?,detail='' WHERE event_id=?",
                        (
                            group_id,
                            power_key,
                            now,
                            request.event_id,
                        ),
                    )
                    status = "NEW"
                    stored_group = group_id
                    stored_key = power_key
                else:
                    detail = (
                        "event identity mapping/reason changed from %s/%s/%s "
                        "to %s/%s/%s; "
                        "automatic power action refused"
                        % (
                            stored_group,
                            stored_key,
                            stored_reason,
                            group_id,
                            power_key,
                            request.recovery_reason,
                        )
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
            if status == "DEFERRED_COOLDOWN" and retry_remaining > 0:
                return False, int(attempts), "cooldown_wait"
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
        severity: str = "CRITICAL",
    ) -> None:
        if severity not in _ALARM_SEVERITIES:
            raise ValueError("invalid durable alarm severity: %s" % severity)
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
                "trigger_bcode,group_id,state,detail,updated_at,severity) "
                "VALUES(?,?,?,?,?,?,?,?)",
                (
                    power_key,
                    event_id,
                    trigger,
                    group_id,
                    status,
                    detail[:1000],
                    now,
                    severity,
                ),
            )
        elif status in RESOLVED_ALARM_STATES:
            db.execute(
                "DELETE FROM power_alarms WHERE power_key=?", (power_key,)
            )

    def finish_event(
        self,
        event_id: str,
        status: str,
        detail: str,
        severity: str = "CRITICAL",
    ) -> None:
        if status not in TERMINAL_EVENT_STATES:
            raise ValueError("event status is not terminal: %s" % status)
        now = time.time()
        with self._db() as db:
            db.execute(
                "UPDATE power_events SET status=?,last_update=?,detail=? "
                "WHERE event_id=?",
                (status, now, detail[:1000], event_id),
            )
            self._update_alarm(
                db, event_id, status, detail, now, severity=severity
            )

    def defer_cooldown(
        self, event_id: str, delay: float, detail: str
    ) -> float:
        """Persist an unresolved event without consuming another cycle.

        Live 1 Hz Driver state re-offers the same event. Before ``eligible_at``
        it remains deferred; afterwards the complete live precheck runs again.
        A recovered Driver state can explicitly archive it via
        ``resolve_deferred_for_trigger``.
        """

        if not math.isfinite(delay) or delay <= 0:
            raise ValueError("cooldown delay must be positive")
        now = time.time()
        eligible_at = now + delay
        with self._db() as db:
            updated = db.execute(
                "UPDATE power_events SET status='DEFERRED_COOLDOWN',"
                "next_attempt=?,last_update=?,detail=? WHERE event_id=? "
                "AND status='PROCESSING'",
                (eligible_at, now, detail[:1000], event_id),
            )
            if updated.rowcount != 1:
                raise StateStoreError(
                    "cooldown event is no longer in PROCESSING state"
                )
        return eligible_at

    def deferred_events_for_trigger(
        self, broadcast_code: str
    ) -> List[Tuple[str, str, str, float, float]]:
        with self._db() as db:
            rows = db.execute(
                "SELECT event_id,recovery_reason,detail,first_seen,next_attempt "
                "FROM power_events WHERE trigger_bcode=? "
                "AND status='DEFERRED_COOLDOWN' ORDER BY first_seen",
                (broadcast_code,),
            ).fetchall()
        return [
            (str(event_id), str(reason), str(detail), float(first), float(next_at))
            for event_id, reason, detail, first, next_at in rows
        ]

    def deferred_event_info(
        self, event_id: str
    ) -> Optional[Tuple[str, float]]:
        with self._db() as db:
            row = db.execute(
                "SELECT detail,next_attempt FROM power_events "
                "WHERE event_id=? AND status='DEFERRED_COOLDOWN'",
                (event_id,),
            ).fetchone()
        if row is None:
            return None
        return str(row[0]), float(row[1])

    def resolve_deferred_event(self, event_id: str, detail: str) -> bool:
        now = time.time()
        with self._db() as db:
            updated = db.execute(
                "UPDATE power_events SET status='STALE_OR_RECOVERED',"
                "next_attempt=0,last_update=?,detail=? WHERE event_id=? "
                "AND status='DEFERRED_COOLDOWN'",
                (now, detail[:1000], event_id),
            )
            if updated.rowcount:
                self._update_alarm(
                    db,
                    event_id,
                    "STALE_OR_RECOVERED",
                    detail,
                    now,
                    severity="INFO",
                )
        return updated.rowcount == 1

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

        The caller must first confirm every selected physical channel ON via B0.
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
                "last_attempt,recovery_reason) "
                "VALUES(?,?,?,?,?,?,?,?,?,?,?,?,0,?)",
                (
                    target.power_key,
                    target.group_id,
                    request.event_id,
                    request.broadcast_code,
                    json.dumps(list(target.members), separators=(",", ":")),
                    target.host,
                    target.port,
                    target.persisted_channel,
                    target.address,
                    1 if target.allow_omitted_status_checksum else 0,
                    target.label,
                    time.time(),
                    request.recovery_reason,
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

    def obligations(
        self,
    ) -> List[Tuple[PowerGroup, str, str, str, float]]:
        with self._db() as db:
            rows = db.execute(
                "SELECT power_key,group_id,event_id,trigger_bcode,members_json,"
                "host,port,channel,address,allow_omitted_checksum,label,"
                "recovery_reason,last_attempt "
                "FROM power_obligations"
            ).fetchall()
        result: List[Tuple[PowerGroup, str, str, str, float]] = []
        for row in rows:
            try:
                raw_members = json.loads(str(row[4]))
                if not isinstance(raw_members, list) or len(raw_members) != 4:
                    raise ValueError("members snapshot must contain exactly four")
                members = tuple(_broadcast_code(item) for item in raw_members)
            except (ValueError, TypeError, json.JSONDecodeError) as exc:
                raise StateStoreError(
                    "invalid members snapshot in persisted ON obligation: %s" % exc
                ) from exc
            power_key = str(row[0])
            target = PowerGroup(
                group_id=str(row[1]),
                label=str(row[10]),
                enabled=True,
                members=members,
                host=str(row[5]),
                port=int(row[6]),
                channels=_channels_from_persisted(power_key, int(row[7])),
                address=int(row[8]),
                allow_omitted_status_checksum=bool(row[9]),
            )
            if target.power_key != power_key:
                raise StateStoreError(
                    "persisted ON obligation relay identity is inconsistent"
                )
            reason = str(row[11])
            if reason not in RECOVERY_REASONS:
                raise StateStoreError(
                    "invalid recovery reason in persisted ON obligation: %s"
                    % reason
                )
            result.append(
                (target, str(row[2]), str(row[3]), reason, float(row[12]))
            )
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
        reported_detail = "startup/watchdog confirmed selected relay channels ON"
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
                            target.persisted_channel,
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

    def current_alerts(
        self,
    ) -> List[Tuple[str, str, str, str, str, str, str, float, float]]:
        """Return the durable still-actionable alarm snapshot per endpoint."""

        with self._db() as db:
            rows = db.execute(
                "SELECT a.event_id,a.trigger_bcode,a.group_id,a.state,a.detail,"
                "e.recovery_reason,a.severity,e.first_seen,a.updated_at "
                "FROM power_alarms AS a "
                "LEFT JOIN power_events AS e ON e.event_id=a.event_id "
                "ORDER BY a.updated_at"
            ).fetchall()
        result = []
        for (
            event_id,
            trigger,
            group_id,
            status,
            detail,
            reason,
            severity,
            first_seen,
            updated_at,
        ) in rows:
            if reason not in RECOVERY_REASONS:
                raise StateStoreError(
                    "durable alarm has no valid recovery reason: %s"
                    % event_id
                )
            if severity not in _ALARM_SEVERITIES:
                raise StateStoreError(
                    "durable alarm has invalid severity: %s" % event_id
                )
            if (
                not isinstance(first_seen, (int, float))
                or not math.isfinite(float(first_seen))
                or not isinstance(updated_at, (int, float))
                or not math.isfinite(float(updated_at))
            ):
                raise StateStoreError(
                    "durable alarm has invalid timestamps: %s" % event_id
                )
            result.append(
                (
                    str(event_id),
                    str(trigger),
                    str(group_id),
                    str(status),
                    str(detail),
                    str(reason),
                    str(severity),
                    float(first_seen),
                    float(updated_at),
                )
            )
        return result


def _double_checksum(body: bytes) -> bytes:
    first = sum(body) & 0xFF
    return bytes((first, (first * 2) & 0xFF))


class CorxLegacyTcpClient:
    """Verified whitelisted-channel client; never exposes an unscoped all-off."""

    def __init__(self, target: PowerGroup, policy: Policy) -> None:
        self.target = target
        self.policy = policy
        self._pending_protocol_warnings: List[str] = []
        self._transaction_active = False
        self._transaction_socket: Optional[socket.socket] = None

    def _record_protocol_warning(self, warning: str) -> None:
        if warning not in self._pending_protocol_warnings:
            self._pending_protocol_warnings.append(warning)

    def _take_protocol_warnings(self) -> List[str]:
        warnings = self._pending_protocol_warnings
        self._pending_protocol_warnings = []
        return warnings

    def _query_frame(self) -> bytes:
        body = bytes((0xB0, self.target.address, 0x00, 0x00, 0x0D))
        return _LEGACY_COMMAND_HEADER + body + _double_checksum(body)

    def _set_frame(self, state: bool) -> bytes:
        enable_mask = self.target.channel_mask
        control_mask = enable_mask if state else 0
        body = bytes((0xA1, self.target.address))
        body += control_mask.to_bytes(2, "big")
        body += enable_mask.to_bytes(2, "big")
        return _LEGACY_COMMAND_HEADER + body + _double_checksum(body)

    def _verified_single_checksum_status(self, frame: bytes) -> bool:
        if len(frame) != 8 or not frame.startswith(_LEGACY_STATUS_HEADER):
            return False
        if frame[3] != self.target.address or frame[6] != 0x0D:
            return False
        mask = int.from_bytes(frame[4:6], "big")
        if mask & ~0x0F:
            return False
        return frame[7] == _double_checksum(frame[2:7])[0]

    @staticmethod
    def _is_connection_refused(exc: OSError) -> bool:
        # Linux reports ECONNREFUSED=111 for the field controller's TCP RST.
        # Keep the Windows value for offline validation/tests of the same
        # configuration without broadening retries to unrelated network errors.
        return getattr(exc, "errno", None) in (errno.ECONNREFUSED, 10061)

    def _connect(self) -> socket.socket:
        """Connect through the relay's measured short listener gaps.

        The field CX-5104E-L remains ICMP-reachable while port 50000
        intermittently answers SYN with RST for roughly half a second.  Retry
        only that explicit refusal at a one-second cadence and keep the whole
        recovery bounded.  Timeouts, routing failures and all other socket
        errors retain their original fail-closed behavior.
        """

        started = time.monotonic()
        deadline = started + self.policy.connection_refused_deadline_seconds
        refusal_count = 0
        while True:
            try:
                sock = socket.create_connection(
                    (self.target.host, self.target.port),
                    timeout=self.policy.connect_timeout_seconds,
                )
                if refusal_count:
                    self._record_protocol_warning(
                        "relay TCP connection refused %d time(s); recovered "
                        "with %.3gs retry interval"
                        % (
                            refusal_count,
                            self.policy.connection_refused_retry_seconds,
                        )
                    )
                return sock
            except OSError as exc:
                if not self._is_connection_refused(exc):
                    raise
                refusal_count += 1
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise
                time.sleep(
                    min(
                        self.policy.connection_refused_retry_seconds,
                        remaining,
                    )
                )

    def begin_transaction(self) -> None:
        """Reuse one TCP session for one bounded physical recovery transaction."""

        if self._transaction_active:
            raise RelayProtocolError("relay TCP transaction is already active")
        self._transaction_active = True

    def _drop_transaction_socket(self) -> None:
        sock = self._transaction_socket
        self._transaction_socket = None
        if sock is not None:
            try:
                sock.close()
            except OSError:
                pass

    def end_transaction(self) -> None:
        self._drop_transaction_socket()
        self._transaction_active = False

    def _acquire_socket(self) -> Tuple[socket.socket, bool]:
        if not self._transaction_active:
            return self._connect(), True
        if self._transaction_socket is None:
            self._transaction_socket = self._connect()
        return self._transaction_socket, False

    def _exchange_on_socket(
        self, sock: socket.socket, payload: bytes, expect: str
    ) -> bytes:
        deadline = time.monotonic() + self.policy.command_timeout_seconds
        sock.settimeout(self.policy.command_timeout_seconds)
        sock.sendall(payload)
        buffer = bytearray()
        version_seen = False
        resent_after_version = False
        status_tail_deadline: Optional[float] = None
        while time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            if status_tail_deadline is not None:
                remaining = min(
                    remaining,
                    max(0.0, status_tail_deadline - time.monotonic()),
                )
            if remaining <= 0:
                break
            sock.settimeout(remaining)
            try:
                chunk = sock.recv(4096)
            except socket.timeout:
                start = buffer.find(_LEGACY_STATUS_HEADER)
                if start >= 0 and self._verified_single_checksum_status(
                    bytes(buffer[start : start + 8])
                ):
                    return bytes(buffer[start : start + 8])
                break
            if not chunk:
                start = buffer.find(_LEGACY_STATUS_HEADER)
                if start >= 0 and self._verified_single_checksum_status(
                    bytes(buffer[start : start + 8])
                ):
                    # Some relay firmware closes a short query connection
                    # immediately after its verified eight-byte B0 response.
                    # The complete status frame remains authoritative.
                    return bytes(buffer[start : start + 8])
                raise ConnectionResetError(
                    errno.ECONNRESET,
                    "relay closed the persistent TCP connection",
                )
            buffer.extend(chunk)

            # Field-captured CX-5104E-L units can answer the first command on
            # each new TCP connection with the exact ASCII registration banner
            # ``v1.0``. The command must then be repeated once on the same
            # connection. B0 is read-only and A1 explicitly sets a state, so
            # this narrowly scoped repeat is idempotent.
            if buffer.startswith(_LEGACY_VERSION_BANNER):
                if version_seen:
                    raise RelayProtocolError(
                        "relay repeated v1.0 handshake after command resend"
                    )
                del buffer[: len(_LEGACY_VERSION_BANNER)]
                version_seen = True
                self._record_protocol_warning(
                    "accepted relay v1.0 handshake; command resent once"
                )
            if expect == "ack" and b"OK!" in buffer:
                return b"OK!"
            start = buffer.find(_LEGACY_STATUS_HEADER)
            if start >= 0 and len(buffer) >= start + 9:
                return bytes(buffer[start : start + 9])
            if (
                start >= 0
                and len(buffer) == start + 8
                and self._verified_single_checksum_status(
                    bytes(buffer[start : start + 8])
                )
                and status_tail_deadline is None
            ):
                status_tail_deadline = min(
                    deadline,
                    time.monotonic() + _LEGACY_STATUS_TAIL_GRACE_SECONDS,
                )
            if version_seen and not resent_after_version and not buffer:
                sock.sendall(payload)
                resent_after_version = True
            if len(buffer) > 8192:
                raise RelayProtocolError("relay response exceeded 8192 bytes")
        partial = bytes(buffer)
        start = partial.find(_LEGACY_STATUS_HEADER)
        if start >= 0 and self._verified_single_checksum_status(
            partial[start : start + 8]
        ):
            return partial[start : start + 8]
        raise TimeoutError(
            "relay did not return a complete %s response; partial=%s"
            % (expect, partial.hex(" ") if partial else "<empty>")
        )

    def _exchange(self, payload: bytes, expect: str) -> bytes:
        sock: Optional[socket.socket] = None
        close_after = False
        try:
            sock, close_after = self._acquire_socket()
            return self._exchange_on_socket(sock, payload, expect)
        except TimeoutError:
            # An A1 ACK may legitimately be omitted; retain that live session
            # so the authoritative B0 can follow on the same connection. A B0
            # timeout cannot establish a trustworthy stream boundary.
            if expect != "ack":
                self._drop_transaction_socket()
            raise
        except (OSError, RelayProtocolError):
            # A later retry always begins with B0 state discovery before any
            # idempotent A1 set, so replacing a broken transaction socket does
            # not blindly repeat an ambiguous state-changing command.
            self._drop_transaction_socket()
            raise
        finally:
            if close_after and sock is not None:
                try:
                    sock.close()
                except OSError:
                    pass

    def query(self) -> Tuple[Tuple[bool, bool, bool, bool], Optional[str]]:
        frame = self._exchange(self._query_frame(), "status")
        warnings = self._take_protocol_warnings()
        if len(frame) not in (8, 9) or not frame.startswith(_LEGACY_STATUS_HEADER):
            raise RelayProtocolError("invalid CORX B0 status frame")
        if frame[3] != self.target.address or frame[6] != 0x0D:
            raise RelayProtocolError("CORX B0 status address/end marker mismatch")
        expected = _double_checksum(frame[2:7])
        if len(frame) == 8:
            if frame[7] != expected[0]:
                raise RelayProtocolError("CORX B0 status checksum mismatch")
            warnings.append(
                "accepted verified single-checksum B0 response with omitted "
                "second checksum byte"
            )
        elif frame[7:9] != expected:
            if frame[7] == expected[0] and frame[8] == 0xAA:
                # Field-captured CX-5104E-L firmware verifies the B0 payload
                # with the correct first checksum byte, but uses a fixed AA
                # tail instead of the documented doubled checksum byte.  This
                # remains deliberately narrow: the payload checksum, address,
                # end marker and four-channel mask are all still validated.
                warnings.append(
                    "accepted verified first-byte B0 checksum with fixed AA tail"
                )
            elif (
                self.target.allow_omitted_status_checksum
                and frame[7:9] == b"\x00\x00"
            ):
                warnings.append(
                    "accepted explicitly allowed omitted B0 checksum (00 00)"
                )
            else:
                raise RelayProtocolError("CORX B0 status checksum mismatch")
        mask = int.from_bytes(frame[4:6], "big")
        if mask & ~0x0F:
            raise RelayProtocolError("CORX B0 status contains bits outside 4 channels")
        warning = "; ".join(dict.fromkeys(warnings)) if warnings else None
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
                unexpected = [
                    channel
                    for channel in self.target.channels
                    if states[channel - 1] is not state
                ]
                if not unexpected:
                    return warning
                self._send_set(state)
                time.sleep(0.2)
                states, warning = self.query()
                unexpected = [
                    channel
                    for channel in self.target.channels
                    if states[channel - 1] is not state
                ]
                if not unexpected:
                    return warning
                last_error = RelayProtocolError(
                    "relay channel(s) %s remained %s"
                    % (
                        ",".join(str(channel) for channel in unexpected),
                        "ON" if not state else "OFF",
                    )
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
            "cannot confirm channel(s) %s %s after %d attempt(s): %s"
            % (
                self.target.channels_text,
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
        intent_emit: Optional[Callable[[Mapping[str, Any]], None]] = None,
        relay_factory: Callable[[PowerGroup, Policy], Any] = CorxLegacyTcpClient,
    ) -> None:
        self.config = config
        self.store = store
        self.emit = emit
        self.intent_emit = intent_emit
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
        self._intent_acks: Dict[str, Mapping[str, Any]] = {}
        self._pending_intent_tokens: set = set()
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
        for (
            event_id,
            trigger,
            _group_id,
            status,
            detail,
            recovery_reason,
            severity,
            first_seen,
            _updated_at,
        ) in self.store.current_alerts():
            request = PowerCycleRequest(
                event_id=event_id,
                broadcast_code=trigger,
                timestamp=time.time(),
                detected_at=first_seen,
                driver_instance=0,
                handle=255,
                episode_count=1,
                recovery_reason=recovery_reason,
            )
            self._emit(status, request, severity, detail)
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
        request = _request_from_live_recovery_state(payload)
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
        else:
            # A cooldown-deferred event is not a permanent alarm. The live
            # Driver state is authoritative: if the exact trigger no longer
            # requires hard recovery, archive the deferred event immediately
            # instead of leaving a stale POWER-EVENT on the dashboard.
            for (
                event_id,
                recovery_reason,
                _old_detail,
                first_seen,
                _eligible_at,
            ) in self.store.deferred_events_for_trigger(code):
                detail = (
                    "cooldown-deferred fault recovered before another OFF; "
                    "live Driver state=%s"
                    % str(payload.get("display_state") or payload.get("state") or "IDLE")
                )
                if not self.store.resolve_deferred_event(event_id, detail):
                    continue
                resolved = PowerCycleRequest(
                    event_id=event_id,
                    broadcast_code=code,
                    timestamp=time.time(),
                    detected_at=first_seen,
                    driver_instance=int(payload.get("driver_instance", 0)),
                    handle=int(payload.get("handle", 255)),
                    episode_count=1,
                    recovery_reason=recovery_reason,
                )
                self._emit(
                    "STALE_OR_RECOVERED", resolved, "INFO", detail
                )
                self._mark_completed(event_id)

    def accept_intent_ack_payload(self, payload: Mapping[str, Any]) -> None:
        if payload.get("schema_version") != WIRE_SCHEMA_VERSION:
            raise ValueError("unsupported group-power ACK schema")
        if payload.get("type") != INTENT_ACK_TYPE:
            raise ValueError("not a GROUP_POWER_CYCLE_INTENT_ACK message")
        token = _required_text(payload, "token", maximum=128)
        _power_group_id(payload.get("group_id"))
        _integer(payload, "driver_instance", minimum=1)
        accepted = payload.get("accepted")
        if not isinstance(accepted, bool):
            raise ValueError("intent ACK accepted must be a boolean")
        members = payload.get("members")
        if not isinstance(members, list) or len(members) != 4:
            raise ValueError("intent ACK must contain exactly four members")
        if len({_broadcast_code(row) for row in members}) != 4:
            raise ValueError("intent ACK members must be unique")
        detail = payload.get("detail", "")
        if not isinstance(detail, str) or len(detail) > 512:
            raise ValueError("intent ACK detail is invalid")
        with self._condition:
            if token not in self._pending_intent_tokens:
                return
            self._intent_acks[token] = dict(payload)
            self._condition.notify_all()

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
                    if reason == "cooldown_wait":
                        deferred = self.store.deferred_event_info(
                            request.event_id
                        )
                        if deferred is not None:
                            detail, eligible_at = deferred
                            self._emit(
                                "DEFERRED_COOLDOWN",
                                request,
                                "WARN",
                                detail,
                                eligible_at=eligible_at,
                            )
                    if reason in {"terminal", "observed", "mapping_changed"}:
                        self._mark_completed(request.event_id)
                    if reason == "mapping_changed":
                        persisted_alert = next(
                            (
                                row
                                for row in self.store.current_alerts()
                                if row[0] == request.event_id
                            ),
                            None,
                        )
                        if persisted_alert is None:
                            raise StateStoreError(
                                "mapping-change alarm disappeared before emit"
                            )
                        self._emit(
                            "MAPPING_CHANGED",
                            request,
                            "CRITICAL",
                            persisted_alert[4],
                            recovery_reason=persisted_alert[5],
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

    def _prepare_driver_intent(
        self, request: PowerCycleRequest, target: PowerGroup
    ) -> Tuple[str, str, str]:
        """Arm the Driver's planned-outage markers and wait for its ACK.

        The barrier is disabled only for direct unit-test configurations which
        deliberately leave both topics empty. Production JSON parsing always
        supplies fixed absolute topics and therefore fails closed here.
        """

        if not self.config.intent_topic and not self.config.intent_ack_topic:
            return "ACKED", "intent barrier disabled by direct test config", ""
        if self.intent_emit is None:
            return (
                "DRIVER_INTENT_REJECTED",
                "manager has no group-power intent publisher",
                "",
            )
        token = uuid.uuid4().hex
        payload = {
            "schema_version": WIRE_SCHEMA_VERSION,
            "type": INTENT_TYPE,
            "token": token,
            "group_id": target.group_id,
            "driver_instance": request.driver_instance,
            "valid_for_ms": int(
                round(self.config.policy.intent_valid_for_seconds * 1000.0)
            ),
            "members": list(target.members),
        }
        deadline = time.monotonic() + self.config.policy.intent_ack_timeout_seconds
        next_publish = 0.0

        def finish(state: str, detail: str) -> Tuple[str, str, str]:
            with self._condition:
                self._pending_intent_tokens.discard(token)
                self._intent_acks.pop(token, None)
            return state, detail, token

        with self._condition:
            self._intent_acks.pop(token, None)
            self._pending_intent_tokens.add(token)
        while not self.stop_event.is_set():
            now = time.monotonic()
            if now >= deadline:
                break
            if now >= next_publish:
                try:
                    self.intent_emit(payload)
                except Exception as exc:
                    return finish(
                        "DRIVER_INTENT_REJECTED",
                        "cannot publish Driver intent: %s" % exc,
                    )
                next_publish = now + 0.25
            with self._condition:
                ack = self._intent_acks.pop(token, None)
                if ack is None:
                    self._condition.wait(
                        timeout=min(0.25, max(0.0, deadline - time.monotonic()))
                    )
                    ack = self._intent_acks.pop(token, None)
            if ack is None:
                continue
            ack_members = tuple(ack.get("members", ()))
            if (
                ack.get("driver_instance") != request.driver_instance
                or ack.get("group_id") != target.group_id
                or ack_members != target.members
            ):
                return finish(
                    "DRIVER_INTENT_REJECTED",
                    "Driver ACK identity/member mapping does not match request",
                )
            detail = str(ack.get("detail") or "")
            if ack.get("accepted") is not True:
                return finish(
                    "DRIVER_INTENT_REJECTED",
                    detail or "Driver rejected planned shared-power outage",
                )
            return finish("ACKED", detail or "Driver armed all group members")
        return finish(
            "DRIVER_INTENT_ACK_TIMEOUT",
            "Driver did not ACK planned shared-power outage within %.1fs"
            % self.config.policy.intent_ack_timeout_seconds,
        )

    def _cancel_driver_intent(
        self, request: PowerCycleRequest, target: PowerGroup, token: str
    ) -> None:
        if not token or self.intent_emit is None:
            return
        payload = {
            "schema_version": WIRE_SCHEMA_VERSION,
            "type": INTENT_CANCEL_TYPE,
            "token": token,
            "group_id": target.group_id,
            "driver_instance": request.driver_instance,
            "valid_for_ms": int(
                round(self.config.policy.intent_valid_for_seconds * 1000.0)
            ),
            "members": list(target.members),
        }
        try:
            self.intent_emit(payload)
        except Exception as exc:
            self._emit(
                "INTENT_CANCEL_PUBLISH_FAILED",
                request,
                "WARN",
                "unused Driver intent will expire automatically: %s" % exc,
            )

    def _start_driver_intent_keepalive(
        self, request: PowerCycleRequest, target: PowerGroup, token: str
    ) -> Tuple[threading.Event, Optional[threading.Thread]]:
        stop = threading.Event()
        if not token or self.intent_emit is None:
            return stop, None
        payload = {
            "schema_version": WIRE_SCHEMA_VERSION,
            "type": INTENT_TYPE,
            "token": token,
            "group_id": target.group_id,
            "driver_instance": request.driver_instance,
            "valid_for_ms": int(
                round(self.config.policy.intent_valid_for_seconds * 1000.0)
            ),
            "members": list(target.members),
        }

        def refresh() -> None:
            while not stop.is_set() and not self.stop_event.is_set():
                try:
                    self.intent_emit(payload)
                except Exception:
                    # The initial ACK already established the barrier. A
                    # transient refresh failure is retried until the relay
                    # operation finishes; the Driver marker also has its TTL.
                    pass
                stop.wait(1.0)

        thread = threading.Thread(
            target=refresh,
            name="livox-group-power-intent-keepalive",
            daemon=True,
        )
        thread.start()
        return stop, thread

    @staticmethod
    def _stop_driver_intent_keepalive(
        stop: threading.Event, thread: Optional[threading.Thread]
    ) -> None:
        stop.set()
        if thread is not None:
            thread.join(timeout=2.0)

    def _cancel_reserved_before_off(
        self,
        request: PowerCycleRequest,
        target: PowerGroup,
        cycle_id: int,
        baseline_states: Sequence[bool],
        state: str,
        severity: str,
        reason: str,
    ) -> None:
        """Confirm ON and atomically release an unused OFF reservation."""

        try:
            restore = self.relay_factory(target, self.config.policy)
            warning = restore.ensure_state(
                True,
                retries=self.config.policy.restore_retries,
                deadline_seconds=self.config.policy.restore_deadline_seconds,
            )
            if warning:
                self._emit("RELAY_PROTOCOL_WARNING", request, "WARN", warning)
            restored_states, restored_warning = restore.query()
            if restored_warning:
                self._emit(
                    "RELAY_PROTOCOL_WARNING", request, "WARN", restored_warning
                )
            unexpected = _unexpected_selected_channels(
                target, restored_states, True
            )
            if unexpected:
                raise RelayProtocolError(
                    "latest B0 query does not confirm channel(s) %s ON"
                    % ",".join(str(channel) for channel in unexpected)
                )
            changed = _changed_non_target_channels(
                target, baseline_states, restored_states
            )
            if changed:
                detail = (
                    "%s; non-target relay channel(s) changed: %s; selected "
                    "channels remain ON"
                    % (reason, ",".join(str(item) for item in changed))
                )
                state = "NON_TARGET_STATE_CHANGED"
                severity = "CRITICAL"
            else:
                detail = (
                    "%s; no OFF was sent and selected channel(s) %s are "
                    "confirmed ON" % (reason, target.channels_text)
                )
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
                "%s; ON archival also failed: %s; persistent obligation retained"
                % (reason, exc)
            )
            self.store.finish_cycle(cycle_id, "POWER_ON_UNCONFIRMED", detail)
            self._terminal(
                request, "POWER_ON_UNCONFIRMED", "CRITICAL", detail
            )

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
        unexpected = _unexpected_selected_channels(target, states, True)
        if unexpected:
            self._terminal(
                request,
                "MAPPING_MISMATCH",
                "CRITICAL",
                "selected channel(s) %s are already OFF while the triggering "
                "recovery "
                "condition is still live; refusing to energize an unverified "
                "power group"
                % ",".join(str(channel) for channel in unexpected),
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

        intent_state = "DRIVER_INTENT_ACK_TIMEOUT"
        intent_detail = "Driver intent ACK was not attempted"
        intent_token = ""
        for intent_attempt in range(1, policy.intent_ack_attempts + 1):
            intent_state, intent_detail, intent_token = (
                self._prepare_driver_intent(request, target)
            )
            if intent_state == "ACKED":
                break
            self._cancel_driver_intent(request, target, intent_token)
            if (
                intent_state != "DRIVER_INTENT_ACK_TIMEOUT"
                or intent_attempt >= policy.intent_ack_attempts
            ):
                self._terminal(
                    request,
                    intent_state,
                    "CRITICAL",
                    "%s (attempt %d/%d)"
                    % (intent_detail, intent_attempt, policy.intent_ack_attempts),
                )
                return
            self._emit(
                "DRIVER_INTENT_ACK_RETRY",
                request,
                "WARN",
                "%s; retrying in %.1fs (attempt %d/%d)"
                % (
                    intent_detail,
                    policy.intent_retry_seconds,
                    intent_attempt,
                    policy.intent_ack_attempts,
                ),
            )
            if self.stop_event.wait(policy.intent_retry_seconds):
                self._terminal(
                    request,
                    "DRIVER_INTENT_ACK_TIMEOUT",
                    "ERROR",
                    "manager shutdown interrupted Driver intent ACK retry",
                )
                return
            retry_trigger, retry_detail = self._wait_trigger_required(
                request, 0.0
            )
            if retry_trigger is not True:
                state = (
                    "STALE_OR_RECOVERED"
                    if retry_trigger is False
                    else "PRECHECK_FAILED"
                )
                severity = "INFO" if retry_trigger is False else "ERROR"
                self._terminal(
                    request,
                    state,
                    severity,
                    "intent retry trigger check: %s" % retry_detail,
                )
                return

        self._emit("DRIVER_INTENT_ACKED", request, "INFO", intent_detail)
        intent_keepalive_stop, intent_keepalive_thread = (
            self._start_driver_intent_keepalive(
                request, target, intent_token
            )
        )

        # ACK waiting creates a new race window. Recheck the exact cause and
        # episode before consuming persistent safety budget.
        post_ack_trigger, post_ack_detail = self._wait_trigger_required(
            request, 0.0
        )
        if post_ack_trigger is not True:
            self._stop_driver_intent_keepalive(
                intent_keepalive_stop, intent_keepalive_thread
            )
            self._cancel_driver_intent(request, target, intent_token)
            state = (
                "STALE_OR_RECOVERED"
                if post_ack_trigger is False
                else "PRECHECK_FAILED"
            )
            severity = "INFO" if post_ack_trigger is False else "ERROR"
            self._terminal(
                request,
                state,
                severity,
                "post-ACK trigger check: %s" % post_ack_detail,
            )
            return

        try:
            cycle_id, reason, remaining = self.store.reserve_cycle(
                request, target, policy
            )
        except Exception:
            self._stop_driver_intent_keepalive(
                intent_keepalive_stop, intent_keepalive_thread
            )
            self._cancel_driver_intent(request, target, intent_token)
            raise
        if cycle_id is None:
            self._stop_driver_intent_keepalive(
                intent_keepalive_stop, intent_keepalive_thread
            )
            self._cancel_driver_intent(request, target, intent_token)
            if reason == "cooldown":
                detail = (
                    "physical power endpoint cooldown has %.0fs remaining; "
                    "live fault will be revalidated at eligibility"
                    % remaining
                )
                eligible_at = self.store.defer_cooldown(
                    request.event_id, remaining, detail
                )
                self._emit(
                    "DEFERRED_COOLDOWN",
                    request,
                    "WARN",
                    detail,
                    eligible_at=eligible_at,
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
            return _changed_non_target_channels(target, states, current_states)

        # Commit the must-be-ON obligation before the first OFF command.  A
        # crash at any later instruction is repaired on process restart.
        try:
            self.store.set_obligation(request, target)
        except Exception as exc:
            self._stop_driver_intent_keepalive(
                intent_keepalive_stop, intent_keepalive_thread
            )
            self._cancel_driver_intent(request, target, intent_token)
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
            self._stop_driver_intent_keepalive(
                intent_keepalive_stop, intent_keepalive_thread
            )
            self._cancel_driver_intent(request, target, intent_token)
            state = (
                "STALE_OR_RECOVERED"
                if final_trigger is False
                else "PRECHECK_FAILED"
            )
            severity = "INFO" if final_trigger is False else "ERROR"
            self._cancel_reserved_before_off(
                request,
                target,
                cycle_id,
                states,
                state,
                severity,
                "%s after durable reservation" % final_trigger_detail,
            )
            return
        off_confirmed = False
        off_phase_ok = False
        on_archived = False
        phase_error = ""
        non_target_error = ""
        transaction_started = False

        try:
            begin_transaction = getattr(relay, "begin_transaction", None)
            if callable(begin_transaction):
                begin_transaction()
                transaction_started = True
            self._emit(
                "POWER_OFF_COMMAND",
                request,
                "WARN",
                "%s %s:%d channel(s) %s"
                % (target.label, target.host, target.port, target.channels_text)
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
            unexpected = _unexpected_selected_channels(target, off_states, False)
            if unexpected:
                raise RelayProtocolError(
                    "latest B0 query does not confirm channel(s) %s OFF"
                    % ",".join(str(channel) for channel in unexpected)
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
                "channel(s) %s OFF confirmed; holding %.1fs"
                % (target.channels_text, policy.off_seconds),
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
                # The field CX-5104E-L has been verified with B0/A1/B0 across
                # one connection and a 5-second OFF hold. Reuse that bounded
                # transaction session for ON. If the peer closed it, the
                # client discards the broken socket; ensure_state begins its
                # retry with a fresh B0 before sending another idempotent A1.
                restore = relay
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
                unexpected = _unexpected_selected_channels(target, on_states, True)
                if unexpected:
                    raise RelayProtocolError(
                        "latest B0 query does not confirm channel(s) %s ON"
                        % ",".join(str(channel) for channel in unexpected)
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
                    "shared channel(s) %s ON confirmed; waiting for all %d "
                    "members" % (target.channels_text, len(target.members)),
                )
            except Exception as exc:
                self._emit(
                    "POWER_ON_UNCONFIRMED",
                    request,
                    "CRITICAL",
                    "%s; persistent ON obligation retained" % exc,
                )
            finally:
                if transaction_started:
                    end_transaction = getattr(relay, "end_transaction", None)
                    if callable(end_transaction):
                        try:
                            end_transaction()
                        except Exception as exc:
                            self._emit(
                                "RELAY_SESSION_CLOSE_FAILED",
                                request,
                                "WARN",
                                str(exc),
                            )
                self._stop_driver_intent_keepalive(
                    intent_keepalive_stop, intent_keepalive_thread
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
            detail = (
                non_target_error
                + "; selected channel(s) %s are confirmed ON"
                % target.channels_text
            )
            self.store.finish_cycle(
                cycle_id, "NON_TARGET_STATE_CHANGED", detail
            )
            self._terminal(
                request, "NON_TARGET_STATE_CHANGED", "CRITICAL", detail
            )
            return
        if not off_confirmed or not off_phase_ok:
            detail = (
                "%s; selected channel(s) %s are confirmed ON"
                % (
                    phase_error or "OFF phase did not complete",
                    target.channels_text,
                )
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
                            current = _request_from_live_recovery_state(trigger)
                            exact_event = (
                                current is not None
                                and current.identity == request.identity
                            )
                        except (TypeError, ValueError):
                            exact_event = False
                        if not exact_event:
                            return (
                                False,
                                "current trigger state no longer matches this "
                                "POWER_CYCLE_REQUIRED event identity and cause",
                            )
                        return (
                            True,
                            "triggering LiDAR still matches the exact live "
                            "POWER_CYCLE_REQUIRED %s event"
                            % request.recovery_reason,
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
                            and payload.get(
                                "recovery_state", RECOVERY_STATE_IDLE
                            )
                            == RECOVERY_STATE_IDLE
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
        for (
            target,
            event_id,
            trigger_bcode,
            recovery_reason,
            last_attempt,
        ) in self.store.obligations():
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
                recovery_reason=recovery_reason,
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
        self.store.finish_event(
            request.event_id, state, detail, severity=severity
        )
        self._mark_completed(request.event_id)
        self._emit(state, request, severity, detail)

    def _emit(
        self,
        state: str,
        request: PowerCycleRequest,
        severity: str,
        detail: str,
        target: Optional[PowerGroup] = None,
        recovery_reason: Optional[str] = None,
        eligible_at: float = 0.0,
        actionable: Optional[bool] = None,
    ) -> None:
        if target is None:
            target = self.config.group_for(request.broadcast_code)
        if actionable is None:
            actionable = (
                state in ACTIVE_ALARM_STATES
                or state in ACTIVE_WORKFLOW_STATES
            )
        payload = {
            "schema_version": WIRE_SCHEMA_VERSION,
            "type": STATUS_TYPE,
            "timestamp": time.time(),
            "event_timestamp": request.detected_at,
            "eligible_at": eligible_at,
            "actionable": actionable,
            "state": state,
            "severity": severity,
            "event_id": request.event_id,
            "broadcast_code": request.broadcast_code,
            "recovery_reason": recovery_reason or request.recovery_reason,
            "power_group": target.group_id if target else "",
            "members": list(target.members) if target else [],
            "relay_channels": list(target.channels) if target else [],
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
                "OK group=%s (%s) members=%d %s:%d channels=%s states=%s%s"
                % (
                    group_id,
                    target.label,
                    len(target.members),
                    target.host,
                    target.port,
                    target.channels_text,
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
    # Serialize the whole TCP controller, not merely one configured channel
    # set. A1 is a read/modify-mask protocol, so two independent managers for
    # overlapping or disjoint sets on the same relay must never race.
    endpoint_keys = {target.relay_endpoint_key for target in targets}
    streams: List[Any] = []
    try:
        for endpoint_key in sorted(endpoint_keys):
            digest = hashlib.sha256(endpoint_key.encode("utf-8")).hexdigest()
            path = os.path.join(lock_root, "endpoint-%s.lock" % digest)
            streams.append(_acquire_file_lock(path, endpoint_key))
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
            [target for target, _event, _trigger, _reason, _last in obligations],
            state_db,
        )
        for target, event_id, trigger, _reason, _last_attempt in obligations:
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
    intent_publisher = rospy.Publisher(
        config.intent_topic, String, queue_size=8, latch=False
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

    def emit_intent(payload: Mapping[str, Any]) -> None:
        text = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        intent_publisher.publish(String(data=text))

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
            target
            for target, _event, _trigger, _reason, _last in store.obligations()
        ]
        endpoint_locks = _acquire_endpoint_locks(
            configured_targets + obligation_targets, config.state_db
        )
        core = PowerCycleManagerCore(
            config, store, emit, intent_emit=emit_intent
        )
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

    def intent_ack_cb(message: Any) -> None:
        try:
            payload = json.loads(message.data)
            if not isinstance(payload, Mapping):
                raise ValueError("intent ACK JSON root is not an object")
            core.accept_intent_ack_payload(payload)
        except (ValueError, ConfigurationError, json.JSONDecodeError) as exc:
            rospy.logwarn_throttle(30, "Invalid group-power ACK ignored: %s", exc)

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
            "relay_channels": [],
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
        rospy.Subscriber(
            config.intent_ack_topic, String, intent_ack_cb, queue_size=16
        )
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
            "relay_channels": [],
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
            "Livox power-cycle manager started: mode=%s request=%s state=%s "
            "intent=%s ack=%s",
            config.mode,
            config.request_topic,
            config.state_topic,
            config.intent_topic,
            config.intent_ack_topic,
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


def parse_arguments(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    """Parse manager options while accepting roslaunch remapping arguments.

    roslaunch appends inherited topic remaps plus ``__name``/``__log`` to a
    Python node's command line.  Keep ordinary CLI parsing strict, but discard
    only unknown ROS remap tokens.  ``parse_known_args`` is used deliberately
    so a recognized option value containing ``:=`` is still consumed normally.
    Importing ``rospy.myargv`` here would break the offline validation and
    emergency-ON paths on hosts where the ROS environment is not sourced.
    """
    parser = build_argument_parser()
    args, unknown = parser.parse_known_args(argv)
    unrecognized = [token for token in unknown if ":=" not in token]
    if unrecognized:
        parser.error("unrecognized arguments: %s" % " ".join(unrecognized))
    return args


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_arguments(argv)
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
        if config.mode != args.mode:
            print(
                "NOTICE: command-line --mode=%s overrides legacy JSON mode=%s; "
                "the launch relay_power_cycle_enable switch is the hardware "
                "authorization source" % (args.mode, config.mode),
                file=sys.stderr,
            )
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
            "members=%d off_seconds=%g state_db=%s"
            % (
                config.mode,
                len(config.power_groups),
                enabled_groups,
                total_members,
                config.policy.off_seconds,
                config.state_db,
            )
        )
        return 0
    if args.check_relays:
        return check_relays(config)
    return run_ros(config)


if __name__ == "__main__":
    raise SystemExit(main())
