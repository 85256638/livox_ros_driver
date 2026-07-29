from __future__ import annotations

import importlib.util
import json
import os
import sqlite3
import socketserver
import sys
import tempfile
import threading
import time
import unittest
import xml.etree.ElementTree as ET
from dataclasses import replace
from pathlib import Path
from unittest import mock


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = (
    ROOT
    / "livox_ros_driver"
    / "livox_ros_driver"
    / "scripts"
    / "livox_power_cycle_manager.py"
)
SPEC = importlib.util.spec_from_file_location("livox_power_cycle_manager", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
manager = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = manager
SPEC.loader.exec_module(manager)

SITE_VALIDATOR = (
    ROOT
    / "livox_ros_driver"
    / "livox_ros_driver"
    / "scripts"
    / "validate_livox_power_cycle_site.py"
)
VALIDATOR_SPEC = importlib.util.spec_from_file_location(
    "validate_livox_power_cycle_site", SITE_VALIDATOR
)
assert VALIDATOR_SPEC is not None and VALIDATOR_SPEC.loader is not None
site_validator = importlib.util.module_from_spec(VALIDATOR_SPEC)
VALIDATOR_SPEC.loader.exec_module(site_validator)


MEMBERS = (
    "TESTLIDAR000001",
    "TESTLIDAR000002",
    "TESTLIDAR000003",
    "TESTLIDAR000004",
)
BCODE = MEMBERS[0]
GROUP_ID = "pit-4-shared"


class _RelayState:
    def __init__(
        self,
        mask=0x0F,
        status_checksum=None,
        version_handshake=False,
        repeat_version_handshake=False,
        fragment_status_tail=False,
    ):
        self.mask = mask
        self.status_checksum = status_checksum
        self.version_handshake = version_handshake
        self.repeat_version_handshake = repeat_version_handshake
        self.fragment_status_tail = fragment_status_tail
        self.lock = threading.Lock()


class _RelayHandler(socketserver.BaseRequestHandler):
    def handle(self):
        data = self.request.recv(64)
        if len(data) < 3 or data[:2] != b"\xCC\xDD":
            return
        state = self.server.relay_state
        if state.version_handshake:
            self.request.sendall(b"v1.0")
            data = self.request.recv(64)
            if len(data) < 3 or data[:2] != b"\xCC\xDD":
                return
            if state.repeat_version_handshake:
                self.request.sendall(b"v1.0")
                return
        if data[2] == 0xB0:
            with state.lock:
                mask = state.mask
                status_checksum = state.status_checksum
            body = bytes((0xB0, 1)) + mask.to_bytes(2, "big") + b"\x0D"
            checksum = manager._double_checksum(body)
            if callable(status_checksum):
                checksum = status_checksum(body)
            elif status_checksum is not None:
                checksum = status_checksum
            response = b"\xAA\xBB" + body + checksum
            if state.fragment_status_tail:
                self.request.sendall(response[:-1])
                time.sleep(0.01)
                self.request.sendall(response[-1:])
            else:
                self.request.sendall(response)
        elif data[2] == 0xA1 and len(data) >= 10:
            control = int.from_bytes(data[4:6], "big")
            enabled = int.from_bytes(data[6:8], "big")
            with state.lock:
                state.mask = (state.mask & ~enabled) | (control & enabled)
            self.request.sendall(b"OK!")


class _RelayServer(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True

    def __init__(self, relay_state):
        super().__init__(("127.0.0.1", 0), _RelayHandler)
        self.relay_state = relay_state


class _RunningRelay:
    def __init__(
        self,
        mask=0x0F,
        status_checksum=None,
        version_handshake=False,
        repeat_version_handshake=False,
        fragment_status_tail=False,
    ):
        self.state = _RelayState(
            mask,
            status_checksum,
            version_handshake,
            repeat_version_handshake,
            fragment_status_tail,
        )
        self.server = _RelayServer(self.state)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)

    def __enter__(self):
        self.thread.start()
        return self

    def __exit__(self, *_args):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=2)

    @property
    def port(self):
        return self.server.server_address[1]


def _policy(**changes):
    values = dict(manager.Policy().__dict__)
    values.update(changes)
    return manager.Policy(**values)


def _group(
    port=50000,
    enabled=True,
    channel=1,
    channels=None,
    allow_omitted=False,
    group_id=GROUP_ID,
    members=MEMBERS,
):
    return manager.PowerGroup(
        group_id=group_id,
        label="four-lidar-shared-power",
        enabled=enabled,
        members=tuple(members),
        host="127.0.0.1",
        port=port,
        channels=tuple(channels) if channels is not None else (channel,),
        address=1,
        allow_omitted_status_checksum=allow_omitted,
    )


def _config(db_path, group, mode="armed", policy=None):
    return manager.ManagerConfig(
        mode=mode,
        state_db=str(db_path),
        request_topic="/livox/power_cycle_request",
        state_topic="/livox/lidar_recovery_state",
        status_topic="/livox/power_cycle_status",
        heartbeat_topic="/livox/power_cycle_heartbeat",
        intent_topic="",
        intent_ack_topic="",
        policy=policy or manager.Policy(),
        power_groups={group.group_id: group},
        member_to_group={code: group.group_id for code in group.members},
    )


def _required_state(broadcast_code=BCODE, driver_instance=123, episode_count=1):
    now = int(time.time())
    return {
        "schema_version": 1,
        "type": "LIDAR_RECOVERY_STATE",
        "timestamp": now,
        "driver_instance": driver_instance,
        "handle": 2,
        "broadcast_code": broadcast_code,
        "connected": False,
        "connect_state": "Off",
        "lidar_state": "?",
        "handshake_state": "POWER_CYCLE_REQUIRED",
        "recovery_state": "POWER_CYCLE_REQUIRED",
        "recovery_reason": "HANDSHAKE_STUCK",
        "wake_state": "IDLE",
        "wake_request_id": 0,
        "wake_connection_generation": 0,
        "wake_dropout_generation": 0,
        "wake_started_at": 0,
        "wake_dropout_at": 0,
        "wake_silence_at": 0,
        "normal_state": "IDLE",
        "normal_connection_generation": 0,
        "normal_dropout_generation": 0,
        "normal_healthy_since_at": 0,
        "normal_dropout_at": 0,
        "normal_silence_at": 0,
        "startup_state": "IDLE",
        "startup_missing_since": 0,
        "broadcast_fresh": True,
        "publishing": False,
        "published_packets": 0,
        "power_cycle_required_count": episode_count,
        "power_cycle_required_at": now,
    }


def _wake_required_state(
    broadcast_code=BCODE,
    driver_instance=123,
    episode_count=1,
    wake_request_id=77,
):
    now = int(time.time())
    payload = _required_state(
        broadcast_code, driver_instance, episode_count
    )
    payload.update(
        {
            "timestamp": now,
            "handshake_state": "IDLE",
            "recovery_state": "POWER_CYCLE_REQUIRED",
            "recovery_reason": "WAKE_DROPOUT",
            "wake_state": "POWER_CYCLE_REQUIRED",
            "wake_request_id": wake_request_id,
            "wake_connection_generation": 9,
            "wake_dropout_generation": 9,
            "wake_started_at": now - 20,
            "wake_dropout_at": now - 10,
            "wake_silence_at": now - 10,
            "broadcast_fresh": False,
            "power_cycle_required_at": now,
        }
    )
    return payload


def _healthy_state(broadcast_code=BCODE):
    payload = _required_state(broadcast_code)
    payload.update(
        {
            "timestamp": int(time.time()),
            "connected": True,
            "connect_state": "Sampling",
            "lidar_state": "Normal",
            "handshake_state": "IDLE",
            "recovery_state": "IDLE",
            "recovery_reason": "NONE",
            "wake_state": "IDLE",
            "wake_request_id": 0,
            "wake_connection_generation": 0,
            "wake_dropout_generation": 0,
            "wake_started_at": 0,
            "wake_dropout_at": 0,
            "wake_silence_at": 0,
            "broadcast_fresh": False,
            "publishing": True,
            "published_packets": 100,
        }
    )
    return payload


def _normal_required_state(
    broadcast_code=BCODE, driver_instance=123, episode_count=1
):
    now = int(time.time())
    payload = _required_state(
        broadcast_code, driver_instance, episode_count
    )
    payload.update(
        {
            "timestamp": now,
            "handshake_state": "IDLE",
            "recovery_reason": "NORMAL_DROPOUT",
            "wake_state": "IDLE",
            "normal_state": "POWER_CYCLE_REQUIRED",
            "normal_connection_generation": 12,
            "normal_dropout_generation": 12,
            "normal_healthy_since_at": now - 40,
            "normal_dropout_at": now - 10,
            "normal_silence_at": now - 5,
            "broadcast_fresh": False,
            "power_cycle_required_at": now,
        }
    )
    return payload


def _startup_required_state(
    broadcast_code=BCODE, driver_instance=123, episode_count=1
):
    now = int(time.time())
    payload = _required_state(
        broadcast_code, driver_instance, episode_count
    )
    payload.update(
        {
            "timestamp": now,
            "handle": 255,
            "handshake_state": "IDLE",
            "recovery_reason": "STARTUP_MISSING",
            "wake_state": "IDLE",
            "startup_state": "POWER_CYCLE_REQUIRED",
            "startup_missing_since": now - 30,
            "broadcast_fresh": False,
            "power_cycle_required_at": now,
        }
    )
    return payload


def _request_payload_from_state(state):
    request = dict(state)
    request.update(
        {
            "type": "POWER_CYCLE_REQUIRED",
            "event_id": "%s:%d:%d:%d"
            % (
                state["broadcast_code"],
                state["driver_instance"],
                int(state["power_cycle_required_at"]),
                state["power_cycle_required_count"],
            ),
            "detected_at": state["power_cycle_required_at"],
            "episode_count": state["power_cycle_required_count"],
            "session_reset_attempts": (
                1
                if state.get("recovery_reason", "HANDSHAKE_STUCK")
                == "HANDSHAKE_STUCK"
                else 0
            ),
        }
    )
    return request


def _trigger_group(core, required_states):
    if isinstance(required_states, dict):
        required_states = [required_states]
    for row in required_states:
        core.accept_state_payload(row)

    # The production manager deliberately requires a state frame received
    # after its relay B0 precheck.  Feed a few distinct 1 Hz-equivalent frames
    # so core integration tests exercise that gate instead of reusing one
    # cached payload for every pre-OFF check.
    def refresh():
        for _ in range(3):
            time.sleep(0.05)
            for row in required_states:
                updated = dict(row)
                updated["timestamp"] = int(time.time())
                core.accept_state_payload(updated)

    feeder = threading.Thread(target=refresh, daemon=True)
    feeder.start()
    return feeder


def _wait_until(predicate, timeout=3.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.005)
    return bool(predicate())


def _publish_health(core, members=MEMBERS, repeats=5, interval=0.02):
    for _ in range(repeats):
        for code in members:
            core.accept_state_payload(_healthy_state(code))
        time.sleep(interval)


class ConfigTests(unittest.TestCase):
    def test_omitted_off_seconds_preserves_legacy_ten_second_default(self):
        self.assertEqual(manager.Policy().off_seconds, 10.0)
        self.assertEqual(manager._policy_from_json({}).off_seconds, 10.0)

    def test_explicit_fast_off_seconds_is_preserved(self):
        self.assertEqual(
            manager._policy_from_json({"off_seconds": 5}).off_seconds,
            5.0,
        )
        self.assertEqual(
            manager._policy_from_json({"off_seconds": 10}).off_seconds,
            10.0,
        )

    def test_off_seconds_rejects_values_below_five(self):
        for value in (4, 4.9):
            with self.subTest(value=value), self.assertRaises(
                manager.ConfigurationError
            ):
                manager._policy_from_json({"off_seconds": value})
        for value in (5, 10):
            with self.subTest(value=value):
                self.assertEqual(
                    manager._policy_from_json(
                        {"off_seconds": value}
                    ).off_seconds,
                    float(value),
                )

    def test_shipped_example_is_safe_and_valid(self):
        config = manager.load_config(
            str(
                ROOT
                / "livox_ros_driver"
                / "config"
                / "livox_power_cycle.example.json"
            )
        )
        self.assertEqual(config.mode, "observe")
        self.assertEqual(config.policy.off_seconds, 5.0)
        self.assertGreaterEqual(len(config.power_groups), 1)
        self.assertFalse(
            any(item.enabled for item in config.power_groups.values())
        )
        configured_members = [
            code
            for group in config.power_groups.values()
            for code in group.members
        ]
        self.assertEqual(len(configured_members), len(set(configured_members)))
        for code in configured_members:
            self.assertIsNotNone(config.group_for(code))
        self.assertEqual(
            next(iter(config.power_groups.values())).channels,
            (1, 2, 3, 4),
        )

    def test_legacy_channel_and_multi_channels_are_both_supported(self):
        with tempfile.TemporaryDirectory() as tmp:
            for relay, expected in (
                ({"channel": 3}, (3,)),
                ({"channels": [4, 2, 1, 3]}, (1, 2, 3, 4)),
            ):
                with self.subTest(relay=relay):
                    path = Path(tmp) / ("config-%d.json" % len(expected))
                    path.write_text(
                        json.dumps(
                            {
                                "schema_version": 2,
                                "mode": "observe",
                                "power_groups": {
                                    "group": {
                                        "members": list(MEMBERS),
                                        "relay": dict(
                                            relay,
                                            protocol="legacy_tcp",
                                            host="192.0.2.55",
                                            port=50000,
                                        ),
                                    }
                                },
                            }
                        ),
                        encoding="utf-8",
                    )
                    target = manager.load_config(str(path)).power_groups["group"]
                    self.assertEqual(target.channels, expected)

    def test_channel_selection_rejects_ambiguous_or_unsafe_values(self):
        invalid_relays = (
            {},
            {"channel": 1, "channels": [1, 2]},
            {"channels": []},
            {"channels": [1, 1]},
            {"channels": [0, 1]},
            {"channels": [1, 5]},
            {"channels": [True]},
            {"channels": "1,2,3,4"},
        )
        with tempfile.TemporaryDirectory() as tmp:
            for index, relay in enumerate(invalid_relays):
                with self.subTest(relay=relay):
                    path = Path(tmp) / ("invalid-%d.json" % index)
                    path.write_text(
                        json.dumps(
                            {
                                "schema_version": 2,
                                "mode": "observe",
                                "power_groups": {
                                    "group": {
                                        "members": list(MEMBERS),
                                        "relay": dict(
                                            relay,
                                            protocol="legacy_tcp",
                                            host="192.0.2.55",
                                            port=50000,
                                        ),
                                    }
                                },
                            }
                        ),
                        encoding="utf-8",
                    )
                    with self.assertRaises(manager.ConfigurationError):
                        manager.load_config(str(path))

    def test_partially_overlapping_enabled_channel_sets_are_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "config.json"
            path.write_text(
                json.dumps(
                    {
                        "schema_version": 2,
                        "mode": "armed",
                        "power_groups": {
                            "group-one": {
                                "enabled": True,
                                "members": list(MEMBERS),
                                "relay": {
                                    "protocol": "legacy_tcp",
                                    "host": "192.0.2.55",
                                    "channels": [1, 2],
                                },
                            },
                            "group-two": {
                                "enabled": True,
                                "members": [
                                    "OTHERLIDAR00001",
                                    "OTHERLIDAR00002",
                                    "OTHERLIDAR00003",
                                    "OTHERLIDAR00004",
                                ],
                                "relay": {
                                    "protocol": "legacy_tcp",
                                    "host": "192.0.2.55",
                                    "channels": [2, 3],
                                },
                            },
                        },
                    }
                ),
                encoding="utf-8",
            )
            with self.assertRaisesRegex(manager.ConfigurationError, "share relay"):
                manager.load_config(str(path))

    def test_duplicate_enabled_channel_is_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "config.json"
            row = {
                "enabled": True,
                "members": ["EXAMPLE00000001"],
                "relay": {
                    "protocol": "legacy_tcp",
                    "host": "192.0.2.55",
                    "port": 50000,
                    "channel": 1,
                },
            }
            path.write_text(
                json.dumps(
                    {
                        "schema_version": 2,
                        "mode": "armed",
                        "power_groups": {
                            "group-one": row,
                            "group-two": dict(
                                row, members=["EXAMPLE00000002"]
                            ),
                        },
                    }
                ),
                encoding="utf-8",
            )
            with self.assertRaises(manager.ConfigurationError):
                manager.load_config(str(path))

    def test_member_cannot_belong_to_two_power_groups(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "config.json"
            relay = {
                "protocol": "legacy_tcp",
                "host": "192.0.2.55",
                "port": 50000,
                "channel": 1,
            }
            path.write_text(
                json.dumps(
                    {
                        "schema_version": 2,
                        "mode": "observe",
                        "power_groups": {
                            "group-one": {
                                "members": ["EXAMPLE00000001"],
                                "relay": relay,
                            },
                            "group-two": {
                                "members": ["EXAMPLE00000001"],
                                "relay": dict(relay, channel=2),
                            },
                        },
                    }
                ),
                encoding="utf-8",
            )
            with self.assertRaises(manager.ConfigurationError):
                manager.load_config(str(path))

    def test_power_group_requires_exactly_four_members(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "config.json"
            path.write_text(
                json.dumps(
                    {
                        "schema_version": 2,
                        "mode": "observe",
                        "power_groups": {
                            "unsafe-incomplete-group": {
                                "members": list(MEMBERS[:3]),
                                "relay": {
                                    "protocol": "legacy_tcp",
                                    "host": "192.0.2.55",
                                    "port": 50000,
                                    "channel": 1,
                                },
                            }
                        },
                    }
                ),
                encoding="utf-8",
            )
            with self.assertRaisesRegex(
                manager.ConfigurationError, "exactly 4"
            ):
                manager.load_config(str(path))

    def test_unknown_policy_key_is_rejected_instead_of_silently_defaulting(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "config.json"
            path.write_text(
                json.dumps(
                    {
                        "schema_version": 2,
                        "mode": "observe",
                        "policy": {"off_second": 10},
                        "power_groups": {},
                    }
                ),
                encoding="utf-8",
            )
            with self.assertRaises(manager.ConfigurationError):
                manager.load_config(str(path))

    def test_request_event_id_must_match_identity_fields(self):
        payload = _required_state()
        payload.update(
            {
                "type": "POWER_CYCLE_REQUIRED",
                "event_id": "forged-event-id",
                "detected_at": payload["power_cycle_required_at"],
                "episode_count": payload["power_cycle_required_count"],
            }
        )
        with self.assertRaises(ValueError):
            manager.PowerCycleRequest.from_payload(payload)

    def test_legacy_schema1_handshake_infers_recovery_reason(self):
        state = _required_state()
        for field in (
            "recovery_state",
            "recovery_reason",
            "wake_state",
            "wake_request_id",
            "wake_started_at",
            "wake_dropout_at",
            "wake_silence_at",
        ):
            state.pop(field)

        from_state = manager.PowerCycleRequest.from_state(state)
        from_request = manager.PowerCycleRequest.from_payload(
            _request_payload_from_state(state)
        )

        self.assertEqual(
            from_state.recovery_reason, manager.RECOVERY_REASON_HANDSHAKE
        )
        self.assertEqual(from_request.identity, from_state.identity)

    def test_wake_request_requires_explicit_ordered_evidence(self):
        payload = _request_payload_from_state(_wake_required_state())
        request = manager.PowerCycleRequest.from_payload(payload)
        self.assertEqual(
            request.recovery_reason, manager.RECOVERY_REASON_WAKE_DROPOUT
        )
        self.assertGreater(request.wake_request_id, 0)

        for field, value in (
            ("recovery_reason", "UNKNOWN"),
            ("wake_request_id", 0),
            ("wake_connection_generation", 0),
            ("wake_dropout_generation", 0),
            ("wake_dropout_generation", payload["wake_connection_generation"] + 1),
            ("wake_started_at", payload["wake_dropout_at"] + 1),
            ("wake_dropout_at", payload["detected_at"] + 1),
            ("wake_silence_at", payload["wake_dropout_at"] - 1),
            ("broadcast_fresh", True),
        ):
            with self.subTest(field=field):
                invalid = dict(payload)
                invalid[field] = value
                with self.assertRaises(ValueError):
                    manager.PowerCycleRequest.from_payload(invalid)

    def test_wake_request_enforces_attribution_and_confirmation_boundaries(self):
        payload = _request_payload_from_state(_wake_required_state())
        detected_at = payload["detected_at"]

        payload["wake_dropout_at"] = detected_at - 10
        payload["wake_silence_at"] = detected_at - 10
        payload["wake_started_at"] = payload["wake_dropout_at"] - 60
        manager.PowerCycleRequest.from_payload(payload)

        too_late = dict(payload)
        too_late["wake_started_at"] = too_late["wake_dropout_at"] - 61
        with self.assertRaisesRegex(ValueError, "60s attribution"):
            manager.PowerCycleRequest.from_payload(too_late)

        too_short = dict(payload)
        too_short["wake_silence_at"] = detected_at - 9
        with self.assertRaisesRegex(ValueError, "10s dropout"):
            manager.PowerCycleRequest.from_payload(too_short)

    def test_wake_live_state_enforces_generation_and_timing_evidence(self):
        payload = _wake_required_state()
        manager.PowerCycleRequest.from_state(payload)

        for field, value in (
            ("wake_connection_generation", 0),
            ("wake_dropout_generation", 0),
            ("wake_dropout_generation", payload["wake_connection_generation"] + 1),
            ("wake_started_at", payload["wake_dropout_at"] - 61),
            ("wake_silence_at", payload["power_cycle_required_at"] - 9),
        ):
            with self.subTest(field=field):
                invalid = dict(payload)
                invalid[field] = value
                with self.assertRaises(ValueError):
                    manager.PowerCycleRequest.from_state(invalid)

        future_detection = dict(payload)
        future_detection["power_cycle_required_at"] = payload["timestamp"] + 3
        future_detection["wake_dropout_at"] = (
            future_detection["power_cycle_required_at"] - 10
        )
        future_detection["wake_silence_at"] = future_detection[
            "wake_dropout_at"
        ]
        with self.assertRaisesRegex(ValueError, "follows state timestamp"):
            manager.PowerCycleRequest.from_state(future_detection)

    def test_normal_dropout_requires_exact_generation_and_timing_evidence(self):
        state = _normal_required_state()
        request = manager.PowerCycleRequest.from_state(state)
        self.assertEqual(
            request.recovery_reason, manager.RECOVERY_REASON_NORMAL_DROPOUT
        )
        manager.PowerCycleRequest.from_payload(_request_payload_from_state(state))

        for field, value in (
            ("normal_connection_generation", 0),
            ("normal_dropout_generation", 0),
            (
                "normal_dropout_generation",
                state["normal_connection_generation"] + 1,
            ),
            ("normal_healthy_since_at", state["normal_dropout_at"] - 29),
            ("normal_silence_at", state["power_cycle_required_at"] - 4),
            ("wake_request_id", 1),
            ("startup_missing_since", state["power_cycle_required_at"] - 30),
            ("broadcast_fresh", True),
            ("handle", 255),
        ):
            with self.subTest(field=field):
                invalid = dict(state)
                invalid[field] = value
                with self.assertRaises(ValueError):
                    manager.PowerCycleRequest.from_state(invalid)

        boundary = dict(state)
        boundary["normal_healthy_since_at"] = boundary["normal_dropout_at"] - 30
        boundary["normal_silence_at"] = boundary["power_cycle_required_at"] - 5
        manager.PowerCycleRequest.from_state(boundary)
        unrelated_reset = _request_payload_from_state(state)
        unrelated_reset["session_reset_attempts"] = 1
        with self.assertRaisesRegex(ValueError, "session-reset evidence"):
            manager.PowerCycleRequest.from_payload(unrelated_reset)

    def test_startup_missing_requires_handle_255_and_thirty_second_grace(self):
        state = _startup_required_state()
        request = manager.PowerCycleRequest.from_state(state)
        self.assertEqual(
            request.recovery_reason, manager.RECOVERY_REASON_STARTUP_MISSING
        )
        self.assertEqual(request.handle, 255)
        manager.PowerCycleRequest.from_payload(_request_payload_from_state(state))

        for field, value in (
            ("handle", 2),
            ("startup_missing_since", state["power_cycle_required_at"] - 29),
            ("normal_connection_generation", 1),
            ("wake_request_id", 1),
            ("broadcast_fresh", True),
            ("startup_state", "IDLE"),
        ):
            with self.subTest(field=field):
                invalid = dict(state)
                invalid[field] = value
                with self.assertRaises(ValueError):
                    manager.PowerCycleRequest.from_state(invalid)
        unrelated_reset = _request_payload_from_state(state)
        unrelated_reset["session_reset_attempts"] = 1
        with self.assertRaisesRegex(ValueError, "session-reset evidence"):
            manager.PowerCycleRequest.from_payload(unrelated_reset)

    def test_only_startup_missing_may_use_synthetic_handle(self):
        for factory in (_required_state, _wake_required_state, _normal_required_state):
            with self.subTest(factory=factory.__name__):
                payload = _request_payload_from_state(factory())
                payload["handle"] = 255
                with self.assertRaisesRegex(ValueError, "only STARTUP_MISSING"):
                    manager.PowerCycleRequest.from_payload(payload)

    def test_normal_state_db_override_must_match_resolved_config_path(self):
        with tempfile.TemporaryDirectory() as tmp:
            expected = Path(tmp) / "authoritative.sqlite3"
            other = Path(tmp) / "other.sqlite3"
            path = Path(tmp) / "config.json"
            path.write_text(
                json.dumps(
                    {
                        "schema_version": 2,
                        "mode": "observe",
                        "state_db": str(expected),
                        "power_groups": {},
                    }
                ),
                encoding="utf-8",
            )
            self.assertEqual(
                manager.main(
                    [
                        "--config",
                        str(path),
                        "--state-db",
                        str(other),
                        "--validate-config",
                    ]
                ),
                2,
            )
            self.assertEqual(
                manager.main(
                    [
                        "--config",
                        str(path),
                        "--state-db",
                        str(expected),
                        "--validate-config",
                    ]
                ),
                0,
            )


class ManagerCliTests(unittest.TestCase):
    @staticmethod
    def _write_config(path, state_db, mode):
        path.write_text(
            json.dumps(
                {
                    "schema_version": 2,
                    "mode": mode,
                    "state_db": str(state_db),
                    "power_groups": {
                        GROUP_ID: {
                            "enabled": True,
                            "members": list(MEMBERS),
                            "relay": {
                                "protocol": "legacy_tcp",
                                "host": "127.0.0.1",
                                "port": 50000,
                                "channel": 1,
                            },
                        }
                    },
                }
            ),
            encoding="utf-8",
        )

    def test_cli_mode_override_wins_over_json_mode(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "config.json"
            state_db = Path(tmp) / "state.sqlite3"
            for json_mode, cli_mode in (
                ("observe", "armed"),
                ("armed", "observe"),
            ):
                with self.subTest(json_mode=json_mode, cli_mode=cli_mode):
                    self._write_config(path, state_db, json_mode)
                    received = []

                    def fake_run(config):
                        received.append(config)
                        return 0

                    with mock.patch.object(
                        manager, "run_ros", side_effect=fake_run
                    ):
                        result = manager.main(
                            [
                                "--config",
                                str(path),
                                "--mode",
                                cli_mode,
                            ]
                        )
                    self.assertEqual(result, 0)
                    self.assertEqual(len(received), 1)
                    self.assertEqual(received[0].mode, cli_mode)

    def test_validate_config_reports_effective_off_seconds(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "config.json"
            state_db = Path(tmp) / "state.sqlite3"
            self._write_config(path, state_db, "observe")
            payload = json.loads(path.read_text(encoding="utf-8"))
            payload["policy"] = {"off_seconds": 10}
            path.write_text(json.dumps(payload), encoding="utf-8")

            with mock.patch("builtins.print") as output:
                result = manager.main(
                    ["--config", str(path), "--validate-config"]
                )

            self.assertEqual(result, 0)
            output.assert_called_once()
            self.assertIn("off_seconds=10", output.call_args.args[0])

    def test_repair_before_start_runs_before_damaged_json_is_read(self):
        with tempfile.TemporaryDirectory() as tmp:
            state_db = Path(tmp) / "state.sqlite3"
            damaged = Path(tmp) / "damaged.json"
            damaged.write_text("{not-json", encoding="utf-8")
            calls = []

            def fake_repair(path):
                calls.append(("repair", path))
                return 0

            original_load = manager.load_config

            def recording_load(path):
                calls.append(("load", path))
                return original_load(path)

            with mock.patch.object(
                manager,
                "repair_obligations_without_config",
                side_effect=fake_repair,
            ), mock.patch.object(
                manager, "load_config", side_effect=recording_load
            ):
                result = manager.main(
                    [
                        "--config",
                        str(damaged),
                        "--state-db",
                        str(state_db),
                        "--repair-before-start",
                    ]
                )

            self.assertEqual(result, 2)
            self.assertEqual([name for name, _value in calls], ["repair", "load"])
            self.assertEqual(
                calls[0][1],
                os.path.abspath(str(state_db)),
            )

    def test_failed_startup_repair_prevents_json_read(self):
        with tempfile.TemporaryDirectory() as tmp:
            state_db = Path(tmp) / "state.sqlite3"
            load = mock.Mock(side_effect=AssertionError("JSON must not be read"))
            with mock.patch.object(
                manager, "repair_obligations_without_config", return_value=1
            ), mock.patch.object(manager, "load_config", load):
                result = manager.main(
                    [
                        "--config",
                        str(Path(tmp) / "damaged.json"),
                        "--state-db",
                        str(state_db),
                        "--repair-before-start",
                    ]
                )
            self.assertEqual(result, 2)
            load.assert_not_called()


class DeploymentTests(unittest.TestCase):
    def test_armed_site_identity_requires_exact_driver_and_relay_members(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            driver = root / "driver.json"
            relay = root / "relay.json"
            launch = root / "site.launch"
            driver.write_text(
                json.dumps(
                    {
                        "lidar_config": [
                            {"broadcast_code": code, "enable_connect": True}
                            for code in MEMBERS
                        ]
                        + [{"broadcast_code": "DISABLEDLIDAR01"}]
                    }
                ),
                encoding="utf-8",
            )
            relay.write_text(
                json.dumps(
                    {
                        "schema_version": 2,
                        "power_groups": {
                            GROUP_ID: {"enabled": True, "members": list(MEMBERS)}
                        },
                    }
                ),
                encoding="utf-8",
            )
            launch.write_text(
                "<launch><arg name=\"relay_power_cycle_enable\" default=\"true\"/>"
                + "".join(
                    '<remap from=\"/livox/lidar_%s\" to=\"/test/%s\"/>'
                    % (code, index)
                    for index, code in enumerate(MEMBERS)
                )
                + "</launch>",
                encoding="utf-8",
            )
            self.assertEqual(site_validator.validate(relay, driver, launch), 0)

            data = json.loads(relay.read_text(encoding="utf-8"))
            data["power_groups"][GROUP_ID]["members"][3] = "OTHERLIDAR00001"
            relay.write_text(json.dumps(data), encoding="utf-8")
            with self.assertRaises(site_validator.ValidationError):
                site_validator.validate(relay, driver, launch)

    def test_updater_targets_paired_network_relay_branches(self):
        updater = (ROOT / "update_livox_geph.sh").read_text(
            encoding="utf-8"
        )
        pin = (
            ROOT / "livox_ros_driver" / "cmake" / "pinned_livox_sdk.cmake"
        ).read_text(encoding="utf-8")
        self.assertIn(
            'SDK_BRANCH="${LIVOX_SDK_BRANCH:-network-relay-added}"',
            updater,
        )
        self.assertIn(
            'DRIVER_BRANCH="${LIVOX_DRIVER_BRANCH:-network-relay-added}"',
            updater,
        )
        self.assertIn(
            '"livox_ros_driver/config/livox_lidar_config_multi.json"',
            updater,
        )
        self.assertIn(
            '"livox_ros_driver/launch/livox_lidar_multi.launch"', updater
        )
        self.assertIn('set(LIVOX_SDK_GIT_BRANCH "network-relay-added")', pin)
        self.assertIn(
            "e45774c5d4f2edab96dd6d61479167784d7df8c9", pin
        )

    def test_endpoint_lock_root_is_stable_across_state_databases(self):
        with tempfile.TemporaryDirectory() as tmp:
            previous_home = os.environ.get("HOME")
            os.environ["HOME"] = tmp
            try:
                first = manager._endpoint_lock_root()
                second = manager._endpoint_lock_root()
            finally:
                if previous_home is None:
                    os.environ.pop("HOME", None)
                else:
                    os.environ["HOME"] = previous_home
            expected = os.path.abspath(
                os.path.join(
                    tmp,
                    ".local",
                    "state",
                    "livox-power-cycle-manager",
                    "endpoint-locks",
                )
            )
            self.assertEqual(first, expected)
            self.assertEqual(second, expected)

    def test_endpoint_lock_identity_serializes_all_sets_on_same_relay(self):
        single = _group(channel=2)
        all_channels = _group(channels=(1, 2, 3, 4))
        another_port = _group(port=50001, channels=(1, 2, 3, 4))
        self.assertEqual(single.relay_endpoint_key, all_channels.relay_endpoint_key)
        self.assertNotEqual(
            single.relay_endpoint_key, another_port.relay_endpoint_key
        )
        self.assertNotEqual(single.power_key, all_channels.power_key)

    def test_driver_dropin_and_installer_migrate_legacy_unit_safely(self):
        template = (
            ROOT / "systemd" / "livox-ros-driver-power-cycle.conf.in"
        ).read_text(encoding="utf-8")
        installer = (ROOT / "install_livox_power_cycle_service.sh").read_text(
            encoding="utf-8"
        )
        tokens = {
            "@HOME@",
            "@MANAGER_RUNTIME@",
            "@STATE_DB@",
        }
        self.assertEqual(
            {word for word in tokens if word in template}, tokens
        )
        for token in tokens:
            self.assertIn("s|%s|" % token, installer)
        self.assertIn("ExecStartPre=/usr/bin/python3", template)
        self.assertIn("ExecStopPost=/usr/bin/python3", template)
        self.assertIn("TimeoutStartSec=600", template)
        self.assertIn("TimeoutStopSec=300", template)
        self.assertEqual(template.count("--repair-obligations"), 2)
        self.assertIn("LIVOX_POWER_CYCLE_SAFETY_DROPIN_V2", template)
        self.assertIn(".local/libexec/livox-power-cycle-manager", installer)
        self.assertIn('mv -f -- "${RUNTIME_TEMP}" "${MANAGER_RUNTIME}"', installer)
        self.assertNotIn("--config", template)
        self.assertNotIn("rosrun", template)
        self.assertFalse(
            (ROOT / "systemd" / "livox-power-cycle-manager.service.in").exists()
        )
        self.assertIn(
            'STATE_DIR="${HOME}/.local/state/livox-power-cycle-manager"',
            installer,
        )
        self.assertIn(
            '[[ "${CONFIG_STATE_DB}" == "${STATE_DB}" ]]', installer
        )
        self.assertIn(
            'TEMPLATE_FILE="${SCRIPT_DIR}/systemd/livox-ros-driver-power-cycle.conf.in"',
            installer,
        )
        self.assertIn(
            'LEGACY_UNIT_NAME="livox-power-cycle-manager.service"', installer
        )
        self.assertIn('sudo systemctl stop "${LEGACY_UNIT_NAME}"', installer)
        self.assertIn('sudo systemctl disable "${LEGACY_UNIT_NAME}"', installer)
        self.assertIn('sudo rm -f -- "${LEGACY_UNIT_PATH}"', installer)
        self.assertIn('repair_obligations ||', installer)
        self.assertIn('sudo systemctl start "${LEGACY_UNIT_NAME}"', installer)
        self.assertIn(
            'DRIVER_LOAD_STATE="$(unit_load_state "${DRIVER_UNIT_NAME}")"',
            installer,
        )
        self.assertIn("DRIVER_RESTART_POLICY=", installer)
        self.assertIn('== "always"', installer)
        self.assertIn("DRIVER_KILL_MODE=", installer)
        self.assertIn('== "control-group"', installer)
        self.assertIn("DRIVER_SERVICE_TYPE=", installer)
        self.assertIn('== "simple"', installer)
        self.assertIn("DRIVER_REMAIN_AFTER_EXIT=", installer)
        self.assertIn('== "no"', installer)
        self.assertIn("DropInPaths", installer)
        self.assertNotIn("systemctl restart \"${DRIVER_UNIT_NAME}\"", installer)

    def test_launch_switch_uses_stable_armed_only_child_launch(self):
        main_path = (
            ROOT / "livox_ros_driver" / "launch" / "livox_lidar_multi.launch"
        )
        child_path = (
            ROOT / "livox_ros_driver" / "launch" / "livox_power_cycle.launch"
        )
        main_text = main_path.read_text(encoding="utf-8")
        main_root = ET.parse(main_path).getroot()
        child_root = ET.parse(child_path).getroot()

        self.assertEqual(main_text.count("LIVOX_RELAY_LAUNCH_INTEGRATION"), 1)
        main_args = {
            row.attrib["name"]: row.attrib.get("default")
            for row in main_root.findall("arg")
        }
        self.assertEqual(main_args["relay_power_cycle_enable"], "false")
        self.assertNotIn("relay_power_cycle_config", main_args)
        self.assertNotIn("relay_power_cycle_state_db", main_args)

        include = next(
            row
            for row in main_root.findall("include")
            if row.attrib.get("file")
            == "$(find livox_ros_driver)/launch/livox_power_cycle.launch"
        )
        include_args = {
            row.attrib["name"]: row.attrib.get("value")
            for row in include.findall("arg")
        }
        self.assertEqual(
            include_args,
            {"enable": "$(arg relay_power_cycle_enable)"},
        )
        main_children = list(main_root)
        driver_node = next(
            row
            for row in main_root.findall("node")
            if row.attrib.get("name") == "livox_driver"
        )
        self.assertLess(main_children.index(include), main_children.index(driver_node))

        child_args = {
            row.attrib["name"]: row.attrib.get("default")
            for row in child_root.findall("arg")
        }
        self.assertEqual(child_args["enable"], "false")
        self.assertEqual(set(child_args), {"enable"})
        groups = child_root.findall("group")
        self.assertEqual(len(groups), 1)
        self.assertEqual(groups[0].attrib, {"if": "$(arg enable)"})
        self.assertEqual(child_root.findall("node"), [])
        nodes = child_root.findall(".//node")
        self.assertEqual(len(nodes), 1)
        node = nodes[0]
        self.assertEqual(node.attrib.get("name"), "livox_power_cycle_manager")
        self.assertEqual(node.attrib.get("required"), "false")
        self.assertEqual(node.attrib.get("respawn"), "true")
        self.assertEqual(node.attrib.get("respawn_delay"), "5")
        self.assertIn("--mode armed", node.attrib.get("args", ""))
        self.assertIn("--repair-before-start", node.attrib.get("args", ""))
        self.assertIn(
            "--config $(env HOME)/.config/livox/power_cycle.json",
            node.attrib.get("args", ""),
        )
        self.assertIn(
            "--state-db $(env HOME)/.local/state/livox-power-cycle-manager/state.sqlite3",
            node.attrib.get("args", ""),
        )
        auto_recover = groups[0].find("param")
        self.assertIsNotNone(auto_recover)
        self.assertEqual(auto_recover.attrib.get("name"), "/auto_recover")
        self.assertEqual(auto_recover.attrib.get("value"), "true")
        self.assertEqual(auto_recover.attrib.get("type"), "bool")
        self.assertNotIn("unless", child_path.read_text(encoding="utf-8"))


class LegacyRelayClientTests(unittest.TestCase):
    def test_frames_match_vendor_tool_golden_vectors(self):
        client = manager.CorxLegacyTcpClient(_group(), manager.Policy())
        self.assertEqual(
            client._query_frame(), bytes.fromhex("CC DD B0 01 00 00 0D BE 7C")
        )
        self.assertEqual(
            client._set_frame(True),
            bytes.fromhex("CC DD A1 01 00 01 00 01 A4 48"),
        )
        self.assertEqual(
            client._set_frame(False),
            bytes.fromhex("CC DD A1 01 00 00 00 01 A3 46"),
        )

    def test_four_channel_frames_use_one_atomic_mask(self):
        target = _group(channels=(1, 2, 3, 4))
        client = manager.CorxLegacyTcpClient(target, manager.Policy())
        self.assertEqual(target.channel_mask, 0x0F)
        self.assertEqual(
            client._set_frame(True),
            bytes.fromhex("CC DD A1 01 00 0F 00 0F C0 80"),
        )
        self.assertEqual(
            client._set_frame(False),
            bytes.fromhex("CC DD A1 01 00 00 00 0F B1 62"),
        )

    def test_four_channel_state_transition_is_confirmed_as_one_group(self):
        with _RunningRelay() as relay:
            target = _group(relay.port, channels=(1, 2, 3, 4))
            client = manager.CorxLegacyTcpClient(
                target,
                _policy(
                    connect_timeout_seconds=1,
                    command_timeout_seconds=1,
                    command_retries=2,
                ),
            )
            client.ensure_state(False)
            self.assertEqual(client.query()[0], (False, False, False, False))
            client.ensure_state(True)
            self.assertEqual(client.query()[0], (True, True, True, True))

    def test_v1_handshake_resends_once_and_accumulates_8_plus_1_status(self):
        def field_checksum(body):
            return bytes((sum(body) & 0xFF, 0xAA))

        with _RunningRelay(
            status_checksum=field_checksum,
            version_handshake=True,
            fragment_status_tail=True,
        ) as relay:
            target = _group(relay.port, channels=(1, 2, 3, 4))
            client = manager.CorxLegacyTcpClient(
                target,
                _policy(
                    connect_timeout_seconds=1,
                    command_timeout_seconds=1,
                    command_retries=2,
                ),
            )
            states, warning = client.query()
            self.assertEqual(states, (True, True, True, True))
            self.assertIn("v1.0 handshake", warning)
            self.assertIn("fixed AA tail", warning)

            transition_warning = client.ensure_state(False)
            self.assertEqual(client.query()[0], (False, False, False, False))
            self.assertIn("v1.0 handshake", transition_warning)
            client.ensure_state(True)
            self.assertEqual(client.query()[0], (True, True, True, True))

    def test_repeated_v1_handshake_fails_closed_without_third_command(self):
        with _RunningRelay(
            version_handshake=True,
            repeat_version_handshake=True,
        ) as relay:
            client = manager.CorxLegacyTcpClient(
                _group(relay.port),
                _policy(connect_timeout_seconds=1, command_timeout_seconds=1),
            )
            with self.assertRaisesRegex(
                manager.RelayProtocolError, "repeated v1.0 handshake"
            ):
                client.query()

    def test_multi_channel_mask_does_not_change_unselected_outputs(self):
        with _RunningRelay() as relay:
            target = _group(relay.port, channels=(1, 3))
            client = manager.CorxLegacyTcpClient(
                target,
                _policy(
                    connect_timeout_seconds=1,
                    command_timeout_seconds=1,
                    command_retries=2,
                ),
            )
            client.ensure_state(False)
            self.assertEqual(client.query()[0], (False, True, False, True))
            client.ensure_state(True)
            self.assertEqual(client.query()[0], (True, True, True, True))

    def test_query_and_confirm_each_state_transition(self):
        with _RunningRelay() as relay:
            client = manager.CorxLegacyTcpClient(
                _group(relay.port),
                _policy(
                    connect_timeout_seconds=1,
                    command_timeout_seconds=1,
                    command_retries=2,
                ),
            )
            states, warning = client.query()
            self.assertEqual(states, (True, True, True, True))
            self.assertIsNone(warning)
            client.ensure_state(False)
            self.assertFalse(client.query()[0][0])
            client.ensure_state(True)
            self.assertTrue(client.query()[0][0])

    def test_omitted_status_checksum_requires_explicit_opt_in(self):
        with _RunningRelay(status_checksum=b"\x00\x00") as relay:
            strict = manager.CorxLegacyTcpClient(
                _group(relay.port), _policy(command_timeout_seconds=1)
            )
            with self.assertRaises(manager.RelayProtocolError):
                strict.query()
            compatible = manager.CorxLegacyTcpClient(
                _group(relay.port, allow_omitted=True),
                _policy(command_timeout_seconds=1),
            )
            states, warning = compatible.query()
            self.assertTrue(states[0])
            self.assertIsNotNone(warning)

    def test_field_aa_tail_status_checksum_supports_state_transitions(self):
        def field_checksum(body):
            return bytes((sum(body) & 0xFF, 0xAA))

        with _RunningRelay(status_checksum=field_checksum) as relay:
            client = manager.CorxLegacyTcpClient(
                _group(relay.port),
                _policy(
                    connect_timeout_seconds=1,
                    command_timeout_seconds=1,
                    command_retries=2,
                ),
            )
            states, warning = client.query()
            self.assertEqual(states, (True, True, True, True))
            self.assertIn("fixed AA tail", warning)
            client.ensure_state(False)
            self.assertFalse(client.query()[0][0])
            client.ensure_state(True)
            self.assertTrue(client.query()[0][0])

    def test_field_aa_tail_acceptance_is_narrow(self):
        client = manager.CorxLegacyTcpClient(_group(), manager.Policy())

        for mask in range(0x10):
            body = bytes((0xB0, 1, 0, mask, 0x0D))
            first = sum(body) & 0xFF
            frame = b"\xAA\xBB" + body + bytes((first, 0xAA))
            with self.subTest(mask=mask), mock.patch.object(
                client, "_exchange", return_value=frame
            ):
                states, warning = client.query()
                self.assertEqual(
                    states,
                    tuple(bool(mask & (1 << bit)) for bit in range(4)),
                )
                self.assertIn("fixed AA tail", warning)

        invalid_frames = (
            bytes.fromhex("AA BB B0 01 00 0F 0D CC AA"),
            bytes.fromhex("AA BB B0 01 00 0F 0D CD AB"),
            bytes.fromhex("AA BB B0 02 00 0F 0D CE AA"),
            bytes.fromhex("AA BB B0 01 00 10 0D CE AA"),
            bytes.fromhex("AA BB B0 01 00 0F 0E CE AA"),
        )
        for frame in invalid_frames:
            with self.subTest(frame=frame.hex()), mock.patch.object(
                client, "_exchange", return_value=frame
            ), self.assertRaises(manager.RelayProtocolError):
                client.query()

    def test_nonzero_bad_status_checksum_is_rejected_even_with_opt_in(self):
        with _RunningRelay(status_checksum=b"\x12\x34") as relay:
            compatible = manager.CorxLegacyTcpClient(
                _group(relay.port, allow_omitted=True),
                _policy(command_timeout_seconds=1),
            )
            with self.assertRaises(manager.RelayProtocolError):
                compatible.query()


class StoreTests(unittest.TestCase):
    def test_schema_v2_migrates_existing_cycles_as_charged(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "state.sqlite3"
            target = _group()
            db = sqlite3.connect(str(path))
            try:
                db.executescript(
                    """
                    CREATE TABLE power_cycles (
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
                      detail TEXT NOT NULL DEFAULT ''
                    );
                    PRAGMA user_version=2;
                    """
                )
                db.execute(
                    "INSERT INTO power_cycles(event_id,trigger_bcode,group_id,"
                    "power_key,started_at,safety_started_at,outcome) "
                    "VALUES(?,?,?,?,?,?,?)",
                    (
                        "legacy-cycle",
                        BCODE,
                        target.group_id,
                        target.power_key,
                        time.time(),
                        0.0,
                        "RECOVERY_VERIFIED",
                    ),
                )
                db.commit()
            finally:
                db.close()

            store = manager.StateStore(str(path))
            db = sqlite3.connect(str(path))
            try:
                version = db.execute("PRAGMA user_version").fetchone()[0]
                charged = db.execute(
                    "SELECT budget_charged FROM power_cycles "
                    "WHERE event_id='legacy-cycle'"
                ).fetchone()[0]
            finally:
                db.close()
            self.assertEqual(version, manager.STATE_DB_SCHEMA_VERSION)
            self.assertEqual(charged, 1)
            allowed, reason, _ = store.cycle_limit(
                target.power_key,
                _policy(
                    minimum_cycle_interval_seconds=0,
                    max_cycles_per_24_hours=1,
                ),
            )
            self.assertFalse(allowed)
            self.assertEqual(reason, "daily_limit")

    def test_schema_v2_half_migration_resumes_and_charges_old_rows(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "state.sqlite3"
            db = sqlite3.connect(str(path))
            try:
                db.executescript(
                    """
                    CREATE TABLE power_cycles (
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
                    );
                    CREATE TABLE power_events (
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
                    );
                    PRAGMA user_version=2;
                    """
                )
                db.execute(
                    "INSERT INTO power_cycles(event_id,trigger_bcode,group_id,"
                    "power_key,started_at,safety_started_at,outcome,"
                    "budget_charged) VALUES(?,?,?,?,?,?,?,0)",
                    (
                        "half-migrated-cycle",
                        BCODE,
                        GROUP_ID,
                        _group().power_key,
                        time.time(),
                        0.0,
                        "INTERRUPTED_BEFORE_OFF",
                    ),
                )
                db.commit()
            finally:
                db.close()

            manager.StateStore(str(path))
            manager.StateStore(str(path))
            db = sqlite3.connect(str(path))
            try:
                version = db.execute("PRAGMA user_version").fetchone()[0]
                charged = db.execute(
                    "SELECT budget_charged FROM power_cycles WHERE event_id=?",
                    ("half-migrated-cycle",),
                ).fetchone()[0]
            finally:
                db.close()
            self.assertEqual(version, manager.STATE_DB_SCHEMA_VERSION)
            self.assertEqual(charged, 1)

    def test_schema_v2_half_migration_rejects_unsafe_budget_column(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "state.sqlite3"
            db = sqlite3.connect(str(path))
            try:
                db.executescript(
                    """
                    CREATE TABLE power_cycles (
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
                      budget_charged INTEGER DEFAULT 0
                    );
                    PRAGMA user_version=2;
                    """
                )
            finally:
                db.close()
            with self.assertRaisesRegex(
                manager.StateStoreError, "unsafe budget_charged"
            ):
                manager.StateStore(str(path))
            db = sqlite3.connect(str(path))
            try:
                self.assertEqual(
                    db.execute("PRAGMA user_version").fetchone()[0], 2
                )
            finally:
                db.close()

    def test_schema_v2_migration_rolls_back_all_ddl_on_failure(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "state.sqlite3"
            db = sqlite3.connect(str(path))
            try:
                db.executescript(
                    """
                    CREATE TABLE power_cycles (
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
                      detail TEXT NOT NULL DEFAULT ''
                    );
                    PRAGMA user_version=2;
                    """
                )
            finally:
                db.close()

            class FailingMigrationStore(manager.StateStore):
                def _connect(self):
                    connection = super()._connect()
                    saw_alter = {"value": False}

                    def authorize(action, _one, _two, _db_name, _source):
                        if action == sqlite3.SQLITE_ALTER_TABLE:
                            saw_alter["value"] = True
                        elif (
                            saw_alter["value"]
                            and action == sqlite3.SQLITE_CREATE_TABLE
                        ):
                            return sqlite3.SQLITE_DENY
                        return sqlite3.SQLITE_OK

                    connection.set_authorizer(authorize)
                    return connection

            with self.assertRaises(sqlite3.DatabaseError):
                FailingMigrationStore(str(path))

            db = sqlite3.connect(str(path))
            try:
                version = db.execute("PRAGMA user_version").fetchone()[0]
                columns = tuple(
                    row[1]
                    for row in db.execute(
                        "PRAGMA table_info(power_cycles)"
                    ).fetchall()
                )
                tables = {
                    row[0]
                    for row in db.execute(
                        "SELECT name FROM sqlite_master WHERE type='table'"
                    ).fetchall()
                }
            finally:
                db.close()
            self.assertEqual(version, 2)
            self.assertNotIn("budget_charged", columns)
            self.assertNotIn("power_events", tables)
            manager.StateStore(str(path))

    def test_schema_v3_migrates_persisted_reason_as_handshake(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "state.sqlite3"
            target = _group()
            request = manager.PowerCycleRequest.from_state(_required_state())
            store = manager.StateStore(str(path))
            store.start_event(
                request, target.group_id, target.power_key, 60
            )
            store.finish_event(
                request.event_id, "RECOVERY_TIMEOUT", "legacy alarm"
            )
            store.set_obligation(request, target)

            db = sqlite3.connect(str(path))
            try:
                db.executescript(
                    """
                    ALTER TABLE power_events RENAME TO power_events_v4;
                    CREATE TABLE power_events (
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
                    );
                    INSERT INTO power_events
                      (event_id,trigger_bcode,group_id,power_key,status,
                       attempts,next_attempt,first_seen,last_update,detail)
                    SELECT event_id,trigger_bcode,group_id,power_key,status,
                           attempts,next_attempt,first_seen,last_update,detail
                      FROM power_events_v4;
                    DROP TABLE power_events_v4;

                    ALTER TABLE power_obligations
                      RENAME TO power_obligations_v4;
                    CREATE TABLE power_obligations (
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
                    );
                    INSERT INTO power_obligations
                      (power_key,group_id,event_id,trigger_bcode,members_json,
                       host,port,channel,address,allow_omitted_checksum,label,
                       created_at,last_attempt)
                    SELECT power_key,group_id,event_id,trigger_bcode,members_json,
                           host,port,channel,address,allow_omitted_checksum,label,
                           created_at,last_attempt
                      FROM power_obligations_v4;
                    DROP TABLE power_obligations_v4;
                    PRAGMA user_version=3;
                    """
                )
            finally:
                db.close()

            migrated = manager.StateStore(str(path))
            obligation = migrated.obligations()[0]
            alert = migrated.current_alerts()[0]
            self.assertEqual(
                obligation[3], manager.RECOVERY_REASON_HANDSHAKE
            )
            self.assertEqual(alert[5], manager.RECOVERY_REASON_HANDSHAKE)
            db = sqlite3.connect(str(path))
            try:
                self.assertEqual(
                    db.execute("PRAGMA user_version").fetchone()[0],
                    manager.STATE_DB_SCHEMA_VERSION,
                )
            finally:
                db.close()

    def test_schema_v4_rebuild_preserves_rows_and_allows_new_reasons(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "state.sqlite3"
            target = _group()
            wake = manager.PowerCycleRequest.from_state(_wake_required_state())
            store = manager.StateStore(str(path))
            store.start_event(wake, target.group_id, target.power_key, 60)
            store.set_obligation(wake, target)
            db = sqlite3.connect(str(path))
            try:
                db.execute("PRAGMA user_version=4")
                db.commit()
            finally:
                db.close()

            migrated = manager.StateStore(str(path))
            self.assertEqual(
                migrated.obligations()[0][3],
                manager.RECOVERY_REASON_WAKE_DROPOUT,
            )
            migrated.clear_obligation(target.power_key, wake.event_id)
            normal = manager.PowerCycleRequest.from_state(
                _normal_required_state(MEMBERS[1], episode_count=2)
            )
            ready, _attempt, _detail = migrated.start_event(
                normal, target.group_id, target.power_key, 60
            )
            self.assertTrue(ready)
            startup = manager.PowerCycleRequest.from_state(
                _startup_required_state(MEMBERS[2], episode_count=3)
            )
            migrated.set_obligation(startup, target)
            self.assertEqual(
                migrated.obligations()[0][3],
                manager.RECOVERY_REASON_STARTUP_MISSING,
            )
            db = sqlite3.connect(str(path))
            try:
                self.assertEqual(
                    db.execute("PRAGMA user_version").fetchone()[0],
                    manager.STATE_DB_SCHEMA_VERSION,
                )
            finally:
                db.close()

    def test_schema_v4_rebuild_rolls_back_both_tables_on_failure(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "state.sqlite3"
            manager.StateStore(str(path))
            db = sqlite3.connect(str(path))
            try:
                db.execute("PRAGMA user_version=4")
                db.commit()
            finally:
                db.close()

            class FailingV5Store(manager.StateStore):
                def _connect(self):
                    connection = super()._connect()

                    def authorize(action, one, _two, _db_name, _source):
                        if (
                            action == sqlite3.SQLITE_CREATE_TABLE
                            and one == "power_obligations_v5"
                        ):
                            return sqlite3.SQLITE_DENY
                        return sqlite3.SQLITE_OK

                    connection.set_authorizer(authorize)
                    return connection

            with self.assertRaises(sqlite3.DatabaseError):
                FailingV5Store(str(path))
            db = sqlite3.connect(str(path))
            try:
                self.assertEqual(db.execute("PRAGMA user_version").fetchone()[0], 4)
                tables = {
                    row[0]
                    for row in db.execute(
                        "SELECT name FROM sqlite_master WHERE type='table'"
                    ).fetchall()
                }
            finally:
                db.close()
            self.assertIn("power_events", tables)
            self.assertIn("power_obligations", tables)
            self.assertNotIn("power_events_v5", tables)
            self.assertNotIn("power_obligations_v5", tables)

    def test_obligation_survives_reopen_with_group_and_trigger_identity(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "state.sqlite3"
            store = manager.StateStore(str(path))
            request = manager.PowerCycleRequest.from_state(
                _wake_required_state(MEMBERS[2])
            )
            target = _group()
            ready, attempt, _ = store.start_event(
                request, target.group_id, target.power_key, 60
            )
            self.assertTrue(ready)
            self.assertEqual(attempt, 1)
            cycle_id, reason, _ = store.reserve_cycle(
                request, target, manager.Policy()
            )
            self.assertIsNotNone(cycle_id)
            self.assertEqual(reason, "ok")
            store.set_obligation(request, target)

            reopened = manager.StateStore(str(path))
            obligations = reopened.obligations()
            self.assertEqual(len(obligations), 1)
            self.assertEqual(obligations[0][0].channels, target.channels)
            self.assertEqual(obligations[0][0].group_id, target.group_id)
            self.assertEqual(obligations[0][1], request.event_id)
            self.assertEqual(obligations[0][2], MEMBERS[2])
            self.assertEqual(
                obligations[0][3], manager.RECOVERY_REASON_WAKE_DROPOUT
            )
            self.assertEqual(obligations[0][0].members, MEMBERS)
            allowed, reason, _ = reopened.cycle_limit(
                target.power_key, manager.Policy()
            )
            self.assertFalse(allowed)
            self.assertEqual(reason, "cooldown")

    def test_four_channel_obligation_survives_reopen_without_identity_loss(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "state.sqlite3"
            store = manager.StateStore(str(path))
            request = manager.PowerCycleRequest.from_state(_required_state())
            target = _group(channels=(1, 2, 3, 4))
            ready, _, _ = store.start_event(
                request, target.group_id, target.power_key, 60
            )
            self.assertTrue(ready)
            cycle_id, reason, _ = store.reserve_cycle(
                request, target, manager.Policy()
            )
            self.assertIsNotNone(cycle_id)
            self.assertEqual(reason, "ok")
            store.set_obligation(request, target)

            restored = manager.StateStore(str(path)).obligations()[0][0]
            self.assertEqual(restored.channels, (1, 2, 3, 4))
            self.assertEqual(restored.channel_mask, 0x0F)
            self.assertEqual(restored.power_key, target.power_key)
            self.assertIn("|channels=1,2,3,4", restored.power_key)

    def test_single_channel_power_identity_remains_backward_compatible(self):
        target = _group(channel=3)
        self.assertEqual(
            target.power_key,
            "legacy_tcp|127.0.0.1|50000|1|3",
        )
        self.assertEqual(target.persisted_channel, 3)

    def test_cooldown_and_daily_limit_are_shared_by_all_group_members(self):
        with tempfile.TemporaryDirectory() as tmp:
            store = manager.StateStore(str(Path(tmp) / "state.sqlite3"))
            first = manager.PowerCycleRequest.from_state(
                _required_state(MEMBERS[0])
            )
            target = _group()
            self.assertIsNotNone(
                store.reserve_cycle(first, target, manager.Policy())[0]
            )

            cooldown = _policy(
                minimum_cycle_interval_seconds=60,
                max_cycles_per_24_hours=3,
            )
            allowed, reason, remaining = store.cycle_limit(
                target.power_key, cooldown
            )
            self.assertFalse(allowed)
            self.assertEqual(reason, "cooldown")
            self.assertGreater(remaining, 0)
            self.assertTrue(store.cycle_limit("other-power-key", cooldown)[0])

            daily = _policy(
                minimum_cycle_interval_seconds=0,
                max_cycles_per_24_hours=1,
            )
            allowed, reason, _ = store.cycle_limit(target.power_key, daily)
            self.assertFalse(allowed)
            self.assertEqual(reason, "daily_limit")
            self.assertTrue(store.cycle_limit("other-power-key", daily)[0])

    def test_cancelled_before_off_cycle_releases_safety_budget(self):
        with tempfile.TemporaryDirectory() as tmp:
            store = manager.StateStore(str(Path(tmp) / "state.sqlite3"))
            request = manager.PowerCycleRequest.from_state(_required_state())
            target = _group()
            policy = _policy(
                minimum_cycle_interval_seconds=60,
                max_cycles_per_24_hours=1,
            )
            cycle_id, reason, _ = store.reserve_cycle(
                request, target, policy
            )
            self.assertIsNotNone(cycle_id)
            self.assertEqual(reason, "ok")
            self.assertFalse(store.cycle_limit(target.power_key, policy)[0])
            store.cancel_cycle_before_off(
                cycle_id, "STALE_OR_RECOVERED", "no OFF command was sent"
            )
            self.assertTrue(store.cycle_limit(target.power_key, policy)[0])
            db = sqlite3.connect(store.path)
            try:
                charged, outcome = db.execute(
                    "SELECT budget_charged,outcome FROM power_cycles WHERE id=?",
                    (cycle_id,),
                ).fetchone()
            finally:
                db.close()
            self.assertEqual(charged, 0)
            self.assertEqual(outcome, "STALE_OR_RECOVERED")

    def test_no_off_archival_rolls_back_if_obligation_delete_fails(self):
        with tempfile.TemporaryDirectory() as tmp:
            store = manager.StateStore(str(Path(tmp) / "state.sqlite3"))
            target = _group()
            request = manager.PowerCycleRequest.from_state(_required_state())
            store.start_event(
                request, target.group_id, target.power_key, 60
            )
            cycle_id = store.reserve_cycle(
                request, target, manager.Policy()
            )[0]
            self.assertIsNotNone(cycle_id)
            store.set_obligation(request, target)
            db = sqlite3.connect(store.path)
            try:
                db.execute(
                    "CREATE TRIGGER reject_obligation_delete BEFORE DELETE "
                    "ON power_obligations BEGIN "
                    "SELECT RAISE(ABORT,'injected delete failure'); END"
                )
                db.commit()
            finally:
                db.close()
            with self.assertRaises(sqlite3.DatabaseError):
                store.confirm_on_and_cancel_before_off(
                    int(cycle_id),
                    target.power_key,
                    request.event_id,
                    "STALE_OR_RECOVERED",
                    "no OFF sent",
                )
            db = sqlite3.connect(store.path)
            try:
                cycle = db.execute(
                    "SELECT on_confirmed_at,budget_charged,outcome "
                    "FROM power_cycles WHERE id=?",
                    (cycle_id,),
                ).fetchone()
                obligation_count = db.execute(
                    "SELECT COUNT(*) FROM power_obligations WHERE power_key=?",
                    (target.power_key,),
                ).fetchone()[0]
            finally:
                db.close()
            self.assertIsNone(cycle[0])
            self.assertEqual(cycle[1], 1)
            self.assertEqual(cycle[2], "STARTED")
            self.assertEqual(obligation_count, 1)

    def test_wall_clock_jump_or_restart_cannot_age_out_safety_budget(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "state.sqlite3"
            store = manager.StateStore(str(path))
            target = _group()
            request = manager.PowerCycleRequest.from_state(_required_state())
            self.assertIsNotNone(
                store.reserve_cycle(request, target, manager.Policy())[0]
            )
            db = sqlite3.connect(str(path))
            try:
                db.execute(
                    "UPDATE power_cycles SET started_at=started_at-1000000000"
                )
                db.commit()
            finally:
                db.close()
            reopened = manager.StateStore(str(path))
            allowed, reason, remaining = reopened.cycle_limit(
                target.power_key, manager.Policy()
            )
            self.assertFalse(allowed)
            self.assertEqual(reason, "cooldown")
            self.assertGreater(remaining, 0)

    def test_terminal_event_cannot_be_revived_by_generic_retry(self):
        with tempfile.TemporaryDirectory() as tmp:
            store = manager.StateStore(str(Path(tmp) / "state.sqlite3"))
            request = manager.PowerCycleRequest.from_state(_required_state())
            target = _group()
            self.assertTrue(
                store.start_event(
                    request, target.group_id, target.power_key, 60
                )[0]
            )
            store.finish_event(request.event_id, "UNMAPPED", "terminal")
            store.retry_event(request.event_id, 0, "must not revive")
            self.assertEqual(
                store.start_event(
                    request, target.group_id, target.power_key, 60
                )[2],
                "terminal",
            )

    def test_observed_event_can_adopt_validated_mapping_when_armed(self):
        with tempfile.TemporaryDirectory() as tmp:
            store = manager.StateStore(str(Path(tmp) / "state.sqlite3"))
            request = manager.PowerCycleRequest.from_state(_required_state())
            self.assertTrue(
                store.start_event(
                    request,
                    "unmapped.%s" % request.broadcast_code,
                    "unmapped|%s" % request.broadcast_code,
                    60,
                )[0]
            )
            store.observe_event(request.event_id, "observe only")
            target = _group()
            ready, attempt, reason = store.start_event(
                request,
                target.group_id,
                target.power_key,
                60,
                allow_observed=True,
            )
            self.assertTrue(ready)
            self.assertEqual(attempt, 1)
            self.assertEqual(reason, "ready")

    def test_active_alarm_survives_reopen_until_verified_recovery(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "state.sqlite3"
            target = _group()
            store = manager.StateStore(str(path))
            failed = manager.PowerCycleRequest.from_state(_required_state())
            store.start_event(
                failed, target.group_id, target.power_key, 60
            )
            store.finish_event(failed.event_id, "RECOVERY_TIMEOUT", "member missing")
            self.assertEqual(
                manager.StateStore(str(path)).current_alerts()[0][3],
                "RECOVERY_TIMEOUT",
            )
            recovered = manager.PowerCycleRequest.from_state(
                _required_state(episode_count=2)
            )
            reopened = manager.StateStore(str(path))
            reopened.start_event(
                recovered, target.group_id, target.power_key, 60
            )
            reopened.finish_event(
                recovered.event_id, "RECOVERY_VERIFIED", "all healthy"
            )
            self.assertFalse(reopened.current_alerts())

    def test_wake_alarm_survives_manager_restart_with_original_reason(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "state.sqlite3"
            target = _group()
            failed = manager.PowerCycleRequest.from_state(
                _wake_required_state(MEMBERS[1])
            )
            store = manager.StateStore(str(path))
            store.start_event(
                failed, target.group_id, target.power_key, 60
            )
            store.finish_event(
                failed.event_id, "RECOVERY_TIMEOUT", "wake recovery failed"
            )

            statuses = []
            config = _config(path, target)
            core = manager.PowerCycleManagerCore(
                config,
                manager.StateStore(str(path)),
                statuses.append,
                relay_factory=_FakeRelay,
            )
            core.start()
            core.stop()

            alarm = manager.StateStore(str(path)).current_alerts()[0]
            self.assertEqual(alarm[5], manager.RECOVERY_REASON_WAKE_DROPOUT)
            replay = next(
                row for row in statuses if row["state"] == "RECOVERY_TIMEOUT"
            )
            self.assertEqual(
                replay["recovery_reason"],
                manager.RECOVERY_REASON_WAKE_DROPOUT,
            )

    def test_same_event_id_with_changed_reason_is_refused(self):
        with tempfile.TemporaryDirectory() as tmp:
            store = manager.StateStore(str(Path(tmp) / "state.sqlite3"))
            target = _group()
            handshake = manager.PowerCycleRequest.from_state(_required_state())
            wake_payload = _wake_required_state()
            wake_payload["power_cycle_required_at"] = handshake.detected_at
            wake_payload["wake_dropout_at"] = handshake.detected_at - 10
            wake_payload["wake_silence_at"] = handshake.detected_at - 10
            wake_payload["wake_started_at"] = handshake.detected_at - 20
            wake = manager.PowerCycleRequest.from_state(wake_payload)
            self.assertEqual(wake.event_id, handshake.event_id)

            self.assertTrue(
                store.start_event(
                    handshake, target.group_id, target.power_key, 60
                )[0]
            )
            ready, _attempt, reason = store.start_event(
                wake, target.group_id, target.power_key, 60
            )
            self.assertFalse(ready)
            self.assertEqual(reason, "mapping_changed")
            alert = store.current_alerts()[0]
            self.assertEqual(alert[3], "MAPPING_CHANGED")
            self.assertEqual(alert[5], manager.RECOVERY_REASON_HANDSHAKE)

    def test_observed_event_cannot_adopt_a_different_recovery_reason(self):
        with tempfile.TemporaryDirectory() as tmp:
            store = manager.StateStore(str(Path(tmp) / "state.sqlite3"))
            target = _group()
            handshake = manager.PowerCycleRequest.from_state(_required_state())
            self.assertTrue(
                store.start_event(
                    handshake,
                    "unmapped.%s" % handshake.broadcast_code,
                    "unmapped|%s" % handshake.broadcast_code,
                    60,
                )[0]
            )
            store.observe_event(handshake.event_id, "observe only")

            wake_payload = _wake_required_state()
            wake_payload["power_cycle_required_at"] = handshake.detected_at
            wake_payload["wake_dropout_at"] = handshake.detected_at - 10
            wake_payload["wake_silence_at"] = handshake.detected_at - 10
            wake_payload["wake_started_at"] = handshake.detected_at - 20
            wake = manager.PowerCycleRequest.from_state(wake_payload)
            self.assertEqual(wake.event_id, handshake.event_id)

            ready, _attempt, reason = store.start_event(
                wake,
                target.group_id,
                target.power_key,
                60,
                allow_observed=True,
            )
            self.assertFalse(ready)
            self.assertEqual(reason, "mapping_changed")
            alert = store.current_alerts()[0]
            self.assertEqual(alert[3], "MAPPING_CHANGED")
            self.assertEqual(alert[5], manager.RECOVERY_REASON_HANDSHAKE)
            self.assertFalse(store.obligations())

    def test_group_rename_cannot_reset_physical_endpoint_budget(self):
        with tempfile.TemporaryDirectory() as tmp:
            store = manager.StateStore(str(Path(tmp) / "state.sqlite3"))
            original = _group()
            renamed = _group(group_id="renamed-group")
            store.bind_power_groups([original])
            with self.assertRaisesRegex(manager.StateStoreError, "renaming"):
                store.bind_power_groups([renamed])

    def test_incomplete_reserved_cycle_is_terminal_after_reopen(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "state.sqlite3"
            store = manager.StateStore(str(path))
            target = _group()
            request = manager.PowerCycleRequest.from_state(_required_state())
            store.start_event(
                request, target.group_id, target.power_key, 60
            )
            self.assertIsNotNone(
                store.reserve_cycle(request, target, manager.Policy())[0]
            )
            reopened = manager.StateStore(str(path))
            self.assertEqual(
                reopened.start_event(
                    request, target.group_id, target.power_key, 60
                )[2],
                "terminal",
            )
            db = sqlite3.connect(str(path))
            try:
                outcome, charged = db.execute(
                    "SELECT outcome,budget_charged FROM power_cycles "
                    "WHERE event_id=?",
                    (request.event_id,),
                ).fetchone()
            finally:
                db.close()
            self.assertEqual(outcome, "INTERRUPTED_BEFORE_OFF")
            self.assertEqual(charged, 1)
            self.assertFalse(
                reopened.cycle_limit(target.power_key, manager.Policy())[0]
            )

    def test_legacy_unversioned_state_db_is_rejected_fail_closed(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "state.sqlite3"
            db = sqlite3.connect(str(path))
            try:
                db.execute(
                    "CREATE TABLE obligations (broadcast_code TEXT PRIMARY KEY)"
                )
                db.commit()
            finally:
                db.close()
            with self.assertRaisesRegex(manager.StateStoreError, "legacy"):
                manager.StateStore(str(path))


class _FakeRelay:
    states = [True, True, True, True]
    transitions = []
    snapshots = []
    lock = threading.Lock()

    def __init__(self, target, policy):
        self.target = target

    @classmethod
    def reset(cls, on=True):
        with cls.lock:
            cls.states = [on, True, True, True]
            cls.transitions = []
            cls.snapshots = []

    def query(self):
        with self.lock:
            return tuple(self.states), None

    def ensure_state(self, state, retries=None, deadline_seconds=None):
        with self.lock:
            for channel in self.target.channels:
                self.states[channel - 1] = state
            self.transitions.append(state)
            self.snapshots.append(tuple(self.states))
        return None


class _CrossTalkRelay(_FakeRelay):
    def ensure_state(self, state, retries=None, deadline_seconds=None):
        result = super().ensure_state(
            state, retries=retries, deadline_seconds=deadline_seconds
        )
        if not state:
            with self.lock:
                self.states[1] = False
        return result


class _DropTargetAfterEnsureOnRelay(_FakeRelay):
    drop_on_next_query = False

    @classmethod
    def reset(cls, on=True):
        super().reset(on=on)
        cls.drop_on_next_query = False

    def ensure_state(self, state, retries=None, deadline_seconds=None):
        result = super().ensure_state(
            state, retries=retries, deadline_seconds=deadline_seconds
        )
        if state:
            type(self).drop_on_next_query = True
        return result

    def query(self):
        with self.lock:
            if type(self).drop_on_next_query:
                for channel in self.target.channels:
                    self.states[channel - 1] = False
                type(self).drop_on_next_query = False
        return super().query()


class _TriggerRecoveringStore(manager.StateStore):
    core = None

    def set_obligation(self, request, target):
        super().set_obligation(request, target)
        self.core.accept_state_payload(
            _healthy_state(request.broadcast_code)
        )


class _TriggerGenerationChangingStore(manager.StateStore):
    core = None

    def set_obligation(self, request, target):
        super().set_obligation(request, target)
        changed = _wake_required_state(
            request.broadcast_code,
            request.driver_instance,
            request.episode_count,
            request.wake_request_id,
        )
        changed.update(
            {
                "timestamp": int(time.time()),
                "power_cycle_required_at": request.detected_at,
                "wake_started_at": request.wake_started_at,
                "wake_dropout_at": request.wake_dropout_at,
                "wake_silence_at": request.wake_silence_at,
                "wake_connection_generation": (
                    request.wake_connection_generation + 1
                ),
                "wake_dropout_generation": (
                    request.wake_dropout_generation + 1
                ),
            }
        )
        self.core.accept_state_payload(changed)


class _FailingObligationStore(manager.StateStore):
    def set_obligation(self, request, target):
        raise manager.StateStoreError("injected obligation write failure")


class CoreTests(unittest.TestCase):
    def test_driver_intent_barrier_accepts_exact_current_mapping(self):
        with tempfile.TemporaryDirectory() as tmp:
            target = _group()
            config = replace(
                _config(Path(tmp) / "state.sqlite3", target),
                intent_topic="/livox/group_power_cycle_intent",
                intent_ack_topic="/livox/group_power_cycle_ack",
            )
            sent = []
            core = None

            def emit_intent(payload):
                sent.append(dict(payload))
                if payload["type"] == manager.INTENT_TYPE:
                    core.accept_intent_ack_payload(
                        {
                            "schema_version": 1,
                            "type": manager.INTENT_ACK_TYPE,
                            "token": payload["token"],
                            "group_id": payload["group_id"],
                            "driver_instance": payload["driver_instance"],
                            "accepted": True,
                            "members": list(payload["members"]),
                            "detail": "all members armed",
                        }
                    )

            core = manager.PowerCycleManagerCore(
                config,
                manager.StateStore(config.state_db),
                lambda _row: None,
                intent_emit=emit_intent,
                relay_factory=_FakeRelay,
            )
            request = manager.PowerCycleRequest.from_state(_required_state())
            state, detail, token = core._prepare_driver_intent(request, target)
            self.assertEqual(state, "ACKED")
            self.assertEqual(detail, "all members armed")
            self.assertTrue(token)
            self.assertEqual(sent[0]["valid_for_ms"], 15000)

    def test_driver_intent_barrier_fails_closed_on_timeout(self):
        with tempfile.TemporaryDirectory() as tmp:
            target = _group()
            config = replace(
                _config(
                    Path(tmp) / "state.sqlite3",
                    target,
                    policy=_policy(intent_ack_timeout_seconds=0.02),
                ),
                intent_topic="/livox/group_power_cycle_intent",
                intent_ack_topic="/livox/group_power_cycle_ack",
            )
            sent = []
            core = manager.PowerCycleManagerCore(
                config,
                manager.StateStore(config.state_db),
                lambda _row: None,
                intent_emit=lambda row: sent.append(dict(row)),
                relay_factory=_FakeRelay,
            )
            request = manager.PowerCycleRequest.from_state(_required_state())
            state, detail, token = core._prepare_driver_intent(request, target)
            self.assertEqual(state, "DRIVER_INTENT_ACK_TIMEOUT")
            self.assertIn("did not ACK", detail)
            self.assertTrue(token)
            self.assertTrue(sent)

    def test_driver_intent_timeout_retries_before_cycle_reservation(self):
        class TrackingStore(manager.StateStore):
            reserve_calls = 0

            def reserve_cycle(self, request, target, policy):
                self.reserve_calls += 1
                return super().reserve_cycle(request, target, policy)

        with tempfile.TemporaryDirectory() as tmp:
            statuses = []
            verified = threading.Event()
            target = _group()
            policy = _policy(
                off_seconds=0.01,
                boot_timeout_seconds=2,
                healthy_seconds=0.05,
                status_stale_seconds=1,
                intent_ack_timeout_seconds=0.02,
                intent_ack_attempts=3,
                intent_retry_seconds=0.01,
            )
            config = replace(
                _config(Path(tmp) / "state.sqlite3", target, policy=policy),
                intent_topic="/livox/group_power_cycle_intent",
                intent_ack_topic="/livox/group_power_cycle_ack",
            )
            store = TrackingStore(config.state_db)
            sent = []
            intent_tokens = []
            core = None

            def emit(row):
                statuses.append(dict(row))
                if row["state"] == "RECOVERY_VERIFIED":
                    verified.set()

            def emit_intent(payload):
                sent.append(dict(payload))
                if (
                    payload["type"] == manager.INTENT_TYPE
                    and payload["token"] not in intent_tokens
                ):
                    intent_tokens.append(payload["token"])
                    self.assertEqual(store.reserve_calls, 0)
                    if len(intent_tokens) == 3:
                        core.accept_intent_ack_payload(
                            {
                                "schema_version": 1,
                                "type": manager.INTENT_ACK_TYPE,
                                "token": payload["token"],
                                "group_id": payload["group_id"],
                                "driver_instance": payload["driver_instance"],
                                "accepted": True,
                                "members": list(payload["members"]),
                                "detail": "all members armed",
                            }
                        )

            _FakeRelay.reset(on=True)
            core = manager.PowerCycleManagerCore(
                config,
                store,
                emit,
                intent_emit=emit_intent,
                relay_factory=_FakeRelay,
            )
            core.start()
            feeder = _trigger_group(core, _required_state())

            def publish_recovery():
                self.assertTrue(
                    _wait_until(
                        lambda: any(
                            row["state"] == "POWER_ON_CONFIRMED"
                            for row in statuses
                        ),
                        timeout=2,
                    ),
                    statuses,
                )
                time.sleep(0.02)
                _publish_health(core, repeats=6)

            recovery = threading.Thread(target=publish_recovery)
            recovery.start()
            self.assertTrue(verified.wait(3), statuses)
            feeder.join(timeout=1)
            recovery.join(timeout=2)
            core.stop()
            self.assertEqual(len(intent_tokens), 3)
            cancelled = {
                row["token"]
                for row in sent
                if row["type"] == manager.INTENT_CANCEL_TYPE
            }
            self.assertEqual(cancelled, set(intent_tokens[:2]))
            self.assertEqual(store.reserve_calls, 1)
            self.assertEqual(_FakeRelay.transitions, [False, True])

    def test_exhausted_intent_ack_retries_cancel_without_reserving(self):
        with tempfile.TemporaryDirectory() as tmp:
            statuses = []
            terminal = threading.Event()
            target = _group()
            policy = _policy(
                status_stale_seconds=1,
                intent_ack_timeout_seconds=0.02,
                intent_ack_attempts=3,
                intent_retry_seconds=0.01,
            )
            config = replace(
                _config(Path(tmp) / "state.sqlite3", target, policy=policy),
                intent_topic="/livox/group_power_cycle_intent",
                intent_ack_topic="/livox/group_power_cycle_ack",
            )
            sent = []

            def emit(row):
                statuses.append(dict(row))
                if row["state"] == "DRIVER_INTENT_ACK_TIMEOUT":
                    terminal.set()

            _FakeRelay.reset(on=True)
            store = manager.StateStore(config.state_db)
            core = manager.PowerCycleManagerCore(
                config,
                store,
                emit,
                intent_emit=lambda row: sent.append(dict(row)),
                relay_factory=_FakeRelay,
            )
            core.start()
            feeder = _trigger_group(core, _required_state())
            self.assertTrue(terminal.wait(2), statuses)
            feeder.join(timeout=1)
            core.stop()
            intent_tokens = {
                row["token"]
                for row in sent
                if row["type"] == manager.INTENT_TYPE
            }
            cancelled = {
                row["token"]
                for row in sent
                if row["type"] == manager.INTENT_CANCEL_TYPE
            }
            self.assertEqual(len(intent_tokens), 3)
            self.assertEqual(cancelled, intent_tokens)
            self.assertNotIn(False, _FakeRelay.transitions)
            self.assertFalse(store.obligations())
            db = sqlite3.connect(store.path)
            try:
                cycle_count = db.execute(
                    "SELECT COUNT(*) FROM power_cycles"
                ).fetchone()[0]
            finally:
                db.close()
            self.assertEqual(cycle_count, 0)

    def test_cycle_reservation_exception_stops_and_cancels_acked_intent(self):
        class FailingReserveStore(manager.StateStore):
            def reserve_cycle(self, request, target, policy):
                raise manager.StateStoreError("injected reservation failure")

        with tempfile.TemporaryDirectory() as tmp:
            statuses = []
            failed = threading.Event()
            target = _group()
            config = replace(
                _config(
                    Path(tmp) / "state.sqlite3",
                    target,
                    policy=_policy(status_stale_seconds=1),
                ),
                intent_topic="/livox/group_power_cycle_intent",
                intent_ack_topic="/livox/group_power_cycle_ack",
            )
            sent = []
            core = None

            def emit(row):
                statuses.append(dict(row))
                if row["state"] == "MANAGER_INTERNAL_ERROR":
                    failed.set()

            def emit_intent(payload):
                sent.append(dict(payload))
                if payload["type"] == manager.INTENT_TYPE:
                    core.accept_intent_ack_payload(
                        {
                            "schema_version": 1,
                            "type": manager.INTENT_ACK_TYPE,
                            "token": payload["token"],
                            "group_id": payload["group_id"],
                            "driver_instance": payload["driver_instance"],
                            "accepted": True,
                            "members": list(payload["members"]),
                            "detail": "all members armed",
                        }
                    )

            _FakeRelay.reset(on=True)
            store = FailingReserveStore(config.state_db)
            core = manager.PowerCycleManagerCore(
                config,
                store,
                emit,
                intent_emit=emit_intent,
                relay_factory=_FakeRelay,
            )
            core.start()
            feeder = _trigger_group(core, _required_state())
            self.assertTrue(failed.wait(2), statuses)
            feeder.join(timeout=1)
            core.stop()
            acked_tokens = {
                row["token"]
                for row in sent
                if row["type"] == manager.INTENT_TYPE
            }
            cancelled = {
                row["token"]
                for row in sent
                if row["type"] == manager.INTENT_CANCEL_TYPE
            }
            self.assertTrue(acked_tokens)
            self.assertEqual(cancelled, acked_tokens)
            self.assertNotIn(False, _FakeRelay.transitions)
            self.assertFalse(store.obligations())

    def _verify_group_health_rows(self, rows):
        with tempfile.TemporaryDirectory() as tmp:
            config = _config(
                Path(tmp) / "state.sqlite3",
                _group(),
                policy=_policy(status_stale_seconds=1),
            )
            core = manager.PowerCycleManagerCore(
                config,
                manager.StateStore(config.state_db),
                lambda _row: None,
                relay_factory=_FakeRelay,
            )
            result = []
            worker = threading.Thread(
                target=lambda: result.append(
                    core._wait_group_healthy(_group(), 123, 0.12, 0.02)
                )
            )
            worker.start()
            time.sleep(0.01)
            while worker.is_alive():
                for row in rows:
                    current = dict(row)
                    current["timestamp"] = int(time.time())
                    core.accept_state_payload(current)
                time.sleep(0.005)
            worker.join(timeout=1)
            self.assertFalse(worker.is_alive())
            self.assertEqual(len(result), 1)
            return result[0]

    def test_recovery_rejects_low_power_without_expected_mode_evidence(self):
        for lidar_state in ("PowerSaving", "StandBy"):
            with self.subTest(lidar_state=lidar_state):
                rows = [_healthy_state(code) for code in MEMBERS]
                rows[0].update(
                    {"lidar_state": lidar_state, "publishing": False}
                )

                recovered, unhealthy = self._verify_group_health_rows(rows)

                self.assertFalse(recovered)
                self.assertIn(MEMBERS[0], unhealthy)

    def test_recovery_keeps_strict_state_and_publishing_requirements(self):
        cases = (
            ("normal_without_data", {"lidar_state": "Normal", "publishing": False}),
            (
                "power_saving_with_data",
                {"lidar_state": "PowerSaving", "publishing": True},
            ),
            ("init", {"lidar_state": "Init", "publishing": False}),
            ("config", {"lidar_state": "Config", "publishing": False}),
            ("error", {"lidar_state": "Error", "publishing": False}),
            ("off_lidar_state", {"lidar_state": "Off", "publishing": False}),
            (
                "off_connect_state",
                {
                    "lidar_state": "PowerSaving",
                    "publishing": False,
                    "connect_state": "Off",
                },
            ),
            (
                "handshake_not_idle",
                {
                    "lidar_state": "PowerSaving",
                    "publishing": False,
                    "handshake_state": "HANDSHAKE_STUCK",
                },
            ),
            (
                "not_connected",
                {
                    "lidar_state": "PowerSaving",
                    "publishing": False,
                    "connected": False,
                },
            ),
        )
        for name, changes in cases:
            with self.subTest(case=name):
                rows = [_healthy_state(code) for code in MEMBERS]
                rows[0].update(changes)
                recovered, unhealthy = self._verify_group_health_rows(rows)
                self.assertFalse(recovered)
                self.assertIn(MEMBERS[0], unhealthy)

    def test_stale_or_inconsistent_recovery_state_is_rejected_before_cache(self):
        with tempfile.TemporaryDirectory() as tmp:
            config = _config(Path(tmp) / "state.sqlite3", _group())
            core = manager.PowerCycleManagerCore(
                config,
                manager.StateStore(config.state_db),
                lambda _row: None,
                relay_factory=_FakeRelay,
            )
            stale = _required_state()
            stale["timestamp"] = int(time.time()) - 60
            with self.assertRaisesRegex(ValueError, "timestamp age"):
                core.accept_state_payload(stale)
            self.assertNotIn(BCODE, core._latest)
            self.assertEqual(core.queue_size(), 0)

            for field, value in (
                ("connected", True),
                ("connect_state", "Sampling"),
                ("publishing", True),
                ("broadcast_fresh", False),
            ):
                with self.subTest(field=field):
                    inconsistent = _required_state()
                    inconsistent[field] = value
                    with self.assertRaisesRegex(
                        ValueError, "internally inconsistent"
                    ):
                        core.accept_state_payload(inconsistent)
            self.assertNotIn(BCODE, core._latest)
            self.assertEqual(core.queue_size(), 0)

    def test_recovery_cause_cross_combinations_are_rejected_before_cache(self):
        handshake = _required_state()
        wake = _wake_required_state()
        cases = (
            (
                "handshake_without_broadcast",
                dict(handshake, broadcast_fresh=False),
            ),
            (
                "handshake_with_wake_reason",
                dict(handshake, recovery_reason="WAKE_DROPOUT"),
            ),
            (
                "handshake_with_observing_wake",
                dict(handshake, wake_state="OBSERVING"),
            ),
            (
                "handshake_with_unknown_wake_state",
                dict(handshake, wake_state="UNKNOWN"),
            ),
            (
                "wake_with_broadcast",
                dict(wake, broadcast_fresh=True),
            ),
            (
                "wake_with_handshake_power_state",
                dict(wake, handshake_state="POWER_CYCLE_REQUIRED"),
            ),
            (
                "wake_without_wake_state",
                dict(wake, wake_state="IDLE"),
            ),
            (
                "wake_without_request_identity",
                dict(wake, wake_request_id=0),
            ),
            (
                "idle_with_active_reason",
                dict(handshake, recovery_state="IDLE"),
            ),
        )
        with tempfile.TemporaryDirectory() as tmp:
            config = _config(Path(tmp) / "state.sqlite3", _group())
            core = manager.PowerCycleManagerCore(
                config,
                manager.StateStore(config.state_db),
                lambda _row: None,
                relay_factory=_FakeRelay,
            )
            for name, payload in cases:
                with self.subTest(case=name), self.assertRaises(ValueError):
                    core.accept_state_payload(payload)
            self.assertNotIn(BCODE, core._latest)
            self.assertEqual(core.queue_size(), 0)

    def test_reason_collision_emits_persisted_reason_and_never_cycles(self):
        with tempfile.TemporaryDirectory() as tmp:
            statuses = []
            emitted = threading.Event()

            def emit(row):
                statuses.append(dict(row))
                if row["state"] == "MAPPING_CHANGED":
                    emitted.set()

            _FakeRelay.reset(on=True)
            config = _config(Path(tmp) / "state.sqlite3", _group())
            store = manager.StateStore(config.state_db)
            handshake = manager.PowerCycleRequest.from_state(_required_state())
            store.start_event(
                handshake,
                "unmapped.%s" % handshake.broadcast_code,
                "unmapped|%s" % handshake.broadcast_code,
                60,
            )
            store.observe_event(handshake.event_id, "observe only")

            wake = _wake_required_state()
            wake["power_cycle_required_at"] = handshake.detected_at
            wake["wake_dropout_at"] = handshake.detected_at - 10
            wake["wake_silence_at"] = handshake.detected_at - 10
            wake["wake_started_at"] = handshake.detected_at - 20

            core = manager.PowerCycleManagerCore(
                config, store, emit, relay_factory=_FakeRelay
            )
            core.start()
            core.accept_state_payload(wake)
            self.assertTrue(emitted.wait(2), statuses)
            core.stop()

            row = next(
                item for item in statuses if item["state"] == "MAPPING_CHANGED"
            )
            self.assertEqual(
                row["recovery_reason"], manager.RECOVERY_REASON_HANDSHAKE
            )
            self.assertIn("HANDSHAKE_STUCK", row["detail"])
            self.assertIn("WAKE_DROPOUT", row["detail"])
            self.assertNotIn(False, _FakeRelay.transitions)
            self.assertFalse(store.obligations())

    def test_exact_precheck_matches_recovery_reason_and_wake_evidence(self):
        with tempfile.TemporaryDirectory() as tmp:
            config = _config(
                Path(tmp) / "state.sqlite3",
                _group(),
                policy=_policy(status_stale_seconds=1),
            )
            core = manager.PowerCycleManagerCore(
                config,
                manager.StateStore(config.state_db),
                lambda _row: None,
                relay_factory=_FakeRelay,
            )
            handshake = _required_state()
            wake = _wake_required_state()
            wake["power_cycle_required_at"] = handshake[
                "power_cycle_required_at"
            ]
            wake["wake_dropout_at"] = wake["power_cycle_required_at"] - 10
            wake["wake_started_at"] = wake["wake_dropout_at"] - 10
            wake_request = manager.PowerCycleRequest.from_state(wake)

            core.accept_state_payload(handshake)
            ready, detail = core._wait_trigger_required(wake_request, 0.03)
            self.assertFalse(ready, detail)
            self.assertIn("identity and cause", detail)

            core.accept_state_payload(wake)
            ready, detail = core._wait_trigger_required(wake_request, 0.03)
            self.assertTrue(ready, detail)

            changed_evidence = dict(wake)
            changed_evidence["wake_request_id"] += 1
            core.accept_state_payload(changed_evidence)
            ready, detail = core._wait_trigger_required(wake_request, 0.03)
            self.assertFalse(ready, detail)

            changed_generation = dict(wake)
            changed_generation["wake_connection_generation"] += 1
            changed_generation["wake_dropout_generation"] += 1
            core.accept_state_payload(changed_generation)
            ready, detail = core._wait_trigger_required(wake_request, 0.03)
            self.assertFalse(ready, detail)

    def test_single_cached_state_cannot_pass_relay_precheck_gate(self):
        with tempfile.TemporaryDirectory() as tmp:
            statuses = []
            retry = threading.Event()

            def emit(row):
                statuses.append(dict(row))
                if row["state"] == "PRECHECK_RETRY":
                    retry.set()

            _FakeRelay.reset(on=True)
            policy = _policy(
                status_stale_seconds=0.08,
                precheck_retry_seconds=60,
                precheck_max_attempts=2,
            )
            config = _config(Path(tmp) / "state.sqlite3", _group(), policy=policy)
            core = manager.PowerCycleManagerCore(
                config,
                manager.StateStore(config.state_db),
                emit,
                relay_factory=_FakeRelay,
            )
            core.start()
            core.accept_state_payload(_wake_required_state())
            self.assertTrue(retry.wait(1), statuses)
            core.stop()
            self.assertEqual(_FakeRelay.transitions, [])
            self.assertTrue(
                any("newer state" in row["detail"] for row in statuses),
                statuses,
            )

    def test_only_trigger_member_must_be_fresh_before_shared_cycle(self):
        with tempfile.TemporaryDirectory() as tmp:
            config = _config(
                Path(tmp) / "state.sqlite3",
                _group(),
                policy=_policy(status_stale_seconds=1),
            )
            core = manager.PowerCycleManagerCore(
                config,
                manager.StateStore(config.state_db),
                lambda _row: None,
                relay_factory=_FakeRelay,
            )
            required = _required_state()
            core.accept_state_payload(required)
            ready, detail = core._wait_trigger_required(
                manager.PowerCycleRequest.from_state(required), 0.03
            )
            self.assertTrue(ready, detail)

    def test_recovery_rejects_health_cached_before_on_confirmation(self):
        with tempfile.TemporaryDirectory() as tmp:
            config = _config(
                Path(tmp) / "state.sqlite3",
                _group(),
                policy=_policy(status_stale_seconds=1),
            )
            core = manager.PowerCycleManagerCore(
                config,
                manager.StateStore(config.state_db),
                lambda _row: None,
                relay_factory=_FakeRelay,
            )
            for code in MEMBERS:
                core.accept_state_payload(_healthy_state(code))
            recovered, unhealthy = core._wait_group_healthy(
                _group(), 123, 0.04, 0.02
            )
            self.assertFalse(recovered)
            self.assertEqual(tuple(unhealthy), MEMBERS)

    def test_group_health_defensively_requires_recovery_state_idle(self):
        """Even a corrupted/legacy cache row cannot verify post-ON health."""

        with tempfile.TemporaryDirectory() as tmp:
            config = _config(
                Path(tmp) / "state.sqlite3",
                _group(),
                policy=_policy(status_stale_seconds=1),
            )
            core = manager.PowerCycleManagerCore(
                config,
                manager.StateStore(config.state_db),
                lambda _row: None,
                relay_factory=_FakeRelay,
            )
            result = []
            worker = threading.Thread(
                target=lambda: result.append(
                    core._wait_group_healthy(_group(), 123, 0.08, 0.01)
                )
            )
            worker.start()
            time.sleep(0.01)
            while worker.is_alive():
                with core._condition:
                    for code in MEMBERS:
                        payload = _healthy_state(code)
                        payload["recovery_state"] = "POWER_CYCLE_REQUIRED"
                        core._state_sequence += 1
                        core._latest[code] = (
                            payload,
                            time.monotonic(),
                            core._state_sequence,
                        )
                    core._condition.notify_all()
                time.sleep(0.005)
            worker.join(timeout=1)

            self.assertEqual(len(result), 1)
            recovered, unhealthy = result[0]
            self.assertFalse(recovered)
            self.assertEqual(tuple(unhealthy), MEMBERS)

    def test_observe_mode_never_constructs_relay_client(self):
        with tempfile.TemporaryDirectory() as tmp:
            statuses = []

            def forbidden_factory(*_args):
                self.fail("observe mode touched relay factory")

            config = _config(
                Path(tmp) / "state.sqlite3",
                _group(enabled=False),
                mode="observe",
            )
            store = manager.StateStore(config.state_db)
            core = manager.PowerCycleManagerCore(
                config,
                store,
                statuses.append,
                relay_factory=forbidden_factory,
            )
            core.start()
            observed_state = _required_state()
            core.accept_state_payload(observed_state)
            self.assertTrue(
                _wait_until(
                    lambda: any(
                        row["state"] == "OBSERVE_ONLY" for row in statuses
                    ),
                    timeout=2,
                ),
                statuses,
            )
            core.stop()
            self.assertTrue(any(row["state"] == "OBSERVE_ONLY" for row in statuses))
            request = manager.PowerCycleRequest.from_state(observed_state)
            self.assertEqual(
                store.start_event(
                    request, GROUP_ID, _group().power_key, 60
                )[2],
                "observed",
            )
            ready, attempt, _ = store.start_event(
                request,
                GROUP_ID,
                _group().power_key,
                60,
                allow_observed=True,
            )
            self.assertTrue(ready)
            self.assertEqual(attempt, 1)

    def test_any_member_triggers_one_shared_cycle_and_all_members_recover(self):
        with tempfile.TemporaryDirectory() as tmp:
            statuses = []
            verified = threading.Event()
            trigger = MEMBERS[2]

            def emit(row):
                statuses.append(dict(row))
                if row["state"] == "RECOVERY_VERIFIED":
                    verified.set()

            _FakeRelay.reset(on=True)
            policy = _policy(
                off_seconds=0.01,
                boot_timeout_seconds=2,
                healthy_seconds=0.05,
                status_stale_seconds=1,
                minimum_cycle_interval_seconds=60,
            )
            config = _config(Path(tmp) / "state.sqlite3", _group(), policy=policy)
            core = manager.PowerCycleManagerCore(
                config,
                manager.StateStore(config.state_db),
                emit,
                relay_factory=_FakeRelay,
            )
            core.start()
            _trigger_group(core, _required_state(trigger))

            def publish_recovery():
                _wait_until(
                    lambda: any(
                        row["state"] == "POWER_ON_CONFIRMED"
                        for row in statuses
                    ),
                    timeout=1,
                )
                # POWER_ON_CONFIRMED is emitted immediately before the core
                # captures its post-ON verification timestamp. Avoid racing
                # that boundary and accidentally publishing all test health
                # frames a few microseconds too early.
                time.sleep(0.02)
                _publish_health(core, repeats=6)

            feeder = threading.Thread(target=publish_recovery)
            feeder.start()
            self.assertTrue(verified.wait(3), statuses)
            feeder.join(timeout=2)
            core.stop()
            self.assertEqual(_FakeRelay.transitions, [False, True])
            self.assertFalse(manager.StateStore(config.state_db).obligations())
            row = next(row for row in statuses if row["state"] == "RECOVERY_VERIFIED")
            self.assertEqual(row["broadcast_code"], trigger)
            self.assertEqual(row["power_group"], GROUP_ID)
            self.assertEqual(tuple(row["members"]), MEMBERS)

    def test_four_channel_group_cycles_all_outputs_and_recovers_all_members(self):
        with tempfile.TemporaryDirectory() as tmp:
            statuses = []
            verified = threading.Event()

            def emit(row):
                statuses.append(dict(row))
                if row["state"] == "RECOVERY_VERIFIED":
                    verified.set()

            _FakeRelay.reset(on=True)
            policy = _policy(
                off_seconds=0.01,
                boot_timeout_seconds=2,
                healthy_seconds=0.05,
                status_stale_seconds=1,
                minimum_cycle_interval_seconds=60,
            )
            target = _group(channels=(1, 2, 3, 4))
            config = _config(Path(tmp) / "state.sqlite3", target, policy=policy)
            core = manager.PowerCycleManagerCore(
                config,
                manager.StateStore(config.state_db),
                emit,
                relay_factory=_FakeRelay,
            )
            core.start()
            _trigger_group(core, _required_state(MEMBERS[1]))

            def publish_recovery():
                _wait_until(
                    lambda: any(
                        row["state"] == "POWER_ON_CONFIRMED"
                        for row in statuses
                    ),
                    timeout=1,
                )
                time.sleep(0.02)
                _publish_health(core, repeats=6)

            feeder = threading.Thread(target=publish_recovery)
            feeder.start()
            self.assertTrue(verified.wait(3), statuses)
            feeder.join(timeout=2)
            core.stop()
            self.assertEqual(_FakeRelay.transitions, [False, True])
            self.assertEqual(
                _FakeRelay.snapshots,
                [(False, False, False, False), (True, True, True, True)],
            )
            self.assertFalse(manager.StateStore(config.state_db).obligations())

    def test_wake_dropout_triggers_one_shared_cycle_and_group_recovery(self):
        with tempfile.TemporaryDirectory() as tmp:
            statuses = []
            verified = threading.Event()
            trigger = MEMBERS[1]

            def emit(row):
                statuses.append(dict(row))
                if row["state"] == "RECOVERY_VERIFIED":
                    verified.set()

            _FakeRelay.reset(on=True)
            policy = _policy(
                off_seconds=0.01,
                boot_timeout_seconds=2,
                healthy_seconds=0.05,
                status_stale_seconds=1,
                minimum_cycle_interval_seconds=60,
            )
            config = _config(Path(tmp) / "state.sqlite3", _group(), policy=policy)
            core = manager.PowerCycleManagerCore(
                config,
                manager.StateStore(config.state_db),
                emit,
                relay_factory=_FakeRelay,
            )
            core.start()
            trigger_feeder = _trigger_group(
                core, _wake_required_state(trigger)
            )

            def publish_recovery():
                _wait_until(
                    lambda: any(
                        row["state"] == "POWER_ON_CONFIRMED"
                        for row in statuses
                    ),
                    timeout=1,
                )
                # Do not let the remaining synthetic pre-OFF frames overwrite
                # post-ON healthy rows for the triggering member.
                trigger_feeder.join(timeout=1)
                time.sleep(0.02)
                _publish_health(core, repeats=6)

            feeder = threading.Thread(target=publish_recovery)
            feeder.start()
            try:
                self.assertTrue(verified.wait(3), statuses)
            finally:
                feeder.join(timeout=2)
                core.stop()

            self.assertEqual(_FakeRelay.transitions, [False, True])
            power_off = next(
                row for row in statuses if row["state"] == "POWER_OFF_COMMAND"
            )
            self.assertEqual(
                power_off["recovery_reason"],
                manager.RECOVERY_REASON_WAKE_DROPOUT,
            )
            self.assertEqual(power_off["broadcast_code"], trigger)
            self.assertFalse(manager.StateStore(config.state_db).obligations())

    def test_normal_and_startup_causes_each_trigger_one_shared_cycle(self):
        cases = (
            (_normal_required_state, manager.RECOVERY_REASON_NORMAL_DROPOUT),
            (_startup_required_state, manager.RECOVERY_REASON_STARTUP_MISSING),
        )
        for factory, expected_reason in cases:
            with self.subTest(reason=expected_reason), tempfile.TemporaryDirectory() as tmp:
                statuses = []
                verified = threading.Event()

                def emit(row):
                    statuses.append(dict(row))
                    if row["state"] == "RECOVERY_VERIFIED":
                        verified.set()

                _FakeRelay.reset(on=True)
                policy = _policy(
                    off_seconds=0.01,
                    boot_timeout_seconds=2,
                    healthy_seconds=0.05,
                    status_stale_seconds=1,
                    minimum_cycle_interval_seconds=60,
                )
                config = _config(
                    Path(tmp) / "state.sqlite3", _group(), policy=policy
                )
                core = manager.PowerCycleManagerCore(
                    config,
                    manager.StateStore(config.state_db),
                    emit,
                    relay_factory=_FakeRelay,
                )
                core.start()
                trigger_feeder = _trigger_group(core, factory(MEMBERS[1]))

                def publish_recovery():
                    _wait_until(
                        lambda: any(
                            row["state"] == "POWER_ON_CONFIRMED"
                            for row in statuses
                        ),
                        timeout=1,
                    )
                    trigger_feeder.join(timeout=1)
                    time.sleep(0.02)
                    _publish_health(core, repeats=6)

                feeder = threading.Thread(target=publish_recovery)
                feeder.start()
                try:
                    self.assertTrue(verified.wait(3), statuses)
                finally:
                    feeder.join(timeout=2)
                    core.stop()
                self.assertEqual(_FakeRelay.transitions, [False, True])
                power_off = next(
                    row for row in statuses if row["state"] == "POWER_OFF_COMMAND"
                )
                self.assertEqual(power_off["recovery_reason"], expected_reason)
                self.assertFalse(
                    manager.StateStore(config.state_db).obligations()
                )

    def test_recovery_is_not_verified_until_last_member_is_healthy(self):
        with tempfile.TemporaryDirectory() as tmp:
            statuses = []
            verified = threading.Event()
            partial_published = threading.Event()
            release_last_member = threading.Event()

            def emit(row):
                statuses.append(dict(row))
                if row["state"] == "RECOVERY_VERIFIED":
                    verified.set()

            _FakeRelay.reset(on=True)
            policy = _policy(
                off_seconds=0.01,
                boot_timeout_seconds=1,
                healthy_seconds=0.04,
                status_stale_seconds=1,
                minimum_cycle_interval_seconds=60,
            )
            config = _config(Path(tmp) / "state.sqlite3", _group(), policy=policy)
            core = manager.PowerCycleManagerCore(
                config,
                manager.StateStore(config.state_db),
                emit,
                relay_factory=_FakeRelay,
            )
            core.start()
            _trigger_group(core, _required_state())

            def publish_recovery():
                _wait_until(
                    lambda: _FakeRelay.transitions[-2:] == [False, True],
                    timeout=1,
                )
                _publish_health(core, members=MEMBERS[:-1], repeats=5)
                partial_published.set()
                release_last_member.wait(1)
                _publish_health(core, repeats=6)

            feeder = threading.Thread(target=publish_recovery)
            feeder.start()
            self.assertTrue(partial_published.wait(2), statuses)
            self.assertFalse(verified.is_set(), statuses)
            release_last_member.set()
            self.assertTrue(verified.wait(2), statuses)
            feeder.join(timeout=2)
            core.stop()
            self.assertEqual(_FakeRelay.transitions, [False, True])

    def test_one_unrecovered_member_times_out_the_entire_group(self):
        with tempfile.TemporaryDirectory() as tmp:
            statuses = []
            timed_out = threading.Event()
            missing = MEMBERS[-1]

            def emit(row):
                statuses.append(dict(row))
                if row["state"] == "RECOVERY_TIMEOUT":
                    timed_out.set()

            _FakeRelay.reset(on=True)
            policy = _policy(
                off_seconds=0.01,
                boot_timeout_seconds=0.2,
                healthy_seconds=0.04,
                status_stale_seconds=1,
                minimum_cycle_interval_seconds=60,
            )
            config = _config(Path(tmp) / "state.sqlite3", _group(), policy=policy)
            core = manager.PowerCycleManagerCore(
                config,
                manager.StateStore(config.state_db),
                emit,
                relay_factory=_FakeRelay,
            )
            core.start()
            _trigger_group(core, _required_state())

            def publish_partial_recovery():
                _wait_until(
                    lambda: _FakeRelay.transitions[-2:] == [False, True],
                    timeout=1,
                )
                _publish_health(
                    core, members=MEMBERS[:-1], repeats=20, interval=0.02
                )

            feeder = threading.Thread(target=publish_partial_recovery)
            feeder.start()
            self.assertTrue(timed_out.wait(2), statuses)
            feeder.join(timeout=2)
            core.stop()
            row = next(row for row in statuses if row["state"] == "RECOVERY_TIMEOUT")
            self.assertIn(missing, row["detail"])
            self.assertEqual(_FakeRelay.transitions, [False, True])

    def test_concurrent_members_in_same_group_do_not_repeat_power_cycle(self):
        with tempfile.TemporaryDirectory() as tmp:
            statuses = []
            first_verified = threading.Event()
            second_terminal = threading.Event()
            first_state = _required_state(MEMBERS[0])
            second_state = _required_state(MEMBERS[1])
            first_request = manager.PowerCycleRequest.from_state(first_state)
            second_request = manager.PowerCycleRequest.from_state(second_state)

            def emit(row):
                statuses.append(dict(row))
                if (
                    row["event_id"] == first_request.event_id
                    and row["state"] == "RECOVERY_VERIFIED"
                ):
                    first_verified.set()
                if (
                    row["event_id"] == second_request.event_id
                    and row["state"] in manager.TERMINAL_EVENT_STATES
                ):
                    second_terminal.set()

            _FakeRelay.reset(on=True)
            policy = _policy(
                off_seconds=0.01,
                boot_timeout_seconds=2,
                healthy_seconds=0.04,
                status_stale_seconds=1,
                minimum_cycle_interval_seconds=60,
            )
            config = _config(Path(tmp) / "state.sqlite3", _group(), policy=policy)
            core = manager.PowerCycleManagerCore(
                config,
                manager.StateStore(config.state_db),
                emit,
                relay_factory=_FakeRelay,
            )
            core.start()
            _trigger_group(core, [first_state, second_state])

            def publish_recovery():
                _wait_until(
                    lambda: _FakeRelay.transitions[:1] == [False], timeout=1
                )
                _wait_until(
                    lambda: _FakeRelay.transitions[-2:] == [False, True],
                    timeout=1,
                )
                _publish_health(core, repeats=8)

            feeder = threading.Thread(target=publish_recovery)
            feeder.start()
            self.assertTrue(first_verified.wait(3), statuses)
            self.assertTrue(second_terminal.wait(2), statuses)
            feeder.join(timeout=2)
            core.stop()
            self.assertEqual(_FakeRelay.transitions, [False, True])
            db = sqlite3.connect(config.state_db)
            try:
                cycle_count, group_count = db.execute(
                    "SELECT COUNT(*),COUNT(DISTINCT group_id) FROM power_cycles"
                ).fetchone()
            finally:
                db.close()
            self.assertEqual(cycle_count, 1)
            self.assertEqual(group_count, 1)

    def test_channel_off_during_live_recovery_is_mapping_mismatch(self):
        with tempfile.TemporaryDirectory() as tmp:
            statuses = []
            mismatch = threading.Event()

            def emit(row):
                statuses.append(dict(row))
                if row["state"] == "MAPPING_MISMATCH":
                    mismatch.set()

            _FakeRelay.reset(on=False)
            config = _config(Path(tmp) / "state.sqlite3", _group())
            core = manager.PowerCycleManagerCore(
                config,
                manager.StateStore(config.state_db),
                emit,
                relay_factory=_FakeRelay,
            )
            core.start()
            _trigger_group(core, _wake_required_state())
            self.assertTrue(mismatch.wait(2), statuses)
            core.stop()
            self.assertEqual(_FakeRelay.transitions, [])
            row = next(
                row for row in statuses if row["state"] == "MAPPING_MISMATCH"
            )
            self.assertIn("recovery condition is still live", row["detail"])
            self.assertNotIn("broadcast is fresh", row["detail"])

    def test_non_target_relay_change_aborts_and_restores_target_on(self):
        with tempfile.TemporaryDirectory() as tmp:
            statuses = []
            failed = threading.Event()

            def emit(row):
                statuses.append(dict(row))
                if row["state"] == "NON_TARGET_STATE_CHANGED":
                    failed.set()

            _CrossTalkRelay.reset(on=True)
            policy = _policy(
                off_seconds=0.01,
                boot_timeout_seconds=1,
                healthy_seconds=0.04,
                status_stale_seconds=1,
            )
            config = _config(Path(tmp) / "state.sqlite3", _group(), policy=policy)
            core = manager.PowerCycleManagerCore(
                config,
                manager.StateStore(config.state_db),
                emit,
                relay_factory=_CrossTalkRelay,
            )
            core.start()
            _trigger_group(core, _required_state())
            self.assertTrue(failed.wait(2), statuses)
            core.stop()
            self.assertTrue(_CrossTalkRelay.states[0])
            self.assertFalse(_CrossTalkRelay.states[1])
            self.assertFalse(manager.StateStore(config.state_db).obligations())

    def test_wake_trigger_recovered_after_reservation_never_sends_off(self):
        with tempfile.TemporaryDirectory() as tmp:
            statuses = []
            stopped = threading.Event()

            def emit(row):
                statuses.append(dict(row))
                if row["state"] == "STALE_OR_RECOVERED":
                    stopped.set()

            _FakeRelay.reset(on=True)
            config = _config(Path(tmp) / "state.sqlite3", _group())
            store = _TriggerRecoveringStore(config.state_db)
            core = manager.PowerCycleManagerCore(
                config, store, emit, relay_factory=_FakeRelay
            )
            store.core = core
            core.start()
            _trigger_group(core, _wake_required_state())
            self.assertTrue(stopped.wait(2), statuses)
            core.stop()
            self.assertNotIn(False, _FakeRelay.transitions)
            self.assertFalse(store.obligations())
            self.assertTrue(
                store.cycle_limit(_group().power_key, config.policy)[0]
            )
            db = sqlite3.connect(store.path)
            try:
                charged = db.execute(
                    "SELECT budget_charged FROM power_cycles"
                ).fetchone()[0]
            finally:
                db.close()
            self.assertEqual(charged, 0)

    def test_wake_generation_change_after_b0_never_sends_off(self):
        with tempfile.TemporaryDirectory() as tmp:
            statuses = []
            stopped = threading.Event()

            def emit(row):
                statuses.append(dict(row))
                if row["state"] == "STALE_OR_RECOVERED":
                    stopped.set()

            _FakeRelay.reset(on=True)
            config = _config(Path(tmp) / "state.sqlite3", _group())
            store = _TriggerGenerationChangingStore(config.state_db)
            core = manager.PowerCycleManagerCore(
                config, store, emit, relay_factory=_FakeRelay
            )
            store.core = core
            core.start()
            _trigger_group(core, _wake_required_state())
            self.assertTrue(stopped.wait(2), statuses)
            core.stop()
            self.assertNotIn(False, _FakeRelay.transitions)
            self.assertFalse(store.obligations())
            self.assertTrue(
                store.cycle_limit(_group().power_key, config.policy)[0]
            )

    def test_no_off_path_keeps_obligation_if_latest_b0_is_not_on(self):
        with tempfile.TemporaryDirectory() as tmp:
            statuses = []
            _DropTargetAfterEnsureOnRelay.reset(on=True)
            policy = _policy(status_stale_seconds=1)
            config = _config(Path(tmp) / "state.sqlite3", _group(), policy=policy)
            store = _TriggerRecoveringStore(config.state_db)
            core = manager.PowerCycleManagerCore(
                config,
                store,
                statuses.append,
                relay_factory=_DropTargetAfterEnsureOnRelay,
            )
            store.core = core
            required = _required_state()
            request = manager.PowerCycleRequest.from_state(required)
            core.accept_state_payload(required)
            ready, attempt, _ = store.start_event(
                request, GROUP_ID, _group().power_key, 60
            )
            self.assertTrue(ready)

            def refresh():
                time.sleep(0.05)
                updated = dict(required)
                updated["timestamp"] = int(time.time())
                core.accept_state_payload(updated)

            feeder = threading.Thread(target=refresh)
            feeder.start()
            core._process(request, attempt)
            feeder.join(timeout=1)
            self.assertNotIn(False, _DropTargetAfterEnsureOnRelay.transitions)
            self.assertEqual(len(store.obligations()), 1)
            db = sqlite3.connect(store.path)
            try:
                off_at, charged = db.execute(
                    "SELECT off_confirmed_at,budget_charged FROM power_cycles"
                ).fetchone()
            finally:
                db.close()
            self.assertIsNone(off_at)
            self.assertEqual(charged, 1)
            self.assertTrue(
                any(row["state"] == "POWER_ON_UNCONFIRMED" for row in statuses)
            )

    def test_restore_path_keeps_obligation_if_latest_b0_is_not_on(self):
        with tempfile.TemporaryDirectory() as tmp:
            statuses = []
            _DropTargetAfterEnsureOnRelay.reset(on=True)
            policy = _policy(
                off_seconds=0.01,
                status_stale_seconds=1,
            )
            config = _config(Path(tmp) / "state.sqlite3", _group(), policy=policy)
            store = manager.StateStore(config.state_db)
            core = manager.PowerCycleManagerCore(
                config,
                store,
                statuses.append,
                relay_factory=_DropTargetAfterEnsureOnRelay,
            )
            required = _required_state()
            request = manager.PowerCycleRequest.from_state(required)
            core.accept_state_payload(required)
            ready, attempt, _ = store.start_event(
                request, GROUP_ID, _group().power_key, 60
            )
            self.assertTrue(ready)

            def refresh():
                time.sleep(0.05)
                updated = dict(required)
                updated["timestamp"] = int(time.time())
                core.accept_state_payload(updated)

            feeder = threading.Thread(target=refresh)
            feeder.start()
            core._process(request, attempt)
            feeder.join(timeout=1)
            self.assertEqual(
                _DropTargetAfterEnsureOnRelay.transitions, [False, True]
            )
            self.assertEqual(len(store.obligations()), 1)
            self.assertTrue(
                any(row["state"] == "POWER_ON_UNCONFIRMED" for row in statuses)
            )

    def test_obligation_failure_before_off_releases_safety_budget(self):
        with tempfile.TemporaryDirectory() as tmp:
            statuses = []
            failed = threading.Event()

            def emit(row):
                statuses.append(dict(row))
                if row["state"] == "POWER_CYCLE_FAILED":
                    failed.set()

            _FakeRelay.reset(on=True)
            policy = _policy(
                minimum_cycle_interval_seconds=60,
                max_cycles_per_24_hours=1,
            )
            config = _config(Path(tmp) / "state.sqlite3", _group(), policy=policy)
            store = _FailingObligationStore(config.state_db)
            core = manager.PowerCycleManagerCore(
                config, store, emit, relay_factory=_FakeRelay
            )
            core.start()
            _trigger_group(core, _required_state())
            self.assertTrue(failed.wait(2), statuses)
            core.stop()
            self.assertNotIn(False, _FakeRelay.transitions)
            self.assertFalse(store.obligations())
            self.assertTrue(store.cycle_limit(_group().power_key, policy)[0])
            db = sqlite3.connect(store.path)
            try:
                charged, outcome = db.execute(
                    "SELECT budget_charged,outcome FROM power_cycles"
                ).fetchone()
            finally:
                db.close()
            self.assertEqual(charged, 0)
            self.assertEqual(outcome, "POWER_CYCLE_FAILED")

    def test_persisted_off_obligation_is_repaired_even_in_observe_mode(self):
        with tempfile.TemporaryDirectory() as tmp:
            restored = threading.Event()
            statuses = []

            def emit(row):
                statuses.append(dict(row))
                if row["state"] == "PERSISTED_ON_RESTORED":
                    restored.set()

            _FakeRelay.reset(on=False)
            config = _config(
                Path(tmp) / "state.sqlite3",
                _group(),
                mode="observe",
                policy=_policy(ensure_on_retry_seconds=0),
            )
            store = manager.StateStore(config.state_db)
            request = manager.PowerCycleRequest.from_state(
                _wake_required_state(MEMBERS[2])
            )
            store.set_obligation(request, _group())
            core = manager.PowerCycleManagerCore(
                config, store, emit, relay_factory=_FakeRelay
            )
            core.start()
            self.assertTrue(restored.wait(2), statuses)
            core.stop()
            self.assertTrue(_FakeRelay.states[0])
            self.assertFalse(store.obligations())
            row = next(
                row for row in statuses if row["state"] == "PERSISTED_ON_RESTORED"
            )
            self.assertEqual(row["broadcast_code"], MEMBERS[2])
            self.assertEqual(row["power_group"], GROUP_ID)
            self.assertEqual(
                row["recovery_reason"],
                manager.RECOVERY_REASON_WAKE_DROPOUT,
            )


if __name__ == "__main__":
    unittest.main()
