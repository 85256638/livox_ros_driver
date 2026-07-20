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
from pathlib import Path


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


MEMBERS = (
    "TESTLIDAR000001",
    "TESTLIDAR000002",
    "TESTLIDAR000003",
    "TESTLIDAR000004",
)
BCODE = MEMBERS[0]
GROUP_ID = "pit-4-shared"


class _RelayState:
    def __init__(self, mask=0x0F, status_checksum=None):
        self.mask = mask
        self.status_checksum = status_checksum
        self.lock = threading.Lock()


class _RelayHandler(socketserver.BaseRequestHandler):
    def handle(self):
        data = self.request.recv(64)
        if len(data) < 3 or data[:2] != b"\xCC\xDD":
            return
        state = self.server.relay_state
        if data[2] == 0xB0:
            with state.lock:
                mask = state.mask
                status_checksum = state.status_checksum
            body = bytes((0xB0, 1)) + mask.to_bytes(2, "big") + b"\x0D"
            checksum = manager._double_checksum(body)
            if status_checksum is not None:
                checksum = status_checksum
            self.request.sendall(b"\xAA\xBB" + body + checksum)
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
    def __init__(self, mask=0x0F, status_checksum=None):
        self.state = _RelayState(mask, status_checksum)
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
        channel=channel,
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
        "broadcast_fresh": True,
        "publishing": False,
        "published_packets": 0,
        "power_cycle_required_count": episode_count,
        "power_cycle_required_at": now,
    }


def _healthy_state(broadcast_code=BCODE):
    payload = _required_state(broadcast_code)
    payload.update(
        {
            "timestamp": int(time.time()),
            "connected": True,
            "connect_state": "Sampling",
            "lidar_state": "Normal",
            "handshake_state": "IDLE",
            "broadcast_fresh": False,
            "publishing": True,
            "published_packets": 100,
        }
    )
    return payload


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


class DeploymentTests(unittest.TestCase):
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

    def test_systemd_template_is_hardened_and_installer_renders_every_token(self):
        template = (
            ROOT / "systemd" / "livox-power-cycle-manager.service.in"
        ).read_text(encoding="utf-8")
        installer = (ROOT / "install_livox_power_cycle_service.sh").read_text(
            encoding="utf-8"
        )
        tokens = {
            "@USER@",
            "@GROUP@",
            "@HOME@",
            "@CATKIN_WS@",
            "@CONFIG@",
            "@STATE_DIR@",
            "@STATE_DB@",
            "@DRIVER_DIR@",
        }
        self.assertEqual(
            {word for word in tokens if word in template}, tokens
        )
        for token in tokens:
            self.assertIn("s|%s|" % token, installer)
        self.assertIn("NoNewPrivileges=true", template)
        self.assertIn("PartOf=livox-ros-driver.service", template)
        self.assertIn("ProtectHome=read-only", template)
        self.assertIn("ReadWritePaths=@STATE_DIR@", template)
        self.assertNotIn("LIVOX_POWER_LOCK_DIR", template)
        self.assertNotIn("RuntimeDirectory=", template)
        self.assertIn("TimeoutStopSec=300", template)
        self.assertIn("SendSIGKILL=no", template)
        self.assertEqual(template.count("--repair-obligations"), 2)
        self.assertIn("ExecStopPost=", template)
        self.assertIn(
            'STATE_DIR="${HOME}/.local/state/livox-power-cycle-manager"',
            installer,
        )
        self.assertIn(
            '[[ "${CONFIG_STATE_DB}" == "${STATE_DB}" ]]', installer
        )
        self.assertIn("unit 已保留，禁止卸载", installer)
        self.assertNotIn(
            'systemctl disable --now "${UNIT_NAME}" 2>/dev/null || true',
            installer,
        )
        updater = (ROOT / "update_livox_geph.sh").read_text(encoding="utf-8")
        self.assertIn("POWER_MANAGER_STOPPED_BY_UPDATER", updater)
        self.assertIn(
            "脚本异常退出，正在恢复此前运行的 livox-power-cycle-manager",
            updater,
        )


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

    def test_obligation_survives_reopen_with_group_and_trigger_identity(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "state.sqlite3"
            store = manager.StateStore(str(path))
            request = manager.PowerCycleRequest.from_state(
                _required_state(MEMBERS[2])
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
            self.assertEqual(obligations[0][0].channel, target.channel)
            self.assertEqual(obligations[0][0].group_id, target.group_id)
            self.assertEqual(obligations[0][1], request.event_id)
            self.assertEqual(obligations[0][2], MEMBERS[2])
            self.assertEqual(obligations[0][0].members, MEMBERS)
            allowed, reason, _ = reopened.cycle_limit(
                target.power_key, manager.Policy()
            )
            self.assertFalse(allowed)
            self.assertEqual(reason, "cooldown")

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
    lock = threading.Lock()

    def __init__(self, target, policy):
        self.target = target

    @classmethod
    def reset(cls, on=True):
        with cls.lock:
            cls.states = [on, True, True, True]
            cls.transitions = []

    def query(self):
        with self.lock:
            return tuple(self.states), None

    def ensure_state(self, state, retries=None, deadline_seconds=None):
        with self.lock:
            self.states[self.target.channel - 1] = state
            self.transitions.append(state)
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
                self.states[self.target.channel - 1] = False
                type(self).drop_on_next_query = False
        return super().query()


class _TriggerRecoveringStore(manager.StateStore):
    core = None

    def set_obligation(self, request, target):
        super().set_obligation(request, target)
        self.core.accept_state_payload(
            _healthy_state(request.broadcast_code)
        )


class _FailingObligationStore(manager.StateStore):
    def set_obligation(self, request, target):
        raise manager.StateStoreError("injected obligation write failure")


class CoreTests(unittest.TestCase):
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
            core.accept_state_payload(_required_state())
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
                    lambda: _FakeRelay.transitions[-2:] == [False, True],
                    timeout=1,
                )
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

    def test_channel_off_during_fresh_broadcast_is_mapping_mismatch(self):
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
            _trigger_group(core, _required_state())
            self.assertTrue(mismatch.wait(2), statuses)
            core.stop()
            self.assertEqual(_FakeRelay.transitions, [])

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

    def test_trigger_recovered_after_reservation_never_sends_off(self):
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
            _trigger_group(core, _required_state())
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
                _required_state(MEMBERS[2])
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


if __name__ == "__main__":
    unittest.main()
