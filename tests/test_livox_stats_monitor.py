import importlib.util
import json
import sqlite3
import sys
import tempfile
import unittest
from contextlib import closing
from pathlib import Path
from types import SimpleNamespace
from unittest import mock


SCRIPT = (
    Path(__file__).resolve().parents[1]
    / "livox_ros_driver"
    / "livox_ros_driver"
    / "scripts"
    / "livox_stats_monitor.py"
)
LAUNCH = (
    Path(__file__).resolve().parents[1]
    / "livox_ros_driver"
    / "launch"
    / "livox_lidar_multi.launch"
).read_text(encoding="utf-8")
SPEC = importlib.util.spec_from_file_location("livox_stats_monitor_tested", SCRIPT)
MONITOR = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MONITOR
SPEC.loader.exec_module(MONITOR)


def valid_payload(**changes):
    payload = {
        "schema_version": 1,
        "type": "POWER_CYCLE_STATUS",
        "timestamp": 1234.5,
        "state": "MANAGER_HEARTBEAT",
        "severity": "INFO",
        "event_id": "",
        "broadcast_code": "",
        "power_group": "",
        "members": [],
        "relay_channels": [],
        "label": "",
        "detail": "mode=auto worker=alive",
    }
    payload.update(changes)
    return payload


def message(payload):
    if isinstance(payload, str):
        data = payload
    else:
        data = json.dumps(payload, separators=(",", ":"))
    return SimpleNamespace(data=data)


DRIVER_STATS = """===== Livox LiDAR Status (1 Hz) =====
==================== SOFTWARE ====================
  (versions embedded in this running binary)
  Driver commit=9fa2371a606e ROS=2.6.0 | paired SDK commit=e45774c5d4f2 SDK=2.3.0 | compatibility=PINNED
==================== CURRENT ALERTS ==============
  [RECOVER] L2 3WEDH7600111081 DATA_VERIFYING
    point-cloud: lost=2026-08-04 22:08:39.100; first data=2026-08-04 22:09:22.700; confirming continuous stream for another 1.2s
==================== CURRENT DEVICES =============
ID  broadcast_code   CURRENT               ASSESS      points/s  HW            connected    disc
0   3WEDH5900101321  NORMAL                STABLE          2504  OK                12m13s       0
1   3WEDH7600109791  NORMAL                WATCH           2504  OK                12m13s       1
2   3WEDH7600111081  DATA_VERIFYING        RECOVERING      2504  OK                 1m03s       1
3   1HDDH3200101851  POWER_SAVING          IDLE               0  OK                12m13s       0
==================== RECENT 60 SECONDS ===========
  (rolling window; samples expire after 60s)
ID  broadcast_code    packet_loss  queue_drops  handshake_timeouts
0   3WEDH5900101321         0.00%            0                   0
1   3WEDH7600109791         0.10%            0                   1
2   3WEDH7600111081         0.00%            0                   0
3   1HDDH3200101851            --            0                   0
==================== ASSESSMENT GUIDE ============
  ACTIVE=current fault; RECOVERING=automatic recovery in progress; IDLE=intentional low-power
==================== MEASUREMENT RECOVERY =========
  (Error budget is per measurement)
  L0 3WEDH5900101321
    MEASUREMENT SESSION: IDLE
    ERROR REBOOTS: 0/3
    POINT-CLOUD: HEALTHY; completed outages=0
    LAST RECOVERY: none in this Driver process
    NEXT ESCALATION: PowerSaving->Normal starts a session; first 3s Error -> soft reboot 1/3
  L1 3WEDH7600109791
    MEASUREMENT SESSION: ACTIVE (explicit id=7)
    ERROR REBOOTS: 1/3
    POINT-CLOUD: HEALTHY; completed outages=1
    LAST RECOVERY: 43.6s (duration ends at first data)
      lost at=2026-08-04 22:07:49.100; first data returned=2026-08-04 22:08:32.700; confirmed healthy=2026-08-04 22:08:35.700
    NEXT ESCALATION: next 3s Error -> soft reboot 2/3
  L2 3WEDH7600111081
    MEASUREMENT SESSION: ACTIVE (explicit id=8)
    ERROR REBOOTS: 1/3
    POINT-CLOUD: VERIFYING; completed outages=0; lost at=2026-08-04 22:08:39.100
    LAST RECOVERY: none in this Driver process
    NEXT ESCALATION: next 3s Error -> soft reboot 2/3
  L3 1HDDH3200101851
    MEASUREMENT SESSION: IDLE
    ERROR REBOOTS: 0/3
    POINT-CLOUD: NOT_EXPECTED; completed outages=0
    LAST RECOVERY: none in this Driver process
    NEXT ESCALATION: PowerSaving->Normal starts a session; first 3s Error -> soft reboot 1/3
==================== PROCESS HISTORY =============
  (Driver process; resets on restart; not current alarms)
  L1 3WEDH7600109791:
    link: disconnect episodes=1; outage duration=1s; current link up=12m13s
    point-cloud outages=1
    hardware fault episodes=1; tags=motor+sys; last=2026-08-04 22:07:49
    automatic reboot actions=1, last=2026-08-04 22:07:52
"""


class MonitorStateTest(unittest.TestCase):
    def setUp(self):
        with MONITOR._lock:
            MONITOR._stats_text = "Waiting for /livox/lidar_stats ...\n"
            MONITOR._stats_received_mono = None
            MONITOR._power_status.clear()
            for key in MONITOR._source_received_mono:
                MONITOR._source_received_mono[key] = None

    def test_malformed_power_messages_do_not_pollute_cache_or_stop_render(self):
        good = valid_payload(timestamp=9.0)
        with mock.patch.object(MONITOR.time, "monotonic", return_value=10.0), mock.patch.object(
            MONITOR, "_render"
        ) as render:
            self.assertTrue(MONITOR.power_heartbeat_cb(message(good)))
            render.assert_called_once_with()

        with MONITOR._lock:
            before_rows = {
                key: dict(value) for key, value in MONITOR._power_status.items()
            }
            before_sources = dict(MONITOR._source_received_mono)

        malformed_payloads = [
            "not-json",
            "[]",
            json.dumps(valid_payload(timestamp=float("nan"))),
            json.dumps(valid_payload(timestamp=float("inf"))),
            message(valid_payload(timestamp=-1)).data,
            message(valid_payload(timestamp="1234.5")).data,
            message(valid_payload(schema_version=True)).data,
            message(valid_payload(state=7)).data,
            message(valid_payload(severity=["INFO"])).data,
            message(valid_payload(broadcast_code=7)).data,
            message(valid_payload(power_group=[])).data,
            message(valid_payload(members="not-an-array")).data,
            message(valid_payload(members=[7])).data,
            message(valid_payload(relay_channels="1,2,3,4")).data,
            message(valid_payload(relay_channels=[1, 1])).data,
            message(valid_payload(relay_channels=[0, 1])).data,
            message(valid_payload(relay_channels=[True])).data,
            message(valid_payload(detail={"bad": "type"})).data,
            "[" * 2000 + "]" * 2000,
            message(
                valid_payload(
                    state="POWER_OFF_FAILED",
                    broadcast_code="",
                    power_group="",
                )
            ).data,
        ]
        with mock.patch.object(MONITOR.time, "monotonic", return_value=20.0), mock.patch.object(
            MONITOR, "_render"
        ) as render:
            for raw in malformed_payloads:
                self.assertFalse(MONITOR.power_cb(SimpleNamespace(data=raw)))
            self.assertFalse(MONITOR.power_cb(SimpleNamespace(data=123)))
            self.assertFalse(MONITOR.power_cb(SimpleNamespace()))
            render.assert_not_called()

        with MONITOR._lock:
            self.assertEqual(before_rows, MONITOR._power_status)
            self.assertEqual(before_sources, MONITOR._source_received_mono)
            rows = [dict(value) for value in MONITOR._power_status.values()]
        # The still-valid cached row remains renderable after every bad input.
        rendered = MONITOR._compose_dashboard(
            "driver\n", 19.0, rows, 20.0, layout="full"
        )
        self.assertIn("MANAGER_HEARTBEAT", rendered)

    def test_invalid_driver_message_does_not_replace_last_good_snapshot(self):
        with mock.patch.object(MONITOR.time, "monotonic", return_value=50.0), mock.patch.object(
            MONITOR, "_render"
        ) as render:
            self.assertTrue(MONITOR.cb(SimpleNamespace(data="good dashboard\n")))
            self.assertFalse(MONITOR.cb(SimpleNamespace(data=object())))
            self.assertFalse(MONITOR.cb(SimpleNamespace(data="")))
            self.assertEqual(render.call_count, 1)
        self.assertEqual(MONITOR._stats_text, "good dashboard\n")
        self.assertEqual(MONITOR._stats_received_mono, 50.0)
        self.assertEqual(MONITOR._source_received_mono["driver_stats"], 50.0)

    def test_each_valid_topic_records_its_own_monotonic_receive_time(self):
        group_status = valid_payload(
            state="RECOVERY_VERIFIED",
            event_id="event-1",
            broadcast_code="1WEDH5900100001",
            power_group="pit1",
        )
        with mock.patch.object(
            MONITOR.time, "monotonic", side_effect=[10.0, 20.0, 30.0]
        ), mock.patch.object(MONITOR, "_render"):
            self.assertTrue(MONITOR.cb(SimpleNamespace(data="driver\n")))
            self.assertTrue(MONITOR.power_cb(message(group_status)))
            self.assertTrue(
                MONITOR.power_heartbeat_cb(message(valid_payload()))
            )
        self.assertEqual(
            {
                "driver_stats": 10.0,
                "power_status": 20.0,
                "power_heartbeat": 30.0,
            },
            MONITOR._source_received_mono,
        )

    def test_empty_group_with_trigger_is_unmapped_not_manager(self):
        payload = valid_payload(
            state="UNMAPPED",
            severity="CRITICAL",
            event_id="event-1",
            broadcast_code="1WEDH5900100001",
            detail="no configured power group",
        )
        row = MONITOR._decode_power_payload(
            message(payload).data, 100.0, "power_status"
        )
        self.assertIsNotNone(row)
        self.assertEqual(
            MONITOR._power_row_key(row), "unmapped:1WEDH5900100001"
        )
        rendered = MONITOR._compose_dashboard(
            "driver\n", 100.0, [row], 101.0, layout="full"
        )
        self.assertIn("UNMAPPED  trigger=1WEDH5900100001", rendered)
        self.assertNotIn("  MANAGER   NOW=UNMAPPED", rendered)
        self.assertIn("NOW=UNMAPPED", rendered)

    def test_group_row_shows_normalized_relay_channel_set(self):
        payload = valid_payload(
            state="POWER_OFF_CONFIRMED",
            severity="WARN",
            event_id="event-1",
            broadcast_code="1WEDH5900100001",
            power_group="pit1",
            members=["1WEDH5900100001"],
            relay_channels=[4, 2, 1, 3],
        )
        row = MONITOR._decode_power_payload(
            message(payload).data, 100.0, "power_status"
        )
        self.assertIsNotNone(row)
        rendered = MONITOR._compose_dashboard(
            "driver\n", 100.0, [row], 101.0, layout="full"
        )
        self.assertIn("relay=1,2,3,4", rendered)

    def test_compact_default_fits_one_screen_and_keeps_actionable_data(self):
        manager = MONITOR._decode_power_payload(
            message(
                valid_payload(
                    detail="mode=armed worker=alive queue=0 obligations=0"
                )
            ).data,
            100.0,
            "power_heartbeat",
        )
        rendered = MONITOR._compose_dashboard(
            DRIVER_STATS, 100.0, [manager], 101.0
        )
        rendered_lines = rendered.rstrip().splitlines()
        self.assertEqual(len(rendered_lines), 24)
        self.assertLessEqual(max(len(line) for line in rendered_lines), 140)
        self.assertIn("Driver=9fa2371 SDK=e45774c PINNED", rendered)
        self.assertIn("PowerMgr=ARMED(1s)", rendered)
        self.assertIn("==================== CURRENT DEVICES", rendered)
        self.assertIn("==================== RECOVERY / RECENT", rendered)
        self.assertIn("==================== ACTION / HISTORY", rendered)
        self.assertIn("DATA_VERIFYING", rendered)
        self.assertIn("[RECOVER] DATA_VERIFYING", rendered)
        self.assertIn("43.6s@22:08:32", rendered)
        self.assertIn("hist disc=1,pc=1,fault=1,reboot=1", rendered)
        self.assertIn("POWER-MGR: state=ARMED", rendered)
        self.assertIn("POWER-EVENT: none", rendered)
        self.assertNotIn("==================== ASSESSMENT GUIDE", rendered)
        self.assertNotIn("MEASUREMENT SESSION:", rendered)
        self.assertNotIn("==================== RELAY HISTORY", rendered)

    def test_compact_power_event_never_hides_trigger_or_relay_channels(self):
        manager = MONITOR._decode_power_payload(
            message(
                valid_payload(
                    detail="mode=armed worker=alive queue=0 obligations=0"
                )
            ).data,
            100.0,
            "power_heartbeat",
        )
        event = MONITOR._decode_power_payload(
            message(
                valid_payload(
                    state="POWER_OFF_CONFIRMED",
                    severity="WARN",
                    event_id="event-1",
                    broadcast_code="3WEDH7600111081",
                    power_group="pit2",
                    members=[
                        "3WEDH5900101321",
                        "3WEDH7600109791",
                        "3WEDH7600111081",
                        "1HDDH3200101851",
                    ],
                    relay_channels=[4, 2, 1, 3],
                )
            ).data,
            100.0,
            "power_status",
        )
        rendered = MONITOR._compose_dashboard(
            DRIVER_STATS, 100.0, [manager, event], 101.0
        )
        self.assertIn("POWER-EVENT: group=pit2", rendered)
        self.assertIn("state=POWER_OFF_CONFIRMED", rendered)
        self.assertIn("trigger=3WEDH7600111081", rendered)
        self.assertIn("relay=1,2,3,4", rendered)

    def test_full_and_history_layouts_remain_available(self):
        full = MONITOR._compose_dashboard(
            DRIVER_STATS, 100.0, [], 101.0, layout="full"
        )
        history = MONITOR._compose_dashboard(
            DRIVER_STATS, 100.0, [], 101.0, layout="history"
        )
        self.assertIn("==================== DATA SOURCE", full)
        self.assertIn("==================== CURRENT DEVICES", full)
        self.assertIn("==================== PROCESS HISTORY", history)
        self.assertIn("point-cloud outages=1", history)
        self.assertIn("==================== MEASUREMENT RECOVERY", history)
        self.assertNotIn("==================== CURRENT DEVICES", history)
        self.assertNotIn("==================== RECENT 60 SECONDS", history)

    def test_history_callback_renders_once_then_requests_shutdown(self):
        fake_rospy = SimpleNamespace(signal_shutdown=mock.Mock())
        original_layout = MONITOR._layout
        try:
            MONITOR._layout = "history"
            MONITOR._history_shutdown_requested = False
            with mock.patch.object(MONITOR, "rospy", fake_rospy), mock.patch.object(
                MONITOR.time, "monotonic", return_value=50.0
            ), mock.patch.object(MONITOR, "_render") as render:
                self.assertTrue(MONITOR.cb(SimpleNamespace(data=DRIVER_STATS)))
                self.assertTrue(MONITOR.cb(SimpleNamespace(data=DRIVER_STATS)))
            self.assertEqual(2, render.call_count)
            fake_rospy.signal_shutdown.assert_called_once_with(
                "one-shot history rendered"
            )
        finally:
            MONITOR._layout = original_layout
            MONITOR._history_shutdown_requested = False

    def test_launch_defaults_to_compact_layout(self):
        self.assertIn('<arg name="monitor_layout" default="compact"/>', LAUNCH)
        self.assertIn('args="--layout $(arg monitor_layout)"', LAUNCH)

    def test_invalid_layout_is_rejected(self):
        with self.assertRaises(ValueError):
            MONITOR._compose_dashboard(
                DRIVER_STATS, 100.0, [], 101.0, layout="sideways"
            )

    def test_driver_stale_and_age_are_based_on_local_monotonic_receive_time(self):
        live = MONITOR._compose_dashboard(
            "driver\n", 100.0, [], 104.9, layout="full"
        )
        stale = MONITOR._compose_dashboard(
            "driver\n", 100.0, [], 106.0, layout="full"
        )
        self.assertIn("==================== DATA SOURCE", live)
        self.assertIn("LIVE=realtime", live)
        self.assertIn("NOW=LIVE", live)
        self.assertIn("driver_age=4s", live)
        self.assertIn("POWER-MGR NOW=NOT_SEEN", live)
        self.assertIn("NOW=DRIVER_STALE", stale)
        self.assertIn("driver_age=6s", stale)

    def test_manager_stale_uses_receive_age_not_payload_wall_timestamp(self):
        future_stamp = valid_payload(timestamp=9999999999.0)
        row = MONITOR._decode_power_payload(
            message(future_stamp).data, 200.0, "power_heartbeat"
        )
        fresh = MONITOR._compose_dashboard(
            "driver\n", 229.9, [row], 229.9, layout="full"
        )
        stale = MONITOR._compose_dashboard(
            "driver\n", 200.0, [row], 231.0, layout="full"
        )
        self.assertIn("==================== POWER RECOVERY", fresh)
        self.assertIn("NOW=MANAGER_HEARTBEAT", fresh)
        self.assertIn("POWER-MGR NOW=MANAGER_HEARTBEAT", fresh)
        self.assertIn("manager_age=29s", fresh)
        self.assertNotIn("MANAGER_STALE", fresh)
        self.assertIn("NOW=MANAGER_STALE", stale)
        self.assertIn("POWER-MGR NOW=MANAGER_STALE", stale)
        self.assertIn("manager_age=31s", stale)

    def test_driver_and_manager_can_both_become_stale_without_new_messages(self):
        row = MONITOR._decode_power_payload(
            message(valid_payload(timestamp=1.0)).data,
            10.0,
            "power_heartbeat",
        )
        rendered = MONITOR._compose_dashboard(
            "old stats\n", 10.0, [row], 50.0, layout="full"
        )
        self.assertIn("NOW=DRIVER_STALE", rendered)
        self.assertIn("NOW=MANAGER_STALE", rendered)
        self.assertIn("driver_age=40s", rendered)
        self.assertIn("manager_age=40s", rendered)

    def test_relay_history_reads_latest_five_persisted_cycles_read_only(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "state.sqlite3"
            with closing(sqlite3.connect(str(path))) as db:
                db.execute(
                    "CREATE TABLE power_cycles (id INTEGER PRIMARY KEY,"
                    "event_id TEXT,started_at REAL,trigger_bcode TEXT,"
                    "group_id TEXT,off_confirmed_at REAL,on_confirmed_at REAL,"
                    "outcome TEXT,detail TEXT)"
                )
                db.execute(
                    "CREATE TABLE power_events (event_id TEXT PRIMARY KEY,"
                    "recovery_reason TEXT)"
                )
                for index in range(1, 7):
                    event_id = "event-%d" % index
                    db.execute(
                        "INSERT INTO power_events VALUES(?,?)",
                        (event_id, "NORMAL_DROPOUT"),
                    )
                    db.execute(
                        "INSERT INTO power_cycles VALUES(?,?,?,?,?,?,?,?,?)",
                        (
                            index,
                            event_id,
                            1700000000.0 + index,
                            "1WEDH5900100001",
                            "pit1",
                            1700000010.0 + index,
                            1700000020.0 + index if index != 6 else None,
                            "RECOVERY_VERIFIED" if index != 6 else "POWER_ON_UNCONFIRMED",
                            "cycle detail %d" % index,
                        ),
                    )
                db.commit()
            history, error = MONITOR._read_relay_history(str(path))
            self.assertIsNone(error)
            self.assertEqual([6, 5, 4, 3, 2], [item["id"] for item in history])
            rendered = MONITOR._compose_dashboard(
                "driver\n", 100.0, [], 101.0, history, error, layout="full"
            )
            self.assertIn("==================== RELAY HISTORY", rendered)
            self.assertIn("reason=NORMAL_DROPOUT", rendered)
            self.assertIn("OFF=YES  ON=--  outcome=POWER_ON_UNCONFIRMED", rendered)
            self.assertNotIn("cycle detail 1", rendered)

    def test_relay_history_missing_database_is_empty_not_an_error(self):
        with tempfile.TemporaryDirectory() as tmp:
            history, error = MONITOR._read_relay_history(
                str(Path(tmp) / "missing.sqlite3")
            )
            self.assertEqual([], history)
            self.assertIsNone(error)
            rendered = MONITOR._compose_dashboard(
                "driver\n", 100.0, [], 101.0, history, error, layout="full"
            )
            self.assertIn("no relay cycle has been recorded", rendered)

    def test_relay_history_schema_error_does_not_break_dashboard(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "bad.sqlite3"
            with closing(sqlite3.connect(str(path))) as db:
                db.execute("CREATE TABLE unrelated (id INTEGER)")
                db.commit()
            history, error = MONITOR._read_relay_history(str(path))
            self.assertEqual([], history)
            self.assertIsNotNone(error)
            rendered = MONITOR._compose_dashboard(
                "driver\n", 100.0, [], 101.0, history, error, layout="full"
            )
            self.assertIn("RELAY HISTORY", rendered)
            self.assertIn("unavailable:", rendered)


class MainTimerTest(unittest.TestCase):
    def test_main_installs_one_second_local_refresh_timer(self):
        class FakeTimer:
            def __init__(self, duration, callback):
                self.duration = duration
                self.callback = callback
                self.shutdown_called = False

            def shutdown(self):
                self.shutdown_called = True

        class FakeRospy:
            def __init__(self):
                self.subscribers = []
                self.timer = None

            def init_node(self, *_args, **_kwargs):
                pass

            def Subscriber(self, *args, **kwargs):
                self.subscribers.append((args, kwargs))

            @staticmethod
            def Duration(seconds):
                return seconds

            def Timer(self, duration, callback):
                self.timer = FakeTimer(duration, callback)
                return self.timer

            @staticmethod
            def spin():
                pass

        fake = FakeRospy()
        with mock.patch.object(MONITOR, "rospy", fake), mock.patch.object(
            MONITOR, "String", object
        ), mock.patch.object(MONITOR, "_render") as render:
            MONITOR.main()

        self.assertEqual(3, len(fake.subscribers))
        self.assertEqual(1.0, fake.timer.duration)
        self.assertIs(fake.timer.callback, MONITOR._refresh_cb)
        self.assertTrue(fake.timer.shutdown_called)
        render.assert_called_once_with()


if __name__ == "__main__":
    unittest.main()
