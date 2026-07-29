import importlib.util
import json
import sys
import unittest
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
        rendered = MONITOR._compose_dashboard("driver\n", 19.0, rows, 20.0)
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
        rendered = MONITOR._compose_dashboard("driver\n", 100.0, [row], 101.0)
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
        rendered = MONITOR._compose_dashboard("driver\n", 100.0, [row], 101.0)
        self.assertIn("relay=1,2,3,4", rendered)

    def test_driver_stale_and_age_are_based_on_local_monotonic_receive_time(self):
        live = MONITOR._compose_dashboard("driver\n", 100.0, [], 104.9)
        stale = MONITOR._compose_dashboard("driver\n", 100.0, [], 106.0)
        self.assertIn("NOW=LIVE", live)
        self.assertIn("driver_age=4s", live)
        self.assertIn("NOW=DRIVER_STALE", stale)
        self.assertIn("driver_age=6s", stale)

    def test_manager_stale_uses_receive_age_not_payload_wall_timestamp(self):
        future_stamp = valid_payload(timestamp=9999999999.0)
        row = MONITOR._decode_power_payload(
            message(future_stamp).data, 200.0, "power_heartbeat"
        )
        fresh = MONITOR._compose_dashboard("driver\n", 229.9, [row], 229.9)
        stale = MONITOR._compose_dashboard("driver\n", 200.0, [row], 231.0)
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
        rendered = MONITOR._compose_dashboard("old stats\n", 10.0, [row], 50.0)
        self.assertIn("NOW=DRIVER_STALE", rendered)
        self.assertIn("NOW=MANAGER_STALE", rendered)
        self.assertIn("driver_age=40s", rendered)
        self.assertIn("manager_age=40s", rendered)


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
