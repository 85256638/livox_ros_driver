#!/usr/bin/env python3
"""Behavioral tests for the isolated ARP/ICMP network watchdog."""

import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = (
    ROOT
    / "livox_ros_driver"
    / "livox_ros_driver"
    / "scripts"
    / "livox_network_health_monitor.py"
)
SPEC = importlib.util.spec_from_file_location(
    "livox_network_health_monitor_tested", SCRIPT
)
MONITOR = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MONITOR
SPEC.loader.exec_module(MONITOR)


class NetworkHealthMonitorTests(unittest.TestCase):
    def test_window_states_and_recovery_boundaries(self):
        window = MONITOR.TargetWindow(5.0, 2, 3, 5)
        self.assertEqual(window.observe(True, 1.0)["state"], MONITOR.STATE_UNKNOWN)
        for second in range(2, 5):
            window.observe(True, float(second))
        self.assertEqual(window.observe(True, 5.0)["state"], MONITOR.STATE_OK)
        self.assertEqual(window.observe(False, 6.0)["state"], MONITOR.STATE_DEGRADED)
        self.assertEqual(window.observe(False, 7.0)["state"], MONITOR.STATE_UNSTABLE)
        self.assertEqual(window.observe(False, 8.0)["state"], MONITOR.STATE_UNREACHABLE)
        for second in range(13, 18):
            state = window.observe(True, float(second))["state"]
        self.assertEqual(state, MONITOR.STATE_OK)

    def test_config_requires_five_second_window_and_four_targets(self):
        raw = {
            "schema_version": 1,
            "probe_interval_seconds": 1,
            "window_seconds": 5,
            "unstable_failures": 2,
            "unreachable_consecutive_failures": 3,
            "healthy_consecutive_successes": 5,
            "soft_reboot_max_attempts": 3,
            "soft_reboot_interval_seconds": 5,
            "soft_reboot_ack_timeout_seconds": 2,
            "targets": [
                {
                    "broadcast_code": "EXAMPLE00000001",
                    "ip": "192.168.31.52",
                    "handle": 0,
                },
                {
                    "broadcast_code": "EXAMPLE00000002",
                    "ip": "192.168.31.53",
                    "handle": 1,
                },
                {
                    "broadcast_code": "EXAMPLE00000003",
                    "ip": "192.168.31.54",
                    "handle": 2,
                },
                {
                    "broadcast_code": "EXAMPLE00000004",
                    "ip": "192.168.31.55",
                    "handle": 3,
                },
            ],
        }
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "network_health.json"
            path.write_text(json.dumps(raw), encoding="utf-8")
            config = MONITOR.load_config(str(path))
        self.assertEqual(config.window_seconds, 5.0)
        self.assertEqual(config.probe_interval_seconds, 1.0)
        self.assertEqual(config.soft_reboot_max_attempts, 3)
        self.assertEqual(config.soft_reboot_interval_seconds, 5.0)
        self.assertEqual(config.soft_reboot_ack_timeout_seconds, 2.0)
        self.assertEqual(len(config.targets), 4)

    def test_arp_permission_failure_falls_back_to_icmp(self):
        target = MONITOR.Target("EXAMPLE00000001", "192.168.31.52", 0)
        responses = [
            mock.Mock(returncode=1),
            mock.Mock(returncode=0),
        ]
        with mock.patch.object(MONITOR, "_default_interface", return_value="enp3s0"), mock.patch.object(
            MONITOR.subprocess, "run", side_effect=responses
        ) as run:
            success, method, _rtt, error = MONITOR._run_probe(target, True)
        self.assertTrue(success)
        self.assertEqual(method, "ICMP")
        self.assertEqual(error, "")
        self.assertEqual(run.call_count, 2)
        self.assertEqual(run.call_args_list[1].args[0][0], "ping")


if __name__ == "__main__":
    unittest.main()
