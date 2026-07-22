import re
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CPP = (
    ROOT / "livox_ros_driver" / "livox_ros_driver" / "lds_lidar.cpp"
).read_text(encoding="utf-8")
HEADER = (
    ROOT / "livox_ros_driver" / "livox_ros_driver" / "lds_lidar.h"
).read_text(encoding="utf-8")
DRIVER = (
    ROOT / "livox_ros_driver" / "livox_ros_driver" / "livox_ros_driver.cpp"
).read_text(encoding="utf-8")


class HandshakeRecoveryPolicySourceTests(unittest.TestCase):
    """Lock the production timing/safety contract until a fake-clock C++ test exists."""

    def test_fast_recovery_timing_and_one_reset_budget(self):
        self.assertIn(
            "kHandshakeFirstResetNs = 5000000000LL", CPP
        )
        self.assertIn(
            "kHandshakePowerCycleNs = 10000000000LL", CPP
        )
        self.assertIn(
            "kHandshakePostResetObserveNs = 5000000000LL", CPP
        )
        self.assertRegex(
            HEADER,
            re.compile(r"HandshakeResetMaxAttempts\(\)\s*\{\s*return 1;\s*\}"),
        )
        self.assertNotIn("kHandshakeResetIntervalNs", CPP)

    def test_hardware_escalation_requires_accepted_completed_reset(self):
        escalation = CPP[
            CPP.index("bool IsPowerCycleEscalationReady(") :
            CPP.index("/** Fill buf with the current wall-clock time")
        ]
        self.assertIn("expected_episode_since_ns", escalation)
        self.assertIn("s.handshake_reset_phase", escalation)
        self.assertIn("kHandshakeResetCompleted", escalation)
        self.assertIn("s.handshake_reset_accepted", escalation)
        self.assertIn("s.handshake_reset_completed", escalation)
        self.assertIn("kHandshakePostResetObserveNs", escalation)
        self.assertIn("kHandshakePowerCycleNs", escalation)

    def test_network_error_has_independent_gate_and_can_cancel_power(self):
        self.assertIn("handshake_last_network_error_ns", HEADER)
        self.assertIn("kHandshakeNetworkErrorGateNs", CPP)
        self.assertIn("POWER_CYCLE_CANCELLED_NETWORK_ERROR", CPP)

    def test_power_alert_is_revalidated_and_counted_only_at_commit(self):
        commit = CPP[
            CPP.index("if (power_cycle_candidate)") :
            CPP.index("if (!request_reset)")
        ]
        self.assertIn("lock_guard<mutex> lock(link_stat_lock_[handle])", commit)
        self.assertIn(
            "IsPowerCycleEscalationReady(s, commit_now, episode_since)", commit
        )
        self.assertLess(
            commit.index("s.power_cycle_required_count++"),
            commit.index('PrintLidarEvent(handle, broadcast_code, "POWER_CYCLE_REQUIRED")'),
        )

    def test_dashboard_has_episode_reset_phase_and_locked_publish_recheck(self):
        self.assertIn("enum HandshakeResetPhase", HEADER)
        self.assertIn("handshake_reset_phase", HEADER)
        self.assertIn('<< ", reset-phase="', DRIVER)
        publish = DRIVER[
            DRIVER.index("const int64_t expected_power_episode") :
            DRIVER.index("ss << line;")
        ]
        self.assertIn("link_stat_lock_[h]", publish)
        self.assertIn(
            "live.broadcast_only_since_ns == expected_power_episode", publish
        )
        self.assertIn(
            "live.power_cycle_required_count == expected_power_count", publish
        )

    def test_dashboard_surfaces_per_lidar_handshake_timeout_history(self):
        stats = DRIVER[
            DRIVER.index("void StatsTimerCb(") :
            DRIVER.index("/** Temp-change footer:")
        ]
        self.assertIn("HS_timeout", stats)
        final_rows = stats[
            stats.index("/** Format both row variants only after") :
            stats.index("ss << line;")
        ]
        self.assertIn("if (connected)", final_rows)
        self.assertIn("} else {", final_rows)
        # The final connected and disconnected render paths must both use the
        # same live per-process timeout snapshot as the footer.
        self.assertEqual(final_rows.count("ls.handshake_timeout_count"), 2)

    def test_dashboard_contextualizes_timeout_with_sdk_event_breakdown(self):
        self.assertIn('"    sdk-events(proc): ack="', DRIVER)
        self.assertIn('<< ", timeout=" << ls.handshake_timeout_count', DRIVER)
        self.assertIn('<< ", rejected=" << ls.handshake_rejected_count', DRIVER)
        self.assertIn('<< ", network=" << ls.handshake_network_error_count', DRIVER)
        self.assertIn('<< ", protocol=" << ls.handshake_protocol_error_count', DRIVER)
        self.assertIn('return "ACK(DeviceInfo pending)";', DRIVER)
        self.assertNotIn('return "SUCCESS(DeviceInfo pending)";', DRIVER)
        self.assertIn('return "ACK_DEVICEINFO_PENDING";', CPP)
        self.assertNotIn('return "SUCCESS_DEVICEINFO_PENDING";', CPP)


if __name__ == "__main__":
    unittest.main()
