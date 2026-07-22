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
        publish = DRIVER[
            DRIVER.index("const int64_t expected_power_episode") :
            DRIVER.index("/** Build every user-visible status")
        ]
        self.assertIn("link_stat_lock_[h]", publish)
        self.assertIn(
            "live.broadcast_only_since_ns == expected_power_episode", publish
        )
        self.assertIn(
            "live.power_cycle_required_count == expected_power_count", publish
        )
        self.assertIn("ls = live", publish)

        alerts = DRIVER[
            DRIVER.index("if (current_incident)") :
            DRIVER.index("const bool handshake_history")
        ]
        self.assertIn("HandshakeResetPhaseStr(ls.handshake_reset_phase)", alerts)
        self.assertIn("waiting SDK RESET completion", alerts)
        self.assertIn("post-reset observation", alerts)

    def test_dashboard_timeout_is_attempt_history_not_a_current_alarm(self):
        stats = DRIVER[
            DRIVER.index("void StatsTimerCb(") :
            DRIVER.index("int main(")
        ]
        self.assertIn("HS60", stats)
        self.assertIn(
            "dashboard_counters.handshake_timeout_attempts =", stats
        )
        self.assertIn("ls.handshake_timeout_count", stats)
        self.assertIn("handshake attempts (SDK): ACK=", stats)
        self.assertIn(
            "PROCESS HISTORY (Driver process; resets on restart; not current alarms)",
            stats,
        )

        current_classification = stats[
            stats.index("const bool handshake_incident") :
            stats.index("const DashboardTrend trend")
        ]
        self.assertNotIn("handshake_timeout_count", current_classification)

    def test_dashboard_active_alerts_use_current_handshake_state(self):
        stats = DRIVER[
            DRIVER.index("void StatsTimerCb(") :
            DRIVER.index("int main(")
        ]
        self.assertIn('" | ATTENTION: ACTIVE="', stats)
        self.assertIn('" | TRANSITION: RECOVERING="', stats)
        self.assertIn('" | OK: STABLE="', stats)
        current_classification = stats[
            stats.index("const bool handshake_incident") :
            stats.index("const DashboardTrend trend")
        ]
        self.assertIn('display_state == "HANDSHAKE_STUCK"', current_classification)
        self.assertIn(
            'display_state == "POWER_CYCLE_REQUIRED"', current_classification
        )
        self.assertIn(
            '(dashboard_connected && health_tags != "OK")',
            current_classification,
        )

        alerts = stats[
            stats.index("if (current_incident)") :
            stats.index("const bool handshake_history")
        ]
        self.assertIn('display_state == "POWER_CYCLE_REQUIRED"', alerts)
        self.assertIn("[CRIT]", alerts)
        self.assertIn("[ALERT]", alerts)
        self.assertRegex(
            alerts,
            re.compile(
                r'"; power-cycle request published; see POWER "\s*'
                r'"RECOVERY manager"'
            ),
        )

    def test_dashboard_power_saving_is_intentionally_idle(self):
        stats = DRIVER[
            DRIVER.index("void StatsTimerCb(") :
            DRIVER.index("int main(")
        ]
        current_classification = stats[
            stats.index("const bool handshake_incident") :
            stats.index("const DashboardTrend trend")
        ]
        current_incident = current_classification[
            current_classification.index("const bool current_incident") :
            current_classification.index("live_signals.recovery_active")
        ]
        self.assertNotIn("POWER_SAVING", current_incident)
        self.assertNotIn("STANDBY", current_incident)
        self.assertIn(
            'display_state == "POWER_SAVING" || display_state == "STANDBY"',
            current_classification,
        )

    def test_dashboard_separates_unique_power_episodes_from_request_edges(self):
        stats = DRIVER[
            DRIVER.index("void StatsTimerCb(") :
            DRIVER.index("int main(")
        ]
        self.assertIn(
            "dashboard_counters.handshake_stuck_episodes = ls.handshake_stuck_count",
            stats,
        )
        self.assertIn(
            "dashboard_counters.power_reached_episodes =", stats
        )
        self.assertIn("ls.power_cycle_required_episode_count", stats)
        self.assertIn("dashboard_counters.power_request_edges =", stats)
        self.assertIn("ls.power_cycle_required_count", stats)
        self.assertIn('<< "    handshake failure episodes: stuck="', stats)
        self.assertIn('<< "; escalated-to-power="', stats)
        self.assertIn('<< " (subset of stuck)\\n"', stats)
        self.assertIn('<< "    POWER_CYCLE_REQUIRED entries="', stats)
        self.assertIn("may repeat within one episode", stats)
        self.assertIn('<< "    session reset actions: accepted="', stats)

    def test_unique_power_episode_latch_survives_network_cancellation(self):
        self.assertIn("power_cycle_required_episode_count", HEADER)
        self.assertIn("power_cycle_required_counted_this_episode", HEADER)
        commit = CPP[
            CPP.index("if (power_cycle_candidate)") :
            CPP.index("if (!request_reset)")
        ]
        self.assertIn("s.power_cycle_required_count++", commit)
        self.assertIn(
            "if (!s.power_cycle_required_counted_this_episode)", commit
        )
        self.assertIn("s.power_cycle_required_episode_count++", commit)
        cancellation = CPP[
            CPP.index("if (status->event == kDeviceHandshakeNetworkError") :
            CPP.index("if (status->event == kDeviceHandshakeReset")
        ]
        self.assertNotIn(
            "power_cycle_required_counted_this_episode = false", cancellation
        )

    def test_handle_reuse_resets_link_history_and_dashboard_local_state(self):
        broadcast = CPP[
            CPP.index("void LdsLidar::OnLidarBroadcastEvent") :
            CPP.index("void LdsLidar::TickHandshakeRecovery")
        ]
        self.assertIn("const bool identity_changed", broadcast)
        self.assertIn("s = LinkStat();", broadcast)
        stats = DRIVER[DRIVER.index("void StatsTimerCb(") : DRIVER.index("int main(")]
        self.assertIn("const bool identity_changed", stats)
        self.assertIn("dashboard_metrics[h].Reset()", stats)
        self.assertIn("emitted_power_cycle_count[h] = 0", stats)

    def test_dashboard_uses_generation_and_cross_snapshot_connection_gate(self):
        stats = DRIVER[DRIVER.index("void StatsTimerCb(") : DRIVER.index("int main(")]
        self.assertIn("GetConnectionGeneration(h)", stats)
        self.assertIn("connection_generation != prev_connection_generation[h]", stats)
        self.assertIn("const bool watchdog_identity_matches", stats)
        self.assertIn("const bool watchdog_connected", stats)
        self.assertIn("if (watchdog_connected)", stats)
        self.assertIn("const bool dashboard_connected", stats)
        self.assertIn("sdk_connected && ls.connect_since_ns != 0 && identity_matches", stats)
        self.assertIn(
            "dashboard_bcode, connection_generation, now_ns, dashboard_counters",
            stats,
        )

    def test_handle_reuse_rejects_stale_callbacks_and_pending_commands(self):
        handshake = CPP[
            CPP.index("void LdsLidar::OnDeviceHandshake") :
            CPP.index("void LdsLidar::OnDeviceBroadcast")
        ]
        self.assertIn("const bool status_has_identity", handshake)
        self.assertIn("strncmp(s.broadcast_code, status->broadcast_code", handshake)

        reset_result = CPP[
            CPP.index("livox_status status = ResetLidarHandshakeSession") :
            CPP.index("char detail[80]", CPP.index("livox_status status = ResetLidarHandshakeSession"))
        ]
        self.assertIn("const bool same_identity", reset_result)
        self.assertIn("if (same_identity)", reset_result)

        remember = CPP[
            CPP.index("void LdsLidar::RememberBroadcastCode") :
            CPP.index("bool LdsLidar::ResetModeRequestIfTarget")
        ]
        self.assertLess(
            remember.index("lock_guard<mutex> send_lock(mode_send_mutex_[handle])"),
            remember.index("lock_guard<mutex> lock(mode_mutex_)"),
        )
        self.assertIn("request = ModeChangeRequest();", remember)

        health = CPP[
            CPP.index("void LdsLidar::LidarErrorStatusCb") :
            CPP.index("void LdsLidar::ControlFanCb")
        ]
        self.assertIn("connection_generation", health)
        self.assertIn("link.health_temp_seen", health)
        self.assertNotIn("static uint8_t prev_temp", health)
        self.assertNotIn("static bool prev_fault", health)

    def test_mode_failure_history_labels_total_and_last_mode(self):
        stats = DRIVER[DRIVER.index("void StatsTimerCb(") : DRIVER.index("int main(")]
        self.assertIn('"    mode failures: total="', stats)
        self.assertIn('<< "; last-mode="', stats)

    def test_error_outranks_config_and_exhausted_config_is_active(self):
        stats = DRIVER[DRIVER.index("void StatsTimerCb(") : DRIVER.index("int main(")]
        self.assertIn("row_state = info.state == kLidarStateError", stats)
        self.assertIn("const bool config_exhausted", stats)
        current = stats[
            stats.index("const bool current_incident") :
            stats.index("const DashboardTrend trend")
        ]
        self.assertIn("handshake_incident || config_exhausted", current)
        self.assertIn('display_state == "CONFIG" && !config_exhausted', current)

    def test_process_history_keeps_last_sdk_event_after_reconnect(self):
        stats = DRIVER[DRIVER.index("void StatsTimerCb(") : DRIVER.index("int main(")]
        history = stats[
            stats.index("const bool handshake_history") :
            stats.index("if (ls.fault_count != 0)")
        ]
        self.assertIn("if (ls.last_handshake_event_wall_s != 0)", history)
        self.assertIn("FmtWall(ls.last_handshake_event_wall_s)", history)

    def test_dashboard_contextualizes_timeout_with_sdk_event_breakdown(self):
        self.assertIn('"    handshake attempts (SDK): ACK="', DRIVER)
        self.assertIn('<< " timeout=" << ls.handshake_timeout_count', DRIVER)
        self.assertIn('<< " rejected=" << ls.handshake_rejected_count', DRIVER)
        self.assertIn('<< "      network="', DRIVER)
        self.assertIn("<< ls.handshake_network_error_count", DRIVER)
        self.assertIn('<< " protocol=" << ls.handshake_protocol_error_count', DRIVER)
        self.assertIn('return "ACK(DeviceInfo pending)";', DRIVER)
        self.assertNotIn('return "SUCCESS(DeviceInfo pending)";', DRIVER)
        self.assertIn('return "ACK_DEVICEINFO_PENDING";', CPP)
        self.assertNotIn('return "SUCCESS_DEVICEINFO_PENDING";', CPP)

    def test_dashboard_network_error_is_diagnostic_not_power_instruction(self):
        self.assertIn('return "NETWORK_ERROR";', DRIVER)
        stats = DRIVER[
            DRIVER.index("void StatsTimerCb(") :
            DRIVER.index("int main(")
        ]
        alerts = stats[
            stats.index("if (current_incident)") :
            stats.index("const bool handshake_history")
        ]
        self.assertIn(
            'const bool critical =\n          display_state == "POWER_CYCLE_REQUIRED";',
            alerts,
        )
        self.assertNotIn("kDeviceHandshakeNetworkError", alerts)
        self.assertIn("last SDK event:", alerts)


if __name__ == "__main__":
    unittest.main()
