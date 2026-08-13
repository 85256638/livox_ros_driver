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
LDDC = (
    ROOT / "livox_ros_driver" / "livox_ros_driver" / "lddc.cpp"
).read_text(encoding="utf-8")
HEALTH_LOGGER = (
    ROOT / "livox_ros_driver" / "livox_ros_driver" / "health_logger.h"
).read_text(encoding="utf-8")
POINT_OUTAGE_POLICY = (
    ROOT
    / "livox_ros_driver"
    / "livox_ros_driver"
    / "point_cloud_outage_policy.h"
).read_text(encoding="utf-8")
STARTUP_MISSING_POLICY = (
    ROOT
    / "livox_ros_driver"
    / "livox_ros_driver"
    / "startup_missing_policy.h"
).read_text(encoding="utf-8")
WAKE_POLICY = (
    ROOT / "livox_ros_driver" / "livox_ros_driver" / "wake_dropout_policy.h"
).read_text(encoding="utf-8")
CMAKE = (ROOT / "livox_ros_driver" / "CMakeLists.txt").read_text(
    encoding="utf-8"
)


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
            DRIVER.index("if (current_incident || point_data_verifying)") :
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
        self.assertIn("RECENT 60 SECONDS", stats)
        self.assertIn("==================== CURRENT DEVICES", stats)
        self.assertIn("==================== ASSESSMENT GUIDE", stats)
        self.assertIn('"connected", "disc"', stats)
        self.assertIn("ls.disconnect_count", stats)
        self.assertIn("handshake_timeouts", stats)
        self.assertIn("not independent fault", stats)
        self.assertIn(
            "dashboard_counters.handshake_timeout_attempts =", stats
        )
        self.assertIn("ls.handshake_timeout_count", stats)
        self.assertIn("handshake attempts (SDK): ACK=", stats)
        self.assertIn(
            "==================== PROCESS HISTORY",
            stats,
        )

        current_classification = stats[
            stats.index("const bool handshake_incident") :
            stats.index("const DashboardTrend trend")
        ]
        self.assertNotIn("handshake_timeout_count", current_classification)

    def test_dashboard_identifies_the_running_binary_build_pair(self):
        self.assertIn("LIVOX_DRIVER_GIT_COMMIT", CMAKE)
        self.assertIn("LIVOX_SDK_GIT_COMMIT", CMAKE)
        self.assertIn("target_compile_definitions", CMAKE)
        self.assertIn("==================== SOFTWARE", DRIVER)
        self.assertIn("versions embedded in this running binary", DRIVER)
        self.assertIn("paired SDK commit=", DRIVER)
        self.assertNotIn("git -C", DRIVER)

    def test_dashboard_rows_and_active_alerts_use_current_handshake_state(self):
        stats = DRIVER[
            DRIVER.index("void StatsTimerCb(") :
            DRIVER.index("int main(")
        ]
        self.assertNotIn("==================== FLEET", stats)
        self.assertIn("==================== CURRENT ALERTS", stats)
        self.assertIn("==================== CURRENT DEVICES", stats)
        self.assertIn("EvaluateTrend(window, live_signals)", stats)
        current_classification = stats[
            stats.index("const bool power_reason_handshake") :
            stats.index("const DashboardTrend trend")
        ]
        self.assertIn('display_state == "HANDSHAKE_STUCK"', current_classification)
        self.assertIn(
            'display_state == "POWER_CYCLE_REQUIRED"', current_classification
        )
        self.assertIn("power_reason_handshake", current_classification)
        self.assertIn("power_reason_wake", current_classification)
        self.assertIn(
            '(dashboard_connected && health_tags != "OK")',
            current_classification,
        )

        alerts = stats[
            stats.index("if (current_incident || point_data_verifying)") :
            stats.index("const bool handshake_history")
        ]
        self.assertIn('display_state == "POWER_CYCLE_REQUIRED"', alerts)
        self.assertIn("[CRIT]", alerts)
        self.assertIn("[ALERT]", alerts)
        self.assertIn('<< " reason="', alerts)
        self.assertIn("PowerCycleReasonStr(ls.power_cycle_reason)", alerts)
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
        self.assertIn('<< "    POWER_CYCLE_REQUIRED: episodes="', stats)
        self.assertIn('<< "; entries=" << ls.power_cycle_required_count', stats)
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

    def test_point_cloud_outage_uses_exact_publish_timeline(self):
        self.assertIn("RecordPointCloudPublished(handle)", LDDC)
        self.assertEqual(LDDC.count("RecordPointCloudPublished(handle)"), 3)
        self.assertIn("ObservePointCloudPublished", CPP)
        self.assertIn("BeginPointCloudOutage", CPP)
        self.assertIn("ExcludePointCloudOutageForPlannedMode", CPP)
        self.assertIn("recovery_first_publish_ns", POINT_OUTAGE_POLICY)
        self.assertIn(
            "recovery_confirm_ns = 3000000000LL", POINT_OUTAGE_POLICY
        )
        self.assertIn(
            "result.duration_ns = state->recovery_first_publish_ns",
            POINT_OUTAGE_POLICY,
        )

    def test_point_cloud_recovery_is_visible_and_persisted(self):
        stats = DRIVER[DRIVER.index("void StatsTimerCb(") : DRIVER.index("int main(")]
        self.assertIn("==================== MEASUREMENT RECOVERY", stats)
        self.assertIn("MEASUREMENT SESSION:", stats)
        self.assertIn("ERROR REBOOTS:", stats)
        self.assertIn("LAST RECOVERY:", stats)
        self.assertIn("NEXT ESCALATION:", stats)
        self.assertIn('<< "    point-cloud outages="', stats)
        self.assertIn('<< "      first data returned="', stats)
        self.assertIn("LogPointCloudRecovery", stats)
        self.assertIn('"POINTCLOUD_RECOVERED"', HEALTH_LOGGER)
        self.assertIn("duration ends at first data", stats)

    def test_error_outranks_config_and_exhausted_config_is_active(self):
        stats = DRIVER[DRIVER.index("void StatsTimerCb(") : DRIVER.index("int main(")]
        self.assertIn("row_state = info.state == kLidarStateError", stats)
        self.assertIn("const bool config_exhausted", stats)
        current = stats[
            stats.index("const bool current_incident") :
            stats.index("const DashboardTrend trend")
        ]
        self.assertIn(
            "handshake_incident || wake_incident || normal_dropout_incident",
            current,
        )
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
            stats.index("if (current_incident || point_data_verifying)") :
            stats.index("const bool handshake_history")
        ]
        self.assertIn('display_state == "POWER_CYCLE_REQUIRED" ||', alerts)
        self.assertIn('display_state == "STARTUP_MISSING"', alerts)
        self.assertNotIn("kDeviceHandshakeNetworkError", alerts)
        self.assertIn("last SDK event:", alerts)

    def test_broadcast_normal_wake_is_staggered_without_blocking_spinner(self):
        service = DRIVER[
            DRIVER.index("bool LidarModeServiceCb(") :
            DRIVER.index("bool LidarRebootServiceCb(")
        ]
        self.assertIn("kBroadcastNormalStaggerMs = 2000", DRIVER)
        self.assertIn("normal_wake_index++ * kBroadcastNormalStaggerMs", service)
        self.assertIn(
            "h, static_cast<LidarMode>(req.mode), delay_ms", service
        )
        self.assertNotIn("sleep_for", service)
        self.assertNotIn("ros::Duration", service)
        self.assertIn("send_not_before_ns", HEADER)

    def test_positive_normal_ack_has_fixed_grace_and_bounded_slow_probes(self):
        self.assertIn(
            "kNormalSpinupGraceNs = 20LL * 1000000000LL", CPP
        )
        self.assertIn(
            "kNormalPostGraceRetryIntervalNs = 5LL * 1000000000LL", CPP
        )
        self.assertIn("kNormalPostGraceMaxRetries = 2", CPP)
        self.assertIn("normal_spinup_grace_deadline_ns", HEADER)
        self.assertIn("normal_post_grace_retry_count", HEADER)

        tick = CPP[
            CPP.index("void LdsLidar::TickSleepModeVerification") :
            CPP.index("void LdsLidar::MarkModeRequestDisconnected")
        ]
        self.assertIn("req.command_id == 0", tick)
        self.assertIn("!req.command_inflight", tick)
        self.assertIn("now >= req.normal_spinup_grace_deadline_ns", tick)
        self.assertIn("kNormalPostGraceRetryIntervalNs", tick)
        self.assertIn("kNormalPostGraceMaxRetries", tick)

        callback = CPP[
            CPP.index("void LdsLidar::SetModeCb") :
            CPP.index("void LdsLidar::RebootCb")
        ]
        self.assertIn("response == 0 || response == 2", callback)
        self.assertIn(
            "if (request.normal_spinup_grace_deadline_ns == 0)", callback
        )
        self.assertIn("kNormalSpinupGraceNs", callback)
        self.assertIn("original spin-up", callback)
        self.assertIn("request.request_id != context->mode_request_id", callback)
        self.assertIn("request.command_id != context->mode_command_id", callback)
        self.assertIn("context->connection_generation", callback)

    def test_config_commands_form_one_serial_per_handle_pipeline(self):
        pipeline = CPP[
            CPP.index("livox_status LdsLidar::SendNextConfigCommand") :
            CPP.index("livox_status LdsLidar::SendCoordinateConfig")
        ]
        ordered = [
            "pending_bits & kConfigCoordinate",
            "pending_bits & kConfigReturnMode",
            "pending_bits & kConfigImuRate",
            "pending_bits & kConfigGetExtrinsicParameter",
            "pending_bits & kConfigSetHighSensitivity",
        ]
        self.assertEqual(sorted(pipeline.index(item) for item in ordered),
                         [pipeline.index(item) for item in ordered])

        state_change = CPP[
            CPP.index("void LdsLidar::OnDeviceChange") :
            CPP.index("void LdsLidar::DeviceInformationCb")
        ]
        self.assertIn("SendNextConfigCommand(handle, config_generation)",
                      state_change)
        for parallel_send in (
            "SendCoordinateConfig(handle, 0, config_generation)",
            "SendReturnModeConfig(handle, 0, config_generation)",
            "SendImuRateConfig(handle, 0, config_generation)",
            "SendExtrinsicConfig(handle, 0, config_generation)",
            "SendHighSensitivityConfig(handle, 0, config_generation)",
        ):
            self.assertNotIn(parallel_send, state_change)

        completion = CPP[
            CPP.index("void LdsLidar::CompleteConfigCommand") :
            CPP.index("void LdsLidar::SetModeCb")
        ]
        self.assertIn("SendNextConfigCommand(handle, connection_generation)",
                      completion)
        self.assertIn("kConfigCommandMaxRetries = 2", CPP)

    def test_wake_dropout_arms_only_for_explicit_low_power_wake(self):
        send = CPP[
            CPP.index("livox_status LdsLidar::SendModeChangeRequest") :
            CPP.index("void LdsLidar::MaybeRetryPendingModeRequest")
        ]
        self.assertIn("request.explicit_wake_source =", send)
        self.assertIn("request.explicit_wake_generation =", send)
        self.assertIn(
            "request.explicit_wake_generation == connection_generation", send
        )
        self.assertIn("same_session_low_power_wake", send)
        self.assertIn("actual_state == kLidarStatePowerSaving", send)
        self.assertIn("actual_state == kLidarStateStandBy", send)
        self.assertIn("mode == kLidarModeNormal", send)
        self.assertIn("ArmWakeObservation(handle, live_broadcast_code", send)
        self.assertIn("CancelWakeObservation(handle)", send)
        self.assertNotIn("actual_state == kLidarStateNormal ||", send)

    def test_staggered_wake_never_transfers_low_power_fact_to_new_generation(self):
        send = CPP[
            CPP.index("livox_status LdsLidar::SendModeChangeRequest") :
            CPP.index("void LdsLidar::MaybeRetryPendingModeRequest")
        ]
        self.assertIn(
            "request.explicit_wake_generation == connection_generation", send
        )
        self.assertIn("!same_session_low_power_wake", send)
        self.assertIn("request.explicit_wake_source = false", send)
        self.assertIn("request.explicit_wake_generation = 0", send)
        self.assertLess(
            send.index("same_session_low_power_wake"),
            send.index("ArmWakeObservation(handle, live_broadcast_code"),
        )

    def test_wake_dropout_has_identity_window_and_double_checked_escalation(self):
        self.assertRegex(
            HEADER,
            re.compile(r"WakeObservationNs\(\)\s*\{\s*return 60000000000LL;"),
        )
        self.assertRegex(
            HEADER,
            re.compile(r"WakeDropoutConfirmNs\(\)\s*\{\s*return 10000000000LL;"),
        )
        for evidence in (
            "wake_request_id",
            "wake_connection_generation",
            "wake_dropout_generation",
            "wake_broadcast_code",
            "wake_dropout_since_ns",
        ):
            self.assertIn(evidence, HEADER)
        tick = CPP[
            CPP.index("void LdsLidar::TickWakeDropoutRecovery") :
            CPP.index("void LdsLidar::TickHandshakeRecovery")
        ]
        self.assertGreaterEqual(tick.count("WakeDropoutEscalationReady"), 2)
        self.assertIn("s.wake_request_id == expected_request_id", tick)
        self.assertIn("s.wake_dropout_since_ns == expected_dropout_since", tick)
        self.assertIn("ResetModeRequestIfTarget(handle, kLidarModeNormal", tick)
        self.assertIn("input.identity_matches", WAKE_POLICY)
        self.assertIn("input.generation_matches", WAKE_POLICY)
        self.assertIn("!input.broadcast_fresh", WAKE_POLICY)
        self.assertIn(
            "input.attributed_disconnect_ns <= input.wake_deadline_ns",
            WAKE_POLICY,
        )
        self.assertIn(
            "input.dropout_since_ns >= input.attributed_disconnect_ns",
            WAKE_POLICY,
        )

    def test_connect_clears_wake_and_transient_broadcast_keeps_attribution(self):
        connect = CPP[
            CPP.index("void LdsLidar::OnLidarConnectEvent") :
            CPP.index("void LdsLidar::OnLidarDisconnectEvent")
        ]
        broadcast = CPP[
            CPP.index("void LdsLidar::OnLidarBroadcastEvent") :
            CPP.index("void LdsLidar::ArmWakeObservation")
        ]
        self.assertIn("ClearWakeRecoveryState(&s)", connect)
        self.assertIn("s.wake_attributed_disconnect_ns != 0", broadcast)
        self.assertIn("s.wake_state != kWakeRecoveryIdle", broadcast)
        self.assertIn("s.wake_state = kWakeRecoveryObserving", broadcast)
        self.assertIn("s.wake_dropout_since_ns = 0", broadcast)
        self.assertIn("awaiting stable broadcast handoff", broadcast)
        self.assertIn("kWakeBroadcastHandoffMinFrames", broadcast)
        self.assertIn("kWakeBroadcastHandoffNs", broadcast)

        tick = CPP[
            CPP.index("void LdsLidar::TickWakeDropoutRecovery") :
            CPP.index("void LdsLidar::TickHandshakeRecovery")
        ]
        self.assertIn("s.wake_attributed_disconnect_ns == 0", tick)
        self.assertIn("WakeDropoutAttributionValid(input)", tick)
        self.assertIn("!input.broadcast_fresh", tick)
        self.assertIn("s.wake_state = kWakeRecoveryNoBroadcast", tick)
        self.assertIn("s.wake_dropout_since_ns = now", tick)

    def test_wake_disconnect_identity_is_fail_closed(self):
        disconnect = CPP[
            CPP.index("void LdsLidar::OnLidarDisconnectEvent") :
            CPP.index("void LdsLidar::OnLidarBroadcastEvent")
        ]
        self.assertIn("broadcast_code != nullptr", disconnect)
        self.assertIn("broadcast_code[0] != '\\0'", disconnect)
        self.assertIn("callback_identity_matches", disconnect)
        self.assertIn(
            "strncmp(s.broadcast_code, broadcast_code", disconnect
        )
        self.assertIn(
            "strncmp(s.wake_broadcast_code, s.broadcast_code", disconnect
        )
        self.assertNotIn("broadcast_code == nullptr ||", disconnect)
        self.assertIn("s.wake_dropout_generation = current_generation", disconnect)

    def test_deliberate_soft_reboot_uses_planned_marker_and_preserves_rejection(self):
        reboot = CPP[
            CPP.index("livox_status LdsLidar::RequestLidarRebootImpl") :
            CPP.index("livox_status LdsLidar::RequestRestartSampling")
        ]
        self.assertIn("planned_reboot_generation = reboot_generation", reboot)
        self.assertIn("RebootDevice(handle", reboot)
        self.assertIn("if (status == kStatusSuccess)", reboot)
        self.assertIn("ClearWakeRecoveryState(&s)", reboot)
        self.assertLess(
            reboot.index("planned_reboot_generation = reboot_generation"),
            reboot.index("RebootDevice(handle"),
        )
        disconnect = CPP[
            CPP.index("void LdsLidar::OnLidarDisconnectEvent") :
            CPP.index("void LdsLidar::OnLidarBroadcastEvent")
        ]
        self.assertIn("planned_reboot_disconnect", disconnect)
        self.assertIn("ClearWakeRecoveryState(&s)", disconnect)

    def test_handshake_power_commit_discards_observing_wake_evidence(self):
        commit = CPP[
            CPP.index("if (power_cycle_candidate)") :
            CPP.index("if (!request_reset)")
        ]
        self.assertIn("ClearWakeRecoveryState(&s)", commit)
        self.assertLess(
            commit.index("ClearWakeRecoveryState(&s)"),
            commit.index("s.power_cycle_reason = kPowerCycleReasonHandshakeStuck"),
        )

    def test_wire_and_dashboard_keep_wake_separate_from_handshake(self):
        self.assertIn('return "WAKE_NO_BROADCAST";', DRIVER)
        self.assertIn('return "WAKE_DROPOUT";', DRIVER)
        self.assertIn('return "HANDSHAKE_STUCK";', DRIVER)
        self.assertIn("PowerCycleReasonStr(link.power_cycle_reason)", DRIVER)
        self.assertIn("WakeStateStr(link.wake_state)", DRIVER)
        self.assertIn("dashboard_counters.wake_dropout_episodes", DRIVER)
        self.assertIn('<< "    wake-dropout episodes="', DRIVER)
        self.assertIn('<< "    handshake failure episodes: stuck="', DRIVER)
        self.assertIn("ls.handshake_power_cycle_episode_count", DRIVER)

    def test_normal_dropout_has_sustained_health_and_silence_boundaries(self):
        self.assertRegex(
            HEADER,
            re.compile(r"NormalHealthyArmNs\(\)\s*\{\s*return 30000000000LL;"),
        )
        self.assertRegex(
            HEADER,
            re.compile(r"NormalDropoutConfirmNs\(\)\s*\{\s*return 5000000000LL;"),
        )
        tick = CPP[
            CPP.index("void LdsLidar::TickNormalDropoutRecovery") :
            CPP.index("void LdsLidar::TickWakeDropoutRecovery")
        ]
        self.assertGreaterEqual(tick.count("NormalDropoutEscalationReady"), 2)
        self.assertIn("expected_generation", tick)
        self.assertIn("expected_dropout_since", tick)
        self.assertIn("kPowerCycleReasonNormalDropout", tick)
        self.assertIn("normal_power_cycle_episode_count", tick)

    def test_normal_dropout_is_cancelled_by_connect_and_planned_actions(self):
        connect = CPP[
            CPP.index("void LdsLidar::OnLidarConnectEvent") :
            CPP.index("void LdsLidar::OnLidarDisconnectEvent")
        ]
        self.assertIn("ClearNormalDropoutState(&s)", connect)
        disconnect = CPP[
            CPP.index("void LdsLidar::OnLidarDisconnectEvent") :
            CPP.index("void LdsLidar::OnLidarBroadcastEvent")
        ]
        planned = disconnect[
            disconnect.index("if (planned_reboot_disconnect)") :
            disconnect.index("else if (s.wake_state")
        ]
        self.assertIn("ClearNormalDropoutState(&s)", planned)
        send = CPP[
            CPP.index("livox_status LdsLidar::SendModeChangeRequest") :
            CPP.index("void LdsLidar::MaybeRetryPendingModeRequest")
        ]
        self.assertIn("ObserveNormalPublishing(handle, false, 0, nullptr)", send)

    def test_normal_dropout_transient_broadcast_requires_stable_handoff(self):
        broadcast = CPP[
            CPP.index("void LdsLidar::OnLidarBroadcastEvent") :
            CPP.index("void LdsLidar::ArmWakeObservation")
        ]
        self.assertIn("normal_broadcast_return_count", broadcast)
        self.assertIn("kWakeBroadcastHandoffMinFrames", broadcast)
        self.assertIn("kWakeBroadcastHandoffNs", broadcast)
        self.assertIn("NORMAL_BROADCAST_STABLE", broadcast)
        tick = CPP[
            CPP.index("void LdsLidar::TickNormalDropoutRecovery") :
            CPP.index("void LdsLidar::TickWakeDropoutRecovery")
        ]
        self.assertIn("broadcast return was transient", tick)
        self.assertIn("s.last_broadcast_ns != 0 ? s.last_broadcast_ns : now", tick)

    def test_normal_power_edge_revalidates_generation_and_silence(self):
        stats = DRIVER[DRIVER.index("void StatsTimerCb(") : DRIVER.index("int main(")]
        publish = stats[
            stats.index("const int64_t expected_power_episode") :
            stats.index("/** Build every user-visible status")
        ]
        self.assertIn("expected_normal_generation", publish)
        self.assertIn("expected_normal_silence", publish)
        self.assertIn("live.normal_dropout_generation", publish)
        self.assertIn("live.normal_dropout_since_ns", publish)

    def test_startup_missing_uses_whitelist_grace_and_synthetic_live_state(self):
        stats = DRIVER[DRIVER.index("void StatsTimerCb(") : DRIVER.index("int main(")]
        self.assertIn("GetWhitelistBroadcastCodes()", stats)
        self.assertIn("kStartupMissingGraceNs = 30000000000LL", DRIVER)
        self.assertIn("PublishStartupRecoveryState", stats)
        self.assertIn("PublishStartupPowerCycleRequest", stats)
        self.assertIn('"STARTUP_MISSING"', stats)
        startup_publish = DRIVER[
            DRIVER.index("static void PublishStartupRecoveryState") :
            DRIVER.index("static const char *TempStr")
        ]
        self.assertIn("g_driver_instance_id, 255", startup_publish)
        self.assertIn('"STARTUP_MISSING"', startup_publish)
        self.assertIn("tracker.ever_observed", stats)
        self.assertIn("startup_present[i]", stats)
        self.assertNotIn("ever_healthy", stats)
        self.assertIn("if (observed_now)", STARTUP_MISSING_POLICY)
        self.assertIn("state->ever_observed = true", STARTUP_MISSING_POLICY)
        self.assertIn("if (state->ever_observed)", STARTUP_MISSING_POLICY)
        self.assertIn("planned_group_power_cycle", STARTUP_MISSING_POLICY)
        self.assertIn("IsPlannedGroupPowerCycleActive", HEADER)


if __name__ == "__main__":
    unittest.main()
