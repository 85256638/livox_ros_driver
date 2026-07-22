#include "../livox_ros_driver/livox_ros_driver/dashboard_metrics.h"

#include <cmath>
#include <iostream>
#include <string>

namespace {

const int64_t kSecond = INT64_C(1000000000);

bool Check(bool condition, const char *message) {
  if (!condition) {
    std::cerr << "dashboard_metrics_qc: " << message << "\n";
    return false;
  }
  return true;
}

bool Near(double actual, double expected) {
  return std::fabs(actual - expected) < 1e-9;
}

bool TestWindowsAndCounterReset() {
  using livox_ros::DashboardCounters;
  using livox_ros::DashboardMetrics;
  using livox_ros::DashboardWindow;

  DashboardMetrics metrics;
  DashboardCounters c;
  c.received_packets = 1000;
  c.lost_packets = 10;
  c.queue_drops = 5;
  c.handshake_ack_attempts = 20;
  c.disconnect_episodes = 7;

  DashboardWindow w = metrics.Update("LIDAR-A", 1, 0, c);
  if (!Check(w.received_60s == 0 && w.disconnect_10m == 0,
             "first cumulative sample must only establish a baseline") ||
      !Check(!w.loss_60s_has_data,
             "zero loss denominator must be reported as no data")) {
    return false;
  }

  c.received_packets += 99;
  c.lost_packets += 1;
  c.queue_drops += 2;
  c.handshake_ack_attempts += 1;
  c.handshake_timeout_attempts += 2;
  c.handshake_rejected_attempts += 3;
  c.handshake_network_attempts += 4;
  c.handshake_protocol_attempts += 5;
  c.disconnect_episodes += 1;
  c.handshake_stuck_episodes += 2;
  c.power_reached_episodes += 3;
  c.power_request_edges += 7;
  c.fault_episodes += 4;
  c.reboot_actions += 5;
  c.mode_fail_episodes += 6;
  w = metrics.Update("LIDAR-A", 1, kSecond, c);
  if (!Check(w.received_60s == 99 && w.lost_60s == 1 &&
                 w.queue_drops_60s == 2,
             "60-second traffic deltas are wrong") ||
      !Check(w.handshake_ack_60s == 1 && w.handshake_timeout_60s == 2 &&
                 w.handshake_rejected_60s == 3 &&
                 w.handshake_network_60s == 4 &&
                 w.handshake_protocol_60s == 5,
             "handshake attempt deltas are wrong") ||
      !Check(w.disconnect_10m == 1 && w.handshake_stuck_10m == 2 &&
                 w.power_reached_10m == 3 &&
                 w.power_request_edges_10m == 7 && w.fault_10m == 4 &&
                 w.reboot_10m == 5 && w.mode_fail_10m == 6,
             "ten-minute episode/action deltas are wrong") ||
      !Check(w.loss_60s_has_data && Near(w.loss_60s_percent, 1.0),
             "loss percent should use received + lost as denominator")) {
    return false;
  }

  /** Simulate every producer counter resetting independently.  New values are
   *  post-reset activity and must not cause unsigned underflow. */
  DashboardCounters reset;
  reset.received_packets = 2;
  reset.lost_packets = 1;
  reset.queue_drops = 1;
  reset.handshake_ack_attempts = 1;
  reset.handshake_timeout_attempts = 1;
  reset.handshake_rejected_attempts = 1;
  reset.handshake_network_attempts = 1;
  reset.handshake_protocol_attempts = 1;
  reset.disconnect_episodes = 1;
  reset.handshake_stuck_episodes = 1;
  reset.power_reached_episodes = 1;
  reset.power_request_edges = 1;
  reset.fault_episodes = 1;
  reset.reboot_actions = 1;
  reset.mode_fail_episodes = 1;
  w = metrics.Update("LIDAR-A", 1, 2 * kSecond, reset);
  return Check(w.received_60s == 101 && w.lost_60s == 2 &&
                   w.queue_drops_60s == 3,
               "counter reset underflow protection failed") &&
         Check(w.handshake_ack_60s == 2 &&
                   w.handshake_timeout_60s == 3 &&
                   w.handshake_rejected_60s == 4 &&
                   w.handshake_network_60s == 5 &&
                   w.handshake_protocol_60s == 6,
               "handshake counter reset protection failed") &&
         Check(w.disconnect_10m == 2 && w.handshake_stuck_10m == 3 &&
                   w.power_reached_10m == 4 &&
                   w.power_request_edges_10m == 8 && w.fault_10m == 5 &&
                   w.reboot_10m == 6 && w.mode_fail_10m == 7,
               "episode/action counter reset protection failed");
}

bool TestConnectionGenerationRebound() {
  using livox_ros::DashboardCounters;
  using livox_ros::DashboardMetrics;
  using livox_ros::DashboardWindow;

  DashboardMetrics metrics;
  DashboardCounters c;
  c.received_packets = 100;
  c.lost_packets = 10;
  c.queue_drops = 5;
  c.handshake_timeout_attempts = 1000;
  c.handshake_stuck_episodes = 20;
  c.power_reached_episodes = 4;
  c.power_request_edges = 9;
  metrics.Update("LIDAR-A", 7, 0, c);

  c.received_packets = 150;
  c.lost_packets = 12;
  c.queue_drops = 8;
  c.handshake_timeout_attempts = 1002;
  c.handshake_stuck_episodes = 21;
  c.power_reached_episodes = 5;
  c.power_request_edges = 11;
  DashboardWindow w = metrics.Update("LIDAR-A", 7, kSecond, c);
  if (!Check(w.received_60s == 50 && w.lost_60s == 2 &&
                 w.queue_drops_60s == 3,
             "same-generation traffic must use ordinary differences") ||
      !Check(w.handshake_timeout_60s == 2 &&
                 w.handshake_stuck_10m == 1 &&
                 w.power_reached_10m == 1 &&
                 w.power_request_edges_10m == 2,
             "same-generation process-counter differences are wrong")) {
    return false;
  }

  /** ResetLidar can clear traffic counters and the replacement connection can
   *  already count past the old values before the next 1 Hz sample.  The
   *  generation edge is the only reliable evidence of that invisible reset:
   *  count all 180/20/9 packets from the new connection, not 30/8/1. */
  c.received_packets = 180;
  c.lost_packets = 20;
  c.queue_drops = 9;
  c.handshake_timeout_attempts = 1005;
  c.handshake_stuck_episodes = 22;
  c.power_reached_episodes = 6;
  c.power_request_edges = 14;
  w = metrics.Update("LIDAR-A", 8, 2 * kSecond, c);
  if (!Check(w.received_60s == 230 && w.lost_60s == 22 &&
                 w.queue_drops_60s == 12,
             "generation rebound lost new-connection traffic") ||
      !Check(w.handshake_timeout_60s == 5 &&
                 w.handshake_stuck_10m == 2 &&
                 w.power_reached_10m == 2 &&
                 w.power_request_edges_10m == 5,
             "generation edge must SafeDelta process-lifetime counters") ||
      !Check(w.observation_ns == static_cast<uint64_t>(2 * kSecond),
             "same-lidar generation edge must retain rolling history")) {
    return false;
  }

  c.received_packets = 200;
  c.lost_packets = 21;
  c.queue_drops = 10;
  c.handshake_timeout_attempts = 1006;
  w = metrics.Update("LIDAR-A", 8, 3 * kSecond, c);
  return Check(w.received_60s == 250 && w.lost_60s == 23 &&
                   w.queue_drops_60s == 13,
               "post-edge sample did not establish the new traffic baseline") &&
         Check(w.handshake_timeout_60s == 6,
               "post-edge process counter baseline is wrong");
}

bool TestCodeReuseClearsHistory() {
  using livox_ros::DashboardCounters;
  using livox_ros::DashboardMetrics;
  using livox_ros::DashboardWindow;

  DashboardMetrics metrics;
  DashboardCounters c;
  metrics.Update("LIDAR-A", 3, 0, c);
  c.lost_packets = 10;
  c.handshake_timeout_attempts = 10;
  c.fault_episodes = 10;
  metrics.Update("LIDAR-A", 3, kSecond, c);

  DashboardCounters replacement;
  replacement.received_packets = 100000;
  replacement.lost_packets = 500;
  replacement.handshake_network_attempts = 30;
  replacement.disconnect_episodes = 40;
  replacement.handshake_stuck_episodes = 23;
  replacement.power_reached_episodes = 6;
  replacement.power_request_edges = 14;
  replacement.fault_episodes = 20;
  DashboardWindow w =
      metrics.Update("LIDAR-B", 99, 2 * kSecond, replacement);
  return Check(w.broadcast_code == "LIDAR-B",
               "snapshot has the wrong replacement broadcast code") &&
         Check(w.observation_ns == 0 && w.received_60s == 0 &&
                   w.lost_60s == 0 && w.handshake_timeout_60s == 0 &&
                   w.handshake_network_60s == 0 &&
                   w.disconnect_10m == 0 &&
                   w.handshake_stuck_10m == 0 &&
                   w.power_reached_10m == 0 &&
                   w.power_request_edges_10m == 0 && w.fault_10m == 0,
               "handle reuse leaked the previous lidar's history");
}

bool TestSteadyTimeCropping() {
  using livox_ros::DashboardCounters;
  using livox_ros::DashboardMetrics;
  using livox_ros::DashboardWindow;

  DashboardMetrics metrics;
  DashboardCounters c;
  metrics.Update("LIDAR-A", 1, 0, c);

  c.received_packets = 100;
  c.disconnect_episodes = 1;
  DashboardWindow w = metrics.Update("LIDAR-A", 1, kSecond, c);
  if (!Check(w.received_60s == 100 && w.disconnect_10m == 1,
             "fresh sample missing from rolling windows")) {
    return false;
  }

  /** The interval is half-open: the t=1 sample expires from 60s at t=61. */
  w = metrics.Update("LIDAR-A", 1, 61 * kSecond, c);
  if (!Check(w.received_60s == 0 && !w.loss_60s_has_data,
             "exactly 60-second-old traffic was not cropped") ||
      !Check(w.disconnect_10m == 1,
             "60-second crop incorrectly removed ten-minute episode")) {
    return false;
  }

  c.received_packets += 50;
  c.fault_episodes += 1;
  w = metrics.Update("LIDAR-A", 1, 550 * kSecond, c);
  if (!Check(w.received_60s == 50 && w.fault_10m == 1 &&
                 w.disconnect_10m == 1,
             "recent traffic or long-window history is wrong")) {
    return false;
  }

  w = metrics.Update("LIDAR-A", 1, 601 * kSecond, c);
  return Check(w.received_60s == 50,
               "51-second-old traffic should remain in 60-second window") &&
         Check(w.disconnect_10m == 0 && w.fault_10m == 1,
               "exactly ten-minute-old episode was not cropped") &&
         Check(w.observation_ns ==
                   static_cast<uint64_t>(DashboardMetrics::kLongWindowNs),
               "observation duration must cap at ten minutes");
}

livox_ros::DashboardWindow MatureWindow() {
  livox_ros::DashboardWindow w;
  w.observation_ns =
      static_cast<uint64_t>(livox_ros::DashboardMetrics::kLongWindowNs);
  return w;
}

bool TestTrendThresholdsAndSleep() {
  using namespace livox_ros;

  DashboardWindow w = MatureWindow();
  DashboardLiveSignals live;
  if (!Check(EvaluateTrend(w, live) == kDashboardTrendStable,
             "mature clean window should be STABLE")) {
    return false;
  }

  w.observation_ns =
      static_cast<uint64_t>(DashboardMetrics::kLongWindowNs) - 1;
  if (!Check(EvaluateTrend(w, live) == kDashboardTrendObserve,
             "short observation should be OBSERVE")) {
    return false;
  }

  w = MatureWindow();
  w.loss_60s_has_data = true;
  w.loss_60s_percent = 0.1;
  if (!Check(EvaluateTrend(w, live) == kDashboardTrendWatch,
             "0.1 percent loss should be WATCH")) {
    return false;
  }
  w.loss_60s_percent = 1.0;
  if (!Check(EvaluateTrend(w, live) == kDashboardTrendUnstable,
             "1 percent loss should be UNSTABLE")) {
    return false;
  }

  /** Each category must independently reach two.  Counts spread across
   *  different categories do not form a same-kind repeat. */
  w = MatureWindow();
  w.handshake_stuck_10m = 1;
  w.power_reached_10m = 1;
  if (!Check(EvaluateTrend(w, live) == kDashboardTrendWatch,
             "different one-off episode kinds must not become UNSTABLE")) {
    return false;
  }

  uint64_t *repeated[] = {&w.handshake_stuck_10m, &w.power_reached_10m,
                          &w.fault_10m, &w.reboot_10m, &w.mode_fail_10m};
  for (size_t i = 0; i < sizeof(repeated) / sizeof(repeated[0]); ++i) {
    DashboardWindow category = MatureWindow();
    if (i == 0) category.handshake_stuck_10m = 2;
    if (i == 1) category.power_reached_10m = 2;
    if (i == 2) category.fault_10m = 2;
    if (i == 3) category.reboot_10m = 2;
    if (i == 4) category.mode_fail_10m = 2;
    if (!Check(EvaluateTrend(category, live) == kDashboardTrendUnstable,
               "same-kind repeated event/action should be UNSTABLE")) {
      return false;
    }
  }

  w = MatureWindow();
  w.disconnect_10m = 20;
  if (!Check(EvaluateTrend(w, live) == kDashboardTrendWatch,
             "disconnect alone must never be UNSTABLE")) {
    return false;
  }
  w = MatureWindow();
  w.power_request_edges_10m = 1000;
  if (!Check(EvaluateTrend(w, live) == kDashboardTrendWatch,
             "power request edges alone must never be UNSTABLE")) {
    return false;
  }
  w = MatureWindow();
  w.handshake_network_60s = 1;
  if (!Check(EvaluateTrend(w, live) == kDashboardTrendWatch,
             "recent handshake error should be WATCH")) {
    return false;
  }
  w = MatureWindow();
  w.queue_drops_60s = 1;
  if (!Check(EvaluateTrend(w, live) == kDashboardTrendWatch,
             "recent queue drop should be WATCH")) {
    return false;
  }

  /** Current live state wins over even severe history.  Sleep/standby maps to
   *  IDLE rather than looking broken merely because it is not publishing. */
  w = MatureWindow();
  w.fault_10m = 99;
  live.intentionally_idle = true;
  if (!Check(EvaluateTrend(w, live) == kDashboardTrendIdle &&
                 std::string(DashboardTrendName(kDashboardTrendIdle)) ==
                     "IDLE",
             "sleep/standby must override history as IDLE")) {
    return false;
  }
  live.recovery_active = true;
  if (!Check(EvaluateTrend(w, live) == kDashboardTrendRecovering,
             "RECOVERING must take precedence over IDLE")) {
    return false;
  }
  live.incident_active = true;
  return Check(EvaluateTrend(w, live) == kDashboardTrendActive,
               "ACTIVE must take precedence over RECOVERING and IDLE");
}

bool TestOperationalMeaningContract() {
  using namespace livox_ros;

  /** TIMEOUT is an SDK attempt counter, not an incident counter.  Even a
   *  large value can only make a clean current lidar WATCH; it must not invent
   *  a stuck or hard-power episode. */
  DashboardWindow w = MatureWindow();
  w.handshake_timeout_60s = 498;
  if (!Check(w.HandshakeErrors60s() == 498,
             "timeout attempts must remain an attempt count") ||
      !Check(w.handshake_stuck_10m == 0 &&
                 w.power_reached_10m == 0 &&
                 w.power_request_edges_10m == 0,
             "timeout attempts must not synthesize recovery episodes") ||
      !Check(EvaluateTrend(w, DashboardLiveSignals()) == kDashboardTrendWatch,
             "timeout attempts alone must be WATCH, not UNSTABLE")) {
    return false;
  }

  /** STUCK and POWER_CYCLE_REQUIRED count different episode transitions.
   *  Escalation can recover before hard power, so the totals are intentionally
   *  independent and are not expected to match. */
  w = MatureWindow();
  w.handshake_stuck_10m = 23;
  w.power_reached_10m = 6;
  w.power_request_edges_10m = 17;
  if (!Check(w.handshake_stuck_10m == 23 &&
                 w.power_reached_10m == 6 &&
                 w.power_request_edges_10m == 17 &&
                 w.handshake_stuck_10m != w.power_reached_10m &&
                 w.power_reached_10m != w.power_request_edges_10m,
             "stuck, power-reached and request-edge counts must stay independent") ||
      !Check(EvaluateTrend(w, DashboardLiveSignals()) ==
                 kDashboardTrendUnstable,
             "repeated same-kind episodes should remain UNSTABLE")) {
    return false;
  }

  /** NETWORK_ERROR is evidence of a network/socket failure, not an
   *  instruction to cut power. */
  w = MatureWindow();
  w.handshake_network_60s = 1;
  if (!Check(w.power_reached_10m == 0 &&
                 w.power_request_edges_10m == 0,
             "network errors must not imply hard-power episodes") ||
      !Check(EvaluateTrend(w, DashboardLiveSignals()) == kDashboardTrendWatch,
             "network error alone must be WATCH")) {
    return false;
  }

  /** A shared relay can disconnect every healthy peer while recovering one
   *  member.  Disconnect episodes alone therefore never condemn an
   *  individual lidar as UNSTABLE. */
  w = MatureWindow();
  w.disconnect_10m = 1000;
  if (!Check(EvaluateTrend(w, DashboardLiveSignals()) == kDashboardTrendWatch,
             "shared-relay disconnects alone must not be UNSTABLE")) {
    return false;
  }

  /** The driver maps PowerSaving/StandBy to intentionally_idle.  IDLE must
   *  override stale counters so an intentional low-power state is not shown
   *  as a current fault. */
  w = MatureWindow();
  w.handshake_timeout_60s = 498;
  w.handshake_stuck_10m = 23;
  w.power_reached_10m = 6;
  w.power_request_edges_10m = 17;
  DashboardLiveSignals power_saving;
  power_saving.intentionally_idle = true;
  return Check(EvaluateTrend(w, power_saving) == kDashboardTrendIdle,
               "PowerSaving must remain IDLE despite stale history");
}

}  // namespace

int main() {
  if (!TestWindowsAndCounterReset()) return 1;
  if (!TestConnectionGenerationRebound()) return 2;
  if (!TestCodeReuseClearsHistory()) return 3;
  if (!TestSteadyTimeCropping()) return 4;
  if (!TestTrendThresholdsAndSleep()) return 5;
  if (!TestOperationalMeaningContract()) return 6;
  std::cout << "dashboard_metrics_qc: OK\n";
  return 0;
}
