//
// The MIT License (MIT)
//
// Copyright (c) 2019 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#include "lds_lidar.h"

#include <stdio.h>
#include <string.h>
#include <time.h>
#include <chrono>
#include <algorithm>
#include <memory>
#include <mutex>
#include <set>
#include <sstream>
#include <thread>

#include "health_logger.h"
#include "normal_dropout_policy.h"
#include "rapidjson/document.h"
#include "rapidjson/filereadstream.h"
#include "rapidjson/stringbuffer.h"
#include "wake_dropout_policy.h"

using namespace std;

namespace livox_ros {

namespace {
/** A Horizon normally broadcasts often while it is waiting for a handshake.
 *  A gap longer than this starts a new diagnostic episode, rather than
 *  carrying a stale HANDSHAKE_STUCK decision across a power/network outage. */
const int64_t kBroadcastEpisodeGapNs = LdsLidar::HandshakeBroadcastFreshNs();
/** The SDK continues its normal handshake attempts as broadcasts arrive.
 *  The driver intervenes once at 5s by clearing only the local SDK session,
 *  then escalates no earlier than both episode age 10s and RESET completion
 *  plus 5s.  A late RESET completion therefore moves escalation later. */
const int64_t kHandshakeFirstResetNs = 5000000000LL;
const int64_t kHandshakePowerCycleNs = 10000000000LL;
const int64_t kHandshakePostResetObserveNs = 5000000000LL;
/** A recent local socket/network error is not evidence that cycling lidar
 *  power will help. Require a quiet window before hardware escalation. */
const int64_t kHandshakeNetworkErrorGateNs = 5000000000LL;
const uint8_t kHandshakeResetMaxAttempts =
    LdsLidar::HandshakeResetMaxAttempts();
const int64_t kWakeObservationNs = LdsLidar::WakeObservationNs();
const int64_t kWakeDropoutConfirmNs = LdsLidar::WakeDropoutConfirmNs();
const int64_t kWakeBroadcastHandoffNs =
    LdsLidar::WakeBroadcastHandoffNs();
const uint32_t kWakeBroadcastHandoffMinFrames =
    LdsLidar::WakeBroadcastHandoffMinFrames();
const int64_t kNormalHealthyArmNs = LdsLidar::NormalHealthyArmNs();
const int64_t kNormalDropoutConfirmNs =
    LdsLidar::NormalDropoutConfirmNs();
const int64_t kNetworkSoftRebootAckTimeoutNs = 2000000000LL;
const int64_t kNetworkSoftRebootRetryNs = 5000000000LL;
const int64_t kNetworkSoftRebootSettleNs = 60000000000LL;
const uint8_t kNetworkSoftRebootMaxAttempts = 3;
const int64_t kNetworkHealthStaleNs = 4000000000LL;

int64_t SecondsToNs(double seconds, int64_t fallback) {
  if (!(seconds > 0.0) || seconds > 300.0) {
    return fallback;
  }
  const double ns = seconds * 1000000000.0;
  if (ns < 1.0 || ns > 300000000000LL) {
    return fallback;
  }
  return static_cast<int64_t>(ns);
}

/** Clear only live wake attribution. Process-lifetime episode/action counters
 *  remain available to the dashboard after recovery. */
void ClearWakeRecoveryState(LdsLidar::LinkStat *s) {
  if (s == nullptr) {
    return;
  }
  if (s->power_cycle_reason == LdsLidar::kPowerCycleReasonWakeDropout) {
    s->power_cycle_reason = LdsLidar::kPowerCycleReasonNone;
  }
  s->wake_state = LdsLidar::kWakeRecoveryIdle;
  s->wake_request_id = 0;
  s->wake_connection_generation = 0;
  s->wake_dropout_generation = 0;
  s->wake_started_ns = 0;
  s->wake_deadline_ns = 0;
  s->wake_started_wall_s = 0;
  s->wake_attributed_disconnect_ns = 0;
  s->wake_attributed_disconnect_wall_s = 0;
  s->wake_dropout_since_ns = 0;
  s->wake_dropout_wall_s = 0;
  s->wake_broadcast_return_since_ns = 0;
  s->wake_broadcast_return_count = 0;
  s->wake_dropout_counted_this_request = false;
  s->wake_power_cycle_counted_this_request = false;
  memset(s->wake_broadcast_code, 0, sizeof(s->wake_broadcast_code));
}

/** Clear only live normal-dropout evidence; retain process history. */
void ClearNormalDropoutState(LdsLidar::LinkStat *s) {
  if (s == nullptr) {
    return;
  }
  if (s->power_cycle_reason == LdsLidar::kPowerCycleReasonNormalDropout) {
    s->power_cycle_reason = LdsLidar::kPowerCycleReasonNone;
  }
  s->normal_dropout_state = LdsLidar::kNormalDropoutIdle;
  s->normal_connection_generation = 0;
  s->normal_dropout_generation = 0;
  s->normal_healthy_since_ns = 0;
  s->normal_healthy_since_wall_s = 0;
  s->normal_attributed_disconnect_ns = 0;
  s->normal_attributed_disconnect_wall_s = 0;
  s->normal_dropout_since_ns = 0;
  s->normal_dropout_wall_s = 0;
  s->normal_broadcast_return_since_ns = 0;
  s->normal_broadcast_return_count = 0;
  s->normal_dropout_counted_this_episode = false;
  s->normal_power_cycle_counted_this_episode = false;
  memset(s->normal_broadcast_code, 0, sizeof(s->normal_broadcast_code));
}

/** A shared physical power cycle starts a fresh network baseline.  Do not
 * carry the pre-cycle watchdog episode into the new connection, otherwise a
 * successful relay recovery can remain stuck in POWER_CYCLE_REQUIRED. */
void ClearNetworkRecoveryState(LdsLidar::LinkStat *s) {
  if (s == nullptr) {
    return;
  }
  if (s->power_cycle_reason ==
      LdsLidar::kPowerCycleReasonNetworkRecoveryExhausted) {
    s->power_cycle_reason = LdsLidar::kPowerCycleReasonNone;
  }
  s->network_health_state = kNetworkHealthUnknown;
  s->network_health_seen = false;
  s->network_shared_suspected = false;
  s->network_health_since_ns = 0;
  s->network_last_health_ns = 0;
  s->network_last_success_ns = 0;
  s->network_window_samples = 0;
  s->network_window_failures = 0;
  s->network_consecutive_failures = 0;
  s->network_consecutive_successes = 0;
  s->network_loss_percent = 0.0;
  s->network_rtt_ms = 0.0;
  s->network_recovery_state = LdsLidar::kNetworkRecoveryIdle;
  s->network_soft_reboot_attempts = 0;
  s->network_soft_reboot_episode_ns = 0;
  s->network_soft_reboot_episode_wall_s = 0;
  s->network_soft_reboot_last_try_ns = 0;
  s->network_soft_reboot_last_try_wall_s = 0;
  s->network_soft_reboot_generation = 0;
  s->network_soft_reboot_inflight = false;
  s->network_soft_reboot_ack = false;
  s->network_soft_reboot_command_accepted = false;
  s->network_soft_reboot_disconnect = false;
  s->network_soft_reboot_reconnected = false;
  s->network_soft_reboot_settle_deadline_ns = 0;
  s->network_soft_reboot_status = 0;
  s->network_soft_reboot_response = 0;
}

/** Clear a network watchdog episode after the probe has verified recovery.
 *
 * Unlike ClearNetworkRecoveryState(), this deliberately keeps the current
 * probe result (NET_OK, rolling window and success counters) visible to the
 * dashboard.  A radar can recover on its own after the final soft reboot;
 * keeping the old POWER_CYCLE_REQUIRED latch in that case produces an
 * internally contradictory recovery-state frame and makes the relay manager
 * reject the already-recovered event.
 */
void ClearRecoveredNetworkRecoveryState(LdsLidar::LinkStat *s) {
  if (s == nullptr) {
    return;
  }
  const bool network_reason =
      s->power_cycle_reason ==
      LdsLidar::kPowerCycleReasonNetworkRecoveryExhausted;
  if (network_reason) {
    s->power_cycle_reason = LdsLidar::kPowerCycleReasonNone;
  }
  s->network_recovery_state = LdsLidar::kNetworkRecoveryIdle;
  s->network_health_since_ns = 0;
  s->network_soft_reboot_attempts = 0;
  s->network_soft_reboot_episode_ns = 0;
  s->network_soft_reboot_episode_wall_s = 0;
  s->network_soft_reboot_last_try_ns = 0;
  s->network_soft_reboot_last_try_wall_s = 0;
  s->network_soft_reboot_generation = 0;
  s->network_soft_reboot_inflight = false;
  s->network_soft_reboot_ack = false;
  s->network_soft_reboot_command_accepted = false;
  s->network_soft_reboot_disconnect = false;
  s->network_soft_reboot_reconnected = false;
  s->network_soft_reboot_settle_deadline_ns = 0;
  s->network_soft_reboot_status = 0;
  s->network_soft_reboot_response = 0;
  if (network_reason) {
    s->power_cycle_required_counted_this_episode = false;
  }
}

NormalDropoutPolicyInput BuildNormalDropoutPolicyInput(
    const LdsLidar::LinkStat &s, int64_t now_ns) {
  NormalDropoutPolicyInput input;
  input.armed = s.normal_dropout_state != LdsLidar::kNormalDropoutIdle;
  input.identity_matches =
      s.broadcast_code[0] != '\0' && s.normal_broadcast_code[0] != '\0' &&
      strncmp(s.broadcast_code, s.normal_broadcast_code,
              sizeof(s.broadcast_code)) == 0;
  input.generation_matches = s.normal_connection_generation != 0 &&
                             s.normal_connection_generation ==
                                 s.normal_dropout_generation;
  input.connected = s.connect_since_ns != 0;
  input.broadcast_fresh =
      s.last_broadcast_ns != 0 &&
      now_ns - s.last_broadcast_ns <= kBroadcastEpisodeGapNs;
  input.healthy_since_ns = s.normal_healthy_since_ns;
  input.attributed_disconnect_ns = s.normal_attributed_disconnect_ns;
  input.silence_since_ns = s.normal_dropout_since_ns;
  return input;
}

WakeDropoutPolicyInput BuildWakePolicyInput(
    const LdsLidar::LinkStat &s, int64_t now_ns) {
  WakeDropoutPolicyInput input;
  input.armed = s.wake_state != LdsLidar::kWakeRecoveryIdle;
  input.identity_matches =
      s.broadcast_code[0] != '\0' && s.wake_broadcast_code[0] != '\0' &&
      strncmp(s.broadcast_code, s.wake_broadcast_code,
              sizeof(s.broadcast_code)) == 0;
  input.generation_matches = s.wake_connection_generation != 0 &&
                             s.wake_connection_generation ==
                                 s.wake_dropout_generation;
  input.connected = s.connect_since_ns != 0;
  input.broadcast_fresh =
      s.last_broadcast_ns != 0 &&
      now_ns - s.last_broadcast_ns <= kBroadcastEpisodeGapNs;
  input.request_id = s.wake_request_id;
  input.wake_started_ns = s.wake_started_ns;
  input.wake_deadline_ns = s.wake_deadline_ns;
  input.attributed_disconnect_ns = s.wake_attributed_disconnect_ns;
  input.dropout_since_ns = s.wake_dropout_since_ns;
  return input;
}

/** Re-evaluate every hardware-escalation guard against one exact live
 *  broadcast episode.  Both the timer's candidate pass and its final commit
 *  pass use this predicate, so a connect, broadcast gap, or NETWORK_ERROR
 *  arriving between them cancels the edge before it is counted or emitted. */
bool IsPowerCycleEscalationReady(const LdsLidar::LinkStat &s, int64_t now,
                                 int64_t expected_episode_since_ns) {
  if (expected_episode_since_ns == 0 || s.connect_since_ns != 0 ||
      s.broadcast_only_since_ns != expected_episode_since_ns ||
      s.handshake_state != LdsLidar::kHandshakeLinkStuck ||
      s.last_broadcast_ns == 0 ||
      now - s.last_broadcast_ns > kBroadcastEpisodeGapNs) {
    return false;
  }

  const int64_t network_error_age = now - s.handshake_last_network_error_ns;
  const bool recent_local_network_error =
      s.handshake_last_network_error_ns != 0 &&
      s.handshake_last_network_error_ns >= expected_episode_since_ns &&
      network_error_age <= kHandshakeNetworkErrorGateNs;
  return !recent_local_network_error &&
         s.handshake_reset_attempts >= kHandshakeResetMaxAttempts &&
         s.handshake_reset_phase == LdsLidar::kHandshakeResetCompleted &&
         s.handshake_reset_accepted && s.handshake_reset_completed &&
         s.handshake_reset_completed_ns != 0 &&
         now - s.handshake_reset_completed_ns >=
             kHandshakePostResetObserveNs &&
         now - expected_episode_since_ns >= kHandshakePowerCycleNs;
}

/** Fill buf with the current wall-clock time as HH:MM:SS. */
void NowHms(char *buf, size_t len) {
  time_t t = time(nullptr);
  struct tm tmv;
  localtime_r(&t, &tmv);
  strftime(buf, len, "%H:%M:%S", &tmv);
}

/** Print a wall-clock timestamped link event line. */
void PrintLidarEvent(uint8_t handle, const char *bcode, const char *what) {
  char ts[16];
  NowHms(ts, sizeof(ts));
  printf("[LivoxEvent] %s Lidar[%d][%s] %s\n", ts, handle,
         (bcode && bcode[0]) ? bcode : "?", what);
}

const char *HandshakeEventName(DeviceHandshakeEvent event) {
  switch (event) {
    case kDeviceHandshakeSuccess:
      return "ACK_DEVICEINFO_PENDING";
    case kDeviceHandshakeTimeout:
      return "TIMEOUT";
    case kDeviceHandshakeRejected:
      return "REJECTED";
    case kDeviceHandshakeNetworkError:
      return "NETWORK_ERROR";
    case kDeviceHandshakeProtocolError:
      return "PROTOCOL_ERROR";
    case kDeviceHandshakeReset:
      return "RESET";
    default:
      return "UNKNOWN";
  }
}
}  // namespace

void LdsLidar::OnLidarConnectEvent(uint8_t handle, const char *broadcast_code) {
  if (handle >= kMaxLidarCount) {
    return;
  }
  lock_guard<mutex> lock(link_stat_lock_[handle]);
  LinkStat &s = link_stat_[handle];
  const bool identity_changed =
      s.broadcast_code[0] != '\0' && broadcast_code != nullptr &&
      broadcast_code[0] != '\0' &&
      strncmp(s.broadcast_code, broadcast_code,
              sizeof(s.broadcast_code)) != 0;
  if (identity_changed) {
    /** A connect event can occasionally be the first callback observed for a
     *  reused handle.  Establish the physical identity here as well as in the
     *  broadcast path so old process history cannot survive that ordering. */
    s = LinkStat();
  }
  if (broadcast_code != nullptr && broadcast_code[0] != '\0') {
    strncpy(s.broadcast_code, broadcast_code, sizeof(s.broadcast_code) - 1);
    s.broadcast_code[sizeof(s.broadcast_code) - 1] = '\0';
  }
  if (s.connect_since_ns != 0) {
    return;  /** already counted as connected */
  }
  int64_t now = std::chrono::steady_clock::now().time_since_epoch().count();
  HandshakeLinkState previous_handshake_state = s.handshake_state;
  WakeRecoveryState previous_wake_state = s.wake_state;
  NormalDropoutRecoveryState previous_normal_dropout_state =
      s.normal_dropout_state;
  const bool planned_group_recovery =
      s.planned_group_power_cycle_active &&
      s.planned_group_power_cycle_deadline_ns >= now;
  uint8_t reset_attempts = s.handshake_reset_attempts;
  int64_t handshake_since_ns = s.broadcast_only_since_ns;
  s.connect_since_ns = now;
  if (s.network_soft_reboot_disconnect &&
      s.network_recovery_state == kNetworkRecoverySoftRebootVerifying) {
    s.network_soft_reboot_reconnected = true;
  }
  s.broadcast_only_since_ns = 0;
  s.handshake_state = kHandshakeLinkIdle;
  s.handshake_reset_attempts = 0;
  s.handshake_last_reset_try_ns = 0;
  s.handshake_reset_phase = kHandshakeResetNone;
  s.handshake_reset_accepted = false;
  s.handshake_reset_completed = false;
  s.handshake_reset_completed_ns = 0;
  s.handshake_event_valid = false;
  s.handshake_last_network_error_ns = 0;
  s.power_cycle_required_counted_this_episode = false;
  const bool network_hard_recovery =
      s.network_recovery_state == kNetworkRecoveryPowerCycleRequired;
  if (network_hard_recovery) {
    ClearNetworkRecoveryState(&s);
  }
  if (s.network_recovery_state != kNetworkRecoverySoftRebootPending &&
      s.network_recovery_state != kNetworkRecoverySoftRebootVerifying) {
    s.power_cycle_reason = kPowerCycleReasonNone;
  }
  /** A real Connect is recovery evidence. In particular, clear a wake edge
   *  after relay ON so the pre-cycle Normal request cannot create a second
   *  dropout episode on the new connection. */
  ClearWakeRecoveryState(&s);
  ClearNormalDropoutState(&s);
  s.planned_reboot_generation = 0;
  s.planned_group_power_cycle_active = false;
  s.planned_group_power_cycle_deadline_ns = 0;
  memset(s.planned_group_power_cycle_token, 0,
         sizeof(s.planned_group_power_cycle_token));
  if (s.last_disconnect_ns != 0) {
    long long down_s = (now - s.last_disconnect_ns) / 1000000000LL;
    char buf[48];
    snprintf(buf, sizeof(buf), "%s (down %llds)",
             planned_group_recovery ? "PLANNED_GROUP_POWER_RECOVERED"
                                    : "RECONNECTED",
             down_s);
    PrintLidarEvent(handle, broadcast_code, buf);
    char detail[32];
    snprintf(detail, sizeof(detail), "down %llds", down_s);
    HealthLogger::Get().LogEvent(
        handle, broadcast_code,
        planned_group_recovery ? "PLANNED_GROUP_POWER_RECOVERED" : "RECONNECT",
        detail);
  } else {
    PrintLidarEvent(handle, broadcast_code, "CONNECTED");
    HealthLogger::Get().LogEvent(handle, broadcast_code, "CONNECT", "");
  }
  if (!planned_group_recovery &&
      (previous_handshake_state >= kHandshakeLinkStuck || reset_attempts != 0)) {
    long long elapsed_s = handshake_since_ns == 0
                              ? 0
                              : (now - handshake_since_ns) / 1000000000LL;
    char detail[80];
    snprintf(detail, sizeof(detail), "after %llds, %u session reset request(s)",
             elapsed_s, reset_attempts);
    PrintLidarEvent(handle, broadcast_code, "HANDSHAKE_RECOVERED");
    HealthLogger::Get().LogEvent(handle, broadcast_code,
                                 "HANDSHAKE_RECOVERED", detail);
  }
  if (!planned_group_recovery &&
      previous_wake_state >= kWakeRecoveryNoBroadcast) {
    PrintLidarEvent(handle, broadcast_code, "WAKE_LINK_RECOVERED");
    HealthLogger::Get().LogEvent(handle, broadcast_code,
                                 "WAKE_LINK_RECOVERED", "Connect returned");
  }
  if (!planned_group_recovery &&
      previous_normal_dropout_state >= kNormalDropoutNoBroadcast) {
    PrintLidarEvent(handle, broadcast_code, "NORMAL_LINK_RECOVERED");
    HealthLogger::Get().LogEvent(handle, broadcast_code,
                                 "NORMAL_LINK_RECOVERED", "Connect returned");
  }
}

void LdsLidar::OnLidarDisconnectEvent(uint8_t handle,
                                      const char *broadcast_code) {
  if (handle >= kMaxLidarCount) {
    return;
  }
  lock_guard<mutex> lock(link_stat_lock_[handle]);
  LinkStat &s = link_stat_[handle];
  /** The SDK can report the same disconnect more than once before a reconnect.
   *  Count only the connected -> disconnected edge and retain the timestamp of
   *  the first report so the outage duration is not shortened. */
  if (s.connect_since_ns == 0 && s.last_disconnect_ns != 0) {
    return;
  }
  const int64_t now =
      std::chrono::steady_clock::now().time_since_epoch().count();
  bool wake_no_broadcast = false;
  bool normal_no_broadcast = false;
  const uint64_t current_generation =
      connection_generation_[handle].load(std::memory_order_acquire);
  const bool callback_identity_matches =
      s.broadcast_code[0] != '\0' && broadcast_code != nullptr &&
      broadcast_code[0] != '\0' &&
      strncmp(s.broadcast_code, broadcast_code,
              sizeof(s.broadcast_code)) == 0;
  std::string planned_group_token = s.planned_group_power_cycle_token;
  const bool consumed_planned_marker =
      callback_identity_matches &&
      ConsumePlannedGroupPowerCycle(broadcast_code, &planned_group_token);
  const bool planned_group_disconnect =
      callback_identity_matches &&
      (s.planned_group_power_cycle_active || consumed_planned_marker);
  if (planned_group_disconnect) {
    if (!s.planned_group_power_cycle_active) {
      ++s.planned_group_power_cycle_count;
    }
    s.planned_group_power_cycle_active = true;
    // Match the manager's default 180 s post-ON health-verification window.
    // After that deadline the current dashboard must show a real disconnect,
    // while the maintenance event remains visible in process history.
    s.planned_group_power_cycle_deadline_ns =
        now + INT64_C(180000000000);
    strncpy(s.planned_group_power_cycle_token, planned_group_token.c_str(),
            sizeof(s.planned_group_power_cycle_token) - 1);
    s.planned_group_power_cycle_token[
        sizeof(s.planned_group_power_cycle_token) - 1] = '\0';
  } else {
    ++s.disconnect_count;
  }
  s.last_disconnect_ns = now;
  const bool planned_reboot_disconnect =
      callback_identity_matches && s.planned_reboot_generation != 0 &&
      s.planned_reboot_generation == current_generation;
  if (planned_group_disconnect) {
    /** Healthy companion lidars are intentionally interrupted by the shared
     * relay and must not gain point-cloud outage history. If this lidar already
     * had a real outage before the intent, preserve that triggering episode so
     * its end-to-end recovery duration includes the relay repair. */
    if (!s.point_cloud_outage.outage_active) {
      ExcludePointCloudOutage(&s.point_cloud_outage);
    }
  } else {
    BeginPointCloudOutage(&s.point_cloud_outage, now,
                          static_cast<int64_t>(time(nullptr)) *
                              INT64_C(1000000000));
  }
  if (planned_group_disconnect) {
    /** The relay manager obtained a Driver ACK before issuing OFF. This edge
     *  belongs to that shared maintenance action and must not make a healthy
     *  companion lidar look unstable or request another power cycle. */
    ClearWakeRecoveryState(&s);
    ClearNormalDropoutState(&s);
    s.handshake_state = kHandshakeLinkIdle;
    s.power_cycle_reason = kPowerCycleReasonNone;
    s.power_cycle_required_counted_this_episode = false;
    ClearNetworkRecoveryState(&s);
  } else if (planned_reboot_disconnect) {
    /** This disconnect belongs to an explicit software reboot, not to the
     *  earlier wake command. It must never grant shared-relay permission. */
    ClearWakeRecoveryState(&s);
    ClearNormalDropoutState(&s);
    if (s.network_soft_reboot_generation == current_generation) {
      s.network_soft_reboot_disconnect = true;
      s.network_soft_reboot_inflight = false;
      s.network_soft_reboot_command_accepted = true;
      s.network_soft_reboot_settle_deadline_ns =
          now + s.network_soft_reboot_settle_ns;
      s.network_recovery_state = kNetworkRecoverySoftRebootVerifying;
    }
    s.planned_reboot_generation = 0;
  } else if (s.wake_state == kWakeRecoveryObserving) {
    const bool same_identity =
        callback_identity_matches && s.wake_broadcast_code[0] != '\0' &&
        strncmp(s.wake_broadcast_code, s.broadcast_code,
                sizeof(s.broadcast_code)) == 0;
    if (same_identity &&
        s.wake_connection_generation == current_generation &&
        now <= s.wake_deadline_ns) {
      s.wake_dropout_generation = current_generation;
      s.wake_attributed_disconnect_ns = now;
      s.wake_attributed_disconnect_wall_s =
          static_cast<int64_t>(time(nullptr));
      s.wake_dropout_since_ns = now;
      s.wake_dropout_wall_s = s.wake_attributed_disconnect_wall_s;
      s.wake_broadcast_return_since_ns = 0;
      s.wake_broadcast_return_count = 0;
      s.wake_state = kWakeRecoveryNoBroadcast;
      ClearNormalDropoutState(&s);
      wake_no_broadcast = true;
    } else {
      ClearWakeRecoveryState(&s);
    }
  } else if (s.normal_dropout_state == kNormalDropoutArmed) {
    NormalDropoutPolicyInput normal_input;
    normal_input.armed = true;
    normal_input.identity_matches =
        callback_identity_matches && s.normal_broadcast_code[0] != '\0' &&
        strncmp(s.normal_broadcast_code, s.broadcast_code,
                sizeof(s.broadcast_code)) == 0;
    normal_input.generation_matches =
        s.normal_connection_generation != 0 &&
        s.normal_connection_generation == current_generation;
    normal_input.healthy_since_ns = s.normal_healthy_since_ns;
    if (NormalDropoutArmMature(normal_input, now, kNormalHealthyArmNs)) {
      s.normal_dropout_generation = current_generation;
      s.normal_attributed_disconnect_ns = now;
      s.normal_attributed_disconnect_wall_s =
          static_cast<int64_t>(time(nullptr));
      s.normal_dropout_since_ns = now;
      s.normal_dropout_wall_s = s.normal_attributed_disconnect_wall_s;
      s.normal_broadcast_return_since_ns = 0;
      s.normal_broadcast_return_count = 0;
      s.normal_dropout_state = kNormalDropoutNoBroadcast;
      normal_no_broadcast = true;
    } else {
      ClearNormalDropoutState(&s);
    }
  } else {
    ClearNormalDropoutState(&s);
  }
  MeasurementSessionDisconnected(&s.measurement_session);
  if (s.power_cycle_reason == kPowerCycleReasonErrorRebootExhausted) {
    s.power_cycle_reason = kPowerCycleReasonNone;
  }
  s.connect_since_ns = 0;
  s.health_code = 0;  /** stale once disconnected */
  s.last_broadcast_ns = 0;
  s.broadcast_only_since_ns = 0;
  s.handshake_state = kHandshakeLinkIdle;
  s.handshake_reset_attempts = 0;
  s.handshake_last_reset_try_ns = 0;
  s.handshake_reset_phase = kHandshakeResetNone;
  s.handshake_reset_accepted = false;
  s.handshake_reset_completed = false;
  s.handshake_reset_completed_ns = 0;
  s.handshake_last_network_error_ns = 0;
  s.power_cycle_required_counted_this_episode = false;
  if (s.power_cycle_reason == kPowerCycleReasonHandshakeStuck) {
    s.power_cycle_reason = kPowerCycleReasonNone;
  }
  if (planned_group_disconnect) {
    PrintLidarEvent(handle, broadcast_code, "PLANNED_GROUP_POWER_CYCLE");
    HealthLogger::Get().LogEvent(handle, broadcast_code,
                                 "PLANNED_GROUP_POWER_CYCLE",
                                 planned_group_token.c_str());
  } else {
    PrintLidarEvent(handle, broadcast_code, "DISCONNECTED");
    HealthLogger::Get().LogEvent(handle, broadcast_code, "DISCONNECT", "");
  }
  if (wake_no_broadcast) {
    PrintLidarEvent(handle, broadcast_code, "WAKE_NO_BROADCAST");
    HealthLogger::Get().LogEvent(
        handle, broadcast_code, "WAKE_NO_BROADCAST",
        "explicit low-power wake lost control link; confirming 10s silence");
  }
  if (normal_no_broadcast) {
    PrintLidarEvent(handle, broadcast_code, "NORMAL_NO_BROADCAST");
    HealthLogger::Get().LogEvent(
        handle, broadcast_code, "NORMAL_NO_BROADCAST",
        "previously healthy Normal stream disconnected; confirming 5s silence");
  }
}

void LdsLidar::OnLidarBroadcastEvent(uint8_t handle,
                                     const char *broadcast_code) {
  if (handle >= kMaxLidarCount || broadcast_code == nullptr ||
      broadcast_code[0] == '\0') {
    return;
  }
  int64_t now = std::chrono::steady_clock::now().time_since_epoch().count();
  bool new_episode = false;
  bool wake_broadcast_returned = false;
  bool wake_handoff_completed = false;
  bool normal_broadcast_returned = false;
  bool normal_handoff_completed = false;
  {
    lock_guard<mutex> lock(link_stat_lock_[handle]);
    LinkStat &s = link_stat_[handle];
    const bool identity_changed =
        s.broadcast_code[0] != '\0' &&
        strncmp(s.broadcast_code, broadcast_code,
                sizeof(s.broadcast_code)) != 0;
    if (identity_changed) {
      /** SDK handles are reusable.  Never let a newly assigned physical lidar
       *  inherit health, disconnect, handshake, or recovery history from the
       *  previous broadcast code which occupied this slot. */
      s = LinkStat();
    }
    strncpy(s.broadcast_code, broadcast_code, sizeof(s.broadcast_code) - 1);
    s.broadcast_code[sizeof(s.broadcast_code) - 1] = '\0';
    s.broadcast_count++;
    const int64_t previous_broadcast_ns = s.last_broadcast_ns;
    bool broadcast_gap = previous_broadcast_ns != 0 &&
                         now - previous_broadcast_ns >
                             kBroadcastEpisodeGapNs;
    s.last_broadcast_ns = now;
    if (s.connect_since_ns == 0 &&
        s.wake_attributed_disconnect_ns != 0 &&
        s.wake_state != kWakeRecoveryIdle) {
      /** One residual frame is not stable recovery. Preserve the strict
       *  in-window disconnect attribution, pause only the current continuous
       *  silence interval, and hand ownership to the handshake path only
       *  after several live frames spanning a full freshness interval. */
      if (s.wake_broadcast_return_since_ns == 0 ||
          previous_broadcast_ns == 0 || broadcast_gap) {
        s.wake_broadcast_return_since_ns = now;
        s.wake_broadcast_return_count = 1;
        wake_broadcast_returned = true;
      } else {
        ++s.wake_broadcast_return_count;
      }
      if (s.power_cycle_reason == kPowerCycleReasonWakeDropout) {
        s.power_cycle_reason = kPowerCycleReasonNone;
      }
      s.wake_state = kWakeRecoveryObserving;
      s.wake_dropout_since_ns = 0;
      s.wake_dropout_wall_s = 0;
      if (s.wake_broadcast_return_count >=
              kWakeBroadcastHandoffMinFrames &&
          now - s.wake_broadcast_return_since_ns >=
              kWakeBroadcastHandoffNs) {
        ClearWakeRecoveryState(&s);
        wake_handoff_completed = true;
      }
    } else if (s.connect_since_ns == 0 &&
               s.normal_attributed_disconnect_ns != 0 &&
               s.normal_dropout_state != kNormalDropoutIdle) {
      if (s.normal_broadcast_return_since_ns == 0 ||
          previous_broadcast_ns == 0 || broadcast_gap) {
        s.normal_broadcast_return_since_ns = now;
        s.normal_broadcast_return_count = 1;
        normal_broadcast_returned = true;
      } else {
        ++s.normal_broadcast_return_count;
      }
      if (s.power_cycle_reason == kPowerCycleReasonNormalDropout) {
        s.power_cycle_reason = kPowerCycleReasonNone;
      }
      s.normal_dropout_state = kNormalDropoutObservingReturn;
      s.normal_dropout_since_ns = 0;
      /** Keep the last returned frame as the start of any subsequent silence.
       *  If frames stop again, the five-second confirmation is measured from
       *  this frame, not from the later 3-second freshness timeout. */
      s.normal_dropout_wall_s = static_cast<int64_t>(time(nullptr));
      if (s.normal_broadcast_return_count >=
              kWakeBroadcastHandoffMinFrames &&
          now - s.normal_broadcast_return_since_ns >=
              kWakeBroadcastHandoffNs) {
        ClearNormalDropoutState(&s);
        normal_handoff_completed = true;
      }
    }
    if (s.connect_since_ns == 0 && s.broadcast_only_since_ns == 0) {
      s.broadcast_only_since_ns = now;
      s.handshake_state = kHandshakeLinkBroadcastOnly;
      s.handshake_reset_attempts = 0;
      s.handshake_last_reset_try_ns = 0;
      s.handshake_reset_phase = kHandshakeResetNone;
      s.handshake_reset_accepted = false;
      s.handshake_reset_completed = false;
      s.handshake_reset_completed_ns = 0;
      s.handshake_event_valid = false;
      s.handshake_last_network_error_ns = 0;
      s.power_cycle_required_counted_this_episode = false;
      s.power_cycle_reason = kPowerCycleReasonNone;
      new_episode = true;
    } else if (s.connect_since_ns == 0 && broadcast_gap) {
      /** Treat a broadcast gap as the end of the live episode even if the 1Hz
       *  timer did not run during the gap. Historical counters remain. */
      s.broadcast_only_since_ns = now;
      s.handshake_state = kHandshakeLinkBroadcastOnly;
      s.handshake_reset_attempts = 0;
      s.handshake_last_reset_try_ns = 0;
      s.handshake_reset_phase = kHandshakeResetNone;
      s.handshake_reset_accepted = false;
      s.handshake_reset_completed = false;
      s.handshake_reset_completed_ns = 0;
      s.handshake_event_valid = false;
      s.handshake_last_network_error_ns = 0;
      s.power_cycle_required_counted_this_episode = false;
      s.power_cycle_reason = kPowerCycleReasonNone;
      new_episode = true;
    }
  }
  if (new_episode) {
    PrintLidarEvent(handle, broadcast_code, "BROADCAST_ONLY");
    HealthLogger::Get().LogEvent(handle, broadcast_code, "BROADCAST_ONLY", "");
  }
  if (wake_broadcast_returned) {
    PrintLidarEvent(handle, broadcast_code, "WAKE_BROADCAST_RETURNED");
    HealthLogger::Get().LogEvent(
        handle, broadcast_code, "WAKE_BROADCAST_RETURNED",
        "wake hard-power path paused; awaiting stable broadcast handoff");
  }
  if (wake_handoff_completed) {
    PrintLidarEvent(handle, broadcast_code, "WAKE_BROADCAST_STABLE");
    HealthLogger::Get().LogEvent(
        handle, broadcast_code, "WAKE_BROADCAST_STABLE",
        "stable broadcasts returned; handshake recovery owns episode");
  }
  if (normal_broadcast_returned) {
    PrintLidarEvent(handle, broadcast_code, "NORMAL_BROADCAST_RETURNED");
    HealthLogger::Get().LogEvent(
        handle, broadcast_code, "NORMAL_BROADCAST_RETURNED",
        "normal-dropout hard-power path paused; awaiting stable broadcasts");
  }
  if (normal_handoff_completed) {
    PrintLidarEvent(handle, broadcast_code, "NORMAL_BROADCAST_STABLE");
    HealthLogger::Get().LogEvent(
        handle, broadcast_code, "NORMAL_BROADCAST_STABLE",
        "stable broadcasts returned; handshake recovery owns episode");
  }
}

void LdsLidar::ArmWakeObservation(uint8_t handle, const char *broadcast_code,
                                  uint64_t request_id,
                                  uint64_t connection_generation) {
  if (handle >= kMaxLidarCount || broadcast_code == nullptr ||
      broadcast_code[0] == '\0' || request_id == 0) {
    return;
  }
  const int64_t now =
      std::chrono::steady_clock::now().time_since_epoch().count();
  bool armed = false;
  {
    lock_guard<mutex> lock(link_stat_lock_[handle]);
    LinkStat &s = link_stat_[handle];
    if (s.connect_since_ns == 0 || s.broadcast_code[0] == '\0' ||
        strncmp(s.broadcast_code, broadcast_code,
                sizeof(s.broadcast_code)) != 0) {
      return;
    }
    if (s.wake_state != kWakeRecoveryIdle &&
        s.wake_request_id == request_id) {
      return;  /** a retry must never extend the attribution window */
    }
    ClearWakeRecoveryState(&s);
    ClearNormalDropoutState(&s);
    s.wake_state = kWakeRecoveryObserving;
    s.wake_request_id = request_id;
    s.wake_connection_generation = connection_generation;
    s.wake_started_ns = now;
    s.wake_deadline_ns = now + kWakeObservationNs;
    s.wake_started_wall_s = static_cast<int64_t>(time(nullptr));
    strncpy(s.wake_broadcast_code, broadcast_code,
            sizeof(s.wake_broadcast_code) - 1);
    s.wake_broadcast_code[sizeof(s.wake_broadcast_code) - 1] = '\0';
    armed = true;
  }
  if (armed) {
    PrintLidarEvent(handle, broadcast_code, "WAKE_OBSERVING_60S");
    HealthLogger::Get().LogEvent(
        handle, broadcast_code, "WAKE_OBSERVING",
        "explicit PowerSaving/StandBy -> Normal command accepted");
  }
}

void LdsLidar::CancelWakeObservation(uint8_t handle,
                                     uint64_t expected_request_id) {
  if (handle >= kMaxLidarCount) {
    return;
  }
  lock_guard<mutex> lock(link_stat_lock_[handle]);
  LinkStat &s = link_stat_[handle];
  if (expected_request_id != 0 &&
      s.wake_request_id != expected_request_id) {
    return;
  }
  ClearWakeRecoveryState(&s);
}

void LdsLidar::ObserveNormalPublishing(uint8_t handle, bool healthy,
                                       uint64_t expected_generation,
                                       const char *expected_broadcast_code) {
  if (handle >= kMaxLidarCount) {
    return;
  }
  const int64_t now =
      std::chrono::steady_clock::now().time_since_epoch().count();
  lock_guard<mutex> lock(link_stat_lock_[handle]);
  LinkStat &s = link_stat_[handle];
  if (!healthy) {
    /** Do not erase an already-attributed outage merely because the 1 Hz
     *  publisher now sees the disconnected row. Only the pre-fault arm is
     *  withdrawn when the live stream ceases to be continuously healthy. */
    if (s.normal_dropout_state == kNormalDropoutArmed) {
      ClearNormalDropoutState(&s);
    }
    return;
  }
  const uint64_t live_generation =
      connection_generation_[handle].load(std::memory_order_acquire);
  /** The data-plane snapshot and LinkStat are copied under different locks.
   *  Reject an old healthy sample if a disconnect/reconnect callback won in
   *  between.  Never erase an already-attributed outage from such a sample. */
  if (s.connect_since_ns == 0 || s.broadcast_code[0] == '\0' ||
      expected_broadcast_code == nullptr ||
      expected_broadcast_code[0] == '\0' || expected_generation == 0 ||
      live_generation != expected_generation ||
      strncmp(s.broadcast_code, expected_broadcast_code,
              sizeof(s.broadcast_code)) != 0) {
    if (s.normal_dropout_state == kNormalDropoutArmed) {
      ClearNormalDropoutState(&s);
    }
    return;
  }
  const bool same_arm =
      s.normal_dropout_state == kNormalDropoutArmed &&
      s.normal_connection_generation == live_generation &&
      s.normal_broadcast_code[0] != '\0' &&
      strncmp(s.normal_broadcast_code, s.broadcast_code,
              sizeof(s.broadcast_code)) == 0;
  if (same_arm) {
    return;
  }
  ClearNormalDropoutState(&s);
  s.normal_dropout_state = kNormalDropoutArmed;
  s.normal_connection_generation = live_generation;
  s.normal_healthy_since_ns = now;
  s.normal_healthy_since_wall_s = static_cast<int64_t>(time(nullptr));
  strncpy(s.normal_broadcast_code, s.broadcast_code,
          sizeof(s.normal_broadcast_code) - 1);
  s.normal_broadcast_code[sizeof(s.normal_broadcast_code) - 1] = '\0';
}

std::vector<std::string> LdsLidar::GetWhitelistBroadcastCodes() const {
  std::vector<std::string> result;
  result.reserve(whitelist_count_);
  for (uint32_t i = 0; i < whitelist_count_; ++i) {
    if (broadcast_code_whitelist_[i][0] != '\0') {
      result.emplace_back(broadcast_code_whitelist_[i]);
    }
  }
  return result;
}

bool LdsLidar::ArmPlannedGroupPowerCycle(
    const std::vector<std::string> &members, const std::string &token,
    uint32_t valid_for_ms, std::string *detail) {
  const std::vector<std::string> whitelist = GetWhitelistBroadcastCodes();
  const std::set<std::string> configured(whitelist.begin(), whitelist.end());
  const std::set<std::string> intended(members.begin(), members.end());
  if (members.size() != 4 || intended.size() != 4 || configured != intended) {
    if (detail != nullptr) {
      std::ostringstream message;
      message << "relay members do not exactly match Driver whitelist"
              << " (members=" << intended.size()
              << ", whitelist=" << configured.size() << ")";
      *detail = message.str();
    }
    return false;
  }
  const int64_t now =
      std::chrono::steady_clock::now().time_since_epoch().count();
  const int64_t expires =
      now + static_cast<int64_t>(valid_for_ms) * 1000000LL;
  {
    lock_guard<mutex> lock(planned_group_power_cycle_lock_);
    for (auto row = planned_group_power_cycles_.begin();
         row != planned_group_power_cycles_.end();) {
      if (row->second.expires_ns <= now) {
        row = planned_group_power_cycles_.erase(row);
      } else {
        ++row;
      }
    }
    for (const std::string &member : members) {
      PlannedGroupPowerCycle marker;
      marker.token = token;
      marker.expires_ns = expires;
      planned_group_power_cycles_[member] = marker;
    }
  }
  if (detail != nullptr) {
    *detail = "all 4 whitelist members armed before relay OFF";
  }
  return true;
}

void LdsLidar::CancelPlannedGroupPowerCycle(
    const std::vector<std::string> &members, const std::string &token) {
  lock_guard<mutex> lock(planned_group_power_cycle_lock_);
  for (const std::string &member : members) {
    const auto row = planned_group_power_cycles_.find(member);
    if (row != planned_group_power_cycles_.end() &&
        row->second.token == token) {
      planned_group_power_cycles_.erase(row);
    }
  }
}

bool LdsLidar::IsPlannedGroupPowerCycleActive(
    const std::string &broadcast_code, int64_t now_ns) {
  lock_guard<mutex> lock(planned_group_power_cycle_lock_);
  const auto row = planned_group_power_cycles_.find(broadcast_code);
  return row != planned_group_power_cycles_.end() &&
         row->second.expires_ns >= now_ns;
}

bool LdsLidar::ConsumePlannedGroupPowerCycle(const char *broadcast_code,
                                             std::string *token) {
  if (broadcast_code == nullptr || broadcast_code[0] == '\0') {
    return false;
  }
  const int64_t now =
      std::chrono::steady_clock::now().time_since_epoch().count();
  lock_guard<mutex> lock(planned_group_power_cycle_lock_);
  const auto row = planned_group_power_cycles_.find(broadcast_code);
  if (row == planned_group_power_cycles_.end()) {
    return false;
  }
  if (row->second.expires_ns <= now) {
    planned_group_power_cycles_.erase(row);
    return false;
  }
  if (token != nullptr) {
    *token = row->second.token;
  }
  planned_group_power_cycles_.erase(row);
  return true;
}

void LdsLidar::TickNormalDropoutRecovery(bool enable_recovery) {
  const int64_t now =
      std::chrono::steady_clock::now().time_since_epoch().count();
  for (uint8_t handle = 0; handle < kMaxLidarCount; ++handle) {
    bool became_no_broadcast = false;
    bool became_dropout = false;
    bool power_candidate = false;
    uint64_t expected_generation = 0;
    int64_t expected_dropout_since = 0;
    char broadcast_code[kBroadcastCodeSize] = {0};
    {
      lock_guard<mutex> lock(link_stat_lock_[handle]);
      LinkStat &s = link_stat_[handle];
      if (s.normal_dropout_state == kNormalDropoutIdle ||
          s.normal_dropout_state == kNormalDropoutArmed ||
          s.normal_dropout_state == kNormalDropoutPowerCycleRequired) {
        continue;
      }
      NormalDropoutPolicyInput input =
          BuildNormalDropoutPolicyInput(s, now);
      if (s.normal_dropout_state == kNormalDropoutObservingReturn) {
        if (!NormalDropoutAttributionValid(input)) {
          ClearNormalDropoutState(&s);
        } else if (!input.broadcast_fresh) {
          s.normal_dropout_state = kNormalDropoutNoBroadcast;
          s.normal_dropout_since_ns =
              s.last_broadcast_ns != 0 ? s.last_broadcast_ns : now;
          if (s.normal_dropout_wall_s == 0) {
            s.normal_dropout_wall_s = static_cast<int64_t>(time(nullptr));
          }
          s.normal_broadcast_return_since_ns = 0;
          s.normal_broadcast_return_count = 0;
          became_no_broadcast = true;
          strncpy(broadcast_code, s.broadcast_code,
                  sizeof(broadcast_code) - 1);
        }
      } else {
        if (!NormalDropoutEscalationReady(input, now,
                                          kNormalDropoutConfirmNs)) {
          continue;
        }
        if (s.normal_dropout_state == kNormalDropoutNoBroadcast) {
          s.normal_dropout_state = kNormalDropoutConfirmed;
          if (!s.normal_dropout_counted_this_episode) {
            s.normal_dropout_count++;
            s.normal_dropout_counted_this_episode = true;
          }
          became_dropout = true;
        }
        strncpy(broadcast_code, s.broadcast_code,
                sizeof(broadcast_code) - 1);
        expected_generation = s.normal_dropout_generation;
        expected_dropout_since = s.normal_dropout_since_ns;
        power_candidate =
            enable_recovery &&
            s.normal_dropout_state == kNormalDropoutConfirmed;
      }
    }

    if (became_no_broadcast) {
      PrintLidarEvent(handle, broadcast_code, "NORMAL_NO_BROADCAST");
      HealthLogger::Get().LogEvent(
          handle, broadcast_code, "NORMAL_NO_BROADCAST",
          "broadcast return was transient; confirming 5s silence");
    }
    if (became_dropout) {
      PrintLidarEvent(handle, broadcast_code, "NORMAL_DROPOUT");
      HealthLogger::Get().LogEvent(
          handle, broadcast_code, "NORMAL_DROPOUT",
          enable_recovery
              ? "previously healthy Normal stream absent for 5s"
              : "previously healthy Normal stream absent for 5s; detection only");
    }
    if (!power_candidate) {
      continue;
    }

    bool committed = false;
    const int64_t commit_now =
        std::chrono::steady_clock::now().time_since_epoch().count();
    {
      lock_guard<mutex> lock(link_stat_lock_[handle]);
      LinkStat &s = link_stat_[handle];
      const NormalDropoutPolicyInput input =
          BuildNormalDropoutPolicyInput(s, commit_now);
      if (s.normal_dropout_state == kNormalDropoutConfirmed &&
          s.normal_dropout_generation == expected_generation &&
          s.normal_dropout_since_ns == expected_dropout_since &&
          NormalDropoutEscalationReady(input, commit_now,
                                       kNormalDropoutConfirmNs)) {
        ClearWakeRecoveryState(&s);
        s.normal_dropout_state = kNormalDropoutPowerCycleRequired;
        s.power_cycle_reason = kPowerCycleReasonNormalDropout;
        s.power_cycle_required_count++;
        if (!s.normal_power_cycle_counted_this_episode) {
          s.power_cycle_required_episode_count++;
          s.normal_power_cycle_episode_count++;
          s.normal_power_cycle_counted_this_episode = true;
        }
        s.power_cycle_required_wall_s = static_cast<int64_t>(time(nullptr));
        committed = true;
        strncpy(broadcast_code, s.broadcast_code,
                sizeof(broadcast_code) - 1);
        broadcast_code[sizeof(broadcast_code) - 1] = '\0';
      }
    }
    if (!committed) {
      continue;
    }
    PrintLidarEvent(handle, broadcast_code, "POWER_CYCLE_REQUIRED");
    HealthLogger::Get().LogEvent(
        handle, broadcast_code, "POWER_CYCLE_REQUIRED",
        "reason=NORMAL_DROPOUT; previously healthy stream absent for 5s");
    printf("[LivoxRecover] Lidar[%d][%s] NORMAL_DROPOUT confirmed; "
           "physical group power cycle required\n",
           handle, broadcast_code);
  }
}

void LdsLidar::TickWakeDropoutRecovery(bool enable_recovery) {
  const int64_t now =
      std::chrono::steady_clock::now().time_since_epoch().count();
  for (uint8_t handle = 0; handle < kMaxLidarCount; ++handle) {
    bool became_no_broadcast = false;
    bool became_dropout = false;
    bool power_candidate = false;
    uint64_t expected_request_id = 0;
    int64_t expected_dropout_since = 0;
    char broadcast_code[kBroadcastCodeSize] = {0};
    {
      lock_guard<mutex> lock(link_stat_lock_[handle]);
      LinkStat &s = link_stat_[handle];
      if (s.wake_state == kWakeRecoveryIdle ||
          s.wake_state == kWakeRecoveryPowerCycleRequired) {
        continue;
      }
      if (s.wake_state == kWakeRecoveryObserving) {
        const WakeDropoutPolicyInput input = BuildWakePolicyInput(s, now);
        if (s.wake_attributed_disconnect_ns == 0) {
          if (WakeObservationExpired(input, now)) {
            ClearWakeRecoveryState(&s);
          }
        } else if (!WakeDropoutAttributionValid(input)) {
          ClearWakeRecoveryState(&s);
        } else if (!input.broadcast_fresh) {
          /** A residual frame returned and then stopped. Begin a fresh,
           *  conservative ten-second continuous-silence interval instead of
           *  losing the original in-window attribution. This confirmation may
           *  cross the 60-second deadline because the causal disconnect was
           *  already captured inside it. */
          s.wake_state = kWakeRecoveryNoBroadcast;
          s.wake_dropout_since_ns = now;
          s.wake_dropout_wall_s = static_cast<int64_t>(time(nullptr));
          s.wake_broadcast_return_since_ns = 0;
          s.wake_broadcast_return_count = 0;
          became_no_broadcast = true;
          strncpy(broadcast_code, s.broadcast_code,
                  sizeof(broadcast_code) - 1);
        }
      } else {
        const WakeDropoutPolicyInput input = BuildWakePolicyInput(s, now);
        if (!WakeDropoutEscalationReady(input, now,
                                        kWakeDropoutConfirmNs)) {
          continue;
        }
        if (s.wake_state == kWakeRecoveryNoBroadcast) {
          s.wake_state = kWakeRecoveryDropout;
          if (!s.wake_dropout_counted_this_request) {
            s.wake_dropout_count++;
            s.wake_dropout_counted_this_request = true;
          }
          became_dropout = true;
        }
        strncpy(broadcast_code, s.broadcast_code,
                sizeof(broadcast_code) - 1);
        expected_request_id = s.wake_request_id;
        expected_dropout_since = s.wake_dropout_since_ns;
        power_candidate = enable_recovery &&
                          s.wake_state == kWakeRecoveryDropout;
      }
    }

    if (became_no_broadcast) {
      PrintLidarEvent(handle, broadcast_code, "WAKE_NO_BROADCAST");
      HealthLogger::Get().LogEvent(
          handle, broadcast_code, "WAKE_NO_BROADCAST",
          "broadcast return was transient; confirming 10s silence");
    }
    if (became_dropout) {
      PrintLidarEvent(handle, broadcast_code, "WAKE_DROPOUT");
      HealthLogger::Get().LogEvent(
          handle, broadcast_code, "WAKE_DROPOUT",
          enable_recovery
              ? "explicit wake lost control+broadcast for 10s"
              : "explicit wake lost control+broadcast for 10s; detection only");
    }
    if (!power_candidate) {
      continue;
    }

    bool committed = false;
    const int64_t commit_now =
        std::chrono::steady_clock::now().time_since_epoch().count();
    {
      lock_guard<mutex> lock(link_stat_lock_[handle]);
      LinkStat &s = link_stat_[handle];
      const WakeDropoutPolicyInput input = BuildWakePolicyInput(s, commit_now);
      if (s.wake_state == kWakeRecoveryDropout &&
          s.wake_request_id == expected_request_id &&
          s.wake_dropout_since_ns == expected_dropout_since &&
          WakeDropoutEscalationReady(input, commit_now,
                                     kWakeDropoutConfirmNs)) {
        s.wake_state = kWakeRecoveryPowerCycleRequired;
        s.power_cycle_reason = kPowerCycleReasonWakeDropout;
        s.power_cycle_required_count++;
        if (!s.wake_power_cycle_counted_this_request) {
          s.power_cycle_required_episode_count++;
          s.wake_power_cycle_episode_count++;
          s.wake_power_cycle_counted_this_request = true;
        }
        s.power_cycle_required_wall_s = static_cast<int64_t>(time(nullptr));
        committed = true;
        strncpy(broadcast_code, s.broadcast_code,
                sizeof(broadcast_code) - 1);
        broadcast_code[sizeof(broadcast_code) - 1] = '\0';
      }
    }
    if (!committed) {
      continue;
    }

    /** Do this after releasing link_stat_lock_: ResetModeRequestIfTarget takes
     *  the per-handle SDK send lock. Leaving the pre-fault Normal request
     *  queued would re-send it immediately after relay ON. */
    ResetModeRequestIfTarget(handle, kLidarModeNormal,
                             expected_request_id);
    PrintLidarEvent(handle, broadcast_code, "POWER_CYCLE_REQUIRED");
    HealthLogger::Get().LogEvent(
        handle, broadcast_code, "POWER_CYCLE_REQUIRED",
        "reason=WAKE_DROPOUT; no control link or broadcast for 10s");
    printf("[LivoxRecover] Lidar[%d][%s] WAKE_DROPOUT confirmed; "
           "physical group power cycle required\n",
           handle, broadcast_code);
  }
}

void LdsLidar::TickHandshakeRecovery(bool enable_recovery) {
  const int64_t now =
      std::chrono::steady_clock::now().time_since_epoch().count();
  for (uint8_t handle = 0; handle < kMaxLidarCount; ++handle) {
    bool request_reset = false;
    bool became_stuck = false;
    bool power_cycle_candidate = false;
    uint8_t attempt = 0;
    int64_t episode_since = 0;
    char broadcast_code[kBroadcastCodeSize] = {0};
    {
      lock_guard<mutex> lock(link_stat_lock_[handle]);
      LinkStat &s = link_stat_[handle];
      if (s.connect_since_ns != 0 || s.broadcast_only_since_ns == 0 ||
          s.last_broadcast_ns == 0) {
        continue;
      }
      if (now - s.last_broadcast_ns > kBroadcastEpisodeGapNs) {
        /** A never-connected lidar has no SDK disconnect edge. Expire only the
         *  live episode here; cumulative stuck/reset/power evidence remains. */
        s.broadcast_only_since_ns = 0;
        s.handshake_state = kHandshakeLinkIdle;
        s.handshake_reset_attempts = 0;
        s.handshake_last_reset_try_ns = 0;
        s.handshake_reset_phase = kHandshakeResetNone;
        s.handshake_reset_accepted = false;
        s.handshake_reset_completed = false;
        s.handshake_reset_completed_ns = 0;
        s.handshake_last_network_error_ns = 0;
        s.power_cycle_required_counted_this_episode = false;
        if (s.power_cycle_reason == kPowerCycleReasonHandshakeStuck) {
          s.power_cycle_reason = kPowerCycleReasonNone;
        }
        continue;
      }
      if (s.handshake_state == kHandshakeLinkPowerCycleRequired) {
        continue;
      }

      episode_since = s.broadcast_only_since_ns;
      int64_t episode_age = now - episode_since;
      strncpy(broadcast_code, s.broadcast_code, sizeof(broadcast_code) - 1);

      if (s.handshake_state == kHandshakeLinkBroadcastOnly &&
          episode_age >= kHandshakeFirstResetNs) {
        s.handshake_state = kHandshakeLinkStuck;
        s.handshake_stuck_count++;
        s.handshake_stuck_wall_s = static_cast<int64_t>(time(nullptr));
        became_stuck = true;
      }

      if (IsPowerCycleEscalationReady(s, now, episode_since)) {
        /** This is only a candidate.  Do not mutate the state or cumulative
         *  count until the final, handle+episode-locked commit below. */
        power_cycle_candidate = true;
      } else if (enable_recovery &&
                 episode_age >= kHandshakeFirstResetNs &&
                 s.handshake_reset_attempts < kHandshakeResetMaxAttempts) {
        /** Reserve the attempt under the lock before calling into the SDK, so
         *  two timer callbacks can never issue the same recovery attempt. */
        s.handshake_last_reset_try_ns = now;
        s.handshake_reset_attempts++;
        s.handshake_reset_phase = kHandshakeResetRequested;
        s.handshake_reset_accepted = false;
        s.handshake_reset_completed = false;
        s.handshake_reset_completed_ns = 0;
        attempt = s.handshake_reset_attempts;
        request_reset = true;
      }
    }

    if (became_stuck) {
      PrintLidarEvent(handle, broadcast_code, "HANDSHAKE_STUCK");
      HealthLogger::Get().LogEvent(handle, broadcast_code, "HANDSHAKE_STUCK",
                                   "broadcast alive; no connection for 5s");
    }
    if (power_cycle_candidate) {
      const int64_t commit_now =
          std::chrono::steady_clock::now().time_since_epoch().count();
      lock_guard<mutex> lock(link_stat_lock_[handle]);
      LinkStat &s = link_stat_[handle];
      if (IsPowerCycleEscalationReady(s, commit_now, episode_since)) {
        /** Count only this committed edge.  Keep the lock through external
         *  logging so a concurrent NETWORK_ERROR cannot cancel the episode
         *  and then be followed by a stale POWER_CYCLE_REQUIRED record. */
        /** Sustained live broadcasts plus a completed session reset now make
         *  handshake recovery the sole cause. Drop any still-observing wake
         *  token so the live relay-manager frame cannot carry evidence from
         *  two different recovery reasons. */
        ClearWakeRecoveryState(&s);
        ClearNormalDropoutState(&s);
        s.handshake_state = kHandshakeLinkPowerCycleRequired;
        s.power_cycle_reason = kPowerCycleReasonHandshakeStuck;
        s.power_cycle_required_count++;
        if (!s.power_cycle_required_counted_this_episode) {
          s.power_cycle_required_episode_count++;
          s.handshake_power_cycle_episode_count++;
          s.power_cycle_required_counted_this_episode = true;
        }
        s.power_cycle_required_wall_s = static_cast<int64_t>(time(nullptr));
        attempt = s.handshake_reset_attempts;
        strncpy(broadcast_code, s.broadcast_code,
                sizeof(broadcast_code) - 1);
        broadcast_code[sizeof(broadcast_code) - 1] = '\0';
        long long elapsed_s =
            (commit_now - episode_since) / 1000000000LL;
        char detail[96];
        snprintf(detail, sizeof(detail),
                 "broadcast alive; %u session reset request(s); stuck %llds",
                 attempt, elapsed_s);
        PrintLidarEvent(handle, broadcast_code, "POWER_CYCLE_REQUIRED");
        HealthLogger::Get().LogEvent(handle, broadcast_code,
                                     "POWER_CYCLE_REQUIRED", detail);
        printf("[LivoxRecover] Lidar[%d][%s] %s; physical power cycle "
               "required\n", handle, broadcast_code, detail);
      }
    }
    if (!request_reset) {
      continue;
    }

    livox_status status = ResetLidarHandshakeSession(broadcast_code);
    int64_t wall_now = static_cast<int64_t>(time(nullptr));
    {
      lock_guard<mutex> lock(link_stat_lock_[handle]);
      LinkStat &s = link_stat_[handle];
      /** The SDK call can outlive this handle's physical identity.  Attribute
       *  its result only when the slot still belongs to the broadcast code
       *  which issued it; otherwise an old reset would pollute the new lidar's
       *  process history.  A same-device successful connect is still valid
       *  history even though it has already cleared the live episode. */
      const bool same_identity =
          s.broadcast_code[0] != '\0' &&
          strncmp(s.broadcast_code, broadcast_code,
                  sizeof(s.broadcast_code)) == 0;
      if (same_identity) {
        s.handshake_last_reset_wall_s = wall_now;
        if (status == kStatusSuccess) {
          s.handshake_reset_count++;
        } else {
          s.handshake_reset_fail_count++;
          /** The one-shot budget counts API requests, not accepted cleanups.
           *  Do not decrement it here: repeatedly calling a rejected cleanup
           *  would create the retry loop this watchdog is intended to bound. */
        }
      }
      /** Bind the synchronous API result to the episode which issued it. A
       *  connect or broadcast gap may have started/cleared another episode
       *  while the SDK call was in progress. */
      if (same_identity && s.connect_since_ns == 0 &&
          s.broadcast_only_since_ns == episode_since) {
        s.handshake_reset_accepted = (status == kStatusSuccess);
        if (status == kStatusSuccess) {
          /** A synchronous SDK RESET callback may already have advanced this
           *  request to Completed; never move that phase backwards. */
          if (s.handshake_reset_phase == kHandshakeResetRequested) {
            s.handshake_reset_phase = kHandshakeResetQueued;
          }
        } else {
          s.handshake_reset_phase = kHandshakeResetRejected;
          s.handshake_reset_completed = false;
          s.handshake_reset_completed_ns = 0;
        }
      }
    }
    char detail[80];
    snprintf(detail, sizeof(detail), "session reset %u/%u returned %d", attempt,
             kHandshakeResetMaxAttempts, status);
    printf("[LivoxRecover] Lidar[%d][%s] %s\n", handle, broadcast_code, detail);
    HealthLogger::Get().LogEvent(handle, broadcast_code, "HANDSHAKE_RESET",
                                 detail);
  }
}

namespace {

LidarState ModeToState(LidarMode mode) {
  switch (mode) {
    case kLidarModeNormal:
      return kLidarStateNormal;
    case kLidarModePowerSaving:
      return kLidarStatePowerSaving;
    case kLidarModeStandby:
      return kLidarStateStandBy;
    default:
      return kLidarStateUnknown;
  }
}

bool ShouldWaitForReconnect(livox_status status) {
  return status == kStatusTimeout || status == kStatusNotConnected ||
         status == kStatusSendFailed || status == kStatusInvalidHandle ||
         status == kStatusChannelNotExist;
}

}  // namespace

/** Const varible ------------------------------------------------------------*/
/** For callback use only */
LdsLidar *g_lds_ldiar = nullptr;

/** Global function for common use -------------------------------------------*/

/** Lds lidar function -------------------------------------------------------*/
LdsLidar::LdsLidar(uint32_t interval_ms)
    : Lds(interval_ms, kSourceRawLidar),
      auto_connect_mode_(true),
      whitelist_count_(0),
      is_initialized_(false),
      enable_timesync_(false),
      timesync_(nullptr),
      timesync_config_(),
      next_mode_request_id_(0),
      next_mode_command_id_(0) {
  memset(broadcast_code_whitelist_, 0, sizeof(broadcast_code_whitelist_));
  for (auto &generation : connection_generation_) {
    generation.store(0, std::memory_order_relaxed);
  }

  ResetLdsLidar();
}

LdsLidar::~LdsLidar() {}

void LdsLidar::ResetLdsLidar(void) {
  ResetLds(kSourceRawLidar);

  lock_guard<mutex> lock(mode_mutex_);
  for (auto &request : mode_requests_) {
    request = ModeChangeRequest();
  }
}

livox_status LdsLidar::RequestLidarModeChange(const char *broadcast_code,
                                              LidarMode mode) {
  if (broadcast_code == nullptr) {
    return kStatusFailure;
  }

  uint8_t handle = 0;
  livox_status status = AddLidarToConnect(broadcast_code, &handle);
  if (status != kStatusSuccess) {
    return status;
  }

  SetDataCallback(handle, OnLidarDataCb, (void *)this);
  RememberBroadcastCode(handle, broadcast_code);
  return RequestLidarModeChange(handle, mode);
}

livox_status LdsLidar::RequestLidarModeChange(uint8_t handle, LidarMode mode) {
  return RequestLidarModeChange(handle, mode, 0);
}

livox_status LdsLidar::RequestLidarModeChange(uint8_t handle, LidarMode mode,
                                              uint32_t delay_ms) {
  int64_t not_before_ns = 0;
  if (delay_ms != 0) {
    not_before_ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            (std::chrono::steady_clock::now() +
             std::chrono::milliseconds(delay_ms))
                .time_since_epoch())
            .count();
  }
  return SendModeChangeRequest(handle, mode, false, 0, 0, 0,
                               not_before_ns);
}

livox_status LdsLidar::RequestLidarReboot(uint8_t handle, uint16_t timeout_ms) {
  return RequestLidarRebootImpl(handle, timeout_ms, false, false);
}

livox_status LdsLidar::RequestLidarRebootIfModeIdle(
    uint8_t handle, uint16_t timeout_ms) {
  return RequestLidarRebootImpl(handle, timeout_ms, true, false);
}

livox_status LdsLidar::RequestNetworkLidarReboot(uint8_t handle,
                                                  uint16_t timeout_ms) {
  return RequestLidarRebootImpl(handle, timeout_ms, true, true);
}

livox_status LdsLidar::RequestLidarRebootImpl(
    uint8_t handle, uint16_t timeout_ms, bool require_mode_idle,
    bool network_reboot) {
  if (handle >= kMaxLidarCount) {
    return kStatusInvalidHandle;
  }
  /** Keep connection validation and SDK enqueue in the same per-handle
   *  session transaction. Otherwise a disconnect/reconnect can reuse this
   *  handle between the check and RebootDevice(), rebooting the new session. */
  lock_guard<mutex> send_lock(mode_send_mutex_[handle]);
  if (require_mode_idle) {
    lock_guard<mutex> lock(mode_mutex_);
    if (mode_requests_[handle].active) {
      return kStatusFailure;
    }
  }
  uint64_t reboot_generation = 0;
  {
    lock_guard<mutex> lock(data_lock_[handle]);
    LidarDevice *p_lidar = &lidars_[handle];
    if (p_lidar->connect_state == kConnectStateOff ||
        p_lidar->handle != handle) {
      return kStatusNotConnected;
    }
    reboot_generation = connection_generation_[handle].load(
        std::memory_order_acquire);
  }
  {
    lock_guard<mutex> lock(link_stat_lock_[handle]);
    LinkStat &s = link_stat_[handle];
    s.planned_reboot_generation = reboot_generation;
    if (network_reboot) {
      s.network_soft_reboot_generation = reboot_generation;
      s.network_soft_reboot_inflight = true;
      s.network_soft_reboot_ack = false;
      s.network_soft_reboot_command_accepted = false;
      s.network_soft_reboot_disconnect = false;
      s.network_soft_reboot_reconnected = false;
      s.network_soft_reboot_settle_deadline_ns = 0;
      s.network_soft_reboot_status = 0;
      s.network_soft_reboot_response = 0;
      s.network_recovery_state = kNetworkRecoverySoftRebootVerifying;
    }
  }
  /** Arm the competing-cause marker before entering the SDK, so even a
   *  synchronous disconnect cannot be attributed to the earlier wake. A
   *  definite synchronous enqueue rejection, however, leaves the original
   *  wake evidence intact because no reboot command was accepted. */
  const livox_status status =
      RebootDevice(handle, timeout_ms, RebootCb, this);
  {
    lock_guard<mutex> lock(link_stat_lock_[handle]);
    LinkStat &s = link_stat_[handle];
    if (s.planned_reboot_generation == reboot_generation) {
      s.planned_reboot_generation = 0;
      if (status == kStatusSuccess) {
        if (!network_reboot) {
          ClearWakeRecoveryState(&s);
          ClearNormalDropoutState(&s);
        }
      }
    }
    if (network_reboot && s.network_soft_reboot_generation == reboot_generation &&
        status != kStatusSuccess) {
      s.network_soft_reboot_inflight = false;
      s.network_soft_reboot_command_accepted = false;
      s.network_soft_reboot_settle_deadline_ns = 0;
      s.network_soft_reboot_status = status;
      s.network_recovery_state = kNetworkRecoveryWaiting;
    }
  }
  return status;
}

void LdsLidar::ApplyNetworkHealthJson(const std::string &json) {
  rapidjson::Document doc;
  doc.Parse(json.c_str());
  if (doc.HasParseError() || !doc.IsObject() ||
      !doc.HasMember("type") || !doc["type"].IsString() ||
      std::string(doc["type"].GetString()) != "LIVOX_NETWORK_HEALTH" ||
      !doc.HasMember("devices") || !doc["devices"].IsArray()) {
    return;
  }
  const bool shared = doc.HasMember("shared_network_suspected") &&
                      doc["shared_network_suspected"].IsBool() &&
                      doc["shared_network_suspected"].GetBool();
  uint8_t soft_max_attempts = kNetworkSoftRebootMaxAttempts;
  if (doc.HasMember("soft_reboot_max_attempts") &&
      doc["soft_reboot_max_attempts"].IsUint()) {
    const unsigned value = doc["soft_reboot_max_attempts"].GetUint();
    if (value >= 3 && value <= 10) {
      soft_max_attempts = static_cast<uint8_t>(value);
    }
  }
  int64_t soft_interval_ns = kNetworkSoftRebootRetryNs;
  if (doc.HasMember("soft_reboot_interval_seconds") &&
      doc["soft_reboot_interval_seconds"].IsNumber()) {
    soft_interval_ns = SecondsToNs(
        doc["soft_reboot_interval_seconds"].GetDouble(),
        kNetworkSoftRebootRetryNs);
    if (soft_interval_ns < 2000000000LL || soft_interval_ns > 30000000000LL) {
      soft_interval_ns = kNetworkSoftRebootRetryNs;
    }
  }
  int64_t soft_ack_timeout_ns = kNetworkSoftRebootAckTimeoutNs;
  if (doc.HasMember("soft_reboot_ack_timeout_seconds") &&
      doc["soft_reboot_ack_timeout_seconds"].IsNumber()) {
    soft_ack_timeout_ns = SecondsToNs(
        doc["soft_reboot_ack_timeout_seconds"].GetDouble(),
        kNetworkSoftRebootAckTimeoutNs);
    if (soft_ack_timeout_ns < 500000000LL ||
        soft_ack_timeout_ns >= soft_interval_ns) {
      soft_ack_timeout_ns = kNetworkSoftRebootAckTimeoutNs;
    }
  }
  if (soft_ack_timeout_ns >= soft_interval_ns) {
    soft_ack_timeout_ns = std::max<int64_t>(500000000LL, soft_interval_ns / 2);
  }
  int64_t soft_settle_ns = kNetworkSoftRebootSettleNs;
  if (doc.HasMember("soft_reboot_settle_seconds") &&
      doc["soft_reboot_settle_seconds"].IsNumber()) {
    soft_settle_ns = SecondsToNs(
        doc["soft_reboot_settle_seconds"].GetDouble(),
        kNetworkSoftRebootSettleNs);
    if (soft_settle_ns < 15000000000LL ||
        soft_settle_ns > 120000000000LL) {
      soft_settle_ns = kNetworkSoftRebootSettleNs;
    }
  }
  const int64_t soft_deadline_ns =
      soft_settle_ns * static_cast<int64_t>(soft_max_attempts);
  const int64_t now =
      std::chrono::steady_clock::now().time_since_epoch().count();
  for (rapidjson::SizeType i = 0; i < doc["devices"].Size(); ++i) {
    const rapidjson::Value &row = doc["devices"][i];
    if (!row.IsObject() || !row.HasMember("broadcast_code") ||
        !row["broadcast_code"].IsString()) {
      continue;
    }
    const char *code = row["broadcast_code"].GetString();
    int handle = -1;
    // The SDK handle is a transient slot and may be reassigned after a
    // reconnect or Driver restart.  Network identity is broadcast_code only;
    // resolve the current handle from the live Driver state every frame.
    for (uint8_t h = 0; h < kMaxLidarCount; ++h) {
      lock_guard<mutex> lock(link_stat_lock_[h]);
      if (strncmp(link_stat_[h].broadcast_code, code,
                  sizeof(link_stat_[h].broadcast_code)) == 0) {
        handle = h;
        break;
      }
    }
    if (handle < 0 || handle >= kMaxLidarCount) {
      continue;
    }
    NetworkHealthState state = kNetworkHealthUnknown;
    if (row.HasMember("state") && row["state"].IsString()) {
      const std::string state_text = row["state"].GetString();
      if (state_text == "NET_OK") {
        state = kNetworkHealthOk;
      } else if (state_text == "NET_DEGRADED") {
        state = kNetworkHealthDegraded;
      } else if (state_text == "NET_UNSTABLE") {
        state = kNetworkHealthUnstable;
      } else if (state_text == "NET_UNREACHABLE") {
        state = kNetworkHealthUnreachable;
      }
    }
    lock_guard<mutex> lock(link_stat_lock_[handle]);
    LinkStat &s = link_stat_[handle];
    if (strncmp(s.broadcast_code, code, sizeof(s.broadcast_code)) != 0) {
      continue;
    }
    s.network_health_seen = true;
    s.network_shared_suspected = shared;
    s.network_soft_reboot_max_attempts = soft_max_attempts;
    s.network_soft_reboot_interval_ns = soft_interval_ns;
    s.network_soft_reboot_ack_timeout_ns = soft_ack_timeout_ns;
    s.network_soft_reboot_settle_ns = soft_settle_ns;
    s.network_soft_recovery_deadline_ns = soft_deadline_ns;
    s.network_health_state = state;
    s.network_last_health_ns = now;
    if (row.HasMember("window_samples") && row["window_samples"].IsUint()) {
      s.network_window_samples = row["window_samples"].GetUint();
    }
    if (row.HasMember("window_failures") && row["window_failures"].IsUint()) {
      s.network_window_failures = row["window_failures"].GetUint();
    }
    if (row.HasMember("consecutive_failures") &&
        row["consecutive_failures"].IsUint()) {
      s.network_consecutive_failures = row["consecutive_failures"].GetUint();
    }
    if (row.HasMember("consecutive_successes") &&
        row["consecutive_successes"].IsUint()) {
      s.network_consecutive_successes = row["consecutive_successes"].GetUint();
    }
    if (row.HasMember("loss_percent") && row["loss_percent"].IsNumber()) {
      s.network_loss_percent = row["loss_percent"].GetDouble();
    }
    if (row.HasMember("rtt_ms") && row["rtt_ms"].IsNumber()) {
      s.network_rtt_ms = row["rtt_ms"].GetDouble();
    }
    if (row.HasMember("success") && row["success"].IsBool() &&
        row["success"].GetBool()) {
      s.network_last_success_ns = now;
    }
  }
}

void LdsLidar::TickNetworkRecovery(bool enable_recovery) {
  const int64_t now =
      std::chrono::steady_clock::now().time_since_epoch().count();
  for (uint8_t handle = 0; handle < kMaxLidarCount; ++handle) {
    bool request_reboot = false;
    bool commit_power = false;
    uint64_t expected_generation = 0;
    uint8_t attempt_number = 0;
    uint8_t max_attempts = kNetworkSoftRebootMaxAttempts;
    int64_t episode_since = 0;
    char broadcast_code[kBroadcastCodeSize] = {0};
    {
      lock_guard<mutex> lock(link_stat_lock_[handle]);
      LinkStat &s = link_stat_[handle];
      if (!s.network_health_seen ||
          now - s.network_last_health_ns > kNetworkHealthStaleNs) {
        if (s.network_recovery_state != kNetworkRecoveryPowerCycleRequired) {
          s.network_health_state = kNetworkHealthUnknown;
          s.network_recovery_state = kNetworkRecoveryIdle;
          s.network_soft_reboot_attempts = 0;
          s.network_soft_reboot_episode_ns = 0;
          s.network_soft_reboot_episode_wall_s = 0;
          s.network_soft_reboot_last_try_ns = 0;
          s.network_soft_reboot_last_try_wall_s = 0;
          s.network_soft_reboot_inflight = false;
        }
        continue;
      }
      const bool bad = s.network_health_state == kNetworkHealthUnstable ||
                       s.network_health_state == kNetworkHealthUnreachable;
      if (!bad) {
        if (s.network_health_state == kNetworkHealthOk &&
            s.network_consecutive_successes >= 5 &&
            (s.network_recovery_state != kNetworkRecoveryIdle ||
             s.network_soft_reboot_attempts != 0 ||
             s.network_soft_reboot_episode_ns != 0 ||
             s.network_soft_reboot_last_try_ns != 0 ||
             s.power_cycle_reason ==
                 kPowerCycleReasonNetworkRecoveryExhausted)) {
          /** NET_OK is authoritative after the configured healthy-success
           *  confirmation, including when the last soft reboot had already
           *  latched POWER_CYCLE_REQUIRED.  Clear the live latch so the
           *  recovery topic remains self-consistent and the manager can
           *  archive a recovered request without attempting OFF/ON. */
          ClearRecoveredNetworkRecoveryState(&s);
        }
        continue;
      }
      if (s.network_soft_reboot_episode_ns == 0) {
        s.network_soft_reboot_episode_ns = now;
        s.network_soft_reboot_episode_wall_s = static_cast<int64_t>(time(nullptr));
        s.network_health_since_ns = now;
        s.network_recovery_state = kNetworkRecoveryWaiting;
      }
      episode_since = s.network_soft_reboot_episode_ns;
      strncpy(broadcast_code, s.broadcast_code, sizeof(broadcast_code) - 1);
      if (s.network_recovery_state == kNetworkRecoveryPowerCycleRequired) {
        continue;
      }
      if (s.network_soft_reboot_inflight &&
          now - s.network_soft_reboot_last_try_ns >=
              s.network_soft_reboot_ack_timeout_ns) {
        s.network_soft_reboot_inflight = false;
        s.network_soft_reboot_command_accepted = false;
        s.network_soft_reboot_settle_deadline_ns = 0;
        s.network_recovery_state = kNetworkRecoveryWaiting;
      }
      /** A successful reboot ACK means the device accepted the command; it
       *  does not mean the post-reboot handshake is complete.  Hold off on
       *  another software reboot until the configured settle deadline. */
      const bool reboot_settling =
          s.network_soft_reboot_command_accepted &&
          s.network_soft_reboot_settle_deadline_ns != 0 &&
          now < s.network_soft_reboot_settle_deadline_ns;
      if (reboot_settling) {
        continue;
      }
      if (s.network_soft_reboot_settle_deadline_ns != 0) {
        s.network_soft_reboot_settle_deadline_ns = 0;
        s.network_soft_reboot_command_accepted = false;
      }
      const bool deadline =
          now - episode_since >= s.network_soft_recovery_deadline_ns;
      if (enable_recovery && deadline &&
          s.network_soft_reboot_attempts >=
              s.network_soft_reboot_max_attempts) {
        commit_power = true;
      } else if (enable_recovery && !s.network_soft_reboot_inflight &&
                 s.network_soft_reboot_attempts <
                     s.network_soft_reboot_max_attempts &&
                 (s.network_soft_reboot_last_try_ns == 0 ||
                  now - s.network_soft_reboot_last_try_ns >=
                      s.network_soft_reboot_interval_ns)) {
        s.network_soft_reboot_attempts++;
        s.network_soft_reboot_last_try_ns = now;
        s.network_soft_reboot_last_try_wall_s = static_cast<int64_t>(time(nullptr));
        s.network_soft_reboot_generation =
            connection_generation_[handle].load(std::memory_order_acquire);
        s.network_soft_reboot_inflight = true;
        s.network_soft_reboot_ack = false;
        s.network_soft_reboot_disconnect = false;
        s.network_soft_reboot_reconnected = false;
        s.network_recovery_state = kNetworkRecoverySoftRebootPending;
        expected_generation = s.network_soft_reboot_generation;
        attempt_number = s.network_soft_reboot_attempts;
        max_attempts = s.network_soft_reboot_max_attempts;
        request_reboot = true;
      }
    }
    if (commit_power) {
      const int64_t commit_now =
          std::chrono::steady_clock::now().time_since_epoch().count();
      bool committed = false;
      uint8_t committed_max_attempts = kNetworkSoftRebootMaxAttempts;
      {
        lock_guard<mutex> lock(link_stat_lock_[handle]);
        LinkStat &s = link_stat_[handle];
        if (s.network_recovery_state != kNetworkRecoveryPowerCycleRequired &&
            s.network_soft_reboot_episode_ns == episode_since &&
            (s.network_health_state == kNetworkHealthUnstable ||
             s.network_health_state == kNetworkHealthUnreachable) &&
            s.network_soft_reboot_attempts >=
                s.network_soft_reboot_max_attempts &&
            commit_now - episode_since >= s.network_soft_recovery_deadline_ns) {
          s.network_recovery_state = kNetworkRecoveryPowerCycleRequired;
          s.power_cycle_reason = kPowerCycleReasonNetworkRecoveryExhausted;
          s.power_cycle_required_count++;
          if (!s.power_cycle_required_counted_this_episode) {
            s.power_cycle_required_episode_count++;
            s.power_cycle_required_counted_this_episode = true;
          }
          s.power_cycle_required_wall_s = static_cast<int64_t>(time(nullptr));
          committed_max_attempts = s.network_soft_reboot_max_attempts;
          committed = true;
          strncpy(broadcast_code, s.broadcast_code,
                  sizeof(broadcast_code) - 1);
        }
      }
      if (committed) {
        PrintLidarEvent(handle, broadcast_code, "NETWORK_POWER_CYCLE_REQUIRED");
        HealthLogger::Get().LogEvent(
            handle, broadcast_code, "POWER_CYCLE_REQUIRED",
            "reason=NETWORK_RECOVERY_EXHAUSTED; configured soft-reboot "
            "budget did not restore the configured network window");
        printf("[LivoxRecover] Lidar[%d][%s] network unstable after %u soft "
               "reboots; physical group power cycle required\n",
               handle, broadcast_code,
               static_cast<unsigned>(committed_max_attempts));
      }
      continue;
    }
    if (!request_reboot) {
      continue;
    }
    livox_status status = RequestNetworkLidarReboot(handle);
    if (status != kStatusSuccess) {
      lock_guard<mutex> lock(link_stat_lock_[handle]);
      LinkStat &s = link_stat_[handle];
      if (s.network_soft_reboot_generation == expected_generation) {
        s.network_soft_reboot_inflight = false;
        s.network_soft_reboot_status = status;
        s.network_recovery_state = kNetworkRecoveryWaiting;
      }
    }
    char detail[128];
    snprintf(detail, sizeof(detail), "network soft reboot %u/%u returned %d",
             static_cast<unsigned>(attempt_number),
             static_cast<unsigned>(max_attempts), status);
    printf("[LivoxRecover] Lidar[%d][%s] %s\n", handle, broadcast_code,
           detail);
    HealthLogger::Get().LogEvent(handle, broadcast_code, "NETWORK_REBOOT",
                                 detail);
  }
}

livox_status LdsLidar::RequestRestartSampling(uint8_t handle) {
  if (handle >= kMaxLidarCount) {
    return kStatusInvalidHandle;
  }
  {
    lock_guard<mutex> lock(data_lock_[handle]);
    if (lidars_[handle].connect_state == kConnectStateOff ||
        lidars_[handle].handle != handle) {
      return kStatusNotConnected;
    }
  }
  /** Re-issue start-sampling; harmless if already sampling, and recovers a
   *  lidar that is Normal/connected but stopped producing point cloud. */
  return SendStartSampling(handle);
}

std::shared_ptr<LdsLidar::AsyncCommandContext>
LdsLidar::CreateCommandContext(uint8_t handle,
                               uint64_t connection_generation,
                               uint8_t retry_count,
                               AsyncCommandKind kind,
                               LidarMode mode,
                               uint64_t mode_request_id,
                               uint64_t mode_command_id) {
  std::shared_ptr<AsyncCommandContext> context(
      new AsyncCommandContext(handle, connection_generation, retry_count,
                              kind, mode, mode_request_id, mode_command_id));
  lock_guard<mutex> lock(command_context_mutex_);
  command_contexts_.push_back(context);
  return context;
}

void LdsLidar::MarkCommandContextCompleted(
    const std::shared_ptr<AsyncCommandContext> &context) {
  if (!context) {
    return;
  }
  lock_guard<mutex> lock(command_context_mutex_);
  bool expected = false;
  if (context->completed.compare_exchange_strong(
          expected, true, std::memory_order_acq_rel,
          std::memory_order_acquire)) {
    context->completed_at = std::chrono::steady_clock::now();
  }
}

std::shared_ptr<LdsLidar::AsyncCommandContext>
LdsLidar::AcquireCommandContext(void *raw_context,
                                AsyncCommandKind expected_kind) {
  if (raw_context == nullptr) {
    return std::shared_ptr<AsyncCommandContext>();
  }

  lock_guard<mutex> lock(command_context_mutex_);
  for (const auto &context : command_contexts_) {
    if (context.get() != raw_context) {
      continue;
    }
    if (context->kind != expected_kind) {
      return std::shared_ptr<AsyncCommandContext>();
    }
    /** The SDK must complete or cancel every accepted command exactly once.
     *  Still claim the context atomically so a malformed duplicate callback
     *  cannot execute Driver state transitions twice. */
    bool expected = false;
    if (!context->completed.compare_exchange_strong(
            expected, true, std::memory_order_acq_rel,
            std::memory_order_acquire)) {
      return std::shared_ptr<AsyncCommandContext>();
    }
    context->completed_at = std::chrono::steady_clock::now();
    return context;
  }
  return std::shared_ptr<AsyncCommandContext>();
}

void LdsLidar::ReapCommandContexts() {
  /** Keep completed entries briefly. Besides making diagnostics easier, this
   *  prevents a late duplicate callback from matching a newly allocated
   *  context at the same address when an older SDK is used accidentally. */
  const auto cutoff = std::chrono::steady_clock::now() -
                      std::chrono::seconds(60);
  lock_guard<mutex> lock(command_context_mutex_);
  for (auto it = command_contexts_.begin(); it != command_contexts_.end();) {
    const std::shared_ptr<AsyncCommandContext> &context = *it;
    if (context->completed.load(std::memory_order_acquire) &&
        context->completed_at != std::chrono::steady_clock::time_point() &&
        context->completed_at <= cutoff) {
      it = command_contexts_.erase(it);
    } else {
      ++it;
    }
  }
}

livox_status LdsLidar::SendStartSampling(uint8_t handle,
                                         uint64_t expected_generation) {
  if (handle >= kMaxLidarCount) {
    return kStatusInvalidHandle;
  }

  lock_guard<mutex> send_lock(mode_send_mutex_[handle]);
  {
    /** Close the timer/service race: the no-data watchdog can decide to
     *  restart sampling just before another thread publishes a sleep request.
     *  Re-check under the same send lock used by mode enqueue so StartSampling
     *  can never be queued after an active PowerSaving/Standby command. */
    lock_guard<mutex> lock(mode_mutex_);
    const ModeChangeRequest &request = mode_requests_[handle];
    if (request.active && request.desired_mode != kLidarModeNormal) {
      return kStatusFailure;
    }
  }
  uint64_t generation = 0;
  {
    lock_guard<mutex> lock(data_lock_[handle]);
    const LidarDevice &lidar = lidars_[handle];
    if (lidar.handle != handle ||
        lidar.connect_state == kConnectStateOff) {
      return kStatusNotConnected;
    }
    generation = connection_generation_[handle].load(
        std::memory_order_acquire);
    if (expected_generation != 0 && expected_generation != generation) {
      return kStatusNotConnected;
    }
  }

  std::shared_ptr<AsyncCommandContext> context =
      CreateCommandContext(handle, generation, 0,
                           kAsyncStartSamplingCommand);
  livox_status status =
      LidarStartSampling(handle, StartSampleCb, context.get());
  if (status != kStatusSuccess) {
    /** A synchronous rejection registers no callback. Reaping is deliberately
     *  deferred so an SDK that violates that contract still cannot cause a
     *  quick address-reuse ABA. */
    MarkCommandContextCompleted(context);
  }
  return status;
}

bool LdsLidar::IsCurrentConfigContext(
    uint8_t handle, uint64_t connection_generation) {
  if (handle >= kMaxLidarCount) {
    return false;
  }
  lock_guard<mutex> lock(data_lock_[handle]);
  const LidarDevice &lidar = lidars_[handle];
  return lidar.handle == handle &&
         lidar.connect_state == kConnectStateConfig &&
          connection_generation == connection_generation_[handle].load(
                                       std::memory_order_acquire);
}

livox_status LdsLidar::SendNextConfigCommand(
    uint8_t handle, uint64_t connection_generation) {
  if (handle >= kMaxLidarCount) {
    return kStatusInvalidHandle;
  }

  uint32_t pending_bits = 0;
  {
    lock_guard<mutex> lock(data_lock_[handle]);
    const LidarDevice &lidar = lidars_[handle];
    if (lidar.handle != handle ||
        lidar.connect_state != kConnectStateConfig ||
        connection_generation != connection_generation_[handle].load(
                                     std::memory_order_acquire)) {
      return kStatusNotConnected;
    }
    pending_bits = lidar.config.set_bits;
  }

  /** One command per handle at a time. Each successful callback clears its bit
   *  and calls this function for the next stage; timeout retries stay inside
   *  the same stage. Never call a Send* function while holding data/config
   *  locks because SDK callbacks can complete on another thread. */
  if (pending_bits & kConfigCoordinate) {
    printf("Lidar[%d] config pipeline -> coordinate\n", handle);
    return SendCoordinateConfig(handle, 0, connection_generation);
  }
  if (pending_bits & kConfigReturnMode) {
    printf("Lidar[%d] config pipeline -> return mode\n", handle);
    return SendReturnModeConfig(handle, 0, connection_generation);
  }
  if (pending_bits & kConfigImuRate) {
    printf("Lidar[%d] config pipeline -> IMU rate\n", handle);
    return SendImuRateConfig(handle, 0, connection_generation);
  }
  if (pending_bits & kConfigGetExtrinsicParameter) {
    printf("Lidar[%d] config pipeline -> extrinsic\n", handle);
    return SendExtrinsicConfig(handle, 0, connection_generation);
  }
  if (pending_bits & kConfigSetHighSensitivity) {
    printf("Lidar[%d] config pipeline -> sensitivity\n", handle);
    return SendHighSensitivityConfig(handle, 0, connection_generation);
  }
  return pending_bits == 0 ? kStatusSuccess : kStatusFailure;
}

livox_status LdsLidar::SendCoordinateConfig(uint8_t handle,
                                            uint8_t retry_count,
                                            uint64_t expected_generation) {
  if (handle >= kMaxLidarCount) {
    return kStatusInvalidHandle;
  }
  lock_guard<mutex> send_lock(mode_send_mutex_[handle]);
  uint64_t generation = 0;
  bool spherical = false;
  {
    lock_guard<mutex> lock(data_lock_[handle]);
    LidarDevice &lidar = lidars_[handle];
    if (lidar.handle != handle ||
        lidar.connect_state != kConnectStateConfig) {
      return kStatusNotConnected;
    }
    generation = connection_generation_[handle].load(
        std::memory_order_acquire);
    if (expected_generation != 0 && expected_generation != generation) {
      return kStatusNotConnected;
    }
    spherical = (lidar.config.coordinate != 0);
  }
  std::shared_ptr<AsyncCommandContext> context =
      CreateCommandContext(handle, generation, retry_count);
  livox_status status = spherical
                            ? SetSphericalCoordinate(
                                  handle, SetCoordinateCb, context.get())
                            : SetCartesianCoordinate(
                                  handle, SetCoordinateCb, context.get());
  if (status != kStatusSuccess) {
    MarkCommandContextCompleted(context);
  }
  return status;
}

livox_status LdsLidar::SendReturnModeConfig(uint8_t handle,
                                            uint8_t retry_count,
                                            uint64_t expected_generation) {
  if (handle >= kMaxLidarCount) {
    return kStatusInvalidHandle;
  }
  lock_guard<mutex> send_lock(mode_send_mutex_[handle]);
  uint64_t generation = 0;
  uint32_t return_mode = 0;
  {
    lock_guard<mutex> lock(data_lock_[handle]);
    LidarDevice &lidar = lidars_[handle];
    if (lidar.handle != handle ||
        lidar.connect_state != kConnectStateConfig) {
      return kStatusNotConnected;
    }
    generation = connection_generation_[handle].load(
        std::memory_order_acquire);
    if (expected_generation != 0 && expected_generation != generation) {
      return kStatusNotConnected;
    }
    return_mode = lidar.config.return_mode;
  }
  std::shared_ptr<AsyncCommandContext> context =
      CreateCommandContext(handle, generation, retry_count);
  livox_status status = LidarSetPointCloudReturnMode(
      handle, static_cast<PointCloudReturnMode>(return_mode),
      SetPointCloudReturnModeCb, context.get());
  if (status != kStatusSuccess) {
    MarkCommandContextCompleted(context);
  }
  return status;
}

livox_status LdsLidar::SendImuRateConfig(uint8_t handle,
                                         uint8_t retry_count,
                                         uint64_t expected_generation) {
  if (handle >= kMaxLidarCount) {
    return kStatusInvalidHandle;
  }
  lock_guard<mutex> send_lock(mode_send_mutex_[handle]);
  uint64_t generation = 0;
  uint32_t imu_rate = 0;
  {
    lock_guard<mutex> lock(data_lock_[handle]);
    LidarDevice &lidar = lidars_[handle];
    if (lidar.handle != handle ||
        lidar.connect_state != kConnectStateConfig) {
      return kStatusNotConnected;
    }
    generation = connection_generation_[handle].load(
        std::memory_order_acquire);
    if (expected_generation != 0 && expected_generation != generation) {
      return kStatusNotConnected;
    }
    imu_rate = lidar.config.imu_rate;
  }
  std::shared_ptr<AsyncCommandContext> context =
      CreateCommandContext(handle, generation, retry_count);
  livox_status status = LidarSetImuPushFrequency(
      handle, static_cast<ImuFreq>(imu_rate),
      SetImuRatePushFrequencyCb, context.get());
  if (status != kStatusSuccess) {
    MarkCommandContextCompleted(context);
  }
  return status;
}

livox_status LdsLidar::SendExtrinsicConfig(uint8_t handle,
                                           uint8_t retry_count,
                                           uint64_t expected_generation) {
  if (handle >= kMaxLidarCount) {
    return kStatusInvalidHandle;
  }
  lock_guard<mutex> send_lock(mode_send_mutex_[handle]);
  uint64_t generation = 0;
  {
    lock_guard<mutex> lock(data_lock_[handle]);
    LidarDevice &lidar = lidars_[handle];
    if (lidar.handle != handle ||
        lidar.connect_state != kConnectStateConfig) {
      return kStatusNotConnected;
    }
    generation = connection_generation_[handle].load(
        std::memory_order_acquire);
    if (expected_generation != 0 && expected_generation != generation) {
      return kStatusNotConnected;
    }
  }
  std::shared_ptr<AsyncCommandContext> context =
      CreateCommandContext(handle, generation, retry_count);
  livox_status status = LidarGetExtrinsicParameter(
      handle, GetLidarExtrinsicParameterCb, context.get());
  if (status != kStatusSuccess) {
    MarkCommandContextCompleted(context);
  }
  return status;
}

livox_status LdsLidar::SendHighSensitivityConfig(uint8_t handle,
                                                  uint8_t retry_count,
                                                  uint64_t expected_generation) {
  if (handle >= kMaxLidarCount) {
    return kStatusInvalidHandle;
  }
  lock_guard<mutex> send_lock(mode_send_mutex_[handle]);
  uint64_t generation = 0;
  bool enable = false;
  {
    lock_guard<mutex> lock(data_lock_[handle]);
    LidarDevice &lidar = lidars_[handle];
    if (lidar.handle != handle ||
        lidar.connect_state != kConnectStateConfig) {
      return kStatusNotConnected;
    }
    generation = connection_generation_[handle].load(
        std::memory_order_acquire);
    if (expected_generation != 0 && expected_generation != generation) {
      return kStatusNotConnected;
    }
    enable = lidar.config.enable_high_sensitivity;
  }
  std::shared_ptr<AsyncCommandContext> context =
      CreateCommandContext(handle, generation, retry_count);
  livox_status status = enable
                            ? LidarEnableHighSensitivity(
                                  handle, SetHighSensitivityCb, context.get())
                            : LidarDisableHighSensitivity(
                                  handle, SetHighSensitivityCb, context.get());
  if (status != kStatusSuccess) {
    MarkCommandContextCompleted(context);
  }
  return status;
}

void LdsLidar::RememberBroadcastCode(uint8_t handle, const char *broadcast_code) {
  if (handle >= kMaxLidarCount || broadcast_code == nullptr ||
      broadcast_code[0] == '\0') {
    return;
  }

  /** Serialize identity replacement with every per-handle SDK command send.
   *  Otherwise a sender could release mode_mutex_, then enqueue an old
   *  device's command after this function had already rebound the handle. */
  lock_guard<mutex> send_lock(mode_send_mutex_[handle]);
  lock_guard<mutex> lock(mode_mutex_);
  ModeChangeRequest &request = mode_requests_[handle];
  const bool identity_changed =
      request.broadcast_code[0] != '\0' &&
      strncmp(request.broadcast_code, broadcast_code,
              sizeof(request.broadcast_code)) != 0;
  if (identity_changed) {
    /** A deferred Normal request belongs to one physical lidar, not to the SDK
     *  handle number.  Drop it when the handle is reassigned so reconnect of a
     *  different lidar cannot execute the previous device's pending command. */
    request = ModeChangeRequest();
  }
  strncpy(request.broadcast_code, broadcast_code,
          sizeof(request.broadcast_code) - 1);
  request.broadcast_code[sizeof(request.broadcast_code) - 1] = '\0';
}

bool LdsLidar::ResetModeRequestIfTarget(uint8_t handle, LidarMode target,
                                        uint64_t expected_request_id,
                                        uint64_t expected_command_id,
                                        uint64_t expected_generation,
                                        bool *measurement_close_eligible) {
  if (handle >= kMaxLidarCount) {
    return false;
  }

  lock_guard<mutex> send_lock(mode_send_mutex_[handle]);
  lock_guard<mutex> lock(mode_mutex_);
  ModeChangeRequest &request = mode_requests_[handle];
  if (!request.active || request.desired_mode != target ||
      (expected_request_id != 0 &&
       request.request_id != expected_request_id) ||
      (expected_command_id != 0 &&
       request.command_id != expected_command_id) ||
      (expected_generation != 0 &&
       expected_generation != connection_generation_[handle].load(
                                  std::memory_order_acquire))) {
    return false;
  }

  char broadcast_code[kBroadcastCodeSize] = {0};
  if (measurement_close_eligible != nullptr) {
    *measurement_close_eligible = request.measurement_close_eligible;
  }
  strncpy(broadcast_code, request.broadcast_code, sizeof(broadcast_code) - 1);
  request = ModeChangeRequest();
  if (broadcast_code[0] != '\0') {
    strncpy(request.broadcast_code, broadcast_code,
            sizeof(request.broadcast_code) - 1);
    request.broadcast_code[sizeof(request.broadcast_code) - 1] = '\0';
  }
  return true;
}

void LdsLidar::RecordMeasurementModeSuccess(uint8_t handle, LidarMode mode,
                                            bool close_was_eligible) {
  if (handle >= kMaxLidarCount) {
    return;
  }
  const int64_t now_ns =
      std::chrono::steady_clock::now().time_since_epoch().count();
  const int64_t now_wall_s = static_cast<int64_t>(time(nullptr));
  bool started = false;
  bool resumed = false;
  bool closed = false;
  bool retained = false;
  uint64_t session_id = 0;
  uint8_t attempts = 0;
  char broadcast_code[kBroadcastCodeSize] = {0};
  {
    lock_guard<mutex> lock(link_stat_lock_[handle]);
    LinkStat &link = link_stat_[handle];
    strncpy(broadcast_code, link.broadcast_code,
            sizeof(broadcast_code) - 1);
    MeasurementSessionState &session = link.measurement_session;
    if (mode == kLidarModeNormal) {
      started = !session.active;
      resumed = session.active && session.paused;
      BeginMeasurementSession(&session, false, now_ns, now_wall_s);
    } else {
      const bool was_active = session.active;
      closed = FinishMeasurementSession(&session, close_was_eligible);
      retained = was_active && !closed;
    }
    if (!session.error_power_cycle_required &&
        link.power_cycle_reason == kPowerCycleReasonErrorRebootExhausted) {
      link.power_cycle_reason = kPowerCycleReasonNone;
    }
    session_id = session.session_id;
    attempts = session.error_reboot_attempts;
  }
  if (started || resumed) {
    char detail[96];
    snprintf(detail, sizeof(detail), "session=%llu; retained-error-reboots=%u",
             static_cast<unsigned long long>(session_id),
             static_cast<unsigned>(attempts));
    HealthLogger::Get().LogEvent(
        handle, broadcast_code,
        started ? "MEASUREMENT_SESSION_STARTED"
                : "MEASUREMENT_SESSION_RESUMED",
        detail);
  } else if (closed) {
    char detail[64];
    snprintf(detail, sizeof(detail), "session=%llu; Error budget cleared",
             static_cast<unsigned long long>(session_id));
    HealthLogger::Get().LogEvent(handle, broadcast_code,
                                 "MEASUREMENT_SESSION_CLOSED", detail);
  } else if (retained) {
    char detail[96];
    snprintf(detail, sizeof(detail),
             "session=%llu; Error budget retained=%u (recovery not confirmed)",
             static_cast<unsigned long long>(session_id),
             static_cast<unsigned>(attempts));
    HealthLogger::Get().LogEvent(handle, broadcast_code,
                                 "MEASUREMENT_SESSION_PAUSED", detail);
  }
}

void LdsLidar::RecordPointCloudPublished(uint8_t handle) {
  if (handle >= kMaxLidarCount) {
    return;
  }
  const int64_t now_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count();
  const int64_t now_wall_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::system_clock::now().time_since_epoch())
          .count();
  lock_guard<mutex> lock(link_stat_lock_[handle]);
  ObservePointCloudPublished(&link_stat_[handle].point_cloud_outage, now_ns,
                             now_wall_ns);
}

void LdsLidar::MarkPointCloudUnexpectedStop(uint8_t handle) {
  if (handle >= kMaxLidarCount) {
    return;
  }
  const int64_t now_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count();
  const int64_t now_wall_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::system_clock::now().time_since_epoch())
          .count();
  lock_guard<mutex> lock(link_stat_lock_[handle]);
  BeginPointCloudOutage(&link_stat_[handle].point_cloud_outage, now_ns,
                        now_wall_ns);
}

void LdsLidar::ExcludePointCloudOutageForPlannedMode(uint8_t handle) {
  if (handle >= kMaxLidarCount) {
    return;
  }
  lock_guard<mutex> lock(link_stat_lock_[handle]);
  ExcludePointCloudOutage(&link_stat_[handle].point_cloud_outage);
}

bool LdsLidar::IsModeTransitionActive(uint8_t handle) {
  if (handle >= kMaxLidarCount) {
    return false;
  }
  lock_guard<mutex> lock(mode_mutex_);
  return mode_requests_[handle].active;
}

uint64_t LdsLidar::GetConnectionGeneration(uint8_t handle) const {
  if (handle >= kMaxLidarCount) {
    return 0;
  }
  return connection_generation_[handle].load(std::memory_order_acquire);
}

namespace {
/** Verify all requested modes from actual heartbeat state. Normal gets a
 *  longer no-ACK retry budget because command delivery can still fail. A
 *  positive Normal ACK is different: Horizon reports response 2 while its
 *  motor is spinning up, and field units need up to about 16 seconds. Re-sending
 *  every two seconds during that healthy transition overloaded the command
 *  service. Give the first positive ACK a fixed, non-extendable 20-second grace
 *  and permit only two later probes, five seconds apart. */
const int64_t kModeVerifyIntervalNs = 2LL * 1000000000LL;  // 2 s
const uint8_t kSleepVerifyMaxRetries = 3;
const uint8_t kNormalNoAckMaxRetries = 7;
const int64_t kNormalSpinupGraceNs = 20LL * 1000000000LL;
const int64_t kNormalPostGraceRetryIntervalNs = 5LL * 1000000000LL;
const uint8_t kNormalPostGraceMaxRetries = 2;
/** Initial config command plus two timeout retries. If all fail, keep the
 *  device in Config; the bounded Config watchdog handles escalation. */
const uint8_t kConfigCommandMaxRetries = 2;

/** Retry only failures that can occur while the command channel is still
 *  usable. Disconnect/cancellation statuses must wait for the reconnect path;
 *  retrying them from inside SDK teardown can re-enter a channel destructor. */
bool ShouldRetryConfigCommand(livox_status status) {
  return status == kStatusSuccess || status == kStatusTimeout;
}
}  // namespace

void LdsLidar::TickSleepModeVerification() {
  ReapCommandContexts();
  int64_t now = std::chrono::steady_clock::now().time_since_epoch().count();
  for (uint8_t h = 0; h < kMaxLidarCount; h++) {
    LidarMode desired = kLidarModeNormal;
    uint64_t request_id = 0;
    uint64_t command_id = 0;
    uint64_t connection_generation = 0;
    uint8_t max_retries = kSleepVerifyMaxRetries;
    uint8_t attempt = 0;
    uint8_t post_grace_attempt = 0;
    bool initial_send = false, resend = false, giveup = false, done = false,
         exhausted = false, acknowledged_normal = false;
    LidarConnectState connect_state;
    LidarState actual_state;
    {
      lock_guard<mutex> lock(data_lock_[h]);
      connect_state = lidars_[h].connect_state;
      actual_state = lidars_[h].info.state;
      connection_generation = connection_generation_[h].load(
          std::memory_order_acquire);
    }
    {
      lock_guard<mutex> lock(mode_mutex_);
      ModeChangeRequest &req = mode_requests_[h];
      if (!req.active) {
        continue;
      }
      desired = req.desired_mode;
      request_id = req.request_id;
      command_id = req.command_id;
      max_retries = (desired == kLidarModeNormal)
                        ? kNormalNoAckMaxRetries
                        : kSleepVerifyMaxRetries;
      acknowledged_normal =
          desired == kLidarModeNormal &&
          req.normal_spinup_grace_deadline_ns != 0;
      if (connect_state == kConnectStateOff) {
        if (desired == kLidarModeNormal) {
          req.command_inflight = false;
          req.waiting_for_reconnect = true;
        } else {
          giveup = true;
        }
      } else if (actual_state == ModeToState(req.desired_mode)) {
        done = true;  // mode actually took effect
      } else if (req.command_id == 0) {
        /** A staggered broadcast wake has not sent its first command yet.
         *  Dispatch it from this timer once due; never sleep the ROS callback. */
        initial_send = !req.command_inflight &&
                       (req.send_not_before_ns == 0 ||
                        now >= req.send_not_before_ns);
      } else if (!req.command_inflight) {
        /** The paired SDK guarantees one terminal callback for every accepted
         *  command, so an in-flight command must never be duplicated. */
        if (acknowledged_normal) {
          if (now >= req.normal_spinup_grace_deadline_ns &&
              now - req.last_command_ns >=
                  kNormalPostGraceRetryIntervalNs) {
            max_retries = kNormalPostGraceMaxRetries;
            if (req.normal_post_grace_retry_count <
                kNormalPostGraceMaxRetries) {
              req.normal_post_grace_retry_count++;
              req.sleep_retry_count++;
              req.last_command_ns = now;
              post_grace_attempt = req.normal_post_grace_retry_count;
              attempt = req.sleep_retry_count;
              resend = true;
            } else {
              exhausted = true;
            }
          }
        } else if (now - req.last_command_ns >= kModeVerifyIntervalNs) {
          if (req.sleep_retry_count < max_retries) {
            req.sleep_retry_count++;
            req.last_command_ns = now;
            attempt = req.sleep_retry_count;
            resend = true;
          } else {
            exhausted = true;  // tried the max times, still not switched
          }
        }
      }
    }
    if (done) {
      bool close_was_eligible = false;
      if (ResetModeRequestIfTarget(h, desired, request_id, 0,
                                   connection_generation,
                                   &close_was_eligible)) {
        RecordMeasurementModeSuccess(h, desired, close_was_eligible);
      }
    } else if (initial_send) {
      printf("Lidar[%d] staggered Normal wake is due -- sending first command\n",
             h);
      livox_status s = SendModeChangeRequest(
          h, desired, true, request_id, connection_generation, command_id);
      if (s != kStatusSuccess) {
        printf("Lidar[%d] delayed mode send returned %d\n", h, s);
      }
    } else if (resend) {
      if (acknowledged_normal) {
        printf("Lidar[%d] not Normal after fixed 20s spin-up grace -- "
               "re-sending probe %u/%u (total retry %u)\n",
               h, post_grace_attempt, kNormalPostGraceMaxRetries, attempt);
      } else {
        printf("Lidar[%d] not in mode[%d] yet -- re-sending (attempt %u/%u)\n",
               h, desired, attempt, max_retries);
      }
      livox_status s = SendModeChangeRequest(
          h, desired, true, request_id, connection_generation, command_id);
      if (s != kStatusSuccess) {
        printf("Lidar[%d] mode re-send returned %d\n", h, s);
      }
    } else if (exhausted) {
      if (!ResetModeRequestIfTarget(h, desired, request_id, command_id,
                                    connection_generation)) {
        continue;
      }
      /** Record for the dashboard "Mode retry" footer so it is visible without
       *  digging through the log; then warn and drop the request. Same thread as
       *  the footer render, so no lock needed for link_stat_. */
      {
        lock_guard<mutex> lock(link_stat_lock_[h]);
        link_stat_[h].mode_fail_count++;
        link_stat_[h].mode_fail_wall_s = (int64_t)time(nullptr);
        link_stat_[h].mode_fail_mode = (uint8_t)desired;
      }
      if (acknowledged_normal) {
        printf("Lidar[%d] did not enter Normal after the fixed spin-up grace "
               "and %u probes -- manual check needed\n",
               h, kNormalPostGraceMaxRetries);
      } else {
        printf("Lidar[%d] did not enter mode[%d] after %u retries -- manual "
               "check needed\n",
               h, desired, max_retries);
      }
    } else if (giveup) {
      ResetModeRequestIfTarget(h, desired, request_id, command_id,
                               connection_generation);
    }
  }
}

void LdsLidar::MarkModeRequestDisconnected(uint8_t handle) {
  if (handle >= kMaxLidarCount) {
    return;
  }

  lock_guard<mutex> lock(mode_mutex_);
  ModeChangeRequest &request = mode_requests_[handle];
  request.command_inflight = false;
  if (request.active && request.desired_mode == kLidarModeNormal) {
    request.waiting_for_reconnect = true;
  } else if (request.active) {
    char broadcast_code[kBroadcastCodeSize] = {0};
    strncpy(broadcast_code, request.broadcast_code,
            sizeof(broadcast_code) - 1);
    request = ModeChangeRequest();
    if (broadcast_code[0] != '\0') {
      strncpy(request.broadcast_code, broadcast_code,
              sizeof(request.broadcast_code) - 1);
      request.broadcast_code[sizeof(request.broadcast_code) - 1] = '\0';
    }
  }
}

livox_status LdsLidar::SendModeChangeRequest(
    uint8_t handle, LidarMode mode, bool from_reconnect,
    uint64_t expected_request_id, uint64_t expected_generation,
    uint64_t expected_command_id, int64_t initial_not_before_ns) {
  if (handle >= kMaxLidarCount) {
    return kStatusInvalidHandle;
  }
  lock_guard<mutex> send_lock(mode_send_mutex_[handle]);

  bool connected = false;
  LidarConnectState connect_state = kConnectStateOff;
  LidarState actual_state = kLidarStateUnknown;
  uint64_t connection_generation = 0;
  const int64_t now_ns =
      std::chrono::steady_clock::now().time_since_epoch().count();
  char live_broadcast_code[kBroadcastCodeSize] = {0};
  {
    lock_guard<mutex> lock(data_lock_[handle]);
    const LidarDevice &p_lidar = lidars_[handle];
    connect_state = p_lidar.connect_state;
    actual_state = p_lidar.info.state;
    connected = (p_lidar.connect_state != kConnectStateOff &&
                 p_lidar.handle == handle);
    connection_generation = connection_generation_[handle].load(
        std::memory_order_acquire);
    if (expected_generation != 0 &&
        expected_generation != connection_generation) {
      return kStatusNotConnected;
    }
    strncpy(live_broadcast_code, p_lidar.info.broadcast_code,
            sizeof(live_broadcast_code) - 1);
  }
  if (connected && mode != kLidarModeNormal &&
      actual_state == ModeToState(mode)) {
    /** Idempotent broadcast/service semantics: a lidar already at the requested
     *  low-power target is a success and must not be disturbed. A conflicting
     *  in-flight request still wins until its lifecycle finishes. */
    {
      lock_guard<mutex> lock(mode_mutex_);
      ModeChangeRequest &request = mode_requests_[handle];
      if (request.active && request.desired_mode != mode) {
        return kStatusFailure;
      }
      if (request.active) {
        char broadcast_code[kBroadcastCodeSize] = {0};
        strncpy(broadcast_code, request.broadcast_code,
                sizeof(broadcast_code) - 1);
        request = ModeChangeRequest();
        if (broadcast_code[0] != '\0') {
          strncpy(request.broadcast_code, broadcast_code,
                  sizeof(request.broadcast_code) - 1);
          request.broadcast_code[sizeof(request.broadcast_code) - 1] = '\0';
        }
      }
    }
    CancelWakeObservation(handle);
    ExcludePointCloudOutageForPlannedMode(handle);
    return kStatusSuccess;
  }
  /** A sleep/standby request is safe only after this session completed its
   *  configuration and is sampling in Normal. In particular, rejecting the
   *  brief initial On window prevents a concurrent sleep request from making
   *  OnDeviceChange skip configuration and later start unconfigured sampling. */
  if (connected && mode != kLidarModeNormal &&
      (connect_state != kConnectStateSampling ||
       actual_state != kLidarStateNormal)) {
    return kStatusFailure;
  }

  bool measurement_close_eligible = false;
  if (connected && mode != kLidarModeNormal) {
    lock_guard<mutex> lock(link_stat_lock_[handle]);
    measurement_close_eligible = MeasurementSessionCloseEligible(
        link_stat_[handle].measurement_session);
  }

  uint64_t request_id = 0;
  uint64_t command_id = 0;
  bool send_now = false;
  bool arm_wake_observation = false;
  bool cancel_wake_observation = false;
  {
    lock_guard<mutex> lock(mode_mutex_);
    ModeChangeRequest &request = mode_requests_[handle];
    if (!from_reconnect) {
      request.active = true;
      request.waiting_for_reconnect = false;
      request.command_inflight = false;
      request.desired_mode = mode;
      request.command_id = 0;
      request.last_command_ns = 0;
      request.send_not_before_ns = initial_not_before_ns;
      request.normal_spinup_grace_deadline_ns = 0;
      request.sleep_retry_count = 0;
      request.normal_post_grace_retry_count = 0;
      request.explicit_wake_source =
          connected && mode == kLidarModeNormal &&
          (actual_state == kLidarStatePowerSaving ||
           actual_state == kLidarStateStandBy);
      request.explicit_wake_generation =
          request.explicit_wake_source ? connection_generation : 0;
      request.wake_observation_armed = false;
      request.measurement_close_eligible = measurement_close_eligible;
      request.request_id = ++next_mode_request_id_;
      cancel_wake_observation = mode != kLidarModeNormal;
    } else if (!request.active || request.desired_mode != mode ||
               (expected_request_id != 0 &&
                 request.request_id != expected_request_id) ||
               request.command_id != expected_command_id ||
               request.command_inflight) {
      return kStatusFailure;
    }
    if (live_broadcast_code[0] != '\0') {
      strncpy(request.broadcast_code, live_broadcast_code,
              sizeof(request.broadcast_code) - 1);
      request.broadcast_code[sizeof(request.broadcast_code) - 1] = '\0';
    }
    request_id = request.request_id;
    if (mode == kLidarModeNormal) {
      request.waiting_for_reconnect = !connected;
    }
    send_now = connected &&
               (request.send_not_before_ns == 0 ||
                now_ns >= request.send_not_before_ns);
    if (send_now) {
      command_id = ++next_mode_command_id_;
      request.command_id = command_id;
      request.command_inflight = true;
      request.waiting_for_reconnect = false;
      request.last_command_ns = now_ns;
      request.send_not_before_ns = 0;
      const bool same_session_low_power_wake =
          request.explicit_wake_source &&
          request.explicit_wake_generation == connection_generation &&
          (actual_state == kLidarStatePowerSaving ||
           actual_state == kLidarStateStandBy);
      if (mode == kLidarModeNormal && same_session_low_power_wake &&
          !request.wake_observation_armed) {
        request.wake_observation_armed = true;
        arm_wake_observation = true;
      } else if (mode == kLidarModeNormal &&
                 request.explicit_wake_source &&
                 !same_session_low_power_wake) {
        /** The Normal intent may safely survive a reconnect, but the old
         *  session's low-power fact may not. Never transfer hard-power
         *  attribution to a new generation or a state already back in Normal. */
        request.explicit_wake_source = false;
        request.explicit_wake_generation = 0;
      }
    } else if (mode != kLidarModeNormal) {
      request.active = false;
      request.command_inflight = false;
      request.waiting_for_reconnect = false;
    }
  }

  if (cancel_wake_observation) {
    CancelWakeObservation(handle);
    ObserveNormalPublishing(handle, false, 0, nullptr);
    ExcludePointCloudOutageForPlannedMode(handle);
  }

  if (!connected) {
    if (mode == kLidarModeNormal) {
      printf("Queue lidar[%d] normal-mode recovery until broadcast reconnect.\n",
             handle);
      return kStatusSuccess;
    }
    return kStatusNotConnected;
  }

  if (!send_now) {
    printf("Queue lidar[%d] Normal wake until its stagger deadline.\n", handle);
    return kStatusSuccess;
  }

  std::shared_ptr<AsyncCommandContext> context = CreateCommandContext(
      handle, connection_generation, 0, kAsyncModeCommand, mode, request_id,
      command_id);
  if (arm_wake_observation) {
    /** Arm immediately before enqueue so even a fast disconnect callback is
     *  attributed. A synchronous SDK rejection below cancels the guard. */
    ArmWakeObservation(handle, live_broadcast_code, request_id,
                       connection_generation);
  }
  livox_status status =
      LidarSetMode(handle, mode, SetModeCb, context.get());
  if (status != kStatusSuccess) {
    MarkCommandContextCompleted(context);
    if (arm_wake_observation) {
      CancelWakeObservation(handle, request_id);
    }
  } else {
    return status;
  }

  bool deferred_until_reconnect = false;
  {
    lock_guard<mutex> lock(mode_mutex_);
    ModeChangeRequest &request = mode_requests_[handle];
    if (!request.active || request.desired_mode != mode ||
        request.request_id != request_id ||
        request.command_id != command_id) {
      return status;
    }
    request.command_inflight = false;
    if (arm_wake_observation) {
      /** The synchronous enqueue failed and its LinkStat guard was cancelled
       *  above. Permit a later reconnect retry to arm a fresh guard. */
      request.wake_observation_armed = false;
    }
    if (mode == kLidarModeNormal && ShouldWaitForReconnect(status)) {
      request.waiting_for_reconnect = true;
      deferred_until_reconnect = true;
    } else {
      request.waiting_for_reconnect = false;
      if (mode == kLidarModeNormal || !from_reconnect) {
        request.active = false;
      }
    }
  }

  if (deferred_until_reconnect) {
    printf("Lidar[%d] normal-mode request deferred until reconnect: %d\n",
           handle, status);
    return kStatusSuccess;
  }

  return status;
}

void LdsLidar::MaybeRetryPendingModeRequest(uint8_t handle) {
  if (handle >= kMaxLidarCount) {
    return;
  }

  bool retry = false;
  uint64_t request_id = 0;
  uint64_t command_id = 0;
  {
    lock_guard<mutex> lock(mode_mutex_);
    const ModeChangeRequest &request = mode_requests_[handle];
    const int64_t now =
        std::chrono::steady_clock::now().time_since_epoch().count();
    retry = request.active && request.desired_mode == kLidarModeNormal &&
            request.waiting_for_reconnect && !request.command_inflight &&
            request.normal_spinup_grace_deadline_ns == 0 &&
            (request.send_not_before_ns == 0 ||
             now >= request.send_not_before_ns);
    request_id = request.request_id;
    command_id = request.command_id;
  }

  if (retry) {
    livox_status status =
        SendModeChangeRequest(handle, kLidarModeNormal, true, request_id, 0,
                              command_id);
    if (status != kStatusSuccess) {
      printf("Retry lidar[%d] normal-mode recovery failed immediately: %d\n",
             handle, status);
    }
  }
}

uint64_t LdsLidar::GetActiveNormalRequestId(uint8_t handle) {
  if (handle >= kMaxLidarCount) {
    return 0;
  }
  lock_guard<mutex> lock(mode_mutex_);
  const ModeChangeRequest &request = mode_requests_[handle];
  return (request.active && request.desired_mode == kLidarModeNormal)
             ? request.request_id
             : 0;
}

int LdsLidar::InitLdsLidar(std::vector<std::string> &broadcast_code_strs,
                           const char *user_config_path) {
  if (is_initialized_) {
    printf("LiDAR data source is already inited!\n");
    return -1;
  }

  if (!Init()) {
    Uninit();
    printf("Livox-SDK init fail!\n");
    return -1;
  }

  LivoxSdkVersion _sdkversion;
  GetLivoxSdkVersion(&_sdkversion);
  printf("Livox SDK version %d.%d.%d\n", _sdkversion.major, _sdkversion.minor,
         _sdkversion.patch);

  SetBroadcastCallback(OnDeviceBroadcast);
  SetDeviceStateUpdateCallback(OnDeviceChange);
  SetDeviceHandshakeCallback(OnDeviceHandshake);

  /** Add commandline input broadcast code */
  for (auto input_str : broadcast_code_strs) {
    AddBroadcastCodeToWhitelist(input_str.c_str());
  }

  ParseConfigFile(user_config_path);

  if (whitelist_count_) {
    DisableAutoConnectMode();
    printf("Disable auto connect mode!\n");

    printf("List all broadcast code in whiltelist:\n");
    for (uint32_t i = 0; i < whitelist_count_; i++) {
      printf("%s\n", broadcast_code_whitelist_[i]);
    }
  } else {
    EnableAutoConnectMode();
    printf(
        "No broadcast code was added to whitelist, swith to automatic "
        "connection mode!\n");
  }

  if (enable_timesync_) {
    timesync_ = TimeSync::GetInstance();
    if (timesync_->InitTimeSync(timesync_config_)) {
      printf("Timesync init fail\n");
      timesync_->DeInitTimeSync();
      timesync_ = nullptr;
      Uninit();
      return -1;
    }

    if (timesync_->SetReceiveSyncTimeCb(ReceiveSyncTimeCallback, this)) {
      printf("Set Timesync callback fail\n");
      timesync_->DeInitTimeSync();
      timesync_ = nullptr;
      Uninit();
      return -1;
    }
  }

  /** Start livox sdk to receive lidar data */
  g_lds_ldiar = this;
  if (!Start()) {
    /** Stop/join every producer of SDK API calls before global Uninit. This
     *  also safely unwinds TimeSync threads that were created but not started. */
    if (timesync_) {
      timesync_->DeInitTimeSync();
      timesync_ = nullptr;
    }
    if (g_lds_ldiar == this) {
      g_lds_ldiar = nullptr;
    }
    Uninit();
    printf("Livox-SDK init fail!\n");
    return -1;
  }

  if (timesync_) {
    /** Do not let GPS input call SDK APIs until the SDK I/O loop is running. */
    timesync_->StartTimesync();
  }

  is_initialized_ = true;
  printf("Livox-SDK init success!\n");

  return 0;
}

int LdsLidar::DeInitLdsLidar(void) {
  if (!is_initialized_) {
    printf("LiDAR data source is not exit");
    return -1;
  }

  /** TimeSync invokes LidarSetRmcSyncTime from its data thread. Stop and join
   *  it before global SDK Uninit so no new API call can race teardown. */
  if (timesync_) {
    timesync_->DeInitTimeSync();
    timesync_ = nullptr;
  }

  SetDeviceHandshakeCallback(nullptr);
  Uninit();
  if (g_lds_ldiar == this) {
    g_lds_ldiar = nullptr;
  }
  is_initialized_ = false;
  printf("Livox SDK Deinit completely!\n");

  return 0;
}

void LdsLidar::PrepareExit(void) { DeInitLdsLidar(); }

/** Static function in LdsLidar for callback or event process ----------------*/

/** Receiving point cloud data from Livox LiDAR. */
void LdsLidar::OnLidarDataCb(uint8_t handle, LivoxEthPacket *data,
                             uint32_t data_num, void *client_data) {
  using namespace std;

  LdsLidar *lds_lidar = static_cast<LdsLidar *>(client_data);
  LivoxEthPacket *eth_packet = data;

  if (!data || !data_num || (handle >= kMaxLidarCount)) {
    return;
  }

  lds_lidar->StorageRawPacket(handle, eth_packet);
}

void LdsLidar::OnDeviceHandshake(const DeviceHandshakeStatus *status) {
  if (status == nullptr || g_lds_ldiar == nullptr ||
      status->handle >= kMaxLidarCount) {
    return;
  }

  const uint8_t handle = status->handle;
  const int64_t event_now_ns =
      std::chrono::steady_clock::now().time_since_epoch().count();
  bool log_event = false;
  bool cancelled_power_cycle = false;
  uint32_t event_count = 0;
  char broadcast_code[kBroadcastCodeSize] = {0};
  char ip[16] = {0};
  {
    /** SDK I/O-thread callback: only copy/counter work is done under this
     *  short lock; formatting and disk logging happen after releasing it. */
    lock_guard<mutex> lock(g_lds_ldiar->link_stat_lock_[handle]);
    LinkStat &s = g_lds_ldiar->link_stat_[handle];
    const bool status_has_identity = status->broadcast_code[0] != '\0';
    if (status_has_identity && s.broadcast_code[0] != '\0' &&
        strncmp(s.broadcast_code, status->broadcast_code,
                sizeof(s.broadcast_code)) != 0) {
      /** A callback from the previous occupant of a reused SDK handle must not
       *  overwrite/cancel the new lidar's episode or counters.  The new
       *  identity is established by its broadcast callback before normal
       *  handshake processing continues. */
      return;
    }
    bool changed = !s.handshake_event_valid ||
                   s.last_handshake_event != status->event ||
                   s.last_handshake_detail != status->detail;
    s.handshake_event_valid = true;
    s.last_handshake_event = status->event;
    s.last_handshake_detail = status->detail;
    s.last_handshake_event_wall_s = static_cast<int64_t>(time(nullptr));
    strncpy(s.last_handshake_ip, status->ip, sizeof(s.last_handshake_ip) - 1);
    strncpy(ip, status->ip, sizeof(ip) - 1);
    if (status_has_identity) {
      strncpy(s.broadcast_code, status->broadcast_code,
              sizeof(s.broadcast_code) - 1);
      s.broadcast_code[sizeof(s.broadcast_code) - 1] = '\0';
    }
    strncpy(broadcast_code, s.broadcast_code, sizeof(broadcast_code) - 1);

    if (status->event == kDeviceHandshakeNetworkError &&
        s.connect_since_ns == 0 && s.broadcast_only_since_ns != 0) {
      s.handshake_last_network_error_ns = event_now_ns;
      if (s.handshake_state == kHandshakeLinkPowerCycleRequired) {
        /** Invalidate an escalation which has not yet been acted on. The
         *  relay manager also verifies the current recovery state immediately
         *  before opening a shared power group. */
        s.handshake_state = kHandshakeLinkStuck;
        if (s.power_cycle_reason == kPowerCycleReasonHandshakeStuck) {
          s.power_cycle_reason = kPowerCycleReasonNone;
        }
        cancelled_power_cycle = true;
      }
    }
    if (status->event == kDeviceHandshakeReset &&
        s.connect_since_ns == 0 && s.broadcast_only_since_ns != 0 &&
        s.handshake_reset_attempts != 0 &&
        (s.handshake_reset_phase == kHandshakeResetRequested ||
         s.handshake_reset_phase == kHandshakeResetQueued)) {
      /** This is completion evidence for the one local cleanup request. The
       *  API return is tracked separately because kStatusSuccess only means
       *  the request was accepted, not that cleanup has finished. */
      s.handshake_reset_completed = true;
      s.handshake_reset_completed_ns = event_now_ns;
      s.handshake_reset_phase = kHandshakeResetCompleted;
    }

    switch (status->event) {
      case kDeviceHandshakeSuccess:
        event_count = ++s.handshake_success_count;
        break;
      case kDeviceHandshakeTimeout:
        event_count = ++s.handshake_timeout_count;
        break;
      case kDeviceHandshakeRejected:
        event_count = ++s.handshake_rejected_count;
        break;
      case kDeviceHandshakeNetworkError:
        event_count = ++s.handshake_network_error_count;
        break;
      case kDeviceHandshakeProtocolError:
        event_count = ++s.handshake_protocol_error_count;
        break;
      case kDeviceHandshakeReset:
        /** Reset count is maintained from the API return in the recovery path;
         *  detail here is the exact number of pending SDK sockets cleared. */
        event_count = s.handshake_reset_count;
        break;
      default:
        break;
    }
    /** Log diagnostic edges and periodic repeats without flooding journal at
     *  the device's broadcast rate. */
    log_event = changed || event_count <= 1 ||
                (event_count != 0 && (event_count % 10) == 0);
  }

  if (cancelled_power_cycle) {
    PrintLidarEvent(handle, broadcast_code,
                    "POWER_CYCLE_CANCELLED_NETWORK_ERROR");
    HealthLogger::Get().LogEvent(handle, broadcast_code,
                                 "POWER_CYCLE_CANCELLED_NETWORK_ERROR",
                                 "recent local network error; keep STUCK");
  }

  if (!log_event) {
    return;
  }
  char detail[128];
  snprintf(detail, sizeof(detail), "HANDSHAKE_%s ip=%s detail=%d count=%u",
           HandshakeEventName(status->event), ip[0] ? ip : "?",
           status->detail, event_count);
  char event_line[160];
  snprintf(event_line, sizeof(event_line), "HANDSHAKE_%s",
           HandshakeEventName(status->event));
  PrintLidarEvent(handle, broadcast_code, detail);
  HealthLogger::Get().LogEvent(handle, broadcast_code, event_line, detail);
}

void LdsLidar::OnDeviceBroadcast(const BroadcastDeviceInfo *info) {
  if (info == nullptr) {
    return;
  }

  if (info->dev_type == kDeviceTypeHub) {
    printf("In lidar mode, couldn't connect a hub : %s\n",
           info->broadcast_code);
    return;
  }

  if (g_lds_ldiar->IsAutoConnectMode()) {
    printf("In automatic connection mode, will connect %s\n",
           info->broadcast_code);
  } else {
    if (!g_lds_ldiar->IsBroadcastCodeExistInWhitelist(info->broadcast_code)) {
      printf("Not in the whitelist, please add %s to if want to connect!\n",
             info->broadcast_code);
      return;
    }
  }

  bool result = false;
  uint8_t handle = 0;
  result = AddLidarToConnect(info->broadcast_code, &handle);
  if (result == kStatusSuccess && handle < kMaxLidarCount) {
    SetDataCallback(handle, OnLidarDataCb, (void *)g_lds_ldiar);
    g_lds_ldiar->OnLidarBroadcastEvent(handle, info->broadcast_code);
    g_lds_ldiar->RememberBroadcastCode(handle, info->broadcast_code);

    UserRawConfig config;
    if (g_lds_ldiar->GetRawConfig(info->broadcast_code, config)) {
      printf("Could not find raw config, set config to default!\n");
      config.enable_fan = 1;
      config.return_mode = kFirstReturn;
      config.coordinate = kCoordinateCartesian;
      config.imu_rate = kImuFreq200Hz;
      config.extrinsic_parameter_source = kNoneExtrinsicParameter;
      config.enable_high_sensitivity = false;
    }

    lock_guard<mutex> lock(g_lds_ldiar->data_lock_[handle]);
    LidarDevice *p_lidar = &(g_lds_ldiar->lidars_[handle]);
    p_lidar->handle = handle;
    p_lidar->connect_state = kConnectStateOff;
    p_lidar->config.enable_fan = config.enable_fan;
    p_lidar->config.return_mode = config.return_mode;
    p_lidar->config.coordinate = config.coordinate;
    p_lidar->config.imu_rate = config.imu_rate;
    p_lidar->config.extrinsic_parameter_source =
        config.extrinsic_parameter_source;
    p_lidar->config.enable_high_sensitivity = config.enable_high_sensitivity;
  } else {
    printf("Add lidar to connect is failed : %d %d \n", result, handle);
  }
}

/** Callback function of changing of device state. */
void LdsLidar::OnDeviceChange(const DeviceInfo *info, DeviceEvent type) {
  if (info == nullptr) {
    return;
  }

  uint8_t handle = info->handle;
  if (handle >= kMaxLidarCount) {
    return;
  }

  LidarDevice *p_lidar = &(g_lds_ldiar->lidars_[handle]);
  if (type == kEventConnect) {
    g_lds_ldiar->OnLidarConnectEvent(handle, info->broadcast_code);
    g_lds_ldiar->RememberBroadcastCode(handle, info->broadcast_code);
    QueryDeviceInformation(handle, DeviceInformationCb, g_lds_ldiar);
    {
      lock_guard<mutex> send_lock(g_lds_ldiar->mode_send_mutex_[handle]);
      lock_guard<mutex> lock(g_lds_ldiar->data_lock_[handle]);
      if (p_lidar->connect_state == kConnectStateOff) {
        g_lds_ldiar->connection_generation_[handle].fetch_add(
            1, std::memory_order_acq_rel);
        p_lidar->connect_state = kConnectStateOn;
        p_lidar->info = *info;
      }
    }
    g_lds_ldiar->MaybeRetryPendingModeRequest(handle);
  } else if (type == kEventDisconnect) {
    printf("Lidar[%s] disconnect!\n", info->broadcast_code);
    g_lds_ldiar->RememberBroadcastCode(handle, info->broadcast_code);
    g_lds_ldiar->OnLidarDisconnectEvent(handle, info->broadcast_code);
    /** Guard against concurrent data-thread access while the queue is freed.
     *  Without this lock the SDK's data callback can write into the just-freed
     *  queue -> use-after-free crash (the original upstream bug). */
    {
      std::lock_guard<std::mutex> send_lock(
          g_lds_ldiar->mode_send_mutex_[handle]);
      g_lds_ldiar->MarkModeRequestDisconnected(handle);
      std::lock_guard<std::mutex> lk(g_lds_ldiar->data_lock_[handle]);
      g_lds_ldiar->connection_generation_[handle].fetch_add(
          1, std::memory_order_acq_rel);
      ResetLidar(p_lidar, kSourceRawLidar);
    }
  } else if (type == kEventStateChange) {
    uint64_t active_normal_request_id = 0;
    uint64_t event_generation = 0;
    {
      lock_guard<mutex> send_lock(g_lds_ldiar->mode_send_mutex_[handle]);
      active_normal_request_id =
          g_lds_ldiar->GetActiveNormalRequestId(handle);
      lock_guard<mutex> lock(g_lds_ldiar->data_lock_[handle]);
      LidarState old_state = p_lidar->info.state;
      p_lidar->info = *info;
      event_generation =
          g_lds_ldiar->connection_generation_[handle].load(
              std::memory_order_acquire);
    /** Re-run config+sampling ONLY when the lidar genuinely resumes from a
     *  low-power state (motor was stopped, sampling halted) or when a
     *  Normal-mode switch we requested is completing. Restricting the trigger
     *  this way prevents transient Error/Init -> Normal state flaps (e.g.
     *  temperature/motor warnings in the field) from needlessly tearing down
     *  and reconfiguring an already-sampling lidar, which caused intermittent
     *  topic stalls. */
      bool resumed_from_lowpower =
          (old_state == kLidarStatePowerSaving ||
           old_state == kLidarStateStandBy);
      if (info->state == kLidarStateNormal &&
          p_lidar->connect_state == kConnectStateSampling &&
          (resumed_from_lowpower || active_normal_request_id != 0)) {
        p_lidar->connect_state = kConnectStateOn;
      }
    }

    if (info->state == kLidarStateError) {
      g_lds_ldiar->MarkPointCloudUnexpectedStop(handle);
    }

    /** A Normal state event only completes a pending NORMAL request. It must
     *  not cancel a newer PowerSaving/Standby request: the SDK re-sends state
     *  events on health/feature changes, and a late Normal event from a wake
     *  would otherwise erase the sleep request queued right after it -- the
     *  sleep verify tick then never runs and the switch is silently lost. */
    if (info->state == ModeToState(kLidarModeNormal) &&
        active_normal_request_id != 0) {
      bool close_was_eligible = false;
      if (g_lds_ldiar->ResetModeRequestIfTarget(
              handle, kLidarModeNormal, active_normal_request_id, 0,
              event_generation, &close_was_eligible)) {
        g_lds_ldiar->RecordMeasurementModeSuccess(
            handle, kLidarModeNormal, close_was_eligible);
      }
    }
  }

  bool device_is_on = false;
  DeviceInfo device_info;
  {
    lock_guard<mutex> lock(g_lds_ldiar->data_lock_[handle]);
    device_is_on = (p_lidar->connect_state == kConnectStateOn);
    device_info = p_lidar->info;
  }
  if (device_is_on) {
    printf("Lidar[%s] status_code[%d] working state[%d] feature[%d]\n",
           device_info.broadcast_code,
           device_info.status.status_code.error_code, device_info.state,
           device_info.feature);
    SetErrorMessageCallback(handle, LidarErrorStatusCb);

    /** Config lidar parameter */
    if (device_info.state == kLidarStateNormal) {
      UserConfig config;
      bool start_config = false;
      uint64_t config_generation = 0;
      {
        lock_guard<mutex> send_lock(
            g_lds_ldiar->mode_send_mutex_[handle]);
        {
          lock_guard<mutex> mode_lock(g_lds_ldiar->mode_mutex_);
          const ModeChangeRequest &request =
              g_lds_ldiar->mode_requests_[handle];
          if (request.active &&
              request.desired_mode != kLidarModeNormal) {
            return;
          }
        }
        /** Mark every pending bit and enter Config BEFORE sending any async
         *  command. A fast callback therefore cannot observe a partial bitset,
         *  and the watchdog will not start sampling midway through config. */
        lock_guard<mutex> config_lock(g_lds_ldiar->config_mutex_);
        lock_guard<mutex> data_lock(g_lds_ldiar->data_lock_[handle]);
        if (p_lidar->connect_state == kConnectStateOn &&
            p_lidar->handle == handle &&
            p_lidar->info.state == kLidarStateNormal) {
          config = p_lidar->config;
          p_lidar->config.set_bits |= kConfigCoordinate;
          if (kDeviceTypeLidarMid40 != device_info.type) {
            p_lidar->config.set_bits |= kConfigReturnMode;
          }
          if ((kDeviceTypeLidarMid70 != device_info.type) &&
              (kDeviceTypeLidarMid40 != device_info.type)) {
            p_lidar->config.set_bits |= kConfigImuRate;
          }
          if (config.extrinsic_parameter_source ==
              kExtrinsicParameterFromLidar) {
            p_lidar->config.set_bits |= kConfigGetExtrinsicParameter;
          }
          if (kDeviceTypeLidarTele == device_info.type) {
            p_lidar->config.set_bits |= kConfigSetHighSensitivity;
          }
          p_lidar->connect_state = kConnectStateConfig;
          config_generation =
              g_lds_ldiar->connection_generation_[handle].load(
                  std::memory_order_acquire);
          start_config = true;
        }
      }
      if (!start_config) {
        return;
      }

      livox_status send_status =
          g_lds_ldiar->SendNextConfigCommand(handle, config_generation);
      if (send_status != kStatusSuccess) {
        printf("Lidar[%d] config pipeline was not accepted: %d\n", handle,
               send_status);
      }
    }
  }
}

/** Query the firmware version of Livox LiDAR. */
void LdsLidar::DeviceInformationCb(livox_status status, uint8_t handle,
                                   DeviceInformationResponse *ack,
                                   void *clent_data) {
  if (status != kStatusSuccess) {
    printf("Device Query Informations Failed : %d\n", status);
  }
  if (ack) {
    printf("firmware version: %d.%d.%d.%d\n", ack->firmware_version[0],
           ack->firmware_version[1], ack->firmware_version[2],
           ack->firmware_version[3]);
  }
}

namespace {
const char *TempStr(uint32_t s) {
  return (s == 0) ? "OK" : (s == 1) ? "WARN" : "HOT!";
}
const char *FanStr(uint32_t s) { return (s == 0) ? "OK" : "WARN"; }
const char *Lvl3Str(uint32_t s) {
  return (s == 0) ? "OK" : (s == 1) ? "WARN" : "ERROR";
}
}  // namespace

/** Callback function of Lidar error message. Stores the latest health bits for
 *  the dashboard and prints a decoded line only when the status CHANGES, so a
 *  thermal/fan/motor event is loud and timestamped instead of buried. */
void LdsLidar::LidarErrorStatusCb(livox_status status, uint8_t handle,
                                  ErrorMessage *message) {
  if (message == NULL || handle >= kMaxLidarCount || g_lds_ldiar == nullptr) {
    return;
  }
  (void)status;
  LidarErrorCode ec = message->lidar_error_code;
  char broadcast_code[kBroadcastCodeSize] = {0};
  uint64_t connection_generation = 0;
  {
    lock_guard<mutex> data_lock(g_lds_ldiar->data_lock_[handle]);
    const LidarDevice &lidar = g_lds_ldiar->lidars_[handle];
    if (lidar.connect_state == kConnectStateOff || lidar.handle != handle ||
        lidar.info.broadcast_code[0] == '\0') {
      return;
    }
    strncpy(broadcast_code, lidar.info.broadcast_code,
            sizeof(broadcast_code) - 1);
    connection_generation =
        g_lds_ldiar->connection_generation_[handle].load(
            std::memory_order_acquire);
  }

  {
    lock_guard<mutex> link_lock(g_lds_ldiar->link_stat_lock_[handle]);
    LinkStat &link = g_lds_ldiar->link_stat_[handle];
    if (connection_generation !=
            g_lds_ldiar->connection_generation_[handle].load(
                std::memory_order_acquire) ||
        link.connect_since_ns == 0 || link.broadcast_code[0] == '\0' ||
        strncmp(link.broadcast_code, broadcast_code,
                sizeof(link.broadcast_code)) != 0) {
      /** Drop a callback which crossed a disconnect or handle reassignment.
       *  Recovery and dashboard history must never attribute an old device's
       *  health edge to the new physical broadcast code. */
      return;
    }
    link.health_code = message->error_code;

    /** Track temp_status transitions (count + wall-clock time of last change). */
    if (link.health_temp_seen && ec.temp_status != link.health_prev_temp) {
      link.temp_change_count++;
      link.temp_change_wall_s = (int64_t)time(nullptr);
    }
    link.health_temp_seen = true;
    link.health_prev_temp = ec.temp_status;

    /** Track fault onsets (motor/fan/dirty/volt/firmware/system going bad) so
     *  the dashboard keeps a record after recovery. Count rising edges only. */
    bool fault = (ec.motor_status || ec.fan_status || ec.dirty_warn ||
                  ec.volt_status || ec.firmware_err || ec.system_status);
    if (fault && !link.health_prev_fault) {
      link.fault_count++;
      link.fault_wall_s = (int64_t)time(nullptr);
      link.fault_code = message->error_code;
    }
    link.health_prev_fault = fault;

    /** Ignore pps/ptp/time-sync churn; emit only when an operator-relevant
     *  health field changes. */
    uint32_t watch = (ec.temp_status) | (ec.volt_status << 2) |
                     (ec.motor_status << 4) | (ec.dirty_warn << 6) |
                     (ec.firmware_err << 7) | (ec.fan_status << 8) |
                     (ec.self_heating << 9) | (ec.system_status << 10);
    if (link.health_watch_seen && watch == link.health_last_watch) {
      return;
    }
    link.health_watch_seen = true;
    link.health_last_watch = watch;
  }

  char ts[16];
  NowHms(ts, sizeof(ts));
  printf("[LivoxHealth] %s Lidar[%d] temp=%s fan=%s motor=%s volt=%s dirty=%u "
         "firmware=%u self_heating=%u system=%s\n",
         ts, handle, TempStr(ec.temp_status), FanStr(ec.fan_status),
         Lvl3Str(ec.motor_status), Lvl3Str(ec.volt_status), ec.dirty_warn,
         ec.firmware_err, ec.self_heating, Lvl3Str(ec.system_status));

  /** Same decoded line to the persistent event log (edge-triggered, so even a
   *  brief fault that recovers before the next snapshot is recorded). */
  if (HealthLogger::Get().enabled()) {
    char detail[192];
    snprintf(detail, sizeof(detail),
             "temp=%s fan=%s motor=%s volt=%s dirty=%u fw=%u self_heat=%u "
             "sys=%s",
             TempStr(ec.temp_status), FanStr(ec.fan_status),
             Lvl3Str(ec.motor_status), Lvl3Str(ec.volt_status), ec.dirty_warn,
             ec.firmware_err, ec.self_heating, Lvl3Str(ec.system_status));
    HealthLogger::Get().LogEvent(handle, broadcast_code, "HEALTH", detail);
  }
}

void LdsLidar::ControlFanCb(livox_status status, uint8_t handle,
                            uint8_t response, void *clent_data) {}

void LdsLidar::CompleteConfigCommand(
    uint8_t handle, uint32_t config_bit,
    uint64_t connection_generation) {
  if (handle >= kMaxLidarCount) {
    return;
  }

  bool start_sampling = false;
  bool send_next = false;
  {
    /** Keep config completion and lifecycle validation in one transaction.
     *  A callback from a connection that has already reset must not clear a
     *  new device's bits or start its sampling early. (A connection generation
     *  will additionally be needed to reject the rare reset+same-handle-reuse
     *  case; handle/state guards close the common disconnect race.) */
    lock_guard<mutex> config_lock(config_mutex_);
    lock_guard<mutex> data_lock(data_lock_[handle]);
    LidarDevice &lidar = lidars_[handle];
    if (lidar.handle != handle ||
        lidar.connect_state != kConnectStateConfig ||
        connection_generation != connection_generation_[handle].load(
                                     std::memory_order_acquire)) {
      return;
    }
    lidar.config.set_bits &= ~config_bit;
    start_sampling = (lidar.config.set_bits == 0);
    send_next = !start_sampling;
  }

  if (send_next) {
    livox_status status =
        SendNextConfigCommand(handle, connection_generation);
    if (status != kStatusSuccess) {
      printf("Lidar[%d] next config pipeline stage was not accepted: %d\n",
             handle, status);
    }
    return;
  }

  livox_status status = SendStartSampling(handle, connection_generation);
  lock_guard<mutex> data_lock(data_lock_[handle]);
  LidarDevice &lidar = lidars_[handle];
  if (lidar.handle == handle &&
      lidar.connect_state == kConnectStateConfig &&
      connection_generation == connection_generation_[handle].load(
                                   std::memory_order_acquire)) {
    /** Sampling is the SDK's accepted/in-flight state. A synchronous callback
     *  may already have moved it to Sampling or On, in which case keep that
     *  more authoritative result. */
    lidar.connect_state = (status == kStatusSuccess)
                              ? kConnectStateSampling
                              : kConnectStateOn;
  }
}

void LdsLidar::SetModeCb(livox_status status, uint8_t handle, uint8_t response,
                         void *client_data) {
  if (g_lds_ldiar == nullptr) {
    return;
  }
  std::shared_ptr<AsyncCommandContext> context =
      g_lds_ldiar->AcquireCommandContext(client_data, kAsyncModeCommand);
  if (!context || handle >= kMaxLidarCount ||
      context->handle != handle || context->mode_request_id == 0 ||
      context->mode_command_id == 0) {
    return;
  }

  LdsLidar *lds_lidar = g_lds_ldiar;
  bool clear_request = false;
  bool confirmed_mode_success = false;
  bool wait_for_reconnect = false;
  bool cancel_wake_observation = false;
  LidarMode desired_mode = context->mode;
  LidarState actual_state;
  uint64_t connection_generation = 0;
  {
    lock_guard<mutex> lock(lds_lidar->data_lock_[handle]);
    actual_state = lds_lidar->lidars_[handle].info.state;
    connection_generation =
        lds_lidar->connection_generation_[handle].load(
            std::memory_order_acquire);
  }
  {
    lock_guard<mutex> lock(lds_lidar->mode_mutex_);
    ModeChangeRequest &request = lds_lidar->mode_requests_[handle];
    if (!request.active || request.desired_mode != context->mode ||
        request.request_id != context->mode_request_id ||
        request.command_id != context->mode_command_id ||
        connection_generation != context->connection_generation ||
        lds_lidar->connection_generation_[handle].load(
            std::memory_order_acquire) !=
            context->connection_generation) {
      return;
    }
    request.command_inflight = false;

    if (status == kStatusSuccess) {
      if (desired_mode == kLidarModeNormal) {
        if (response == 0 || response == 2) {
          if (actual_state == kLidarStateNormal) {
            printf("Lidar[%d] already in Normal; mode request complete\n",
                   handle);
            clear_request = true;
            confirmed_mode_success = true;
          } else {
            /** response 2 explicitly means motor spin-up. response 0 also only
             *  acknowledges the command, not the later heartbeat state. Start
             *  one fixed grace window at the first positive ACK; a retry ACK
             *  must not keep extending the logical request forever. */
            if (request.normal_spinup_grace_deadline_ns == 0) {
              request.normal_spinup_grace_deadline_ns =
                  std::chrono::steady_clock::now()
                      .time_since_epoch()
                      .count() +
                  kNormalSpinupGraceNs;
              printf("Lidar[%d] set mode Normal ACK[%d]; fixed 20s spin-up "
                     "grace started\n",
                     handle, response);
            } else {
              printf("Lidar[%d] set mode Normal ACK[%d]; original spin-up "
                     "grace retained (not extended)\n",
                     handle, response);
            }
          }
          request.waiting_for_reconnect = false;
        } else {
          printf("Lidar[%d] set mode Normal FAILED, response[%d]\n", handle,
                 response);
          clear_request = true;
          cancel_wake_observation = true;
        }
      } else {
        /** PowerSaving/Standby: the lidar acked, but some units ack "success"
         *  without actually switching. Keep the request active -- the 1Hz
         *  TickSleepModeVerification confirms the real state and re-sends if the
         *  mode did not take effect. (command_inflight is already cleared.) */
        printf("Lidar[%d] set mode[%d] acked; verifying actual state\n", handle,
               desired_mode);
      }
    } else {
      printf("Lidar[%d] set mode[%d] status[%d] response[%d]\n", handle,
             desired_mode, status, response);
      if (desired_mode == kLidarModeNormal && ShouldWaitForReconnect(status)) {
        request.waiting_for_reconnect = true;
        wait_for_reconnect = true;
      } else if (desired_mode != kLidarModeNormal) {
        /** Sleep/standby command failed at the ack stage; leave the request
         *  active so the verify tick re-sends it. */
      } else {
        clear_request = true;
        cancel_wake_observation = true;
      }
    }
  }

  if (clear_request) {
    bool close_was_eligible = false;
    if (lds_lidar->ResetModeRequestIfTarget(
            handle, desired_mode, context->mode_request_id,
            context->mode_command_id, context->connection_generation,
            &close_was_eligible) && confirmed_mode_success) {
      lds_lidar->RecordMeasurementModeSuccess(
          handle, desired_mode, close_was_eligible);
    }
    if (cancel_wake_observation) {
      lds_lidar->CancelWakeObservation(handle,
                                       context->mode_request_id);
    }
  } else if (wait_for_reconnect) {
    printf("Lidar[%d] will verify/retry Normal on timer or reconnect.\n",
           handle);
  }
}

void LdsLidar::RebootCb(livox_status status, uint8_t handle, uint8_t response,
                        void *client_data) {
  printf("Lidar[%d] reboot command status[%d] response[%d]\n", handle, status,
         response);
  if (g_lds_ldiar == nullptr || handle >= kMaxLidarCount) {
    return;
  }
  lock_guard<mutex> lock(g_lds_ldiar->link_stat_lock_[handle]);
  LinkStat &s = g_lds_ldiar->link_stat_[handle];
  if (s.network_soft_reboot_inflight) {
    s.network_soft_reboot_inflight = false;
    s.network_soft_reboot_ack = status == kStatusSuccess;
    s.network_soft_reboot_command_accepted = status == kStatusSuccess;
    s.network_soft_reboot_status = status;
    s.network_soft_reboot_response = response;
    if (status == kStatusSuccess) {
      const int64_t now =
          std::chrono::steady_clock::now().time_since_epoch().count();
      s.network_soft_reboot_settle_deadline_ns =
          now + s.network_soft_reboot_settle_ns;
      printf("Lidar[%d] network reboot command accepted; waiting %.0fs "
             "for post-reboot handshake before retry\n",
             handle,
             static_cast<double>(s.network_soft_reboot_settle_ns) /
                 1000000000.0);
    } else {
      s.network_soft_reboot_settle_deadline_ns = 0;
      printf("Lidar[%d] network reboot command was not accepted; retry "
             "after the configured short interval\n",
             handle);
    }
    s.network_recovery_state = kNetworkRecoverySoftRebootVerifying;
  }
}

void LdsLidar::SetPointCloudReturnModeCb(livox_status status, uint8_t handle,
                                         uint8_t response, void *clent_data) {
  if (g_lds_ldiar == nullptr) {
    return;
  }
  std::shared_ptr<AsyncCommandContext> context =
      g_lds_ldiar->AcquireCommandContext(clent_data, kAsyncConfigCommand);
  if (!context || handle >= kMaxLidarCount || context->handle != handle) {
    return;
  }
  LdsLidar *lds_lidar = g_lds_ldiar;
  if (!lds_lidar->IsCurrentConfigContext(
          handle, context->connection_generation)) {
    return;
  }

  if (status == kStatusSuccess && response == 0) {
    printf("Set return mode success!\n");
    lds_lidar->CompleteConfigCommand(handle, kConfigReturnMode,
                                     context->connection_generation);
  } else {
    if (context->retry_count < kConfigCommandMaxRetries &&
        ShouldRetryConfigCommand(status)) {
      lds_lidar->SendReturnModeConfig(
          handle, context->retry_count + 1,
          context->connection_generation);
    }
    printf("Set return mode failed: status[%d] response[%d] retry[%u/%u]\n",
           status, response, context->retry_count,
           kConfigCommandMaxRetries);
  }
}

void LdsLidar::SetCoordinateCb(livox_status status, uint8_t handle,
                               uint8_t response, void *clent_data) {
  if (g_lds_ldiar == nullptr) {
    return;
  }
  std::shared_ptr<AsyncCommandContext> context =
      g_lds_ldiar->AcquireCommandContext(clent_data, kAsyncConfigCommand);
  if (!context || handle >= kMaxLidarCount || context->handle != handle) {
    return;
  }
  LdsLidar *lds_lidar = g_lds_ldiar;
  if (!lds_lidar->IsCurrentConfigContext(
          handle, context->connection_generation)) {
    return;
  }

  if (status == kStatusSuccess && response == 0) {
    printf("Set coordinate success!\n");
    lds_lidar->CompleteConfigCommand(handle, kConfigCoordinate,
                                     context->connection_generation);
  } else {
    if (context->retry_count < kConfigCommandMaxRetries &&
        ShouldRetryConfigCommand(status)) {
      lds_lidar->SendCoordinateConfig(
          handle, context->retry_count + 1,
          context->connection_generation);
    }
    printf("Set coordinate failed: status[%d] response[%d] retry[%u/%u]\n",
           status, response, context->retry_count,
           kConfigCommandMaxRetries);
  }
}

void LdsLidar::SetImuRatePushFrequencyCb(livox_status status, uint8_t handle,
                                         uint8_t response, void *clent_data) {
  if (g_lds_ldiar == nullptr) {
    return;
  }
  std::shared_ptr<AsyncCommandContext> context =
      g_lds_ldiar->AcquireCommandContext(clent_data, kAsyncConfigCommand);
  if (!context || handle >= kMaxLidarCount || context->handle != handle) {
    return;
  }
  LdsLidar *lds_lidar = g_lds_ldiar;
  if (!lds_lidar->IsCurrentConfigContext(
          handle, context->connection_generation)) {
    return;
  }

  if (status == kStatusSuccess && response == 0) {
    printf("Set imu rate success!\n");
    lds_lidar->CompleteConfigCommand(handle, kConfigImuRate,
                                     context->connection_generation);
  } else {
    if (context->retry_count < kConfigCommandMaxRetries &&
        ShouldRetryConfigCommand(status)) {
      lds_lidar->SendImuRateConfig(
          handle, context->retry_count + 1,
          context->connection_generation);
    }
    printf("Set imu rate failed: status[%d] response[%d] retry[%u/%u]\n",
           status, response, context->retry_count,
           kConfigCommandMaxRetries);
  }
}

/** Callback function of get LiDARs' extrinsic parameter. */
void LdsLidar::GetLidarExtrinsicParameterCb(
    livox_status status, uint8_t handle,
    LidarGetExtrinsicParameterResponse *response, void *clent_data) {
  if (g_lds_ldiar == nullptr) {
    return;
  }
  std::shared_ptr<AsyncCommandContext> context =
      g_lds_ldiar->AcquireCommandContext(clent_data, kAsyncConfigCommand);
  if (!context || handle >= kMaxLidarCount || context->handle != handle) {
    return;
  }
  LdsLidar *lds_lidar = g_lds_ldiar;
  if (!lds_lidar->IsCurrentConfigContext(
          handle, context->connection_generation)) {
    return;
  }

  if (status == kStatusSuccess && response != nullptr &&
      response->ret_code == 0) {
      printf("Lidar[%d] get ExtrinsicParameter status[%d] response[%d]\n",
             handle, status, response->ret_code);
      LidarDevice *p_lidar = &(lds_lidar->lidars_[handle]);
      {
        lock_guard<mutex> data_lock(lds_lidar->data_lock_[handle]);
        if (p_lidar->handle != handle ||
            p_lidar->connect_state != kConnectStateConfig ||
            context->connection_generation !=
                lds_lidar->connection_generation_[handle].load(
                    std::memory_order_acquire)) {
          return;
        }
        ExtrinsicParameter *p_extrinsic = &p_lidar->extrinsic_parameter;
        p_extrinsic->euler[0] =
            static_cast<float>(response->roll * PI / 180.0);
        p_extrinsic->euler[1] =
            static_cast<float>(response->pitch * PI / 180.0);
        p_extrinsic->euler[2] =
            static_cast<float>(response->yaw * PI / 180.0);
        p_extrinsic->trans[0] = static_cast<float>(response->x / 1000.0);
        p_extrinsic->trans[1] = static_cast<float>(response->y / 1000.0);
        p_extrinsic->trans[2] = static_cast<float>(response->z / 1000.0);
        EulerAnglesToRotationMatrix(p_extrinsic->euler,
                                    p_extrinsic->rotation);
        if (p_lidar->config.extrinsic_parameter_source) {
          p_extrinsic->enable = true;
        }
      }
      printf("Lidar[%d] get ExtrinsicParameter success!\n", handle);

      lds_lidar->CompleteConfigCommand(handle,
                                       kConfigGetExtrinsicParameter,
                                       context->connection_generation);
  } else {
    printf("Lidar[%d] get ExtrinsicParameter failed: status[%d] response[%d] "
           "retry[%u/%u]\n", handle, status,
           response == nullptr ? -1 : response->ret_code,
           context->retry_count, kConfigCommandMaxRetries);
    if (context->retry_count < kConfigCommandMaxRetries &&
        ShouldRetryConfigCommand(status)) {
      lds_lidar->SendExtrinsicConfig(
          handle, context->retry_count + 1,
          context->connection_generation);
    }
  }
}

void LdsLidar::SetHighSensitivityCb(livox_status status, uint8_t handle,
                                    DeviceParameterResponse *response,
                                    void *clent_data) {
  if (g_lds_ldiar == nullptr) {
    return;
  }
  std::shared_ptr<AsyncCommandContext> context =
      g_lds_ldiar->AcquireCommandContext(clent_data, kAsyncConfigCommand);
  if (!context || handle >= kMaxLidarCount || context->handle != handle) {
    return;
  }
  LdsLidar *lds_lidar = g_lds_ldiar;
  if (!lds_lidar->IsCurrentConfigContext(
          handle, context->connection_generation)) {
    return;
  }

  if (status == kStatusSuccess && response != nullptr &&
      response->ret_code == 0) {
    printf("Set high sensitivity success!\n");
    lds_lidar->CompleteConfigCommand(handle, kConfigSetHighSensitivity,
                                     context->connection_generation);
  } else {
    if (context->retry_count < kConfigCommandMaxRetries &&
        ShouldRetryConfigCommand(status)) {
      lds_lidar->SendHighSensitivityConfig(handle,
                                           context->retry_count + 1,
                                           context->connection_generation);
    }
    printf("Set high sensitivity failed: status[%d] response[%d] "
           "retry[%u/%u]\n", status,
           response == nullptr ? -1 : response->ret_code,
           context->retry_count, kConfigCommandMaxRetries);
  }
}

/** Callback function of starting sampling. */
void LdsLidar::StartSampleCb(livox_status status, uint8_t handle,
                             uint8_t response, void *clent_data) {
  if (g_lds_ldiar == nullptr) {
    return;
  }
  std::shared_ptr<AsyncCommandContext> context =
      g_lds_ldiar->AcquireCommandContext(
          clent_data, kAsyncStartSamplingCommand);
  if (!context || handle >= kMaxLidarCount || context->handle != handle) {
    return;
  }

  LdsLidar *lds_lidar = g_lds_ldiar;
  LidarDevice *p_lidar = &(lds_lidar->lidars_[handle]);
  /** Serialize the callback's state transition with disconnect/ResetLidar.
   *  If reset won the race, both the invalid handle and Off state reject this
   *  late callback; if the callback won, reset runs afterwards and remains the
   *  final state. This prevents a late ACK from reviving a freed queue slot. */
  lock_guard<mutex> lock(lds_lidar->data_lock_[handle]);
  if (context->connection_generation !=
          lds_lidar->connection_generation_[handle].load(
              std::memory_order_acquire) ||
      p_lidar->handle != handle ||
      p_lidar->connect_state == kConnectStateOff) {
    printf("Ignore stale start-sampling callback for lidar[%d]\n", handle);
    return;
  }
  if (status == kStatusSuccess) {
    if (response != 0) {
      p_lidar->connect_state = kConnectStateOn;
      printf("Lidar start sample fail : state[%d] handle[%d] res[%d]\n", status,
             handle, response);
    } else {
      /** Promote back to Sampling on success. Without this, a start-sampling
       *  retry (auto_recover) after an earlier timeout demoted the state to On
       *  leaves it at On forever: the lidar streams, but the consumer only
       *  reads Sampling-state queues, so every packet is queue-dropped
       *  (field-confirmed: recv normal, drop 100%, no ROS output). Guard
       *  against a disconnect that raced the ack. */
      p_lidar->connect_state = kConnectStateSampling;
      printf("Lidar start sample success\n");
    }
  } else if (status == kStatusTimeout) {
    p_lidar->connect_state = kConnectStateOn;
    printf("Lidar start sample timeout : state[%d] handle[%d] res[%d]\n",
           status, handle, response);
  }
}

/** Callback function of stopping sampling. */
void LdsLidar::StopSampleCb(livox_status status, uint8_t handle,
                            uint8_t response, void *clent_data) {}

void LdsLidar::SetRmcSyncTimeCb(livox_status status, uint8_t handle,
                                uint8_t response, void *client_data) {
  if (handle >= kMaxLidarCount) {
    return;
  }
  printf("Set lidar[%d] sync time status[%d] response[%d]\n", handle, status,
         response);
}

void LdsLidar::ReceiveSyncTimeCallback(const char *rmc, uint32_t rmc_length,
                                       void *client_data) {
  LdsLidar *lds_lidar = static_cast<LdsLidar *>(client_data);
  if (lds_lidar == nullptr || rmc == nullptr || rmc_length == 0) {
    return;
  }
  for (uint8_t handle = 0; handle < kMaxLidarCount; handle++) {
    /** Serialize the eligibility snapshot and enqueue with connect/disconnect
     *  so an RMC command cannot cross into a newly reused handle session. */
    lock_guard<mutex> send_lock(lds_lidar->mode_send_mutex_[handle]);
    bool eligible = false;
    {
      lock_guard<mutex> lock(lds_lidar->data_lock_[handle]);
      const LidarDevice &lidar = lds_lidar->lidars_[handle];
      eligible = lidar.handle == handle &&
                 lidar.connect_state == kConnectStateSampling &&
                 lidar.info.state == kLidarStateNormal;
    }
    if (eligible) {
      livox_status status = LidarSetRmcSyncTime(handle, rmc, rmc_length,
                                                SetRmcSyncTimeCb, lds_lidar);
      if (status != kStatusSuccess) {
        printf("Set GPRMC synchronization time error code: %d.\n", status);
      }
    }
  }
}

/** Add broadcast code to whitelist */
int LdsLidar::AddBroadcastCodeToWhitelist(const char *broadcast_code) {
  if (!broadcast_code || (strlen(broadcast_code) > kBroadcastCodeSize) ||
      (whitelist_count_ >= kMaxLidarCount)) {
    return -1;
  }

  if (LdsLidar::IsBroadcastCodeExistInWhitelist(broadcast_code)) {
    printf("%s is alrealy exist!\n", broadcast_code);
    return -1;
  }

  strcpy(broadcast_code_whitelist_[whitelist_count_], broadcast_code);
  ++whitelist_count_;

  return 0;
}

bool LdsLidar::IsBroadcastCodeExistInWhitelist(const char *broadcast_code) {
  if (!broadcast_code) {
    return false;
  }

  for (uint32_t i = 0; i < whitelist_count_; i++) {
    if (strncmp(broadcast_code, broadcast_code_whitelist_[i],
                kBroadcastCodeSize) == 0) {
      return true;
    }
  }

  return false;
}

int LdsLidar::ParseTimesyncConfig(rapidjson::Document &doc) {
  do {
    if (!doc.HasMember("timesync_config") || !doc["timesync_config"].IsObject())
      break;

    const rapidjson::Value &object = doc["timesync_config"];
    if (!object.IsObject()) break;

    if (!object.HasMember("enable_timesync") ||
        !object["enable_timesync"].IsBool())
      break;
    enable_timesync_ = object["enable_timesync"].GetBool();

    if (!object.HasMember("device_name") || !object["device_name"].IsString())
      break;
    std::string device_name = object["device_name"].GetString();
    std::strncpy(timesync_config_.dev_config.name, device_name.c_str(),
                 sizeof(timesync_config_.dev_config.name));

    if (!object.HasMember("comm_device_type") ||
        !object["comm_device_type"].IsInt())
      break;
    timesync_config_.dev_config.type = object["comm_device_type"].GetInt();

    if (timesync_config_.dev_config.type == kCommDevUart) {
      if (!object.HasMember("baudrate_index") ||
          !object["baudrate_index"].IsInt())
        break;
      timesync_config_.dev_config.config.uart.baudrate =
          object["baudrate_index"].GetInt();

      if (!object.HasMember("parity_index") || !object["parity_index"].IsInt())
        break;
      timesync_config_.dev_config.config.uart.parity =
          object["parity_index"].GetInt();
    }

    if (enable_timesync_) {
      printf("Enable timesync : \n");
      if (timesync_config_.dev_config.type == kCommDevUart) {
        printf("Uart[%s],baudrate index[%d],parity index[%d]\n",
               timesync_config_.dev_config.name,
               timesync_config_.dev_config.config.uart.baudrate,
               timesync_config_.dev_config.config.uart.parity);
      }
    } else {
      printf("Disable timesync\n");
    }
    return 0;
  } while (0);

  return -1;
}

/** Config file process */
int LdsLidar::ParseConfigFile(const char *pathname) {
  FILE *raw_file = std::fopen(pathname, "rb");
  if (!raw_file) {
    printf("Open json config file fail!\n");
    return -1;
  }

  char read_buffer[32768];
  rapidjson::FileReadStream config_file(raw_file, read_buffer,
                                        sizeof(read_buffer));

  rapidjson::Document doc;
  if (!doc.ParseStream(config_file).HasParseError()) {
    if (doc.HasMember("lidar_config") && doc["lidar_config"].IsArray()) {
      const rapidjson::Value &array = doc["lidar_config"];
      size_t len = array.Size();
      for (size_t i = 0; i < len; i++) {
        const rapidjson::Value &object = array[i];
        if (object.IsObject()) {
          UserRawConfig config = {0};
          memset(&config, 0, sizeof(config));
          if (object.HasMember("broadcast_code") &&
              object["broadcast_code"].IsString()) {
            std::string broadcast_code = object["broadcast_code"].GetString();
            std::strncpy(config.broadcast_code, broadcast_code.c_str(),
                         sizeof(config.broadcast_code));
          } else {
            printf("User config file parse error\n");
            continue;
          }

          if (object.HasMember("enable_connect") &&
              object["enable_connect"].IsBool()) {
            config.enable_connect = object["enable_connect"].GetBool();
          }
          if (object.HasMember("enable_fan") && object["enable_fan"].IsBool()) {
            config.enable_fan = object["enable_fan"].GetBool();
          }
          if (object.HasMember("return_mode") &&
              object["return_mode"].IsInt()) {
            config.return_mode = object["return_mode"].GetInt();
          }
          if (object.HasMember("coordinate") && object["coordinate"].IsInt()) {
            config.coordinate = object["coordinate"].GetInt();
          }
          if (object.HasMember("imu_rate") && object["imu_rate"].IsInt()) {
            config.imu_rate = object["imu_rate"].GetInt();
          }
          if (object.HasMember("extrinsic_parameter_source") &&
              object["extrinsic_parameter_source"].IsInt()) {
            config.extrinsic_parameter_source =
                object["extrinsic_parameter_source"].GetInt();
          }
          if (object.HasMember("enable_high_sensitivity") &&
              object["enable_high_sensitivity"].GetBool()) {
            config.enable_high_sensitivity =
                object["enable_high_sensitivity"].GetBool();
          }

          printf("broadcast code[%s] : %d %d %d %d %d %d\n",
                 config.broadcast_code, config.enable_connect,
                 config.enable_fan, config.return_mode, config.coordinate,
                 config.imu_rate, config.extrinsic_parameter_source);
          if (config.enable_connect) {
            if (!AddBroadcastCodeToWhitelist(config.broadcast_code)) {
              if (AddRawUserConfig(config)) {
                printf("Raw config is already exist : %s \n",
                       config.broadcast_code);
              }
            }
          }
        }
      }
    }

    if (ParseTimesyncConfig(doc)) {
      printf("Parse timesync config fail\n");
      enable_timesync_ = false;
    }
  } else {
    printf("User config file parse error[%d]\n",
           doc.ParseStream(config_file).HasParseError());
  }

  std::fclose(raw_file);

  return 0;
}

int LdsLidar::AddRawUserConfig(UserRawConfig &config) {
  if (IsExistInRawConfig(config.broadcast_code)) {
    return -1;
  }

  raw_config_.push_back(config);
  printf("Add Raw user config : %s \n", config.broadcast_code);

  return 0;
}

bool LdsLidar::IsExistInRawConfig(const char *broadcast_code) {
  if (broadcast_code == nullptr) {
    return false;
  }

  for (auto ite_config : raw_config_) {
    if (strncmp(ite_config.broadcast_code, broadcast_code,
                kBroadcastCodeSize) == 0) {
      return true;
    }
  }

  return false;
}

int LdsLidar::GetRawConfig(const char *broadcast_code, UserRawConfig &config) {
  if (broadcast_code == nullptr) {
    return -1;
  }

  for (auto ite_config : raw_config_) {
    if (strncmp(ite_config.broadcast_code, broadcast_code,
                kBroadcastCodeSize) == 0) {
      config.enable_fan = ite_config.enable_fan;
      config.return_mode = ite_config.return_mode;
      config.coordinate = ite_config.coordinate;
      config.imu_rate = ite_config.imu_rate;
      config.extrinsic_parameter_source = ite_config.extrinsic_parameter_source;
      config.enable_high_sensitivity = ite_config.enable_high_sensitivity;
      return 0;
    }
  }

  return -1;
}

}  // namespace livox_ros
