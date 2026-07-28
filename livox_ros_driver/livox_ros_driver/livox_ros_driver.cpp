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

#include "include/livox_ros_driver.h"

#include <chrono>
#include <cstddef>
#include <cstdio>
#include <vector>
#include <sstream>
#include <cstring>
#include <ctime>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include "dashboard_metrics.h"
#include "group_power_cycle_protocol.h"
#include "health_logger.h"
#include "recovery_event_json.h"
#include "lddc.h"
#include "lds_hub.h"
#include "lds_lidar.h"
#include "lds_lvx.h"
#include "livox_sdk.h"
#include "livox_ros_driver/LidarMode.h"
#include "livox_ros_driver/LidarReboot.h"

using namespace livox_ros;

const int32_t kSdkVersionMajorLimit = 2;

/** Pointer to LdsLidar for service callback, only valid when data_src == raw lidar */
static LdsLidar *g_read_lidar = nullptr;
/** Four simultaneous Horizon motor starts reproduced a command-service storm in
 *  the field. Broadcast Normal remains one logical service request, but its
 *  per-device sends are queued 0/2/4/6 seconds apart by the 1 Hz mode tick. */
static const uint32_t kBroadcastNormalStaggerMs = 2000;

/** A broadcast service request must only target a live SDK handle.  ResetLidar
 *  deliberately sets LidarDevice::handle to kMaxSourceLidar, so checking both
 *  fields also prevents a stale slot from receiving a queued request. */
static bool IsCurrentConnectedHandle(uint8_t handle) {
  if (g_read_lidar == nullptr || handle >= kMaxLidarCount) {
    return false;
  }
  std::lock_guard<std::mutex> lock(g_read_lidar->data_lock_[handle]);
  const LidarDevice &lidar = g_read_lidar->lidars_[handle];
  return lidar.connect_state != kConnectStateOff && lidar.handle == handle;
}

bool LidarModeServiceCb(livox_ros_driver::LidarMode::Request &req,
                        livox_ros_driver::LidarMode::Response &res) {
  if (g_read_lidar == nullptr) {
    ROS_ERROR("LiDAR mode service: data source is not raw lidar");
    res.ret_code = -1;
    return true;
  }

  if (req.mode < 1 || req.mode > 3) {
    ROS_ERROR("LiDAR mode service: invalid mode %d (1=Normal, 2=PowerSaving, 3=Standby)", req.mode);
    res.ret_code = -1;
    return true;
  }

  if (req.handle == 255) {
    /** Broadcast to all connected LiDARs */
    ROS_INFO("LiDAR mode service: ALL lidars -> mode=%d", req.mode);
    livox_status last_failure = kStatusSuccess;
    bool requested = false;
    bool have_failure = false;
    uint32_t normal_wake_index = 0;
    for (uint8_t h = 0; h < kMaxLidarCount; h++) {
      if (!IsCurrentConnectedHandle(h)) {
        continue;
      }
      requested = true;
      const uint32_t delay_ms =
          req.mode == static_cast<uint8_t>(kLidarModeNormal)
              ? normal_wake_index++ * kBroadcastNormalStaggerMs
              : 0;
      livox_status s = g_read_lidar->RequestLidarModeChange(
          h, static_cast<LidarMode>(req.mode), delay_ms);
      if (s == kStatusSuccess && delay_ms != 0) {
        ROS_INFO("LiDAR Normal wake queued for handle=%d, not-before +%.1fs",
                 h, delay_ms / 1000.0);
      }
      if (s != kStatusSuccess) {
        ROS_WARN("LiDAR mode change failed for handle=%d: %d", h, s);
        /** Preserve a partial failure: a later successful handle must not make
         *  a broadcast request look wholly successful to the scheduler. */
        last_failure = s;
        have_failure = true;
      }
    }
    if (!requested) {
      ROS_WARN("LiDAR mode service: no connected lidar to broadcast to");
    }
    res.ret_code = !requested ? kStatusNotConnected
                              : have_failure ? last_failure : kStatusSuccess;
    return true;
  }

  ROS_INFO("LiDAR mode service: handle=%d mode=%d", req.handle, req.mode);
  livox_status status = g_read_lidar->RequestLidarModeChange(
      req.handle, static_cast<LidarMode>(req.mode));
  res.ret_code = status;

  if (status == kStatusSuccess) {
    ROS_INFO("LiDAR mode change request accepted");
  } else {
    ROS_WARN("LiDAR mode change request returned: %d", status);
  }
  return true;
}

bool LidarRebootServiceCb(livox_ros_driver::LidarReboot::Request &req,
                          livox_ros_driver::LidarReboot::Response &res) {
  if (g_read_lidar == nullptr) {
    ROS_ERROR("LiDAR reboot service: data source is not raw lidar");
    res.ret_code = -1;
    return true;
  }

  if (req.handle == 255) {
    /** Reboot all connected LiDARs */
    ROS_INFO("LiDAR reboot service: ALL lidars");
    livox_status last_failure = kStatusSuccess;
    bool requested = false;
    bool have_failure = false;
    for (uint8_t h = 0; h < kMaxLidarCount; h++) {
      if (!IsCurrentConnectedHandle(h)) {
        continue;
      }
      requested = true;
      livox_status s = g_read_lidar->RequestLidarReboot(h);
      if (s != kStatusSuccess) {
        ROS_WARN("LiDAR reboot failed for handle=%d: %d", h, s);
        /** As above, retain any partial failure for the caller. */
        last_failure = s;
        have_failure = true;
      }
    }
    if (!requested) {
      ROS_WARN("LiDAR reboot service: no connected lidar to broadcast to");
    }
    res.ret_code = !requested ? kStatusNotConnected
                              : have_failure ? last_failure : kStatusSuccess;
    return true;
  }

  ROS_INFO("LiDAR reboot service: handle=%d", req.handle);
  livox_status status = g_read_lidar->RequestLidarReboot(req.handle);
  res.ret_code = status;

  if (status == kStatusSuccess) {
    ROS_INFO("LiDAR reboot request accepted");
  } else {
    ROS_WARN("LiDAR reboot request returned: %d", status);
  }
  return true;
}

/** Publisher for the per-second stats dashboard (std_msgs/String). */
static ros::Publisher g_stats_pub;
/** Machine-readable recovery topics consumed by the independent relay power
 *  manager.  The request topic is latched and every request has a stable
 *  event_id; the manager additionally watches the 1 Hz state topic so a
 *  restart cannot miss a still-active POWER_CYCLE_REQUIRED episode. */
static ros::Publisher g_power_cycle_request_pub;
static ros::Publisher g_recovery_state_pub;
static ros::Publisher g_group_power_cycle_ack_pub;
static uint64_t g_driver_instance_id = 0;
static int64_t g_driver_started_ns = 0;
static int64_t g_driver_started_wall_s = 0;
/** When true, the stats timer auto-recovers a lidar that is connected/Normal
 *  but has produced no point cloud for a while (restart sampling, then reboot). */
static bool g_auto_recover = false;

/** Complete the manager -> Driver intent/ACK barrier before a shared relay
 *  OFF.  The ACK is published only after all four whitelist identities have
 *  live, token-bound suppression markers. */
static void GroupPowerCycleIntentCb(
    const std_msgs::String::ConstPtr &message) {
  if (!message) {
    return;
  }
  GroupPowerCycleIntent intent;
  std::string detail;
  if (!ParseGroupPowerCycleIntentJson(message->data, &intent, &detail)) {
    ROS_WARN_THROTTLE(30, "Invalid group power-cycle intent ignored: %s",
                      detail.c_str());
    return;
  }
  if (intent.cancel) {
    if (g_read_lidar != nullptr &&
        intent.driver_instance == g_driver_instance_id) {
      g_read_lidar->CancelPlannedGroupPowerCycle(intent.members, intent.token);
      ROS_INFO("Cancelled uncommitted group power-cycle intent token=%s group=%s",
               intent.token.c_str(), intent.group_id.c_str());
    }
    return;
  }

  bool accepted = false;
  if (g_read_lidar == nullptr) {
    detail = "raw lidar data source is unavailable";
  } else if (intent.driver_instance != g_driver_instance_id) {
    std::ostringstream mismatch;
    mismatch << "driver_instance mismatch: intent=" << intent.driver_instance
             << " running=" << g_driver_instance_id;
    detail = mismatch.str();
  } else {
    accepted = g_read_lidar->ArmPlannedGroupPowerCycle(
        intent.members, intent.token, intent.valid_for_ms, &detail);
  }

  if (g_group_power_cycle_ack_pub) {
    std_msgs::String ack;
    ack.data = BuildGroupPowerCycleIntentAckJson(intent, accepted, detail);
    g_group_power_cycle_ack_pub.publish(ack);
  }
  if (accepted) {
    ROS_WARN("Armed planned shared power-cycle token=%s group=%s for %u ms",
             intent.token.c_str(), intent.group_id.c_str(),
             intent.valid_for_ms);
  } else {
    ROS_ERROR("Rejected shared power-cycle intent token=%s group=%s: %s",
              intent.token.c_str(), intent.group_id.c_str(), detail.c_str());
  }
}

static const char *LidarStateStr(uint8_t state) {
  switch (state) {
    case kLidarStateInit:        return "Init";
    case kLidarStateNormal:      return "Normal";
    case kLidarStatePowerSaving:  return "PowerSaving";
    case kLidarStateStandBy:     return "StandBy";
    case kLidarStateError:       return "Error";
    default:                     return "?";
  }
}

static const char *HandshakeStateStr(LdsLidar::HandshakeLinkState state) {
  switch (state) {
    case LdsLidar::kHandshakeLinkBroadcastOnly:
      return "BROADCAST_ONLY";
    case LdsLidar::kHandshakeLinkStuck:
      return "HANDSHAKE_STUCK";
    case LdsLidar::kHandshakeLinkPowerCycleRequired:
      return "POWER_CYCLE_REQUIRED";
    default:
      return "IDLE";
  }
}

static const char *WakeStateStr(LdsLidar::WakeRecoveryState state) {
  switch (state) {
    case LdsLidar::kWakeRecoveryObserving:
      return "OBSERVING";
    case LdsLidar::kWakeRecoveryNoBroadcast:
      return "WAKE_NO_BROADCAST";
    case LdsLidar::kWakeRecoveryDropout:
      return "WAKE_DROPOUT";
    case LdsLidar::kWakeRecoveryPowerCycleRequired:
      return "POWER_CYCLE_REQUIRED";
    default:
      return "IDLE";
  }
}

static const char *NormalDropoutStateStr(
    LdsLidar::NormalDropoutRecoveryState state) {
  switch (state) {
    case LdsLidar::kNormalDropoutNoBroadcast:
      return "NORMAL_NO_BROADCAST";
    case LdsLidar::kNormalDropoutObservingReturn:
      return "BROADCAST_RETURNING";
    case LdsLidar::kNormalDropoutConfirmed:
      return "NORMAL_DROPOUT";
    case LdsLidar::kNormalDropoutPowerCycleRequired:
      return "POWER_CYCLE_REQUIRED";
    default:
      return "IDLE";
  }
}

static const char *PowerCycleReasonStr(LdsLidar::PowerCycleReason reason) {
  switch (reason) {
    case LdsLidar::kPowerCycleReasonHandshakeStuck:
      return "HANDSHAKE_STUCK";
    case LdsLidar::kPowerCycleReasonWakeDropout:
      return "WAKE_DROPOUT";
    case LdsLidar::kPowerCycleReasonNormalDropout:
      return "NORMAL_DROPOUT";
    case LdsLidar::kPowerCycleReasonStartupMissing:
      return "STARTUP_MISSING";
    default:
      return "NONE";
  }
}

static bool IsPowerCycleRequired(const LdsLidar::LinkStat &link) {
  return (link.power_cycle_reason ==
              LdsLidar::kPowerCycleReasonHandshakeStuck &&
          link.handshake_state ==
              LdsLidar::kHandshakeLinkPowerCycleRequired) ||
          (link.power_cycle_reason ==
               LdsLidar::kPowerCycleReasonWakeDropout &&
           link.wake_state ==
               LdsLidar::kWakeRecoveryPowerCycleRequired) ||
         (link.power_cycle_reason ==
              LdsLidar::kPowerCycleReasonNormalDropout &&
          link.normal_dropout_state ==
              LdsLidar::kNormalDropoutPowerCycleRequired);
}

static const char *RecoveryStateStr(const LdsLidar::LinkStat &link) {
  return IsPowerCycleRequired(link) ? "POWER_CYCLE_REQUIRED" : "IDLE";
}

static const char *HandshakeResetPhaseStr(
    LdsLidar::HandshakeResetPhase phase) {
  switch (phase) {
    case LdsLidar::kHandshakeResetRequested:
      return "requested";
    case LdsLidar::kHandshakeResetQueued:
      return "queued";
    case LdsLidar::kHandshakeResetCompleted:
      return "completed";
    case LdsLidar::kHandshakeResetRejected:
      return "rejected";
    default:
      return "none";
  }
}

static const char *HandshakeEventStr(DeviceHandshakeEvent event) {
  switch (event) {
    case kDeviceHandshakeSuccess:
      return "ACK(DeviceInfo pending)";
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

static const char *ConnectStateStr(LidarConnectState state) {
  switch (state) {
    case kConnectStateOff:
      return "Off";
    case kConnectStateOn:
      return "On";
    case kConnectStateConfig:
      return "Config";
    case kConnectStateSampling:
      return "Sampling";
    default:
      return "?";
  }
}

static void PublishRecoveryState(
    uint8_t handle, const char *broadcast_code, bool connected,
    LidarConnectState connect_state, uint8_t lidar_state,
    const LdsLidar::LinkStat &link, bool broadcast_fresh, bool publishing,
    uint64_t published_packets) {
  if (!g_recovery_state_pub) {
    return;
  }
  std_msgs::String msg;
  const bool wake_reason =
      IsPowerCycleRequired(link) &&
      link.power_cycle_reason == LdsLidar::kPowerCycleReasonWakeDropout;
  const bool normal_reason =
      IsPowerCycleRequired(link) &&
      link.power_cycle_reason == LdsLidar::kPowerCycleReasonNormalDropout;
  msg.data = BuildLidarRecoveryStateJson(
      static_cast<int64_t>(time(nullptr)), g_driver_instance_id, handle,
      broadcast_code, connected, ConnectStateStr(connect_state),
      LidarStateStr(lidar_state), HandshakeStateStr(link.handshake_state),
      RecoveryStateStr(link),
      IsPowerCycleRequired(link) ? PowerCycleReasonStr(link.power_cycle_reason)
                                 : "NONE",
      WakeStateStr(link.wake_state), wake_reason ? link.wake_request_id : 0,
      wake_reason ? link.wake_connection_generation : 0,
      wake_reason ? link.wake_dropout_generation : 0,
      wake_reason ? link.wake_started_wall_s : 0,
      wake_reason ? link.wake_attributed_disconnect_wall_s : 0,
      wake_reason ? link.wake_dropout_wall_s : 0,
      broadcast_fresh, publishing, published_packets,
      link.power_cycle_required_count, link.power_cycle_required_wall_s,
      NormalDropoutStateStr(link.normal_dropout_state),
      normal_reason ? link.normal_connection_generation : 0,
      normal_reason ? link.normal_dropout_generation : 0,
      normal_reason ? link.normal_healthy_since_wall_s : 0,
      normal_reason ? link.normal_attributed_disconnect_wall_s : 0,
      normal_reason ? link.normal_dropout_wall_s : 0, "IDLE", 0);
  g_recovery_state_pub.publish(msg);
}

static void PublishPowerCycleRequest(uint8_t handle, const char *broadcast_code,
                                     const LdsLidar::LinkStat &link,
                                     bool broadcast_fresh) {
  if (!g_power_cycle_request_pub || !broadcast_code || !broadcast_code[0]) {
    return;
  }
  const long long detected_at =
      static_cast<long long>(link.power_cycle_required_wall_s);
  std::ostringstream event_id;
  event_id << broadcast_code << ":" << g_driver_instance_id << ":"
           << detected_at << ":" << link.power_cycle_required_count;
  const std::string event_id_text = event_id.str();
  const bool wake_reason =
      link.power_cycle_reason == LdsLidar::kPowerCycleReasonWakeDropout;
  const bool normal_reason =
      link.power_cycle_reason == LdsLidar::kPowerCycleReasonNormalDropout;
  const bool handshake_reason =
      link.power_cycle_reason == LdsLidar::kPowerCycleReasonHandshakeStuck;
  std_msgs::String msg;
  msg.data = BuildPowerCycleRequestJson(
      event_id_text.c_str(), static_cast<int64_t>(time(nullptr)), detected_at,
      g_driver_instance_id, handle, broadcast_code,
      PowerCycleReasonStr(link.power_cycle_reason), broadcast_fresh,
      wake_reason ? link.wake_request_id : 0,
      wake_reason ? link.wake_connection_generation : 0,
      wake_reason ? link.wake_dropout_generation : 0,
      wake_reason ? link.wake_started_wall_s : 0,
      wake_reason ? link.wake_attributed_disconnect_wall_s : 0,
      wake_reason ? link.wake_dropout_wall_s : 0,
      handshake_reason ? link.handshake_reset_attempts : 0,
      link.power_cycle_required_count,
      normal_reason ? link.normal_connection_generation : 0,
      normal_reason ? link.normal_dropout_generation : 0,
      normal_reason ? link.normal_healthy_since_wall_s : 0,
      normal_reason ? link.normal_attributed_disconnect_wall_s : 0,
      normal_reason ? link.normal_dropout_wall_s : 0, 0);
  g_power_cycle_request_pub.publish(msg);
  ROS_ERROR("[LivoxPowerCycle] published request event_id=%s lidar[%u][%s]",
            event_id_text.c_str(), static_cast<unsigned>(handle),
             broadcast_code);
}

struct StartupMissingTracker {
  std::string broadcast_code;
  bool ever_healthy = false;
  int64_t absent_since_ns = 0;
  int64_t absent_since_wall_s = 0;
  int64_t required_at_wall_s = 0;
  uint32_t episode_count = 0;
  bool active = false;
  bool request_emitted = false;
};

static void PublishStartupRecoveryState(const StartupMissingTracker &tracker,
                                        bool power_required) {
  if (!g_recovery_state_pub) {
    return;
  }
  std_msgs::String msg;
  msg.data = BuildLidarRecoveryStateJson(
      static_cast<int64_t>(time(nullptr)), g_driver_instance_id, 255,
      tracker.broadcast_code.c_str(), false, "Off", "?", "IDLE",
      power_required ? "POWER_CYCLE_REQUIRED" : "IDLE",
      power_required ? "STARTUP_MISSING" : "NONE", "IDLE", 0, 0, 0, 0,
      0, 0, false, false, 0, power_required ? tracker.episode_count : 0,
      power_required ? tracker.required_at_wall_s : 0, "IDLE", 0, 0, 0,
      0, 0,
      power_required ? "POWER_CYCLE_REQUIRED" : "STARTUP_MISSING",
      tracker.absent_since_wall_s);
  g_recovery_state_pub.publish(msg);
}

static void PublishStartupPowerCycleRequest(
    const StartupMissingTracker &tracker) {
  if (!g_power_cycle_request_pub || tracker.required_at_wall_s <= 0 ||
      tracker.episode_count == 0) {
    return;
  }
  std::ostringstream event_id;
  event_id << tracker.broadcast_code << ":" << g_driver_instance_id << ":"
           << tracker.required_at_wall_s << ":" << tracker.episode_count;
  std_msgs::String msg;
  msg.data = BuildPowerCycleRequestJson(
      event_id.str().c_str(), static_cast<int64_t>(time(nullptr)),
      tracker.required_at_wall_s, g_driver_instance_id, 255,
      tracker.broadcast_code.c_str(), "STARTUP_MISSING", false, 0, 0, 0, 0,
      0, 0, 0, tracker.episode_count, 0, 0, 0, 0, 0,
      tracker.absent_since_wall_s);
  g_power_cycle_request_pub.publish(msg);
  ROS_ERROR("[LivoxPowerCycle] published STARTUP_MISSING request event_id=%s",
            event_id.str().c_str());
}

static const char *TempStr(uint32_t s) {
  return (s == 0) ? "OK" : (s == 1) ? "WARN" : "HOT!";
}
static const char *FanStr(uint32_t s) { return (s == 0) ? "OK" : "WARN"; }
static const char *MotorStr(uint32_t s) {
  return (s == 0) ? "OK" : (s == 1) ? "WARN" : "ERR!";
}

/** Decode a health code into a short "+"-joined tag of the faulted fields
 *  (e.g. "motor+fan"), for the dashboard's fault-event history. */
static std::string FaultTags(uint32_t code) {
  ErrorMessage em;
  em.error_code = code;
  const LidarErrorCode &e = em.lidar_error_code;
  std::string s;
  auto add = [&](uint32_t bad, const char *tag) {
    if (bad) { if (!s.empty()) s += "+"; s += tag; }
  };
  add(e.motor_status, "motor");
  add(e.fan_status, "fan");
  add(e.dirty_warn, "dirty");
  add(e.volt_status, "volt");
  add(e.firmware_err, "fw");
  add(e.system_status, "sys");
  return s.empty() ? "?" : s;
}

/** Current health fields, kept separate from the historical FaultTags() value.
 *  Temperature is included here because the main table's HW column describes
 *  what needs attention now, not only edge-triggered fault history. */
static std::string CurrentHealthTags(uint32_t code) {
  ErrorMessage em;
  em.error_code = code;
  const LidarErrorCode &e = em.lidar_error_code;
  std::string tags;
  auto add = [&](uint32_t bad, const char *tag) {
    if (!bad) {
      return;
    }
    if (!tags.empty()) {
      tags += "+";
    }
    tags += tag;
  };
  add(e.temp_status, "temp");
  add(e.motor_status, "motor");
  add(e.fan_status, "fan");
  add(e.dirty_warn, "dirty");
  add(e.volt_status, "volt");
  add(e.firmware_err, "fw");
  add(e.system_status, "sys");
  return tags.empty() ? "OK" : tags;
}

/** One canonical current-state label is reused by the table, summary and
 *  active-alert section so a single 1 Hz frame cannot describe the same lidar
 *  with three different terms. */
static std::string DashboardNowState(
    bool connected, bool broadcast_recent, const char *connected_state,
    const LdsLidar::LinkStat &link) {
  if (!connected) {
    const int64_t now =
        std::chrono::steady_clock::now().time_since_epoch().count();
    if (link.planned_group_power_cycle_active &&
        link.planned_group_power_cycle_deadline_ns >= now) {
      return "PLANNED_POWER_CYCLE";
    }
    if (IsPowerCycleRequired(link)) {
      return "POWER_CYCLE_REQUIRED";
    }
    if (link.wake_state == LdsLidar::kWakeRecoveryDropout) {
      return "WAKE_DROPOUT";
    }
    if (link.wake_state == LdsLidar::kWakeRecoveryNoBroadcast) {
      return "WAKE_NO_BROADCAST";
    }
    if (link.normal_dropout_state ==
        LdsLidar::kNormalDropoutConfirmed) {
      return "NORMAL_DROPOUT";
    }
    if (link.normal_dropout_state ==
        LdsLidar::kNormalDropoutNoBroadcast) {
      return "NORMAL_NO_BROADCAST";
    }
    if (link.normal_dropout_state ==
        LdsLidar::kNormalDropoutObservingReturn) {
      return "BROADCAST_RETURNING";
    }
    if (broadcast_recent &&
        link.handshake_state != LdsLidar::kHandshakeLinkIdle) {
      return HandshakeStateStr(link.handshake_state);
    }
    return "DISCONNECTED";
  }
  if (strcmp(connected_state, "NO DATA") == 0) {
    return "NO_DATA";
  }
  if (strcmp(connected_state, "PowerSaving") == 0) {
    return "POWER_SAVING";
  }
  if (strcmp(connected_state, "StandBy") == 0) {
    return "STANDBY";
  }
  if (strcmp(connected_state, "Config") == 0) {
    return "CONFIG";
  }
  if (strcmp(connected_state, "Normal") == 0) {
    return "NORMAL";
  }
  if (strcmp(connected_state, "Error") == 0) {
    return "ERROR";
  }
  if (strcmp(connected_state, "Init") == 0) {
    return "INIT";
  }
  return connected_state;
}

static const char *DashboardModeName(uint8_t mode) {
  switch (mode) {
    case 1:
      return "Normal";
    case 2:
      return "PowerSaving";
    case 3:
      return "Standby";
    default:
      return "Unknown";
  }
}

/** Keep a numeric dashboard cell within its declared width without silently
 *  truncating the most-significant digits.  Values which cannot fit are shown
 *  as e.g. ">9999", which preserves the useful lower-bound meaning. */
static std::string DashboardCountCell(uint64_t value, std::size_t width) {
  char exact[32];
  int written = snprintf(exact, sizeof(exact), "%llu",
                         static_cast<unsigned long long>(value));
  if (written > 0 && static_cast<std::size_t>(written) <= width) {
    return std::string(exact);
  }
  if (width < 2) {
    return ">";
  }
  return ">" + std::string(width - 1, '9');
}

/** Format a wall-clock time_t with its date, or "--" when 0 (never). */
static std::string FmtWall(int64_t t) {
  if (t == 0) {
    return "--";
  }
  time_t tt = (time_t)t;
  struct tm tmv;
  localtime_r(&tt, &tmv);
  char buf[24];
  strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &tmv);
  return std::string(buf);
}

/** Format a steady-clock duration (ns) as a short human string. */
static std::string FmtDur(int64_t ns) {
  if (ns < 0) ns = 0;
  long long s = ns / 1000000000LL;
  char buf[24];
  if (s < 60) {
    snprintf(buf, sizeof(buf), "%llds", s);
  } else if (s < 3600) {
    snprintf(buf, sizeof(buf), "%lldm%llds", s / 60, s % 60);
  } else {
    snprintf(buf, sizeof(buf), "%lldh%lldm", s / 3600, (s % 3600) / 60);
  }
  return std::string(buf);
}

/** auto_recover policy for a lidar stuck in Error state (e.g. a motor fault):
 *  reboot after it has been in Error for kErrorRebootDelaySec, then retry once
 *  every kErrorRebootCooldownSec, up to kErrorRebootMaxAttempts times. After
 *  that, stop rebooting and flag for manual intervention -- this avoids
 *  reboot-looping a physically dead fan/motor that a reboot cannot fix. */
static const uint32_t kErrorRebootDelaySec    = 3;
static const uint32_t kErrorRebootCooldownSec = 40;
static const uint32_t kErrorRebootMaxAttempts = 3;
/** Config commands get their own bounded recovery path. Never start sampling
 *  through a partial configuration; reboot only after a generous timeout. */
static const uint32_t kConfigRebootDelaySec    = 30;
static const uint32_t kConfigRebootCooldownSec = 40;
static const uint32_t kConfigRebootMaxAttempts = 3;

/** A "Normal but no data" (silent) episode is logged to the event log and shown
 *  as "NO DATA" on the dashboard once it has lasted this long, so trivial 1-2s
 *  hiccups don't spam either. */
static const uint32_t kNoDataLogSec = 3;
static const int64_t kStartupMissingGraceNs = 30000000000LL;

/** The Error reboot budget (kErrorRebootMaxAttempts) is cleared only after the
 *  lidar has been out of Error this long. A reboot cycles through Init/Normal
 *  for a few ticks; clearing on any single non-Error tick would reset the
 *  budget every cycle and bypass the "max attempts then manual" stop. */
static const uint32_t kErrorClearAfterSec = 60;

/** Timer callback (runs on the AsyncSpinner thread, independent of the data
 *  loop so it keeps updating even if a lidar stops sending). Publishes a
 *  preformatted dashboard of all connected lidars. */
void StatsTimerCb(const ros::TimerEvent &) {
  if (g_read_lidar == nullptr) {
    return;
  }
  /** Verify/retry every in-progress mode switch from actual heartbeat state;
   *  this also bounds Normal requests whose ACK/state event was lost. */
  g_read_lidar->TickSleepModeVerification();
  /** A device can keep broadcasting after its control/handshake service has
   *  wedged. Track that separately from an ordinary disconnected device and,
   *  when auto recovery is enabled, perform bounded local-session resets. */
  g_read_lidar->TickHandshakeRecovery(g_auto_recover);
  g_read_lidar->TickWakeDropoutRecovery(g_auto_recover);
  g_read_lidar->TickNormalDropoutRecovery(g_auto_recover);
  static uint64_t prev_recv[kMaxLidarCount] = {0};
  static uint64_t prev_pub[kMaxLidarCount] = {0};
  static uint64_t prev_connection_generation[kMaxLidarCount] = {0};
  static bool ever_seen[kMaxLidarCount] = {false};
  static char last_bcode[kMaxLidarCount][kBdCodeSize + 1] = {{0}};
  static uint32_t zero_secs[kMaxLidarCount] = {0};      /**< stage timer of the current stall (reset when a recovery cycle recycles) */
  static uint32_t nodata_secs[kMaxLidarCount] = {0};    /**< whole-episode stall duration (display + NODATA/DATABACK log) */
  static uint8_t recover_stage[kMaxLidarCount] = {0};   /**< 0=ok 1=restarted sampling 2=rebooted */
  static uint32_t error_secs[kMaxLidarCount] = {0};     /**< consecutive 1s ticks in Error state */
  static uint32_t error_free_secs[kMaxLidarCount] = {0}; /**< consecutive non-Error ticks, for clearing the attempt budget */
  static uint8_t error_reboots[kMaxLidarCount] = {0};   /**< reboots attempted this Error episode */
  static uint32_t config_secs[kMaxLidarCount] = {0};    /**< consecutive ticks stuck in Config */
  static uint8_t config_reboots[kMaxLidarCount] = {0};  /**< bounded reboots for the Config episode */
  static uint32_t config_healthy_secs[kMaxLidarCount] = {0}; /**< sustained published recovery before budget reset */
  static bool nodata_logged[kMaxLidarCount] = {false};  /**< a NODATA onset event has been logged for the current silent episode */
  static uint32_t emitted_power_cycle_count[kMaxLidarCount] = {0};
  /** Display-only rolling windows.  They never drive recovery or relay
   *  decisions; broadcast-code isolation prevents a reused handle from
   *  inheriting another physical lidar's trend. */
  static DashboardMetrics dashboard_metrics[kMaxLidarCount];

  /** Whitelist-only startup supervision gives a configured device a visible
   *  identity even when the SDK has never assigned it a handle. */
  static bool startup_trackers_initialized = false;
  static std::vector<StartupMissingTracker> startup_trackers;
  if (!startup_trackers_initialized) {
    const std::vector<std::string> whitelist =
        g_read_lidar->GetWhitelistBroadcastCodes();
    startup_trackers.reserve(whitelist.size());
    for (const std::string &code : whitelist) {
      StartupMissingTracker tracker;
      tracker.broadcast_code = code;
      tracker.absent_since_ns = g_driver_started_ns;
      tracker.absent_since_wall_s = g_driver_started_wall_s;
      startup_trackers.push_back(tracker);
    }
    startup_trackers_initialized = true;
  }
  std::vector<bool> startup_present(startup_trackers.size(), false);
  std::vector<bool> startup_healthy(startup_trackers.size(), false);
  std::vector<bool> startup_dashboard_row(startup_trackers.size(), false);

  int64_t now_ns = std::chrono::steady_clock::now().time_since_epoch().count();

  /** Snapshot pacing for the persistent health log: one row per lidar every
   *  snapshot_period_s (this timer ticks at 1 Hz). Events are logged elsewhere,
   *  edge-triggered. Snapshot counters may be differenced only within one
   *  continuous connection; ResetLidar clears them at a reconnect boundary. */
  HealthLogger &hlog = HealthLogger::Get();
  static int snap_counter = 0;
  bool do_snapshot = false;
  if (hlog.enabled() && ++snap_counter >= hlog.snapshot_period_s()) {
    snap_counter = 0;
    do_snapshot = true;
  }

  std::ostringstream table;
  std::ostringstream active_alerts;
  std::ostringstream process_history;
  static const char kDashboardRowFormat[] =
      "%-2.2s  %-15.15s  %-20.20s  %-10.10s  %6.6s  %7.7s  %7.7s  "
      "%-10.10s  %11.11s  %5.5s\n";
  char table_header[160];
  snprintf(table_header, sizeof(table_header), kDashboardRowFormat, "ID",
           "broadcast_code", "NOW", "TREND", "recv/s", "loss60",
           "qdrop60", "HW", "link_up", "HS60");
  table << table_header;
  bool any_active_alert = false;
  bool any_process_history = false;
  uint32_t known_count = 0;
  uint32_t trend_count[kDashboardTrendStable + 1] = {0};
  for (uint8_t h = 0; h < kMaxLidarCount; h++) {
    /** Counters are written by the ingest/publish threads under data_lock_.
     *  Take one coherent snapshot so the watchdog never drives hardware from
     *  a torn/racing 64-bit read. */
    LidarConnectState connect_state;
    DeviceInfo info;
    LidarPacketStatistic st;
    uint64_t connection_generation = 0;
    {
      std::lock_guard<std::mutex> lock(g_read_lidar->data_lock_[h]);
      const LidarDevice &live = g_read_lidar->lidars_[h];
      connect_state = live.connect_state;
      info = live.info;
      st = live.statistic_info;
      /** This generation changes under the same data lock which resets or
       *  installs the per-connection packet counters.  It therefore detects a
       *  fast disconnect+reconnect even when the new counter has already
       *  climbed above the previous 1 Hz sample. */
      connection_generation = g_read_lidar->GetConnectionGeneration(h);
    }
    LdsLidar::LinkStat ls;
    {
      std::lock_guard<std::mutex> lock(g_read_lidar->link_stat_lock_[h]);
      ls = g_read_lidar->link_stat_[h];
    }
    const bool sdk_connected = (connect_state != kConnectStateOff);
    const char *observed_bcode = nullptr;
    if (ls.broadcast_code[0] != '\0' &&
        (ls.last_broadcast_ns != 0 || ls.connect_since_ns != 0)) {
      /** Do not require a successful first handshake before showing a device.
       *  Prefer LinkStat's physical identity during split connect/disconnect
       *  transitions; the problematic BROADCAST_ONLY case otherwise never
       *  gets a row and a reused handle could briefly bounce back to old data. */
      observed_bcode = ls.broadcast_code;
    } else if (sdk_connected && info.broadcast_code[0] != '\0') {
      observed_bcode = info.broadcast_code;
    }
    if (observed_bcode != nullptr) {
      const bool identity_changed =
          last_bcode[h][0] != '\0' &&
          strncmp(last_bcode[h], observed_bcode, kBdCodeSize) != 0;
      if (identity_changed) {
        /** A reused SDK handle is a new physical lidar.  Clear every local
         *  watchdog/display budget as well as the rolling windows; otherwise
         *  the new device could inherit an old recovery stage or suppress its
         *  first power request behind emitted_power_cycle_count. */
        prev_recv[h] = 0;
        prev_pub[h] = 0;
        prev_connection_generation[h] = connection_generation;
        zero_secs[h] = 0;
        nodata_secs[h] = 0;
        recover_stage[h] = 0;
        error_secs[h] = 0;
        error_free_secs[h] = 0;
        error_reboots[h] = 0;
        config_secs[h] = 0;
        config_reboots[h] = 0;
        config_healthy_secs[h] = 0;
        nodata_logged[h] = false;
        emitted_power_cycle_count[h] = 0;
        dashboard_metrics[h].Reset();
      }
      ever_seen[h] = true;
      strncpy(last_bcode[h], observed_bcode, kBdCodeSize);
      last_bcode[h][kBdCodeSize] = '\0';
    }
    /** Skip handles that have never connected; but keep showing a lidar once
     *  seen, so a disconnect is loudly visible (DISCONNECTED) instead of the
     *  row silently vanishing. */
    if (!ever_seen[h]) {
      continue;
    }
    /** The SDK connect callback updates LinkStat and LidarDevice in separate
     *  critical sections.  Recovery commands require both snapshots to name
     *  the same non-empty physical identity; during the split transition we
     *  intentionally skip one watchdog tick instead of targeting a stale or
     *  newly reused handle. */
    const bool watchdog_identity_matches =
        info.broadcast_code[0] != '\0' && ls.broadcast_code[0] != '\0' &&
        strncmp(info.broadcast_code, ls.broadcast_code, kBdCodeSize) == 0;
    const bool watchdog_connected =
        sdk_connected && ls.connect_since_ns != 0 &&
        watchdog_identity_matches;
    uint32_t disc = ls.disconnect_count;
    bool broadcast_recent =
        ls.last_broadcast_ns != 0 &&
        now_ns - ls.last_broadcast_ns <=
            LdsLidar::HandshakeBroadcastFreshNs();
    ErrorMessage em;
    em.error_code = ls.health_code;
    const char *temp = TempStr(em.lidar_error_code.temp_status);
    const char *fan = FanStr(em.lidar_error_code.fan_status);
    const char *motor = MotorStr(em.lidar_error_code.motor_status);
    /** cumulative loss% since connect: total_loss / (total_recv + total_loss) */
    uint64_t tot = (uint64_t)st.receive_packet_count + st.loss_packet_count;
    /** Extra fields the snapshot log wants (MotorStr also maps the 3-level
     *  system_status: 0/1/2 -> OK/WARN/ERR!). */
    unsigned dirty = em.lidar_error_code.dirty_warn;
    const char *sys = MotorStr(em.lidar_error_code.system_status);
    double loss_pct_d = tot ? (100.0 * st.loss_packet_count / tot) : 0.0;
    char line[256];
    bool publishing_now = false;
    bool transition_active = false;
    uint64_t row_recv = 0;
    const char *row_state = "?";
    if (watchdog_connected) {
      const bool new_connection =
          connection_generation != prev_connection_generation[h];
      row_recv = (new_connection || st.receive_packet_count < prev_recv[h])
                     ? st.receive_packet_count
                     : st.receive_packet_count - prev_recv[h];
      uint64_t d_pub =
          (new_connection || st.publish_packet_count < prev_pub[h])
              ? st.publish_packet_count
              : st.publish_packet_count - prev_pub[h];
      publishing_now =
          (connect_state == kConnectStateSampling && d_pub > 0);
      prev_recv[h] = st.receive_packet_count;
      prev_pub[h] = st.publish_packet_count;
      prev_connection_generation[h] = connection_generation;

      /** Watch "should be DELIVERING points but is not" -- keyed on published
       *  packets, not received ones. A lidar stuck mid-configure keeps
       *  receiving into a full queue that nobody consumes (recv/s normal,
       *  drop 100%, zero ROS output; field-confirmed), which a recv-based
       *  check is blind to. PowerSaving/Standby/Init/Error legitimately
       *  produce nothing, and a lidar inside a planned mode switch is left to
       *  the mode verify/retry machinery instead of this watchdog. */
      transition_active = g_read_lidar->IsModeTransitionActive(h);
      /** Config is explicitly excluded: starting/rebooting while coordinate,
       *  return-mode, IMU or extrinsic commands are still pending can publish
       *  data with only part of the requested configuration applied. On is
       *  retained because StartSampleCb uses it after a start timeout, which is
       *  the field-confirmed state the restart path must recover. */
      bool should_stream = (info.state == kLidarStateNormal) &&
                           !transition_active &&
                           (connect_state != kConnectStateConfig);
      bool configuring = (info.state == kLidarStateNormal) &&
                          (connect_state == kConnectStateConfig);
      if (configuring) {
        config_secs[h]++;
      } else {
        config_secs[h] = 0;
      }
      if (connect_state == kConnectStateSampling && d_pub > 0) {
        if (config_reboots[h] != 0 &&
            ++config_healthy_secs[h] >= kErrorClearAfterSec) {
          config_reboots[h] = 0;
          config_healthy_secs[h] = 0;
        }
      } else {
        config_healthy_secs[h] = 0;
      }
      if (should_stream && d_pub == 0) {
        zero_secs[h]++;
        nodata_secs[h]++;
        /** Log the onset of a "Normal but silent" episode (once), with the
         *  wall-clock time -- so afterwards you can see exactly when a lidar
         *  went silent (e.g. correlate a slow wake with the scheduler log). */
        if (!nodata_logged[h] && nodata_secs[h] >= kNoDataLogSec) {
          hlog.LogEvent(h, last_bcode[h], "NODATA", "");
          nodata_logged[h] = true;
        }
      } else {
        /** Episode ended: if we logged its onset, record how long it stayed
         *  silent. d_pub>0 => data resumed; otherwise the state left Normal
         *  (slept / errored / planned switch started). */
        if (nodata_logged[h]) {
          char det[40];
          snprintf(det, sizeof(det), "silent %us%s", nodata_secs[h],
                   d_pub > 0 ? "" : " (left Normal)");
          hlog.LogEvent(h, last_bcode[h], "DATABACK", det);
          nodata_logged[h] = false;
        }
        zero_secs[h] = 0;
        nodata_secs[h] = 0;
        recover_stage[h] = 0;
      }

      /** (B) Two-stage auto-recovery for a "Normal but not publishing" stall.
       *  RestartSampling also re-arms a lidar whose earlier start-sampling
       *  timed out and left it demoted out of the Sampling state. */
      if (g_auto_recover && should_stream) {
        if (recover_stage[h] == 0 && zero_secs[h] >= 5 &&
            ((zero_secs[h] - 5) % 5) == 0) {
          livox_status s = g_read_lidar->RequestRestartSampling(h);
          if (s == kStatusSuccess) {
            ROS_WARN("[LivoxRecover] Lidar[%d] no published data for 5s -> "
                     "restart sampling", h);
            recover_stage[h] = 1;
            zero_secs[h] = 5;  // preserve a full 10s verification window
          } else {
            ROS_WARN("[LivoxRecover] Lidar[%d] restart sampling was not "
                     "accepted: %d", h, s);
          }
        } else if (recover_stage[h] == 1 && zero_secs[h] >= 15 &&
                   ((zero_secs[h] - 15) % 5) == 0) {
          /** Re-check the planned-mode guard atomically with enqueue. A sleep
           *  service can win after the earlier transition snapshot; in that
           *  case the watchdog stands down, while manual reboot remains an
           *  explicit override through RequestLidarReboot(). */
          livox_status s =
              g_read_lidar->RequestLidarRebootIfModeIdle(h);
          if (s == kStatusSuccess) {
            int64_t now_wall = (int64_t)time(nullptr);
            {
              std::lock_guard<std::mutex> lock(
                  g_read_lidar->link_stat_lock_[h]);
              LdsLidar::LinkStat &live = g_read_lidar->link_stat_[h];
              live.recover_reboot_count++;
              live.recover_last_wall_s = now_wall;
              ls.recover_reboot_count = live.recover_reboot_count;
              ls.recover_last_wall_s = live.recover_last_wall_s;
            }
            ROS_WARN("[LivoxRecover] Lidar[%d] still no published data for 15s "
                     "-> reboot", h);
            hlog.LogEvent(h, last_bcode[h], "REBOOT", "no-data 15s");
            recover_stage[h] = 2;
            zero_secs[h] = 15;  // preserve the post-reboot observation window
          } else {
            ROS_WARN("[LivoxRecover] Lidar[%d] no-data reboot was not "
                     "accepted: %d", h, s);
          }
        } else if (recover_stage[h] == 2 && zero_secs[h] >= 45) {
          /** Reboot didn't help; allow another cycle. Reset the stage timer
           *  too -- leaving it running keeps every threshold permanently
           *  exceeded and degrades the cycle into a restart/reboot every
           *  tick (a 3-second reboot storm). */
          recover_stage[h] = 0;
          zero_secs[h] = 0;
        }
      }

      /** (B2) Configuration recovery. Config is deliberately excluded from
       *  the no-data path above: issuing StartSampling while only some command
       *  bits completed can change coordinates/return semantics mid-stream.
       *  If callbacks never complete, use slow, bounded single-lidar reboots. */
      if (g_auto_recover && configuring) {
        if (config_reboots[h] < kConfigRebootMaxAttempts) {
          uint32_t due = (config_reboots[h] == 0)
                             ? kConfigRebootDelaySec
                             : kConfigRebootCooldownSec;
          if (config_secs[h] >= due &&
              ((config_secs[h] - due) % 5) == 0) {
            livox_status s = g_read_lidar->RequestLidarReboot(h);
            if (s == kStatusSuccess) {
              uint32_t stuck_secs = config_secs[h];
              int64_t now_wall = (int64_t)time(nullptr);
              {
                std::lock_guard<std::mutex> lock(
                    g_read_lidar->link_stat_lock_[h]);
                LdsLidar::LinkStat &live = g_read_lidar->link_stat_[h];
                live.recover_reboot_count++;
                live.recover_last_wall_s = now_wall;
                ls.recover_reboot_count = live.recover_reboot_count;
                ls.recover_last_wall_s = live.recover_last_wall_s;
              }
              config_reboots[h]++;
              config_secs[h] = 0;
              ROS_WARN("[LivoxRecover] Lidar[%d] stuck in Config %us -> "
                       "reboot (attempt %u/%u)", h, stuck_secs,
                       static_cast<unsigned>(config_reboots[h]),
                       kConfigRebootMaxAttempts);
              char rb[48];
              snprintf(rb, sizeof(rb), "Config %us attempt %u/%u",
                       stuck_secs,
                       static_cast<unsigned>(config_reboots[h]),
                       kConfigRebootMaxAttempts);
              hlog.LogEvent(h, last_bcode[h], "REBOOT", rb);
            } else {
              ROS_WARN("[LivoxRecover] Lidar[%d] Config reboot was not "
                       "accepted: %d", h, s);
            }
          }
        } else if ((config_secs[h] % 30) == 0) {
          ROS_ERROR("[LivoxRecover] Lidar[%d] still in Config after %u "
                    "reboots; manual intervention needed", h,
                    kConfigRebootMaxAttempts);
        }
      }

      /** (C) Error-state auto-recovery. A lidar reporting Error (e.g. a
       *  recoverable motor fault) never counts as "streaming", so path (B)
       *  above ignores it. Reboot just this lidar on a bounded schedule. */
      bool in_error = (info.state == kLidarStateError);
      if (in_error) {
        error_secs[h]++;
        error_free_secs[h] = 0;
      } else {
        error_secs[h] = 0;      /** left Error (recovered or other state) */
        /** Clear the reboot budget only after a SUSTAINED recovery
         *  (kErrorClearAfterSec out of Error). A reboot passes through
         *  Init/Normal for a few ticks; clearing on any one of them would
         *  reset the budget every cycle and turn "max 3 attempts then
         *  manual" into an endless reboot loop. */
        if (error_reboots[h] != 0 && ++error_free_secs[h] >= kErrorClearAfterSec) {
          error_reboots[h] = 0;
          error_free_secs[h] = 0;
        }
      }
      if (g_auto_recover && in_error) {
        if (error_reboots[h] < kErrorRebootMaxAttempts) {
          /** reboot #n is due at delay + n*cooldown seconds in Error
           *  (3s, 43s, 83s for delay=3, cooldown=40). error_secs is frozen
           *  while the lidar is disconnected mid-reboot, so the real gap is
           *  the cooldown plus reconnect time. */
          uint32_t due = kErrorRebootDelaySec +
                         error_reboots[h] * kErrorRebootCooldownSec;
          if (error_secs[h] >= due &&
              ((error_secs[h] - due) % 5) == 0) {
            livox_status s = g_read_lidar->RequestLidarReboot(h);
            if (s == kStatusSuccess) {
              int64_t now_wall = (int64_t)time(nullptr);
              {
                std::lock_guard<std::mutex> lock(
                    g_read_lidar->link_stat_lock_[h]);
                LdsLidar::LinkStat &live = g_read_lidar->link_stat_[h];
                live.recover_reboot_count++;
                live.recover_last_wall_s = now_wall;
                ls.recover_reboot_count = live.recover_reboot_count;
                ls.recover_last_wall_s = live.recover_last_wall_s;
              }
              error_reboots[h]++;
              ROS_WARN("[LivoxRecover] Lidar[%d] in Error %us -> reboot "
                       "(attempt %u/%u)", h, error_secs[h],
                       static_cast<unsigned>(error_reboots[h]),
                       kErrorRebootMaxAttempts);
              char rb[40];
              snprintf(rb, sizeof(rb), "Error %us attempt %u/%u",
                       error_secs[h],
                       static_cast<unsigned>(error_reboots[h]),
                       kErrorRebootMaxAttempts);
              hlog.LogEvent(h, last_bcode[h], "REBOOT", rb);
            } else {
              ROS_WARN("[LivoxRecover] Lidar[%d] Error reboot was not "
                       "accepted: %d", h, s);
            }
          }
        } else if ((error_secs[h] % 30) == 0) {
          /** Exhausted attempts: stop rebooting, warn loudly every 30s. */
          ROS_ERROR("[LivoxRecover] Lidar[%d] still in Error after %u reboots; "
                    "manual intervention needed (likely fan/motor hardware "
                    "fault)", h, kErrorRebootMaxAttempts);
        }
      }

      /** A real firmware Error outranks the host-side Config phase.  Showing
       *  Config first would hide the fault and incorrectly classify a lidar
       *  whose error recovery budget is already running as RECOVERING. */
      row_state = info.state == kLidarStateError
                      ? "Error"
                      : connect_state == kConnectStateConfig
                            ? "Config"
                            : LidarStateStr(info.state);
      if (should_stream && nodata_secs[h] >= kNoDataLogSec) {
        row_state = "NO DATA";
      }
      if (do_snapshot) {
        hlog.LogSnapshot(h, last_bcode[h], row_state, temp, fan, motor, dirty, sys,
                         st.receive_packet_count, st.loss_packet_count,
                         st.queue_drop_count, loss_pct_d, disc);
      }
    } else {
      prev_recv[h] = prev_pub[h] = 0;
      prev_connection_generation[h] = connection_generation;
      zero_secs[h] = 0;
      nodata_secs[h] = 0;
      recover_stage[h] = 0;
      config_secs[h] = 0;
      config_healthy_secs[h] = 0;
      /** Close any open silent episode; the DISCONNECT event already marks the
       *  transition, so no separate DATA row is needed here. */
      nodata_logged[h] = false;
      const std::string offline_state =
          DashboardNowState(false, broadcast_recent, "DISCONNECTED", ls);
      if (do_snapshot) {
        hlog.LogSnapshot(h, last_bcode[h], offline_state.c_str(), "-", "-", "-", 0, "-",
                         st.receive_packet_count, st.loss_packet_count,
                         st.queue_drop_count, loss_pct_d, disc);
      }
    }
    const bool normal_healthy =
        watchdog_connected && info.state == kLidarStateNormal &&
        connect_state == kConnectStateSampling && publishing_now &&
        !transition_active;
    g_read_lidar->ObserveNormalPublishing(
        h, normal_healthy, connection_generation,
        watchdog_connected ? info.broadcast_code : nullptr);
    const char *startup_identity =
        ls.broadcast_code[0] != '\0'
            ? ls.broadcast_code
            : (info.broadcast_code[0] != '\0' ? info.broadcast_code : nullptr);
    if (startup_identity != nullptr) {
      for (std::size_t i = 0; i < startup_trackers.size(); ++i) {
        if (startup_trackers[i].broadcast_code == startup_identity) {
          startup_present[i] = startup_present[i] || watchdog_connected ||
                               broadcast_recent;
          startup_healthy[i] = startup_healthy[i] || normal_healthy;
          startup_dashboard_row[i] = true;
        }
      }
    }
    /** Revalidate a pending request from a freshly locked handle+episode
     *  snapshot.  ROS publication happens after releasing the SDK callback
     *  lock; the relay manager performs its own final live-state precheck for
     *  any cancellation which races this immutable committed-edge snapshot. */
    const int64_t expected_power_episode = ls.broadcast_only_since_ns;
    const uint64_t expected_wake_request_id = ls.wake_request_id;
    const int64_t expected_wake_dropout = ls.wake_dropout_since_ns;
    const uint64_t expected_normal_generation =
        ls.normal_dropout_generation;
    const int64_t expected_normal_silence = ls.normal_dropout_since_ns;
    const LdsLidar::PowerCycleReason expected_power_reason =
        ls.power_cycle_reason;
    const uint32_t expected_power_count = ls.power_cycle_required_count;
    const bool expected_power_edge =
        IsPowerCycleRequired(ls) &&
        expected_power_count > emitted_power_cycle_count[h];
    bool publish_power_edge = false;
    {
      std::lock_guard<std::mutex> lock(g_read_lidar->link_stat_lock_[h]);
      const LdsLidar::LinkStat &live = g_read_lidar->link_stat_[h];
      broadcast_recent =
          live.last_broadcast_ns != 0 &&
          now_ns - live.last_broadcast_ns <=
              LdsLidar::HandshakeBroadcastFreshNs();
      publish_power_edge =
          expected_power_edge &&
          IsPowerCycleRequired(live) &&
          live.power_cycle_reason == expected_power_reason &&
          (expected_power_reason ==
                   LdsLidar::kPowerCycleReasonHandshakeStuck
               ? live.broadcast_only_since_ns == expected_power_episode
               : expected_power_reason ==
                         LdsLidar::kPowerCycleReasonWakeDropout
                     ? live.wake_request_id == expected_wake_request_id &&
                           live.wake_dropout_since_ns == expected_wake_dropout
                     : expected_power_reason ==
                               LdsLidar::kPowerCycleReasonNormalDropout &&
                           live.normal_dropout_generation ==
                               expected_normal_generation &&
                           live.normal_dropout_since_ns ==
                               expected_normal_silence) &&
          live.power_cycle_required_count == expected_power_count;
      /** Keep the later footer coherent with the state just published. */
      ls = live;
    }
    const char *publish_bcode =
        ls.broadcast_code[0] ? ls.broadcast_code : last_bcode[h];
    const std::string dashboard_bcode(publish_bcode);
    const bool identity_matches =
        info.broadcast_code[0] != '\0' && !dashboard_bcode.empty() &&
        strncmp(info.broadcast_code, dashboard_bcode.c_str(), kBdCodeSize) == 0;
    /** Connect/disconnect callbacks update LinkStat before LidarDevice.  Require
     *  both snapshots (and their identities) to agree, so the dashboard and
     *  recovery-state topic never emit an impossible NORMAL + link_up=-- frame. */
    const bool dashboard_connected =
        sdk_connected && ls.connect_since_ns != 0 && identity_matches;
    PublishRecoveryState(h, publish_bcode, dashboard_connected, connect_state,
                         info.state, ls, broadcast_recent,
                         dashboard_connected && publishing_now,
                         st.publish_packet_count);
    if (publish_power_edge) {
      PublishPowerCycleRequest(h, publish_bcode, ls, broadcast_recent);
      emitted_power_cycle_count[h] = ls.power_cycle_required_count;
    }
    /** Build every user-visible status from the same final LinkStat snapshot.
     *  The rolling metrics and labels below are display-only and never feed
     *  watchdog or relay decisions. */
    /** Link-derived durations must be computed after the final LinkStat copy,
     *  otherwise NOW/HS60 and link_up can disagree within one dashboard frame. */
    std::string outage_duration;
    if (ls.last_disconnect_ns == 0) {
      outage_duration = "--";
    } else if (dashboard_connected) {
      outage_duration =
          FmtDur(ls.connect_since_ns - ls.last_disconnect_ns);
    } else {
      outage_duration = FmtDur(now_ns - ls.last_disconnect_ns);
    }
    /** Link uptime is heartbeat uptime, not point-streaming uptime; it remains
     *  valid while a connected lidar is intentionally sleeping. */
    const std::string link_up = dashboard_connected
                                     ? FmtDur(now_ns - ls.connect_since_ns)
                                     : "--";
    const std::string health_tags =
        dashboard_connected ? CurrentHealthTags(ls.health_code) : "-";
    const std::string health_cell =
        health_tags.size() <= 10 ? health_tags : "MULTI";
    std::string display_state =
        DashboardNowState(dashboard_connected, broadcast_recent, row_state, ls);
    if (!dashboard_connected && !broadcast_recent) {
      for (const StartupMissingTracker &tracker : startup_trackers) {
        if (tracker.active && tracker.broadcast_code == dashboard_bcode) {
          display_state = "STARTUP_MISSING";
          break;
        }
      }
    }

    DashboardCounters dashboard_counters;
    dashboard_counters.received_packets = st.receive_packet_count;
    dashboard_counters.lost_packets = st.loss_packet_count;
    dashboard_counters.queue_drops = st.queue_drop_count;
    dashboard_counters.handshake_ack_attempts = ls.handshake_success_count;
    dashboard_counters.handshake_timeout_attempts =
        ls.handshake_timeout_count;
    dashboard_counters.handshake_rejected_attempts =
        ls.handshake_rejected_count;
    dashboard_counters.handshake_network_attempts =
        ls.handshake_network_error_count;
    dashboard_counters.handshake_protocol_attempts =
        ls.handshake_protocol_error_count;
    dashboard_counters.disconnect_episodes = ls.disconnect_count;
    dashboard_counters.handshake_stuck_episodes = ls.handshake_stuck_count;
    dashboard_counters.wake_dropout_episodes = ls.wake_dropout_count;
    dashboard_counters.normal_dropout_episodes = ls.normal_dropout_count;
    dashboard_counters.power_reached_episodes =
        ls.power_cycle_required_episode_count;
    dashboard_counters.power_request_edges =
        ls.power_cycle_required_count;
    dashboard_counters.fault_episodes = ls.fault_count;
    dashboard_counters.reboot_actions = ls.recover_reboot_count;
    dashboard_counters.mode_fail_episodes = ls.mode_fail_count;
    const DashboardWindow window = dashboard_metrics[h].Update(
        dashboard_bcode, connection_generation, now_ns, dashboard_counters);

    DashboardLiveSignals live_signals;
    const bool power_reason_handshake =
        display_state == "POWER_CYCLE_REQUIRED" &&
        ls.power_cycle_reason ==
            LdsLidar::kPowerCycleReasonHandshakeStuck;
    const bool power_reason_wake =
        display_state == "POWER_CYCLE_REQUIRED" &&
        ls.power_cycle_reason == LdsLidar::kPowerCycleReasonWakeDropout;
    const bool power_reason_normal =
        display_state == "POWER_CYCLE_REQUIRED" &&
        ls.power_cycle_reason == LdsLidar::kPowerCycleReasonNormalDropout;
    const bool handshake_incident =
        display_state == "HANDSHAKE_STUCK" ||
        power_reason_handshake;
    const bool wake_incident =
        display_state == "WAKE_NO_BROADCAST" ||
        display_state == "WAKE_DROPOUT" || power_reason_wake;
    const bool normal_dropout_incident =
        display_state == "NORMAL_NO_BROADCAST" ||
        display_state == "NORMAL_DROPOUT" ||
        display_state == "BROADCAST_RETURNING" || power_reason_normal;
    const bool config_exhausted =
        display_state == "CONFIG" && g_auto_recover &&
        config_reboots[h] >= kConfigRebootMaxAttempts;
    const bool current_incident =
        display_state == "DISCONNECTED" ||
        display_state == "STARTUP_MISSING" ||
        display_state == "NO_DATA" ||
        display_state == "ERROR" || display_state == "?" ||
        handshake_incident || wake_incident || normal_dropout_incident ||
        config_exhausted ||
        (dashboard_connected && health_tags != "OK");
    live_signals.incident_active = current_incident;
    live_signals.recovery_active =
        display_state == "BROADCAST_ONLY" ||
        (display_state == "CONFIG" && !config_exhausted) ||
        display_state == "INIT" ||
        (display_state == "NORMAL" && !publishing_now);
    live_signals.intentionally_idle =
        display_state == "POWER_SAVING" || display_state == "STANDBY";
    const DashboardTrend trend = EvaluateTrend(window, live_signals);
    trend_count[static_cast<unsigned>(trend)]++;
    known_count++;

    char loss60_text[16];
    const std::string recv_cell =
        dashboard_connected ? DashboardCountCell(row_recv, 6) : "-";
    const std::string id_cell = DashboardCountCell(h, 2);
    const std::string qdrop_cell =
        DashboardCountCell(window.queue_drops_60s, 7);
    const std::string hs60_cell =
        DashboardCountCell(window.handshake_timeout_60s, 5);
    if (window.loss_60s_has_data) {
      snprintf(loss60_text, sizeof(loss60_text), "%.2f%%",
               window.loss_60s_percent);
    } else {
      snprintf(loss60_text, sizeof(loss60_text), "--");
    }
    snprintf(line, sizeof(line), kDashboardRowFormat, id_cell.c_str(),
             dashboard_bcode.c_str(), display_state.c_str(),
             DashboardTrendName(trend), recv_cell.c_str(), loss60_text,
             qdrop_cell.c_str(), health_cell.c_str(), link_up.c_str(),
             hs60_cell.c_str());
    table << line;

    if (current_incident) {
      any_active_alert = true;
      const bool critical =
          display_state == "POWER_CYCLE_REQUIRED" ||
          (display_state == "STARTUP_MISSING" && g_auto_recover);
      active_alerts << "  " << (critical ? "[CRIT]" : "[ALERT]") << " L"
                    << static_cast<unsigned>(h) << " " << dashboard_bcode
                    << " " << display_state;
      if (critical) {
        active_alerts
            << " reason="
            << (display_state == "STARTUP_MISSING"
                    ? "STARTUP_MISSING"
                    : PowerCycleReasonStr(ls.power_cycle_reason));
      }
      if (wake_incident && ls.wake_dropout_since_ns != 0) {
        const int64_t wake_age_ns = now_ns - ls.wake_dropout_since_ns;
        active_alerts << " age=" << FmtDur(wake_age_ns) << "\n";
        active_alerts
            << "    wake: explicit PowerSaving/StandBy -> Normal; "
               "control=down; broadcast=absent";
        if (display_state == "WAKE_NO_BROADCAST") {
          const long long age_s = wake_age_ns / 1000000000LL;
          const long long remaining_s = age_s >= 10 ? 0 : 10 - age_s;
          active_alerts << "; confirming " << remaining_s
                        << "s before escalation";
        } else if (display_state == "WAKE_DROPOUT" && !g_auto_recover) {
          active_alerts << "; confirmed; detection only (auto_recover=off)";
        } else if (power_reason_wake) {
          active_alerts << "; confirmed; shared power-cycle request published";
        }
        active_alerts << "\n";
        active_alerts << "    wake request=" << ls.wake_request_id
                      << "; observation=60s; dropout-confirm=10s; "
                         "broadcast-handoff=3s/3frames\n";
      } else if (normal_dropout_incident &&
                 ls.normal_attributed_disconnect_ns != 0) {
        const int64_t normal_age_ns =
            now_ns - ls.normal_attributed_disconnect_ns;
        active_alerts << " age=" << FmtDur(normal_age_ns) << "\n";
        active_alerts
            << "    normal dropout: previously healthy Normal/Sampling "
               "publication >=30s; control=down; broadcast="
            << (broadcast_recent ? "returning" : "absent");
        if (display_state == "NORMAL_NO_BROADCAST" &&
            ls.normal_dropout_since_ns != 0) {
          const long long silence_s =
              (now_ns - ls.normal_dropout_since_ns) / 1000000000LL;
          active_alerts << "; confirming "
                        << (silence_s >= 5 ? 0 : 5 - silence_s)
                        << "s before escalation";
        } else if (display_state == "BROADCAST_RETURNING") {
          active_alerts
              << "; awaiting stable 3s/3-frame handoff to handshake recovery";
        } else if (display_state == "NORMAL_DROPOUT" && !g_auto_recover) {
          active_alerts << "; confirmed; detection only (auto_recover=off)";
        } else if (power_reason_normal) {
          active_alerts << "; shared power-cycle request published";
        }
        active_alerts << "\n";
        active_alerts
            << "    normal evidence: generation="
            << ls.normal_dropout_generation
            << "; healthy-since=" << FmtWall(ls.normal_healthy_since_wall_s)
            << "; silence-since=" << FmtWall(ls.normal_dropout_wall_s)
            << "\n";
      } else if (handshake_incident && ls.broadcast_only_since_ns != 0) {
        active_alerts << " age="
                      << FmtDur(now_ns - ls.broadcast_only_since_ns) << "\n";
        active_alerts << "    handshake: broadcast=alive; reset="
                      << HandshakeResetPhaseStr(ls.handshake_reset_phase);
        if (!g_auto_recover) {
          active_alerts << "; detection only; session reset disabled";
        } else if (ls.handshake_reset_phase ==
                       LdsLidar::kHandshakeResetRequested ||
                   ls.handshake_reset_phase ==
                       LdsLidar::kHandshakeResetQueued) {
          active_alerts << "; waiting SDK RESET completion";
        } else if (ls.handshake_reset_phase ==
                   LdsLidar::kHandshakeResetRejected) {
          active_alerts << "; hard-power escalation blocked";
        } else if (display_state == "POWER_CYCLE_REQUIRED") {
          active_alerts << "; power-cycle request published; see POWER "
                           "RECOVERY manager";
        } else if (ls.handshake_reset_phase ==
                   LdsLidar::kHandshakeResetCompleted) {
          active_alerts << "; post-reset observation";
        }
        active_alerts << "\n";
      } else if (display_state == "STARTUP_MISSING") {
        active_alerts
            << "\n    startup: configured whitelist member absent; "
               "synthetic handle=255; shared power-cycle supervision active\n";
      } else if (display_state == "DISCONNECTED") {
        active_alerts << "\n    link: broadcast=absent; outage="
                      << outage_duration << "\n";
      } else if (display_state == "NO_DATA") {
        const char *stage = recover_stage[h] == 0
                                ? "monitoring"
                                : recover_stage[h] == 1
                                      ? "restart-sampling sent"
                                      : "reboot sent";
        active_alerts << "\n    stream: silent=" << nodata_secs[h]
                      << "s; recovery=" << stage << "\n";
      } else if (display_state == "ERROR") {
        active_alerts << "\n    error: age=" << error_secs[h]
                      << "s; reboot actions this episode="
                      << static_cast<unsigned>(error_reboots[h]) << "/"
                      << kErrorRebootMaxAttempts
                       << "; auto-recover=" << (g_auto_recover ? "on" : "off")
                       << "\n";
      } else if (config_exhausted) {
        active_alerts << "\n    config: reboot budget exhausted "
                      << static_cast<unsigned>(config_reboots[h]) << "/"
                      << kConfigRebootMaxAttempts
                      << "; manual check required\n";
      } else {
        active_alerts << "\n";
      }
      if (dashboard_connected && health_tags != "OK") {
        active_alerts << "    hardware: " << health_tags << "\n";
      }
      if (ls.handshake_event_valid && handshake_incident) {
        active_alerts << "    last SDK event: "
                      << HandshakeEventStr(ls.last_handshake_event)
                      << "; detail=" << ls.last_handshake_detail << "\n";
      }
    }

    const bool handshake_history =
        ls.handshake_reset_count != 0 ||
        ls.handshake_reset_fail_count != 0 ||
        ls.handshake_stuck_count != 0 ||
        ls.handshake_power_cycle_episode_count != 0 ||
        ls.handshake_timeout_count != 0 ||
        ls.handshake_rejected_count != 0 ||
        ls.handshake_network_error_count != 0 ||
        ls.handshake_protocol_error_count != 0;
    const bool wake_history = ls.wake_dropout_count != 0 ||
                               ls.wake_power_cycle_episode_count != 0;
    const bool normal_dropout_history =
        ls.normal_dropout_count != 0 ||
        ls.normal_power_cycle_episode_count != 0;
    const bool hard_power_history =
        ls.power_cycle_required_episode_count != 0 ||
        ls.power_cycle_required_count != 0;
    const bool has_history =
        ls.disconnect_count != 0 || ls.temp_change_count != 0 ||
        ls.fault_count != 0 || ls.recover_reboot_count != 0 ||
        ls.mode_fail_count != 0 || handshake_history || wake_history ||
        normal_dropout_history || hard_power_history ||
        ls.planned_group_power_cycle_count != 0;
    if (has_history) {
      any_process_history = true;
      process_history << "  L" << static_cast<unsigned>(h) << " "
                      << dashboard_bcode << ":\n";
      if (ls.disconnect_count != 0) {
        process_history << "    link: disconnect episodes="
                        << ls.disconnect_count << "; outage duration="
                        << outage_duration << "; current link up=" << link_up
                        << "\n";
      }
      if (ls.planned_group_power_cycle_count != 0) {
        process_history << "    planned shared power cycles="
                        << ls.planned_group_power_cycle_count
                        << " (maintenance; excluded from disconnect/trend faults)\n";
      }
      if (handshake_history) {
        process_history << "    handshake attempts (SDK): ACK="
                        << ls.handshake_success_count
                        << " timeout=" << ls.handshake_timeout_count
                        << " rejected=" << ls.handshake_rejected_count
                        << "\n";
        process_history << "      network="
                        << ls.handshake_network_error_count
                        << " protocol=" << ls.handshake_protocol_error_count
                        << "\n";
        process_history << "    handshake failure episodes: stuck="
                        << ls.handshake_stuck_count
                        << "; escalated-to-power="
                        << ls.handshake_power_cycle_episode_count
                        << " (subset of stuck)\n";
        process_history << "    session reset actions: accepted="
                        << ls.handshake_reset_count
                        << "; rejected=" << ls.handshake_reset_fail_count
                        << "\n";
        if (ls.last_handshake_event_wall_s != 0) {
          process_history << "    last SDK event: "
                          << HandshakeEventStr(ls.last_handshake_event)
                          << " detail=" << ls.last_handshake_detail
                          << " ip="
                          << (ls.last_handshake_ip[0]
                                  ? ls.last_handshake_ip
                                  : "?")
                          << " at=" << FmtWall(ls.last_handshake_event_wall_s)
                          << "\n";
        }
      }
      if (wake_history) {
        process_history << "    wake-dropout episodes="
                        << ls.wake_dropout_count
                        << "; escalated-to-power="
                        << ls.wake_power_cycle_episode_count << "\n";
        process_history << "      cause: explicit low-power -> Normal, then "
                           "control+broadcast absent for 10s\n";
      }
      if (normal_dropout_history) {
        process_history << "    normal-dropout episodes="
                        << ls.normal_dropout_count
                        << "; escalated-to-power="
                        << ls.normal_power_cycle_episode_count << "\n";
        process_history
            << "      cause: >=30s healthy Normal/Sampling publication, then "
               "control+broadcast absent for 5s\n";
      }
      if (hard_power_history) {
        process_history << "    POWER_CYCLE_REQUIRED: episodes="
                        << ls.power_cycle_required_episode_count
                        << "; entries=" << ls.power_cycle_required_count
                        << " (all causes; entries may repeat within one episode)\n";
      }
      if (ls.fault_count != 0) {
        process_history << "    hardware fault episodes=" << ls.fault_count
                        << "; tags=" << FaultTags(ls.fault_code)
                        << "; last=" << FmtWall(ls.fault_wall_s) << "\n";
      }
      if (ls.temp_change_count != 0) {
        process_history << "    temperature state changes="
                        << ls.temp_change_count
                        << "; last=" << FmtWall(ls.temp_change_wall_s)
                        << "\n";
      }
      if (ls.recover_reboot_count != 0) {
        process_history << "    automatic reboot actions="
                        << ls.recover_reboot_count
                        << ", last=" << FmtWall(ls.recover_last_wall_s)
                        << "\n";
      }
      if (ls.mode_fail_count != 0) {
        process_history << "    mode failures: total=" << ls.mode_fail_count
                        << "; last-mode="
                        << DashboardModeName(ls.mode_fail_mode)
                        << ", last=" << FmtWall(ls.mode_fail_wall_s) << "\n";
      }
    }
  }

  for (std::size_t i = 0; i < startup_trackers.size(); ++i) {
    StartupMissingTracker &tracker = startup_trackers[i];
    if (startup_healthy[i]) {
      tracker.ever_healthy = true;
    }
    if (startup_present[i]) {
      if (tracker.active) {
        ROS_INFO("[LivoxRecover] configured lidar[%s] appeared; cancelling "
                 "STARTUP_MISSING",
                 tracker.broadcast_code.c_str());
      }
      tracker.absent_since_ns = 0;
      tracker.absent_since_wall_s = 0;
      tracker.required_at_wall_s = 0;
      tracker.active = false;
      tracker.request_emitted = false;
      continue;
    }
    /** Once a device has published healthily, its later failure belongs to the
     *  generation-bound NORMAL_DROPOUT path, never to synthetic handle 255. */
    if (tracker.ever_healthy) {
      continue;
    }
    if (tracker.absent_since_ns == 0) {
      tracker.absent_since_ns = now_ns;
      tracker.absent_since_wall_s = static_cast<int64_t>(time(nullptr));
    }
    if (now_ns < tracker.absent_since_ns ||
        now_ns - tracker.absent_since_ns < kStartupMissingGraceNs) {
      continue;
    }
    if (!tracker.active) {
      tracker.active = true;
      tracker.request_emitted = false;
      ++tracker.episode_count;
      tracker.required_at_wall_s = static_cast<int64_t>(time(nullptr));
      ROS_ERROR("[LivoxRecover] configured lidar[%s] STARTUP_MISSING after "
                "30s startup grace%s",
                tracker.broadcast_code.c_str(),
                g_auto_recover ? "; shared power cycle required"
                               : "; detection only");
      HealthLogger::Get().LogEvent(
          255, tracker.broadcast_code.c_str(), "STARTUP_MISSING",
          g_auto_recover ? "configured member absent for 30s"
                         : "configured member absent for 30s; detection only");
    }
    PublishStartupRecoveryState(tracker, g_auto_recover);
    if (g_auto_recover && !tracker.request_emitted) {
      PublishStartupPowerCycleRequest(tracker);
      tracker.request_emitted = true;
    }
    if (!startup_dashboard_row[i]) {
      char startup_line[256];
      snprintf(startup_line, sizeof(startup_line), kDashboardRowFormat, "S",
               tracker.broadcast_code.c_str(), "STARTUP_MISSING", "ACTIVE",
               "-", "--", "-", "-", "--", "-");
      table << startup_line;
      ++known_count;
      ++trend_count[kDashboardTrendActive];
      any_active_alert = true;
      active_alerts << "  " << (g_auto_recover ? "[CRIT]" : "[ALERT]")
                    << " L255 " << tracker.broadcast_code
                    << " STARTUP_MISSING";
      if (g_auto_recover) {
        active_alerts << " reason=STARTUP_MISSING";
      }
      active_alerts
          << " age=" << FmtDur(now_ns - tracker.absent_since_ns) << "\n"
          << "    startup: configured whitelist member has no connection or "
             "fresh broadcast; grace=30s; synthetic handle=255; auto-recover="
          << (g_auto_recover ? "on; shared power-cycle request published"
                             : "off; detection only")
          << "\n";
    }
  }

  std::ostringstream ss;
  ss << "===== Livox LiDAR Status (1 Hz) =====\n";
  ss << "SUMMARY: known=" << known_count
     << " | ATTENTION: ACTIVE=" << trend_count[kDashboardTrendActive]
     << " UNSTABLE=" << trend_count[kDashboardTrendUnstable]
     << " WATCH=" << trend_count[kDashboardTrendWatch]
     << " | TRANSITION: RECOVERING="
     << trend_count[kDashboardTrendRecovering]
     << " OBSERVE=" << trend_count[kDashboardTrendObserve]
     << " | OK: STABLE=" << trend_count[kDashboardTrendStable]
     << " IDLE=" << trend_count[kDashboardTrendIdle] << "\n";
  if (any_active_alert) {
    ss << "ACTIVE ALERTS:\n" << active_alerts.str();
  } else {
    ss << "ACTIVE ALERTS: none\n";
  }
  ss << "LEGEND: loss60=point-packet loss; qdrop60=local queue drops; "
        "HS60=SDK timeout attempts, not incidents (last 60s)\n";
  ss << "TREND: repeated episodes/actions use last 10m; PROCESS HISTORY is "
        "Driver-process cumulative only\n";
  ss << table.str();
  if (known_count == 0) {
    ss << "(no lidar seen yet)\n";
  }
  if (any_process_history) {
    ss << "PROCESS HISTORY (Driver process; resets on restart; not current alarms):\n"
       << process_history.str();
  }

  std_msgs::String msg;
  msg.data = ss.str();
  g_stats_pub.publish(msg);
}

int main(int argc, char **argv) {
  /** Ros related */
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                     ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  }
  ros::init(argc, argv, "livox_lidar_publisher");
  ros::NodeHandle livox_node;
  g_driver_instance_id = ros::WallTime::now().toNSec();
  g_driver_started_ns =
      std::chrono::steady_clock::now().time_since_epoch().count();
  g_driver_started_wall_s = static_cast<int64_t>(time(nullptr));

  ROS_INFO("Livox Ros Driver Version: %s", LIVOX_ROS_DRIVER_VERSION_STRING);
  /** Check sdk version */
  LivoxSdkVersion _sdkversion;
  GetLivoxSdkVersion(&_sdkversion);
  if (_sdkversion.major < kSdkVersionMajorLimit) {
    ROS_INFO("The SDK version[%d.%d.%d] is too low", _sdkversion.major,
             _sdkversion.minor, _sdkversion.patch);
    return 0;
  }

  /** Init default system parameter */
  int xfer_format = kPointCloud2Msg;
  int multi_topic = 0;
  int data_src = kSourceRawLidar;
  double publish_freq  = 10.0; /* Hz */
  int output_type      = kOutputToRos;
  std::string frame_id = "livox_frame";
  bool lidar_bag = true;
  bool imu_bag   = false;
  double max_distance  = 0.0; /* meters, 0 = disabled */

  livox_node.getParam("xfer_format", xfer_format);
  livox_node.getParam("multi_topic", multi_topic);
  livox_node.getParam("data_src", data_src);
  livox_node.getParam("publish_freq", publish_freq);
  livox_node.getParam("output_data_type", output_type);
  livox_node.getParam("frame_id", frame_id);
  livox_node.getParam("enable_lidar_bag", lidar_bag);
  livox_node.getParam("enable_imu_bag", imu_bag);
  livox_node.getParam("max_distance", max_distance);
  livox_node.getParam("auto_recover", g_auto_recover);

  /** Optional persistent CSV health logging (events + periodic snapshot) for
   *  long-run, unattended deployments. Off by default. */
  bool health_log = false;
  std::string health_log_dir;
  int health_log_snapshot_s = 600;
  livox_node.getParam("health_log", health_log);
  livox_node.getParam("health_log_dir", health_log_dir);
  livox_node.getParam("health_log_snapshot_s", health_log_snapshot_s);
  if (health_log) {
    HealthLogger::Get().Enable(health_log_dir, health_log_snapshot_s);
    ROS_INFO("Health logging ENABLED -> dir='%s', snapshot every %ds (events "
             "always edge-triggered)",
             health_log_dir.empty() ? "(node cwd, ~/.ros)"
                                    : health_log_dir.c_str(),
             health_log_snapshot_s);
    /** Marker row so multiple runs that share a day's file are easy to tell
     *  apart (the log is append-only and keyed on date, not on run/PID). */
    HealthLogger::Get().LogEvent(
        -1, "-", "STARTUP",
        std::string("driver ") + LIVOX_ROS_DRIVER_VERSION_STRING);
  } else {
    ROS_INFO("Health logging disabled (enable with health_log:=true)");
  }
  if (publish_freq > 100.0) {
    publish_freq = 100.0;
  } else if (publish_freq < 0.1) {
    publish_freq = 0.1;
  } else {
    publish_freq = publish_freq;
  }

  /** Lidar data distribute control and lidar data source set */
  Lddc *lddc = new Lddc(xfer_format, multi_topic, data_src, output_type,
                        publish_freq, frame_id, lidar_bag, imu_bag);
  lddc->SetRosNode(&livox_node);
  if (max_distance > 0.0) {
    lddc->SetMaxDistance(static_cast<float>(max_distance));
    ROS_INFO("Distance filter enabled: max_distance = %.2f m", max_distance);
  } else {
    ROS_INFO("Distance filter disabled");
  }

  int ret = 0;
  if (data_src == kSourceRawLidar) {
    ROS_INFO("Data Source is raw lidar.");

    std::string user_config_path;
    livox_node.getParam("user_config_path", user_config_path);
    ROS_INFO("Config file : %s", user_config_path.c_str());

    std::string cmdline_bd_code;
    livox_node.getParam("cmdline_str", cmdline_bd_code);

    std::vector<std::string> bd_code_list;
    ParseCommandlineInputBdCode(cmdline_bd_code.c_str(), bd_code_list);

    LdsLidar *read_lidar = LdsLidar::GetInstance(1000 / publish_freq);
    lddc->RegisterLds(static_cast<Lds *>(read_lidar));
    ret = read_lidar->InitLdsLidar(bd_code_list, user_config_path.c_str());
    if (!ret) {
      ROS_INFO("Init lds lidar success!");
      g_read_lidar = read_lidar;
    } else {
      ROS_ERROR("Init lds lidar fail!");
    }
  } else if (data_src == kSourceRawHub) {
    ROS_INFO("Data Source is hub.");

    std::string user_config_path;
    livox_node.getParam("user_config_path", user_config_path);
    ROS_INFO("Config file : %s", user_config_path.c_str());

    std::string cmdline_bd_code;
    livox_node.getParam("cmdline_str", cmdline_bd_code);

    std::vector<std::string> bd_code_list;
    ParseCommandlineInputBdCode(cmdline_bd_code.c_str(), bd_code_list);

    LdsHub *read_hub = LdsHub::GetInstance(1000 / publish_freq);
    lddc->RegisterLds(static_cast<Lds *>(read_hub));
    ret = read_hub->InitLdsHub(bd_code_list, user_config_path.c_str());
    if (!ret) {
      ROS_INFO("Init lds hub success!");
    } else {
      ROS_ERROR("Init lds hub fail!");
    }
  } else {
    ROS_INFO("Data Source is lvx file.");

    std::string cmdline_file_path;
    livox_node.getParam("cmdline_file_path", cmdline_file_path);

    do {
      if (!IsFilePathValid(cmdline_file_path.c_str())) {
        ROS_ERROR("File path invalid : %s !", cmdline_file_path.c_str());
        break;
      }

      std::string rosbag_file_path;
      int path_end_pos = cmdline_file_path.find_last_of('.');
      rosbag_file_path = cmdline_file_path.substr(0, path_end_pos);
      rosbag_file_path += ".bag";

      LdsLvx *read_lvx = LdsLvx::GetInstance(1000 / publish_freq);
      lddc->RegisterLds(static_cast<Lds *>(read_lvx));
      lddc->CreateBagFile(rosbag_file_path);
      int ret = read_lvx->InitLdsLvx(cmdline_file_path.c_str());
      if (!ret) {
        ROS_INFO("Init lds lvx file success!");
      } else {
        ROS_ERROR("Init lds lvx file fail!");
      }
    } while (0);
  }

  if ((data_src == kSourceRawLidar || data_src == kSourceRawHub) && ret != 0) {
    /** Do not enter the distribution loop with a failed SDK data source. */
    delete lddc;
    g_read_lidar = nullptr;
    return 1;
  }

  /** Advertise lidar mode service */
  ros::ServiceServer mode_srv =
      livox_node.advertiseService("livox_lidar_mode", LidarModeServiceCb);
  ROS_INFO("Advertised service: livox_lidar_mode");

  /** Advertise lidar reboot service */
  ros::ServiceServer reboot_srv =
      livox_node.advertiseService("livox_lidar_reboot", LidarRebootServiceCb);
  ROS_INFO("Advertised service: livox_lidar_reboot");

  /** Per-second stats dashboard topic (view with scripts/livox_stats_monitor.py
   *  in a separate terminal for an always-current, isolated panel) */
  ros::Timer stats_timer;
  ros::Subscriber group_power_cycle_intent_sub;
  if (data_src == kSourceRawLidar) {
    g_stats_pub = livox_node.advertise<std_msgs::String>("livox/lidar_stats", 1);
    g_power_cycle_request_pub = livox_node.advertise<std_msgs::String>(
        "livox/power_cycle_request", 16, true);
    g_recovery_state_pub = livox_node.advertise<std_msgs::String>(
        "livox/lidar_recovery_state", 32);
    g_group_power_cycle_ack_pub = livox_node.advertise<std_msgs::String>(
        "livox/group_power_cycle_ack", 8);
    group_power_cycle_intent_sub = livox_node.subscribe(
        "livox/group_power_cycle_intent", 8, GroupPowerCycleIntentCb);
    stats_timer = livox_node.createTimer(ros::Duration(1.0), StatsTimerCb);
    ROS_INFO("Publishing stats topic: livox/lidar_stats (1Hz)");
    ROS_INFO("Publishing recovery topics: livox/power_cycle_request (latched) "
             "and livox/lidar_recovery_state (1Hz)");
    ROS_INFO("Shared-power intent barrier: livox/group_power_cycle_intent -> "
             "livox/group_power_cycle_ack");
    ROS_INFO("Auto-recover (no-data/Config/Error watchdogs): %s",
             g_auto_recover ? "ENABLED" : "disabled");
    ROS_INFO("Handshake session recovery (broadcast-only watchdog): %s",
             g_auto_recover ? "ENABLED" : "disabled");
  }

  /** Use async spinner so service callbacks are processed in a separate thread,
   *  while the main thread keeps distributing lidar data */
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Time::init();
  while (ros::ok()) {
    lddc->DistributeLidarData();
  }

  stats_timer.stop();
  spinner.stop();
  /** Lddc owns shutdown sequencing for the registered source. This explicitly
   *  reaches TimeSync stop/join -> SDK Uninit instead of relying on static
   *  destruction while SDK I/O threads are still running. */
  lddc->PrepareExit();
  g_read_lidar = nullptr;
  delete lddc;
  return 0;
}
