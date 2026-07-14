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
#include <vector>
#include <sstream>
#include <cstring>
#include <ctime>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include "health_logger.h"
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
    for (uint8_t h = 0; h < kMaxLidarCount; h++) {
      if (!IsCurrentConnectedHandle(h)) {
        continue;
      }
      requested = true;
      livox_status s = g_read_lidar->RequestLidarModeChange(
          h, static_cast<LidarMode>(req.mode));
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
/** When true, the stats timer auto-recovers a lidar that is connected/Normal
 *  but has produced no point cloud for a while (restart sampling, then reboot). */
static bool g_auto_recover = false;

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
  static uint64_t prev_recv[kMaxLidarCount] = {0};
  static uint64_t prev_pub[kMaxLidarCount] = {0};
  static uint64_t prev_drop[kMaxLidarCount] = {0};
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

  int64_t now_ns = std::chrono::steady_clock::now().time_since_epoch().count();

  /** Snapshot pacing for the persistent health log: one row per lidar every
   *  snapshot_period_s (this timer ticks at 1 Hz). Events are logged elsewhere,
   *  edge-triggered. Snapshot carries cumulative counters so two rows difference
   *  into that interval's loss/recv totals with no gap. */
  HealthLogger &hlog = HealthLogger::Get();
  static int snap_counter = 0;
  bool do_snapshot = false;
  if (hlog.enabled() && ++snap_counter >= hlog.snapshot_period_s()) {
    snap_counter = 0;
    do_snapshot = true;
  }

  std::ostringstream ss;
  ss << "===== Livox LiDAR Stats (1Hz) =====\n";
  ss << "handle  broadcast_code   state         temp  fan   motor recv/s  "
        "loss%    drop/s   disc  HB_lost   heartbeat\n";
  LdsLidar::LinkStat link_snapshot[kMaxLidarCount];
  bool any = false;
  for (uint8_t h = 0; h < kMaxLidarCount; h++) {
    /** Counters are written by the ingest/publish threads under data_lock_.
     *  Take one coherent snapshot so the watchdog never drives hardware from
     *  a torn/racing 64-bit read. */
    LidarConnectState connect_state;
    DeviceInfo info;
    LidarPacketStatistic st;
    {
      std::lock_guard<std::mutex> lock(g_read_lidar->data_lock_[h]);
      const LidarDevice &live = g_read_lidar->lidars_[h];
      connect_state = live.connect_state;
      info = live.info;
      st = live.statistic_info;
    }
    {
      std::lock_guard<std::mutex> lock(g_read_lidar->link_stat_lock_[h]);
      link_snapshot[h] = g_read_lidar->link_stat_[h];
    }
    bool connected = (connect_state != kConnectStateOff);
    if (connected) {
      ever_seen[h] = true;
      strncpy(last_bcode[h], info.broadcast_code, kBdCodeSize);
      last_bcode[h][kBdCodeSize] = '\0';
    }
    /** Skip handles that have never connected; but keep showing a lidar once
     *  seen, so a disconnect is loudly visible (DISCONNECTED) instead of the
     *  row silently vanishing. */
    if (!ever_seen[h]) {
      continue;
    }
    any = true;
    LdsLidar::LinkStat &ls = link_snapshot[h];
    uint32_t disc = ls.disconnect_count;
    /** "hb_lost" = heartbeat loss duration: how long the most recent heartbeat
     *  outage lasted. Still down -> the still-growing down time; reconnected ->
     *  duration of the last completed outage; never dropped -> "--". */
    std::string hb_lost;
    if (ls.last_disconnect_ns == 0) {
      hb_lost = "--";
    } else if (connected && ls.connect_since_ns) {
      hb_lost = FmtDur(ls.connect_since_ns - ls.last_disconnect_ns);
    } else {
      hb_lost = FmtDur(now_ns - ls.last_disconnect_ns);
    }
    /** "heartbeat" = heartbeat maintained: how long the heartbeat link has been
     *  up since the last (re)connect. NOT streaming/Normal time — a sleeping
     *  lidar keeps its heartbeat, so this keeps counting through PowerSaving. */
    std::string heartbeat = (connected && ls.connect_since_ns)
                                ? FmtDur(now_ns - ls.connect_since_ns)
                                : "--";
    ErrorMessage em;
    em.error_code = ls.health_code;
    const char *temp = TempStr(em.lidar_error_code.temp_status);
    const char *fan = FanStr(em.lidar_error_code.fan_status);
    const char *motor = MotorStr(em.lidar_error_code.motor_status);
    /** cumulative loss% since connect: total_loss / (total_recv + total_loss) */
    uint64_t tot = (uint64_t)st.receive_packet_count + st.loss_packet_count;
    char losspct[12];
    snprintf(losspct, sizeof(losspct), "%.2f%%",
             tot ? (100.0 * st.loss_packet_count / tot) : 0.0);
    /** Extra fields the snapshot log wants (MotorStr also maps the 3-level
     *  system_status: 0/1/2 -> OK/WARN/ERR!). */
    unsigned dirty = em.lidar_error_code.dirty_warn;
    const char *sys = MotorStr(em.lidar_error_code.system_status);
    double loss_pct_d = tot ? (100.0 * st.loss_packet_count / tot) : 0.0;
    char line[256];
    if (connected) {
      uint32_t d_recv = (uint32_t)(st.receive_packet_count - prev_recv[h]);
      uint32_t d_pub = (uint32_t)(st.publish_packet_count - prev_pub[h]);
      uint32_t d_drop = (uint32_t)(st.queue_drop_count - prev_drop[h]);
      prev_recv[h] = st.receive_packet_count;
      prev_pub[h] = st.publish_packet_count;
      prev_drop[h] = st.queue_drop_count;

      /** Watch "should be DELIVERING points but is not" -- keyed on published
       *  packets, not received ones. A lidar stuck mid-configure keeps
       *  receiving into a full queue that nobody consumes (recv/s normal,
       *  drop 100%, zero ROS output; field-confirmed), which a recv-based
       *  check is blind to. PowerSaving/Standby/Init/Error legitimately
       *  produce nothing, and a lidar inside a planned mode switch is left to
       *  the mode verify/retry machinery instead of this watchdog. */
      bool transition = g_read_lidar->IsModeTransitionActive(h);
      /** Config is explicitly excluded: starting/rebooting while coordinate,
       *  return-mode, IMU or extrinsic commands are still pending can publish
       *  data with only part of the requested configuration applied. On is
       *  retained because StartSampleCb uses it after a start timeout, which is
       *  the field-confirmed state the restart path must recover. */
      bool should_stream = (info.state == kLidarStateNormal) && !transition &&
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
                       config_reboots[h], kConfigRebootMaxAttempts);
              char rb[48];
              snprintf(rb, sizeof(rb), "Config %us attempt %u/%u",
                       stuck_secs, config_reboots[h],
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
                       "(attempt %u/%u)", h, error_secs[h], error_reboots[h],
                       kErrorRebootMaxAttempts);
              char rb[40];
              snprintf(rb, sizeof(rb), "Error %us attempt %u/%u", error_secs[h],
                       error_reboots[h], kErrorRebootMaxAttempts);
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

      /** (A) Flag a connected-but-silent lidar loudly instead of "Normal". */
      const char *st_str = (connect_state == kConnectStateConfig)
                               ? "Config"
                               : LidarStateStr(info.state);
      if (should_stream && nodata_secs[h] >= kNoDataLogSec) {
        st_str = "NO DATA";
      }
      snprintf(line, sizeof(line),
               "%-6d  %-15s  %-12s  %-4s  %-4s  %-4s  %6u  %7s  %6u   %4u  %8s  %9s\n",
               h, last_bcode[h], st_str, temp, fan, motor, d_recv, losspct,
               d_drop, disc, hb_lost.c_str(), heartbeat.c_str());
      if (do_snapshot) {
        hlog.LogSnapshot(h, last_bcode[h], st_str, temp, fan, motor, dirty, sys,
                         st.receive_packet_count, st.loss_packet_count,
                         st.queue_drop_count, loss_pct_d, disc);
      }
    } else {
      prev_recv[h] = prev_pub[h] = prev_drop[h] = 0;
      zero_secs[h] = 0;
      nodata_secs[h] = 0;
      recover_stage[h] = 0;
      config_secs[h] = 0;
      config_healthy_secs[h] = 0;
      /** Close any open silent episode; the DISCONNECT event already marks the
       *  transition, so no separate DATA row is needed here. */
      nodata_logged[h] = false;
      snprintf(line, sizeof(line),
               "%-6d  %-15s  %-12s  %-4s  %-4s  %-4s  %6s  %7s  %6s   %4u  %8s  %9s\n",
               h, last_bcode[h], "DISCONNECTED", "-", "-", "-", "-", losspct,
               "-", disc, hb_lost.c_str(), heartbeat.c_str());
      if (do_snapshot) {
        hlog.LogSnapshot(h, last_bcode[h], "DISCONNECTED", "-", "-", "-", 0, "-",
                         st.receive_packet_count, st.loss_packet_count,
                         st.queue_drop_count, loss_pct_d, disc);
      }
    }
    ss << line;
  }
  if (!any) {
    ss << "(no lidar seen yet)\n";
  }

  /** Temp-change footer: only list lidars that actually changed temp state,
   *  otherwise a single plain "all normal" note. */
  std::string temp_note;
  for (uint8_t h = 0; h < kMaxLidarCount; h++) {
    if (!ever_seen[h]) {
      continue;
    }
    LdsLidar::LinkStat &ls = link_snapshot[h];
    if (ls.temp_change_count > 0) {
      char buf[80];
      snprintf(buf, sizeof(buf), "  lidar %d: %u time(s), last at %s", h,
               ls.temp_change_count, FmtWall(ls.temp_change_wall_s).c_str());
      temp_note += buf;
    }
  }
  if (temp_note.empty()) {
    ss << "Temp changes: none (all lidars normal since start)\n";
  } else {
    ss << "Temp changes:" << temp_note << "\n";
  }

  /** Fault-event footer: motor/fan/volt/fw/system faults since start, kept
   *  even after the lidar recovers (live columns only show current state). */
  std::string fault_note;
  for (uint8_t h = 0; h < kMaxLidarCount; h++) {
    if (!ever_seen[h]) {
      continue;
    }
    LdsLidar::LinkStat &ls = link_snapshot[h];
    if (ls.fault_count > 0) {
      char buf[96];
      snprintf(buf, sizeof(buf), "  lidar %d: %u time(s) (%s), last at %s", h,
               ls.fault_count, FaultTags(ls.fault_code).c_str(),
               FmtWall(ls.fault_wall_s).c_str());
      fault_note += buf;
    }
  }
  if (fault_note.empty()) {
    ss << "Fault events: none (no motor/fan/dirty/volt/fw/system fault since "
          "start)\n";
  } else {
    ss << "Fault events:" << fault_note << "\n";
  }

  /** Auto-recover footer: watchdog reboots issued (only shown when non-zero,
   *  so it stays hidden unless auto_recover actually acted). */
  std::string rec_note;
  for (uint8_t h = 0; h < kMaxLidarCount; h++) {
    if (!ever_seen[h]) {
      continue;
    }
    LdsLidar::LinkStat &ls = link_snapshot[h];
    if (ls.recover_reboot_count > 0) {
      char buf[80];
      snprintf(buf, sizeof(buf), "  lidar %d: %u reboot(s), last at %s", h,
               ls.recover_reboot_count, FmtWall(ls.recover_last_wall_s).c_str());
      rec_note += buf;
    }
  }
  if (!rec_note.empty()) {
    ss << "Auto-recover:" << rec_note << "\n";
  }

  /** Mode-switch footer: requests that failed after all bounded retries
   *  (only shown when non-zero). Surfaces the "manual check needed"
   *  alert on the dashboard so it is not missed in the scrolling log. */
  std::string mode_note;
  for (uint8_t h = 0; h < kMaxLidarCount; h++) {
    if (!ever_seen[h]) {
      continue;
    }
    LdsLidar::LinkStat &ls = link_snapshot[h];
    if (ls.mode_fail_count > 0) {
      const char *m = (ls.mode_fail_mode == 1)
                          ? "Normal"
                          : (ls.mode_fail_mode == 3) ? "Standby"
                                                    : "PowerSaving";
      char buf[96];
      snprintf(buf, sizeof(buf), "  lidar %d: %s FAILED %u time(s), last at %s",
               h, m, ls.mode_fail_count, FmtWall(ls.mode_fail_wall_s).c_str());
      mode_note += buf;
    }
  }
  if (!mode_note.empty()) {
    ss << "Mode switch:" << mode_note << "\n";
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
  if (data_src == kSourceRawLidar) {
    g_stats_pub = livox_node.advertise<std_msgs::String>("livox/lidar_stats", 1);
    stats_timer = livox_node.createTimer(ros::Duration(1.0), StatsTimerCb);
    ROS_INFO("Publishing stats topic: livox/lidar_stats (1Hz)");
    ROS_INFO("Auto-recover (no-data/Config/Error watchdogs): %s",
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
