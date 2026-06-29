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
#include <csignal>
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
    livox_status last_status = kStatusSuccess;
    for (uint8_t h = 0; h < kMaxLidarCount; h++) {
      livox_status s = g_read_lidar->RequestLidarModeChange(
          h, static_cast<LidarMode>(req.mode));
      if (s != kStatusSuccess && s != kStatusNotConnected) {
        ROS_WARN("LiDAR mode change failed for handle=%d: %d", h, s);
        last_status = s;
      }
    }
    res.ret_code = last_status;
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
    livox_status last_status = kStatusSuccess;
    for (uint8_t h = 0; h < kMaxLidarCount; h++) {
      livox_status s = g_read_lidar->RequestLidarReboot(h);
      if (s != kStatusSuccess && s != kStatusNotConnected) {
        ROS_WARN("LiDAR reboot failed for handle=%d: %d", h, s);
        last_status = s;
      }
    }
    res.ret_code = last_status;
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

/** Format a wall-clock time_t as HH:MM:SS, or "--" when 0 (never). */
static std::string FmtWall(int64_t t) {
  if (t == 0) {
    return "--";
  }
  time_t tt = (time_t)t;
  struct tm tmv;
  localtime_r(&tt, &tmv);
  char buf[16];
  strftime(buf, sizeof(buf), "%H:%M:%S", &tmv);
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
static const uint32_t kErrorRebootDelaySec    = 5;
static const uint32_t kErrorRebootCooldownSec = 40;
static const uint32_t kErrorRebootMaxAttempts = 3;

/** Timer callback (runs on the AsyncSpinner thread, independent of the data
 *  loop so it keeps updating even if a lidar stops sending). Publishes a
 *  preformatted dashboard of all connected lidars. */
void StatsTimerCb(const ros::TimerEvent &) {
  if (g_read_lidar == nullptr) {
    return;
  }
  static uint32_t prev_recv[kMaxLidarCount] = {0};
  static uint32_t prev_loss[kMaxLidarCount] = {0};
  static uint32_t prev_drop[kMaxLidarCount] = {0};
  static bool ever_seen[kMaxLidarCount] = {false};
  static char last_bcode[kMaxLidarCount][kBdCodeSize + 1] = {{0}};
  static uint32_t zero_secs[kMaxLidarCount] = {0};      /**< consecutive 1s ticks with no data while Normal */
  static uint8_t recover_stage[kMaxLidarCount] = {0};   /**< 0=ok 1=restarted sampling 2=rebooted */
  static uint32_t error_secs[kMaxLidarCount] = {0};     /**< consecutive 1s ticks in Error state */
  static uint8_t error_reboots[kMaxLidarCount] = {0};   /**< reboots attempted this Error episode */

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
  ss << "handle  broadcast_code   state         temp  fan   motor recv/s  loss/s  "
        "loss%    drop/s   disc  down_for   uptime\n";
  bool any = false;
  for (uint8_t h = 0; h < kMaxLidarCount; h++) {
    LidarDevice *l = &g_read_lidar->lidars_[h];
    bool connected = (l->connect_state != kConnectStateOff);
    if (connected) {
      ever_seen[h] = true;
      strncpy(last_bcode[h], l->info.broadcast_code, kBdCodeSize);
      last_bcode[h][kBdCodeSize] = '\0';
    }
    /** Skip handles that have never connected; but keep showing a lidar once
     *  seen, so a disconnect is loudly visible (DISCONNECTED) instead of the
     *  row silently vanishing. */
    if (!ever_seen[h]) {
      continue;
    }
    any = true;
    LidarPacketStatistic &st = l->statistic_info;
    LdsLidar::LinkStat &ls = g_read_lidar->link_stat_[h];
    uint32_t disc = ls.disconnect_count;
    /** How long the most recent outage lasted: if currently disconnected it is
     *  the still-growing down time; if reconnected it is the duration of the
     *  last completed outage (reconnect - disconnect). More useful than "time
     *  since last drop", which once reconnected just mirrors uptime. */
    std::string down_for;
    if (ls.last_disconnect_ns == 0) {
      down_for = "--";  /** never dropped */
    } else if (connected && ls.connect_since_ns) {
      down_for = FmtDur(ls.connect_since_ns - ls.last_disconnect_ns);
    } else {
      down_for = FmtDur(now_ns - ls.last_disconnect_ns);  /** still down */
    }
    std::string uptime = (connected && ls.connect_since_ns)
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
      uint32_t d_recv = st.receive_packet_count - prev_recv[h];
      uint32_t d_loss = st.loss_packet_count - prev_loss[h];
      uint32_t d_drop = st.queue_drop_count - prev_drop[h];
      prev_recv[h] = st.receive_packet_count;
      prev_loss[h] = st.loss_packet_count;
      prev_drop[h] = st.queue_drop_count;

      /** A lidar in Normal state should be streaming; PowerSaving/Standby
       *  legitimately produce no data, so only watch when state == Normal. */
      bool should_stream = (l->info.state == kLidarStateNormal);
      if (should_stream && d_recv == 0) {
        zero_secs[h]++;
      } else {
        zero_secs[h] = 0;
        recover_stage[h] = 0;
      }

      /** (B) Two-stage auto-recovery for a "Normal but no data" stall. */
      if (g_auto_recover && should_stream) {
        if (recover_stage[h] == 0 && zero_secs[h] >= 5) {
          g_read_lidar->RequestRestartSampling(h);
          ROS_WARN("[LivoxRecover] Lidar[%d] no data for 5s -> restart sampling",
                   h);
          recover_stage[h] = 1;
        } else if (recover_stage[h] == 1 && zero_secs[h] >= 15) {
          g_read_lidar->RequestLidarReboot(h);
          ls.recover_reboot_count++;
          ls.recover_last_wall_s = (int64_t)time(nullptr);
          ROS_WARN("[LivoxRecover] Lidar[%d] still no data for 15s -> reboot", h);
          hlog.LogEvent(h, last_bcode[h], "REBOOT", "no-data 15s");
          recover_stage[h] = 2;
        } else if (recover_stage[h] == 2 && zero_secs[h] >= 45) {
          recover_stage[h] = 0;  /** reboot didn't help; allow another cycle */
        }
      }

      /** (C) Error-state auto-recovery. A lidar reporting Error (e.g. a
       *  recoverable motor fault) never counts as "streaming", so path (B)
       *  above ignores it. Reboot just this lidar on a bounded schedule. */
      bool in_error = (l->info.state == kLidarStateError);
      if (in_error) {
        error_secs[h]++;
      } else {
        error_secs[h] = 0;      /** left Error (recovered or other state) */
        error_reboots[h] = 0;
      }
      if (g_auto_recover && in_error) {
        if (error_reboots[h] < kErrorRebootMaxAttempts) {
          /** reboot #n is due at delay + n*cooldown seconds in Error
           *  (5s, 45s, 85s for delay=5, cooldown=40). error_secs is frozen
           *  while the lidar is disconnected mid-reboot, so the real gap is
           *  the cooldown plus reconnect time. */
          uint32_t due = kErrorRebootDelaySec +
                         error_reboots[h] * kErrorRebootCooldownSec;
          if (error_secs[h] >= due) {
            g_read_lidar->RequestLidarReboot(h);
            ls.recover_reboot_count++;
            ls.recover_last_wall_s = (int64_t)time(nullptr);
            error_reboots[h]++;
            ROS_WARN("[LivoxRecover] Lidar[%d] in Error %us -> reboot "
                     "(attempt %u/%u)", h, error_secs[h], error_reboots[h],
                     kErrorRebootMaxAttempts);
            char rb[40];
            snprintf(rb, sizeof(rb), "Error %us attempt %u/%u", error_secs[h],
                     error_reboots[h], kErrorRebootMaxAttempts);
            hlog.LogEvent(h, last_bcode[h], "REBOOT", rb);
          }
        } else if ((error_secs[h] % 30) == 0) {
          /** Exhausted attempts: stop rebooting, warn loudly every 30s. */
          ROS_ERROR("[LivoxRecover] Lidar[%d] still in Error after %u reboots; "
                    "manual intervention needed (likely fan/motor hardware "
                    "fault)", h, kErrorRebootMaxAttempts);
        }
      }

      /** (A) Flag a connected-but-silent lidar loudly instead of "Normal". */
      const char *st_str = LidarStateStr(l->info.state);
      if (should_stream && zero_secs[h] >= 3) {
        st_str = "NO DATA";
      }
      snprintf(line, sizeof(line),
               "%-6d  %-15s  %-12s  %-4s  %-4s  %-4s  %6u  %6u  %7s  %6u   %4u  %9s  %7s\n",
               h, last_bcode[h], st_str, temp, fan, motor, d_recv, d_loss, losspct,
               d_drop, disc, down_for.c_str(), uptime.c_str());
      if (do_snapshot) {
        hlog.LogSnapshot(h, last_bcode[h], st_str, temp, fan, motor, dirty, sys,
                         st.receive_packet_count, st.loss_packet_count,
                         st.queue_drop_count, loss_pct_d, disc);
      }
    } else {
      prev_recv[h] = prev_loss[h] = prev_drop[h] = 0;
      zero_secs[h] = 0;
      recover_stage[h] = 0;
      snprintf(line, sizeof(line),
               "%-6d  %-15s  %-12s  %-4s  %-4s  %-4s  %6s  %6s  %7s  %6s   %4u  %9s  %7s\n",
               h, last_bcode[h], "DISCONNECTED", "-", "-", "-", "-", "-", losspct,
               "-", disc, down_for.c_str(), uptime.c_str());
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
    LdsLidar::LinkStat &ls = g_read_lidar->link_stat_[h];
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
    LdsLidar::LinkStat &ls = g_read_lidar->link_stat_[h];
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
    LdsLidar::LinkStat &ls = g_read_lidar->link_stat_[h];
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

  std_msgs::String msg;
  msg.data = ss.str();
  g_stats_pub.publish(msg);
}

inline void SignalHandler(int signum) {
  printf("livox ros driver will exit\r\n");
  ros::shutdown();
  exit(signum);
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
  signal(SIGINT, SignalHandler);
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
    ROS_INFO("Auto-recover (Normal-but-no-data watchdog): %s",
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

  spinner.stop();
  return 0;
}
