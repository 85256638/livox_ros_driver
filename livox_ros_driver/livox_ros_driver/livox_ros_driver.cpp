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

#include <ros/ros.h>
#include <std_msgs/String.h>
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

  int64_t now_ns = std::chrono::steady_clock::now().time_since_epoch().count();

  std::ostringstream ss;
  ss << "===== Livox LiDAR Stats (1Hz) =====\n";
  ss << "handle  broadcast_code   state         recv/s  loss/s  drop/s   "
        "disc  last_drop   uptime\n";
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
    std::string last_drop =
        ls.last_disconnect_ns ? FmtDur(now_ns - ls.last_disconnect_ns) : "--";
    std::string uptime = (connected && ls.connect_since_ns)
                             ? FmtDur(now_ns - ls.connect_since_ns)
                             : "--";
    char line[256];
    if (connected) {
      uint32_t d_recv = st.receive_packet_count - prev_recv[h];
      uint32_t d_loss = st.loss_packet_count - prev_loss[h];
      uint32_t d_drop = st.queue_drop_count - prev_drop[h];
      prev_recv[h] = st.receive_packet_count;
      prev_loss[h] = st.loss_packet_count;
      prev_drop[h] = st.queue_drop_count;
      snprintf(line, sizeof(line),
               "%-6d  %-15s  %-12s  %6u  %6u  %6u   %4u  %9s  %7s\n",
               h, last_bcode[h], LidarStateStr(l->info.state), d_recv, d_loss,
               d_drop, disc, last_drop.c_str(), uptime.c_str());
    } else {
      prev_recv[h] = prev_loss[h] = prev_drop[h] = 0;
      snprintf(line, sizeof(line),
               "%-6d  %-15s  %-12s  %6s  %6s  %6s   %4u  %9s  %7s\n",
               h, last_bcode[h], "DISCONNECTED", "-", "-", "-", disc,
               last_drop.c_str(), uptime.c_str());
    }
    ss << line;
  }
  if (!any) {
    ss << "(no lidar seen yet)\n";
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
