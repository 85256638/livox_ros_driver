// livox_ros_driver.cpp

#include "include/livox_ros_driver.h"

#include <chrono>
#include <vector>
#include <csignal>
#include <mutex>
#include <unordered_map>
#include <ctime>
#include <sstream>
#include <algorithm>

#include <ros/ros.h>
#include "lddc.h"
#include "lds_lidar.h"
#include "lds_hub.h"
#include "lds_lvx.h"
#include "livox_sdk.h"
#include "livox_mode_switcher/SetLidarMode.h"

using namespace livox_ros;

// 全局变量：模式切换映射及白名单
static std::mutex                               g_mtx;
static std::unordered_map<std::string, uint8_t> g_bc2handle;
static std::vector<std::string>                 g_whitelist;

// 时间戳辅助
static std::string ts() {
  std::time_t t = std::time(nullptr);
  std::tm tm{};
  localtime_r(&t, &tm);
  char buf[32];
  std::strftime(buf, sizeof(buf), "%H:%M:%S", &tm);
  return std::string(buf);
}

// === 链式回调：在 lds_lidar.cpp 中注册并调用 ===
void OnBroadcast(const BroadcastDeviceInfo* info) {
  if (!info) return;
  ROS_INFO("[%s] [ModeSwitch] Broadcast bc=%s",
           ts().c_str(),
           info->broadcast_code);
}

void OnChange(const DeviceInfo* info, DeviceEvent evt) {
  if (!info) return;
  const std::string bc = info->broadcast_code;
  std::lock_guard<std::mutex> lk(g_mtx);
  if (evt == kEventDisconnect) {
    g_bc2handle.erase(bc);
    ROS_WARN("[%s] [ModeSwitch] DISCONNECT bc=%s",
             ts().c_str(), bc.c_str());
  } else {
    g_bc2handle[bc] = info->handle;
    ROS_INFO("[%s] [ModeSwitch] ONLINE bc=%s handle=%u",
             ts().c_str(), bc.c_str(), info->handle);
  }
}

// === ROS Service：下发模式切换命令 ===
bool SetModeSrv(livox_mode_switcher::SetLidarMode::Request &req,
                livox_mode_switcher::SetLidarMode::Response &res) {
  std::lock_guard<std::mutex> lk(g_mtx);
  auto it = g_bc2handle.find(req.broadcast_code);
  if (it == g_bc2handle.end()) {
    // fallback: 遍历 LdsLidar 实例
    LdsLidar* lds = LdsLidar::GetInstance(0);
    for (uint8_t h = 0; h < kMaxLidarCount; ++h) {
      if (lds->lidars_[h].info.broadcast_code == req.broadcast_code) {
        it = g_bc2handle.emplace(req.broadcast_code, h).first;
        ROS_WARN("[%s] [ModeSwitch] Fallback map bc=%s→handle=%u",
                 ts().c_str(), req.broadcast_code.c_str(), h);
        break;
      }
    }
    if (it == g_bc2handle.end()) {
      res.success = false;
      res.message = "device not connected";
      return true;
    }
  }
  auto r = LidarSetMode(it->second,
                        static_cast<LidarMode>(req.mode),
                        nullptr, nullptr);
  res.success = (r == kStatusSuccess);
  std::ostringstream oss; oss << "SDK ret=" << r;
  res.message = res.success ? "OK" : oss.str();
  ROS_INFO("[%s] [ModeSwitch] Service SetMode bc=%s mode=%u ret=%d",
           ts().c_str(), req.broadcast_code.c_str(), req.mode, r);
  return true;
}

const int32_t kSdkVersionMajorLimit = 2;
inline void SignalHandler(int signum) {
  printf("livox ros driver will exit\n");
  ros::shutdown();
  exit(signum);
}

int main(int argc, char **argv) {
  // ROS 初始化
  if (ros::console::set_logger_level(
        ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  }
  ros::init(argc, argv, "livox_lidar_publisher");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  signal(SIGINT, SignalHandler);

  ROS_INFO("Livox Ros Driver Version: %s",
           LIVOX_ROS_DRIVER_VERSION_STRING);

  // SDK 版本检查
  LivoxSdkVersion sdkv;
  GetLivoxSdkVersion(&sdkv);
  if (sdkv.major < kSdkVersionMajorLimit) {
    ROS_INFO("The SDK version[%d.%d.%d] is too low",
             sdkv.major, sdkv.minor, sdkv.patch);
    return 0;
  }

  // 读取参数
  int    xfer_format   = kPointCloud2Msg;
  int    multi_topic   = 0;
  int    data_src      = kSourceRawLidar;
  double publish_freq  = 10.0;
  int    output_type   = kOutputToRos;
  std::string frame_id = "livox_frame";
  bool   lidar_bag     = true;
  bool   imu_bag       = false;

  nh.getParam("xfer_format",      xfer_format);
  nh.getParam("multi_topic",      multi_topic);
  nh.getParam("data_src",         data_src);
  nh.getParam("publish_freq",     publish_freq);
  nh.getParam("output_data_type", output_type);
  nh.getParam("frame_id",         frame_id);
  nh.getParam("enable_lidar_bag", lidar_bag);
  nh.getParam("enable_imu_bag",   imu_bag);
  publish_freq = std::min(std::max(publish_freq, 0.1), 100.0);

  // 读取 broadcast_codes 白名单
  pnh.getParam("broadcast_codes", g_whitelist);
  if (g_whitelist.empty()) {
    ROS_ERROR("No broadcast_codes provided! Aborting.");
    return -1;
  }
  for (auto &bc : g_whitelist) {
    ROS_INFO("Whitelist bc=%s", bc.c_str());
  }

  // 构造 Lddc
  Lddc *lddc = new Lddc(xfer_format, multi_topic,
                        data_src, output_type,
                        publish_freq, frame_id,
                        lidar_bag, imu_bag);
  lddc->SetRosNode(&nh);

  // 初始化数据源
  int ret = 0;
  if (data_src == kSourceRawLidar) {
    ROS_INFO("Data Source is raw lidar.");
    std::string cfg;
    nh.getParam("user_config_path", cfg);
    ROS_INFO("Config file: %s", cfg.c_str());
    LdsLidar *reader = LdsLidar::GetInstance(1000 / publish_freq);
    lddc->RegisterLds(static_cast<Lds *>(reader));
    ret = reader->InitLdsLidar(g_whitelist, cfg.c_str());
    if (ret) {
      ROS_ERROR("InitLdsLidar failed: %d", ret);
      return -1;
    }
    ROS_INFO("Init lds lidar success!");
  } else if (data_src == kSourceRawHub) {
    ROS_INFO("Data Source is hub.");
    std::string cfg;
    nh.getParam("user_config_path", cfg);
    ROS_INFO("Config file: %s", cfg.c_str());
    LdsHub *reader = LdsHub::GetInstance(1000 / publish_freq);
    lddc->RegisterLds(static_cast<Lds *>(reader));
    ret = reader->InitLdsHub(g_whitelist, cfg.c_str());
    if (ret) {
      ROS_ERROR("InitLdsHub failed: %d", ret);
      return -1;
    }
    ROS_INFO("Init lds hub success!");
  } else {
    ROS_INFO("Data Source is lvx file.");
    std::string path;
    nh.getParam("cmdline_file_path", path);
    if (!IsFilePathValid(path.c_str())) {
      ROS_ERROR("LVX file path invalid: %s", path.c_str());
      return -1;
    }
    std::string bag = path.substr(0, path.find_last_of('.')) + ".bag";
    LdsLvx *reader = LdsLvx::GetInstance(1000 / publish_freq);
    lddc->RegisterLds(static_cast<Lds *>(reader));
    lddc->CreateBagFile(bag);
    ret = reader->InitLdsLvx(path.c_str());
    if (ret) {
      ROS_ERROR("InitLdsLvx failed: %d", ret);
      return -1;
    }
    ROS_INFO("Init lds lvx file success!");
  }

  // 注册 set_mode 服务
  ros::ServiceServer srv = pnh.advertiseService("set_mode", SetModeSrv);
  ROS_INFO("set_mode service advertised");

  // 主循环
  ros::Time::init();
  while (ros::ok()) {
    lddc->DistributeLidarData();
    ros::spinOnce();
  }

  return 0;
}

