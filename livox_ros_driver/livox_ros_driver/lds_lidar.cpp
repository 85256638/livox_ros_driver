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

// Forward declarations for mode-switcher callbacks
extern void OnBroadcast(const BroadcastDeviceInfo* info);
extern void OnChange(const DeviceInfo* info, DeviceEvent evt);


#include <stdio.h>
#include <string.h>
#include <memory>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <string>
#include <ros/ros.h>

#include "rapidjson/document.h"
#include "rapidjson/filereadstream.h"
#include "rapidjson/stringbuffer.h"

using namespace std;

namespace livox_ros {

static std::unordered_map<std::string, int> g_state_change_counter;
static std::unordered_map<std::string, int> g_manual_state_change_counter;
static std::unordered_map<std::string, int> g_auto_state_change_counter;
static std::unordered_map<std::string, std::string> g_bc_to_name = {
    {"3WEDJA700100021", "farleft_2"},
    {"3WEDJA600100571", "left_57"},
    {"1HDDH3200104431", "mid_65"},
    {"3WEDJA600100751", "right_75"}
};
std::unordered_map<std::string, double> g_manual_switch_time;
static std::string GetLidarName(const std::string& bc) {
    auto it = g_bc_to_name.find(bc);
    if (it != g_bc_to_name.end()) return it->second;
    return "unknown";
}

/** Const varible ------------------------------------------------------------*/
/** For callback use only */
LdsLidar *g_lds_ldiar = nullptr;

/** Global function for common use -------------------------------------------*/

/** Lds lidar function -------------------------------------------------------*/
LdsLidar::LdsLidar(uint32_t interval_ms) : Lds(interval_ms, kSourceRawLidar) {
  auto_connect_mode_ = true;
  is_initialized_ = false;

  whitelist_count_ = 0;
  memset(broadcast_code_whitelist_, 0, sizeof(broadcast_code_whitelist_));

  ResetLdsLidar();
}

LdsLidar::~LdsLidar() {}

void LdsLidar::ResetLdsLidar(void) { ResetLds(kSourceRawLidar); }

int LdsLidar::InitLdsLidar(std::vector<std::string> &broadcast_code_strs,
                           const char *user_config_path) {
  if (is_initialized_) {
    ROS_INFO("LiDAR data source is already inited!");
    return -1;
  }

  if (!Init()) {
    Uninit();
    ROS_ERROR("Livox-SDK init fail!");
    return -1;
  }

  LivoxSdkVersion _sdkversion;
  GetLivoxSdkVersion(&_sdkversion);
  ROS_INFO("Livox SDK version %d.%d.%d", _sdkversion.major, _sdkversion.minor,
         _sdkversion.patch);

  SetBroadcastCallback(OnDeviceBroadcast);
  SetDeviceStateUpdateCallback(OnDeviceChange);

  /** Add commandline input broadcast code */
  for (auto input_str : broadcast_code_strs) {
    AddBroadcastCodeToWhitelist(input_str.c_str());
  }

  ParseConfigFile(user_config_path);

  if (whitelist_count_) {
    DisableAutoConnectMode();
    ROS_INFO("Disable auto connect mode!");

    ROS_INFO("List all broadcast code in whiltelist:");
    for (uint32_t i = 0; i < whitelist_count_; i++) {
      ROS_INFO("%s", broadcast_code_whitelist_[i]);
    }
  } else {
    EnableAutoConnectMode();
    ROS_INFO(
        "No broadcast code was added to whitelist, swith to automatic "
        "connection mode!");
  }

  if (enable_timesync_) {
    timesync_ = TimeSync::GetInstance();
    if (timesync_->InitTimeSync(timesync_config_)) {
      ROS_ERROR("Timesync init fail");
      return -1;
    }

    if (timesync_->SetReceiveSyncTimeCb(ReceiveSyncTimeCallback, this)) {
      ROS_ERROR("Set Timesync callback fail");
      return -1;
    }

    timesync_->StartTimesync();
  }

  /** Start livox sdk to receive lidar data */
  if (!Start()) {
    Uninit();
    ROS_ERROR("Livox-SDK init fail!");
    return -1;
  }

  /** Add here, only for callback use */
  if (g_lds_ldiar == nullptr) {
    g_lds_ldiar = this;
  }
  is_initialized_ = true;
  ROS_INFO("Livox-SDK init success!");

  return 0;
}

int LdsLidar::DeInitLdsLidar(void) {
  if (!is_initialized_) {
    ROS_INFO("LiDAR data source is not exit");
    return -1;
  }

  Uninit();
  ROS_INFO("Livox SDK Deinit completely!");

  if (timesync_) {
    timesync_->DeInitTimeSync();
  }

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

void LdsLidar::OnDeviceBroadcast(const BroadcastDeviceInfo *info) {
  if (info == nullptr) {
    return;
  }

  if (info->dev_type == kDeviceTypeHub) {
    ROS_INFO("In lidar mode, couldn't connect a hub : %s",
           info->broadcast_code);
    return;
  }

  if (g_lds_ldiar->IsAutoConnectMode()) {
    ROS_INFO("In automatic connection mode, will connect %s",
           info->broadcast_code);
  } else {
    if (!g_lds_ldiar->IsBroadcastCodeExistInWhitelist(info->broadcast_code)) {
      ROS_INFO("Not in the whitelist, please add %s to if want to connect!",
             info->broadcast_code);
      return;
    }
  }

  bool result = false;
  uint8_t handle = 0;
  result = AddLidarToConnect(info->broadcast_code, &handle);
  if (result == kStatusSuccess && handle < kMaxLidarCount) {
    SetDataCallback(handle, OnLidarDataCb, (void *)g_lds_ldiar);

    LidarDevice *p_lidar = &(g_lds_ldiar->lidars_[handle]);
    p_lidar->handle = handle;
    p_lidar->connect_state = kConnectStateOff;

    UserRawConfig config;
    if (g_lds_ldiar->GetRawConfig(info->broadcast_code, config)) {
      ROS_INFO("Could not find raw config, set config to default!");
      config.enable_fan = 1;
      config.return_mode = kFirstReturn;
      config.coordinate = kCoordinateCartesian;
      config.imu_rate = kImuFreq200Hz;
      config.extrinsic_parameter_source = kNoneExtrinsicParameter;
      config.enable_high_sensitivity = false;
    }

    p_lidar->config.enable_fan = config.enable_fan;
    p_lidar->config.return_mode = config.return_mode;
    p_lidar->config.coordinate = config.coordinate;
    p_lidar->config.imu_rate = config.imu_rate;
    p_lidar->config.extrinsic_parameter_source =
        config.extrinsic_parameter_source;
    p_lidar->config.enable_high_sensitivity = config.enable_high_sensitivity;
  } else {
    ROS_ERROR("Add lidar to connect is failed : %d %d ", result, handle);
  }
}

/** Callback function of changing of device state. */
void LdsLidar::OnDeviceChange(const DeviceInfo *info, DeviceEvent type) {
  // ------- Mode switch: restart sampling on returning to NORMAL -------
  if (info && type == kEventStateChange && info->state == kLidarStateNormal) {
    LidarStartSampling(info->handle, StartSampleCb, g_lds_ldiar);
    ROS_INFO("ModeSwitch: restart sampling bc=%s|%s handle=%u", info->broadcast_code, GetLidarName(info->broadcast_code).c_str(), info->handle);
  }

  if (info == nullptr) {
    return;
  }

  uint8_t handle = info->handle;
  if (handle >= kMaxLidarCount) {
    return;
  }

  LidarDevice *p_lidar = &(g_lds_ldiar->lidars_[handle]);
  if (type == kEventConnect) {
    QueryDeviceInformation(handle, DeviceInformationCb, g_lds_ldiar);
    if (p_lidar->connect_state == kConnectStateOff) {
      p_lidar->connect_state = kConnectStateOn;
      p_lidar->info = *info;
    }
  } else if (type == kEventDisconnect) {
    ROS_INFO("Lidar[%s|%s] disconnect!", info->broadcast_code, GetLidarName(info->broadcast_code).c_str());
    ResetLidar(p_lidar, kSourceRawLidar);
  } else if (type == kEventStateChange) {
    p_lidar->info = *info;
    std::string bc = info->broadcast_code;
    double now = ros::Time::now().toSec();
    bool is_manual = false;
    if (g_manual_switch_time.count(bc) && now - g_manual_switch_time[bc] < 5.0) {
      is_manual = true;
      g_manual_switch_time.erase(bc);
    }
    if (info->state == 2) {
      g_state_change_counter[bc]++;
      if (is_manual) {
        g_manual_state_change_counter[bc]++;
        ROS_INFO("[Manual State Switch] Update State to 2-PowerSaving [%s|%s] (manual: %d, auto: %d)", bc.c_str(), GetLidarName(bc).c_str(), g_manual_state_change_counter[bc], g_auto_state_change_counter[bc]);
      } else {
        g_auto_state_change_counter[bc]++;
        ROS_INFO("[SDK Auto State Sync] Update State to 2-PowerSaving [%s|%s] (manual: %d, auto: %d)", bc.c_str(), GetLidarName(bc).c_str(), g_manual_state_change_counter[bc], g_auto_state_change_counter[bc]);
      }
    } else if (info->state == 1) {
      g_state_change_counter[bc]++;
      if (is_manual) {
        g_manual_state_change_counter[bc]++;
        ROS_INFO("[Manual State Switch] Update State to 1-Normal [%s|%s] (manual: %d, auto: %d)", bc.c_str(), GetLidarName(bc).c_str(), g_manual_state_change_counter[bc], g_auto_state_change_counter[bc]);
      } else {
        g_auto_state_change_counter[bc]++;
        ROS_INFO("[SDK Auto State Sync] Update State to 1-Normal [%s|%s] (manual: %d, auto: %d)", bc.c_str(), GetLidarName(bc).c_str(), g_manual_state_change_counter[bc], g_auto_state_change_counter[bc]);
      }
    }
  }

  if (p_lidar->connect_state == kConnectStateOn) {
    ROS_INFO("Lidar[%s|%s] status_code[%d] working state[%d] feature[%d]",
           p_lidar->info.broadcast_code,
           GetLidarName(p_lidar->info.broadcast_code).c_str(),
           p_lidar->info.status.status_code.error_code, p_lidar->info.state,
           p_lidar->info.feature);
    SetErrorMessageCallback(handle, LidarErrorStatusCb);

    /** Config lidar parameter */
    if (p_lidar->info.state == kLidarStateNormal) {
      /** Ensure the thread safety for set_bits and connect_state */
      lock_guard<mutex> lock(g_lds_ldiar->config_mutex_);

      if (p_lidar->config.coordinate != 0) {
        SetSphericalCoordinate(handle, SetCoordinateCb, g_lds_ldiar);
      } else {
        SetCartesianCoordinate(handle, SetCoordinateCb, g_lds_ldiar);
      }
      p_lidar->config.set_bits |= kConfigCoordinate;

      if (kDeviceTypeLidarMid40 != info->type) {
        LidarSetPointCloudReturnMode(
            handle, (PointCloudReturnMode)(p_lidar->config.return_mode),
            SetPointCloudReturnModeCb, g_lds_ldiar);
        p_lidar->config.set_bits |= kConfigReturnMode;
      }

      if ((kDeviceTypeLidarMid70 != info->type) &&
          (kDeviceTypeLidarMid40 != info->type)) {
        LidarSetImuPushFrequency(handle, (ImuFreq)(p_lidar->config.imu_rate),
                                 SetImuRatePushFrequencyCb, g_lds_ldiar);
        p_lidar->config.set_bits |= kConfigImuRate;
      }

      if (p_lidar->config.extrinsic_parameter_source ==
          kExtrinsicParameterFromLidar) {
        LidarGetExtrinsicParameter(handle, GetLidarExtrinsicParameterCb,
                                   g_lds_ldiar);
        p_lidar->config.set_bits |= kConfigGetExtrinsicParameter;
      }

      if (kDeviceTypeLidarTele == info->type) {
        if (p_lidar->config.enable_high_sensitivity) {
          LidarEnableHighSensitivity(handle, SetHighSensitivityCb, g_lds_ldiar);
          ROS_INFO("Enable high sensitivity");
        } else {
          LidarDisableHighSensitivity(handle, SetHighSensitivityCb,
                                      g_lds_ldiar);
          ROS_INFO("Disable high sensitivity");
        }
        p_lidar->config.set_bits |= kConfigSetHighSensitivity;
      }

      p_lidar->connect_state = kConnectStateConfig;
    }
  }
}

/** Query the firmware version of Livox LiDAR. */
void LdsLidar::DeviceInformationCb(livox_status status, uint8_t handle,
                                   DeviceInformationResponse *ack,
                                   void *clent_data) {
  if (status != kStatusSuccess) {
    ROS_ERROR("Device Query Informations Failed : %d", status);
  }
  if (ack) {
    ROS_INFO("firmware version: %d.%d.%d.%d", ack->firmware_version[0],
           ack->firmware_version[1], ack->firmware_version[2],
           ack->firmware_version[3]);
  }
}

/** Callback function of Lidar error message. */
void LdsLidar::LidarErrorStatusCb(livox_status status, uint8_t handle,
                                  ErrorMessage *message) {
  static uint32_t error_message_count = 0;
  if (message != NULL) {
    ++error_message_count;
    if (0 == (error_message_count % 100)) {
      ROS_INFO("handle: %u", handle);
      ROS_INFO("temp_status : %u", message->lidar_error_code.temp_status);
      ROS_INFO("volt_status : %u", message->lidar_error_code.volt_status);
      ROS_INFO("motor_status : %u", message->lidar_error_code.motor_status);
      ROS_INFO("dirty_warn : %u", message->lidar_error_code.dirty_warn);
      ROS_INFO("firmware_err : %u", message->lidar_error_code.firmware_err);
      ROS_INFO("pps_status : %u", message->lidar_error_code.device_status);
      ROS_INFO("fan_status : %u", message->lidar_error_code.fan_status);
      ROS_INFO("self_heating : %u", message->lidar_error_code.self_heating);
      ROS_INFO("ptp_status : %u", message->lidar_error_code.ptp_status);
      ROS_INFO("time_sync_status : %u",
             message->lidar_error_code.time_sync_status);
      ROS_INFO("system_status : %u", message->lidar_error_code.system_status);
    }
  }
}

void LdsLidar::ControlFanCb(livox_status status, uint8_t handle,
                            uint8_t response, void *clent_data) {}

void LdsLidar::SetPointCloudReturnModeCb(livox_status status, uint8_t handle,
                                         uint8_t response, void *clent_data) {
  LdsLidar *lds_lidar = static_cast<LdsLidar *>(clent_data);

  if (handle >= kMaxLidarCount) {
    return;
  }
  LidarDevice *p_lidar = &(lds_lidar->lidars_[handle]);

  if (status == kStatusSuccess) {
    ROS_INFO("Set return mode success!");

    lock_guard<mutex> lock(lds_lidar->config_mutex_);
    p_lidar->config.set_bits &= ~((uint32_t)(kConfigReturnMode));
    if (!p_lidar->config.set_bits) {
      LidarStartSampling(handle, StartSampleCb, lds_lidar);
      p_lidar->connect_state = kConnectStateSampling;
    }
  } else {
    LidarSetPointCloudReturnMode(
        handle, (PointCloudReturnMode)(p_lidar->config.return_mode),
        SetPointCloudReturnModeCb, lds_lidar);
    ROS_INFO("Set return mode fail, try again!");
  }
}

void LdsLidar::SetCoordinateCb(livox_status status, uint8_t handle,
                               uint8_t response, void *clent_data) {
  LdsLidar *lds_lidar = static_cast<LdsLidar *>(clent_data);

  if (handle >= kMaxLidarCount) {
    return;
  }
  LidarDevice *p_lidar = &(lds_lidar->lidars_[handle]);

  if (status == kStatusSuccess) {
    ROS_INFO("Set coordinate success!");

    lock_guard<mutex> lock(lds_lidar->config_mutex_);
    p_lidar->config.set_bits &= ~((uint32_t)(kConfigCoordinate));
    if (!p_lidar->config.set_bits) {
      LidarStartSampling(handle, StartSampleCb, lds_lidar);
      p_lidar->connect_state = kConnectStateSampling;
    }
  } else {
    if (p_lidar->config.coordinate != 0) {
      SetSphericalCoordinate(handle, SetCoordinateCb, lds_lidar);
    } else {
      SetCartesianCoordinate(handle, SetCoordinateCb, lds_lidar);
    }

    ROS_INFO("Set coordinate fail, try again!");
  }
}

void LdsLidar::SetImuRatePushFrequencyCb(livox_status status, uint8_t handle,
                                         uint8_t response, void *clent_data) {
  LdsLidar *lds_lidar = static_cast<LdsLidar *>(clent_data);

  if (handle >= kMaxLidarCount) {
    return;
  }
  LidarDevice *p_lidar = &(lds_lidar->lidars_[handle]);

  if (status == kStatusSuccess) {
    ROS_INFO("Set imu rate success!");

    lock_guard<mutex> lock(lds_lidar->config_mutex_);
    p_lidar->config.set_bits &= ~((uint32_t)(kConfigImuRate));
    if (!p_lidar->config.set_bits) {
      LidarStartSampling(handle, StartSampleCb, lds_lidar);
      p_lidar->connect_state = kConnectStateSampling;
    }
  } else {
    LidarSetImuPushFrequency(handle, (ImuFreq)(p_lidar->config.imu_rate),
                             SetImuRatePushFrequencyCb, g_lds_ldiar);
    ROS_INFO("Set imu rate fail, try again!");
  }
}

/** Callback function of get LiDARs' extrinsic parameter. */
void LdsLidar::GetLidarExtrinsicParameterCb(
    livox_status status, uint8_t handle,
    LidarGetExtrinsicParameterResponse *response, void *clent_data) {
  LdsLidar *lds_lidar = static_cast<LdsLidar *>(clent_data);
  if (handle >= kMaxLidarCount) {
    return;
  }

  if (status == kStatusSuccess) {
    if (response != nullptr) {
      ROS_INFO("Lidar[%d] get ExtrinsicParameter status[%d] response[%d]",
             handle, status, response->ret_code);
      LidarDevice *p_lidar = &(lds_lidar->lidars_[handle]);
      ExtrinsicParameter *p_extrinsic = &p_lidar->extrinsic_parameter;
      p_extrinsic->euler[0] = static_cast<float>(response->roll * PI / 180.0);
      p_extrinsic->euler[1] = static_cast<float>(response->pitch * PI / 180.0);
      p_extrinsic->euler[2] = static_cast<float>(response->yaw * PI / 180.0);
      p_extrinsic->trans[0] = static_cast<float>(response->x / 1000.0);
      p_extrinsic->trans[1] = static_cast<float>(response->y / 1000.0);
      p_extrinsic->trans[2] = static_cast<float>(response->z / 1000.0);
      EulerAnglesToRotationMatrix(p_extrinsic->euler, p_extrinsic->rotation);
      if (p_lidar->config.extrinsic_parameter_source) {
        p_extrinsic->enable = true;
      }
      ROS_INFO("Lidar[%d] get ExtrinsicParameter success!", handle);

      lock_guard<mutex> lock(lds_lidar->config_mutex_);
      p_lidar->config.set_bits &= ~((uint32_t)(kConfigGetExtrinsicParameter));
      if (!p_lidar->config.set_bits) {
        LidarStartSampling(handle, StartSampleCb, lds_lidar);
        p_lidar->connect_state = kConnectStateSampling;
      }
    } else {
      ROS_INFO("Lidar[%d] get ExtrinsicParameter fail!", handle);
    }
  } else if (status == kStatusTimeout) {
    ROS_INFO("Lidar[%d] get ExtrinsicParameter timeout!", handle);
  }
}

void LdsLidar::SetHighSensitivityCb(livox_status status, uint8_t handle,
                                    DeviceParameterResponse *response,
                                    void *clent_data) {
  LdsLidar *lds_lidar = static_cast<LdsLidar *>(clent_data);

  if (handle >= kMaxLidarCount) {
    return;
  }
  LidarDevice *p_lidar = &(lds_lidar->lidars_[handle]);

  if (status == kStatusSuccess) {
    p_lidar->config.set_bits &= ~((uint32_t)(kConfigSetHighSensitivity));
    ROS_INFO("Set high sensitivity success!");

    lock_guard<mutex> lock(lds_lidar->config_mutex_);
    if (!p_lidar->config.set_bits) {
      LidarStartSampling(handle, StartSampleCb, lds_lidar);
      p_lidar->connect_state = kConnectStateSampling;
    };
  } else {
    if (p_lidar->config.enable_high_sensitivity) {
      LidarEnableHighSensitivity(handle, SetHighSensitivityCb, g_lds_ldiar);
    } else {
      LidarDisableHighSensitivity(handle, SetHighSensitivityCb, g_lds_ldiar);
    }
    ROS_INFO("Set high sensitivity fail, try again!");
  }
}

/** Callback function of starting sampling. */
void LdsLidar::StartSampleCb(livox_status status, uint8_t handle,
                             uint8_t response, void *clent_data) {
  LdsLidar *lds_lidar = static_cast<LdsLidar *>(clent_data);

  if (handle >= kMaxLidarCount) {
    return;
  }

  LidarDevice *p_lidar = &(lds_lidar->lidars_[handle]);
  if (status == kStatusSuccess) {
    if (response != 0) {
      p_lidar->connect_state = kConnectStateOn;
      ROS_INFO("Lidar start sample fail : state[%d] handle[%d] res[%d]", status,
             handle, response);
    } else {
      ROS_INFO("Lidar start sample success");
    }
  } else if (status == kStatusTimeout) {
    p_lidar->connect_state = kConnectStateOn;
    ROS_INFO("Lidar start sample timeout : state[%d] handle[%d] res[%d]",
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
  ROS_INFO("Set lidar[%d] sync time status[%d] response[%d]", handle, status,
         response);
}

void LdsLidar::ReceiveSyncTimeCallback(const char *rmc, uint32_t rmc_length,
                                       void *client_data) {
  LdsLidar *lds_lidar = static_cast<LdsLidar *>(client_data);
  // std::unique_lock<std::mutex> lock(mtx);
  LidarDevice *p_lidar = nullptr;
  for (uint8_t handle = 0; handle < kMaxLidarCount; handle++) {
    p_lidar = &(lds_lidar->lidars_[handle]);
    if (p_lidar->connect_state == kConnectStateSampling &&
        p_lidar->info.state == kLidarStateNormal) {
      livox_status status = LidarSetRmcSyncTime(handle, rmc, rmc_length,
                                                SetRmcSyncTimeCb, lds_lidar);
      if (status != kStatusSuccess) {
        ROS_ERROR("Set GPRMC synchronization time error code: %d.", status);
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
    ROS_INFO("%s is alrealy exist!", broadcast_code);
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
      ROS_INFO("Enable timesync : ");
      if (timesync_config_.dev_config.type == kCommDevUart) {
        ROS_INFO("Uart[%s],baudrate index[%d],parity index[%d]",
               timesync_config_.dev_config.name,
               timesync_config_.dev_config.config.uart.baudrate,
               timesync_config_.dev_config.config.uart.parity);
      }
    } else {
      ROS_INFO("Disable timesync");
    }
    return 0;
  } while (0);

  return -1;
}

/** Config file process */
int LdsLidar::ParseConfigFile(const char *pathname) {
  FILE *raw_file = std::fopen(pathname, "rb");
  if (!raw_file) {
    ROS_ERROR("Open json config file fail!");
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
            ROS_ERROR("User config file parse error");
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

          ROS_INFO("broadcast code[%s] : %d %d %d %d %d %d",
                 config.broadcast_code, config.enable_connect,
                 config.enable_fan, config.return_mode, config.coordinate,
                 config.imu_rate, config.extrinsic_parameter_source);
          if (config.enable_connect) {
            if (!AddBroadcastCodeToWhitelist(config.broadcast_code)) {
              if (AddRawUserConfig(config)) {
                ROS_INFO("Raw config is already exist : %s ",
                       config.broadcast_code);
              }
            }
          }
        }
      }
    }

    if (ParseTimesyncConfig(doc)) {
      ROS_ERROR("Parse timesync config fail");
      enable_timesync_ = false;
    }
  } else {
    ROS_ERROR("User config file parse error[%d]",
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
  ROS_INFO("Add Raw user config : %s ", config.broadcast_code);

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



