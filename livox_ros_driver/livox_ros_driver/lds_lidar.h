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

/** Livox LiDAR data source, data from dependent lidar */

#ifndef LIVOX_ROS_DRIVER_LDS_LIDAR_H_
#define LIVOX_ROS_DRIVER_LDS_LIDAR_H_

#include <cstring>
#include <array>
#include <memory>
#include <mutex>
#include <vector>

#include "lds.h"
#include "livox_sdk.h"
#include "rapidjson/document.h"
#include "timesync.h"

namespace livox_ros {

/**
 * LiDAR data source, data from dependent lidar.
 */
class LdsLidar : public Lds {
 public:
  static LdsLidar *GetInstance(uint32_t interval_ms) {
    static LdsLidar lds_lidar(interval_ms);
    return &lds_lidar;
  }

  int InitLdsLidar(std::vector<std::string> &broadcast_code_strs,
                   const char *user_config_path);
  int DeInitLdsLidar(void);
  livox_status RequestLidarModeChange(const char *broadcast_code,
                                      LidarMode mode);
  livox_status RequestLidarModeChange(uint8_t handle, LidarMode mode);
  /** Called at 1 Hz: for an in-progress PowerSaving/Standby request, verify the
   *  lidar's actual state really reached the target and re-send if it did not
   *  (some lidars ack "success" without switching). No-op for Normal requests. */
  void TickSleepModeVerification();
  livox_status RequestLidarReboot(uint8_t handle, uint16_t timeout_ms = 100);
  livox_status RequestRestartSampling(uint8_t handle);

  /** Per-lidar connection history. Lives outside LidarDevice (which ResetLidar
   *  memsets on disconnect) so it survives disconnect/reconnect cycles. Read by
   *  the stats dashboard. */
  struct LinkStat {
    uint32_t disconnect_count = 0;
    int64_t last_disconnect_ns = 0;  /**< steady_clock ns, 0 = never */
    int64_t connect_since_ns = 0;    /**< steady_clock ns, 0 = not connected */
    uint32_t health_code = 0;        /**< latest LidarErrorCode bits (temp/fan/...) */
    uint32_t temp_change_count = 0;  /**< times temp_status changed since start */
    int64_t temp_change_wall_s = 0;  /**< wall-clock (time_t) of last temp change, 0=never */
    uint32_t fault_count = 0;        /**< times entered a motor/fan/volt/fw/system fault */
    int64_t fault_wall_s = 0;        /**< wall-clock (time_t) of last fault onset, 0=never */
    uint32_t fault_code = 0;         /**< health bits captured at last fault onset */
    uint32_t recover_reboot_count = 0; /**< auto_recover reboots issued for this lidar */
    int64_t recover_last_wall_s = 0;   /**< wall-clock (time_t) of last auto_recover reboot */
    uint32_t mode_fail_count = 0;      /**< PowerSaving/Standby switches that failed after all retries */
    int64_t mode_fail_wall_s = 0;      /**< wall-clock (time_t) of last such failure, 0=never */
    uint8_t mode_fail_mode = 0;        /**< mode it failed to enter (2=PowerSaving, 3=Standby) */
  };
  LinkStat link_stat_[kMaxLidarCount];

 private:
  void OnLidarConnectEvent(uint8_t handle, const char *broadcast_code);
  void OnLidarDisconnectEvent(uint8_t handle, const char *broadcast_code);

  struct ModeChangeRequest {
    ModeChangeRequest() {
      active = false;
      waiting_for_reconnect = false;
      command_inflight = false;
      desired_mode = kLidarModeNormal;
      last_command_ns = 0;
      sleep_retry_count = 0;
      memset(broadcast_code, 0, sizeof(broadcast_code));
    }

    bool active;
    bool waiting_for_reconnect;
    bool command_inflight;
    LidarMode desired_mode;
    int64_t last_command_ns;    /**< steady_clock ns of the last SetMode send (for sleep verify/retry) */
    uint8_t sleep_retry_count;  /**< PowerSaving/Standby re-sends done for the current request */
    char broadcast_code[kBroadcastCodeSize];
  };

  LdsLidar(uint32_t interval_ms);
  LdsLidar(const LdsLidar &) = delete;
  ~LdsLidar();
  LdsLidar &operator=(const LdsLidar &) = delete;
  virtual void PrepareExit(void);

  static void OnLidarDataCb(uint8_t handle, LivoxEthPacket *data,
                            uint32_t data_num, void *client_data);
  static void OnDeviceBroadcast(const BroadcastDeviceInfo *info);
  static void OnDeviceChange(const DeviceInfo *info, DeviceEvent type);
  static void StartSampleCb(livox_status status, uint8_t handle,
                            uint8_t response, void *clent_data);
  static void StopSampleCb(livox_status status, uint8_t handle,
                           uint8_t response, void *clent_data);
  static void DeviceInformationCb(livox_status status, uint8_t handle,
                                  DeviceInformationResponse *ack,
                                  void *clent_data);
  static void LidarErrorStatusCb(livox_status status, uint8_t handle,
                                 ErrorMessage *message);
  static void ControlFanCb(livox_status status, uint8_t handle,
                           uint8_t response, void *clent_data);
  static void SetPointCloudReturnModeCb(livox_status status, uint8_t handle,
                                        uint8_t response, void *clent_data);
  static void SetCoordinateCb(livox_status status, uint8_t handle,
                              uint8_t response, void *clent_data);
  static void SetImuRatePushFrequencyCb(livox_status status, uint8_t handle,
                                        uint8_t response, void *clent_data);
  static void SetRmcSyncTimeCb(livox_status status, uint8_t handle,
                               uint8_t response, void *client_data);
  static void ReceiveSyncTimeCallback(const char *rmc, uint32_t rmc_length,
                                      void *client_data);
  static void GetLidarExtrinsicParameterCb(
      livox_status status, uint8_t handle,
      LidarGetExtrinsicParameterResponse *response, void *clent_data);
  static void SetHighSensitivityCb(livox_status status, uint8_t handle,
                                   DeviceParameterResponse *response,
                                   void *clent_data);
  static void SetModeCb(livox_status status, uint8_t handle, uint8_t response,
                        void *client_data);
  static void RebootCb(livox_status status, uint8_t handle, uint8_t response,
                       void *client_data);

  void ResetLdsLidar(void);
  int AddBroadcastCodeToWhitelist(const char *broadcast_code);
  bool IsBroadcastCodeExistInWhitelist(const char *broadcast_code);
  void RememberBroadcastCode(uint8_t handle, const char *broadcast_code);
  void ResetModeRequest(uint8_t handle);
  void MarkModeRequestDisconnected(uint8_t handle);
  livox_status SendModeChangeRequest(uint8_t handle, LidarMode mode,
                                     bool from_reconnect);
  void MaybeRetryPendingModeRequest(uint8_t handle);
  bool HasActiveNormalRequest(uint8_t handle);

  void EnableAutoConnectMode(void) { auto_connect_mode_ = true; }
  void DisableAutoConnectMode(void) { auto_connect_mode_ = false; }
  bool IsAutoConnectMode(void) { return auto_connect_mode_; }
  int ParseTimesyncConfig(rapidjson::Document &doc);
  int ParseConfigFile(const char *pathname);
  int AddRawUserConfig(UserRawConfig &config);
  bool IsExistInRawConfig(const char *broadcast_code);
  int GetRawConfig(const char *broadcast_code, UserRawConfig &config);

  bool auto_connect_mode_;
  uint32_t whitelist_count_;
  volatile bool is_initialized_;
  char broadcast_code_whitelist_[kMaxLidarCount][kBroadcastCodeSize];
  std::vector<UserRawConfig> raw_config_;

  bool enable_timesync_;
  TimeSync *timesync_;
  TimeSyncConfig timesync_config_;
  std::mutex config_mutex_;
  std::mutex mode_mutex_;
  std::array<ModeChangeRequest, kMaxLidarCount> mode_requests_;
};

}  // namespace livox_ros
#endif
