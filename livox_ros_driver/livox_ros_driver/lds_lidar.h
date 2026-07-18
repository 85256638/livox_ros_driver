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

#include <atomic>
#include <cstring>
#include <array>
#include <chrono>
#include <list>
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
  /** Called at 1 Hz: verify every in-progress mode request from actual state
   *  and re-send within its bounded retry budget. */
  void TickSleepModeVerification();
  /** Called at 1 Hz: classify a disconnected device which is still
   *  broadcasting, and (when enabled) clear/retry its local SDK handshake
   *  session on a bounded schedule. */
  void TickHandshakeRecovery(bool enable_recovery);
  static int64_t HandshakeBroadcastFreshNs() { return 3000000000LL; }
  static uint8_t HandshakeResetMaxAttempts() { return 3; }
  livox_status RequestLidarReboot(uint8_t handle, uint16_t timeout_ms = 100);
  /** Watchdog variant: reject if a planned mode request won the per-handle
   *  send race. Manual reboot remains an explicit override. */
  livox_status RequestLidarRebootIfModeIdle(uint8_t handle,
                                            uint16_t timeout_ms = 100);
  livox_status RequestRestartSampling(uint8_t handle);
  /** True while this lidar has an unfinished Normal/PowerSaving/Standby
   *  request. Thread-safe; used by recovery code to avoid fighting a planned
   *  mode transition. */
  bool IsModeTransitionActive(uint8_t handle);
  /** Reclaim callback contexts after the paired SDK has completed/cancelled
   *  them and a grace period has elapsed. */
  void ReapCommandContexts();

  /** Per-lidar connection history. Lives outside LidarDevice (which ResetLidar
   *  memsets on disconnect) so it survives disconnect/reconnect cycles. Read by
   *  the stats dashboard. */
  enum HandshakeLinkState {
    kHandshakeLinkIdle = 0,
    kHandshakeLinkBroadcastOnly,
    kHandshakeLinkStuck,
    kHandshakeLinkPowerCycleRequired
  };

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
    uint32_t mode_fail_count = 0;      /**< mode switches that exhausted retries */
    int64_t mode_fail_wall_s = 0;      /**< wall-clock (time_t) of last such failure, 0=never */
    uint8_t mode_fail_mode = 0;        /**< mode it failed to enter (1/2/3) */
    /** Broadcast/handshake state is independent of the heartbeat connection.
     *  This makes the field failure "broadcast thread alive, control service
     *  cannot handshake" visible even if this process has never connected. */
    char broadcast_code[kBroadcastCodeSize] = {0};
    uint64_t broadcast_count = 0;
    int64_t last_broadcast_ns = 0;
    int64_t broadcast_only_since_ns = 0;
    HandshakeLinkState handshake_state = kHandshakeLinkIdle;
    uint8_t handshake_reset_attempts = 0; /**< current episode, max 3 */
    int64_t handshake_last_reset_try_ns = 0;
    uint32_t handshake_reset_count = 0;   /**< accepted SDK session resets */
    uint32_t handshake_reset_fail_count = 0;
    int64_t handshake_last_reset_wall_s = 0;
    uint32_t handshake_stuck_count = 0;   /**< episodes reaching STUCK */
    int64_t handshake_stuck_wall_s = 0;
    uint32_t power_cycle_required_count = 0;
    int64_t power_cycle_required_wall_s = 0;
    /** Exact reason from the paired SDK's handshake diagnostics. A handshake
     *  ACK is not a public kEventConnect: DeviceInfo may still be pending, so
     *  these events never clear the recovery budget. */
    bool handshake_event_valid = false;
    DeviceHandshakeEvent last_handshake_event = kDeviceHandshakeSuccess;
    int32_t last_handshake_detail = 0;
    int64_t last_handshake_event_wall_s = 0;
    char last_handshake_ip[16] = {0};
    uint32_t handshake_success_count = 0;
    uint32_t handshake_timeout_count = 0;
    uint32_t handshake_rejected_count = 0;
    uint32_t handshake_network_error_count = 0;
    uint32_t handshake_protocol_error_count = 0;
  };
  LinkStat link_stat_[kMaxLidarCount];
  /** SDK event callbacks update LinkStat concurrently with the 1 Hz dashboard
   *  and recovery timer. Use one lock per handle for coherent snapshots. */
  std::mutex link_stat_lock_[kMaxLidarCount];

 private:
  void OnLidarConnectEvent(uint8_t handle, const char *broadcast_code);
  void OnLidarDisconnectEvent(uint8_t handle, const char *broadcast_code);
  void OnLidarBroadcastEvent(uint8_t handle, const char *broadcast_code);
  livox_status RequestLidarRebootImpl(uint8_t handle, uint16_t timeout_ms,
                                      bool require_mode_idle);

  struct ModeChangeRequest {
    ModeChangeRequest() {
      active = false;
      waiting_for_reconnect = false;
      command_inflight = false;
      desired_mode = kLidarModeNormal;
      request_id = 0;
      command_id = 0;
      last_command_ns = 0;
      sleep_retry_count = 0;
      memset(broadcast_code, 0, sizeof(broadcast_code));
    }

    bool active;
    bool waiting_for_reconnect;
    bool command_inflight;
    LidarMode desired_mode;
    uint64_t request_id;       /**< identifies callbacks belonging to this request */
    uint64_t command_id;       /**< identifies the latest send attempt */
    int64_t last_command_ns;    /**< steady_clock ns of the last SetMode send (for sleep verify/retry) */
    uint8_t sleep_retry_count;  /**< verification re-sends for the request */
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
  static void OnDeviceHandshake(const DeviceHandshakeStatus *status);
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
  bool ResetModeRequestIfTarget(uint8_t handle, LidarMode target,
                                uint64_t expected_request_id = 0,
                                uint64_t expected_command_id = 0,
                                uint64_t expected_generation = 0);
  void MarkModeRequestDisconnected(uint8_t handle);
  livox_status SendModeChangeRequest(uint8_t handle, LidarMode mode,
                                     bool from_reconnect,
                                     uint64_t expected_request_id = 0,
                                     uint64_t expected_generation = 0,
                                     uint64_t expected_command_id = 0);
  void MaybeRetryPendingModeRequest(uint8_t handle);
  uint64_t GetActiveNormalRequestId(uint8_t handle);
  void CompleteConfigCommand(uint8_t handle, uint32_t config_bit,
                             uint64_t connection_generation);
  livox_status SendStartSampling(uint8_t handle,
                                 uint64_t expected_generation = 0);
  livox_status SendCoordinateConfig(uint8_t handle, uint8_t retry_count = 0,
                                    uint64_t expected_generation = 0);
  livox_status SendReturnModeConfig(uint8_t handle, uint8_t retry_count = 0,
                                    uint64_t expected_generation = 0);
  livox_status SendImuRateConfig(uint8_t handle, uint8_t retry_count = 0,
                                 uint64_t expected_generation = 0);
  livox_status SendExtrinsicConfig(uint8_t handle, uint8_t retry_count = 0,
                                   uint64_t expected_generation = 0);
  livox_status SendHighSensitivityConfig(uint8_t handle,
                                         uint8_t retry_count = 0,
                                         uint64_t expected_generation = 0);
  bool IsCurrentConfigContext(uint8_t handle,
                              uint64_t connection_generation);

  enum AsyncCommandKind {
    kAsyncConfigCommand,
    kAsyncStartSamplingCommand,
    kAsyncModeCommand
  };

  struct AsyncCommandContext {
    uint8_t handle;
    uint64_t connection_generation;
    uint8_t retry_count;
    AsyncCommandKind kind;
    LidarMode mode;
    uint64_t mode_request_id;
    uint64_t mode_command_id;
    std::atomic<bool> completed;
    std::chrono::steady_clock::time_point created_at;
    std::chrono::steady_clock::time_point completed_at;
    AsyncCommandContext(uint8_t h, uint64_t generation, uint8_t retry,
                        AsyncCommandKind command_kind, LidarMode command_mode,
                        uint64_t request_id, uint64_t command_id)
        : handle(h),
          connection_generation(generation),
          retry_count(retry),
          kind(command_kind),
          mode(command_mode),
          mode_request_id(request_id),
          mode_command_id(command_id),
          completed(false),
          created_at(std::chrono::steady_clock::now()),
          completed_at() {}
  };
  using StartSampleContext = AsyncCommandContext;
  using ConfigCommandContext = AsyncCommandContext;

  std::shared_ptr<AsyncCommandContext> CreateCommandContext(
      uint8_t handle, uint64_t connection_generation, uint8_t retry_count,
      AsyncCommandKind kind = kAsyncConfigCommand,
      LidarMode mode = kLidarModeNormal, uint64_t mode_request_id = 0,
      uint64_t mode_command_id = 0);
  void MarkCommandContextCompleted(
      const std::shared_ptr<AsyncCommandContext> &context);
  std::shared_ptr<AsyncCommandContext> AcquireCommandContext(
      void *raw_context, AsyncCommandKind expected_kind);

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
  /** Serialize SDK session validation/enqueue and connect/disconnect per
   *  handle. It also keeps newer mode requests behind older sends. */
  std::mutex mode_send_mutex_[kMaxLidarCount];
  std::array<ModeChangeRequest, kMaxLidarCount> mode_requests_;
  uint64_t next_mode_request_id_;
  uint64_t next_mode_command_id_;
  /** Incremented on every connect/disconnect edge. Async start-sampling
   *  callbacks carry the generation they were sent in, so a late ACK cannot
   *  mutate a new connection that reused the same SDK handle. */
  std::array<std::atomic<uint64_t>, kMaxLidarCount> connection_generation_;
  std::mutex command_context_mutex_;
  std::list<std::shared_ptr<AsyncCommandContext>> command_contexts_;
};

}  // namespace livox_ros
#endif
