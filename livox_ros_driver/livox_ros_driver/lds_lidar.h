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
#include <map>
#include <memory>
#include <mutex>
#include <string>
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
  /** Queue a mode request with a steady-clock not-before delay. This is used
   *  by broadcast Normal requests to stagger motor spin-up without blocking
   *  the single ROS spinner thread. */
  livox_status RequestLidarModeChange(uint8_t handle, LidarMode mode,
                                      uint32_t delay_ms);
  /** Called at 1 Hz: verify every in-progress mode request from actual state
   *  and re-send within its bounded retry budget. */
  void TickSleepModeVerification();
  /** Called at 1 Hz: classify a disconnected device which is still
   *  broadcasting, and (when enabled) clear its local SDK handshake session
   *  once before escalating on a bounded schedule. */
  void TickHandshakeRecovery(bool enable_recovery);
  /** Called at 1 Hz: detect a control+broadcast dropout attributable to one
   *  explicit PowerSaving/StandBy -> Normal request and, when enabled, commit
   *  a reason-tagged hard-power escalation after ten quiet seconds. */
  void TickWakeDropoutRecovery(bool enable_recovery);
  /** Called by the 1 Hz dashboard after a coherent data-plane snapshot. A
   *  connection must publish in Normal/Sampling for 30 seconds before a later
   *  no-broadcast disconnect is allowed to request shared hard power. */
  void ObserveNormalPublishing(uint8_t handle, bool healthy,
                               uint64_t connection_generation,
                               const char *broadcast_code);
  void TickNormalDropoutRecovery(bool enable_recovery);
  static int64_t HandshakeBroadcastFreshNs() { return 3000000000LL; }
  static uint8_t HandshakeResetMaxAttempts() { return 1; }
  static int64_t WakeObservationNs() { return 60000000000LL; }
  static int64_t WakeDropoutConfirmNs() { return 10000000000LL; }
  static int64_t WakeBroadcastHandoffNs() { return 3000000000LL; }
  static uint32_t WakeBroadcastHandoffMinFrames() { return 3; }
  static int64_t NormalHealthyArmNs() { return 30000000000LL; }
  static int64_t NormalDropoutConfirmNs() { return 5000000000LL; }
  /** Fixed after InitLdsLidar; used by the Driver heartbeat to publish a
   *  fail-closed STARTUP_MISSING row for configured devices which never
   *  produce any connection/broadcast state. */
  std::vector<std::string> GetWhitelistBroadcastCodes() const;
  /** Arm a short, token-bound window immediately before the relay manager
   *  removes power from a shared group. The exact member set must equal this
   *  Driver's whitelist; an ACK is sent only after these markers are live. */
  bool ArmPlannedGroupPowerCycle(const std::vector<std::string> &members,
                                 const std::string &token,
                                 uint32_t valid_for_ms,
                                 std::string *detail);
  /** Remove an uncommitted intent when the manager cancels before its first
   *  OFF command. Already-consumed disconnect markers remain historical. */
  void CancelPlannedGroupPowerCycle(const std::vector<std::string> &members,
                                    const std::string &token);
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
  /** Read-only session identity for dashboard snapshots.  Callers which also
   *  copy LidarDevice should load this while holding data_lock_[handle]. */
  uint64_t GetConnectionGeneration(uint8_t handle) const;
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

  /** Progress of the driver's one local-session cleanup in the current
   *  broadcast-only episode.  This is deliberately independent of
   *  last_handshake_event: later SDK TIMEOUT/NETWORK_ERROR diagnostics must
   *  not make the dashboard forget whether cleanup was accepted/completed. */
  enum HandshakeResetPhase {
    kHandshakeResetNone = 0,
    kHandshakeResetRequested,
    kHandshakeResetQueued,
    kHandshakeResetCompleted,
    kHandshakeResetRejected
  };

  /** Wake recovery is independent of broadcast-only handshake recovery. */
  enum WakeRecoveryState {
    kWakeRecoveryIdle = 0,
    kWakeRecoveryObserving,
    kWakeRecoveryNoBroadcast,
    kWakeRecoveryDropout,
    kWakeRecoveryPowerCycleRequired
  };

  enum NormalDropoutRecoveryState {
    kNormalDropoutIdle = 0,
    kNormalDropoutArmed,
    kNormalDropoutNoBroadcast,
    kNormalDropoutObservingReturn,
    kNormalDropoutConfirmed,
    kNormalDropoutPowerCycleRequired
  };

  /** Current cause of the generic POWER_CYCLE_REQUIRED edge. */
  enum PowerCycleReason {
    kPowerCycleReasonNone = 0,
    kPowerCycleReasonHandshakeStuck,
    kPowerCycleReasonWakeDropout,
    kPowerCycleReasonNormalDropout,
    kPowerCycleReasonStartupMissing
  };

  struct LinkStat {
    uint32_t disconnect_count = 0;
    int64_t last_disconnect_ns = 0;  /**< steady_clock ns, 0 = never */
    int64_t connect_since_ns = 0;    /**< steady_clock ns, 0 = not connected */
    uint32_t health_code = 0;        /**< latest LidarErrorCode bits (temp/fan/...) */
    /** Edge-detection baselines belong to this physical broadcast code.  Keep
     *  them in LinkStat so handle reassignment resets them together with the
     *  public health history instead of leaking static per-handle state. */
    bool health_temp_seen = false;
    uint8_t health_prev_temp = 0;
    bool health_prev_fault = false;
    bool health_watch_seen = false;
    uint32_t health_last_watch = 0;
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
    uint8_t handshake_reset_attempts = 0; /**< reset requests this episode,
                                           *   max 1 */
    int64_t handshake_last_reset_try_ns = 0;
    HandshakeResetPhase handshake_reset_phase = kHandshakeResetNone;
    bool handshake_reset_accepted = false;  /**< API accepted this episode */
    bool handshake_reset_completed = false; /**< SDK RESET event confirmed */
    int64_t handshake_reset_completed_ns = 0; /**< steady_clock completion */
    uint32_t handshake_reset_count = 0;   /**< accepted SDK session resets */
    uint32_t handshake_reset_fail_count = 0;
    int64_t handshake_last_reset_wall_s = 0;
    uint32_t handshake_stuck_count = 0;   /**< episodes reaching STUCK */
    int64_t handshake_stuck_wall_s = 0;
    /** Committed entries into POWER_CYCLE_REQUIRED.  A NETWORK_ERROR can
     *  cancel an entry before its request is published and allow a later
     *  re-entry in the same episode, so this is a transition sequence rather
     *  than an incident count. */
    uint32_t power_cycle_required_count = 0;
    /** Number of distinct handle-backed handshake, wake or normal-dropout
     *  episodes which reached POWER_CYCLE_REQUIRED at least once. Startup-
     *  missing events use synthetic handle 255 and are tracked separately. */
    uint32_t power_cycle_required_episode_count = 0;
    /** Per-episode latch preventing a cancelled/re-offered edge from being
     *  counted as another distinct failure episode. */
    bool power_cycle_required_counted_this_episode = false;
    int64_t power_cycle_required_wall_s = 0;
    PowerCycleReason power_cycle_reason = kPowerCycleReasonNone;
    /** Cause-specific episode counts keep wake failures out of handshake
     *  history while the generic counts above retain unique event identity. */
    uint32_t handshake_power_cycle_episode_count = 0;
    uint32_t wake_power_cycle_episode_count = 0;
    uint32_t normal_power_cycle_episode_count = 0;
    /** Shared relay outages acknowledged before OFF are maintenance actions,
     *  not single-lidar instability. Keep them visible without incrementing
     *  disconnect/normal-dropout/power-escalation history. */
    uint32_t planned_group_power_cycle_count = 0;
    bool planned_group_power_cycle_active = false;
    int64_t planned_group_power_cycle_deadline_ns = 0;
    char planned_group_power_cycle_token[129] = {0};
    /** An explicit wake guard survives completion of ModeChangeRequest because
     *  field units can enter Normal/Config and then disappear tens of seconds
     *  later. Identity and generation are captured before the command send. */
    WakeRecoveryState wake_state = kWakeRecoveryIdle;
    uint64_t wake_request_id = 0;
    uint64_t wake_connection_generation = 0;
    /** Generation captured by the disconnect callback.  Keeping the concrete
     *  value (rather than only a boolean) lets the relay manager validate the
     *  same-session evidence carried on the recovery wire. */
    uint64_t wake_dropout_generation = 0;
    /** A deliberate soft reboot temporarily owns the next disconnect. */
    uint64_t planned_reboot_generation = 0;
    int64_t wake_started_ns = 0;
    int64_t wake_deadline_ns = 0;
    int64_t wake_started_wall_s = 0;
    /** First strict-identity disconnect attributed inside the 60s window. */
    int64_t wake_attributed_disconnect_ns = 0;
    int64_t wake_attributed_disconnect_wall_s = 0;
    /** Start of the current uninterrupted no-broadcast confirmation interval. */
    int64_t wake_dropout_since_ns = 0;
    int64_t wake_dropout_wall_s = 0;
    /** A few residual broadcast frames do not prove stable recovery. */
    int64_t wake_broadcast_return_since_ns = 0;
    uint32_t wake_broadcast_return_count = 0;
    uint32_t wake_dropout_count = 0;
    /** One explicit wake request can contain more than one silence interval
     *  when a stray broadcast frame briefly returns.  Count the causal wake
     *  episode once while allowing a later live power edge to be re-offered. */
    bool wake_dropout_counted_this_request = false;
    bool wake_power_cycle_counted_this_request = false;
    char wake_broadcast_code[kBroadcastCodeSize] = {0};
    /** A normal dropout is armed only by sustained real point publication.
     *  It is independent of explicit-wake attribution and retains the first
     *  disconnect while residual broadcasts are tested for a stable handoff. */
    NormalDropoutRecoveryState normal_dropout_state = kNormalDropoutIdle;
    uint64_t normal_connection_generation = 0;
    uint64_t normal_dropout_generation = 0;
    int64_t normal_healthy_since_ns = 0;
    int64_t normal_healthy_since_wall_s = 0;
    int64_t normal_attributed_disconnect_ns = 0;
    int64_t normal_attributed_disconnect_wall_s = 0;
    int64_t normal_dropout_since_ns = 0;
    int64_t normal_dropout_wall_s = 0;
    int64_t normal_broadcast_return_since_ns = 0;
    uint32_t normal_broadcast_return_count = 0;
    uint32_t normal_dropout_count = 0;
    bool normal_dropout_counted_this_episode = false;
    bool normal_power_cycle_counted_this_episode = false;
    char normal_broadcast_code[kBroadcastCodeSize] = {0};
    /** Exact reason from the paired SDK's handshake diagnostics. A handshake
     *  ACK is not a public kEventConnect: DeviceInfo may still be pending, so
     *  these events never clear the recovery budget. */
    bool handshake_event_valid = false;
    DeviceHandshakeEvent last_handshake_event = kDeviceHandshakeSuccess;
    int32_t last_handshake_detail = 0;
    int64_t last_handshake_event_wall_s = 0;
    char last_handshake_ip[16] = {0};
    /** steady_clock timestamp for the most recent NETWORK_ERROR in the live
     *  broadcast-only episode. Kept separately because later RESET/TIMEOUT
     *  diagnostics must not erase the local-network power-cycle gate. */
    int64_t handshake_last_network_error_ns = 0;
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
  bool ConsumePlannedGroupPowerCycle(const char *broadcast_code,
                                     std::string *token);
  void ArmWakeObservation(uint8_t handle, const char *broadcast_code,
                          uint64_t request_id,
                          uint64_t connection_generation);
  void CancelWakeObservation(uint8_t handle, uint64_t expected_request_id = 0);
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
      send_not_before_ns = 0;
      normal_spinup_grace_deadline_ns = 0;
      sleep_retry_count = 0;
      normal_post_grace_retry_count = 0;
      explicit_wake_source = false;
      explicit_wake_generation = 0;
      wake_observation_armed = false;
      memset(broadcast_code, 0, sizeof(broadcast_code));
    }

    bool active;
    bool waiting_for_reconnect;
    bool command_inflight;
    LidarMode desired_mode;
    uint64_t request_id;       /**< identifies callbacks belonging to this request */
    uint64_t command_id;       /**< identifies the latest send attempt */
    int64_t last_command_ns;    /**< steady_clock ns of the last SetMode send (for sleep verify/retry) */
    int64_t send_not_before_ns; /**< delayed first send; never sleeps a callback thread */
    int64_t normal_spinup_grace_deadline_ns; /**< fixed by first positive Normal ACK; retries never extend it */
    uint8_t sleep_retry_count;  /**< verification re-sends for the request */
    uint8_t normal_post_grace_retry_count; /**< retries after acknowledged spin-up grace */
    bool explicit_wake_source; /**< initial explicit request saw PowerSaving/StandBy */
    uint64_t explicit_wake_generation; /**< session which supplied that low-power fact */
    bool wake_observation_armed; /**< first accepted SDK enqueue armed LinkStat */
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
                                     uint64_t expected_command_id = 0,
                                     int64_t initial_not_before_ns = 0);
  void MaybeRetryPendingModeRequest(uint8_t handle);
  uint64_t GetActiveNormalRequestId(uint8_t handle);
  void CompleteConfigCommand(uint8_t handle, uint32_t config_bit,
                             uint64_t connection_generation);
  livox_status SendNextConfigCommand(uint8_t handle,
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

  struct PlannedGroupPowerCycle {
    std::string token;
    int64_t expires_ns;
  };
  std::mutex planned_group_power_cycle_lock_;
  std::map<std::string, PlannedGroupPowerCycle> planned_group_power_cycles_;

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
