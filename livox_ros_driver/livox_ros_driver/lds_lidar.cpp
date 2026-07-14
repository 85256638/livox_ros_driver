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
#include <memory>
#include <mutex>
#include <thread>

#include "health_logger.h"
#include "rapidjson/document.h"
#include "rapidjson/filereadstream.h"
#include "rapidjson/stringbuffer.h"

using namespace std;

namespace livox_ros {

namespace {
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
}  // namespace

void LdsLidar::OnLidarConnectEvent(uint8_t handle, const char *broadcast_code) {
  if (handle >= kMaxLidarCount) {
    return;
  }
  lock_guard<mutex> lock(link_stat_lock_[handle]);
  LinkStat &s = link_stat_[handle];
  if (s.connect_since_ns != 0) {
    return;  /** already counted as connected */
  }
  int64_t now = std::chrono::steady_clock::now().time_since_epoch().count();
  s.connect_since_ns = now;
  if (s.last_disconnect_ns != 0) {
    long long down_s = (now - s.last_disconnect_ns) / 1000000000LL;
    char buf[48];
    snprintf(buf, sizeof(buf), "RECONNECTED (down %llds)", down_s);
    PrintLidarEvent(handle, broadcast_code, buf);
    char detail[32];
    snprintf(detail, sizeof(detail), "down %llds", down_s);
    HealthLogger::Get().LogEvent(handle, broadcast_code, "RECONNECT", detail);
  } else {
    PrintLidarEvent(handle, broadcast_code, "CONNECTED");
    HealthLogger::Get().LogEvent(handle, broadcast_code, "CONNECT", "");
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
  s.disconnect_count++;
  s.last_disconnect_ns =
      std::chrono::steady_clock::now().time_since_epoch().count();
  s.connect_since_ns = 0;
  s.health_code = 0;  /** stale once disconnected */
  PrintLidarEvent(handle, broadcast_code, "DISCONNECTED");
  HealthLogger::Get().LogEvent(handle, broadcast_code, "DISCONNECT", "");
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
  return SendModeChangeRequest(handle, mode, false);
}

livox_status LdsLidar::RequestLidarReboot(uint8_t handle, uint16_t timeout_ms) {
  return RequestLidarRebootImpl(handle, timeout_ms, false);
}

livox_status LdsLidar::RequestLidarRebootIfModeIdle(
    uint8_t handle, uint16_t timeout_ms) {
  return RequestLidarRebootImpl(handle, timeout_ms, true);
}

livox_status LdsLidar::RequestLidarRebootImpl(
    uint8_t handle, uint16_t timeout_ms, bool require_mode_idle) {
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
  {
    lock_guard<mutex> lock(data_lock_[handle]);
    LidarDevice *p_lidar = &lidars_[handle];
    if (p_lidar->connect_state == kConnectStateOff ||
        p_lidar->handle != handle) {
      return kStatusNotConnected;
    }
  }
  return RebootDevice(handle, timeout_ms, RebootCb, this);
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

  lock_guard<mutex> lock(mode_mutex_);
  strncpy(mode_requests_[handle].broadcast_code, broadcast_code,
          sizeof(mode_requests_[handle].broadcast_code) - 1);
  mode_requests_[handle]
      .broadcast_code[sizeof(mode_requests_[handle].broadcast_code) - 1] = '\0';
}

bool LdsLidar::ResetModeRequestIfTarget(uint8_t handle, LidarMode target,
                                        uint64_t expected_request_id,
                                        uint64_t expected_command_id,
                                        uint64_t expected_generation) {
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
  strncpy(broadcast_code, request.broadcast_code, sizeof(broadcast_code) - 1);
  request = ModeChangeRequest();
  if (broadcast_code[0] != '\0') {
    strncpy(request.broadcast_code, broadcast_code,
            sizeof(request.broadcast_code) - 1);
    request.broadcast_code[sizeof(request.broadcast_code) - 1] = '\0';
  }
  return true;
}

bool LdsLidar::IsModeTransitionActive(uint8_t handle) {
  if (handle >= kMaxLidarCount) {
    return false;
  }
  lock_guard<mutex> lock(mode_mutex_);
  return mode_requests_[handle].active;
}

namespace {
/** Verify all requested modes from actual heartbeat state. Normal gets a
 *  longer retry budget because motor spin-up can take several seconds. */
const int64_t kModeVerifyIntervalNs = 2LL * 1000000000LL;  // 2 s
const uint8_t kSleepVerifyMaxRetries = 3;
const uint8_t kNormalVerifyMaxRetries = 7;
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
    bool resend = false, giveup = false, done = false, exhausted = false;
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
                        ? kNormalVerifyMaxRetries
                        : kSleepVerifyMaxRetries;
      /** Gate on elapsed time + actual state (not command_inflight): resending
       *  is idempotent, so even a lost ack still gets retried after the interval. */
      if (connect_state == kConnectStateOff) {
        if (desired == kLidarModeNormal) {
          req.command_inflight = false;
          req.waiting_for_reconnect = true;
        } else {
          giveup = true;
        }
      } else if (actual_state == ModeToState(req.desired_mode)) {
        done = true;  // mode actually took effect
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
    if (done) {
      ResetModeRequestIfTarget(h, desired, request_id, 0,
                               connection_generation);
    } else if (resend) {
      printf("Lidar[%d] not in mode[%d] yet -- re-sending (attempt %u/%u)\n", h,
             desired, attempt, max_retries);
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
      printf("Lidar[%d] did not enter mode[%d] after %u retries -- manual check "
             "needed\n",
             h, desired, max_retries);
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
    uint64_t expected_command_id) {
  if (handle >= kMaxLidarCount) {
    return kStatusInvalidHandle;
  }
  lock_guard<mutex> send_lock(mode_send_mutex_[handle]);

  bool connected = false;
  LidarConnectState connect_state = kConnectStateOff;
  LidarState actual_state = kLidarStateUnknown;
  uint64_t connection_generation = 0;
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

  uint64_t request_id = 0;
  uint64_t command_id = 0;
  {
    lock_guard<mutex> lock(mode_mutex_);
    ModeChangeRequest &request = mode_requests_[handle];
    if (!from_reconnect) {
      request.active = true;
      request.waiting_for_reconnect = false;
      request.command_inflight = false;
      request.desired_mode = mode;
      request.sleep_retry_count = 0;
      request.request_id = ++next_mode_request_id_;
    } else if (!request.active || request.desired_mode != mode ||
               (expected_request_id != 0 &&
                request.request_id != expected_request_id) ||
               request.command_id != expected_command_id) {
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
    if (connected) {
      command_id = ++next_mode_command_id_;
      request.command_id = command_id;
      request.command_inflight = true;
      request.waiting_for_reconnect = false;
      request.last_command_ns =
          std::chrono::steady_clock::now().time_since_epoch().count();
    } else if (mode != kLidarModeNormal) {
      request.active = false;
      request.command_inflight = false;
      request.waiting_for_reconnect = false;
    }
  }

  if (!connected) {
    if (mode == kLidarModeNormal) {
      printf("Queue lidar[%d] normal-mode recovery until broadcast reconnect.\n",
             handle);
      return kStatusSuccess;
    }
    return kStatusNotConnected;
  }

  std::shared_ptr<AsyncCommandContext> context = CreateCommandContext(
      handle, connection_generation, 0, kAsyncModeCommand, mode, request_id,
      command_id);
  livox_status status =
      LidarSetMode(handle, mode, SetModeCb, context.get());
  if (status != kStatusSuccess) {
    MarkCommandContextCompleted(context);
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
    retry = request.active && request.desired_mode == kLidarModeNormal &&
            request.waiting_for_reconnect && !request.command_inflight;
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
    g_lds_ldiar->RememberBroadcastCode(handle, info->broadcast_code);
    g_lds_ldiar->OnLidarConnectEvent(handle, info->broadcast_code);
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

    /** A Normal state event only completes a pending NORMAL request. It must
     *  not cancel a newer PowerSaving/Standby request: the SDK re-sends state
     *  events on health/feature changes, and a late Normal event from a wake
     *  would otherwise erase the sleep request queued right after it -- the
     *  sleep verify tick then never runs and the switch is silently lost. */
    if (info->state == ModeToState(kLidarModeNormal) &&
        active_normal_request_id != 0) {
      g_lds_ldiar->ResetModeRequestIfTarget(
          handle, kLidarModeNormal, active_normal_request_id, 0,
          event_generation);
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
          g_lds_ldiar->SendCoordinateConfig(handle, 0, config_generation);
      if (send_status != kStatusSuccess) {
        printf("Lidar[%d] coordinate config was not accepted: %d\n", handle,
               send_status);
      }

      if (kDeviceTypeLidarMid40 != device_info.type) {
        send_status = g_lds_ldiar->SendReturnModeConfig(
            handle, 0, config_generation);
        if (send_status != kStatusSuccess) {
          printf("Lidar[%d] return-mode config was not accepted: %d\n", handle,
                 send_status);
        }
      }

      if ((kDeviceTypeLidarMid70 != device_info.type) &&
          (kDeviceTypeLidarMid40 != device_info.type)) {
        send_status = g_lds_ldiar->SendImuRateConfig(
            handle, 0, config_generation);
        if (send_status != kStatusSuccess) {
          printf("Lidar[%d] IMU config was not accepted: %d\n", handle,
                 send_status);
        }
      }

      if (config.extrinsic_parameter_source == kExtrinsicParameterFromLidar) {
        send_status = g_lds_ldiar->SendExtrinsicConfig(
            handle, 0, config_generation);
        if (send_status != kStatusSuccess) {
          printf("Lidar[%d] extrinsic config was not accepted: %d\n", handle,
                 send_status);
        }
      }

      if (kDeviceTypeLidarTele == device_info.type) {
        send_status = g_lds_ldiar->SendHighSensitivityConfig(
            handle, 0, config_generation);
        if (send_status != kStatusSuccess) {
          printf("Lidar[%d] sensitivity config was not accepted: %d\n", handle,
                 send_status);
        }
        if (config.enable_high_sensitivity) {
          printf("Enable high sensitivity\n");
        } else {
          printf("Disable high sensitivity\n");
        }
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
  LidarErrorCode ec = message->lidar_error_code;
  lock_guard<mutex> link_lock(g_lds_ldiar->link_stat_lock_[handle]);
  g_lds_ldiar->link_stat_[handle].health_code = message->error_code;

  /** Track temp_status transitions (count + wall-clock time of last change). */
  static uint8_t prev_temp[kMaxLidarCount] = {0};
  static bool temp_seen[kMaxLidarCount] = {false};
  if (temp_seen[handle] && ec.temp_status != prev_temp[handle]) {
    g_lds_ldiar->link_stat_[handle].temp_change_count++;
    g_lds_ldiar->link_stat_[handle].temp_change_wall_s = (int64_t)time(nullptr);
  }
  temp_seen[handle] = true;
  prev_temp[handle] = ec.temp_status;

  /** Track fault onsets (motor/fan/dirty/volt/firmware/system going bad) so the
   *  dashboard keeps a record even after the lidar recovers -- the live
   *  columns only ever show the current state. Count the rising edge only.
   *  dirty_warn (optical window dirty/blocked) matters in dusty environments. */
  static bool prev_fault[kMaxLidarCount] = {false};
  bool fault = (ec.motor_status || ec.fan_status || ec.dirty_warn ||
                ec.volt_status || ec.firmware_err || ec.system_status);
  if (fault && !prev_fault[handle]) {
    g_lds_ldiar->link_stat_[handle].fault_count++;
    g_lds_ldiar->link_stat_[handle].fault_wall_s = (int64_t)time(nullptr);
    g_lds_ldiar->link_stat_[handle].fault_code = message->error_code;
  }
  prev_fault[handle] = fault;

  /** Only print when one of the fields worth alerting on changes (ignore
   *  pps/ptp/time-sync churn that would otherwise spam every message). */
  uint32_t watch = (ec.temp_status) | (ec.volt_status << 2) |
                   (ec.motor_status << 4) | (ec.dirty_warn << 6) |
                   (ec.firmware_err << 7) | (ec.fan_status << 8) |
                   (ec.self_heating << 9) | (ec.system_status << 10);
  static uint32_t last_watch[kMaxLidarCount] = {0};
  static bool seen[kMaxLidarCount] = {false};
  if (seen[handle] && watch == last_watch[handle]) {
    return;
  }
  seen[handle] = true;
  last_watch[handle] = watch;

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
    char broadcast_code[kBroadcastCodeSize] = {0};
    {
      lock_guard<mutex> data_lock(g_lds_ldiar->data_lock_[handle]);
      strncpy(broadcast_code,
              g_lds_ldiar->lidars_[handle].info.broadcast_code,
              sizeof(broadcast_code) - 1);
    }
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
  }

  if (!start_sampling) {
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
  bool wait_for_reconnect = false;
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
        if (response == 0) {
          if (actual_state == kLidarStateNormal) {
            printf("Lidar[%d] already in Normal; mode request complete\n",
                   handle);
            clear_request = true;
          } else {
            printf("Lidar[%d] set mode Normal accepted, waiting for state "
                   "change\n", handle);
          }
          request.waiting_for_reconnect = false;
        } else if (response == 2) {
          printf("Lidar[%d] set mode Normal: spinning up, waiting...\n", handle);
          request.waiting_for_reconnect = false;
        } else {
          printf("Lidar[%d] set mode Normal FAILED, response[%d]\n", handle, response);
          clear_request = true;
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
      }
    }
  }

  if (clear_request) {
    lds_lidar->ResetModeRequestIfTarget(
        handle, desired_mode, context->mode_request_id,
        context->mode_command_id, context->connection_generation);
  } else if (wait_for_reconnect) {
    printf("Lidar[%d] will verify/retry Normal on timer or reconnect.\n",
           handle);
  }
}

void LdsLidar::RebootCb(livox_status status, uint8_t handle, uint8_t response,
                        void *client_data) {
  printf("Lidar[%d] reboot command status[%d] response[%d]\n", handle, status,
         response);
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
