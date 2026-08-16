#ifndef LIVOX_ROS_DRIVER_NETWORK_HEALTH_POLICY_H_
#define LIVOX_ROS_DRIVER_NETWORK_HEALTH_POLICY_H_

#include <cstdint>
#include <deque>
#include <utility>

namespace livox_ros {

enum NetworkHealthState {
  kNetworkHealthUnknown = 0,
  kNetworkHealthOk,
  kNetworkHealthDegraded,
  kNetworkHealthUnstable,
  kNetworkHealthUnreachable,
};

struct NetworkHealthPolicyInput {
  bool success = false;
  int64_t now_ns = 0;
};

struct NetworkHealthPolicyResult {
  NetworkHealthState state = kNetworkHealthUnknown;
  uint32_t window_samples = 0;
  uint32_t window_failures = 0;
  uint32_t consecutive_failures = 0;
  uint32_t consecutive_successes = 0;
  double loss_percent = 0.0;
};

/**
 * Sliding ten-second policy used by the independent network probe and the
 * Driver's recovery gate. One failure is visible as DEGRADED; two failures in
 * the rolling window are UNSTABLE. Consecutive failures therefore trigger on
 * the second sample without waiting for the full window to expire, while a
 * single isolated packet loss does not immediately start a reboot.
 */
class NetworkHealthPolicy {
 public:
  NetworkHealthPolicy(int64_t window_ns = 10000000000LL,
                      uint32_t unstable_failures = 2,
                      uint32_t unreachable_consecutive_failures = 3,
                      uint32_t healthy_consecutive_successes = 5)
      : window_ns_(window_ns),
        unstable_failures_(unstable_failures),
        unreachable_consecutive_failures_(unreachable_consecutive_failures),
        healthy_consecutive_successes_(healthy_consecutive_successes) {}

  NetworkHealthPolicyResult Observe(const NetworkHealthPolicyInput &input) {
    if (input.now_ns <= 0) {
      return result_;
    }
    samples_.push_back(std::make_pair(input.now_ns, input.success));
    if (input.success) {
      ++consecutive_successes_;
      consecutive_failures_ = 0;
    } else {
      ++consecutive_failures_;
      consecutive_successes_ = 0;
    }
    while (!samples_.empty() &&
           input.now_ns - samples_.front().first > window_ns_) {
      samples_.pop_front();
    }
    uint32_t failures = 0;
    for (const auto &sample : samples_) {
      if (!sample.second) {
        ++failures;
      }
    }
    result_.window_samples = static_cast<uint32_t>(samples_.size());
    result_.window_failures = failures;
    result_.consecutive_failures = consecutive_failures_;
    result_.consecutive_successes = consecutive_successes_;
    result_.loss_percent = result_.window_samples == 0
                               ? 0.0
                               : 100.0 * failures / result_.window_samples;
    if (consecutive_failures_ >= unreachable_consecutive_failures_) {
      result_.state = kNetworkHealthUnreachable;
    } else if (failures >= unstable_failures_) {
      result_.state = kNetworkHealthUnstable;
    } else if (failures != 0) {
      result_.state = kNetworkHealthDegraded;
    } else if (consecutive_successes_ >= healthy_consecutive_successes_) {
      result_.state = kNetworkHealthOk;
    } else {
      // A clean sample before the healthy-success threshold is observation,
      // not packet loss. Keep the state UNKNOWN so a sleeping lidar is not
      // falsely shown as degraded during the first few probe seconds.
      result_.state = kNetworkHealthUnknown;
    }
    return result_;
  }

  void Reset() {
    samples_.clear();
    consecutive_failures_ = 0;
    consecutive_successes_ = 0;
    result_ = NetworkHealthPolicyResult();
  }

 private:
  int64_t window_ns_;
  uint32_t unstable_failures_;
  uint32_t unreachable_consecutive_failures_;
  uint32_t healthy_consecutive_successes_;
  std::deque<std::pair<int64_t, bool>> samples_;
  uint32_t consecutive_failures_ = 0;
  uint32_t consecutive_successes_ = 0;
  NetworkHealthPolicyResult result_;
};

}  // namespace livox_ros

#endif  // LIVOX_ROS_DRIVER_NETWORK_HEALTH_POLICY_H_
