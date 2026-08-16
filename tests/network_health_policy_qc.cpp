#include "../livox_ros_driver/livox_ros_driver/network_health_policy.h"

#include <iostream>

namespace {

const int64_t kSecond = INT64_C(1000000000);

bool Check(bool condition, const char *message) {
  if (!condition) {
    std::cerr << "network_health_policy_qc: " << message << "\n";
  }
  return condition;
}

livox_ros::NetworkHealthPolicyResult Observe(
    livox_ros::NetworkHealthPolicy *policy, int64_t second, bool success) {
  livox_ros::NetworkHealthPolicyInput input;
  input.now_ns = second * kSecond;
  input.success = success;
  return policy->Observe(input);
}

bool TestSlidingTenSecondWindow() {
  livox_ros::NetworkHealthPolicy policy;
  livox_ros::NetworkHealthPolicyResult observing = Observe(&policy, 1, true);
  if (!Check(observing.state == livox_ros::kNetworkHealthUnknown,
             "clean startup samples before confirmation must be unknown")) {
    return false;
  }
  Observe(&policy, 2, true);
  Observe(&policy, 3, true);
  Observe(&policy, 4, true);
  livox_ros::NetworkHealthPolicyResult healthy = Observe(&policy, 5, true);
  if (!Check(healthy.state == livox_ros::kNetworkHealthOk,
             "five clean samples must be healthy")) {
    return false;
  }
  livox_ros::NetworkHealthPolicyResult one_loss = Observe(&policy, 6, false);
  if (!Check(one_loss.state == livox_ros::kNetworkHealthDegraded,
             "one isolated loss must be degraded")) {
    return false;
  }
  livox_ros::NetworkHealthPolicyResult two_losses =
      Observe(&policy, 7, false);
  if (!Check(two_losses.state == livox_ros::kNetworkHealthUnstable,
             "two losses inside ten seconds must be unstable")) {
    return false;
  }
  livox_ros::NetworkHealthPolicyResult aged_out =
      Observe(&policy, 18, true);
  return Check(aged_out.window_failures == 0,
               "failures older than ten seconds must expire");
}

bool TestUnreachableAndRecoveryBoundaries() {
  livox_ros::NetworkHealthPolicy policy;
  Observe(&policy, 1, false);
  Observe(&policy, 2, false);
  livox_ros::NetworkHealthPolicyResult unreachable =
      Observe(&policy, 3, false);
  if (!Check(unreachable.state == livox_ros::kNetworkHealthUnreachable,
             "three consecutive failures must be unreachable")) {
    return false;
  }
  for (int second = 4; second <= 13; ++second) {
    Observe(&policy, second, true);
  }
  livox_ros::NetworkHealthPolicyResult healthy = Observe(&policy, 14, true);
  return Check(healthy.state == livox_ros::kNetworkHealthOk,
               "five consecutive successes must be healthy");
}

}  // namespace

int main() {
  if (!TestSlidingTenSecondWindow()) return 1;
  if (!TestUnreachableAndRecoveryBoundaries()) return 2;
  std::cout << "network_health_policy_qc: OK\n";
  return 0;
}
