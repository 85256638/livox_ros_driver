#include "../livox_ros_driver/livox_ros_driver/normal_dropout_policy.h"

#include <iostream>

namespace {

const int64_t kSecond = INT64_C(1000000000);

bool Check(bool condition, const char *message) {
  if (!condition) {
    std::cerr << "normal_dropout_policy_qc: " << message << "\n";
  }
  return condition;
}

livox_ros::NormalDropoutPolicyInput Armed() {
  livox_ros::NormalDropoutPolicyInput input;
  input.armed = true;
  input.identity_matches = true;
  input.generation_matches = true;
  input.healthy_since_ns = 1 * kSecond;
  return input;
}

livox_ros::NormalDropoutPolicyInput Outage() {
  livox_ros::NormalDropoutPolicyInput input = Armed();
  input.attributed_disconnect_ns = 31 * kSecond;
  input.silence_since_ns = 31 * kSecond;
  input.connected = false;
  input.broadcast_fresh = false;
  return input;
}

bool TestArmBoundary() {
  livox_ros::NormalDropoutPolicyInput input = Armed();
  return Check(!livox_ros::NormalDropoutArmMature(
                   input, 30 * kSecond, 30 * kSecond),
               "must not arm after only 29 seconds") &&
         Check(livox_ros::NormalDropoutArmMature(
                   input, 31 * kSecond, 30 * kSecond),
               "must arm at the 30-second boundary");
}

bool TestSilenceBoundary() {
  livox_ros::NormalDropoutPolicyInput input = Outage();
  return Check(!livox_ros::NormalDropoutEscalationReady(
                   input, 35 * kSecond, 5 * kSecond),
               "must not escalate after only four quiet seconds") &&
         Check(livox_ros::NormalDropoutEscalationReady(
                   input, 36 * kSecond, 5 * kSecond),
               "must escalate at the five-second boundary");
}

bool TestFailClosedInputs() {
  livox_ros::NormalDropoutPolicyInput input = Outage();
  input.generation_matches = false;
  if (!Check(!livox_ros::NormalDropoutEscalationReady(
                 input, 40 * kSecond, 5 * kSecond),
             "generation mismatch must invalidate attribution")) {
    return false;
  }
  input = Outage();
  input.identity_matches = false;
  if (!Check(!livox_ros::NormalDropoutEscalationReady(
                 input, 40 * kSecond, 5 * kSecond),
             "handle reuse must invalidate attribution")) {
    return false;
  }
  input = Outage();
  input.connected = true;
  if (!Check(!livox_ros::NormalDropoutEscalationReady(
                 input, 40 * kSecond, 5 * kSecond),
             "a live control connection must cancel hard recovery")) {
    return false;
  }
  input = Outage();
  input.broadcast_fresh = true;
  return Check(!livox_ros::NormalDropoutEscalationReady(
                   input, 40 * kSecond, 5 * kSecond),
               "fresh broadcasts must pause power and hand off to handshake");
}

}  // namespace

int main() {
  if (!TestArmBoundary()) return 1;
  if (!TestSilenceBoundary()) return 2;
  if (!TestFailClosedInputs()) return 3;
  std::cout << "normal_dropout_policy_qc: OK\n";
  return 0;
}
