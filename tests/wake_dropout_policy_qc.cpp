#include "../livox_ros_driver/livox_ros_driver/wake_dropout_policy.h"

#include <iostream>

namespace {

const int64_t kSecond = INT64_C(1000000000);

bool Check(bool condition, const char *message) {
  if (!condition) {
    std::cerr << "wake_dropout_policy_qc: " << message << "\n";
  }
  return condition;
}

livox_ros::WakeDropoutPolicyInput ArmedOutage() {
  livox_ros::WakeDropoutPolicyInput input;
  input.armed = true;
  input.identity_matches = true;
  input.generation_matches = true;
  input.request_id = 42;
  input.wake_started_ns = 1 * kSecond;
  input.wake_deadline_ns = 61 * kSecond;
  input.attributed_disconnect_ns = 33 * kSecond;
  input.dropout_since_ns = 33 * kSecond;
  input.connected = false;
  input.broadcast_fresh = false;
  return input;
}

bool TestConfirmationTiming() {
  livox_ros::WakeDropoutPolicyInput input = ArmedOutage();
  return Check(!livox_ros::WakeDropoutEscalationReady(
                   input, 42 * kSecond, 10 * kSecond),
               "must not escalate before ten quiet seconds") &&
         Check(livox_ros::WakeDropoutEscalationReady(
                   input, 43 * kSecond, 10 * kSecond),
               "must escalate at the ten-second boundary") &&
         Check(livox_ros::WakeDropoutEscalationReady(
                   input, 70 * kSecond, 10 * kSecond),
               "an outage begun inside the window must finish confirmation") &&
         ([&input]() {
           input.dropout_since_ns = 65 * kSecond;
           return Check(livox_ros::WakeDropoutEscalationReady(
                            input, 75 * kSecond, 10 * kSecond),
                        "transient broadcast silence may confirm after deadline");
         })();
}

bool TestFailClosedInputs() {
  livox_ros::WakeDropoutPolicyInput input = ArmedOutage();
  input.connected = true;
  if (!Check(!livox_ros::WakeDropoutEscalationReady(
                 input, 50 * kSecond, 10 * kSecond),
             "a connected lidar must never request hard power")) {
    return false;
  }
  input = ArmedOutage();
  input.broadcast_fresh = true;
  if (!Check(!livox_ros::WakeDropoutEscalationReady(
                 input, 50 * kSecond, 10 * kSecond),
             "fresh broadcasts must transfer recovery to handshake logic")) {
    return false;
  }
  input = ArmedOutage();
  input.identity_matches = false;
  if (!Check(!livox_ros::WakeDropoutEscalationReady(
                 input, 50 * kSecond, 10 * kSecond),
             "handle reuse must invalidate wake attribution")) {
    return false;
  }
  input = ArmedOutage();
  input.generation_matches = false;
  if (!Check(!livox_ros::WakeDropoutEscalationReady(
                 input, 50 * kSecond, 10 * kSecond),
             "a new connection generation must invalidate old attribution")) {
    return false;
  }
  input = ArmedOutage();
  input.attributed_disconnect_ns = 62 * kSecond;
  input.dropout_since_ns = 62 * kSecond;
  return Check(!livox_ros::WakeDropoutEscalationReady(
                   input, 80 * kSecond, 10 * kSecond),
               "an outage beginning after the window must not escalate");
}

bool TestObservationExpiry() {
  livox_ros::WakeDropoutPolicyInput input = ArmedOutage();
  input.attributed_disconnect_ns = 0;
  input.dropout_since_ns = 0;
  return Check(!livox_ros::WakeObservationExpired(input, 61 * kSecond),
               "window is inclusive at its deadline") &&
         Check(livox_ros::WakeObservationExpired(input, 62 * kSecond),
               "quiet observation must expire after its deadline");
}

}  // namespace

int main() {
  if (!TestConfirmationTiming()) return 1;
  if (!TestFailClosedInputs()) return 2;
  if (!TestObservationExpiry()) return 3;
  std::cout << "wake_dropout_policy_qc: OK\n";
  return 0;
}
