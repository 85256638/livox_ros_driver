#include <iostream>

#include "startup_missing_policy.h"

namespace {

using livox_ros_driver::StartupMissingPolicyDecision;
using livox_ros_driver::StartupMissingPolicyState;
using livox_ros_driver::UpdateStartupMissingPolicy;

constexpr int64_t kSecond = 1000000000LL;

bool Expect(bool condition, const char *message) {
  if (!condition) {
    std::cerr << message << "\n";
  }
  return condition;
}

}  // namespace

int main() {
  // Field regression: the lidar was genuinely connected while PowerSaving,
  // then PowerSaving->Normal lost both control and broadcast. WAKE_DROPOUT may
  // own that runtime incident, but STARTUP_MISSING must never appear at 30s.
  StartupMissingPolicyState powersaving_seen;
  StartupMissingPolicyDecision seen = UpdateStartupMissingPolicy(
      &powersaving_seen, true, false, 1 * kSecond, 1001, 30 * kSecond);
  if (!Expect(seen.observed_now && powersaving_seen.ever_observed,
              "PowerSaving connection did not permanently mark device seen")) {
    return 1;
  }
  for (int second = 2; second <= 120; ++second) {
    StartupMissingPolicyDecision decision = UpdateStartupMissingPolicy(
        &powersaving_seen, false, false, second * kSecond, 1000 + second,
        30 * kSecond);
    if (!Expect(!decision.required && !powersaving_seen.active,
                "observed runtime dropout was relabelled STARTUP_MISSING")) {
      return 2;
    }
  }

  StartupMissingPolicyState never_seen;
  UpdateStartupMissingPolicy(&never_seen, false, false, 1 * kSecond, 1001,
                             30 * kSecond);
  StartupMissingPolicyDecision before = UpdateStartupMissingPolicy(
      &never_seen, false, false, 30 * kSecond, 1030, 30 * kSecond);
  StartupMissingPolicyDecision required = UpdateStartupMissingPolicy(
      &never_seen, false, false, 31 * kSecond, 1031, 30 * kSecond);
  if (!Expect(!before.required && required.required &&
                  required.newly_required && never_seen.episode_count == 1,
              "never-seen whitelist member did not escalate after grace")) {
    return 3;
  }

  StartupMissingPolicyState planned;
  UpdateStartupMissingPolicy(&planned, false, false, 1 * kSecond, 1001,
                             30 * kSecond);
  StartupMissingPolicyDecision blocked = UpdateStartupMissingPolicy(
      &planned, false, true, 40 * kSecond, 1040, 30 * kSecond);
  if (!Expect(blocked.planned_outage && !planned.active &&
                  planned.absent_since_ns == 0,
              "planned shared outage did not reset startup grace")) {
    return 4;
  }
  UpdateStartupMissingPolicy(&planned, false, false, 41 * kSecond, 1041,
                             30 * kSecond);
  StartupMissingPolicyDecision after_plan = UpdateStartupMissingPolicy(
      &planned, false, false, 70 * kSecond, 1070, 30 * kSecond);
  if (!Expect(!after_plan.required,
              "planned outage time was incorrectly counted as startup absence")) {
    return 5;
  }

  std::cout << "startup_missing_policy_qc: OK\n";
  return 0;
}
