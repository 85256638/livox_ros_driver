#include <iostream>

#include "measurement_session_policy.h"

namespace {

using livox_ros::BeginMeasurementSession;
using livox_ros::CommitMeasurementErrorPowerCycle;
using livox_ros::CommitMeasurementErrorReboot;
using livox_ros::FinishMeasurementSession;
using livox_ros::MeasurementSessionCloseEligible;
using livox_ros::MeasurementSessionState;
using livox_ros::MeasurementTickInput;
using livox_ros::MeasurementTickResult;
using livox_ros::TickMeasurementSession;
using livox_ros::kMeasurementErrorNone;
using livox_ros::kMeasurementErrorPowerCycle;
using livox_ros::kMeasurementErrorSoftReboot;

MeasurementTickResult Tick(MeasurementSessionState *state, bool error,
                           bool publishing, uint64_t generation,
                           int64_t second) {
  MeasurementTickInput input;
  input.connected = true;
  input.in_error = error;
  input.normal_sampling_publishing = publishing;
  input.auto_recover = true;
  input.connection_generation = generation;
  input.now_ns = second * 1000000000LL;
  input.now_wall_s = 1000 + second;
  return TickMeasurementSession(state, input);
}

bool Expect(bool condition, const char *message) {
  if (!condition) {
    std::cerr << message << "\n";
  }
  return condition;
}

}  // namespace

int main() {
  MeasurementSessionState state;
  BeginMeasurementSession(&state, false, 1000000000LL, 1001);
  if (!Expect(state.active && !state.implicit && state.session_id == 1,
              "explicit session did not start")) {
    return 1;
  }

  if (!Expect(Tick(&state, true, false, 10, 2).action ==
                  kMeasurementErrorNone &&
                  Tick(&state, true, false, 10, 3).action ==
                  kMeasurementErrorNone &&
                  Tick(&state, true, false, 10, 4).action ==
                  kMeasurementErrorNone &&
                  Tick(&state, true, false, 10, 5).action ==
                  kMeasurementErrorSoftReboot,
              "first Error was not confirmed after three full seconds")) {
    return 2;
  }
  CommitMeasurementErrorReboot(&state, 10, 5000000000LL);
  for (int second = 6; second < 66; ++second) {
    Tick(&state, false, true, 11, second);
  }
  if (!Expect(state.error_reboot_attempts == 1,
              "healthy publication incorrectly cleared the session budget")) {
    return 3;
  }

  for (uint8_t expected = 2; expected <= 3; ++expected) {
    const int64_t base = 70 + expected * 10;
    Tick(&state, true, false, expected + 10, base);
    Tick(&state, true, false, expected + 10, base + 1);
    Tick(&state, true, false, expected + 10, base + 2);
    MeasurementTickResult result =
        Tick(&state, true, false, expected + 10, base + 3);
    if (!Expect(result.action == kMeasurementErrorSoftReboot,
                "new-generation Error did not request the next reboot")) {
      return 4;
    }
    CommitMeasurementErrorReboot(&state, expected + 10,
                                 (base + 3) * 1000000000LL);
    if (!Expect(state.error_reboot_attempts == expected,
                "accepted reboot attempt count mismatch")) {
      return 5;
    }
    Tick(&state, false, true, expected + 11, base + 4);
  }

  Tick(&state, true, false, 20, 120);
  Tick(&state, true, false, 20, 121);
  Tick(&state, true, false, 20, 122);
  MeasurementTickResult exhausted = Tick(&state, true, false, 20, 123);
  if (!Expect(exhausted.action == kMeasurementErrorPowerCycle,
              "fourth Error did not escalate after three soft reboots")) {
    return 6;
  }
  CommitMeasurementErrorPowerCycle(&state);
  if (!Expect(state.error_power_cycle_required,
              "power-cycle requirement was not latched")) {
    return 7;
  }

  MeasurementTickResult recovered = Tick(&state, false, true, 20, 124);
  if (!Expect(recovered.power_cycle_cancelled &&
                  !state.error_power_cycle_required &&
                  state.error_reboot_attempts == 3,
              "transient recovery did not cancel only the live power edge")) {
    return 8;
  }

  MeasurementSessionState ignored_reboot;
  BeginMeasurementSession(&ignored_reboot, false, 0, 1000);
  ignored_reboot.error_reboot_attempts = 1;
  ignored_reboot.last_error_reboot_generation = 7;
  ignored_reboot.last_error_reboot_ns = 10000000000LL;
  Tick(&ignored_reboot, true, false, 7, 11);
  Tick(&ignored_reboot, true, false, 7, 12);
  Tick(&ignored_reboot, true, false, 7, 13);
  if (!Expect(Tick(&ignored_reboot, true, false, 7, 13).action ==
                  kMeasurementErrorNone,
              "same-generation accepted reboot ignored its cooldown")) {
    return 9;
  }
  if (!Expect(Tick(&ignored_reboot, true, false, 7, 50).action ==
                  kMeasurementErrorSoftReboot,
              "same-generation cooldown never released")) {
    return 10;
  }

  MeasurementSessionState close_state;
  BeginMeasurementSession(&close_state, false, 0, 1000);
  Tick(&close_state, false, true, 1, 1);
  Tick(&close_state, false, true, 1, 2);
  Tick(&close_state, false, true, 1, 3);
  if (!Expect(!MeasurementSessionCloseEligible(close_state),
              "less than three full seconds incorrectly confirmed recovery")) {
    return 11;
  }
  Tick(&close_state, false, true, 1, 4);
  if (!Expect(MeasurementSessionCloseEligible(close_state),
              "three point-cloud ticks did not confirm recovery")) {
    return 12;
  }
  if (!Expect(FinishMeasurementSession(&close_state, true) &&
                  !close_state.active &&
                  close_state.error_reboot_attempts == 0,
              "clean low-power completion did not clear the budget")) {
    return 13;
  }

  MeasurementSessionState retained;
  BeginMeasurementSession(&retained, false, 0, 1000);
  retained.error_reboot_attempts = 2;
  if (!Expect(!FinishMeasurementSession(&retained, false) && retained.active &&
                  retained.paused && retained.error_reboot_attempts == 2,
              "unclean low-power completion did not retain the budget")) {
    return 14;
  }
  BeginMeasurementSession(&retained, false, 5000000000LL, 1005);
  if (!Expect(retained.active && !retained.paused &&
                  retained.error_reboot_attempts == 2,
              "next measurement did not inherit the retained budget")) {
    return 15;
  }

  MeasurementSessionState implicit;
  MeasurementTickResult implicit_first = Tick(&implicit, true, false, 1, 1);
  if (!Expect(implicit_first.implicit_session_started && implicit.active &&
                  implicit.implicit,
              "Driver-started-Normal Error did not create an implicit session")) {
    return 16;
  }

  std::cout << "measurement_session_policy_qc: OK\n";
  return 0;
}
