#include <iostream>

#include "point_cloud_outage_policy.h"

namespace {

using livox_ros::BeginPointCloudOutage;
using livox_ros::ExcludePointCloudOutage;
using livox_ros::ObservePointCloudPublished;
using livox_ros::PointCloudOutageState;
using livox_ros::PointCloudOutageTickInput;
using livox_ros::PointCloudOutageTickResult;
using livox_ros::TickPointCloudOutage;

const int64_t kSecond = 1000000000LL;

bool Expect(bool condition, const char *message) {
  if (!condition) {
    std::cerr << message << "\n";
  }
  return condition;
}

PointCloudOutageTickResult Tick(PointCloudOutageState *state, int64_t tenth,
                                bool expected, bool idle = false,
                                bool mode = false, bool group = false) {
  PointCloudOutageTickInput input;
  input.expected_stream = expected;
  input.intentional_idle = idle;
  input.planned_mode_transition = mode;
  input.planned_group_power_cycle = group;
  input.now_ns = tenth * kSecond / 10;
  input.now_wall_ns = (1000 * kSecond) + input.now_ns;
  return TickPointCloudOutage(state, input);
}

void Publish(PointCloudOutageState *state, int64_t tenth) {
  const int64_t now_ns = tenth * kSecond / 10;
  ObservePointCloudPublished(state, now_ns, (1000 * kSecond) + now_ns);
}

void SetLastPublish(PointCloudOutageState *state, int64_t tenth) {
  state->last_publish_ns = tenth * kSecond / 10;
  state->last_publish_wall_ns = (1000 * kSecond) + state->last_publish_ns;
}

}  // namespace

int main() {
  PointCloudOutageState state;
  Publish(&state, 10);  // t=1.0
  Tick(&state, 11, true);
  if (!Expect(state.stream_armed, "healthy stream did not arm")) {
    return 1;
  }

  SetLastPublish(&state, 98);  // last good batch t=9.8
  if (!Expect(BeginPointCloudOutage(&state, 100 * kSecond / 10,
                                    1010 * kSecond),
              "unplanned disconnect did not begin outage")) {
    return 2;
  }
  Publish(&state, 202);  // first returned batch t=20.2
  Publish(&state, 210);
  Publish(&state, 220);
  Publish(&state, 230);
  if (!Expect(!Tick(&state, 231, true).recovery_confirmed_edge,
              "return was accepted before three continuous seconds")) {
    return 3;
  }
  PointCloudOutageTickResult recovered = Tick(&state, 232, true);
  if (!Expect(recovered.recovery_confirmed_edge &&
                  recovered.duration_ns == 104 * kSecond / 10 &&
                  state.completed_outage_count == 1,
              "recovery duration did not end at first returned batch")) {
    return 4;
  }

  /** A return which breaks during the three-second confirmation belongs to the
   * original outage. The next sustained return replaces only the candidate. */
  SetLastPublish(&state, 250);
  Tick(&state, 251, true);
  BeginPointCloudOutage(&state, 260 * kSecond / 10, 1026 * kSecond);
  Publish(&state, 300);
  Tick(&state, 320, true);  // stale for 2.0s: reject first candidate
  if (!Expect(state.outage_active &&
                  state.recovery_first_publish_ns == 0,
              "broken confirmation incorrectly ended the outage")) {
    return 5;
  }
  Publish(&state, 350);
  Publish(&state, 360);
  Publish(&state, 370);
  Publish(&state, 380);
  PointCloudOutageTickResult second = Tick(&state, 380, true);
  if (!Expect(second.recovery_confirmed_edge &&
                  second.duration_ns == 100 * kSecond / 10 &&
                  state.completed_outage_count == 2,
              "second sustained return lost the original outage start")) {
    return 6;
  }

  /** An intentional sleep is excluded, including an unconfirmed live row. */
  SetLastPublish(&state, 400);
  Tick(&state, 401, true);
  BeginPointCloudOutage(&state, 410 * kSecond / 10, 1041 * kSecond);
  if (!Expect(ExcludePointCloudOutage(&state) && !state.outage_active &&
                  !state.stream_armed &&
                  state.completed_outage_count == 2,
              "planned low-power interval polluted outage history")) {
    return 7;
  }

  /** Planned shared power disarms a healthy companion, but preserves a fault
   * which was already active before the relay intent. */
  SetLastPublish(&state, 500);
  Tick(&state, 501, true);
  Tick(&state, 502, false, false, false, true);
  if (!Expect(!state.stream_armed && !state.outage_active,
              "healthy companion became a planned relay outage")) {
    return 8;
  }
  SetLastPublish(&state, 600);
  Tick(&state, 601, true);
  BeginPointCloudOutage(&state, 610 * kSecond / 10, 1061 * kSecond);
  Tick(&state, 620, false, false, false, true);
  if (!Expect(state.outage_active,
              "triggering outage was discarded by shared relay repair")) {
    return 9;
  }

  /** A long gap which returns entirely between timer ticks must still form an
   * episode, with the first returned batch captured immediately. */
  PointCloudOutageState gap;
  Publish(&gap, 10);
  Tick(&gap, 11, true);
  if (!Expect(ObservePointCloudPublished(&gap, 50 * kSecond / 10,
                                         1005 * kSecond),
              "long publish gap was not detected at return")) {
    return 10;
  }
  if (!Expect(gap.outage_active &&
                  gap.outage_started_ns == 10 * kSecond / 10 &&
                  gap.recovery_first_publish_ns == 50 * kSecond / 10,
              "gap episode timestamps are incorrect")) {
    return 11;
  }

  std::cout << "point_cloud_outage_policy_qc: OK\n";
  return 0;
}
