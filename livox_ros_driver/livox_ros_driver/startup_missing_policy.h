#pragma once

#include <cstdint>

namespace livox_ros_driver {

/** Persistent per-whitelist-member state for startup-only supervision.
 *
 * STARTUP_MISSING is deliberately a one-way classification gate: after a
 * physical lidar has ever been observed through a real SDK connection or a
 * fresh broadcast in this Driver process, later outages belong to the normal
 * runtime recovery paths and must never be re-labelled as startup absence.
 */
struct StartupMissingPolicyState {
  bool ever_observed = false;
  int64_t absent_since_ns = 0;
  int64_t absent_since_wall_s = 0;
  int64_t required_at_wall_s = 0;
  uint32_t episode_count = 0;
  bool active = false;
  bool request_emitted = false;
};

struct StartupMissingPolicyDecision {
  bool observed_now = false;
  bool planned_outage = false;
  bool newly_required = false;
  bool required = false;
};

inline StartupMissingPolicyDecision UpdateStartupMissingPolicy(
    StartupMissingPolicyState *state, bool observed_now,
    bool planned_group_power_cycle, int64_t now_ns, int64_t now_wall_s,
    int64_t grace_ns) {
  StartupMissingPolicyDecision decision;
  if (state == nullptr || now_ns <= 0 || now_wall_s <= 0 || grace_ns <= 0) {
    return decision;
  }

  if (observed_now) {
    state->ever_observed = true;
    state->absent_since_ns = 0;
    state->absent_since_wall_s = 0;
    state->required_at_wall_s = 0;
    state->active = false;
    state->request_emitted = false;
    decision.observed_now = true;
    return decision;
  }

  if (state->ever_observed) {
    return decision;
  }

  // A manager-authorized shared outage is never startup evidence. Restart the
  // grace window after it ends so a planned OFF interval cannot manufacture a
  // synthetic handle-255 fault for another group member.
  if (planned_group_power_cycle) {
    state->absent_since_ns = 0;
    state->absent_since_wall_s = 0;
    state->required_at_wall_s = 0;
    state->active = false;
    state->request_emitted = false;
    decision.planned_outage = true;
    return decision;
  }

  if (state->absent_since_ns == 0 || now_ns < state->absent_since_ns) {
    state->absent_since_ns = now_ns;
    state->absent_since_wall_s = now_wall_s;
    return decision;
  }
  if (now_ns - state->absent_since_ns < grace_ns) {
    return decision;
  }

  if (!state->active) {
    state->active = true;
    state->request_emitted = false;
    state->required_at_wall_s = now_wall_s;
    ++state->episode_count;
    decision.newly_required = true;
  }
  decision.required = true;
  return decision;
}

}  // namespace livox_ros_driver
