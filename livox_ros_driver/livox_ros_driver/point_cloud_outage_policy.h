#ifndef LIVOX_ROS_DRIVER_POINT_CLOUD_OUTAGE_POLICY_H_
#define LIVOX_ROS_DRIVER_POINT_CLOUD_OUTAGE_POLICY_H_

#include <stdint.h>

namespace livox_ros {

/** Process-lifetime point-cloud outage history for one physical lidar.
 *
 * The start/end timestamps are publication-plane timestamps, not heartbeat
 * timestamps: an outage starts at the last successfully published point-cloud
 * batch and its recovery duration ends at the first batch of a later sustained
 * return.  Three continuous seconds are required before that return is
 * accepted, but those verification seconds are not added to the duration. */
struct PointCloudOutageState {
  bool stream_armed = false;
  bool outage_active = false;

  int64_t last_publish_ns = 0;
  int64_t last_publish_wall_ns = 0;

  int64_t outage_started_ns = 0;
  int64_t outage_started_wall_ns = 0;
  int64_t recovery_first_publish_ns = 0;
  int64_t recovery_first_publish_wall_ns = 0;

  uint32_t completed_outage_count = 0;
  bool last_recovery_valid = false;
  int64_t last_outage_started_wall_ns = 0;
  int64_t last_first_publish_wall_ns = 0;
  int64_t last_recovery_confirmed_wall_ns = 0;
  int64_t last_outage_duration_ns = 0;
};

struct PointCloudOutageTickInput {
  bool expected_stream = false;
  bool intentional_idle = false;
  bool planned_mode_transition = false;
  bool planned_group_power_cycle = false;
  int64_t now_ns = 0;
  int64_t now_wall_ns = 0;
};

struct PointCloudOutageTickResult {
  bool outage_started_edge = false;
  bool recovery_confirmed_edge = false;
  bool excluded_by_planned_state = false;
  int64_t outage_started_wall_ns = 0;
  int64_t first_publish_wall_ns = 0;
  int64_t recovery_confirmed_wall_ns = 0;
  int64_t duration_ns = 0;
};

inline void ClearPointCloudRecoveryCandidate(PointCloudOutageState *state) {
  if (state == nullptr) {
    return;
  }
  state->recovery_first_publish_ns = 0;
  state->recovery_first_publish_wall_ns = 0;
}

inline bool BeginPointCloudOutage(PointCloudOutageState *state,
                                  int64_t fallback_now_ns,
                                  int64_t fallback_wall_ns) {
  if (state == nullptr || !state->stream_armed || state->outage_active ||
      state->last_publish_ns <= 0) {
    return false;
  }
  state->outage_active = true;
  state->outage_started_ns = state->last_publish_ns;
  state->outage_started_wall_ns =
      state->last_publish_wall_ns > 0 ? state->last_publish_wall_ns
                                      : fallback_wall_ns;
  if (state->outage_started_ns > fallback_now_ns && fallback_now_ns > 0) {
    state->outage_started_ns = fallback_now_ns;
  }
  ClearPointCloudRecoveryCandidate(state);
  return true;
}

/** Exclude an intentional low-power/mode-transition interval. Completed
 * history is retained, while any not-yet-confirmed live episode is discarded. */
inline bool ExcludePointCloudOutage(PointCloudOutageState *state) {
  if (state == nullptr) {
    return false;
  }
  const bool discarded = state->outage_active;
  state->stream_armed = false;
  state->outage_active = false;
  state->outage_started_ns = 0;
  state->outage_started_wall_ns = 0;
  ClearPointCloudRecoveryCandidate(state);
  return discarded;
}

/** Called immediately after a real point-cloud message has been published.
 * A >= outage-confirm gap can therefore be recovered between two 1 Hz timer
 * ticks without disappearing from history. */
inline bool ObservePointCloudPublished(
    PointCloudOutageState *state, int64_t publish_ns, int64_t publish_wall_ns,
    int64_t outage_confirm_ns = 3000000000LL,
    int64_t continuous_publish_gap_ns = 1500000000LL) {
  if (state == nullptr || publish_ns <= 0) {
    return false;
  }

  const int64_t previous_publish_ns = state->last_publish_ns;
  const int64_t previous_publish_wall_ns = state->last_publish_wall_ns;
  bool started = false;
  if (!state->outage_active && state->stream_armed &&
      previous_publish_ns > 0 && publish_ns >= previous_publish_ns &&
      publish_ns - previous_publish_ns >= outage_confirm_ns) {
    state->outage_active = true;
    state->outage_started_ns = previous_publish_ns;
    state->outage_started_wall_ns =
        previous_publish_wall_ns > 0 ? previous_publish_wall_ns
                                     : publish_wall_ns;
    started = true;
  }

  if (state->outage_active &&
      (state->recovery_first_publish_ns == 0 ||
       (previous_publish_ns > 0 && publish_ns >= previous_publish_ns &&
        publish_ns - previous_publish_ns > continuous_publish_gap_ns))) {
    state->recovery_first_publish_ns = publish_ns;
    state->recovery_first_publish_wall_ns = publish_wall_ns;
  }
  state->last_publish_ns = publish_ns;
  state->last_publish_wall_ns = publish_wall_ns;
  return started;
}

inline PointCloudOutageTickResult TickPointCloudOutage(
    PointCloudOutageState *state, const PointCloudOutageTickInput &input,
    int64_t outage_confirm_ns = 3000000000LL,
    int64_t recovery_confirm_ns = 3000000000LL,
    int64_t continuous_publish_gap_ns = 1500000000LL) {
  PointCloudOutageTickResult result;
  if (state == nullptr || input.now_ns <= 0) {
    return result;
  }

  if (input.intentional_idle) {
    result.excluded_by_planned_state = ExcludePointCloudOutage(state);
    return result;
  }

  /** A healthy companion entering a planned mode/relay interruption must not
   * become an outage. An episode which was already active before a shared
   * relay repair is preserved so the triggering lidar's end-to-end recovery
   * time still includes that repair action. */
  if (input.planned_mode_transition || input.planned_group_power_cycle) {
    if (!state->outage_active) {
      state->stream_armed = false;
      ClearPointCloudRecoveryCandidate(state);
    }
    return result;
  }

  const bool publish_fresh =
      state->last_publish_ns > 0 && input.now_ns >= state->last_publish_ns &&
      input.now_ns - state->last_publish_ns <= continuous_publish_gap_ns;

  if (!state->outage_active) {
    if (input.expected_stream && publish_fresh) {
      state->stream_armed = true;
      return result;
    }
    if (!state->stream_armed || state->last_publish_ns <= 0) {
      return result;
    }

    const bool confirmed_silence =
        input.expected_stream && input.now_ns >= state->last_publish_ns &&
        input.now_ns - state->last_publish_ns >= outage_confirm_ns;
    const bool unexpected_stop = !input.expected_stream;
    if (confirmed_silence || unexpected_stop) {
      result.outage_started_edge = BeginPointCloudOutage(
          state, input.now_ns, input.now_wall_ns);
    }
  }

  if (!state->outage_active) {
    return result;
  }

  if (state->recovery_first_publish_ns != 0 && !publish_fresh) {
    ClearPointCloudRecoveryCandidate(state);
    return result;
  }
  if (!input.expected_stream || state->recovery_first_publish_ns == 0 ||
      input.now_ns < state->recovery_first_publish_ns ||
      input.now_ns - state->recovery_first_publish_ns < recovery_confirm_ns) {
    return result;
  }

  result.recovery_confirmed_edge = true;
  result.outage_started_wall_ns = state->outage_started_wall_ns;
  result.first_publish_wall_ns = state->recovery_first_publish_wall_ns;
  result.recovery_confirmed_wall_ns = input.now_wall_ns;
  result.duration_ns = state->recovery_first_publish_ns >=
                               state->outage_started_ns
                           ? state->recovery_first_publish_ns -
                                 state->outage_started_ns
                           : 0;

  if (state->completed_outage_count != UINT32_MAX) {
    ++state->completed_outage_count;
  }
  state->last_recovery_valid = true;
  state->last_outage_started_wall_ns = result.outage_started_wall_ns;
  state->last_first_publish_wall_ns = result.first_publish_wall_ns;
  state->last_recovery_confirmed_wall_ns =
      result.recovery_confirmed_wall_ns;
  state->last_outage_duration_ns = result.duration_ns;
  state->outage_active = false;
  state->outage_started_ns = 0;
  state->outage_started_wall_ns = 0;
  ClearPointCloudRecoveryCandidate(state);
  state->stream_armed = true;
  return result;
}

}  // namespace livox_ros

#endif  // LIVOX_ROS_DRIVER_POINT_CLOUD_OUTAGE_POLICY_H_
