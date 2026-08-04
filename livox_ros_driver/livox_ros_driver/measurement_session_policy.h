#ifndef LIVOX_ROS_DRIVER_MEASUREMENT_SESSION_POLICY_H_
#define LIVOX_ROS_DRIVER_MEASUREMENT_SESSION_POLICY_H_

#include <stdint.h>

namespace livox_ros {

enum MeasurementErrorAction {
  kMeasurementErrorNone = 0,
  kMeasurementErrorSoftReboot,
  kMeasurementErrorPowerCycle
};

/** Process-lifetime state for one physical lidar's measurement-session Error
 *  budget.  It deliberately survives SDK disconnect/reconnect generations;
 *  only a verified, healthy Normal -> low-power completion clears it. */
struct MeasurementSessionState {
  bool active = false;
  bool implicit = false;
  bool paused = false;
  uint64_t session_id = 0;
  int64_t started_ns = 0;
  int64_t started_wall_s = 0;

  uint8_t error_reboot_attempts = 0;
  bool error_active = false;
  uint32_t error_consecutive_ticks = 0;
  int64_t error_since_ns = 0;
  int64_t error_since_wall_s = 0;
  uint64_t last_error_reboot_generation = 0;
  int64_t last_error_reboot_ns = 0;
  int64_t last_error_action_try_ns = 0;

  uint32_t healthy_publish_ticks = 0;
  int64_t healthy_publish_since_ns = 0;
  bool recovery_confirmed = false;
  bool error_power_cycle_required = false;
  bool error_power_cycle_counted_this_session = false;
};

struct MeasurementTickInput {
  bool connected = false;
  bool in_error = false;
  bool normal_sampling_publishing = false;
  bool mode_transition_active = false;
  bool auto_recover = false;
  uint64_t connection_generation = 0;
  int64_t now_ns = 0;
  int64_t now_wall_s = 0;
};

struct MeasurementTickResult {
  MeasurementErrorAction action = kMeasurementErrorNone;
  bool implicit_session_started = false;
  bool recovery_confirmed_edge = false;
  bool power_cycle_cancelled = false;
};

inline void BeginMeasurementSession(MeasurementSessionState *state,
                                    bool implicit, int64_t now_ns,
                                    int64_t now_wall_s) {
  if (state == nullptr) {
    return;
  }
  if (!state->active) {
    state->active = true;
    state->implicit = implicit;
    state->paused = false;
    ++state->session_id;
    if (state->session_id == 0) {
      ++state->session_id;
    }
    state->started_ns = now_ns;
    state->started_wall_s = now_wall_s;
    state->error_reboot_attempts = 0;
    state->last_error_reboot_generation = 0;
    state->last_error_reboot_ns = 0;
    state->last_error_action_try_ns = 0;
    state->error_power_cycle_counted_this_session = false;
  } else if (!implicit) {
    /** A retained budget from an unclean previous measurement becomes an
     *  explicit session again, but its attempt count is intentionally kept. */
    state->implicit = false;
    state->paused = false;
  }
  state->healthy_publish_ticks = 0;
  state->healthy_publish_since_ns = 0;
  state->recovery_confirmed = false;
  state->error_active = false;
  state->error_consecutive_ticks = 0;
  state->error_since_ns = 0;
  state->error_since_wall_s = 0;
  state->error_power_cycle_required = false;
}

inline bool MeasurementSessionCloseEligible(
    const MeasurementSessionState &state) {
  return state.active && state.recovery_confirmed && !state.error_active &&
         !state.error_power_cycle_required;
}

/** Complete a confirmed PowerSaving/Standby request. Returns true only when
 *  this is the clean boundary which cleared the Error reboot budget. */
inline bool FinishMeasurementSession(MeasurementSessionState *state,
                                     bool close_was_eligible) {
  if (state == nullptr || !state->active) {
    return false;
  }
  if (!close_was_eligible || state->error_active ||
      state->error_power_cycle_required) {
    state->paused = true;
    state->healthy_publish_ticks = 0;
    state->healthy_publish_since_ns = 0;
    state->recovery_confirmed = false;
    state->error_active = false;
    state->error_consecutive_ticks = 0;
    state->error_since_ns = 0;
    state->error_since_wall_s = 0;
    return false;
  }

  const uint64_t completed_id = state->session_id;
  *state = MeasurementSessionState();
  state->session_id = completed_id;
  return true;
}

inline void MeasurementSessionDisconnected(MeasurementSessionState *state) {
  if (state == nullptr) {
    return;
  }
  state->error_active = false;
  state->error_consecutive_ticks = 0;
  state->error_since_ns = 0;
  state->error_since_wall_s = 0;
  state->healthy_publish_ticks = 0;
  state->healthy_publish_since_ns = 0;
  state->recovery_confirmed = false;
  state->error_power_cycle_required = false;
}

inline MeasurementTickResult TickMeasurementSession(
    MeasurementSessionState *state, const MeasurementTickInput &input,
    uint32_t error_confirm_ticks = 3,
    uint32_t recovery_confirm_ticks = 3,
    uint8_t max_soft_reboots = 3,
    int64_t same_generation_cooldown_ns = 40000000000LL,
    int64_t retry_interval_ns = 5000000000LL) {
  MeasurementTickResult result;
  if (state == nullptr) {
    return result;
  }
  if (!input.connected) {
    MeasurementSessionDisconnected(state);
    return result;
  }

  if (input.in_error && !state->active) {
    BeginMeasurementSession(state, true, input.now_ns, input.now_wall_s);
    result.implicit_session_started = true;
  }
  if (!state->active) {
    return result;
  }

  if (input.in_error) {
    state->paused = false;
    state->healthy_publish_ticks = 0;
    state->recovery_confirmed = false;
    if (!state->error_active) {
      state->error_active = true;
      state->error_consecutive_ticks = 1;
      state->error_since_ns = input.now_ns;
      state->error_since_wall_s = input.now_wall_s;
    } else if (state->error_consecutive_ticks != UINT32_MAX) {
      ++state->error_consecutive_ticks;
    }
    const int64_t error_confirm_ns =
        static_cast<int64_t>(error_confirm_ticks) * 1000000000LL;
    if (!input.auto_recover || state->error_power_cycle_required ||
        state->error_since_ns == 0 ||
        input.now_ns - state->error_since_ns < error_confirm_ns) {
      return result;
    }
    if (state->last_error_action_try_ns != 0 &&
        input.now_ns - state->last_error_action_try_ns < retry_interval_ns) {
      return result;
    }
    const bool same_generation_wait =
        state->last_error_reboot_generation != 0 &&
        state->last_error_reboot_generation == input.connection_generation &&
        state->last_error_reboot_ns != 0 &&
        input.now_ns - state->last_error_reboot_ns <
            same_generation_cooldown_ns;
    if (same_generation_wait) {
      return result;
    }
    state->last_error_action_try_ns = input.now_ns;
    result.action = state->error_reboot_attempts < max_soft_reboots
                        ? kMeasurementErrorSoftReboot
                        : kMeasurementErrorPowerCycle;
    return result;
  }

  if (state->error_power_cycle_required) {
    state->error_power_cycle_required = false;
    result.power_cycle_cancelled = true;
  }
  state->error_active = false;
  state->error_consecutive_ticks = 0;
  state->error_since_ns = 0;
  state->error_since_wall_s = 0;
  if (input.normal_sampling_publishing && !input.mode_transition_active) {
    if (state->healthy_publish_since_ns == 0) {
      state->healthy_publish_since_ns = input.now_ns;
    }
    if (state->healthy_publish_ticks != UINT32_MAX) {
      ++state->healthy_publish_ticks;
    }
    const int64_t recovery_confirm_ns =
        static_cast<int64_t>(recovery_confirm_ticks) * 1000000000LL;
    if (!state->recovery_confirmed &&
        input.now_ns - state->healthy_publish_since_ns >=
            recovery_confirm_ns) {
      state->recovery_confirmed = true;
      result.recovery_confirmed_edge = true;
    }
  } else {
    state->healthy_publish_ticks = 0;
    state->healthy_publish_since_ns = 0;
    state->recovery_confirmed = false;
  }
  return result;
}

inline void CommitMeasurementErrorReboot(MeasurementSessionState *state,
                                         uint64_t connection_generation,
                                         int64_t now_ns) {
  if (state == nullptr) {
    return;
  }
  if (state->error_reboot_attempts != UINT8_MAX) {
    ++state->error_reboot_attempts;
  }
  state->last_error_reboot_generation = connection_generation;
  state->last_error_reboot_ns = now_ns;
  state->error_active = false;
  state->error_consecutive_ticks = 0;
  state->error_since_ns = 0;
  state->error_since_wall_s = 0;
  state->healthy_publish_ticks = 0;
  state->healthy_publish_since_ns = 0;
  state->recovery_confirmed = false;
}

inline void CommitMeasurementErrorPowerCycle(
    MeasurementSessionState *state) {
  if (state == nullptr) {
    return;
  }
  state->error_power_cycle_required = true;
}

}  // namespace livox_ros

#endif  // LIVOX_ROS_DRIVER_MEASUREMENT_SESSION_POLICY_H_
