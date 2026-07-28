#ifndef LIVOX_ROS_DRIVER_WAKE_DROPOUT_POLICY_H_
#define LIVOX_ROS_DRIVER_WAKE_DROPOUT_POLICY_H_

#include <stdint.h>

namespace livox_ros {

/** Pure input for the wake-dropout safety gate.  Keeping the timing and
 *  identity predicate independent of ROS/SDK callbacks makes the dangerous
 *  hard-power decision directly testable. */
struct WakeDropoutPolicyInput {
  bool armed;
  bool identity_matches;
  bool generation_matches;
  bool connected;
  bool broadcast_fresh;
  uint64_t request_id;
  int64_t wake_started_ns;
  int64_t wake_deadline_ns;
  int64_t attributed_disconnect_ns;
  int64_t dropout_since_ns;

  WakeDropoutPolicyInput()
      : armed(false),
        identity_matches(false),
        generation_matches(false),
        connected(false),
        broadcast_fresh(false),
        request_id(0),
        wake_started_ns(0),
        wake_deadline_ns(0),
        attributed_disconnect_ns(0),
        dropout_since_ns(0) {}
};

inline bool WakeObservationArmedValid(const WakeDropoutPolicyInput &input) {
  return input.armed && input.identity_matches &&
         input.request_id != 0 && input.wake_started_ns != 0 &&
         input.wake_deadline_ns >= input.wake_started_ns;
}

inline bool WakeDropoutAttributionValid(
    const WakeDropoutPolicyInput &input) {
  return WakeObservationArmedValid(input) && input.generation_matches &&
         input.attributed_disconnect_ns != 0 &&
         input.attributed_disconnect_ns >= input.wake_started_ns &&
         input.attributed_disconnect_ns <= input.wake_deadline_ns;
}

/** A quiet observation can expire, but an outage which began inside the
 *  attribution window remains eligible for its bounded confirmation period. */
inline bool WakeObservationExpired(const WakeDropoutPolicyInput &input,
                                   int64_t now_ns) {
  return WakeObservationArmedValid(input) &&
         input.attributed_disconnect_ns == 0 &&
         now_ns > input.wake_deadline_ns;
}

inline bool WakeDropoutEscalationReady(const WakeDropoutPolicyInput &input,
                                       int64_t now_ns,
                                       int64_t confirm_ns) {
  return WakeDropoutAttributionValid(input) && !input.connected &&
         !input.broadcast_fresh && input.dropout_since_ns != 0 &&
         input.dropout_since_ns >= input.attributed_disconnect_ns &&
         now_ns >= input.dropout_since_ns &&
         now_ns - input.dropout_since_ns >= confirm_ns;
}

}  // namespace livox_ros

#endif  // LIVOX_ROS_DRIVER_WAKE_DROPOUT_POLICY_H_
