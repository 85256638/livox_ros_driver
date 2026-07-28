#ifndef LIVOX_ROS_DRIVER_NORMAL_DROPOUT_POLICY_H_
#define LIVOX_ROS_DRIVER_NORMAL_DROPOUT_POLICY_H_

#include <stdint.h>

namespace livox_ros {

/** Pure timing/identity predicates for a previously healthy Normal lidar
 *  which loses both its SDK connection and broadcasts. */
struct NormalDropoutPolicyInput {
  bool armed;
  bool identity_matches;
  bool generation_matches;
  bool connected;
  bool broadcast_fresh;
  int64_t healthy_since_ns;
  int64_t attributed_disconnect_ns;
  int64_t silence_since_ns;

  NormalDropoutPolicyInput()
      : armed(false),
        identity_matches(false),
        generation_matches(false),
        connected(false),
        broadcast_fresh(false),
        healthy_since_ns(0),
        attributed_disconnect_ns(0),
        silence_since_ns(0) {}
};

inline bool NormalDropoutArmMature(const NormalDropoutPolicyInput &input,
                                   int64_t now_ns,
                                   int64_t healthy_required_ns) {
  return input.armed && input.identity_matches && input.generation_matches &&
         input.healthy_since_ns > 0 && now_ns >= input.healthy_since_ns &&
         now_ns - input.healthy_since_ns >= healthy_required_ns;
}

inline bool NormalDropoutAttributionValid(
    const NormalDropoutPolicyInput &input) {
  return input.armed && input.identity_matches && input.generation_matches &&
         input.healthy_since_ns > 0 && input.attributed_disconnect_ns >=
                                           input.healthy_since_ns;
}

inline bool NormalDropoutEscalationReady(
    const NormalDropoutPolicyInput &input, int64_t now_ns,
    int64_t silence_required_ns) {
  return NormalDropoutAttributionValid(input) && !input.connected &&
         !input.broadcast_fresh && input.silence_since_ns > 0 &&
         now_ns >= input.silence_since_ns &&
         now_ns - input.silence_since_ns >= silence_required_ns;
}

}  // namespace livox_ros

#endif  // LIVOX_ROS_DRIVER_NORMAL_DROPOUT_POLICY_H_
