#ifndef LIVOX_ROS_DRIVER_RECOVERY_EVENT_JSON_H_
#define LIVOX_ROS_DRIVER_RECOVERY_EVENT_JSON_H_

#include <stdint.h>
#include <stdio.h>

#include <sstream>
#include <string>

namespace livox_ros {

/** Escape a small diagnostic string for JSON. Broadcast codes are normally
 *  alphanumeric, but keeping this generic prevents malformed messages if a
 *  future device family changes that contract. */
inline std::string RecoveryJsonEscape(const char *text) {
  std::ostringstream out;
  const unsigned char *p =
      reinterpret_cast<const unsigned char *>(text ? text : "");
  for (; *p; ++p) {
    switch (*p) {
      case '\\':
        out << "\\\\";
        break;
      case '"':
        out << "\\\"";
        break;
      case '\b':
        out << "\\b";
        break;
      case '\f':
        out << "\\f";
        break;
      case '\n':
        out << "\\n";
        break;
      case '\r':
        out << "\\r";
        break;
      case '\t':
        out << "\\t";
        break;
      default:
        if (*p < 0x20) {
          char escaped[7];
          snprintf(escaped, sizeof(escaped), "\\u%04x", *p);
          out << escaped;
        } else {
          out << static_cast<char>(*p);
        }
    }
  }
  return out.str();
}

inline std::string BuildLidarRecoveryStateJson(
    int64_t timestamp, uint64_t driver_instance, uint8_t handle,
    const char *broadcast_code, bool connected, const char *connect_state,
    const char *lidar_state, const char *handshake_state,
    const char *recovery_state, const char *recovery_reason,
    const char *wake_state, uint64_t wake_request_id,
    uint64_t wake_connection_generation,
    uint64_t wake_dropout_generation,
    int64_t wake_started_at, int64_t wake_dropout_at,
    int64_t wake_silence_at,
    bool broadcast_fresh, bool publishing, uint64_t published_packets,
    uint32_t power_cycle_required_count,
    int64_t power_cycle_required_at,
    const char *normal_state = "IDLE",
    uint64_t normal_connection_generation = 0,
    uint64_t normal_dropout_generation = 0,
    int64_t normal_healthy_since_at = 0,
    int64_t normal_dropout_at = 0,
    int64_t normal_silence_at = 0,
    const char *startup_state = "IDLE",
    int64_t startup_missing_since = 0) {
  std::ostringstream json;
  json << "{\"schema_version\":1,\"type\":\"LIDAR_RECOVERY_STATE\""
       << ",\"timestamp\":" << timestamp
       << ",\"driver_instance\":" << driver_instance
       << ",\"handle\":" << static_cast<unsigned>(handle)
       << ",\"broadcast_code\":\"" << RecoveryJsonEscape(broadcast_code)
       << "\""
       << ",\"connected\":" << (connected ? "true" : "false")
       << ",\"connect_state\":\"" << RecoveryJsonEscape(connect_state)
       << "\""
       << ",\"lidar_state\":\"" << RecoveryJsonEscape(lidar_state) << "\""
       << ",\"handshake_state\":\"" << RecoveryJsonEscape(handshake_state)
       << "\""
       << ",\"recovery_state\":\"" << RecoveryJsonEscape(recovery_state)
       << "\""
       << ",\"recovery_reason\":\"" << RecoveryJsonEscape(recovery_reason)
       << "\""
       << ",\"wake_state\":\"" << RecoveryJsonEscape(wake_state) << "\""
       << ",\"wake_request_id\":" << wake_request_id
       << ",\"wake_connection_generation\":"
       << wake_connection_generation
       << ",\"wake_dropout_generation\":" << wake_dropout_generation
       << ",\"wake_started_at\":" << wake_started_at
       << ",\"wake_dropout_at\":" << wake_dropout_at
       << ",\"wake_silence_at\":" << wake_silence_at
       << ",\"normal_state\":\"" << RecoveryJsonEscape(normal_state)
       << "\""
       << ",\"normal_connection_generation\":"
       << normal_connection_generation
       << ",\"normal_dropout_generation\":" << normal_dropout_generation
       << ",\"normal_healthy_since_at\":" << normal_healthy_since_at
       << ",\"normal_dropout_at\":" << normal_dropout_at
       << ",\"normal_silence_at\":" << normal_silence_at
       << ",\"startup_state\":\"" << RecoveryJsonEscape(startup_state)
       << "\""
       << ",\"startup_missing_since\":" << startup_missing_since
       << ",\"broadcast_fresh\":"
       << (broadcast_fresh ? "true" : "false")
       << ",\"publishing\":" << (publishing ? "true" : "false")
       << ",\"published_packets\":" << published_packets
       << ",\"power_cycle_required_count\":"
       << power_cycle_required_count
       << ",\"power_cycle_required_at\":" << power_cycle_required_at << "}";
  return json.str();
}

inline std::string BuildPowerCycleRequestJson(
    const char *event_id, int64_t timestamp, int64_t detected_at,
    uint64_t driver_instance, uint8_t handle, const char *broadcast_code,
    const char *recovery_reason, bool broadcast_fresh,
    uint64_t wake_request_id, uint64_t wake_connection_generation,
    uint64_t wake_dropout_generation, int64_t wake_started_at,
    int64_t wake_dropout_at, int64_t wake_silence_at,
    uint8_t session_reset_attempts,
    uint32_t episode_count,
    uint64_t normal_connection_generation = 0,
    uint64_t normal_dropout_generation = 0,
    int64_t normal_healthy_since_at = 0,
    int64_t normal_dropout_at = 0,
    int64_t normal_silence_at = 0,
    int64_t startup_missing_since = 0) {
  std::ostringstream json;
  json << "{\"schema_version\":1,\"type\":\"POWER_CYCLE_REQUIRED\""
       << ",\"event_id\":\"" << RecoveryJsonEscape(event_id) << "\""
       << ",\"timestamp\":" << timestamp
       << ",\"detected_at\":" << detected_at
       << ",\"driver_instance\":" << driver_instance
       << ",\"handle\":" << static_cast<unsigned>(handle)
       << ",\"broadcast_code\":\"" << RecoveryJsonEscape(broadcast_code)
       << "\""
       << ",\"recovery_reason\":\"" << RecoveryJsonEscape(recovery_reason)
       << "\""
       << ",\"broadcast_fresh\":"
       << (broadcast_fresh ? "true" : "false")
       << ",\"wake_request_id\":" << wake_request_id
       << ",\"wake_connection_generation\":"
       << wake_connection_generation
       << ",\"wake_dropout_generation\":" << wake_dropout_generation
       << ",\"wake_started_at\":" << wake_started_at
       << ",\"wake_dropout_at\":" << wake_dropout_at
       << ",\"wake_silence_at\":" << wake_silence_at
       << ",\"normal_connection_generation\":"
       << normal_connection_generation
       << ",\"normal_dropout_generation\":" << normal_dropout_generation
       << ",\"normal_healthy_since_at\":" << normal_healthy_since_at
       << ",\"normal_dropout_at\":" << normal_dropout_at
       << ",\"normal_silence_at\":" << normal_silence_at
       << ",\"startup_missing_since\":" << startup_missing_since
       << ",\"session_reset_attempts\":"
       << static_cast<unsigned>(session_reset_attempts)
       << ",\"episode_count\":" << episode_count << "}";
  return json.str();
}

}  // namespace livox_ros

#endif  // LIVOX_ROS_DRIVER_RECOVERY_EVENT_JSON_H_
