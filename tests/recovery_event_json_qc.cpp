#include "../livox_ros_driver/livox_ros_driver/recovery_event_json.h"

#include <iostream>
#include <string>

int main() {
  using livox_ros::BuildLidarRecoveryStateJson;
  using livox_ros::BuildPowerCycleRequestJson;
  using livox_ros::RecoveryJsonEscape;

  if (RecoveryJsonEscape("a\"b\\c\n") != "a\\\"b\\\\c\\n") {
    std::cerr << "JSON escaping mismatch\n";
    return 1;
  }

  const std::string state = BuildLidarRecoveryStateJson(
      100, 200, 3, "TESTLIDAR000001", false, "Off", "?",
      "POWER_CYCLE_REQUIRED", "POWER_CYCLE_REQUIRED", "HANDSHAKE_STUCK",
      "IDLE", 0, 0, 0, 0, 0, 0, true, false, 1234, 2, 99);
  const std::string expected_state =
      "{\"schema_version\":1,\"type\":\"LIDAR_RECOVERY_STATE\","
      "\"timestamp\":100,\"driver_instance\":200,\"handle\":3,"
      "\"broadcast_code\":\"TESTLIDAR000001\",\"connected\":false,"
      "\"connect_state\":\"Off\",\"lidar_state\":\"?\","
      "\"handshake_state\":\"POWER_CYCLE_REQUIRED\","
      "\"recovery_state\":\"POWER_CYCLE_REQUIRED\","
      "\"recovery_reason\":\"HANDSHAKE_STUCK\",\"wake_state\":\"IDLE\","
      "\"wake_request_id\":0,\"wake_connection_generation\":0,"
      "\"wake_dropout_generation\":0,\"wake_started_at\":0,"
      "\"wake_dropout_at\":0,\"wake_silence_at\":0,"
      "\"broadcast_fresh\":true,\"publishing\":false,"
      "\"published_packets\":1234,\"power_cycle_required_count\":2,"
      "\"power_cycle_required_at\":99}";
  if (state != expected_state) {
    std::cerr << "state JSON mismatch:\n" << state << "\n";
    return 2;
  }

  const std::string request = BuildPowerCycleRequestJson(
      "event:1", 100, 99, 200, 3, "TESTLIDAR000001", "HANDSHAKE_STUCK",
      true, 0, 0, 0, 0, 0, 0, 1, 2);
  const std::string expected_request =
      "{\"schema_version\":1,\"type\":\"POWER_CYCLE_REQUIRED\","
      "\"event_id\":\"event:1\",\"timestamp\":100,\"detected_at\":99,"
      "\"driver_instance\":200,\"handle\":3,"
      "\"broadcast_code\":\"TESTLIDAR000001\","
      "\"recovery_reason\":\"HANDSHAKE_STUCK\","
      "\"broadcast_fresh\":true,\"wake_request_id\":0,"
      "\"wake_connection_generation\":0,"
      "\"wake_dropout_generation\":0,\"wake_started_at\":0,"
      "\"wake_dropout_at\":0,\"wake_silence_at\":0,"
      "\"session_reset_attempts\":1,"
      "\"episode_count\":2}";
  if (request != expected_request) {
    std::cerr << "request JSON mismatch:\n" << request << "\n";
    return 3;
  }

  const std::string wake_state = BuildLidarRecoveryStateJson(
      110, 201, 1, "TESTLIDAR000002", false, "Off", "?", "IDLE",
      "POWER_CYCLE_REQUIRED", "WAKE_DROPOUT", "POWER_CYCLE_REQUIRED", 77,
      12, 12, 101, 105, 105, false, false, 0, 4, 109);
  if (wake_state.find("\"recovery_reason\":\"WAKE_DROPOUT\"") ==
          std::string::npos ||
      wake_state.find("\"wake_request_id\":77") == std::string::npos ||
      wake_state.find("\"wake_connection_generation\":12") ==
          std::string::npos ||
      wake_state.find("\"wake_dropout_generation\":12") ==
          std::string::npos ||
      wake_state.find("\"broadcast_fresh\":false") == std::string::npos) {
    std::cerr << "wake state JSON mismatch:\n" << wake_state << "\n";
    return 4;
  }

  const std::string wake_request = BuildPowerCycleRequestJson(
      "wake:event", 110, 109, 201, 1, "TESTLIDAR000002", "WAKE_DROPOUT",
      false, 77, 12, 12, 101, 105, 105, 0, 4);
  if (wake_request.find("\"recovery_reason\":\"WAKE_DROPOUT\"") ==
          std::string::npos ||
      wake_request.find("\"wake_dropout_at\":105") == std::string::npos ||
      wake_request.find("\"wake_silence_at\":105") ==
          std::string::npos ||
      wake_request.find("\"wake_dropout_generation\":12") ==
          std::string::npos ||
      wake_request.find("\"broadcast_fresh\":false") == std::string::npos) {
    std::cerr << "wake request JSON mismatch:\n" << wake_request << "\n";
    return 5;
  }

  std::cout << "recovery_event_json_qc: OK\n";
  return 0;
}
