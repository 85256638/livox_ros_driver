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
      "POWER_CYCLE_REQUIRED", true, false, 1234, 2, 99);
  const std::string expected_state =
      "{\"schema_version\":1,\"type\":\"LIDAR_RECOVERY_STATE\","
      "\"timestamp\":100,\"driver_instance\":200,\"handle\":3,"
      "\"broadcast_code\":\"TESTLIDAR000001\",\"connected\":false,"
      "\"connect_state\":\"Off\",\"lidar_state\":\"?\","
      "\"handshake_state\":\"POWER_CYCLE_REQUIRED\","
      "\"broadcast_fresh\":true,\"publishing\":false,"
      "\"published_packets\":1234,\"power_cycle_required_count\":2,"
      "\"power_cycle_required_at\":99}";
  if (state != expected_state) {
    std::cerr << "state JSON mismatch:\n" << state << "\n";
    return 2;
  }

  const std::string request = BuildPowerCycleRequestJson(
      "event:1", 100, 99, 200, 3, "TESTLIDAR000001", true, 1, 2);
  const std::string expected_request =
      "{\"schema_version\":1,\"type\":\"POWER_CYCLE_REQUIRED\","
      "\"event_id\":\"event:1\",\"timestamp\":100,\"detected_at\":99,"
      "\"driver_instance\":200,\"handle\":3,"
      "\"broadcast_code\":\"TESTLIDAR000001\","
      "\"broadcast_fresh\":true,\"session_reset_attempts\":1,"
      "\"episode_count\":2}";
  if (request != expected_request) {
    std::cerr << "request JSON mismatch:\n" << request << "\n";
    return 3;
  }

  std::cout << "recovery_event_json_qc: OK\n";
  return 0;
}
