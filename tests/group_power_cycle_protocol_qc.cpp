#include <cstdlib>
#include <iostream>
#include <string>

#include "group_power_cycle_protocol.h"
#include "rapidjson/document.h"

namespace {

void Check(bool condition, const char *message) {
  if (!condition) {
    std::cerr << "group_power_cycle_protocol_qc: " << message << "\n";
    std::exit(1);
  }
}

}  // namespace

int main() {
  const std::string valid =
      "{\"schema_version\":1,\"type\":\"GROUP_POWER_CYCLE_INTENT\","
      "\"token\":\"abc-123\",\"group_id\":\"pit-4\","
      "\"driver_instance\":42,\"valid_for_ms\":15000,\"members\":["
      "\"TESTLIDAR000001\",\"TESTLIDAR000002\","
      "\"TESTLIDAR000003\",\"TESTLIDAR000004\"]}";
  livox_ros::GroupPowerCycleIntent intent;
  std::string error;
  Check(livox_ros::ParseGroupPowerCycleIntentJson(valid, &intent, &error),
        error.c_str());
  Check(!intent.cancel && intent.driver_instance == 42 &&
            intent.members.size() == 4,
        "valid intent fields were not retained");

  std::string duplicate = valid;
  const std::string needle = "TESTLIDAR000004";
  duplicate.replace(duplicate.find(needle), needle.size(),
                    "TESTLIDAR000003");
  Check(!livox_ros::ParseGroupPowerCycleIntentJson(duplicate, &intent, &error),
        "duplicate members were accepted");

  std::string wrong_instance = valid;
  wrong_instance.replace(wrong_instance.find("\"driver_instance\":42"), 20,
                         "\"driver_instance\":0");
  Check(!livox_ros::ParseGroupPowerCycleIntentJson(wrong_instance, &intent,
                                                   &error),
        "zero driver instance was accepted");

  std::string invalid_group = valid;
  invalid_group.replace(invalid_group.find("pit-4"), 5, ":pit4");
  Check(!livox_ros::ParseGroupPowerCycleIntentJson(invalid_group, &intent,
                                                   &error),
        "invalid power-group identity was accepted");

  Check(livox_ros::ParseGroupPowerCycleIntentJson(valid, &intent, &error),
        "valid intent no longer parses");
  const std::string ack = livox_ros::BuildGroupPowerCycleIntentAckJson(
      intent, true, "armed before OFF");
  rapidjson::Document document;
  document.Parse(ack.c_str());
  Check(!document.HasParseError() && document.IsObject(),
        "ACK is not valid JSON");
  Check(document["accepted"].IsBool() && document["accepted"].GetBool(),
        "ACK accepted flag is invalid");
  Check(document["members"].Size() == 4,
        "ACK member identity is incomplete");

  std::cout << "group_power_cycle_protocol_qc: OK\n";
  return 0;
}
