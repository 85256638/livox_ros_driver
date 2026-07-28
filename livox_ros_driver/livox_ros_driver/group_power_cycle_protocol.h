#ifndef LIVOX_ROS_DRIVER_GROUP_POWER_CYCLE_PROTOCOL_H_
#define LIVOX_ROS_DRIVER_GROUP_POWER_CYCLE_PROTOCOL_H_

#include <stdint.h>

#include <set>
#include <sstream>
#include <string>
#include <vector>

#include "rapidjson/document.h"
#include "recovery_event_json.h"

namespace livox_ros {

struct GroupPowerCycleIntent {
  bool cancel;
  std::string token;
  std::string group_id;
  uint64_t driver_instance;
  uint32_t valid_for_ms;
  std::vector<std::string> members;

  GroupPowerCycleIntent()
      : cancel(false), driver_instance(0), valid_for_ms(0) {}
};

inline bool IsAsciiAlnum(unsigned char ch) {
  return (ch >= '0' && ch <= '9') || (ch >= 'A' && ch <= 'Z') ||
         (ch >= 'a' && ch <= 'z');
}

inline bool IsSafeProtocolToken(const std::string &value, std::size_t maximum) {
  if (value.empty() || value.size() > maximum) {
    return false;
  }
  for (char value_char : value) {
    const unsigned char ch = static_cast<unsigned char>(value_char);
    if (!IsAsciiAlnum(ch) && ch != '.' && ch != '_' && ch != '-' && ch != ':') {
      return false;
    }
  }
  return true;
}

inline bool IsPowerGroupId(const std::string &value) {
  if (value.empty() || value.size() > 64 ||
      !IsAsciiAlnum(static_cast<unsigned char>(value[0]))) {
    return false;
  }
  for (char value_char : value) {
    const unsigned char ch = static_cast<unsigned char>(value_char);
    if (!IsAsciiAlnum(ch) && ch != '.' && ch != '_' && ch != '-') {
      return false;
    }
  }
  return true;
}

inline bool IsBroadcastCode(const std::string &value) {
  if (value.size() != 15) {
    return false;
  }
  for (char value_char : value) {
    if (!IsAsciiAlnum(static_cast<unsigned char>(value_char))) {
      return false;
    }
  }
  return true;
}

inline bool ParseGroupPowerCycleIntentJson(
    const std::string &json, GroupPowerCycleIntent *intent,
    std::string *error) {
  if (intent == nullptr) {
    if (error != nullptr) {
      *error = "intent output is null";
    }
    return false;
  }
  rapidjson::Document document;
  document.Parse(json.c_str());
  if (document.HasParseError() || !document.IsObject()) {
    if (error != nullptr) {
      *error = "intent is not valid JSON object";
    }
    return false;
  }
  if (!document.HasMember("schema_version") ||
      !document["schema_version"].IsUint() ||
      document["schema_version"].GetUint() != 1 ||
      !document.HasMember("type") || !document["type"].IsString()) {
    if (error != nullptr) {
      *error = "intent schema/type is invalid";
    }
    return false;
  }
  const std::string type = document["type"].GetString();
  if (type != "GROUP_POWER_CYCLE_INTENT" &&
      type != "GROUP_POWER_CYCLE_CANCEL") {
    if (error != nullptr) {
      *error = "intent type is unsupported";
    }
    return false;
  }
  if (!document.HasMember("token") || !document["token"].IsString() ||
      !document.HasMember("group_id") ||
      !document["group_id"].IsString() ||
      !document.HasMember("driver_instance") ||
      !document["driver_instance"].IsUint64() ||
      !document.HasMember("valid_for_ms") ||
      !document["valid_for_ms"].IsUint() ||
      !document.HasMember("members") || !document["members"].IsArray()) {
    if (error != nullptr) {
      *error = "intent required fields are invalid";
    }
    return false;
  }

  GroupPowerCycleIntent parsed;
  parsed.cancel = type == "GROUP_POWER_CYCLE_CANCEL";
  parsed.token = document["token"].GetString();
  parsed.group_id = document["group_id"].GetString();
  parsed.driver_instance = document["driver_instance"].GetUint64();
  parsed.valid_for_ms = document["valid_for_ms"].GetUint();
  if (!IsSafeProtocolToken(parsed.token, 128) ||
      !IsPowerGroupId(parsed.group_id) ||
      parsed.driver_instance == 0 || parsed.valid_for_ms < 3000 ||
      parsed.valid_for_ms > 60000 || document["members"].Size() != 4) {
    if (error != nullptr) {
      *error = "intent identity, lifetime, or member count is invalid";
    }
    return false;
  }
  std::set<std::string> unique_members;
  for (rapidjson::SizeType index = 0; index < document["members"].Size();
       ++index) {
    const rapidjson::Value &row = document["members"][index];
    if (!row.IsString() || !IsBroadcastCode(row.GetString()) ||
        !unique_members.insert(row.GetString()).second) {
      if (error != nullptr) {
        *error = "intent members must be four unique broadcast codes";
      }
      return false;
    }
    parsed.members.push_back(row.GetString());
  }
  *intent = parsed;
  if (error != nullptr) {
    error->clear();
  }
  return true;
}

inline std::string BuildGroupPowerCycleIntentAckJson(
    const GroupPowerCycleIntent &intent, bool accepted,
    const std::string &detail) {
  std::ostringstream json;
  json << "{\"schema_version\":1,\"type\":\"GROUP_POWER_CYCLE_INTENT_ACK\""
       << ",\"token\":\"" << RecoveryJsonEscape(intent.token.c_str()) << "\""
       << ",\"group_id\":\"" << RecoveryJsonEscape(intent.group_id.c_str())
       << "\""
       << ",\"driver_instance\":" << intent.driver_instance
       << ",\"accepted\":" << (accepted ? "true" : "false")
       << ",\"members\":[";
  for (std::size_t index = 0; index < intent.members.size(); ++index) {
    if (index != 0) {
      json << ',';
    }
    json << "\"" << RecoveryJsonEscape(intent.members[index].c_str()) << "\"";
  }
  json << "]"
       << ",\"detail\":\"" << RecoveryJsonEscape(detail.c_str()) << "\"}";
  return json.str();
}

}  // namespace livox_ros

#endif  // LIVOX_ROS_DRIVER_GROUP_POWER_CYCLE_PROTOCOL_H_
