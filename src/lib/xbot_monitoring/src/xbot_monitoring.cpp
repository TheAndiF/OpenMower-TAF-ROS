//
// Created by Clemens Elflein on 22.11.22.
// Copyright (c) 2022 Clemens Elflein. All rights reserved.
//
#include <filesystem>
#include <algorithm>
#include <cstdio>
#include <fstream>
#include <chrono>
#include <cmath>
#include <ctime>
#include <cctype>
#include <iomanip>
#include <limits>
#include <sstream>
#include <csignal>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include "ros/ros.h"
#include <memory>
#include <boost/regex.hpp>
#include "xbot_msgs/SensorInfo.h"
#include "xbot_msgs/SensorDataString.h"
#include "xbot_msgs/SensorDataDouble.h"
#include "xbot_msgs/RobotState.h"
#include "xbot_msgs/GnssSatelliteArray.h"
#include "xbot_msgs/AbsolutePose.h"
#include <mqtt/async_client.h>
#include <nlohmann/json.hpp>
#include <vector>
#include <set>
#include <mutex>
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include "xbot_msgs/RegisterActionsSrv.h"
#include "xbot_msgs/ActionInfo.h"
#include "xbot_msgs/MapOverlay.h"
#include "xbot_rpc/RpcError.h"
#include "xbot_rpc/RpcRequest.h"
#include "xbot_rpc/RpcResponse.h"
#include "xbot_rpc/constants.h"
#include "xbot_rpc/provider.h"
#include "xbot_rpc/RegisterMethodsSrv.h"
#include "capabilities.h"
#include "open_mower/settings_persistence.h"
#include "robot_localization/navsat_conversions.h"

using json = nlohmann::ordered_json;


static std::string trim_settings_string(const std::string &value) {
    std::size_t begin = 0;
    while (begin < value.size() && std::isspace(static_cast<unsigned char>(value[begin]))) ++begin;
    std::size_t end = value.size();
    while (end > begin && std::isspace(static_cast<unsigned char>(value[end - 1]))) --end;
    return value.substr(begin, end - begin);
}

static bool validate_group_metadata_value(const json &value, std::string &group, std::string &reason) {
    if (!value.is_string()) {
        reason = "group must be a string";
        return false;
    }
    group = trim_settings_string(value.get<std::string>());
    if (group.empty()) {
        reason = "group must not be empty";
        return false;
    }
    if (group.size() > 80) {
        reason = "group must not be longer than 80 characters";
        return false;
    }
    return true;
}


void publish_capabilities();
void publish_sensor_metadata();
void publish_sensor_infos_validation(const json &validation);
void handle_sensor_infos_persistent_payload(const std::string &payload);
void publish_gps_state_settings();
void publish_gps_state_validation(const json &validation);
void handle_gps_state_set_payload(const std::string &payload_text, bool persistent);
void publish_latest_gps_state_payloads(bool force = false);
void gps_state_ll_gps_pose_callback(const xbot_msgs::AbsolutePose::ConstPtr &msg);
void gps_state_xb_pose_callback(const xbot_msgs::AbsolutePose::ConstPtr &msg);
void publish_map();
void publish_map_validation(const json &validation);
void publish_settings_validation(const std::string &settings_namespace, const json &validation);
json validate_map_payload_for_mqtt(const json &payload);
void publish_map_overlay();
void publish_mowing_progress(const std::string &payload);
void publish_mowing_progress_status(const std::string &payload);
void publish_timetable();
void maybe_publish_timetable(bool force = false);
void publish_timetable_validation(const json &validation);
void publish_statustransition_log(std::size_t requested_limit = 0);
void publish_actions();
void publish_version();
void publish_params();
void publish_ll_power_status_request();
void try_publish(const std::string &topic, const std::string &data, bool retain = false);
std::string utc_timestamp_iso8601(const std::chrono::system_clock::time_point &time_point);
void rpc_request_callback(const std::string &payload);


struct WorldPoseConversionResult {
    bool valid = false;
    double latitude = 0.0;
    double longitude = 0.0;
    double altitude = 0.0;
    std::string reason;
};

struct GpsDatumCache {
    bool loaded = false;
    bool attempted = false;
    double datum_lat = 0.0;
    double datum_long = 0.0;
    double datum_height = 0.0;
    double datum_northing = 0.0;
    double datum_easting = 0.0;
    std::string datum_zone;
    ros::WallTime last_attempt;
};

std::mutex gps_datum_cache_mutex;
GpsDatumCache gps_datum_cache;

bool load_gps_datum_for_world_pose(GpsDatumCache &cache) {
    const ros::WallTime now = ros::WallTime::now();
    if (cache.loaded) {
        return true;
    }
    if (cache.attempted && (now - cache.last_attempt).toSec() < 5.0) {
        return false;
    }

    cache.attempted = true;
    cache.last_attempt = now;

    double datum_lat = 0.0;
    double datum_long = 0.0;
    double datum_height = 0.0;
    const bool has_datum = ros::param::get("/ll/services/gps/datum_lat", datum_lat) &&
                           ros::param::get("/ll/services/gps/datum_long", datum_long) &&
                           ros::param::get("/ll/services/gps/datum_height", datum_height);

    if (!has_datum) {
        ROS_WARN_THROTTLE(30.0, "Cannot publish robot_state/world_pose: GPS datum parameters are missing");
        return false;
    }

    if (!std::isfinite(datum_lat) || !std::isfinite(datum_long) || !std::isfinite(datum_height)) {
        ROS_WARN_THROTTLE(30.0, "Cannot publish robot_state/world_pose: GPS datum parameters are not finite");
        return false;
    }

    RobotLocalization::NavsatConversions::LLtoUTM(
        datum_lat, datum_long, cache.datum_northing, cache.datum_easting, cache.datum_zone);
    cache.datum_lat = datum_lat;
    cache.datum_long = datum_long;
    cache.datum_height = datum_height;
    cache.loaded = true;

    ROS_INFO_STREAM("robot_state/world_pose enabled using datum " << datum_lat << ", " << datum_long
                    << ", height " << datum_height << ", UTM zone " << cache.datum_zone);
    return true;
}

WorldPoseConversionResult convert_robot_pose_to_world_pose(const xbot_msgs::AbsolutePose &robot_pose) {
    WorldPoseConversionResult result;

    const double x = robot_pose.pose.pose.position.x;
    const double y = robot_pose.pose.pose.position.y;
    const double z = robot_pose.pose.pose.position.z;
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        result.reason = "robot_pose_not_finite";
        return result;
    }

    std::lock_guard<std::mutex> lk(gps_datum_cache_mutex);
    if (!load_gps_datum_for_world_pose(gps_datum_cache)) {
        result.reason = "gps_datum_unavailable";
        return result;
    }

    const double northing = gps_datum_cache.datum_northing + y;
    const double easting = gps_datum_cache.datum_easting + x;
    RobotLocalization::NavsatConversions::UTMtoLL(
        northing, easting, gps_datum_cache.datum_zone, result.latitude, result.longitude);
    result.altitude = gps_datum_cache.datum_height + z;
    result.valid = std::isfinite(result.latitude) && std::isfinite(result.longitude) && std::isfinite(result.altitude);
    if (!result.valid) {
        result.reason = "conversion_failed";
    }
    return result;
}

// Stores registered actions (prefix to vector<action>)
std::map<std::string, std::vector<xbot_msgs::ActionInfo>> registered_actions;
std::mutex registered_actions_mutex;

// Stores registered RPC methods
std::map<std::string, std::vector<std::string>> registered_methods;
std::mutex registered_methods_mutex;

std::map<std::string, xbot_msgs::SensorInfo> found_sensors;
std::mutex found_sensors_mutex;

ros::NodeHandle *n;

// The MQTT Client
std::shared_ptr<mqtt::async_client> client_;
std::shared_ptr<mqtt::async_client> client_external_;


// Publisher for cmd_vel and commands
ros::Publisher cmd_vel_pub;
ros::Publisher action_pub;
ros::Publisher rpc_request_pub;
ros::Publisher mow_load_factor_set_enabled_pub;
ros::Publisher mow_load_factor_set_min_factor_pub;
ros::Publisher mow_load_factor_set_current_start_pub;
ros::Publisher mow_load_factor_set_current_end_pub;
ros::Publisher mow_load_factor_set_persistent_enabled_pub;
ros::Publisher mow_load_factor_set_persistent_min_factor_pub;
ros::Publisher mow_load_factor_set_persistent_current_start_pub;
ros::Publisher mow_load_factor_set_persistent_current_end_pub;
ros::Publisher mow_load_factor_renew_pub;
ros::Publisher mower_logic_settings_set_session_json_pub;
ros::Publisher mower_logic_settings_set_persistent_json_pub;
ros::Publisher mower_logic_settings_renew_pub;
ros::Publisher mower_logic_satellite_logging_control_pub;
ros::Publisher mower_logic_satellite_logging_renew_pub;
ros::Publisher map_mowing_progress_renew_pub;
ros::Publisher ll_power_set_battery_critical_voltage_pub;

std::mutex load_factor_state_mutex;
double load_factor_computed_snapshot = 1.0;
double load_factor_effective_snapshot = 1.0;
ros::Publisher ll_power_set_battery_empty_voltage_pub;
ros::Publisher ll_power_set_battery_full_voltage_pub;
ros::Publisher ll_power_set_battery_critical_high_voltage_pub;
ros::Publisher ll_power_set_charge_critical_high_voltage_pub;
ros::Publisher ll_power_set_charge_critical_high_current_pub;
ros::Publisher ll_power_set_persistent_battery_critical_voltage_pub;
ros::Publisher ll_power_set_persistent_battery_empty_voltage_pub;
ros::Publisher ll_power_set_persistent_battery_full_voltage_pub;
ros::Publisher ll_power_set_persistent_battery_critical_high_voltage_pub;
ros::Publisher ll_power_set_persistent_charge_critical_high_voltage_pub;
ros::Publisher ll_power_set_persistent_charge_critical_high_current_pub;
ros::Publisher ftc_settings_set_speed_fast_pub;
ros::Publisher ftc_settings_set_speed_slow_pub;
ros::Publisher ftc_settings_set_speed_fast_threshold_pub;
ros::Publisher ftc_settings_set_persistent_speed_fast_pub;
ros::Publisher ftc_settings_set_persistent_speed_slow_pub;
ros::Publisher ftc_settings_set_persistent_speed_fast_threshold_pub;
ros::Publisher ll_power_renew_pub;

// properties for external mqtt
bool external_mqtt_enable = false;
std::string external_mqtt_username = "";
std::string external_mqtt_password = "";
std::string external_mqtt_hostname = "";
std::string external_mqtt_topic_prefix = "";
std::string external_mqtt_port = "";
std::string version_string = "";

class MqttCallback : public mqtt::callback {

    void connected(const mqtt::string &string) override {
        ROS_INFO_STREAM("MQTT Connected");
        publish_capabilities();
        publish_sensor_metadata();
        publish_map();
        publish_map_overlay();
        publish_timetable();
        publish_statustransition_log();
        publish_actions();
        publish_version();
        publish_params();
        publish_gps_state_settings();
        publish_latest_gps_state_payloads(true);

        // BEGIN: Deprecated code (1/2)
        // Earlier implementations subscribed to "/action" and "prefix//action" topics, we do it to not break stuff as well.
        client_->subscribe(this->mqtt_topic_prefix + "/teleop", 0);
        client_->subscribe(this->mqtt_topic_prefix + "/command", 0);
        client_->subscribe(this->mqtt_topic_prefix + "/action", 0);
        // END: Deprecated code (1/2)

        client_->subscribe(this->mqtt_topic_prefix + "teleop", 0);
        client_->subscribe(this->mqtt_topic_prefix + "command", 0);
        client_->subscribe(this->mqtt_topic_prefix + "action", 0);
        client_->subscribe(this->mqtt_topic_prefix + "rpc/request", 0);
        client_->subscribe(this->mqtt_topic_prefix + "timetable/set/json", 0);
        client_->subscribe(this->mqtt_topic_prefix + "timetable/set/bson", 0);
        client_->subscribe(this->mqtt_topic_prefix + "timetable/set/renew/json", 0);
        client_->subscribe(this->mqtt_topic_prefix + "timetable/set/renew/bson", 0);
        client_->subscribe(this->mqtt_topic_prefix + "timetable/set/suspension/json", 0);
        client_->subscribe(this->mqtt_topic_prefix + "timetable/set/suspension/bson", 0);
        client_->subscribe(this->mqtt_topic_prefix + "map/set/renew/json", 0);
        client_->subscribe(this->mqtt_topic_prefix + "map/set/json", 0);
        client_->subscribe(this->mqtt_topic_prefix + "map/mowing_progress/set/renew/json", 0);
        client_->subscribe(this->mqtt_topic_prefix + "statustransition_log/set/renew/json", 0);
        client_->subscribe(this->mqtt_topic_prefix + "settings/mower_logic/satellite_logging/set/control/json", 0);
        client_->subscribe(this->mqtt_topic_prefix + "settings/mower_logic/satellite_logging/set/renew/json", 0);
        // settings/mow_load_factor is deprecated and is no longer published.
        // Load regulation settings are exposed exclusively through settings/mower_logic
        // to avoid duplicate UI groups.
        client_->subscribe(this->mqtt_topic_prefix + "settings/mower_logic/set/session/json", 0);
        client_->subscribe(this->mqtt_topic_prefix + "settings/mower_logic/set/persistent/json", 0);
        client_->subscribe(this->mqtt_topic_prefix + "settings/mower_logic/set/renew/json", 0);
        client_->subscribe(this->mqtt_topic_prefix + "sensors/settings/set/renew/json", 0);
        client_->subscribe(this->mqtt_topic_prefix + "sensors/settings/set/persistent/json", 0);
        client_->subscribe(this->mqtt_topic_prefix + "gps_state/settings/set/renew/json", 0);
        client_->subscribe(this->mqtt_topic_prefix + "gps_state/settings/set/session/json", 0);
        client_->subscribe(this->mqtt_topic_prefix + "gps_state/settings/set/persistent/json", 0);
        client_->subscribe(this->mqtt_topic_prefix + "settings/ll_board/set/session/json", 0);
        client_->subscribe(this->mqtt_topic_prefix + "settings/ll_board/set/persistent/json", 0);
        client_->subscribe(this->mqtt_topic_prefix + "settings/ll_board/set/renew/json", 0);
    }

public:
    void setMqttClient(std::shared_ptr<mqtt::async_client> c, const std::string &mqtt_topic_prefix) {
        this->client_ = std::move(c);
        this->mqtt_topic_prefix = mqtt_topic_prefix;
    }
    void message_arrived(mqtt::const_message_ptr ptr) override {
        if(ptr->get_topic() == this->mqtt_topic_prefix + "teleop") {
            try {
                json json = json::from_bson(ptr->get_payload().begin(), ptr->get_payload().end());
                geometry_msgs::Twist t;
                t.linear.x = json["vx"];
                t.angular.z = json["vz"];
                cmd_vel_pub.publish(t);
            } catch (const json::exception &e) {
                ROS_ERROR_STREAM("Error decoding teleop bson: " << e.what());
            }
        } else if(ptr->get_topic() == this->mqtt_topic_prefix + "action") {
            ROS_INFO_STREAM("Got action: " + ptr->get_payload());
            std_msgs::String action_msg;
            action_msg.data = ptr->get_payload_str();
            action_pub.publish(action_msg);
        } else if(ptr->get_topic() == this->mqtt_topic_prefix + "/action") {
            // BEGIN: Deprecated code (2/2)
            ROS_WARN_STREAM("Got action on deprecated topic! Change your topic names!: " + ptr->get_payload());
            std_msgs::String action_msg;
            action_msg.data = ptr->get_payload_str();
            action_pub.publish(action_msg);
            // END: Deprecated code (2/2)
        } else if (ptr->get_topic() == this->mqtt_topic_prefix + "rpc/request") {
          std::string payload = ptr->get_payload_str();
          rpc_request_callback(payload);
        } else if (ptr->get_topic() == this->mqtt_topic_prefix + "timetable/set/json") {
            try {
                json payload = json::parse(ptr->get_payload_str());
                xbot_rpc::RpcRequest msg;
                msg.method = "timetable.replace";
                msg.params = json::array({payload}).dump();
                msg.id = "mqtt_timetable_set_json";
                rpc_request_pub.publish(msg);
            } catch (const json::exception &e) {
                publish_timetable_validation({{"valid", false}, {"remarks", {std::string("Error decoding timetable JSON: ") + e.what()}}});
            }
        } else if (ptr->get_topic() == this->mqtt_topic_prefix + "timetable/set/bson") {
            try {
                json payload = json::from_bson(ptr->get_payload().begin(), ptr->get_payload().end());
                if (payload.is_object() && payload.contains("d")) {
                    payload = payload["d"];
                }
                xbot_rpc::RpcRequest msg;
                msg.method = "timetable.replace";
                msg.params = json::array({payload}).dump();
                msg.id = "mqtt_timetable_set_bson";
                rpc_request_pub.publish(msg);
            } catch (const json::exception &e) {
                publish_timetable_validation({{"valid", false}, {"remarks", {std::string("Error decoding timetable BSON: ") + e.what()}}});
            }
        } else if (ptr->get_topic() == this->mqtt_topic_prefix + "timetable/set/suspension/json") {
            try {
                json payload = json::parse(ptr->get_payload_str());
                xbot_rpc::RpcRequest msg;
                msg.method = "timetable.suspension_set";
                msg.params = json::array({payload}).dump();
                msg.id = "mqtt_timetable_suspension_set_json";
                rpc_request_pub.publish(msg);
            } catch (const json::exception &e) {
                publish_timetable_validation({{"valid", false}, {"remarks", {std::string("Error decoding suspension JSON: ") + e.what()}}});
            }
        } else if (ptr->get_topic() == this->mqtt_topic_prefix + "timetable/set/suspension/bson") {
            try {
                json payload = json::from_bson(ptr->get_payload().begin(), ptr->get_payload().end());
                if (payload.is_object() && payload.contains("d")) {
                    payload = payload["d"];
                }
                xbot_rpc::RpcRequest msg;
                msg.method = "timetable.suspension_set";
                msg.params = json::array({payload}).dump();
                msg.id = "mqtt_timetable_suspension_set_bson";
                rpc_request_pub.publish(msg);
            } catch (const json::exception &e) {
                publish_timetable_validation({{"valid", false}, {"remarks", {std::string("Error decoding suspension BSON: ") + e.what()}}});
            }
        } else if (ptr->get_topic() == this->mqtt_topic_prefix + "map/set/json") {
            try {
                json payload = json::parse(ptr->get_payload_str());
                json validation = validate_map_payload_for_mqtt(payload);
                if (!validation.value("valid", false)) {
                    publish_map_validation(validation);
                } else {
                    xbot_rpc::RpcRequest msg;
                    msg.method = "map.replace";
                    msg.params = json::array({payload}).dump();
                    msg.id = "mqtt_map_set_json";
                    rpc_request_pub.publish(msg);
                }
            } catch (const json::exception &e) {
                publish_map_validation({{"valid", false}, {"remarks", {std::string("Error decoding map JSON: ") + e.what()}}});
            }
        } else if (ptr->get_topic() == this->mqtt_topic_prefix + "map/set/renew/json") {
            // App opened the areas page and requests the current retained map again.
            // The payload is intentionally optional; any message on this topic triggers a republish.
            publish_map();
        } else if (ptr->get_topic() == this->mqtt_topic_prefix + "map/mowing_progress/set/renew/json") {
            std_msgs::Empty msg;
            map_mowing_progress_renew_pub.publish(msg);
        } else if (ptr->get_topic() == this->mqtt_topic_prefix + "statustransition_log/set/renew/json") {
            // App requests the current retained status transition log again.
            // Empty payload: use the configured default limit.
            // Optional JSON payload: {"limit": 50} returns the newest N entries.
            std::size_t requested_limit = 0;
            const std::string payload_text = ptr->get_payload_str();
            if (!payload_text.empty()) {
                try {
                    json payload = json::parse(payload_text);
                    if (payload.is_object() && payload.contains("limit") && payload["limit"].is_number_unsigned()) {
                        requested_limit = payload["limit"].get<std::size_t>();
                    } else if (payload.is_object() && payload.contains("limit") && payload["limit"].is_number_integer()) {
                        const auto limit = payload["limit"].get<long long>();
                        if (limit > 0) {
                            requested_limit = static_cast<std::size_t>(limit);
                        }
                    }
                } catch (const json::exception &e) {
                    ROS_WARN_STREAM("Error decoding statustransition log renew JSON: " << e.what()
                                    << ". Falling back to configured default limit.");
                }
            }
            publish_statustransition_log(requested_limit);
        } else if (ptr->get_topic() == this->mqtt_topic_prefix + "settings/mower_logic/satellite_logging/set/control/json") {
            std_msgs::String msg;
            msg.data = ptr->get_payload_str();
            mower_logic_satellite_logging_control_pub.publish(msg);
        } else if (ptr->get_topic() == this->mqtt_topic_prefix + "settings/mower_logic/satellite_logging/set/renew/json") {
            std_msgs::Empty msg;
            mower_logic_satellite_logging_renew_pub.publish(msg);
        } else if (ptr->get_topic() == this->mqtt_topic_prefix + "settings/mower_logic/set/session/json") {
            std_msgs::String msg;
            msg.data = ptr->get_payload_str();
            mower_logic_settings_set_session_json_pub.publish(msg);
        } else if (ptr->get_topic() == this->mqtt_topic_prefix + "settings/mower_logic/set/persistent/json") {
            std_msgs::String msg;
            msg.data = ptr->get_payload_str();
            mower_logic_settings_set_persistent_json_pub.publish(msg);
        } else if (ptr->get_topic() == this->mqtt_topic_prefix + "settings/mower_logic/set/renew/json") {
            std_msgs::Empty msg;
            mower_logic_settings_renew_pub.publish(msg);
        } else if (ptr->get_topic() == this->mqtt_topic_prefix + "sensors/settings/set/renew/json") {
            publish_sensor_metadata();
        } else if (ptr->get_topic() == this->mqtt_topic_prefix + "sensors/settings/set/persistent/json") {
            handle_sensor_infos_persistent_payload(ptr->get_payload_str());
        } else if (ptr->get_topic() == this->mqtt_topic_prefix + "gps_state/settings/set/renew/json") {
            publish_gps_state_settings();
            publish_latest_gps_state_payloads(true);
        } else if (ptr->get_topic() == this->mqtt_topic_prefix + "gps_state/settings/set/session/json") {
            handle_gps_state_set_payload(ptr->get_payload_str(), false);
        } else if (ptr->get_topic() == this->mqtt_topic_prefix + "gps_state/settings/set/persistent/json") {
            handle_gps_state_set_payload(ptr->get_payload_str(), true);
        } else if (ptr->get_topic() == this->mqtt_topic_prefix + "settings/ll_board/set/session/json" ||
                   ptr->get_topic() == this->mqtt_topic_prefix + "settings/ll_board/set/persistent/json") {
            const bool persistent = ptr->get_topic() == this->mqtt_topic_prefix + "settings/ll_board/set/persistent/json";
            const std::string mode = persistent ? "persistent" : "session";
            json validation = {
                {"valid", false},
                {"namespace", "ll_board"},
                {"mode", mode},
                {"accepted", json::array()},
                {"rejected", json::array()}
            };
            try {
                json payload = json::parse(ptr->get_payload_str());
                if (!payload.is_object()) {
                    validation["rejected"].push_back({{"key", "$"}, {"reason", "payload must be a JSON object"}});
                    ROS_WARN_STREAM("Ignoring settings/ll_board set payload because it is not a JSON object.");
                } else {
                    const std::map<std::string, std::pair<ros::Publisher*, ros::Publisher*>> publishers = {
                        {"battery_critical_voltage", {&ll_power_set_battery_critical_voltage_pub, &ll_power_set_persistent_battery_critical_voltage_pub}},
                        {"battery_empty_voltage", {&ll_power_set_battery_empty_voltage_pub, &ll_power_set_persistent_battery_empty_voltage_pub}},
                        {"battery_full_voltage", {&ll_power_set_battery_full_voltage_pub, &ll_power_set_persistent_battery_full_voltage_pub}},
                        {"battery_critical_high_voltage", {&ll_power_set_battery_critical_high_voltage_pub, &ll_power_set_persistent_battery_critical_high_voltage_pub}},
                        {"charge_critical_high_voltage", {&ll_power_set_charge_critical_high_voltage_pub, &ll_power_set_persistent_charge_critical_high_voltage_pub}},
                        {"charge_critical_high_current", {&ll_power_set_charge_critical_high_current_pub, &ll_power_set_persistent_charge_critical_high_current_pub}},
                        {"speed_fast", {&ftc_settings_set_speed_fast_pub, &ftc_settings_set_persistent_speed_fast_pub}},
                        {"speed_slow", {&ftc_settings_set_speed_slow_pub, &ftc_settings_set_persistent_speed_slow_pub}}
                    };
                    std::map<std::string, double> accepted_values;
                    std::map<std::string, std::map<std::string, open_mower_settings::json>> accepted_metadata;
                    for (auto it = payload.begin(); it != payload.end(); ++it) {
                        const std::string key = it.key();
                        const json &entry = it.value();
                        const auto publisher_it = publishers.find(key);
                        if (publisher_it == publishers.end()) {
                            validation["rejected"].push_back({{"key", key}, {"reason", "unknown setting"}});
                            continue;
                        }
                        if (!entry.is_object()) {
                            validation["rejected"].push_back({{"key", key}, {"reason", "setting entry must be an object"}});
                            continue;
                        }
                        if (entry.empty()) {
                            validation["rejected"].push_back({{"key", key}, {"reason", "setting entry must contain at least one field"}});
                            continue;
                        }

                        bool rejected = false;
                        for (auto field = entry.begin(); field != entry.end(); ++field) {
                            if (field.key() != "value" && field.key() != "group" && field.key() != "expert") {
                                validation["rejected"].push_back({{"key", key}, {"field", field.key()}, {"reason", "unknown field"}});
                                rejected = true;
                            }
                        }
                        if (rejected) continue;

                        if (!persistent && (entry.contains("group") || entry.contains("expert"))) {
                            validation["rejected"].push_back({{"key", key}, {"reason", "metadata changes require persistent mode"}});
                            continue;
                        }

                        if (entry.contains("value")) {
                            if (!entry["value"].is_number()) {
                                validation["rejected"].push_back({{"key", key}, {"field", "value"}, {"reason", "value must be numeric"}});
                                continue;
                            }
                            accepted_values[key] = entry["value"].get<double>();
                        }

                        if (entry.contains("group")) {
                            std::string group;
                            std::string reason;
                            if (!validate_group_metadata_value(entry["group"], group, reason)) {
                                validation["rejected"].push_back({{"key", key}, {"field", "group"}, {"reason", reason}});
                                continue;
                            }
                            accepted_metadata[key]["group"] = group;
                        }

                        if (entry.contains("expert")) {
                            if (!entry["expert"].is_boolean()) {
                                validation["rejected"].push_back({{"key", key}, {"field", "expert"}, {"reason", "expert must be a boolean"}});
                                continue;
                            }
                            accepted_metadata[key]["expert"] = entry["expert"];
                        }
                    }
                    if (accepted_values.empty() && accepted_metadata.empty() && validation["rejected"].empty()) {
                        validation["rejected"].push_back({{"key", "$"}, {"reason", "payload does not contain any settings"}});
                    }
                    if (validation["rejected"].empty()) {
                        bool metadata_write_ok = true;
                        if (!accepted_metadata.empty()) {
                            std::string settings_persistent_path;
                            ros::param::param<std::string>("/settings/persistent_file", settings_persistent_path,
                                                           std::string("/data/ros/settings_persistent.json"));
                            metadata_write_ok = open_mower_settings::updateEntryFields(settings_persistent_path, "ll_board", accepted_metadata);
                            if (!metadata_write_ok) {
                                validation["rejected"].push_back({{"key", "$"}, {"reason", "could not write settings_persistent.json"}});
                            }
                        }
                        if (metadata_write_ok && validation["rejected"].empty()) {
                            std::map<std::string, json> accepted_field_names;
                            for (const auto &pair : accepted_values) {
                                std_msgs::Float64 msg;
                                msg.data = pair.second;
                                const auto publisher_it = publishers.find(pair.first);
                                ros::Publisher *publisher = persistent ? publisher_it->second.second : publisher_it->second.first;
                                publisher->publish(msg);
                                accepted_field_names[pair.first].push_back("value");
                            }
                            for (const auto &pair : accepted_metadata) {
                                for (const auto &field : pair.second) {
                                    accepted_field_names[pair.first].push_back(field.first);
                                }
                            }
                            for (const auto &pair : accepted_field_names) {
                                validation["accepted"].push_back({{"key", pair.first}, {"fields", pair.second}});
                            }
                            if (!accepted_metadata.empty()) {
                                publish_ll_power_status_request();
                            }
                        }
                    }
                }
            } catch (const json::exception &e) {
                validation["rejected"].push_back({{"key", "$"}, {"reason", std::string("Error decoding JSON: ") + e.what()}});
                ROS_WARN_STREAM("Error decoding settings/ll_board set JSON: " << e.what());
            }
            validation["valid"] = !validation["accepted"].empty() && validation["rejected"].empty();
            publish_settings_validation("ll_board", validation);
        } else if (ptr->get_topic() == this->mqtt_topic_prefix + "settings/ll_board/set/renew/json") {
            publish_ll_power_status_request();
        } else if (ptr->get_topic() == this->mqtt_topic_prefix + "timetable/set/renew/json" ||
                   ptr->get_topic() == this->mqtt_topic_prefix + "timetable/set/renew/bson") {
            // App opened the timetable page and requests the current retained timetable again.
            // The payload is intentionally optional; any message on this topic triggers a republish.
            maybe_publish_timetable(true);
        }
    }
private:
    std::shared_ptr<mqtt::async_client> client_;
    std::string mqtt_topic_prefix = "";
};

MqttCallback mqtt_callback;
MqttCallback mqtt_callback_external;

json map;
std::mutex map_mutex;
json map_overlay;
std::mutex map_overlay_mutex;
bool has_map = false;
bool has_map_overlay = false;

json timetable_status = json::object();
json timetable_confirmed = json::object();
std::mutex timetable_mutex;
bool has_timetable = false;
bool timetable_auto_mowing_time = false;
std::string timetable_auto_mow_id = "";
json timetable_auto_mow_suspension = 0;
ros::Time last_timetable_publish_time;
double mqtt_timetable_publish_interval_sec = 60.0;

std::string statustransition_log_file = "/data/ros/log_statustransition.json";
constexpr std::size_t STATUSTRANSITION_LOG_MAX_ENTRIES = 300;
std::size_t mqtt_statustransition_log_default_limit = 20;
std::mutex statustransition_log_mutex;
json statustransition_log_entries = json::array();
bool statustransition_log_loaded = false;
bool has_last_statustransition_key = false;
std::string last_statustransition_state;
std::string last_statustransition_sub_state;
bool last_statustransition_charging = false;
bool last_statustransition_emergency = false;
bool has_last_statustransition_timestamp = false;
std::chrono::system_clock::time_point last_statustransition_timestamp;

std::mutex latest_double_sensor_values_mutex;
std::map<std::string, double> latest_double_sensor_values;
std::mutex latest_string_sensor_values_mutex;
std::map<std::string, std::string> latest_string_sensor_values;

constexpr const char *SENSOR_INFOS_NAMESPACE = "sensors";
constexpr const char *SENSOR_INFOS_SCHEMA = "settings_v2";
constexpr const char *GPS_STATE_NAMESPACE = "gps_state";
constexpr const char *GPS_STATE_SCHEMA = "settings_v2";

struct GpsStateSettings {
    bool enabled = true;
    double publish_rate_hz = 1.0;
    bool publish_state1 = true;
    bool publish_state2 = true;
    bool publish_state3 = true;
    bool publish_state4 = false;
    double weak_cn0_threshold = 20.0;
    double good_cn0_threshold = 30.0;
};

std::mutex gps_state_settings_mutex;
GpsStateSettings gps_state_settings;
bool gps_state_settings_loaded = false;

std::mutex gps_state_payload_mutex;
json latest_gps_state_payloads = json::object();
bool latest_gps_state_available = false;
ros::Time last_gps_state_publish_time;

std::mutex gps_state_pose_mutex;
xbot_msgs::AbsolutePose latest_ll_gps_pose;
xbot_msgs::AbsolutePose latest_xb_pose;
bool latest_ll_gps_pose_available = false;
bool latest_xb_pose_available = false;
ros::Time latest_ll_gps_pose_received_at;
ros::Time latest_xb_pose_received_at;
ros::Time latest_gps_drive_ready_at;



static json gps_state_setting_entry(const std::string &label,
                                    const std::string &description,
                                    const std::string &group,
                                    int order,
                                    const std::string &type,
                                    const json &value,
                                    const json &min_value = nullptr,
                                    const json &max_value = nullptr,
                                    const std::string &unit = "",
                                    bool expert = false,
                                    bool session_apply_supported = true) {
    json entry = json::object();
    entry["label"] = label;
    entry["description"] = description;
    entry["group"] = group;
    entry["order"] = order;
    entry["type"] = type;
    entry["value"] = value;
    entry["active"] = value;
    entry["persistent"] = value;
    entry["visible"] = true;
    entry["expert"] = expert;
    entry["different"] = false;
    entry["restart_required"] = false;
    entry["session_apply_supported"] = session_apply_supported;
    if (!unit.empty()) entry["unit"] = unit;
    if (!min_value.is_null()) entry["min"] = min_value;
    if (!max_value.is_null()) entry["max"] = max_value;
    return entry;
}

static json gps_state_descriptor_entry(const std::string &label,
                                       const std::string &description,
                                       int order,
                                       const std::string &topic,
                                       bool expert) {
    json entry = json::object();
    entry["label"] = label;
    entry["description"] = description;
    entry["group"] = "states";
    entry["order"] = order;
    entry["type"] = "json";
    entry["topic"] = topic;
    entry["value"] = nullptr;
    entry["active"] = nullptr;
    entry["persistent"] = nullptr;
    entry["visible"] = true;
    entry["expert"] = expert;
    entry["readonly"] = true;
    entry["different"] = false;
    entry["restart_required"] = false;
    entry["session_apply_supported"] = false;
    return entry;
}

static json read_gps_state_persisted_namespace() {
    std::string settings_persistent_path;
    ros::param::param<std::string>("/settings/persistent_file", settings_persistent_path,
                                   std::string("/data/ros/settings_persistent.json"));
    const open_mower_settings::json persisted = open_mower_settings::readNamespace(settings_persistent_path, GPS_STATE_NAMESPACE);
    return json::parse(persisted.dump());
}

static bool persisted_bool_or(const json &persisted, const std::string &key, bool fallback) {
    if (!persisted.is_object() || !persisted.contains(key) || !persisted[key].is_object()) return fallback;
    const json &entry = persisted[key];
    if (entry.contains("persistent") && entry["persistent"].is_boolean()) return entry["persistent"].get<bool>();
    if (entry.contains("value") && entry["value"].is_boolean()) return entry["value"].get<bool>();
    return fallback;
}

static double persisted_number_or(const json &persisted, const std::string &key, double fallback) {
    if (!persisted.is_object() || !persisted.contains(key) || !persisted[key].is_object()) return fallback;
    const json &entry = persisted[key];
    if (entry.contains("persistent") && entry["persistent"].is_number()) return entry["persistent"].get<double>();
    if (entry.contains("value") && entry["value"].is_number()) return entry["value"].get<double>();
    return fallback;
}

static double clamp_gps_state_rate(double value) {
    if (!std::isfinite(value)) return 1.0;
    return std::max(0.1, std::min(5.0, value));
}

static void load_gps_state_settings_if_needed() {
    std::lock_guard<std::mutex> lk(gps_state_settings_mutex);
    if (gps_state_settings_loaded) return;
    const json persisted = read_gps_state_persisted_namespace();
    gps_state_settings.enabled = persisted_bool_or(persisted, "enabled", gps_state_settings.enabled);
    gps_state_settings.publish_rate_hz = clamp_gps_state_rate(
        persisted_number_or(persisted, "publish_rate_hz", gps_state_settings.publish_rate_hz));
    gps_state_settings.publish_state1 = persisted_bool_or(persisted, "publish_state1", gps_state_settings.publish_state1);
    gps_state_settings.publish_state2 = persisted_bool_or(persisted, "publish_state2", gps_state_settings.publish_state2);
    gps_state_settings.publish_state3 = persisted_bool_or(persisted, "publish_state3", gps_state_settings.publish_state3);
    gps_state_settings.publish_state4 = persisted_bool_or(persisted, "publish_state4", gps_state_settings.publish_state4);
    gps_state_settings.weak_cn0_threshold = persisted_number_or(persisted, "weak_cn0_threshold", gps_state_settings.weak_cn0_threshold);
    gps_state_settings.good_cn0_threshold = persisted_number_or(persisted, "good_cn0_threshold", gps_state_settings.good_cn0_threshold);
    gps_state_settings_loaded = true;
}

static GpsStateSettings current_gps_state_settings() {
    load_gps_state_settings_if_needed();
    std::lock_guard<std::mutex> lk(gps_state_settings_mutex);
    return gps_state_settings;
}

static json build_gps_state_settings_payload() {
    const GpsStateSettings cfg = current_gps_state_settings();
    json root = json::object();
    root["namespace"] = GPS_STATE_NAMESPACE;
    root["schema"] = GPS_STATE_SCHEMA;
    root["groups"] = {
        {"general", {{"label", "Allgemein"}, {"order", 10}}},
        {"states", {{"label", "GPS States"}, {"order", 20}}},
        {"debug", {{"label", "Debug"}, {"order", 90}}}
    };
    root["settings"] = json::object();
    root["settings"]["enabled"] = gps_state_setting_entry(
        "GPS State aktiv",
        "Aktiviert die MQTT-Ausgabe der GPS-State-Daten.",
        "general", 10, "bool", cfg.enabled);
    root["settings"]["publish_rate_hz"] = gps_state_setting_entry(
        "Publish-Rate",
        "Maximale Veröffentlichungsrate der GPS-State-Daten.",
        "general", 20, "double", cfg.publish_rate_hz, 0.1, 5.0, "Hz");
    root["settings"]["state1"] = gps_state_descriptor_entry(
        "GPS State 1",
        "Kompakter Bedienerstatus. Enthält die bestehende GPS-Übersicht und ergänzt die zusammengefasste Entscheidung, ob GPS aktuell für die OpenMower-Fahrt ausreicht. Technische Ursachen bleiben in State 2.",
        10, "gps_state/state1", false);
    root["settings"]["state2"] = gps_state_descriptor_entry(
        "GPS State 2",
        "Erweiterte GPS-Zusammenfassung mit statistischer Bewertung, Verteilung nach Satellitensystemen und technischen Fahrfreigabe-Diagnosen aus /ll/position/gps und /xbot_positioning/xb_pose.",
        20, "gps_state/state2", false);
    root["settings"]["state3"] = gps_state_descriptor_entry(
        "GPS State 3",
        "Detaildaten der Satelliten, die aktuell aktiv für den Positionsfix verwendet werden. Enthält eine Liste der USED=true Satelliten mit GNSS-System, SV-ID, C/N0-Signalstärke, Elevation, Azimut, Residual und Qualitätswert.",
        30, "gps_state/state3", false);
    root["settings"]["state4"] = gps_state_descriptor_entry(
        "GPS State 4",
        "Vollständige Satelliten-Diagnose mit allen sichtbaren Satelliten, auch wenn sie nicht für den Positionsfix verwendet werden. Enthält used=true/false, GNSS-System, SV-ID, C/N0, Elevation, Azimut, Residual und Qualität. Gedacht für Experten- und Debuganalyse.",
        40, "gps_state/state4", true);
    root["settings"]["publish_state1"] = gps_state_setting_entry(
        "State 1 veröffentlichen",
        "Veröffentlicht den kompakten GPS-State auf gps_state/state1.",
        "states", 110, "bool", cfg.publish_state1);
    root["settings"]["publish_state2"] = gps_state_setting_entry(
        "State 2 veröffentlichen",
        "Veröffentlicht die erweiterte GPS-Zusammenfassung auf gps_state/state2.",
        "states", 120, "bool", cfg.publish_state2);
    root["settings"]["publish_state3"] = gps_state_setting_entry(
        "State 3 veröffentlichen",
        "Veröffentlicht die Liste der aktuell verwendeten Satelliten auf gps_state/state3.",
        "states", 130, "bool", cfg.publish_state3);
    root["settings"]["publish_state4"] = gps_state_setting_entry(
        "State 4 veröffentlichen",
        "Veröffentlicht die vollständige Satellitenliste auf gps_state/state4. Diese Ausgabe kann deutlich größer sein und ist primär für Debug- und Expertenansichten gedacht.",
        "debug", 10, "bool", cfg.publish_state4, nullptr, nullptr, "", true);
    root["settings"]["weak_cn0_threshold"] = gps_state_setting_entry(
        "Schwach-Schwelle C/N0",
        "Grenzwert in dB-Hz, unterhalb dessen ein verwendeter Satellit als schwach gezählt wird.",
        "debug", 20, "double", cfg.weak_cn0_threshold, 0.0, 60.0, "dB-Hz", true);
    root["settings"]["good_cn0_threshold"] = gps_state_setting_entry(
        "Gut-Schwelle C/N0",
        "Grenzwert in dB-Hz, ab dem ein verwendeter Satellit als gut gezählt wird.",
        "debug", 30, "double", cfg.good_cn0_threshold, 0.0, 60.0, "dB-Hz", true);
    return root;
}

void publish_gps_state_settings() {
    const json payload = build_gps_state_settings_payload();
    try_publish("gps_state/settings/json", payload.dump(), true);
}

void publish_gps_state_validation(const json &validation) {
    try_publish("gps_state/settings/validation/json", validation.dump(), false);
}

static bool gps_state_validate_bool_setting(const json &entry, bool &value, std::string &reason) {
    if (!entry.is_object() || !entry.contains("value")) {
        reason = "setting entry must be an object with a value field";
        return false;
    }
    if (!entry["value"].is_boolean()) {
        reason = "value must be a boolean";
        return false;
    }
    value = entry["value"].get<bool>();
    return true;
}

static bool gps_state_validate_number_setting(const json &entry, double min_value, double max_value,
                                              double &value, std::string &reason) {
    if (!entry.is_object() || !entry.contains("value")) {
        reason = "setting entry must be an object with a value field";
        return false;
    }
    if (!entry["value"].is_number()) {
        reason = "value must be numeric";
        return false;
    }
    value = entry["value"].get<double>();
    if (!std::isfinite(value) || value < min_value || value > max_value) {
        std::ostringstream msg;
        msg << "value must be between " << min_value << " and " << max_value;
        reason = msg.str();
        return false;
    }
    return true;
}

void handle_gps_state_set_payload(const std::string &payload_text, bool persistent) {
    json validation = {
        {"valid", false},
        {"namespace", GPS_STATE_NAMESPACE},
        {"mode", persistent ? "persistent" : "session"},
        {"accepted", json::object()},
        {"rejected", json::object()},
        {"remarks", json::array()}
    };

    try {
        const json payload = json::parse(payload_text.empty() ? "{}" : payload_text);
        if (!payload.is_object()) {
            validation["rejected"]["$"] = "payload must be a JSON object";
            publish_gps_state_validation(validation);
            return;
        }

        GpsStateSettings new_cfg = current_gps_state_settings();
        std::map<std::string, std::map<std::string, open_mower_settings::json>> persistent_updates;

        for (auto it = payload.begin(); it != payload.end(); ++it) {
            const std::string key = it.key();
            const json &entry = it.value();
            std::string reason;
            bool bool_value = false;
            double number_value = 0.0;
            bool accepted = false;
            json accepted_fields = json::array();

            if (key == "enabled") {
                accepted = gps_state_validate_bool_setting(entry, bool_value, reason);
                if (accepted) new_cfg.enabled = bool_value;
            } else if (key == "publish_state1") {
                accepted = gps_state_validate_bool_setting(entry, bool_value, reason);
                if (accepted) new_cfg.publish_state1 = bool_value;
            } else if (key == "publish_state2") {
                accepted = gps_state_validate_bool_setting(entry, bool_value, reason);
                if (accepted) new_cfg.publish_state2 = bool_value;
            } else if (key == "publish_state3") {
                accepted = gps_state_validate_bool_setting(entry, bool_value, reason);
                if (accepted) new_cfg.publish_state3 = bool_value;
            } else if (key == "publish_state4") {
                accepted = gps_state_validate_bool_setting(entry, bool_value, reason);
                if (accepted) new_cfg.publish_state4 = bool_value;
            } else if (key == "publish_rate_hz") {
                accepted = gps_state_validate_number_setting(entry, 0.1, 5.0, number_value, reason);
                if (accepted) new_cfg.publish_rate_hz = number_value;
            } else if (key == "weak_cn0_threshold") {
                accepted = gps_state_validate_number_setting(entry, 0.0, 60.0, number_value, reason);
                if (accepted) new_cfg.weak_cn0_threshold = number_value;
            } else if (key == "good_cn0_threshold") {
                accepted = gps_state_validate_number_setting(entry, 0.0, 60.0, number_value, reason);
                if (accepted) new_cfg.good_cn0_threshold = number_value;
            } else {
                validation["rejected"][key] = "unknown gps_state setting";
                continue;
            }

            if (!accepted) {
                validation["rejected"][key] = reason;
                continue;
            }

            accepted_fields.push_back("value");
            validation["accepted"][key] = accepted_fields;
            if (persistent) {
                persistent_updates[key]["persistent"] = open_mower_settings::json::parse(entry["value"].dump());
            }
        }

        if (validation["accepted"].empty() && validation["rejected"].empty()) {
            validation["rejected"]["$"] = "payload does not contain any gps_state settings";
        }

        if (validation["rejected"].empty()) {
            {
                std::lock_guard<std::mutex> lk(gps_state_settings_mutex);
                gps_state_settings = new_cfg;
                gps_state_settings_loaded = true;
            }
            if (persistent && !persistent_updates.empty()) {
                std::string settings_persistent_path;
                ros::param::param<std::string>("/settings/persistent_file", settings_persistent_path,
                                               std::string("/data/ros/settings_persistent.json"));
                if (!open_mower_settings::updateEntryFields(settings_persistent_path, GPS_STATE_NAMESPACE, persistent_updates)) {
                    validation["accepted"] = json::object();
                    validation["rejected"]["$"] = "could not write settings_persistent.json";
                }
            }
        }
    } catch (const json::exception &e) {
        validation["rejected"]["$"] = std::string("Error decoding JSON: ") + e.what();
    }

    validation["valid"] = !validation["accepted"].empty() && validation["rejected"].empty();
    publish_gps_state_validation(validation);
    publish_gps_state_settings();
    publish_latest_gps_state_payloads(true);
}


static double gps_state_age_ms_or_negative(const ros::Time &now, const ros::Time &received_at) {
    if (now.isZero() || received_at.isZero()) return -1.0;
    return std::max(0.0, (now - received_at).toSec() * 1000.0);
}

static json gps_state_age_ms_json(const ros::Time &now, const ros::Time &received_at) {
    const double age_ms = gps_state_age_ms_or_negative(now, received_at);
    if (age_ms < 0.0) return nullptr;
    return age_ms;
}

static std::string gps_state_rtk_state(uint16_t flags) {
    if (flags & xbot_msgs::AbsolutePose::FLAG_GPS_RTK_FIXED) return "fixed";
    if (flags & xbot_msgs::AbsolutePose::FLAG_GPS_RTK_FLOAT) return "float";
    if (flags & xbot_msgs::AbsolutePose::FLAG_GPS_RTK) return "rtk";
    return "none";
}

static std::string gps_state_drive_block_reason(bool xb_pose_available,
                                                bool orientation_valid,
                                                bool recent_absolute_pose,
                                                bool xb_accuracy_ok,
                                                bool ll_pose_available,
                                                const std::string &rtk_state) {
    if (!xb_pose_available) return "no_xbot_positioning_pose";
    if (!orientation_valid) return "orientation_invalid";
    if (!recent_absolute_pose) {
        if (ll_pose_available && rtk_state == "float") return "rtk_float_not_sufficient";
        if (ll_pose_available && rtk_state != "fixed") return "rtk_fixed_missing";
        return "recent_absolute_pose_missing";
    }
    if (!xb_accuracy_ok) return "position_accuracy_too_low";
    return "unknown";
}

static std::string gps_state_drive_reason_text(const std::string &block_reason,
                                               const std::string &rtk_state) {
    if (block_reason.empty()) return "RTK Fixed, Pose aktuell, Genauigkeit ausreichend";
    if (block_reason == "no_xbot_positioning_pose") return "Keine aktuelle Pose von xbot_positioning vorhanden";
    if (block_reason == "orientation_invalid") return "Orientierung ist nicht gueltig";
    if (block_reason == "rtk_float_not_sufficient") return "RTK Float reicht nicht zum Fahren aus";
    if (block_reason == "rtk_fixed_missing") return "RTK Fixed fehlt; aktueller RTK-Zustand: " + rtk_state;
    if (block_reason == "recent_absolute_pose_missing") return "Positioning-Pose enthaelt keine aktuelle absolute GPS-Pose";
    if (block_reason == "position_accuracy_too_low") return "Positionsgenauigkeit ist schlechter als der erlaubte Grenzwert";
    return "GPS-Fahrfreigabe konnte nicht bestaetigt werden";
}

static json build_gps_drive_status_payload(const ros::Time &now) {
    double max_position_accuracy = 0.2;
    double max_gps_accuracy = 0.2;
    double gps_timeout = 10.0;
    ros::param::param<double>("/mower_logic/max_position_accuracy", max_position_accuracy, 0.2);
    ros::param::param<double>("/xbot_positioning/max_gps_accuracy", max_gps_accuracy, 0.2);
    ros::param::param<double>("/mower_logic/gps_timeout", gps_timeout, 10.0);

    xbot_msgs::AbsolutePose ll_pose;
    xbot_msgs::AbsolutePose xb_pose;
    bool ll_pose_available = false;
    bool xb_pose_available = false;
    ros::Time ll_received_at;
    ros::Time xb_received_at;
    ros::Time drive_ready_at;
    {
        std::lock_guard<std::mutex> lk(gps_state_pose_mutex);
        ll_pose = latest_ll_gps_pose;
        xb_pose = latest_xb_pose;
        ll_pose_available = latest_ll_gps_pose_available;
        xb_pose_available = latest_xb_pose_available;
        ll_received_at = latest_ll_gps_pose_received_at;
        xb_received_at = latest_xb_pose_received_at;
        drive_ready_at = latest_gps_drive_ready_at;
    }

    const std::string rtk_state = ll_pose_available ? gps_state_rtk_state(ll_pose.flags) : "unknown";
    const bool ll_rtk_fixed = ll_pose_available &&
        ((ll_pose.flags & xbot_msgs::AbsolutePose::FLAG_GPS_RTK_FIXED) != 0);
    const bool ll_rtk_float = ll_pose_available &&
        ((ll_pose.flags & xbot_msgs::AbsolutePose::FLAG_GPS_RTK_FLOAT) != 0);
    const bool ll_accuracy_ok = ll_pose_available && ll_pose.position_accuracy <= max_gps_accuracy;

    const bool orientation_valid = xb_pose_available && xb_pose.orientation_valid;
    const bool recent_absolute_pose = xb_pose_available &&
        ((xb_pose.flags & xbot_msgs::AbsolutePose::FLAG_SENSOR_FUSION_RECENT_ABSOLUTE_POSE) != 0);
    const bool xb_accuracy_ok = xb_pose_available && xb_pose.position_accuracy < max_position_accuracy;
    const bool gps_drive_ready = xb_pose_available && orientation_valid && recent_absolute_pose && xb_accuracy_ok;

    const bool gps_timeout_estimated = !gps_drive_ready &&
        (drive_ready_at.isZero() || (now - drive_ready_at).toSec() > gps_timeout);
    const double grace_remaining_s = (!gps_drive_ready && !drive_ready_at.isZero())
        ? std::max(0.0, gps_timeout - (now - drive_ready_at).toSec())
        : 0.0;

    const std::string block_reason = gps_drive_ready
        ? std::string()
        : gps_state_drive_block_reason(xb_pose_available, orientation_valid, recent_absolute_pose,
                                       xb_accuracy_ok, ll_pose_available, rtk_state);

    json summary = json::object();
    summary["gps_drive_ready"] = gps_drive_ready;
    summary["gps_drive_state"] = gps_drive_ready ? "ready" : "blocked";
    summary["gps_drive_label"] = gps_drive_ready ? "GPS reicht zum Fahren aus" : "GPS reicht nicht zum Fahren aus";
    summary["gps_drive_reason"] = gps_state_drive_reason_text(block_reason, rtk_state);
    summary["gps_drive_block_reason"] = gps_drive_ready ? json(nullptr) : json(block_reason);
    summary["rtk_state"] = rtk_state;
    summary["position_accuracy_m"] = xb_pose_available ? json(static_cast<double>(xb_pose.position_accuracy)) : json(nullptr);
    summary["max_position_accuracy_m"] = max_position_accuracy;
    summary["orientation_valid"] = xb_pose_available ? json(static_cast<bool>(xb_pose.orientation_valid)) : json(nullptr);
    summary["recent_absolute_pose"] = xb_pose_available ? json(recent_absolute_pose) : json(nullptr);
    summary["gps_timeout"] = gps_timeout_estimated;
    summary["age_ms"] = gps_state_age_ms_json(now, xb_received_at);

    json details = summary;
    details["mower_logic_gps_timeout_s"] = gps_timeout;
    details["mower_logic_gps_grace_remaining_s"] = grace_remaining_s;
    details["last_drive_ready_age_ms"] = gps_state_age_ms_json(now, drive_ready_at);
    details["xbot_positioning_max_gps_accuracy_m"] = max_gps_accuracy;
    details["ll_gps_available"] = ll_pose_available;
    details["ll_gps_flags"] = ll_pose_available ? json(ll_pose.flags) : json(nullptr);
    details["ll_gps_rtk_fixed"] = ll_pose_available ? json(ll_rtk_fixed) : json(nullptr);
    details["ll_gps_rtk_float"] = ll_pose_available ? json(ll_rtk_float) : json(nullptr);
    details["ll_gps_position_accuracy_m"] = ll_pose_available ? json(static_cast<double>(ll_pose.position_accuracy)) : json(nullptr);
    details["ll_gps_accuracy_ok_for_positioning"] = ll_pose_available ? json(ll_accuracy_ok) : json(nullptr);
    details["ll_gps_age_ms"] = gps_state_age_ms_json(now, ll_received_at);
    details["xb_pose_available"] = xb_pose_available;
    details["xb_pose_flags"] = xb_pose_available ? json(xb_pose.flags) : json(nullptr);
    details["xb_pose_source"] = xb_pose_available ? json(xb_pose.source) : json(nullptr);
    details["xb_pose_accuracy_ok_for_mower_logic"] = xb_pose_available ? json(xb_accuracy_ok) : json(nullptr);
    details["xb_pose_age_ms"] = gps_state_age_ms_json(now, xb_received_at);
    details["decision_source"] = "xbot_positioning/xb_pose";
    details["rtk_source"] = "ll/position/gps";

    return {
        {"summary", summary},
        {"details", details}
    };
}

static void apply_gps_drive_status_to_payloads(json &payloads, const json &drive_status) {
    if (payloads.contains("state1") && payloads["state1"].is_object()) {
        for (auto it = drive_status["summary"].begin(); it != drive_status["summary"].end(); ++it) {
            payloads["state1"][it.key()] = it.value();
        }
    }
    if (payloads.contains("state2") && payloads["state2"].is_object()) {
        payloads["state2"]["drive_diagnostics"] = drive_status["details"];
    }
}

static std::string gps_state_quality(bool available, int used_count, double avg_cn0) {
    if (!available || used_count <= 0) return "unavailable";
    if (used_count < 6 || avg_cn0 < 20.0) return "poor";
    if (used_count < 10 || avg_cn0 < 30.0) return "fair";
    if (used_count < 16 || avg_cn0 < 38.0) return "good";
    return "very_good";
}

static json gps_state_satellite_json(const xbot_msgs::GnssSatellite &sat) {
    json entry = json::object();
    entry["gnss"] = sat.gnss;
    entry["gnss_id"] = sat.gnss_id;
    entry["sv"] = sat.sv_id;
    entry["used"] = sat.used;
    entry["cn0"] = sat.cno;
    entry["elev"] = sat.elev;
    entry["azim"] = sat.azim;
    entry["prres"] = sat.pr_res;
    entry["qual"] = sat.quality_ind;
    return entry;
}

static json build_gps_state_payloads(const xbot_msgs::GnssSatelliteArray::ConstPtr &msg,
                                     const GpsStateSettings &cfg) {
    const int visible_count = msg->num_svs > 0 ? static_cast<int>(msg->num_svs)
                                               : static_cast<int>(msg->satellites.size());
    int used_count = 0;
    int weak_count = 0;
    int good_count = 0;
    double cno_sum = 0.0;
    double min_cn0 = std::numeric_limits<double>::infinity();
    double max_cn0 = 0.0;
    json systems = json::object();
    json used_satellites = json::array();
    json all_satellites = json::array();

    std::vector<xbot_msgs::GnssSatellite> satellites = msg->satellites;
    std::sort(satellites.begin(), satellites.end(), [](const auto &a, const auto &b) {
        if (a.gnss != b.gnss) return a.gnss < b.gnss;
        return a.sv_id < b.sv_id;
    });

    for (const auto &sat : satellites) {
        const std::string system = sat.gnss.empty() ? std::string("UNKNOWN") : sat.gnss;
        if (!systems.contains(system)) {
            systems[system] = {{"visible", 0}, {"used", 0}};
        }
        systems[system]["visible"] = systems[system]["visible"].get<int>() + 1;
        if (sat.used) {
            systems[system]["used"] = systems[system]["used"].get<int>() + 1;
            used_count += 1;
            cno_sum += sat.cno;
            min_cn0 = std::min(min_cn0, static_cast<double>(sat.cno));
            max_cn0 = std::max(max_cn0, static_cast<double>(sat.cno));
            if (static_cast<double>(sat.cno) < cfg.weak_cn0_threshold) weak_count += 1;
            if (static_cast<double>(sat.cno) >= cfg.good_cn0_threshold) good_count += 1;
            used_satellites.push_back(gps_state_satellite_json(sat));
        }
        all_satellites.push_back(gps_state_satellite_json(sat));
    }

    const bool available = !satellites.empty();
    const double avg_cn0 = used_count > 0 ? cno_sum / static_cast<double>(used_count) : 0.0;
    if (used_count == 0) min_cn0 = 0.0;
    const ros::Time now = ros::Time::now();
    const double stamp = msg->header.stamp.toSec();
    const std::string quality = gps_state_quality(available, used_count, avg_cn0);
    const json drive_status = build_gps_drive_status_payload(now);

    json base = json::object();
    base["available"] = available;
    base["quality"] = quality;
    base["visible"] = visible_count;
    base["used"] = used_count;
    base["avg_cn0"] = avg_cn0;
    base["updated_at"] = stamp;

    json state1 = base;
    state1["state"] = "state1";

    json state2 = base;
    state2["state"] = "state2";
    state2["sensor_stamp"] = msg->sensor_stamp;
    state2["min_cn0"] = min_cn0;
    state2["max_cn0"] = max_cn0;
    state2["weak_count"] = weak_count;
    state2["good_count"] = good_count;
    state2["systems"] = systems;

    json state3 = base;
    state3["state"] = "state3";
    state3["sensor_stamp"] = msg->sensor_stamp;
    state3["min_cn0"] = min_cn0;
    state3["max_cn0"] = max_cn0;
    state3["satellites"] = used_satellites;

    json state4 = base;
    state4["state"] = "state4";
    state4["sensor_stamp"] = msg->sensor_stamp;
    state4["min_cn0"] = min_cn0;
    state4["max_cn0"] = max_cn0;
    state4["satellites"] = all_satellites;

    json payloads = {
        {"state1", state1},
        {"state2", state2},
        {"state3", state3},
        {"state4", state4}
    };
    apply_gps_drive_status_to_payloads(payloads, drive_status);
    return payloads;
}

void publish_latest_gps_state_payloads(bool force) {
    GpsStateSettings cfg = current_gps_state_settings();
    if (!cfg.enabled && !force) return;

    json snapshot;
    bool has_snapshot = false;
    {
        std::lock_guard<std::mutex> lk(gps_state_payload_mutex);
        has_snapshot = latest_gps_state_available;
        snapshot = latest_gps_state_payloads;
    }
    if (!has_snapshot) return;

    const json drive_status = build_gps_drive_status_payload(ros::Time::now());
    apply_gps_drive_status_to_payloads(snapshot, drive_status);

    if (cfg.publish_state1 || force) try_publish("gps_state/state1", snapshot["state1"].dump(), true);
    if (cfg.publish_state2 || force) try_publish("gps_state/state2", snapshot["state2"].dump(), true);
    if (cfg.publish_state3 || force) try_publish("gps_state/state3", snapshot["state3"].dump(), false);
    if (cfg.publish_state4 || force) try_publish("gps_state/state4", snapshot["state4"].dump(), false);
}

void gps_state_satellites_callback(const xbot_msgs::GnssSatelliteArray::ConstPtr &msg) {
    const GpsStateSettings cfg = current_gps_state_settings();
    if (!cfg.enabled) return;

    const ros::Time now = ros::Time::now();
    const double min_interval = 1.0 / std::max(0.1, cfg.publish_rate_hz);
    if (!last_gps_state_publish_time.isZero() &&
        (now - last_gps_state_publish_time).toSec() < min_interval) {
        return;
    }
    last_gps_state_publish_time = now;

    {
        std::lock_guard<std::mutex> lk(gps_state_payload_mutex);
        latest_gps_state_payloads = build_gps_state_payloads(msg, cfg);
        latest_gps_state_available = true;
    }
    publish_latest_gps_state_payloads(false);
}

void gps_state_ll_gps_pose_callback(const xbot_msgs::AbsolutePose::ConstPtr &msg) {
    std::lock_guard<std::mutex> lk(gps_state_pose_mutex);
    latest_ll_gps_pose = *msg;
    latest_ll_gps_pose_received_at = ros::Time::now();
    latest_ll_gps_pose_available = true;
}

void gps_state_xb_pose_callback(const xbot_msgs::AbsolutePose::ConstPtr &msg) {
    double max_position_accuracy = 0.2;
    ros::param::param<double>("/mower_logic/max_position_accuracy", max_position_accuracy, 0.2);

    const bool orientation_valid = msg->orientation_valid;
    const bool recent_absolute_pose =
        (msg->flags & xbot_msgs::AbsolutePose::FLAG_SENSOR_FUSION_RECENT_ABSOLUTE_POSE) != 0;
    const bool accuracy_ok = msg->position_accuracy < max_position_accuracy;

    std::lock_guard<std::mutex> lk(gps_state_pose_mutex);
    latest_xb_pose = *msg;
    latest_xb_pose_received_at = ros::Time::now();
    latest_xb_pose_available = true;
    if (orientation_valid && recent_absolute_pose && accuracy_ok) {
        latest_gps_drive_ready_at = latest_xb_pose_received_at;
    }
}

xbot_rpc::RpcProvider rpc_provider("xbot_monitoring", {{
    RPC_METHOD("rpc.ping", {
        return "pong";
    }),
    RPC_METHOD("rpc.methods", {
        std::lock_guard<std::mutex> lk(registered_methods_mutex);
        json methods = json::array();
        for (const auto& [_, method_ids] : registered_methods) {
            for (const auto& method_id : method_ids) {
                methods.push_back(method_id);
            }
        }
        std::sort(methods.begin(), methods.end());
        return methods;
    }),
}});

void setupMqttClient() {
    // setup mqtt client for app use
    {
        // MQTT connection options
        mqtt::connect_options connect_options_;

        // basic client connection options
        connect_options_.set_automatic_reconnect(true);
        connect_options_.set_clean_session(true);
        connect_options_.set_keep_alive_interval(1000);
        connect_options_.set_max_inflight(10);

        // create MQTT client
        std::string uri = "tcp" + std::string("://") + "127.0.0.1" +
                          std::string(":") + std::to_string(1883);

        try {
            client_ = std::make_shared<mqtt::async_client>(
                    uri, "xbot_monitoring");
            mqtt_callback.setMqttClient(client_, "");
            client_->set_callback(mqtt_callback);

            client_->connect(connect_options_);

        } catch (const mqtt::exception &e) {
            ROS_ERROR("Client could not be initialized: %s", e.what());
            exit(EXIT_FAILURE);
        }
    }
    // setup external mqtt client
    if(external_mqtt_enable) {
        // MQTT connection options
        mqtt::connect_options connect_options_;

        // basic client connection options
        connect_options_.set_automatic_reconnect(true);
        connect_options_.set_clean_session(true);
        connect_options_.set_keep_alive_interval(1000);
        connect_options_.set_max_inflight(10);

        if(!external_mqtt_username.empty()) {
            connect_options_.set_user_name(external_mqtt_username);
            connect_options_.set_password(external_mqtt_password);
        }

        // create MQTT client
        std::string uri = "tcp" + std::string("://") + external_mqtt_hostname +
                          std::string(":") + external_mqtt_port;

        try {
            client_external_ = std::make_shared<mqtt::async_client>(
                    uri, "ext_xbot_monitoring");
            mqtt_callback_external.setMqttClient(client_external_, external_mqtt_topic_prefix);
            client_external_->set_callback(mqtt_callback_external);

            client_external_->connect(connect_options_);

        } catch (const mqtt::exception &e) {
            ROS_ERROR("External Client could not be initialized: %s", e.what());
            exit(EXIT_FAILURE);
        }
    }
}

void try_publish(const std::string &topic, const std::string &data, bool retain) {
    try {
        if (retain) {
            // QOS 1 so that the data actually arrives at the client at least once.
            client_->publish(topic, data, 1, true);
        } else {
            client_->publish(topic, data);
        }
    } catch (const mqtt::exception &e) {
        // client disconnected or something, we drop it.
    }
    // publish external
    if(external_mqtt_enable) {
        try {
            if (retain) {
                // QOS 1 so that the data actually arrives at the client at least once.
                client_external_->publish(external_mqtt_topic_prefix + topic, data, 1, true);
            } else {
                client_external_->publish(external_mqtt_topic_prefix + topic, data);
            }
        } catch (const mqtt::exception &e) {
            // client disconnected or something, we drop it.
        }
    }
}

void try_publish_binary(std::string topic, const void *data, size_t size, bool retain = false) {
    try {
        if (retain) {
            // QOS 1 so that the data actually arrives at the client at least once.
            client_->publish(topic, data, size, 1, true);
        } else {
            client_->publish(topic, data, size);
        }
    } catch (const mqtt::exception &e) {
        // client disconnected or something, we drop it.
    }
}

void mower_logic_settings_status_json_callback(const std_msgs::String::ConstPtr &msg) {
    try_publish("settings/mower_logic/json", msg->data, true);
}

void mower_logic_settings_validation_json_callback(const std_msgs::String::ConstPtr &msg) {
    try_publish("settings/mower_logic/validation/json", msg->data, true);
}

void mower_logic_satellite_logging_status_json_callback(const std_msgs::String::ConstPtr &msg) {
    try_publish("settings/mower_logic/satellite_logging/json", msg->data, true);
}

void load_factor_computed_callback(const std_msgs::Float32::ConstPtr &msg) {
    std::lock_guard<std::mutex> lk(load_factor_state_mutex);
    load_factor_computed_snapshot = static_cast<double>(msg->data);
}

void load_factor_effective_callback(const std_msgs::Float32::ConstPtr &msg) {
    std::lock_guard<std::mutex> lk(load_factor_state_mutex);
    load_factor_effective_snapshot = static_cast<double>(msg->data);
}

void ll_power_status_json_callback(const std_msgs::String::ConstPtr &msg) {
    try_publish("settings/ll_board/json", msg->data, true);
}

void publish_ll_power_status_request() {
    std_msgs::Empty msg;
    ll_power_renew_pub.publish(msg);
}

void publish_version() {
    json version = {
            {"version", version_string}
    };
    try_publish("version/json", version.dump(), true);
    auto bson = json::to_bson(version);
    try_publish_binary("version", bson.data(), bson.size(), true);
}

void publish_capabilities() {
  try_publish("capabilities/json", CAPABILITIES.dump(2), true);
}

#pragma GCC diagnostic push
#pragma GCC diagnostic warning "-Wswitch-enum"
json xmlrpc_to_json(XmlRpc::XmlRpcValue value) {
    switch (value.getType()) {
        case XmlRpc::XmlRpcValue::TypeBoolean:
            return static_cast<bool>(value);
        case XmlRpc::XmlRpcValue::TypeInt:
            return static_cast<int>(value);
        case XmlRpc::XmlRpcValue::TypeDouble:
            return static_cast<double>(value);
        case XmlRpc::XmlRpcValue::TypeString:
            return static_cast<std::string>(value);
        case XmlRpc::XmlRpcValue::TypeArray: {
            json arr = json::array();
            for (int i = 0; i < value.size(); ++i)
                arr.push_back(xmlrpc_to_json(value[i]));
            return arr;
        }
        case XmlRpc::XmlRpcValue::TypeStruct: {
            json obj = json::object();
            for (auto it = value.begin(); it != value.end(); ++it)
                obj[it->first] = xmlrpc_to_json(it->second);
            return obj;
        }
        case XmlRpc::XmlRpcValue::TypeDateTime: {
            const struct tm& t = static_cast<const struct tm&>(value);
            char buf[32];
            std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%S", &t);
            return std::string(buf);
        }
        case XmlRpc::XmlRpcValue::TypeBase64: {
            const XmlRpc::XmlRpcValue::BinaryData& data = static_cast<const XmlRpc::XmlRpcValue::BinaryData&>(value);
            static const char* b64 = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
            std::string out;
            out.reserve(((data.size() + 2) / 3) * 4);
            for (size_t i = 0; i < data.size(); i += 3) {
                unsigned int n = (static_cast<unsigned char>(data[i]) << 16)
                    | (i + 1 < data.size() ? static_cast<unsigned char>(data[i + 1]) << 8 : 0)
                    | (i + 2 < data.size() ? static_cast<unsigned char>(data[i + 2]) : 0);
                out += b64[(n >> 18) & 0x3F];
                out += b64[(n >> 12) & 0x3F];
                out += (i + 1 < data.size()) ? b64[(n >> 6) & 0x3F] : '=';
                out += (i + 2 < data.size()) ? b64[n & 0x3F] : '=';
            }
            return out;
        }
        case XmlRpc::XmlRpcValue::TypeInvalid:
            return nullptr;
    }
    return nullptr;
}
#pragma GCC diagnostic pop

void publish_params() {
    std::vector<std::string> param_names;
    ros::param::getParamNames(param_names);
    std::sort(param_names.begin(), param_names.end());

    json params = json::object();
    for (const auto &name : param_names) {
        if (name.find("password") != std::string::npos) {
            params[name] = nullptr;
            continue;
        }
        XmlRpc::XmlRpcValue value;
        if (ros::param::get(name, value)) {
            params[name] = xmlrpc_to_json(value);
        }
    }
    try_publish("params/json", params.dump(), true);
}

static std::string sensor_value_type_to_settings_type(const xbot_msgs::SensorInfo &info) {
    switch (info.value_type) {
        case xbot_msgs::SensorInfo::TYPE_STRING:
            return "string";
        case xbot_msgs::SensorInfo::TYPE_DOUBLE:
            return "number";
        default:
            return "string";
    }
}

static std::string sensor_value_type_to_legacy_string(const xbot_msgs::SensorInfo &info) {
    switch (info.value_type) {
        case xbot_msgs::SensorInfo::TYPE_STRING:
            return "STRING";
        case xbot_msgs::SensorInfo::TYPE_DOUBLE:
            return "DOUBLE";
        default:
            return "UNKNOWN";
    }
}

static std::string sensor_value_description_to_string(const xbot_msgs::SensorInfo &info) {
    switch (info.value_description) {
        case xbot_msgs::SensorInfo::VALUE_DESCRIPTION_TEMPERATURE:
            return "TEMPERATURE";
        case xbot_msgs::SensorInfo::VALUE_DESCRIPTION_VELOCITY:
            return "VELOCITY";
        case xbot_msgs::SensorInfo::VALUE_DESCRIPTION_ACCELERATION:
            return "ACCELERATION";
        case xbot_msgs::SensorInfo::VALUE_DESCRIPTION_VOLTAGE:
            return "VOLTAGE";
        case xbot_msgs::SensorInfo::VALUE_DESCRIPTION_CURRENT:
            return "CURRENT";
        case xbot_msgs::SensorInfo::VALUE_DESCRIPTION_PERCENT:
            return "PERCENT";
        case xbot_msgs::SensorInfo::VALUE_DESCRIPTION_RPM:
            return "REVOLUTIONS";
        default:
            return "UNKNOWN";
    }
}

static std::string normalize_sensor_infos_group(const std::string &raw_group) {
    std::string group = trim_settings_string(raw_group);
    if (group.empty() || group == "UNKNOWN") {
        return "general";
    }
    std::string normalized;
    normalized.reserve(group.size());
    bool previous_separator = false;
    for (const unsigned char c : group) {
        if (std::isalnum(c)) {
            normalized.push_back(static_cast<char>(std::tolower(c)));
            previous_separator = false;
        } else if (!previous_separator) {
            normalized.push_back('_');
            previous_separator = true;
        }
    }
    while (!normalized.empty() && normalized.front() == '_') normalized.erase(normalized.begin());
    while (!normalized.empty() && normalized.back() == '_') normalized.pop_back();
    return normalized.empty() ? "general" : normalized;
}

static json current_sensor_value_json(const xbot_msgs::SensorInfo &info) {
    if (info.value_type == xbot_msgs::SensorInfo::TYPE_DOUBLE) {
        std::lock_guard<std::mutex> lk(latest_double_sensor_values_mutex);
        const auto it = latest_double_sensor_values.find(info.sensor_id);
        return it == latest_double_sensor_values.end() ? json(nullptr) : json(it->second);
    }
    if (info.value_type == xbot_msgs::SensorInfo::TYPE_STRING) {
        std::lock_guard<std::mutex> lk(latest_string_sensor_values_mutex);
        const auto it = latest_string_sensor_values.find(info.sensor_id);
        return it == latest_string_sensor_values.end() ? json(nullptr) : json(it->second);
    }
    return nullptr;
}

static json sensor_info_to_settings_entry(const xbot_msgs::SensorInfo &info, int order) {
    const std::string group = normalize_sensor_infos_group(info.sensor_origin);
    const json active_value = current_sensor_value_json(info);

    json entry = json::object();
    entry["label"] = info.sensor_name.empty() ? info.sensor_id : info.sensor_name;
    entry["description"] = info.sensor_name.empty() ? info.sensor_id : info.sensor_name;
    entry["group"] = group;
    entry["order"] = order;
    entry["type"] = sensor_value_type_to_settings_type(info);
    entry["unit"] = info.unit;
    entry["value"] = active_value;
    entry["active"] = active_value;
    entry["persistent"] = nullptr;
    entry["visible"] = true;
    entry["expert"] = false;
    entry["readonly"] = true;
    entry["different"] = false;
    entry["restart_required"] = false;
    entry["session_apply_supported"] = false;

    entry["sensor_id"] = info.sensor_id;
    entry["sensor_name"] = info.sensor_name;
    entry["sensor_origin"] = info.sensor_origin.empty() ? "UNKNOWN" : info.sensor_origin;
    entry["value_type"] = sensor_value_type_to_legacy_string(info);
    entry["value_description"] = sensor_value_description_to_string(info);
    entry["value_topic"] = "sensors/" + info.sensor_id + "/data";
    entry["bson_topic"] = "sensors/" + info.sensor_id + "/bson";
    entry["has_min_max"] = info.has_min_max;
    entry["min"] = info.min_value;
    entry["max"] = info.max_value;
    entry["min_value"] = info.min_value;
    entry["max_value"] = info.max_value;
    entry["has_critical_low"] = info.has_critical_low;
    entry["lower_critical_value"] = info.lower_critical_value;
    entry["has_critical_high"] = info.has_critical_high;
    entry["upper_critical_value"] = info.upper_critical_value;
    return entry;
}

static void apply_sensor_infos_overrides(json &entry, const json &overrides) {
    if (!overrides.is_object()) return;
    static const std::set<std::string> override_fields = {
        "label", "description", "group", "order", "visible", "expert"
    };
    for (const auto &field : override_fields) {
        if (overrides.contains(field)) {
            entry[field] = overrides[field];
        }
    }
}

static std::string sensor_group_default_label(const std::string &group_id) {
    if (group_id == "host_system") return "Host-System";
    if (group_id == "openmower") return "OpenMower";
    if (group_id == "general") return "Allgemein";

    std::string label;
    label.reserve(group_id.size());
    bool next_upper = true;
    for (const char c : group_id) {
        if (c == '_' || c == '-') {
            label.push_back(' ');
            next_upper = true;
        } else if (next_upper) {
            label.push_back(static_cast<char>(std::toupper(static_cast<unsigned char>(c))));
            next_upper = false;
        } else {
            label.push_back(c);
        }
    }
    return label.empty() ? group_id : label;
}

static int sensor_group_default_order(const std::string &group_id) {
    if (group_id == "host_system") return 10;
    if (group_id == "openmower") return 20;
    if (group_id == "general") return 100;
    return 1000;
}

static json default_sensor_group_entry(const std::string &group_id, int fallback_order) {
    json entry = json::object();
    entry["label"] = sensor_group_default_label(group_id);
    const int preferred_order = sensor_group_default_order(group_id);
    entry["order"] = preferred_order == 1000 ? fallback_order : preferred_order;
    return entry;
}

static void apply_sensor_group_overrides(json &entry, const json &overrides) {
    if (!overrides.is_object()) return;
    if (overrides.contains("label")) entry["label"] = overrides["label"];
    if (overrides.contains("order")) entry["order"] = overrides["order"];
}

static json read_sensor_infos_persisted_namespace(const std::string &settings_persistent_path) {
    json current = open_mower_settings::readNamespace(settings_persistent_path, SENSOR_INFOS_NAMESPACE);
    json legacy = open_mower_settings::readNamespace(settings_persistent_path, "sensor_infos");

    json result = json::object();
    result["settings"] = json::object();
    result["groups"] = json::object();

    if (legacy.is_object()) {
        result["settings"] = legacy;
    }

    if (current.is_object()) {
        if (current.contains("settings") && current["settings"].is_object()) {
            for (auto it = current["settings"].begin(); it != current["settings"].end(); ++it) {
                result["settings"][it.key()] = it.value();
            }
        }
        if (current.contains("groups") && current["groups"].is_object()) {
            result["groups"] = current["groups"];
        }

        // Backward compatibility: old builds stored sensor ids directly below settings.sensors.<sensor_id>.
        for (auto it = current.begin(); it != current.end(); ++it) {
            if (it.key() == "settings" || it.key() == "groups" || it.key() == "schema") continue;
            if (it.value().is_object()) {
                result["settings"][it.key()] = it.value();
            }
        }
    }

    return result;
}

static bool validate_sensor_group_id(const std::string &raw_group_id, std::string &group_id, std::string &reason) {
    group_id = trim_settings_string(raw_group_id);
    if (group_id.empty()) {
        reason = "group id must not be empty";
        return false;
    }
    if (group_id.size() > 80) {
        reason = "group id must not be longer than 80 characters";
        return false;
    }
    for (const unsigned char c : group_id) {
        if (!(std::isalnum(c) || c == '_' || c == '-' || c == '.')) {
            reason = "group id may only contain letters, digits, underscore, hyphen or dot";
            return false;
        }
    }
    return true;
}

static bool validate_sensor_group_label_field(const json &value, std::string &normalized, std::string &reason) {
    if (!value.is_string()) {
        reason = "value must be a string";
        return false;
    }
    normalized = trim_settings_string(value.get<std::string>());
    if (normalized.size() > 120) {
        reason = "value must not be longer than 120 characters";
        return false;
    }
    return true;
}

static json build_sensor_infos_settings_payload() {
    json root = json::object();
    root["namespace"] = SENSOR_INFOS_NAMESPACE;
    root["schema"] = SENSOR_INFOS_SCHEMA;
    root["readonly"] = true;
    root["groups"] = json::object();
    root["settings"] = json::object();

    std::map<std::string, xbot_msgs::SensorInfo> sensors_by_id;
    {
        std::unique_lock<std::mutex> lk(found_sensors_mutex);
        for (const auto &kv : found_sensors) {
            if (!kv.second.sensor_id.empty()) {
                sensors_by_id[kv.second.sensor_id] = kv.second;
            }
        }
    }

    if (sensors_by_id.empty()) {
        return root;
    }

    std::string settings_persistent_path;
    ros::param::param<std::string>("/settings/persistent_file", settings_persistent_path,
                                   std::string("/data/ros/settings_persistent.json"));
    const json persisted = read_sensor_infos_persisted_namespace(settings_persistent_path);
    const json sensor_overrides = persisted.contains("settings") && persisted["settings"].is_object()
                                      ? persisted["settings"]
                                      : json::object();
    const json group_overrides = persisted.contains("groups") && persisted["groups"].is_object()
                                     ? persisted["groups"]
                                     : json::object();

    int order = 10;
    int next_group_order = 1000;
    for (const auto &kv : sensors_by_id) {
        json entry = sensor_info_to_settings_entry(kv.second, order);
        if (sensor_overrides.contains(kv.first)) {
            apply_sensor_infos_overrides(entry, sensor_overrides[kv.first]);
        }

        std::string group_id = "general";
        if (entry.contains("group") && entry["group"].is_string()) {
            std::string reason;
            std::string validated_group;
            if (validate_group_metadata_value(entry["group"], validated_group, reason)) {
                group_id = validated_group;
            }
        }
        entry["group"] = group_id;

        if (!root["groups"].contains(group_id)) {
            root["groups"][group_id] = default_sensor_group_entry(group_id, next_group_order);
            next_group_order += 10;
        }

        root["settings"][kv.first] = entry;
        order += 10;
    }

    for (auto group_it = group_overrides.begin(); group_it != group_overrides.end(); ++group_it) {
        std::string group_id;
        std::string reason;
        if (!validate_sensor_group_id(group_it.key(), group_id, reason)) {
            continue;
        }
        if (!root["groups"].contains(group_id)) {
            root["groups"][group_id] = default_sensor_group_entry(group_id, next_group_order);
            next_group_order += 10;
        }
        apply_sensor_group_overrides(root["groups"][group_id], group_it.value());
    }

    return root;
}

void publish_sensor_infos_validation(const json &validation) {
    try_publish("sensors/settings/validation/json", validation.dump(), true);
}

static bool validate_sensor_infos_label_field(const json &value, std::string &normalized, std::string &reason) {
    if (!value.is_string()) {
        reason = "value must be a string";
        return false;
    }
    normalized = trim_settings_string(value.get<std::string>());
    if (normalized.size() > 120) {
        reason = "value must not be longer than 120 characters";
        return false;
    }
    return true;
}

static bool validate_sensor_infos_description_field(const json &value, std::string &normalized, std::string &reason) {
    if (!value.is_string()) {
        reason = "value must be a string";
        return false;
    }
    normalized = trim_settings_string(value.get<std::string>());
    if (normalized.size() > 500) {
        reason = "value must not be longer than 500 characters";
        return false;
    }
    return true;
}

static bool validate_sensor_infos_order_field(const json &value, int &order, std::string &reason) {
    if (!value.is_number_integer()) {
        reason = "order must be an integer";
        return false;
    }
    order = value.get<int>();
    if (order < -100000 || order > 100000) {
        reason = "order must be between -100000 and 100000";
        return false;
    }
    return true;
}

static bool validate_sensor_infos_bool_field(const json &value, bool &flag, std::string &reason) {
    if (!value.is_boolean()) {
        reason = "value must be a boolean";
        return false;
    }
    flag = value.get<bool>();
    return true;
}

void handle_sensor_infos_persistent_payload(const std::string &payload_text) {
    json validation = {
        {"valid", false},
        {"namespace", SENSOR_INFOS_NAMESPACE},
        {"mode", "persistent"},
        {"accepted", json::object()},
        {"rejected", json::object()},
        {"remarks", json::array()}
    };

    try {
        const json payload = json::parse(payload_text.empty() ? "{}" : payload_text);
        if (!payload.is_object()) {
            validation["rejected"]["$"] = {{"reason", "payload must be a JSON object"}};
            publish_sensor_infos_validation(validation);
            return;
        }

        const bool wrapped_payload = payload.contains("settings") || payload.contains("groups");
        static const std::set<std::string> allowed_top_level_fields = {
            "namespace", "schema", "readonly", "settings", "groups"
        };
        if (wrapped_payload) {
            for (auto it = payload.begin(); it != payload.end(); ++it) {
                if (allowed_top_level_fields.count(it.key()) == 0) {
                    validation["rejected"][it.key()] = {{"reason", "unknown top-level field"}};
                }
            }
        }

        json settings_payload = json::object();
        json groups_payload = json::object();
        if (wrapped_payload) {
            if (payload.contains("settings")) {
                if (!payload["settings"].is_object()) {
                    validation["rejected"]["settings"] = {{"reason", "settings must be an object"}};
                } else {
                    settings_payload = payload["settings"];
                }
            }
            if (payload.contains("groups")) {
                if (!payload["groups"].is_object()) {
                    validation["rejected"]["groups"] = {{"reason", "groups must be an object"}};
                } else {
                    groups_payload = payload["groups"];
                }
            }
        } else {
            settings_payload = payload;
            validation["remarks"].push_back("legacy flat sensors payload accepted; it will be migrated to sensors.settings");
        }

        if (!validation["rejected"].empty()) {
            publish_sensor_infos_validation(validation);
            return;
        }

        std::map<std::string, xbot_msgs::SensorInfo> sensors_by_id;
        {
            std::unique_lock<std::mutex> lk(found_sensors_mutex);
            for (const auto &kv : found_sensors) {
                if (!kv.second.sensor_id.empty()) {
                    sensors_by_id[kv.second.sensor_id] = kv.second;
                }
            }
        }

        std::map<std::string, std::map<std::string, open_mower_settings::json>> accepted_sensor_updates;
        json accepted_group_updates = json::object();

        for (auto sensor_it = settings_payload.begin(); sensor_it != settings_payload.end(); ++sensor_it) {
            const std::string sensor_id = sensor_it.key();
            if (sensors_by_id.count(sensor_id) == 0) {
                validation["rejected"]["settings"][sensor_id] = {{"reason", "unknown sensor id"}};
                continue;
            }
            if (!sensor_it.value().is_object()) {
                validation["rejected"]["settings"][sensor_id] = {{"reason", "sensor entry must be an object"}};
                continue;
            }

            json rejected_fields = json::object();
            std::map<std::string, open_mower_settings::json> entry_updates;
            for (auto field_it = sensor_it.value().begin(); field_it != sensor_it.value().end(); ++field_it) {
                const std::string field = field_it.key();
                std::string reason;
                if (field == "label") {
                    std::string value;
                    if (validate_sensor_infos_label_field(field_it.value(), value, reason)) {
                        entry_updates[field] = value;
                    } else {
                        rejected_fields[field] = reason;
                    }
                } else if (field == "description") {
                    std::string value;
                    if (validate_sensor_infos_description_field(field_it.value(), value, reason)) {
                        entry_updates[field] = value;
                    } else {
                        rejected_fields[field] = reason;
                    }
                } else if (field == "group") {
                    std::string value;
                    if (validate_group_metadata_value(field_it.value(), value, reason)) {
                        entry_updates[field] = value;
                        if (!accepted_group_updates.contains(value)) {
                            accepted_group_updates[value] = default_sensor_group_entry(value, 1000 + static_cast<int>(accepted_group_updates.size()) * 10);
                        }
                    } else {
                        rejected_fields[field] = reason;
                    }
                } else if (field == "order") {
                    int value = 0;
                    if (validate_sensor_infos_order_field(field_it.value(), value, reason)) {
                        entry_updates[field] = value;
                    } else {
                        rejected_fields[field] = reason;
                    }
                } else if (field == "visible" || field == "expert") {
                    bool value = false;
                    if (validate_sensor_infos_bool_field(field_it.value(), value, reason)) {
                        entry_updates[field] = value;
                    } else {
                        rejected_fields[field] = reason;
                    }
                } else {
                    rejected_fields[field] = "field is readonly or unknown";
                }
            }

            if (!rejected_fields.empty()) {
                validation["rejected"]["settings"][sensor_id] = rejected_fields;
            }
            if (!entry_updates.empty()) {
                accepted_sensor_updates[sensor_id] = entry_updates;
            }
        }

        for (auto group_it = groups_payload.begin(); group_it != groups_payload.end(); ++group_it) {
            std::string group_id;
            std::string reason;
            if (!validate_sensor_group_id(group_it.key(), group_id, reason)) {
                validation["rejected"]["groups"][group_it.key()] = {{"reason", reason}};
                continue;
            }
            if (!group_it.value().is_object()) {
                validation["rejected"]["groups"][group_it.key()] = {{"reason", "group entry must be an object"}};
                continue;
            }

            json group_update = json::object();
            json rejected_fields = json::object();
            for (auto field_it = group_it.value().begin(); field_it != group_it.value().end(); ++field_it) {
                const std::string field = field_it.key();
                if (field == "label") {
                    std::string value;
                    if (validate_sensor_group_label_field(field_it.value(), value, reason)) {
                        group_update[field] = value;
                    } else {
                        rejected_fields[field] = reason;
                    }
                } else if (field == "order") {
                    int value = 0;
                    if (validate_sensor_infos_order_field(field_it.value(), value, reason)) {
                        group_update[field] = value;
                    } else {
                        rejected_fields[field] = reason;
                    }
                } else {
                    rejected_fields[field] = "field is readonly or unknown";
                }
            }

            if (!rejected_fields.empty()) {
                validation["rejected"]["groups"][group_id] = rejected_fields;
            }
            if (!group_update.empty()) {
                if (!accepted_group_updates.contains(group_id)) {
                    accepted_group_updates[group_id] = default_sensor_group_entry(group_id, 1000 + static_cast<int>(accepted_group_updates.size()) * 10);
                }
                for (auto update_it = group_update.begin(); update_it != group_update.end(); ++update_it) {
                    accepted_group_updates[group_id][update_it.key()] = update_it.value();
                }
            }
        }

        if (accepted_sensor_updates.empty() && accepted_group_updates.empty() && validation["rejected"].empty()) {
            validation["rejected"]["$"] = {{"reason", "payload does not contain any sensor settings or group metadata changes"}};
        }

        if (validation["rejected"].empty() && (!accepted_sensor_updates.empty() || !accepted_group_updates.empty())) {
            std::string settings_persistent_path;
            ros::param::param<std::string>("/settings/persistent_file", settings_persistent_path,
                                           std::string("/data/ros/settings_persistent.json"));

            json persisted = read_sensor_infos_persisted_namespace(settings_persistent_path);
            if (!persisted.contains("settings") || !persisted["settings"].is_object()) persisted["settings"] = json::object();
            if (!persisted.contains("groups") || !persisted["groups"].is_object()) persisted["groups"] = json::object();

            for (const auto &entry : accepted_sensor_updates) {
                if (!persisted["settings"].contains(entry.first) || !persisted["settings"][entry.first].is_object()) {
                    persisted["settings"][entry.first] = json::object();
                }
                json fields = json::array();
                for (const auto &field : entry.second) {
                    persisted["settings"][entry.first][field.first] = field.second;
                    fields.push_back(field.first);
                }
                validation["accepted"]["settings"][entry.first] = fields;
            }

            for (auto group_it = accepted_group_updates.begin(); group_it != accepted_group_updates.end(); ++group_it) {
                if (!persisted["groups"].contains(group_it.key()) || !persisted["groups"][group_it.key()].is_object()) {
                    persisted["groups"][group_it.key()] = default_sensor_group_entry(group_it.key(), 1000);
                }
                json fields = json::array();
                for (auto field_it = group_it.value().begin(); field_it != group_it.value().end(); ++field_it) {
                    persisted["groups"][group_it.key()][field_it.key()] = field_it.value();
                    fields.push_back(field_it.key());
                }
                validation["accepted"]["groups"][group_it.key()] = fields;
            }

            if (!open_mower_settings::updateNamespace(settings_persistent_path, SENSOR_INFOS_NAMESPACE, persisted)) {
                validation["accepted"] = json::object();
                validation["rejected"]["$"] = {{"reason", "could not write settings_persistent.json"}};
            }
        }
    } catch (const json::exception &e) {
        validation["rejected"]["$"] = {{"reason", std::string("Error decoding JSON: ") + e.what()}};
        ROS_WARN_STREAM("Error decoding sensors/settings persistent JSON: " << e.what());
    }

    validation["valid"] = !validation["accepted"].empty() && validation["rejected"].empty();
    publish_sensor_infos_validation(validation);
    if (validation["valid"].get<bool>()) {
        publish_sensor_metadata();
    }
}

void publish_sensor_metadata() {
    json sensor_info = build_sensor_infos_settings_payload();
    if (sensor_info["settings"].empty()) {
        return;
    }

    try_publish("sensors/settings/json", sensor_info.dump(), true);
    json data;
    data["d"] = sensor_info;
    auto bson = json::to_bson(data);
    try_publish_binary("sensors/settings/bson", bson.data(), bson.size(), true);
}

void subscribe_to_sensor(std::string topic, std::vector<ros::Subscriber> &sensor_data_subscribers) {
    xbot_msgs::SensorInfo sensor;
    {
        std::unique_lock<std::mutex> lk(found_sensors_mutex);
        sensor = found_sensors[topic];
    }

    ROS_INFO_STREAM("Subscribing to sensor data for sensor with name: " << sensor.sensor_name);

    std::string data_topic = "xbot_monitoring/sensors/" + sensor.sensor_id + "/data";

    switch (sensor.value_type) {
        case xbot_msgs::SensorInfo::TYPE_DOUBLE: {
            ros::Subscriber s = n->subscribe<xbot_msgs::SensorDataDouble>(data_topic, 10, [info = sensor](
                    const xbot_msgs::SensorDataDouble::ConstPtr &msg) {
                try_publish("sensors/" + info.sensor_id + "/data", std::to_string(msg->data), true);
                {
                    std::lock_guard<std::mutex> lk(latest_double_sensor_values_mutex);
                    latest_double_sensor_values[info.sensor_id] = msg->data;
                }

                json data;
                data["d"] = msg->data;
                auto bson = json::to_bson(data);
                try_publish_binary("sensors/" + info.sensor_id + "/bson", bson.data(), bson.size(), true);
            });
            sensor_data_subscribers.push_back(s);
            break;
        }
        case xbot_msgs::SensorInfo::TYPE_STRING: {
            ros::Subscriber s = n->subscribe<xbot_msgs::SensorDataString>(data_topic, 10, [info = sensor](
                    const xbot_msgs::SensorDataString::ConstPtr &msg) {
                try_publish("sensors/" + info.sensor_id + "/data", msg->data, true);
                {
                    std::lock_guard<std::mutex> lk(latest_string_sensor_values_mutex);
                    latest_string_sensor_values[info.sensor_id] = msg->data;
                }

                json data;
                data["d"] = msg->data;
                auto bson = json::to_bson(data);
                try_publish_binary("sensors/" + info.sensor_id + "/bson", bson.data(), bson.size(), true);
            });
            sensor_data_subscribers.push_back(s);
            break;
        }
        default: {
            ROS_ERROR_STREAM("Invalid Sensor Data Type: " << (int) sensor.value_type);
        }
    }
}

std::string utc_timestamp_iso8601(const std::chrono::system_clock::time_point &time_point) {
    const auto time = std::chrono::system_clock::to_time_t(time_point);
    const auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(
        time_point.time_since_epoch()) % 1000;
    std::tm tm_utc{};
#if defined(_WIN32)
    gmtime_s(&tm_utc, &time);
#else
    gmtime_r(&time, &tm_utc);
#endif
    std::ostringstream out;
    out << std::put_time(&tm_utc, "%Y-%m-%dT%H:%M:%S")
        << '.' << std::setfill('0') << std::setw(3) << milliseconds.count() << 'Z';
    return out.str();
}

bool try_parse_utc_timestamp_iso8601(const std::string &timestamp,
                                     std::chrono::system_clock::time_point &time_point) {
    if (timestamp.size() < 20) {
        return false;
    }

    std::tm tm_utc{};
    std::istringstream date_part(timestamp.substr(0, 19));
    date_part >> std::get_time(&tm_utc, "%Y-%m-%dT%H:%M:%S");
    if (date_part.fail()) {
        return false;
    }

    long milliseconds = 0;
    if (timestamp.size() >= 24 && timestamp[19] == '.') {
        const std::string milliseconds_part = timestamp.substr(20, 3);
        if (milliseconds_part.find_first_not_of("0123456789") != std::string::npos) {
            return false;
        }
        milliseconds = std::stol(milliseconds_part);
    }

#if defined(_WIN32)
    const std::time_t seconds_since_epoch = _mkgmtime(&tm_utc);
#else
    const std::time_t seconds_since_epoch = timegm(&tm_utc);
#endif
    if (seconds_since_epoch == static_cast<std::time_t>(-1)) {
        return false;
    }

    time_point = std::chrono::system_clock::from_time_t(seconds_since_epoch) +
                 std::chrono::milliseconds(milliseconds);
    return true;
}

void load_statustransition_log_if_needed_locked() {
    if (statustransition_log_loaded) {
        return;
    }
    statustransition_log_loaded = true;
    statustransition_log_entries = json::array();

    try {
        std::ifstream in(statustransition_log_file);
        if (!in.good()) {
            return;
        }
        json existing = json::parse(in, nullptr, false);
        if (existing.is_object() && existing.contains("entries") && existing["entries"].is_array()) {
            statustransition_log_entries = existing["entries"];
        } else if (existing.is_array()) {
            statustransition_log_entries = existing;
        }
        while (statustransition_log_entries.size() > STATUSTRANSITION_LOG_MAX_ENTRIES) {
            statustransition_log_entries.erase(statustransition_log_entries.begin());
        }
        if (!statustransition_log_entries.empty()) {
            const auto &latest_entry = statustransition_log_entries.back();
            if (latest_entry.contains("timestamp") && latest_entry["timestamp"].is_string()) {
                has_last_statustransition_timestamp = try_parse_utc_timestamp_iso8601(
                    latest_entry["timestamp"].get<std::string>(),
                    last_statustransition_timestamp);
            }
        }
    } catch (const std::exception &e) {
        ROS_WARN_STREAM("Unable to load statustransition log '" << statustransition_log_file
                        << "': " << e.what());
        statustransition_log_entries = json::array();
    }
}

void persist_statustransition_log_locked() {
    try {
        const std::filesystem::path log_path(statustransition_log_file);
        const auto parent = log_path.parent_path();
        if (!parent.empty()) {
            std::filesystem::create_directories(parent);
        }
        const auto temp_path = log_path.string() + ".tmp";
        json root;
        root["max_entries"] = STATUSTRANSITION_LOG_MAX_ENTRIES;
        root["entries"] = statustransition_log_entries;
        {
            std::ofstream out(temp_path, std::ios::trunc);
            if (!out.good()) {
                ROS_WARN_STREAM("Unable to write statustransition log temp file '" << temp_path << "'.");
                return;
            }
            out << root.dump(2) << std::endl;
        }
        std::filesystem::rename(temp_path, log_path);
    } catch (const std::exception &e) {
        ROS_WARN_STREAM("Unable to persist statustransition log '" << statustransition_log_file
                        << "': " << e.what());
    }
}

std::size_t normalize_statustransition_log_limit(std::size_t requested_limit, std::size_t available_entries) {
    if (available_entries == 0) {
        return 0;
    }

    std::size_t effective_limit = requested_limit;
    if (effective_limit == 0) {
        effective_limit = mqtt_statustransition_log_default_limit;
    }
    if (effective_limit == 0) {
        effective_limit = available_entries;
    }

    effective_limit = std::min(effective_limit, STATUSTRANSITION_LOG_MAX_ENTRIES);
    effective_limit = std::min(effective_limit, available_entries);
    return effective_limit;
}

void publish_statustransition_log(std::size_t requested_limit) {
    json payload = json::object();
    {
        std::lock_guard<std::mutex> lk(statustransition_log_mutex);
        load_statustransition_log_if_needed_locked();

        json snapshot = statustransition_log_entries;
        if (!snapshot.empty() && has_last_statustransition_timestamp) {
            const auto current_timestamp = std::chrono::system_clock::now();
            const auto active_status_duration = std::chrono::duration<double>(
                current_timestamp - last_statustransition_timestamp).count();
            snapshot.back()["duration_seconds"] = std::max(0.0, active_status_duration);
            snapshot.back()["duration_is_current"] = true;
        }

        const std::size_t total_entries = snapshot.size();
        const std::size_t effective_limit = normalize_statustransition_log_limit(requested_limit, total_entries);
        json selected_entries = json::array();
        if (effective_limit > 0) {
            const std::size_t first_index = total_entries - effective_limit;
            for (std::size_t index = first_index; index < total_entries; ++index) {
                selected_entries.push_back(snapshot[index]);
            }
        }

        payload["total_entries"] = total_entries;
        payload["returned_entries"] = selected_entries.size();
        payload["limit"] = effective_limit;
        payload["entries"] = std::move(selected_entries);
    }

    // Retained resource payload for status log pages and external consumers.
    try_publish("statustransition_log/json", payload.dump(2), true);
}

json current_temperature_snapshot() {
    json temperatures = json::object();
    const std::vector<std::string> sensor_ids = {
        "om_left_esc_temp",
        "om_right_esc_temp",
        "om_mow_esc_temp",
        "om_mow_motor_temp"
    };
    std::lock_guard<std::mutex> lk(latest_double_sensor_values_mutex);
    for (const auto &sensor_id : sensor_ids) {
        auto it = latest_double_sensor_values.find(sensor_id);
        if (it != latest_double_sensor_values.end()) {
            temperatures[sensor_id] = it->second;
        } else {
            temperatures[sensor_id] = nullptr;
        }
    }
    return temperatures;
}

json current_mow_motor_direction_snapshot() {
    std::lock_guard<std::mutex> lk(latest_double_sensor_values_mutex);
    auto it = latest_double_sensor_values.find("om_mow_motor_direction");
    if (it == latest_double_sensor_values.end()) {
        return nullptr;
    }

    const double raw_direction = it->second;
    if (raw_direction > 0.5) {
        return 1;
    }
    if (raw_direction < -0.5) {
        return -1;
    }
    return 0;
}

void maybe_append_statustransition_log(const xbot_msgs::RobotState::ConstPtr &msg) {
    std::lock_guard<std::mutex> lk(statustransition_log_mutex);
    load_statustransition_log_if_needed_locked();

    const bool is_transition = !has_last_statustransition_key ||
                               last_statustransition_state != msg->current_state ||
                               last_statustransition_sub_state != msg->current_sub_state ||
                               last_statustransition_charging != msg->is_charging ||
                               last_statustransition_emergency != msg->emergency;

    if (!is_transition) {
        return;
    }

    const auto transition_timestamp = std::chrono::system_clock::now();
    if (!statustransition_log_entries.empty() && has_last_statustransition_timestamp) {
        const auto previous_status_duration = std::chrono::duration<double>(
            transition_timestamp - last_statustransition_timestamp).count();
        statustransition_log_entries.back()["duration_seconds"] =
            std::max(0.0, previous_status_duration);
    }

    json entry;
    entry["timestamp"] = utc_timestamp_iso8601(transition_timestamp);
    entry["duration_seconds"] = 0.0;
    entry["state"] = msg->current_state;
    entry["sub_state"] = msg->current_sub_state;
    entry["previous_state"] = has_last_statustransition_key ? json(last_statustransition_state) : json(nullptr);
    entry["previous_sub_state"] = has_last_statustransition_key ? json(last_statustransition_sub_state) : json(nullptr);
    entry["battery_percentage"] = msg->battery_percentage;
    entry["gps_percentage"] = msg->gps_percentage;
    entry["is_charging"] = msg->is_charging;
    entry["emergency"] = msg->emergency;
    {
        std::lock_guard<std::mutex> timetable_lk(timetable_mutex);
        entry["automow"] = timetable_auto_mowing_time;
        entry["automow_id"] = timetable_auto_mow_id;
    }
    entry["current_area_id"] = msg->current_area_id;
    entry["checkpoint_area_id"] = msg->checkpoint_area_id;
    entry["position"]["x"] = msg->robot_pose.pose.pose.position.x;
    entry["position"]["y"] = msg->robot_pose.pose.pose.position.y;
    entry["position"]["heading"] = msg->robot_pose.vehicle_heading;
    entry["position"]["pos_accuracy"] = msg->robot_pose.position_accuracy;
    entry["position"]["heading_accuracy"] = msg->robot_pose.orientation_accuracy;
    entry["position"]["heading_valid"] = msg->robot_pose.orientation_valid;
    entry["temperatures"] = current_temperature_snapshot();
    entry["mow_motor_direction"] = current_mow_motor_direction_snapshot();

    statustransition_log_entries.push_back(entry);
    while (statustransition_log_entries.size() > STATUSTRANSITION_LOG_MAX_ENTRIES) {
        statustransition_log_entries.erase(statustransition_log_entries.begin());
    }
    persist_statustransition_log_locked();

    last_statustransition_state = msg->current_state;
    last_statustransition_sub_state = msg->current_sub_state;
    last_statustransition_charging = msg->is_charging;
    last_statustransition_emergency = msg->emergency;
    last_statustransition_timestamp = transition_timestamp;
    has_last_statustransition_timestamp = true;
    has_last_statustransition_key = true;
}

void robot_state_callback(const xbot_msgs::RobotState::ConstPtr &msg) {
    // Build a JSON and publish it
    json j;

    j["battery_percentage"] = msg->battery_percentage;
    j["gps_percentage"] = msg->gps_percentage;
    j["current_action_progress"] = msg->current_action_progress;
    j["current_state"] = msg->current_state;
    j["current_sub_state"] = msg->current_sub_state;
    j["current_area"] = msg->current_area;
    j["current_area_id"] = msg->current_area_id;
    j["checkpoint_area_id"] = msg->checkpoint_area_id;
    j["current_path"] = msg->current_path;
    j["current_path_index"] = msg->current_path_index;
    j["emergency"] = msg->emergency;
    j["is_charging"] = msg->is_charging;
    {
        std::lock_guard<std::mutex> lk(timetable_mutex);
        j["AutoMow"] = timetable_auto_mowing_time ? 1 : 0;
        j["AutoMowID"] = timetable_auto_mow_id;
        j["AutoMowSuspension"] = timetable_auto_mow_suspension;
    }
    j["rain_detected"] = msg->rain_detected;
    j["pose"]["x"] = msg->robot_pose.pose.pose.position.x;
    j["pose"]["y"] = msg->robot_pose.pose.pose.position.y;
    j["pose"]["heading"] = msg->robot_pose.vehicle_heading;
    j["pose"]["pos_accuracy"] = msg->robot_pose.position_accuracy;
    j["pose"]["heading_accuracy"] = msg->robot_pose.orientation_accuracy;
    j["pose"]["heading_valid"] = msg->robot_pose.orientation_valid;

    const auto world_pose = convert_robot_pose_to_world_pose(msg->robot_pose);
    j["world_pose"]["valid"] = world_pose.valid;
    j["world_pose"]["coordinate_system"] = "WGS84";
    j["world_pose"]["source"] = "robot_pose_to_wgs84";
    if (world_pose.valid) {
        j["world_pose"]["latitude"] = world_pose.latitude;
        j["world_pose"]["longitude"] = world_pose.longitude;
        j["world_pose"]["altitude"] = world_pose.altitude;
        j["world_pose"]["pos_accuracy"] = msg->robot_pose.position_accuracy;
    } else {
        j["world_pose"]["reason"] = world_pose.reason;
    }

    {
        std::lock_guard<std::mutex> lk(load_factor_state_mutex);
        j["load_factor_computed"] = load_factor_computed_snapshot;
        j["load_factor_effective"] = load_factor_effective_snapshot;
    }

    maybe_append_statustransition_log(msg);

    try_publish("robot_state/json", j.dump());
    json data;
    data["d"] = j;
    auto bson = json::to_bson(data);
    try_publish_binary("robot_state/bson", bson.data(), bson.size());
}

void mowing_progress_callback(const std_msgs::String::ConstPtr &msg) {
    publish_mowing_progress(msg->data);
}

void mowing_progress_status_callback(const std_msgs::String::ConstPtr &msg) {
    publish_mowing_progress_status(msg->data);
}

void publish_actions() {
    json actions = json::array();
    {
        std::lock_guard<std::mutex> lk(registered_actions_mutex);
        for(const auto &kv : registered_actions) {
            for(const auto &action : kv.second) {
                json action_info;
                action_info["action_id"] = kv.first + "/" + action.action_id;
                action_info["action_name"] = action.action_name;
                action_info["enabled"] = action.enabled;
                actions.push_back(action_info);
            }
        }
    }

    try_publish("actions/json", actions.dump(), true);
    json data;
    data["d"] = actions;

    auto bson = json::to_bson(data);
    try_publish_binary("actions/bson", bson.data(), bson.size(), true);
}

void publish_map() {
    json m;
    {
        std::lock_guard<std::mutex> lk(map_mutex);
        if(!has_map)
            return;
        m = map;
    }
    try_publish("map/json", m.dump(2), true);
    json data;
    data["d"] = m;
    auto bson = json::to_bson(data);
    try_publish_binary("map/bson", bson.data(), bson.size(), true);
}

void publish_map_validation(const json &validation) {
    try_publish("map/validation/json", validation.dump(2), true);
}

void publish_settings_validation(const std::string &settings_namespace, const json &validation) {
    try_publish("settings/" + settings_namespace + "/validation/json", validation.dump(2), true);
}

json validate_map_payload_for_mqtt(const json &payload) {
    json remarks = json::array();

    if (!payload.is_object()) {
        return {{"valid", false}, {"remarks", {"Map payload must be a JSON object"}}};
    }

    if (!payload.contains("areas") || !payload["areas"].is_array()) {
        return {{"valid", false}, {"remarks", {"Map payload must contain an areas array"}}};
    }

    std::set<int> used_orders;
    for (const auto &area : payload["areas"]) {
        if (!area.is_object()) {
            remarks.push_back("Each area must be a JSON object");
            continue;
        }

        const std::string area_id = area.value("id", std::string("<unknown>"));
        json properties = area.value("properties", json::object());
        const std::string type = properties.value("type", std::string("draft"));

        if (type != "mow") {
            continue;
        }

        if (!properties.contains("mowing_order") || !properties["mowing_order"].is_number_integer()) {
            remarks.push_back("Mowing area " + area_id + " has missing or non-integer mowing_order");
            continue;
        }

        const int order = properties["mowing_order"].get<int>();
        if (order < 1 || order > 99) {
            remarks.push_back("Mowing area " + area_id + " has invalid mowing_order " + std::to_string(order) + " (allowed: 1-99)");
            continue;
        }

        if (!used_orders.insert(order).second) {
            char buf[3];
            std::snprintf(buf, sizeof(buf), "%02d", order);
            remarks.push_back(std::string("Mowing order ") + buf + " is used more than once");
        }
    }

    if (!remarks.empty()) {
        return {{"valid", false}, {"remarks", remarks}};
    }

    return {{"valid", true}, {"remarks", {"Map payload accepted"}}};
}

void publish_map_overlay() {
    json m;
    {
        std::lock_guard<std::mutex> lk(map_overlay_mutex);
        if(!has_map_overlay)
            return;
        m = map_overlay;
    }
    // Canonical map overlay topics. The legacy map_overlay/* aliases stay during the transition period.
    try_publish("map/overlay/json", m.dump(), true);
    try_publish("map_overlay/json", m.dump(), true);
    json data;
    data["d"] = m;
    auto bson = json::to_bson(data);
    try_publish_binary("map/overlay/bson", bson.data(), bson.size(), true);
    try_publish_binary("map_overlay/bson", bson.data(), bson.size(), true);
}

void publish_mowing_progress(const std::string &payload) {
    // Retained heavy map overlay payload for graphical mowing progress per area.
    // This contains path geometry and is intentionally published at a low rate by mower_logic.
    try_publish("map/mowing_progress/json", payload, true);
}

void publish_mowing_progress_status(const std::string &payload) {
    // Retained lightweight progress status. Contains percent/current path, but no planned_paths/mowed_paths geometry.
    // Apps should use this for frequent progress text updates and robot_state/json for the live pose.
    try_publish("map/mowing_progress/status/json", payload, true);
}

void publish_timetable_validation(const json &validation) {
    try_publish("timetable/validation/json", validation.dump(2), true);
    json data;
    data["d"] = validation;
    auto bson = json::to_bson(data);
    try_publish_binary("timetable/validation/bson", bson.data(), bson.size(), true);
}

void publish_timetable() {
    json confirmed;
    {
        std::lock_guard<std::mutex> lk(timetable_mutex);
        if(!has_timetable)
            return;
        confirmed = timetable_confirmed;
        last_timetable_publish_time = ros::Time::now();
    }

    // Confirmed timetable payload: same values as timetable.json / incoming MQTT payload.
    // This is the resource state and is retained. It is sent on boot, renew, reload/change,
    // and optionally in a slow heartbeat. There is intentionally no timetable_state topic.
    try_publish("timetable/json", confirmed.dump(2), true);
    json timetable_data;
    timetable_data["d"] = confirmed;
    auto timetable_bson = json::to_bson(timetable_data);
    try_publish_binary("timetable/bson", timetable_bson.data(), timetable_bson.size(), true);
}

void maybe_publish_timetable(bool force) {
    if (force) {
        publish_timetable();
        return;
    }

    if (mqtt_timetable_publish_interval_sec <= 0.0) {
        return;
    }

    if (last_timetable_publish_time.isZero() ||
        (ros::Time::now() - last_timetable_publish_time).toSec() >= mqtt_timetable_publish_interval_sec) {
        publish_timetable();
    }
}

void timetable_status_callback(const std_msgs::String::ConstPtr &msg) {
    try {
        json status = json::parse(msg->data);
        json confirmed = json::object();
        bool auto_mowing_time = false;
        std::string auto_mow_id;
        json auto_mow_suspension = 0;
        bool timetable_changed = false;

        if (status.is_object() && status.contains("timetable") && !status["timetable"].is_null()) {
            confirmed = status["timetable"];
        }

        if (status.is_object()) {
            json robot_state = json::object();
            if (status.contains("robot_state") && status["robot_state"].is_object()) {
                robot_state = status["robot_state"];
            } else if (status.contains("state") && status["state"].is_object()) {
                // Backwards compatibility with older timetable_service status field name.
                robot_state = status["state"];
            }

            if (robot_state.is_object()) {
                if (robot_state.contains("AutoMow")) {
                    auto_mowing_time = robot_state.value("AutoMow", 0) == 1;
                } else {
                    auto_mowing_time = robot_state.value("auto_mowing_time", false);
                }
                if (robot_state.contains("AutoMowID") && robot_state["AutoMowID"].is_string()) {
                    auto_mow_id = robot_state["AutoMowID"].get<std::string>();
                } else if (robot_state.contains("active_entry_id") && robot_state["active_entry_id"].is_string()) {
                    auto_mow_id = robot_state["active_entry_id"].get<std::string>();
                }
                if (robot_state.contains("AutoMowSuspension")) {
                    auto_mow_suspension = robot_state["AutoMowSuspension"];
                } else if (robot_state.contains("suspended_until") && robot_state["suspended_until"].is_string()) {
                    auto_mow_suspension = robot_state["suspended_until"];
                }
            }
        }

        if (!auto_mowing_time) {
            auto_mow_id.clear();
        }

        {
            std::lock_guard<std::mutex> lk(timetable_mutex);
            timetable_changed = !has_timetable || timetable_confirmed != confirmed;
            timetable_status = status;
            timetable_confirmed = confirmed;
            timetable_auto_mowing_time = auto_mowing_time;
            timetable_auto_mow_id = auto_mow_id;
            timetable_auto_mow_suspension = auto_mow_suspension;
            has_timetable = true;
        }

        // Publish the timetable resource only when it appears/changes. A slow heartbeat is
        // handled in the main loop, and app requests use timetable/set/renew/json or /bson.
        if (timetable_changed) {
            publish_timetable();
        }
    } catch (const json::exception &e) {
        ROS_ERROR_STREAM("Error processing timetable status JSON: " << e.what());
    }
}

void map_callback(const std_msgs::String::ConstPtr &msg) {
    try {
        json m = json::parse(msg->data);
        {
            std::lock_guard<std::mutex> lk(map_mutex);
            map = m;
            has_map = true;
        }
        publish_map();
    } catch (const json::exception &e) {
        ROS_ERROR_STREAM("Error processing map JSON: " << e.what());
    }
}


void map_overlay_callback(const xbot_msgs::MapOverlay::ConstPtr &msg) {
    // Build a JSON and publish it

    json polys;
    for(const auto &poly : msg->polygons) {
        if(poly.polygon.points.size() < 2)
            continue;
        json poly_j;
        {
            json outline_poly_j;
            for (const auto &pt: poly.polygon.points) {
                json p_j;
                p_j["x"] = pt.x;
                p_j["y"] = pt.y;
                outline_poly_j.push_back(p_j);
            }
            poly_j["poly"] = outline_poly_j;
            poly_j["is_closed"] = poly.closed;
            poly_j["line_width"] = poly.line_width;
            poly_j["color"] = poly.color;
        }
        polys.push_back(poly_j);
    }

    json j;
    j["polygons"] = polys;
    {
        std::lock_guard<std::mutex> lk(map_overlay_mutex);
        map_overlay = j;
        has_map_overlay = true;
    }

    publish_map_overlay();
}


bool registerActions(xbot_msgs::RegisterActionsSrvRequest &req, xbot_msgs::RegisterActionsSrvResponse &res) {

    ROS_INFO_STREAM("new actions registered: " << req.node_prefix << " registered " << req.actions.size() << " actions.");

    {
        std::lock_guard<std::mutex> lk(registered_actions_mutex);
        registered_actions[req.node_prefix] = req.actions;
    }

    publish_actions();
    return true;
}

void rpc_publish_error(const int16_t code, const std::string &message, const nlohmann::basic_json<> &id = nullptr) {
    json err_resp = {{"jsonrpc", "2.0"},
                       {"error", {{"code", code}, {"message", message}}},
                       {"id", id}};
    try_publish("rpc/response", err_resp.dump(2));
}

void rpc_request_callback(const std::string &payload) {
    // Parse
    json req;
    try {
      req = json::parse(payload);
    } catch (const json::parse_error &e) {
      return rpc_publish_error(xbot_rpc::RpcError::ERROR_INVALID_JSON, "Could not parse request JSON");
    }

    // Validate
    if (!req.is_object()) {
        return rpc_publish_error(xbot_rpc::RpcError::ERROR_INVALID_REQUEST, "Request is not a JSON object");
    }
    json id = req.contains("id") ? req["id"] : nullptr;
    if (id != nullptr && !id.is_string()) {
        return rpc_publish_error(xbot_rpc::RpcError::ERROR_INVALID_REQUEST, "ID is not a string", id);
    } else if (!req.contains("jsonrpc") || !req["jsonrpc"].is_string() || req["jsonrpc"] != "2.0") {
        return rpc_publish_error(xbot_rpc::RpcError::ERROR_INVALID_REQUEST, "Invalid JSON-RPC version");
    } else if (!req.contains("method") || !req["method"].is_string()) {
        return rpc_publish_error(xbot_rpc::RpcError::ERROR_INVALID_REQUEST, "Method is not a string", req["id"]);
    }

    // Check if the method is registered
    const std::string method = req["method"];
    if (method.compare(0, 5, "meta.") == 0) {
      // Silently ignore methods that are handled by the meta service.
      return;
    }
    bool is_registered = false;
    {
        std::lock_guard<std::mutex> lk(registered_methods_mutex);
        for (const auto& [_, method_ids] : registered_methods) {
            if (std::find(method_ids.begin(), method_ids.end(), method) != method_ids.end()) {
                is_registered = true;
                break;
            }
        }
    }
    if (!is_registered) {
        return rpc_publish_error(xbot_rpc::RpcError::ERROR_METHOD_NOT_FOUND, "Method \"" + method + "\" not found", req["id"]);
    }

    // Forward to the providers as ROS message
    xbot_rpc::RpcRequest msg;
    msg.method = method;
    msg.params = req.contains("params") ? req["params"].dump() : "";
    msg.id = id != nullptr ? id : "";
    rpc_request_pub.publish(msg);
}

void rpc_response_callback(const xbot_rpc::RpcResponse::ConstPtr &msg) {
    json result;
    try {
        result = json::parse(msg->result);
    } catch (const json::parse_error &e) {
        return rpc_publish_error(xbot_rpc::RpcError::ERROR_INTERNAL, "Internal error while parsing result JSON: " + std::string(e.what()), msg->id);
    }

    if (msg->id.find("mqtt_timetable_set") == 0 || msg->id.find("mqtt_timetable_suspension_set") == 0) {
        if (result.is_object() && result.contains("valid")) {
            publish_timetable_validation(result);
        }
    }

    if (msg->id.find("mqtt_map_set") == 0) {
        if (result.is_object() && result.contains("valid")) {
            publish_map_validation(result);
        } else if (result.is_string()) {
            publish_map_validation({{"valid", true}, {"remarks", {result.get<std::string>()}}});
        } else {
            publish_map_validation({{"valid", true}, {"remarks", {"Map gespeichert"}}});
        }
    }

    json j = {{"jsonrpc", "2.0"}, {"result", result}, {"id", msg->id}};
    try_publish("rpc/response", j.dump(2));
}

void rpc_error_callback(const xbot_rpc::RpcError::ConstPtr &msg) {
    if (msg->id.find("mqtt_timetable_set") == 0 || msg->id.find("mqtt_timetable_suspension_set") == 0) {
        try {
            json validation = json::parse(msg->message);
            publish_timetable_validation(validation);
        } catch (const json::exception &) {
            publish_timetable_validation({{"valid", false}, {"remarks", {msg->message}}});
        }
    }
    if (msg->id.find("mqtt_map_set") == 0) {
        try {
            json validation = json::parse(msg->message);
            publish_map_validation(validation);
        } catch (const json::exception &) {
            publish_map_validation({{"valid", false}, {"remarks", {msg->message}}});
        }
    }
    rpc_publish_error(msg->code, msg->message, msg->id);
}

bool register_methods(xbot_rpc::RegisterMethodsSrvRequest &req, xbot_rpc::RegisterMethodsSrvResponse &res) {
    std::lock_guard<std::mutex> lk(registered_methods_mutex);
    registered_methods[req.node_id] = req.methods;
    ROS_INFO_STREAM("new methods registered: " << req.node_id << " registered " << req.methods.size() << " methods.");
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "xbot_monitoring");
    has_map = false;
    has_map_overlay = false;


    n = new ros::NodeHandle();
    ros::NodeHandle paramNh("~");

    version_string = paramNh.param("software_version", std::string("UNKNOWN VERSION"));
    if(version_string.empty()) {
        version_string = "UNKNOWN VERSION";
    }

    statustransition_log_file = paramNh.param("statustransition_log_file", std::string("/data/ros/log_statustransition.json"));
    const int configured_statustransition_log_mqtt_limit = paramNh.param("statustransition_log_mqtt_default_limit", 20);
    if (configured_statustransition_log_mqtt_limit <= 0) {
        mqtt_statustransition_log_default_limit = STATUSTRANSITION_LOG_MAX_ENTRIES;
    } else {
        mqtt_statustransition_log_default_limit = std::min<std::size_t>(
            static_cast<std::size_t>(configured_statustransition_log_mqtt_limit),
            STATUSTRANSITION_LOG_MAX_ENTRIES);
    }
    {
        std::lock_guard<std::mutex> lk(statustransition_log_mutex);
        load_statustransition_log_if_needed_locked();
    }

    external_mqtt_enable = paramNh.param("external_mqtt_enable", false);
    external_mqtt_topic_prefix = paramNh.param("external_mqtt_topic_prefix", std::string(""));
    if(!external_mqtt_topic_prefix.empty() && external_mqtt_topic_prefix.back() != '/') {
        // append the /
        external_mqtt_topic_prefix = external_mqtt_topic_prefix+"/";
    }

    mqtt_timetable_publish_interval_sec = paramNh.param("mqtt_timetable_publish_interval_sec", 60.0);
    external_mqtt_hostname = paramNh.param("external_mqtt_hostname", std::string(""));
    external_mqtt_port = std::to_string(paramNh.param("external_mqtt_port", 1883));
    external_mqtt_username = paramNh.param("external_mqtt_username", std::string(""));
    external_mqtt_password = paramNh.param("external_mqtt_password", std::string(""));

    if(external_mqtt_enable) {
        ROS_INFO_STREAM("Using external MQTT broker: " << external_mqtt_hostname << ":" << external_mqtt_port << " with topic prefix: " + external_mqtt_topic_prefix);
    }

    // First setup MQTT
    setupMqttClient();

    ros::ServiceServer register_action_service = n->advertiseService("xbot/register_actions", registerActions);

    ros::Subscriber robotStateSubscriber = n->subscribe("xbot_monitoring/robot_state", 10, robot_state_callback);
    ros::Subscriber mapSubscriber = n->subscribe("mower_map_service/json_map", 10, map_callback);
    ros::Subscriber timetableSubscriber = n->subscribe("timetable/status", 10, timetable_status_callback);
    ros::Subscriber mapOverlaySubscriber = n->subscribe("xbot_monitoring/map_overlay", 10, map_overlay_callback);
    ros::Subscriber mapMowingProgressSubscriber = n->subscribe("/mower_logic/map/mowing_progress/json", 2, mowing_progress_callback);
    ros::Subscriber mapMowingProgressStatusSubscriber =
        n->subscribe("/mower_logic/map/mowing_progress/status/json", 10, mowing_progress_status_callback);
    ros::Subscriber mowerLogicSettingsStatusSubscriber = n->subscribe("/mower_logic/settings/status_json", 10, mower_logic_settings_status_json_callback);
    ros::Subscriber mowerLogicSettingsValidationSubscriber = n->subscribe("/mower_logic/settings/validation_json", 10, mower_logic_settings_validation_json_callback);
    ros::Subscriber mowerLogicSatelliteLoggingStatusSubscriber = n->subscribe("/mower_logic/satellite_logging/status_json", 10, mower_logic_satellite_logging_status_json_callback);
    ros::Subscriber loadFactorComputedSubscriber = n->subscribe("/mower_logic/mow_load_factor/load_factor_computed", 10, load_factor_computed_callback);
    ros::Subscriber loadFactorEffectiveSubscriber = n->subscribe("/mower_logic/mow_load_factor/load_factor_effective", 10, load_factor_effective_callback);
    ros::Subscriber llPowerStatusSubscriber = n->subscribe("/ll/services/power/status_json", 10, ll_power_status_json_callback);
    ros::Subscriber gpsStateLlGpsPoseSubscriber = n->subscribe("/ll/position/gps", 1, gps_state_ll_gps_pose_callback);
    ros::Subscriber gpsStateXbPoseSubscriber = n->subscribe("/xbot_positioning/xb_pose", 1, gps_state_xb_pose_callback);
    ros::Subscriber gpsStateSatellitesSubscriber = n->subscribe("/ll/position/gps/satellites", 1, gps_state_satellites_callback);

    cmd_vel_pub = n->advertise<geometry_msgs::Twist>("xbot_monitoring/remote_cmd_vel", 1);
    action_pub = n->advertise<std_msgs::String>("xbot/action", 1);
    mow_load_factor_set_enabled_pub = n->advertise<std_msgs::Bool>("/mower_logic/mow_load_factor/set_enabled", 10);
    mow_load_factor_set_min_factor_pub = n->advertise<std_msgs::Float32>("/mower_logic/mow_load_factor/set_min_factor", 10);
    mow_load_factor_set_current_start_pub = n->advertise<std_msgs::Float32>("/mower_logic/mow_load_factor/set_current_start", 10);
    mow_load_factor_set_current_end_pub = n->advertise<std_msgs::Float32>("/mower_logic/mow_load_factor/set_current_end", 10);
    mow_load_factor_set_persistent_enabled_pub = n->advertise<std_msgs::Bool>("/mower_logic/mow_load_factor/set_persistent_enabled", 10);
    mow_load_factor_set_persistent_min_factor_pub = n->advertise<std_msgs::Float32>("/mower_logic/mow_load_factor/set_persistent_min_factor", 10);
    mow_load_factor_set_persistent_current_start_pub = n->advertise<std_msgs::Float32>("/mower_logic/mow_load_factor/set_persistent_current_start", 10);
    mow_load_factor_set_persistent_current_end_pub = n->advertise<std_msgs::Float32>("/mower_logic/mow_load_factor/set_persistent_current_end", 10);
    mow_load_factor_renew_pub = n->advertise<std_msgs::Empty>("/mower_logic/mow_load_factor/renew", 10);
    mower_logic_settings_set_session_json_pub = n->advertise<std_msgs::String>("/mower_logic/settings/set_session_json", 10);
    mower_logic_settings_set_persistent_json_pub = n->advertise<std_msgs::String>("/mower_logic/settings/set_persistent_json", 10);
    mower_logic_settings_renew_pub = n->advertise<std_msgs::Empty>("/mower_logic/settings/renew", 10);
    mower_logic_satellite_logging_control_pub = n->advertise<std_msgs::String>("/mower_logic/satellite_logging/set_control_json", 10);
    mower_logic_satellite_logging_renew_pub = n->advertise<std_msgs::Empty>("/mower_logic/satellite_logging/renew", 10);
    map_mowing_progress_renew_pub = n->advertise<std_msgs::Empty>("/mower_logic/map/mowing_progress/renew", 10);
    ll_power_set_battery_critical_voltage_pub = n->advertise<std_msgs::Float64>("/ll/services/power/set/battery_critical_voltage", 10);
    ll_power_set_battery_empty_voltage_pub = n->advertise<std_msgs::Float64>("/ll/services/power/set/battery_empty_voltage", 10);
    ll_power_set_battery_full_voltage_pub = n->advertise<std_msgs::Float64>("/ll/services/power/set/battery_full_voltage", 10);
    ll_power_set_battery_critical_high_voltage_pub = n->advertise<std_msgs::Float64>("/ll/services/power/set/battery_critical_high_voltage", 10);
    ll_power_set_charge_critical_high_voltage_pub = n->advertise<std_msgs::Float64>("/ll/services/power/set/charge_critical_high_voltage", 10);
    ll_power_set_charge_critical_high_current_pub = n->advertise<std_msgs::Float64>("/ll/services/power/set/charge_critical_high_current", 10);
    ll_power_set_persistent_battery_critical_voltage_pub = n->advertise<std_msgs::Float64>("/ll/services/power/set_persistent/battery_critical_voltage", 10);
    ll_power_set_persistent_battery_empty_voltage_pub = n->advertise<std_msgs::Float64>("/ll/services/power/set_persistent/battery_empty_voltage", 10);
    ll_power_set_persistent_battery_full_voltage_pub = n->advertise<std_msgs::Float64>("/ll/services/power/set_persistent/battery_full_voltage", 10);
    ll_power_set_persistent_battery_critical_high_voltage_pub = n->advertise<std_msgs::Float64>("/ll/services/power/set_persistent/battery_critical_high_voltage", 10);
    ll_power_set_persistent_charge_critical_high_voltage_pub = n->advertise<std_msgs::Float64>("/ll/services/power/set_persistent/charge_critical_high_voltage", 10);
    ll_power_set_persistent_charge_critical_high_current_pub = n->advertise<std_msgs::Float64>("/ll/services/power/set_persistent/charge_critical_high_current", 10);
    ftc_settings_set_speed_fast_pub = n->advertise<std_msgs::Float64>("/ftc_local_planner/settings/set/speed_fast", 10);
    ftc_settings_set_speed_slow_pub = n->advertise<std_msgs::Float64>("/ftc_local_planner/settings/set/speed_slow", 10);
    ftc_settings_set_speed_fast_threshold_pub = n->advertise<std_msgs::Float64>("/ftc_local_planner/settings/set/speed_fast_threshold", 10);
    ftc_settings_set_persistent_speed_fast_pub = n->advertise<std_msgs::Float64>("/ftc_local_planner/settings/set_persistent/speed_fast", 10);
    ftc_settings_set_persistent_speed_slow_pub = n->advertise<std_msgs::Float64>("/ftc_local_planner/settings/set_persistent/speed_slow", 10);
    ftc_settings_set_persistent_speed_fast_threshold_pub = n->advertise<std_msgs::Float64>("/ftc_local_planner/settings/set_persistent/speed_fast_threshold", 10);
    ll_power_renew_pub = n->advertise<std_msgs::Empty>("/ll/services/power/renew", 10);

    rpc_request_pub = n->advertise<xbot_rpc::RpcRequest>(xbot_rpc::TOPIC_REQUEST, 100);
    ros::Subscriber rpc_response_sub = n->subscribe(xbot_rpc::TOPIC_RESPONSE, 100, rpc_response_callback);
    ros::Subscriber rpc_error_sub = n->subscribe(xbot_rpc::TOPIC_ERROR, 100, rpc_error_callback);
    ros::ServiceServer register_methods_service = n->advertiseService(xbot_rpc::SERVICE_REGISTER_METHODS, register_methods);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    rpc_provider.init();

    ros::Rate sensor_check_rate(10.0);

    boost::regex topic_regex("/xbot_monitoring/sensors/.*/info");

    // Maps a sensor info topic to its subscriber. Only touched by this thread.
    std::map<std::string, ros::Subscriber> active_subscribers;
    std::vector<ros::Subscriber> sensor_data_subscribers;

    while (ros::ok()) {
        // Read the topics in /xbot_monitoring/sensors/.*/info and subscribe to them.
        ros::master::V_TopicInfo topics;
        ros::master::getTopics(topics);
        std::for_each(topics.begin(), topics.end(), [&](const ros::master::TopicInfo &item) {

            if (!boost::regex_match(item.name, topic_regex) || active_subscribers.count(item.name) != 0)
                return;

            ROS_INFO_STREAM("Found new sensor topic " << item.name);
            active_subscribers[item.name] = n->subscribe<xbot_msgs::SensorInfo>(
                item.name, 1, [topic = item.name, &sensor_data_subscribers](const xbot_msgs::SensorInfo::ConstPtr &msg) {
                    ROS_INFO_STREAM("Got sensor info for sensor on topic " << msg->sensor_name << " on topic " << topic);

                    bool is_new = false;
                    {
                        std::unique_lock<std::mutex> lk(found_sensors_mutex);
                        is_new = found_sensors.count(topic) == 0;

                        // Sensor already known and sensor-info equals?
                        if (!is_new && found_sensors[topic] == *msg) return;

                        found_sensors[topic] = *msg;  // Save the (new|changed) sensor info
                    }

                    // Let the info subscription alive for dynamic threshold changes
                    //active_subscribers.erase(topic);  // Stop subscribing to infos

                    if (is_new) {
                        subscribe_to_sensor(topic, sensor_data_subscribers);  // Subscribe for data
                    }

                    // Republish (new|changed) sensor info
                    // NOTE: If a sensor name or id changes, the related data topic wouldn't change!
                    //       But do we dynamically change a sensor name or id?
                    publish_sensor_metadata();
                }
            );
        });
        maybe_publish_timetable(false);
        sensor_check_rate.sleep();
    }
    return 0;
}
