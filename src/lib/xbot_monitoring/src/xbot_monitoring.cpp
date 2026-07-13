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
#include <stdexcept>
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
#include <map>
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
void publish_gps_state_definitions();
void handle_gps_state_renew_payload(const std::string &payload_text);
void publish_gps_state0_snapshot();
void handle_gps_restart_set_payload(const std::string &payload_text);
void publish_gps_restart_validation(const json &validation);
void publish_gps_restart_status();
void handle_gps_logging_control_payload(const std::string &payload_text);
void publish_gps_logging_status();
void publish_gps_logging_last();
void gps_restart_status_callback(const std_msgs::String::ConstPtr &msg);
void gps_state_ll_gps_pose_callback(const xbot_msgs::AbsolutePose::ConstPtr &msg);
void gps_state_fix_status_callback(const std_msgs::String::ConstPtr &msg);
void gps_state_xb_pose_callback(const xbot_msgs::AbsolutePose::ConstPtr &msg);
void gps_state_positioning_debug_callback(const std_msgs::String::ConstPtr &msg);
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
bool try_parse_utc_timestamp_iso8601(const std::string &timestamp,
                                     std::chrono::system_clock::time_point &time_point);
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
ros::Publisher gps_restart_request_pub;

// properties for external mqtt
bool external_mqtt_enable = false;
std::string external_mqtt_username = "";
std::string external_mqtt_password = "";
std::string external_mqtt_hostname = "";
std::string external_mqtt_topic_prefix = "";
std::string external_mqtt_port = "";
std::string version_string = "";

// Forward declarations for restart status state used by MqttCallback::connected().
// The corresponding definitions remain with the GPS-state globals below.
extern std::mutex gps_restart_status_mutex;
extern json last_completed_gps_restart;
extern bool last_completed_gps_restart_available;

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
        publish_gps_state_definitions();
        publish_latest_gps_state_payloads(true);
        publish_gps_restart_status();
        publish_gps_logging_status();
        publish_gps_logging_last();
        {
            std::lock_guard<std::mutex> lk(gps_restart_status_mutex);
            if (last_completed_gps_restart_available) {
                try_publish("gps_state/restart/last/json", last_completed_gps_restart.dump(), true);
            }
        }

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
        client_->subscribe(this->mqtt_topic_prefix + "gps_state/set/renew/json", 0);
        // Deprecated compatibility alias. New clients use gps_state/set/renew/json.
        client_->subscribe(this->mqtt_topic_prefix + "gps_state/state0/set/renew/json", 0);
        client_->subscribe(this->mqtt_topic_prefix + "gps_state/settings/set/renew/json", 0);
        client_->subscribe(this->mqtt_topic_prefix + "gps_state/settings/set/session/json", 0);
        client_->subscribe(this->mqtt_topic_prefix + "gps_state/settings/set/persistent/json", 0);
        client_->subscribe(this->mqtt_topic_prefix + "gps_state/logging/set/control/json", 0);
        client_->subscribe(this->mqtt_topic_prefix + "gps_state/logging/set/renew/json", 0);
        client_->subscribe(this->mqtt_topic_prefix + "gps_state/restart/set/json", 0);
        client_->subscribe(this->mqtt_topic_prefix + "gps_state/restart/set/renew/json", 0);
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
        } else if (ptr->get_topic() == this->mqtt_topic_prefix + "gps_state/set/renew/json") {
            handle_gps_state_renew_payload(ptr->get_payload_str());
        } else if (ptr->get_topic() == this->mqtt_topic_prefix + "gps_state/state0/set/renew/json") {
            ROS_WARN_STREAM("Deprecated GPS-State renew topic used; switch to gps_state/set/renew/json");
            publish_gps_state0_snapshot();
        } else if (ptr->get_topic() == this->mqtt_topic_prefix + "gps_state/settings/set/renew/json") {
            std_msgs::Empty mower_logic_renew;
            mower_logic_settings_renew_pub.publish(mower_logic_renew);
            publish_gps_state_settings();
            publish_gps_state_definitions();
            publish_latest_gps_state_payloads(true);
        } else if (ptr->get_topic() == this->mqtt_topic_prefix + "gps_state/settings/set/session/json") {
            handle_gps_state_set_payload(ptr->get_payload_str(), false);
        } else if (ptr->get_topic() == this->mqtt_topic_prefix + "gps_state/settings/set/persistent/json") {
            handle_gps_state_set_payload(ptr->get_payload_str(), true);
        } else if (ptr->get_topic() == this->mqtt_topic_prefix + "gps_state/logging/set/control/json") {
            handle_gps_logging_control_payload(ptr->get_payload_str());
        } else if (ptr->get_topic() == this->mqtt_topic_prefix + "gps_state/logging/set/renew/json") {
            std_msgs::Empty msg;
            mower_logic_satellite_logging_renew_pub.publish(msg);
            publish_gps_logging_status();
            publish_gps_logging_last();
            publish_gps_state_settings();
        } else if (ptr->get_topic() == this->mqtt_topic_prefix + "gps_state/restart/set/json") {
            handle_gps_restart_set_payload(ptr->get_payload_str());
        } else if (ptr->get_topic() == this->mqtt_topic_prefix + "gps_state/restart/set/renew/json") {
            publish_gps_restart_status();
            publish_gps_state_settings();
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
constexpr const char *GPS_STATE_PAYLOAD_SCHEMA = "gps_state.v3";
constexpr int GPS_STATE_DEFINITION_VERSION = 3;
constexpr double GPS_STATE_STALE_AFTER_S = 3.0;

struct GpsStateSettings {
    bool enabled = true;
    double publish_rate_hz = 1.0;
    bool publish_state0 = false;
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
json latest_gps_fix_status = json::object();
bool latest_gps_fix_status_available = false;
ros::Time latest_gps_fix_status_received_at;
json latest_xbot_positioning_gps_debug = json::object();
bool latest_xbot_positioning_gps_debug_available = false;
ros::Time latest_xbot_positioning_gps_debug_received_at;

std::mutex gps_restart_status_mutex;
json latest_gps_restart_status = json::object();
bool latest_gps_restart_status_available = false;
json last_completed_gps_restart = json::object();
bool last_completed_gps_restart_available = false;

std::mutex mower_logic_settings_cache_mutex;
json latest_mower_logic_settings_payload = json::object();
bool latest_mower_logic_settings_available = false;

std::mutex gps_logging_status_mutex;
json latest_gps_logging_status = json::object();
bool latest_gps_logging_status_available = false;
json last_completed_gps_logging = json::object();
bool last_completed_gps_logging_available = false;

std::mutex gps_logging_pending_mutex;
json pending_gps_logging_settings = json::object();
bool pending_gps_logging_settings_persistent = false;



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
                                       bool expert,
                                       const std::string &group = "states") {
    json entry = json::object();
    entry["label"] = label;
    entry["description"] = description;
    entry["group"] = group;
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

static json gps_state_command_entry(const std::string &label,
                                    const std::string &description,
                                    int order,
                                    const std::string &topic,
                                    const json &payload_schema,
                                    bool expert = false,
                                    const std::string &group = "restart") {
    json entry = json::object();
    entry["label"] = label;
    entry["description"] = description;
    entry["group"] = group;
    entry["order"] = order;
    entry["type"] = "json_command";
    entry["topic"] = topic;
    entry["payload_schema"] = payload_schema;
    entry["value"] = nullptr;
    entry["active"] = nullptr;
    entry["persistent"] = nullptr;
    entry["visible"] = true;
    entry["expert"] = expert;
    entry["readonly"] = false;
    entry["different"] = false;
    entry["restart_required"] = false;
    entry["session_apply_supported"] = true;
    entry["persistent_apply_supported"] = false;
    return entry;
}

static const std::map<std::string, std::string> &gps_logging_public_to_internal_settings() {
    static const std::map<std::string, std::string> mapping = {
        {"logging_default_trigger", "satellite_logging_default_trigger"},
        {"logging_default_mode", "satellite_logging_default_mode"},
        {"logging_default_area_id", "satellite_logging_default_area_id"},
        {"logging_script_path", "satellite_logging_script_path"},
        {"logging_ram_path", "satellite_logging_ram_path"},
        {"logging_output_path", "satellite_logging_output_path"},
        {"logging_container_name", "satellite_logging_container_name"}
    };
    return mapping;
}

static bool is_gps_logging_internal_setting(const std::string &key) {
    if (key == "satellite_logging_enabled") return true;
    for (const auto &entry : gps_logging_public_to_internal_settings()) {
        if (entry.second == key) return true;
    }
    return false;
}

static json gps_logging_cached_internal_entry(const std::string &internal_key) {
    std::lock_guard<std::mutex> lk(mower_logic_settings_cache_mutex);
    if (!latest_mower_logic_settings_available || !latest_mower_logic_settings_payload.is_object() ||
        !latest_mower_logic_settings_payload.contains("settings") ||
        !latest_mower_logic_settings_payload["settings"].is_object() ||
        !latest_mower_logic_settings_payload["settings"].contains(internal_key) ||
        !latest_mower_logic_settings_payload["settings"][internal_key].is_object()) {
        return json::object();
    }
    return latest_mower_logic_settings_payload["settings"][internal_key];
}

static json gps_logging_string_setting_entry(const std::string &internal_key,
                                             const std::string &label,
                                             const std::string &description,
                                             int order,
                                             const std::string &fallback,
                                             bool expert,
                                             const json &allowed_values = nullptr) {
    json entry = gps_state_setting_entry(label, description, "logging", order, "string", fallback,
                                         nullptr, nullptr, "", expert);
    const json cached = gps_logging_cached_internal_entry(internal_key);
    if (cached.is_object() && !cached.empty()) {
        for (const char *field : {"value", "active", "persistent", "default", "different"}) {
            if (cached.contains(field)) entry[field] = cached[field];
        }
    } else {
        std::string active = fallback;
        std::string persistent = fallback;
        ros::param::get("/settings/mower_logic/active/" + internal_key, active);
        ros::param::get("/settings/mower_logic/persistent/" + internal_key, persistent);
        entry["value"] = active;
        entry["active"] = active;
        entry["persistent"] = persistent;
        entry["different"] = active != persistent;
    }
    if (!allowed_values.is_null()) entry["enum"] = allowed_values;
    return entry;
}

static std::string gps_logging_public_key_for_internal(const std::string &internal_key) {
    for (const auto &entry : gps_logging_public_to_internal_settings()) {
        if (entry.second == internal_key) return entry.first;
    }
    return "";
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
    gps_state_settings.publish_state0 = persisted_bool_or(persisted, "publish_state0", gps_state_settings.publish_state0);
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
        {"refresh", {{"label", "Aktualisierung"}, {"order", 25}}},
        {"logging", {{"label", "GPS Logging"}, {"order", 30}}},
        {"restart", {{"label", "F9P Neustart"}, {"order", 40}}},
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
    root["settings"]["state0_definition"] = gps_state_descriptor_entry(
        "GPS State 0 Definition",
        "Statische Definition der 12 Entscheidungsknoten fuer die GPS-Fahrfaehigkeitsdiagnose. Enthält Titel, Beschreibung, Quelle, Grenzwertbeschreibung, Fehlerfolge und naechste Pruefung.",
        1, "gps_state/state0/definition", true);
    root["settings"]["state0_status"] = gps_state_descriptor_entry(
        "GPS State 0 Status",
        "Live-Werte der 12 Entscheidungsknoten fuer die sofortige Beurteilung der Fahrfaehigkeit. Enthält Status, aktuellen Wert, Grenzwert, Abweichung und blockierende Stufe.",
        2, "gps_state/state0/status", true);
    root["settings"]["state1_definition"] = gps_state_descriptor_entry(
        "GPS State 1 Definition",
        "Statische Felddefinition des kompakten Bedienerstatus.",
        10, "gps_state/state1/definition", false);
    root["settings"]["state1_status"] = gps_state_descriptor_entry(
        "GPS State 1 Status",
        "Dynamischer Bedienerstatus zur GPS-Fahrfreigabe.",
        11, "gps_state/state1/status", false);
    root["settings"]["state2_definition"] = gps_state_descriptor_entry(
        "GPS State 2 Definition",
        "Statische Felddefinition der technischen GNSS- und Pose-Zusammenfassung.",
        20, "gps_state/state2/definition", false);
    root["settings"]["state2_status"] = gps_state_descriptor_entry(
        "GPS State 2 Status",
        "Dynamische technische GNSS- und Pose-Zusammenfassung ohne Satellitenliste.",
        21, "gps_state/state2/status", false);
    root["settings"]["state3_definition"] = gps_state_descriptor_entry(
        "GPS State 3 Definition",
        "Statische Felddefinition der Liste aktuell verwendeter Satelliten.",
        30, "gps_state/state3/definition", false);
    root["settings"]["state3_status"] = gps_state_descriptor_entry(
        "GPS State 3 Status",
        "Dynamische Liste der aktuell verwendeten Satelliten.",
        31, "gps_state/state3/status", false);
    root["settings"]["state4_definition"] = gps_state_descriptor_entry(
        "GPS State 4 Definition",
        "Statische Felddefinition der vollständigen Experten-Satellitenliste.",
        40, "gps_state/state4/definition", true);
    root["settings"]["state4_status"] = gps_state_descriptor_entry(
        "GPS State 4 Status",
        "Dynamische vollständige Satelliten-Diagnose mit allen sichtbaren Satelliten.",
        41, "gps_state/state4/status", true);
    root["settings"]["renew"] = gps_state_descriptor_entry(
        "GPS States aktualisieren",
        "Zentrale Aktualisierung. Leerer Inhalt oder {} aktualisiert Definition und Status aller aktivierten States. Optional koennen states und parts angegeben werden.",
        1, "gps_state/set/renew/json", false, "refresh");
    root["settings"]["renew"]["readonly"] = false;
    root["settings"]["renew"]["type"] = "json_command";
    json renew_schema = json::object();
    renew_schema["type"] = "object";
    renew_schema["properties"] = json::object();
    renew_schema["properties"]["states"] = {
        {"type", "array"},
        {"items", {{"oneOf", json::array({
            json{{"type", "integer"}, {"minimum", 0}, {"maximum", 4}},
            json{{"type", "string"}}
        })}}}
    };
    renew_schema["properties"]["parts"] = {
        {"type", "array"},
        {"items", {{"type", "string"}, {"enum", json::array({"definition", "status"})}}}
    };
    renew_schema["examples"] = json::array({
        json::object(),
        json{{"states", json::array({0, 2})}, {"parts", json::array({"status"})}},
        json{{"states", json::array({0, 1, 2, 3, 4})}, {"parts", json::array({"definition"})}}
    });
    root["settings"]["renew"]["payload_schema"] = renew_schema;

    root["settings"]["logging_default_trigger"] = gps_logging_string_setting_entry(
        "satellite_logging_default_trigger",
        "Logging Standard-Startart",
        "Standard-Trigger, wenn ein Startbefehl trigger nicht explizit angibt.",
        310, "next_cycle", false,
        json::array({"next_cycle", "ad_hoc", "area_id"}));
    root["settings"]["logging_default_mode"] = gps_logging_string_setting_entry(
        "satellite_logging_default_mode",
        "Logging Standard-Modus",
        "Standard-Endbedingung, wenn ein Startbefehl mode nicht explizit angibt.",
        320, "from_start_to_docking", false,
        json::array({"from_start_to_docking", "from_docking_to_docking", "until_docking", "manual", "area_only", "area_to_docking"}));
    root["settings"]["logging_default_area_id"] = gps_logging_string_setting_entry(
        "satellite_logging_default_area_id",
        "Logging Ziel-Flächen-ID",
        "Standard-Fläche für trigger=area_id. Ein leerer Wert erfordert area_id im Startbefehl.",
        330, "", false);
    root["settings"]["logging_output_path"] = gps_logging_string_setting_entry(
        "satellite_logging_output_path",
        "Logging Zielpfad",
        "Persistentes Zielverzeichnis, in das eine beendete Aufzeichnung kopiert wird.",
        340, "/home/openmower/recordings/logs", true);
    root["settings"]["logging_ram_path"] = gps_logging_string_setting_entry(
        "satellite_logging_ram_path",
        "Logging RAM-Pfad",
        "Temporäres Verzeichnis für laufende Aufzeichnungen. Bei einem harten Ausfall können nicht kopierte Daten verloren gehen.",
        350, "/dev/shm/openmower_satellite_logs", true);
    root["settings"]["logging_script_path"] = gps_logging_string_setting_entry(
        "satellite_logging_script_path",
        "Logging Skriptpfad",
        "Pfad zum ausführbaren GPS-Logging-Skript.",
        360, "/home/openmower/scripts/record_satellites.sh", true);
    root["settings"]["logging_container_name"] = gps_logging_string_setting_entry(
        "satellite_logging_container_name",
        "Logging ROS-Container",
        "Optionaler ROS-Containername. Leer bedeutet direkte Ausführung beziehungsweise automatische Erkennung durch das Skript.",
        370, "", true);

    json logging_control_schema = {
        {"type", "object"},
        {"required", json::array({"command"})},
        {"properties", {
            {"command", {{"type", "string"}, {"enum", json::array({"start", "stop", "cancel"})}}},
            {"trigger", {{"type", "string"}, {"enum", json::array({"next_cycle", "ad_hoc", "area_id"})}}},
            {"mode", {{"type", "string"}, {"enum", json::array({"from_start_to_docking", "from_docking_to_docking", "until_docking", "manual", "area_only", "area_to_docking"})}}},
            {"area_id", {{"oneOf", json::array({json{{"type", "string"}}, json{{"type", "integer"}}})}}},
            {"request_id", {{"description", "Optional correlation value echoed by validation."}}}
        }},
        {"examples", json::array({
            json{{"command", "start"}, {"trigger", "ad_hoc"}, {"mode", "until_docking"}},
            json{{"command", "start"}, {"trigger", "next_cycle"}, {"mode", "from_start_to_docking"}},
            json{{"command", "start"}, {"trigger", "area_id"}, {"mode", "until_docking"}, {"area_id", "3"}},
            json{{"command", "stop"}},
            json{{"command", "cancel"}}
        })}
    };
    root["settings"]["logging_control"] = gps_state_command_entry(
        "GPS Logging steuern",
        "Startet, stoppt oder bricht eine GPS-Logging-Anforderung ab. Bei cancel bleibt eine bereits erzeugte Session als abgebrochen nachvollziehbar. Start/Stop ist ein Befehl und kein persistenter Einstellungswert.",
        380, "gps_state/logging/set/control/json", logging_control_schema, false, "logging");
    root["settings"]["logging_status"] = gps_state_descriptor_entry(
        "GPS Logging Laufzeitstatus",
        "Retained Laufzeitstatus mit Anfrage, Session, Zeitstempeln, Dateien, Speicherpfaden und Fehlerzustand.",
        390, "gps_state/logging/status/json", false, "logging");
    root["settings"]["logging_last"] = gps_state_descriptor_entry(
        "Letzte GPS Logging Session",
        "Retained Abschlussdatensatz der zuletzt beendeten Aufzeichnung.",
        400, "gps_state/logging/last/json", false, "logging");
    root["settings"]["logging_validation"] = gps_state_descriptor_entry(
        "GPS Logging Validierung",
        "Ergebnis des letzten Logging-Steuerbefehls. Nicht retained und für direkte App-Rückmeldung gedacht. Einstellungsänderungen werden separat über gps_state/settings/validation/json bestätigt.",
        410, "gps_state/logging/validation/json", true, "logging");
    root["settings"]["logging_renew"] = gps_state_command_entry(
        "GPS Logging aktualisieren",
        "Fordert Laufzeitstatus, letzte Session und GPS-State-Einstellungen erneut an.",
        420, "gps_state/logging/set/renew/json", json{{"type", "object"}, {"examples", json::array({json::object()})}}, false, "logging");

    root["settings"]["f9p_restart"] = gps_state_command_entry(
        "F9P Neustart auslösen",
        "Sendet eine UBX-CFG-RST-Neustartanforderung an den u-blox/ZED-F9P. Unterstützt hot_start, warm_start und cold_start. Standard ist reset_mode=controlled_software; Experten können gnss_only oder hardware_watchdog angeben.",
        210, "gps_state/restart/set/json",
        json{{"type", "object"},
             {"required", json::array({"mode"})},
             {"properties", {
                 {"mode", {{"type", "string"}, {"enum", json::array({"hot_start", "warm_start", "cold_start"})}}},
                 {"reset_mode", {{"type", "string"}, {"default", "controlled_software"}, {"enum", json::array({"controlled_software", "gnss_only", "hardware_watchdog"})}}}
             }},
             {"examples", json::array({
                 json{{"mode", "hot_start"}},
                 json{{"mode", "warm_start"}},
                 json{{"mode", "cold_start"}}
             })}},
        true);
    root["settings"]["f9p_restart_status"] = gps_state_descriptor_entry(
        "F9P Neustart Status",
        "Retained Status des zuletzt angeforderten oder vom GPS-Treiber gemeldeten F9P-Neustarts.",
        220, "gps_state/restart/status/json", true, "restart");
    root["settings"]["f9p_restart_last"] = gps_state_descriptor_entry(
        "Letzter abgeschlossener F9P Neustart",
        "Retained Datensatz des letzten erfolgreich oder fehlerhaft abgeschlossenen Neustarts mit Zeitstempeln.",
        230, "gps_state/restart/last/json", true, "restart");
    root["settings"]["publish_state0"] = gps_state_setting_entry(
        "State 0 Diagnose veröffentlichen",
        "Veröffentlicht die schaltbare 12-Stufen-Fahrfaehigkeitsdiagnose auf gps_state/state0/definition und gps_state/state0/status. Die statische Definition wird retained gesendet, die Live-Werte folgen der Publish-Rate.",
        "debug", 5, "bool", cfg.publish_state0, nullptr, nullptr, "", true);
    root["settings"]["publish_state1"] = gps_state_setting_entry(
        "State 1 veröffentlichen",
        "Veröffentlicht den kompakten GPS-State auf gps_state/state1/status.",
        "states", 110, "bool", cfg.publish_state1);
    root["settings"]["publish_state2"] = gps_state_setting_entry(
        "State 2 veröffentlichen",
        "Veröffentlicht die erweiterte GPS-Zusammenfassung auf gps_state/state2/status.",
        "states", 120, "bool", cfg.publish_state2);
    root["settings"]["publish_state3"] = gps_state_setting_entry(
        "State 3 veröffentlichen",
        "Veröffentlicht die Liste der aktuell verwendeten Satelliten auf gps_state/state3/status.",
        "states", 130, "bool", cfg.publish_state3);
    root["settings"]["publish_state4"] = gps_state_setting_entry(
        "State 4 veröffentlichen",
        "Veröffentlicht die vollständige Satellitenliste auf gps_state/state4/status. Diese Ausgabe kann deutlich größer sein und ist primär für Debug- und Expertenansichten gedacht.",
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

static bool gps_state_validate_string_setting(const json &entry,
                                              std::string &value,
                                              std::string &reason,
                                              const std::set<std::string> &allowed = {},
                                              bool allow_empty = true) {
    if (!entry.is_object() || !entry.contains("value")) {
        reason = "setting entry must be an object with a value field";
        return false;
    }
    if (!entry["value"].is_string()) {
        reason = "value must be a string";
        return false;
    }
    value = trim_settings_string(entry["value"].get<std::string>());
    if (!allow_empty && value.empty()) {
        reason = "value must not be empty";
        return false;
    }
    if (!allowed.empty() && allowed.find(value) == allowed.end()) {
        reason = "value is not part of the supported enum";
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
        json logging_forward_payload = json::object();
        bool has_native_gps_state_change = false;

        for (auto it = payload.begin(); it != payload.end(); ++it) {
            const std::string key = it.key();
            const json &entry = it.value();
            std::string reason;
            bool bool_value = false;
            double number_value = 0.0;
            std::string string_value;
            bool accepted = false;
            json accepted_fields = json::array();

            if (key == "enabled") {
                accepted = gps_state_validate_bool_setting(entry, bool_value, reason);
                if (accepted) { new_cfg.enabled = bool_value; has_native_gps_state_change = true; }
            } else if (key == "publish_state0") {
                accepted = gps_state_validate_bool_setting(entry, bool_value, reason);
                if (accepted) { new_cfg.publish_state0 = bool_value; has_native_gps_state_change = true; }
            } else if (key == "publish_state1") {
                accepted = gps_state_validate_bool_setting(entry, bool_value, reason);
                if (accepted) { new_cfg.publish_state1 = bool_value; has_native_gps_state_change = true; }
            } else if (key == "publish_state2") {
                accepted = gps_state_validate_bool_setting(entry, bool_value, reason);
                if (accepted) { new_cfg.publish_state2 = bool_value; has_native_gps_state_change = true; }
            } else if (key == "publish_state3") {
                accepted = gps_state_validate_bool_setting(entry, bool_value, reason);
                if (accepted) { new_cfg.publish_state3 = bool_value; has_native_gps_state_change = true; }
            } else if (key == "publish_state4") {
                accepted = gps_state_validate_bool_setting(entry, bool_value, reason);
                if (accepted) { new_cfg.publish_state4 = bool_value; has_native_gps_state_change = true; }
            } else if (key == "publish_rate_hz") {
                accepted = gps_state_validate_number_setting(entry, 0.1, 5.0, number_value, reason);
                if (accepted) { new_cfg.publish_rate_hz = number_value; has_native_gps_state_change = true; }
            } else if (key == "weak_cn0_threshold") {
                accepted = gps_state_validate_number_setting(entry, 0.0, 60.0, number_value, reason);
                if (accepted) { new_cfg.weak_cn0_threshold = number_value; has_native_gps_state_change = true; }
            } else if (key == "good_cn0_threshold") {
                accepted = gps_state_validate_number_setting(entry, 0.0, 60.0, number_value, reason);
                if (accepted) { new_cfg.good_cn0_threshold = number_value; has_native_gps_state_change = true; }
            } else if (gps_logging_public_to_internal_settings().find(key) != gps_logging_public_to_internal_settings().end()) {
                if (key == "logging_default_trigger") {
                    accepted = gps_state_validate_string_setting(
                        entry, string_value, reason, {"next_cycle", "ad_hoc", "area_id"}, false);
                } else if (key == "logging_default_mode") {
                    accepted = gps_state_validate_string_setting(
                        entry, string_value, reason,
                        {"from_start_to_docking", "from_docking_to_docking", "until_docking", "manual", "area_only", "area_to_docking"}, false);
                } else if (key == "logging_script_path" || key == "logging_ram_path" || key == "logging_output_path") {
                    accepted = gps_state_validate_string_setting(entry, string_value, reason, {}, false);
                } else {
                    accepted = gps_state_validate_string_setting(entry, string_value, reason);
                }
                if (accepted) {
                    const std::string &internal_key = gps_logging_public_to_internal_settings().at(key);
                    logging_forward_payload[internal_key] = {{"value", string_value}};
                }
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
            if (persistent && gps_logging_public_to_internal_settings().find(key) == gps_logging_public_to_internal_settings().end()) {
                persistent_updates[key]["persistent"] = open_mower_settings::json::parse(entry["value"].dump());
            }
        }

        if (validation["accepted"].empty() && validation["rejected"].empty()) {
            validation["rejected"]["$"] = "payload does not contain any gps_state settings";
        }

        if (validation["rejected"].empty()) {
            if (has_native_gps_state_change) {
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
            if (!logging_forward_payload.empty() && validation["rejected"].empty()) {
                {
                    std::lock_guard<std::mutex> lk(gps_logging_pending_mutex);
                    pending_gps_logging_settings = json::object();
                    for (auto it = logging_forward_payload.begin(); it != logging_forward_payload.end(); ++it) {
                        const std::string public_key = gps_logging_public_key_for_internal(it.key());
                        if (!public_key.empty()) pending_gps_logging_settings[public_key] = it.value()["value"];
                    }
                    pending_gps_logging_settings_persistent = persistent;
                }
                std_msgs::String forward;
                forward.data = logging_forward_payload.dump();
                if (persistent) {
                    mower_logic_settings_set_persistent_json_pub.publish(forward);
                } else {
                    mower_logic_settings_set_session_json_pub.publish(forward);
                }
                validation["pending"] = true;
                validation["status"] = "forwarded";
                validation["remarks"].push_back(
                    "GPS logging settings were forwarded to mower_logic; confirm applied values via gps_state/settings/json.");
            }
        }
    } catch (const json::exception &e) {
        validation["rejected"]["$"] = std::string("Error decoding JSON: ") + e.what();
    }

    validation["valid"] = !validation["accepted"].empty() && validation["rejected"].empty();
    if (!validation.contains("pending")) validation["pending"] = false;
    if (!validation.contains("status")) validation["status"] = validation["valid"] ? "applied" : "rejected";
    publish_gps_state_validation(validation);
    publish_gps_state_settings();
    publish_gps_state_definitions();
    publish_latest_gps_state_payloads(true);
}

static std::string gps_logging_string_or(const json &payload,
                                         const std::string &key,
                                         const std::string &fallback = "") {
    if (!payload.is_object() || !payload.contains(key) || !payload[key].is_string()) return fallback;
    return payload[key].get<std::string>();
}

static json gps_logging_json_or_null(const json &payload, const std::string &key) {
    if (!payload.is_object() || !payload.contains(key)) return nullptr;
    return payload[key];
}

static json gps_logging_duration_seconds(const json &payload) {
    const std::string started_at = gps_logging_string_or(payload, "started_at");
    if (started_at.empty()) return nullptr;
    std::chrono::system_clock::time_point start;
    if (!try_parse_utc_timestamp_iso8601(started_at, start)) return nullptr;

    std::chrono::system_clock::time_point end = std::chrono::system_clock::now();
    const std::string finished_at = gps_logging_string_or(payload, "finished_at");
    if (!finished_at.empty() && !try_parse_utc_timestamp_iso8601(finished_at, end)) return nullptr;
    const double seconds = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() / 1000.0;
    return std::max(0.0, seconds);
}

static json build_gps_logging_status_payload(const json &source) {
    const std::string state = gps_logging_string_or(source, "state", "unknown");
    const bool running = source.value("running", false);
    const bool armed = source.value("armed", false);
    const bool request_active = source.value("request_active", running || armed);
    const json error = gps_logging_json_or_null(source, "error");

    int severity = 0;
    std::string summary = "GPS Logging ist inaktiv";
    if (!error.is_null() || state == "error") {
        severity = 4;
        summary = "GPS Logging meldet einen Fehler";
    } else if (running) {
        severity = 1;
        summary = "GPS-Aufzeichnung läuft";
    } else if (armed) {
        severity = 1;
        summary = "GPS-Aufzeichnung ist vorgemerkt";
    } else if (state == "finished") {
        summary = "GPS-Aufzeichnung wurde beendet";
    }

    json root = {
        {"schema", "openmower.gps_state.logging.v1"},
        {"type", "status"},
        {"published_at", ros::Time::now().toSec()},
        {"status", state},
        {"severity", severity},
        {"summary", summary},
        {"runtime", {
            {"state", state},
            {"request_active", request_active},
            {"request_origin", gps_logging_json_or_null(source, "request_origin")},
            {"armed", armed},
            {"running", running},
            {"pid", gps_logging_json_or_null(source, "pid")},
            {"session_id", gps_logging_json_or_null(source, "session_id")},
            {"requested_at", gps_logging_json_or_null(source, "requested_at")},
            {"started_at", gps_logging_json_or_null(source, "started_at")},
            {"finished_at", gps_logging_json_or_null(source, "finished_at")},
            {"duration_s", gps_logging_duration_seconds(source)},
            {"stop_reason", gps_logging_json_or_null(source, "stop_reason")}
        }},
        {"request", {
            {"trigger", gps_logging_json_or_null(source, "trigger")},
            {"mode", gps_logging_json_or_null(source, "mode")},
            {"target_area_id", gps_logging_json_or_null(source, "target_area_id")}
        }},
        {"storage", {
            {"ram_path", gps_logging_json_or_null(source, "ram_path")},
            {"output_path", gps_logging_json_or_null(source, "output_path")},
            {"files", source.contains("files") ? source["files"] : json::array()}
        }},
        {"implementation", {
            {"script_path", gps_logging_json_or_null(source, "script_path")},
            {"container_name", gps_logging_json_or_null(source, "container_name")},
            {"legacy_setting_enabled", source.value("enabled", false)}
        }},
        {"error", error}
    };

    // Stable top-level compatibility fields make simple clients possible while
    // the structured blocks are preferred for new app implementations.
    root["state"] = state;
    root["request_active"] = request_active;
    root["armed"] = armed;
    root["running"] = running;
    root["session_id"] = gps_logging_json_or_null(source, "session_id");
    root["started_at"] = gps_logging_json_or_null(source, "started_at");
    root["finished_at"] = gps_logging_json_or_null(source, "finished_at");
    root["stop_reason"] = gps_logging_json_or_null(source, "stop_reason");
    return root;
}

static bool gps_logging_status_is_completed(const json &status) {
    if (!status.is_object() || !status.contains("runtime") || !status["runtime"].is_object()) return false;
    const json &runtime = status["runtime"];
    return !runtime.value("running", false) &&
           runtime.contains("session_id") && !runtime["session_id"].is_null() &&
           runtime.contains("finished_at") && !runtime["finished_at"].is_null();
}

static json build_gps_logging_last_payload(const json &status) {
    const json &runtime = status["runtime"];
    const json &request = status["request"];
    const json &storage = status["storage"];
    const json error = status.value("error", json(nullptr));
    const std::string stop_reason = runtime.contains("stop_reason") && runtime["stop_reason"].is_string()
        ? runtime["stop_reason"].get<std::string>() : std::string();
    std::string result = "finished";
    if (!error.is_null() || status.value("status", std::string()) == "error") result = "error";
    else if (stop_reason == "cancelled") result = "cancelled";

    return {
        {"schema", "openmower.gps_state.logging.last.v1"},
        {"type", "last"},
        {"published_at", ros::Time::now().toSec()},
        {"result", result},
        {"session_id", runtime.value("session_id", json(nullptr))},
        {"requested_at", runtime.value("requested_at", json(nullptr))},
        {"started_at", runtime.value("started_at", json(nullptr))},
        {"finished_at", runtime.value("finished_at", json(nullptr))},
        {"duration_s", runtime.value("duration_s", json(nullptr))},
        {"stop_reason", runtime.value("stop_reason", json(nullptr))},
        {"trigger", request.value("trigger", json(nullptr))},
        {"mode", request.value("mode", json(nullptr))},
        {"target_area_id", request.value("target_area_id", json(nullptr))},
        {"output_path", storage.value("output_path", json(nullptr))},
        {"files", storage.value("files", json::array())},
        {"error", error}
    };
}

void publish_gps_logging_status() {
    std::lock_guard<std::mutex> lk(gps_logging_status_mutex);
    if (latest_gps_logging_status_available) {
        try_publish("gps_state/logging/status/json", latest_gps_logging_status.dump(), true);
    }
}

void publish_gps_logging_last() {
    std::lock_guard<std::mutex> lk(gps_logging_status_mutex);
    if (last_completed_gps_logging_available) {
        try_publish("gps_state/logging/last/json", last_completed_gps_logging.dump(), true);
    }
}

void handle_gps_logging_control_payload(const std::string &payload_text) {
    json validation = {
        {"schema", "openmower.gps_state.logging.validation.v1"},
        {"type", "control"},
        {"valid", false},
        {"accepted", json::array()},
        {"rejected", json::array()},
        {"published_at", ros::Time::now().toSec()}
    };

    try {
        const json payload = json::parse(payload_text.empty() ? "{}" : payload_text);
        if (!payload.is_object()) {
            validation["rejected"].push_back({{"field", "$"}, {"reason", "payload must be a JSON object"}});
        } else if (!payload.contains("command") || !payload["command"].is_string()) {
            validation["rejected"].push_back({{"field", "command"}, {"reason", "command must be a string"}});
        } else {
            const std::string command = payload["command"].get<std::string>();
            if (command != "start" && command != "stop" && command != "cancel") {
                validation["rejected"].push_back({{"field", "command"}, {"reason", "unsupported command"}});
            }
            if (command == "start") {
                {
                    std::lock_guard<std::mutex> lk(gps_logging_status_mutex);
                    if (latest_gps_logging_status_available &&
                        latest_gps_logging_status.contains("runtime") &&
                        latest_gps_logging_status["runtime"].is_object()) {
                        const json &runtime = latest_gps_logging_status["runtime"];
                        if (runtime.value("request_active", false) || runtime.value("armed", false) ||
                            runtime.value("running", false)) {
                            validation["rejected"].push_back(
                                {{"field", "command"}, {"reason", "a logging request is already active"}});
                        }
                    }
                }
                if (payload.contains("trigger")) {
                    if (!payload["trigger"].is_string() ||
                        std::set<std::string>{"next_cycle", "ad_hoc", "area_id"}.count(payload["trigger"].get<std::string>()) == 0) {
                        validation["rejected"].push_back({{"field", "trigger"}, {"reason", "unsupported trigger"}});
                    }
                }
                if (payload.contains("mode")) {
                    if (!payload["mode"].is_string() ||
                        std::set<std::string>{"from_start_to_docking", "from_docking_to_docking", "until_docking", "manual", "area_only", "area_to_docking"}.count(payload["mode"].get<std::string>()) == 0) {
                        validation["rejected"].push_back({{"field", "mode"}, {"reason", "unsupported mode"}});
                    }
                }
                if (payload.contains("area_id") && !payload["area_id"].is_string() && !payload["area_id"].is_number_integer()) {
                    validation["rejected"].push_back({{"field", "area_id"}, {"reason", "area_id must be a string or integer"}});
                }
            }
            if (payload.contains("request_id")) validation["request_id"] = payload["request_id"];

            if (validation["rejected"].empty()) {
                validation["valid"] = true;
                validation["accepted"].push_back({{"command", command}});
                validation["status"] = "forwarded";
                std_msgs::String msg;
                msg.data = payload.dump();
                mower_logic_satellite_logging_control_pub.publish(msg);
            }
        }
    } catch (const json::exception &e) {
        validation["rejected"].push_back({{"field", "$"}, {"reason", std::string("Error decoding JSON: ") + e.what()}});
    }

    if (!validation.contains("status")) validation["status"] = "rejected";
    try_publish("gps_state/logging/validation/json", validation.dump(), false);
}


static std::string normalize_f9p_restart_token(std::string value) {
    value = trim_settings_string(value);
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
        if (c == '-' || c == ' ') return '_';
        return static_cast<char>(std::tolower(c));
    });
    return value;
}

static bool normalize_f9p_restart_request(const json &payload,
                                          std::string &mode,
                                          std::string &reset_mode,
                                          uint16_t &nav_bbr_mask,
                                          uint8_t &reset_mode_value,
                                          std::string &reason) {
    std::string raw_mode;
    std::string raw_reset_mode = "controlled_software";

    if (payload.is_string()) {
        raw_mode = payload.get<std::string>();
    } else if (payload.is_object()) {
        if (payload.contains("mode") && payload["mode"].is_string()) {
            raw_mode = payload["mode"].get<std::string>();
        } else if (payload.contains("command") && payload["command"].is_string()) {
            raw_mode = payload["command"].get<std::string>();
        } else if (payload.contains("restart") && payload["restart"].is_string()) {
            raw_mode = payload["restart"].get<std::string>();
        } else {
            reason = "payload must contain string field mode, command or restart";
            return false;
        }

        if (payload.contains("reset_mode")) {
            if (!payload["reset_mode"].is_string()) {
                reason = "reset_mode must be a string";
                return false;
            }
            raw_reset_mode = payload["reset_mode"].get<std::string>();
        }
    } else {
        reason = "payload must be a JSON object or string";
        return false;
    }

    mode = normalize_f9p_restart_token(raw_mode);
    reset_mode = normalize_f9p_restart_token(raw_reset_mode);

    if (mode == "hot" || mode == "hot_start") {
        mode = "hot_start";
        nav_bbr_mask = 0x0000;
    } else if (mode == "warm" || mode == "warm_start") {
        mode = "warm_start";
        nav_bbr_mask = 0x0001;
    } else if (mode == "cold" || mode == "cold_start") {
        mode = "cold_start";
        nav_bbr_mask = 0xffff;
    } else {
        reason = "unknown restart mode; allowed: hot_start, warm_start, cold_start";
        return false;
    }

    if (reset_mode.empty() || reset_mode == "default" || reset_mode == "controlled" ||
        reset_mode == "software" || reset_mode == "controlled_software") {
        reset_mode = "controlled_software";
        reset_mode_value = 0x01;
    } else if (reset_mode == "gnss" || reset_mode == "gnss_only" || reset_mode == "gnss_tasks") {
        reset_mode = "gnss_only";
        reset_mode_value = 0x02;
    } else if (reset_mode == "hardware" || reset_mode == "watchdog" || reset_mode == "hardware_watchdog") {
        reset_mode = "hardware_watchdog";
        reset_mode_value = 0x00;
    } else {
        reason = "unknown reset_mode; allowed: controlled_software, gnss_only, hardware_watchdog";
        return false;
    }
    return true;
}

static json default_gps_restart_status_payload() {
    return {
        {"available", true},
        {"status", "idle"},
        {"source", "xbot_monitoring"},
        {"command_topic", "gps_state/restart/set/json"},
        {"renew_topic", "gps_state/restart/set/renew/json"},
        {"ros_request_topic", "/ll/position/gps/restart_request"},
        {"ros_status_topic", "/ll/position/gps/restart_status"},
        {"modes", json::array({"hot_start", "warm_start", "cold_start"})},
        {"reset_modes", json::array({"controlled_software", "gnss_only", "hardware_watchdog"})},
        {"note", "UBX-CFG-RST may reset the receiver before an ACK is returned"}
    };
}

static void store_and_publish_gps_restart_status(json status) {
    if (!status.is_object()) {
        status = default_gps_restart_status_payload();
    }
    status["mqtt_topic"] = "gps_state/restart/status/json";
    bool completed = false;
    {
        std::lock_guard<std::mutex> lk(gps_restart_status_mutex);
        latest_gps_restart_status = status;
        latest_gps_restart_status_available = true;
        const std::string state = status.value("status", std::string());
        completed = state == "success" || state == "failed";
        if (completed) {
            last_completed_gps_restart = status;
            last_completed_gps_restart["mqtt_topic"] = "gps_state/restart/last/json";
            last_completed_gps_restart_available = true;
        }
    }
    try_publish("gps_state/restart/status/json", status.dump(), true);
    if (completed) {
        std::lock_guard<std::mutex> lk(gps_restart_status_mutex);
        try_publish("gps_state/restart/last/json", last_completed_gps_restart.dump(), true);
    }
}

void publish_gps_restart_status() {
    json status;
    {
        std::lock_guard<std::mutex> lk(gps_restart_status_mutex);
        status = latest_gps_restart_status_available ? latest_gps_restart_status : default_gps_restart_status_payload();
    }
    try_publish("gps_state/restart/status/json", status.dump(), true);
}

void publish_gps_restart_validation(const json &validation) {
    try_publish("gps_state/restart/validation/json", validation.dump(), false);
}

void handle_gps_restart_set_payload(const std::string &payload_text) {
    json validation = {
        {"valid", false},
        {"namespace", GPS_STATE_NAMESPACE},
        {"command", "f9p_restart"},
        {"accepted", json::object()},
        {"rejected", json::object()},
        {"remarks", json::array()}
    };

    try {
        json payload = json::parse(payload_text.empty() ? "{}" : payload_text, nullptr, false);
        if (payload.is_discarded()) {
            payload = trim_settings_string(payload_text);
        }

        std::string mode;
        std::string reset_mode;
        uint16_t nav_bbr_mask = 0;
        uint8_t reset_mode_value = 0;
        std::string reason;
        if (!normalize_f9p_restart_request(payload, mode, reset_mode, nav_bbr_mask, reset_mode_value, reason)) {
            validation["rejected"]["$"] = reason;
            publish_gps_restart_validation(validation);
            return;
        }

        const std::string ros_command = mode + ":" + reset_mode;
        std_msgs::String request_msg;
        request_msg.data = ros_command;
        gps_restart_request_pub.publish(request_msg);

        validation["valid"] = true;
        validation["accepted"]["mode"] = mode;
        validation["accepted"]["reset_mode"] = reset_mode;
        validation["accepted"]["nav_bbr_mask"] = nav_bbr_mask;
        validation["accepted"]["reset_mode_value"] = reset_mode_value;
        validation["accepted"]["ros_command"] = ros_command;
        validation["remarks"].push_back("F9P restart request forwarded to /ll/position/gps/restart_request");
        publish_gps_restart_validation(validation);

        store_and_publish_gps_restart_status({
            {"accepted", true},
            {"status", "requested"},
            {"source", "xbot_monitoring"},
            {"mode", mode},
            {"reset_mode", reset_mode},
            {"nav_bbr_mask", nav_bbr_mask},
            {"reset_mode_value", reset_mode_value},
            {"ros_command", ros_command},
            {"ros_request_topic", "/ll/position/gps/restart_request"},
            {"driver_ack_expected", false}
        });
    } catch (const json::exception &e) {
        validation["rejected"]["$"] = std::string("Error decoding JSON: ") + e.what();
        publish_gps_restart_validation(validation);
    }
}

void gps_restart_status_callback(const std_msgs::String::ConstPtr &msg) {
    json status = json::parse(msg->data, nullptr, false);
    if (!status.is_object()) {
        status = {
            {"accepted", false},
            {"status", "raw"},
            {"source", "xbot_driver_gps"},
            {"raw", msg->data}
        };
    }
    status["received_by"] = "xbot_monitoring";
    status["ros_status_topic"] = "/ll/position/gps/restart_status";
    store_and_publish_gps_restart_status(status);
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

static std::string gps_state_solution_from_pose(uint16_t flags) {
    if (flags & xbot_msgs::AbsolutePose::FLAG_GPS_RTK_FIXED) return "fixed";
    if (flags & xbot_msgs::AbsolutePose::FLAG_GPS_RTK_FLOAT) return "float";
    if (flags & xbot_msgs::AbsolutePose::FLAG_GPS_RTK) return "rtk";
    // /ll/position/gps is only published for an accepted position. Without RTK flags,
    // the best available classification is therefore a normal GNSS fix.
    return "gps_fix";
}

static std::string gps_state_solution_from_fix_status(const json &status) {
    if (!status.is_object()) return "unknown";

    if (status.contains("solution_state") && status["solution_state"].is_string()) {
        const std::string value = status["solution_state"].get<std::string>();
        if (value == "fixed" || value == "float" || value == "gps_fix" ||
            value == "no_fix" || value == "rtk") {
            return value;
        }
    }

    const bool gnss_fix_ok = status.value("gnss_fix_ok", false);
    const bool invalid_llh = status.value("invalid_llh", true);
    const bool position_valid = status.value("position_valid", false);
    if (!gnss_fix_ok || invalid_llh || !position_valid) return "no_fix";

    const std::string rtk = status.value("rtk_state", std::string("none"));
    if (rtk == "fixed") return "fixed";
    if (rtk == "float") return "float";

    const std::string fix_type = status.value("fix_type", std::string("no_fix"));
    if (fix_type == "2d_fix" || fix_type == "3d_fix" || fix_type == "gnss_dead_reckoning") {
        return "gps_fix";
    }
    return "no_fix";
}

static std::string gps_state_solution_display(const std::string &state,
                                              const std::string &fix_type) {
    if (state == "fixed") {
        return "RTK Fixed - eindeutige RTK-Loesung mit typischer Zentimetergenauigkeit";
    }
    if (state == "float") {
        return "RTK Float - Korrekturdaten vorhanden, Loesung noch nicht eindeutig";
    }
    if (state == "gps_fix") {
        if (fix_type == "2d_fix") {
            return "GPS Fix / 2D Fix - Position vorhanden, aber keine RTK-Loesung";
        }
        return "GPS Fix / 3D Fix - Position vorhanden, aber keine RTK-Loesung";
    }
    if (state == "rtk") {
        return "RTK-Korrekturen erkannt, aber weder Float noch Fixed gemeldet";
    }
    if (state == "no_fix") {
        if (fix_type == "dead_reckoning") {
            return "No Fix - nur Dead Reckoning, keine brauchbare GNSS-Position";
        }
        return "No Fix - keine brauchbare GNSS-Position";
    }
    return "Kein aktueller GPS-Status empfangen";
}

static std::string gps_state_accuracy_display(double value_m,
                                              double threshold_m,
                                              bool accuracy_ok,
                                              bool position_valid,
                                              const std::string &solution_state) {
    std::ostringstream out;
    out << std::fixed << std::setprecision(3) << value_m << " m";
    if (value_m < 1.0) {
        out << " (" << std::setprecision(1) << value_m * 100.0 << " cm)";
    }

    if (!position_valid || solution_state == "no_fix") {
        out << " - nicht belastbar, da keine gueltige GNSS-Position vorliegt";
    } else if (accuracy_ok) {
        out << " - ausreichend";
    } else {
        out << " - Grenzwert ueberschritten";
    }
    out << "; Grenzwert <= " << std::setprecision(3) << threshold_m << " m";
    return out.str();
}

static std::string gps_state_drive_block_reason(bool xb_pose_available,
                                                bool orientation_valid,
                                                bool recent_absolute_pose,
                                                bool xb_accuracy_ok,
                                                bool gps_input_available,
                                                const std::string &solution_state) {
    if (!xb_pose_available) return "no_xbot_positioning_pose";
    if (!orientation_valid) return "orientation_invalid";
    if (!recent_absolute_pose) {
        if (gps_input_available && solution_state == "float") return "rtk_float_not_sufficient";
        if (gps_input_available && solution_state == "gps_fix") return "gps_fix_not_sufficient";
        if (gps_input_available && solution_state == "no_fix") return "no_gnss_fix";
        if (!gps_input_available || solution_state == "unknown") return "gps_status_unavailable";
        if (solution_state != "fixed") return "rtk_fixed_missing";
        return "recent_absolute_pose_missing";
    }
    if (!xb_accuracy_ok) return "position_accuracy_too_low";
    return "unknown";
}

static std::string gps_state_drive_reason_text(const std::string &block_reason,
                                               const std::string &solution_state) {
    if (block_reason.empty()) return "RTK Fixed, Pose aktuell, Genauigkeit ausreichend";
    if (block_reason == "no_xbot_positioning_pose") return "Keine aktuelle Pose von xbot_positioning vorhanden";
    if (block_reason == "orientation_invalid") return "Orientierung ist nicht gueltig";
    if (block_reason == "rtk_float_not_sufficient") return "RTK Float reicht nicht zum Fahren aus";
    if (block_reason == "gps_fix_not_sufficient") return "GPS Fix / 3D Fix vorhanden, aber RTK Fixed fehlt";
    if (block_reason == "no_gnss_fix") return "No Fix: keine brauchbare GNSS-Position vorhanden";
    if (block_reason == "gps_status_unavailable") return "Kein aktueller GPS-Status vom Empfaenger vorhanden";
    if (block_reason == "rtk_fixed_missing") return "RTK Fixed fehlt; aktueller Zustand: " + solution_state;
    if (block_reason == "recent_absolute_pose_missing") return "Positioning-Pose enthaelt keine aktuelle absolute GPS-Pose";
    if (block_reason == "position_accuracy_too_low") return "Positionsgenauigkeit ist schlechter als der erlaubte Grenzwert";
    return "GPS-Fahrfreigabe konnte nicht bestaetigt werden";
}



static json gps_state0_definition_entry(int stage,
                                        const std::string &key,
                                        const std::string &title,
                                        const std::string &description,
                                        const std::string &source,
                                        const std::string &operator_text,
                                        const json &expected,
                                        const std::string &threshold_param,
                                        const json &default_threshold,
                                        const std::string &unit,
                                        const std::string &effect_if_failed,
                                        const std::string &next_check) {
    json entry = json::object();
    entry["stage"] = stage;
    entry["key"] = key;
    entry["title"] = title;
    entry["description"] = description;
    entry["source"] = source;
    if (!operator_text.empty()) entry["operator"] = operator_text;
    if (!expected.is_null()) entry["expected"] = expected;
    if (!threshold_param.empty()) entry["threshold_param"] = threshold_param;
    if (!default_threshold.is_null()) entry["default_threshold"] = default_threshold;
    if (!unit.empty()) entry["unit"] = unit;
    entry["effect_if_failed"] = effect_if_failed;
    entry["next_check"] = next_check;
    return entry;
}

static json build_gps_state0_definition_payload() {
    json root = json::object();
    root["schema"] = GPS_STATE_PAYLOAD_SCHEMA;
    root["state"] = "state0";
    root["type"] = "definition";
    root["definition_version"] = GPS_STATE_DEFINITION_VERSION;
    root["name"] = "gps_drive_diagnostics";
    root["label"] = "GPS-Fahrdiagnose";
    root["description"] = "Statische 12-Stufen-Definition der GPS-Fahrfaehigkeitsdiagnose. Die Live-Werte stehen in gps_state/state0/status.";
    root["role"] = "expert";
    root["order"] = 0;
    root["status_retained"] = true;
    root["update_mode"] = "periodic_and_event";
    root["fields"] = {
        {"drive_ready", {{"label", "Fahrfreigabe"}, {"description", "Gesamtergebnis der GPS-Fahrfaehigkeitspruefung."}, {"type", "bool"}, {"order", 10}, {"expert", false}}},
        {"drive_state", {{"label", "Fahrzustand"}, {"description", "ready, blocked oder stop."}, {"type", "string"}, {"order", 20}, {"expert", false}}},
        {"checks", {{"label", "Pruefstufen"}, {"description", "Live-Ergebnisse der 12 Entscheidungsknoten."}, {"type", "object"}, {"order", 30}, {"expert", true}}}
    };
    root["checks"] = json::object();
    root["checks"]["01_gps_enabled"] = gps_state0_definition_entry(
        1, "gps_enabled", "GPS-Verarbeitung aktiv?",
        "Prueft, ob xbot_positioning GPS-Updates grundsaetzlich verarbeitet.",
        "xbot_positioning.gps_enabled", "==", true, "", nullptr, "",
        "GPS-Update wird verworfen", "bei Konfigurationsaenderung oder naechstem GPS-Update");
    root["checks"]["02_rtk_fixed"] = gps_state0_definition_entry(
        2, "rtk_fixed", "RTK Fixed vorhanden?",
        "Prueft den aktuellen GNSS- und RTK-Zustand. Moegliche Zustaende: No Fix = keine brauchbare Position; GPS Fix oder 3D Fix = normale Satellitenposition ohne eindeutige RTK-Loesung, typischerweise etwa 0,5 bis 3 m Genauigkeit; RTK Float = Korrekturdaten werden verwendet, die Traegerphasenmehrdeutigkeiten sind aber noch nicht eindeutig geloest, typischerweise etwa 0,05 bis 0,5 m Genauigkeit; RTK Fixed = eindeutige RTK-Loesung mit typischer Zentimetergenauigkeit. Nur RTK Fixed erfuellt diese Pruefung.",
        "/ll/position/gps/fix_status.solution_state", "==", "fixed", "", nullptr, "",
        "Bei No Fix, GPS Fix, 3D Fix oder RTK Float wird das GPS-Update fuer die Fahrfreigabe verworfen.",
        "beim naechsten GPS-Update");
    root["checks"]["03_gps_input_accuracy"] = gps_state0_definition_entry(
        3, "gps_input_accuracy", "GPS-Genauigkeit Eingang gut?",
        "Prueft die vom GPS-Empfaenger gemeldete Positionsgenauigkeit in Metern. Ein Wert von 0,02 m entspricht beispielsweise 2 cm. Als typische Orientierung gilt: RTK Fixed meist etwa 0,01 bis 0,03 m, RTK Float oft etwa 0,05 bis 0,5 m und ein normaler GPS- oder 3D-Fix oft etwa 0,5 bis 3 m. Diese Bereiche sind keine festen Zustandsgrenzen. Fuer diese Pruefung ist ausschliesslich der konfigurierte Grenzwert max_gps_accuracy massgeblich.",
        "/ll/position/gps/fix_status.position_accuracy_m", "<=", nullptr, "/xbot_positioning/max_gps_accuracy", 0.2, "m",
        "Die gemeldete GPS-Genauigkeit liegt ueber dem erlaubten Grenzwert oder es liegt keine gueltige GNSS-Position vor; das GPS-Update wird verworfen.",
        "beim naechsten GPS-Update");
    root["checks"]["04_valid_gps_samples"] = gps_state0_definition_entry(
        4, "valid_gps_samples", "Genuegend gueltige GPS-Samples?",
        "Prueft, ob nach Start oder Reset mehr als 10 gueltige GPS-Updates gesammelt wurden.",
        "xbot_positioning.valid_gps_samples", ">", nullptr, "", 10, "samples",
        "Initialisierung bleibt instabil", "bei jedem gueltigen GPS-Update");
    root["checks"]["05_absolute_gps_pose_recent"] = gps_state0_definition_entry(
        5, "absolute_gps_pose_recent", "Absolute GPS-Pose aktuell?",
        "Prueft, ob die letzte akzeptierte absolute GPS-Position juenger als 10 Sekunden ist.",
        "xbot_positioning.last_accepted_gps", "<", nullptr, "", 10.0, "s",
        "RECENT_ABSOLUTE_POSE fehlt", "bei neuem gueltigem GPS-Update");
    root["checks"]["06_xb_pose_received"] = gps_state0_definition_entry(
        6, "xb_pose_received", "Aktuelle xb_pose vorhanden?",
        "Prueft, ob mower_logic eine Pose von /xbot_positioning/xb_pose empfangen kann.",
        "/xbot_positioning/xb_pose", "==", true, "", nullptr, "",
        "Fahrfreigabe blockiert", "im naechsten mower_logic-Zyklus");
    root["checks"]["07_xb_pose_age"] = gps_state0_definition_entry(
        7, "xb_pose_age", "Pose aktuell?",
        "Prueft, ob die empfangene xb_pose juenger als 1 Sekunde ist.",
        "/xbot_positioning/xb_pose.header.stamp", "<", nullptr, "", 1.0, "s",
        "Sicherheitsstopp: Messer aus, Fahren aus", "bei neuer aktueller Pose");
    root["checks"]["08_orientation_valid"] = gps_state0_definition_entry(
        8, "orientation_valid", "Orientierung gueltig?",
        "Prueft, ob die Orientierung der Pose gueltig ist.",
        "/xbot_positioning/xb_pose.orientation_valid", "==", true, "", nullptr, "",
        "Keine sichere Navigation", "bei neuer gueltiger Pose");
    root["checks"]["09_pose_accuracy"] = gps_state0_definition_entry(
        9, "pose_accuracy", "Positionsgenauigkeit ausreichend?",
        "Prueft die von xbot_positioning erzeugte Positionsgenauigkeit der aktuellen Pose in Metern. Dieser Wert kann sich von der direkten GPS-Eingangsgenauigkeit aus Stufe 3 unterscheiden, da hier die ausgegebene beziehungsweise sensorfusionierte Roboterpose bewertet wird. Ein Wert von 0,02 m entspricht 2 cm. Fuer die Fahrfreigabe muss der Wert unter dem konfigurierten Grenzwert max_position_accuracy liegen.",
        "/xbot_positioning/xb_pose.position_accuracy", "<", nullptr, "/mower_logic/max_position_accuracy", 0.2, "m",
        "Die Genauigkeit der aktuellen Roboterpose liegt ausserhalb des erlaubten Bereichs und die Fahrfreigabe bleibt blockiert.",
        "bei verbesserter Pose");
    root["checks"]["10_recent_absolute_pose"] = gps_state0_definition_entry(
        10, "recent_absolute_pose", "Aktuelle absolute GPS-Pose vorhanden?",
        "Prueft, ob die Sensorfusion eine aktuelle absolute GPS-Pose meldet.",
        "/xbot_positioning/xb_pose.flags", "contains", "FLAG_SENSOR_FUSION_RECENT_ABSOLUTE_POSE", "", nullptr, "",
        "Weiter zur Toleranzpruefung", "bei aktueller absoluter Pose");
    root["checks"]["11_gps_timeout"] = gps_state0_definition_entry(
        11, "gps_timeout", "Letzter guter GPS-Zustand innerhalb Toleranz?",
        "Prueft, ob der letzte gute GPS-Zustand noch innerhalb gps_timeout liegt.",
        "mower_logic.last_good_gps", "<", nullptr, "/mower_logic/gps_timeout", 10.0, "s",
        "GPS-Timeout: Fahren und Messer aus", "nach wieder gueltiger Pose");
    root["checks"]["12_gps_drive_ready"] = gps_state0_definition_entry(
        12, "gps_drive_ready", "Fahrfreigabe erreicht?",
        "Gesamtergebnis der GPS-Fahrfaehigkeitspruefung.",
        "gps_state/state0/status.drive_ready", "==", true, "", nullptr, "",
        "Roboter darf GPS-abhaengig nicht fahren", "wenn alle vorherigen Stufen erfuellt sind");
    return root;
}


static json gps_state_field_definition(const std::string &label,
                                       const std::string &description,
                                       const std::string &type,
                                       int order,
                                       const std::string &unit = "",
                                       bool expert = false) {
    json field = {
        {"label", label},
        {"description", description},
        {"type", type},
        {"order", order},
        {"expert", expert}
    };
    if (!unit.empty()) field["unit"] = unit;
    return field;
}

static json gps_state_definition_base(int state,
                                      const std::string &name,
                                      const std::string &label,
                                      const std::string &description,
                                      const std::string &role,
                                      int order,
                                      bool status_retained) {
    return {
        {"schema", GPS_STATE_PAYLOAD_SCHEMA},
        {"state", "state" + std::to_string(state)},
        {"type", "definition"},
        {"definition_version", GPS_STATE_DEFINITION_VERSION},
        {"name", name},
        {"label", label},
        {"description", description},
        {"role", role},
        {"order", order},
        {"status_retained", status_retained},
        {"update_mode", state >= 3 ? "sensor_event" : "periodic_and_event"},
        {"fields", json::object()}
    };
}

static json build_gps_state1_definition_payload() {
    json root = gps_state_definition_base(
        1, "gps_operator_status", "GPS-Fahrstatus",
        "Kompakter Bedienerstatus zur GPS-seitigen Fahrfreigabe.",
        "operator", 10, true);
    json &fields = root["fields"];
    fields["quality_class"] = gps_state_field_definition("Qualitaetsklasse", "Kompakte Einordnung der GPS-Qualitaet.", "string", 10);
    fields["gps_drive_ready"] = gps_state_field_definition("GPS ausreichend", "Gibt an, ob GPS-seitig gefahren werden darf.", "bool", 20);
    fields["gps_drive_state"] = gps_state_field_definition("Fahrzustand", "ready, blocked oder stop.", "string", 30);
    fields["gps_drive_label"] = gps_state_field_definition("Kurzstatus", "Direkt anzeigbarer Fahrstatus.", "string", 40);
    fields["gps_drive_reason"] = gps_state_field_definition("Begruendung", "Direkt anzeigbare Begruendung des Zustands.", "string", 50);
    fields["gps_drive_block_reason"] = gps_state_field_definition("Blockiergrund", "Technischer Schluessel des Blockiergrunds.", "string_or_null", 60, "", true);
    fields["rtk_state"] = gps_state_field_definition("RTK-Zustand", "Aktueller GNSS-/RTK-Loesungszustand.", "string", 70);
    fields["position_accuracy_m"] = gps_state_field_definition("Positionsgenauigkeit", "Genauigkeit der aktuellen Roboterpose.", "double_or_null", 80, "m");
    fields["max_position_accuracy_m"] = gps_state_field_definition("Erlaubte Genauigkeit", "Grenzwert der mower_logic-Fahrfreigabe.", "double", 90, "m");
    fields["pose_age_ms"] = gps_state_field_definition("Pose-Alter", "Alter der zuletzt empfangenen Roboterpose.", "integer_or_null", 100, "ms");
    return root;
}

static json build_gps_state2_definition_payload() {
    json root = gps_state_definition_base(
        2, "gps_technical_summary", "GPS-Technikstatus",
        "Technische GNSS-, RTK- und Pose-Zusammenfassung ohne Satellitenliste.",
        "technical", 20, true);
    json &fields = root["fields"];
    fields["available"] = gps_state_field_definition("GNSS-Daten vorhanden", "Zeigt, ob eine aktuelle Satellitenmeldung vorhanden ist.", "bool", 10);
    fields["quality_class"] = gps_state_field_definition("Qualitaetsklasse", "Aus verwendeten Satelliten und C/N0 abgeleitete Klasse.", "string", 20);
    fields["visible_count"] = gps_state_field_definition("Sichtbare Satelliten", "Anzahl sichtbarer Satelliten.", "integer", 30);
    fields["used_count"] = gps_state_field_definition("Verwendete Satelliten", "Anzahl fuer den Fix verwendeter Satelliten.", "integer", 40);
    fields["avg_cn0"] = gps_state_field_definition("C/N0 Mittelwert", "Mittleres C/N0 der verwendeten Satelliten.", "double", 50, "dB-Hz");
    fields["min_cn0"] = gps_state_field_definition("C/N0 Minimum", "Niedrigstes C/N0 der verwendeten Satelliten.", "double", 60, "dB-Hz");
    fields["max_cn0"] = gps_state_field_definition("C/N0 Maximum", "Hoechstes C/N0 der verwendeten Satelliten.", "double", 70, "dB-Hz");
    fields["weak_count"] = gps_state_field_definition("Schwache Satelliten", "Anzahl unterhalb der konfigurierten Schwelle.", "integer", 80);
    fields["good_count"] = gps_state_field_definition("Gute Satelliten", "Anzahl oberhalb der konfigurierten Schwelle.", "integer", 90);
    fields["systems"] = gps_state_field_definition("GNSS-Systeme", "Verteilung sichtbarer und verwendeter Satelliten nach System.", "object", 100);
    fields["rtk_state"] = gps_state_field_definition("RTK-Zustand", "Aktueller GNSS-/RTK-Loesungszustand.", "string", 110);
    fields["ll_gps_accuracy_m"] = gps_state_field_definition("GPS-Eingangsgenauigkeit", "Vom Empfaenger gemeldete Genauigkeit.", "double_or_null", 120, "m");
    fields["xb_pose_accuracy_m"] = gps_state_field_definition("Pose-Genauigkeit", "Genauigkeit der ausgegebenen Roboterpose.", "double_or_null", 130, "m");
    fields["orientation_valid"] = gps_state_field_definition("Orientierung gueltig", "Gueltigkeit der Pose-Orientierung.", "bool_or_null", 140);
    fields["recent_absolute_pose"] = gps_state_field_definition("Absolute Pose aktuell", "Kennzeichen fuer eine aktuelle absolute GPS-Pose.", "bool_or_null", 150);
    fields["gps_timeout"] = gps_state_field_definition("GPS-Timeout", "Zeigt einen ueberschrittenen GPS-Toleranzzeitraum.", "bool", 160);
    fields["diagnostic_summary"] = gps_state_field_definition("Diagnose", "Direkt anzeigbare technische Zusammenfassung.", "string", 170);
    fields["drive_diagnostics"] = gps_state_field_definition("Fahrdiagnose Details", "Technische Detailwerte der Fahrfreigabe.", "object", 180, "", true);
    return root;
}

static json build_gps_state3_definition_payload() {
    json root = gps_state_definition_base(
        3, "gps_used_satellites", "Verwendete Satelliten",
        "Liste der aktuell fuer den Positionsfix verwendeten Satelliten.",
        "technical", 30, false);
    root["fields"]["used_count"] = gps_state_field_definition("Verwendete Satelliten", "Anzahl der Listeneintraege.", "integer", 10);
    root["fields"]["satellites"] = gps_state_field_definition("Satelliten", "Liste mit GNSS-System, SV-ID, C/N0, Elevation, Azimut, Residual und Qualitaet.", "array", 20);
    root["item_fields"] = {
        {"gnss", gps_state_field_definition("GNSS-System", "Name des GNSS-Systems.", "string", 10)},
        {"gnss_id", gps_state_field_definition("GNSS-ID", "Numerische Systemkennung.", "integer", 20)},
        {"sv", gps_state_field_definition("SV-ID", "Satellitenkennung.", "integer", 30)},
        {"cn0", gps_state_field_definition("C/N0", "Signal-Rausch-Verhaeltnis.", "double", 40, "dB-Hz")},
        {"elev", gps_state_field_definition("Elevation", "Hoehenwinkel.", "double", 50, "deg")},
        {"azim", gps_state_field_definition("Azimut", "Richtungswinkel.", "double", 60, "deg")},
        {"prres", gps_state_field_definition("Pseudorange-Residual", "Residualwert des Satelliten.", "double", 70)},
        {"qual", gps_state_field_definition("Qualitaetsindikator", "Empfaengerseitiger Qualitaetsindikator.", "integer", 80)}
    };
    return root;
}

static json build_gps_state4_definition_payload() {
    json root = gps_state_definition_base(
        4, "gps_visible_satellites", "Alle sichtbaren Satelliten",
        "Vollstaendige Experten- und Debugliste aller sichtbaren Satelliten.",
        "expert", 40, false);
    root["fields"]["available"] = gps_state_field_definition("Satellitendaten vorhanden", "Zeigt, ob eine aktuelle Satellitenmeldung vorhanden ist.", "bool", 10);
    root["fields"]["quality_class"] = gps_state_field_definition("Qualitaetsklasse", "Aggregierte Qualitaetsklasse.", "string", 20);
    root["fields"]["visible_count"] = gps_state_field_definition("Sichtbare Satelliten", "Anzahl aller sichtbaren Satelliten.", "integer", 30);
    root["fields"]["used_count"] = gps_state_field_definition("Verwendete Satelliten", "Anzahl der verwendeten Satelliten.", "integer", 40);
    root["fields"]["avg_cn0"] = gps_state_field_definition("C/N0 Mittelwert", "Mittleres C/N0 der verwendeten Satelliten.", "double", 50, "dB-Hz");
    root["fields"]["min_cn0"] = gps_state_field_definition("C/N0 Minimum", "Niedrigstes C/N0 der verwendeten Satelliten.", "double", 60, "dB-Hz");
    root["fields"]["max_cn0"] = gps_state_field_definition("C/N0 Maximum", "Hoechstes C/N0 der verwendeten Satelliten.", "double", 70, "dB-Hz");
    root["fields"]["satellites"] = gps_state_field_definition("Satelliten", "Vollstaendige Liste mit used=true/false.", "array", 80);
    const json state3_definition = build_gps_state3_definition_payload();
    root["item_fields"] = state3_definition["item_fields"];
    root["item_fields"]["used"] = gps_state_field_definition("Verwendet", "Wird der Satellit aktuell fuer den Fix verwendet?", "bool", 35);
    return root;
}

static json build_gps_state_definition_payload(int state) {
    switch (state) {
        case 0: return build_gps_state0_definition_payload();
        case 1: return build_gps_state1_definition_payload();
        case 2: return build_gps_state2_definition_payload();
        case 3: return build_gps_state3_definition_payload();
        case 4: return build_gps_state4_definition_payload();
        default: return json::object();
    }
}

static std::string gps_state0_display_number(double value, const std::string &unit, int precision = 2) {
    std::ostringstream out;
    out << std::fixed << std::setprecision(precision) << value;
    if (!unit.empty()) out << " " << unit;
    return out.str();
}

static json gps_state0_live_entry(int stage,
                                  const std::string &key,
                                  const std::string &status,
                                  int severity,
                                  const json &value,
                                  const json &expected,
                                  const json &threshold,
                                  const json &deviation,
                                  const std::string &display) {
    json entry = json::object();
    entry["stage"] = stage;
    entry["key"] = key;
    entry["status"] = status;
    entry["severity"] = severity;
    entry["value"] = value;
    if (!expected.is_null()) entry["expected"] = expected;
    if (!threshold.is_null()) entry["threshold"] = threshold;
    if (!deviation.is_null()) entry["deviation"] = deviation;
    entry["display"] = display;
    return entry;
}

static void gps_state0_add_live_check(json &checks,
                                      const std::string &id,
                                      int stage,
                                      const std::string &key,
                                      const std::string &status,
                                      int severity,
                                      const json &value,
                                      const json &expected,
                                      const json &threshold,
                                      const json &deviation,
                                      const std::string &display,
                                      bool &blocking_found,
                                      int &blocking_stage,
                                      std::string &blocking_key,
                                      std::string &blocking_title,
                                      std::string &summary) {
    checks[id] = gps_state0_live_entry(stage, key, status, severity, value, expected, threshold, deviation, display);
    if (!blocking_found && (status == "blocked" || status == "stop")) {
        blocking_found = true;
        blocking_stage = stage;
        blocking_key = key;
        blocking_title = build_gps_state0_definition_payload()["checks"][id]["title"].get<std::string>();
        summary = display;
    }
}

static json build_gps_state0_status_payload(const ros::Time &now,
                                            double max_position_accuracy,
                                            double max_gps_accuracy,
                                            double gps_timeout,
                                            const ros::Time &gps_input_received_at,
                                            const xbot_msgs::AbsolutePose &xb_pose,
                                            bool xb_pose_available,
                                            const ros::Time &xb_received_at,
                                            const ros::Time &drive_ready_at,
                                            const std::string &solution_state,
                                            const std::string &solution_display,
                                            bool gps_input_available,
                                            bool input_position_valid,
                                            bool input_accuracy_available,
                                            double input_accuracy_m,
                                            bool ll_rtk_fixed,
                                            bool ll_accuracy_ok,
                                            bool orientation_valid,
                                            bool recent_absolute_pose,
                                            bool xb_accuracy_ok,
                                            bool gps_drive_ready,
                                            double grace_remaining_s,
                                            bool gps_timeout_estimated) {
    json positioning_debug = json::object();
    bool positioning_debug_available = false;
    ros::Time positioning_debug_received_at;
    {
        std::lock_guard<std::mutex> lk(gps_state_pose_mutex);
        positioning_debug = latest_xbot_positioning_gps_debug;
        positioning_debug_available = latest_xbot_positioning_gps_debug_available;
        positioning_debug_received_at = latest_xbot_positioning_gps_debug_received_at;
    }

    json checks = json::object();
    bool blocking_found = false;
    int blocking_stage = 0;
    std::string blocking_key;
    std::string blocking_title;
    std::string summary;

    auto inactive_if_blocked = [&](const std::string &id, int stage, const std::string &key, const std::string &display) {
        gps_state0_add_live_check(checks, id, stage, key, "inactive", 0, nullptr, nullptr, nullptr, nullptr, display,
                                  blocking_found, blocking_stage, blocking_key, blocking_title, summary);
    };

    bool gps_enabled = true;
    bool gps_enabled_known = false;
    if (positioning_debug_available && positioning_debug.contains("gps_enabled") && positioning_debug["gps_enabled"].is_boolean()) {
        gps_enabled = positioning_debug["gps_enabled"].get<bool>();
        gps_enabled_known = true;
    }
    gps_state0_add_live_check(checks, "01_gps_enabled", 1, "gps_enabled",
                              gps_enabled_known ? (gps_enabled ? "ok" : "blocked") : "unknown",
                              gps_enabled_known ? (gps_enabled ? 0 : 3) : 1,
                              gps_enabled_known ? json(gps_enabled) : json(nullptr), true, nullptr, nullptr,
                              gps_enabled_known ? (gps_enabled ? "aktiv" : "inaktiv") : "unbekannt",
                              blocking_found, blocking_stage, blocking_key, blocking_title, summary);

    if (blocking_found) {
        inactive_if_blocked("02_rtk_fixed", 2, "rtk_fixed", "nicht bewertet");
    } else {
        const std::string status = gps_input_available ? (ll_rtk_fixed ? "ok" : "blocked") : "blocked";
        const std::string display = gps_input_available ? solution_display : "Kein aktueller GPS-Status empfangen";
        gps_state0_add_live_check(checks, "02_rtk_fixed", 2, "rtk_fixed", status, ll_rtk_fixed ? 0 : 3,
                                  gps_input_available ? json(solution_state) : json(nullptr), "fixed", nullptr, nullptr, display,
                                  blocking_found, blocking_stage, blocking_key, blocking_title, summary);
    }

    // Stufe 3 bleibt absichtlich sichtbar, auch wenn Stufe 2 bereits blockiert. So kann
    // die aktuelle Empfaenger-Genauigkeit bei GPS Fix oder RTK Float beurteilt werden.
    if (!input_accuracy_available) {
        gps_state0_add_live_check(checks, "03_gps_input_accuracy", 3, "gps_input_accuracy",
                                  "blocked", 3, nullptr, nullptr, max_gps_accuracy, nullptr,
                                  "Keine aktuelle GPS-Eingangsgenauigkeit vorhanden",
                                  blocking_found, blocking_stage, blocking_key, blocking_title, summary);
    } else {
        const double deviation = input_accuracy_m - max_gps_accuracy;
        gps_state0_add_live_check(checks, "03_gps_input_accuracy", 3, "gps_input_accuracy",
                                  ll_accuracy_ok ? "ok" : "blocked", ll_accuracy_ok ? 0 : 3,
                                  input_accuracy_m, nullptr, max_gps_accuracy, deviation,
                                  gps_state_accuracy_display(input_accuracy_m, max_gps_accuracy, ll_accuracy_ok,
                                                             input_position_valid, solution_state),
                                  blocking_found, blocking_stage, blocking_key, blocking_title, summary);
    }

    if (blocking_found) {
        inactive_if_blocked("04_valid_gps_samples", 4, "valid_gps_samples", "nicht bewertet");
    } else {
        int samples = -1;
        bool has_gps_value = false;
        if (positioning_debug_available) {
            if (positioning_debug.contains("valid_gps_samples") && positioning_debug["valid_gps_samples"].is_number_integer()) {
                samples = positioning_debug["valid_gps_samples"].get<int>();
            }
            if (positioning_debug.contains("has_gps") && positioning_debug["has_gps"].is_boolean()) {
                has_gps_value = positioning_debug["has_gps"].get<bool>();
            }
        }
        const bool samples_ok = has_gps_value || samples > 10;
        const std::string display = samples >= 0 ? (std::to_string(samples) + " / 11 Samples") : "Sample-Anzahl unbekannt";
        gps_state0_add_live_check(checks, "04_valid_gps_samples", 4, "valid_gps_samples",
                                  samples_ok ? "ok" : (samples >= 0 ? "blocked" : "unknown"), samples_ok ? 0 : (samples >= 0 ? 3 : 1),
                                  samples >= 0 ? json(samples) : json(nullptr), nullptr, 10, samples >= 0 ? json(10 - samples) : json(nullptr), display,
                                  blocking_found, blocking_stage, blocking_key, blocking_title, summary);
    }

    if (blocking_found) {
        inactive_if_blocked("05_absolute_gps_pose_recent", 5, "absolute_gps_pose_recent", "nicht bewertet");
    } else {
        json age_value = nullptr;
        double age_s = -1.0;
        if (positioning_debug_available && positioning_debug.contains("last_accepted_gps_age_s") && positioning_debug["last_accepted_gps_age_s"].is_number()) {
            age_s = positioning_debug["last_accepted_gps_age_s"].get<double>();
            age_value = age_s;
        }
        const bool recent = age_s >= 0.0 && age_s < 10.0;
        gps_state0_add_live_check(checks, "05_absolute_gps_pose_recent", 5, "absolute_gps_pose_recent",
                                  age_s >= 0.0 ? (recent ? "ok" : "blocked") : "unknown", recent ? 0 : (age_s >= 0.0 ? 3 : 1),
                                  age_value, nullptr, 10.0, age_s >= 0.0 ? json(age_s - 10.0) : json(nullptr),
                                  age_s >= 0.0 ? (gps_state0_display_number(age_s, "s") + " / max. 10.00 s") : "Alter unbekannt",
                                  blocking_found, blocking_stage, blocking_key, blocking_title, summary);
    }

    if (blocking_found) {
        inactive_if_blocked("06_xb_pose_received", 6, "xb_pose_received", "nicht bewertet");
    } else {
        gps_state0_add_live_check(checks, "06_xb_pose_received", 6, "xb_pose_received",
                                  xb_pose_available ? "ok" : "blocked", xb_pose_available ? 0 : 3,
                                  xb_pose_available, true, nullptr, nullptr,
                                  xb_pose_available ? "empfangen" : "keine xb_pose empfangen",
                                  blocking_found, blocking_stage, blocking_key, blocking_title, summary);
    }

    const double xb_pose_age_s = xb_pose_available ? std::max(0.0, (now - xb_received_at).toSec()) : -1.0;
    if (blocking_found) {
        inactive_if_blocked("07_xb_pose_age", 7, "xb_pose_age", "nicht bewertet");
    } else {
        const bool age_ok = xb_pose_age_s >= 0.0 && xb_pose_age_s < 1.0;
        gps_state0_add_live_check(checks, "07_xb_pose_age", 7, "xb_pose_age",
                                  age_ok ? "ok" : "stop", age_ok ? 0 : 4,
                                  xb_pose_age_s >= 0.0 ? json(xb_pose_age_s) : json(nullptr), nullptr, 1.0,
                                  xb_pose_age_s >= 0.0 ? json(xb_pose_age_s - 1.0) : json(nullptr),
                                  xb_pose_age_s >= 0.0 ? (gps_state0_display_number(xb_pose_age_s, "s") + " / max. 1.00 s") : "Pose-Alter unbekannt",
                                  blocking_found, blocking_stage, blocking_key, blocking_title, summary);
    }

    if (blocking_found) {
        inactive_if_blocked("08_orientation_valid", 8, "orientation_valid", "nicht bewertet");
    } else {
        gps_state0_add_live_check(checks, "08_orientation_valid", 8, "orientation_valid",
                                  orientation_valid ? "ok" : "blocked", orientation_valid ? 0 : 3,
                                  xb_pose_available ? json(static_cast<bool>(xb_pose.orientation_valid)) : json(nullptr), true, nullptr, nullptr,
                                  orientation_valid ? "gueltig" : "ungueltig",
                                  blocking_found, blocking_stage, blocking_key, blocking_title, summary);
    }

    if (blocking_found) {
        inactive_if_blocked("09_pose_accuracy", 9, "pose_accuracy", "nicht bewertet");
    } else {
        const double value = static_cast<double>(xb_pose.position_accuracy);
        const double deviation = value - max_position_accuracy;
        gps_state0_add_live_check(checks, "09_pose_accuracy", 9, "pose_accuracy",
                                  xb_accuracy_ok ? "ok" : "blocked", xb_accuracy_ok ? 0 : 3,
                                  value, nullptr, max_position_accuracy, deviation,
                                  gps_state0_display_number(value, "m") + " / max. < " + gps_state0_display_number(max_position_accuracy, "m"),
                                  blocking_found, blocking_stage, blocking_key, blocking_title, summary);
    }

    bool stage10_failed = false;
    if (blocking_found) {
        inactive_if_blocked("10_recent_absolute_pose", 10, "recent_absolute_pose", "nicht bewertet");
    } else {
        stage10_failed = !recent_absolute_pose;
        gps_state0_add_live_check(checks, "10_recent_absolute_pose", 10, "recent_absolute_pose",
                                  recent_absolute_pose ? "ok" : "warning", recent_absolute_pose ? 0 : 2,
                                  recent_absolute_pose, true, nullptr, nullptr,
                                  recent_absolute_pose ? "aktuelle absolute Pose vorhanden" : "fehlt - Toleranzpruefung",
                                  blocking_found, blocking_stage, blocking_key, blocking_title, summary);
    }

    if (blocking_found) {
        inactive_if_blocked("11_gps_timeout", 11, "gps_timeout", "nicht bewertet");
    } else if (stage10_failed) {
        const double since_ready_s = drive_ready_at.isZero() ? gps_timeout + 1.0 : std::max(0.0, (now - drive_ready_at).toSec());
        const bool within_timeout = !gps_timeout_estimated;
        gps_state0_add_live_check(checks, "11_gps_timeout", 11, "gps_timeout",
                                  within_timeout ? "warning" : "stop", within_timeout ? 2 : 4,
                                  since_ready_s, nullptr, gps_timeout, since_ready_s - gps_timeout,
                                  within_timeout ? ("noch " + gps_state0_display_number(grace_remaining_s, "s") + " Resttoleranz")
                                                 : (gps_state0_display_number(since_ready_s, "s") + " / max. " + gps_state0_display_number(gps_timeout, "s")),
                                  blocking_found, blocking_stage, blocking_key, blocking_title, summary);
    } else {
        gps_state0_add_live_check(checks, "11_gps_timeout", 11, "gps_timeout", "ok", 0, 0.0, nullptr, gps_timeout, nullptr,
                                  "nicht erforderlich", blocking_found, blocking_stage, blocking_key, blocking_title, summary);
    }

    gps_state0_add_live_check(checks, "12_gps_drive_ready", 12, "gps_drive_ready",
                              gps_drive_ready ? "ok" : (gps_timeout_estimated ? "stop" : "blocked"),
                              gps_drive_ready ? 0 : (gps_timeout_estimated ? 4 : 3),
                              gps_drive_ready, true, nullptr, nullptr,
                              gps_drive_ready ? "fahrbereit" : "nicht fahrbereit",
                              blocking_found, blocking_stage, blocking_key, blocking_title, summary);

    json root = json::object();
    root["schema"] = GPS_STATE_PAYLOAD_SCHEMA;
    root["state"] = "state0";
    root["type"] = "status";
    root["definition_version"] = GPS_STATE_DEFINITION_VERSION;
    root["timestamp"] = now.toSec();
    root["drive_ready"] = gps_drive_ready;
    root["drive_state"] = gps_drive_ready ? "ready" : (gps_timeout_estimated ? "stop" : "blocked");
    root["severity"] = gps_drive_ready ? 0 : (gps_timeout_estimated ? 4 : 3);
    root["blocking_stage"] = blocking_found ? json(blocking_stage) : json(nullptr);
    root["blocking_key"] = blocking_found ? json(blocking_key) : json(nullptr);
    root["blocking_title"] = blocking_found ? json(blocking_title) : json(nullptr);
    root["summary"] = blocking_found ? summary : "Alle Bedingungen erfuellt";
    root["positioning_debug_available"] = positioning_debug_available;
    root["positioning_debug_age_ms"] = gps_state_age_ms_json(now, positioning_debug_received_at);
    root["ll_gps_age_ms"] = gps_state_age_ms_json(now, gps_input_received_at);
    root["xb_pose_age_ms"] = gps_state_age_ms_json(now, xb_received_at);
    root["checks"] = checks;
    return root;
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
    json fix_status = json::object();
    bool ll_pose_available = false;
    bool xb_pose_available = false;
    bool fix_status_available = false;
    ros::Time ll_received_at;
    ros::Time xb_received_at;
    ros::Time fix_status_received_at;
    ros::Time drive_ready_at;
    {
        std::lock_guard<std::mutex> lk(gps_state_pose_mutex);
        ll_pose = latest_ll_gps_pose;
        xb_pose = latest_xb_pose;
        fix_status = latest_gps_fix_status;
        ll_pose_available = latest_ll_gps_pose_available;
        xb_pose_available = latest_xb_pose_available;
        fix_status_available = latest_gps_fix_status_available;
        ll_received_at = latest_ll_gps_pose_received_at;
        xb_received_at = latest_xb_pose_received_at;
        fix_status_received_at = latest_gps_fix_status_received_at;
        drive_ready_at = latest_gps_drive_ready_at;
    }

    constexpr double GPS_INPUT_FRESHNESS_S = 3.0;
    const double ll_pose_age_s = ll_pose_available && !ll_received_at.isZero()
        ? (now - ll_received_at).toSec() : -1.0;
    const double fix_status_age_s = fix_status_available && !fix_status_received_at.isZero()
        ? (now - fix_status_received_at).toSec() : -1.0;
    const bool ll_pose_current = ll_pose_age_s >= 0.0 && ll_pose_age_s <= GPS_INPUT_FRESHNESS_S;
    const bool fix_status_current = fix_status_age_s >= 0.0 && fix_status_age_s <= GPS_INPUT_FRESHNESS_S;

    std::string solution_state = "unknown";
    std::string fix_type = "unknown";
    std::string rtk_source = "unavailable";
    bool input_position_valid = false;
    bool input_accuracy_available = false;
    double input_accuracy_m = 0.0;
    ros::Time gps_input_received_at;

    if (fix_status_current) {
        solution_state = gps_state_solution_from_fix_status(fix_status);
        fix_type = fix_status.value("fix_type", std::string("unknown"));
        input_position_valid = fix_status.value("position_valid", false) &&
            fix_status.value("gnss_fix_ok", false) && !fix_status.value("invalid_llh", true);
        if (fix_status.contains("position_accuracy_m") && fix_status["position_accuracy_m"].is_number()) {
            input_accuracy_m = fix_status["position_accuracy_m"].get<double>();
            input_accuracy_available = std::isfinite(input_accuracy_m) && input_accuracy_m >= 0.0 && input_accuracy_m < 100000.0;
        }
        gps_input_received_at = fix_status_received_at;
        rtk_source = "/ll/position/gps/fix_status";
    } else if (ll_pose_current) {
        solution_state = gps_state_solution_from_pose(ll_pose.flags);
        fix_type = "3d_fix";
        input_position_valid = true;
        input_accuracy_m = static_cast<double>(ll_pose.position_accuracy);
        input_accuracy_available = std::isfinite(input_accuracy_m) && input_accuracy_m >= 0.0 && input_accuracy_m < 100000.0;
        gps_input_received_at = ll_received_at;
        rtk_source = "/ll/position/gps";
    }

    const bool gps_input_available = fix_status_current || ll_pose_current;
    const bool ll_rtk_fixed = gps_input_available && solution_state == "fixed";
    const bool ll_rtk_float = gps_input_available && solution_state == "float";
    const bool ll_accuracy_ok = input_accuracy_available && input_position_valid &&
        solution_state != "no_fix" && solution_state != "unknown" &&
        input_accuracy_m <= max_gps_accuracy;
    const std::string solution_display = gps_state_solution_display(solution_state, fix_type);

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
                                       xb_accuracy_ok, gps_input_available, solution_state);
    const std::string drive_state = gps_drive_ready ? "ready" : (gps_timeout_estimated ? "stop" : "blocked");

    // State 1 remains the compact operator-facing drive status. It intentionally
    // does not include the State 0 decision chain or the State 2/3 GNSS statistics.
    json summary = json::object();
    summary["gps_drive_ready"] = gps_drive_ready;
    summary["gps_drive_state"] = drive_state;
    summary["gps_drive_label"] = gps_drive_ready ? "GPS reicht zum Fahren aus" : "GPS reicht nicht zum Fahren aus";
    summary["gps_drive_reason"] = gps_state_drive_reason_text(block_reason, solution_state);
    summary["gps_drive_block_reason"] = gps_drive_ready ? json(nullptr) : json(block_reason);
    summary["rtk_state"] = solution_state;
    summary["position_accuracy_m"] = xb_pose_available ? json(static_cast<double>(xb_pose.position_accuracy)) : json(nullptr);
    summary["max_position_accuracy_m"] = max_position_accuracy;
    summary["pose_age_ms"] = gps_state_age_ms_json(now, xb_received_at);

    // State 2 gets the technical details. The existing MQTT fields are reused;
    // no additional MQTT keys are introduced for the receiver fix-status transport.
    json details = json::object();
    details["mower_logic_gps_timeout_s"] = gps_timeout;
    details["mower_logic_gps_grace_remaining_s"] = grace_remaining_s;
    details["last_drive_ready_age_ms"] = gps_state_age_ms_json(now, drive_ready_at);
    details["xbot_positioning_max_gps_accuracy_m"] = max_gps_accuracy;
    details["ll_gps_available"] = gps_input_available;
    details["ll_gps_flags"] = ll_pose_current ? json(ll_pose.flags) : json(nullptr);
    details["ll_gps_rtk_fixed"] = gps_input_available ? json(ll_rtk_fixed) : json(nullptr);
    details["ll_gps_rtk_float"] = gps_input_available ? json(ll_rtk_float) : json(nullptr);
    details["ll_gps_position_accuracy_m"] = input_accuracy_available ? json(input_accuracy_m) : json(nullptr);
    details["ll_gps_accuracy_ok_for_positioning"] = input_accuracy_available ? json(ll_accuracy_ok) : json(nullptr);
    details["ll_gps_age_ms"] = gps_state_age_ms_json(now, gps_input_received_at);
    details["xb_pose_available"] = xb_pose_available;
    details["xb_pose_flags"] = xb_pose_available ? json(xb_pose.flags) : json(nullptr);
    details["xb_pose_source"] = xb_pose_available ? json(xb_pose.source) : json(nullptr);
    details["xb_pose_accuracy_ok_for_mower_logic"] = xb_pose_available ? json(xb_accuracy_ok) : json(nullptr);
    details["xb_pose_age_ms"] = gps_state_age_ms_json(now, xb_received_at);
    details["decision_source"] = "xbot_positioning/xb_pose";
    details["rtk_source"] = rtk_source;

    const json state0_status = build_gps_state0_status_payload(
        now, max_position_accuracy, max_gps_accuracy, gps_timeout,
        gps_input_received_at,
        xb_pose, xb_pose_available, xb_received_at, drive_ready_at,
        solution_state, solution_display, gps_input_available, input_position_valid,
        input_accuracy_available, input_accuracy_m, ll_rtk_fixed, ll_accuracy_ok,
        orientation_valid, recent_absolute_pose, xb_accuracy_ok, gps_drive_ready,
        grace_remaining_s, gps_timeout_estimated);

    return {
        {"summary", summary},
        {"details", details},
        {"state0_status", state0_status},
        {"rtk_state", solution_state},
        {"ll_gps_accuracy_m", input_accuracy_available ? json(input_accuracy_m) : json(nullptr)},
        {"xb_pose_accuracy_m", xb_pose_available ? json(static_cast<double>(xb_pose.position_accuracy)) : json(nullptr)},
        {"orientation_valid", xb_pose_available ? json(static_cast<bool>(xb_pose.orientation_valid)) : json(nullptr)},
        {"recent_absolute_pose", xb_pose_available ? json(recent_absolute_pose) : json(nullptr)},
        {"gps_timeout", gps_timeout_estimated},
        {"diagnostic_summary", gps_state_drive_reason_text(block_reason, solution_state)}
    };
}

static void apply_gps_drive_status_to_payloads(json &payloads, const json &drive_status) {
    if (payloads.contains("state1") && payloads["state1"].is_object()) {
        for (auto it = drive_status["summary"].begin(); it != drive_status["summary"].end(); ++it) {
            payloads["state1"][it.key()] = it.value();
        }
    }
    if (payloads.contains("state2") && payloads["state2"].is_object()) {
        payloads["state2"]["rtk_state"] = drive_status["rtk_state"];
        payloads["state2"]["ll_gps_accuracy_m"] = drive_status["ll_gps_accuracy_m"];
        payloads["state2"]["xb_pose_accuracy_m"] = drive_status["xb_pose_accuracy_m"];
        payloads["state2"]["orientation_valid"] = drive_status["orientation_valid"];
        payloads["state2"]["recent_absolute_pose"] = drive_status["recent_absolute_pose"];
        payloads["state2"]["gps_timeout"] = drive_status["gps_timeout"];
        payloads["state2"]["diagnostic_summary"] = drive_status["diagnostic_summary"];
        payloads["state2"]["drive_diagnostics"] = drive_status["details"];
    }
    if (payloads.contains("state0_status") && drive_status.contains("state0_status")) {
        payloads["state0_status"] = drive_status["state0_status"];
    }
}

static std::string gps_state_quality(bool available, int used_count, double avg_cn0) {
    if (!available || used_count <= 0) return "unavailable";
    if (used_count < 6 || avg_cn0 < 20.0) return "poor";
    if (used_count < 10 || avg_cn0 < 30.0) return "fair";
    if (used_count < 16 || avg_cn0 < 38.0) return "good";
    return "very_good";
}

static json gps_state_satellite_json(const xbot_msgs::GnssSatellite &sat, bool include_used) {
    json entry = json::object();
    entry["gnss"] = sat.gnss;
    entry["gnss_id"] = sat.gnss_id;
    entry["sv"] = sat.sv_id;
    if (include_used) entry["used"] = sat.used;
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
            used_satellites.push_back(gps_state_satellite_json(sat, false));
        }
        all_satellites.push_back(gps_state_satellite_json(sat, true));
    }

    const bool available = !satellites.empty();
    const double avg_cn0 = used_count > 0 ? cno_sum / static_cast<double>(used_count) : 0.0;
    if (used_count == 0) min_cn0 = 0.0;
    const ros::Time now = ros::Time::now();
    const double stamp = msg->header.stamp.toSec();
    const std::string quality = gps_state_quality(available, used_count, avg_cn0);
    const json drive_status = build_gps_drive_status_payload(now);

    json state1 = json::object();
    state1["schema"] = GPS_STATE_PAYLOAD_SCHEMA;
    state1["state"] = "state1";
    state1["updated_at"] = stamp;
    state1["quality_class"] = quality;

    json state2 = json::object();
    state2["schema"] = GPS_STATE_PAYLOAD_SCHEMA;
    state2["state"] = "state2";
    state2["updated_at"] = stamp;
    state2["available"] = available;
    state2["quality_class"] = quality;
    state2["visible_count"] = visible_count;
    state2["used_count"] = used_count;
    state2["avg_cn0"] = avg_cn0;
    state2["min_cn0"] = min_cn0;
    state2["max_cn0"] = max_cn0;
    state2["weak_count"] = weak_count;
    state2["good_count"] = good_count;
    state2["systems"] = systems;
    state2["sensor_stamp"] = msg->sensor_stamp;

    json state3 = json::object();
    state3["schema"] = GPS_STATE_PAYLOAD_SCHEMA;
    state3["state"] = "state3";
    state3["updated_at"] = stamp;
    state3["available"] = available;
    state3["used_count"] = used_count;
    state3["satellites"] = used_satellites;

    json state4 = json::object();
    state4["schema"] = GPS_STATE_PAYLOAD_SCHEMA;
    state4["state"] = "state4";
    state4["updated_at"] = stamp;
    state4["available"] = available;
    state4["quality_class"] = quality;
    state4["visible_count"] = visible_count;
    state4["used_count"] = used_count;
    state4["avg_cn0"] = avg_cn0;
    state4["sensor_stamp"] = msg->sensor_stamp;
    state4["min_cn0"] = min_cn0;
    state4["max_cn0"] = max_cn0;
    state4["satellites"] = all_satellites;

    json payloads = {
        {"state0_status", drive_status["state0_status"]},
        {"state1", state1},
        {"state2", state2},
        {"state3", state3},
        {"state4", state4}
    };
    apply_gps_drive_status_to_payloads(payloads, drive_status);
    return payloads;
}

static std::string gps_state_definition_topic(int state) {
    return "gps_state/state" + std::to_string(state) + "/definition";
}

static std::string gps_state_status_topic(int state) {
    return "gps_state/state" + std::to_string(state) + "/status";
}

static bool gps_state_status_retained(int state) {
    return state >= 0 && state <= 2;
}

static bool gps_state_enabled_for_regular_publish(const GpsStateSettings &cfg, int state) {
    switch (state) {
        case 0: return cfg.publish_state0;
        case 1: return cfg.publish_state1;
        case 2: return cfg.publish_state2;
        case 3: return cfg.publish_state3;
        case 4: return cfg.publish_state4;
        default: return false;
    }
}

static json gps_state_standard_status_payload(int state, const json &source, const ros::Time &now) {
    json root = source.is_object() ? source : json::object();
    const double published_at = now.toSec();
    double source_at = published_at;
    if (root.contains("source_at") && root["source_at"].is_number()) {
        source_at = root["source_at"].get<double>();
    } else if (root.contains("updated_at") && root["updated_at"].is_number()) {
        source_at = root["updated_at"].get<double>();
    } else if (root.contains("timestamp") && root["timestamp"].is_number()) {
        source_at = root["timestamp"].get<double>();
    }
    if (!std::isfinite(source_at) || source_at <= 0.0) source_at = published_at;

    bool available = root.value("available", true);
    std::string status = "ok";
    int severity = 0;
    std::string summary = "Daten aktuell";

    if (state == 0) {
        const std::string drive_state = root.value("drive_state", std::string("unknown"));
        available = root.value("positioning_debug_available", false) ||
                    (root.contains("ll_gps_age_ms") && !root["ll_gps_age_ms"].is_null()) ||
                    (root.contains("xb_pose_age_ms") && !root["xb_pose_age_ms"].is_null());
        status = drive_state == "ready" ? "ok" : drive_state;
        severity = root.value("severity", drive_state == "stop" ? 4 : (drive_state == "blocked" ? 3 : 1));
        summary = root.value("summary", std::string("GPS-Fahrdiagnose"));
    } else if (state == 1) {
        const std::string drive_state = root.value("gps_drive_state", std::string("unknown"));
        const std::string rtk_state = root.value("rtk_state", std::string("unknown"));
        available = rtk_state != "unknown" ||
                    (root.contains("position_accuracy_m") && !root["position_accuracy_m"].is_null());
        source_at = published_at;
        status = drive_state == "ready" ? "ok" : drive_state;
        severity = drive_state == "ready" ? 0 : (drive_state == "stop" ? 4 : (drive_state == "blocked" ? 3 : 1));
        summary = root.value("gps_drive_label", std::string("GPS-Fahrstatus unbekannt"));
        const std::string reason = root.value("gps_drive_reason", std::string());
        if (!reason.empty()) summary += ": " + reason;
    } else if (state == 2) {
        available = root.value("available", false) || root.contains("drive_diagnostics");
        const bool timeout = root.value("gps_timeout", false);
        status = timeout ? "stop" : (available ? "ok" : "unavailable");
        severity = timeout ? 4 : (available ? 0 : 1);
        summary = root.value("diagnostic_summary", available ? std::string("GNSS-/Pose-Diagnose verfuegbar") : std::string("Keine GNSS-/Pose-Daten"));
    } else if (state == 3) {
        const int used_count = root.value("used_count", 0);
        available = root.value("available", root.contains("satellites"));
        status = !available ? "unavailable" : (used_count > 0 ? "ok" : "warning");
        severity = !available ? 1 : (used_count > 0 ? 0 : 2);
        summary = used_count > 0 ? std::to_string(used_count) + " Satelliten werden verwendet" : "Keine verwendeten Satelliten";
    } else if (state == 4) {
        const int visible_count = root.value("visible_count", 0);
        available = root.value("available", root.contains("satellites"));
        status = !available ? "unavailable" : (visible_count > 0 ? "ok" : "warning");
        severity = !available ? 1 : (visible_count > 0 ? 0 : 2);
        summary = visible_count > 0 ? std::to_string(visible_count) + " Satelliten sichtbar" : "Keine sichtbaren Satelliten";
    }

    const long long age_ms = static_cast<long long>(std::max(0.0, (published_at - source_at) * 1000.0));

    json data = root;
    for (const char *key : {"schema", "state", "type", "definition_version", "published_at", "source_at", "age_ms", "stale", "status", "severity", "summary", "data"}) {
        data.erase(key);
    }

    root["schema"] = GPS_STATE_PAYLOAD_SCHEMA;
    root["state"] = "state" + std::to_string(state);
    root["type"] = "status";
    root["definition_version"] = GPS_STATE_DEFINITION_VERSION;
    root["published_at"] = published_at;
    root["source_at"] = source_at;
    root["age_ms"] = age_ms;
    const bool stale = age_ms > static_cast<long long>(GPS_STATE_STALE_AFTER_S * 1000.0);
    if (stale && state >= 2 && state <= 4) {
        available = false;
        status = "stale";
        severity = std::max(severity, 1);
        summary = "Keine aktuellen Satellitendaten seit " + std::to_string(age_ms / 1000) + " s";
    }
    root["available"] = available;
    root["stale"] = stale;
    root["status"] = status;
    root["severity"] = severity;
    root["summary"] = summary;
    root["data"] = data;
    return root;
}

static void publish_gps_state_definition(int state) {
    const json definition = build_gps_state_definition_payload(state);
    if (!definition.empty()) try_publish(gps_state_definition_topic(state), definition.dump(), true);
}

void publish_gps_state_definitions() {
    for (int state = 0; state <= 4; ++state) publish_gps_state_definition(state);
}

static void publish_gps_state_status(int state, const json &raw_status) {
    const json status = gps_state_standard_status_payload(state, raw_status, ros::Time::now());
    const bool retain = gps_state_status_retained(state);
    try_publish(gps_state_status_topic(state), status.dump(), retain);

    // Transitional aliases keep the currently bundled web application and older
    // MQTT consumers operational while stateN/status becomes the canonical API.
    if (state >= 1 && state <= 4) {
        try_publish("gps_state/state" + std::to_string(state), status.dump(), retain);
    }
}

static json build_gps_state_fallback_payloads(const ros::Time &now) {
    const json drive_status = build_gps_drive_status_payload(now);
    json payloads = {
        {"state0_status", drive_status["state0_status"]},
        {"state1", {{"updated_at", now.toSec()}, {"quality_class", "unavailable"}}},
        {"state2", {{"updated_at", now.toSec()}, {"available", false}, {"quality_class", "unavailable"},
                    {"visible_count", 0}, {"used_count", 0}, {"avg_cn0", 0.0}, {"min_cn0", 0.0},
                    {"max_cn0", 0.0}, {"weak_count", 0}, {"good_count", 0}, {"systems", json::object()}}},
        {"state3", {{"updated_at", now.toSec()}, {"available", false}, {"used_count", 0}, {"satellites", json::array()}}},
        {"state4", {{"updated_at", now.toSec()}, {"available", false}, {"quality_class", "unavailable"},
                    {"visible_count", 0}, {"used_count", 0}, {"avg_cn0", 0.0}, {"min_cn0", 0.0},
                    {"max_cn0", 0.0}, {"satellites", json::array()}}}
    };
    apply_gps_drive_status_to_payloads(payloads, drive_status);
    return payloads;
}

static json current_gps_state_payload_snapshot(const ros::Time &now) {
    json snapshot;
    bool has_snapshot = false;
    {
        std::lock_guard<std::mutex> lk(gps_state_payload_mutex);
        has_snapshot = latest_gps_state_available;
        snapshot = latest_gps_state_payloads;
    }
    if (!has_snapshot || !snapshot.is_object()) snapshot = build_gps_state_fallback_payloads(now);
    const json drive_status = build_gps_drive_status_payload(now);
    apply_gps_drive_status_to_payloads(snapshot, drive_status);
    return snapshot;
}

static json gps_state_raw_status_from_snapshot(const json &snapshot, int state) {
    if (state == 0) return snapshot.value("state0_status", json::object());
    return snapshot.value("state" + std::to_string(state), json::object());
}

static std::set<int> gps_state_regular_states(const GpsStateSettings &cfg) {
    std::set<int> states;
    if (!cfg.enabled) return states;
    for (int state = 0; state <= 4; ++state) {
        if (gps_state_enabled_for_regular_publish(cfg, state)) states.insert(state);
    }
    return states;
}

static bool parse_gps_state_number(const json &value, int &state) {
    if (value.is_number_integer()) {
        state = value.get<int>();
        return state >= 0 && state <= 4;
    }
    if (!value.is_string()) return false;
    std::string text = trim_settings_string(value.get<std::string>());
    std::transform(text.begin(), text.end(), text.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (text.rfind("state", 0) == 0) text = text.substr(5);
    if (text.size() != 1 || text[0] < '0' || text[0] > '4') return false;
    state = text[0] - '0';
    return true;
}

void handle_gps_state_renew_payload(const std::string &payload_text) {
    const GpsStateSettings cfg = current_gps_state_settings();
    std::set<int> states = gps_state_regular_states(cfg);
    bool definitions = true;
    bool statuses = true;

    const std::string trimmed = trim_settings_string(payload_text);
    if (!trimmed.empty()) {
        try {
            const json request = json::parse(trimmed);
            if (!request.is_object()) throw std::runtime_error("payload must be a JSON object");

            if (request.contains("states")) {
                if (!request["states"].is_array()) throw std::runtime_error("states must be an array");
                states.clear();
                for (const auto &entry : request["states"]) {
                    if (entry.is_string()) {
                        std::string text = trim_settings_string(entry.get<std::string>());
                        std::transform(text.begin(), text.end(), text.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
                        if (text == "all") {
                            states = {0, 1, 2, 3, 4};
                            continue;
                        }
                    }
                    int state = -1;
                    if (!parse_gps_state_number(entry, state)) throw std::runtime_error("states contains an invalid value");
                    states.insert(state);
                }
            }

            if (request.contains("parts")) {
                if (!request["parts"].is_array()) throw std::runtime_error("parts must be an array");
                definitions = false;
                statuses = false;
                for (const auto &entry : request["parts"]) {
                    if (!entry.is_string()) throw std::runtime_error("parts contains a non-string value");
                    const std::string part = trim_settings_string(entry.get<std::string>());
                    if (part == "definition") definitions = true;
                    else if (part == "status") statuses = true;
                    else throw std::runtime_error("parts supports only definition and status");
                }
                if (!definitions && !statuses) throw std::runtime_error("parts must not be empty");
            }
        } catch (const std::exception &e) {
            ROS_WARN_STREAM("Rejected GPS-State renew request: " << e.what());
            publish_gps_state_validation({
                {"valid", false},
                {"namespace", GPS_STATE_NAMESPACE},
                {"mode", "renew"},
                {"reason", e.what()}
            });
            return;
        }
    }

    publish_gps_state_settings();
    if (definitions) {
        for (const int state : states) publish_gps_state_definition(state);
    }
    if (statuses) {
        const json snapshot = current_gps_state_payload_snapshot(ros::Time::now());
        for (const int state : states) publish_gps_state_status(state, gps_state_raw_status_from_snapshot(snapshot, state));
    }
    json published_parts = json::array();
    if (definitions) published_parts.push_back("definition");
    if (statuses) published_parts.push_back("status");
    publish_gps_state_validation({
        {"valid", true},
        {"namespace", GPS_STATE_NAMESPACE},
        {"mode", "renew"},
        {"states", states},
        {"parts", published_parts}
    });
}

void publish_gps_state0_snapshot() {
    publish_gps_state_definition(0);
    const json drive_status = build_gps_drive_status_payload(ros::Time::now());
    if (drive_status.contains("state0_status") && drive_status["state0_status"].is_object()) {
        publish_gps_state_status(0, drive_status["state0_status"]);
    }
}

void publish_latest_gps_state_payloads(bool force) {
    const GpsStateSettings cfg = current_gps_state_settings();
    if (!cfg.enabled && !force) return;

    const json snapshot = current_gps_state_payload_snapshot(ros::Time::now());
    for (int state = 0; state <= 4; ++state) {
        if (force || gps_state_enabled_for_regular_publish(cfg, state)) {
            publish_gps_state_status(state, gps_state_raw_status_from_snapshot(snapshot, state));
        }
    }
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

void gps_state_positioning_debug_callback(const std_msgs::String::ConstPtr &msg) {
    json parsed = json::parse(msg->data, nullptr, false);
    if (!parsed.is_object()) {
        parsed = {{"raw", msg->data}, {"parse_error", true}};
    }
    std::lock_guard<std::mutex> lk(gps_state_pose_mutex);
    latest_xbot_positioning_gps_debug = parsed;
    latest_xbot_positioning_gps_debug_received_at = ros::Time::now();
    latest_xbot_positioning_gps_debug_available = true;
}

void gps_state_ll_gps_pose_callback(const xbot_msgs::AbsolutePose::ConstPtr &msg) {
    std::lock_guard<std::mutex> lk(gps_state_pose_mutex);
    latest_ll_gps_pose = *msg;
    latest_ll_gps_pose_received_at = ros::Time::now();
    latest_ll_gps_pose_available = true;
}

void gps_state_fix_status_callback(const std_msgs::String::ConstPtr &msg) {
    json parsed = json::parse(msg->data, nullptr, false);
    if (!parsed.is_object()) {
        ROS_WARN_THROTTLE(10.0, "Ignoring malformed GPS fix-status JSON from xbot_driver_gps");
        return;
    }

    {
        std::lock_guard<std::mutex> lk(gps_state_pose_mutex);
        latest_gps_fix_status = parsed;
        latest_gps_fix_status_received_at = ros::Time::now();
        latest_gps_fix_status_available = true;
    }

    // Update retained MQTT state promptly when the receiver changes between No Fix,
    // GPS Fix, RTK Float and RTK Fixed. The existing payload structure is retained.
    publish_latest_gps_state_payloads(false);
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
    try {
        json payload = json::parse(msg->data);
        {
            std::lock_guard<std::mutex> lk(mower_logic_settings_cache_mutex);
            latest_mower_logic_settings_payload = payload;
            latest_mower_logic_settings_available = true;
        }

        json filtered = payload;
        if (filtered.is_object() && filtered.contains("settings") && filtered["settings"].is_object()) {
            std::vector<std::string> keys_to_remove;
            for (auto it = filtered["settings"].begin(); it != filtered["settings"].end(); ++it) {
                if (is_gps_logging_internal_setting(it.key())) keys_to_remove.push_back(it.key());
            }
            for (const auto &key : keys_to_remove) filtered["settings"].erase(key);
        }
        try_publish("settings/mower_logic/json", filtered.dump(), true);
        publish_gps_state_settings();

        json pending;
        bool pending_persistent = false;
        {
            std::lock_guard<std::mutex> lk(gps_logging_pending_mutex);
            pending = pending_gps_logging_settings;
            pending_persistent = pending_gps_logging_settings_persistent;
        }
        if (pending.is_object() && !pending.empty()) {
            bool all_match = true;
            json accepted = json::object();
            for (auto it = pending.begin(); it != pending.end(); ++it) {
                const auto mapping = gps_logging_public_to_internal_settings().find(it.key());
                if (mapping == gps_logging_public_to_internal_settings().end()) continue;
                const json internal = gps_logging_cached_internal_entry(mapping->second);
                const char *field = pending_persistent ? "persistent" : "active";
                if (!internal.is_object() || !internal.contains(field) || internal[field] != it.value()) {
                    all_match = false;
                    break;
                }
                accepted[it.key()] = json::array({"value"});
            }
            if (all_match) {
                {
                    std::lock_guard<std::mutex> lk(gps_logging_pending_mutex);
                    pending_gps_logging_settings = json::object();
                }
                publish_gps_state_validation({
                    {"valid", true},
                    {"namespace", GPS_STATE_NAMESPACE},
                    {"mode", pending_persistent ? "persistent" : "session"},
                    {"status", "applied"},
                    {"pending", false},
                    {"accepted", accepted},
                    {"rejected", json::object()},
                    {"remarks", json::array({"GPS logging settings were applied by mower_logic."})}
                });
            }
        }
    } catch (const json::exception &e) {
        ROS_WARN_STREAM("Could not transform mower_logic settings payload: " << e.what());
        try_publish("settings/mower_logic/json", msg->data, true);
    }
}

void mower_logic_settings_validation_json_callback(const std_msgs::String::ConstPtr &msg) {
    try_publish("settings/mower_logic/validation/json", msg->data, true);
    try {
        const json source = json::parse(msg->data);
        json accepted = json::object();
        json rejected = json::object();
        if (source.contains("accepted") && source["accepted"].is_array()) {
            for (const auto &item : source["accepted"]) {
                if (!item.is_object() || !item.contains("key") || !item["key"].is_string()) continue;
                const std::string public_key = gps_logging_public_key_for_internal(item["key"].get<std::string>());
                if (!public_key.empty()) accepted[public_key] = item.value("fields", json::array({"value"}));
            }
        }
        if (source.contains("rejected") && source["rejected"].is_array()) {
            for (const auto &item : source["rejected"]) {
                if (!item.is_object() || !item.contains("key") || !item["key"].is_string()) continue;
                const std::string public_key = gps_logging_public_key_for_internal(item["key"].get<std::string>());
                if (!public_key.empty()) rejected[public_key] = item.value("reason", std::string("mower_logic rejected the setting"));
            }
        }
        if (!accepted.empty() || !rejected.empty()) {
            if (!rejected.empty()) {
                std::lock_guard<std::mutex> lk(gps_logging_pending_mutex);
                pending_gps_logging_settings = json::object();
            }
            publish_gps_state_validation({
                {"valid", !accepted.empty() && rejected.empty()},
                {"namespace", GPS_STATE_NAMESPACE},
                {"mode", source.value("mode", std::string("unknown"))},
                {"status", rejected.empty() ? "accepted_by_mower_logic" : "rejected"},
                {"pending", rejected.empty()},
                {"accepted", accepted},
                {"rejected", rejected},
                {"remarks", json::array()}
            });
        }
    } catch (const json::exception &) {
    }
}

void mower_logic_satellite_logging_status_json_callback(const std_msgs::String::ConstPtr &msg) {
    try_publish("settings/mower_logic/satellite_logging/json", msg->data, true);
    try {
        const json source = json::parse(msg->data);
        const json transformed = build_gps_logging_status_payload(source);
        bool publish_last = false;
        json last;
        {
            std::lock_guard<std::mutex> lk(gps_logging_status_mutex);
            latest_gps_logging_status = transformed;
            latest_gps_logging_status_available = true;
            if (gps_logging_status_is_completed(transformed)) {
                last = build_gps_logging_last_payload(transformed);
                last_completed_gps_logging = last;
                last_completed_gps_logging_available = true;
                publish_last = true;
            }
        }
        try_publish("gps_state/logging/status/json", transformed.dump(), true);
        if (publish_last) try_publish("gps_state/logging/last/json", last.dump(), true);
    } catch (const json::exception &e) {
        ROS_WARN_STREAM("Could not transform satellite logging status: " << e.what());
    }
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

    // The restart command publisher is needed before MQTT subscriptions can receive commands.
    gps_restart_request_pub = n->advertise<std_msgs::String>("/ll/position/gps/restart_request", 10);

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
    ros::Subscriber gpsStateFixStatusSubscriber = n->subscribe("/ll/position/gps/fix_status", 1, gps_state_fix_status_callback);
    ros::Subscriber gpsStateXbPoseSubscriber = n->subscribe("/xbot_positioning/xb_pose", 1, gps_state_xb_pose_callback);
    ros::Subscriber gpsStatePositioningDebugSubscriber = n->subscribe("/xbot_positioning/gps_debug_state", 1, gps_state_positioning_debug_callback);
    ros::Subscriber gpsStateSatellitesSubscriber = n->subscribe("/ll/position/gps/satellites", 1, gps_state_satellites_callback);
    ros::Subscriber gpsRestartStatusSubscriber = n->subscribe("/ll/position/gps/restart_status", 10, gps_restart_status_callback);

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
