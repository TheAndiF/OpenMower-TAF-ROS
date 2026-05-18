//
// Created by Clemens Elflein on 22.11.22.
// Copyright (c) 2022 Clemens Elflein. All rights reserved.
//
#include <filesystem>
#include <algorithm>
#include <cstdio>
#include <fstream>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <cstdlib>
#include <optional>
#include <regex>
#include <unordered_map>

#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/BoolParameter.h>
#include <dynamic_reconfigure/IntParameter.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/StrParameter.h>

#include "ros/ros.h"
#include <memory>
#include <boost/regex.hpp>
#include "xbot_msgs/SensorInfo.h"
#include "xbot_msgs/SensorDataString.h"
#include "xbot_msgs/SensorDataDouble.h"
#include "xbot_msgs/RobotState.h"
#include <mqtt/async_client.h>
#include <nlohmann/json.hpp>
#include <vector>
#include <set>
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Float32.h"
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

using json = nlohmann::ordered_json;

void publish_capabilities();
void publish_sensor_metadata();
void publish_map();
void publish_map_validation(const json &validation);
json validate_map_payload_for_mqtt(const json &payload);
void publish_map_overlay();
void publish_timetable();
void maybe_publish_timetable(bool force = false);
void publish_timetable_validation(const json &validation);
void publish_statustransition_log(std::size_t requested_limit = 0);
void publish_actions();
void publish_version();
void publish_params();
void publish_mower_logic_settings();
void publish_mower_logic_settings_validation(const json &validation);
json handle_mower_logic_session_settings(const json &payload);
json handle_mower_logic_persistent_settings(const json &payload);
void rpc_request_callback(const std::string &payload);

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
ros::Publisher mow_load_factor_renew_pub;
ros::ServiceClient mower_logic_reconfigure_client;

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
        publish_mower_logic_settings();
        publish_actions();
        publish_version();
        publish_params();

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
        client_->subscribe(this->mqtt_topic_prefix + "statustransition_log/set/renew/json", 0);
        client_->subscribe(this->mqtt_topic_prefix + "settings/mower_logic/set/session/json", 0);
        client_->subscribe(this->mqtt_topic_prefix + "settings/mower_logic/set/persistent/json", 0);
        client_->subscribe(this->mqtt_topic_prefix + "settings/mower_logic/set/renew/json", 0);
        client_->subscribe(this->mqtt_topic_prefix + "mow_load_factor/set/json", 0);
        client_->subscribe(this->mqtt_topic_prefix + "mow_load_factor/set/renew/json", 0);
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
        } else if (ptr->get_topic() == this->mqtt_topic_prefix + "settings/mower_logic/set/renew/json") {
            // App requests the current retained settings again.
            publish_mower_logic_settings();
        } else if (ptr->get_topic() == this->mqtt_topic_prefix + "settings/mower_logic/set/session/json") {
            try {
                json payload = json::parse(ptr->get_payload_str());
                json validation = handle_mower_logic_session_settings(payload);
                publish_mower_logic_settings_validation(validation);
                publish_mower_logic_settings();
            } catch (const json::exception &e) {
                publish_mower_logic_settings_validation({{"valid", false}, {"scope", "session"}, {"remarks", {std::string("Error decoding mower_logic session JSON: ") + e.what()}}});
            }
        } else if (ptr->get_topic() == this->mqtt_topic_prefix + "settings/mower_logic/set/persistent/json") {
            try {
                json payload = json::parse(ptr->get_payload_str());
                json validation = handle_mower_logic_persistent_settings(payload);
                publish_mower_logic_settings_validation(validation);
                publish_mower_logic_settings();
            } catch (const json::exception &e) {
                publish_mower_logic_settings_validation({{"valid", false}, {"scope", "persistent"}, {"remarks", {std::string("Error decoding mower_logic persistent JSON: ") + e.what()}}});
            }
        } else if (ptr->get_topic() == this->mqtt_topic_prefix + "mow_load_factor/set/json") {
            try {
                json payload = json::parse(ptr->get_payload_str());
                if (!payload.is_object()) {
                    ROS_WARN_STREAM("Ignoring mow_load_factor/set/json payload because it is not a JSON object.");
                } else {
                    bool handled = false;
                    if (payload.contains("enabled") && payload["enabled"].is_boolean()) {
                        std_msgs::Bool msg;
                        msg.data = payload["enabled"].get<bool>();
                        mow_load_factor_set_enabled_pub.publish(msg);
                        handled = true;
                    }
                    if (payload.contains("min_factor") && payload["min_factor"].is_number()) {
                        std_msgs::Float32 msg;
                        msg.data = payload["min_factor"].get<float>();
                        mow_load_factor_set_min_factor_pub.publish(msg);
                        handled = true;
                    }
                    if (payload.contains("current_start") && payload["current_start"].is_number()) {
                        std_msgs::Float32 msg;
                        msg.data = payload["current_start"].get<float>();
                        mow_load_factor_set_current_start_pub.publish(msg);
                        handled = true;
                    }
                    if (payload.contains("current_end") && payload["current_end"].is_number()) {
                        std_msgs::Float32 msg;
                        msg.data = payload["current_end"].get<float>();
                        mow_load_factor_set_current_end_pub.publish(msg);
                        handled = true;
                    }
                    if (!handled) {
                        ROS_WARN_STREAM("Ignoring mow_load_factor/set/json payload without boolean 'enabled' or numeric 'min_factor', 'current_start', or 'current_end'.");
                    }
                }
            } catch (const json::exception &e) {
                ROS_WARN_STREAM("Error decoding mow_load_factor set JSON: " << e.what());
            }
        } else if (ptr->get_topic() == this->mqtt_topic_prefix + "mow_load_factor/set/renew/json") {
            std_msgs::Empty msg;
            mow_load_factor_renew_pub.publish(msg);
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

void try_publish(std::string topic, std::string data, bool retain = false) {
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

// BEGIN: mower_logic settings MQTT bridge
namespace mower_logic_settings {

enum class SettingType { Bool, Int, Double, String };

struct SettingDefinition {
    const char *key;
    SettingType type;
    double min_value;
    double max_value;
    bool has_numeric_range;
    const char *group;
    const char *label;
    const char *unit;
    const char *description;
    int order;
    bool session_apply_supported;
};

// Registry is intentionally complete for the currently editable mower_logic dynamic_reconfigure parameters.
// The WebApp may hide or group entries, but the backend exposes one stable canonical source.
const std::vector<SettingDefinition> &registry() {
    static const std::vector<SettingDefinition> definitions = {
        {"automatic_mode", SettingType::Int, 0, 2, true, "general", "Automatikmodus", "", "0 = manuell, 1 = halbautomatisch, 2 = automatisch", 10, true},
        {"enable_mower", SettingType::Bool, 0, 0, false, "general", "Mähmotor freigeben", "", "Aktiviert die automatische Freigabe des Mähmotors.", 20, true},
        {"manual_pause_mowing", SettingType::Bool, 0, 0, false, "general", "Mähen manuell pausieren", "", "Unterbindet das Mähen trotz aktiver Navigation.", 30, true},

        {"undock_distance", SettingType::Double, 0, 100, true, "undocking", "Rückfahrstrecke Ausdocken", "m", "Erste gerade Rückfahrstrecke beim Ausdocken.", 100, true},
        {"undock_angled_distance", SettingType::Double, 0, 100, true, "undocking", "Schrägstrecke Ausdocken", "m", "Zweite Ausdockstrecke im Winkel.", 110, true},
        {"undock_angle", SettingType::Double, -90, 90, true, "undocking", "Ausklinkwinkel", "°", "Winkel der zweiten Ausdockphase.", 120, true},
        {"undock_fixed_angle", SettingType::Bool, 0, 0, false, "undocking", "Fester Ausdockwinkel", "", "Verwendet einen festen statt variierenden Ausdockwinkel.", 130, true},
        {"undock_use_curve", SettingType::Bool, 0, 0, false, "undocking", "Kurvenausdockung", "", "Fährt die zweite Ausdockphase als Kurve.", 140, true},
        {"undocking_waiting_time", SettingType::Double, 0, 60, true, "undocking", "Wartezeit vor Ausdocken", "s", "Wartezeit vor Start des Ausdockens.", 150, true},

        {"docking_distance", SettingType::Double, 0, 100, true, "docking", "Docking-Fahrstrecke", "m", "Vorwärtsstrecke während des Dockings.", 200, true},
        {"docking_approach_distance", SettingType::Double, 0, 5, true, "docking", "Docking-Anfahrdistanz", "m", "Distanz zur Annäherung an den Docking-Punkt.", 210, true},
        {"docking_retry_count", SettingType::Int, 0, 50, true, "docking", "Docking-Wiederholungen", "", "Maximale Anzahl Docking-Wiederholungen.", 220, true},
        {"docking_extra_time", SettingType::Double, 0, 1, true, "docking", "Docking-Nachlaufzeit", "s", "Zusätzliche Kontaktzeit zur sicheren Ladekontakterkennung.", 230, true},
        {"docking_redock", SettingType::Bool, 0, 0, false, "docking", "Erneut andocken", "", "Versucht ein erneutes Andocken, wenn die Ladeerkennung wegfällt.", 240, true},
        {"docking_waiting_time", SettingType::Double, 0, 60, true, "docking", "Wartezeit vor Docking", "s", "Wartezeit vor Start des Dockings.", 250, true},
        {"perimeter_signal", SettingType::Int, -2, 2, true, "docking", "Perimetersignal", "", "0 = aus, Vorzeichen bestimmt die Docking-Richtung.", 260, true},

        {"outline_count", SettingType::Int, 0, 255, true, "mowing_strategy", "Anzahl Außenbahnen", "", "Anzahl Außenbahnen vor dem Füllen der Fläche.", 300, true},
        {"outline_overlap_count", SettingType::Int, 0, 255, true, "mowing_strategy", "Überlappende Außenbahnen", "", "Anzahl Außenbahnen, die mit dem Füllmuster überlappen.", 310, true},
        {"outline_offset", SettingType::Double, -1, 1, true, "mowing_strategy", "Außenbahn-Offset", "m", "Zusätzlicher Außenbahn-Offset; positiv ist konservativer.", 320, true},
        {"mow_angle_offset", SettingType::Double, -180, 180, true, "mowing_strategy", "Mähwinkel-Offset", "°", "Zusätzlicher Winkel für das Flächenmuster.", 330, true},
        {"mow_angle_offset_is_absolute", SettingType::Bool, 0, 0, false, "mowing_strategy", "Absoluter Mähwinkel", "", "Interpretiert den Offset absolut statt relativ zur automatisch ermittelten Richtung.", 340, true},
        {"mow_angle_increment", SettingType::Double, 0, 180, true, "mowing_strategy", "Mähwinkel-Inkrement", "°", "Winkeländerung nach einem vollständig abgearbeiteten Kartenzyklus.", 350, true},
        {"tool_width", SettingType::Double, 0.1, 2, true, "mowing_strategy", "Arbeitsbreite", "m", "Effektive Arbeitsbreite des Mähers.", 360, true},
        {"max_first_point_attempts", SettingType::Int, 1, 10, true, "mowing_strategy", "Versuche erster Pfadpunkt", "", "Versuche zum Erreichen des ersten Mähpfadpunkts.", 370, true},
        {"max_first_point_trim_attempts", SettingType::Int, 1, 10, true, "mowing_strategy", "Trim-Versuche Pfadbeginn", "", "Anzahl Kürzungen des Pfadbeginns nach fehlgeschlagenen Erstpunktversuchen.", 380, true},
        {"add_fake_obstacle", SettingType::Bool, 0, 0, false, "mowing_strategy", "Hilfshindernis ergänzen", "", "Fügt ein künstliches Hindernis zur Unterstützung der Pfadannäherung hinzu.", 390, true},

        {"motor_hot_temperature", SettingType::Double, 20, 150, true, "temperature_protection", "Mähmotor heiß", "°C", "Temperatur, ab der das Mähen pausiert.", 400, true},
        {"motor_cold_temperature", SettingType::Double, 20, 150, true, "temperature_protection", "Mähmotor wieder kalt", "°C", "Temperatur, ab der das Mähen wieder freigegeben wird.", 410, true},
        {"mow_load_motor_temp_start", SettingType::Double, 0, 150, true, "mowing_load_control", "Temperatur-Derating Start Motor", "°C", "Mähmotortemperatur, ab der der Lastfaktor reduziert wird.", 420, true},
        {"mow_load_motor_temp_end", SettingType::Double, 0, 150, true, "mowing_load_control", "Temperatur-Derating Ende Motor", "°C", "Mähmotortemperatur für den minimalen Lastfaktor.", 430, true},
        {"mow_load_esc_temp_start", SettingType::Double, 0, 150, true, "mowing_load_control", "Temperatur-Derating Start ESC", "°C", "Mäh-ESC-Temperatur, ab der der Lastfaktor reduziert wird.", 440, true},
        {"mow_load_esc_temp_end", SettingType::Double, 0, 150, true, "mowing_load_control", "Temperatur-Derating Ende ESC", "°C", "Mäh-ESC-Temperatur für den minimalen Lastfaktor.", 450, true},

        {"mow_load_factor_enabled", SettingType::Bool, 0, 0, false, "mowing_load_control", "Lastfaktor aktiv", "", "Aktiviert die berechnete Mäh-Lastfaktor-Regelung.", 500, true},
        {"mow_load_factor_min", SettingType::Double, 0.10, 1.00, true, "mowing_load_control", "Minimaler Lastfaktor", "", "Untergrenze des berechneten Mäh-Lastfaktors.", 510, true},
        {"mow_load_current_start", SettingType::Double, 0, 100, true, "mowing_load_control", "Strom-Derating Start", "A", "Mähmotorstrom, ab dem der Lastfaktor reduziert wird.", 520, true},
        {"mow_load_current_end", SettingType::Double, 0, 100, true, "mowing_load_control", "Strom-Derating Ende", "A", "Mähmotorstrom für den minimalen Lastfaktor.", 530, true},

        {"max_position_accuracy", SettingType::Double, 0.01, 1.0, true, "gps", "Maximale Positionsungenauigkeit", "m", "Fahrt ist nur erlaubt, solange die Positionsungenauigkeit darunter liegt.", 600, true},
        {"gps_wait_time", SettingType::Double, 0, 60, true, "gps", "GPS-Wartezeit", "s", "Wartezeit nach gutem GPS-Fix.", 610, true},
        {"gps_timeout", SettingType::Double, 0, 60, true, "gps", "GPS-Timeout", "s", "Erlaubte Fahrzeit ohne gültiges GPS.", 620, true},
        {"ignore_gps_errors", SettingType::Bool, 0, 0, false, "gps", "GPS-Fehler ignorieren", "", "Ignoriert GPS-Fehler. Nur für Simulation verwenden.", 630, true},

        {"rain_mode", SettingType::Int, 0, 3, true, "rain", "Regenmodus", "", "0 = ignorieren, 1 = andocken, 2 = bis trocken andocken, 3 = Automatik pausieren.", 700, true},
        {"rain_delay_minutes", SettingType::Int, 30, 1440, true, "rain", "Regen-Nachlaufzeit", "min", "Wartezeit nach Regen, bevor das Mähen wieder erlaubt ist.", 710, true},
        {"rain_check_seconds", SettingType::Int, 20, 300, true, "rain", "Regen-Erkennungszeit", "s", "Regen muss so lange kontinuierlich erkannt werden.", 720, true},
        {"cu_rain_threshold", SettingType::Int, -1, 2147483647, true, "rain", "CoverUI Regen-Schwellwert", "", "Schwellwert für den stock CoverUI-Regenwert; -1 nutzt die vorhandene Voreinstellung.", 730, true},

        {"emergency_lift_period", SettingType::Int, -1, 2147483647, true, "safety", "Notfallzeit Mehrfach-Lift", "ms", "Zeitfenster für mehrere Lift-/Hall-Signale, damit ein Notfall zählt.", 800, true},
        {"emergency_tilt_period", SettingType::Int, -1, 2147483647, true, "safety", "Notfallzeit Einzel-Tilt", "ms", "Zeitfenster für einzelnes Lift-/Hall-Signal, damit ein Notfall zählt.", 810, true},
        {"emergency_input_config", SettingType::String, 0, 0, false, "safety", "Notfall-Eingangskonfiguration", "", "Kommagetrennte Notfall-Eingangskonfiguration für Hall-/Stop-Sensoren.", 820, true},
        {"shutdown_esc_max_pitch", SettingType::Int, 0, 180, true, "safety", "Maximaler Pitch für ESC-Shutdown", "°", "ESC-Abschaltung ist nur bis zu diesem Pitch-Winkel erlaubt; 0 deaktiviert die Logik.", 830, true},
    };
    return definitions;
}

const SettingDefinition *find_definition(const std::string &key) {
    const auto &definitions = registry();
    const auto it = std::find_if(definitions.begin(), definitions.end(), [&](const SettingDefinition &entry) {
        return key == entry.key;
    });
    return it == definitions.end() ? nullptr : &(*it);
}

std::string trim_copy(std::string value) {
    const auto first = value.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) return "";
    const auto last = value.find_last_not_of(" \t\r\n");
    return value.substr(first, last - first + 1);
}

std::string unquote_yaml_scalar(const std::string &value) {
    if (value.size() >= 2 && ((value.front() == '"' && value.back() == '"') || (value.front() == '\'' && value.back() == '\''))) {
        return value.substr(1, value.size() - 2);
    }
    return value;
}

json parse_yaml_scalar(const std::string &raw_value) {
    std::string value = trim_copy(raw_value);
    const auto comment_pos = value.find(" #");
    if (comment_pos != std::string::npos) value = trim_copy(value.substr(0, comment_pos));
    value = unquote_yaml_scalar(value);
    if (value == "true" || value == "True") return true;
    if (value == "false" || value == "False") return false;
    static const std::regex integer_re(R"(^[-+]?[0-9]+$)");
    static const std::regex double_re(R"(^[-+]?(?:[0-9]+\.[0-9]*|[0-9]*\.[0-9]+)(?:[eE][-+]?[0-9]+)?$|^[-+]?[0-9]+(?:[eE][-+]?[0-9]+)$)");
    try {
        if (std::regex_match(value, integer_re)) return std::stoll(value);
        if (std::regex_match(value, double_re)) return std::stod(value);
    } catch (const std::exception &) {
        // Keep as string if numeric parsing fails.
    }
    return value;
}

std::string format_yaml_scalar(const json &value, SettingType type) {
    std::ostringstream oss;
    switch (type) {
        case SettingType::Bool:
            return value.get<bool>() ? "true" : "false";
        case SettingType::Int:
            return std::to_string(value.get<long long>());
        case SettingType::Double:
            oss << std::setprecision(12) << value.get<double>();
            return oss.str();
        case SettingType::String: {
            const std::string text = value.get<std::string>();
            std::string escaped;
            escaped.reserve(text.size());
            for (const char ch : text) {
                if (ch == '"' || ch == '\\') escaped.push_back('\\');
                escaped.push_back(ch);
            }
            return "\"" + escaped + "\"";
        }
    }
    return "";
}

std::string mower_logic_settings_yaml_path;
std::mutex mower_logic_settings_yaml_mutex;
std::map<std::string, json> startup_effective_persistent_values;

std::map<std::string, json> load_persistent_mower_logic_settings_locked() {
    std::map<std::string, json> values;
    if (mower_logic_settings_yaml_path.empty()) return values;
    std::ifstream in(mower_logic_settings_yaml_path);
    if (!in.good()) return values;

    bool in_mower_logic = false;
    std::string line;
    const std::regex top_level_re(R"(^[^\s#][^:]*:\s*(?:#.*)?$)");
    const std::regex setting_re(R"(^\s{2}([A-Za-z0-9_]+):\s*(.*?)\s*$)");
    while (std::getline(in, line)) {
        if (std::regex_match(line, top_level_re)) {
            in_mower_logic = trim_copy(line).rfind("mower_logic:", 0) == 0;
            continue;
        }
        if (!in_mower_logic) continue;
        std::smatch match;
        if (std::regex_match(line, match, setting_re)) {
            const std::string key = match[1].str();
            if (find_definition(key) != nullptr) {
                values[key] = parse_yaml_scalar(match[2].str());
            }
        }
    }
    return values;
}

std::map<std::string, json> load_persistent_mower_logic_settings() {
    std::lock_guard<std::mutex> lk(mower_logic_settings_yaml_mutex);
    return load_persistent_mower_logic_settings_locked();
}

bool write_persistent_mower_logic_settings(const json &accepted, std::string &error_message) {
    std::lock_guard<std::mutex> lk(mower_logic_settings_yaml_mutex);
    if (mower_logic_settings_yaml_path.empty()) {
        error_message = "Persistent YAML path is empty.";
        return false;
    }

    std::vector<std::string> lines;
    {
        std::ifstream in(mower_logic_settings_yaml_path);
        std::string line;
        while (std::getline(in, line)) lines.push_back(line);
    }

    bool found_section = false;
    bool in_section = false;
    std::size_t section_insert_index = lines.size();
    std::set<std::string> remaining;
    for (auto it = accepted.begin(); it != accepted.end(); ++it) remaining.insert(it.key());

    const std::regex top_level_re(R"(^[^\s#][^:]*:\s*(?:#.*)?$)");
    const std::regex setting_re(R"(^(\s{2})([A-Za-z0-9_]+):(.*)$)");

    for (std::size_t i = 0; i < lines.size(); ++i) {
        std::string &line = lines[i];
        if (std::regex_match(line, top_level_re)) {
            if (in_section) {
                section_insert_index = i;
                in_section = false;
            }
            if (trim_copy(line).rfind("mower_logic:", 0) == 0) {
                found_section = true;
                in_section = true;
                section_insert_index = i + 1;
            }
            continue;
        }
        if (!in_section) continue;
        section_insert_index = i + 1;
        std::smatch match;
        if (std::regex_match(line, match, setting_re)) {
            const std::string key = match[2].str();
            if (accepted.contains(key)) {
                const SettingDefinition *definition = find_definition(key);
                if (definition != nullptr) {
                    const std::string tail = match[3].str();
                    const auto comment_pos = tail.find('#');
                    const std::string comment = comment_pos == std::string::npos ? "" : " " + trim_copy(tail.substr(comment_pos));
                    line = match[1].str() + key + ": " + format_yaml_scalar(accepted[key], definition->type) + comment;
                    remaining.erase(key);
                }
            }
        }
    }

    if (!found_section) {
        if (!lines.empty() && !lines.back().empty()) lines.push_back("");
        lines.push_back("mower_logic:");
        section_insert_index = lines.size();
    }

    std::vector<std::string> additions;
    for (const auto &key : remaining) {
        const SettingDefinition *definition = find_definition(key);
        if (definition != nullptr) {
            additions.push_back("  " + key + ": " + format_yaml_scalar(accepted[key], definition->type));
        }
    }
    lines.insert(lines.begin() + static_cast<std::ptrdiff_t>(section_insert_index), additions.begin(), additions.end());

    const std::filesystem::path target_path(mower_logic_settings_yaml_path);
    const std::filesystem::path parent_path = target_path.parent_path();
    try {
        if (!parent_path.empty()) std::filesystem::create_directories(parent_path);
        const std::filesystem::path backup_path = target_path.string() + ".bak";
        if (std::filesystem::exists(target_path)) {
            std::error_code ec;
            std::filesystem::copy_file(target_path, backup_path, std::filesystem::copy_options::overwrite_existing, ec);
            if (ec) ROS_WARN_STREAM("Unable to create settings YAML backup '" << backup_path << "': " << ec.message());
        }
        const std::filesystem::path temp_path = target_path.string() + ".tmp";
        {
            std::ofstream out(temp_path, std::ios::trunc);
            if (!out.good()) {
                error_message = "Unable to open temporary YAML file for writing.";
                return false;
            }
            for (const auto &line : lines) out << line << '\n';
            out.flush();
            if (!out.good()) {
                error_message = "Unable to write temporary YAML file.";
                return false;
            }
        }
        std::filesystem::rename(temp_path, target_path);
    } catch (const std::exception &e) {
        error_message = std::string("Unable to persist mower_logic settings YAML: ") + e.what();
        return false;
    }
    return true;
}

json active_value_for(const SettingDefinition &definition) {
    const std::string path = std::string("/mower_logic/") + definition.key;
    switch (definition.type) {
        case SettingType::Bool: {
            bool value = false;
            return ros::param::get(path, value) ? json(value) : json(nullptr);
        }
        case SettingType::Int: {
            int value = 0;
            return ros::param::get(path, value) ? json(value) : json(nullptr);
        }
        case SettingType::Double: {
            double value = 0.0;
            return ros::param::get(path, value) ? json(value) : json(nullptr);
        }
        case SettingType::String: {
            std::string value;
            return ros::param::get(path, value) ? json(value) : json(nullptr);
        }
    }
    return nullptr;
}

bool numeric_json_value(const json &value, double &number) {
    if (!value.is_number()) return false;
    number = value.get<double>();
    return true;
}

json make_validation_base(const std::string &scope) {
    return {{"valid", true}, {"scope", scope}, {"remarks", json::array()}, {"applied", json::object()}, {"rejected", json::object()}};
}

json validate_mower_logic_payload(const json &payload, const std::string &scope, json &accepted) {
    json validation = make_validation_base(scope);
    accepted = json::object();
    if (!payload.is_object()) {
        validation["valid"] = false;
        validation["remarks"].push_back("Payload must be a JSON object.");
        return validation;
    }

    for (auto it = payload.begin(); it != payload.end(); ++it) {
        const std::string key = it.key();
        const json &value = it.value();
        const SettingDefinition *definition = find_definition(key);
        if (definition == nullptr) {
            validation["valid"] = false;
            validation["remarks"].push_back("Unknown mower_logic setting: " + key);
            validation["rejected"][key] = value;
            continue;
        }
        bool type_valid = false;
        switch (definition->type) {
            case SettingType::Bool:
                type_valid = value.is_boolean();
                break;
            case SettingType::Int:
                type_valid = value.is_number_integer();
                break;
            case SettingType::Double:
                type_valid = value.is_number();
                break;
            case SettingType::String:
                type_valid = value.is_string();
                break;
        }
        if (!type_valid) {
            validation["valid"] = false;
            validation["remarks"].push_back("Invalid type for mower_logic setting: " + key);
            validation["rejected"][key] = value;
            continue;
        }
        if (definition->has_numeric_range) {
            double numeric_value = 0.0;
            if (!numeric_json_value(value, numeric_value) || numeric_value < definition->min_value || numeric_value > definition->max_value) {
                validation["valid"] = false;
                std::ostringstream oss;
                oss << key << " must be within [" << definition->min_value << ", " << definition->max_value << "].";
                validation["remarks"].push_back(oss.str());
                validation["rejected"][key] = value;
                continue;
            }
        }
        accepted[key] = value;
    }

    if (accepted.empty() && validation["valid"].get<bool>()) {
        validation["valid"] = false;
        validation["remarks"].push_back("Payload does not contain any editable mower_logic settings.");
    }
    return validation;
}

json value_from_base_or_updates(const std::string &key, const json &updates, const std::map<std::string, json> &base_values) {
    if (updates.contains(key)) return updates[key];
    const auto it = base_values.find(key);
    return it == base_values.end() ? json(nullptr) : it->second;
}

std::map<std::string, json> active_values_as_map() {
    std::map<std::string, json> values;
    for (const auto &definition : registry()) {
        const json value = active_value_for(definition);
        if (!value.is_null()) values[definition.key] = value;
    }
    return values;
}

std::map<std::string, json> effective_persistent_values_as_map() {
    std::map<std::string, json> values = startup_effective_persistent_values;
    const auto explicitly_persistent = load_persistent_mower_logic_settings();
    for (const auto &[key, value] : explicitly_persistent) values[key] = value;
    return values;
}

void validate_cross_constraints(json &validation, const json &accepted, const std::map<std::string, json> &base_values) {
    auto fail = [&](const std::string &message, const std::vector<std::string> &keys) {
        validation["valid"] = false;
        validation["remarks"].push_back(message);
        for (const auto &key : keys) {
            if (accepted.contains(key)) validation["rejected"][key] = accepted[key];
        }
    };

    auto compare_greater = [&](const std::string &upper_key, const std::string &lower_key, const std::string &message) {
        const json upper = value_from_base_or_updates(upper_key, accepted, base_values);
        const json lower = value_from_base_or_updates(lower_key, accepted, base_values);
        if (upper.is_number() && lower.is_number() && upper.get<double>() < lower.get<double>()) {
            fail(message, {upper_key, lower_key});
        }
    };

    compare_greater("motor_hot_temperature", "motor_cold_temperature", "motor_hot_temperature must be greater than or equal to motor_cold_temperature.");
    compare_greater("mow_load_current_end", "mow_load_current_start", "mow_load_current_end must be greater than or equal to mow_load_current_start.");
    compare_greater("mow_load_motor_temp_end", "mow_load_motor_temp_start", "mow_load_motor_temp_end must be greater than or equal to mow_load_motor_temp_start.");
    compare_greater("mow_load_esc_temp_end", "mow_load_esc_temp_start", "mow_load_esc_temp_end must be greater than or equal to mow_load_esc_temp_start.");
}

}  // namespace mower_logic_settings

void publish_mower_logic_settings_validation(const json &validation) {
    try_publish("settings/mower_logic/validation/json", validation.dump(2), true);
}

void publish_mower_logic_settings() {
    const auto persistent_values = mower_logic_settings::load_persistent_mower_logic_settings();
    json payload = json::object();
    payload["schema"] = "openmower.settings.mower_logic.v1";
    payload["persistent_storage"] = {
        {"path", mower_logic_settings::mower_logic_settings_yaml_path},
        {"mode", "yaml_only"}
    };
    payload["settings"] = json::object();

    for (const auto &definition : mower_logic_settings::registry()) {
        const json active = mower_logic_settings::active_value_for(definition);
        const auto persistent_it = persistent_values.find(definition.key);
        const auto startup_it = mower_logic_settings::startup_effective_persistent_values.find(definition.key);
        const json persistent = persistent_it != persistent_values.end()
            ? persistent_it->second
            : (startup_it != mower_logic_settings::startup_effective_persistent_values.end() ? startup_it->second : json(nullptr));
        const bool different = !active.is_null() && !persistent.is_null() && active != persistent;
        payload["settings"][definition.key] = {
            {"active", active},
            {"persistent", persistent},
            {"different", different},
            {"session_apply_supported", definition.session_apply_supported},
            {"restart_required", !definition.session_apply_supported},
            {"group", definition.group},
            {"label", definition.label},
            {"unit", definition.unit},
            {"description", definition.description},
            {"order", definition.order},
            {"type", definition.type == mower_logic_settings::SettingType::Bool ? "bool" :
                       definition.type == mower_logic_settings::SettingType::Int ? "int" :
                       definition.type == mower_logic_settings::SettingType::Double ? "float" : "string"}
        };
        if (definition.has_numeric_range) {
            payload["settings"][definition.key]["min"] = definition.min_value;
            payload["settings"][definition.key]["max"] = definition.max_value;
        }
    }
    try_publish("settings/mower_logic/json", payload.dump(2), true);
}

json handle_mower_logic_session_settings(const json &payload) {
    json accepted;
    json validation = mower_logic_settings::validate_mower_logic_payload(payload, "session", accepted);
    const auto active_values = mower_logic_settings::active_values_as_map();
    mower_logic_settings::validate_cross_constraints(validation, accepted, active_values);
    if (!validation.value("valid", false)) return validation;

    dynamic_reconfigure::Reconfigure service;
    for (auto it = accepted.begin(); it != accepted.end(); ++it) {
        const mower_logic_settings::SettingDefinition *definition = mower_logic_settings::find_definition(it.key());
        if (definition == nullptr) continue;
        switch (definition->type) {
            case mower_logic_settings::SettingType::Bool: {
                dynamic_reconfigure::BoolParameter parameter;
                parameter.name = it.key();
                parameter.value = it.value().get<bool>();
                service.request.config.bools.push_back(parameter);
                break;
            }
            case mower_logic_settings::SettingType::Int: {
                dynamic_reconfigure::IntParameter parameter;
                parameter.name = it.key();
                parameter.value = it.value().get<int>();
                service.request.config.ints.push_back(parameter);
                break;
            }
            case mower_logic_settings::SettingType::Double: {
                dynamic_reconfigure::DoubleParameter parameter;
                parameter.name = it.key();
                parameter.value = it.value().get<double>();
                service.request.config.doubles.push_back(parameter);
                break;
            }
            case mower_logic_settings::SettingType::String: {
                dynamic_reconfigure::StrParameter parameter;
                parameter.name = it.key();
                parameter.value = it.value().get<std::string>();
                service.request.config.strs.push_back(parameter);
                break;
            }
        }
    }
    if (!mower_logic_reconfigure_client.call(service)) {
        validation["valid"] = false;
        validation["remarks"].push_back("Failed to call /mower_logic/set_parameters.");
        validation["rejected"] = accepted;
        validation["applied"] = json::object();
        return validation;
    }
    validation["applied"] = accepted;
    validation["remarks"].push_back("Session settings applied via dynamic_reconfigure.");
    return validation;
}

json handle_mower_logic_persistent_settings(const json &payload) {
    json accepted;
    json validation = mower_logic_settings::validate_mower_logic_payload(payload, "persistent", accepted);
    const auto persistent_values = mower_logic_settings::effective_persistent_values_as_map();
    mower_logic_settings::validate_cross_constraints(validation, accepted, persistent_values);
    if (!validation.value("valid", false)) return validation;

    std::string error_message;
    if (!mower_logic_settings::write_persistent_mower_logic_settings(accepted, error_message)) {
        validation["valid"] = false;
        validation["remarks"].push_back(error_message);
        validation["rejected"] = accepted;
        validation["applied"] = json::object();
        return validation;
    }
    validation["applied"] = accepted;
    validation["remarks"].push_back("Persistent settings stored in YAML. Active session values were not changed.");
    return validation;
}
// END: mower_logic settings MQTT bridge

void mow_load_factor_status_json_callback(const std_msgs::String::ConstPtr &msg) {
    try_publish("mow_load_factor/json", msg->data, true);
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

void publish_sensor_metadata() {
    json sensor_info;
    {
        std::unique_lock<std::mutex> lk(found_sensors_mutex);

        if(found_sensors.empty())
            return;

        for (const auto &kv: found_sensors) {
            json info;
            info["sensor_id"] = kv.second.sensor_id;
            info["sensor_name"] = kv.second.sensor_name;

            switch (kv.second.value_type) {
                case xbot_msgs::SensorInfo::TYPE_STRING: {
                    info["value_type"] = "STRING";
                    break;
                }
                case xbot_msgs::SensorInfo::TYPE_DOUBLE: {
                    info["value_type"] = "DOUBLE";
                    break;
                }
                default: {
                    info["value_type"] = "UNKNOWN";
                    break;
                }


            }

            switch (kv.second.value_description) {
                case xbot_msgs::SensorInfo::VALUE_DESCRIPTION_TEMPERATURE: {
                    info["value_description"] = "TEMPERATURE";
                    break;
                }
                case xbot_msgs::SensorInfo::VALUE_DESCRIPTION_VELOCITY: {
                    info["value_description"] = "VELOCITY";
                    break;
                }
                case xbot_msgs::SensorInfo::VALUE_DESCRIPTION_ACCELERATION: {
                    info["value_description"] = "ACCELERATION";
                    break;
                }
                case xbot_msgs::SensorInfo::VALUE_DESCRIPTION_VOLTAGE: {
                    info["value_description"] = "VOLTAGE";
                    break;
                }
                case xbot_msgs::SensorInfo::VALUE_DESCRIPTION_CURRENT: {
                    info["value_description"] = "CURRENT";
                    break;
                }
                case xbot_msgs::SensorInfo::VALUE_DESCRIPTION_PERCENT: {
                    info["value_description"] = "PERCENT";
                    break;
                }
                case xbot_msgs::SensorInfo::VALUE_DESCRIPTION_RPM: {
                    info["value_description"] = "REVOLUTIONS";
                    break;
                }
                default: {
                    info["value_description"] = "UNKNOWN";
                    break;
                }
            }

            info["unit"] = kv.second.unit;
            info["has_min_max"] = kv.second.has_min_max;
            info["min_value"] = kv.second.min_value;
            info["max_value"] = kv.second.max_value;
            info["has_critical_low"] = kv.second.has_critical_low;
            info["lower_critical_value"] = kv.second.lower_critical_value;
            info["has_critical_high"] = kv.second.has_critical_high;
            info["upper_critical_value"] = kv.second.upper_critical_value;
            sensor_info.push_back(info);
        }
    }
    try_publish("sensor_infos/json", sensor_info.dump(), true);
    json data;
    data["d"] = sensor_info;
    auto bson = json::to_bson(data);
    try_publish_binary("sensor_infos/bson", bson.data(), bson.size(), true);
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
                try_publish("sensors/" + info.sensor_id + "/data", std::to_string(msg->data));
                {
                    std::lock_guard<std::mutex> lk(latest_double_sensor_values_mutex);
                    latest_double_sensor_values[info.sensor_id] = msg->data;
                }

                json data;
                data["d"] = msg->data;
                auto bson = json::to_bson(data);
                try_publish_binary("sensors/" + info.sensor_id + "/bson", bson.data(), bson.size());
            });
            sensor_data_subscribers.push_back(s);
            break;
        }
        case xbot_msgs::SensorInfo::TYPE_STRING: {
            ros::Subscriber s = n->subscribe<xbot_msgs::SensorDataString>(data_topic, 10, [info = sensor](
                    const xbot_msgs::SensorDataString::ConstPtr &msg) {
                try_publish("sensors/" + info.sensor_id + "/data", msg->data);

                json data;
                data["d"] = msg->data;
                auto bson = json::to_bson(data);
                try_publish_binary("sensors/" + info.sensor_id + "/bson", bson.data(), bson.size());
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

    maybe_append_statustransition_log(msg);

    try_publish("robot_state/json", j.dump());
    json data;
    data["d"] = j;
    auto bson = json::to_bson(data);
    try_publish_binary("robot_state/bson", bson.data(), bson.size());
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
    try_publish("map_overlay/json", m.dump(), true);
    json data;
    data["d"] = m;
    auto bson = json::to_bson(data);
    try_publish_binary("map_overlay/bson", bson.data(), bson.size(), true);
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
    const char *params_path_env = std::getenv("PARAMS_PATH");
    const std::string default_mower_logic_settings_yaml_path = params_path_env != nullptr
        ? std::string(params_path_env) + "/mower_params.yaml"
        : std::string("/data/ros/mower_params.yaml");
    mower_logic_settings::mower_logic_settings_yaml_path = paramNh.param(
        "mower_logic_settings_yaml_path", default_mower_logic_settings_yaml_path);
    mower_logic_settings::startup_effective_persistent_values = mower_logic_settings::active_values_as_map();
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
    ros::Subscriber mowLoadFactorStatusSubscriber = n->subscribe("/mower_logic/mow_load_factor/status_json", 10, mow_load_factor_status_json_callback);

    cmd_vel_pub = n->advertise<geometry_msgs::Twist>("xbot_monitoring/remote_cmd_vel", 1);
    action_pub = n->advertise<std_msgs::String>("xbot/action", 1);
    mow_load_factor_set_enabled_pub = n->advertise<std_msgs::Bool>("/mower_logic/mow_load_factor/set_enabled", 10);
    mow_load_factor_set_min_factor_pub = n->advertise<std_msgs::Float32>("/mower_logic/mow_load_factor/set_min_factor", 10);
    mow_load_factor_set_current_start_pub = n->advertise<std_msgs::Float32>("/mower_logic/mow_load_factor/set_current_start", 10);
    mow_load_factor_set_current_end_pub = n->advertise<std_msgs::Float32>("/mower_logic/mow_load_factor/set_current_end", 10);
    mow_load_factor_renew_pub = n->advertise<std_msgs::Empty>("/mower_logic/mow_load_factor/renew", 10);
    mower_logic_reconfigure_client = n->serviceClient<dynamic_reconfigure::Reconfigure>("/mower_logic/set_parameters");

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
