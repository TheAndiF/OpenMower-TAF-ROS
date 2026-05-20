// Mowing load factor calculation and runtime configuration bridge.
// This node intentionally does not influence driving speed yet.

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

#include <mower_msgs/Status.h>
#include <nlohmann/json.hpp>
#include <open_mower/settings_persistence.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>

class MowLoadFactorNode {
 public:
  MowLoadFactorNode()
      : nh_(), last_status_json_publish_wall_time_(), last_factor_current_(1.0),
        last_factor_motor_temp_(1.0), last_factor_esc_temp_(1.0), last_computed_factor_(1.0),
        last_effective_factor_(1.0) {
    nh_.param("/settings/persistent_file", settings_persistent_path_,
              std::string("/data/ros/settings_persistent.json"));

    loadBootstrapParameters();
    initializePersistentSettings();

    computed_pub_ = nh_.advertise<std_msgs::Float32>("/mower_logic/mow_load_factor/computed", 1, true);
    effective_pub_ = nh_.advertise<std_msgs::Float32>("/mower_logic/mow_load_factor/effective", 1, true);
    load_factor_computed_pub_ = nh_.advertise<std_msgs::Float32>("/mower_logic/mow_load_factor/load_factor_computed", 1, true);
    load_factor_effective_pub_ = nh_.advertise<std_msgs::Float32>("/mower_logic/mow_load_factor/load_factor_effective", 1, true);
    status_json_pub_ = nh_.advertise<std_msgs::String>("/mower_logic/mow_load_factor/status_json", 1, true);

    status_sub_ = nh_.subscribe("/ll/mower_status", 10, &MowLoadFactorNode::statusCallback, this);
    set_enabled_sub_ = nh_.subscribe("/mower_logic/mow_load_factor/set_enabled", 10,
                                     &MowLoadFactorNode::setEnabledSessionCallback, this);
    set_min_factor_sub_ = nh_.subscribe("/mower_logic/mow_load_factor/set_min_factor", 10,
                                        &MowLoadFactorNode::setMinFactorSessionCallback, this);
    set_current_start_sub_ = nh_.subscribe("/mower_logic/mow_load_factor/set_current_start", 10,
                                           &MowLoadFactorNode::setCurrentStartSessionCallback, this);
    set_current_end_sub_ = nh_.subscribe("/mower_logic/mow_load_factor/set_current_end", 10,
                                         &MowLoadFactorNode::setCurrentEndSessionCallback, this);
    set_persistent_enabled_sub_ = nh_.subscribe("/mower_logic/mow_load_factor/set_persistent_enabled", 10,
                                                &MowLoadFactorNode::setEnabledPersistentCallback, this);
    set_persistent_min_factor_sub_ = nh_.subscribe("/mower_logic/mow_load_factor/set_persistent_min_factor", 10,
                                                   &MowLoadFactorNode::setMinFactorPersistentCallback, this);
    set_persistent_current_start_sub_ = nh_.subscribe("/mower_logic/mow_load_factor/set_persistent_current_start", 10,
                                                      &MowLoadFactorNode::setCurrentStartPersistentCallback, this);
    set_persistent_current_end_sub_ = nh_.subscribe("/mower_logic/mow_load_factor/set_persistent_current_end", 10,
                                                    &MowLoadFactorNode::setCurrentEndPersistentCallback, this);
    renew_sub_ = nh_.subscribe("/mower_logic/mow_load_factor/renew", 10,
                               &MowLoadFactorNode::renewCallback, this);

    publishFactorTopics();
    publishStatusJson();

    ROS_INFO_STREAM("Mow load factor node ready: enabled=" << (enabled_ ? "true" : "false")
                    << ", min_factor=" << min_factor_ << ", current_start=" << current_start_
                    << ", current_end=" << current_end_ << ", settings_persistent_file="
                    << settings_persistent_path_);
  }

 private:
  using json = open_mower_settings::json;

  static double minAllowedFactor() { return 0.10; }
  static double maxAllowedFactor() { return 1.00; }
  static constexpr const char* kNamespace = "mow_load_factor";

  void loadBootstrapParameters() {
    nh_.param("/mower_logic/mow_load_factor_enabled", enabled_, false);
    nh_.param("/mower_logic/mow_load_factor_min", min_factor_, 0.40);
    nh_.param("/mower_logic/mow_load_current_start", current_start_, 0.75);
    nh_.param("/mower_logic/mow_load_current_end", current_end_, 1.25);
    nh_.param("/mower_logic/mow_load_motor_temp_start", motor_temp_start_, 55.0);
    nh_.param("/mower_logic/mow_load_motor_temp_end", motor_temp_end_, 68.0);
    nh_.param("/mower_logic/mow_load_esc_temp_start", esc_temp_start_, 60.0);
    nh_.param("/mower_logic/mow_load_esc_temp_end", esc_temp_end_, 78.0);
    nh_.param("/mower_logic/mow_load_status_publish_period", status_publish_period_, 0.50);

    if (!std::isfinite(min_factor_) || min_factor_ < minAllowedFactor() || min_factor_ > maxAllowedFactor()) {
      ROS_WARN_STREAM("Invalid /mower_logic/mow_load_factor_min=" << min_factor_
                      << ". Falling back to 0.40.");
      min_factor_ = 0.40;
      ros::param::set("/mower_logic/mow_load_factor_min", min_factor_);
    }
    if (!std::isfinite(current_start_) || current_start_ < 0.0) {
      ROS_WARN_STREAM("Invalid /mower_logic/mow_load_current_start=" << current_start_
                      << ". Falling back to 0.75 A.");
      current_start_ = 0.75;
      ros::param::set("/mower_logic/mow_load_current_start", current_start_);
    }
    if (!std::isfinite(current_end_) || current_end_ <= current_start_) {
      ROS_WARN_STREAM("Invalid /mower_logic/mow_load_current_end=" << current_end_
                      << ". Falling back to 1.25 A.");
      current_end_ = std::max(1.25, current_start_ + 0.01);
      ros::param::set("/mower_logic/mow_load_current_end", current_end_);
    }
    if (!std::isfinite(status_publish_period_) || status_publish_period_ < 0.05) {
      ROS_WARN_STREAM("Invalid /mower_logic/mow_load_status_publish_period=" << status_publish_period_
                      << ". Falling back to 0.50 s.");
      status_publish_period_ = 0.50;
      ros::param::set("/mower_logic/mow_load_status_publish_period", status_publish_period_);
    }
  }

  json makeBooleanEntry(const std::string& label, const std::string& description, int order, bool default_value) {
    json entry = json::object();
    entry["label"] = label;
    entry["description"] = description;
    entry["group"] = kNamespace;
    entry["order"] = order;
    entry["session_apply_supported"] = true;
    entry["restart_required"] = false;
    entry["default"] = default_value;
    entry["persistent"] = default_value;
    entry["unit"] = "";
    entry["type"] = "boolean";
    return entry;
  }

  json makeNumberEntry(const std::string& label, const std::string& description, int order, double default_value,
                       double min_value, double max_value, const std::string& unit) {
    json entry = json::object();
    entry["label"] = label;
    entry["description"] = description;
    entry["group"] = kNamespace;
    entry["order"] = order;
    entry["session_apply_supported"] = true;
    entry["restart_required"] = false;
    entry["default"] = default_value;
    entry["persistent"] = default_value;
    entry["unit"] = unit;
    entry["type"] = "number";
    entry["min"] = min_value;
    entry["max"] = max_value;
    return entry;
  }

  json seedEntriesFromBootstrap() {
    json seed = json::object();
    seed["enabled"] = makeBooleanEntry("Mäh-Lastfaktor aktiv",
                                        "Berechneten Mäh-Lastfaktor statt festem Faktor 1.0 verwenden.", 10,
                                        enabled_);
    seed["min_factor"] = makeNumberEntry("Minimaler Lastfaktor",
                                          "Untergrenze des berechneten Mäh-Lastfaktors.", 20, min_factor_,
                                          minAllowedFactor(), maxAllowedFactor(), "");
    seed["current_start"] = makeNumberEntry("Strombeginn Derating",
                                             "Mähmotorstrom, ab dem die Lastfaktor-Absenkung beginnt.", 30,
                                             current_start_, 0.0, 100.0, "A");
    seed["current_end"] = makeNumberEntry("Stromende Derating",
                                           "Mähmotorstrom, bei dem der Minimalfaktor erreicht wird.", 40,
                                           current_end_, 0.0, 100.0, "A");
    return seed;
  }

  void initializePersistentSettings() {
    settings_entries_ = open_mower_settings::mergeNamespaceWithSeed(settings_persistent_path_, kNamespace,
                                                                     seedEntriesFromBootstrap());
    loadPersistentValuesFromEntries(true);
  }

  void reloadPersistentSettingsMetadata() {
    settings_entries_ = open_mower_settings::mergeNamespaceWithSeed(settings_persistent_path_, kNamespace,
                                                                     seedEntriesFromBootstrap());
    loadPersistentValuesFromEntries(false);
  }

  double entryNumber(const std::string& key, const std::string& field, double fallback) const {
    if (!settings_entries_.contains(key) || !settings_entries_[key].is_object()) {
      return fallback;
    }
    return open_mower_settings::numberOr(settings_entries_[key], field, fallback);
  }

  bool entryBoolean(const std::string& key, const std::string& field, bool fallback) const {
    if (!settings_entries_.contains(key) || !settings_entries_[key].is_object()) {
      return fallback;
    }
    return open_mower_settings::boolOr(settings_entries_[key], field, fallback);
  }

  double entryMin(const std::string& key, double fallback) const {
    return entryNumber(key, "min", fallback);
  }

  double entryMax(const std::string& key, double fallback) const {
    return entryNumber(key, "max", fallback);
  }

  void loadPersistentValuesFromEntries(bool apply_to_active) {
    const bool default_enabled = entryBoolean("enabled", "default", enabled_);
    const double default_min_factor = entryNumber("min_factor", "default", min_factor_);
    const double default_current_start = entryNumber("current_start", "default", current_start_);
    const double default_current_end = entryNumber("current_end", "default", current_end_);

    persistent_enabled_ = entryBoolean("enabled", "persistent", default_enabled);
    persistent_min_factor_ = entryNumber("min_factor", "persistent", default_min_factor);
    persistent_current_start_ = entryNumber("current_start", "persistent", default_current_start);
    persistent_current_end_ = entryNumber("current_end", "persistent", default_current_end);

    bool repaired = false;
    if (!std::isfinite(persistent_min_factor_) || persistent_min_factor_ < entryMin("min_factor", minAllowedFactor()) ||
        persistent_min_factor_ > entryMax("min_factor", maxAllowedFactor())) {
      persistent_min_factor_ = default_min_factor;
      settings_entries_["min_factor"]["persistent"] = persistent_min_factor_;
      open_mower_settings::updateEntryField(settings_persistent_path_, kNamespace, "min_factor", "persistent",
                                            persistent_min_factor_);
      repaired = true;
    }
    if (!std::isfinite(persistent_current_start_) ||
        persistent_current_start_ < entryMin("current_start", 0.0) ||
        persistent_current_start_ > entryMax("current_start", 100.0)) {
      persistent_current_start_ = default_current_start;
      settings_entries_["current_start"]["persistent"] = persistent_current_start_;
      open_mower_settings::updateEntryField(settings_persistent_path_, kNamespace, "current_start", "persistent",
                                            persistent_current_start_);
      repaired = true;
    }
    if (!std::isfinite(persistent_current_end_) ||
        persistent_current_end_ < entryMin("current_end", 0.0) ||
        persistent_current_end_ > entryMax("current_end", 100.0) ||
        persistent_current_end_ <= persistent_current_start_) {
      persistent_current_end_ = std::max(default_current_end, persistent_current_start_ + 0.01);
      settings_entries_["current_end"]["persistent"] = persistent_current_end_;
      open_mower_settings::updateEntryField(settings_persistent_path_, kNamespace, "current_end", "persistent",
                                            persistent_current_end_);
      repaired = true;
    }
    if (repaired) {
      ROS_WARN_STREAM("Repaired invalid persistent mow_load_factor values in " << settings_persistent_path_);
    }

    syncPersistentParamTree();

    if (apply_to_active) {
      enabled_ = persistent_enabled_;
      min_factor_ = persistent_min_factor_;
      current_start_ = persistent_current_start_;
      current_end_ = persistent_current_end_;
      syncActiveParamTree();
      syncLegacyWorkingParams();
      last_effective_factor_ = enabled_ ? last_computed_factor_ : 1.0;
    }
  }

  void syncDefaultParamTree() const {
    for (const std::string key : {"min_factor", "current_start", "current_end"}) {
      ros::param::set(std::string("/settings/") + kNamespace + "/default/" + key,
                      entryNumber(key, "default", 0.0));
    }
    ros::param::set(std::string("/settings/") + kNamespace + "/default/enabled",
                    entryBoolean("enabled", "default", false));
  }

  void syncPersistentParamTree() const {
    syncDefaultParamTree();
    ros::param::set(std::string("/settings/") + kNamespace + "/persistent/enabled", persistent_enabled_);
    ros::param::set(std::string("/settings/") + kNamespace + "/persistent/min_factor", persistent_min_factor_);
    ros::param::set(std::string("/settings/") + kNamespace + "/persistent/current_start", persistent_current_start_);
    ros::param::set(std::string("/settings/") + kNamespace + "/persistent/current_end", persistent_current_end_);
  }

  void syncActiveParamTree() const {
    ros::param::set(std::string("/settings/") + kNamespace + "/active/enabled", enabled_);
    ros::param::set(std::string("/settings/") + kNamespace + "/active/min_factor", min_factor_);
    ros::param::set(std::string("/settings/") + kNamespace + "/active/current_start", current_start_);
    ros::param::set(std::string("/settings/") + kNamespace + "/active/current_end", current_end_);
  }

  void syncLegacyWorkingParams() const {
    ros::param::set("/mower_logic/mow_load_factor_enabled", enabled_);
    ros::param::set("/mower_logic/mow_load_factor_min", min_factor_);
    ros::param::set("/mower_logic/mow_load_current_start", current_start_);
    ros::param::set("/mower_logic/mow_load_current_end", current_end_);
  }

  double derateFactor(double value, double start, double end) const {
    if (!std::isfinite(value) || !std::isfinite(start) || !std::isfinite(end)) {
      return 1.0;
    }
    if (end <= start) {
      ROS_WARN_STREAM_THROTTLE(10.0,
                               "Invalid mowing load factor thresholds: end <= start. Derating disabled for this input.");
      return 1.0;
    }
    if (value <= start) {
      return 1.0;
    }
    if (value >= end) {
      return min_factor_;
    }
    const double ratio = (value - start) / (end - start);
    return 1.0 - ratio * (1.0 - min_factor_);
  }

  void statusCallback(const mower_msgs::Status::ConstPtr& msg) {
    last_factor_current_ = derateFactor(msg->mower_esc_current, current_start_, current_end_);
    last_factor_motor_temp_ = derateFactor(msg->mower_motor_temperature, motor_temp_start_, motor_temp_end_);
    last_factor_esc_temp_ = derateFactor(msg->mower_esc_temperature, esc_temp_start_, esc_temp_end_);

    last_computed_factor_ = std::min({last_factor_current_, last_factor_motor_temp_, last_factor_esc_temp_});
    last_effective_factor_ = enabled_ ? last_computed_factor_ : 1.0;

    publishFactorTopics();
  }

  bool validateMinFactor(double requested) const {
    return std::isfinite(requested) && requested >= entryMin("min_factor", minAllowedFactor()) &&
           requested <= entryMax("min_factor", maxAllowedFactor());
  }

  bool validateCurrentStart(double requested) const {
    return std::isfinite(requested) && requested >= entryMin("current_start", 0.0) &&
           requested <= entryMax("current_start", 100.0) && requested < current_end_;
  }

  bool validateCurrentEnd(double requested) const {
    return std::isfinite(requested) && requested >= entryMin("current_end", 0.0) &&
           requested <= entryMax("current_end", 100.0) && requested > current_start_;
  }

  void applyEnabled(bool value, bool persist) {
    enabled_ = value;
    if (persist) {
      persistent_enabled_ = value;
      settings_entries_["enabled"]["persistent"] = value;
      open_mower_settings::updateEntryField(settings_persistent_path_, kNamespace, "enabled", "persistent", value);
      syncPersistentParamTree();
    }
    syncActiveParamTree();
    syncLegacyWorkingParams();
    last_effective_factor_ = enabled_ ? last_computed_factor_ : 1.0;
    publishFactorTopics();
    publishStatusJson();
    ROS_INFO_STREAM("Mow load factor " << (persist ? "persistent" : "session")
                    << " enabled set to " << (enabled_ ? "true" : "false"));
  }

  void applyMinFactor(double requested, bool persist) {
    if (!validateMinFactor(requested)) {
      ROS_WARN_STREAM("Rejected mowing load min_factor=" << requested << ". Allowed range is "
                      << entryMin("min_factor", minAllowedFactor()) << " to "
                      << entryMax("min_factor", maxAllowedFactor()) << ".");
      publishStatusJson();
      return;
    }
    min_factor_ = requested;
    if (persist) {
      persistent_min_factor_ = requested;
      settings_entries_["min_factor"]["persistent"] = requested;
      open_mower_settings::updateEntryField(settings_persistent_path_, kNamespace, "min_factor", "persistent",
                                            requested);
      syncPersistentParamTree();
    }
    syncActiveParamTree();
    syncLegacyWorkingParams();
    publishStatusJson();
    ROS_INFO_STREAM("Mow load factor " << (persist ? "persistent" : "session")
                    << " min_factor set to " << min_factor_);
  }

  void applyCurrentStart(double requested, bool persist) {
    if (!validateCurrentStart(requested)) {
      ROS_WARN_STREAM("Rejected mowing load current_start=" << requested
                      << ". It must be within metadata limits and lower than current_end=" << current_end_ << ".");
      publishStatusJson();
      return;
    }
    current_start_ = requested;
    if (persist) {
      persistent_current_start_ = requested;
      settings_entries_["current_start"]["persistent"] = requested;
      open_mower_settings::updateEntryField(settings_persistent_path_, kNamespace, "current_start", "persistent",
                                            requested);
      syncPersistentParamTree();
    }
    syncActiveParamTree();
    syncLegacyWorkingParams();
    publishStatusJson();
    ROS_INFO_STREAM("Mow load factor " << (persist ? "persistent" : "session")
                    << " current_start set to " << current_start_ << " A");
  }

  void applyCurrentEnd(double requested, bool persist) {
    if (!validateCurrentEnd(requested)) {
      ROS_WARN_STREAM("Rejected mowing load current_end=" << requested
                      << ". It must be within metadata limits and higher than current_start=" << current_start_ << ".");
      publishStatusJson();
      return;
    }
    current_end_ = requested;
    if (persist) {
      persistent_current_end_ = requested;
      settings_entries_["current_end"]["persistent"] = requested;
      open_mower_settings::updateEntryField(settings_persistent_path_, kNamespace, "current_end", "persistent",
                                            requested);
      syncPersistentParamTree();
    }
    syncActiveParamTree();
    syncLegacyWorkingParams();
    publishStatusJson();
    ROS_INFO_STREAM("Mow load factor " << (persist ? "persistent" : "session")
                    << " current_end set to " << current_end_ << " A");
  }

  void setEnabledSessionCallback(const std_msgs::Bool::ConstPtr& msg) { applyEnabled(msg->data, false); }
  void setEnabledPersistentCallback(const std_msgs::Bool::ConstPtr& msg) { applyEnabled(msg->data, true); }
  void setMinFactorSessionCallback(const std_msgs::Float32::ConstPtr& msg) {
    applyMinFactor(static_cast<double>(msg->data), false);
  }
  void setMinFactorPersistentCallback(const std_msgs::Float32::ConstPtr& msg) {
    applyMinFactor(static_cast<double>(msg->data), true);
  }
  void setCurrentStartSessionCallback(const std_msgs::Float32::ConstPtr& msg) {
    applyCurrentStart(static_cast<double>(msg->data), false);
  }
  void setCurrentStartPersistentCallback(const std_msgs::Float32::ConstPtr& msg) {
    applyCurrentStart(static_cast<double>(msg->data), true);
  }
  void setCurrentEndSessionCallback(const std_msgs::Float32::ConstPtr& msg) {
    applyCurrentEnd(static_cast<double>(msg->data), false);
  }
  void setCurrentEndPersistentCallback(const std_msgs::Float32::ConstPtr& msg) {
    applyCurrentEnd(static_cast<double>(msg->data), true);
  }

  void renewCallback(const std_msgs::Empty::ConstPtr&) {
    reloadPersistentSettingsMetadata();
    publishFactorTopics();
    publishStatusJson();
  }

  void publishFactorTopics() const {
    std_msgs::Float32 computed;
    computed.data = static_cast<float>(last_computed_factor_);
    computed_pub_.publish(computed);

    std_msgs::Float32 effective;
    effective.data = static_cast<float>(last_effective_factor_);
    effective_pub_.publish(effective);

    // Clear semantic names for consumers that want to enrich RobotState and UIs.
    load_factor_computed_pub_.publish(computed);
    load_factor_effective_pub_.publish(effective);
  }


  static bool differs(double active, double persistent) { return std::fabs(active - persistent) > 1e-9; }

  json orderedStatusBase(const json& entry) const {
    json out = json::object();
    out["label"] = open_mower_settings::stringOr(entry, "label", "");
    out["description"] = open_mower_settings::stringOr(entry, "description", "");
    out["group"] = open_mower_settings::stringOr(entry, "group", kNamespace);
    out["order"] = open_mower_settings::intOr(entry, "order", 0);
    out["session_apply_supported"] = open_mower_settings::boolOr(entry, "session_apply_supported", true);
    out["restart_required"] = open_mower_settings::boolOr(entry, "restart_required", false);
    return out;
  }

  json statusBoolean(const std::string& key, bool active, bool persistent) const {
    const json& entry = settings_entries_.at(key);
    json out = orderedStatusBase(entry);
    out["default"] = open_mower_settings::boolOr(entry, "default", false);
    out["persistent"] = persistent;
    out["active"] = active;
    out["different"] = active != persistent;
    out["unit"] = open_mower_settings::stringOr(entry, "unit", "");
    out["type"] = open_mower_settings::stringOr(entry, "type", "boolean");
    return out;
  }

  json statusNumber(const std::string& key, double active, double persistent) const {
    const json& entry = settings_entries_.at(key);
    json out = orderedStatusBase(entry);
    out["default"] = open_mower_settings::numberOr(entry, "default", 0.0);
    out["persistent"] = persistent;
    out["active"] = active;
    out["different"] = differs(active, persistent);
    out["unit"] = open_mower_settings::stringOr(entry, "unit", "");
    out["type"] = open_mower_settings::stringOr(entry, "type", "number");
    if (entry.contains("min") && entry["min"].is_number()) {
      out["min"] = entry["min"];
    }
    if (entry.contains("max") && entry["max"].is_number()) {
      out["max"] = entry["max"];
    }
    return out;
  }

  void publishStatusJson() {
    json status = json::object();
    status["schema"] = "settings_v1";
    status["namespace"] = kNamespace;
    status["settings"] = json::object();
    status["settings"]["enabled"] = statusBoolean("enabled", enabled_, persistent_enabled_);
    status["settings"]["min_factor"] = statusNumber("min_factor", min_factor_, persistent_min_factor_);
    status["settings"]["current_start"] = statusNumber("current_start", current_start_, persistent_current_start_);
    status["settings"]["current_end"] = statusNumber("current_end", current_end_, persistent_current_end_);

    std_msgs::String msg;
    msg.data = status.dump();
    status_json_pub_.publish(msg);
    last_status_json_publish_wall_time_ = ros::WallTime::now();
  }

  ros::NodeHandle nh_;

  ros::Subscriber status_sub_;
  ros::Subscriber set_enabled_sub_;
  ros::Subscriber set_min_factor_sub_;
  ros::Subscriber set_current_start_sub_;
  ros::Subscriber set_current_end_sub_;
  ros::Subscriber set_persistent_enabled_sub_;
  ros::Subscriber set_persistent_min_factor_sub_;
  ros::Subscriber set_persistent_current_start_sub_;
  ros::Subscriber set_persistent_current_end_sub_;
  ros::Subscriber renew_sub_;

  mutable ros::Publisher computed_pub_;
  mutable ros::Publisher effective_pub_;
  mutable ros::Publisher load_factor_computed_pub_;
  mutable ros::Publisher load_factor_effective_pub_;
  mutable ros::Publisher status_json_pub_;

  std::string settings_persistent_path_;
  json settings_entries_;

  bool enabled_;
  double min_factor_;
  double current_start_;
  double current_end_;
  bool persistent_enabled_;
  double persistent_min_factor_;
  double persistent_current_start_;
  double persistent_current_end_;
  double motor_temp_start_;
  double motor_temp_end_;
  double esc_temp_start_;
  double esc_temp_end_;
  double status_publish_period_;

  ros::WallTime last_status_json_publish_wall_time_;

  double last_factor_current_;
  double last_factor_motor_temp_;
  double last_factor_esc_temp_;
  double last_computed_factor_;
  double last_effective_factor_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "mow_load_factor");
  MowLoadFactorNode node;
  ros::spin();
  return 0;
}
