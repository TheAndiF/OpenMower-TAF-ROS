// Mowing load factor calculation and runtime configuration bridge.
// This node intentionally does not influence driving speed yet.

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>

#include <mower_msgs/Status.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>

class MowLoadFactorNode {
 public:
  MowLoadFactorNode() : nh_(), last_status_json_publish_wall_time_(),
                        last_factor_current_(1.0), last_factor_motor_temp_(1.0), last_factor_esc_temp_(1.0),
                        last_computed_factor_(1.0), last_effective_factor_(1.0) {
    loadParameters();

    computed_pub_ = nh_.advertise<std_msgs::Float32>("/mower_logic/mow_load_factor/computed", 1, true);
    effective_pub_ = nh_.advertise<std_msgs::Float32>("/mower_logic/mow_load_factor/effective", 1, true);
    status_json_pub_ = nh_.advertise<std_msgs::String>("/mower_logic/mow_load_factor/status_json", 1, true);

    status_sub_ = nh_.subscribe("/ll/mower_status", 10, &MowLoadFactorNode::statusCallback, this);
    set_enabled_sub_ = nh_.subscribe("/mower_logic/mow_load_factor/set_enabled", 10,
                                     &MowLoadFactorNode::setEnabledCallback, this);
    set_min_factor_sub_ = nh_.subscribe("/mower_logic/mow_load_factor/set_min_factor", 10,
                                        &MowLoadFactorNode::setMinFactorCallback, this);
    set_current_start_sub_ = nh_.subscribe("/mower_logic/mow_load_factor/set_current_start", 10,
                                           &MowLoadFactorNode::setCurrentStartCallback, this);
    set_current_end_sub_ = nh_.subscribe("/mower_logic/mow_load_factor/set_current_end", 10,
                                         &MowLoadFactorNode::setCurrentEndCallback, this);
    renew_sub_ = nh_.subscribe("/mower_logic/mow_load_factor/renew", 10,
                               &MowLoadFactorNode::renewCallback, this);

    publishFactorTopics();
    publishStatusJson();

    ROS_INFO_STREAM("Mow load factor node ready: enabled=" << (enabled_ ? "true" : "false")
                    << ", min_factor=" << min_factor_
                    << ", current_start=" << current_start_
                    << ", current_end=" << current_end_);
  }

 private:
  static double minAllowedFactor() { return 0.10; }
  static double maxAllowedFactor() { return 1.00; }

  void loadParameters() {
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
    if (!std::isfinite(status_publish_period_) || status_publish_period_ < 0.05) {
      ROS_WARN_STREAM("Invalid /mower_logic/mow_load_status_publish_period=" << status_publish_period_
                      << ". Falling back to 0.50 s.");
      status_publish_period_ = 0.50;
      ros::param::set("/mower_logic/mow_load_status_publish_period", status_publish_period_);
    }
  }

  double derateFactor(double value, double start, double end) const {
    if (!std::isfinite(value) || !std::isfinite(start) || !std::isfinite(end)) {
      return 1.0;
    }
    if (end <= start) {
      ROS_WARN_STREAM_THROTTLE(10.0, "Invalid mowing load factor thresholds: end <= start. Derating disabled for this input.");
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
    // Keep this callback lightweight: parameters are loaded once at startup,
    // updated locally by MQTT set callbacks, or reloaded explicitly via renew.
    // Avoiding ROS parameter-server reads here prevents unnecessary rosmaster load.
    last_factor_current_ = derateFactor(msg->mower_esc_current, current_start_, current_end_);
    last_factor_motor_temp_ = derateFactor(msg->mower_motor_temperature, motor_temp_start_, motor_temp_end_);
    last_factor_esc_temp_ = derateFactor(msg->mower_esc_temperature, esc_temp_start_, esc_temp_end_);

    last_computed_factor_ = std::min({last_factor_current_, last_factor_motor_temp_, last_factor_esc_temp_});
    last_effective_factor_ = enabled_ ? last_computed_factor_ : 1.0;

    publishFactorTopics();
    publishStatusJsonIfDue();
  }

  void setEnabledCallback(const std_msgs::Bool::ConstPtr& msg) {
    enabled_ = msg->data;
    ros::param::set("/mower_logic/mow_load_factor_enabled", enabled_);
    last_effective_factor_ = enabled_ ? last_computed_factor_ : 1.0;
    publishFactorTopics();
    publishStatusJson();
    ROS_INFO_STREAM("Mow load factor runtime enabled set to " << (enabled_ ? "true" : "false"));
  }

  void setMinFactorCallback(const std_msgs::Float32::ConstPtr& msg) {
    const double requested = static_cast<double>(msg->data);
    if (!std::isfinite(requested) || requested < minAllowedFactor() || requested > maxAllowedFactor()) {
      ROS_WARN_STREAM("Rejected mowing load min_factor=" << requested
                      << ". Allowed range is " << minAllowedFactor() << " to " << maxAllowedFactor() << ".");
      publishStatusJson();
      return;
    }

    min_factor_ = requested;
    ros::param::set("/mower_logic/mow_load_factor_min", min_factor_);
    // Factors are recalculated on the next mower status. Until then, keep the last sensor-based
    // factors untouched and publish the updated configuration immediately.
    publishStatusJson();
    ROS_INFO_STREAM("Mow load factor runtime min_factor set to " << min_factor_);
  }

  void setCurrentStartCallback(const std_msgs::Float32::ConstPtr& msg) {
    const double requested = static_cast<double>(msg->data);
    if (!std::isfinite(requested) || requested < 0.0 || requested >= current_end_) {
      ROS_WARN_STREAM("Rejected mowing load current_start=" << requested
                      << ". It must be finite, >= 0.0, and lower than current_end=" << current_end_ << ".");
      publishStatusJson();
      return;
    }

    current_start_ = requested;
    ros::param::set("/mower_logic/mow_load_current_start", current_start_);
    // Current-based factor is recalculated on the next mower status message.
    publishStatusJson();
    ROS_INFO_STREAM("Mow load factor runtime current_start set to " << current_start_ << " A");
  }

  void setCurrentEndCallback(const std_msgs::Float32::ConstPtr& msg) {
    const double requested = static_cast<double>(msg->data);
    if (!std::isfinite(requested) || requested <= current_start_) {
      ROS_WARN_STREAM("Rejected mowing load current_end=" << requested
                      << ". It must be finite and higher than current_start=" << current_start_ << ".");
      publishStatusJson();
      return;
    }

    current_end_ = requested;
    ros::param::set("/mower_logic/mow_load_current_end", current_end_);
    // Current-based factor is recalculated on the next mower status message.
    publishStatusJson();
    ROS_INFO_STREAM("Mow load factor runtime current_end set to " << current_end_ << " A");
  }

  void renewCallback(const std_msgs::Empty::ConstPtr&) {
    loadParameters();
    last_effective_factor_ = enabled_ ? last_computed_factor_ : 1.0;
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
  }

  void publishStatusJsonIfDue() {
    const ros::WallTime now = ros::WallTime::now();
    if (last_status_json_publish_wall_time_.isZero() ||
        (now - last_status_json_publish_wall_time_).toSec() >= status_publish_period_) {
      publishStatusJson();
      last_status_json_publish_wall_time_ = now;
    }
  }

  void publishStatusJson() {
    std::ostringstream json;
    json.setf(std::ios::fixed);
    json << std::setprecision(6)
         << "{"
         << "\"enabled\":" << (enabled_ ? "true" : "false")
         << ",\"min_factor\":" << min_factor_
         << ",\"current_start\":" << current_start_
         << ",\"current_end\":" << current_end_
         << ",\"factor_current\":" << last_factor_current_
         << ",\"factor_motor_temp\":" << last_factor_motor_temp_
         << ",\"factor_esc_temp\":" << last_factor_esc_temp_
         << ",\"computed_factor\":" << last_computed_factor_
         << ",\"effective_factor\":" << last_effective_factor_
         << "}";

    std_msgs::String msg;
    msg.data = json.str();
    status_json_pub_.publish(msg);
    last_status_json_publish_wall_time_ = ros::WallTime::now();
  }

  ros::NodeHandle nh_;

  ros::Subscriber status_sub_;
  ros::Subscriber set_enabled_sub_;
  ros::Subscriber set_min_factor_sub_;
  ros::Subscriber set_current_start_sub_;
  ros::Subscriber set_current_end_sub_;
  ros::Subscriber renew_sub_;

  mutable ros::Publisher computed_pub_;
  mutable ros::Publisher effective_pub_;
  mutable ros::Publisher status_json_pub_;

  bool enabled_;
  double min_factor_;
  double current_start_;
  double current_end_;
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
