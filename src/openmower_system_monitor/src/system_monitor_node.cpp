#include <algorithm>
#include <array>
#include <chrono>
#include <cstdio>
#include <ctime>
#include <iomanip>
#include <fstream>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <sys/statvfs.h>

#include "ros/ros.h"
#include "xbot_msgs/SensorDataDouble.h"
#include "xbot_msgs/SensorDataString.h"
#include "xbot_msgs/SensorInfo.h"

struct SystemSensorConfig {
  std::string sensor_name;
  std::string unit;
  uint8_t value_description;
  uint8_t value_type;

  bool has_min_max = false;
  double min_value = 0.0;
  double max_value = 0.0;

  bool has_critical_low = false;
  double lower_critical_value = 0.0;

  bool has_critical_high = false;
  double upper_critical_value = 0.0;

  xbot_msgs::SensorInfo sensor_info;
  ros::Publisher sensor_info_pub;
  ros::Publisher sensor_data_pub;
};

static std::map<std::string, SystemSensorConfig> sensor_configs = {
    {"om_system_wifi_signal_dbm",
     {"System WLAN Signal", "dBm", xbot_msgs::SensorInfo::VALUE_DESCRIPTION_UNKNOWN,
      xbot_msgs::SensorInfo::TYPE_DOUBLE, true, -100.0, -30.0, true, -80.0}},

    {"om_system_wifi_signal_percent",
     {"System WLAN Signal", "%", xbot_msgs::SensorInfo::VALUE_DESCRIPTION_PERCENT,
      xbot_msgs::SensorInfo::TYPE_DOUBLE, true, 0.0, 100.0, true, 25.0}},

    {"om_system_disk_free_percent",
     {"System Free Disk", "%", xbot_msgs::SensorInfo::VALUE_DESCRIPTION_PERCENT,
      xbot_msgs::SensorInfo::TYPE_DOUBLE, true, 0.0, 100.0, true, 10.0}},

    {"om_system_disk_free_gb",
     {"System Free Disk", "GB", xbot_msgs::SensorInfo::VALUE_DESCRIPTION_UNKNOWN,
      xbot_msgs::SensorInfo::TYPE_DOUBLE}},

    {"om_system_time",
     {"System Time", "", xbot_msgs::SensorInfo::VALUE_DESCRIPTION_UNKNOWN,
      xbot_msgs::SensorInfo::TYPE_STRING}},

    {"om_system_date",
     {"System Date", "", xbot_msgs::SensorInfo::VALUE_DESCRIPTION_UNKNOWN,
      xbot_msgs::SensorInfo::TYPE_STRING}},

    {"om_system_last_reboot",
     {"System Last Reboot", "", xbot_msgs::SensorInfo::VALUE_DESCRIPTION_UNKNOWN,
      xbot_msgs::SensorInfo::TYPE_STRING}},

    {"om_system_uptime_hours",
     {"System Host Uptime", "h", xbot_msgs::SensorInfo::VALUE_DESCRIPTION_UNKNOWN,
      xbot_msgs::SensorInfo::TYPE_DOUBLE}},
};

static std::string exec_command(const std::string& command) {
  std::array<char, 256> buffer{};
  std::string result;

  std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(command.c_str(), "r"), pclose);
  if (!pipe) {
    return result;
  }

  while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
    result += buffer.data();
  }
  return result;
}

static bool read_wifi_signal_dbm(const std::string& interface, double& signal_dbm) {
  const std::string output = exec_command("iw dev " + interface + " link 2>/dev/null");
  const std::string needle = "signal:";

  const auto pos = output.find(needle);
  if (pos == std::string::npos) {
    return false;
  }

  std::istringstream stream(output.substr(pos + needle.size()));
  stream >> signal_dbm;
  return !stream.fail();
}

static double wifi_dbm_to_percent(double dbm) {
  // Simple RSSI mapping for dashboard display:
  // -100 dBm = 0 %, -50 dBm = 100 %, values outside are clamped.
  const double percent = 2.0 * (dbm + 100.0);
  return std::max(0.0, std::min(100.0, percent));
}

static bool read_disk_usage(const std::string& path, double& free_gb, double& free_percent) {
  struct statvfs stat {};
  if (statvfs(path.c_str(), &stat) != 0 || stat.f_blocks == 0) {
    return false;
  }

  const double block_size = static_cast<double>(stat.f_frsize);
  const double total_bytes = static_cast<double>(stat.f_blocks) * block_size;
  const double free_bytes = static_cast<double>(stat.f_bavail) * block_size;

  free_gb = free_bytes / 1000000000.0;
  free_percent = (free_bytes / total_bytes) * 100.0;
  return true;
}

static double read_uptime_hours() {
  std::ifstream file("/proc/uptime");
  double uptime_seconds = 0.0;
  file >> uptime_seconds;
  return uptime_seconds / 3600.0;
}

static std::string format_time(std::time_t timestamp, const char* format) {
  std::tm local_tm {};
  localtime_r(&timestamp, &local_tm);

  std::ostringstream out;
  out << std::put_time(&local_tm, format);
  return out.str();
}

static std::string current_time_string() {
  return format_time(std::time(nullptr), "%H:%M:%S");
}

static std::string current_date_string() {
  return format_time(std::time(nullptr), "%Y-%m-%d");
}

static std::string last_reboot_string() {
  const double uptime_hours = read_uptime_hours();
  const auto uptime_seconds = static_cast<std::time_t>(uptime_hours * 3600.0);
  const std::time_t boot_time = std::time(nullptr) - uptime_seconds;
  return format_time(boot_time, "%Y-%m-%d %H:%M:%S");
}

static void register_system_sensors(ros::NodeHandle& nh) {
  // Same xbot_monitoring contract as mower_logic/src/monitoring/monitoring.cpp:
  // every sensor publishes a latched SensorInfo topic and a matching data topic.
  // xbot_monitoring discovers /xbot_monitoring/sensors/.*/info and builds
  // sensor_infos/json, sensor_infos/bson and sensors/<sensor_id>/data on MQTT.
  for (auto& sc_pair : sensor_configs) {
    const std::string& sensor_id = sc_pair.first;
    SystemSensorConfig& config = sc_pair.second;

    config.sensor_info.sensor_id = sensor_id;
    config.sensor_info.sensor_name = config.sensor_name;
    config.sensor_info.unit = config.unit;
    config.sensor_info.value_type = config.value_type;
    config.sensor_info.value_description = config.value_description;

    config.sensor_info.has_min_max = config.has_min_max;
    config.sensor_info.min_value = config.min_value;
    config.sensor_info.max_value = config.max_value;

    config.sensor_info.has_critical_low = config.has_critical_low;
    config.sensor_info.lower_critical_value = config.lower_critical_value;

    config.sensor_info.has_critical_high = config.has_critical_high;
    config.sensor_info.upper_critical_value = config.upper_critical_value;

    const std::string base_topic = "xbot_monitoring/sensors/" + sensor_id;
    config.sensor_info_pub = nh.advertise<xbot_msgs::SensorInfo>(base_topic + "/info", 1, true);

    switch (config.value_type) {
      case xbot_msgs::SensorInfo::TYPE_DOUBLE:
        config.sensor_data_pub = nh.advertise<xbot_msgs::SensorDataDouble>(base_topic + "/data", 10);
        break;
      case xbot_msgs::SensorInfo::TYPE_STRING:
        config.sensor_data_pub = nh.advertise<xbot_msgs::SensorDataString>(base_topic + "/data", 10);
        break;
      default:
        ROS_ERROR_STREAM("Invalid system sensor data type for " << sensor_id << ": "
                         << static_cast<int>(config.value_type));
        break;
    }

    config.sensor_info_pub.publish(config.sensor_info);
  }
}

static void publish_double(const std::string& sensor_id, double value) {
  auto it = sensor_configs.find(sensor_id);
  if (it == sensor_configs.end()) return;

  xbot_msgs::SensorDataDouble data;
  data.stamp = ros::Time::now();
  data.data = value;
  it->second.sensor_data_pub.publish(data);
}

static void publish_string(const std::string& sensor_id, const std::string& value) {
  auto it = sensor_configs.find(sensor_id);
  if (it == sensor_configs.end()) return;

  xbot_msgs::SensorDataString data;
  data.stamp = ros::Time::now();
  data.data = value;
  it->second.sensor_data_pub.publish(data);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "system_monitor_node");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::string wifi_interface;
  std::string disk_path;
  double publish_rate_hz;

  pnh.param<std::string>("wifi_interface", wifi_interface, "wlan0");
  pnh.param<std::string>("disk_path", disk_path, "/");
  pnh.param<double>("publish_rate_hz", publish_rate_hz, 0.033333);

  if (publish_rate_hz <= 0.0) {
    ROS_WARN("publish_rate_hz <= 0, using 0.033333 Hz");
    publish_rate_hz = 0.033333;
  }

  register_system_sensors(nh);

  ROS_INFO_STREAM("system_monitor_node started. wifi_interface=" << wifi_interface
                  << ", disk_path=" << disk_path
                  << ", publish_rate_hz=" << publish_rate_hz);

  ros::Rate rate(publish_rate_hz);
  while (ros::ok()) {
    double wifi_dbm = 0.0;
    if (read_wifi_signal_dbm(wifi_interface, wifi_dbm)) {
      publish_double("om_system_wifi_signal_dbm", wifi_dbm);
      publish_double("om_system_wifi_signal_percent", wifi_dbm_to_percent(wifi_dbm));
    } else {
      ROS_WARN_THROTTLE(60.0, "Could not read WLAN signal from interface '%s'", wifi_interface.c_str());
    }

    double disk_free_gb = 0.0;
    double disk_free_percent = 0.0;
    if (read_disk_usage(disk_path, disk_free_gb, disk_free_percent)) {
      publish_double("om_system_disk_free_gb", disk_free_gb);
      publish_double("om_system_disk_free_percent", disk_free_percent);
    } else {
      ROS_WARN_THROTTLE(60.0, "Could not read disk usage for path '%s'", disk_path.c_str());
    }

    publish_string("om_system_time", current_time_string());
    publish_string("om_system_date", current_date_string());
    publish_string("om_system_last_reboot", last_reboot_string());
    publish_double("om_system_uptime_hours", read_uptime_hours());

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
