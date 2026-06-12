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
#include <stdexcept>
#include <string>
#include <sys/statvfs.h>

#include "ros/ros.h"
#include "xbot_msgs/SensorDataDouble.h"
#include "xbot_msgs/SensorDataString.h"
#include "xbot_msgs/SensorInfo.h"

struct DoubleSensor {
  xbot_msgs::SensorInfo info;
  ros::Publisher info_pub;
  ros::Publisher data_pub;
};

struct StringSensor {
  xbot_msgs::SensorInfo info;
  ros::Publisher info_pub;
  ros::Publisher data_pub;
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
  // Simple and common RSSI mapping:
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

static xbot_msgs::SensorInfo make_info(const std::string& id,
                                       const std::string& name,
                                       uint8_t type,
                                       uint8_t description,
                                       const std::string& unit,
                                       bool has_min_max = false,
                                       double min_value = 0.0,
                                       double max_value = 0.0,
                                       bool has_critical_low = false,
                                       double critical_low = 0.0) {
  xbot_msgs::SensorInfo info;
  info.sensor_id = id;
  info.sensor_name = name;
  info.value_type = type;
  info.value_description = description;
  info.unit = unit;

  info.has_min_max = has_min_max;
  info.min_value = min_value;
  info.max_value = max_value;

  info.has_critical_low = has_critical_low;
  info.lower_critical_value = critical_low;
  info.has_critical_high = false;
  info.upper_critical_value = 0.0;

  return info;
}

static void publish_double(DoubleSensor& sensor, double value) {
  xbot_msgs::SensorDataDouble data;
  data.stamp = ros::Time::now();
  data.data = value;
  sensor.data_pub.publish(data);
}

static void publish_string(StringSensor& sensor, const std::string& value) {
  xbot_msgs::SensorDataString data;
  data.stamp = ros::Time::now();
  data.data = value;
  sensor.data_pub.publish(data);
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

  std::map<std::string, DoubleSensor> double_sensors;
  std::map<std::string, StringSensor> string_sensors;

  double_sensors["om_system_wifi_signal_dbm"].info = make_info(
      "om_system_wifi_signal_dbm", "WLAN Signal", xbot_msgs::SensorInfo::TYPE_DOUBLE,
      xbot_msgs::SensorInfo::VALUE_DESCRIPTION_UNKNOWN, "dBm", true, -100.0, -30.0, true, -80.0);

  double_sensors["om_system_wifi_signal_percent"].info = make_info(
      "om_system_wifi_signal_percent", "WLAN Signal", xbot_msgs::SensorInfo::TYPE_DOUBLE,
      xbot_msgs::SensorInfo::VALUE_DESCRIPTION_PERCENT, "%", true, 0.0, 100.0, true, 25.0);

  double_sensors["om_system_disk_free_percent"].info = make_info(
      "om_system_disk_free_percent", "Free Disk Space", xbot_msgs::SensorInfo::TYPE_DOUBLE,
      xbot_msgs::SensorInfo::VALUE_DESCRIPTION_PERCENT, "%", true, 0.0, 100.0, true, 10.0);

  double_sensors["om_system_disk_free_gb"].info = make_info(
      "om_system_disk_free_gb", "Free Disk Space", xbot_msgs::SensorInfo::TYPE_DOUBLE,
      xbot_msgs::SensorInfo::VALUE_DESCRIPTION_UNKNOWN, "GB", false);

  double_sensors["om_system_uptime_hours"].info = make_info(
      "om_system_uptime_hours", "Host Uptime", xbot_msgs::SensorInfo::TYPE_DOUBLE,
      xbot_msgs::SensorInfo::VALUE_DESCRIPTION_UNKNOWN, "h", false);

  string_sensors["om_system_time"].info = make_info(
      "om_system_time", "System Time", xbot_msgs::SensorInfo::TYPE_STRING,
      xbot_msgs::SensorInfo::VALUE_DESCRIPTION_UNKNOWN, "");

  string_sensors["om_system_date"].info = make_info(
      "om_system_date", "System Date", xbot_msgs::SensorInfo::TYPE_STRING,
      xbot_msgs::SensorInfo::VALUE_DESCRIPTION_UNKNOWN, "");

  string_sensors["om_system_last_reboot"].info = make_info(
      "om_system_last_reboot", "Last Reboot", xbot_msgs::SensorInfo::TYPE_STRING,
      xbot_msgs::SensorInfo::VALUE_DESCRIPTION_UNKNOWN, "");

  for (auto& item : double_sensors) {
    const std::string base = "xbot_monitoring/sensors/" + item.first;
    item.second.info_pub = nh.advertise<xbot_msgs::SensorInfo>(base + "/info", 1, true);
    item.second.data_pub = nh.advertise<xbot_msgs::SensorDataDouble>(base + "/data", 10);
    item.second.info_pub.publish(item.second.info);
  }

  for (auto& item : string_sensors) {
    const std::string base = "xbot_monitoring/sensors/" + item.first;
    item.second.info_pub = nh.advertise<xbot_msgs::SensorInfo>(base + "/info", 1, true);
    item.second.data_pub = nh.advertise<xbot_msgs::SensorDataString>(base + "/data", 10);
    item.second.info_pub.publish(item.second.info);
  }

  ROS_INFO_STREAM("system_monitor_node started. wifi_interface=" << wifi_interface
                  << ", disk_path=" << disk_path
                  << ", publish_rate_hz=" << publish_rate_hz);

  ros::Rate rate(publish_rate_hz);
  while (ros::ok()) {
    double wifi_dbm = 0.0;
    if (read_wifi_signal_dbm(wifi_interface, wifi_dbm)) {
      publish_double(double_sensors["om_system_wifi_signal_dbm"], wifi_dbm);
      publish_double(double_sensors["om_system_wifi_signal_percent"], wifi_dbm_to_percent(wifi_dbm));
    } else {
      ROS_WARN_THROTTLE(60.0, "Could not read WLAN signal from interface '%s'", wifi_interface.c_str());
    }

    double disk_free_gb = 0.0;
    double disk_free_percent = 0.0;
    if (read_disk_usage(disk_path, disk_free_gb, disk_free_percent)) {
      publish_double(double_sensors["om_system_disk_free_gb"], disk_free_gb);
      publish_double(double_sensors["om_system_disk_free_percent"], disk_free_percent);
    } else {
      ROS_WARN_THROTTLE(60.0, "Could not read disk usage for path '%s'", disk_path.c_str());
    }

    publish_string(string_sensors["om_system_time"], current_time_string());
    publish_string(string_sensors["om_system_date"], current_date_string());
    publish_string(string_sensors["om_system_last_reboot"], last_reboot_string());
    publish_double(double_sensors["om_system_uptime_hours"], read_uptime_hours());

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
