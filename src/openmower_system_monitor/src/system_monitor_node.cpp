#include <sys/statvfs.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cstdio>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <map>
#include <memory>
#include <sstream>
#include <string>

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

  std::string sensor_origin = xbot_msgs::SensorInfo::ORIGIN_HOST_SYSTEM;

  xbot_msgs::SensorInfo sensor_info;
  ros::Publisher sensor_info_pub;
  ros::Publisher sensor_data_pub;
};

static std::map<std::string, SystemSensorConfig> sensor_configs = {
    {"om_system_wifi_signal_dbm",
     {"System WLAN Signal", "dBm", xbot_msgs::SensorInfo::VALUE_DESCRIPTION_UNKNOWN, xbot_msgs::SensorInfo::TYPE_DOUBLE,
      true, -100.0, -30.0, true, -80.0}},

    {"om_system_wifi_signal_percent",
     {"System WLAN Signal", "%", xbot_msgs::SensorInfo::VALUE_DESCRIPTION_PERCENT, xbot_msgs::SensorInfo::TYPE_DOUBLE,
      true, 0.0, 100.0, true, 25.0}},

    {"om_system_wifi_ssid",
     {"System WLAN Name", "", xbot_msgs::SensorInfo::VALUE_DESCRIPTION_UNKNOWN, xbot_msgs::SensorInfo::TYPE_STRING}},

    {"om_system_wifi_bssid",
     {"System WLAN Access Point", "", xbot_msgs::SensorInfo::VALUE_DESCRIPTION_UNKNOWN,
      xbot_msgs::SensorInfo::TYPE_STRING}},

    {"om_system_wifi_band",
     {"System WLAN Band", "", xbot_msgs::SensorInfo::VALUE_DESCRIPTION_UNKNOWN, xbot_msgs::SensorInfo::TYPE_STRING}},

    {"om_system_wifi_ip",
     {"System WLAN IP", "", xbot_msgs::SensorInfo::VALUE_DESCRIPTION_UNKNOWN, xbot_msgs::SensorInfo::TYPE_STRING}},

    {"om_system_wifi_bitrate",
     {"System WLAN Bitrate", "", xbot_msgs::SensorInfo::VALUE_DESCRIPTION_UNKNOWN, xbot_msgs::SensorInfo::TYPE_STRING}},

    {"om_system_disk_free_percent",
     {"System Free Disk", "%", xbot_msgs::SensorInfo::VALUE_DESCRIPTION_PERCENT, xbot_msgs::SensorInfo::TYPE_DOUBLE,
      true, 0.0, 100.0, true, 10.0}},

    {"om_system_disk_free_gb",
     {"System Free Disk", "GB", xbot_msgs::SensorInfo::VALUE_DESCRIPTION_UNKNOWN, xbot_msgs::SensorInfo::TYPE_DOUBLE,
      true, 0.0, 0.0, true, 9.0}},

    {"om_system_time",
     {"System Time", "", xbot_msgs::SensorInfo::VALUE_DESCRIPTION_UNKNOWN, xbot_msgs::SensorInfo::TYPE_STRING}},

    {"om_system_date",
     {"System Date", "", xbot_msgs::SensorInfo::VALUE_DESCRIPTION_UNKNOWN, xbot_msgs::SensorInfo::TYPE_STRING}},

    {"om_system_last_reboot",
     {"System Last Reboot", "", xbot_msgs::SensorInfo::VALUE_DESCRIPTION_UNKNOWN, xbot_msgs::SensorInfo::TYPE_STRING}},

    {"om_system_uptime_hours",
     {"System Host Uptime", "", xbot_msgs::SensorInfo::VALUE_DESCRIPTION_UNKNOWN, xbot_msgs::SensorInfo::TYPE_STRING}},
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

static std::string trim_copy(const std::string& value);

struct WifiLinkInfo {
  bool connected = false;
  std::string ssid;
  std::string bssid;
  double frequency_mhz = 0.0;
  double signal_dbm = 0.0;
  bool has_signal_dbm = false;
  std::string rx_bitrate_mbps;
  std::string tx_bitrate_mbps;
};

static bool read_wifi_link_info_now(const std::string& interface, WifiLinkInfo& info) {
  const std::string output = exec_command("iw dev " + interface + " link 2>/dev/null");
  if (output.empty() || output.find("Not connected") != std::string::npos) {
    return false;
  }

  std::istringstream lines(output);
  std::string line;
  while (std::getline(lines, line)) {
    const std::string trimmed = trim_copy(line);

    if (trimmed.rfind("Connected to ", 0) == 0) {
      std::istringstream stream(trimmed.substr(std::string("Connected to ").size()));
      stream >> info.bssid;
      info.connected = !info.bssid.empty();
      continue;
    }

    if (trimmed.rfind("SSID:", 0) == 0) {
      info.ssid = trim_copy(trimmed.substr(std::string("SSID:").size()));
      continue;
    }

    if (trimmed.rfind("freq:", 0) == 0) {
      std::istringstream stream(trimmed.substr(std::string("freq:").size()));
      stream >> info.frequency_mhz;
      continue;
    }

    if (trimmed.rfind("signal:", 0) == 0) {
      std::istringstream stream(trimmed.substr(std::string("signal:").size()));
      stream >> info.signal_dbm;
      info.has_signal_dbm = !stream.fail();
      continue;
    }

    if (trimmed.rfind("rx bitrate:", 0) == 0) {
      std::istringstream stream(trimmed.substr(std::string("rx bitrate:").size()));
      stream >> info.rx_bitrate_mbps;
      continue;
    }

    if (trimmed.rfind("tx bitrate:", 0) == 0) {
      std::istringstream stream(trimmed.substr(std::string("tx bitrate:").size()));
      stream >> info.tx_bitrate_mbps;
      continue;
    }
  }

  return info.connected || !info.ssid.empty();
}

static bool read_cached_wifi_link_info(const std::string& interface, WifiLinkInfo& info) {
  static std::string cached_interface;
  static WifiLinkInfo cached_info;
  static ros::Time last_update{0};

  const ros::Time now = ros::Time::now();
  if (cached_interface == interface && !last_update.isZero() && (now - last_update).toSec() < 30.0) {
    info = cached_info;
    return info.connected || !info.ssid.empty();
  }

  WifiLinkInfo fresh_info;
  const bool ok = read_wifi_link_info_now(interface, fresh_info);
  cached_interface = interface;
  cached_info = fresh_info;
  last_update = now;
  info = cached_info;
  return ok;
}

static std::string wifi_band_from_frequency(double frequency_mhz) {
  if (frequency_mhz >= 2400.0 && frequency_mhz < 2500.0) return "2.4 GHz";
  if (frequency_mhz >= 4900.0 && frequency_mhz < 5900.0) return "5 GHz";
  if (frequency_mhz >= 5925.0 && frequency_mhz < 7125.0) return "6 GHz";
  return "unknown";
}

static std::string format_wifi_bitrate(const WifiLinkInfo& info) {
  if (info.rx_bitrate_mbps.empty() && info.tx_bitrate_mbps.empty()) return "unknown";
  const std::string rx = info.rx_bitrate_mbps.empty() ? "?" : info.rx_bitrate_mbps;
  const std::string tx = info.tx_bitrate_mbps.empty() ? "?" : info.tx_bitrate_mbps;
  return "RX " + rx + " / TX " + tx + " MBit/s";
}

static bool read_cached_wifi_ip(const std::string& interface, std::string& ip) {
  static std::string cached_interface;
  static std::string cached_ip;
  static ros::Time last_update{0};

  const ros::Time now = ros::Time::now();
  if (cached_interface == interface && !last_update.isZero() && (now - last_update).toSec() < 60.0) {
    ip = cached_ip;
    return !ip.empty();
  }

  const std::string output = exec_command("ip -4 -o addr show dev " + interface + " 2>/dev/null");
  std::istringstream stream(output);
  std::string token;
  while (stream >> token) {
    if (token.find('/') != std::string::npos) {
      const auto slash = token.find('/');
      ip = token.substr(0, slash);
      break;
    }
  }

  cached_interface = interface;
  cached_ip = ip;
  last_update = now;
  return !ip.empty();
}

static bool read_wifi_signal_dbm_from_proc_wireless(const std::string& interface, double& signal_dbm) {
  // Prefer /proc/net/wireless because it is available on the OpenMower/Raspberry Pi
  // setup even when the iw command is not installed in the runtime/container.
  // Example line:
  // wlan0: 0000   47.  -63.  -256        0      0      0      20      0        0
  // Columns after the interface are: status, link, level, noise, ...
  std::ifstream file("/proc/net/wireless");
  if (!file.is_open()) {
    return false;
  }

  std::string line;
  while (std::getline(file, line)) {
    const auto colon_pos = line.find(':');
    if (colon_pos == std::string::npos) {
      continue;
    }

    std::string current_interface = line.substr(0, colon_pos);
    current_interface.erase(std::remove_if(current_interface.begin(), current_interface.end(),
                                           [](unsigned char c) { return std::isspace(c); }),
                            current_interface.end());

    if (current_interface != interface) {
      continue;
    }

    std::istringstream values(line.substr(colon_pos + 1));
    std::string status;
    double link_quality = 0.0;
    double level = 0.0;
    double noise = 0.0;

    values >> status >> link_quality >> level >> noise;
    if (values.fail()) {
      return false;
    }

    signal_dbm = level;
    return true;
  }

  return false;
}

static bool read_wifi_signal_dbm_from_iw(const std::string& interface, double& signal_dbm) {
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

static bool read_wifi_signal_dbm(const std::string& interface, double& signal_dbm) {
  if (read_wifi_signal_dbm_from_proc_wireless(interface, signal_dbm)) {
    return true;
  }

  // Fallback for systems where /proc/net/wireless does not expose the link data.
  return read_wifi_signal_dbm_from_iw(interface, signal_dbm);
}

static std::string trim_copy(const std::string& value) {
  const auto begin = std::find_if_not(value.begin(), value.end(), [](unsigned char c) { return std::isspace(c); });
  const auto end =
      std::find_if_not(value.rbegin(), value.rend(), [](unsigned char c) { return std::isspace(c); }).base();
  if (begin >= end) {
    return "";
  }
  return std::string(begin, end);
}

static bool read_wifi_ssid_from_iwgetid(const std::string& interface, std::string& ssid) {
  ssid = trim_copy(exec_command("iwgetid -r " + interface + " 2>/dev/null"));
  return !ssid.empty();
}

static bool read_wifi_ssid_from_iw(const std::string& interface, std::string& ssid) {
  const std::string output = exec_command("iw dev " + interface + " link 2>/dev/null");
  const std::string needle = "SSID:";

  const auto pos = output.find(needle);
  if (pos == std::string::npos) {
    return false;
  }

  const auto line_end = output.find('\n', pos);
  ssid = trim_copy(output.substr(pos + needle.size(), line_end - (pos + needle.size())));
  return !ssid.empty();
}

static bool read_wifi_ssid(const std::string& interface, std::string& ssid) {
  if (read_wifi_ssid_from_iwgetid(interface, ssid)) {
    return true;
  }
  return read_wifi_ssid_from_iw(interface, ssid);
}

static double wifi_dbm_to_percent(double dbm) {
  // Simple RSSI mapping for dashboard display:
  // -100 dBm = 0 %, -50 dBm = 100 %, values outside are clamped.
  const double percent = 2.0 * (dbm + 100.0);
  return std::max(0.0, std::min(100.0, percent));
}

static bool read_disk_usage(const std::string& path, double& free_gb, double& free_percent, double& total_gb) {
  struct statvfs stat {};
  if (statvfs(path.c_str(), &stat) != 0 || stat.f_blocks == 0) {
    return false;
  }

  const double block_size = static_cast<double>(stat.f_frsize);
  const double total_bytes = static_cast<double>(stat.f_blocks) * block_size;
  const double free_bytes = static_cast<double>(stat.f_bavail) * block_size;

  total_gb = total_bytes / 1000000000.0;
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

static std::string format_uptime_days_hours() {
  const double uptime_hours = read_uptime_hours();
  const int days = static_cast<int>(uptime_hours / 24.0);
  const double hours = uptime_hours - static_cast<double>(days * 24);

  std::ostringstream out;
  out << days << " d " << std::fixed << std::setprecision(1) << hours << " h";
  return out.str();
}

static std::string format_time(std::time_t timestamp, const char* format) {
  std::tm local_tm{};
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

static void register_system_sensors(ros::NodeHandle& nh, const std::string& disk_path) {
  // Same xbot_monitoring contract as mower_logic/src/monitoring/monitoring.cpp:
  // every sensor publishes a latched SensorInfo topic and a matching data topic.
  // xbot_monitoring discovers /xbot_monitoring/sensors/.*/info and builds
  // sensor_infos/json, sensor_infos/bson and sensors/<sensor_id>/data on MQTT.
  double disk_free_gb_for_info = 0.0;
  double disk_free_percent_for_info = 0.0;
  double disk_total_gb_for_info = 0.0;
  if (read_disk_usage(disk_path, disk_free_gb_for_info, disk_free_percent_for_info, disk_total_gb_for_info)) {
    auto disk_it = sensor_configs.find("om_system_disk_free_gb");
    if (disk_it != sensor_configs.end()) {
      disk_it->second.max_value = disk_total_gb_for_info;
    }
  }

  for (auto& sc_pair : sensor_configs) {
    const std::string& sensor_id = sc_pair.first;
    SystemSensorConfig& config = sc_pair.second;

    config.sensor_info.sensor_id = sensor_id;
    config.sensor_info.sensor_name = config.sensor_name;
    config.sensor_info.sensor_origin =
        config.sensor_origin.empty() ? xbot_msgs::SensorInfo::ORIGIN_HOST_SYSTEM : config.sensor_origin;
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

static void publish_system_sensor_values(const std::string& wifi_interface, const std::string& disk_path) {
  double wifi_dbm = 0.0;
  const bool has_proc_wifi_signal = read_wifi_signal_dbm(wifi_interface, wifi_dbm);
  if (has_proc_wifi_signal) {
    publish_double("om_system_wifi_signal_dbm", wifi_dbm);
    publish_double("om_system_wifi_signal_percent", wifi_dbm_to_percent(wifi_dbm));
  } else {
    ROS_WARN_THROTTLE(60.0, "Could not read WLAN signal from interface '%s'", wifi_interface.c_str());
  }

  WifiLinkInfo wifi_info;
  const bool has_wifi_details = read_cached_wifi_link_info(wifi_interface, wifi_info);
  if (has_wifi_details) {
    if (!wifi_info.ssid.empty()) {
      publish_string("om_system_wifi_ssid", wifi_info.ssid);
    } else {
      publish_string("om_system_wifi_ssid", "unknown");
    }

    publish_string("om_system_wifi_bssid", wifi_info.bssid.empty() ? "unknown" : wifi_info.bssid);
    publish_string("om_system_wifi_band",
                   wifi_info.frequency_mhz > 0.0 ? wifi_band_from_frequency(wifi_info.frequency_mhz) : "unknown");
    publish_string("om_system_wifi_bitrate", format_wifi_bitrate(wifi_info));

    if (!has_proc_wifi_signal && wifi_info.has_signal_dbm) {
      publish_double("om_system_wifi_signal_dbm", wifi_info.signal_dbm);
      publish_double("om_system_wifi_signal_percent", wifi_dbm_to_percent(wifi_info.signal_dbm));
    }
  } else if (has_proc_wifi_signal) {
    publish_string("om_system_wifi_ssid", "unknown");
    publish_string("om_system_wifi_bssid", "unknown");
    publish_string("om_system_wifi_band", "unknown");
    publish_string("om_system_wifi_bitrate", "unknown");
    ROS_WARN_THROTTLE(60.0, "Could not read WLAN details from interface '%s'", wifi_interface.c_str());
  } else {
    publish_string("om_system_wifi_ssid", "not connected");
    publish_string("om_system_wifi_bssid", "not connected");
    publish_string("om_system_wifi_band", "not connected");
    publish_string("om_system_wifi_bitrate", "not connected");
  }

  std::string wifi_ip;
  if (read_cached_wifi_ip(wifi_interface, wifi_ip)) {
    publish_string("om_system_wifi_ip", wifi_ip);
  } else {
    publish_string("om_system_wifi_ip", has_proc_wifi_signal ? "unknown" : "not connected");
  }

  double disk_free_gb = 0.0;
  double disk_free_percent = 0.0;
  double disk_total_gb = 0.0;
  if (read_disk_usage(disk_path, disk_free_gb, disk_free_percent, disk_total_gb)) {
    publish_double("om_system_disk_free_gb", disk_free_gb);
    publish_double("om_system_disk_free_percent", disk_free_percent);
  } else {
    ROS_WARN_THROTTLE(60.0, "Could not read disk usage for path '%s'", disk_path.c_str());
  }

  publish_string("om_system_time", current_time_string());
  publish_string("om_system_date", current_date_string());
  publish_string("om_system_last_reboot", last_reboot_string());
  publish_string("om_system_uptime_hours", format_uptime_days_hours());
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
  pnh.param<double>("publish_rate_hz", publish_rate_hz, 0.2);

  if (publish_rate_hz <= 0.0) {
    ROS_WARN("publish_rate_hz <= 0, using 0.2 Hz");
    publish_rate_hz = 0.2;
  }

  register_system_sensors(nh, disk_path);

  ROS_INFO_STREAM("system_monitor_node started. wifi_interface=" << wifi_interface << ", disk_path=" << disk_path
                                                                 << ", publish_rate_hz=" << publish_rate_hz);

  // Publish a few initial samples shortly after the latched SensorInfo messages.
  // This reduces the dashboard window where a known sensor is shown with its default value.
  for (int i = 0; ros::ok() && i < 3; ++i) {
    publish_system_sensor_values(wifi_interface, disk_path);
    ros::spinOnce();
    ros::Duration(0.5).sleep();
  }

  ros::Rate rate(publish_rate_hz);
  while (ros::ok()) {
    publish_system_sensor_values(wifi_interface, disk_path);
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
