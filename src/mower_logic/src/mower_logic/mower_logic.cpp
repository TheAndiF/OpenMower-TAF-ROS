// Created by Clemens Elflein on 2/21/22.
// Copyright (c) 2022 Clemens Elflein and OpenMower contributors. All rights reserved.
//
// This file is part of OpenMower.
//
// OpenMower is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
// License as published by the Free Software Foundation, version 3 of the License.
//
// OpenMower is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
// warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License along with OpenMower. If not, see
// <https://www.gnu.org/licenses/>.
//

// #define VERBOSE_DEBUG   1

#include <actionlib/client/simple_action_client.h>
#include <dynamic_reconfigure/server.h>
#include <dynamic_reconfigure/Config.h>
#include <dynamic_reconfigure/ConfigDescription.h>
#include <mower_logic/PowerConfig.h>
#include <mower_msgs/ESCStatus.h>
#include <mower_msgs/Emergency.h>
#include <mower_msgs/Power.h>
#include <tf2/LinearMath/Transform.h>

#include <atomic>
#include <cctype>
#include <cmath>
#include <map>
#include <set>
#include <ios>
#include <mutex>
#include <sstream>
#include <vector>
#include <unistd.h>
#include <sys/wait.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <iomanip>
#include <ctime>
#include <csignal>
#include <chrono>
#include <cerrno>
#include <cstring>

#include "StateSubscriber.h"
#include "behaviors/AreaRecordingBehavior.h"
#include "behaviors/Behavior.h"
#include "behaviors/IdleBehavior.h"
#include "ftc_local_planner/PlannerGetProgress.h"
#include "mbf_msgs/ExePathAction.h"
#include "mbf_msgs/MoveBaseAction.h"
#include "mower_logic/MowerLogicConfig.h"
#include "mower_map/ClearMapSrv.h"
#include "mower_map/ClearNavPointSrv.h"
#include "mower_map/GetDockingPointSrv.h"
#include "mower_map/GetMowingAreaSrv.h"
#include "mower_map/GetMowingAreaByIdSrv.h"
#include "mower_map/GetMowingAreaListSrv.h"
#include "mower_map/SetNavPointSrv.h"
#include "mowing_path_order_optimizer/OptimizePaths.h"
#include "mower_msgs/EmergencyStopSrv.h"
#include "mower_msgs/HighLevelControlSrv.h"
#include "mower_msgs/HighLevelStatus.h"
#include "mower_msgs/MowerControlSrv.h"
#include "mower_msgs/Status.h"
#include "ros/ros.h"
#include "slic3r_coverage_planner/PlanPath.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Float64.h"
#include <open_mower/settings_persistence.h>
#include "utils.h"
#include "xbot_msgs/AbsolutePose.h"
#include "xbot_msgs/RegisterActionsSrv.h"
#include "xbot_positioning/GPSControlSrv.h"
#include "xbot_positioning/SetPoseSrv.h"

ros::ServiceClient pathClient, pathOrderOptimizerClient, mapClient, mapAreaListClient, mapAreaByIdClient,
    dockingPointClient, gpsClient, mowClient, emergencyClient, pathProgressClient, setNavPointClient,
    clearNavPointClient, clearMapClient, positioningClient, actionRegistrationClient;

ros::NodeHandle* n;
ros::NodeHandle* paramNh;

dynamic_reconfigure::Server<mower_logic::MowerLogicConfig>* reconfigServer;
actionlib::SimpleActionClient<mbf_msgs::MoveBaseAction>* mbfClient;
actionlib::SimpleActionClient<mbf_msgs::ExePathAction>* mbfClientExePath;

ros::Publisher cmd_vel_pub, high_level_state_publisher;
mower_logic::MowerLogicConfig last_config;
ll::PowerConfig last_power_config;

class MowerLogicSettingsBridge;
class SatelliteLoggingController;
MowerLogicSettingsBridge* mower_logic_settings_bridge = nullptr;
SatelliteLoggingController* satellite_logging_controller = nullptr;

StateSubscriber<mower_msgs::Emergency> emergency_state_subscriber{"/ll/emergency"};
StateSubscriber<mower_msgs::Status> status_state_subscriber{"/ll/mower_status"};
StateSubscriber<mower_msgs::Power> power_state_subscriber{"/ll/power"};
StateSubscriber<mower_msgs::ESCStatus> left_esc_status_state_subscriber{"/ll/diff_drive/left_esc_status"};
StateSubscriber<mower_msgs::ESCStatus> right_esc_status_state_subscriber{"/ll/diff_drive/right_esc_status"};
StateSubscriber<xbot_msgs::AbsolutePose> pose_state_subscriber{"/xbot_positioning/xb_pose"};
ros::Time joy_vel_time(0.0);

ros::Time last_good_gps(0.0);

std::recursive_mutex mower_logic_mutex;

mower_msgs::HighLevelStatus high_level_status;

std::atomic<bool> mowerAllowed;

Behavior* currentBehavior = &IdleBehavior::INSTANCE;

constexpr double CHARGING_DOCKED_IDLE_DELAY_SECONDS = 30.0;
std::atomic<bool> dockedIdleOnChargingRequested{false};
ros::Time chargingDetectedSince(0.0);
Behavior* chargingDetectedBehavior = nullptr;

void resetDockedIdleChargingTimer() {
  chargingDetectedSince = ros::Time(0.0);
  chargingDetectedBehavior = nullptr;
  dockedIdleOnChargingRequested = false;
}

std::vector<xbot_msgs::ActionInfo> rootActions;
ros::Time last_v_battery_check;
double max_v_battery_seen = 0.0;

ros::Time last_rain_check;
bool rain_detected = true;
ros::Time rain_resume;

/**
 * Some thread safe methods to get a copy of the logic state
 */
ros::Time getLastGoodGPS() {
  std::lock_guard<std::recursive_mutex> lk{mower_logic_mutex};
  return last_good_gps;
}

void setLastGoodGPS(ros::Time time) {
  std::lock_guard<std::recursive_mutex> lk{mower_logic_mutex};
  last_good_gps = time;
}

mower_logic::MowerLogicConfig getConfig() {
  std::lock_guard<std::recursive_mutex> lk{mower_logic_mutex};
  return last_config;
}

ll::PowerConfig getPowerConfig() {
  std::lock_guard<std::recursive_mutex> lk{mower_logic_mutex};
  return last_power_config;
}

void setConfig(mower_logic::MowerLogicConfig c) {
  std::lock_guard<std::recursive_mutex> lk{mower_logic_mutex};
  last_config = c;
  reconfigServer->updateConfig(c);
}

mower_msgs::Status getStatus() {
  return status_state_subscriber.getMessage();
}

mower_msgs::Power getPower() {
  return power_state_subscriber.getMessage();
}

xbot_msgs::AbsolutePose getPose() {
  return pose_state_subscriber.getMessage();
}

void setEmergencyMode(bool emergency);

void registerActions(std::string prefix, const std::vector<xbot_msgs::ActionInfo>& actions) {
  xbot_msgs::RegisterActionsSrv srv;
  srv.request.node_prefix = prefix;
  srv.request.actions = actions;

  ros::Rate retry_delay(1);
  for (int i = 0; i < 10; i++) {
    if (actionRegistrationClient.call(srv)) {
      ROS_INFO_STREAM("successfully registered actions for " << prefix);
      break;
    }
    ROS_ERROR_STREAM("Error registering actions for " << prefix << ". Retrying.");
    retry_delay.sleep();
  }
}

void setRobotPose(geometry_msgs::Pose& pose) {
  // set the robot pose internally as well. othwerise we need to wait for xbot_positioning to send a new one once it has
  // updated the internal pose.
  auto last_pose = pose_state_subscriber.getMessage();
  last_pose.pose.pose = pose;
  pose_state_subscriber.setMessage(last_pose);

  xbot_positioning::SetPoseSrv pose_srv;
  pose_srv.request.robot_pose = pose;

  ros::Rate retry_delay(1);
  bool success = false;
  for (int i = 0; i < 10; i++) {
    if (positioningClient.call(pose_srv)) {
      //            ROS_INFO_STREAM("successfully set pose to " << pose);
      success = true;
      break;
    }
    ROS_ERROR_STREAM("Error setting robot pose to " << pose << ". Retrying.");
    retry_delay.sleep();
  }

  if (!success) {
    ROS_ERROR_STREAM("Error setting robot pose. Going to emergency. THIS SHOULD NEVER HAPPEN");
    setEmergencyMode(true);
  }
}

// Abort the currently running behaviour
void abortExecution() {
  if (currentBehavior != nullptr) {
    currentBehavior->abort();
  }
}

bool setGPS(bool enabled) {
  xbot_positioning::GPSControlSrv gps_srv;
  gps_srv.request.gps_enabled = enabled;

  ros::Rate retry_delay(1);
  bool success = false;
  for (int i = 0; i < 10; i++) {
    if (gpsClient.call(gps_srv)) {
      ROS_INFO_STREAM("successfully set GPS to " << enabled);
      success = true;
      break;
    }
    ROS_ERROR_STREAM("Error setting GPS to " << enabled << ". Retrying.");
    retry_delay.sleep();
  }

  if (!success) {
    ROS_ERROR_STREAM("Error setting GPS. Going to emergency. THIS SHOULD NEVER HAPPEN");
    setEmergencyMode(true);
  }

  return success;
}

/// @brief If the BLADE Motor is not in the requested status (enabled),we call the
///        the mower_service/mow_enabled service to enable/disable. TODO: get feedback about spinup and delay if needed
/// @param enabled
/// @return
bool setMowerEnabled(bool enabled) {
  const auto last_config = getConfig();

  if (!last_config.enable_mower && enabled) {
    // ROS_INFO_STREAM("om_mower_logic: setMowerEnabled() - Mower should be enabled but is hard-disabled in the
    // config.");
    enabled = false;
  }

  // status change ?
  const auto last_status = status_state_subscriber.getMessage();
  if (last_status.mow_enabled != enabled) {
    ros::Time started = ros::Time::now();
    mower_msgs::MowerControlSrv mow_srv;
    mow_srv.request.mow_enabled = enabled;
    // User-selectable direction mode:
    // -1: fixed reverse/left, 0: alternate using the historic timestamp-based behaviour, 1: fixed forward/right.
    if (last_config.mow_motor_direction_mode < 0) {
      mow_srv.request.mow_direction = 0;
    } else if (last_config.mow_motor_direction_mode > 0) {
      mow_srv.request.mow_direction = 1;
    } else {
      // Alternate direction on every real motor start. This avoids a time-based pseudo-random choice.
      static bool next_alternating_direction = false;
      if (enabled) {
        next_alternating_direction = !next_alternating_direction;
      }
      mow_srv.request.mow_direction = next_alternating_direction ? 1 : 0;
    }
    ROS_WARN_STREAM("#### om_mower_logic: setMowerEnabled("
                    << enabled << ", " << static_cast<unsigned>(mow_srv.request.mow_direction)
                    << ", direction_mode=" << last_config.mow_motor_direction_mode << ") call");

    ros::Rate retry_delay(1);
    bool success = false;
    for (int i = 0; i < 10; i++) {
      if (mowClient.call(mow_srv)) {
        ROS_INFO_STREAM("successfully set mower enabled to "
                        << enabled << " (direction " << static_cast<unsigned>(mow_srv.request.mow_direction) << ")");
        success = true;
        break;
      }
      ROS_ERROR_STREAM("Error setting mower enabled to " << enabled << ". Retrying.");
      retry_delay.sleep();
    }

    if (!success) {
      ROS_ERROR_STREAM("Error setting mower enabled. THIS SHOULD NEVER HAPPEN");
    }

    ROS_WARN_STREAM("#### om_mower_logic: setMowerEnabled("
                    << enabled << ", " << static_cast<unsigned>(mow_srv.request.mow_direction)
                    << ") call completed within " << (ros::Time::now() - started).toSec() << "s");
  }

  // TODO: Spinup feedback & delay
  /*    if (enabled) {
          ROS_INFO_STREAM("enabled mower, waiting for it to speed up");

          // TODO timeout and error
          ros::Time started = ros::Time::now();
          while (true) {
              if (status_time > started) {
                  // we have a current status message, wait for mower to speed up
                  bool mower_running = (last_status.speed_mow_status & 0b10);
                  if (mower_running) {
                      ROS_INFO_STREAM("mower motor started");
                      return true;
                  }
              }
              if (ros::Time::now() - started > ros::Duration(25.0)) {
                  // mower was not able to start
                  ROS_ERROR_STREAM("error starting mower motor...");
                  setMowerEnabled(false);
                  return false;
              }
          }
      }*/

  return true;
}

/// @brief Halt all bot movement
void stopMoving() {
  // ROS_INFO_STREAM("om_mower_logic: stopMoving() - stopping bot movement");
  geometry_msgs::Twist stop;
  stop.angular.z = 0;
  stop.linear.x = 0;
  cmd_vel_pub.publish(stop);
}

/// @brief If the BLADE motor is currently enabled, we stop it
void stopBlade() {
  // ROS_INFO_STREAM("om_mower_logic: stopBlade() - stopping blade motor if running");
  setMowerEnabled(false);
  mowerAllowed = false;
  // ROS_INFO_STREAM("om_mower_logic: stopBlade() - finished");
}

/// @brief Stop BLADE motor and any movement
/// @param emergency
void setEmergencyMode(bool emergency) {
  stopBlade();
  stopMoving();
  mower_msgs::EmergencyStopSrv emergencyStop;
  emergencyStop.request.emergency = emergency;

  ros::Rate retry_delay(1);
  bool success = false;
  for (int i = 0; i < 10; i++) {
    if (emergencyClient.call(emergencyStop)) {
      ROS_INFO_STREAM("successfully set emergency enabled to " << emergency);
      success = true;
      break;
    }
    ROS_ERROR_STREAM("Error setting emergency enabled to " << emergency << ". Retrying.");
    retry_delay.sleep();
  }

  if (!success) {
    ROS_ERROR_STREAM("Error setting emergency. THIS SHOULD NEVER HAPPEN");
  }
}


static std::string mower_logic_utc_timestamp_iso8601(const std::chrono::system_clock::time_point& time_point) {
  const std::time_t t = std::chrono::system_clock::to_time_t(time_point);
  std::tm tm{};
  gmtime_r(&t, &tm);
  std::ostringstream out;
  out << std::put_time(&tm, "%Y-%m-%dT%H:%M:%SZ");
  return out.str();
}

static std::string mower_logic_now_compact() {
  const std::time_t t = std::time(nullptr);
  std::tm tm{};
  gmtime_r(&t, &tm);
  std::ostringstream out;
  out << std::put_time(&tm, "%Y%m%d_%H%M%S");
  return out.str();
}

class SatelliteLoggingController {
 public:
  using json = open_mower_settings::json;

  explicit SatelliteLoggingController(ros::NodeHandle& nh) : nh_(nh) {
    status_pub_ = nh_.advertise<std_msgs::String>("/mower_logic/satellite_logging/status_json", 1, true);
    control_sub_ = nh_.subscribe("/mower_logic/satellite_logging/set_control_json", 10,
                                 &SatelliteLoggingController::controlCallback, this);
    renew_sub_ = nh_.subscribe("/mower_logic/satellite_logging/renew", 10,
                               &SatelliteLoggingController::renewCallback, this);
    notifyConfigChanged();
  }

  void notifyConfigChanged() {
    bool disable_setting_after = false;
    {
      std::lock_guard<std::mutex> lk(mutex_);
      const bool enabled = last_config.satellite_logging_enabled;

      if (!enabled) {
        // The legacy dynamic_reconfigure flag is retained for backwards
        // compatibility. It only owns requests that were created through the
        // legacy setting. Explicit gps_state/logging commands remain active
        // until they are stopped or cancelled explicitly.
        if (request_origin_ == "legacy_setting") {
          if (pid_ > 0) stopLocked("legacy_disabled");
          request_active_ = false;
          armed_ = false;
          if (state_ == "armed") state_ = "idle";
        }
      } else if (!last_enabled_) {
        request_active_ = true;
        request_origin_ = "legacy_setting";
        requested_at_ = mower_logic_utc_timestamp_iso8601(std::chrono::system_clock::now());
        startOrArmFromConfigLocked();
        disable_setting_after = state_ == "error";
      }

      last_enabled_ = enabled;
      publishStatusLocked();
    }

    if (disable_setting_after) setEnabledSession(false);
  }

  void updateFromMowerState(bool docked, bool active_work, const std::string& area_id) {
    bool publish_after = false;
    bool disable_setting_after = false;
    {
      std::lock_guard<std::mutex> lk(mutex_);

      if (state_ == "running" && docked && mode_ != "manual" && mode_ != "area_only") {
        stopLocked("docked");
        request_active_ = false;
        publish_after = true;
        disable_setting_after = request_origin_ == "legacy_setting";
      } else if (armed_ && pid_ <= 0 && request_active_) {
        const bool left_dock = has_last_docked_ && last_docked_ && !docked;
        const bool area_match = !target_area_id_.empty() && area_id == target_area_id_;
        const bool should_start =
            (trigger_ == "next_cycle" && mode_ == "from_docking_to_docking" && left_dock) ||
            (trigger_ == "next_cycle" && mode_ == "from_start_to_docking" && !docked && active_work) ||
            (trigger_ == "next_cycle" && mode_ == "until_docking" && !docked && active_work) ||
            (trigger_ == "area_id" && area_match && !docked && active_work);
        if (should_start) {
          startLocked();
          publish_after = true;
          disable_setting_after = state_ == "error";
        }
      }

      last_docked_ = docked;
      has_last_docked_ = true;
      if (publish_after) publishStatusLocked();
    }

    if (disable_setting_after) setEnabledSession(false);
  }

 private:
  static bool triggerSupported(const std::string& trigger) {
    return trigger == "next_cycle" || trigger == "ad_hoc" || trigger == "area_id";
  }

  static bool modeSupported(const std::string& mode) {
    return mode == "from_start_to_docking" || mode == "from_docking_to_docking" ||
           mode == "until_docking" || mode == "manual" || mode == "area_only" || mode == "area_to_docking";
  }

  static void setEnabledSession(bool enabled) {
    mower_logic::MowerLogicConfig cfg = getConfig();
    if (cfg.satellite_logging_enabled == enabled) return;
    cfg.satellite_logging_enabled = enabled;
    setConfig(cfg);
  }

  bool ensureScriptReadyLocked() {
    const std::string& path = last_config.satellite_logging_script_path;
    struct stat st{};
    if (path.empty()) {
      state_ = "error";
      error_ = "satellite logging script path is empty";
      return false;
    }
    if (stat(path.c_str(), &st) != 0) {
      state_ = "error";
      error_ = "satellite logging script not found: " + path;
      return false;
    }
    if (!S_ISREG(st.st_mode)) {
      state_ = "error";
      error_ = "satellite logging script is not a regular file: " + path;
      return false;
    }
    if (access(path.c_str(), X_OK) != 0) {
      const mode_t executable_mode = st.st_mode | S_IXUSR | S_IXGRP | S_IXOTH;
      if (chmod(path.c_str(), executable_mode) != 0) {
        state_ = "error";
        error_ = "could not make satellite logging script executable: " +
                 path + ": " + std::strerror(errno);
        return false;
      }
    }
    if (access(path.c_str(), X_OK) != 0) {
      state_ = "error";
      error_ = "satellite logging script is not executable: " + path;
      return false;
    }
    return true;
  }

  bool applyRequestedStartLocked(const std::string& requested_trigger, const std::string& requested_mode,
                                 const std::string& requested_area) {
    if (!request_active_) {
      state_ = "idle";
      error_ = "logging request is not active";
      return false;
    }
    if (!triggerSupported(requested_trigger)) {
      state_ = "error";
      error_ = "unsupported trigger: " + requested_trigger;
      return false;
    }
    if (!modeSupported(requested_mode)) {
      state_ = "error";
      error_ = "unsupported mode: " + requested_mode;
      return false;
    }
    if (requested_trigger == "area_id" && requested_area.empty()) {
      state_ = "error";
      error_ = "area_id trigger requires satellite_logging_default_area_id or control area_id";
      return false;
    }
    if (!ensureScriptReadyLocked()) return false;

    trigger_ = requested_trigger;
    mode_ = requested_mode;
    target_area_id_ = requested_area;
    started_at_.clear();
    finished_at_.clear();
    session_id_.clear();
    stop_reason_.clear();
    error_.clear();
    files_.clear();
    if (trigger_ == "ad_hoc") {
      startLocked();
    } else {
      state_ = "armed";
      armed_ = true;
    }
    return state_ != "error";
  }

  void startOrArmFromConfigLocked() {
    applyRequestedStartLocked(last_config.satellite_logging_default_trigger,
                              last_config.satellite_logging_default_mode,
                              last_config.satellite_logging_default_area_id);
  }

  static std::vector<std::string> expectedFiles(const std::string& session_id) {
    return {"gps_satellite_list_" + session_id + ".log",
            "gps_position_" + session_id + ".log",
            "gps_accuracy_" + session_id + ".log",
            "gps_rtcm_hz_" + session_id + ".log",
            "satellite_logging_" + session_id + ".meta.json"};
  }

  void startLocked() {
    if (pid_ > 0 || state_ == "running") return;
    if (!request_active_) {
      state_ = "idle";
      armed_ = false;
      error_ = "logging request is not active";
      return;
    }
    if (!ensureScriptReadyLocked()) {
      armed_ = false;
      return;
    }

    session_id_ = mower_logic_now_compact();
    started_at_ = mower_logic_utc_timestamp_iso8601(std::chrono::system_clock::now());
    finished_at_.clear();
    stop_reason_.clear();
    error_.clear();
    files_ = expectedFiles(session_id_);

    const pid_t pid = fork();
    if (pid < 0) {
      state_ = "error";
      error_ = "fork failed";
      return;
    }
    if (pid == 0) {
      setsid();
      execl(last_config.satellite_logging_script_path.c_str(), last_config.satellite_logging_script_path.c_str(),
            "--session-id", session_id_.c_str(),
            "--ram-path", last_config.satellite_logging_ram_path.c_str(),
            "--output-path", last_config.satellite_logging_output_path.c_str(),
            "--container-name", last_config.satellite_logging_container_name.c_str(),
            static_cast<char*>(nullptr));
      _exit(127);
    }
    pid_ = pid;
    state_ = "running";
    armed_ = false;
  }

  void stopLocked(const std::string& reason) {
    if (pid_ <= 0) return;
    kill(-pid_, SIGTERM);
    bool exited = false;
    int status = 0;
    for (int i = 0; i < 50; ++i) {
      const pid_t result = waitpid(pid_, &status, WNOHANG);
      if (result == pid_) { exited = true; break; }
      usleep(100000);
    }
    if (!exited) {
      kill(-pid_, SIGKILL);
      waitpid(pid_, &status, 0);
    }
    pid_ = -1;
    finished_at_ = mower_logic_utc_timestamp_iso8601(std::chrono::system_clock::now());
    state_ = exited ? "finished" : "error";
    stop_reason_ = exited ? reason : "error";
    if (!exited) error_ = "logger did not stop after SIGTERM and was killed";
  }

  void controlCallback(const std_msgs::String::ConstPtr& msg) {
    json payload;
    bool publish_after = true;
    {
      std::lock_guard<std::mutex> lk(mutex_);
      try {
        payload = json::parse(msg->data);
      } catch (const json::exception& e) {
        state_ = "error";
        error_ = std::string("Error decoding JSON: ") + e.what();
        publishStatusLocked();
        return;
      }
      if (!payload.is_object()) {
        state_ = "error";
        error_ = "payload must be a JSON object";
        publishStatusLocked();
        return;
      }

      const std::string command = payload.value("command", std::string("start"));
      if (command == "stop") {
        if (pid_ > 0) {
          stopLocked("manual_stop");
        } else if (armed_) {
          state_ = "idle";
          stop_reason_ = "manual_stop";
        }
        request_active_ = false;
        armed_ = false;
      } else if (command == "cancel") {
        if (pid_ > 0) stopLocked("cancelled");
        request_active_ = false;
        armed_ = false;
        state_ = "idle";
        stop_reason_ = "cancelled";
        error_.clear();
      } else if (command == "start") {
        if (pid_ > 0 || armed_ || request_active_) {
          // Keep the currently confirmed runtime state intact. A duplicate
          // start must never clear the active session id or file list.
          publishStatusLocked();
          return;
        }
        request_active_ = true;
        request_origin_ = "command";
        requested_at_ = mower_logic_utc_timestamp_iso8601(std::chrono::system_clock::now());
        const std::string requested_trigger = payload.value("trigger", last_config.satellite_logging_default_trigger);
        const std::string requested_mode = payload.value("mode", requested_trigger == "ad_hoc" ?
                                                        std::string("until_docking") :
                                                        last_config.satellite_logging_default_mode);
        std::string requested_area = last_config.satellite_logging_default_area_id;
        if (payload.contains("area_id")) {
          if (payload["area_id"].is_string()) {
            requested_area = payload["area_id"].get<std::string>();
          } else if (payload["area_id"].is_number_integer()) {
            requested_area = std::to_string(payload["area_id"].get<long long>());
          }
        }
        applyRequestedStartLocked(requested_trigger, requested_mode, requested_area);
        if (state_ == "error") request_active_ = false;
      } else {
        state_ = "error";
        error_ = "unsupported command: " + command;
      }
    }
    if (publish_after) publishStatus();
  }

  void renewCallback(const std_msgs::Empty::ConstPtr&) { publishStatus(); }

  void publishStatus() {
    std::lock_guard<std::mutex> lk(mutex_);
    publishStatusLocked();
  }

  void publishStatusLocked() {
    if (state_ == "idle" && !armed_ && pid_ <= 0) {
      trigger_ = last_config.satellite_logging_default_trigger;
      mode_ = last_config.satellite_logging_default_mode;
    }
    json payload = json::object();
    payload["enabled"] = last_config.satellite_logging_enabled;
    payload["request_active"] = request_active_;
    payload["request_origin"] = request_origin_.empty() ? json(nullptr) : json(request_origin_);
    payload["state"] = state_;
    payload["trigger"] = trigger_;
    payload["mode"] = mode_;
    payload["target_area_id"] = target_area_id_.empty() ? json(nullptr) : json(target_area_id_);
    payload["armed"] = armed_;
    payload["running"] = pid_ > 0;
    payload["pid"] = pid_ > 0 ? json(pid_) : json(nullptr);
    payload["requested_at"] = requested_at_.empty() ? json(nullptr) : json(requested_at_);
    payload["started_at"] = started_at_.empty() ? json(nullptr) : json(started_at_);
    payload["finished_at"] = finished_at_.empty() ? json(nullptr) : json(finished_at_);
    payload["stop_reason"] = stop_reason_.empty() ? json(nullptr) : json(stop_reason_);
    payload["session_id"] = session_id_.empty() ? json(nullptr) : json(session_id_);
    payload["files"] = files_;
    payload["ram_path"] = last_config.satellite_logging_ram_path;
    payload["output_path"] = last_config.satellite_logging_output_path;
    payload["script_path"] = last_config.satellite_logging_script_path;
    payload["container_name"] = last_config.satellite_logging_container_name.empty() ? json(nullptr) : json(last_config.satellite_logging_container_name);
    payload["error"] = error_.empty() ? json(nullptr) : json(error_);
    std_msgs::String out;
    out.data = payload.dump();
    status_pub_.publish(out);
  }

  ros::NodeHandle& nh_;
  ros::Publisher status_pub_;
  ros::Subscriber control_sub_;
  ros::Subscriber renew_sub_;
  std::mutex mutex_;
  std::string state_ = "idle";
  std::string trigger_ = "next_cycle";
  std::string mode_ = "from_start_to_docking";
  std::string target_area_id_;
  bool request_active_ = false;
  std::string request_origin_;
  bool armed_ = false;
  pid_t pid_ = -1;
  std::string requested_at_;
  std::string session_id_;
  std::string started_at_;
  std::string finished_at_;
  std::string stop_reason_;
  std::vector<std::string> files_;
  std::string error_;
  bool last_docked_ = true;
  bool has_last_docked_ = false;
  bool last_enabled_ = false;
};

void updateUI(const ros::TimerEvent& timer_event) {
  if (currentBehavior == &MowingBehavior::INSTANCE) {
    try {
      high_level_status.current_area = MowingBehavior::INSTANCE.get_current_area();
      high_level_status.current_area_id = MowingBehavior::INSTANCE.get_current_area_id();
    } catch (const std::runtime_error& re) {
      // specific handling for runtime_error
      ROS_ERROR_STREAM("Error getting current area: " << re.what());
    }
    try {
      high_level_status.current_path = MowingBehavior::INSTANCE.get_current_path();
    } catch (const std::runtime_error& re) {
      ROS_ERROR_STREAM("Error getting current path: " << re.what());
    }
    try {
      high_level_status.current_path_index = MowingBehavior::INSTANCE.get_current_path_index();
    } catch (const std::runtime_error& re) {
      ROS_ERROR_STREAM("Error getting current path index: " << re.what());
    }
  } else {
    high_level_status.current_area = -1;
    high_level_status.current_area_id = "";
    high_level_status.current_path = -1;
    high_level_status.current_path_index = -1;
  }

  try {
    high_level_status.checkpoint_area_id = MowingBehavior::INSTANCE.get_checkpoint_area_id();
  } catch (const std::runtime_error& re) {
    ROS_ERROR_STREAM("Error getting checkpoint area id: " << re.what());
    high_level_status.checkpoint_area_id = "";
  }

  if (currentBehavior) {
    high_level_status.state_name = currentBehavior->state_name();
    high_level_status.state = (currentBehavior->get_state() & 0b11111) |
                              (currentBehavior->get_sub_state() << mower_msgs::HighLevelStatus::SUBSTATE_SHIFT);
    high_level_status.sub_state_name = currentBehavior->sub_state_name();
  } else {
    high_level_status.state_name = "NULL";
    high_level_status.sub_state_name = "";
    high_level_status.state = mower_msgs::HighLevelStatus::HIGH_LEVEL_STATE_NULL;
  }

  if (satellite_logging_controller != nullptr) {
    const bool docked = high_level_status.is_charging || currentBehavior == &IdleBehavior::DOCKED_INSTANCE;
    const bool active_work = currentBehavior == &MowingBehavior::INSTANCE;
    satellite_logging_controller->updateFromMowerState(docked, active_work, high_level_status.current_area_id);
  }

  high_level_state_publisher.publish(high_level_status);
}

bool isGpsGood() {
  std::lock_guard<std::recursive_mutex> lk{mower_logic_mutex};
  // GPS is good if orientation is valid, we have low accuracy and we have a recent GPS update.
  // TODO: think about the "recent gps flag" since it only looks at the time. E.g. if we were standing still this would
  // still pause even if no GPS updates are needed during standstill.
  const auto last_pose = pose_state_subscriber.getMessage();
  return last_pose.orientation_valid && last_pose.position_accuracy < last_config.max_position_accuracy &&
         (last_pose.flags & xbot_msgs::AbsolutePose::FLAG_SENSOR_FUSION_RECENT_ABSOLUTE_POSE);
}

/// @brief Called every 0.5s, used to control BLADE motor via mower_enabled variable and stop any movement in case of
/// /odom and /mower/status outages
/// @param timer_event
void checkSafety(const ros::TimerEvent& timer_event) {
  const auto last_status = status_state_subscriber.getMessage();
  const auto last_emergency = emergency_state_subscriber.getMessage();
  const auto last_config = getConfig();
  const auto last_pose = pose_state_subscriber.getMessage();
  const auto last_power = power_state_subscriber.getMessage();
  const auto last_left_esc_state = left_esc_status_state_subscriber.getMessage();
  const auto last_left_esc_state_time = left_esc_status_state_subscriber.getMessageTime();
  const auto last_right_esc_state = right_esc_status_state_subscriber.getMessage();
  const auto last_right_esc_state_time = right_esc_status_state_subscriber.getMessageTime();
  const auto pose_time = pose_state_subscriber.getMessageTime();
  const auto status_time = status_state_subscriber.getMessageTime();
  const auto power_time = power_state_subscriber.getMessageTime();
  const auto last_good_gps = getLastGoodGPS();

  high_level_status.emergency = last_emergency.latched_emergency;
  high_level_status.is_charging = last_power.charge_voltage_chg > 10.0 || last_power.charge_voltage_adc > 10.0;

  // Initialize to true, if after all checks it is still true then mower should be enabled.
  mowerAllowed = true;

  // send to idle if emergency and we're not recording
  if (currentBehavior != nullptr) {
    if (last_emergency.latched_emergency) {
      currentBehavior->requestPause(pauseType::PAUSE_EMERGENCY);
    } else {
      currentBehavior->requestContinue(pauseType::PAUSE_EMERGENCY);
    }
  }

  if (!high_level_status.is_charging || currentBehavior == nullptr) {
    resetDockedIdleChargingTimer();
  } else {
    const ros::Time now = ros::Time::now();
    if (chargingDetectedSince.isZero() || chargingDetectedBehavior != currentBehavior) {
      chargingDetectedSince = now;
      chargingDetectedBehavior = currentBehavior;
      dockedIdleOnChargingRequested = false;
    }

    if (currentBehavior != &IdleBehavior::DOCKED_INSTANCE &&
        now - chargingDetectedSince >= ros::Duration(CHARGING_DOCKED_IDLE_DELAY_SECONDS)) {
      ROS_INFO_STREAM_THROTTLE(5, "Charging detected for " << CHARGING_DOCKED_IDLE_DELAY_SECONDS
                                                            << "s, requesting docked idle state.");
      dockedIdleOnChargingRequested = true;
      currentBehavior->abort();
    }
  }

  // TODO: Have a single point where we check for this timeout instead of twice (here and in the behavior)
  // check if odometry is current. If not, the GPS was bad so we stop moving.
  // Note that the mowing behavior will pause as well by itself.
  if (ros::Time::now() - pose_time > ros::Duration(1.0)) {
    stopBlade();
    stopMoving();
    ROS_WARN_STREAM_THROTTLE(
        5, "om_mower_logic: EMERGENCY pose values stopped. dt was: " << (ros::Time::now() - pose_time));
    return;
  }

  // check if status is current. if not, we have a problem since it contains wheel ticks and so on.
  // Since these should never drop out, we enter emergency instead of "only" stopping
  if (ros::Time::now() - status_time > ros::Duration(3) || ros::Time::now() - power_time > ros::Duration(3)) {
    setEmergencyMode(true);
    ROS_WARN_STREAM_THROTTLE(
        5, "om_mower_logic: EMERGENCY /mower/status values stopped. dt was: " << (ros::Time::now() - status_time));
    return;
  }

  // If the motor controllers error, we enter emergency mode in the hope to save them. They should not error.
  if (last_left_esc_state.status <= mower_msgs::ESCStatus::ESC_STATUS_ERROR ||
      last_right_esc_state.status <= mower_msgs::ESCStatus::ESC_STATUS_ERROR) {
    setEmergencyMode(true);
    ROS_ERROR_STREAM("EMERGENCY: at least one motor control errored. errors left: "
                     << (last_left_esc_state.status) << ", status right: " << last_right_esc_state.status);
    return;
  }

  // We need orientation and a positional accuracy less than configured
  bool gpsGoodNow = isGpsGood();
  if (gpsGoodNow || last_config.ignore_gps_errors) {
    setLastGoodGPS(ros::Time::now());
    high_level_status.gps_quality_percent =
        1.0 - fmin(1.0, last_pose.position_accuracy / last_config.max_position_accuracy);
    ROS_INFO_STREAM_THROTTLE(10, "GPS quality: " << high_level_status.gps_quality_percent);
  } else {
    // GPS = bad, set quality to 0
    high_level_status.gps_quality_percent = 0;
    if (last_pose.orientation_valid) {
      // set this if we don't even have an orientation
      high_level_status.gps_quality_percent = -1;
    }
    ROS_WARN_STREAM_THROTTLE(1, "Low quality GPS");
  }

  bool gpsTimeout = ros::Time::now() - last_good_gps > ros::Duration(last_config.gps_timeout);

  if (gpsTimeout) {
    // GPS = bad, set quality to 0
    high_level_status.gps_quality_percent = 0;
    ROS_WARN_STREAM_THROTTLE(1, "GPS timeout");
  }

  if (currentBehavior != nullptr && currentBehavior->needs_gps()) {
    currentBehavior->setGoodGPS(!gpsTimeout);
    // Stop the mower
    if (gpsTimeout) {
      stopBlade();
      stopMoving();
      return;
    }
  }

  if (currentBehavior != nullptr && currentBehavior->redirect_joystick()) {
    if (ros::Time::now() - joy_vel_time > ros::Duration(10)) {
      stopMoving();  // To avoid cmd_vel receive timeout in mower_comms
    }
  }

  // enable the mower (if not aleady) if mowerAllowed is still true after checks and bahavior agrees
  setMowerEnabled(currentBehavior != nullptr && mowerAllowed && currentBehavior->mower_enabled());

  // Get the best available battery voltage using fallback chain: ADC -> BMS -> CHG
  const float last_battery_v = utils::GetFirstValid(
      {last_power.battery_voltage_adc, last_power.battery_voltage_bms, last_power.battery_voltage_chg});

  double battery_percent = (last_battery_v - last_power_config.battery_empty_voltage) /
                           (last_power_config.battery_full_voltage - last_power_config.battery_empty_voltage);
  battery_percent = std::max(battery_percent, 0.0);  // Clamp from below to 0.0
  battery_percent = std::min(battery_percent, 1.0);  // Clamp from above to 1.0
  high_level_status.battery_percent = battery_percent;

  // we are in non emergency, check if we should pause. This could be empty battery, rain or hot mower motor etc.
  bool dockingNeeded = false;

  std::stringstream dockingReason("Docking: ", std::ios_base::ate | std::ios_base::in | std::ios_base::out);

  if (last_config.manual_pause_mowing) {
    dockingReason << "Manual pause";
    dockingNeeded = true;
  }

  // Dock if below critical voltage to avoid BMS undervoltage protection
  if (!dockingNeeded && (last_battery_v < last_power_config.battery_critical_voltage)) {
    dockingReason << "Battery voltage min critical: " << last_battery_v;
    dockingNeeded = true;
  }

  // Otherwise take the max battery voltage over 20s to ignore droop during short current spikes
  max_v_battery_seen = std::max<double>(max_v_battery_seen, last_battery_v);
  if (ros::Time::now() - last_v_battery_check > ros::Duration(20.0)) {
    if (!dockingNeeded && (max_v_battery_seen < last_power_config.battery_empty_voltage)) {
      dockingReason << "Battery average voltage low: " << max_v_battery_seen;
      dockingNeeded = true;
    }
    max_v_battery_seen = 0.0;
    last_v_battery_check = ros::Time::now();
  }

  if (!dockingNeeded && last_status.mower_motor_temperature >= last_config.motor_hot_temperature) {
    dockingReason << "Mow motor over temp: " << last_status.mower_motor_temperature;
    dockingNeeded = true;
  }

  // Rain detected is initialized to true and flips to false if rain is not detected
  // continuously for rain_check_seconds. This is to avoid false positives due to noise
  rain_detected = rain_detected && last_status.rain_detected;
  if (last_config.rain_check_seconds == 0 ||
      ros::Time::now() - last_rain_check > ros::Duration(last_config.rain_check_seconds)) {
    if (rain_detected) {
      // Reset rain resume time
      rain_resume =
          ros::Time::now() + ros::Duration(last_config.rain_check_seconds + last_config.rain_delay_minutes * 60);
    }
    if (!dockingNeeded && rain_detected && last_config.rain_mode) {
      dockingReason << "Rain detected";
      dockingNeeded = true;
      if (last_config.rain_mode == 3) {
        auto new_config = getConfig();
        new_config.manual_pause_mowing = true;
        setConfig(new_config);
      }
    }
    last_rain_check = ros::Time::now();
    rain_detected = true;
  }

  if (dockingNeeded && currentBehavior != &DockingBehavior::INSTANCE &&
      currentBehavior != &UndockingBehavior::RETRY_INSTANCE && currentBehavior != &IdleBehavior::INSTANCE &&
      currentBehavior != &IdleBehavior::DOCKED_INSTANCE) {
    ROS_INFO_STREAM(dockingReason.rdbuf());
    abortExecution();
  }
}


class MowerLogicSettingsBridge {
 public:
  using json = open_mower_settings::json;

  MowerLogicSettingsBridge(ros::NodeHandle& nh, ros::NodeHandle& private_nh) : nh_(nh) {
    private_nh.param("/settings/persistent_file", settings_persistent_path_,
                     std::string("/data/ros/settings_persistent.json"));
    status_pub_ = nh_.advertise<std_msgs::String>("/mower_logic/settings/status_json", 1, true);
    validation_pub_ = nh_.advertise<std_msgs::String>("/mower_logic/settings/validation_json", 1, true);
    ftc_speed_fast_threshold_pub_ = nh_.advertise<std_msgs::Float64>("/ftc_local_planner/settings/set/speed_fast_threshold", 10);
    ftc_persistent_speed_fast_threshold_pub_ = nh_.advertise<std_msgs::Float64>("/ftc_local_planner/settings/set_persistent/speed_fast_threshold", 10);
    session_sub_ = nh_.subscribe("/mower_logic/settings/set_session_json", 10,
                                 &MowerLogicSettingsBridge::sessionSetCallback, this);
    persistent_sub_ = nh_.subscribe("/mower_logic/settings/set_persistent_json", 10,
                                    &MowerLogicSettingsBridge::persistentSetCallback, this);
    renew_sub_ = nh_.subscribe("/mower_logic/settings/renew", 10,
                               &MowerLogicSettingsBridge::renewCallback, this);
    initializeMetadataAndPersistentValues();
    publishStatus();
  }

  void notifyConfigChanged() { publishStatus(); }

 private:
  enum class ValueType { kBoolean, kInteger, kNumber, kString, kUnknown };

  static constexpr const char* kNamespace = "mower_logic";

  struct ParamMeta {
    std::string name;
    ValueType type = ValueType::kUnknown;
    std::string type_name;
    std::string description;
    json default_value;
    json min_value;
    json max_value;
    bool has_min = false;
    bool has_max = false;
    bool external_ftc_planner = false;
    int order = 0;
  };

  static ValueType valueTypeForDescription(const std::string& type) {
    if (type == "bool") return ValueType::kBoolean;
    if (type == "int") return ValueType::kInteger;
    if (type == "double") return ValueType::kNumber;
    if (type == "str") return ValueType::kString;
    return ValueType::kUnknown;
  }

  static std::string jsonTypeName(ValueType type) {
    switch (type) {
      case ValueType::kBoolean: return "boolean";
      case ValueType::kInteger: return "integer";
      case ValueType::kNumber: return "number";
      case ValueType::kString: return "string";
      default: return "unknown";
    }
  }

  static bool getConfigValue(const dynamic_reconfigure::Config& config, const std::string& key,
                             ValueType type, json& out) {
    switch (type) {
      case ValueType::kBoolean:
        // dynamic_reconfigure::BoolParameter::value is represented as an integer-like
        // ROS message field in C++. Convert explicitly to bool before storing it in
        // JSON. Without this cast, nlohmann::json stores 0/1 as a number and later
        // get<bool>() aborts with type_error.302 during settings initialization.
        for (const auto& item : config.bools) if (item.name == key) { out = static_cast<bool>(item.value); return true; }
        return false;
      case ValueType::kInteger:
        for (const auto& item : config.ints) if (item.name == key) { out = item.value; return true; }
        return false;
      case ValueType::kNumber:
        for (const auto& item : config.doubles) if (item.name == key) { out = item.value; return true; }
        return false;
      case ValueType::kString:
        for (const auto& item : config.strs) if (item.name == key) { out = item.value; return true; }
        return false;
      default:
        return false;
    }
  }

  static bool setConfigValue(dynamic_reconfigure::Config& config, const std::string& key,
                             ValueType type, const json& value) {
    switch (type) {
      case ValueType::kBoolean:
        for (auto& item : config.bools) if (item.name == key) { item.value = value.get<bool>(); return true; }
        return false;
      case ValueType::kInteger:
        for (auto& item : config.ints) if (item.name == key) { item.value = value.get<int>(); return true; }
        return false;
      case ValueType::kNumber:
        for (auto& item : config.doubles) if (item.name == key) { item.value = value.get<double>(); return true; }
        return false;
      case ValueType::kString:
        for (auto& item : config.strs) if (item.name == key) { item.value = value.get<std::string>(); return true; }
        return false;
      default:
        return false;
    }
  }

  static bool valuesDiffer(const json& active, const json& persistent) {
    if (active.is_number() && persistent.is_number()) {
      return std::fabs(active.get<double>() - persistent.get<double>()) > 1e-9;
    }
    return active != persistent;
  }

  bool isTypeValid(const ParamMeta& meta, const json& value) const {
    switch (meta.type) {
      case ValueType::kBoolean: return value.is_boolean();
      case ValueType::kInteger: return value.is_number_integer();
      case ValueType::kNumber: return value.is_number();
      case ValueType::kString: return value.is_string();
      default: return false;
    }
  }

  bool isRangeValid(const ParamMeta& meta, const json& value) const {
    if (meta.type != ValueType::kInteger && meta.type != ValueType::kNumber) return true;
    const double requested = value.get<double>();
    if (!std::isfinite(requested)) return false;
    if (meta.has_min && requested < meta.min_value.get<double>()) return false;
    if (meta.has_max && requested > meta.max_value.get<double>()) return false;
    return true;
  }

  static std::string trimString(const std::string& value) {
    std::size_t begin = 0;
    while (begin < value.size() && std::isspace(static_cast<unsigned char>(value[begin]))) ++begin;
    std::size_t end = value.size();
    while (end > begin && std::isspace(static_cast<unsigned char>(value[end - 1]))) --end;
    return value.substr(begin, end - begin);
  }

  static bool validateGroupName(const json& value, std::string& group, std::string& reason) {
    if (!value.is_string()) {
      reason = "group must be a string";
      return false;
    }
    group = trimString(value.get<std::string>());
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

  std::string labelForKey(const std::string& key) const {
    if (key == "path_order_optimizer_processing_mode") return "POO Abarbeitungsmodus";
    if (key == "path_order_optimizer_outline_entry_mode") return "POO Outline-Einstiegsmodus";
    if (key == "obstacle_outline_count") return "Innere Outline-Anzahl";
    if (key == "obstacle_outline_overlap_count") return "Innere Outline-Überlappung";
    if (key == "outline_simplify_per_loop") return "Outline-Glättung pro Linie";
    if (key == "outline_simplify_max_tolerance") return "Maximale Outline-Glättung";
    if (key == "outline_simplify_safety_factor") return "Outline-Glättung Sicherheitsfaktor";
    if (key == "outline_simplify_min_distance_factor") return "Minimaler Outline-Abstandsfaktor";
    if (key == "outline_simplify_affects_next_offset") return "Progressive Outline-Glättung";
    if (key == "mow_motor_direction_mode") return "Mähmotor-Drehrichtungsmodus";
    if (key == "satellite_logging_enabled") return "Satelliten-Logging aktiv";
    if (key == "satellite_logging_default_trigger") return "Satelliten-Logging Standard-Startart";
    if (key == "satellite_logging_default_mode") return "Satelliten-Logging Standard-Modus";
    if (key == "satellite_logging_default_area_id") return "Satelliten-Logging Ziel-Flächen-ID";
    if (key == "satellite_logging_script_path") return "Satelliten-Logging Skriptpfad";
    if (key == "satellite_logging_ram_path") return "Satelliten-Logging RAM-Pfad";
    if (key == "satellite_logging_output_path") return "Satelliten-Logging Zielpfad";
    if (key == "satellite_logging_container_name") return "Satelliten-Logging Containername";
    if (key == "speed_fast_threshold") return "Schnell ab Geradeausdistanz";
    return key;
  }

  std::string unitForKey(const std::string& key) const {
    if (key.find("temperature") != std::string::npos || key.find("_temp_") != std::string::npos) return "°C";
    if (key.find("distance") != std::string::npos || key == "tool_width" || key == "max_position_accuracy" ||
        key == "outline_simplify_per_loop" || key == "outline_simplify_max_tolerance" ||
        key == "speed_fast_threshold") return "m";
    if (key.find("angle") != std::string::npos || key == "shutdown_esc_max_pitch") return "°";
    if (key.find("minutes") != std::string::npos) return "min";
    if (key.find("seconds") != std::string::npos || key.find("_time") != std::string::npos || key.find("period") != std::string::npos) return "s";
    if (key.find("current") != std::string::npos) return "A";
    return "";
  }

  std::string groupForKey(const std::string& key) const {
    if (key.find("path_order_optimizer_") == 0) return "path_order_optimizer";
    if (key.find("outline_simplify_") == 0 || key == "obstacle_outline_count" ||
        key == "obstacle_outline_overlap_count") return "outline_simplification";
    if (key.find("satellite_logging_") == 0) return "system";
    return kNamespace;
  }

  bool expertForKey(const std::string& key) const {
    if (key.find("path_order_optimizer_") == 0 && key != "path_order_optimizer_enabled") return true;
    if (key.find("outline_simplify_") == 0 || key == "obstacle_outline_count" ||
        key == "obstacle_outline_overlap_count") return true;
    if (key == "satellite_logging_script_path" || key == "satellite_logging_ram_path" ||
        key == "satellite_logging_output_path" || key == "satellite_logging_container_name") return true;
    return false;
  }

  json seedEntriesFromDynamicReconfigure() {
    json seed = json::object();
    const auto& description = mower_logic::MowerLogicConfig::__getDescriptionMessage__();
    const auto& defaults = mower_logic::MowerLogicConfig::__getDefault__();
    const auto& mins = mower_logic::MowerLogicConfig::__getMin__();
    const auto& maxs = mower_logic::MowerLogicConfig::__getMax__();
    dynamic_reconfigure::Config default_msg, min_msg, max_msg;
    defaults.__toMessage__(default_msg);
    mins.__toMessage__(min_msg);
    maxs.__toMessage__(max_msg);

    int order = 10;
    for (const auto& group : description.groups) {
      for (const auto& param : group.parameters) {
        ParamMeta meta;
        meta.name = param.name;
        meta.type = valueTypeForDescription(param.type);
        meta.type_name = jsonTypeName(meta.type);
        meta.description = param.description;
        meta.order = order;
        order += 10;
        if (meta.type == ValueType::kUnknown) continue;
        if (!getConfigValue(default_msg, meta.name, meta.type, meta.default_value)) continue;
        meta.has_min = getConfigValue(min_msg, meta.name, meta.type, meta.min_value);
        meta.has_max = getConfigValue(max_msg, meta.name, meta.type, meta.max_value);
        metadata_[meta.name] = meta;

        json entry = json::object();
        entry["label"] = labelForKey(meta.name);
        entry["description"] = meta.description;
        entry["group"] = groupForKey(meta.name);
        entry["expert"] = expertForKey(meta.name);
        entry["order"] = meta.order;
        entry["session_apply_supported"] = true;
        entry["restart_required"] = false;
        entry["default"] = meta.default_value;
        entry["persistent"] = meta.default_value;
        entry["unit"] = unitForKey(meta.name);
        entry["type"] = meta.type_name;
        if ((meta.type == ValueType::kInteger || meta.type == ValueType::kNumber) && meta.has_min) entry["min"] = meta.min_value;
        if ((meta.type == ValueType::kInteger || meta.type == ValueType::kNumber) && meta.has_max) entry["max"] = meta.max_value;
        seed[meta.name] = entry;
      }
    }

    ParamMeta speed_threshold_meta;
    speed_threshold_meta.name = "speed_fast_threshold";
    speed_threshold_meta.type = ValueType::kNumber;
    speed_threshold_meta.type_name = jsonTypeName(speed_threshold_meta.type);
    speed_threshold_meta.description = "Ab dieser berechneten Geradeausdistanz vor dem Roboter wird speed_fast verwendet.";
    speed_threshold_meta.default_value = 1.5;
    speed_threshold_meta.min_value = 0.0;
    speed_threshold_meta.max_value = 5.0;
    speed_threshold_meta.has_min = true;
    speed_threshold_meta.has_max = true;
    speed_threshold_meta.external_ftc_planner = true;
    speed_threshold_meta.order = order;
    metadata_[speed_threshold_meta.name] = speed_threshold_meta;

    json speed_threshold_entry = json::object();
    speed_threshold_entry["label"] = labelForKey(speed_threshold_meta.name);
    speed_threshold_entry["description"] = speed_threshold_meta.description;
    speed_threshold_entry["group"] = groupForKey(speed_threshold_meta.name);
    speed_threshold_entry["expert"] = expertForKey(speed_threshold_meta.name);
    speed_threshold_entry["order"] = speed_threshold_meta.order;
    speed_threshold_entry["session_apply_supported"] = true;
    speed_threshold_entry["restart_required"] = false;
    speed_threshold_entry["default"] = speed_threshold_meta.default_value;
    speed_threshold_entry["persistent"] = speed_threshold_meta.default_value;
    speed_threshold_entry["unit"] = unitForKey(speed_threshold_meta.name);
    speed_threshold_entry["type"] = speed_threshold_meta.type_name;
    speed_threshold_entry["min"] = speed_threshold_meta.min_value;
    speed_threshold_entry["max"] = speed_threshold_meta.max_value;
    seed[speed_threshold_meta.name] = speed_threshold_entry;
    return seed;
  }

  void initializeMetadataAndPersistentValues() {
    open_mower_settings::migratePersistentFieldsAndRemoveNamespace(
        settings_persistent_path_, "mow_load_factor", kNamespace,
        {{"enabled", "mow_load_factor_enabled"},
         {"min_factor", "mow_load_factor_min"},
         {"current_start", "mow_load_current_start"},
         {"current_end", "mow_load_current_end"}});
    settings_entries_ = open_mower_settings::mergeNamespaceWithSeed(settings_persistent_path_, kNamespace,
                                                                     seedEntriesFromDynamicReconfigure());
    mower_logic::MowerLogicConfig target = getConfig();
    dynamic_reconfigure::Config target_msg;
    target.__toMessage__(target_msg);
    bool changed = false;
    for (const auto& pair : metadata_) {
      const std::string& key = pair.first;
      const ParamMeta& meta = pair.second;
      json persistent = meta.default_value;
      if (settings_entries_.contains(key) && settings_entries_[key].is_object() &&
          settings_entries_[key].contains("persistent") && isTypeValid(meta, settings_entries_[key]["persistent"]) &&
          isRangeValid(meta, settings_entries_[key]["persistent"])) {
        persistent = settings_entries_[key]["persistent"];
      } else {
        settings_entries_[key]["persistent"] = persistent;
        open_mower_settings::updateEntryField(settings_persistent_path_, kNamespace, key, "persistent", persistent);
      }
      if (!meta.external_ftc_planner && setConfigValue(target_msg, key, meta.type, persistent)) changed = true;
      syncParamTree(key, meta, persistent, persistent);
    }
    if (changed) {
      mower_logic::MowerLogicConfig applied = target;
      if (applied.__fromMessage__(target_msg)) {
        setConfig(applied);
      } else {
        ROS_WARN_STREAM("Could not apply persistent mower_logic settings from " << settings_persistent_path_);
      }
    }
  }

  void syncParamTree(const std::string& key, const ParamMeta& meta, const json& persistent, const json& active) const {
    const std::string base = std::string("/settings/") + kNamespace;
    const json& def = meta.default_value;
    if (meta.type == ValueType::kBoolean) {
      ros::param::set(base + "/default/" + key, def.get<bool>());
      ros::param::set(base + "/persistent/" + key, persistent.get<bool>());
      ros::param::set(base + "/active/" + key, active.get<bool>());
    } else if (meta.type == ValueType::kInteger) {
      ros::param::set(base + "/default/" + key, def.get<int>());
      ros::param::set(base + "/persistent/" + key, persistent.get<int>());
      ros::param::set(base + "/active/" + key, active.get<int>());
    } else if (meta.type == ValueType::kNumber) {
      ros::param::set(base + "/default/" + key, def.get<double>());
      ros::param::set(base + "/persistent/" + key, persistent.get<double>());
      ros::param::set(base + "/active/" + key, active.get<double>());
    } else if (meta.type == ValueType::kString) {
      ros::param::set(base + "/default/" + key, def.get<std::string>());
      ros::param::set(base + "/persistent/" + key, persistent.get<std::string>());
      ros::param::set(base + "/active/" + key, active.get<std::string>());
    }
  }

  json activeValue(const std::string& key, const ParamMeta& meta) const {
    if (meta.external_ftc_planner) {
      double value = meta.default_value.get<double>();
      ros::param::param<double>(std::string("/settings/") + kNamespace + "/active/" + key, value, value);
      return value;
    }
    mower_logic::MowerLogicConfig current = getConfig();
    dynamic_reconfigure::Config current_msg;
    current.__toMessage__(current_msg);
    json value = meta.default_value;
    getConfigValue(current_msg, key, meta.type, value);
    return value;
  }

  json persistentValue(const std::string& key, const ParamMeta& meta) const {
    if (settings_entries_.contains(key) && settings_entries_[key].is_object() &&
        settings_entries_[key].contains("persistent") && isTypeValid(meta, settings_entries_[key]["persistent"])) {
      return settings_entries_[key]["persistent"];
    }
    return meta.default_value;
  }

  bool crossFieldPlausible(const std::map<std::string, json>& prospective, std::string& reason) const {
    auto getNumber = [&](const std::string& key) -> double {
      auto it = prospective.find(key);
      if (it != prospective.end() && it->second.is_number()) return it->second.get<double>();
      auto meta_it = metadata_.find(key);
      if (meta_it == metadata_.end()) return 0.0;
      return activeValue(key, meta_it->second).get<double>();
    };
    if (getNumber("motor_hot_temperature") < getNumber("motor_cold_temperature")) {
      reason = "motor_hot_temperature must be >= motor_cold_temperature";
      return false;
    }
    if (getNumber("mow_load_current_end") < getNumber("mow_load_current_start")) {
      reason = "mow_load_current_end must be >= mow_load_current_start";
      return false;
    }
    if (getNumber("mow_load_motor_temp_end") < getNumber("mow_load_motor_temp_start")) {
      reason = "mow_load_motor_temp_end must be >= mow_load_motor_temp_start";
      return false;
    }
    if (getNumber("mow_load_esc_temp_end") < getNumber("mow_load_esc_temp_start")) {
      reason = "mow_load_esc_temp_end must be >= mow_load_esc_temp_start";
      return false;
    }
    return true;
  }

  void handleSet(const std::string& payload_text, bool persistent) {
    json validation = {{"valid", false}, {"namespace", kNamespace},
                       {"mode", persistent ? "persistent" : "session"},
                       {"accepted", json::array()}, {"rejected", json::array()}};
    json payload;
    try {
      payload = json::parse(payload_text);
    } catch (const json::exception& e) {
      validation["rejected"].push_back({{"key", "$"}, {"reason", std::string("Error decoding JSON: ") + e.what()}});
      publishValidation(validation);
      return;
    }
    if (!payload.is_object()) {
      validation["rejected"].push_back({{"key", "$"}, {"reason", "payload must be a JSON object"}});
      publishValidation(validation);
      return;
    }

    std::map<std::string, json> accepted_values;
    std::map<std::string, std::map<std::string, json>> accepted_metadata;

    for (auto it = payload.begin(); it != payload.end(); ++it) {
      const std::string key = it.key();
      const json& entry = it.value();
      auto meta_it = metadata_.find(key);
      if (meta_it == metadata_.end()) {
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

      const ParamMeta& meta = meta_it->second;
      json accepted_fields = json::array();
      if (entry.contains("value")) {
        const json& value = entry["value"];
        if (!isTypeValid(meta, value)) {
          validation["rejected"].push_back({{"key", key}, {"field", "value"},
                                             {"reason", std::string("value must be ") + meta.type_name}});
          continue;
        }
        if (!isRangeValid(meta, value)) {
          validation["rejected"].push_back({{"key", key}, {"field", "value"},
                                             {"reason", "value is outside metadata limits"}});
          continue;
        }
        accepted_values[key] = value;
        accepted_fields.push_back("value");
      }

      if (entry.contains("group")) {
        std::string group;
        std::string reason;
        if (!validateGroupName(entry["group"], group, reason)) {
          validation["rejected"].push_back({{"key", key}, {"field", "group"}, {"reason", reason}});
          continue;
        }
        accepted_metadata[key]["group"] = group;
        accepted_fields.push_back("group");
      }

      if (entry.contains("expert")) {
        if (!entry["expert"].is_boolean()) {
          validation["rejected"].push_back({{"key", key}, {"field", "expert"}, {"reason", "expert must be a boolean"}});
          continue;
        }
        accepted_metadata[key]["expert"] = entry["expert"];
        accepted_fields.push_back("expert");
      }
    }

    if (accepted_values.empty() && accepted_metadata.empty() && validation["rejected"].empty()) {
      validation["rejected"].push_back({{"key", "$"}, {"reason", "payload does not contain any settings"}});
    }

    std::string cross_reason;
    if (!accepted_values.empty() && !crossFieldPlausible(accepted_values, cross_reason)) {
      validation["rejected"].push_back({{"key", "$"}, {"reason", cross_reason}});
      accepted_values.clear();
    }
    if (!validation["rejected"].empty()) {
      publishValidation(validation);
      return;
    }

    mower_logic::MowerLogicConfig active = getConfig();
    dynamic_reconfigure::Config active_msg;
    active.__toMessage__(active_msg);
    bool has_internal_values = false;
    for (const auto& pair : accepted_values) {
      const auto& meta = metadata_.at(pair.first);
      if (meta.external_ftc_planner) continue;
      setConfigValue(active_msg, pair.first, meta.type, pair.second);
      has_internal_values = true;
    }
    if (has_internal_values) {
      mower_logic::MowerLogicConfig applied = active;
      if (!applied.__fromMessage__(active_msg)) {
        validation["rejected"].push_back({{"key", "$"}, {"reason", "dynamic_reconfigure rejected the payload"}});
        publishValidation(validation);
        return;
      }
      setConfig(applied);
    }
    for (const auto& pair : accepted_values) {
      const auto& meta = metadata_.at(pair.first);
      if (!meta.external_ftc_planner) continue;
      std_msgs::Float64 msg;
      msg.data = pair.second.get<double>();
      if (persistent) {
        ftc_persistent_speed_fast_threshold_pub_.publish(msg);
      } else {
        ftc_speed_fast_threshold_pub_.publish(msg);
      }
    }

    std::map<std::string, std::map<std::string, json>> persistent_updates;
    for (const auto& pair : accepted_values) {
      if (persistent) {
        persistent_updates[pair.first]["persistent"] = pair.second;
      }
    }
    for (const auto& pair : accepted_metadata) {
      for (const auto& field : pair.second) {
        persistent_updates[pair.first][field.first] = field.second;
      }
    }
    if (!persistent_updates.empty()) {
      if (!open_mower_settings::updateEntryFields(settings_persistent_path_, kNamespace, persistent_updates)) {
        validation["rejected"].push_back({{"key", "$"}, {"reason", "could not write settings_persistent.json"}});
        publishValidation(validation);
        return;
      }
      for (const auto& pair : accepted_values) {
        if (persistent) settings_entries_[pair.first]["persistent"] = pair.second;
      }
      for (const auto& pair : accepted_metadata) {
        for (const auto& field : pair.second) {
          settings_entries_[pair.first][field.first] = field.second;
        }
      }
    }

    std::map<std::string, json> accepted_field_names;
    for (const auto& pair : accepted_values) {
      const auto& meta = metadata_.at(pair.first);
      const json persistent_value = persistent ? pair.second : persistentValue(pair.first, meta);
      syncParamTree(pair.first, meta, persistent_value, pair.second);
      accepted_field_names[pair.first].push_back("value");
    }
    for (const auto& pair : accepted_metadata) {
      for (const auto& field : pair.second) {
        accepted_field_names[pair.first].push_back(field.first);
      }
    }
    for (const auto& pair : accepted_field_names) {
      validation["accepted"].push_back({{"key", pair.first}, {"fields", pair.second}});
    }
    validation["valid"] = true;
    publishValidation(validation);
    publishStatus();
  }

  void publishValidation(const json& validation) const {
    std_msgs::String msg;
    msg.data = validation.dump();
    validation_pub_.publish(msg);
  }

  void publishStatus() const {
    if (metadata_.empty()) return;
    json status = json::object();
    status["schema"] = "settings_v2";
    status["namespace"] = kNamespace;
    status["settings"] = json::object();
    for (const auto& pair : metadata_) {
      const std::string& key = pair.first;
      const ParamMeta& meta = pair.second;
      const json active = activeValue(key, meta);
      const json persistent = persistentValue(key, meta);
      json entry = json::object();
      const json persisted_meta = settings_entries_.contains(key) ? settings_entries_.at(key) : json::object();
      entry["label"] = open_mower_settings::stringOr(persisted_meta, "label", labelForKey(key));
      entry["description"] = open_mower_settings::stringOr(persisted_meta, "description", meta.description);
      entry["group"] = open_mower_settings::stringOr(persisted_meta, "group", kNamespace);
      entry["expert"] = open_mower_settings::boolOr(persisted_meta, "expert", false);
      entry["order"] = open_mower_settings::intOr(persisted_meta, "order", meta.order);
      entry["session_apply_supported"] = open_mower_settings::boolOr(persisted_meta, "session_apply_supported", true);
      entry["restart_required"] = open_mower_settings::boolOr(persisted_meta, "restart_required", false);
      entry["default"] = meta.default_value;
      entry["value"] = active;
      entry["persistent"] = persistent;
      entry["active"] = active;
      entry["different"] = valuesDiffer(active, persistent);
      entry["unit"] = open_mower_settings::stringOr(persisted_meta, "unit", unitForKey(key));
      entry["type"] = open_mower_settings::stringOr(persisted_meta, "type", meta.type_name);
      if ((meta.type == ValueType::kInteger || meta.type == ValueType::kNumber) && meta.has_min) entry["min"] = meta.min_value;
      if ((meta.type == ValueType::kInteger || meta.type == ValueType::kNumber) && meta.has_max) entry["max"] = meta.max_value;
      status["settings"][key] = entry;
      syncParamTree(key, meta, persistent, active);
    }
    std_msgs::String msg;
    msg.data = status.dump();
    status_pub_.publish(msg);
  }

  void sessionSetCallback(const std_msgs::String::ConstPtr& msg) { handleSet(msg->data, false); }
  void persistentSetCallback(const std_msgs::String::ConstPtr& msg) { handleSet(msg->data, true); }
  void renewCallback(const std_msgs::Empty::ConstPtr&) { publishStatus(); }

  ros::NodeHandle& nh_;
  ros::Publisher status_pub_;
  ros::Publisher validation_pub_;
  ros::Publisher ftc_speed_fast_threshold_pub_;
  ros::Publisher ftc_persistent_speed_fast_threshold_pub_;
  ros::Subscriber session_sub_;
  ros::Subscriber persistent_sub_;
  ros::Subscriber renew_sub_;
  std::string settings_persistent_path_;
  std::map<std::string, ParamMeta> metadata_;
  json settings_entries_ = json::object();
};

void reconfigureCB(mower_logic::MowerLogicConfig& c, uint32_t level) {
  ROS_INFO_STREAM("om_mower_logic: Setting mower_logic config");
  last_config = c;
  if (mower_logic_settings_bridge != nullptr) {
    mower_logic_settings_bridge->notifyConfigChanged();
  }
  if (satellite_logging_controller != nullptr) {
    satellite_logging_controller->notifyConfigChanged();
  }
}

bool highLevelCommand(mower_msgs::HighLevelControlSrvRequest& req, mower_msgs::HighLevelControlSrvResponse& res) {
  switch (req.command) {
    case mower_msgs::HighLevelControlSrvRequest::COMMAND_HOME:
      ROS_INFO_STREAM("COMMAND_HOME");
      if (currentBehavior) {
        currentBehavior->command_home();
      }
      break;
    case mower_msgs::HighLevelControlSrvRequest::COMMAND_START:
      ROS_INFO_STREAM("COMMAND_START");
      if (currentBehavior) {
        currentBehavior->command_start();
      }
      break;
    case mower_msgs::HighLevelControlSrvRequest::COMMAND_S1:
      ROS_INFO_STREAM("COMMAND_S1");
      if (currentBehavior) {
        currentBehavior->command_s1();
      }
      break;
    case mower_msgs::HighLevelControlSrvRequest::COMMAND_S2:
      ROS_INFO_STREAM("COMMAND_S2");
      if (currentBehavior) {
        currentBehavior->command_s2();
      }
      break;
    case mower_msgs::HighLevelControlSrvRequest::COMMAND_DELETE_MAPS: {
      ROS_WARN_STREAM("COMMAND_DELETE_MAPS");
      if (currentBehavior != &AreaRecordingBehavior::INSTANCE && currentBehavior != &IdleBehavior::INSTANCE &&
          currentBehavior != &IdleBehavior::DOCKED_INSTANCE && currentBehavior != nullptr) {
        ROS_ERROR_STREAM("Deleting maps is only allowed during IDLE or AreaRecording!");
        return true;
      }
      mower_map::ClearMapSrv clear_map_srv;
      // TODO check result
      clearMapClient.call(clear_map_srv);

      // Abort the current behavior. Idle will refresh and go to AreaRecorder, AreaRecorder will to to Idle wich will go
      // to a fresh AreaRecorder
      currentBehavior->abort();
    } break;
    case mower_msgs::HighLevelControlSrvRequest::COMMAND_RESET_EMERGENCY:
      ROS_WARN_STREAM("COMMAND_RESET_EMERGENCY");
      setEmergencyMode(false);
      break;
  }
  return true;
}

void actionReceived(const std_msgs::String::ConstPtr& action) {
  if (action->data == "mower_logic/reset_emergency") {
    ROS_WARN_STREAM("Got reset emergency action.");
    setEmergencyMode(false);
    return;
  }

  if (currentBehavior) {
    currentBehavior->handle_action(action->data);
  }
}

void joyVelReceived(const geometry_msgs::Twist::ConstPtr& joy_vel) {
  joy_vel_time = ros::Time::now();
  if (currentBehavior && currentBehavior->redirect_joystick()) {
    cmd_vel_pub.publish(joy_vel);
  }
}

void buildRootActions() {
  xbot_msgs::ActionInfo reset_emergency_action;
  reset_emergency_action.action_id = "reset_emergency";
  reset_emergency_action.enabled = true;
  reset_emergency_action.action_name = "Reset Emergency";
  rootActions.push_back(reset_emergency_action);
}

int main(int argc, char** argv) {
  buildRootActions();

  ros::init(argc, argv, "mower_logic");

  n = new ros::NodeHandle();
  paramNh = new ros::NodeHandle("~");
  ros::NodeHandle powerNodeHandle("/ll/services/power");
  mowerAllowed = false;

  boost::recursive_mutex mutex;

  reconfigServer = new dynamic_reconfigure::Server<mower_logic::MowerLogicConfig>(mutex, *paramNh);
  reconfigServer->setCallback(reconfigureCB);
  mower_logic_settings_bridge = new MowerLogicSettingsBridge(*n, *paramNh);
  satellite_logging_controller = new SatelliteLoggingController(*n);

  last_power_config = ll::PowerConfig::__getDefault__();
  last_power_config.__fromServer__(powerNodeHandle);

  cmd_vel_pub = n->advertise<geometry_msgs::Twist>("/logic_vel", 1);

  high_level_state_publisher = n->advertise<mower_msgs::HighLevelStatus>("mower_logic/current_state", 100, true);

  pathClient = n->serviceClient<slic3r_coverage_planner::PlanPath>("slic3r_coverage_planner/plan_path");
  pathOrderOptimizerClient =
      n->serviceClient<mowing_path_order_optimizer::OptimizePaths>("mowing_path_order_optimizer/optimize_paths");
  mapClient = n->serviceClient<mower_map::GetMowingAreaSrv>("mower_map_service/get_mowing_area");
  mapAreaListClient = n->serviceClient<mower_map::GetMowingAreaListSrv>("mower_map_service/get_mowing_area_list");
  mapAreaByIdClient = n->serviceClient<mower_map::GetMowingAreaByIdSrv>("mower_map_service/get_mowing_area_by_id");
  clearMapClient = n->serviceClient<mower_map::ClearMapSrv>("mower_map_service/clear_map");

  gpsClient = n->serviceClient<xbot_positioning::GPSControlSrv>("xbot_positioning/set_gps_state");
  positioningClient = n->serviceClient<xbot_positioning::SetPoseSrv>("xbot_positioning/set_robot_pose");
  actionRegistrationClient = n->serviceClient<xbot_msgs::RegisterActionsSrv>("xbot/register_actions");

  mowClient = n->serviceClient<mower_msgs::MowerControlSrv>("ll/_service/mow_enabled");
  emergencyClient = n->serviceClient<mower_msgs::EmergencyStopSrv>("ll/_service/emergency");

  dockingPointClient = n->serviceClient<mower_map::GetDockingPointSrv>("mower_map_service/get_docking_point");

  pathProgressClient =
      n->serviceClient<ftc_local_planner::PlannerGetProgress>("/move_base_flex/FTCPlanner/planner_get_progress");

  setNavPointClient = n->serviceClient<mower_map::SetNavPointSrv>("mower_map_service/set_nav_point");
  clearNavPointClient = n->serviceClient<mower_map::ClearNavPointSrv>("mower_map_service/clear_nav_point");

  mbfClient = new actionlib::SimpleActionClient<mbf_msgs::MoveBaseAction>("/move_base_flex/move_base");
  mbfClientExePath = new actionlib::SimpleActionClient<mbf_msgs::ExePathAction>("/move_base_flex/exe_path");

  emergency_state_subscriber.Start(n);
  status_state_subscriber.Start(n);
  power_state_subscriber.Start(n);
  left_esc_status_state_subscriber.Start(n);
  right_esc_status_state_subscriber.Start(n);
  pose_state_subscriber.Start(n);

  ros::Subscriber joy_cmd = n->subscribe("/joy_vel", 0, joyVelReceived, ros::TransportHints().tcpNoDelay(true));
  ros::Subscriber action = n->subscribe("xbot/action", 0, actionReceived, ros::TransportHints().tcpNoDelay(true));

  ros::ServiceServer high_level_control_srv = n->advertiseService("mower_service/high_level_control", highLevelCommand);

  ros::AsyncSpinner asyncSpinner(1);
  asyncSpinner.start();

  ros::Rate r(1.0);

  ROS_INFO("Waiting for emergency message");
  while (!emergency_state_subscriber.hasMessage()) {
    if (!ros::ok()) {
      delete (reconfigServer);
      delete (mbfClient);
      delete (mbfClientExePath);
      return 1;
    }
    r.sleep();
  }
  ROS_INFO("Waiting for a power message");
  while (!power_state_subscriber.hasMessage()) {
    if (!ros::ok()) {
      delete (reconfigServer);
      delete (mbfClient);
      delete (mbfClientExePath);
      return 1;
    }
    r.sleep();
  }

  ROS_INFO("Waiting for a status message");
  while (!status_state_subscriber.hasMessage()) {
    if (!ros::ok()) {
      delete (reconfigServer);
      delete (mbfClient);
      delete (mbfClientExePath);
      return 1;
    }
    r.sleep();
  }

  ROS_INFO("Waiting for a pose message");
  while (!power_state_subscriber.hasMessage()) {
    if (!ros::ok()) {
      delete (reconfigServer);
      delete (mbfClient);
      delete (mbfClientExePath);
      return 1;
    }
    r.sleep();
  }
  ROS_INFO("Waiting for left ESC status message");
  while (!left_esc_status_state_subscriber.hasMessage()) {
    if (!ros::ok()) {
      delete (reconfigServer);
      delete (mbfClient);
      delete (mbfClientExePath);
      return 1;
    }
    r.sleep();
  }
  ROS_INFO("Waiting for right ESC status message");
  while (!right_esc_status_state_subscriber.hasMessage()) {
    if (!ros::ok()) {
      delete (reconfigServer);
      delete (mbfClient);
      delete (mbfClientExePath);
      return 1;
    }
    r.sleep();
  }

  ROS_INFO("Waiting for emergency service");
  if (!emergencyClient.waitForExistence(ros::Duration(60.0, 0.0))) {
    ROS_ERROR("Emergency server not found.");
    delete (reconfigServer);
    delete (mbfClient);
    delete (mbfClientExePath);

    return 1;
  }

  ROS_INFO("Waiting for path server");
  if (!pathClient.waitForExistence(ros::Duration(60.0, 0.0))) {
    ROS_ERROR("Path service not found.");
    delete (reconfigServer);
    delete (mbfClient);
    delete (mbfClientExePath);

    return 1;
  }
  ROS_INFO("Waiting for mower service");
  if (!mowClient.waitForExistence(ros::Duration(60.0, 0.0))) {
    ROS_ERROR("Mower service not found.");
    delete (reconfigServer);
    delete (mbfClient);
    delete (mbfClientExePath);

    return 1;
  }

  ROS_INFO("Waiting for gps service");
  if (!gpsClient.waitForExistence(ros::Duration(60.0, 0.0))) {
    ROS_ERROR("GPS service not found.");
    delete (reconfigServer);
    delete (mbfClient);
    delete (mbfClientExePath);

    return 1;
  }
  ROS_INFO("Waiting for positioning service");
  if (!positioningClient.waitForExistence(ros::Duration(60.0, 0.0))) {
    ROS_ERROR("positioning service not found.");
    delete (reconfigServer);
    delete (mbfClient);
    delete (mbfClientExePath);

    return 1;
  }

  ROS_INFO("Waiting for map server");
  if (!mapClient.waitForExistence(ros::Duration(60.0, 0.0))) {
    ROS_ERROR("Map server service not found.");
    delete (reconfigServer);
    delete (mbfClient);
    delete (mbfClientExePath);
    return 2;
  }
  ROS_INFO("Waiting for docking point server");
  if (!dockingPointClient.waitForExistence(ros::Duration(60.0, 0.0))) {
    ROS_ERROR("Docking server service not found.");
    delete (reconfigServer);
    delete (mbfClient);
    delete (mbfClientExePath);
    return 2;
  }
  ROS_INFO("Waiting for nav point server");
  if (!setNavPointClient.waitForExistence(ros::Duration(60.0, 0.0))) {
    ROS_ERROR("Set Nav Point server service not found.");
    delete (reconfigServer);
    delete (mbfClient);
    delete (mbfClientExePath);
    return 2;
  }
  ROS_INFO("Waiting for clear nav point server");
  if (!clearNavPointClient.waitForExistence(ros::Duration(60.0, 0.0))) {
    ROS_ERROR("Clear Nav Point server service not found.");
    delete (reconfigServer);
    delete (mbfClient);
    delete (mbfClientExePath);
    return 2;
  }

  ROS_INFO("Waiting for move base flex");
  if (!mbfClient->waitForServer(ros::Duration(60.0, 0.0))) {
    ROS_ERROR("Move base flex not found.");
    delete (reconfigServer);
    delete (mbfClient);
    delete (mbfClientExePath);
    return 3;
  }

  ROS_INFO("Waiting for mowing path progress server");
  if (!pathProgressClient.waitForExistence(ros::Duration(60.0, 0.0))) {
    ROS_ERROR("FTCLocalPlanner progress server not found.");
    delete (reconfigServer);
    delete (mbfClient);
    delete (mbfClientExePath);
    return 3;
  }

  ros::Time started = ros::Time::now();
  while ((ros::Time::now() - started).toSec() < 10.0) {
    ROS_INFO_STREAM("Waiting for an emergency status message");
    r.sleep();
    if (emergency_state_subscriber.getMessage().latched_emergency) {
      ROS_INFO_STREAM("Got emergency, resetting it");
      setEmergencyMode(false);
      break;
    }
  }

  ROS_INFO("registering actions");
  registerActions("mower_logic", rootActions);

  ROS_INFO("om_mower_logic: Got all servers, we can mow");

  rain_resume = last_rain_check = last_v_battery_check = ros::Time::now();
  ros::Timer safety_timer = n->createTimer(ros::Duration(0.5), checkSafety);
  ros::Timer ui_timer = n->createTimer(ros::Duration(1.0), updateUI);

  // release emergency if it was set
  setEmergencyMode(false);

  // initialise the shared state object to be passed into the behaviors
  auto shared_state = std::make_shared<sSharedState>();
  shared_state->active_semiautomatic_task = false;

  // Behavior execution loop
  while (ros::ok()) {
    if (currentBehavior != nullptr) {
      currentBehavior->start(last_config, shared_state);
      Behavior* previousBehavior = currentBehavior;
      Behavior* newBehavior = currentBehavior->execute();
      currentBehavior->exit();

      if (dockedIdleOnChargingRequested.exchange(false)) {
        newBehavior = &IdleBehavior::DOCKED_INSTANCE;
      }

      currentBehavior = newBehavior;
      if (currentBehavior != previousBehavior) {
        resetDockedIdleChargingTimer();
      }
    } else {
      high_level_status.state_name = "NULL";
      high_level_status.state = mower_msgs::HighLevelStatus::HIGH_LEVEL_STATE_NULL;
      high_level_state_publisher.publish(high_level_status);
      // we have no defined behavior, set emergency
      ROS_ERROR_STREAM("null behavior - emergency mode");
      setEmergencyMode(true);
      ros::Rate r(1.0);
      r.sleep();
    }
  }

  delete (n);
  delete (satellite_logging_controller);
  delete (paramNh);
  delete (reconfigServer);
  delete (mbfClient);
  delete (mbfClientExePath);
  return 0;
}
