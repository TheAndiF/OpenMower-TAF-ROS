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
#include "MowingBehavior.h"

#include <cryptopp/cryptlib.h>
#include <cryptopp/hex.h>
#include <cryptopp/sha.h>
#include <nav_msgs/Path.h>
#include <nlohmann/json.hpp>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <sys/stat.h>
#include <sys/types.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include "mower_logic/CheckPoint.h"
#include "mower_map/ClearNavPointSrv.h"
#include "mower_map/GetMowingAreaSrv.h"
#include "mower_map/GetMowingAreaByIdSrv.h"
#include "mower_map/GetMowingAreaListSrv.h"
#include "mower_map/SetNavPointSrv.h"
#include "mowing_path_order_optimizer/OptimizePaths.h"
#include "xbot_msgs/AbsolutePose.h"

extern ros::ServiceClient mapClient;
extern ros::ServiceClient mapAreaListClient;
extern ros::ServiceClient mapAreaByIdClient;
extern ros::ServiceClient pathClient;
extern ros::ServiceClient pathOrderOptimizerClient;
extern ros::ServiceClient pathProgressClient;
extern ros::ServiceClient setNavPointClient;
extern ros::ServiceClient clearNavPointClient;
extern ros::NodeHandle* n;

extern actionlib::SimpleActionClient<mbf_msgs::MoveBaseAction>* mbfClient;
extern actionlib::SimpleActionClient<mbf_msgs::ExePathAction>* mbfClientExePath;
extern mower_logic::MowerLogicConfig getConfig();
extern xbot_msgs::AbsolutePose getPose();
extern void setConfig(mower_logic::MowerLogicConfig);

extern void registerActions(std::string prefix, const std::vector<xbot_msgs::ActionInfo>& actions);

MowingBehavior MowingBehavior::INSTANCE;

constexpr uint8_t MowingBehavior::MOW_STATUS_DONE;
constexpr uint8_t MowingBehavior::MOW_STATUS_IN_PROGRESS;
constexpr uint8_t MowingBehavior::MOW_STATUS_OPEN;
constexpr uint8_t MowingBehavior::PATH_DIRECTION_FORWARD;
constexpr uint8_t MowingBehavior::PATH_DIRECTION_REVERSE;

using json = nlohmann::json;

namespace {
std::string make_path_id(int path_index) {
  std::ostringstream ss;
  ss << "pa_" << std::setw(6) << std::setfill('0') << path_index;
  return ss.str();
}

std::string make_plan_id() {
  const std::time_t now = std::time(nullptr);
  std::tm tm{};
  localtime_r(&now, &tm);
  std::ostringstream ss;
  ss << "mp_" << std::put_time(&tm, "%Y%m%d_%H%M%S");
  return ss.str();
}

void ensure_directory(const std::string& path) {
  mkdir(path.c_str(), 0755);
}

void update_hash_with_polygon(CryptoPP::SHA256& hash, const geometry_msgs::Polygon& polygon) {
  for (const auto& point : polygon.points) {
    hash.Update(reinterpret_cast<const byte*>(&point.x), sizeof(point.x));
    hash.Update(reinterpret_cast<const byte*>(&point.y), sizeof(point.y));
    hash.Update(reinterpret_cast<const byte*>(&point.z), sizeof(point.z));
  }
}

std::string compute_area_digest(const mower_map::MapArea& area) {
  CryptoPP::SHA256 hash;
  byte digest[CryptoPP::SHA256::DIGESTSIZE];
  update_hash_with_polygon(hash, area.area);
  for (const auto& obstacle : area.obstacles) update_hash_with_polygon(hash, obstacle);
  hash.Final(digest);

  std::string result;
  CryptoPP::HexEncoder encoder;
  encoder.Attach(new CryptoPP::StringSink(result));
  encoder.Put(digest, sizeof(digest));
  encoder.MessageEnd();
  return "ad_" + result.substr(0, 16);
}

void recompute_execution_orientations(slic3r_coverage_planner::Path& path) {
  auto& poses = path.path.poses;
  if (poses.size() < 2) return;
  for (std::size_t i = 0; i + 1 < poses.size(); ++i) {
    const auto& current = poses[i].pose.position;
    const auto& next = poses[i + 1].pose.position;
    const double yaw = std::atan2(next.y - current.y, next.x - current.x);
    poses[i].pose.orientation.x = 0.0;
    poses[i].pose.orientation.y = 0.0;
    poses[i].pose.orientation.z = std::sin(yaw * 0.5);
    poses[i].pose.orientation.w = std::cos(yaw * 0.5);
  }
  poses.back().pose.orientation = poses[poses.size() - 2].pose.orientation;
}

slic3r_coverage_planner::Path make_execution_path_from_slicer(
    const slic3r_coverage_planner::Path& source, uint8_t path_direction) {
  slic3r_coverage_planner::Path execution = source;
  if (path_direction == MowingBehavior::PATH_DIRECTION_REVERSE) {
    std::reverse(execution.path.poses.begin(), execution.path.poses.end());
    recompute_execution_orientations(execution);
  }
  return execution;
}

const geometry_msgs::PoseStamped& execution_pose(const MowingBehavior::MowingPathExecutionItem& item,
                                                 std::size_t execution_index) {
  const auto& poses = item.execution.path.path.poses;
  execution_index = std::min(execution_index, poses.size() - 1);
  return poses[execution_index];
}

nav_msgs::Path execution_path_from_index(const MowingBehavior::MowingPathExecutionItem& item,
                                         std::size_t execution_start_index) {
  nav_msgs::Path path;
  const auto& source = item.execution.path.path;
  path.header = source.header;
  const std::size_t size = source.poses.size();
  execution_start_index = std::min(execution_start_index, size);
  path.poses.insert(path.poses.end(), source.poses.begin() + execution_start_index, source.poses.end());
  return path;
}

json path_points_to_json(const MowingBehavior::MowingPathExecutionItem& item, std::size_t begin, std::size_t end) {
  json points = json::array();
  const auto& poses = item.execution.path.path.poses;
  end = std::min(end, poses.size());
  begin = std::min(begin, end);
  for (std::size_t i = begin; i < end; ++i) {
    const auto& pose = poses[i];
    points.push_back({{"x", pose.pose.position.x}, {"y", pose.pose.position.y}});
  }
  return points;
}

json pose_to_json(const geometry_msgs::PoseStamped& pose) {
  return {{"x", pose.pose.position.x},
          {"y", pose.pose.position.y},
          {"z", pose.pose.position.z},
          {"qx", pose.pose.orientation.x},
          {"qy", pose.pose.orientation.y},
          {"qz", pose.pose.orientation.z},
          {"qw", pose.pose.orientation.w}};
}

std::string mow_status_to_json_string(uint8_t status) {
  switch (status) {
    case MowingBehavior::MOW_STATUS_DONE:
      return "mowed";
    case MowingBehavior::MOW_STATUS_IN_PROGRESS:
      return "mowing";
    case MowingBehavior::MOW_STATUS_OPEN:
    default:
      return "unmowed";
  }
}

std::string path_direction_to_json_string(uint8_t direction) {
  switch (direction) {
    case MowingBehavior::PATH_DIRECTION_REVERSE:
      return "reverse";
    case MowingBehavior::PATH_DIRECTION_FORWARD:
    default:
      return "forward";
  }
}

json string_vector_to_json(const std::vector<std::string>& values) {
  json result = json::array();
  for (const auto& value : values) result.push_back(value);
  return result;
}

std::string json_escape_string(const std::string& value) {
  return json(value).dump();
}

std::vector<std::string> split_transform_flags(const std::string& flags) {
  std::vector<std::string> result;
  std::stringstream ss(flags);
  std::string item;
  while (std::getline(ss, item, ',')) {
    if (!item.empty()) result.push_back(item);
  }
  return result;
}
}

std::string MowingBehavior::state_name() {
  if (paused) {
    return "PAUSED";
  }
  return "MOWING";
}


void MowingBehavior::clear_current_mowing_plan() {
  currentMowingPlan.paths.clear();
  currentMowingPlan.area_id.clear();
  currentMowingPlan.area_digest.clear();
  currentMowingPlan.plan_id.clear();
  currentMowingPlan.current_order = 0;
  currentMowingPlan.current_path_id.clear();
  currentMowingPlan.plan_file.clear();
  currentMowingPlan.processing_mode = 2;
  currentMowingPlan.outline_entry_mode = 0;
}

std::string MowingBehavior::make_plan_file_path(const std::string& plan_id) const {
  return std::string("/home/openmower/mowing_plans/") + plan_id + ".json";
}

void MowingBehavior::normalize_current_mowing_plan_orders() {
  for (std::size_t i = 0; i < currentMowingPlan.paths.size(); ++i) {
    currentMowingPlan.paths[i].order = static_cast<uint32_t>(i);
  }
}

void MowingBehavior::start_current_mowing_plan_path() {
  if (currentMowingPath >= 0 && currentMowingPath < static_cast<int>(currentMowingPlan.paths.size())) {
    auto& item = currentMowingPlan.paths[currentMowingPath];
    item.mow_status = MOW_STATUS_IN_PROGRESS;
    item.current_pose_index = static_cast<uint32_t>(std::max(currentMowingPathIndex, 0));
    currentMowingPlan.current_order = item.order;
    currentMowingPlan.current_path_id = item.path_id;
  } else {
    currentMowingPlan.current_order = static_cast<uint32_t>(std::max(currentMowingPath, 0));
    currentMowingPlan.current_path_id.clear();
  }
}

void MowingBehavior::update_current_mowing_plan_progress() {
  if (currentMowingPath >= 0 && currentMowingPath < static_cast<int>(currentMowingPlan.paths.size())) {
    auto& item = currentMowingPlan.paths[currentMowingPath];
    item.mow_status = MOW_STATUS_IN_PROGRESS;
    item.current_pose_index = static_cast<uint32_t>(std::max(currentMowingPathIndex, 0));
    currentMowingPlan.current_order = item.order;
    currentMowingPlan.current_path_id = item.path_id;
  }
}

void MowingBehavior::finish_current_mowing_plan_path() {
  if (currentMowingPath >= 0 && currentMowingPath < static_cast<int>(currentMowingPlan.paths.size())) {
    auto& item = currentMowingPlan.paths[currentMowingPath];
    item.mow_status = MOW_STATUS_DONE;
    const auto pose_count = item.execution.path.path.poses.size();
    item.current_pose_index = pose_count == 0 ? 0 : static_cast<uint32_t>(pose_count - 1);
  }
}

void MowingBehavior::build_current_mowing_plan(
    const std::vector<slic3r_coverage_planner::Path>& slicer_paths,
    const std::string& area_id,
    const std::string& area_digest) {
  clear_current_mowing_plan();
  currentMowingPlan.area_id = area_id;
  currentMowingPlan.area_digest = area_digest;
  currentMowingPlan.plan_id = make_plan_id();
  currentMowingPlan.plan_file = make_plan_file_path(currentMowingPlan.plan_id);
  currentMowingPlan.processing_mode = static_cast<uint8_t>(config.path_order_optimizer_processing_mode);
  currentMowingPlan.outline_entry_mode = static_cast<uint8_t>(config.path_order_optimizer_outline_entry_mode);

  currentMowingPlan.paths.reserve(slicer_paths.size());
  for (std::size_t i = 0; i < slicer_paths.size(); ++i) {
    MowingPathExecutionItem item;
    item.area_id = currentMowingPlan.area_id;
    item.area_digest = currentMowingPlan.area_digest;
    item.plan_id = currentMowingPlan.plan_id;
    item.path_id = make_path_id(static_cast<int>(i));
    item.order = static_cast<uint32_t>(i);
    item.path_direction = PATH_DIRECTION_FORWARD;
    // Mow status:
    //   20 = noch nicht bearbeitet
    //   10 = in Arbeit
    //   00 = fertig bearbeitet
    item.mow_status = MOW_STATUS_OPEN;
    item.current_pose_index = 0;
    item.slicer_source.path = slicer_paths[i];
    item.slicer_source.path_id = static_cast<uint32_t>(i);
    item.execution.path = make_execution_path_from_slicer(item.slicer_source.path, item.path_direction);
    item.execution.rotation_offset = 0;
    item.execution.transform_flags.clear();
    currentMowingPlan.paths.push_back(item);
  }

  currentMowingPath = 0;
  currentMowingPathIndex = 0;
  start_current_mowing_plan_path();
}

bool MowingBehavior::optimize_current_mowing_plan(const geometry_msgs::PoseStamped& current_pose) {
  if (currentMowingPlan.paths.empty()) return true;

  mowing_path_order_optimizer::OptimizePaths optimizeSrv;
  for (const auto& item : currentMowingPlan.paths) {
    optimizeSrv.request.paths.push_back(item.slicer_source.path);
    optimizeSrv.request.path_indices.push_back(static_cast<int32_t>(item.slicer_source.path_id));
  }
  optimizeSrv.request.enabled = config.path_order_optimizer_enabled;
  optimizeSrv.request.processing_mode = static_cast<uint8_t>(config.path_order_optimizer_processing_mode);
  optimizeSrv.request.outline_entry_mode = config.path_order_optimizer_outline_entry_mode;
  optimizeSrv.request.allow_reverse = config.path_order_optimizer_allow_reverse;
  optimizeSrv.request.cost_mode = config.path_order_optimizer_cost_mode;
  optimizeSrv.request.max_fill_paths = config.path_order_optimizer_max_fill_paths;
  optimizeSrv.request.candidate_limit = config.path_order_optimizer_candidate_limit;
  optimizeSrv.request.planner_timeout = config.path_order_optimizer_planner_timeout;
  optimizeSrv.request.fallback_to_euclidean = config.path_order_optimizer_fallback_to_euclidean;
  optimizeSrv.request.fallback_to_slicer_order = config.path_order_optimizer_fallback_to_slicer_order;
  optimizeSrv.request.fail_open = config.path_order_optimizer_fail_open;
  optimizeSrv.request.planner_action = config.path_order_optimizer_planner_action;
  optimizeSrv.request.planner_name = config.path_order_optimizer_planner_name;
  optimizeSrv.request.current_pose = current_pose;

  if (pathOrderOptimizerClient.call(optimizeSrv)) {
    if (!optimizeSrv.response.success) {
      if (config.path_order_optimizer_fail_open) {
        ROS_WARN_STREAM("MowingBehavior: path order optimizer returned failure, using slicer order: "
                        << optimizeSrv.response.message);
        return true;
      }
      ROS_ERROR_STREAM("MowingBehavior: path order optimizer returned failure: " << optimizeSrv.response.message);
      return false;
    }

    std::vector<MowingPathExecutionItem> reordered;
    reordered.reserve(currentMowingPlan.paths.size());
    for (std::size_t i = 0; i < optimizeSrv.response.path_indices.size(); ++i) {
      const int32_t source_index = optimizeSrv.response.path_indices[i];
      auto it = std::find_if(currentMowingPlan.paths.begin(), currentMowingPlan.paths.end(),
                             [source_index](const MowingPathExecutionItem& item) {
                               return static_cast<int32_t>(item.slicer_source.path_id) == source_index;
                             });
      if (it == currentMowingPlan.paths.end()) {
        ROS_WARN_STREAM("MowingBehavior: optimizer returned unknown slicer path index " << source_index
                        << "; keeping slicer order");
        return true;
      }
      MowingPathExecutionItem item = *it;
      item.path_direction = (i < optimizeSrv.response.path_reversed.size() && optimizeSrv.response.path_reversed[i])
                                ? PATH_DIRECTION_REVERSE
                                : PATH_DIRECTION_FORWARD;
      if (i < optimizeSrv.response.paths.size()) {
        // The POO response path is the finished execution geometry. If reversed or rotated,
        // it is already prepared and re-oriented by the optimizer.
        item.execution.path = optimizeSrv.response.paths[i];
      } else {
        // Defensive fallback for older optimizer responses.
        item.execution.path = make_execution_path_from_slicer(item.slicer_source.path, item.path_direction);
      }
      item.execution.rotation_offset = (i < optimizeSrv.response.rotation_offsets.size())
                                           ? optimizeSrv.response.rotation_offsets[i]
                                           : 0;
      item.execution.transform_flags = (i < optimizeSrv.response.transform_flags.size())
                                           ? split_transform_flags(optimizeSrv.response.transform_flags[i])
                                           : std::vector<std::string>();
      if (item.path_direction == PATH_DIRECTION_REVERSE && item.execution.transform_flags.empty()) {
        item.execution.transform_flags.push_back("reversed");
      }
      item.mow_status = MOW_STATUS_OPEN;
      item.current_pose_index = 0;
      reordered.push_back(item);
    }

    if (reordered.size() == currentMowingPlan.paths.size()) {
      currentMowingPlan.paths = reordered;
      currentMowingPlan.processing_mode = static_cast<uint8_t>(config.path_order_optimizer_processing_mode);
      currentMowingPlan.outline_entry_mode = static_cast<uint8_t>(config.path_order_optimizer_outline_entry_mode);
      normalize_current_mowing_plan_orders();
      currentMowingPath = 0;
      currentMowingPathIndex = 0;
      start_current_mowing_plan_path();
    }
    ROS_INFO_STREAM("MowingBehavior: path order optimizer response: " << optimizeSrv.response.message);
    return true;
  }

  if (config.path_order_optimizer_enabled && !config.path_order_optimizer_fail_open) {
    ROS_ERROR_STREAM("MowingBehavior: path order optimizer unavailable and fail-open is disabled");
    return false;
  }
  if (config.path_order_optimizer_enabled) {
    ROS_WARN_STREAM("MowingBehavior: path order optimizer unavailable, using slicer order");
  }
  return true;
}

bool MowingBehavior::load_current_mowing_plan_snapshot(const std::string& plan_file,
                                                        const std::string& expected_area_id,
                                                        const std::string& expected_area_digest) {
  std::ifstream in(plan_file);
  if (!in.good()) return false;

  json root;
  try {
    in >> root;
  } catch (const std::exception& e) {
    ROS_WARN_STREAM("MowingBehavior: Could not parse mowing plan snapshot " << plan_file << ": " << e.what());
    return false;
  }

  if (root.value("area_id", std::string()) != expected_area_id ||
      root.value("area_digest", std::string()) != expected_area_digest) {
    ROS_INFO_STREAM("MowingBehavior: Saved mowing plan snapshot does not match current area/digest; ignoring "
                    << plan_file);
    return false;
  }

  MowingExecutionPlan loaded;
  loaded.area_id = root.value("area_id", std::string());
  loaded.area_digest = root.value("area_digest", std::string());
  loaded.plan_id = root.value("plan_id", std::string());
  loaded.plan_file = plan_file;
  loaded.current_order = root.value("current_order", 0);
  loaded.current_path_id = root.value("current_path_id", std::string());
  loaded.processing_mode = root.value("processing_mode", 2);
  loaded.outline_entry_mode = static_cast<uint8_t>(root.value("outline_entry_mode", 0));

  if (!root.contains("paths") || !root["paths"].is_array()) return false;

  for (const auto& path_json : root["paths"]) {
    MowingPathExecutionItem item;
    item.area_id = path_json.value("area_id", loaded.area_id);
    item.area_digest = path_json.value("area_digest", loaded.area_digest);
    item.plan_id = path_json.value("plan_id", loaded.plan_id);
    item.path_id = path_json.value("path_id", std::string());
    item.order = path_json.value("order", 0);
    item.path_direction = path_json.value("path_direction", PATH_DIRECTION_FORWARD);
    item.execution.rotation_offset = 0;
    item.execution.transform_flags.clear();
    item.mow_status = MOW_STATUS_OPEN;
    item.current_pose_index = 0;

    const auto& source = path_json["slicer_source"];
    item.slicer_source.path_id = source.value("path_id", source.value("path_index", 0));
    item.slicer_source.path.path_type = source.value("path_type", 0);
    item.slicer_source.path.is_outline = source.value("is_outline", 0);
    item.slicer_source.path.path.header.frame_id = source.value("frame_id", std::string("map"));

    if (source.contains("poses") && source["poses"].is_array()) {
      for (const auto& pose_json : source["poses"]) {
        geometry_msgs::PoseStamped pose;
        pose.header = item.slicer_source.path.path.header;
        pose.pose.position.x = pose_json.value("x", 0.0);
        pose.pose.position.y = pose_json.value("y", 0.0);
        pose.pose.position.z = pose_json.value("z", 0.0);
        pose.pose.orientation.x = pose_json.value("qx", 0.0);
        pose.pose.orientation.y = pose_json.value("qy", 0.0);
        pose.pose.orientation.z = pose_json.value("qz", 0.0);
        pose.pose.orientation.w = pose_json.value("qw", 1.0);
        item.slicer_source.path.path.poses.push_back(pose);
      }
    }

    if (path_json.contains("execution") && path_json["execution"].contains("path")) {
      const auto& execution = path_json["execution"]["path"];
      item.execution.path.path_type = item.slicer_source.path.path_type;
      item.execution.path.is_outline = item.slicer_source.path.is_outline;
      item.execution.path.path.header.frame_id = execution.value("frame_id", source.value("frame_id", std::string("map")));
      if (execution.contains("poses") && execution["poses"].is_array()) {
        for (const auto& pose_json : execution["poses"]) {
          geometry_msgs::PoseStamped pose;
          pose.header = item.execution.path.path.header;
          pose.pose.position.x = pose_json.value("x", 0.0);
          pose.pose.position.y = pose_json.value("y", 0.0);
          pose.pose.position.z = pose_json.value("z", 0.0);
          pose.pose.orientation.x = pose_json.value("qx", 0.0);
          pose.pose.orientation.y = pose_json.value("qy", 0.0);
          pose.pose.orientation.z = pose_json.value("qz", 0.0);
          pose.pose.orientation.w = pose_json.value("qw", 1.0);
          item.execution.path.path.poses.push_back(pose);
        }
      }
    }

    if (path_json.contains("execution") && path_json["execution"].is_object()) {
      const auto& execution_meta = path_json["execution"];
      item.execution.rotation_offset = execution_meta.value("rotation_offset", 0);
      if (execution_meta.contains("transform_flags") && execution_meta["transform_flags"].is_array()) {
        item.execution.transform_flags.clear();
        for (const auto& flag_json : execution_meta["transform_flags"]) {
          if (flag_json.is_string()) item.execution.transform_flags.push_back(flag_json.get<std::string>());
        }
      }
    }

    if (item.execution.path.path.poses.empty()) {
      item.execution.path = make_execution_path_from_slicer(item.slicer_source.path, item.path_direction);
    }
    if (item.path_direction == PATH_DIRECTION_REVERSE && item.execution.transform_flags.empty()) {
      item.execution.transform_flags.push_back("reversed");
    }

    loaded.paths.push_back(item);
  }

  if (loaded.paths.empty()) return false;
  currentMowingPlan = loaded;

  for (std::size_t i = 0; i < currentMowingPlan.paths.size(); ++i) {
    auto& item = currentMowingPlan.paths[i];
    if (static_cast<int>(i) < currentMowingPath) {
      item.mow_status = MOW_STATUS_DONE;
      const auto pose_count = item.execution.path.path.poses.size();
      item.current_pose_index = pose_count == 0 ? 0 : static_cast<uint32_t>(pose_count - 1);
    } else if (static_cast<int>(i) == currentMowingPath) {
      item.mow_status = MOW_STATUS_IN_PROGRESS;
      item.current_pose_index = static_cast<uint32_t>(std::max(currentMowingPathIndex, 0));
    } else {
      item.mow_status = MOW_STATUS_OPEN;
      item.current_pose_index = 0;
    }
  }
  start_current_mowing_plan_path();
  ROS_INFO_STREAM("MowingBehavior: Loaded mowing plan snapshot: " << plan_file);
  return true;
}

bool MowingBehavior::save_current_mowing_plan() const {
  if (currentMowingPlan.plan_id.empty() || currentMowingPlan.plan_file.empty()) return false;
  ensure_directory("/home/openmower");
  ensure_directory("/home/openmower/mowing_plans");

  json root;
  root["area_id"] = currentMowingPlan.area_id;
  root["area_digest"] = currentMowingPlan.area_digest;
  root["plan_id"] = currentMowingPlan.plan_id;
  root["current_order"] = currentMowingPlan.current_order;
  root["current_path_id"] = currentMowingPlan.current_path_id;
  root["processing_mode"] = static_cast<int>(currentMowingPlan.processing_mode);
  root["outline_entry_mode"] = static_cast<int>(currentMowingPlan.outline_entry_mode);
  root["paths"] = json::array();

  for (const auto& item : currentMowingPlan.paths) {
    json path_json;
    path_json["area_id"] = item.area_id;
    path_json["area_digest"] = item.area_digest;
    path_json["plan_id"] = item.plan_id;
    path_json["path_id"] = item.path_id;
    path_json["order"] = item.order;
    path_json["path_direction"] = static_cast<int>(item.path_direction);
    path_json["mow_status"] = static_cast<int>(item.mow_status);
    path_json["current_pose_index"] = item.current_pose_index;
    path_json["slicer_source"]["path_id"] = item.slicer_source.path_id;
    path_json["slicer_source"]["path_type"] = static_cast<int>(item.slicer_source.path.path_type);
    path_json["slicer_source"]["is_outline"] = static_cast<int>(item.slicer_source.path.is_outline);
    path_json["slicer_source"]["frame_id"] = item.slicer_source.path.path.header.frame_id;
    path_json["slicer_source"]["poses"] = json::array();
    for (const auto& pose : item.slicer_source.path.path.poses) {
      path_json["slicer_source"]["poses"].push_back(pose_to_json(pose));
    }
    path_json["execution"]["order"] = item.order;
    path_json["execution"]["path_direction"] = static_cast<int>(item.path_direction);
    path_json["execution"]["rotation_offset"] = item.execution.rotation_offset;
    path_json["execution"]["transform_flags"] = string_vector_to_json(item.execution.transform_flags);
    path_json["execution"]["path"]["frame_id"] = item.execution.path.path.header.frame_id;
    path_json["execution"]["path"]["poses"] = json::array();
    for (const auto& pose : item.execution.path.path.poses) {
      path_json["execution"]["path"]["poses"].push_back(pose_to_json(pose));
    }
    root["paths"].push_back(path_json);
  }

  const std::string tmp = currentMowingPlan.plan_file + ".tmp";
  std::ofstream out(tmp);
  if (!out.good()) {
    ROS_WARN_STREAM("MowingBehavior: Could not open mowing plan snapshot for writing: " << tmp);
    return false;
  }
  out << root.dump(2);
  out.close();
  if (std::rename(tmp.c_str(), currentMowingPlan.plan_file.c_str()) != 0) {
    ROS_WARN_STREAM("MowingBehavior: Could not finalize mowing plan snapshot: " << currentMowingPlan.plan_file);
    return false;
  }
  ROS_INFO_STREAM("MowingBehavior: Saved mowing plan snapshot: " << currentMowingPlan.plan_file);
  return true;
}

Behavior* MowingBehavior::execute() {
  shared_state->active_semiautomatic_task = true;

  if (!mowingAreaQueueInitialized && !build_mowing_area_queue()) {
    ROS_ERROR_STREAM("MowingBehavior: Could not build requested mowing area queue, docking");
    clear_direct_mowing_area_request();
    reset();
    return &DockingBehavior::INSTANCE;
  }

  while (ros::ok() && !aborted) {
    if (currentMowingPlan.paths.empty() && !create_mowing_plan(currentMowingArea)) {
      ROS_INFO_STREAM("MowingBehavior: Could not create mowing plan, docking");
      // Start again from first area next time.
      reset();
      // We cannot create a plan, so we're probably done. Go to docking station
      return &DockingBehavior::INSTANCE;
    }

    // No plan will be created if the area is skipped
    if (currentMowingPlan.paths.empty()) {
      mark_current_area_status(skip_area ? "skipped" : "done");
      currentMowingArea++;
      currentMowingPath = 0;
      currentMowingPathIndex = 0;
      continue;
    }

    // We have a plan, execute it
    ROS_INFO_STREAM("MowingBehavior: Executing mowing plan");
    bool finished = execute_mowing_plan();
    if (finished) {
      // skip to next area if current
      ROS_INFO_STREAM("MowingBehavior: Executing mowing plan - finished");
      mark_current_area_status(skip_area ? "skipped" : "done");
      skip_area = false;
      currentMowingArea++;
      clear_current_mowing_plan();
      currentMowingPath = 0;
      currentMowingPathIndex = 0;
      publish_mowing_progress(true);
    }
  }

  if (!ros::ok()) {
    // something went wrong
    return nullptr;
  }
  // we got aborted, go to docking station
  return &DockingBehavior::INSTANCE;
}

void MowingBehavior::enter() {
  skip_area = false;
  skip_path = false;
  paused = aborted = false;

  for (auto& a : actions) {
    a.enabled = true;
  }
  registerActions("mower_logic:mowing", actions);
  publish_mowing_progress(true);
}

void MowingBehavior::exit() {
  for (auto& a : actions) {
    a.enabled = false;
  }
  registerActions("mower_logic:mowing", actions);
}

void MowingBehavior::reset() {
  clear_current_mowing_plan();
  currentMowingArea = 0;
  currentMowingAreaQueue.clear();
  mowingAreaQueueInitialized = false;
  activeMowingAreaMode = "normal";
  currentMowingAreaQueueDigest.clear();
  clear_direct_mowing_area_request();
  currentAreaId.clear();
  checkpointAreaId.clear();
  currentMowingPath = 0;
  currentMowingPathIndex = 0;
  publish_mowing_progress(true);
  // increase cumulative mowing angle offset increment
  currentMowingAngleIncrementSum = std::fmod(currentMowingAngleIncrementSum + getConfig().mow_angle_increment, 360);
  checkpoint();

  if (config.automatic_mode == eAutoMode::SEMIAUTO) {
    ROS_INFO_STREAM("MowingBehavior: Finished semiautomatic task");
    shared_state->active_semiautomatic_task = false;
  }
}

bool MowingBehavior::needs_gps() {
  return true;
}

bool MowingBehavior::mower_enabled() {
  return mowerEnabled;
}

void MowingBehavior::update_actions() {
  for (auto& a : actions) {
    a.enabled = true;
  }

  // pause / resume switch. other actions are always available
  actions[0].enabled = !(requested_pause_flag & pauseType::PAUSE_MANUAL);
  actions[1].enabled = requested_pause_flag & pauseType::PAUSE_MANUAL;

  registerActions("mower_logic:mowing", actions);
}


std::string MowingBehavior::compute_mowing_area_queue_digest() const {
  CryptoPP::SHA256 hash;
  byte digest[CryptoPP::SHA256::DIGESTSIZE];
  for (const auto& entry : currentMowingAreaQueue) {
    hash.Update(reinterpret_cast<const byte*>(entry.area_id.data()), entry.area_id.size());
    hash.Update(reinterpret_cast<const byte*>(&entry.mowing_order), sizeof(entry.mowing_order));
  }
  hash.Final(digest);

  std::string result;
  CryptoPP::HexEncoder encoder;
  encoder.Attach(new CryptoPP::StringSink(result));
  encoder.Put(digest, sizeof(digest));
  encoder.MessageEnd();
  return "aq_" + result.substr(0, 16);
}

void MowingBehavior::mark_current_area_status(const std::string& status) {
  if (currentMowingArea >= 0 && currentMowingArea < static_cast<int>(currentMowingAreaQueue.size())) {
    currentMowingAreaQueue[currentMowingArea].status = status;
  }
}

void MowingBehavior::clear_direct_mowing_area_request() {
  directMowingAreaRequested = false;
  requestedMowingAreaId.clear();
  requestedMowingAreaMode = "normal";
}

void MowingBehavior::request_direct_mowing_area(const std::string& area_id, const std::string& mode) {
  // A direct area command is an explicit new job.  Do not let a restored
  // checkpoint/current plan from a previous run influence the selected area.
  clear_current_mowing_plan();
  currentMowingArea = 0;
  currentMowingPath = 0;
  currentMowingPathIndex = 0;
  currentMowingPlanDigest.clear();
  currentMowingAreaQueue.clear();
  currentMowingAreaQueueDigest.clear();
  currentAreaId.clear();
  checkpointAreaId.clear();

  requestedMowingAreaId = area_id;
  requestedMowingAreaMode = mode.empty() ? "single" : mode;
  if (requestedMowingAreaMode != "single" && requestedMowingAreaMode != "from_here") {
    ROS_WARN_STREAM("MowingBehavior: Unknown direct mowing mode '" << requestedMowingAreaMode
                    << "', falling back to single");
    requestedMowingAreaMode = "single";
  }
  directMowingAreaRequested = !requestedMowingAreaId.empty();
  mowingAreaQueueInitialized = false;
  ROS_INFO_STREAM("MowingBehavior: Direct mowing request: area_id=" << requestedMowingAreaId
                  << " mode=" << requestedMowingAreaMode);
}

bool MowingBehavior::build_mowing_area_queue() {
  currentMowingAreaQueue.clear();

  mower_map::GetMowingAreaListSrv listSrv;
  if (!mapAreaListClient.call(listSrv) || !listSrv.response.success) {
    ROS_WARN_STREAM("MowingBehavior: Could not load mowing area queue via list service; falling back to legacy index mode");
    mowingAreaQueueInitialized = false;
    activeMowingAreaMode = "legacy_index";
    currentMowingAreaQueueDigest.clear();
    return true;
  }

  for (const auto& info : listSrv.response.areas) {
    MowingAreaQueueEntry entry;
    entry.queue_index = static_cast<uint32_t>(currentMowingAreaQueue.size());
    entry.area_id = info.area_id;
    entry.name = info.name;
    entry.mowing_order = info.mowing_order;
    entry.status = "pending";
    currentMowingAreaQueue.push_back(entry);
  }

  if (currentMowingAreaQueue.empty()) {
    ROS_WARN_STREAM("MowingBehavior: Mowing area queue is empty");
    mowingAreaQueueInitialized = true;
    activeMowingAreaMode = directMowingAreaRequested ? requestedMowingAreaMode : "normal";
    currentMowingAreaQueueDigest = compute_mowing_area_queue_digest();
    return true;
  }

  activeMowingAreaMode = directMowingAreaRequested ? requestedMowingAreaMode : "normal";

  if (directMowingAreaRequested) {
    auto it = std::find_if(currentMowingAreaQueue.begin(), currentMowingAreaQueue.end(), [&](const MowingAreaQueueEntry& entry) {
      return entry.area_id == requestedMowingAreaId;
    });

    if (it == currentMowingAreaQueue.end()) {
      ROS_ERROR_STREAM("MowingBehavior: Requested direct mowing area is not valid or not enabled: " << requestedMowingAreaId);
      currentMowingAreaQueue.clear();
      mowingAreaQueueInitialized = true;
      currentMowingAreaQueueDigest = compute_mowing_area_queue_digest();
      return false;
    }

    if (requestedMowingAreaMode == "single") {
      MowingAreaQueueEntry selected = *it;
      selected.queue_index = 0;
      selected.status = "pending";
      currentMowingAreaQueue.clear();
      currentMowingAreaQueue.push_back(selected);
      currentMowingArea = 0;
      ROS_INFO_STREAM("MowingBehavior: Direct mowing selected single area_id=" << selected.area_id
                      << " name=" << selected.name << " order=" << selected.mowing_order);
    } else {  // from_here
      const std::size_t start = static_cast<std::size_t>(std::distance(currentMowingAreaQueue.begin(), it));
      std::vector<MowingAreaQueueEntry> trimmed;
      for (std::size_t i = start; i < currentMowingAreaQueue.size(); ++i) {
        auto entry = currentMowingAreaQueue[i];
        entry.queue_index = static_cast<uint32_t>(trimmed.size());
        entry.status = "pending";
        trimmed.push_back(entry);
      }
      currentMowingAreaQueue = trimmed;
      currentMowingArea = 0;
    }
  } else if (currentMowingArea < 0) {
    currentMowingArea = 0;
  }

  currentMowingAreaQueueDigest = compute_mowing_area_queue_digest();
  mowingAreaQueueInitialized = true;
  ROS_INFO_STREAM("MowingBehavior: Built mowing area queue with " << currentMowingAreaQueue.size()
                  << " entries, mode=" << activeMowingAreaMode);
  return true;
}

bool MowingBehavior::create_mowing_plan(int area_index) {
  ROS_INFO_STREAM("MowingBehavior: Creating mowing plan for area: " << area_index);
  // Preserve checkpoint resume metadata before clearing any stale in-RAM path list.
  const std::string checkpoint_plan_file = currentMowingPlan.plan_file;
  const std::string checkpoint_plan_id = currentMowingPlan.plan_id;
  const std::string checkpoint_plan_area_id = currentMowingPlan.area_id;
  const std::string checkpoint_plan_area_digest = currentMowingPlan.area_digest;
  const int checkpoint_current_path = currentMowingPath;
  const int checkpoint_current_path_index = currentMowingPathIndex;

  // Delete old plan and progress.
  clear_current_mowing_plan();
  currentMowingPlan.plan_file = checkpoint_plan_file;
  currentMowingPlan.plan_id = checkpoint_plan_id;
  currentMowingPlan.area_id = checkpoint_plan_area_id;
  currentMowingPlan.area_digest = checkpoint_plan_area_digest;
  currentAreaId.clear();
  currentMowingPath = checkpoint_current_path;
  currentMowingPathIndex = checkpoint_current_path_index;
  publish_mowing_progress(true);

  // get the mowing area. Prefer the new stable area_id queue, keep legacy index lookup as fallback.
  mower_map::MapArea selected_area;
  std::string selected_area_id;

  if (mowingAreaQueueInitialized) {
    if (area_index < 0 || area_index >= static_cast<int>(currentMowingAreaQueue.size())) {
      ROS_INFO_STREAM("MowingBehavior: No more mowing areas in queue at index " << area_index);
      currentAreaId.clear();
      return false;
    }

    selected_area_id = currentMowingAreaQueue[area_index].area_id;
    mower_map::GetMowingAreaByIdSrv mapByIdSrv;
    mapByIdSrv.request.area_id = selected_area_id;
    if (!mapAreaByIdClient.call(mapByIdSrv) || !mapByIdSrv.response.success) {
      ROS_ERROR_STREAM("MowingBehavior: Error loading mowing area by id: " << selected_area_id
                       << " message=" << mapByIdSrv.response.message);
      currentAreaId.clear();
      mark_current_area_status("failed");
      return false;
    }
    selected_area = mapByIdSrv.response.area;
    selected_area_id = mapByIdSrv.response.area_id;
  } else {
    mower_map::GetMowingAreaSrv mapSrv;
    mapSrv.request.index = area_index;
    if (!mapClient.call(mapSrv)) {
      ROS_ERROR_STREAM("MowingBehavior: Error loading mowing area");
      currentAreaId.clear();
      return false;
    }
    selected_area = mapSrv.response.area;
    selected_area_id = mapSrv.response.area_id;
  }

  currentAreaId = selected_area_id;
  checkpointAreaId = currentAreaId;
  mark_current_area_status("in_progress");

  if (selected_area.area.points.empty()) {
    currentAreaId.clear();
    checkpointAreaId.clear();
    mark_current_area_status("skipped");
    ROS_INFO_STREAM("MowingBehavior: Skipping inactive mowing area");
    return true;
  }

  const std::string area_digest = compute_area_digest(selected_area);

  // If checkpoint.bag points to a saved currentMowingPlan snapshot for this unchanged area,
  // load it directly into RAM and avoid re-slicing.
  if (!checkpoint_plan_file.empty() && checkpoint_plan_area_id == currentAreaId &&
      checkpoint_plan_area_digest == area_digest &&
      load_current_mowing_plan_snapshot(checkpoint_plan_file, currentAreaId, area_digest)) {
    publish_mowing_progress(true);
    return true;
  }

  // Area orientation is the same as the first point
  double angle = 0;
  auto points = selected_area.area.points;
  if (points.size() >= 2) {
    tf2::Vector3 first(points[0].x, points[0].y, 0);
    for (auto point : points) {
      tf2::Vector3 second(point.x, point.y, 0);
      auto diff = second - first;
      if (diff.length() > 2.0) {
        // we have found a point that has a distance of > 1 m, calculate the angle
        angle = atan2(diff.y(), diff.x());
        ROS_INFO_STREAM("MowingBehavior: Detected mow angle: " << angle);
        break;
      }
    }
  }

  // add mowing angle offset increment and return into the <-180, 180> range
  double mow_angle_offset = std::fmod(getConfig().mow_angle_offset + currentMowingAngleIncrementSum + 180, 360);
  if (mow_angle_offset < 0) mow_angle_offset += 360;
  mow_angle_offset -= 180;
  ROS_INFO_STREAM("MowingBehavior: mowing angle offset (deg): " << mow_angle_offset);
  if (config.mow_angle_offset_is_absolute) {
    angle = mow_angle_offset * (M_PI / 180.0);
    ROS_INFO_STREAM("MowingBehavior: Custom mowing angle: " << angle);
  } else {
    angle = angle + mow_angle_offset * (M_PI / 180.0);
    ROS_INFO_STREAM("MowingBehavior: Auto-detected mowing angle + mowing angle offset: " << angle);
  }

  // calculate coverage
  slic3r_coverage_planner::PlanPath pathSrv;
  pathSrv.request.angle = angle;
  pathSrv.request.outline_count = config.outline_count;
  pathSrv.request.outline_overlap_count = config.outline_overlap_count;
  pathSrv.request.obstacle_outline_count = config.obstacle_outline_count;
  pathSrv.request.obstacle_outline_overlap_count = config.obstacle_outline_overlap_count;
  pathSrv.request.outline_simplify_per_loop = config.outline_simplify_per_loop;
  pathSrv.request.outline_simplify_max_tolerance = config.outline_simplify_max_tolerance;
  pathSrv.request.outline_simplify_safety_factor = config.outline_simplify_safety_factor;
  pathSrv.request.outline_simplify_min_distance_factor = config.outline_simplify_min_distance_factor;
  pathSrv.request.outline_simplify_affects_next_offset = config.outline_simplify_affects_next_offset;
  pathSrv.request.outline = selected_area.area;
  pathSrv.request.holes = selected_area.obstacles;
  pathSrv.request.fill_type = slic3r_coverage_planner::PlanPathRequest::FILL_LINEAR;
  pathSrv.request.outer_offset = config.outline_offset;
  pathSrv.request.distance = config.tool_width;
  if (!pathClient.call(pathSrv)) {
    ROS_ERROR_STREAM("MowingBehavior: Error during coverage planning");
    return false;
  }

  build_current_mowing_plan(pathSrv.response.paths, currentAreaId, area_digest);

  geometry_msgs::PoseStamped current_pose;
  current_pose.header.frame_id = "map";
  current_pose.header.stamp = ros::Time::now();
  current_pose.pose = getPose().pose.pose;

  // Optional path order optimization now operates on the currentMowingPlan identity layer.
  // It may change order/path_direction, but slicer_source.path stays unchanged.
  if (!optimize_current_mowing_plan(current_pose)) {
    return false;
  }

  save_current_mowing_plan();

  // Re-apply the checkpoint position after build/optional POO reset the plan to the first path.
  currentMowingPath = checkpoint_current_path;
  currentMowingPathIndex = checkpoint_current_path_index;

  publish_mowing_progress(true);

  // Calculate mowing plan digest from the poses
  // TODO: move to slic3r_coverage_planner
  CryptoPP::SHA256 hash;
  byte digest[CryptoPP::SHA256::DIGESTSIZE];
  for (const auto& item : currentMowingPlan.paths) {
    hash.Update(reinterpret_cast<const byte*>(&item.slicer_source.path_id), sizeof(item.slicer_source.path_id));
    hash.Update(reinterpret_cast<const byte*>(&item.path_direction), sizeof(item.path_direction));
    hash.Update(reinterpret_cast<const byte*>(&item.execution.rotation_offset), sizeof(item.execution.rotation_offset));
    for (const auto& pose_stamped : item.execution.path.path.poses) {
      hash.Update(reinterpret_cast<const byte*>(&pose_stamped.pose), sizeof(geometry_msgs::Pose));
    }
  }
  hash.Final((byte*)&digest[0]);
  CryptoPP::HexEncoder encoder;
  std::string mowingPlanDigest = "";
  encoder.Attach(new CryptoPP::StringSink(mowingPlanDigest));
  encoder.Put(digest, sizeof(digest));
  encoder.MessageEnd();

  // Proceed to checkpoint?
  if (mowingPlanDigest == currentMowingPlanDigest) {
    ROS_INFO_STREAM("MowingBehavior: Advancing to checkpoint, path: " << currentMowingPath
                                                                      << " index: " << currentMowingPathIndex);
  } else {
    ROS_INFO_STREAM("MowingBehavior: Ignoring checkpoint for plan ("
                    << currentMowingPlanDigest << ") current mowing plan is (" << mowingPlanDigest << ")");
    // Plan has changed so must restart the area
    currentMowingPlanDigest = mowingPlanDigest;
    currentMowingPath = 0;
    currentMowingPathIndex = 0;
  }

  for (std::size_t i = 0; i < currentMowingPlan.paths.size(); ++i) {
    auto& item = currentMowingPlan.paths[i];
    if (static_cast<int>(i) < currentMowingPath) {
      item.mow_status = MOW_STATUS_DONE;
      const auto pose_count = item.execution.path.path.poses.size();
      item.current_pose_index = pose_count == 0 ? 0 : static_cast<uint32_t>(pose_count - 1);
    } else if (static_cast<int>(i) == currentMowingPath) {
      item.mow_status = MOW_STATUS_IN_PROGRESS;
      item.current_pose_index = static_cast<uint32_t>(std::max(currentMowingPathIndex, 0));
    } else {
      item.mow_status = MOW_STATUS_OPEN;
      item.current_pose_index = 0;
    }
  }
  start_current_mowing_plan_path();

  // Persist the freshly created and selected mowing plan immediately.
  // This makes checkpoint.bag contain the active area_id and plan metadata before
  // the mower reaches the first periodic checkpoint during path execution.
  checkpoint();
  ROS_INFO_STREAM("MowingBehavior: Saved checkpoint after successful mowing plan creation for area_id="
                  << currentAreaId << " plan_id=" << currentMowingPlan.plan_id);

  publish_mowing_progress(true);
  return true;
}

int getCurrentMowPathIndex() {
  ftc_local_planner::PlannerGetProgress progressSrv;
  int currentIndex = -1;
  if (pathProgressClient.call(progressSrv)) {
    currentIndex = progressSrv.response.index;
  } else {
    ROS_ERROR("MowingBehavior: getMowIndex() - Error getting progress from FTC planner");
  }
  return (currentIndex);
}

void printNavState(int state) {
  switch (state) {
    case actionlib::SimpleClientGoalState::PENDING: ROS_INFO(">>> State: Pending <<<"); break;
    case actionlib::SimpleClientGoalState::ACTIVE: ROS_INFO(">>> State: Active <<<"); break;
    case actionlib::SimpleClientGoalState::RECALLED: ROS_INFO(">>> State: Recalled <<<"); break;
    case actionlib::SimpleClientGoalState::REJECTED: ROS_INFO(">>> State: Rejected <<<"); break;
    case actionlib::SimpleClientGoalState::PREEMPTED: ROS_INFO(">>> State: Preempted <<<"); break;
    case actionlib::SimpleClientGoalState::ABORTED: ROS_INFO(">>> State: Aborted <<<"); break;
    case actionlib::SimpleClientGoalState::SUCCEEDED: ROS_INFO(">>> State: Succeeded <<<"); break;
    case actionlib::SimpleClientGoalState::LOST: ROS_INFO(">>> State: Lost <<<"); break;
    default: ROS_INFO(">>> State: Unknown Hu ? <<<"); break;
  }
}

bool MowingBehavior::execute_mowing_plan() {
  int first_point_attempt_counter = 0;
  int first_point_trim_counter = 0;
  ros::Time paused_time(0.0);

  // loop through all mowingPaths to execute the plan fully.
  while (currentMowingPath < currentMowingPlan.paths.size() && ros::ok() && !aborted) {
    ////////////////////////////////////////////////
    // PAUSE HANDLING
    ////////////////////////////////////////////////
    if (requested_pause_flag) {  // pause was requested
      paused = true;
      mowerEnabled = false;
      u_int8_t last_requested_pause_flags = 0;
      while (requested_pause_flag && !aborted)  // while emergency and/or manual pause not asked to continue, we wait
      {
        if (last_requested_pause_flags != requested_pause_flag) {
          update_actions();
        }
        last_requested_pause_flags = requested_pause_flag;

        std::string pause_reason = "";
        if (requested_pause_flag & pauseType::PAUSE_EMERGENCY) {
          pause_reason += "on EMERGENCY";
          if (requested_pause_flag & pauseType::PAUSE_MANUAL) {
            pause_reason += " and ";
          }
        }
        if (requested_pause_flag & pauseType::PAUSE_MANUAL) {
          pause_reason += "waiting for CONTINUE";
        }
        ROS_INFO_STREAM_THROTTLE(30, "MowingBehavior: PAUSED (" << pause_reason << ")");
        ros::Rate r(1.0);
        r.sleep();
      }
      // we will drop into paused, thus will also wait for GPS to be valid again
    }
    if (paused) {
      paused_time = ros::Time::now();
      while (!this->hasGoodGPS() && !aborted)  // while no good GPS we wait
      {
        ROS_INFO_STREAM("MowingBehavior: PAUSED (" << (ros::Time::now() - paused_time).toSec()
                                                   << "s) (waiting for GPS)");
        ros::Rate r(1.0);
        r.sleep();
      }
      ROS_INFO_STREAM("MowingBehavior: CONTINUING");
      paused = false;
      update_actions();
    }

    auto& item = currentMowingPlan.paths[currentMowingPath];
    auto& path = item.execution.path;
    start_current_mowing_plan_path();
    ROS_INFO_STREAM("MowingBehavior: Path segment length: " << path.path.poses.size() << " poses.");

    // Check if path is empty. If so, directly skip it
    if (currentMowingPathIndex >= static_cast<int>(path.path.poses.size())) {
      ROS_INFO_STREAM("MowingBehavior: Skipping empty path.");
      finish_current_mowing_plan_path();
      currentMowingPath++;
      currentMowingPathIndex = 0;
      start_current_mowing_plan_path();
      continue;
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    // DRIVE TO THE FIRST POINT OF THE MOW PATH
    //
    // * we have n attempts, if we fail we go to pause() mode because most likely it was GPS problems that
    //   prevented us from reaching the inital pose
    // * after n attempts, we fail the mow area and skip to the next one
    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    {
      ROS_INFO_STREAM("MowingBehavior: (FIRST POINT)  Moving to path segment starting point");
      if (path.is_outline && getConfig().add_fake_obstacle) {
        mower_map::SetNavPointSrv set_nav_point_srv;
        set_nav_point_srv.request.nav_pose = execution_pose(item, currentMowingPathIndex).pose;
        setNavPointClient.call(set_nav_point_srv);
        sleep(1);
      }

      mbf_msgs::MoveBaseGoal moveBaseGoal;
      moveBaseGoal.target_pose = execution_pose(item, currentMowingPathIndex);
      moveBaseGoal.controller = "FTCPlanner";
      mbfClient->sendGoal(moveBaseGoal);
      sleep(1);
      actionlib::SimpleClientGoalState current_status(actionlib::SimpleClientGoalState::PENDING);
      ros::Rate r(10);

      // wait for path execution to finish
      while (ros::ok()) {
        current_status = mbfClient->getState();
        if (current_status.state_ == actionlib::SimpleClientGoalState::ACTIVE ||
            current_status.state_ == actionlib::SimpleClientGoalState::PENDING) {
          // path is being executed, everything seems fine.
          // check if we should pause or abort mowing
          if (skip_area) {
            ROS_INFO_STREAM("MowingBehavior: (FIRST POINT) SKIP AREA was requested.");
            // remove all paths in current area and return true
            mowerEnabled = false;
            mbfClientExePath->cancelAllGoals();
            clear_current_mowing_plan();
            skip_area = false;
            return true;
          }
          if (skip_path) {
            ROS_INFO_STREAM("MowingBehavior: (FIRST POINT) SKIP PATH was requested.");
            mbfClient->cancelAllGoals();
            mowerEnabled = false;
            skip_path = false;
            finish_current_mowing_plan_path();
            currentMowingPath++;
            currentMowingPathIndex = 0;
            start_current_mowing_plan_path();
            checkpoint();
            publish_mowing_progress(true);
            return false;
          }
          if (aborted) {
            ROS_INFO_STREAM("MowingBehavior: (FIRST POINT) ABORT was requested - stopping path execution.");
            mbfClientExePath->cancelAllGoals();
            mowerEnabled = false;
            return false;
          }
          if (requested_pause_flag) {
            ROS_INFO_STREAM("MowingBehavior: (FIRST POINT) PAUSE was requested - stopping path execution.");
            mbfClientExePath->cancelAllGoals();
            mowerEnabled = false;
            return false;
          }
        } else {
          ROS_INFO_STREAM("MowingBehavior: (FIRST POINT)  Got status "
                          << current_status.state_ << " from MBF/FTCPlanner -> Stopping path execution.");
          // we're done, break out of the loop
          break;
        }
        r.sleep();
      }

      first_point_attempt_counter++;
      if (current_status.state_ != actionlib::SimpleClientGoalState::SUCCEEDED) {
        // we cannot reach the start point
        ROS_ERROR_STREAM("MowingBehavior: (FIRST POINT) - Could not reach goal (first point). Planner Status was: "
                         << current_status.state_);
        // we have 3 attempts to get to the start pose of the mowing area
        if (first_point_attempt_counter < config.max_first_point_attempts) {
          ROS_WARN_STREAM("MowingBehavior: (FIRST POINT) - Attempt " << first_point_attempt_counter << " / "
                                                                     << config.max_first_point_attempts
                                                                     << " Making a little pause ...");
          paused = true;
          update_actions();
        } else {
          // We failed to reach the first point in the mow path by simply repeating the drive to process
          // So now we will trim the path by removing the first pose
          if (first_point_trim_counter < config.max_first_point_trim_attempts) {
            // We try now to remove the first point so the 2nd, 3rd etc point becomes our target
            // mow path points are offset by 10cm
            ROS_WARN_STREAM("MowingBehavior: (FIRST POINT) - Attempt "
                            << first_point_trim_counter << " / " << config.max_first_point_trim_attempts
                            << " Trimming first point off the beginning of the mow path.");
            currentMowingPathIndex++;
            first_point_trim_counter++;
            first_point_attempt_counter = 0;  // give it another <config.max_first_point_attempts> attempts
            paused = true;
            update_actions();
          } else {
            // Unable to reach the start of the mow path (we tried multiple attempts for the same point, and we skipped
            // points which also didnt work, time to give up)
            ROS_ERROR_STREAM(
                "MowingBehavior: (FIRST POINT) Max retries reached, we are unable to reach any of the first points - "
                "aborting at index: "
                << currentMowingPathIndex << " path: " << currentMowingPath << " area: " << currentMowingArea);
            this->abort();
          }
        }
        continue;
      }

      mower_map::ClearNavPointSrv clear_nav_point_srv;
      clearNavPointClient.call(clear_nav_point_srv);

      // we have reached the start pose of the mow area, reset error handling values
      first_point_attempt_counter = 0;
      first_point_trim_counter = 0;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Execute the path segment and either drop it if we finished it successfully or trim it if we were aborted
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    {
      // enable mower (only when we reach the start not on the way to mowing already)
      mowerEnabled = true;

      mbf_msgs::ExePathGoal exePathGoal;
      nav_msgs::Path exePath;
      exePath = execution_path_from_index(item, currentMowingPathIndex);
      int exePathStartIndex = currentMowingPathIndex;
      exePathGoal.path = exePath;
      exePathGoal.angle_tolerance = 5.0 * (M_PI / 180.0);
      exePathGoal.dist_tolerance = 0.2;
      exePathGoal.tolerance_from_action = true;
      exePathGoal.controller = "FTCPlanner";

      ROS_INFO_STREAM("MowingBehavior: (MOW) First point reached - Executing mow path with "
                      << path.path.poses.size() << " poses, from index " << exePathStartIndex);
      mbfClientExePath->sendGoal(exePathGoal);
      sleep(1);
      actionlib::SimpleClientGoalState current_status(actionlib::SimpleClientGoalState::PENDING);
      ros::Rate r(10);

      // wait for path execution to finish
      while (ros::ok()) {
        current_status = mbfClientExePath->getState();
        if (current_status.state_ == actionlib::SimpleClientGoalState::ACTIVE ||
            current_status.state_ == actionlib::SimpleClientGoalState::PENDING) {
          // path is being executed, everything seems fine.
          // check if we should pause or abort mowing
          if (skip_area) {
            ROS_INFO_STREAM("MowingBehavior: (MOW) SKIP AREA was requested.");
            // remove all paths in current area and return true
            mowerEnabled = false;
            clear_current_mowing_plan();
            skip_area = false;
            return true;
          }
          if (skip_path) {
            ROS_INFO_STREAM("MowingBehavior: (MOW) SKIP PATH was requested.");
            mbfClientExePath->cancelAllGoals();
            mowerEnabled = false;
            skip_path = false;
            finish_current_mowing_plan_path();
            currentMowingPath++;
            currentMowingPathIndex = 0;
            start_current_mowing_plan_path();
            checkpoint();
            publish_mowing_progress(true);
            return false;
          }
          if (aborted) {
            ROS_INFO_STREAM("MowingBehavior: (MOW) ABORT was requested - stopping path execution.");
            mbfClientExePath->cancelAllGoals();
            mowerEnabled = false;
            break;  // Trim path
          }
          if (requested_pause_flag) {
            ROS_INFO_STREAM("MowingBehavior: (MOW) PAUSE was requested - stopping path execution.");
            mbfClientExePath->cancelAllGoals();
            mowerEnabled = false;
            break;  // Trim path
          }
          if (current_status.state_ == actionlib::SimpleClientGoalState::ACTIVE) {
            // show progress
            int currentIndex = getCurrentMowPathIndex();
            if (currentIndex != -1) {
              currentMowingPathIndex = exePathStartIndex + currentIndex;
              update_current_mowing_plan_progress();
              // Keep live pose updates prioritized: publish only the small status payload regularly,
              // and throttle the heavy path geometry payload aggressively.
              publish_mowing_progress_status(false);
              publish_mowing_progress(false);
            }
            ROS_INFO_STREAM_THROTTLE(
                5, "MowingBehavior: (MOW) Progress: " << currentMowingPathIndex << "/" << path.path.poses.size());
            if (ros::Time::now() - last_checkpoint > ros::Duration(30.0)) checkpoint();
          }
        } else {
          ROS_INFO_STREAM("MowingBehavior: (MOW)  Got status " << current_status.state_
                                                               << " from MBF/FTCPlanner -> Stopping path execution.");
          // we're done, break out of the loop
          break;
        }
        r.sleep();
      }

      // Only skip/trim if goal execution began
      if (current_status.state_ != actionlib::SimpleClientGoalState::PENDING &&
          current_status.state_ != actionlib::SimpleClientGoalState::RECALLED) {
        ROS_INFO_STREAM(">> MowingBehavior: (MOW) PlannerGetProgress currentMowingPathIndex = "
                        << currentMowingPathIndex << " of " << path.path.poses.size());
        printNavState(current_status.state_);
        // if we have fully processed the segment or we have encountered an error, drop the path segment
        /* TODO: we can not trust the SUCCEEDED state because the planner sometimes says suceeded with
            the currentIndex far from the size of the poses ! (BUG in planner ?)
            instead we trust only the currentIndex vs. poses.size() */
        if (currentMowingPathIndex >= path.path.poses.size() ||
            (path.path.poses.size() - currentMowingPathIndex) < 5)  // fully mowed the path ?
        {
          ROS_INFO_STREAM("MowingBehavior: (MOW) Mow path finished, skipping to next mow path.");
          finish_current_mowing_plan_path();
          currentMowingPath++;
          currentMowingPathIndex = 0;
          start_current_mowing_plan_path();
          publish_mowing_progress(true);
          // continue with next segment
        } else {
          // we didnt drive all points in the mow path, so we go into pause mode
          // TODO: we should figure out the likely reason for our failure to complete the path
          // if GPS -> PAUSE
          // if something else -> Recovery Behaviour ?

          // currentMowingPathIndex might be 0 if we never consumed one of the points, we advance at least 1 point
          if (currentMowingPathIndex == 0) currentMowingPathIndex++;
          update_current_mowing_plan_progress();
          publish_mowing_progress(true);
          if (!requested_pause_flag) {
            ROS_INFO_STREAM("MowingBehavior: (MOW) PAUSED due to MBF Error at " << currentMowingPathIndex);
            paused = true;
            update_actions();
          }
        }
      }
    }
  }

  mowerEnabled = false;
  publish_mowing_progress(true);

  // true, if we have executed all paths
  return currentMowingPath >= currentMowingPlan.paths.size();
}


void MowingBehavior::ensure_mowing_progress_interface() {
  if (mowing_progress_interface_initialized || n == nullptr) return;

  // Heavy retained geometry snapshot for map overlays. Contains all path geometries, but no mow_status.
  mowing_progress_pub = n->advertise<std_msgs::String>("/mower_logic/map/mowing_progress/json", 1, true);

  // Lightweight retained status snapshot for frequent UI progress updates. Contains all path states, but no geometry.
  // This prevents large mowing-progress MQTT messages from delaying robot_state/json pose updates.
  mowing_progress_status_pub = n->advertise<std_msgs::String>("/mower_logic/map/mowing_progress/status/json", 1, true);

  mowing_progress_renew_sub = n->subscribe("/mower_logic/map/mowing_progress/renew", 10,
                                           &MowingBehavior::mowing_progress_renew_callback, this);
  last_mowing_progress_publish = ros::Time(0.0);
  last_mowing_progress_status_publish = ros::Time(0.0);
  mowing_progress_interface_initialized = true;
}

void MowingBehavior::mowing_progress_renew_callback(const std_msgs::Empty::ConstPtr&) {
  publish_mowing_progress(true);
}

json MowingBehavior::build_mowing_progress_payload(bool include_paths) {
  const ros::Time now = ros::Time::now();

  json payload;
  payload["timestamp"] = now.toSec();
  payload["frame_id"] = "map";
  payload["current_area_id"] = currentAreaId;
  payload["current_area_queue_index"] = currentMowingArea;
  payload["mowing_area_mode"] = activeMowingAreaMode;
  payload["area_queue_digest"] = currentMowingAreaQueueDigest;
  payload["area_queue"] = json::array();
  for (const auto& entry : currentMowingAreaQueue) {
    payload["area_queue"].push_back({
        {"queue_index", entry.queue_index},
        {"area_id", entry.area_id},
        {"name", entry.name},
        {"mowing_order", entry.mowing_order},
        {"status", entry.status}});
  }
  payload["processing_mode"] = static_cast<int>(currentMowingPlan.processing_mode);
  payload["outline_entry_mode"] = static_cast<int>(currentMowingPlan.outline_entry_mode);
  payload["areas"] = json::object();

  if (!currentAreaId.empty()) {
    json area;
    area["area_id"] = currentAreaId;
    area["state"] = currentMowingPlan.paths.empty() ? "pending" :
        (currentMowingPath >= static_cast<int>(currentMowingPlan.paths.size()) ? "done" : (paused ? "paused" : "mowing"));

    area["paths"] = json::array();

    std::size_t total_points = 0;
    std::size_t completed_points = 0;

    for (std::size_t path_index = 0; path_index < currentMowingPlan.paths.size(); ++path_index) {
      const auto& item = currentMowingPlan.paths[path_index];
      const auto& poses = item.execution.path.path.poses;
      total_points += poses.size();

      std::size_t completed_for_path = 0;
      if (item.mow_status == MOW_STATUS_DONE) {
        completed_for_path = poses.size();
      } else if (item.mow_status == MOW_STATUS_IN_PROGRESS) {
        completed_for_path = std::min<std::size_t>(item.current_pose_index, poses.size());
      }

      const double completed_percent = poses.empty() ? 0.0 :
          std::min(100.0, 100.0 * static_cast<double>(completed_for_path) / static_cast<double>(poses.size()));

      completed_points += completed_for_path;

      json path;
      path["path_id"] = item.path_id;

      if (include_paths) {
        // Heavy retained geometry snapshot: stable path identity and geometry only.
        // The app should store this as the geometry reference and wait for the
        // small status snapshot before drawing path state.
        path["order"] = item.order;
        path["slicer_source"]["path_id"] = item.slicer_source.path_id;
        path["path_direction"] = path_direction_to_json_string(item.path_direction);
        path["execution"]["order"] = item.order;
        path["execution"]["path_direction"] = path_direction_to_json_string(item.path_direction);
        path["execution"]["rotation_offset"] = item.execution.rotation_offset;
        path["execution"]["transform_flags"] = string_vector_to_json(item.execution.transform_flags);
        path["points"] = path_points_to_json(item, 0, poses.size());
      } else {
        // Lightweight retained status snapshot: all path states, no heavy geometry.
        // The app can join this with map/mowing_progress/json via area_id + path_id.
        path["mow_status"] = mow_status_to_json_string(item.mow_status);
        path["current_pose_index"] = item.current_pose_index;
        path["completed_percent"] = completed_percent;
      }

      area["paths"].push_back(path);
    }

    area["percent"] = total_points == 0 ? 0.0 :
        std::min(100.0, 100.0 * static_cast<double>(completed_points) / static_cast<double>(total_points));

    if (!include_paths) {
      area["current_path"] = currentMowingPath;
      area["current_path_index"] = currentMowingPathIndex;
      if (currentMowingPath >= 0 && currentMowingPath < static_cast<int>(currentMowingPlan.paths.size())) {
        area["current_path_id"] = currentMowingPlan.paths[currentMowingPath].path_id;
      } else {
        area["current_path_id"] = "";
      }
    }

    payload["areas"][currentAreaId] = area;
  }

  return payload;
}

void MowingBehavior::publish_mowing_progress(bool force) {
  ensure_mowing_progress_interface();
  if (!mowing_progress_interface_initialized) return;

  const ros::Time now = ros::Time::now();

  // This is the heavy geometry payload. Keep it retained, but do not publish it frequently while mowing,
  // otherwise it can block/delay robot_state/json pose updates on the MQTT connection.
  if (!force) {
    if ((now - last_mowing_progress_publish) < ros::Duration(30.0)) {
      return;
    }
    if (currentMowingPath == last_published_mowing_path &&
        currentMowingPathIndex == last_published_mowing_path_index &&
        (now - last_mowing_progress_publish) < ros::Duration(120.0)) {
      return;
    }
  }

  std_msgs::String msg;
  msg.data = build_mowing_progress_payload(true).dump(2);
  mowing_progress_pub.publish(msg);
  last_mowing_progress_publish = now;
  last_published_mowing_path = currentMowingPath;
  last_published_mowing_path_index = currentMowingPathIndex;

  // Always update the small status payload after a forced full update, so the app has a fresh percentage too.
  if (force) {
    publish_mowing_progress_status(true);
  }
}

void MowingBehavior::publish_mowing_progress_status(bool force) {
  ensure_mowing_progress_interface();
  if (!mowing_progress_interface_initialized) return;

  const ros::Time now = ros::Time::now();
  if (!force) {
    if (currentMowingPath == last_published_mowing_status_path &&
        currentMowingPathIndex == last_published_mowing_status_path_index &&
        (now - last_mowing_progress_status_publish) < ros::Duration(2.0)) {
      return;
    }
    if ((now - last_mowing_progress_status_publish) < ros::Duration(1.0)) {
      return;
    }
  }

  std_msgs::String msg;
  msg.data = build_mowing_progress_payload(false).dump();
  mowing_progress_status_pub.publish(msg);
  last_mowing_progress_status_publish = now;
  last_published_mowing_status_path = currentMowingPath;
  last_published_mowing_status_path_index = currentMowingPathIndex;
}

void MowingBehavior::command_home() {
  if (shared_state->active_semiautomatic_task) {
    // We are in semiautomatic task, mark it as manually paused.
    ROS_INFO_STREAM("Manually pausing semiautomatic task");
    auto config = getConfig();
    config.manual_pause_mowing = true;
    setConfig(config);
  }
  if (paused) {
    // Request continue to wait for odom
    this->requestContinue();
    // Then instantly abort i.e. go to dock.
  }
  this->abort();
}

void MowingBehavior::command_start() {
  ROS_INFO_STREAM("MowingBehavior: MANUAL CONTINUE");
  auto config = getConfig();
  if (shared_state->active_semiautomatic_task && config.manual_pause_mowing) {
    // We are in semiautomatic task and paused, user wants to resume, so store that immediately.
    // This way, once we are docked the mower will continue as soon as all other conditions are g2g
    ROS_INFO_STREAM("Resuming semiautomatic task");
    config.manual_pause_mowing = false;
    setConfig(config);
  }
  this->requestContinue();
}

void MowingBehavior::command_s1() {
  ROS_INFO_STREAM("MowingBehavior: MANUAL PAUSED");
  this->requestPause();
}

void MowingBehavior::command_s2() {
  skip_area = true;
}

bool MowingBehavior::redirect_joystick() {
  return false;
}

uint8_t MowingBehavior::get_sub_state() {
  return 0;
}

uint8_t MowingBehavior::get_state() {
  return mower_msgs::HighLevelStatus::HIGH_LEVEL_STATE_AUTONOMOUS;
}

int16_t MowingBehavior::get_current_area() {
  return currentMowingArea;
}

std::string MowingBehavior::get_current_area_id() {
  return currentAreaId;
}

std::string MowingBehavior::get_checkpoint_area_id() {
  return checkpointAreaId;
}

int16_t MowingBehavior::get_current_path() {
  return currentMowingPath;
}

int16_t MowingBehavior::get_current_path_index() {
  return currentMowingPathIndex;
}

MowingBehavior::MowingBehavior() {
  last_checkpoint = ros::Time(0.0);
  xbot_msgs::ActionInfo pause_action;
  pause_action.action_id = "pause";
  pause_action.enabled = false;
  pause_action.action_name = "Pause Mowing";

  xbot_msgs::ActionInfo continue_action;
  continue_action.action_id = "continue";
  continue_action.enabled = false;
  continue_action.action_name = "Continue Mowing";

  xbot_msgs::ActionInfo abort_mowing_action;
  abort_mowing_action.action_id = "abort_mowing";
  abort_mowing_action.enabled = false;
  abort_mowing_action.action_name = "Stop Mowing";

  xbot_msgs::ActionInfo skip_area_action;
  skip_area_action.action_id = "skip_area";
  skip_area_action.enabled = false;
  skip_area_action.action_name = "Skip Area";

  xbot_msgs::ActionInfo skip_path_action;
  skip_path_action.action_id = "skip_path";
  skip_path_action.enabled = false;
  skip_path_action.action_name = "Skip Path";

  actions.clear();
  actions.push_back(pause_action);
  actions.push_back(continue_action);
  actions.push_back(abort_mowing_action);
  actions.push_back(skip_area_action);
  actions.push_back(skip_path_action);
  restore_checkpoint();
}

void MowingBehavior::handle_action(std::string action) {
  if (action == "mower_logic:mowing/pause") {
    ROS_INFO_STREAM("got pause command");
    this->requestPause();
  } else if (action == "mower_logic:mowing/continue") {
    ROS_INFO_STREAM("got continue command");
    this->requestContinue();
  } else if (action == "mower_logic:mowing/abort_mowing") {
    ROS_INFO_STREAM("got abort mowing command");
    command_home();
  } else if (action == "mower_logic:mowing/skip_area") {
    ROS_INFO_STREAM("got skip_area command");
    skip_area = true;
  } else if (action == "mower_logic:mowing/skip_path") {
    ROS_INFO_STREAM("got skip_path command");
    skip_path = true;
  }
  update_actions();
}

void MowingBehavior::checkpoint() {
  rosbag::Bag bag;
  mower_logic::CheckPoint cp;
  cp.currentMowingPath = currentMowingPath;
  cp.currentMowingArea = currentMowingArea;
  cp.currentMowingPathIndex = currentMowingPathIndex;
  cp.currentMowingPlanDigest = currentMowingPlanDigest;
  cp.currentMowingAngleIncrementSum = currentMowingAngleIncrementSum;
  cp.currentMowingAreaId = currentAreaId;
  cp.area_id = currentMowingPlan.area_id;
  cp.area_digest = currentMowingPlan.area_digest;
  cp.plan_id = currentMowingPlan.plan_id;
  cp.plan_file = currentMowingPlan.plan_file;
  cp.current_path_id = currentMowingPlan.current_path_id;
  cp.current_order = static_cast<int32_t>(currentMowingPlan.current_order);
  cp.current_pose_index = currentMowingPathIndex;
  cp.last_completed_order = currentMowingPath > 0 ? currentMowingPath - 1 : -1;
  cp.saved_at = ros::Time::now();
  cp.mowing_area_mode = activeMowingAreaMode;
  cp.area_queue_digest = currentMowingAreaQueueDigest;
  cp.current_area_queue_index = currentMowingArea;
  checkpointAreaId = currentAreaId;
  bag.open("checkpoint.bag", rosbag::bagmode::Write);
  bag.write("checkpoint", ros::Time::now(), cp);
  bag.close();
  last_checkpoint = ros::Time::now();
}

bool MowingBehavior::restore_checkpoint() {
  // Default to a clean start. This also makes old/incompatible checkpoint.bag files safe
  // after CheckPoint.msg changes: if no compatible message can be instantiated, these
  // values remain in effect.
  clear_current_mowing_plan();
  currentMowingArea = 0;
  currentMowingAreaQueue.clear();
  mowingAreaQueueInitialized = false;
  activeMowingAreaMode = "normal";
  currentMowingAreaQueueDigest.clear();
  currentAreaId.clear();
  checkpointAreaId.clear();
  currentMowingPath = 0;
  currentMowingPathIndex = 0;
  currentMowingAngleIncrementSum = 0;

  rosbag::Bag bag;
  bool found = false;
  try {
    bag.open("checkpoint.bag");
  } catch (rosbag::BagIOException& e) {
    // Checkpoint does not exist or is corrupt, start at the very beginning.
    return false;
  }
  {
    rosbag::View view(bag, rosbag::TopicQuery("checkpoint"));
    for (rosbag::MessageInstance const m : view) {
      auto cp = m.instantiate<mower_logic::CheckPoint>();
      if (cp) {
        ROS_INFO_STREAM("Restoring checkpoint for plan ("
                        << cp->currentMowingPlanDigest << ")"
                        << " area: " << cp->currentMowingArea
                        << " area_id: " << cp->currentMowingAreaId << " path: " << cp->currentMowingPath
                        << " index: " << cp->currentMowingPathIndex
                        << " plan_id: " << cp->plan_id
                        << " current_path_id: " << cp->current_path_id
                        << " angle increment sum: " << cp->currentMowingAngleIncrementSum);
        currentMowingPath = cp->currentMowingPath;
        currentMowingArea = cp->currentMowingArea;
        checkpointAreaId = cp->currentMowingAreaId;
        currentMowingPathIndex = cp->currentMowingPathIndex;
        currentMowingPlanDigest = cp->currentMowingPlanDigest;
        currentMowingAngleIncrementSum = cp->currentMowingAngleIncrementSum;
        currentMowingPlan.area_id = cp->area_id;
        currentMowingPlan.area_digest = cp->area_digest;
        currentMowingPlan.plan_id = cp->plan_id;
        currentMowingPlan.plan_file = cp->plan_file;
        currentMowingPlan.current_path_id = cp->current_path_id;
        currentMowingPlan.current_order = cp->current_order >= 0 ? static_cast<uint32_t>(cp->current_order) : 0;
        activeMowingAreaMode = cp->mowing_area_mode.empty() ? "normal" : cp->mowing_area_mode;
        currentMowingAreaQueueDigest = cp->area_queue_digest;
        if (cp->current_area_queue_index >= 0) currentMowingArea = cp->current_area_queue_index;
        found = true;
        break;
      }
    }
    bag.close();
  }
  return found;
}
