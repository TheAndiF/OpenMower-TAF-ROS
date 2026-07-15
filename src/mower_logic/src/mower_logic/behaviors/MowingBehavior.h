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
#ifndef SRC_MOWINGBEHAVIOR_H
#define SRC_MOWINGBEHAVIOR_H

#include <cstdint>
#include <nlohmann/json.hpp>
#include <string>

#include "Behavior.h"
#include "UndockingBehavior.h"
#include "ftc_local_planner/PlannerGetProgress.h"
#include "slic3r_coverage_planner/Path.h"
#include "slic3r_coverage_planner/PlanPath.h"
#include "std_msgs/Empty.h"
#include "std_msgs/String.h"
#include "xbot_msgs/ActionInfo.h"

class MowingBehavior : public Behavior {
 private:
  std::vector<xbot_msgs::ActionInfo> actions;

  bool skip_area;
  bool skip_path;

  struct MowingAreaQueueEntry {
    uint32_t queue_index = 0;
    std::string area_id;
    std::string name;
    int32_t mowing_order = 0;
    std::string status = "pending";
  };

  std::vector<MowingAreaQueueEntry> currentMowingAreaQueue;
  bool mowingAreaQueueInitialized = false;
  bool directMowingAreaRequested = false;
  std::string requestedMowingAreaId;
  std::string requestedMowingAreaMode = "normal";
  std::string activeMowingAreaMode = "normal";
  std::string currentMowingAreaQueueDigest;

  bool build_mowing_area_queue();
  bool create_mowing_plan(int area_index);
  void mark_current_area_status(const std::string& status);
  std::string compute_mowing_area_queue_digest() const;

  bool execute_mowing_plan();

 public:
  static constexpr uint8_t MOW_STATUS_DONE = 0;          // 00 = fertig bearbeitet
  static constexpr uint8_t MOW_STATUS_IN_PROGRESS = 10;  // 10 = in Arbeit
  static constexpr uint8_t MOW_STATUS_OPEN = 20;         // 20 = noch nicht bearbeitet

  static constexpr uint8_t PATH_DIRECTION_FORWARD = 0;  // wie vom Slicer geliefert
  static constexpr uint8_t PATH_DIRECTION_REVERSE = 1;  // rückwärts abfahren, Rohdaten bleiben unverändert

  struct MowingPathSlicerSource {
    // Originale Slicer-Quelle. path_id entspricht der urspruenglichen Reihenfolge/ID
    // aus der Slicer-Datei und bleibt auch nach POO-Optimierung unveraendert.
    slic3r_coverage_planner::Path path;
    uint32_t path_id = 0;
  };

  struct MowingPathExecution {
    // Fertiger realer Fahrpfad. Die Mower-Logic faehrt ausschliesslich diesen Pfad.
    // Bei FORWARD ist er eine Kopie von slicer_source.path. Bei REVERSE oder Rotation
    // ist er eine vorbereitete und neu orientierte Ausfuehrungsgeometrie.
    slic3r_coverage_planner::Path path;

    // 0 = keine Rotation. >0 bedeutet: execution.path beginnt bei diesem urspruenglichen
    // Slicer-Punkt. Wird aktuell fuer den optimierten Einstieg in den Area-Outline-Block genutzt.
    uint32_t rotation_offset = 0;

    // Lesbare Debug-/Statusinformation, z.B. "reversed" oder "rotated_approach_outer_outline_entry".
    std::vector<std::string> transform_flags;
  };

  struct MowingPathExecutionItem {
    std::string area_id;
    std::string area_digest;
    std::string plan_id;
    std::string path_id;

    uint32_t order = 0;
    uint8_t path_direction = PATH_DIRECTION_FORWARD;

    // Mähstatus dieses Pfades:
    //   20 = noch nicht bearbeitet
    //   10 = in Arbeit
    //   00 = fertig bearbeitet
    uint8_t mow_status = MOW_STATUS_OPEN;

    uint32_t current_pose_index = 0;
    MowingPathSlicerSource slicer_source;
    MowingPathExecution execution;
  };

  struct MowingExecutionPlan {
    std::string area_id;
    std::string area_digest;
    std::string plan_id;

    uint32_t current_order = 0;
    std::string current_path_id;
    std::string plan_file;
    uint8_t processing_mode = 2;
    uint8_t outline_entry_mode = 0;

    std::vector<MowingPathExecutionItem> paths;
  };

 private:
  // Progress
  bool mowerEnabled = false;
  MowingExecutionPlan currentMowingPlan;

  ros::Time last_checkpoint;
  int currentMowingPath;
  int currentMowingArea;
  std::string currentAreaId;
  std::string checkpointAreaId;
  int currentMowingPathIndex;
  std::string currentMowingPlanDigest;
  double currentMowingAngleIncrementSum;

  void clear_current_mowing_plan();
  void build_current_mowing_plan(const std::vector<slic3r_coverage_planner::Path>& slicer_paths,
                                 const std::string& area_id, const std::string& area_digest);
  bool optimize_current_mowing_plan(const geometry_msgs::PoseStamped& current_pose);
  void normalize_current_mowing_plan_orders();
  void update_current_mowing_plan_progress();
  void finish_current_mowing_plan_path();
  void start_current_mowing_plan_path();
  bool save_current_mowing_plan() const;
  bool load_current_mowing_plan_snapshot(const std::string& plan_file, const std::string& expected_area_id,
                                         const std::string& expected_area_digest);
  std::string make_plan_file_path(const std::string& plan_id) const;

  ros::Publisher mowing_progress_pub;
  ros::Publisher mowing_progress_status_pub;
  ros::Subscriber mowing_progress_renew_sub;
  bool mowing_progress_interface_initialized = false;
  ros::Time last_mowing_progress_publish;
  ros::Time last_mowing_progress_status_publish;
  int last_published_mowing_path = -1;
  int last_published_mowing_path_index = -1;
  int last_published_mowing_status_path = -1;
  int last_published_mowing_status_path_index = -1;

  void ensure_mowing_progress_interface();
  void mowing_progress_renew_callback(const std_msgs::Empty::ConstPtr& msg);
  nlohmann::json build_mowing_progress_payload(bool include_paths);
  void publish_mowing_progress(bool force = false);
  void publish_mowing_progress_status(bool force = false);

 public:
  MowingBehavior();

  static MowingBehavior INSTANCE;

  std::string state_name() override;

  Behavior* execute() override;

  void enter() override;

  void exit() override;

  void reset() override;

  bool needs_gps() override;

  bool mower_enabled() override;

  void command_home() override;

  void command_start() override;

  void command_s1() override;

  void command_s2() override;

  bool redirect_joystick() override;

  uint8_t get_sub_state() override;

  uint8_t get_state() override;

  int16_t get_current_area();

  std::string get_current_area_id();

  std::string get_checkpoint_area_id();

  int16_t get_current_path();

  int16_t get_current_path_index();

  void handle_action(std::string action) override;

  void request_direct_mowing_area(const std::string& area_id, const std::string& mode = "single");

  void clear_direct_mowing_area_request();

  void update_actions();

  void checkpoint();

  bool restore_checkpoint();
};

#endif  // SRC_MOWINGBEHAVIOR_H
