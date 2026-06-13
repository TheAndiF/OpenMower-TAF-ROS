#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <mbf_msgs/GetPathAction.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "mowing_path_order_optimizer/OptimizePaths.h"
#include "slic3r_coverage_planner/Path.h"

namespace {

double dist2d(const geometry_msgs::PoseStamped& a, const geometry_msgs::PoseStamped& b) {
  const double dx = a.pose.position.x - b.pose.position.x;
  const double dy = a.pose.position.y - b.pose.position.y;
  return std::sqrt(dx * dx + dy * dy);
}

double pathLength(const nav_msgs::Path& path) {
  if (path.poses.size() < 2) return 0.0;
  double length = 0.0;
  for (std::size_t i = 1; i < path.poses.size(); ++i) {
    length += dist2d(path.poses[i - 1], path.poses[i]);
  }
  return length;
}

bool hasUsablePath(const slic3r_coverage_planner::Path& path) {
  return !path.path.poses.empty();
}

geometry_msgs::PoseStamped firstPose(const slic3r_coverage_planner::Path& path) {
  return path.path.poses.front();
}

geometry_msgs::PoseStamped lastPose(const slic3r_coverage_planner::Path& path) {
  return path.path.poses.back();
}

void recomputeOrientations(slic3r_coverage_planner::Path& path) {
  if (path.path.poses.size() < 2) return;
  for (std::size_t i = 0; i + 1 < path.path.poses.size(); ++i) {
    const auto& current = path.path.poses[i].pose.position;
    const auto& next = path.path.poses[i + 1].pose.position;
    const double yaw = std::atan2(next.y - current.y, next.x - current.x);
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    path.path.poses[i].pose.orientation = tf2::toMsg(q);
  }
  path.path.poses.back().pose.orientation = path.path.poses[path.path.poses.size() - 2].pose.orientation;
}

void reversePath(slic3r_coverage_planner::Path& path) {
  std::reverse(path.path.poses.begin(), path.path.poses.end());
  recomputeOrientations(path);
}

bool isClosedPath(const slic3r_coverage_planner::Path& path) {
  return path.path.poses.size() >= 4 && dist2d(path.path.poses.front(), path.path.poses.back()) < 0.20;
}

void rotateClosedPath(slic3r_coverage_planner::Path& path, std::size_t offset) {
  auto& poses = path.path.poses;
  if (!isClosedPath(path)) return;
  const std::size_t unique_count = poses.size() - 1;
  if (unique_count == 0) return;
  offset = offset % unique_count;
  if (offset == 0) return;

  std::vector<geometry_msgs::PoseStamped> rotated;
  rotated.reserve(poses.size());
  for (std::size_t i = offset; i < unique_count; ++i) rotated.push_back(poses[i]);
  for (std::size_t i = 0; i <= offset; ++i) rotated.push_back(poses[i]);
  poses.swap(rotated);
  recomputeOrientations(path);
}

std::string appendFlag(const std::string& flags, const std::string& flag) {
  if (flags.empty()) return flag;
  return flags + "," + flag;
}

struct OptimizerPath {
  slic3r_coverage_planner::Path path;
  int32_t source_index = 0;
  bool reversed = false;
  uint32_t rotation_offset = 0;
  std::string transform_flags;
};

struct CandidateCost {
  std::size_t index = 0;
  bool reverse = false;
  double cost = std::numeric_limits<double>::infinity();
  bool planner_cost_used = false;
};

class PathOrderOptimizer {
 public:
  explicit PathOrderOptimizer(ros::NodeHandle& nh) : nh_(nh) {}

  bool handle(mowing_path_order_optimizer::OptimizePaths::Request& req,
              mowing_path_order_optimizer::OptimizePaths::Response& res) {
    fillPassThroughResponse(req, res);

    if (!req.enabled) {
      res.message = "path order optimizer disabled; passing slicer order through unchanged";
      return true;
    }

    if (req.paths.empty()) {
      res.message = "path list is empty";
      return true;
    }

    try {
      std::vector<OptimizerPath> input_paths;
      input_paths.reserve(req.paths.size());
      for (std::size_t i = 0; i < req.paths.size(); ++i) {
        OptimizerPath item;
        item.path = req.paths[i];
        item.source_index = (req.path_indices.size() == req.paths.size()) ? req.path_indices[i] : static_cast<int32_t>(i);
        input_paths.push_back(item);
      }

      std::vector<OptimizerPath> area_outlines;
      std::vector<OptimizerPath> fill_paths;
      std::vector<OptimizerPath> obstacle_outlines;
      std::vector<OptimizerPath> unknown_paths;
      splitPaths(input_paths, area_outlines, fill_paths, obstacle_outlines, unknown_paths);

      if (!unknown_paths.empty()) {
        ROS_WARN_STREAM("PathOrderOptimizer: " << unknown_paths.size()
                                                << " paths have unknown path_type; preserving them before optimized groups.");
      }

      bool used_fallback = res.used_fallback;
      std::vector<OptimizerPath> ordered;
      geometry_msgs::PoseStamped current = req.current_pose;
      const uint8_t mode = normalizeProcessingMode(req.processing_mode, req);

      if (mode == mowing_path_order_optimizer::OptimizePaths::Request::PROCESSING_MODE_SLICER_ORDER) {
        // True slicer mode: keep the request order exactly. The optional outer-outline entry optimization
        // only rotates the prepared execution path of the first area outline; it does not reorder paths.
        ordered = input_paths;
        if (req.optimize_outer_outline_entry) {
          for (auto& item : ordered) {
            if (item.path.path_type == slic3r_coverage_planner::Path::TYPE_AREA_OUTLINE) {
              optimizeOuterOutlineEntry(item, current, req, used_fallback);
              break;
            }
          }
        }
        res.used_fallback = used_fallback;
        res.used_optimization = req.optimize_outer_outline_entry;
      } else {
        // The outer area outline stays the first group, but its entry point can be rotated to the
        // best reachable start. slicer_source remains unchanged in mower_logic; this response path
        // is the prepared execution path.
        if (req.optimize_outer_outline_entry && !area_outlines.empty()) {
          optimizeOuterOutlineEntry(area_outlines.front(), current, req, used_fallback);
        }
        appendAll(ordered, area_outlines);
        if (!ordered.empty() && hasUsablePath(ordered.back().path)) current = lastPose(ordered.back().path);

        appendAll(ordered, unknown_paths);
        if (!ordered.empty() && hasUsablePath(ordered.back().path)) current = lastPose(ordered.back().path);

      switch (mode) {

        case mowing_path_order_optimizer::OptimizePaths::Request::PROCESSING_MODE_ORDERED_OBSTACLES_PLUS_ORDERED_FILLS: {
          auto ordered_obstacles = optimizePaths(obstacle_outlines, current, req, used_fallback, "obstacle outlines");
          appendAll(ordered, ordered_obstacles);
          if (!ordered.empty() && hasUsablePath(ordered.back().path)) current = lastPose(ordered.back().path);
          auto ordered_fills = optimizePaths(fill_paths, current, req, used_fallback, "fill paths");
          appendAll(ordered, ordered_fills);
          break;
        }

        case mowing_path_order_optimizer::OptimizePaths::Request::PROCESSING_MODE_MIXED_FILLS_AND_OBSTACLES: {
          std::vector<OptimizerPath> mixed = fill_paths;
          mixed.insert(mixed.end(), obstacle_outlines.begin(), obstacle_outlines.end());
          auto ordered_mixed = optimizePaths(mixed, current, req, used_fallback, "mixed fill/obstacle paths");
          appendAll(ordered, ordered_mixed);
          break;
        }

        case mowing_path_order_optimizer::OptimizePaths::Request::PROCESSING_MODE_ORDERED_FILLS_PLUS_ORDERED_OBSTACLES:
        default: {
          auto ordered_fills = optimizePaths(fill_paths, current, req, used_fallback, "fill paths");
          appendAll(ordered, ordered_fills);
          if (!ordered.empty() && hasUsablePath(ordered.back().path)) current = lastPose(ordered.back().path);
          auto ordered_obstacles = optimizePaths(obstacle_outlines, current, req, used_fallback, "obstacle outlines");
          appendAll(ordered, ordered_obstacles);
          break;
        }
      }
        res.used_optimization = true;
      }

      res.paths.clear();
      res.path_indices.clear();
      res.path_reversed.clear();
      res.rotation_offsets.clear();
      res.transform_flags.clear();
      for (const auto& item : ordered) {
        res.paths.push_back(item.path);
        res.path_indices.push_back(item.source_index);
        res.path_reversed.push_back(item.reversed);
        res.rotation_offsets.push_back(item.rotation_offset);
        res.transform_flags.push_back(item.transform_flags);
      }
      res.success = true;
      res.used_fallback = used_fallback;
      std::ostringstream msg;
      msg << "optimized path order: mode=" << static_cast<int>(mode)
          << ", area_outlines=" << area_outlines.size()
          << ", fill_paths=" << fill_paths.size()
          << ", obstacle_outlines=" << obstacle_outlines.size()
          << ", unknown=" << unknown_paths.size();
      if (res.used_optimization) msg << ", optimization used";
      if (res.used_fallback) msg << ", fallback used";
      res.message = msg.str();
      return true;
    } catch (const std::exception& e) {
      ROS_ERROR_STREAM("PathOrderOptimizer: optimization failed: " << e.what());
      fillPassThroughResponse(req, res);
      res.success = req.fail_open;
      res.used_optimization = false;
      res.used_fallback = true;
      res.message = std::string("optimization failed; using original slicer order: ") + e.what();
      return true;
    }
  }

 private:
  ros::NodeHandle nh_;
  std::string planner_action_name_;
  std::unique_ptr<actionlib::SimpleActionClient<mbf_msgs::GetPathAction>> get_path_client_;

  void fillPassThroughResponse(const mowing_path_order_optimizer::OptimizePaths::Request& req,
                               mowing_path_order_optimizer::OptimizePaths::Response& res) const {
    res.paths = req.paths;
    res.path_indices.clear();
    res.path_reversed.clear();
    res.rotation_offsets.clear();
    res.transform_flags.clear();
    for (std::size_t i = 0; i < req.paths.size(); ++i) {
      res.path_indices.push_back((req.path_indices.size() == req.paths.size()) ? req.path_indices[i] : static_cast<int32_t>(i));
      res.path_reversed.push_back(false);
      res.rotation_offsets.push_back(0);
      res.transform_flags.push_back("");
    }
    res.success = true;
    res.used_optimization = false;
    res.used_fallback = false;
  }

  void appendAll(std::vector<OptimizerPath>& target,
                 const std::vector<OptimizerPath>& source) const {
    target.insert(target.end(), source.begin(), source.end());
  }

  uint8_t normalizeProcessingMode(uint8_t mode,
                                  const mowing_path_order_optimizer::OptimizePaths::Request& req) const {
    switch (mode) {
      case mowing_path_order_optimizer::OptimizePaths::Request::PROCESSING_MODE_SLICER_ORDER:
      case mowing_path_order_optimizer::OptimizePaths::Request::PROCESSING_MODE_ORDERED_FILLS_PLUS_ORDERED_OBSTACLES:
      case mowing_path_order_optimizer::OptimizePaths::Request::PROCESSING_MODE_ORDERED_OBSTACLES_PLUS_ORDERED_FILLS:
      case mowing_path_order_optimizer::OptimizePaths::Request::PROCESSING_MODE_MIXED_FILLS_AND_OBSTACLES:
        return mode;
      default:
        // Backward compatibility for older configs that only had optimize_fill_order/move_obstacles_to_end.
        if (!req.optimize_fill_order) return mowing_path_order_optimizer::OptimizePaths::Request::PROCESSING_MODE_SLICER_ORDER;
        if (!req.move_obstacles_to_end) return mowing_path_order_optimizer::OptimizePaths::Request::PROCESSING_MODE_ORDERED_OBSTACLES_PLUS_ORDERED_FILLS;
        return mowing_path_order_optimizer::OptimizePaths::Request::PROCESSING_MODE_ORDERED_FILLS_PLUS_ORDERED_OBSTACLES;
    }
  }

  void splitPaths(const std::vector<OptimizerPath>& paths,
                  std::vector<OptimizerPath>& area_outlines,
                  std::vector<OptimizerPath>& fill_paths,
                  std::vector<OptimizerPath>& obstacle_outlines,
                  std::vector<OptimizerPath>& unknown_paths) const {
    for (const auto& item : paths) {
      switch (item.path.path_type) {
        case slic3r_coverage_planner::Path::TYPE_FILL:
          fill_paths.push_back(item);
          break;
        case slic3r_coverage_planner::Path::TYPE_AREA_OUTLINE:
          area_outlines.push_back(item);
          break;
        case slic3r_coverage_planner::Path::TYPE_OBSTACLE_OUTLINE:
          obstacle_outlines.push_back(item);
          break;
        default:
          if (item.path.is_outline) {
            unknown_paths.push_back(item);
          } else {
            fill_paths.push_back(item);
          }
          break;
      }
    }
  }

  void optimizeOuterOutlineEntry(OptimizerPath& outline,
                                 const geometry_msgs::PoseStamped& current,
                                 const mowing_path_order_optimizer::OptimizePaths::Request& req,
                                 bool& used_fallback) {
    if (!isClosedPath(outline.path)) return;
    const std::size_t unique_count = outline.path.path.poses.size() - 1;
    if (unique_count < 2) return;

    std::vector<CandidateCost> candidates;
    candidates.reserve(unique_count);
    for (std::size_t i = 0; i < unique_count; ++i) {
      CandidateCost candidate;
      candidate.index = i;
      candidate.reverse = false;
      candidate.cost = dist2d(current, outline.path.path.poses[i]);
      candidates.push_back(candidate);
    }
    std::sort(candidates.begin(), candidates.end(), [](const CandidateCost& a, const CandidateCost& b) {
      return a.cost < b.cost;
    });

    CandidateCost best = candidates.front();
    if (req.cost_mode != mowing_path_order_optimizer::OptimizePaths::Request::COST_EUCLIDEAN) {
      CandidateCost planner_best;
      const std::size_t limit = std::max<std::size_t>(1, req.candidate_limit);
      const std::size_t check_count = std::min(limit, candidates.size());
      for (std::size_t i = 0; i < check_count; ++i) {
        CandidateCost candidate = candidates[i];
        double planner_cost = std::numeric_limits<double>::infinity();
        if (getPlannerCost(current, outline.path.path.poses[candidate.index], req, planner_cost)) {
          candidate.cost = planner_cost;
          candidate.planner_cost_used = true;
        } else if (req.fallback_to_euclidean || req.cost_mode == mowing_path_order_optimizer::OptimizePaths::Request::COST_HYBRID) {
          used_fallback = true;
        } else {
          used_fallback = true;
          continue;
        }
        if (candidate.cost < planner_best.cost) planner_best = candidate;
      }
      if (std::isfinite(planner_best.cost)) {
        best = planner_best;
      } else if (!req.fallback_to_euclidean) {
        return;
      }
    }

    if (best.index == 0) return;
    rotateClosedPath(outline.path, best.index);
    outline.rotation_offset = static_cast<uint32_t>(best.index);
    outline.transform_flags = appendFlag(outline.transform_flags, "rotated_outer_outline_entry");
  }

  std::vector<OptimizerPath> optimizePaths(
      const std::vector<OptimizerPath>& paths,
      geometry_msgs::PoseStamped current,
      const mowing_path_order_optimizer::OptimizePaths::Request& req,
      bool& used_fallback,
      const std::string& label) {
    std::vector<OptimizerPath> remaining;
    std::vector<OptimizerPath> empty_paths;
    for (const auto& p : paths) {
      if (hasUsablePath(p.path)) {
        remaining.push_back(p);
      } else {
        used_fallback = true;
        empty_paths.push_back(p);
      }
    }

    std::vector<OptimizerPath> ordered;
    ordered.reserve(paths.size());
    if (remaining.size() <= 1) {
      appendAll(ordered, remaining);
      appendAll(ordered, empty_paths);
      return ordered;
    }

    if (req.max_fill_paths > 0 && remaining.size() > req.max_fill_paths) {
      ROS_WARN_STREAM("PathOrderOptimizer: " << label << " count " << remaining.size()
                                              << " exceeds max_fill_paths " << req.max_fill_paths
                                              << "; keeping original order.");
      used_fallback = true;
      appendAll(ordered, remaining);
      appendAll(ordered, empty_paths);
      return ordered;
    }

    while (!remaining.empty()) {
      CandidateCost best = chooseBestCandidate(remaining, current, req, used_fallback);
      if (!std::isfinite(best.cost) || best.index >= remaining.size()) {
        used_fallback = true;
        if (req.fallback_to_slicer_order) {
          ROS_WARN_STREAM("PathOrderOptimizer: no finite candidate cost; appending remaining " << label
                                                                                              << " in slicer order.");
          appendAll(ordered, remaining);
          remaining.clear();
          break;
        }
        best.index = 0;
        best.reverse = false;
      }

      auto selected = remaining[best.index];
      if (best.reverse && req.allow_reverse) {
        reversePath(selected.path);
        selected.reversed = !selected.reversed;
        selected.transform_flags = appendFlag(selected.transform_flags, "reversed");
      }
      current = lastPose(selected.path);
      ordered.push_back(selected);
      remaining.erase(remaining.begin() + best.index);
    }

    appendAll(ordered, empty_paths);
    return ordered;
  }

  CandidateCost chooseBestCandidate(const std::vector<OptimizerPath>& remaining,
                                    const geometry_msgs::PoseStamped& current,
                                    const mowing_path_order_optimizer::OptimizePaths::Request& req,
                                    bool& used_fallback) {
    std::vector<CandidateCost> euclidean_candidates;
    euclidean_candidates.reserve(remaining.size() * 2);
    for (std::size_t i = 0; i < remaining.size(); ++i) {
      CandidateCost start;
      start.index = i;
      start.reverse = false;
      start.cost = dist2d(current, firstPose(remaining[i].path));
      euclidean_candidates.push_back(start);

      if (req.allow_reverse) {
        CandidateCost end;
        end.index = i;
        end.reverse = true;
        end.cost = dist2d(current, lastPose(remaining[i].path));
        euclidean_candidates.push_back(end);
      }
    }

    std::sort(euclidean_candidates.begin(), euclidean_candidates.end(), [](const CandidateCost& a, const CandidateCost& b) {
      return a.cost < b.cost;
    });

    if (req.cost_mode == mowing_path_order_optimizer::OptimizePaths::Request::COST_EUCLIDEAN ||
        euclidean_candidates.empty()) {
      return euclidean_candidates.empty() ? CandidateCost{} : euclidean_candidates.front();
    }

    const std::size_t limit = std::max<std::size_t>(1, req.candidate_limit);
    const std::size_t check_count = std::min(limit, euclidean_candidates.size());

    CandidateCost best;
    for (std::size_t i = 0; i < check_count; ++i) {
      CandidateCost candidate = euclidean_candidates[i];
      const auto& path = remaining[candidate.index].path;
      const auto goal = candidate.reverse ? lastPose(path) : firstPose(path);
      double planner_cost = std::numeric_limits<double>::infinity();
      if (getPlannerCost(current, goal, req, planner_cost)) {
        candidate.cost = planner_cost;
        candidate.planner_cost_used = true;
      } else if (req.fallback_to_euclidean || req.cost_mode == mowing_path_order_optimizer::OptimizePaths::Request::COST_HYBRID) {
        used_fallback = true;
      } else {
        used_fallback = true;
        continue;
      }

      if (candidate.cost < best.cost) best = candidate;
    }

    if (!std::isfinite(best.cost)) {
      used_fallback = true;
      if (req.fallback_to_euclidean && !euclidean_candidates.empty()) return euclidean_candidates.front();
    }
    return best;
  }

  bool getPlannerCost(const geometry_msgs::PoseStamped& start,
                      const geometry_msgs::PoseStamped& goal_pose,
                      const mowing_path_order_optimizer::OptimizePaths::Request& req,
                      double& cost) {
    if (req.planner_action.empty()) return false;

    if (!get_path_client_ || planner_action_name_ != req.planner_action) {
      planner_action_name_ = req.planner_action;
      get_path_client_.reset(new actionlib::SimpleActionClient<mbf_msgs::GetPathAction>(planner_action_name_, true));
    }

    if (!get_path_client_->waitForServer(ros::Duration(req.planner_timeout))) {
      ROS_WARN_STREAM_THROTTLE(10.0, "PathOrderOptimizer: planner action not available: " << req.planner_action);
      return false;
    }

    mbf_msgs::GetPathGoal goal;
    goal.use_start_pose = true;
    goal.start_pose = start;
    goal.target_pose = goal_pose;
    goal.start_pose.header.stamp = ros::Time::now();
    goal.target_pose.header.stamp = goal.start_pose.header.stamp;
    goal.tolerance = 0.20;
    goal.planner = req.planner_name;

    get_path_client_->sendGoal(goal);
    if (!get_path_client_->waitForResult(ros::Duration(req.planner_timeout))) {
      get_path_client_->cancelGoal();
      ROS_WARN_STREAM_THROTTLE(10.0, "PathOrderOptimizer: planner action timeout: " << req.planner_action);
      return false;
    }

    const auto result = get_path_client_->getResult();
    if (!result || result->path.poses.empty()) return false;

    cost = pathLength(result->path);
    return std::isfinite(cost);
  }
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "mowing_path_order_optimizer");
  ros::NodeHandle nh;
  PathOrderOptimizer optimizer(nh);
  ros::ServiceServer server = nh.advertiseService("mowing_path_order_optimizer/optimize_paths",
                                                  &PathOrderOptimizer::handle, &optimizer);
  ROS_INFO("mowing_path_order_optimizer ready");
  ros::spin();
  return 0;
}
