#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <sstream>
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
    res.paths = req.paths;
    res.success = true;
    res.used_optimization = false;
    res.used_fallback = false;

    if (!req.enabled) {
      res.message = "path order optimizer disabled; passing slicer order through unchanged";
      return true;
    }

    if (req.paths.empty()) {
      res.message = "path list is empty";
      return true;
    }

    try {
      std::vector<slic3r_coverage_planner::Path> area_outlines;
      std::vector<slic3r_coverage_planner::Path> fill_paths;
      std::vector<slic3r_coverage_planner::Path> obstacle_outlines;
      std::vector<slic3r_coverage_planner::Path> unknown_paths;

      splitPaths(req.paths, area_outlines, fill_paths, obstacle_outlines, unknown_paths);

      if (!unknown_paths.empty()) {
        ROS_WARN_STREAM("PathOrderOptimizer: " << unknown_paths.size()
                                                << " paths have unknown path_type; preserving them before fills.");
      }

      std::vector<slic3r_coverage_planner::Path> ordered;
      appendAll(ordered, area_outlines);
      appendAll(ordered, unknown_paths);
      if (!req.move_obstacles_to_end) {
        // Compatibility mode: keep obstacle outlines before fill paths, matching the old slicer group order.
        appendAll(ordered, obstacle_outlines);
      }

      std::vector<slic3r_coverage_planner::Path> ordered_fill_paths = fill_paths;
      if (req.optimize_fill_order && fill_paths.size() > 1) {
        if (req.max_fill_paths > 0 && fill_paths.size() > req.max_fill_paths) {
          ROS_WARN_STREAM("PathOrderOptimizer: fill path count " << fill_paths.size()
                                                                  << " exceeds max_fill_paths " << req.max_fill_paths
                                                                  << "; keeping original fill order.");
          res.used_fallback = true;
        } else {
          geometry_msgs::PoseStamped current = req.current_pose;
          if (!ordered.empty() && hasUsablePath(ordered.back())) current = lastPose(ordered.back());
          bool used_fallback = res.used_fallback;
          ordered_fill_paths = optimizeFillPaths(fill_paths, current, req, used_fallback);
          res.used_fallback = used_fallback;
          res.used_optimization = true;
        }
      }
      appendAll(ordered, ordered_fill_paths);

      if (req.move_obstacles_to_end) {
        appendAll(ordered, obstacle_outlines);
      }

      res.paths = ordered;
      std::ostringstream msg;
      msg << "optimized path order: area_outlines=" << area_outlines.size() << ", fill_paths=" << fill_paths.size()
          << ", obstacle_outlines=" << obstacle_outlines.size() << ", unknown=" << unknown_paths.size();
      if (res.used_fallback) msg << ", fallback used";
      res.message = msg.str();
      return true;
    } catch (const std::exception& e) {
      ROS_ERROR_STREAM("PathOrderOptimizer: optimization failed: " << e.what());
      res.paths = req.paths;
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

  void appendAll(std::vector<slic3r_coverage_planner::Path>& target,
                 const std::vector<slic3r_coverage_planner::Path>& source) const {
    target.insert(target.end(), source.begin(), source.end());
  }

  void splitPaths(const std::vector<slic3r_coverage_planner::Path>& paths,
                  std::vector<slic3r_coverage_planner::Path>& area_outlines,
                  std::vector<slic3r_coverage_planner::Path>& fill_paths,
                  std::vector<slic3r_coverage_planner::Path>& obstacle_outlines,
                  std::vector<slic3r_coverage_planner::Path>& unknown_paths) const {
    for (const auto& path : paths) {
      switch (path.path_type) {
        case slic3r_coverage_planner::Path::TYPE_FILL:
          fill_paths.push_back(path);
          break;
        case slic3r_coverage_planner::Path::TYPE_AREA_OUTLINE:
          area_outlines.push_back(path);
          break;
        case slic3r_coverage_planner::Path::TYPE_OBSTACLE_OUTLINE:
          obstacle_outlines.push_back(path);
          break;
        default:
          // Backward compatibility fallback for paths produced by older code.
          if (path.is_outline) {
            unknown_paths.push_back(path);
          } else {
            fill_paths.push_back(path);
          }
          break;
      }
    }
  }

  std::vector<slic3r_coverage_planner::Path> optimizeFillPaths(
      const std::vector<slic3r_coverage_planner::Path>& fill_paths,
      geometry_msgs::PoseStamped current,
      const mowing_path_order_optimizer::OptimizePaths::Request& req,
      bool& used_fallback) {
    std::vector<slic3r_coverage_planner::Path> remaining;
    for (const auto& p : fill_paths) {
      if (hasUsablePath(p)) {
        remaining.push_back(p);
      } else {
        used_fallback = true;
      }
    }

    std::vector<slic3r_coverage_planner::Path> ordered;
    ordered.reserve(fill_paths.size());

    while (!remaining.empty()) {
      CandidateCost best = chooseBestCandidate(remaining, current, req, used_fallback);
      if (!std::isfinite(best.cost) || best.index >= remaining.size()) {
        used_fallback = true;
        if (req.fallback_to_slicer_order) {
          ROS_WARN("PathOrderOptimizer: no finite candidate cost; appending remaining fill paths in slicer order.");
          appendAll(ordered, remaining);
          remaining.clear();
          break;
        }
        best.index = 0;
        best.reverse = false;
      }

      auto selected = remaining[best.index];
      if (best.reverse && req.allow_reverse) reversePath(selected);
      current = lastPose(selected);
      ordered.push_back(selected);
      remaining.erase(remaining.begin() + best.index);
    }

    // Preserve empty paths in original order at the end, if any existed.
    for (const auto& p : fill_paths) {
      if (!hasUsablePath(p)) ordered.push_back(p);
    }
    return ordered;
  }

  CandidateCost chooseBestCandidate(const std::vector<slic3r_coverage_planner::Path>& remaining,
                                    const geometry_msgs::PoseStamped& current,
                                    const mowing_path_order_optimizer::OptimizePaths::Request& req,
                                    bool& used_fallback) {
    std::vector<CandidateCost> euclidean_candidates;
    euclidean_candidates.reserve(remaining.size() * 2);
    for (std::size_t i = 0; i < remaining.size(); ++i) {
      CandidateCost start;
      start.index = i;
      start.reverse = false;
      start.cost = dist2d(current, firstPose(remaining[i]));
      euclidean_candidates.push_back(start);

      if (req.allow_reverse) {
        CandidateCost end;
        end.index = i;
        end.reverse = true;
        end.cost = dist2d(current, lastPose(remaining[i]));
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
      const auto& path = remaining[candidate.index];
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
