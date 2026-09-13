#ifndef ELEVATION_LOCAL_PLANNER_D1_CONTROL_TYPES_H_
#define ELEVATION_LOCAL_PLANNER_D1_CONTROL_TYPES_H_

#include <algorithm>
#include <cmath>
#include <cctype>
#include <limits>
#include <string>
#include <vector>
#include <geometry_msgs/PoseStamped.h>

namespace elevation_local_planner
{

struct RobotPose2D
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double yaw{0.0};
};

struct TrackingTarget
{
  double base_x{0.0};
  double base_y{0.0};
};

class D1ControlUtils
{
public:
  static double clamp(double v, double lo, double hi)
  {
    return std::max(lo, std::min(hi, v));
  }

  static double applyDeadband(double v, double db)
  {
    return std::abs(v) < db ? 0.0 : v;
  }

  static double normalizeAngle(double a)
  {
    return std::atan2(std::sin(a), std::cos(a));
  }

  static std::vector<std::string> splitCsv(const std::string & text)
  {
    std::vector<std::string> parts;
    std::string cur;
    for (const char ch : text) {
      if (ch == ',') {
        const auto t = trim(cur);
        if (!t.empty()) parts.push_back(t);
        cur.clear();
      } else {
        cur.push_back(ch);
      }
    }
    const auto t = trim(cur);
    if (!t.empty()) parts.push_back(t);
    return parts;
  }

  static std::string trim(const std::string & text)
  {
    std::size_t first = 0;
    while (first < text.size() && std::isspace(static_cast<unsigned char>(text[first]))) ++first;
    std::size_t last = text.size();
    while (last > first && std::isspace(static_cast<unsigned char>(text[last - 1]))) --last;
    return text.substr(first, last - first);
  }

  // Projection-based continuous pure pursuit lookahead target interpolation
  static bool interpolateLookaheadTarget(
    const std::vector<geometry_msgs::PoseStamped> & plan,
    const RobotPose2D & robot_pose,
    int & target_index,
    double lookahead_dist,
    TrackingTarget & target)
  {
    if (plan.empty()) return false;

    const std::size_t plan_size = plan.size();
    if (plan_size == 1) {
      target_index = 0;
      const auto & p = plan[0].pose.position;
      const double dx = p.x - robot_pose.x, dy = p.y - robot_pose.y;
      const double cy = std::cos(robot_pose.yaw), sy = std::sin(robot_pose.yaw);
      target.base_x =  cy * dx + sy * dy;
      target.base_y = -sy * dx + cy * dy;
      return true;
    }

    // 1. Find nearest segment and projection point of robot onto path
    int best_seg_idx = target_index;
    if (best_seg_idx < 0 || best_seg_idx >= static_cast<int>(plan_size) - 1) best_seg_idx = 0;

    double min_proj_sq_dist = std::numeric_limits<double>::max();
    double proj_x = plan[best_seg_idx].pose.position.x;
    double proj_y = plan[best_seg_idx].pose.position.y;

    const int search_start = std::max(0, best_seg_idx - 5);
    const int search_end = std::min(static_cast<int>(plan_size) - 1, best_seg_idx + 40);

    for (int i = search_start; i < search_end; ++i) {
      const auto & p1 = plan[i].pose.position;
      const auto & p2 = plan[i + 1].pose.position;
      const double dx = p2.x - p1.x, dy = p2.y - p1.y;
      const double seg_sq_len = dx * dx + dy * dy;

      double t = 0.0;
      if (seg_sq_len > 1.0e-6) {
        t = ((robot_pose.x - p1.x) * dx + (robot_pose.y - p1.y) * dy) / seg_sq_len;
        t = std::max(0.0, std::min(1.0, t));
      }
      const double px = p1.x + t * dx;
      const double py = p1.y + t * dy;
      const double sq_dist = (robot_pose.x - px) * (robot_pose.x - px) + (robot_pose.y - py) * (robot_pose.y - py);

      if (sq_dist < min_proj_sq_dist) {
        min_proj_sq_dist = sq_dist;
        best_seg_idx = i;
        proj_x = px;
        proj_y = py;
      }
    }
    target_index = best_seg_idx;

    // 2. Goal Proximity Protection: Cap lookahead distance by distance to goal
    const auto & goal_pos = plan.back().pose.position;
    const double dist_to_goal = std::hypot(goal_pos.x - robot_pose.x, goal_pos.y - robot_pose.y);
    const double effective_lookahead = std::max(0.15, std::min(lookahead_dist, dist_to_goal));

    // 3. Measure exact effective_lookahead forward along path starting from (proj_x, proj_y)
    double target_map_x = proj_x;
    double target_map_y = proj_y;
    double accum_dist = 0.0;

    const auto & p2_first = plan[best_seg_idx + 1].pose.position;
    const double first_rem_dist = std::hypot(p2_first.x - proj_x, p2_first.y - proj_y);

    if (first_rem_dist >= effective_lookahead) {
      if (first_rem_dist > 1.0e-5) {
        const double ratio = effective_lookahead / first_rem_dist;
        target_map_x = proj_x + ratio * (p2_first.x - proj_x);
        target_map_y = proj_y + ratio * (p2_first.y - proj_y);
      }
    } else {
      accum_dist = first_rem_dist;
      target_map_x = p2_first.x;
      target_map_y = p2_first.y;

      for (std::size_t i = static_cast<std::size_t>(best_seg_idx + 1); i < plan_size - 1; ++i) {
        const auto & p1 = plan[i].pose.position;
        const auto & p2 = plan[i + 1].pose.position;
        const double seg_len = std::hypot(p2.x - p1.x, p2.y - p1.y);
        if (seg_len < 1.0e-5) continue;

        if (accum_dist + seg_len >= effective_lookahead) {
          const double remaining = effective_lookahead - accum_dist;
          const double ratio = std::max(0.0, std::min(1.0, remaining / seg_len));
          target_map_x = p1.x + ratio * (p2.x - p1.x);
          target_map_y = p1.y + ratio * (p2.y - p1.y);
          break;
        } else {
          accum_dist += seg_len;
          target_map_x = p2.x;
          target_map_y = p2.y;
        }
      }
    }

    // 4. Transform target coordinates to robot local frame
    const double dx_map = target_map_x - robot_pose.x;
    const double dy_map = target_map_y - robot_pose.y;
    const double cy = std::cos(robot_pose.yaw), sy = std::sin(robot_pose.yaw);

    target.base_x =  cy * dx_map + sy * dy_map;
    target.base_y = -sy * dx_map + cy * dy_map;
    return true;
  }
};

} // namespace elevation_local_planner

#endif // ELEVATION_LOCAL_PLANNER_D1_CONTROL_TYPES_H_
