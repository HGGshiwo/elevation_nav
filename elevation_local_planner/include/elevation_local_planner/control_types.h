#ifndef ELEVATION_LOCAL_PLANNER_CONTROL_TYPES_H_
#define ELEVATION_LOCAL_PLANNER_CONTROL_TYPES_H_

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

class LocalControlUtils
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

  // 投影法连续前瞻目标点插值
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

    // 1. 寻找机器人投影到路径上的最近线段
    int best_seg_idx = target_index;
    if (best_seg_idx < 0 || best_seg_idx >= static_cast<int>(plan_size) - 1) best_seg_idx = 0;

    double min_proj_sq_dist = std::numeric_limits<double>::max();
    double proj_x = plan[best_seg_idx].pose.position.x;
    double proj_y = plan[best_seg_idx].pose.position.y;
    double best_t = 0.0;

    const int search_start = std::max(0, target_index - 3);
    const int search_end   = std::min(static_cast<int>(plan_size) - 1, target_index + 12);

    for (int i = search_start; i < search_end; ++i) {
      const auto & p0 = plan[i].pose.position;
      const auto & p1 = plan[i + 1].pose.position;
      const double seg_dx = p1.x - p0.x;
      const double seg_dy = p1.y - p0.y;
      const double seg_len_sq = seg_dx * seg_dx + seg_dy * seg_dy;

      double t = 0.0;
      if (seg_len_sq > 1e-6) {
        t = ((robot_pose.x - p0.x) * seg_dx + (robot_pose.y - p0.y) * seg_dy) / seg_len_sq;
        t = std::max(0.0, std::min(1.0, t));
      }
      const double cur_proj_x = p0.x + t * seg_dx;
      const double cur_proj_y = p0.y + t * seg_dy;
      const double d_proj_sq  = (robot_pose.x - cur_proj_x) * (robot_pose.x - cur_proj_x) +
                                (robot_pose.y - cur_proj_y) * (robot_pose.y - cur_proj_y);

      if (d_proj_sq < min_proj_sq_dist) {
        min_proj_sq_dist = d_proj_sq;
        best_seg_idx     = i;
        proj_x           = cur_proj_x;
        proj_y           = cur_proj_y;
        best_t           = t;
      }
    }

    target_index = best_seg_idx;

    // 2. 从投影点沿折线前瞻 lookahead_dist
    double remain = lookahead_dist;
    double look_x = proj_x;
    double look_y = proj_y;

    const auto & p_seg_end = plan[best_seg_idx + 1].pose.position;
    const double first_seg_remain = (1.0 - best_t) * std::hypot(
      p_seg_end.x - plan[best_seg_idx].pose.position.x,
      p_seg_end.y - plan[best_seg_idx].pose.position.y);

    if (remain <= first_seg_remain && first_seg_remain > 1e-5) {
      const double ratio = remain / first_seg_remain;
      look_x = proj_x + ratio * (p_seg_end.x - proj_x);
      look_y = proj_y + ratio * (p_seg_end.y - proj_y);
    } else {
      remain -= first_seg_remain;
      bool found = false;
      for (size_t i = best_seg_idx + 1; i < plan_size - 1; ++i) {
        const auto & pa = plan[i].pose.position;
        const auto & pb = plan[i + 1].pose.position;
        const double d_seg = std::hypot(pb.x - pa.x, pb.y - pa.y);
        if (remain <= d_seg && d_seg > 1e-5) {
          const double ratio = remain / d_seg;
          look_x = pa.x + ratio * (pb.x - pa.x);
          look_y = pa.y + ratio * (pb.y - pa.y);
          found = true;
          break;
        }
        remain -= d_seg;
      }
      if (!found) {
        look_x = plan.back().pose.position.x;
        look_y = plan.back().pose.position.y;
      }
    }

    // 3. 变换到机器人机身系
    const double dx = look_x - robot_pose.x;
    const double dy = look_y - robot_pose.y;
    const double cy = std::cos(robot_pose.yaw);
    const double sy = std::sin(robot_pose.yaw);

    target.base_x =  cy * dx + sy * dy;
    target.base_y = -sy * dx + cy * dy;

    return true;
  }
};

} // namespace elevation_local_planner

#endif // ELEVATION_LOCAL_PLANNER_CONTROL_TYPES_H_
