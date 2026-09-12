#include "elevation_local_planner/elevation_local_planner.hpp"
#include <tf2/utils.h>
#include <cmath>
#include <algorithm>

namespace elevation_local_planner
{

ElevationLocalPlanner::ElevationLocalPlanner()
  : collision_checker_(0.30, 0.50)
{
}

bool ElevationLocalPlanner::setPlan(const nav_msgs::Path & global_path)
{
  global_path_ = global_path;
  target_idx_ = 0;
  goal_reached_ = false;
  return !global_path_.poses.empty();
}

bool ElevationLocalPlanner::computeVelocityCommands(geometry_msgs::Twist & cmd_vel)
{
  geometry_msgs::PoseStamped dummy_pose;
  return computeVelocityCommands(dummy_pose, cmd_vel);
}

bool ElevationLocalPlanner::computeVelocityCommands(
  const geometry_msgs::PoseStamped & current_pose,
  geometry_msgs::Twist & cmd_vel)
{
  if (global_path_.poses.empty()) {
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
    return false;
  }

  double cur_x = current_pose.pose.position.x;
  double cur_y = current_pose.pose.position.y;
  double cur_z = current_pose.pose.position.z;
  double cur_yaw = tf2::getYaw(current_pose.pose.orientation);

  // 机体足印碰撞检查: 当前位置足印半径内存在阻挡节点 (顶头净空不足 / 侧向墙体膨胀) 时停车
  if (graph_.numNodes() > 0) {
    uint32_t nid = 0;
    if (graph_.findClosestNode(cur_x, cur_y, cur_z, nid, 0.5, 0.8)) {
      int layer = graph_.getNode(nid).layer_id;
      if (collision_checker_.checkCollision(cur_x, cur_y, cur_z, layer, graph_)) {
        cmd_vel.linear.x = 0.0;
        cmd_vel.linear.y = 0.0;
        cmd_vel.angular.z = 0.0;
        return false;
      }
    }
  }

  // 1. 查找前瞻追踪目标点
  while (target_idx_ < global_path_.poses.size() - 1) {
    double tx = global_path_.poses[target_idx_].pose.position.x;
    double ty = global_path_.poses[target_idx_].pose.position.y;
    if (std::hypot(tx - cur_x, ty - cur_y) >= config_.lookahead_dist) {
      break;
    }
    target_idx_++;
  }

  const auto & goal_pose = global_path_.poses.back();
  double dist_to_goal = std::hypot(goal_pose.pose.position.x - cur_x, goal_pose.pose.position.y - cur_y);

  if (dist_to_goal < config_.goal_tolerance_xy) {
    goal_reached_ = true;
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
    return true;
  }

  const auto & target_pt = global_path_.poses[target_idx_].pose.position;
  double dx = target_pt.x - cur_x;
  double dy = target_pt.y - cur_y;
  double target_yaw = std::atan2(dy, dx);
  double yaw_diff = target_yaw - cur_yaw;

  while (yaw_diff > M_PI) yaw_diff -= 2.0 * M_PI;
  while (yaw_diff < -M_PI) yaw_diff += 2.0 * M_PI;

  // 纯追踪比例控制 (Pure Pursuit / PID)
  double v = config_.max_vel_x * std::max(0.1, std::cos(yaw_diff));
  double w = 2.0 * yaw_diff;
  w = std::min(config_.max_vel_theta, std::max(-config_.max_vel_theta, w));

  cmd_vel.linear.x = v;
  cmd_vel.linear.y = 0.0;
  cmd_vel.angular.z = w;
  return true;
}

} // namespace elevation_local_planner
