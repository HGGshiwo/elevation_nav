#pragma once

#include "elevation_planner_core/planner_interface.hpp"
#include "elevation_local_planner/collision_checker.hpp"

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

namespace elevation_local_planner
{

struct LocalPlannerConfig
{
  double max_vel_x{0.8};       ///< 最大前进速度 (m/s)
  double max_vel_theta{1.0};   ///< 最大转向角速度 (rad/s)
  double lookahead_dist{1.0};  ///< 前瞻跟踪距离 (m)
  double goal_tolerance_xy{0.25};
  double goal_tolerance_yaw{0.3};
  double footprint_radius{0.30}; ///< 机体足印碰撞检查半径 (m)
};

class ElevationLocalPlanner : public elevation_planner::LocalPlannerInterface
{
public:
  ElevationLocalPlanner();
  ~ElevationLocalPlanner() override = default;

  void setConfig(const LocalPlannerConfig & config)
  {
    config_ = config;
    collision_checker_ = CollisionChecker(config.footprint_radius, 0.50);
  }
  void setGraph(const elevation_planner::ManifoldGraph & graph) { graph_ = graph; }

  bool setPlan(const nav_msgs::Path & global_path) override;
  bool computeVelocityCommands(geometry_msgs::Twist & cmd_vel) override;
  bool computeVelocityCommands(const geometry_msgs::PoseStamped & current_pose,
                               geometry_msgs::Twist & cmd_vel);
  bool isGoalReached() const override { return goal_reached_; }

private:
  LocalPlannerConfig config_;
  elevation_planner::ManifoldGraph graph_;
  CollisionChecker collision_checker_;
  nav_msgs::Path global_path_;
  size_t target_idx_{0};
  bool goal_reached_{false};
};

} // namespace elevation_local_planner
