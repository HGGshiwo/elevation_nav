#pragma once

#include "elevation_planner_core/manifold_graph.hpp"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <vector>

namespace elevation_planner
{

/**
 * @brief 全局跨层路径规划器纯虚抽象基类
 */
class GlobalPlannerInterface
{
public:
  virtual ~GlobalPlannerInterface() = default;

  virtual bool initialize(const ManifoldGraph & graph) = 0;
  virtual bool plan(const geometry_msgs::PoseStamped & start,
                    const geometry_msgs::PoseStamped & goal,
                    nav_msgs::Path & out_path) = 0;
};

/**
 * @brief 局部轨迹规划与机体控制纯虚抽象基类
 */
class LocalPlannerInterface
{
public:
  virtual ~LocalPlannerInterface() = default;

  virtual bool setPlan(const nav_msgs::Path & global_path) = 0;
  virtual bool computeVelocityCommands(geometry_msgs::Twist & cmd_vel) = 0;
  virtual bool isGoalReached() const = 0;
};

} // namespace elevation_planner
