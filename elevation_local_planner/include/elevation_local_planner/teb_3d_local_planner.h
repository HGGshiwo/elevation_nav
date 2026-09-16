#pragma once

#include <ros/ros.h>
#include <nav_core/base_local_planner.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <costmap_converter/ObstacleArrayMsg.h>
#include <base_local_planner/odometry_helper_ros.h>

#include <teb_local_planner/teb_config.h>
#include <teb_local_planner/optimal_planner.h>
#include <teb_local_planner/homotopy_class_planner.h>
#include <teb_local_planner/visualization.h>
#include <teb_local_planner/robot_footprint_model.h>

#include "elevation_local_planner/manifold_obstacles_3d.h"
#include <elevation_planner_core/trajectory_validator.hpp>

#include <vector>
#include <string>
#include <mutex>
#include <memory>

namespace elevation_local_planner
{

/**
 * @class Teb3DLocalPlanner
 * @brief 深度融合三维高程流形图的 TEB 局部轨迹规划器
 * 
 * 1. 原生复用 teb_local_planner 的时空弹性带 (Timed-Elastic-Band) 与 g2o 优化图
 * 2. 对接全套三维几何障碍物 (LineObstacle3D, CylinderObstacle3D, PolygonObstacle3D)，以空间真实三维距离计算势场
 * 3. 在多同伦探索阶段，使用 TrajectoryValidator 在流形图上查询连通性、步高极限与终点 Z，优选出真正合法的 3D 轨迹
 */
class Teb3DLocalPlanner : public nav_core::BaseLocalPlanner
{
public:
  Teb3DLocalPlanner();
  virtual ~Teb3DLocalPlanner();

  virtual void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros) override;
  virtual bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) override;
  virtual bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel) override;
  virtual bool isGoalReached() override;

private:
  void customObstacleCB(const costmap_converter::ObstacleArrayMsg::ConstPtr& obst_msg);
  void updateObstaclesFromMsg();

  bool pruneGlobalPlan(const geometry_msgs::PoseStamped& global_pose,
                       std::vector<geometry_msgs::PoseStamped>& global_plan,
                       double dist_behind_robot);

  bool transformGlobalPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan,
                           const geometry_msgs::PoseStamped& global_pose,
                           const costmap_2d::Costmap2D& costmap,
                           const std::string& global_frame,
                           double max_plan_length,
                           std::vector<geometry_msgs::PoseStamped>& transformed_plan);

  void updateViaPointsSafe(const std::vector<geometry_msgs::PoseStamped>& transformed_plan,
                          double min_separation);

private:
  bool initialized_{false};
  std::string name_;
  tf2_ros::Buffer* tf_{nullptr};
  costmap_2d::Costmap2DROS* costmap_ros_{nullptr};
  costmap_2d::Costmap2D* costmap_{nullptr};
  std::string global_frame_;
  std::string robot_base_frame_;

  teb_local_planner::TebConfig cfg_;
  teb_local_planner::PlannerInterfacePtr planner_;
  teb_local_planner::ObstContainer obstacles_;
  teb_local_planner::ViaPointContainer via_points_;
  teb_local_planner::TebVisualizationPtr visualization_;

  base_local_planner::OdometryHelperRos odom_helper_;

  ros::Subscriber custom_obst_sub_;
  std::mutex custom_obst_mutex_;
  costmap_converter::ObstacleArrayMsg custom_obstacle_msg_;
  bool has_new_obstacles_{false};

  std::vector<geometry_msgs::PoseStamped> global_plan_;
  geometry_msgs::PoseStamped current_robot_pose_;
  geometry_msgs::Twist robot_vel_;

  double max_step_height_{0.25};
  double plan_slice_horizon_{2.5};
  double dog_height_{0.45};
};

} // namespace elevation_local_planner
