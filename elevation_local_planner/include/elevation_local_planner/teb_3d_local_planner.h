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
#include <elevation_planner_core/frenet_frame.hpp>

#include <vector>
#include <string>
#include <mutex>
#include <memory>

namespace elevation_local_planner
{

/**
 * @class Teb3DLocalPlanner
 * @brief 深度融合三维高程流形图与 Frenet 空间参数化的 TEB 局部轨迹规划器
 * 
 * 1. 沿 3D A* 规划路径实时构建无投影失真的 Frenet (s, l) 坐标系，从根本上杜绝上下层重叠与空间多义性
 * 2. 管道物理断面作为 Frenet 左右硬边界 LineObstacle，TEB 在管道内部自由避障优化，无需任何脆弱的 via-points
 * 3. 通过真实 3D 弧长累积与切线微分几何，完美自适应大坡度攀爬、斜坡与直角/螺旋大转弯，输出带解析曲率前馈的运动指令
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
  void populateFrenetObstacles(const elevation_planner::FrenetFrame& frenet_frame,
                               double s_start, double s_end, double l_robot);

  bool pruneGlobalPlan(const geometry_msgs::PoseStamped& global_pose,
                       std::vector<geometry_msgs::PoseStamped>& global_plan,
                       double dist_behind_robot);

  bool transformGlobalPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan,
                           const geometry_msgs::PoseStamped& global_pose,
                           const costmap_2d::Costmap2D& costmap,
                           const std::string& global_frame,
                           double max_plan_length,
                           std::vector<geometry_msgs::PoseStamped>& transformed_plan);

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
  double corridor_radius_{2.5};

  ros::Publisher corridor_pub_;
  ros::Publisher corridor_boundary_pub_;
  ros::Time last_corridor_pub_time_{0};
};

} // namespace elevation_local_planner
