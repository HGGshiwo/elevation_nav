#ifndef ELEVATION_LOCAL_PLANNER_ASTAR_LOCAL_PLANNER_H_
#define ELEVATION_LOCAL_PLANNER_ASTAR_LOCAL_PLANNER_H_

#include <ros/ros.h>
#include <nav_core/base_local_planner.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <vector>
#include <string>
#include <memory>
#include <mutex>

#include <elevation_planner_core/manifold_graph.hpp>
#include <elevation_planner_core/graph_store.hpp>
#include <elevation_global_planner/manifold_astar.hpp>
#include <elevation_global_planner/path_smoother.hpp>
#include "elevation_local_planner/control_types.h"
#include "elevation_local_planner/velocity_smoother.h"
#include "elevation_local_planner/collision_checker.hpp"

namespace elevation_local_planner
{

/**
 * @class AStarLocalPlanner
 * @brief 基于三维流形拓扑图的手动调用 A* 避障与前瞻跟踪局部规划器
 */
class AStarLocalPlanner : public nav_core::BaseLocalPlanner
{
public:
  AStarLocalPlanner();
  virtual ~AStarLocalPlanner() = default;

  virtual void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros) override;
  virtual bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) override;
  virtual bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel) override;
  virtual bool isGoalReached() override;

private:
  // 位姿与变换
  bool lookupRobotPose2D(RobotPose2D & robot_pose);
  bool lookupRobotPose3D(double & x, double & y, double & z);
  bool transformToBase(const geometry_msgs::PoseStamped & pose_in, geometry_msgs::PoseStamped & pose_out);
  bool computeFinalYawErrorXY(const geometry_msgs::PoseStamped & final_pose_in, double & yaw_error);

  // 局部路径处理与主动 A* 避障
  std::vector<geometry_msgs::PoseStamped> extractLocalBand(const RobotPose2D & robot_pose, double horizon_dist);
  bool isPathBlocked(const std::vector<geometry_msgs::PoseStamped> & path,
                     const elevation_planner::ManifoldGraph & graph,
                     double check_dist,
                     size_t & blocked_idx);
  std::vector<geometry_msgs::PoseStamped> checkAndReplanAStarDetour(
    const std::vector<geometry_msgs::PoseStamped> & local_band,
    const RobotPose2D & robot_pose,
    const elevation_planner::ManifoldGraph & graph);

  // 前瞻跟踪与目标选择
  bool selectTrackingTarget(const std::vector<geometry_msgs::PoseStamped> & plan, TrackingTarget & target);
  bool isFinalTrackingPointReached(const TrackingTarget & target) const;

  tf2_ros::Buffer* tf_buffer_{nullptr};
  costmap_2d::Costmap2DROS* costmap_ros_{nullptr};
  bool initialized_{false};
  std::string map_frame_{"map"};

  // 全局路径与跟踪状态
  std::vector<geometry_msgs::PoseStamped> global_plan_;
  std::vector<geometry_msgs::PoseStamped> active_local_plan_;
  int target_index_{0};
  int global_tracking_index_{0};
  bool pose_adjusting_{false};
  bool goal_reached_{true};
  ros::Time last_control_time_;

  // ---- 全部前瞻与避障参数均可配置 (非硬编码) ----
  double local_horizon_distance_{2.5};    ///< 前方宏观切片长度 (m)
  double lookahead_distance_{0.45};       ///< 速度控制前瞻采样距离 (m)
  double obstacle_check_distance_{1.5};   ///< 前方障碍预警检测距离 (m)
  double detour_clearance_margin_{0.20};  ///< 绕障搜索额外容差 (m)
  double max_step_height_{0.25};          ///< 单步垂直高度允许极限 (m)

  double tracking_xy_tol_{0.20};
  double goal_pos_tol_{0.05};
  double goal_yaw_tol_{0.10};
  double linear_gain_{1.2};
  double lateral_gain_{0.4};
  double heading_gain_{1.2};
  double final_yaw_gain_{0.5};
  bool enable_lateral_motion_{true};
  bool align_final_yaw_{true};

  // 算法模块
  VelocitySmoother velocity_smoother_;
  CollisionChecker collision_checker_;
  elevation_global_planner::ManifoldAStarPlanner astar_planner_;
  elevation_global_planner::PathSmoother path_smoother_;

  geometry_msgs::Twist last_cmd_vel_;

  // 话题发布
  ros::Publisher local_plan_pub_;
  ros::Publisher local_astar_plan_pub_;
};

} // namespace elevation_local_planner

#endif // ELEVATION_LOCAL_PLANNER_ASTAR_LOCAL_PLANNER_H_
