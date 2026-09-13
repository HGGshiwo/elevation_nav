#ifndef ELEVATION_LOCAL_PLANNER_ELEVATION_LOCAL_PLANNER_PLUGIN_H_
#define ELEVATION_LOCAL_PLANNER_ELEVATION_LOCAL_PLANNER_PLUGIN_H_

#include <ros/ros.h>
#include <nav_core/base_local_planner.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>
#include <memory>

#include "elevation_planner_core/manifold_graph.hpp"
#include "elevation_planner_core/graph_store.hpp"
#include "elevation_local_planner/d1_control_types.h"
#include "elevation_local_planner/d1_velocity_smoother.h"
#include "elevation_local_planner/collision_checker.hpp"

namespace elevation_local_planner
{

/**
 * @brief move_base 局部规划器插件: D1 比例跟踪控制 + 融合图足印碰撞安全层。
 *
 * 跟踪层: 3D 前瞻投影选点 (D1ControlUtils) + 纵/横/航向三通道比例控制 + 加速度限幅平滑,
 *         移植自 octo_planner D1LocalPlanner, 移除 OpenCV 调试图。
 * 安全层: initialize() 启动后台线程, 每帧将实时点云裁剪为机器人周围 ROI 滚动窗口,
 *         与 GraphStore 中的全局先验柱表逐柱并集融合后重建流形图 (双缓冲刷新);
 *         控制周期入口先对当前位置做足印碰撞检查, 命中禁行节点即零速停车。
 */
class ElevationLocalPlannerPlugin : public nav_core::BaseLocalPlanner
{
public:
  ElevationLocalPlannerPlugin();
  virtual ~ElevationLocalPlannerPlugin();

  // nav_core::BaseLocalPlanner interface methods
  virtual void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros) override;
  virtual bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) override;
  virtual bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel) override;
  virtual bool isGoalReached() override;

private:
  // ---- 跟踪控制 (D1) ----
  bool isFinalTrackingPointReached(const TrackingTarget & target) const;
  bool selectTrackingTarget(TrackingTarget & target);
  bool lookupRobotPose2D(RobotPose2D & robot_pose);
  bool computeFinalYawErrorXY(const geometry_msgs::PoseStamped & final_pose_in, double & yaw_error);
  bool transformToBase(const geometry_msgs::PoseStamped & pose_in, geometry_msgs::PoseStamped & pose_out);

  // ---- 融合图安全层 ----
  void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr & msg);
  void fusionLoop();
  bool lookupRobotPose3D(double & x, double & y, double & z);
  bool checkFusedGraphCollision(double x, double y, double z);

  // TF & Costmap Pointers
  tf2_ros::Buffer* tf_buffer_;
  costmap_2d::Costmap2DROS* costmap_ros_;
  bool initialized_;

  // ROS communications
  ros::NodeHandle nh_;
  ros::Subscriber cloud_sub_;
  ros::Time last_control_time_;

  // ---- 控制参数 (D1) ----
  std::string map_frame_;
  double lookahead_distance_, tracking_xy_tol_;
  double goal_pos_tol_, goal_yaw_tol_;
  double linear_gain_, lateral_gain_, heading_gain_, final_yaw_gain_;
  bool   enable_lateral_motion_;
  bool   align_final_yaw_;

  // ---- 建图/融合参数 (与全局插件同源, launch 统一注入) ----
  elevation_planner::GraphBuildConfig build_cfg_;
  double crop_radius_xy_{1.5};      ///< ROI 滚动窗口半径 (m), 窗口边长 = 2 * crop_radius_xy
  double crop_height_above_{2.0};   ///< ROI 裁剪上界: 机器人脚下平面以上 (m)
  double crop_height_below_{1.0};   ///< ROI 裁剪下界: 机器人脚下平面以下 (m)
  double fusion_rate_{5.0};         ///< 融合图刷新频率 (Hz)
  double collision_footprint_radius_{0.30}; ///< 安全层足印碰撞检查半径 (m)

  // Planner components & state
  D1VelocitySmoother velocity_smoother_;
  CollisionChecker collision_checker_;
  std::vector<geometry_msgs::PoseStamped> global_plan_;
  int    target_index_;
  bool   pose_adjusting_;
  bool   goal_reached_;
  geometry_msgs::Twist last_cmd_vel_;

  // ---- 融合图后台线程 ----
  elevation_planner::CloudGraphBuilder builder_;
  std::thread fusion_thread_;
  std::atomic<bool> fusion_running_{false};
  std::atomic<bool> has_map_{false};
  std::mutex cloud_mutex_;
  sensor_msgs::PointCloud2::ConstPtr latest_cloud_;
  std::atomic<bool> cloud_dirty_{false};
  ros::Time last_cloud_stamp_;

  // 双缓冲融合图: 融合线程写 / 控制循环读, 取 shared_ptr 拷贝后无锁使用
  std::mutex fused_graph_mutex_;
  std::shared_ptr<const elevation_planner::ManifoldGraph> fused_graph_;
};

} // namespace elevation_local_planner

#endif // ELEVATION_LOCAL_PLANNER_ELEVATION_LOCAL_PLANNER_PLUGIN_H_
