#ifndef ELEVATION_LOCAL_PLANNER_ASTAR_LOCAL_PLANNER_H_
#define ELEVATION_LOCAL_PLANNER_ASTAR_LOCAL_PLANNER_H_

#include <ros/ros.h>
#include <nav_core/base_local_planner.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/PointCloud2.h>
#include <vector>
#include <string>
#include <memory>
#include <mutex>
#include <atomic>
#include <Eigen/Core>

#include <elevation_planner_core/manifold_graph.hpp>
#include <elevation_planner_core/graph_store.hpp>
#include <elevation_planner_core/path_simplifier.hpp>
#include "elevation_local_planner/control_types.h"
#include "elevation_local_planner/velocity_smoother.h"
#include "elevation_local_planner/collision_checker.hpp"
#include "elevation_local_planner/kinematic_astar.hpp"
#include "elevation_local_planner/bspline_trajectory.hpp"
#include "elevation_local_planner/sfc_corridor.hpp"
#include "elevation_local_planner/scan_bspline_optimizer.hpp"

namespace elevation_local_planner
{

/**
 * @class AStarLocalPlanner
 * @brief 基于三维流形图运动学 A* 搜索与轻量三次 B 样条解析速度前馈的局部规划器
 * @details 全程 100% 运行在流形图拓扑结构内部，无任何空间逆向反查，零跳层、零越界、天然高阶 C^2 平滑。
 */
class AStarLocalPlanner : public nav_core::BaseLocalPlanner
{
public:
  AStarLocalPlanner();
  virtual ~AStarLocalPlanner() = default;

  virtual void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros) override;
  virtual bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) override;
  virtual bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel) override;

  /// 冻结模式的实际规划体 (wrapper 按需调用; 冻结时输出零速、恒返回 true)
  bool computeVelocityCommandsImpl(geometry_msgs::Twist& cmd_vel);

  virtual bool isGoalReached() override;

private:
  // 位姿与坐标获取
  bool lookupRobotPose2D(RobotPose2D & robot_pose);
  bool lookupRobotPose3D(double & x, double & y, double & z);
  bool transformToBase(const geometry_msgs::PoseStamped & pose_in, geometry_msgs::PoseStamped & pose_out);
  bool computeFinalYawErrorXY(const geometry_msgs::PoseStamped & final_pose_in, double & yaw_error);
  double remainingPlanLength3D(const RobotPose2D & robot_pose) const;

  // 局部路径截取与图内运动学主动重搜
  std::vector<geometry_msgs::PoseStamped> extractLocalBand(const RobotPose2D & robot_pose, double horizon_dist);
  bool isPathBlocked(const std::vector<geometry_msgs::PoseStamped> & path,
                     const elevation_planner::ManifoldGraph & graph,
                     double check_dist,
                     size_t & blocked_idx);
  std::vector<geometry_msgs::PoseStamped> checkAndReplanKinematicDetour(
    const std::vector<geometry_msgs::PoseStamped> & local_band,
    const RobotPose2D & robot_pose,
    const elevation_planner::ManifoldGraph & graph);

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

  // ---- 局部规划与平滑关键参数 ----
  double local_horizon_distance_{2.50};    ///< 前方宏观切片长度 (m)
  double obstacle_check_distance_{1.50};   ///< 前方障碍预警检测距离 (m)
  double max_step_height_{0.25};           ///< 单步垂直高度允许极限 (m)
  double max_stride_length_{0.35};         ///< 单步水平跨步允许极限 (m)
  double weight_turn_{1.5};               ///< 运动学 A* 三点转角平滑惩罚权重
  double max_lateral_acc_{0.80};           ///< 弯道最大向心加速度 (用于曲率自适应降速)
  bool los_prune_enabled_{true};           ///< SC-LOS 带内剪枝开关
  double los_max_segment_{0.60};           ///< 剪枝最大段长 (m, 2D)
  std::string sfc_corridor_mode_{"segment"};  ///< 走廊模式: segment=段式(沿线发散) / point=点式(旧行为)
  bool sfc_seed_robot_state_{true};           ///< 起始两条段走廊并入机器人位姿/锚点种子 (转角交集覆盖动力学锚点)
  double rebound_weight_{100.0};              ///< 样条优化 rebound 定向排斥权重 (0 = 关闭)
  double rebound_clearance_{0.17};            ///< rebound 安全间距 (= body_hard_radius, 与代价地图同口径)
  double body_hard_radius_{0.17};             ///< 机体硬半径 (planner_common 同源): 走廊宽度闸 = 2×该值

  // ---- 冻结调试模式 (freeze:=true): 狗不动、不自动重规划, 摆障碍/设终点时按需规划一轮 ----
  bool planning_freeze_{false};               ///< 冻结总开关
  std::atomic<bool> plan_once_pending_{false};///< 单次规划请求 (setPlan/摆障碍直接触发/手动话题置位)

  double goal_pos_tol_{0.08};
  double goal_z_tol_{0.15};   ///< 到达判定的 z 轴容差 (m), 与踏面高程同基准 (base_link 贴地)
  double goal_yaw_tol_{0.10};
  double linear_gain_{1.2};
  double lateral_gain_{0.4};
  double heading_gain_{1.2};
  double final_yaw_gain_{0.6};
  bool enable_lateral_motion_{true};
  bool align_final_yaw_{true};

  // 算法模块
  VelocitySmoother velocity_smoother_;
  CollisionChecker collision_checker_;
  KinematicAStar kinematic_astar_;
  BSplineTrajectory bspline_traj_;
  ScanBsplineOptimizer scan_optimizer_;
  elevation_planner::PathSimplifier band_simplifier_;      ///< SC-LOS 带内剪枝器 (与全局共用同一检测逻辑)
  const elevation_planner::ManifoldGraph* simplifier_graph_{nullptr};  ///< 已绑定图实例 (指针换图时重建绑定)

  geometry_msgs::Twist last_cmd_vel_;

  // 话题发布
  ros::Publisher local_plan_pub_;
  ros::Publisher local_spline_plan_pub_;
  ros::Publisher local_corridor_pub_;
  ros::Publisher local_corridor_debug_pub_;
  ros::Publisher local_se_debug_pub_;      ///< 起终点高亮 + 样条线违例着色 (调试)
  ros::Publisher rebound_debug_pub_;       ///< rebound 定向排斥向量可视化 (障碍面 → 控制点箭头)

  /**
   * @brief [DBG] 起终点高亮 + 线级约束检查 MarkerArray
   *        原始起点/终点 (A* 链未改写前) vs 修改后起点/终点, 各带文字标签;
   *        control_points 非空时输出两组线:
   *        - 控制折线 (逐边): 边两端点均在本段走廊自身行内 => 绿 (凸性保证整边在走廊内), 否则红
   *        - 样条曲线 (分段): 该段 4 个跨控制点均在走廊自身行内 => 绿 (曲线保证在走廊内);
   *          保证破缺时按采样点自身违例着色: 黄 = 无保证但在内, 红 = 实际出界
   */
  void publishSeDebugMarkers(const geometry_msgs::Point& orig_start,
                             const geometry_msgs::Point& mod_start,
                             const geometry_msgs::Point& orig_end,
                             const geometry_msgs::Point& mod_end,
                             const nav_msgs::Path* spline_path,
                             const std::vector<ConvexCorridor2D>& corridors,
                             const std::vector<Eigen::Vector2d>* control_points,
                             double knot_dt,
                             double path_step_dt);
};

} // namespace elevation_local_planner

#endif // ELEVATION_LOCAL_PLANNER_ASTAR_LOCAL_PLANNER_H_
