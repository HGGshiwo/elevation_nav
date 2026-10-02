#include "elevation_local_planner/astar_local_planner.h"
#include "elevation_planner_core/crash_handler.hpp"
#include <pluginlib/class_list_macros.h>
#include <chrono>
#include <string>
#include <vector>

namespace
{
// 规划分段耗时打点: RAII, 函数任意出口 (含提前 return false) 都会自动打印阶段表
struct PerfTrace
{
  using Clock = std::chrono::steady_clock;
  Clock::time_point t0_;
  Clock::time_point t_mark_;
  std::vector<std::pair<std::string, double>> marks_;
  std::string name_;

  explicit PerfTrace(std::string name) : t0_(Clock::now()), t_mark_(t0_), name_(std::move(name)) {}
  void mark(const std::string & stage)
  {
    auto now = Clock::now();
    marks_.emplace_back(stage, std::chrono::duration<double, std::milli>(now - t_mark_).count());
    t_mark_ = now;
  }
  ~PerfTrace()
  {
    double total = std::chrono::duration<double, std::milli>(Clock::now() - t0_).count();
    std::ostringstream oss;
    oss << "[Perf][" << name_ << "] total=" << total << "ms";
    for (auto & m : marks_) oss << " " << m.first << "=" << m.second << "ms";
    // 慢规划 (>80ms) 无条件打印, 快的节流 5s 防刷屏
    if (total > 80.0)
      ROS_WARN("%s", oss.str().c_str());
    else
      ROS_INFO_THROTTLE(5.0, "%s", oss.str().c_str());
  }
};
} // namespace
#include <std_msgs/Empty.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <angles/angles.h>
#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>

PLUGINLIB_EXPORT_CLASS(elevation_local_planner::AStarLocalPlanner, nav_core::BaseLocalPlanner)

namespace elevation_local_planner
{

AStarLocalPlanner::AStarLocalPlanner()
: tf_buffer_(nullptr),
  costmap_ros_(nullptr),
  initialized_(false),
  target_index_(0),
  global_tracking_index_(0),
  pose_adjusting_(false),
  goal_reached_(true)
{
}

void AStarLocalPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
{
  if (initialized_)
  {
    ROS_WARN("[AStarLocalPlanner] Already initialized, doing nothing.");
    return;
  }

  elevation_planner::CrashHandler::install("/tmp/elevation_local_planner_crash.log");

  tf_buffer_ = tf;
  costmap_ros_ = costmap_ros;

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~/" + name);

  // ---- 坐标系与前瞻避障参数 ----
  private_nh.param<std::string>("map_frame", map_frame_, "map");
  private_nh.param<double>("local_horizon_distance",  local_horizon_distance_,  2.50);
  private_nh.param<double>("obstacle_check_distance", obstacle_check_distance_, 1.50);
  private_nh.param<double>("max_step_height",         max_step_height_,         0.25);
  private_nh.param<double>("max_stride_length",       max_stride_length_,       0.35);
  private_nh.param<bool>("los_prune_enabled",         los_prune_enabled_,       true);
  private_nh.param<double>("los_max_segment",         los_max_segment_,         0.60);
  private_nh.param<std::string>("sfc_corridor_mode",  sfc_corridor_mode_,       "segment");
  private_nh.param<bool>("sfc_seed_robot_state",      sfc_seed_robot_state_,    true);
  private_nh.param<double>("weight_turn",             weight_turn_,             1.50);
  private_nh.param<double>("max_lateral_acc",         max_lateral_acc_,         0.80);

  // ---- 控制与容差参数 ----
  private_nh.param<double>("goal_position_tolerance", goal_pos_tol_, 0.08);
  private_nh.param<double>("goal_z_tolerance",        goal_z_tol_,  0.15);
  private_nh.param<double>("goal_yaw_tolerance",      goal_yaw_tol_, 0.10);
  private_nh.param<double>("linear_gain",             linear_gain_,  1.20);
  private_nh.param<double>("lateral_gain",            lateral_gain_, 0.40);
  private_nh.param<double>("heading_gain",            heading_gain_, 1.20);
  private_nh.param<double>("final_yaw_gain",          final_yaw_gain_, 0.60);
  private_nh.param<bool>  ("enable_lateral_motion",    enable_lateral_motion_, true);
  private_nh.param<bool>  ("align_final_yaw",         align_final_yaw_, true);
  private_nh.param<double>("rebound_weight",          rebound_weight_, 100.0);
  private_nh.param<double>("rebound_clearance",       rebound_clearance_, 0.17);
  // 机体硬半径: 与 planner_common.yaml/伪 TF 桥接同源 (全局参数), 走廊宽度闸阈值 = 2×该值
  nh.param<double>("body_hard_radius", body_hard_radius_, 0.17);
  private_nh.param<bool>  ("planning_freeze",         planning_freeze_, false);
  if (planning_freeze_)
    ROS_WARN("[AStarLocalPlanner] FREEZE mode: robot will not move; planning on goal/obstacle-change (goal republish)");

  // 配置运动学 A* 搜索器参数
  KinematicAStarConfig k_cfg;
  k_cfg.weight_z = 2.0;
  k_cfg.weight_turn = weight_turn_;
  k_cfg.weight_traversability = 3.0;
  k_cfg.max_step_height = max_step_height_;
  k_cfg.max_stride_length = max_stride_length_;
  // [DBG] 绕障预算可调: 预算耗尽会让绕障误判无路, 原封锁链直接进走廊生成
  int detour_max_expansions = 30000;
  private_nh.param<int>("detour_max_expansions", detour_max_expansions, 30000);
  k_cfg.max_expansions = detour_max_expansions;
  kinematic_astar_.setConfig(k_cfg);

  // 速度平滑器配置
  VelocitySmootherParams smoother_params;
  private_nh.param<double>("max_linear_speed",  smoother_params.max_linear_speed,  0.60);
  private_nh.param<double>("max_lateral_speed", smoother_params.max_lateral_speed, 0.30);
  private_nh.param<double>("max_angular_speed", smoother_params.max_angular_speed, 1.00);
  private_nh.param<double>("max_linear_acc",    smoother_params.max_linear_acc,    0.80);
  private_nh.param<double>("max_lateral_acc",   smoother_params.max_lateral_acc,   0.50);
  private_nh.param<double>("max_angular_acc",   smoother_params.max_angular_acc,   1.50);
  private_nh.param<bool>  ("enable_lateral_decoupling", smoother_params.enable_lateral_decoupling, true);
  private_nh.param<double>("linear_deadband",   smoother_params.linear_deadband,   0.02);
  private_nh.param<double>("lateral_deadband",  smoother_params.lateral_deadband,  0.02);
  private_nh.param<double>("angular_deadband",  smoother_params.angular_deadband,  0.02);
  velocity_smoother_.setParams(smoother_params);

  // 话题发布
  local_plan_pub_        = nh.advertise<nav_msgs::Path>("/move_base/local_plan", 1);
  local_spline_plan_pub_ = nh.advertise<nav_msgs::Path>("/move_base/local_spline_plan", 1);
  local_corridor_pub_    = nh.advertise<visualization_msgs::MarkerArray>("/move_base/local_sfc_corridor", 1);
  local_corridor_debug_pub_ = nh.advertise<std_msgs::String>("/elevation_local_corridors_debug", 1);
  local_se_debug_pub_       = nh.advertise<visualization_msgs::MarkerArray>("/move_base/local_se_debug", 1);
  rebound_debug_pub_        = nh.advertise<visualization_msgs::MarkerArray>("/elevation_rebound_debug", 1);

  initialized_ = true;
  ROS_INFO("[AStarLocalPlanner] Initialized with Kinematic A* + Uniform Cubic B-Spline Velocity Engine (horizon: %.2fm, turn_w: %.2f)",
           local_horizon_distance_, weight_turn_);
}

bool AStarLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
{
  if (!initialized_) return false;

  // 冻结模式: 新全局计划 (设置终点) 到达 → 自动规划一轮局部管线
  if (planning_freeze_) plan_once_pending_ = true;

  velocity_smoother_.reset();
  last_control_time_ = ros::Time(0);

  if (plan.empty())
  {
    global_plan_.clear();
    active_local_plan_.clear();
    target_index_ = 0;
    global_tracking_index_ = 0;
    pose_adjusting_ = false;
    goal_reached_ = true;
    return true;
  }

  global_plan_ = plan;
  active_local_plan_ = plan;
  target_index_ = 0;
  pose_adjusting_ = false;
  goal_reached_ = false;

  // 智能继承追踪索引 (防止全局重规划突变重置为 0 导致倒退)
  RobotPose2D robot_pose;
  if (lookupRobotPose2D(robot_pose))
  {
    double min_d = std::numeric_limits<double>::max();
    int best_idx = 0;
    size_t search_limit = std::min(plan.size(), size_t(25));
    for (size_t i = 0; i < search_limit; ++i)
    {
      const auto& p = plan[i].pose.position;
      double dz = std::abs(p.z - robot_pose.z);
      if (dz > 0.35) continue;
      double d = std::hypot(p.x - robot_pose.x, p.y - robot_pose.y) + 4.0 * dz;
      if (d < min_d)
      {
        min_d = d;
        best_idx = static_cast<int>(i);
      }
    }
    global_tracking_index_ = best_idx;
  }
  else
  {
    global_tracking_index_ = 0;
  }

  ROS_INFO("[AStarLocalPlanner] Received new 3D global plan with %zu poses, tracking_idx=%d",
           global_plan_.size(), global_tracking_index_);
  return true;
}

bool AStarLocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
  if (planning_freeze_)
  {
    // 消费单次规划请求; 无请求时廉价空转 (跳过走廊/优化/绕障检测)
    if (!plan_once_pending_.exchange(false))
    {
      cmd_vel = geometry_msgs::Twist();
      return true;   // 恒成功, 避免 move_base 进入 recovery/abort
    }
    (void)computeVelocityCommandsImpl(cmd_vel);
    cmd_vel = geometry_msgs::Twist();   // 只看轨迹不动狗 (伪 TF 侧另有 cmd_vel_freeze 双保险)
    return true;
  }
  return computeVelocityCommandsImpl(cmd_vel);
}

bool AStarLocalPlanner::computeVelocityCommandsImpl(geometry_msgs::Twist& cmd_vel)
{
  if (!initialized_) return false;

  if (global_plan_.empty())
  {
    ROS_WARN_THROTTLE(2.0, "[AStarLocalPlanner] Global plan is empty.");
    cmd_vel = geometry_msgs::Twist();
    return false;
  }

  if (goal_reached_)
  {
    cmd_vel = geometry_msgs::Twist();
    return true;
  }

  RobotPose2D robot_pose;
  if (!lookupRobotPose2D(robot_pose))
  {
    cmd_vel = geometry_msgs::Twist();
    return false;
  }

  // 1. 获取全局共享流形图 (同进程零拷贝)。
  //    融合引擎对这份图原位刷新节点属性 (动态障碍 traversability/headroom),
  //    节点集合与 id 永不改变 —— 全局规划透传的 node_id (orientation.x) 在此有效。
  PerfTrace perf("local");
  auto graph = elevation_planner::GraphStore::instance().getGlobalGraph();

  ros::Time now = ros::Time::now();
  double dt = 0.05;
  if (!last_control_time_.isZero())
  {
    dt = (now - last_control_time_).toSec();
    if (dt <= 1.0e-4 || dt > 0.5) dt = 0.05;
  }
  last_control_time_ = now;

  // 2. 终点直接距离探测与全向对齐吸附模式 (真 3D + 姿态全向解耦微调: 允许正负 vx/vy, 绝不绕圈掉头)
  const auto & final_goal_pose = global_plan_.back();
  const double dist_to_goal_xy = std::hypot(final_goal_pose.pose.position.x - robot_pose.x,
                                            final_goal_pose.pose.position.y - robot_pose.y);
  const double dist_to_goal_z  = std::abs(final_goal_pose.pose.position.z - robot_pose.z);
  const double remaining_dist_3d = remainingPlanLength3D(robot_pose);

  // 严格的真 3D 接近终点判定：必须剩余 3D 路径 < 0.35m，或 (同层 z 满足 && xy 满足 && 剩余 3D 路径 < 0.60m)
  const bool is_near_final_goal = (remaining_dist_3d < 0.35) ||
                                  (dist_to_goal_xy < 0.35 && dist_to_goal_z < goal_z_tol_ + 0.05 && remaining_dist_3d < 0.60);

  if (!pose_adjusting_ && is_near_final_goal)
  {
    pose_adjusting_ = true;
    ROS_INFO("[AStarLocalPlanner] Near final goal (3D path: %.2fm, direct xy: %.2fm, dz: %.2fm). Activating holonomic goal alignment.",
             remaining_dist_3d, dist_to_goal_xy, dist_to_goal_z);
  }

  if (pose_adjusting_)
  {
    // 防脱靶迟滞保护: 只要未大幅脱离 0.60m 或 z 高程错位，保持吸附模式
    if (dist_to_goal_xy > 0.60 || dist_to_goal_z > goal_z_tol_ + 0.20 || remaining_dist_3d > 1.0)
    {
      pose_adjusting_ = false;
      ROS_WARN("[AStarLocalPlanner] Deviated from goal alignment region (xy: %.3fm, dz: %.3fm, 3D: %.2fm), fallback to trajectory tracking.",
               dist_to_goal_xy, dist_to_goal_z, remaining_dist_3d);
    }
    else
    {
      geometry_msgs::PoseStamped final_pose_base;
      if (!transformToBase(global_plan_.back(), final_pose_base))
      {
        cmd_vel = geometry_msgs::Twist();
        return false;
      }

      double dx_b = final_pose_base.pose.position.x;
      double dy_b = final_pose_base.pose.position.y;
      double dz_b = final_pose_base.pose.position.z;
      double cur_dist_xy = std::hypot(dx_b, dy_b);

      double final_yaw_error = 0.0;
      if (align_final_yaw_)
      {
        if (!computeFinalYawErrorXY(global_plan_.back(), final_yaw_error))
        {
          cmd_vel = geometry_msgs::Twist();
          return false;
        }
      }

      const bool pos_ok = cur_dist_xy < goal_pos_tol_;
      const bool z_ok   = std::abs(dz_b) < goal_z_tol_;
      const bool yaw_ok = !align_final_yaw_ || std::abs(final_yaw_error) < goal_yaw_tol_;
      if (pos_ok && z_ok && yaw_ok)
      {
        goal_reached_ = true;
        velocity_smoother_.reset();
        cmd_vel = geometry_msgs::Twist();
        ROS_INFO("[AStarLocalPlanner] Goal successfully reached (xy: %.3fm, dz: %.3fm, yaw: %.3frad).",
                 cur_dist_xy, dz_b, final_yaw_error);
        return true;
      }

      // 全向速度限幅与比例控制 (前后进退、左右平移、原地自转三自由度解耦)
      const double max_align_v = 0.25;
      const double max_align_w = 0.60;
      geometry_msgs::Twist raw_cmd;
      raw_cmd.linear.x = std::max(-max_align_v, std::min(max_align_v, dx_b * linear_gain_));
      raw_cmd.linear.y = enable_lateral_motion_ ? std::max(-max_align_v, std::min(max_align_v, dy_b * lateral_gain_)) : 0.0;
      raw_cmd.linear.z = 0.0;

      if (align_final_yaw_)
      {
        raw_cmd.angular.z = std::max(-max_align_w, std::min(max_align_w, final_yaw_error * final_yaw_gain_));
      }
      else if (cur_dist_xy > 0.05)
      {
        double angle_to_target = std::atan2(dy_b, dx_b);
        raw_cmd.angular.z = std::max(-max_align_w, std::min(max_align_w, angle_to_target * heading_gain_));
      }

      cmd_vel = velocity_smoother_.smooth(raw_cmd, dt);
      last_cmd_vel_ = cmd_vel;
      return true;
    }
  }

  // 3. 截取前方局部切片 (2.5m 视距，严格单调向前推进，彻底防止发夹弯跳点)
  std::vector<geometry_msgs::PoseStamped> local_band = extractLocalBand(robot_pose, local_horizon_distance_);
  perf.mark("band");
  if (local_band.empty())
  {
    cmd_vel = geometry_msgs::Twist();
    return false;
  }

  // 4. 提取局部拓扑 node_id 序列
  //    全局先验图上, 路径 pose 的 orientation.x 透传全局图 node_id (方案 B 零查图透传);
  //    实时融合图逐帧重建, 节点 id 与全局图完全不同源 —— 把全局 id 当融合图索引用,
  //    轻则全部越界过滤 (路径节点序列清空 → 走廊/优化静默跳过), 重则错指到无关节点。
  //    故仅当当前图即全局先验图时信任透传 id, 否则按坐标在当前图上重新锚定。
  std::vector<uint32_t> path_node_ids;
  if (graph && graph->numNodes() > 0)
  {
    const auto & global_graph = elevation_planner::GraphStore::instance().getGlobalGraph();
    const bool ids_from_same_graph = (graph.get() == global_graph.get());
    size_t dropped = 0;
    for (const auto& ps : local_band)
    {
      if (ids_from_same_graph)
      {
        uint32_t nid = static_cast<uint32_t>(std::round(ps.pose.orientation.x));
        if (nid < graph->numNodes())
        {
          path_node_ids.push_back(nid);
        }
        else
        {
          ++dropped;
        }
      }
      else
      {
        uint32_t nid = 0;
        // include_blocked=true: 路径锚定要感知动态障碍封锁的节点 ( id 与图同源后按坐标映射)
        if (graph->findClosestNode(ps.pose.position.x, ps.pose.position.y, ps.pose.position.z, nid, 0.4, 0.6, true))
        {
          path_node_ids.push_back(nid);
        }
        else
        {
          ++dropped;
        }
      }
    }
    if (dropped > 0)
    {
      ROS_WARN_THROTTLE(2.0, "[AStarLocalPlanner] %zu/%zu path waypoints not anchored on active graph (%s).",
                        dropped, local_band.size(), ids_from_same_graph ? "global-prior" : "fused");
    }
  }

  // 4.1 链可通行不变式 + 绕障: 扫描锚定链的第一个硬封锁节点 (trav>=0.95;
  //     顶头禁行与侧向硬阻挡在图口径下同为 1.0)。
  //     不变量: 进入走廊/ALM 的链全程可通行 —— 封锁节点绝不作航点/走廊种子,
  //     否则 ALM 会把控制点钉在障碍坐标上 (chain census / audit d=0.000 的根源)。
  if (graph && graph->numNodes() > 0 && !path_node_ids.empty())
  {
    size_t k = path_node_ids.size();
    for (size_t i = 0; i < path_node_ids.size(); ++i)
    {
      if (graph->getNode(path_node_ids[i]).traversability >= 0.95f)
      {
        k = i;
        break;
      }
    }

    if (k < path_node_ids.size())
    {
      const auto & blk = graph->getNode(path_node_ids[k]);

      // 绕障目标: 封锁位置之后索引最大的可通行节点 (链尾可走即取链尾;
      // 链尾自身被封锁时退而取封锁点之后最远的可通行节点 —— 旧逻辑无条件取链尾,
      // 链尾恰好被封锁时绕障必然无解, 是 ok/failed 随融合抖动翻转的来源之一)
      int goal_j = -1;
      for (int j = static_cast<int>(path_node_ids.size()) - 1; j > static_cast<int>(k); --j)
      {
        if (graph->getNode(path_node_ids[j]).traversability < 0.95f)
        {
          goal_j = j;
          break;
        }
      }

      const uint32_t start_nid = path_node_ids.front();
      uint32_t goal_nid = (goal_j >= 0) ? path_node_ids[goal_j] : 0;
      std::vector<uint32_t> detour_nids;
      std::vector<Eigen::Vector3d> detour_pts;
      if (goal_nid != 0 &&
          kinematic_astar_.search(*graph, start_nid, goal_nid, detour_nids, detour_pts) &&
          !detour_nids.empty())
      {
        ROS_INFO_THROTTLE(2.0, "[AStarLocalPlanner] Detour A* ok: start=%u goal=%u -> %zu nodes (chain replaced, blockage at idx %zu)",
                          start_nid, goal_nid, detour_nids.size(), k);
        path_node_ids = detour_nids;
      }
      else
      {
        // 绕障无路 (或封锁点之后整段无可通行节点): 只保留可通行前缀 (idx [0,k))
        // 并返回 false, 强制 move_base 在融合图上重新全局规划; 全局也无解则明式
        // abort —— 绝不让封锁节点进链继续走廊/ALM (那会规划出穿障轨迹)
        ROS_WARN_THROTTLE(2.0, "[AStarLocalPlanner] blockage at chain idx %zu/%zu (node %u at %.2f,%.2f), detour %s; "
                               "traversable prefix = %zu nodes -> return false, hand over to global planner",
                          k, path_node_ids.size(), path_node_ids[k], blk.x, blk.y,
                          goal_nid != 0 ? "failed (see KinematicAStar)" : "target invalid (no free node beyond blockage)",
                          k);
        cmd_vel = geometry_msgs::Twist();
        return false;
      }
    }
  }

  // [DBG] 链内封锁节点普查: 无论绕障成败, 最终链上残留的封锁节点 (trav>=0.95)
  //       就是"航点在障碍里"的直接来源 —— 0 才是健康状态
  if (graph && graph->numNodes() > 0 && !path_node_ids.empty())
  {
    size_t blocked_in_chain = 0;
    uint32_t first_blocked = 0;
    for (uint32_t nid : path_node_ids)
    {
      if (nid < graph->numNodes() && graph->getNode(nid).traversability >= 0.95f)
      {
        ++blocked_in_chain;
        if (first_blocked == 0) first_blocked = nid;
      }
    }
    if (blocked_in_chain > 0)
      ROS_WARN_THROTTLE(1.0, "[AStarLocalPlanner] chain census: %zu/%zu path nodes HARD-BLOCKED (first nid=%u at %.2f,%.2f), waypoints sit on obstacles",
                        blocked_in_chain, path_node_ids.size(), first_blocked,
                        graph->getNode(first_blocked).x, graph->getNode(first_blocked).y);
  }

  // 保证至少有 4 个节点满足三次 B 样条阶数
  if (graph && !path_node_ids.empty())
  {
    while (path_node_ids.size() < 4)
    {
      uint32_t last_id = path_node_ids.back();
      std::vector<uint32_t> nbrs;
      graph->getSingleStepNeighbors(last_id, nbrs, max_step_height_, max_stride_length_);
      if (!nbrs.empty()) {
        path_node_ids.push_back(nbrs.front());
      } else {
        path_node_ids.push_back(last_id);
      }
    }
  }

  // 4.2 SC-LOS 带内剪枝: 与全局规划共用同一支撑链直线连通检测器,
  //     合并直线可通行的节点段 (楼梯区 z 链断裂自动保留密集点, 安全兜底)
  perf.mark("detour");
  if (los_prune_enabled_ && graph && path_node_ids.size() > 2)
  {
    if (simplifier_graph_ != graph.get() || band_simplifier_.boundNodeCount() != graph->numNodes())
    {
      band_simplifier_.build(*graph);
      simplifier_graph_ = graph.get();
    }
    std::vector<uint32_t> pruned_ids = band_simplifier_.simplifyPath(path_node_ids, los_max_segment_);
    if (pruned_ids.size() >= 4 && pruned_ids.size() < path_node_ids.size())
    {
      ROS_DEBUG_THROTTLE(5.0, "[AStarLocalPlanner] SC-LOS band pruned: %zu -> %zu waypoints",
                        path_node_ids.size(), pruned_ids.size());
      path_node_ids = std::move(pruned_ids);
    }
  }

  perf.mark("los");
  // 5. 纯通过 A* 的 node_id 序列直接提取 2D 坐标 (零坐标查图)
  std::vector<Eigen::Vector2d> waypoints_2d;
  waypoints_2d.reserve(path_node_ids.size());
  for (uint32_t nid : path_node_ids)
  {
    const auto& nd = graph->getNode(nid);
    waypoints_2d.emplace_back(nd.x, nd.y);
  }

  // 6. 纯 2D 安全凸走廊 (SFC 2D) 膨胀与 1:1 ALM 拟牛顿优化
  const double cruise_speed = velocity_smoother_.getParams().max_linear_speed;
  double knot_dt = 0.15;

  bool opt_success = false;
  std::vector<ConvexCorridor2D> corridors_2d;
  std::vector<Eigen::Vector2d> opt_control_points_2d;

  if (graph && graph->numNodes() > 0 && path_node_ids.size() >= 4)
  {
    // 6.0 机器人当前 2D 运动矢量 + 航点起点改写 + 动态时间分配
    Eigen::Vector2d robot_start_2d(robot_pose.x, robot_pose.y);
    if (!waypoints_2d.empty())
    {
      waypoints_2d.front() = robot_start_2d;
    }
    double curr_lin_v = std::max(0.0, last_cmd_vel_.linear.x);
    Eigen::Vector2d v_start_2d(curr_lin_v * std::cos(robot_pose.yaw),
                               curr_lin_v * std::sin(robot_pose.yaw));

    // 动态时间分配: dt = 最长边 / 巡航速度 —— 初始参数化速度绝不超限, feas 初始惩罚近似为零
    // 边长用 3D (图节点高程差): 楼梯上 2D 距离低估 1.4~1.8 倍, 会导致参数化踏面速度超限
    double max_edge = 0.0;
    for (size_t k = 0; k + 1 < waypoints_2d.size(); ++k)
    {
      const double dxy = (waypoints_2d[k + 1] - waypoints_2d[k]).norm();
      double dz = 0.0;
      if (k + 1 < path_node_ids.size() && path_node_ids[k + 1] < graph->numNodes())
      {
        const double z0 = (k == 0) ? robot_pose.z
                                   : static_cast<double>(graph->getNode(path_node_ids[k]).z);
        const double z1 = static_cast<double>(graph->getNode(path_node_ids[k + 1]).z);
        dz = std::abs(z1 - z0);
      }
      max_edge = std::max(max_edge, std::sqrt(dxy * dxy + dz * dz));
    }
    knot_dt = std::max(0.10, std::min(5.0, max_edge / std::max(0.05, cruise_speed)));

    // 6.1 生成 1:1 严格对应的 2D 凸多边形走廊 (零 findClosestNode 查点)
    //     segment 模式: 以 SC-LOS 支撑链为种子的多源 BFS, 走廊沿 A→B 连线向外发散 (0.5m 阈值不变);
    //     point 模式: 旧点式扩散, 参数可回退
    if (sfc_corridor_mode_ == "segment")
    {
      if (simplifier_graph_ != graph.get() || band_simplifier_.boundNodeCount() != graph->numNodes())
      {
        band_simplifier_.build(*graph);
        simplifier_graph_ = graph.get();
      }
      // 凸走廊: 走廊纯由 A* 段生成 (状态点不进折线 —— 机器人位姿/锚点由优化器软约束处理)
      std::vector<Eigen::Vector2d> resampled_waypoints;
      corridors_2d = SFCGenerator::generateSegmentCorridors(path_node_ids, *graph, band_simplifier_,
                                                            0.50, max_step_height_, max_stride_length_,
                                                            {}, robot_pose.z, &resampled_waypoints);
      // 对齐校验: 数量不匹配 (异常) 时退回原航点序列
      if (resampled_waypoints.size() == corridors_2d.size() && !resampled_waypoints.empty())
      {
        waypoints_2d = resampled_waypoints;
      }
      // 转角交集拼接: 转角控制点 q_{i+1} ∈ C_i ∩ C_{i+1}, 凸性锁死控制折线不穿墙
      SFCGenerator::stackAdjacentCorridors(corridors_2d);
      perf.mark("sfc");
    }
    else
    {
      corridors_2d = SFCGenerator::generateCorridors(path_node_ids, *graph, 0.50, max_step_height_, max_stride_length_);
    }


    // 发布 3D 贴地凸走廊可视化 MarkerArray 供 RViz / Web 前端渲染 (直接使用走廊节点高程，零二次查图)
    if (local_corridor_pub_.getNumSubscribers() > 0)
    {
      visualization_msgs::MarkerArray corridor_markers;
      SFCGenerator::toVisualMarkers(corridors_2d, map_frame_, corridor_markers);
      local_corridor_pub_.publish(corridor_markers);
    }

    // 6.2 严格绑定走廊 0 中心至当前机器人真实位姿
    if (!corridors_2d.empty())
    {
      corridors_2d.front().center = robot_start_2d;
      corridors_2d.front().z_ref = robot_pose.z;
    }

    // 6.3 1:1 物理点 ALM 内外循环优化求解 (Jerk 极小化 + 线性凸约束穿墙三次重罚 + 外循环拉格朗日乘子更新)
    scan_optimizer_.setParams(1.0, 1000.0, 2.0, 1.0, cruise_speed, 1.5);
    scan_optimizer_.setReboundParams(rebound_weight_, rebound_clearance_);
    // 起步动力学锚点软约束: 拉 q2 靠近 机器人位姿 + knot_dt·当前速度 (硬约束是走廊 C1∩C2)
    scan_optimizer_.setDynamicsAnchor(robot_start_2d + knot_dt * v_start_2d, 10.0);

    // rebound 占据查询回调: 与伪 TF 落脚点预检同语义 —— 控制点 body_hard_radius 内
    // 存在本层 (|dz| <= max_step_height) 禁行节点即碰撞, 输出最近禁行节点作排斥参考点;
    // 融合引擎对全局图原位刷新 traversability, 动态障碍天然生效; 图外/悬空返回 false
    // (不装弹簧, 交走廊 ALM 项兜底)。零 A* 调用: 排斥方向取径向, 绕行侧已由
    // 上游阻挡检测绕障与 SFC 走廊拓扑确定
    auto* graph_ptr = graph.get();
    const double occ_band = max_step_height_;
    const double occ_radius2 = rebound_clearance_ * rebound_clearance_;
    // 查找范围必须覆盖 clearance: 半径 = ceil(clearance/res)+1 格, 否则弹簧对
    // 恰好落在范围外 (0.3~0.35m) 的禁行格失明, 0.35 的间距要求形同虚设
    const int occ_scan = static_cast<int>(std::ceil(rebound_clearance_ / graph_ptr->getResolution())) + 1;
    std::function<bool(double, double, double, Eigen::Vector2d*)> occupancy_check =
        [graph_ptr, occ_band, occ_radius2, occ_scan](double x, double y, double z,
                                                     Eigen::Vector2d* obstacle_pt) -> bool {
          int r = 0, c = 0;
          if (!graph_ptr->toGridIndex(x, y, r, c)) return false;  // 图外: 不装弹簧
          double best_d2 = occ_radius2;
          bool hit = false;
          for (int dr = -occ_scan; dr <= occ_scan; ++dr)
          {
            for (int dc = -occ_scan; dc <= occ_scan; ++dc)
            {
              for (uint32_t nid : graph_ptr->getSpatialCellNodes(r + dr, c + dc))
              {
                const auto& nd = graph_ptr->getNode(nid);
                if (std::fabs(nd.z - z) > occ_band) continue;      // 异层节点不参与
                if (nd.traversability < 0.95f) continue;           // 只关心禁行节点
                const double dx = nd.x - x, dy = nd.y - y;
                const double d2 = dx * dx + dy * dy;
                if (d2 <= best_d2)
                {
                  best_d2 = d2;
                  if (obstacle_pt) *obstacle_pt = Eigen::Vector2d(nd.x, nd.y);
                  hit = true;
                }
              }
            }
          }
          return hit;
        };
    scan_optimizer_.setOccupancyCallback(occupancy_check);

    if (scan_optimizer_.optimize(waypoints_2d, corridors_2d, robot_start_2d, v_start_2d, knot_dt, opt_control_points_2d))
    {
      // [DBG] 优化后控制点占据审计: 用与 rebound 完全相同的占据语义复查最终控制点。
      //       命中 = 优化轨迹本体贴/入禁行区, 但走廊约束 (viol≈0) 仍判成功 —— 当前
      //       管线不会据此触发重规划, 此日志是"控制点在障碍物内"的权威判据
      {
        int hit = 0;
        double worst_d = 1e9;
        size_t worst_i = 0;
        Eigen::Vector2d worst_ob(0.0, 0.0);
        for (size_t k = 0; k < opt_control_points_2d.size(); ++k)
        {
          // 控制点 q_k 的贴地高程取走廊序号 k-1 (与优化器 checkCollisionAndRebound 同口径)
          const size_t ci = (k >= 1) ? k - 1 : 0;
          const double zq = corridors_2d[std::min(ci, corridors_2d.size() - 1)].queryZFromNodes(
              opt_control_points_2d[k].x(), opt_control_points_2d[k].y());
          Eigen::Vector2d ob;
          if (occupancy_check(opt_control_points_2d[k].x(), opt_control_points_2d[k].y(), zq, &ob))
          {
            ++hit;
            const double d = (opt_control_points_2d[k] - ob).norm();
            if (d < worst_d)
            {
              worst_d = d;
              worst_i = k;
              worst_ob = ob;
            }
          }
        }
        if (hit > 0)
          ROS_WARN_THROTTLE(1.0, "[AStarLocalPlanner] post-opt audit: %d/%zu control points within %.2fm of blocked node "
                   "(worst q%zu d=%.3fm, obstacle at %.2f,%.2f) - trajectory on obstacle but corridor constraints satisfied",
                   hit, opt_control_points_2d.size(), rebound_clearance_, worst_i, worst_d,
                   worst_ob.x(), worst_ob.y());
        else
          ROS_INFO_THROTTLE(5.0, "[AStarLocalPlanner] post-opt audit: all %zu control points clear of blocked nodes",
                            opt_control_points_2d.size());
      }

      if (bspline_traj_.initialize2D(opt_control_points_2d, knot_dt))
      {
        opt_success = true;
      }
      else
      {
        // 样条初始化静默失败会让 move_base 无任何解释地停止 —— 必须留痕
        ROS_ERROR("[AStarLocalPlanner] B-spline initialize2D failed: %zu ctrl pts, dt=%.3f (waypoints=%zu, corridors=%zu)",
                  opt_control_points_2d.size(), knot_dt, waypoints_2d.size(), corridors_2d.size());
      }
    }
    else
    {
      ROS_WARN("[AStarLocalPlanner] ALM optimize rejected trajectory (corridor violation > 1cm after %d outer rounds)", 24);
    }

    // 6.4 rebound 定向排斥可视化与诊断:
    //     每个激活约束发布一支箭头 (障碍面参考点 → 控制点), 高度取走廊节点高程贴地;
    //     红 = 控制点仍在 clearance 内 (弹簧受压中), 绿 = 已弹到安全间距之外;
    //     箭头出现即说明拟牛顿识别到了障碍物, 数量与颜色直观反映收敛进度
    {
      const auto& constraints = scan_optimizer_.getReboundConstraints();
      visualization_msgs::MarkerArray arr;
      visualization_msgs::Marker del;
      del.header.frame_id = map_frame_;
      del.action = visualization_msgs::Marker::DELETEALL;
      arr.markers.push_back(del);

      const int active = scan_optimizer_.getReboundActiveCount();
      if (active > 0 && opt_control_points_2d.size() == static_cast<size_t>(scan_optimizer_.getControlPointCount()))
      {
        const ros::Time stamp = ros::Time::now();
        int mid = 0;
        double worst_err = 0.0;
        for (int i = 1; i < static_cast<int>(waypoints_2d.size()); ++i)
        {
          const auto& rc = constraints[i + 1];
          if (rc.direction.squaredNorm() < 0.5) continue;

          const Eigen::Vector2d& q = opt_control_points_2d[i + 1];
          const double z_base = corridors_2d[i].queryZFromNodes(rc.base_point.x(), rc.base_point.y());
          const double z_q = corridors_2d[i].queryZFromNodes(q.x(), q.y());

          visualization_msgs::Marker arrow;
          arrow.header.frame_id = map_frame_;
          arrow.header.stamp = stamp;
          arrow.ns = "rebound_spring";
          arrow.id = mid++;
          arrow.type = visualization_msgs::Marker::ARROW;
          arrow.action = visualization_msgs::Marker::ADD;
          geometry_msgs::Point p0, p1;
          p0.x = rc.base_point.x(); p0.y = rc.base_point.y(); p0.z = z_base + 0.05;
          p1.x = q.x();             p1.y = q.y();             p1.z = z_q + 0.05;
          arrow.points = {p0, p1};
          arrow.scale.x = 0.02;   // 杆径
          arrow.scale.y = 0.05;   // 头径
          arrow.scale.z = 0.08;   // 头长
          const double dist = (q - rc.base_point).dot(rc.direction);
          const double err = rebound_clearance_ - dist;
          worst_err = std::max(worst_err, err);
          arrow.color.r = 1.0f; arrow.color.g = (err <= 0.0f) ? 1.0f : 0.2f; arrow.color.b = 0.2f;
          arrow.color.a = 0.9f;
          arrow.lifetime = ros::Duration(0.5);
          arr.markers.push_back(arrow);
        }
        rebound_debug_pub_.publish(arr);
        ROS_INFO_THROTTLE(2.0,
            "[AStarLocalPlanner] rebound active: %d control points, worst clearance violation %.3fm (red arrows = still compressed)",
            active, worst_err);
      }
      else
      {
        rebound_debug_pub_.publish(arr);  // 无碰撞: 清空上一帧箭头
      }
    }

    perf.mark("alm");
    // 发布 3D 走廊调试详细 JSON 数据 (包含 1:1 对应的优化后物理点与违约量)
    if (local_corridor_debug_pub_.getNumSubscribers() > 0)
    {
      std_msgs::String debug_msg;
      debug_msg.data = SFCGenerator::toJsonString(corridors_2d, opt_control_points_2d);
      local_corridor_debug_pub_.publish(debug_msg);
    }
  }

  // 优化失败或无图数据
  if (!opt_success)
  {
    // [DBG] 失败时同样发布起终点高亮 (修改后终点取末航点), 便于定位拒绝场景
    if (local_se_debug_pub_.getNumSubscribers() > 0 && !waypoints_2d.empty() && !corridors_2d.empty())
    {
      geometry_msgs::Point mod_start;
      mod_start.x = robot_pose.x;
      mod_start.y = robot_pose.y;
      mod_start.z = robot_pose.z;
      geometry_msgs::Point mod_end;
      mod_end.x = waypoints_2d.back().x();
      mod_end.y = waypoints_2d.back().y();
      mod_end.z = corridors_2d.back().z_ref;
      publishSeDebugMarkers(local_band.front().pose.position, mod_start,
                            local_band.back().pose.position, mod_end, nullptr, corridors_2d,
                            nullptr, 0.0, 0.05);
    }
    cmd_vel = geometry_msgs::Twist();
    return false;
  }

  // 7. 将 2D 轨迹披覆 (Drape) 回 3D 流形表面并发布 (纯使用走廊节点数据，零二次图查表)
  nav_msgs::Path spline_path = bspline_traj_.toPathMsgFromCorridors(map_frame_, corridors_2d, 0.05);
  perf.mark("drape");
  ROS_INFO_THROTTLE(5.0, "[AStarLocalPlanner] spline published: %zu poses (waypoints=%zu corridors=%zu dt=%.3f)",
                    spline_path.poses.size(), waypoints_2d.size(), corridors_2d.size(), knot_dt);
  if (local_spline_plan_pub_.getNumSubscribers() > 0 || local_plan_pub_.getNumSubscribers() > 0)
  {
    local_spline_plan_pub_.publish(spline_path);
    local_plan_pub_.publish(spline_path);
  }
  if (local_se_debug_pub_.getNumSubscribers() > 0)
  {
    geometry_msgs::Point mod_start;
    mod_start.x = robot_pose.x;
    mod_start.y = robot_pose.y;
    mod_start.z = robot_pose.z;
    geometry_msgs::Point mod_end;
    if (!spline_path.poses.empty())
    {
      mod_end = spline_path.poses.back().pose.position;
    }
    else
    {
      mod_end.x = waypoints_2d.back().x();
      mod_end.y = waypoints_2d.back().y();
      mod_end.z = corridors_2d.back().z_ref;
    }
    publishSeDebugMarkers(local_band.front().pose.position, mod_start,
                          local_band.back().pose.position, mod_end, &spline_path, corridors_2d,
                          &opt_control_points_2d, knot_dt, 0.05);
  }

  // 8. 终点距离判断: 剩余 3D 路程长度在真 3D 满足时无缝进入终点吸附 (直接计算控制量，严禁自递归)
  if (is_near_final_goal)
  {
    pose_adjusting_ = true;
    ROS_INFO("[AStarLocalPlanner] Near final goal in step 8 (remaining 3D: %.2fm, direct xy: %.2fm, dz: %.2fm). Activating alignment.",
             remaining_dist_3d, dist_to_goal_xy, dist_to_goal_z);

    geometry_msgs::PoseStamped final_pose_base;
    if (transformToBase(global_plan_.back(), final_pose_base))
    {
      double dx_b = final_pose_base.pose.position.x;
      double dy_b = final_pose_base.pose.position.y;
      double dz_b = final_pose_base.pose.position.z;
      double cur_dist_xy = std::hypot(dx_b, dy_b);
      double final_yaw_error = 0.0;
      if (align_final_yaw_) computeFinalYawErrorXY(global_plan_.back(), final_yaw_error);

      if (cur_dist_xy < goal_pos_tol_ && std::abs(dz_b) < goal_z_tol_ && (!align_final_yaw_ || std::abs(final_yaw_error) < goal_yaw_tol_))
      {
        goal_reached_ = true;
        velocity_smoother_.reset();
        cmd_vel = geometry_msgs::Twist();
        return true;
      }

      const double max_align_v = 0.25;
      const double max_align_w = 0.60;
      geometry_msgs::Twist raw_cmd;
      raw_cmd.linear.x = std::max(-max_align_v, std::min(max_align_v, dx_b * linear_gain_));
      raw_cmd.linear.y = enable_lateral_motion_ ? std::max(-max_align_v, std::min(max_align_v, dy_b * lateral_gain_)) : 0.0;
      raw_cmd.linear.z = 0.0;
      if (align_final_yaw_)
      {
        raw_cmd.angular.z = std::max(-max_align_w, std::min(max_align_w, final_yaw_error * final_yaw_gain_));
      }
      else if (cur_dist_xy > 0.05)
      {
        double angle_to_target = std::atan2(dy_b, dx_b);
        raw_cmd.angular.z = std::max(-max_align_w, std::min(max_align_w, angle_to_target * heading_gain_));
      }
      cmd_vel = velocity_smoother_.smooth(raw_cmd, dt);
      last_cmd_vel_ = cmd_vel;
      return true;
    }
  }

  // 9. 纯 2D 沿曲线单调前瞻与微分几何速度前馈 (消灭发夹弯掉头区航向 180 度跳变)
  double total_t = bspline_traj_.getTotalTime();
  // 寻找机器人在样条上的最近投影时间参数 t_proj
  double best_t = 0.0;
  double min_proj_d_sq = std::numeric_limits<double>::max();
  for (double t_eval = 0.0; t_eval <= std::min(total_t, 1.2); t_eval += 0.05)
  {
    Eigen::Vector2d p_eval = bspline_traj_.evaluatePosition(t_eval);
    double d_sq = (p_eval.x() - robot_pose.x) * (p_eval.x() - robot_pose.x) +
                  (p_eval.y() - robot_pose.y) * (p_eval.y() - robot_pose.y);
    if (d_sq < min_proj_d_sq)
    {
      min_proj_d_sq = d_sq;
      best_t = t_eval;
    }
  }

  // 从当前投影点沿曲线单调向前延拓 0.35m
  double dt_lookahead = std::max(0.15, std::min(0.40, 0.35 / std::max(0.20, cruise_speed)));
  double t_lookahead = std::min(total_t, best_t + dt_lookahead);
  Eigen::Vector2d p_lookahead = bspline_traj_.evaluatePosition(t_lookahead);
  Eigen::Vector2d v_lookahead = bspline_traj_.evaluateVelocity(t_lookahead);

  double target_yaw = (v_lookahead.norm() > 0.05) ? std::atan2(v_lookahead.y(), v_lookahead.x())
                                                  : std::atan2(p_lookahead.y() - robot_pose.y, p_lookahead.x() - robot_pose.x);
  double heading_error = angles::shortest_angular_distance(robot_pose.yaw, target_yaw);

  // 9.0 航向门控 (SCAN-Planner 式防绕圈): 航向偏差过大 (>35°) 时原地旋转对准，抑制前进线速度
  const double max_w = velocity_smoother_.getParams().max_angular_speed;
  if (std::abs(heading_error) > 0.60)
  {
    geometry_msgs::Twist turn_cmd;
    turn_cmd.linear.x = 0.0;
    turn_cmd.linear.y = 0.0;
    turn_cmd.linear.z = 0.0;
    turn_cmd.angular.z = std::max(-max_w, std::min(max_w, heading_error * heading_gain_));
    cmd_vel = velocity_smoother_.smooth(turn_cmd, dt);
    last_cmd_vel_ = cmd_vel;
    return true;
  }

  // 9.1 连续平滑余弦调速与近终点平滑减速 (进站动量自然衰减)
  double kappa = bspline_traj_.evaluateCurvature(t_lookahead);
  double curve_speed_limit = cruise_speed;
  if (kappa > 0.15)
  {
    curve_speed_limit = std::min(cruise_speed, std::sqrt(max_lateral_acc_ / kappa));
  }
  double goal_scale = (remaining_dist_3d < 0.60) ? std::max(0.15, remaining_dist_3d / 0.60) : 1.0;
  double heading_scale = std::max(0.20, std::cos(heading_error));
  double forward_speed = std::min(cruise_speed, curve_speed_limit) * goal_scale * heading_scale;

  // 近终点保底速度平滑淡出 (剩余距离 < 0.50m 时保底由 0.12m/s 衰减到 0.0m/s，消除冲出动量)
  double min_floor = (remaining_dist_3d < 0.50) ? (0.12 * std::max(0.0, (remaining_dist_3d - 0.20) / 0.30)) : 0.12;
  forward_speed = std::max(min_floor, forward_speed);

  // 9.2 构造下发指令 (B 样条曲率前馈 + 比例航向跟踪 + 四足横向全向辅助)
  double omega_ff = bspline_traj_.evaluateAngularVelocity(t_lookahead);
  geometry_msgs::Twist raw_cmd;
  raw_cmd.linear.x = forward_speed;
  // 四足全向平移协同：角度误差较大时输出平滑横向速度帮助过弯
  raw_cmd.linear.y = (enable_lateral_motion_) ? std::max(-0.25, std::min(0.25, forward_speed * std::sin(heading_error) * lateral_gain_)) : 0.0;
  raw_cmd.linear.z = 0.0; // 纯平面速度控制，由底层四足控制器自主贴地
  raw_cmd.angular.z = omega_ff + heading_error * heading_gain_;

  // 10. 平滑滤波与动力学保护
  cmd_vel = velocity_smoother_.smooth(raw_cmd, dt);
  last_cmd_vel_ = cmd_vel;

  return true;
}

bool AStarLocalPlanner::isGoalReached()
{
  return goal_reached_;
}

std::vector<geometry_msgs::PoseStamped> AStarLocalPlanner::extractLocalBand(
  const RobotPose2D & robot_pose, double horizon_dist)
{
  std::vector<geometry_msgs::PoseStamped> band;
  if (global_plan_.empty()) return band;

  // 1. 严格单调向前推进 (Monotonic Step Forward)
  // 只在 [global_tracking_index_, global_tracking_index_ + 6] 极小前瞻窗口内搜索当前线段
  // 严禁大范围跳跃，彻底防止在发夹弯/楼梯转台跳到对向车道
  size_t search_start = global_tracking_index_;
  size_t search_end   = std::min(global_plan_.size(), size_t(global_tracking_index_ + 6));
  if (global_tracking_index_ == 0) search_end = std::min(global_plan_.size(), size_t(15));

  size_t closest_idx = global_tracking_index_;
  double min_d_sq = std::numeric_limits<double>::max();

  for (size_t i = search_start; i + 1 < search_end; ++i)
  {
    const auto& p1 = global_plan_[i].pose.position;
    const auto& p2 = global_plan_[i + 1].pose.position;
    double vx = p2.x - p1.x;
    double vy = p2.y - p1.y;
    double vz = p2.z - p1.z;
    double v_sq = vx * vx + vy * vy + vz * vz;
    if (v_sq < 1e-5) continue;

    double t = ((robot_pose.x - p1.x) * vx + (robot_pose.y - p1.y) * vy + (robot_pose.z - p1.z) * vz) / v_sq;
    t = std::max(0.0, std::min(1.0, t));

    double px = p1.x + t * vx;
    double py = p1.y + t * vy;
    double pz = p1.z + t * vz;
    double d_sq = (robot_pose.x - px) * (robot_pose.x - px) +
                  (robot_pose.y - py) * (robot_pose.y - py) +
                  4.0 * (robot_pose.z - pz) * (robot_pose.z - pz);

    if (d_sq < min_d_sq)
    {
      min_d_sq = d_sq;
      // 只要机器人在线段上的投影进度过半 (t >= 0.5)，前向追踪索引推进到 i+1
      closest_idx = (t >= 0.5) ? (i + 1) : i;
    }
  }

  // 严格单调递增，绝不倒退
  global_tracking_index_ = std::max(global_tracking_index_, static_cast<int>(closest_idx));

  // 2. 严格从 global_tracking_index_ 向前截取 horizon_dist，身后航点 100% 剪除
  double accum_dist = 0.0;
  for (size_t i = global_tracking_index_; i < global_plan_.size(); ++i)
  {
    if (!band.empty())
    {
      const auto & p_prev = band.back().pose.position;
      const auto & p_curr = global_plan_[i].pose.position;
      double seg_len = std::sqrt(std::pow(p_curr.x - p_prev.x, 2) +
                                 std::pow(p_curr.y - p_prev.y, 2) +
                                 std::pow(p_curr.z - p_prev.z, 2));
      accum_dist += seg_len;
      if (accum_dist >= horizon_dist) break;
    }
    band.push_back(global_plan_[i]);
  }

  if (band.empty() && !global_plan_.empty())
  {
    band.push_back(global_plan_.back());
  }

  return band;
}

bool AStarLocalPlanner::isPathBlocked(
  const std::vector<geometry_msgs::PoseStamped> & path,
  const elevation_planner::ManifoldGraph & graph,
  double check_dist,
  size_t & blocked_idx)
{
  if (path.size() < 2) return false;

  size_t checked = 0, unanchored = 0;
  double accum = 0.0;
  for (size_t i = 1; i < path.size(); ++i)
  {
    const auto & prev = path[i - 1].pose.position;
    const auto & curr = path[i].pose.position;
    accum += std::hypot(curr.x - prev.x, curr.y - prev.y);

    uint32_t nid = 0;
    // include_blocked=true: 动态障碍封锁的节点也要找得到, 否则阻挡检测失明
    if (graph.findClosestNode(curr.x, curr.y, curr.z, nid, 0.4, 0.6, true))
    {
      ++checked;
      const auto & node = graph.getNode(nid);
      // 节点不可通行: 障碍物阻挡或顶头净空不足
      if (node.traversability >= 0.8f || node.headroom < 0.45f)
      {
        blocked_idx = i;
        ROS_WARN_THROTTLE(1.0, "[AStarLocalPlanner] Path blocked: band pose %zu (arc %.2fm / window %.2fm) -> "
                 "node %u (%.2f,%.2f,%.2f) trav=%.2f headroom=%.2f -> Kinematic A* detour",
                 i, accum, check_dist, nid, node.x, node.y, node.z,
                 node.traversability, node.headroom);
        return true;
      }
    }
    else
    {
      // [DBG] 锚定失败 = 该 pose 完全没被检查 (其周边 0.4m 内无任何图节点)
      ++unanchored;
    }

    if (accum >= check_dist) break;
  }

  // [DBG] 未触发也要留痕: checked < 总数说明检查窗口截断 (obstacle_check_distance
  //       之后的路径是盲区, freeze 模式机器人不前进则永远盲); unanchored 说明
  //       有 pose 周边无图节点, 同样是探测盲区
  ROS_INFO_THROTTLE(2.0, "[AStarLocalPlanner] isPathBlocked: no block (%zu/%zu poses checked within %.2fm, %zu unanchored)",
                    checked, path.size() - 1, check_dist, unanchored);
  return false;
}

std::vector<geometry_msgs::PoseStamped> AStarLocalPlanner::checkAndReplanKinematicDetour(
  const std::vector<geometry_msgs::PoseStamped> & local_band,
  const RobotPose2D & robot_pose,
  const elevation_planner::ManifoldGraph & graph)
{
  size_t blocked_idx = 0;
  if (!isPathBlocked(local_band, graph, obstacle_check_distance_, blocked_idx))
  {
    return local_band;
  }

  ROS_WARN_THROTTLE(1.0, "[AStarLocalPlanner] Dynamic obstacle ahead at index %zu. Triggering Kinematic A* detour on graph...", blocked_idx);

  // 1. 查找起点图节点
  uint32_t start_nid = 0;
  if (!graph.findClosestNode(robot_pose.x, robot_pose.y, robot_pose.z, start_nid, 0.5, 0.8))
  {
    return local_band;
  }

  // 2. 查找受阻区域之后的目标重合节点
  size_t target_idx = std::min(local_band.size() - 1, blocked_idx + 10);
  uint32_t goal_nid = 0;
  bool found_goal = false;
  for (size_t idx = target_idx; idx < local_band.size(); ++idx)
  {
    const auto& pt = local_band[idx].pose.position;
    if (graph.findClosestNode(pt.x, pt.y, pt.z, goal_nid, 0.5, 0.8))
    {
      const auto& g_node = graph.getNode(goal_nid);
      if (g_node.traversability < 0.8f && g_node.headroom >= 0.45f)
      {
        target_idx = idx;
        found_goal = true;
        break;
      }
    }
  }

  if (!found_goal)
  {
    return local_band;
  }

  // 3. 执行图内运动学平滑 A* 搜索 (零脱图、大圆弧绕行)
  std::vector<Eigen::Vector3d> detour_pts;
  if (!kinematic_astar_.search(graph, start_nid, goal_nid, detour_pts) || detour_pts.empty())
  {
    ROS_WARN_THROTTLE(1.0, "[AStarLocalPlanner] Kinematic A* failed to find a valid detour on graph!");
    return local_band;
  }

  // 4. 组装绕障后的全新局部路径
  std::vector<geometry_msgs::PoseStamped> new_band;
  new_band.reserve(detour_pts.size() + (local_band.size() - target_idx));

  ros::Time now = ros::Time::now();
  for (const auto& pt : detour_pts)
  {
    geometry_msgs::PoseStamped ps;
    ps.header.stamp = now;
    ps.header.frame_id = map_frame_;
    ps.pose.position.x = pt.x();
    ps.pose.position.y = pt.y();
    ps.pose.position.z = pt.z();
    ps.pose.orientation.w = 1.0;
    new_band.push_back(ps);
  }

  for (size_t i = target_idx + 1; i < local_band.size(); ++i)
  {
    new_band.push_back(local_band[i]);
  }

  return new_band;
}

bool AStarLocalPlanner::lookupRobotPose2D(RobotPose2D & robot_pose)
{
  if (!costmap_ros_) return false;

  geometry_msgs::PoseStamped global_pose;
  if (!costmap_ros_->getRobotPose(global_pose)) return false;

  robot_pose.x = global_pose.pose.position.x;
  robot_pose.y = global_pose.pose.position.y;
  robot_pose.z = global_pose.pose.position.z;
  robot_pose.yaw = tf2::getYaw(global_pose.pose.orientation);
  return true;
}

bool AStarLocalPlanner::lookupRobotPose3D(double & x, double & y, double & z)
{
  RobotPose2D pose;
  if (!lookupRobotPose2D(pose)) return false;
  x = pose.x;
  y = pose.y;
  z = pose.z;
  return true;
}

bool AStarLocalPlanner::transformToBase(
  const geometry_msgs::PoseStamped & pose_in,
  geometry_msgs::PoseStamped & pose_out)
{
  if (!tf_buffer_) return false;
  try
  {
    geometry_msgs::TransformStamped tf = tf_buffer_->lookupTransform(
      costmap_ros_->getBaseFrameID(),
      pose_in.header.frame_id.empty() ? map_frame_ : pose_in.header.frame_id,
      ros::Time(0),
      ros::Duration(0.05));
    tf2::doTransform(pose_in, pose_out, tf);
    return true;
  }
  catch (const tf2::TransformException & ex)
  {
    ROS_WARN_THROTTLE(2.0, "[AStarLocalPlanner] TF lookup failed: %s", ex.what());
    return false;
  }
}

bool AStarLocalPlanner::computeFinalYawErrorXY(
  const geometry_msgs::PoseStamped & final_pose_in,
  double & yaw_error)
{
  RobotPose2D robot_pose;
  if (!lookupRobotPose2D(robot_pose)) return false;

  const double goal_yaw = tf2::getYaw(final_pose_in.pose.orientation);
  yaw_error = angles::shortest_angular_distance(robot_pose.yaw, goal_yaw);
  return true;
}

double AStarLocalPlanner::remainingPlanLength3D(const RobotPose2D & robot_pose) const
{
  const size_t n = global_plan_.size();
  if (n == 0) return std::numeric_limits<double>::max();
  const size_t seg_count = n - 1;
  if (seg_count == 0)
  {
    const auto& g = global_plan_.back().pose.position;
    return std::sqrt(std::pow(g.x - robot_pose.x, 2) + std::pow(g.y - robot_pose.y, 2) +
                     std::pow(g.z - robot_pose.z, 2));
  }

  // 在跟踪索引附近的小窗口内做 3D 投影取沿路径进度, 消除航点合并后索引点间距 (0.6m) 的量化误差;
  // z 加权 4.0 与 extractLocalBand 同口径, 防止异层重叠段抢走投影
  size_t start = static_cast<size_t>(std::max(0, global_tracking_index_ - 1));
  size_t end = std::min(seg_count, start + 4);

  size_t best_seg = seg_count;
  double best_t = 0.0, best_d_sq = std::numeric_limits<double>::max();
  for (size_t i = start; i < end; ++i)
  {
    const auto& p1 = global_plan_[i].pose.position;
    const auto& p2 = global_plan_[i + 1].pose.position;
    double vx = p2.x - p1.x, vy = p2.y - p1.y, vz = p2.z - p1.z;
    double v_sq = vx * vx + vy * vy + vz * vz;
    if (v_sq < 1e-9) continue;
    double t = ((robot_pose.x - p1.x) * vx + (robot_pose.y - p1.y) * vy +
                (robot_pose.z - p1.z) * vz) / v_sq;
    t = std::max(0.0, std::min(1.0, t));
    double dx = robot_pose.x - (p1.x + t * vx);
    double dy = robot_pose.y - (p1.y + t * vy);
    double dz = robot_pose.z - (p1.z + t * vz);
    double d_sq = dx * dx + dy * dy + 4.0 * dz * dz;
    if (d_sq < best_d_sq) { best_d_sq = d_sq; best_seg = i; best_t = t; }
  }

  if (best_seg >= seg_count)
  {
    const auto& g = global_plan_.back().pose.position;
    return std::sqrt(std::pow(g.x - robot_pose.x, 2) + std::pow(g.y - robot_pose.y, 2) +
                     std::pow(g.z - robot_pose.z, 2));
  }

  const auto& p1 = global_plan_[best_seg].pose.position;
  const auto& p2 = global_plan_[best_seg + 1].pose.position;
  double remaining = (1.0 - best_t) * std::sqrt(std::pow(p2.x - p1.x, 2) +
                                                std::pow(p2.y - p1.y, 2) +
                                                std::pow(p2.z - p1.z, 2));
  for (size_t i = best_seg + 1; i < seg_count; ++i)
  {
    const auto& a = global_plan_[i].pose.position;
    const auto& b = global_plan_[i + 1].pose.position;
    remaining += std::sqrt(std::pow(b.x - a.x, 2) + std::pow(b.y - a.y, 2) +
                           std::pow(b.z - a.z, 2));
  }
  return remaining;
}

// ===== [DBG] 起终点高亮 + 线级约束检查 =====
void AStarLocalPlanner::publishSeDebugMarkers(const geometry_msgs::Point& orig_start,
                                              const geometry_msgs::Point& mod_start,
                                              const geometry_msgs::Point& orig_end,
                                              const geometry_msgs::Point& mod_end,
                                              const nav_msgs::Path* spline_path,
                                              const std::vector<ConvexCorridor2D>& corridors,
                                              const std::vector<Eigen::Vector2d>* control_points,
                                              double knot_dt,
                                              double path_step_dt)
{
  visualization_msgs::MarkerArray arr;

  visualization_msgs::Marker clear;
  clear.action = visualization_msgs::Marker::DELETEALL;
  arr.markers.push_back(clear);

  struct SeItem
  {
    const geometry_msgs::Point* pt;
    const char* label;
    float r, g, b;
    int id;
  };
  const SeItem items[] = {
    { &orig_start, "orig_start", 0.0f, 1.0f, 1.0f, 0 },
    { &mod_start,  "mod_start",  0.0f, 1.0f, 0.0f, 2 },
    { &orig_end,   "orig_end",   1.0f, 0.5f, 0.0f, 4 },
    { &mod_end,    "mod_end",    1.0f, 0.1f, 0.1f, 6 },
  };
  for (const auto& it : items)
  {
    visualization_msgs::Marker s;
    s.header.frame_id = map_frame_;
    s.header.stamp = ros::Time::now();
    s.ns = "se_point";
    s.id = it.id;
    s.type = visualization_msgs::Marker::SPHERE;
    s.pose.position = *it.pt;
    s.pose.position.z += 0.05;
    s.pose.orientation.w = 1.0;
    s.scale.x = 0.06;
    s.scale.y = 0.06;
    s.scale.z = 0.06;
    s.color.r = it.r;
    s.color.g = it.g;
    s.color.b = it.b;
    s.color.a = 1.0f;
    arr.markers.push_back(s);

    visualization_msgs::Marker t;
    t.header = s.header;
    t.ns = "se_label";
    t.id = it.id + 1;
    t.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    t.pose.position = s.pose.position;
    t.pose.position.z += 0.10;
    t.pose.orientation.w = 1.0;
    t.scale.z = 0.09;
    t.color.r = it.r;
    t.color.g = it.g;
    t.color.b = it.b;
    t.color.a = 1.0f;
    t.text = it.label;
    arr.markers.push_back(t);
  }

  if (control_points == nullptr || control_points->size() < 4 || corridors.empty()) {
    local_se_debug_pub_.publish(arr);
    return;
  }

  // 本段自身行的最大违例 (拼接走廊切 [0, base_rows); -1 = 全部行)
  auto ownViolation = [](const ConvexCorridor2D& cor, double x, double y) {
    if (cor.constraint.A.rows() == 0) return 0.0;
    const int total = static_cast<int>(cor.constraint.A.rows());
    const int rows = (cor.constraint.base_rows >= 0 && cor.constraint.base_rows <= total)
                         ? cor.constraint.base_rows : total;
    if (rows <= 0) return 0.0;
    Eigen::VectorXd v = cor.constraint.A.topRows(rows) * Eigen::Vector2d(x, y) - cor.constraint.b.head(rows);
    return v.maxCoeff();
  };

  const int num_cp = static_cast<int>(control_points->size());  // = M + 2
  const int num_pieces = num_cp - 3;                            // = M - 1 段样条/折线

  // --- 控制折线逐边检查 (边 k 跨段 S_k, 对应走廊 corridors[k] 的自身行) ---
  // 凸性: 边两端点均在凸走廊内 => 整条边在走廊内
  {
    visualization_msgs::Marker poly;
    poly.header.frame_id = map_frame_;
    poly.header.stamp = ros::Time::now();
    poly.ns = "control_polyline";
    poly.id = 90;
    poly.type = visualization_msgs::Marker::LINE_LIST;  // 每边独立着色
    poly.pose.orientation.w = 1.0;
    poly.scale.x = 0.025;
    poly.color.a = 1.0f;
    for (int k = 1; k <= num_pieces; ++k)
    {
      const int ci = std::min(std::max(k, 1), static_cast<int>(corridors.size()) - 1);
      const double v0 = ownViolation(corridors[ci], (*control_points)[k].x(), (*control_points)[k].y());
      const double v1 = ownViolation(corridors[ci], (*control_points)[k + 1].x(), (*control_points)[k + 1].y());
      const bool ok = (v0 <= 0.0 && v1 <= 0.0);
      std_msgs::ColorRGBA c;
      c.a = 1.0f;
      if (ok) { c.g = 1.0f; }
      else { c.r = 1.0f; c.g = 0.4f; }
      for (int e = 0; e < 2; ++e)
      {
        const auto& q = (*control_points)[k + e];
        geometry_msgs::Point pt;
        pt.x = q.x();
        pt.y = q.y();
        pt.z = corridors[ci].z_ref + 0.06;
        poly.points.push_back(pt);
        poly.colors.push_back(c);
      }
    }
    arr.markers.push_back(poly);
  }

  // --- 样条分段检查 (曲线段 i 位于 4 个跨控制点的凸包内, 4 点全在走廊内 => 曲线保证在走廊内) ---
  if (spline_path != nullptr && !spline_path->poses.empty() && knot_dt > 1e-6)
  {
    visualization_msgs::Marker line;
    line.header.frame_id = map_frame_;
    line.header.stamp = ros::Time::now();
    line.ns = "spline_pieces";
    line.id = 100;
    line.type = visualization_msgs::Marker::LINE_STRIP;
    line.pose.orientation.w = 1.0;
    line.scale.x = 0.035;
    line.color.a = 1.0f;
    for (size_t s = 0; s < spline_path->poses.size(); ++s)
    {
      const auto& ps = spline_path->poses[s].pose.position;
      line.points.push_back(ps);

      const double t = static_cast<double>(s) * path_step_dt;
      int piece = static_cast<int>(std::floor(t / knot_dt)) + 1;  // 段 i 覆盖 t∈[(i-1)dt, i·dt]
      piece = std::min(std::max(piece, 1), num_pieces);
      const int ci = std::min(std::max(piece, 1), static_cast<int>(corridors.size()) - 1);

      // 4 个跨控制点 (q_{i-1}..q_{i+2}) 全部在走廊自身行内 => 该段曲线保证在走廊内
      bool guaranteed = true;
      for (int q = piece - 1; q <= piece + 2; ++q)
      {
        if (q < 0 || q >= num_cp) continue;
        if (ownViolation(corridors[ci], (*control_points)[q].x(), (*control_points)[q].y()) > 0.0)
        {
          guaranteed = false;
          break;
        }
      }

      std_msgs::ColorRGBA c;
      c.a = 1.0f;
      if (guaranteed)
      {
        c.g = 1.0f;  // 有保证: 绿
      }
      else
      {
        const double viol = ownViolation(corridors[ci], ps.x, ps.y);
        if (viol <= 0.0) { c.r = 1.0f; c.g = 1.0f; }  // 无保证但实际在内: 黄
        else { c.r = 1.0f; }                           // 实际出界: 红
      }
      line.colors.push_back(c);
    }
    arr.markers.push_back(line);
  }

  local_se_debug_pub_.publish(arr);
}

} // namespace elevation_local_planner
