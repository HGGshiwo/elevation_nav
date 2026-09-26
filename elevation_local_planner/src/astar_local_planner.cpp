#include "elevation_local_planner/astar_local_planner.h"
#include <pluginlib/class_list_macros.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <angles/angles.h>
#include <algorithm>
#include <cmath>
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
  private_nh.param<double>("goal_yaw_tolerance",      goal_yaw_tol_, 0.10);
  private_nh.param<double>("linear_gain",             linear_gain_,  1.20);
  private_nh.param<double>("lateral_gain",            lateral_gain_, 0.40);
  private_nh.param<double>("heading_gain",            heading_gain_, 1.20);
  private_nh.param<double>("final_yaw_gain",          final_yaw_gain_, 0.60);
  private_nh.param<bool>  ("enable_lateral_motion",    enable_lateral_motion_, true);
  private_nh.param<bool>  ("align_final_yaw",         align_final_yaw_, true);

  // 配置运动学 A* 搜索器参数
  KinematicAStarConfig k_cfg;
  k_cfg.weight_z = 2.0;
  k_cfg.weight_turn = weight_turn_;
  k_cfg.weight_traversability = 3.0;
  k_cfg.max_step_height = max_step_height_;
  k_cfg.max_stride_length = max_stride_length_;
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

  initialized_ = true;
  ROS_INFO("[AStarLocalPlanner] Initialized with Kinematic A* + Uniform Cubic B-Spline Velocity Engine (horizon: %.2fm, turn_w: %.2f)",
           local_horizon_distance_, weight_turn_);
}

bool AStarLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
{
  if (!initialized_) return false;

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

  // 1. 获取全局共享的最新流形图 (同进程零拷贝)
  auto graph = elevation_planner::GraphStore::instance().getFusedGraph();
  if (!graph || graph->numNodes() == 0)
  {
    graph = elevation_planner::GraphStore::instance().getGlobalGraph();
  }

  ros::Time now = ros::Time::now();
  double dt = 0.05;
  if (!last_control_time_.isZero())
  {
    dt = (now - last_control_time_).toSec();
    if (dt <= 1.0e-4 || dt > 0.5) dt = 0.05;
  }
  last_control_time_ = now;

  // 2. 终点高精度对齐模式
  if (pose_adjusting_)
  {
    geometry_msgs::PoseStamped final_pose_base;
    if (!transformToBase(global_plan_.back(), final_pose_base))
    {
      cmd_vel = geometry_msgs::Twist();
      return false;
    }

    geometry_msgs::Twist raw_cmd;
    raw_cmd.linear.x = final_pose_base.pose.position.x * linear_gain_;
    raw_cmd.linear.y = enable_lateral_motion_ ? final_pose_base.pose.position.y * lateral_gain_ : 0.0;

    double final_yaw_error = 0.0;
    if (align_final_yaw_)
    {
      if (!computeFinalYawErrorXY(global_plan_.back(), final_yaw_error))
      {
        cmd_vel = geometry_msgs::Twist();
        return false;
      }
      raw_cmd.angular.z = final_yaw_error * final_yaw_gain_;
    }

    const bool pos_ok = std::hypot(final_pose_base.pose.position.x, final_pose_base.pose.position.y) < goal_pos_tol_;
    const bool yaw_ok = !align_final_yaw_ || std::abs(final_yaw_error) < goal_yaw_tol_;
    if (pos_ok && yaw_ok)
    {
      goal_reached_ = true;
      velocity_smoother_.reset();
      cmd_vel = geometry_msgs::Twist();
      ROS_INFO("[AStarLocalPlanner] Goal successfully reached with 3D precision.");
      return true;
    }

    cmd_vel = velocity_smoother_.smooth(raw_cmd, dt);
    last_cmd_vel_ = cmd_vel;
    return true;
  }

  // 3. 截取前方局部切片 (2.5m 视距，严格单调向前推进，彻底防止发夹弯跳点)
  std::vector<geometry_msgs::PoseStamped> local_band = extractLocalBand(robot_pose, local_horizon_distance_);
  if (local_band.empty())
  {
    cmd_vel = geometry_msgs::Twist();
    return false;
  }

  // 4. 【提取局部拓扑 node_id 序列 (方案 B 全链路透传，零查图开销)】
  std::vector<uint32_t> path_node_ids;
  for (const auto& ps : local_band)
  {
    uint32_t nid = static_cast<uint32_t>(std::round(ps.pose.orientation.x));
    if (graph && nid < graph->numNodes())
    {
      path_node_ids.push_back(nid);
    }
  }

  // 4.1 避障检测：若前方局部路径受阻，调用 Kinematic A* 局部绕障重搜
  if (graph && graph->numNodes() > 0)
  {
    size_t blocked_idx = 0;
    if (isPathBlocked(local_band, *graph, obstacle_check_distance_, blocked_idx))
    {
      ROS_WARN_THROTTLE(1.0, "[AStarLocalPlanner] Path blocked ahead. Triggering Kinematic A* detour on graph...");
      uint32_t start_nid = path_node_ids.empty() ? 0 : path_node_ids.front();
      uint32_t goal_nid = path_node_ids.empty() ? 0 : path_node_ids.back();
      std::vector<uint32_t> detour_nids;
      std::vector<Eigen::Vector3d> detour_pts;
      if (kinematic_astar_.search(*graph, start_nid, goal_nid, detour_nids, detour_pts) && !detour_nids.empty())
      {
        path_node_ids = detour_nids;
      }
    }
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
      ROS_INFO_THROTTLE(5.0, "[AStarLocalPlanner] SC-LOS band pruned: %zu -> %zu waypoints",
                        path_node_ids.size(), pruned_ids.size());
      path_node_ids = std::move(pruned_ids);
    }
  }

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

    const Eigen::Vector2d anchor_2d = robot_start_2d + knot_dt * v_start_2d;

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
      std::vector<Eigen::Vector2d> state_points;
      if (sfc_seed_robot_state_)
      {
        state_points = {robot_start_2d, anchor_2d};
      }
      corridors_2d = SFCGenerator::generateSegmentCorridors(path_node_ids, *graph, band_simplifier_,
                                                            0.50, max_step_height_, max_stride_length_,
                                                            state_points, robot_pose.z);
      // 转角交集拼接: 转角控制点 q_{i+1} ∈ C_i ∩ C_{i+1}, 凸性锁死控制折线不穿墙
      SFCGenerator::stackAdjacentCorridors(corridors_2d);
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
    if (scan_optimizer_.optimize(waypoints_2d, corridors_2d, robot_start_2d, v_start_2d, knot_dt, opt_control_points_2d))
    {
      if (bspline_traj_.initialize2D(opt_control_points_2d, knot_dt))
      {
        opt_success = true;
      }
    }

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

  // 8. 终点距离判断
  const auto& goal_pos = global_plan_.back().pose.position;
  double dist_to_goal_2d = std::hypot(robot_pose.x - goal_pos.x, robot_pose.y - goal_pos.y);
  if (dist_to_goal_2d < goal_pos_tol_ + 0.10)
  {
    pose_adjusting_ = true;
    ROS_INFO("[AStarLocalPlanner] Near final goal (dist: %.2fm). Entering final alignment.", dist_to_goal_2d);
    return computeVelocityCommands(cmd_vel);
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

  // 9.1 连续平滑余弦调速与保底蠕动 (保证转弯前进不卡死，彻底消灭震荡看门狗停机)
  double kappa = bspline_traj_.evaluateCurvature(t_lookahead);
  double curve_speed_limit = cruise_speed;
  if (kappa > 0.15)
  {
    curve_speed_limit = std::min(cruise_speed, std::sqrt(max_lateral_acc_ / kappa));
  }
  double goal_scale = (dist_to_goal_2d < 0.60) ? std::max(0.30, dist_to_goal_2d / 0.60) : 1.0;
  // 保底保留 0.20 系数保证转弯时持续前行出弯
  double heading_scale = std::max(0.20, std::cos(heading_error));
  double forward_speed = std::min(cruise_speed, curve_speed_limit) * goal_scale * heading_scale;
  forward_speed = std::max(0.12, forward_speed); // 保底 0.12m/s，永不挂零锁死

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

  double accum = 0.0;
  for (size_t i = 1; i < path.size(); ++i)
  {
    const auto & prev = path[i - 1].pose.position;
    const auto & curr = path[i].pose.position;
    accum += std::hypot(curr.x - prev.x, curr.y - prev.y);

    uint32_t nid = 0;
    if (graph.findClosestNode(curr.x, curr.y, curr.z, nid, 0.4, 0.6))
    {
      const auto & node = graph.getNode(nid);
      // 节点不可通行: 障碍物阻挡或顶头净空不足
      if (node.traversability >= 0.8f || node.headroom < 0.45f)
      {
        blocked_idx = i;
        return true;
      }
    }

    if (accum >= check_dist) break;
  }

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
