#include "elevation_local_planner/astar_local_planner.h"
#include <pluginlib/class_list_macros.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
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

  // ---- 全部前瞻与避障关键参数 (支持 launch / yaml 配置，杜绝硬编码) ----
  private_nh.param<std::string>("map_frame", map_frame_, "map");
  private_nh.param<double>("local_horizon_distance",  local_horizon_distance_,  2.50);
  private_nh.param<double>("lookahead_distance",      lookahead_distance_,      0.45);
  private_nh.param<double>("obstacle_check_distance", obstacle_check_distance_, 1.50);
  private_nh.param<double>("detour_clearance_margin", detour_clearance_margin_, 0.20);
  private_nh.param<double>("max_step_height",         max_step_height_,         0.25);

  // ---- 控制与容差参数 ----
  private_nh.param<double>("tracking_point_reached_xy_tolerance", tracking_xy_tol_, 0.20);
  private_nh.param<double>("goal_position_tolerance", goal_pos_tol_, 0.05);
  private_nh.param<double>("goal_yaw_tolerance",      goal_yaw_tol_, 0.10);
  private_nh.param<double>("linear_gain",             linear_gain_,  1.2);
  private_nh.param<double>("lateral_gain",            lateral_gain_, 0.4);
  private_nh.param<double>("heading_gain",            heading_gain_, 1.2);
  private_nh.param<double>("final_yaw_gain",          final_yaw_gain_, 0.5);
  private_nh.param<bool>  ("enable_lateral_motion",    enable_lateral_motion_, true);
  private_nh.param<bool>  ("align_final_yaw",         align_final_yaw_, true);

  VelocitySmootherParams smoother_params;
  private_nh.param<double>("max_linear_speed",  smoother_params.max_linear_speed,  0.60);
  private_nh.param<double>("max_lateral_speed", smoother_params.max_lateral_speed, 0.30);
  private_nh.param<double>("max_angular_speed", smoother_params.max_angular_speed, 1.00);
  private_nh.param<double>("max_linear_acc",    smoother_params.max_linear_acc,    0.80);
  private_nh.param<double>("max_lateral_acc",   smoother_params.max_lateral_acc,   0.40);
  private_nh.param<double>("max_angular_acc",   smoother_params.max_angular_acc,   1.20);
  private_nh.param<bool>  ("enable_lateral_decoupling", smoother_params.enable_lateral_decoupling, true);
  private_nh.param<double>("linear_deadband",   smoother_params.linear_deadband,   0.05);
  private_nh.param<double>("lateral_deadband",  smoother_params.lateral_deadband,  0.05);
  private_nh.param<double>("angular_deadband",  smoother_params.angular_deadband,  0.05);
  velocity_smoother_.setParams(smoother_params);

  double collision_radius = 0.08;
  private_nh.param<double>("collision_footprint_radius", collision_radius, 0.08);
  double dog_height = 0.45;
  private_nh.param<double>("dog_height", dog_height, 0.45);
  collision_checker_ = CollisionChecker(collision_radius, dog_height);

  // 话题发布 (本地路径与 A* 避障局部路径可视化)
  local_plan_pub_       = nh.advertise<nav_msgs::Path>("/move_base/local_plan", 1);
  local_astar_plan_pub_ = nh.advertise<nav_msgs::Path>("/move_base/local_astar_plan", 1);

  initialized_ = true;
  ROS_INFO("[AStarLocalPlanner] Initialized with 3D Manifold A* Detour & Pure Pursuit (horizon: %.2fm, lookahead: %.2fm, check: %.2fm, max_step: %.2fm)",
           local_horizon_distance_, lookahead_distance_, obstacle_check_distance_, max_step_height_);
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
  global_tracking_index_ = 0;
  pose_adjusting_ = false;
  goal_reached_ = false;
  ROS_INFO("[AStarLocalPlanner] Received new global plan with %zu poses", global_plan_.size());
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

  // 1. 获取全局共享的最新融合图 (同进程零拷贝)
  auto graph = elevation_planner::GraphStore::instance().getFusedGraph();
  if (!graph || graph->numNodes() == 0)
  {
    graph = elevation_planner::GraphStore::instance().getGlobalGraph();
  }

  // 2. 紧急硬碰撞安全兜底
  if (graph && graph->numNodes() > 0)
  {
    uint32_t nid = 0;
    if (graph->findClosestNode(robot_pose.x, robot_pose.y, robot_pose.z, nid, 0.5, 0.8))
    {
      int layer = graph->getNode(nid).layer_id;
      if (collision_checker_.checkCollision(robot_pose.x, robot_pose.y, robot_pose.z, layer, *graph))
      {
        ROS_WARN_THROTTLE(1.0, "[AStarLocalPlanner] Collision at (%.2f, %.2f, %.2f), emergency stopping!",
                          robot_pose.x, robot_pose.y, robot_pose.z);
        cmd_vel = geometry_msgs::Twist();
        last_cmd_vel_ = cmd_vel;
        return false;
      }
    }
  }

  ros::Time now = ros::Time::now();
  double dt = 0.05;
  if (!last_control_time_.isZero())
  {
    dt = (now - last_control_time_).toSec();
    if (dt <= 1.0e-4 || dt > 0.5) dt = 0.05;
  }
  last_control_time_ = now;

  // 3. 终点姿态微调模式
  geometry_msgs::Twist raw_cmd;
  if (pose_adjusting_)
  {
    geometry_msgs::PoseStamped final_pose_base;
    if (!transformToBase(global_plan_.back(), final_pose_base))
    {
      cmd_vel = geometry_msgs::Twist();
      return false;
    }

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
      ROS_INFO("[AStarLocalPlanner] Goal reached.");
      return true;
    }

    cmd_vel = velocity_smoother_.smooth(raw_cmd, dt);
    last_cmd_vel_ = cmd_vel;
    return true;
  }

  // 4. 截取前方宏观切片 (local_horizon_distance)
  std::vector<geometry_msgs::PoseStamped> local_band = extractLocalBand(robot_pose, local_horizon_distance_);

  // 5. 【核心避障】检查前方路径受阻情况，必要时手动调用 3D Manifold A* 搜索绕障路径
  if (graph && graph->numNodes() > 0)
  {
    local_band = checkAndReplanAStarDetour(local_band, robot_pose, *graph);
  }

  active_local_plan_ = local_band;

  // 发布局部可视化路径
  if (local_plan_pub_.getNumSubscribers() > 0 && !active_local_plan_.empty())
  {
    nav_msgs::Path path_msg;
    path_msg.header.stamp = now;
    path_msg.header.frame_id = map_frame_;
    path_msg.poses = active_local_plan_;
    local_plan_pub_.publish(path_msg);
  }

  // 6. 前瞻追踪目标计算
  TrackingTarget target;
  if (!selectTrackingTarget(active_local_plan_, target))
  {
    cmd_vel = geometry_msgs::Twist();
    return false;
  }

  if (isFinalTrackingPointReached(target))
  {
    pose_adjusting_ = true;
    ROS_INFO("[AStarLocalPlanner] Near final goal. Entering final yaw adjustment.");
    return computeVelocityCommands(cmd_vel);
  }

  // 7. 解耦前瞻跟踪控制律
  const double heading_error = std::atan2(target.base_y, target.base_x);
  const double abs_heading   = std::abs(heading_error);

  // 沿路径剩余距离估算 (杜绝多层垂直重叠时误将楼上目标当作近在咫尺)
  double remain_path_dist = 0.0;
  for (size_t i = global_tracking_index_ + 1; i < global_plan_.size(); ++i)
  {
    const auto & p0 = global_plan_[i - 1].pose.position;
    const auto & p1 = global_plan_[i].pose.position;
    remain_path_dist += std::hypot(p1.x - p0.x, p1.y - p0.y);
    if (remain_path_dist > 2.0) break; // 超过 2m 即可退出，全速巡航
  }

  const double cruise_speed = velocity_smoother_.getParams().max_linear_speed;

  double corner_scale = 1.0;
  if (abs_heading > 0.785)
  {
    corner_scale = std::max(0.0, std::cos(heading_error));
  }
  const double goal_scale = remain_path_dist < 0.6 ? std::max(0.2, remain_path_dist / 0.6) : 1.0;

  raw_cmd.linear.x  = cruise_speed * corner_scale * goal_scale;
  raw_cmd.linear.y  = (enable_lateral_motion_ && abs_heading < 0.785) ? target.base_y * lateral_gain_ : 0.0;
  raw_cmd.angular.z = heading_error * heading_gain_;

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

  // 沿当前全局跟踪进度向前搜索机器人投影点（加权垂直高差 dz，防止在多层结构中跳层）
  int search_start = std::max(0, global_tracking_index_ - 3);
  int search_end   = std::min(static_cast<int>(global_plan_.size()), global_tracking_index_ + 30);
  if (global_tracking_index_ == 0) search_end = std::min(static_cast<int>(global_plan_.size()), 40);

  size_t start_idx = global_tracking_index_;
  double min_dist_sq = std::numeric_limits<double>::max();
  for (int i = search_start; i < search_end; ++i)
  {
    double dx = global_plan_[i].pose.position.x - robot_pose.x;
    double dy = global_plan_[i].pose.position.y - robot_pose.y;
    double dz = global_plan_[i].pose.position.z - robot_pose.z;
    double d2 = dx * dx + dy * dy + 4.0 * dz * dz;
    if (d2 < min_dist_sq)
    {
      min_dist_sq = d2;
      start_idx = i;
    }
  }
  global_tracking_index_ = start_idx;

  // 累积沿路径 3D 距离截取 horizon_dist
  double accum_dist = 0.0;
  band.push_back(global_plan_[start_idx]);
  for (size_t i = start_idx + 1; i < global_plan_.size(); ++i)
  {
    const auto & p_prev = global_plan_[i - 1].pose.position;
    const auto & p_curr = global_plan_[i].pose.position;
    double seg_len = std::sqrt(std::pow(p_curr.x - p_prev.x, 2) +
                               std::pow(p_curr.y - p_prev.y, 2) +
                               std::pow(p_curr.z - p_prev.z, 2));
    accum_dist += seg_len;
    band.push_back(global_plan_[i]);
    if (accum_dist >= horizon_dist) break;
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
      // 节点不可通行: traversability >= 0.8 或 顶头净空不足
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

std::vector<geometry_msgs::PoseStamped> AStarLocalPlanner::checkAndReplanAStarDetour(
  const std::vector<geometry_msgs::PoseStamped> & local_band,
  const RobotPose2D & robot_pose,
  const elevation_planner::ManifoldGraph & graph)
{
  size_t blocked_idx = 0;
  if (!isPathBlocked(local_band, graph, obstacle_check_distance_, blocked_idx))
  {
    // 前方通畅，无需绕障
    return local_band;
  }

  ROS_WARN_THROTTLE(1.0, "[AStarLocalPlanner] Dynamic obstacle detected ahead at index %zu! Triggering local Manifold A*...", blocked_idx);

  // 初始化局部 A* 规划器
  astar_planner_.initialize(graph);

  // 构造起点与安全恢复终点
  geometry_msgs::PoseStamped start_pose;
  start_pose.header.frame_id = map_frame_;
  start_pose.pose.position.x = robot_pose.x;
  start_pose.pose.position.y = robot_pose.y;
  start_pose.pose.position.z = robot_pose.z;
  start_pose.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), robot_pose.yaw));

  // 终点选择受阻点后方未受阻的安全路点
  size_t goal_idx = std::min(local_band.size() - 1, blocked_idx + 6);
  geometry_msgs::PoseStamped goal_pose = local_band[goal_idx];

  nav_msgs::Path raw_detour_path;
  raw_detour_path.header.frame_id = map_frame_;

  if (astar_planner_.plan(start_pose, goal_pose, raw_detour_path) && raw_detour_path.poses.size() >= 2)
  {
    nav_msgs::Path smoothed_detour;
    smoothed_detour.header.frame_id = map_frame_;
    path_smoother_.smooth(raw_detour_path, graph, smoothed_detour);

    // 拼接绕障路径与远端剩余路径
    std::vector<geometry_msgs::PoseStamped> detour_band = smoothed_detour.poses;
    for (size_t i = goal_idx + 1; i < local_band.size(); ++i)
    {
      detour_band.push_back(local_band[i]);
    }

    // 发布给可视化话题
    if (local_astar_plan_pub_.getNumSubscribers() > 0)
    {
      nav_msgs::Path vis_msg;
      vis_msg.header.stamp = ros::Time::now();
      vis_msg.header.frame_id = map_frame_;
      vis_msg.poses = detour_band;
      local_astar_plan_pub_.publish(vis_msg);
    }

    ROS_INFO_THROTTLE(1.0, "[AStarLocalPlanner] Local 3D A* Detour succeeded! Detour poses: %zu", detour_band.size());
    return detour_band;
  }
  else
  {
    ROS_WARN_THROTTLE(1.0, "[AStarLocalPlanner] Local A* detour failed to find a valid bypass! Holding trajectory.");
    return local_band;
  }
}

bool AStarLocalPlanner::selectTrackingTarget(const std::vector<geometry_msgs::PoseStamped> & plan, TrackingTarget & target)
{
  if (plan.empty()) return false;
  RobotPose2D robot_pose;
  if (!lookupRobotPose2D(robot_pose)) return false;

  int local_idx = 0;
  return LocalControlUtils::interpolateLookaheadTarget(
    plan, robot_pose, local_idx, lookahead_distance_, target);
}

bool AStarLocalPlanner::isFinalTrackingPointReached(const TrackingTarget & target) const
{
  (void)target;
  if (global_plan_.empty()) return true;

  // 1. 路径跟踪进度守卫：未跟踪到路径最后 4 个点以内，绝不进入终点判定（杜绝多层空间垂直重叠误判）
  if (global_tracking_index_ < static_cast<int>(global_plan_.size()) - 4)
  {
    return false;
  }

  RobotPose2D robot_pose;
  if (!const_cast<AStarLocalPlanner*>(this)->lookupRobotPose2D(robot_pose)) return false;

  const auto & goal_pos = global_plan_.back().pose.position;
  const double xy_dist = std::hypot(goal_pos.x - robot_pose.x, goal_pos.y - robot_pose.y);
  const double z_dist  = std::abs(goal_pos.z - robot_pose.z);

  // 2. 必须同时满足水平容差与垂直高度差容差（防止楼上楼下跨层误判）
  return (xy_dist < tracking_xy_tol_) && (z_dist < max_step_height_);
}

bool AStarLocalPlanner::lookupRobotPose2D(RobotPose2D & robot_pose)
{
  if (!tf_buffer_) return false;
  try {
    auto tf = tf_buffer_->lookupTransform(map_frame_, "base_link", ros::Time(0), ros::Duration(0.05));
    robot_pose.x   = tf.transform.translation.x;
    robot_pose.y   = tf.transform.translation.y;
    robot_pose.z   = tf.transform.translation.z;
    robot_pose.yaw = tf2::getYaw(tf.transform.rotation);
    return true;
  } catch (const tf2::TransformException &) {
    return false;
  }
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

bool AStarLocalPlanner::computeFinalYawErrorXY(const geometry_msgs::PoseStamped & final_pose_in, double & yaw_error)
{
  RobotPose2D robot_pose;
  if (!lookupRobotPose2D(robot_pose)) return false;

  geometry_msgs::PoseStamped final_pose = final_pose_in;
  if (final_pose.header.frame_id.empty()) final_pose.header.frame_id = map_frame_;
  final_pose.header.stamp = ros::Time(0);

  try {
    if (final_pose.header.frame_id != map_frame_ && tf_buffer_)
      tf_buffer_->transform(final_pose, final_pose, map_frame_, ros::Duration(0.05));
  } catch (const tf2::TransformException &) {
    return false;
  }

  yaw_error = LocalControlUtils::normalizeAngle(tf2::getYaw(final_pose.pose.orientation) - robot_pose.yaw);
  return true;
}

bool AStarLocalPlanner::transformToBase(const geometry_msgs::PoseStamped & pose_in, geometry_msgs::PoseStamped & pose_out)
{
  if (!tf_buffer_) return false;
  geometry_msgs::PoseStamped stamped = pose_in;
  if (stamped.header.frame_id.empty()) stamped.header.frame_id = map_frame_;
  stamped.header.stamp = ros::Time(0);

  try {
    tf_buffer_->transform(stamped, pose_out, "base_link", ros::Duration(0.05));
    return true;
  } catch (const tf2::TransformException &) {
    return false;
  }
}

} // namespace elevation_local_planner
