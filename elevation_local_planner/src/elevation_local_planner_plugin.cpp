#include "elevation_local_planner/elevation_local_planner_plugin.h"
#include <pluginlib/class_list_macros.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <algorithm>
#include <cmath>
#include <limits>

PLUGINLIB_EXPORT_CLASS(elevation_local_planner::ElevationLocalPlannerPlugin, nav_core::BaseLocalPlanner)

namespace elevation_local_planner
{

ElevationLocalPlannerPlugin::ElevationLocalPlannerPlugin()
: tf_buffer_(nullptr),
  costmap_ros_(nullptr),
  initialized_(false),
  target_index_(0),
  pose_adjusting_(false),
  goal_reached_(true)
{
}

ElevationLocalPlannerPlugin::~ElevationLocalPlannerPlugin()
{
  if (fusion_running_) {
    fusion_running_ = false;
    if (fusion_thread_.joinable()) fusion_thread_.join();
  }
}

void ElevationLocalPlannerPlugin::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
{
  if (initialized_)
  {
    ROS_WARN("ElevationLocalPlannerPlugin has already been initialized, doing nothing.");
    return;
  }

  tf_buffer_ = tf;
  costmap_ros_ = costmap_ros;

  ros::NodeHandle private_nh("~/" + name);

  // ---- 跟踪控制参数 (D1) ----
  private_nh.param<std::string>("map_frame",       map_frame_,       "map");
  private_nh.param<double>("lookahead_distance",   lookahead_distance_, 0.45);
  private_nh.param<double>("tracking_point_reached_xy_tolerance", tracking_xy_tol_, 0.20);
  private_nh.param<double>("goal_position_tolerance", goal_pos_tol_, 0.05);
  private_nh.param<double>("goal_yaw_tolerance",   goal_yaw_tol_, 0.10);
  private_nh.param<double>("linear_gain",          linear_gain_,  1.2);
  private_nh.param<double>("lateral_gain",         lateral_gain_, 0.4);
  private_nh.param<double>("heading_gain",         heading_gain_, 1.2);
  private_nh.param<double>("final_yaw_gain",       final_yaw_gain_, 0.5);
  private_nh.param<bool>  ("enable_lateral_motion", enable_lateral_motion_, true);
  private_nh.param<bool>  ("align_final_yaw",      align_final_yaw_, true);

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

  // ---- 建图/融合参数 (与全局插件同源, launch 统一注入) ----
  private_nh.param<double>("resolution",         build_cfg_.resolution, 0.10);
  private_nh.param<double>("max_step_height",    build_cfg_.max_step_height, 0.25);
  private_nh.param<double>("max_stride_length",  build_cfg_.max_stride_length, 0.35);
  private_nh.param<double>("dog_height",         build_cfg_.dog_height, 0.45);
  private_nh.param<double>("footprint_radius",   build_cfg_.footprint_radius, 0.30);
  private_nh.param<double>("body_hard_radius",   build_cfg_.body_hard_radius, 0.20);
  private_nh.param<double>("sweep_penalty_weight", build_cfg_.sweep_penalty_weight, 1.0);
  private_nh.param<double>("foot_clearance",     build_cfg_.foot_clearance, 0.05);
  private_nh.param<int>   ("sor_mean_k",         build_cfg_.sor_mean_k, 16);
  private_nh.param<double>("sor_std_mul",        build_cfg_.sor_std_mul, 1.5);
  private_nh.param<double>("cluster_height_diff", build_cfg_.cluster_height_diff, 0.08);
  private_nh.param<int>   ("min_cluster_points", build_cfg_.min_cluster_points, 2);
  builder_.setConfig(build_cfg_);

  private_nh.param<double>("crop_radius_xy",     crop_radius_xy_, 1.5);
  private_nh.param<double>("crop_height_above",  crop_height_above_, 2.0);
  private_nh.param<double>("crop_height_below",  crop_height_below_, 1.0);
  private_nh.param<double>("fusion_rate",        fusion_rate_, 5.0);
  private_nh.param<double>("collision_footprint_radius", collision_footprint_radius_, 0.30);
  collision_checker_ = CollisionChecker(collision_footprint_radius_, 0.50);

  // ---- 实时点云订阅与融合线程 ----
  std::string cloud_topic;
  private_nh.param<std::string>("cloud_topic", cloud_topic, "/lidar_points");
  ros::NodeHandle nh;
  cloud_sub_ = nh.subscribe(cloud_topic, 1, &ElevationLocalPlannerPlugin::cloudCallback, this);

  fusion_running_ = true;
  fusion_thread_ = std::thread(&ElevationLocalPlannerPlugin::fusionLoop, this);

  initialized_ = true;
  ROS_INFO("[ElevationLocalPlannerPlugin] D1 tracking + fused-graph safety layer ready (cloud: %s, window: %.1fm, fusion: %.1fHz)",
           cloud_topic.c_str(), 2.0 * crop_radius_xy_, fusion_rate_);
}

bool ElevationLocalPlannerPlugin::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
{
  if (!initialized_)
  {
    ROS_ERROR("ElevationLocalPlannerPlugin is not initialized! Call initialize first.");
    return false;
  }

  velocity_smoother_.reset();
  last_control_time_ = ros::Time(0);

  if (plan.empty())
  {
    global_plan_.clear();
    target_index_ = 0;
    pose_adjusting_ = false;
    goal_reached_ = true;
    return true;
  }

  global_plan_ = plan;
  target_index_ = 0;
  pose_adjusting_ = false;
  goal_reached_ = false;
  ROS_INFO("[ElevationLocalPlannerPlugin] Plan set with %zu points.", global_plan_.size());
  return true;
}

bool ElevationLocalPlannerPlugin::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
  if (!initialized_)
  {
    ROS_ERROR("ElevationLocalPlannerPlugin is not initialized!");
    return false;
  }

  if (global_plan_.empty())
  {
    ROS_WARN_THROTTLE(2.0, "[ElevationLocalPlannerPlugin] Global plan is empty.");
    cmd_vel = geometry_msgs::Twist();
    return false;
  }

  if (goal_reached_)
  {
    cmd_vel = geometry_msgs::Twist();
    return true;
  }

  // ---- 融合图足印碰撞安全层: 当前位置足印半径内存在禁行节点即零速停车 ----
  {
    double rx = 0.0, ry = 0.0, rz = 0.0;
    if (lookupRobotPose3D(rx, ry, rz) && checkFusedGraphCollision(rx, ry, rz))
    {
      ROS_WARN_THROTTLE(1.0, "[ElevationLocalPlannerPlugin] Fused-graph collision at (%.2f, %.2f, %.2f), emergency stop", rx, ry, rz);
      cmd_vel = geometry_msgs::Twist();
      last_cmd_vel_ = cmd_vel;
      return false;
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
      ROS_INFO("[ElevationLocalPlannerPlugin] Goal reached.");
      return true;
    }

    cmd_vel = velocity_smoother_.smooth(raw_cmd, dt);
    last_cmd_vel_ = cmd_vel;
    return true;
  }

  TrackingTarget target;
  if (!selectTrackingTarget(target))
  {
    cmd_vel = geometry_msgs::Twist();
    return false;
  }

  if (isFinalTrackingPointReached(target))
  {
    pose_adjusting_ = true;
    ROS_INFO("[ElevationLocalPlannerPlugin] Final tracking point reached. Switching to final yaw adjustment.");

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

    cmd_vel = velocity_smoother_.smooth(raw_cmd, dt);
    last_cmd_vel_ = cmd_vel;
    return true;
  }

  // 1. Decoupled Pure Pursuit: smooth heading error calculation
  const double heading_error = std::atan2(target.base_y, target.base_x);
  const double abs_heading   = std::abs(heading_error);

  // 2. Smooth cruise forward velocity (cruise speed + cornering adapt + goal ramp down)
  RobotPose2D robot_pose;
  double dist_to_goal = 1.0;
  if (lookupRobotPose2D(robot_pose) && !global_plan_.empty()) {
    const auto & g = global_plan_.back().pose.position;
    dist_to_goal = std::hypot(g.x - robot_pose.x, g.y - robot_pose.y);
  }

  const double cruise_speed = velocity_smoother_.getParams().max_linear_speed;
  double corner_scale = 0.0;
  if (abs_heading < 0.785) {
    corner_scale = std::cos(heading_error);
  } else if (abs_heading < 1.05) {
    corner_scale = std::cos(heading_error) * (1.05 - abs_heading) / (1.05 - 0.785);
  }
  const double goal_scale   = dist_to_goal < 0.6 ? std::max(0.0, dist_to_goal / 0.6) : 1.0;

  raw_cmd.linear.x  = cruise_speed * corner_scale * goal_scale;
  raw_cmd.linear.y  = (enable_lateral_motion_ && abs_heading < 0.785) ? target.base_y * lateral_gain_ : 0.0;
  raw_cmd.angular.z = heading_error * heading_gain_;

  cmd_vel = velocity_smoother_.smooth(raw_cmd, dt);
  last_cmd_vel_ = cmd_vel;

  ROS_DEBUG_THROTTLE(1.0,
    "[ElevationLocalPlannerPlugin] Track target: x=%.3f y=%.3f heading_err=%.3f cmd=(%.3f, %.3f, %.3f)",
    target.base_x, target.base_y, heading_error,
    cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);

  return true;
}

bool ElevationLocalPlannerPlugin::isGoalReached()
{
  if (!initialized_)
  {
    ROS_ERROR("ElevationLocalPlannerPlugin is not initialized!");
    return false;
  }
  return goal_reached_;
}

bool ElevationLocalPlannerPlugin::isFinalTrackingPointReached(const TrackingTarget & target) const
{
  if (global_plan_.empty() || target_index_ < static_cast<int>(global_plan_.size()) - 3)
    return false;
  RobotPose2D robot_pose;
  if (!const_cast<ElevationLocalPlannerPlugin*>(this)->lookupRobotPose2D(robot_pose)) return false;
  const auto & goal_pos = global_plan_.back().pose.position;
  return std::hypot(goal_pos.x - robot_pose.x, goal_pos.y - robot_pose.y) < tracking_xy_tol_;
}

bool ElevationLocalPlannerPlugin::selectTrackingTarget(TrackingTarget & target)
{
  if (global_plan_.empty()) return false;
  RobotPose2D robot_pose;
  if (!lookupRobotPose2D(robot_pose)) return false;

  if (!D1ControlUtils::interpolateLookaheadTarget(
        global_plan_, robot_pose, target_index_, lookahead_distance_, target))
  {
    return false;
  }
  return true;
}

bool ElevationLocalPlannerPlugin::lookupRobotPose2D(RobotPose2D & robot_pose)
{
  if (!costmap_ros_) return false;
  const std::string base_frame = costmap_ros_->getBaseFrameID();
  try {
    const auto tf = tf_buffer_->lookupTransform(map_frame_, base_frame, ros::Time(0), ros::Duration(0.05));
    robot_pose.x   = tf.transform.translation.x;
    robot_pose.y   = tf.transform.translation.y;
    robot_pose.z   = tf.transform.translation.z;
    robot_pose.yaw = tf2::getYaw(tf.transform.rotation);
    return true;
  } catch (const tf2::TransformException & ex) {
    ROS_WARN_THROTTLE(2.0, "[ElevationLocalPlannerPlugin] Lookup robot pose %s -> %s failed: %s",
      map_frame_.c_str(), base_frame.c_str(), ex.what());
    return false;
  }
}

bool ElevationLocalPlannerPlugin::lookupRobotPose3D(double & x, double & y, double & z)
{
  if (!costmap_ros_) return false;
  const std::string base_frame = costmap_ros_->getBaseFrameID();
  try {
    const auto tf = tf_buffer_->lookupTransform(map_frame_, base_frame, ros::Time(0), ros::Duration(0.05));
    x = tf.transform.translation.x;
    y = tf.transform.translation.y;
    z = tf.transform.translation.z;
    return true;
  } catch (const tf2::TransformException &) {
    return false;
  }
}

bool ElevationLocalPlannerPlugin::computeFinalYawErrorXY(const geometry_msgs::PoseStamped & final_pose_in, double & yaw_error)
{
  RobotPose2D robot_pose;
  if (!lookupRobotPose2D(robot_pose)) return false;
  geometry_msgs::PoseStamped final_pose = final_pose_in;
  if (final_pose.header.frame_id.empty()) final_pose.header.frame_id = map_frame_;
  final_pose.header.stamp = ros::Time(0);
  try {
    if (final_pose.header.frame_id != map_frame_)
      tf_buffer_->transform(final_pose, final_pose, map_frame_, ros::Duration(0.05));
  } catch (const tf2::TransformException & ex) {
    ROS_WARN_THROTTLE(2.0, "[ElevationLocalPlannerPlugin] Transform final pose yaw %s -> %s failed: %s",
      final_pose.header.frame_id.c_str(), map_frame_.c_str(), ex.what());
    return false;
  }
  yaw_error = D1ControlUtils::normalizeAngle(tf2::getYaw(final_pose.pose.orientation) - robot_pose.yaw);
  return true;
}

bool ElevationLocalPlannerPlugin::transformToBase(const geometry_msgs::PoseStamped & pose_in, geometry_msgs::PoseStamped & pose_out)
{
  if (!costmap_ros_) return false;
  const std::string base_frame = costmap_ros_->getBaseFrameID();
  geometry_msgs::PoseStamped stamped = pose_in;
  if (stamped.header.frame_id.empty()) stamped.header.frame_id = map_frame_;
  stamped.header.stamp = ros::Time(0);
  try {
    tf_buffer_->transform(stamped, pose_out, base_frame, ros::Duration(0.05));
    return true;
  } catch (const tf2::TransformException & ex) {
    ROS_WARN_THROTTLE(2.0, "[ElevationLocalPlannerPlugin] Transform %s -> %s failed: %s",
      stamped.header.frame_id.c_str(), base_frame.c_str(), ex.what());
    return false;
  }
}

void ElevationLocalPlannerPlugin::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr & msg)
{
  std::lock_guard<std::mutex> lock(cloud_mutex_);
  latest_cloud_ = msg;
  cloud_dirty_ = true;
  last_cloud_stamp_ = msg->header.stamp;
}

void ElevationLocalPlannerPlugin::fusionLoop()
{
  ros::Rate rate(fusion_rate_);
  while (fusion_running_ && ros::ok())
  {
    rate.sleep();

    sensor_msgs::PointCloud2::ConstPtr msg;
    {
      std::lock_guard<std::mutex> lock(cloud_mutex_);
      if (!cloud_dirty_ || !latest_cloud_) continue;
      msg = latest_cloud_;
      cloud_dirty_ = false;
    }

    double rx = 0.0, ry = 0.0, rz = 0.0;
    if (!lookupRobotPose3D(rx, ry, rz)) continue;

    // 点云变换到 map 系 (雷达/装配系与 map 不一致时)
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    try {
      if (msg->header.frame_id != map_frame_) {
        pcl::PointCloud<pcl::PointXYZ> raw;
        pcl::fromROSMsg(*msg, raw);
        if (!pcl_ros::transformPointCloud(map_frame_, raw, *cloud, *tf_buffer_)) continue;
      } else {
        pcl::fromROSMsg(*msg, *cloud);
      }
    } catch (const tf2::TransformException & ex) {
      ROS_WARN_THROTTLE(2.0, "[ElevationLocalPlannerPlugin] Cloud transform to %s failed: %s", map_frame_.c_str(), ex.what());
      continue;
    }

    // ROI 滚动窗口裁剪: 机器人周围方形窗口 + 垂直高度带
    const double band_low  = rz - crop_height_below_;
    const double band_high = rz + crop_height_above_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cropped(new pcl::PointCloud<pcl::PointXYZ>);
    cropped->reserve(cloud->size());
    const double r2 = crop_radius_xy_ * crop_radius_xy_;
    for (const auto & pt : cloud->points) {
      if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;
      const double dx = pt.x - rx, dy = pt.y - ry;
      if (dx * dx + dy * dy > r2) continue;
      if (pt.z < band_low || pt.z > band_high) continue;
      cropped->push_back(pt);
    }
    if (cropped->empty()) continue;

    // 局部 extent: 以机器人为中心的方形窗口 (与全局柱表分辨率一致, 融合时按世界坐标对齐)
    elevation_planner::GridExtent local_extent;
    local_extent.resolution = build_cfg_.resolution;
    local_extent.min_x = rx - crop_radius_xy_;
    local_extent.min_y = ry - crop_radius_xy_;
    local_extent.rows = static_cast<int>(std::ceil(2.0 * crop_radius_xy_ / local_extent.resolution)) + 1;
    local_extent.cols = local_extent.rows;

    elevation_planner::ColumnTable observed;
    if (!builder_.buildColumnTable(cropped, observed, &local_extent)) continue;
    if (observed.empty()) continue;

    // 逐柱并集融合: 观测优先, 先验补盲 (雷达扫不到的脚下支撑面), 边界圈强制先验
    auto prior = elevation_planner::GraphStore::instance().getGlobalTable();
    elevation_planner::ColumnTable fused = prior
      ? elevation_planner::fuseColumnTables(*prior, observed, build_cfg_.cluster_height_diff, 1)
      : observed;

    // 融合柱表重建流形图 (净空/膨胀/建边统一走同一套判据), 双缓冲刷新
    auto graph = std::make_shared<elevation_planner::ManifoldGraph>();
    if (!builder_.buildGraphFromColumnTable(fused, *graph)) continue;

    {
      std::lock_guard<std::mutex> lock(fused_graph_mutex_);
      fused_graph_ = graph;
      has_map_ = true;
    }
    ROS_DEBUG_THROTTLE(5.0, "[ElevationLocalPlannerPlugin] Fused graph updated: %zu nodes", graph->numNodes());
  }
}

bool ElevationLocalPlannerPlugin::checkFusedGraphCollision(double x, double y, double z)
{
  std::shared_ptr<const elevation_planner::ManifoldGraph> graph;
  {
    std::lock_guard<std::mutex> lock(fused_graph_mutex_);
    graph = fused_graph_;
  }
  if (!graph || graph->numNodes() == 0) return false;

  uint32_t nid = 0;
  if (!graph->findClosestNode(x, y, z, nid, 0.5, 0.8)) return false;
  int layer = graph->getNode(nid).layer_id;
  return collision_checker_.checkCollision(x, y, z, layer, *graph);
}

} // namespace elevation_local_planner
