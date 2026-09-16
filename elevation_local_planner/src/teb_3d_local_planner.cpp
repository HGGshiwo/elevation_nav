#include "elevation_local_planner/teb_3d_local_planner.h"
#include <pluginlib/class_list_macros.h>
#include <teb_local_planner/teb_local_planner_ros.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <base_local_planner/goal_functions.h>
#include <angles/angles.h>
#include <limits>

PLUGINLIB_EXPORT_CLASS(elevation_local_planner::Teb3DLocalPlanner, nav_core::BaseLocalPlanner)

namespace elevation_local_planner
{

Teb3DLocalPlanner::Teb3DLocalPlanner()
  : initialized_(false), tf_(nullptr), costmap_ros_(nullptr), costmap_(nullptr)
{
}

Teb3DLocalPlanner::~Teb3DLocalPlanner()
{
}

void Teb3DLocalPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
{
  if (initialized_)
  {
    ROS_WARN("Teb3DLocalPlanner has already been initialized, doing nothing.");
    return;
  }

  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();

  ros::NodeHandle nh("~/" + name);
  ros::NodeHandle nh_move_base("~");

  // 兼容参数命名空间 (支持 ~/Teb3DLocalPlanner 与 ~/TebLocalPlannerROS)
  ros::NodeHandle nh_param = nh.hasParam("max_vel_x") ? nh : ros::NodeHandle("~/TebLocalPlannerROS");

  // 加载 TEB 配置
  cfg_.loadRosParamFromNodeHandle(nh_param);

  // 3D 专属参数
  nh_param.param<double>("max_step_height", max_step_height_, 0.25);
  nh_param.param<double>("plan_slice_horizon", plan_slice_horizon_, 2.5);
  nh_param.param<double>("dog_height", dog_height_, 0.45);

  global_frame_ = costmap_ros_->getGlobalFrameID();
  cfg_.map_frame = global_frame_;
  robot_base_frame_ = costmap_ros_->getBaseFrameID();

  // 预留障碍物容量
  obstacles_.reserve(500);

  // 创建可视化
  visualization_ = teb_local_planner::TebVisualizationPtr(new teb_local_planner::TebVisualization(nh_param, cfg_));

  // 获取足端模型: 优先根据统一参数 (robot_length, robot_width, obstacle_safety_margin) 自适应构建倒角八边形
  double robot_length = 0.0, robot_width = 0.0;
  if (!nh_param.getParam("robot_width", robot_width)) nh.getParam("robot_width", robot_width);
  if (!nh_param.getParam("robot_length", robot_length)) nh.getParam("robot_length", robot_length);

  if (robot_length > 0.0 && robot_width > 0.0)
  {
    double margin = 0.04;
    if (!nh_param.getParam("obstacle_safety_margin", margin)) nh.getParam("obstacle_safety_margin", margin);
    cfg_.obstacles.min_obstacle_dist = margin;

    const double hx = robot_length * 0.5;
    const double hy = robot_width * 0.5;
    teb_local_planner::Point2dContainer poly;
    poly.reserve(8);
    poly.emplace_back(hx, hy * 0.6);
    poly.emplace_back(hx * 0.67, hy);
    poly.emplace_back(-hx * 0.67, hy);
    poly.emplace_back(-hx, hy * 0.6);
    poly.emplace_back(-hx, -hy * 0.6);
    poly.emplace_back(-hx * 0.67, -hy);
    poly.emplace_back(hx * 0.67, -hy);
    poly.emplace_back(hx, -hy * 0.6);

    cfg_.robot_model = boost::make_shared<teb_local_planner::PolygonRobotFootprint>(poly);
    ROS_INFO("[Teb3DLocalPlanner] Unified footprint polygon applied (L=%.2fm, W=%.2fm, margin=%.2fm)",
             robot_length, robot_width, margin);
  }
  else
  {
    cfg_.robot_model = teb_local_planner::TebLocalPlannerROS::getRobotFootprintFromParamServer(nh_param, cfg_);
  }

  // 初始化规划器 (支持同伦类探索)
  if (cfg_.hcp.enable_homotopy_class_planning)
  {
    planner_ = teb_local_planner::PlannerInterfacePtr(
        new teb_local_planner::HomotopyClassPlanner(cfg_, &obstacles_, visualization_, &via_points_));
    ROS_INFO("[Teb3DLocalPlanner] 3D-TEB parallel HCP enabled.");
  }
  else
  {
    planner_ = teb_local_planner::PlannerInterfacePtr(
        new teb_local_planner::TebOptimalPlanner(cfg_, &obstacles_, visualization_, &via_points_));
    ROS_INFO("[Teb3DLocalPlanner] 3D-TEB single trajectory planner enabled.");
  }

  // 初始化里程计辅助
  odom_helper_.setOdomTopic(cfg_.odom_topic);

  // 订阅结构化 3D 障碍物消息 (优先读取 private /move_base/TebLocalPlannerROS/obstacles)
  std::string obst_topic = "/move_base/TebLocalPlannerROS/obstacles";
  nh.param<std::string>("obstacles_topic", obst_topic, obst_topic);
  custom_obst_sub_ = nh_move_base.subscribe(obst_topic, 1, &Teb3DLocalPlanner::customObstacleCB, this);

  initialized_ = true;
  ROS_INFO("[Teb3DLocalPlanner] Plugin initialized with obstacles topic: %s", obst_topic.c_str());
}

void Teb3DLocalPlanner::customObstacleCB(const costmap_converter::ObstacleArrayMsg::ConstPtr& obst_msg)
{
  std::lock_guard<std::mutex> lock(custom_obst_mutex_);
  custom_obstacle_msg_ = *obst_msg;
  has_new_obstacles_ = true;
}

void Teb3DLocalPlanner::updateObstaclesFromMsg()
{
  std::lock_guard<std::mutex> lock(custom_obst_mutex_);
  obstacles_.clear();

  if (custom_obstacle_msg_.obstacles.empty()) return;

  for (const auto & obs : custom_obstacle_msg_.obstacles)
  {
    if (obs.polygon.points.empty()) continue;

    if (obs.polygon.points.size() == 2 && obs.radius <= 0.001)
    {
      // 3D 悬崖/台阶边缘线段
      Eigen::Vector3d p1(obs.polygon.points[0].x, obs.polygon.points[0].y, obs.polygon.points[0].z);
      Eigen::Vector3d p2(obs.polygon.points[1].x, obs.polygon.points[1].y, obs.polygon.points[1].z);
      obstacles_.push_back(boost::make_shared<LineObstacle3D>(p1, p2));
    }
    else if (obs.polygon.points.size() == 1 && obs.radius > 0.0)
    {
      // 3D 柱体
      Eigen::Vector3d center(obs.polygon.points[0].x, obs.polygon.points[0].y, obs.polygon.points[0].z);
      obstacles_.push_back(boost::make_shared<CylinderObstacle3D>(center, obs.radius, /*height=*/2.0));
    }
    else if (obs.polygon.points.size() > 2)
    {
      // 3D 多棱柱体
      std::vector<Eigen::Vector3d> verts;
      double z_min = std::numeric_limits<double>::max();
      double z_max = -std::numeric_limits<double>::max();
      for (const auto & pt : obs.polygon.points)
      {
        verts.emplace_back(pt.x, pt.y, pt.z);
        z_min = std::min(z_min, static_cast<double>(pt.z));
        z_max = std::max(z_max, static_cast<double>(pt.z));
      }
      obstacles_.push_back(boost::make_shared<PolygonObstacle3D>(verts, z_min, z_max + 1.5));
    }
    else if (obs.polygon.points.size() == 1)
    {
      // 孤立点障碍物
      Eigen::Vector3d pt(obs.polygon.points[0].x, obs.polygon.points[0].y, obs.polygon.points[0].z);
      obstacles_.push_back(boost::make_shared<CylinderObstacle3D>(pt, cfg_.obstacles.min_obstacle_dist * 0.5, 2.0));
    }
  }
}

bool Teb3DLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
{
  if (!initialized_)
  {
    ROS_ERROR("[Teb3DLocalPlanner] Not initialized, cannot call setPlan.");
    return false;
  }

  global_plan_ = orig_global_plan;
  return true;
}

bool Teb3DLocalPlanner::pruneGlobalPlan(const geometry_msgs::PoseStamped& global_pose,
                                       std::vector<geometry_msgs::PoseStamped>& global_plan,
                                       double dist_behind_robot)
{
  if (global_plan.empty()) return true;

  double rx = global_pose.pose.position.x;
  double ry = global_pose.pose.position.y;
  double rz = global_pose.pose.position.z;
  double r_yaw = tf2::getYaw(global_pose.pose.orientation);

  // 1. 连续线段正交投影：在前瞻搜索窗口内寻找机器人投影最近的线段
  size_t max_search = std::min(global_plan.size(), size_t(30));
  size_t seg_idx = 0;
  double min_d_sq = std::numeric_limits<double>::max();

  for (size_t i = 0; i + 1 < max_search; ++i)
  {
    const auto& p1 = global_plan[i].pose.position;
    const auto& p2 = global_plan[i + 1].pose.position;
    double vx = p2.x - p1.x;
    double vy = p2.y - p1.y;
    double vz = p2.z - p1.z;
    double v_sq = vx * vx + vy * vy + vz * vz;
    if (v_sq < 1e-6) continue;

    double t = ((rx - p1.x) * vx + (ry - p1.y) * vy + (rz - p1.z) * vz) / v_sq;
    t = std::max(0.0, std::min(1.0, t));

    double px = p1.x + t * vx;
    double py = p1.y + t * vy;
    double pz = p1.z + t * vz;
    double d_sq = (rx - px) * (rx - px) + (ry - py) * (ry - py) + (rz - pz) * (rz - pz);

    if (d_sq < min_d_sq)
    {
      min_d_sq = d_sq;
      seg_idx = (t >= 0.99) ? (i + 1) : i;
    }
  }

  // 2. 前瞻视线锚点搜索 (Lookahead Anchor Selection)
  // 从 seg_idx 开始向前寻找第一个满足前向可行视野锥与最小前瞻距离的引导航点
  size_t anchor_idx = seg_idx;
  bool found_anchor = false;
  double best_angle_diff = std::numeric_limits<double>::max();
  size_t best_relaxed_idx = seg_idx;

  const double min_lookahead = 0.25;                    // 最小前瞻距离，避免 10cm 内直角横折
  const double forward_cone = M_PI / 3.0;               // 60度前向可行视野锥
  const double relaxed_cone = 5.0 * M_PI / 12.0;        // 75度放宽视角

  for (size_t i = seg_idx; i < max_search; ++i)
  {
    double dx = global_plan[i].pose.position.x - rx;
    double dy = global_plan[i].pose.position.y - ry;
    double dist = std::hypot(dx, dy);

    double angle_to_p = std::atan2(dy, dx);
    double angle_diff = std::abs(angles::shortest_angular_distance(r_yaw, angle_to_p));

    if (angle_diff < best_angle_diff)
    {
      best_angle_diff = angle_diff;
      best_relaxed_idx = i;
    }

    if (dist >= min_lookahead && angle_diff <= forward_cone)
    {
      anchor_idx = i;
      found_anchor = true;
      break;
    }
  }

  // 3. 降级兜底逻辑 (Fallback Hierarchy)
  if (!found_anchor)
  {
    if (global_plan.size() <= 3 || seg_idx + 2 >= global_plan.size())
    {
      // 接近终点：直接保留至终点
      anchor_idx = (seg_idx < global_plan.size() - 1) ? seg_idx : (global_plan.size() - 1);
    }
    else if (best_angle_diff <= relaxed_cone)
    {
      // 自适应放宽视角（<= 75度）
      anchor_idx = best_relaxed_idx;
    }
    else
    {
      // 航向偏差较大，选择夹角最小的前方点
      anchor_idx = (best_relaxed_idx > seg_idx) ? best_relaxed_idx : seg_idx;
    }
  }

  // 4. 彻底剪除 anchor_idx 之前的历史/横折航点，至少保留终点
  if (anchor_idx >= global_plan.size())
  {
    anchor_idx = global_plan.size() - 1;
  }

  if (anchor_idx > 0)
  {
    global_plan.erase(global_plan.begin(), global_plan.begin() + anchor_idx);
  }
  return true;
}

bool Teb3DLocalPlanner::transformGlobalPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan,
                                           const geometry_msgs::PoseStamped& global_pose,
                                           const costmap_2d::Costmap2D& costmap,
                                           const std::string& global_frame,
                                           double max_plan_length,
                                           std::vector<geometry_msgs::PoseStamped>& transformed_plan)
{
  transformed_plan.clear();
  if (global_plan.empty()) return false;

  double max_sq = max_plan_length * max_plan_length;
  double acc_len = 0.0;

  for (size_t i = 0; i < global_plan.size(); ++i)
  {
    const auto & p = global_plan[i];
    double dx = p.pose.position.x - global_pose.pose.position.x;
    double dy = p.pose.position.y - global_pose.pose.position.y;
    double dz = p.pose.position.z - global_pose.pose.position.z;
    if (dx * dx + dy * dy + dz * dz > max_sq) break;

    if (!transformed_plan.empty())
    {
      // 沿路径累积 3D 几何欧氏距离，克服大坡度楼梯在 2D 投影距离严重压缩失真的缺陷
      double step = std::sqrt(std::pow(p.pose.position.x - transformed_plan.back().pose.position.x, 2) +
                              std::pow(p.pose.position.y - transformed_plan.back().pose.position.y, 2) +
                              std::pow(p.pose.position.z - transformed_plan.back().pose.position.z, 2));
      acc_len += step;
      if (acc_len > max_plan_length) break;
    }
    transformed_plan.push_back(p);
  }

  if (transformed_plan.empty() && !global_plan.empty())
  {
    transformed_plan.push_back(global_plan.front());
  }
  return true;
}

void Teb3DLocalPlanner::updateViaPointsSafe(const std::vector<geometry_msgs::PoseStamped>& transformed_plan,
                                           double min_separation)
{
  via_points_.clear();
  if (min_separation <= 0.0 || transformed_plan.size() <= 1) return;

  auto graph = elevation_planner::GraphStore::instance().getGlobalGraph();
  if (!graph || graph->numNodes() == 0)
  {
    graph = elevation_planner::GraphStore::instance().getFusedGraph();
  }

  // 沿路径 3D 欧氏距离均匀采样并筛选安全 via-points

  size_t prev_idx = 0;
  for (size_t i = 1; i < transformed_plan.size(); ++i)
  {
    const auto & p_prev = transformed_plan[prev_idx].pose.position;
    const auto & p_curr = transformed_plan[i].pose.position;

    // 沿路径 3D 欧氏距离均匀采样
    double dist_3d = std::sqrt(std::pow(p_curr.x - p_prev.x, 2) +
                               std::pow(p_curr.y - p_prev.y, 2) +
                               std::pow(p_curr.z - p_prev.z, 2));
    if (dist_3d < min_separation) continue;

    bool is_safe = true;

    // 1. 检查流形图节点属性 (若落入硬阻挡/顶头净空不足/过大通行阻力则剔除)
    if (graph && graph->numNodes() > 0)
    {
      uint32_t nid = 0;
      if (graph->findClosestNode(p_curr.x, p_curr.y, p_curr.z, nid, 0.35, 0.50))
      {
        const auto & nd = graph->getNode(nid);
        if (nd.traversability >= 0.8f ||
            (nd.headroom > 0.0f && nd.headroom < dog_height_) ||
            (nd.flags & elevation_planner::node_flags::BLOCK_HEADROOM))
        {
          is_safe = false;
        }
      }
    }

    // 2. 检查 3D 几何障碍物距离 (若落入或过分贴近障碍物则剔除，避免与 TEB 斥力拔河)
    if (is_safe && !obstacles_.empty())
    {
      Eigen::Vector2d pt_2d(p_curr.x, p_curr.y);
      for (const auto & obs : obstacles_)
      {
        if (obs && obs->getMinimumDistance(pt_2d) < cfg_.obstacles.min_obstacle_dist)
        {
          is_safe = false;
          break;
        }
      }
    }

    if (is_safe)
    {
      via_points_.emplace_back(p_curr.x, p_curr.y);
    }
    prev_idx = i;
  }
}

bool Teb3DLocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
  if (!initialized_)
  {
    ROS_ERROR("[Teb3DLocalPlanner] Not initialized!");
    return false;
  }

  cmd_vel.linear.x = 0.0;
  cmd_vel.linear.y = 0.0;
  cmd_vel.angular.z = 0.0;

  // 1. 获取当前机器人位姿 (含 3D 高程)
  geometry_msgs::PoseStamped robot_pose;
  if (!costmap_ros_->getRobotPose(robot_pose))
  {
    ROS_ERROR("[Teb3DLocalPlanner] Cannot get robot pose from costmap_ros!");
    return false;
  }
  current_robot_pose_ = robot_pose;

  // 2. 获取当前机器人速度
  geometry_msgs::PoseStamped robot_vel_tf;
  odom_helper_.getRobotVel(robot_vel_tf);
  robot_vel_.linear.x = robot_vel_tf.pose.position.x;
  robot_vel_.linear.y = robot_vel_tf.pose.position.y;
  robot_vel_.angular.z = tf2::getYaw(robot_vel_tf.pose.orientation);

  // 3. 检查全局路径有效性
  if (global_plan_.empty())
  {
    ROS_WARN_NAMED("teb_3d_local_planner", "Global plan is empty!");
    return false;
  }

  // 4. 路径剪裁与局部前瞻提取
  pruneGlobalPlan(robot_pose, global_plan_, cfg_.trajectory.global_plan_prune_distance);

  std::vector<geometry_msgs::PoseStamped> transformed_plan;
  if (!transformGlobalPlan(global_plan_, robot_pose, *costmap_, global_frame_,
                           cfg_.trajectory.max_global_plan_lookahead_dist, transformed_plan))
  {
    ROS_WARN("[Teb3DLocalPlanner] Failed to transform and crop global plan!");
    return false;
  }

  if (transformed_plan.empty())
  {
    return false;
  }

  // 起点对齐当前机器人实际位置并确保前向航点平滑延伸
  if (transformed_plan.size() <= 1)
  {
    transformed_plan.push_back(robot_pose);
    std::swap(transformed_plan.front(), transformed_plan.back());
  }
  else
  {
    double dx = transformed_plan.front().pose.position.x - robot_pose.pose.position.x;
    double dy = transformed_plan.front().pose.position.y - robot_pose.pose.position.y;
    double dz = transformed_plan.front().pose.position.z - robot_pose.pose.position.z;
    if (dx * dx + dy * dy + dz * dz < 0.15 * 0.15)
    {
      transformed_plan.front() = robot_pose;
    }
    else
    {
      transformed_plan.insert(transformed_plan.begin(), robot_pose);
    }
  }

  // 5. 更新装载 3D 几何障碍物
  updateObstaclesFromMsg();

  // 5.5 安全生成 Via-Points 路径引导约束 (带障碍物与流形图通行度校验)
  updateViaPointsSafe(transformed_plan, cfg_.trajectory.global_plan_viapoint_sep);

  // 6. 执行 TEB 弹性带时空优化
  bool success = planner_->plan(transformed_plan, &robot_vel_, cfg_.goal_tolerance.free_goal_vel);
  if (!success)
  {
    planner_->clearPlanner();
    ROS_WARN_THROTTLE(1.0, "[Teb3DLocalPlanner] TEB failed to produce a feasible trajectory.");
    return false;
  }

  // 7. 三维流形物理校验与多同伦候选轨迹优选 (TrajectoryValidator)
  teb_local_planner::TebOptimalPlannerPtr selected_teb = nullptr;
  double robot_z = robot_pose.pose.position.z;
  double target_z = transformed_plan.back().pose.position.z;
  Eigen::Vector2d target_xy(transformed_plan.back().pose.position.x, transformed_plan.back().pose.position.y);

  // 提取局部前瞻范围内的全局路径峰值高程 (针对"上->转角->下"立体构型, 峰值即转角平台)
  double plan_peak_z = -std::numeric_limits<double>::max();
  for (const auto & p : transformed_plan)
  {
    plan_peak_z = std::max(plan_peak_z, static_cast<double>(p.pose.position.z));
  }
  double min_peak_z = std::numeric_limits<double>::quiet_NaN();
  if (plan_peak_z > std::max(robot_z, target_z) + 0.30)
  {
    min_peak_z = plan_peak_z;
  }

  std::string last_reject_reason = "none";
  auto hcp = boost::dynamic_pointer_cast<teb_local_planner::HomotopyClassPlanner>(planner_);
  if (hcp && !hcp->getTrajectoryContainer().empty())
  {
    double best_cost = std::numeric_limits<double>::max();
    const auto & candidates = hcp->getTrajectoryContainer();

    for (const auto & opt : candidates)
    {
      if (!opt || !opt->isOptimized()) continue;

      std::vector<Eigen::Vector2d> xy_poses;
      const auto & poses = opt->teb().poses();
      xy_poses.reserve(poses.size());
      for (const auto & p : poses)
      {
        xy_poses.push_back(p->position());
      }

      // 复用建图算法口径的高程物理连续性、中间支撑、目标 Z 与峰值高程验证 (杜绝底层穿模绕行)
      std::string current_reason;
      bool is_3d_valid = elevation_planner::TrajectoryValidator::validate(
          xy_poses, robot_z, target_z, target_xy, max_step_height_, min_peak_z, &current_reason);

      if (is_3d_valid)
      {
        double cost = opt->getCurrentCost();
        if (cost < best_cost)
        {
          best_cost = cost;
          selected_teb = opt;
        }
      }
      else
      {
        last_reject_reason = current_reason;
        ROS_DEBUG_THROTTLE(1.0, "[Teb3DLocalPlanner] Candidate rejected by 3D validation: %s", current_reason.c_str());
      }
    }
  }
  else
  {
    // 单轨迹优化模式校验
    auto single_teb = boost::dynamic_pointer_cast<teb_local_planner::TebOptimalPlanner>(planner_);
    if (single_teb && single_teb->isOptimized())
    {
      std::vector<Eigen::Vector2d> xy_poses;
      const auto & poses = single_teb->teb().poses();
      xy_poses.reserve(poses.size());
      for (const auto & p : poses)
      {
        xy_poses.push_back(p->position());
      }
      if (elevation_planner::TrajectoryValidator::validate(
              xy_poses, robot_z, target_z, target_xy, max_step_height_, min_peak_z, &last_reject_reason))
      {
        selected_teb = single_teb;
      }
    }
  }

  // 严格安全把关：若所有候选均未通过 3D 物理验证，坚决不执行错误轨迹，直接判定失败停车
  if (!selected_teb)
  {
    ROS_WARN_THROTTLE(1.0, "[Teb3DLocalPlanner] All trajectories rejected by 3D manifold validation (reason: %s). Halting.",
                      last_reject_reason.c_str());
    planner_->clearPlanner();
    return false;
  }



  // 8. 提取速度指令
  selected_teb->getVelocityCommand(cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z,
                                   cfg_.trajectory.control_look_ahead_poses);

  // 9. 构建并发布具备真实三维高程 Z 的局部规划路径与整体规划可视化
  if (visualization_)
  {
    std::vector<geometry_msgs::PoseStamped> local_3d_plan;
    const auto & poses = selected_teb->teb().poses();
    local_3d_plan.reserve(poses.size());
    ros::Time now = ros::Time::now();

    size_t tp_start_idx = 0;
    double last_ref_z = robot_z;

    for (const auto & p : poses)
    {
      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = global_frame_;
      pose.header.stamp = now;
      pose.pose.position.x = p->position().x();
      pose.pose.position.y = p->position().y();

      // 沿 transformed_plan 单调前向搜索最近全局高程（加权 3D 距离，防止折返楼梯处 2D 投影误吸附到下层）
      double ref_z = last_ref_z;
      double min_d3_sq = std::numeric_limits<double>::max();
      size_t best_tp_idx = tp_start_idx;
      size_t search_end = std::min(transformed_plan.size(), tp_start_idx + 15);

      for (size_t j = tp_start_idx; j < search_end; ++j)
      {
        const auto & tp = transformed_plan[j];
        double d2_xy = std::pow(p->position().x() - tp.pose.position.x, 2) +
                       std::pow(p->position().y() - tp.pose.position.y, 2);
        double dz = tp.pose.position.z - last_ref_z;
        double d3_sq = d2_xy + 3.0 * dz * dz;
        if (d3_sq < min_d3_sq)
        {
          min_d3_sq = d3_sq;
          ref_z = tp.pose.position.z;
          best_tp_idx = j;
        }
      }

      tp_start_idx = best_tp_idx;
      last_ref_z = ref_z;

      pose.pose.position.z = querySurfaceZ(p->position().x(), p->position().y(), ref_z, max_step_height_);

      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, p->theta());
      pose.pose.orientation = tf2::toMsg(q);
      local_3d_plan.push_back(pose);
    }

    visualization_->publishLocalPlan(local_3d_plan);
    visualization_->publishObstacles(obstacles_, costmap_->getResolution());
    visualization_->publishViaPoints(via_points_);
    visualization_->publishGlobalPlan(global_plan_);
  }

  return true;
}

bool Teb3DLocalPlanner::isGoalReached()
{
  if (!initialized_ || global_plan_.empty()) return false;

  geometry_msgs::PoseStamped robot_pose;
  if (!costmap_ros_->getRobotPose(robot_pose)) return false;

  const auto & goal = global_plan_.back();
  double dx = robot_pose.pose.position.x - goal.pose.position.x;
  double dy = robot_pose.pose.position.y - goal.pose.position.y;
  double dist_xy = std::hypot(dx, dy);

  if (dist_xy > cfg_.goal_tolerance.xy_goal_tolerance)
  {
    return false;
  }

  double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);
  double goal_yaw = tf2::getYaw(goal.pose.orientation);
  double yaw_diff = std::abs(angles::shortest_angular_distance(robot_yaw, goal_yaw));

  return (yaw_diff <= cfg_.goal_tolerance.yaw_goal_tolerance);
}

} // namespace elevation_local_planner
