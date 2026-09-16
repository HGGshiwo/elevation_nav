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

  global_frame_ = costmap_ros_->getGlobalFrameID();
  cfg_.map_frame = global_frame_;
  robot_base_frame_ = costmap_ros_->getBaseFrameID();

  // 预留障碍物容量
  obstacles_.reserve(500);

  // 创建可视化
  visualization_ = teb_local_planner::TebVisualizationPtr(new teb_local_planner::TebVisualization(nh_param, cfg_));

  // 获取足端模型
  cfg_.robot_model = teb_local_planner::TebLocalPlannerROS::getRobotFootprintFromParamServer(nh_param, cfg_);

  // 初始化规划器 (支持同伦类探索)
  if (cfg_.hcp.enable_homotopy_class_planning)
  {
    planner_ = teb_local_planner::PlannerInterfacePtr(
        new teb_local_planner::HomotopyClassPlanner(cfg_, &obstacles_, visualization_, &via_points_));
    ROS_INFO("[Teb3DLocalPlanner] 3D-TEB 多拓扑同伦类规划器已启用 (Parallel HCP Enabled).");
  }
  else
  {
    planner_ = teb_local_planner::PlannerInterfacePtr(
        new teb_local_planner::TebOptimalPlanner(cfg_, &obstacles_, visualization_, &via_points_));
    ROS_INFO("[Teb3DLocalPlanner] 3D-TEB 单轨迹优化规划器已启用.");
  }

  // 初始化里程计辅助
  odom_helper_.setOdomTopic(cfg_.odom_topic);

  // 订阅结构化 3D 障碍物消息 (优先读取 private /move_base/TebLocalPlannerROS/obstacles)
  std::string obst_topic = "/move_base/TebLocalPlannerROS/obstacles";
  nh.param<std::string>("obstacles_topic", obst_topic, obst_topic);
  custom_obst_sub_ = nh_move_base.subscribe(obst_topic, 1, &Teb3DLocalPlanner::customObstacleCB, this);

  initialized_ = true;
  ROS_INFO("[Teb3DLocalPlanner] 插件初始化完成, 障碍物话题: %s", obst_topic.c_str());
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
    ROS_ERROR("[Teb3DLocalPlanner] 未初始化, 无法调用 setPlan");
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

  double dist_thresh_sq = dist_behind_robot * dist_behind_robot;
  auto it = global_plan.begin();
  auto erase_end = it;

  while (it != global_plan.end())
  {
    double dx = global_pose.pose.position.x - it->pose.position.x;
    double dy = global_pose.pose.position.y - it->pose.position.y;
    if (dx * dx + dy * dy < dist_thresh_sq)
    {
      erase_end = it;
      break;
    }
    ++it;
  }

  if (erase_end != global_plan.begin() && erase_end != global_plan.end())
  {
    global_plan.erase(global_plan.begin(), erase_end);
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
    if (dx * dx + dy * dy > max_sq) break;

    if (!transformed_plan.empty())
    {
      double step = std::hypot(p.pose.position.x - transformed_plan.back().pose.position.x,
                               p.pose.position.y - transformed_plan.back().pose.position.y);
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

bool Teb3DLocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
  if (!initialized_)
  {
    ROS_ERROR("[Teb3DLocalPlanner] 未初始化!");
    return false;
  }

  cmd_vel.linear.x = 0.0;
  cmd_vel.linear.y = 0.0;
  cmd_vel.angular.z = 0.0;

  // 1. 获取当前机器人位姿 (含 3D 高程)
  geometry_msgs::PoseStamped robot_pose;
  if (!costmap_ros_->getRobotPose(robot_pose))
  {
    ROS_ERROR("[Teb3DLocalPlanner] 无法从 costmap_ros 获取机器人当前位姿!");
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
    ROS_WARN_NAMED("teb_3d_local_planner", "全局路径为空!");
    return false;
  }

  // 4. 路径剪裁与局部前瞻提取
  pruneGlobalPlan(robot_pose, global_plan_, cfg_.trajectory.global_plan_prune_distance);

  std::vector<geometry_msgs::PoseStamped> transformed_plan;
  if (!transformGlobalPlan(global_plan_, robot_pose, *costmap_, global_frame_,
                           cfg_.trajectory.max_global_plan_lookahead_dist, transformed_plan))
  {
    ROS_WARN("[Teb3DLocalPlanner] 全局路径局部截取失败!");
    return false;
  }

  if (transformed_plan.empty())
  {
    return false;
  }

  // 起点对齐当前机器人实际位置
  transformed_plan.front() = robot_pose;

  // 5. 更新装载 3D 几何障碍物
  updateObstaclesFromMsg();

  // 6. 执行 TEB 弹性带时空优化
  bool success = planner_->plan(transformed_plan, &robot_vel_, cfg_.goal_tolerance.free_goal_vel);
  if (!success)
  {
    planner_->clearPlanner();
    ROS_WARN_THROTTLE(1.0, "[Teb3DLocalPlanner] TEB 求解器未能获得可行局部轨迹");
    return false;
  }

  // 7. 三维流形物理校验与多同伦候选轨迹优选 (TrajectoryValidator)
  teb_local_planner::TebOptimalPlannerPtr selected_teb = nullptr;
  auto hcp = boost::dynamic_pointer_cast<teb_local_planner::HomotopyClassPlanner>(planner_);
  if (hcp && !hcp->getTrajectoryContainer().empty())
  {
    double robot_z = robot_pose.pose.position.z;
    double target_z = transformed_plan.back().pose.position.z;
    Eigen::Vector2d target_xy(transformed_plan.back().pose.position.x, transformed_plan.back().pose.position.y);

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

      // 复用建图算法口径的高程物理连续性、中间支撑与目标 Z 验证
      bool is_3d_valid = elevation_planner::TrajectoryValidator::validate(
          xy_poses, robot_z, target_z, target_xy, max_step_height_);

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
        ROS_DEBUG_THROTTLE(1.0, "[Teb3DLocalPlanner] 候选同伦轨迹 3D 验证不合格 (跨层/悬崖/步高超标), 已淘汰");
      }
    }
  }

  // 若 HCP 没能优选或单轨迹模式，回退检查默认最佳轨迹
  if (!selected_teb)
  {
    if (hcp)
    {
      selected_teb = hcp->bestTeb();
    }
    else
    {
      selected_teb = boost::dynamic_pointer_cast<teb_local_planner::TebOptimalPlanner>(planner_);
    }
  }

  if (!selected_teb)
  {
    ROS_WARN_THROTTLE(1.0, "[Teb3DLocalPlanner] 无可用的局部轨迹!");
    planner_->clearPlanner();
    return false;
  }

  // 8. 提取速度指令
  selected_teb->getVelocityCommand(cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z,
                                   cfg_.trajectory.control_look_ahead_poses);

  // 9. 发布局部路径与整体规划可视化
  selected_teb->visualize();
  if (visualization_)
  {
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
