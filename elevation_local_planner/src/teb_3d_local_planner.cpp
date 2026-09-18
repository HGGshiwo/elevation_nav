#include "elevation_local_planner/teb_3d_local_planner.h"
#include <elevation_planner_core/topological_corridor.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
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
  nh_param.param<double>("corridor_radius", corridor_radius_, 2.5);

  ros::NodeHandle nh_root;
  corridor_pub_ = nh_root.advertise<sensor_msgs::PointCloud2>("/elevation_corridor_nodes", 1);
  corridor_boundary_pub_ = nh_root.advertise<visualization_msgs::MarkerArray>("/elevation_corridor_boundaries", 1);

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

void Teb3DLocalPlanner::populateFrenetObstacles(const elevation_planner::FrenetFrame& frenet_frame,
                                               double s_start, double s_end, double l_robot)
{
  obstacles_.clear();
  via_points_.clear(); // 彻底禁用 via-points，释放避障弹性自由度

  if (frenet_frame.empty()) return;

  // 1. 生成 Frenet 走廊左/右硬边界 (由连续 LineObstacle 折线段构成)
  const double step_s = 0.25; // 每隔 25cm 采样一段边界线段
  double cur_s = std::max(0.0, s_start - 0.20);
  double max_s = std::min(frenet_frame.totalLength(), s_end + 0.30);

  double prev_s = cur_s;
  double prev_wl = frenet_frame.getLeftWidth(prev_s);
  double prev_wr = frenet_frame.getRightWidth(prev_s);

  // 起点断面自适应包络当前机器人横向偏距，防止因轻微偏差触发边界碰撞
  if (std::abs(prev_s - s_start) < 0.50)
  {
    prev_wl = std::max(prev_wl, l_robot + 0.20);
    prev_wr = std::max(prev_wr, -l_robot + 0.20);
  }

  for (cur_s = prev_s + step_s; cur_s <= max_s + 1e-4; cur_s += step_s)
  {
    double cur_eval_s = std::min(cur_s, max_s);
    double cur_wl = frenet_frame.getLeftWidth(cur_eval_s);
    double cur_wr = frenet_frame.getRightWidth(cur_eval_s);

    if (std::abs(cur_eval_s - s_start) < 0.50)
    {
      cur_wl = std::max(cur_wl, l_robot + 0.20);
      cur_wr = std::max(cur_wr, -l_robot + 0.20);
    }

    // 左边界折线段 (l = +W_left)
    obstacles_.push_back(boost::make_shared<teb_local_planner::LineObstacle>(
        Eigen::Vector2d(prev_s, prev_wl), Eigen::Vector2d(cur_eval_s, cur_wl)));

    // 右边界折线段 (l = -W_right)
    obstacles_.push_back(boost::make_shared<teb_local_planner::LineObstacle>(
        Eigen::Vector2d(prev_s, -prev_wr), Eigen::Vector2d(cur_eval_s, -cur_wr)));

    prev_s = cur_eval_s;
    prev_wl = cur_wl;
    prev_wr = cur_wr;
    if (prev_s >= max_s) break;
  }

  // 2. 投影结构化外部 3D 障碍物 (若有)
  {
    std::lock_guard<std::mutex> lock(custom_obst_mutex_);
    for (const auto& obs : custom_obstacle_msg_.obstacles)
    {
      // 方案 1: 忽略流形图踏面断坎/网格边界线段 (ID >= 2000)。
      // 踏面可行走物理边界在步骤 1 中已由 3D 拓扑管道左右护栏 (LineObstacle) 严格约束，
      // 绝不可将楼梯与平台边缘重复投影为障碍物，避免在机器人脚底与正前方形成虚假路障导致锁死。
      if (obs.id >= 2000) continue;

      for (const auto& pt : obs.polygon.points)
      {
        double so = 0.0, lo = 0.0, zo = 0.0;
        frenet_frame.toFrenet(pt.x, pt.y, pt.z, so, lo, zo);
        if (so >= s_start - 0.2 && so <= s_end + 0.3)
        {
          if (std::abs(pt.z - zo) <= dog_height_ + 0.15)
          {
            double r = std::max(0.04, static_cast<double>(obs.radius));
            obstacles_.push_back(boost::make_shared<teb_local_planner::CircularObstacle>(so, lo, r));
          }
        }
      }
    }
  }

  // 3. 从局部 Costmap 投影当前踏面高度层内的致命实体障碍物
  if (costmap_ && cfg_.obstacles.include_costmap_obstacles)
  {
    double rx = current_robot_pose_.pose.position.x;
    double ry = current_robot_pose_.pose.position.y;
    double rz = current_robot_pose_.pose.position.z;

    const double search_r = std::max(2.5, s_end - s_start + 0.5);
    int min_cx = 0, min_cy = 0, max_cx = 0, max_cy = 0;
    costmap_->worldToMapEnforceBounds(rx - search_r, ry - search_r, min_cx, min_cy);
    costmap_->worldToMapEnforceBounds(rx + search_r, ry + search_r, max_cx, max_cy);

    // 稀疏栅格哈希，防止障碍点过密导致优化迟滞 (间距 12cm)
    std::set<std::pair<int, int>> visited_cells;
    const int step = 2; // 10cm 采样

    for (int cy = min_cy; cy <= max_cy; cy += step)
    {
      for (int cx = min_cx; cx <= max_cx; cx += step)
      {
        if (costmap_->getCost(cx, cy) >= costmap_2d::LETHAL_OBSTACLE)
        {
          double wx = 0.0, wy = 0.0;
          costmap_->mapToWorld(cx, cy, wx, wy);
          double wz = querySurfaceZ(wx, wy, rz, 0.50);

          double so = 0.0, lo = 0.0, zo = 0.0;
          frenet_frame.toFrenet(wx, wy, wz, so, lo, zo);

          if (so >= s_start - 0.15 && so <= s_end + 0.30)
          {
            // 垂直高程过滤：严格排除上层天花板/下层地板点云
            if (std::abs(wz - zo) <= dog_height_ + 0.15)
            {
              double wl = frenet_frame.getLeftWidth(so);
              double wr = frenet_frame.getRightWidth(so);
              if (lo >= -wr - 0.20 && lo <= wl + 0.20)
              {
                int g_s = static_cast<int>(std::round(so / 0.12));
                int g_l = static_cast<int>(std::round(lo / 0.12));
                if (visited_cells.insert({g_s, g_l}).second)
                {
                  obstacles_.push_back(boost::make_shared<teb_local_planner::PointObstacle>(so, lo));
                }
              }
            }
          }
        }
      }
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

  // 1. 连续线段三维几何正交投影：在前瞻搜索窗口内寻找机器人投影最近的线段
  size_t max_search = std::min(global_plan.size(), size_t(30));
  size_t closest_idx = 0;
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
      // 当在当前段上投影进展超过前半程 (t >= 0.5) 时，当前点推进至 p2
      closest_idx = (t >= 0.5) ? (i + 1) : i;
    }
  }

  // 2. 严格修剪准则：仅剪除机器人身后已经走过的历史航点 (closest_idx 之前的航点)
  // 严禁越权向前方搜索吞噬未来航点！在楼梯与发夹弯处，前方每个 10cm 细微台阶
  // 和绕弯微元都是避免切角踏空跌落的绝对生命线。
  if (closest_idx >= global_plan.size())
  {
    closest_idx = global_plan.size() - 1;
  }

  if (closest_idx > 0)
  {
    global_plan.erase(global_plan.begin(), global_plan.begin() + closest_idx);
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

  // 5. 沿 3D 参考路径构建 Frenet 弧长-横向参数化流形空间 (FrenetFrame)
  elevation_planner::FrenetFrame frenet_frame;
  if (!frenet_frame.initialize(transformed_plan))
  {
    ROS_WARN_THROTTLE(1.0, "[Teb3DLocalPlanner] Failed to initialize FrenetFrame!");
    return false;
  }

  // 5.1 从流形连通图提取当前前瞻段的 3D 拓扑管道并精确解析各断面通行净宽
  auto graph = elevation_planner::GraphStore::instance().getGlobalGraph();
  if (!graph || graph->numNodes() == 0)
  {
    graph = elevation_planner::GraphStore::instance().getFusedGraph();
  }

  if (graph && graph->numNodes() > 0)
  {
    auto corridor = elevation_planner::TopologicalCorridorGenerator::generate(
        *graph, transformed_plan, corridor_radius_, /*lookahead_dist=*/0.0, max_step_height_, dog_height_);
    frenet_frame.computeCorridorWidth(*graph, corridor, corridor_radius_, max_step_height_, dog_height_);

    // 发布 3D 走廊连续曲面与发光护栏 Marker (节流 5Hz)
    ros::Time now = ros::Time::now();
    if ((now - last_corridor_pub_time_).toSec() >= 0.2)
    {
      last_corridor_pub_time_ = now;
      visualization_msgs::MarkerArray marker_msg;
      frenet_frame.toCorridorMarkers(global_frame_, marker_msg, dog_height_ * 0.75);
      corridor_boundary_pub_.publish(marker_msg);
    }
  }

  // 6. 将当前机器人位姿投影至 Frenet 坐标系 (s_robot, l_robot, yaw_robot_rel)
  double s_robot = 0.0, l_robot = 0.0, z_proj = 0.0, ref_yaw_robot = 0.0;
  frenet_frame.toFrenet(robot_pose.pose.position.x, robot_pose.pose.position.y, robot_pose.pose.position.z,
                        s_robot, l_robot, z_proj, &ref_yaw_robot);
  s_robot = std::max(0.0, s_robot);

  double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);
  double yaw_robot_rel = angles::shortest_angular_distance(ref_yaw_robot, robot_yaw);

  // 6.0 原地对齐检查：当机体朝向与路径切向偏差过大（> 40度）时，优先纯原地回正
  // 四足底盘原地自转对齐，避免在通道内倒车或打横侧滑漂移
  if (std::abs(yaw_robot_rel) > 0.70) // > ~40 degrees
  {
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    // -yaw_robot_rel = ref_yaw_robot - robot_yaw: 驱动机器人直接转向路径切线
    cmd_vel.angular.z = std::copysign(std::min(cfg_.robot.max_vel_theta, 0.75), -yaw_robot_rel);
    return true;
  }

  // 6.1 确定 Frenet 空间规划终点 (s_goal, l_goal = 0, yaw_goal_rel)
  double s_goal = frenet_frame.totalLength();
  double l_goal = 0.0;
  double yaw_goal_rel = 0.0;

  double dist_to_global_goal = std::hypot(
      robot_pose.pose.position.x - global_plan_.back().pose.position.x,
      robot_pose.pose.position.y - global_plan_.back().pose.position.y);
  if (dist_to_global_goal <= s_goal + 0.10)
  {
    double final_goal_yaw = tf2::getYaw(global_plan_.back().pose.orientation);
    yaw_goal_rel = angles::shortest_angular_distance(frenet_frame.getTangentYaw(s_goal), final_goal_yaw);
  }

  // 6.2 在 Frenet 空间生成初始种子轨迹 (平滑连接当前位姿与局部前瞻终点)
  double plan_dist = std::max(0.20, s_goal - s_robot);
  size_t n_samples = std::max(size_t(5), static_cast<size_t>(std::ceil(plan_dist / 0.15)));
  std::vector<geometry_msgs::PoseStamped> frenet_initial_plan;
  frenet_initial_plan.reserve(n_samples);

  ros::Time now = ros::Time::now();
  for (size_t i = 0; i < n_samples; ++i)
  {
    double t = static_cast<double>(i) / (n_samples - 1);
    double s = s_robot + t * (s_goal - s_robot);
    double l = (1.0 - t) * l_robot + t * l_goal;
    double theta = (1.0 - t) * yaw_robot_rel + t * yaw_goal_rel;

    geometry_msgs::PoseStamped p;
    p.header.frame_id = global_frame_;
    p.header.stamp = now;
    p.pose.position.x = s;
    p.pose.position.y = l;
    p.pose.position.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, theta);
    p.pose.orientation = tf2::toMsg(q);
    frenet_initial_plan.push_back(p);
  }

  // 7. 装载 Frenet 空间障碍物 (走廊左右 LineObstacle 边界 + 踏面投影点障碍物)
  populateFrenetObstacles(frenet_frame, s_robot, s_goal, l_robot);

  // 8. 计算 Frenet 空间初始速度
  double v_s_start = robot_vel_.linear.x * std::cos(yaw_robot_rel) - robot_vel_.linear.y * std::sin(yaw_robot_rel);
  double v_l_start = robot_vel_.linear.x * std::sin(yaw_robot_rel) + robot_vel_.linear.y * std::cos(yaw_robot_rel);
  double kappa_0 = frenet_frame.getCurvature(s_robot);
  double denom_0 = std::max(0.2, 1.0 - kappa_0 * l_robot);
  double omega_rel_start = robot_vel_.angular.z - (kappa_0 / denom_0) * v_s_start;

  geometry_msgs::Twist start_vel_frenet;
  start_vel_frenet.linear.x = v_s_start;
  start_vel_frenet.linear.y = v_l_start;
  start_vel_frenet.angular.z = omega_rel_start;

  // 9. 在纯净无折叠的 Frenet (s, l) 空间执行 TEB 弹性带时空优化 (零 via-points, 彻底杜绝与避障死锁)
  bool success = planner_->plan(frenet_initial_plan, &start_vel_frenet, cfg_.goal_tolerance.free_goal_vel);
  if (!success)
  {
    planner_->clearPlanner();
    ROS_WARN_THROTTLE(1.0, "[Teb3DLocalPlanner] TEB optimization failed in Frenet space.");
    return false;
  }

  // 10. 提取最优轨迹
  teb_local_planner::TebOptimalPlannerPtr selected_teb = nullptr;
  auto hcp = boost::dynamic_pointer_cast<teb_local_planner::HomotopyClassPlanner>(planner_);
  if (hcp && !hcp->getTrajectoryContainer().empty())
  {
    selected_teb = hcp->bestTeb();
  }
  else
  {
    auto single_teb = boost::dynamic_pointer_cast<teb_local_planner::TebOptimalPlanner>(planner_);
    if (single_teb && single_teb->isOptimized())
    {
      selected_teb = single_teb;
    }
  }

  if (!selected_teb)
  {
    ROS_WARN_THROTTLE(1.0, "[Teb3DLocalPlanner] TEB failed to produce a valid trajectory.");
    planner_->clearPlanner();
    return false;
  }

  // 11. 提取速度指令并叠加解析曲率前馈
  // 注意：TEB 内部 extractVelocity 已根据起点位姿 Pose(0).theta 自动投影到机体坐标系，
  // getVelocityCommand 输出的 cmd_vx, cmd_vy 已经是机器狗机体坐标系下的速度指令，绝不可再做二次旋转！
  double cmd_vx = 0.0, cmd_vy = 0.0, cmd_omega_rel = 0.0;
  selected_teb->getVelocityCommand(cmd_vx, cmd_vy, cmd_omega_rel,
                                   cfg_.trajectory.control_look_ahead_poses);

  // 计算沿参考切线的真实前进分速度以合成曲率前馈
  double v_s_eff = cmd_vx * std::cos(yaw_robot_rel) - cmd_vy * std::sin(yaw_robot_rel);
  double omega_z = cmd_omega_rel + (kappa_0 / denom_0) * v_s_eff;

  // 动力学限幅保护
  cmd_vel.linear.x = std::max(-cfg_.robot.max_vel_x_backwards, std::min(cfg_.robot.max_vel_x, cmd_vx));
  cmd_vel.linear.y = std::max(-cfg_.robot.max_vel_y, std::min(cfg_.robot.max_vel_y, cmd_vy));
  cmd_vel.angular.z = std::max(-cfg_.robot.max_vel_theta, std::min(cfg_.robot.max_vel_theta, omega_z));

  // 12. 将优化后的 Frenet 轨迹严格映射回三维全局坐标系 (具备连续真实 Z 与姿态)
  if (visualization_)
  {
    std::vector<geometry_msgs::PoseStamped> local_3d_plan;
    const auto& poses = selected_teb->teb().poses();
    local_3d_plan.reserve(poses.size());

    for (const auto& p : poses)
    {
      double s_i = p->x();
      double l_i = p->y();
      double theta_rel_i = p->theta();

      double wx = 0.0, wy = 0.0, wz = 0.0, ref_yaw_i = 0.0;
      frenet_frame.toCartesian(s_i, l_i, wx, wy, wz, ref_yaw_i);
      double ground_z = querySurfaceZ(wx, wy, wz, max_step_height_);

      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = global_frame_;
      pose.header.stamp = now;
      pose.pose.position.x = wx;
      pose.pose.position.y = wy;
      pose.pose.position.z = ground_z;

      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, angles::normalize_angle(ref_yaw_i + theta_rel_i));
      pose.pose.orientation = tf2::toMsg(q);
      local_3d_plan.push_back(pose);
    }

    visualization_->publishLocalPlan(local_3d_plan);
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

  // 3D 高程高度差检查 (防止跨楼层/上下重叠层误判到达)
  double dz = std::abs(robot_pose.pose.position.z - goal.pose.position.z);
  if (dz > std::max(max_step_height_, 0.35))
  {
    return false;
  }

  double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);
  double goal_yaw = tf2::getYaw(goal.pose.orientation);
  double yaw_diff = std::abs(angles::shortest_angular_distance(robot_yaw, goal_yaw));

  return (yaw_diff <= cfg_.goal_tolerance.yaw_goal_tolerance);
}

} // namespace elevation_local_planner
