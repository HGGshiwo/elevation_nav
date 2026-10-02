#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_core/base_global_planner.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <pluginlib/class_list_macros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <yaml-cpp/yaml.h>
#include <string>
#include <cmath>

#include "elevation_planner_core/cloud_graph_builder.hpp"
#include "elevation_planner_core/graph_store.hpp"
#include "elevation_planner_core/pcd_map_io.hpp"
#include "elevation_planner_core/topological_corridor.hpp"
#include "elevation_planner_core/DiagnoseQuery.h"
#include "elevation_global_planner/manifold_astar.hpp"
#include "elevation_global_planner/path_smoother.hpp"

namespace elevation_global_planner
{

/**
 * @brief move_base 全局规划器插件: 离线 PCD -> 多层流形拓扑图 -> 跨层 A* + 平滑。
 *
 * 全局图保持纯先验 (不订阅实时点云, 实时观测由局部插件的融合图消化);
 * 建图产物 (先验柱表 + 全局图) 写入进程级 GraphStore, 供同进程的局部插件
 * 做 ROI 逐柱融合。诊断查询/可视化/PCD 热切换话题自旧独立节点原样迁移。
 */
class ElevationGlobalPlanner : public nav_core::BaseGlobalPlanner
{
public:
  ElevationGlobalPlanner() = default;
  ~ElevationGlobalPlanner() override = default;

  void initialize(std::string name, costmap_2d::Costmap2DROS * /*costmap_ros*/) override
  {
    if (initialized_) {
      ROS_WARN("ElevationGlobalPlanner has already been initialized, doing nothing.");
      return;
    }

    ros::NodeHandle private_nh("~/" + name);
    private_nh.param<std::string>("map_frame", map_frame_, "map");
    private_nh.param<std::string>("map_config_file", map_config_file_, "");

    elevation_planner::GraphBuildConfig cfg;
    private_nh.param<double>("resolution", cfg.resolution, 0.10);
    private_nh.param<double>("max_step_height", cfg.max_step_height, 0.25);
    private_nh.param<double>("max_stride_length", cfg.max_stride_length, 0.35);
    private_nh.param<double>("dog_height", cfg.dog_height, 0.45);
    private_nh.param<double>("footprint_radius", cfg.footprint_radius, 0.26);
    private_nh.param<double>("body_hard_radius", cfg.body_hard_radius, 0.17);
    private_nh.param<double>("inflation_radius", cfg.inflation_radius, 0.50);
    private_nh.param<double>("sweep_penalty_weight", cfg.sweep_penalty_weight, 1.0);
    private_nh.param<bool>("body_hard_ring_enabled", cfg.body_hard_ring_enabled, true);
    private_nh.param<int>("sor_mean_k", cfg.sor_mean_k, 16);
    private_nh.param<double>("sor_std_mul", cfg.sor_std_mul, 1.5);
    private_nh.param<double>("cluster_height_diff", cfg.cluster_height_diff, 0.08);
    private_nh.param<int>("min_cluster_points", cfg.min_cluster_points, 2);

    private_nh.param<double>("corridor_radius", corridor_radius_, 3.0);
    private_nh.param<double>("corridor_lookahead", corridor_lookahead_, 5.0);
    max_step_height_ = cfg.max_step_height;
    dog_height_ = cfg.dog_height;

    private_nh.param<bool>("enable_homotopy_filter", enable_homotopy_filter_, true);
    private_nh.param<double>("hysteresis_ratio", hysteresis_ratio_, 0.20);
    private_nh.param<double>("min_side_offset", min_side_offset_, 0.20);
    ROS_INFO("[ElevationGlobalPlanner] Homotopy filter: %s (hysteresis=%.2f, min_side_offset=%.2fm)",
             enable_homotopy_filter_ ? "enabled" : "disabled", hysteresis_ratio_, min_side_offset_);

    bool los_prune_enabled = true;
    double los_max_segment = 0.60;
    private_nh.param<bool>("los_prune_enabled", los_prune_enabled, true);
    private_nh.param<double>("los_max_segment", los_max_segment, 0.60);
    planner_.setLosPruning(los_prune_enabled, los_max_segment);
    ROS_INFO("[ElevationGlobalPlanner] SC-LOS pruning: %s, max_segment=%.2fm",
             los_prune_enabled ? "enabled" : "disabled", los_max_segment);

    double robot_length = 0.0, robot_width = 0.0, margin = 0.04;
    private_nh.param<double>("obstacle_safety_margin", margin, 0.04);
    if (private_nh.getParam("robot_width", robot_width) && robot_width > 0.0) {
      cfg.body_hard_radius = robot_width * 0.5 + margin;
      if (private_nh.getParam("robot_length", robot_length) && robot_length > 0.0) {
        cfg.footprint_radius = std::hypot(robot_length * 0.5, robot_width * 0.5) + margin;
      }
      ROS_INFO("[ElevationGlobalPlanner] Unified robot geometry: body_hard_radius=%.3fm, footprint_radius=%.3fm (W=%.2f, L=%.2f, margin=%.2f)",
               cfg.body_hard_radius, cfg.footprint_radius, robot_width, robot_length, margin);
    }
    builder_.setConfig(cfg);

    ros::NodeHandle nh;
    path_pub_ = nh.advertise<nav_msgs::Path>("/elevation_global_plan", 1, true);
    dense_path_pub_ = nh.advertise<nav_msgs::Path>("/elevation_global_plan_dense", 1, true);
    corridor_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/elevation_corridor_nodes", 1, true);
    corridor_boundary_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/elevation_global_corridor_boundaries", 1, true);
    nodes_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/elevation_graph_nodes", 1, true);
    edges_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/elevation_graph_edges", 1, true);
    debug_result_pub_ = nh.advertise<std_msgs::String>("/elevation_debug_result", 5, true);

    pcd_cmd_sub_ = nh.subscribe("/pcd_file_cmd", 1, &ElevationGlobalPlanner::onPcdCmd, this);
    debug_query_sub_ = nh.subscribe("/elevation_debug_query", 5, &ElevationGlobalPlanner::onDebugQuery, this);
    debug_srv_ = nh.advertiseService("/elevation_debug_diagnose", &ElevationGlobalPlanner::onDiagnoseService, this);

    std::string pcd_file;
    private_nh.param<std::string>("pcd_file", pcd_file, "");
    if (!pcd_file.empty()) {
      loadMap(pcd_file, map_config_file_);
    }

    initialized_ = true;
    ROS_INFO("[ElevationGlobalPlanner] Manifold global planner plugin is ready (corridor_radius=%.2fm, corridor_lookahead=%.2fm)",
             corridor_radius_, corridor_lookahead_);
  }

  bool makePlan(const geometry_msgs::PoseStamped & start,
                const geometry_msgs::PoseStamped & goal,
                std::vector<geometry_msgs::PoseStamped> & plan) override
  {
    if (!has_map_) {
      ROS_WARN_THROTTLE(2.0, "[ElevationGlobalPlanner] No map data yet, cannot plan");
      return false;
    }

    ros::Time t0 = ros::Time::now();

    // 接入融合活图 (零拷贝): 融合引擎原位刷新的是 GraphStore 那份 (动态禁行/膨胀
    // 叠加层), 插件静态副本永远看不到 —— 直接绑定活图指针, 规划在活图上进行。
    // 融合 worker 线程同时写节点属性为已接受的良性竞争 (对齐 float 单字原子性)
    const auto live = elevation_planner::GraphStore::instance().getGlobalGraph();
    const elevation_planner::ManifoldGraph & plan_graph =
        (live && live->numNodes() > 0) ? *live : graph_;
    planner_.initialize(plan_graph);

    nav_msgs::Path raw_path;
    raw_path.header.frame_id = map_frame_;
    if (!planner_.plan(start, goal, raw_path)) {
      ROS_WARN("[ElevationGlobalPlanner] Plan failed: no valid path between start (%.2f, %.2f, %.2f) and goal (%.2f, %.2f, %.2f)",
               start.pose.position.x, start.pose.position.y, start.pose.position.z,
               goal.pose.position.x, goal.pose.position.y, goal.pose.position.z);
      last_raw_path_.poses.clear();
      last_goal_set_ = false;
      return false;
    }
    const double t_astar = (ros::Time::now() - t0).toSec() * 1e3;

    // 拓扑同伦侧向判定与滞后切换过滤 (抑制动态/移动障碍物导致的左右高频横跳)
    if (enable_homotopy_filter_ && !last_raw_path_.poses.empty() && last_goal_set_) {
      double d_goal = std::hypot(goal.pose.position.x - last_goal_.pose.position.x,
                                 goal.pose.position.y - last_goal_.pose.position.y);
      if (d_goal < 0.5) {
        double side_old = computeDominantSide(last_raw_path_, start.pose.position, goal.pose.position);
        double side_new = computeDominantSide(raw_path, start.pose.position, goal.pose.position);

        // 两路径属于中轴线两侧 (一正一负) 且偏离幅度均超过阈值 (确认为不同绕行分支)
        if (side_old * side_new < 0.0 &&
            std::abs(side_old) >= min_side_offset_ &&
            std::abs(side_new) >= min_side_offset_)
        {
          nav_msgs::Path old_spliced;
          double old_cost = 0.0;
          if (evaluateOldPath(last_raw_path_, start, plan_graph, old_spliced, old_cost)) {
            double new_cost = computePathCost(raw_path, plan_graph);
            // 只有当新分支代价明显优于旧分支 ((1 - hysteresis_ratio) * old_cost) 时才允许切换侧向;
            // 否则保留上一帧同侧路径，彻底消除移动障碍物导致的心跳式左右横跳
            if (new_cost >= (1.0 - hysteresis_ratio_) * old_cost) {
              ROS_INFO("[ElevationGlobalPlanner] Homotopy filter: suppressed side flip (old_side=%.2fm, new_side=%.2fm, cost_old=%.2f, cost_new=%.2f), retaining previous side",
                       side_old, side_new, old_cost, new_cost);
              raw_path = old_spliced;
            } else {
              ROS_INFO("[ElevationGlobalPlanner] Homotopy filter: switched side (new path cost %.2f is significantly better than old %.2f)",
                       new_cost, old_cost);
            }
          }
        }
      }
    }

    last_raw_path_ = raw_path;
    last_goal_ = goal;
    last_goal_set_ = true;

    // Check dynamic obstacle intersections on raw path
    size_t raw_dyn_hits = 0;
    for (const auto & ps : raw_path.poses) {
      uint32_t nid = static_cast<uint32_t>(std::max(0.0, std::round(ps.pose.orientation.x)));
      if (nid < plan_graph.numNodes()) {
        const auto & nd = plan_graph.getNode(nid);
        if (nd.dynamic_zone != 0 || nd.dynamic_trav > 0.0f) {
          ++raw_dyn_hits;
        }
      }
    }

    ros::Time t1 = ros::Time::now();
    nav_msgs::Path smoothed_path;
    smoothed_path.header.frame_id = map_frame_;
    smoother_.smooth(raw_path, plan_graph, smoothed_path);
    plan = smoothed_path.poses;
    const double t_smooth = (ros::Time::now() - t1).toSec() * 1e3;
    ROS_INFO("[Perf][global] astar=%.1fms smooth=%.1fms total=%.1fms raw_poses=%zu (dyn_hits=%zu) smoothed_poses=%zu",
             t_astar, t_smooth, t_astar + t_smooth, raw_path.poses.size(), raw_dyn_hits, plan.size());

    // 朝向沿路径方向重算: A* 输出的单位四元数 (yaw=0) 会被 TEB
    // (global_plan_overwrite_orientation=false) 当作各航点的目标航向,
    // 行进方向与 yaw=0 夹角大时 TEB 优化出倒车进入目标的轨迹
    // (max_vel_x_backwards 通道), 机器狗表现为倒着走。
    // 按相邻航点差分计算行进方向并写入 yaw, 机头始终朝向前方。
    for (size_t i = 0; i < plan.size(); ++i)
    {
      const size_t i0 = (i > 0) ? i - 1 : i;
      const size_t i1 = (i + 1 < plan.size()) ? i + 1 : i;
      const double dx = plan[i1].pose.position.x - plan[i0].pose.position.x;
      const double dy = plan[i1].pose.position.y - plan[i0].pose.position.y;
      double yaw = 0.0;
      if (std::hypot(dx, dy) >= 1e-4) yaw = std::atan2(dy, dx);
      // orientation.x 透传 node_id 给 move_base 局部规划器 (方案 B)
      plan[i].pose.orientation.x = (i < raw_path.poses.size()) ? raw_path.poses[i].pose.orientation.x : 0.0;
      plan[i].pose.orientation.y = 0.0;
      plan[i].pose.orientation.z = std::sin(yaw * 0.5);
      plan[i].pose.orientation.w = std::cos(yaw * 0.5);
    }

    // 扩展 A* 规划结果为局部前瞻拓扑流形管道 (供 TEB 局部规划器在管道内部避障与同伦优化)
    auto corridor = std::make_shared<elevation_planner::TopologicalCorridor>(
        elevation_planner::TopologicalCorridorGenerator::generate(
            plan_graph, plan, corridor_radius_, corridor_lookahead_, max_step_height_, dog_height_));
    elevation_planner::GraphStore::instance().setTopologicalCorridor(corridor);
    ROS_INFO("[ElevationGlobalPlanner] Initial topological corridor generated: %zu nodes within radius %.2fm, lookahead %.2fm",
             corridor->size(), corridor_radius_, corridor_lookahead_);
    publishCorridorVisuals(*corridor);

    // latched 兼容发布: Web 端与旧消费者订阅 /elevation_global_plan
    smoothed_path.header.stamp = ros::Time::now();
    path_pub_.publish(smoothed_path);

    // 剪枝前的原始密集 A* 路径 (调试对比用)
    if (dense_path_pub_.getNumSubscribers() > 0) {
      nav_msgs::Path dense = planner_.getLastDensePath();
      dense.header.stamp = ros::Time::now();
      dense_path_pub_.publish(dense);
    }

    double plan_time = (ros::Time::now() - t0).toSec() * 1000.0;
    ROS_INFO("[ElevationGlobalPlanner] Global plan success: time = %.2f ms, points = %zu",
             plan_time, plan.size());
    return true;
  }

private:
  void loadMap(const std::string & path, const std::string & override_map_config = "")
  {
    std::string active_map_config = override_map_config.empty() ? map_config_file_ : override_map_config;

    auto cloud = elevation_planner::loadAndCropPcd(path, active_map_config);
    if (!cloud) {
      ROS_ERROR("[ElevationGlobalPlanner] Failed to load PCD file: %s", path.c_str());
      return;
    }

    // 覆盖流形拓扑图构建参数 (抗空洞: min_cluster_points, sor_mean_k)
    if (!active_map_config.empty()) {
      try {
        YAML::Node root = YAML::LoadFile(active_map_config);
        if (root["manifold_graph"]) {
          auto mg = root["manifold_graph"];
          auto cfg = builder_.getConfig();
          if (mg["sor_mean_k"]) cfg.sor_mean_k = mg["sor_mean_k"].as<int>();
          if (mg["min_cluster_points"]) cfg.min_cluster_points = mg["min_cluster_points"].as<int>();
          builder_.setConfig(cfg);
          ROS_INFO("[ElevationGlobalPlanner] Overrode manifold_graph params: sor_mean_k=%d, min_cluster_points=%d",
                   cfg.sor_mean_k, cfg.min_cluster_points);
        }
      } catch (...) {}
    }

    ROS_INFO("[ElevationGlobalPlanner] Building manifold graph from %zu points...", cloud->size());

    ros::Time t0 = ros::Time::now();
    auto table = std::make_shared<elevation_planner::ColumnTable>();
    if (!builder_.buildColumnTable(cloud, *table)) {
      ROS_ERROR("[ElevationGlobalPlanner] Failed to build column table from PCD");
      return;
    }

    if (!builder_.buildGraphFromColumnTable(*table, graph_)) {
      ROS_ERROR("[ElevationGlobalPlanner] Failed to build manifold graph from column table");
      return;
    }
    planner_.initialize(graph_);

    // 先验柱表与全局图入进程级共享存储, 供局部插件融合
    elevation_planner::GraphStore::instance().setGlobalTable(table);
    auto shared_graph = std::make_shared<elevation_planner::ManifoldGraph>(graph_);
    elevation_planner::GraphStore::instance().setGlobalGraph(shared_graph);

    has_map_ = true;
    double elapsed = (ros::Time::now() - t0).toSec();
    ROS_INFO("[ElevationGlobalPlanner] Graph built: %zu nodes, %zu edges, time: %.3f s",
             graph_.numNodes(), graph_.numEdges(), elapsed);
    publishGraphVisuals();
  }

  void publishGraphVisuals()
  {
    if (!has_map_) return;
    sensor_msgs::PointCloud2 cloud_msg;
    elevation_planner::CloudGraphBuilder::toPointCloudMsg(graph_, map_frame_, cloud_msg);
    nodes_pub_.publish(cloud_msg);

    visualization_msgs::MarkerArray marker_msg;
    elevation_planner::CloudGraphBuilder::toMarkerArray(graph_, map_frame_, marker_msg);
    edges_pub_.publish(marker_msg);
  }

  void onPcdCmd(const std_msgs::String::ConstPtr & msg)
  {
    if (msg && !msg->data.empty()) {
      std::string pcd_path = msg->data;
      std::string custom_config = "";
      size_t sep = pcd_path.find(';');
      if (sep != std::string::npos) {
        custom_config = pcd_path.substr(sep + 1);
        pcd_path = pcd_path.substr(0, sep);
      }
      ROS_INFO("[ElevationGlobalPlanner] Received PCD switch command: %s (map_config: %s)",
               pcd_path.c_str(), custom_config.empty() ? "(default)" : custom_config.c_str());
      loadMap(pcd_path, custom_config);
    }
  }

  bool onDiagnoseService(elevation_planner_core::DiagnoseQuery::Request & req,
                         elevation_planner_core::DiagnoseQuery::Response & res)
  {
    if (!has_map_) {
      res.result = "{\"status\":\"error\",\"message\":\"地图未就绪\"}";
      return true;
    }
    const std::string & s = req.query;
    auto getVal = [&](const std::string & k) -> double {
      size_t pos = s.find("\"" + k + "\"");
      if (pos == std::string::npos) return 0.0;
      size_t colon = s.find(":", pos);
      if (colon == std::string::npos) return 0.0;
      return std::strtod(s.c_str() + colon + 1, nullptr);
    };
    const auto live = elevation_planner::GraphStore::instance().getGlobalGraph();
    const elevation_planner::ManifoldGraph & dbg_graph =
        (live && live->numNodes() > 0) ? *live : graph_;

    size_t mode_pos = s.find("\"mode\"");
    bool node_mode = false;
    if (mode_pos != std::string::npos) {
      size_t vpos = s.find("node", mode_pos);
      node_mode = (vpos != std::string::npos) && (vpos - mode_pos < 20);
    }
    if (node_mode) {
      double x1 = getVal("x1"), y1 = getVal("y1"), z1 = getVal("z1");
      res.result = builder_.diagnoseNode(dbg_graph, x1, y1, z1);
    } else {
      double x1 = getVal("x1"), y1 = getVal("y1"), z1 = getVal("z1");
      double x2 = getVal("x2"), y2 = getVal("y2"), z2 = getVal("z2");
      res.result = builder_.diagnoseEdge(dbg_graph, x1, y1, z1, x2, y2, z2);
    }
    return true;
  }

  void onDebugQuery(const std_msgs::String::ConstPtr & msg)
  {
    if (!msg || msg->data.empty()) return;
    if (!has_map_) return;
    elevation_planner_core::DiagnoseQuery::Request req;
    elevation_planner_core::DiagnoseQuery::Response res;
    req.query = msg->data;
    onDiagnoseService(req, res);
    std_msgs::String out_msg;
    out_msg.data = res.result;
    debug_result_pub_.publish(out_msg);
  }

  void publishCorridorVisuals(const elevation_planner::TopologicalCorridor & corridor)
  {
    if (corridor.empty() || !has_map_) return;

    sensor_msgs::PointCloud2 cloud_msg;
    elevation_planner::TopologicalCorridorGenerator::toPointCloudMsg(
        graph_, corridor, map_frame_, cloud_msg);
    corridor_pub_.publish(cloud_msg);

    visualization_msgs::MarkerArray marker_msg;
    elevation_planner::TopologicalCorridorGenerator::toBoundaryMarkers(
        graph_, corridor, map_frame_, marker_msg, dog_height_, max_step_height_);
    corridor_boundary_pub_.publish(marker_msg);
  }

  double computeDominantSide(const nav_msgs::Path & path,
                             const geometry_msgs::Point & start,
                             const geometry_msgs::Point & goal) const
  {
    if (path.poses.empty()) return 0.0;
    double vx = goal.x - start.x;
    double vy = goal.y - start.y;
    double L = std::hypot(vx, vy);
    if (L < 0.2) return 0.0;

    double max_left = 0.0;   // 正叉积 (左侧)
    double max_right = 0.0;  // 负叉积 (右侧, 存正模长)

    for (const auto & ps : path.poses) {
      double dx = ps.pose.position.x - start.x;
      double dy = ps.pose.position.y - start.y;
      // 2D 叉积: vx * dy - vy * dx (沿 start->goal 方向: >0 为左, <0 为右)
      double cross = (vx * dy - vy * dx) / L;
      if (cross > max_left) {
        max_left = cross;
      } else if (-cross > max_right) {
        max_right = -cross;
      }
    }

    return (max_left >= max_right) ? max_left : -max_right;
  }

  double computePathCost(const nav_msgs::Path & path, const elevation_planner::ManifoldGraph & graph) const
  {
    if (path.poses.size() < 2) return 0.0;
    double total_cost = 0.0;
    for (size_t i = 1; i < path.poses.size(); ++i) {
      const auto & p0 = path.poses[i - 1].pose.position;
      const auto & p1 = path.poses[i].pose.position;
      double dxy = std::hypot(p1.x - p0.x, p1.y - p0.y);
      double dz = std::abs(p1.z - p0.z);
      double d3 = std::sqrt(dxy * dxy + dz * dz);

      uint32_t nid = static_cast<uint32_t>(std::max(0.0, std::round(path.poses[i].pose.orientation.x)));
      float trav = 0.0f;
      if (nid < graph.numNodes()) {
        const auto & nd = graph.getNode(nid);
        trav = nd.traversability + nd.dynamic_trav;
      }
      total_cost += d3 + 2.0 * dz + 3.0 * trav;
    }
    return total_cost;
  }

  bool evaluateOldPath(const nav_msgs::Path & old_raw_path,
                       const geometry_msgs::PoseStamped & start,
                       const elevation_planner::ManifoldGraph & graph,
                       nav_msgs::Path & out_spliced,
                       double & out_cost) const
  {
    if (old_raw_path.poses.size() < 2) return false;

    // 查找起点在旧路径上的最近投影点
    size_t closest_idx = 0;
    double min_dist_sq = std::numeric_limits<double>::max();
    for (size_t i = 0; i < old_raw_path.poses.size(); ++i) {
      double dx = old_raw_path.poses[i].pose.position.x - start.pose.position.x;
      double dy = old_raw_path.poses[i].pose.position.y - start.pose.position.y;
      double dz = old_raw_path.poses[i].pose.position.z - start.pose.position.z;
      double d2 = dx * dx + dy * dy + dz * dz;
      if (d2 < min_dist_sq) {
        min_dist_sq = d2;
        closest_idx = i;
      }
    }

    // 若机器人已偏离旧路径过远 (> 1.2m)，放弃复用
    if (std::sqrt(min_dist_sq) > 1.2) {
      return false;
    }

    // 若起点离旧路径终点太近，无需复用
    if (closest_idx >= old_raw_path.poses.size() - 1) {
      return false;
    }

    out_spliced.header = old_raw_path.header;
    out_spliced.poses.clear();

    // 放入当前起点
    geometry_msgs::PoseStamped start_ps = start;
    start_ps.pose.orientation.x = old_raw_path.poses[closest_idx].pose.orientation.x;
    out_spliced.poses.push_back(start_ps);

    // 检查从 closest_idx 到终点的每一个节点是否在活图上有效 (无碰撞、无硬阻挡)
    for (size_t i = closest_idx; i < old_raw_path.poses.size(); ++i) {
      const auto & ps = old_raw_path.poses[i];
      uint32_t nid = static_cast<uint32_t>(std::max(0.0, std::round(ps.pose.orientation.x)));
      if (nid >= graph.numNodes()) {
        return false;
      }
      const auto & nd = graph.getNode(nid);
      // 检查硬阻挡、动态硬障碍区、动态极高阻尼
      if (nd.hardBlocked() || nd.dynamic_zone == 1 || nd.dynamic_trav > 0.8f) {
        return false;
      }
      if (i == closest_idx) {
        double d_start = std::hypot(ps.pose.position.x - start.pose.position.x,
                                    ps.pose.position.y - start.pose.position.y);
        if (d_start < 0.15) {
          continue;
        }
      }
      out_spliced.poses.push_back(ps);
    }

    if (out_spliced.poses.size() < 2) return false;

    out_cost = computePathCost(out_spliced, graph);
    return true;
  }

  std::string map_frame_;
  std::string map_config_file_;
  bool initialized_{false};
  bool has_map_{false};
  double corridor_radius_{3.0};
  double corridor_lookahead_{5.0};
  double max_step_height_{0.25};
  double dog_height_{0.45};

  bool enable_homotopy_filter_{true};
  double hysteresis_ratio_{0.20};
  double min_side_offset_{0.20};
  bool last_goal_set_{false};
  nav_msgs::Path last_raw_path_;
  geometry_msgs::PoseStamped last_goal_;

  elevation_planner::CloudGraphBuilder builder_;
  elevation_planner::ManifoldGraph graph_;
  ManifoldAStarPlanner planner_;
  PathSmoother smoother_;

  ros::Publisher path_pub_;
  ros::Publisher dense_path_pub_;
  ros::Publisher corridor_pub_;
  ros::Publisher corridor_boundary_pub_;
  ros::Publisher nodes_pub_;
  ros::Publisher edges_pub_;
  ros::Publisher debug_result_pub_;
  ros::Subscriber pcd_cmd_sub_;
  ros::Subscriber debug_query_sub_;
  ros::ServiceServer debug_srv_;
};

} // namespace elevation_global_planner

PLUGINLIB_EXPORT_CLASS(elevation_global_planner::ElevationGlobalPlanner, nav_core::BaseGlobalPlanner)
