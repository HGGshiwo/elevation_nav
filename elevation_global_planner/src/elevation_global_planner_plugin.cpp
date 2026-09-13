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
#include <string>

#include "elevation_planner_core/cloud_graph_builder.hpp"
#include "elevation_planner_core/graph_store.hpp"
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

    elevation_planner::GraphBuildConfig cfg;
    private_nh.param<double>("resolution", cfg.resolution, 0.10);
    private_nh.param<double>("max_step_height", cfg.max_step_height, 0.25);
    private_nh.param<double>("max_stride_length", cfg.max_stride_length, 0.35);
    private_nh.param<double>("dog_height", cfg.dog_height, 0.45);
    private_nh.param<double>("footprint_radius", cfg.footprint_radius, 0.30);
    private_nh.param<double>("body_hard_radius", cfg.body_hard_radius, 0.20);
    private_nh.param<double>("sweep_penalty_weight", cfg.sweep_penalty_weight, 1.0);
    private_nh.param<double>("foot_clearance", cfg.foot_clearance, 0.05);
    private_nh.param<int>("sor_mean_k", cfg.sor_mean_k, 16);
    private_nh.param<double>("sor_std_mul", cfg.sor_std_mul, 1.5);
    builder_.setConfig(cfg);

    ros::NodeHandle nh;
    path_pub_ = nh.advertise<nav_msgs::Path>("/elevation_global_plan", 1, true);
    nodes_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/elevation_graph_nodes", 1, true);
    edges_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/elevation_graph_edges", 1, true);
    debug_result_pub_ = nh.advertise<std_msgs::String>("/elevation_debug_result", 5, true);

    pcd_cmd_sub_ = nh.subscribe("/pcd_file_cmd", 1, &ElevationGlobalPlanner::onPcdCmd, this);
    debug_query_sub_ = nh.subscribe("/elevation_debug_query", 5, &ElevationGlobalPlanner::onDebugQuery, this);

    std::string pcd_file;
    private_nh.param<std::string>("pcd_file", pcd_file, "");
    if (!pcd_file.empty()) {
      loadMap(pcd_file);
    }

    initialized_ = true;
    ROS_INFO("[ElevationGlobalPlanner] Manifold global planner plugin is ready");
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
    nav_msgs::Path raw_path;
    raw_path.header.frame_id = map_frame_;
    if (!planner_.plan(start, goal, raw_path)) {
      ROS_WARN("[ElevationGlobalPlanner] Plan failed: no valid path between start and goal");
      return false;
    }

    nav_msgs::Path smoothed_path;
    smoothed_path.header.frame_id = map_frame_;
    smoother_.smooth(raw_path, graph_, smoothed_path);
    plan = smoothed_path.poses;

    // latched 兼容发布: Web 端与旧消费者订阅 /elevation_global_plan
    smoothed_path.header.stamp = ros::Time::now();
    path_pub_.publish(smoothed_path);

    double plan_time = (ros::Time::now() - t0).toSec() * 1000.0;
    ROS_INFO("[ElevationGlobalPlanner] Global plan success: time = %.2f ms, points = %zu",
             plan_time, plan.size());
    return true;
  }

private:
  void loadMap(const std::string & path)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(path, *cloud) == -1) {
      ROS_ERROR("[ElevationGlobalPlanner] Failed to read PCD file: %s", path.c_str());
      return;
    }
    ROS_INFO("[ElevationGlobalPlanner] Loaded PCD with %zu points, building manifold graph...", cloud->size());

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
      ROS_INFO("[ElevationGlobalPlanner] Received PCD switch command: %s", msg->data.c_str());
      loadMap(msg->data);
    }
  }

  void onDebugQuery(const std_msgs::String::ConstPtr & msg)
  {
    if (!msg || msg->data.empty()) return;
    if (!has_map_) return;
    std::string s = msg->data;
    auto getVal = [&](const std::string & k) -> double {
      size_t pos = s.find("\"" + k + "\"");
      if (pos == std::string::npos) return 0.0;
      size_t colon = s.find(":", pos);
      if (colon == std::string::npos) return 0.0;
      return std::strtod(s.c_str() + colon + 1, nullptr);
    };
    // 模式分发: {"mode":"node"} 为单节点禁行原因诊断, 其余 (含旧版无 mode) 为两点邻边诊断
    size_t mode_pos = s.find("\"mode\"");
    bool node_mode = false;
    if (mode_pos != std::string::npos) {
      size_t vpos = s.find("node", mode_pos);
      node_mode = (vpos != std::string::npos) && (vpos - mode_pos < 20);
    }
    std::string res;
    if (node_mode) {
      double x1 = getVal("x1"), y1 = getVal("y1"), z1 = getVal("z1");
      res = builder_.diagnoseNode(graph_, x1, y1, z1);
    } else {
      double x1 = getVal("x1"), y1 = getVal("y1"), z1 = getVal("z1");
      double x2 = getVal("x2"), y2 = getVal("y2"), z2 = getVal("z2");
      res = builder_.diagnoseEdge(graph_, x1, y1, z1, x2, y2, z2);
    }
    std_msgs::String out_msg;
    out_msg.data = res;
    debug_result_pub_.publish(out_msg);
  }

  std::string map_frame_;
  bool initialized_{false};
  bool has_map_{false};

  elevation_planner::CloudGraphBuilder builder_;
  elevation_planner::ManifoldGraph graph_;
  ManifoldAStarPlanner planner_;
  PathSmoother smoother_;

  ros::Publisher path_pub_;
  ros::Publisher nodes_pub_;
  ros::Publisher edges_pub_;
  ros::Publisher debug_result_pub_;
  ros::Subscriber pcd_cmd_sub_;
  ros::Subscriber debug_query_sub_;
};

} // namespace elevation_global_planner

PLUGINLIB_EXPORT_CLASS(elevation_global_planner::ElevationGlobalPlanner, nav_core::BaseGlobalPlanner)
