#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2_ros/transform_listener.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "elevation_planner_core/cloud_graph_builder.hpp"
#include "elevation_global_planner/manifold_astar.hpp"
#include "elevation_global_planner/path_smoother.hpp"

class ElevationGlobalPlannerNode
{
public:
  ElevationGlobalPlannerNode(ros::NodeHandle & nh, ros::NodeHandle & pnh)
    : tf_listener_(tf_buffer_)
  {
    pnh.param<std::string>("map_frame", map_frame_, "map");
    pnh.param<std::string>("base_frame", base_frame_, "base_link");
    pnh.param<int>("max_layers", max_layers_, 3);

    elevation_planner::GraphBuildConfig cfg;
    pnh.param<double>("resolution", cfg.resolution, 0.10);
    pnh.param<double>("max_step_height", cfg.max_step_height, 0.25);
    pnh.param<double>("max_stride_length", cfg.max_stride_length, 0.35);
    pnh.param<double>("dog_height", cfg.dog_height, 0.45);
    pnh.param<double>("footprint_radius", cfg.footprint_radius, 0.30);
    pnh.param<double>("body_hard_radius", cfg.body_hard_radius, 0.20);
    pnh.param<double>("sweep_penalty_weight", cfg.sweep_penalty_weight, 1.0);
    pnh.param<double>("foot_clearance", cfg.foot_clearance, 0.05);
    pnh.param<int>("sor_mean_k", cfg.sor_mean_k, 16);
    pnh.param<double>("sor_std_mul", cfg.sor_std_mul, 1.5);
    builder_.setConfig(cfg);

    path_pub_ = nh.advertise<nav_msgs::Path>("/elevation_global_plan", 1, true);
    nodes_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/elevation_graph_nodes", 1, true);
    edges_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/elevation_graph_edges", 1, true);
    debug_result_pub_ = nh.advertise<std_msgs::String>("/elevation_debug_result", 5, true);

    cloud_sub_ = nh.subscribe("/elevation_cloud", 1, &ElevationGlobalPlannerNode::onPointCloud, this);
    goal_sub_ = nh.subscribe("/move_base_simple/goal", 1, &ElevationGlobalPlannerNode::onGoal, this);
    initial_pose_sub_ = nh.subscribe("/initialpose", 1, &ElevationGlobalPlannerNode::onInitialPose, this);
    pcd_cmd_sub_ = nh.subscribe("/pcd_file_cmd", 1, &ElevationGlobalPlannerNode::onPcdCmd, this);
    debug_query_sub_ = nh.subscribe("/elevation_debug_query", 5, &ElevationGlobalPlannerNode::onDebugQuery, this);

    std::string pcd_file;
    pnh.param<std::string>("pcd_file", pcd_file, "");
    if (!pcd_file.empty()) {
      loadPcdDirect(pcd_file);
    }

    ROS_INFO("[ElevationGlobalPlannerNode] Manifold global planner node is ready");
  }

  void loadPcdDirect(const std::string & path)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(path, *cloud) == -1) {
      ROS_ERROR("[ElevationGlobalPlannerNode] Failed to read PCD file: %s", path.c_str());
      return;
    }
    ROS_INFO("[ElevationGlobalPlannerNode] Loaded PCD with %zu points, building manifold graph...", cloud->size());
    ros::Time t0 = ros::Time::now();
    if (builder_.buildFromPointCloud(cloud, graph_)) {
      planner_.initialize(graph_);
      has_map_ = true;
      double elapsed = (ros::Time::now() - t0).toSec();
      ROS_INFO("[ElevationGlobalPlannerNode] Graph built: %zu nodes, %zu edges, time: %.3f s",
               graph_.numNodes(), graph_.numEdges(), elapsed);
      publishGraphVisuals();
      if (has_goal_) triggerPlan();
    }
  }

  void onPointCloud(const sensor_msgs::PointCloud2::ConstPtr & msg)
  {
    ros::Time t0 = ros::Time::now();
    if (builder_.buildFromROSMsg(*msg, graph_)) {
      planner_.initialize(graph_);
      has_map_ = true;
      double elapsed = (ros::Time::now() - t0).toSec();
      ROS_INFO("[ElevationGlobalPlannerNode] Live cloud processed: %zu nodes, %zu edges, time: %.3f s",
               graph_.numNodes(), graph_.numEdges(), elapsed);
      publishGraphVisuals();
      if (has_goal_) triggerPlan();
    }
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
    ROS_INFO("[ElevationGlobalPlannerNode] Published graph visuals to /elevation_graph_nodes and /elevation_graph_edges");
  }

  void onInitialPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr & msg)
  {
    manual_start_ = msg->pose.pose;
    has_manual_start_ = true;
    ROS_INFO("[ElevationGlobalPlannerNode] Set start pose: (%.2f, %.2f, %.2f)",
             manual_start_.position.x, manual_start_.position.y, manual_start_.position.z);
    if (has_goal_) {
      triggerPlan();
    }
  }

  void onGoal(const geometry_msgs::PoseStamped::ConstPtr & goal_msg)
  {
    current_goal_ = *goal_msg;
    has_goal_ = true;
    ROS_INFO("[ElevationGlobalPlannerNode] Set goal pose: (%.2f, %.2f, %.2f)",
             current_goal_.pose.position.x, current_goal_.pose.position.y, current_goal_.pose.position.z);
    triggerPlan();
  }

  void triggerPlan()
  {
    if (!has_map_) {
      ROS_WARN("[ElevationGlobalPlannerNode] No map data yet, cannot plan");
      return;
    }
    if (!has_goal_) return;

    geometry_msgs::PoseStamped start;
    start.header.frame_id = map_frame_;
    start.header.stamp = ros::Time::now();

    bool got_tf = false;
    try {
      geometry_msgs::TransformStamped tf_stamped =
        tf_buffer_.lookupTransform(map_frame_, base_frame_, ros::Time(0), ros::Duration(0.05));
      start.pose.position.x = tf_stamped.transform.translation.x;
      start.pose.position.y = tf_stamped.transform.translation.y;
      start.pose.position.z = tf_stamped.transform.translation.z;
      start.pose.orientation = tf_stamped.transform.rotation;
      got_tf = true;
    } catch (const tf2::TransformException &) {
      got_tf = false;
    }

    if (!got_tf) {
      if (has_manual_start_) {
        start.pose = manual_start_;
        ROS_INFO("[ElevationGlobalPlannerNode] Using manual start: (%.2f, %.2f, %.2f)",
                 start.pose.position.x, start.pose.position.y, start.pose.position.z);
      } else {
        ROS_WARN("[ElevationGlobalPlannerNode] No robot TF and no start set. Please click 'Set Start' in Web UI");
        return;
      }
    }

    ros::Time t0 = ros::Time::now();
    nav_msgs::Path raw_path;
    if (planner_.plan(start, current_goal_, raw_path)) {
      nav_msgs::Path smoothed_path;
      smoother_.smooth(raw_path, graph_, smoothed_path);
      path_pub_.publish(smoothed_path);
      double plan_time = (ros::Time::now() - t0).toSec() * 1000.0;
      ROS_INFO("[ElevationGlobalPlannerNode] Global plan success: time = %.2f ms, points = %zu",
               plan_time, smoothed_path.poses.size());
    } else {
      ROS_WARN("[ElevationGlobalPlannerNode] Plan failed: no valid path between start and goal");
    }
  }

  void onPcdCmd(const std_msgs::String::ConstPtr & msg)
  {
    if (msg && !msg->data.empty()) {
      ROS_INFO("[ElevationGlobalPlannerNode] Received PCD switch command: %s", msg->data.c_str());
      loadPcdDirect(msg->data);
    }
  }

  void onDebugQuery(const std_msgs::String::ConstPtr & msg)
  {
    if (!msg || msg->data.empty()) return;
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

private:
  std::string map_frame_;
  std::string base_frame_;
  int max_layers_{3};
  bool has_map_{false};
  bool has_manual_start_{false};
  bool has_goal_{false};
  geometry_msgs::Pose manual_start_;
  geometry_msgs::PoseStamped current_goal_;

  elevation_planner::CloudGraphBuilder builder_;
  elevation_planner::ManifoldGraph graph_;
  elevation_global_planner::ManifoldAStarPlanner planner_;
  elevation_global_planner::PathSmoother smoother_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  ros::Publisher path_pub_;
  ros::Publisher nodes_pub_;
  ros::Publisher edges_pub_;
  ros::Publisher debug_result_pub_;
  ros::Subscriber cloud_sub_;
  ros::Subscriber goal_sub_;
  ros::Subscriber initial_pose_sub_;
  ros::Subscriber pcd_cmd_sub_;
  ros::Subscriber debug_query_sub_;
};

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "elevation_global_planner_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ElevationGlobalPlannerNode node(nh, pnh);
  ros::spin();
  return 0;
}
