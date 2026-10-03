#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include "elevation_planner_core/cloud_graph_builder.hpp"
#include "elevation_planner_core/pcd_map_io.hpp"

class ManifoldMapServerNode
{
public:
  ManifoldMapServerNode(ros::NodeHandle & nh, ros::NodeHandle & pnh)
    : nh_(nh), pnh_(pnh)
  {
    pnh_.param<std::string>("frame_id", frame_id_, "map");

    crop_box_cfg_ = elevation_planner::loadCropBoxConfig(pnh_);

    elevation_planner::GraphBuildConfig cfg;
    pnh_.param<double>("resolution", cfg.resolution, 0.10);
    pnh_.param<double>("max_step_height", cfg.max_step_height, 0.25);
    pnh_.param<double>("max_stride_length", cfg.max_stride_length, 0.35);
    pnh_.param<double>("dog_height", cfg.dog_height, 0.45);
    pnh_.param<double>("footprint_radius", cfg.footprint_radius, 0.26);
    pnh_.param<double>("body_hard_radius", cfg.body_hard_radius, 0.17);
    pnh_.param<double>("inflation_radius", cfg.inflation_radius, 0.50);
    pnh_.param<double>("sweep_penalty_weight", cfg.sweep_penalty_weight, 1.0);
    pnh_.param<bool>("body_hard_ring_enabled", cfg.body_hard_ring_enabled, true);
    pnh_.param<int>("sor_mean_k", cfg.sor_mean_k, 16);
    pnh_.param<double>("sor_std_mul", cfg.sor_std_mul, 1.5);
    pnh_.param<bool>("cluster_filter_enable", cfg.cluster_filter_enable, true);
    pnh_.param<double>("cluster_tolerance", cfg.cluster_tolerance, 0.15);
    pnh_.param<int>("cluster_min_size", cfg.cluster_min_size, 30);
    pnh_.param<int>("cluster_max_size", cfg.cluster_max_size, 0);
    pnh_.param<double>("cluster_height_diff", cfg.cluster_height_diff, 0.08);
    pnh_.param<int>("min_cluster_points", cfg.min_cluster_points, 2);

    double robot_length = 0.0, robot_width = 0.0, margin = 0.04;
    pnh_.param<double>("obstacle_safety_margin", margin, 0.04);
    if (pnh_.getParam("robot_width", robot_width) && robot_width > 0.0) {
      cfg.body_hard_radius = robot_width * 0.5 + margin;
      if (pnh_.getParam("robot_length", robot_length) && robot_length > 0.0) {
        cfg.footprint_radius = std::hypot(robot_length * 0.5, robot_width * 0.5) + margin;
      }
      ROS_INFO("[ManifoldMapServer] Geometry configured: body_hard_radius=%.3fm, footprint_radius=%.3fm (W=%.2f, L=%.2f)",
               cfg.body_hard_radius, cfg.footprint_radius, robot_width, robot_length);
    }

    builder_.setConfig(cfg);

    nodes_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/elevation_graph_nodes", 1, true);
    edges_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/elevation_graph_edges", 1, true);

    pcd_cmd_sub_ = nh_.subscribe("/pcd_file_cmd", 1, &ManifoldMapServerNode::onPcdCmd, this);

    std::string pcd_file;
    pnh_.param<std::string>("pcd_file", pcd_file, "");
    if (!pcd_file.empty()) {
      loadMap(pcd_file);
    }
  }

  void loadMap(const std::string & path)
  {
    ROS_INFO("[ManifoldMapServer] Loading PCD file: %s", path.c_str());
    auto cloud = elevation_planner::loadAndCropPcd(path, crop_box_cfg_);
    if (!cloud || cloud->empty()) {
      ROS_ERROR("[ManifoldMapServer] Failed to load or crop PCD file: %s", path.c_str());
      return;
    }

    ros::Time t0 = ros::Time::now();
    elevation_planner::ColumnTable table;
    if (!builder_.buildColumnTable(cloud, table)) {
      ROS_ERROR("[ManifoldMapServer] Failed to build column table from point cloud");
      return;
    }

    graph_.clear();
    if (!builder_.buildGraphFromColumnTable(table, graph_)) {
      ROS_ERROR("[ManifoldMapServer] Failed to build manifold graph from column table");
      return;
    }

    has_map_ = true;
    double elapsed = (ros::Time::now() - t0).toSec();
    ROS_INFO("[ManifoldMapServer] Graph built successfully: %zu nodes, %zu edges in %.3f s",
             graph_.numNodes(), graph_.numEdges(), elapsed);

    publishGraphVisuals();
  }

  void publishGraphVisuals()
  {
    if (!has_map_) return;

    sensor_msgs::PointCloud2 cloud_msg;
    elevation_planner::CloudGraphBuilder::toPointCloudMsg(graph_, frame_id_, cloud_msg);
    nodes_pub_.publish(cloud_msg);

    visualization_msgs::MarkerArray marker_msg;
    elevation_planner::CloudGraphBuilder::toMarkerArray(graph_, frame_id_, marker_msg);
    edges_pub_.publish(marker_msg);

    ROS_INFO("[ManifoldMapServer] Published %zu nodes and %zu edges (latched)",
             graph_.numNodes(), graph_.numEdges());
  }

  void onPcdCmd(const std_msgs::String::ConstPtr & msg)
  {
    if (msg && !msg->data.empty()) {
      std::string pcd_path = msg->data;
      size_t sep = pcd_path.find(';');
      if (sep != std::string::npos) {
        pcd_path = pcd_path.substr(0, sep);
      }
      ROS_INFO("[ManifoldMapServer] Received PCD switch command: %s", pcd_path.c_str());
      loadMap(pcd_path);
    }
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher nodes_pub_;
  ros::Publisher edges_pub_;
  ros::Subscriber pcd_cmd_sub_;

  elevation_planner::CloudGraphBuilder builder_;
  elevation_planner::CropBoxConfig crop_box_cfg_;
  elevation_planner::ManifoldGraph graph_;
  std::string frame_id_;
  bool has_map_{false};
};

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "manifold_map_server");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  ManifoldMapServerNode server(nh, pnh);
  ROS_INFO("[ManifoldMapServer] Node started and ready");

  ros::spin();
  return 0;
}
