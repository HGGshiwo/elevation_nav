#include <ros/ros.h>
#include <ros/package.h>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_pcl/GridMapPclLoader.hpp>
#include <grid_map_msgs/GridMap.h>
#include <std_msgs/String.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "elevation_map_loader/manifold_forest.hpp"

class PcdToGridMapNode
{
public:
  PcdToGridMapNode(ros::NodeHandle & nh, ros::NodeHandle & pnh)
  {
    std::string pcd_file, pcd_file_cmd_topic, grid_map_topic;
    pnh.param<std::string>("pcd_file", pcd_file, "");
    pnh.param<std::string>("pcd_file_cmd_topic", pcd_file_cmd_topic, "/pcd_file_cmd");
    pnh.param<std::string>("grid_map_topic", grid_map_topic, "/grid_map");
    pnh.param<std::string>("frame_id", frame_id_, "map");
    pnh.param<std::string>("config_file", config_file_, "");

    elevation_map_loader::ExtractorConfig cfg;
    pnh.param<double>("resolution", cfg.resolution, 0.10);
    pnh.param<double>("max_step_height", cfg.max_step_height, 0.25);
    pnh.param<double>("max_slope_deg", cfg.max_slope_deg, 30.0);
    pnh.param<double>("max_stride_length", cfg.max_stride_length, 0.35);
    pnh.param<double>("dog_height", cfg.dog_height, 0.50);
    extractor_.setConfig(cfg);

    if (config_file_.empty()) {
      std::string pkg_path = ros::package::getPath("elevation_map_loader");
      if (pkg_path.empty()) {
        pkg_path = ros::package::getPath("elevation_nav");
      }
      if (!pkg_path.empty()) {
        config_file_ = pkg_path + "/config/parameters.yaml";
      }
    }

    grid_map_pub_ = nh.advertise<grid_map_msgs::GridMap>(grid_map_topic, 1, true);
    pcd_file_sub_ = nh.subscribe(pcd_file_cmd_topic, 1, &PcdToGridMapNode::onPcdFileCmd, this);
    timer_ = nh.createTimer(ros::Duration(1.0), &PcdToGridMapNode::onTimer, this);

    if (!pcd_file.empty()) {
      loadPcdAndProcess(pcd_file);
    }
  }

  void onPcdFileCmd(const std_msgs::String::ConstPtr & msg)
  {
    if (msg && !msg->data.empty()) {
      ROS_INFO("[PcdToGridMapNode] Received PCD switch command: %s", msg->data.c_str());
      loadPcdAndProcess(msg->data);
    }
  }

  void onTimer(const ros::TimerEvent &)
  {
    publishMap();
  }

  void loadPcdAndProcess(const std::string & pcd_path)
  {
    ROS_INFO("[PcdToGridMapNode] Loading PCD file: %s", pcd_path.c_str());
    grid_map::GridMapPclLoader loader;
    loader.loadParameters(config_file_);

    try {
      loader.loadCloudFromPcdFile(pcd_path);
      loader.preProcessInputCloud();
      loader.initializeGridMapGeometryFromInputCloud();
      loader.addLayerFromInputCloud("elevation");
      grid_map_ = loader.getGridMap();
      grid_map_.setFrameId(frame_id_);
    } catch (const std::exception & e) {
      ROS_ERROR("[PcdToGridMapNode] PCL preprocessing failed: %s", e.what());
      return;
    }

    extractor_.extract(loader, grid_map_);

    has_map_ = true;
    const auto & size = grid_map_.getSize();
    ROS_INFO("[PcdToGridMapNode] Multi-surface map built: %d x %d grid, res = %.3fm",
             size(0), size(1), grid_map_.getResolution());

    publishMap();
  }

  void publishMap()
  {
    if (!has_map_) return;
    grid_map_.setTimestamp(ros::Time::now().toNSec());
    grid_map_msgs::GridMap msg;
    grid_map::GridMapRosConverter::toMessage(grid_map_, msg);
    grid_map_pub_.publish(msg);
  }

private:
  std::string frame_id_;
  std::string config_file_;
  elevation_map_loader::ManifoldForestExtractor extractor_;

  grid_map::GridMap grid_map_;
  bool has_map_{false};

  ros::Publisher grid_map_pub_;
  ros::Subscriber pcd_file_sub_;
  ros::Timer timer_;
};

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "pcd_to_grid_map_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  PcdToGridMapNode node(nh, pnh);
  ros::spin();
  return 0;
}
