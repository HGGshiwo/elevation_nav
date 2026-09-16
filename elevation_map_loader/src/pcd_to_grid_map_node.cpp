#include <ros/ros.h>
#include <ros/package.h>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_pcl/GridMapPclLoader.hpp>
#include <grid_map_msgs/GridMap.h>
#include <std_msgs/String.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <yaml-cpp/yaml.h>

#include "elevation_map_loader/manifold_forest.hpp"

struct CropBoxConfig {
  bool enable{false};
  float min_x{-100.0f};
  float max_x{100.0f};
  float min_y{-100.0f};
  float max_y{100.0f};
  float min_z{-100.0f};
  float max_z{100.0f};
};

static CropBoxConfig parseCropBoxConfig(const std::string & yaml_file)
{
  CropBoxConfig cfg;
  if (yaml_file.empty()) return cfg;
  try {
    YAML::Node root = YAML::LoadFile(yaml_file);
    if (root["crop_box"] && root["crop_box"].IsMap()) {
      auto node = root["crop_box"];
      if (node["enable"]) cfg.enable = node["enable"].as<bool>();
      if (node["min_x"]) cfg.min_x = node["min_x"].as<float>();
      if (node["max_x"]) cfg.max_x = node["max_x"].as<float>();
      if (node["min_y"]) cfg.min_y = node["min_y"].as<float>();
      if (node["max_y"]) cfg.max_y = node["max_y"].as<float>();
      if (node["min_z"]) cfg.min_z = node["min_z"].as<float>();
      if (node["max_z"]) cfg.max_z = node["max_z"].as<float>();
    }
  } catch (const std::exception & e) {
    ROS_WARN("[PcdToGridMapNode] Failed to parse crop_box from %s: %s", yaml_file.c_str(), e.what());
  }
  return cfg;
}

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
    pnh.param<std::string>("map_config_file", map_config_file_, "");

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
      std::string pcd_path = msg->data;
      std::string custom_config = "";
      size_t sep = pcd_path.find(';');
      if (sep != std::string::npos) {
        custom_config = pcd_path.substr(sep + 1);
        pcd_path = pcd_path.substr(0, sep);
      }
      ROS_INFO("[PcdToGridMapNode] Received PCD switch command: %s (map_config: %s)",
               pcd_path.c_str(), custom_config.empty() ? "(default)" : custom_config.c_str());
      loadPcdAndProcess(pcd_path, custom_config);
    }
  }

  void onTimer(const ros::TimerEvent &)
  {
    publishMap();
  }

  void loadPcdAndProcess(const std::string & pcd_path, const std::string & override_map_config = "")
  {
    ROS_INFO("[PcdToGridMapNode] Loading PCD file: %s", pcd_path.c_str());
    std::string active_map_config = override_map_config.empty() ? map_config_file_ : override_map_config;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path, *cloud) == -1) {
      ROS_ERROR("[PcdToGridMapNode] Failed to read PCD file: %s", pcd_path.c_str());
      return;
    }
    ROS_INFO("[PcdToGridMapNode] Loaded raw PCD: %zu points", cloud->size());

    // 1. 点云空间三维裁剪 (CropBox)
    if (!active_map_config.empty()) {
      CropBoxConfig crop = parseCropBoxConfig(active_map_config);
      if (crop.enable) {
        pcl::CropBox<pcl::PointXYZ> box;
        box.setInputCloud(cloud);
        box.setMin(Eigen::Vector4f(crop.min_x, crop.min_y, crop.min_z, 1.0f));
        box.setMax(Eigen::Vector4f(crop.max_x, crop.max_y, crop.max_z, 1.0f));
        pcl::PointCloud<pcl::PointXYZ>::Ptr cropped(new pcl::PointCloud<pcl::PointXYZ>);
        box.filter(*cropped);
        ROS_INFO("[PcdToGridMapNode] CropBox applied: %zu -> %zu points ([%.2f, %.2f] x [%.2f, %.2f] x [%.2f, %.2f])",
                 cloud->size(), cropped->size(),
                 crop.min_x, crop.max_x, crop.min_y, crop.max_y, crop.min_z, crop.max_z);
        cloud = cropped;
      }
    }

    if (cloud->empty()) {
      ROS_ERROR("[PcdToGridMapNode] Point cloud is empty after cropping!");
      return;
    }

    // 2. 网格与高程图提取参数：专属配置优先，缺省回退基础配置
    std::string config_to_load = config_file_;
    if (!active_map_config.empty()) {
      try {
        YAML::Node root = YAML::LoadFile(active_map_config);
        if (root["pcl_grid_map_extraction"]) {
          config_to_load = active_map_config;
        }
      } catch (...) {}
    }
    ROS_INFO("[PcdToGridMapNode] Applying grid map parameters from: %s", config_to_load.c_str());

    grid_map::GridMapPclLoader loader;
    loader.loadParameters(config_to_load);

    try {
      loader.setInputCloud(cloud);
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
  std::string map_config_file_;
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
