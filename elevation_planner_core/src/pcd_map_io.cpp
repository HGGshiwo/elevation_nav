#include "elevation_planner_core/pcd_map_io.hpp"

#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/crop_box.h>
#include <yaml-cpp/yaml.h>
#include <Eigen/Core>

namespace elevation_planner
{

CropBoxConfig parseCropBoxConfig(const std::string & yaml_file)
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
    ROS_WARN("[PcdMapIO] Failed to parse crop_box from %s: %s", yaml_file.c_str(), e.what());
  }
  return cfg;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr loadAndCropPcd(
  const std::string & pcd_path,
  const std::string & map_config_yaml,
  CropBoxConfig * out_crop)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path, *cloud) == -1) {
    ROS_ERROR("[PcdMapIO] Failed to read PCD file: %s", pcd_path.c_str());
    return {};
  }
  const size_t raw_count = cloud->size();
  ROS_INFO("[PcdMapIO] Loaded raw PCD: %zu points (%s)", raw_count, pcd_path.c_str());

  if (!map_config_yaml.empty()) {
    CropBoxConfig crop = parseCropBoxConfig(map_config_yaml);
    if (out_crop) *out_crop = crop;
    if (crop.enable) {
      pcl::CropBox<pcl::PointXYZ> box;
      box.setInputCloud(cloud);
      box.setMin(Eigen::Vector4f(crop.min_x, crop.min_y, crop.min_z, 1.0f));
      box.setMax(Eigen::Vector4f(crop.max_x, crop.max_y, crop.max_z, 1.0f));
      pcl::PointCloud<pcl::PointXYZ>::Ptr cropped(new pcl::PointCloud<pcl::PointXYZ>);
      box.filter(*cropped);
      ROS_INFO("[PcdMapIO] CropBox applied: %zu -> %zu points ([%.2f, %.2f] x [%.2f, %.2f] x [%.2f, %.2f])",
               cloud->size(), cropped->size(),
               crop.min_x, crop.max_x, crop.min_y, crop.max_y, crop.min_z, crop.max_z);
      cloud = cropped;
    }
  } else if (out_crop) {
    *out_crop = CropBoxConfig{};
  }

  if (cloud->empty()) {
    ROS_ERROR("[PcdMapIO] Point cloud is empty after cropping!");
    return {};
  }
  return cloud;
}

} // namespace elevation_planner
