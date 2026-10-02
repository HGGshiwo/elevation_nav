#include "elevation_planner_core/pcd_map_io.hpp"

#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/crop_box.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <Eigen/Core>

namespace elevation_planner
{

CropBoxConfig loadCropBoxConfig(const ros::NodeHandle & nh, const std::string & prefix)
{
  CropBoxConfig cfg;
  std::string p = prefix.empty() ? "" : (prefix + "/");
  nh.param<bool>(p + "enable", cfg.enable, false);
  nh.param<float>(p + "min_x", cfg.min_x, -100.0f);
  nh.param<float>(p + "max_x", cfg.max_x, 100.0f);
  nh.param<float>(p + "min_y", cfg.min_y, -100.0f);
  nh.param<float>(p + "max_y", cfg.max_y, 100.0f);
  nh.param<float>(p + "min_z", cfg.min_z, -100.0f);
  nh.param<float>(p + "max_z", cfg.max_z, 100.0f);
  return cfg;
}

ClusterFilterConfig loadClusterFilterConfig(const ros::NodeHandle & nh, const std::string & prefix)
{
  ClusterFilterConfig cfg;
  std::string p = prefix.empty() ? "" : (prefix + "/");
  nh.param<bool>(p + "cluster_filter_enable", cfg.enable, false);
  nh.param<float>(p + "cluster_tolerance", cfg.tolerance, 0.15f);
  nh.param<int>(p + "cluster_min_size", cfg.min_size, 30);
  nh.param<int>(p + "cluster_max_size", cfg.max_size, 1000000);
  return cfg;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr loadAndCropPcd(
  const std::string & pcd_path,
  const CropBoxConfig & crop,
  const ClusterFilterConfig & cluster)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path, *cloud) == -1) {
    ROS_ERROR("[PcdMapIO] Failed to read PCD file: %s", pcd_path.c_str());
    return {};
  }
  const size_t raw_count = cloud->size();
  ROS_INFO("[PcdMapIO] Loaded raw PCD: %zu points (%s)", raw_count, pcd_path.c_str());

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

  if (cluster.enable && cluster.min_size > 0 && cloud->size() > static_cast<size_t>(cluster.min_size)) {
    const size_t pre_cluster_count = cloud->size();
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(cluster.tolerance > 0.0f ? cluster.tolerance : 0.15f);
    ec.setMinClusterSize(cluster.min_size);
    ec.setMaxClusterSize(cluster.max_size > 0 ? cluster.max_size : 1000000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    pcl::PointCloud<pcl::PointXYZ>::Ptr clustered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto & indices : cluster_indices) {
      for (int idx : indices.indices) {
        clustered_cloud->points.push_back(cloud->points[idx]);
      }
    }
    clustered_cloud->width = clustered_cloud->points.size();
    clustered_cloud->height = 1;
    clustered_cloud->is_dense = true;
    ROS_INFO("[PcdMapIO] EuclideanClusterExtraction applied: %zu -> %zu points (kept %zu clusters, removed %zu noise points)",
             pre_cluster_count, clustered_cloud->size(), cluster_indices.size(), pre_cluster_count - clustered_cloud->size());
    cloud = clustered_cloud;
  }

  if (cloud->empty()) {
    ROS_ERROR("[PcdMapIO] Point cloud is empty after filtering!");
    return {};
  }
  return cloud;
}

} // namespace elevation_planner
