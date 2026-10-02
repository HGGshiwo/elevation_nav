#pragma once

#include <string>
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace elevation_planner
{

/// CropBox 三维裁剪配置
struct CropBoxConfig
{
  bool enable{false};
  float min_x{-100.0f};
  float max_x{100.0f};
  float min_y{-100.0f};
  float max_y{100.0f};
  float min_z{-100.0f};
  float max_z{100.0f};
};

/// 欧氏聚类过滤配置
struct ClusterFilterConfig
{
  bool enable{false};
  float tolerance{0.15f};
  int min_size{30};
  int max_size{1000000};
};

/// 从 ROS NodeHandle 读取 crop_box 参数 (默认前缀 "crop_box")
CropBoxConfig loadCropBoxConfig(const ros::NodeHandle & nh, const std::string & prefix = "crop_box");

/// 从 ROS NodeHandle 读取 欧氏聚类过滤 参数 (默认前缀 "")
ClusterFilterConfig loadClusterFilterConfig(const ros::NodeHandle & nh, const std::string & prefix = "");

/**
 * @brief 统一的先验点云加载管线: 读 PCD 文件 → 按 CropBoxConfig 做三维裁剪 → 欧氏聚类剔除小簇噪点
 *
 * @param pcd_path PCD 文件路径
 * @param crop 裁剪配置 (默认不裁剪)
 * @param cluster 聚类过滤配置 (默认不聚类)
 * @return 过滤后点云; 读取失败或处理后为空返回空指针
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr loadAndCropPcd(
  const std::string & pcd_path,
  const CropBoxConfig & crop = CropBoxConfig{},
  const ClusterFilterConfig & cluster = ClusterFilterConfig{});

} // namespace elevation_planner
