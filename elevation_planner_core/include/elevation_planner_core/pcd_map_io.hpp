#pragma once

#include <string>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace elevation_planner
{

/// CropBox 三维裁剪配置 (map 专属配置 yaml 的 crop_box 段)
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

/// 解析 yaml 中的 crop_box 段; 文件缺失或无该段时返回默认(关闭)配置
CropBoxConfig parseCropBoxConfig(const std::string & yaml_file);

/**
 * @brief 统一的先验点云加载管线前缀: 读 PCD 文件 → 按 map 配置做 CropBox 三维裁剪
 *
 * 全局规划器 (先验流形图) 与 Web 展示节点 (grid_map) 共用此入口,
 * 保证两边的裁剪口径与日志一致。裁剪边界针对单张地图手工配置在
 * map_crop_config.yaml 中, 切图时通过 /pcd_file_cmd 的 "路径;配置" 语法切换。
 *
 * @param pcd_path PCD 文件路径
 * @param map_config_yaml map 专属配置文件 (crop_box 段), 允许为空串
 * @param out_crop 非空时回传解析出的裁剪配置
 * @return 裁剪后点云; 读取失败或裁剪后为空返回空指针
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr loadAndCropPcd(
  const std::string & pcd_path,
  const std::string & map_config_yaml,
  CropBoxConfig * out_crop = nullptr);

} // namespace elevation_planner
