#pragma once

#include "elevation_planner_core/manifold_graph.hpp"
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <memory>
#include <string>

namespace elevation_planner
{

struct GraphBuildConfig
{
  double resolution{0.10};          ///< 2D 空间栅格投影分辨率 (m)
  double max_step_height{0.22};     ///< 四足狗台阶最大踏步高差 (m)
  double max_stride_length{0.35};   ///< 最大跨步步长 (m)
  double dog_height{0.45};          ///< 机器狗身高净空阈值 (m)
  double cluster_height_diff{0.08}; ///< 垂直单柱点云聚类厚度间距 (m)
  int min_cluster_points{2};        ///< 形成踏面的最少激光点数
};

class CloudGraphBuilder
{
public:
  CloudGraphBuilder() = default;
  explicit CloudGraphBuilder(const GraphBuildConfig & config) : config_(config) {}

  void setConfig(const GraphBuildConfig & config) { config_ = config; }
  const GraphBuildConfig & getConfig() const { return config_; }

  /// @brief 从 ROS PointCloud2 直接端到端构建流形拓扑图
  bool buildFromROSMsg(const sensor_msgs::PointCloud2 & cloud_msg, ManifoldGraph & out_graph);

  /// @brief 从 PCL PointXYZ 点云直接构建流形拓扑图
  bool buildFromPointCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & cloud, ManifoldGraph & out_graph);

  /// @brief 将图中的踏面节点导出为点云消息 (用于 RViz 彩色显示)
  static void toPointCloudMsg(const ManifoldGraph & graph,
                              const std::string & frame_id,
                              sensor_msgs::PointCloud2 & out_cloud);

  /// @brief 将图中的拓扑边导出为 RViz MarkerArray (用于三维连通性可视化)
  static void toMarkerArray(const ManifoldGraph & graph,
                            const std::string & frame_id,
                            visualization_msgs::MarkerArray & out_markers,
                            size_t max_edges = 30000);

  /// @brief 权威诊断两点之间的拓扑连通性及未建边物理原因 (输出 JSON 字符串)
  std::string diagnoseEdge(const ManifoldGraph & graph,
                           double x1, double y1, double z1,
                           double x2, double y2, double z2) const;

private:
  GraphBuildConfig config_;
};

} // namespace elevation_planner
