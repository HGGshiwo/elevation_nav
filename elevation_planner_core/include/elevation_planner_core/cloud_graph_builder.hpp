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
  double max_step_height{0.25};     ///< 四足狗台阶最大踏步高差 (m)
  double max_stride_length{0.35};   ///< 最大跨步步长 (m)
  double dog_height{0.45};          ///< 机器狗身高净空阈值 (m)
  double cluster_height_diff{0.08}; ///< 垂直单柱点云聚类厚度间距 (m)
  int min_cluster_points{2};        ///< 形成踏面的最少激光点数

  // ---- 残影点过滤 (SOR 统计离点滤): 悬浮稀疏点串 (动态物体残留/配准抖动) 会伪装成
  //      水平表面, 撑爆上方净空判定; 其近邻距离远大于真实表面点, 统计上可分离 ----
  int sor_mean_k{16};               ///< SOR 近邻统计点数
  double sor_std_mul{1.5};          ///< SOR 标准差倍数阈值 (越大越保守, 极大值等效关闭)

  // ---- 机体碰撞建模 (足印膨胀 + 建边扫掠) ----
  double footprint_radius{0.30};    ///< 机体足印外接半径 (m): 软代价膨胀边界
  double body_hard_radius{0.20};    ///< 机体硬阻挡半径 (m): 约半身宽+安全余量, 侧向障碍进入此范围则节点不可通行
  double sweep_penalty_weight{1.0}; ///< 建边时机体扫掠区软代价权重
  double foot_clearance{0.05};      ///< 足底容差 (m): 行走面下方此深度内的禁行节点仍保守视为剐蹭, 更深的视为脚下楼梯结构/其他层
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
  /// @note max_edges 须大于无向边总数, 否则地图后半段 (按栅格行序) 的边会被截断,
  ///       导致前端边可视化与权威诊断不一致; 20万约对应千万级节点的大图
  static void toMarkerArray(const ManifoldGraph & graph,
                            const std::string & frame_id,
                            visualization_msgs::MarkerArray & out_markers,
                            size_t max_edges = 200000);

  /// @brief 权威诊断两点之间的拓扑连通性及未建边物理原因 (输出 JSON 字符串)
  std::string diagnoseEdge(const ManifoldGraph & graph,
                           double x1, double y1, double z1,
                           double x2, double y2, double z2) const;

  /// @brief 权威诊断单个踏面节点的通行状态与禁行原因 (净空不足/侧向机体膨胀/软代价区)
  std::string diagnoseNode(const ManifoldGraph & graph,
                           double x, double y, double z) const;

private:
  GraphBuildConfig config_;
};

} // namespace elevation_planner
