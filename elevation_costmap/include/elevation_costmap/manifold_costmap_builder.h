#pragma once

#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <elevation_planner_core/manifold_graph.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <string>

namespace elevation_costmap
{

struct ManifoldCostmapBuilderConfig
{
  double resolution{0.05};             ///< 局部栅格分辨率 (m)
  double map_width{6.0};               ///< 局部代价地图横向总宽度 (m)
  double map_length{6.0};              ///< 局部代价地图纵向总长度 (m)
  double forward_offset{3.0};          ///< 机器人到地图后边缘的偏移 (m), 3.0m 居中对齐
  double lookahead_distance{0.6};      ///< 全局路径前瞻采样距离 (m), 用于确定切向前进方向
  double height_tolerance{0.30};       ///< 贴地离面高度容差 (m)
  double dog_height{0.45};             ///< 净空高度阈值 (m)
  std::string map_frame{"map"};        ///< 全局参考系
  std::string base_frame{"base_link"}; ///< 机器人机身参考系
  std::string output_frame{"local_manifold_frame"}; ///< 局部流形切空间参考系
};

/**
 * @class ManifoldCostmapBuilder
 * @brief 基于三维多层流形图构建 1:1 无畸变局部代价地图的核心构建器
 * 
 * 算法原理:
 * 1. 从流形图中提取机器狗脚底踏面点 P0, 前进切向点 P_front, 侧向踏面点 P_side;
 * 2. 叉乘构建贴地法向量 N 与正交切空间主轴 [X_axis, Y_axis];
 * 3. 对视野内节点点乘降维投影, 严格保距, 消除坡度速度失真与横向拉伸畸变.
 */
class ManifoldCostmapBuilder
{
public:
  explicit ManifoldCostmapBuilder(const ManifoldCostmapBuilderConfig & config = ManifoldCostmapBuilderConfig());

  void setConfig(const ManifoldCostmapBuilderConfig & config) { config_ = config; }
  const ManifoldCostmapBuilderConfig & getConfig() const { return config_; }

  /**
   * @brief 从流形图构建局部 1:1 代价地图
   * @param graph 3D流形图 (融合图或全局先验图)
   * @param robot_pose 机器人当前3D世界位姿
   * @param global_plan 全局参考规划路径 (3D)
   * @param out_grid 输出的标准局部代价地图 (OccupancyGrid)
   * @param out_tf 输出的局部切空间坐标系到世界坐标系的变换
   * @return true 构建成功, false 失败
   */
  bool buildCostmap(const elevation_planner::ManifoldGraph & graph,
                    const geometry_msgs::Pose & robot_pose,
                    const std::vector<geometry_msgs::PoseStamped> & global_plan,
                    nav_msgs::OccupancyGrid & out_grid,
                    geometry_msgs::TransformStamped & out_tf) const;

private:
  ManifoldCostmapBuilderConfig config_;
};

} // namespace elevation_costmap
