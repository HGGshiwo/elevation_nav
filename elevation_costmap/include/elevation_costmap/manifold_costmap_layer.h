#pragma once

#include <ros/ros.h>
#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/layered_costmap.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose.h>
#include <elevation_planner_core/manifold_fusion_engine.hpp>
#include <thread>
#include <mutex>
#include <memory>
#include <string>

namespace elevation_costmap
{

/**
 * @class ManifoldCostmapLayer
 * @brief move_base 进程内的实时点云融合宿主 (costmap_2d::Layer 插件)
 *
 * 职责: 承载 core 的 ManifoldFusionEngine —— 订阅实时点云并按固定节拍维护融合
 * 流形图写入 GraphStore, 供同进程的 AStarLocalPlanner 零拷贝读图做动态避障。
 * GraphStore 为进程级单例, 融合写入者必须与局部规划器同进程; costmap 插件位是
 * move_base 唯一的进程内寄生点, 故本层保留插件身份, 代价网格本身不产出数据
 * (updateBounds/updateCosts 为空实现)。
 *
 * 历史: 本层曾同时负责 1:1 地毯代价地图的构建与发布 (供 TEB 局部规划器消费),
 * TEB 移除后规划器直读流形图, 地毯渲染链路已整体删除。
 */
class ManifoldCostmapLayer : public costmap_2d::CostmapLayer
{
public:
  ManifoldCostmapLayer();
  virtual ~ManifoldCostmapLayer();

  virtual void onInitialize() override;
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw,
                            double* min_x, double* min_y,
                            double* max_x, double* max_y) override;
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid,
                           int min_i, int min_j, int max_i, int max_j) override;
  virtual void activate() override;
  virtual void deactivate() override;
  virtual void reset() override;

private:
  void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr & msg);
  bool lookupRobotPose(geometry_msgs::Pose & pose);
  void fusionLoop();

  ros::Subscriber cloud_sub_;
  ros::Subscriber aux_cloud_sub_;   ///< 编辑器障碍叠加云 (/elevation_editor_obstacles)

  void auxCloudCallback(const sensor_msgs::PointCloud2::ConstPtr & msg);
  ros::Publisher dynamic_nodes_pub_;  ///< 融合引擎动态改写节点实时可视化 (x,y,z,intensity=traversability)
  std::string map_frame_{"map"};
  std::string base_frame_{"base_link"};
  std::string cloud_topic_{"/lidar_points"};
  double fusion_rate_{10.0};

  elevation_planner::ManifoldFusionEngine fusion_engine_;

  bool worker_running_{false};
  std::thread worker_thread_;
};

} // namespace elevation_costmap
