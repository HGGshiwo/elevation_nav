#pragma once

#include <ros/ros.h>
#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/layered_costmap.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Int32MultiArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <elevation_planner_core/cloud_graph_builder.hpp>
#include <elevation_planner_core/manifold_graph.hpp>
#include <elevation_planner_core/graph_store.hpp>
#include "elevation_costmap/manifold_costmap_builder.h"
#include <thread>
#include <mutex>
#include <memory>
#include <string>

namespace elevation_costmap
{

/**
 * @class ManifoldCostmapLayer
 * @brief 在 move_base 进程内部直接运行的 1:1 流形代价地图图层插件
 *
 * 核心特性:
 * 1. 同进程直接零拷贝访问 GraphStore::instance()，无需重复加载 PCD 地图;
 * 2. 后台线程维护实时点云融合图，以恒定 10Hz 避障节拍执行纯几何地毯铺设;
 * 3. updateCosts 直接将 1:1 踏面与障碍写进 local_costmap 的 master_grid，原生喂给 TEB 等局部规划器;
 * 4. 彻底消除死等阻塞，启动 0ms 顺畅运行。
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
  void planCallback(const nav_msgs::Path::ConstPtr & msg);
  bool lookupRobotPose(geometry_msgs::Pose & pose);
  void fusionAndCarpetLoop();

  // ROS 订阅与发布
  ros::Subscriber cloud_sub_;
  ros::Subscriber plan_sub_;
  ros::Publisher costmap_pub_; // 方便 RViz/Web 可视化
  ros::Publisher debug_pub_;       // 逐格成因码调试图层 (与地毯同几何, data=CellReason)
  ros::Publisher debug_nodes_pub_; // 逐格胜出节点 id (Int32MultiArray: [w, h, id...])

  std::string map_frame_{"map"};
  std::string base_frame_{"base_link"};
  std::string cloud_topic_{"/lidar_points"};
  std::string plan_topic_{"/move_base/ElevationGlobalPlanner/global_plan"};

  double fusion_rate_{10.0};
  double crop_radius_xy_{2.0};
  double crop_height_above_{2.0};
  double crop_height_below_{1.0};

  elevation_planner::CloudGraphBuilder graph_builder_;
  ManifoldCostmapBuilder costmap_builder_;

  sensor_msgs::PointCloud2::ConstPtr latest_cloud_;
  bool cloud_dirty_{false};
  std::mutex cloud_mutex_;

  std::vector<geometry_msgs::PoseStamped> latest_plan_;
  std::mutex plan_mutex_;

  std::shared_ptr<elevation_planner::ManifoldGraph> fused_graph_;
  std::mutex fused_graph_mutex_;

  // 内部缓存的最新的 1:1 地图
  nav_msgs::OccupancyGrid cached_grid_;
  nav_msgs::OccupancyGrid cached_debug_;
  std_msgs::Int32MultiArray cached_debug_nodes_;
  bool has_cached_grid_{false};
  std::mutex grid_mutex_;

  bool worker_running_{false};
  std::thread worker_thread_;
};

} // namespace elevation_costmap
