#include "elevation_costmap/manifold_costmap_builder.h"
#include <elevation_planner_core/manifold_graph.hpp>
#include <iostream>
#include <cassert>
#include <cmath>

int main()
{
  ros::Time::init();
  std::cout << "[TestManifoldCostmapBuilder] Starting unit tests..." << std::endl;

  // 1. 构造包含 35 度斜坡/楼梯踏面的 ManifoldGraph
  elevation_planner::ManifoldGraph graph;
  graph.initSpatialGrid(0.10, -5.0, -5.0, 100, 100);

  for (int r = 0; r <= 30; ++r)
  {
    double x = r * 0.10;
    double z = r * 0.08; // 梯级上升: dx=0.1, dz=0.08 -> 坡度约 38.6 度
    for (int c = -5; c <= 5; ++c)
    {
      double y = c * 0.10;
      elevation_planner::GraphNode node;
      node.x = static_cast<float>(x);
      node.y = static_cast<float>(y);
      node.z = static_cast<float>(z);
      node.traversability = 0.0f;
      node.headroom = 2.0f;
      node.row = 50 + r;
      node.col = 50 + c;
      node.layer_id = 0;
      graph.addNode(node);
    }
  }
  graph.finalizeCSR();

  std::cout << "  Graph built with " << graph.numNodes() << " nodes." << std::endl;
  assert(graph.numNodes() > 0);

  // 2. 初始化 ManifoldCostmapBuilder
  elevation_costmap::ManifoldCostmapBuilderConfig config;
  config.resolution = 0.05;
  config.map_length = 3.0;
  config.map_width = 2.0;
  config.forward_offset = 0.5;
  config.lookahead_distance = 0.6;
  elevation_costmap::ManifoldCostmapBuilder builder(config);

  // 3. 模拟机器人位于 (0.2, 0.0, 0.16)
  geometry_msgs::Pose robot_pose;
  robot_pose.position.x = 0.20;
  robot_pose.position.y = 0.0;
  robot_pose.position.z = 0.16;
  robot_pose.orientation.w = 1.0;

  // 模拟全局路径: 沿台阶向上
  std::vector<geometry_msgs::PoseStamped> global_plan;
  for (int i = 0; i <= 20; ++i)
  {
    geometry_msgs::PoseStamped ps;
    ps.pose.position.x = i * 0.10;
    ps.pose.position.y = 0.0;
    ps.pose.position.z = i * 0.08;
    ps.pose.orientation.w = 1.0;
    global_plan.push_back(ps);
  }

  // 4. 执行代价地图构建
  nav_msgs::OccupancyGrid out_grid;
  geometry_msgs::TransformStamped out_tf;
  bool ok = builder.buildCostmap(graph, robot_pose, global_plan, out_grid, out_tf);
  assert(ok);

  std::cout << "  Manifold costmap build success! Grid dimensions: "
            << out_grid.info.width << " x " << out_grid.info.height
            << ", resolution: " << out_grid.info.resolution << "m" << std::endl;

  // 5. 验证法向量恒朝上 (N.z > 0)
  Eigen::Quaterniond q(out_grid.info.origin.orientation.w,
                       out_grid.info.origin.orientation.x,
                       out_grid.info.origin.orientation.y,
                       out_grid.info.origin.orientation.z);
  Eigen::Vector3d n = q.toRotationMatrix().col(2);
  std::cout << "  Estimated Terrain Normal N = ["
            << n.x() << ", " << n.y() << ", " << n.z() << "]" << std::endl;
  assert(n.z() > 0.0);

  // 6. 验证踏面通道通畅与悬崖阻断
  int free_count = 0;
  int obstacle_count = 0;
  for (int8_t cost : out_grid.data)
  {
    if (cost < 50) free_count++;
    else if (cost == 100) obstacle_count++;
  }
  std::cout << "  Grid cell statistics: Free cells = " << free_count
            << ", Obstacle/Cliff cells = " << obstacle_count << std::endl;
  assert(free_count > 0);
  assert(obstacle_count > 0);

  // 7. 测试多层上下重合 (Z-Buffer 深度测试):
  // 构造一楼踏面 (z = 0.0) 和二楼天花板/踏面 (z = 3.0), 机器人位于一楼 (z = 0.0)
  elevation_planner::ManifoldGraph multi_graph;
  multi_graph.initSpatialGrid(0.10, -5.0, -5.0, 100, 100);
  // 一楼踏面 (自由可通)
  elevation_planner::GraphNode n_floor1;
  n_floor1.x = 1.0f; n_floor1.y = 1.0f; n_floor1.z = 0.0f;
  n_floor1.traversability = 0.0f; n_floor1.headroom = 2.0f;
  n_floor1.row = 60; n_floor1.col = 60; n_floor1.layer_id = 0;
  multi_graph.addNode(n_floor1);

  // 二楼踏面 (假设不可通行或者代价不同)
  elevation_planner::GraphNode n_floor2;
  n_floor2.x = 1.0f; n_floor2.y = 1.0f; n_floor2.z = 3.0f;
  n_floor2.traversability = 0.9f; n_floor2.headroom = 2.0f;
  n_floor2.row = 60; n_floor2.col = 60; n_floor2.layer_id = 1;
  multi_graph.addNode(n_floor2);
  multi_graph.finalizeCSR();

  elevation_costmap::ManifoldCostmapBuilderConfig cfg_6m;
  cfg_6m.map_width = 6.0;
  cfg_6m.map_length = 6.0;
  cfg_6m.resolution = 0.05;
  elevation_costmap::ManifoldCostmapBuilder builder_6m(cfg_6m);

  geometry_msgs::Pose bot_1f;
  bot_1f.position.x = 1.0; bot_1f.position.y = 1.0; bot_1f.position.z = 0.0;
  bot_1f.orientation.w = 1.0;

  nav_msgs::OccupancyGrid grid_1f;
  geometry_msgs::TransformStamped tf_1f;
  bool ok_1f = builder_6m.buildCostmap(multi_graph, bot_1f, {}, grid_1f, tf_1f);
  assert(ok_1f);
  assert(grid_1f.info.width == 120 && grid_1f.info.height == 120);

  // 验证在 (1.0, 1.0) 对应的中心栅格处，保留了一楼的自由代价 (0)，丢弃了二楼的不可通行 (100)
  int c_center = static_cast<int>(std::floor((1.0 - grid_1f.info.origin.position.x) / 0.05));
  int r_center = static_cast<int>(std::floor((1.0 - grid_1f.info.origin.position.y) / 0.05));
  size_t idx_center = static_cast<size_t>(r_center * grid_1f.info.width + c_center);
  std::cout << "  Z-Buffer overlap test at (1.0, 1.0): cell cost = "
            << static_cast<int>(grid_1f.data[idx_center])
            << " (Expected 0 for 1F floor)" << std::endl;
  assert(grid_1f.data[idx_center] == 0);

  std::cout << "[TestManifoldCostmapBuilder] ALL ASSERTIONS PASSED!" << std::endl;
  return 0;
}
