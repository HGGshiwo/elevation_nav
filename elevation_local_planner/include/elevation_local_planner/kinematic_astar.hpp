#pragma once

#include <elevation_planner_core/manifold_graph.hpp>
#include <elevation_planner_core/manifold_search.hpp>
#include <vector>
#include <cmath>
#include <Eigen/Core>
#include <geometry_msgs/PoseStamped.h>

namespace elevation_local_planner
{

struct KinematicAStarConfig
{
  double weight_z = 2.0;              // 垂直爬升重力势能惩罚
  double weight_turn = 1.5;           // 转角三点平滑惩罚 (抑制锐角拐弯, 诱导大圆弧)
  double weight_traversability = 3.0; // 踏面阻尼/障碍物代价权重
  double max_step_height = 0.25;      // 最大单步踏步台阶高度 (m)
  double max_stride_length = 0.35;    // 最大单步水平跨步步长 (m)
  double max_xy_radius = 6.0;         // 绕障子图半径 (m): 距起点 XY 硬边界, 防阻挡时全图穷举
  int    max_expansions = 5000;       // 绕障 A* 弹出节点预算: 超限判定局部无解
};

/**
 * @brief 局部绕障搜索 (统一 A* 内核的局部模式封装: 单步邻域过滤 + 运动学代价 + 转角平滑)
 */
class KinematicAStar
{
public:
  KinematicAStar() = default;

  void setConfig(const KinematicAStarConfig& config)
  {
    cfg_ = config;
  }

  const KinematicAStarConfig& getConfig() const
  {
    return cfg_;
  }

  /**
   * @brief 在流形图上搜索兼具避障与运动学转角平滑的最优路径 (直接输出 node_id 序列)
   * @param graph 局部/全局流形图
   * @param start_nid 起始图节点 ID
   * @param goal_nid 目标图节点 ID
   * @param[out] path_nids 输出的图节点 ID 序列
   * @param[out] path_points 输出的 3D 节点坐标序列
   */
  bool search(const elevation_planner::ManifoldGraph& graph,
              uint32_t start_nid,
              uint32_t goal_nid,
              std::vector<uint32_t>& path_nids,
              std::vector<Eigen::Vector3d>& path_points)
  {
    path_nids.clear();
    path_points.clear();
    if (graph.numNodes() == 0 || start_nid >= graph.numNodes() || goal_nid >= graph.numNodes())
    {
      return false;
    }

    // 局部绕障模式: 与原实现完全一致的代价模型, 由统一内核执行;
    // 子图边界 (起点 XY 半径 + 扩展预算) 防止阻挡时在 19 万节点全图上穷举
    elevation_planner::ManifoldAstarParams prm;
    prm.filter_single_step = true;
    prm.max_step_height = cfg_.max_step_height;
    prm.max_stride_length = cfg_.max_stride_length;
    prm.use_edge_cost = false;
    prm.weight_z = cfg_.weight_z;
    prm.weight_traversability = cfg_.weight_traversability;
    prm.turn_weight = cfg_.weight_turn;
    prm.h_use_3d = true;
    prm.h_dist_weight = 1.0;
    prm.h_z_weight = cfg_.weight_z;
    prm.use_closed = false;
    prm.stale_push_limit = 200.0;
    prm.max_xy_radius = cfg_.max_xy_radius;
    prm.max_expansions = cfg_.max_expansions;

    if (!elevation_planner::manifoldAstarSearch(graph, start_nid, goal_nid, prm, path_nids))
    {
      return false;
    }

    path_points.reserve(path_nids.size());
    for (uint32_t nid : path_nids)
    {
      const auto& node = graph.getNode(nid);
      path_points.emplace_back(node.x, node.y, node.z);
    }
    return true;
  }

  /**
   * @brief 兼容旧重载
   */
  bool search(const elevation_planner::ManifoldGraph& graph,
              uint32_t start_nid,
              uint32_t goal_nid,
              std::vector<Eigen::Vector3d>& path_points)
  {
    std::vector<uint32_t> dummy_nids;
    return search(graph, start_nid, goal_nid, dummy_nids, path_points);
  }

private:
  KinematicAStarConfig cfg_;
};

} // namespace elevation_local_planner
