#include "elevation_global_planner/manifold_astar.hpp"
#include <ros/ros.h>
#include <tf2/utils.h>
#include <cmath>
#include <algorithm>
#include <limits>

namespace elevation_global_planner
{

ManifoldAStarPlanner::ManifoldAStarPlanner()
  : cost_evaluator_(elevation_planner::QuadrupedDynamicsConfig())
{
}

bool ManifoldAStarPlanner::initialize(const elevation_planner::ManifoldGraph & graph)
{
  graph_ = graph;
  is_initialized_ = (graph_.numNodes() > 0);
  if (is_initialized_)
  {
    los_simplifier_.build(graph_);
  }
  return is_initialized_;
}

void ManifoldAStarPlanner::setPortalManager(const elevation_planner::LayerPortalManager & portal_mgr)
{
  portal_mgr_ = portal_mgr;
}

void ManifoldAStarPlanner::setCostEvaluator(const elevation_planner::CostEvaluator & evaluator)
{
  cost_evaluator_ = evaluator;
}

float ManifoldAStarPlanner::computeHeuristic(uint32_t a_id, uint32_t b_id) const
{
  const auto & a = graph_.getNode(a_id);
  const auto & b = graph_.getNode(b_id);
  float dxy = std::hypot(a.x - b.x, a.y - b.y);
  float dz = std::abs(a.z - b.z);
  return dxy + 1.8f * dz;
}

bool ManifoldAStarPlanner::findClosestNode(double x, double y, double z,
                                          elevation_planner::ManifoldNode & out_node) const
{
  if (!is_initialized_) return false;
  uint32_t nid = 0;
  if (graph_.findClosestNode(x, y, z, nid, 1.0, 1.2)) {
    out_node = graph_.getNode(nid);
    return true;
  }
  return false;
}

bool ManifoldAStarPlanner::plan(const geometry_msgs::PoseStamped & start,
                               const geometry_msgs::PoseStamped & goal,
                               nav_msgs::Path & out_path)
{
  if (!is_initialized_ || graph_.numNodes() == 0) {
    ROS_WARN("[ManifoldAStarPlanner] Graph is not initialized or empty");
    return false;
  }

  uint32_t start_id = 0, goal_id = 0;

  // 提取机器人前向矢量 (优先车头朝向，缺省则使用目标方向)
  double hx = 0.0, hy = 0.0;
  const auto & q = start.pose.orientation;
  if (q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w > 0.1) {
    double yaw = tf2::getYaw(q);
    hx = std::cos(yaw);
    hy = std::sin(yaw);
  } else {
    hx = goal.pose.position.x - start.pose.position.x;
    hy = goal.pose.position.y - start.pose.position.y;
    double len = std::hypot(hx, hy);
    if (len > 1e-3) {
      hx /= len;
      hy /= len;
    }
  }

  // 采用严格前向约束查找起点，杜绝重规划将起点倒退吸附到身后与跨层误吸附 (max_dist_z: 0.35m)
  if (!graph_.findStartNode(start.pose.position.x, start.pose.position.y, start.pose.position.z,
                            hx, hy, start_id, 1.5, 0.35)) {
    ROS_ERROR("[ManifoldAStarPlanner] Cannot find valid node near start: (%.2f, %.2f, %.2f)",
              start.pose.position.x, start.pose.position.y, start.pose.position.z);
    return false;
  }
  if (!graph_.findClosestNode(goal.pose.position.x, goal.pose.position.y, goal.pose.position.z, goal_id, 1.5, 0.50)) {
    ROS_ERROR("[ManifoldAStarPlanner] Cannot find valid node near goal: (%.2f, %.2f, %.2f)",
              goal.pose.position.x, goal.pose.position.y, goal.pose.position.z);
    return false;
  }

  // 统一 A* 内核 (全局模式: CSR 边代价 + closed 去重 + dxy+1.8dz 启发 + 目标容差早退)
  elevation_planner::ManifoldAstarParams prm;
  prm.use_closed = true;
  prm.filter_single_step = false;
  prm.use_edge_cost = true;
  prm.h_use_3d = false;
  prm.h_dist_weight = 1.0;
  prm.h_z_weight = 1.8;
  prm.goal_tol_xy = graph_.getResolution();
  prm.goal_tol_z = 0.20;

  std::vector<uint32_t> path_ids;
  if (!elevation_planner::manifoldAstarSearch(graph_, start_id, goal_id, prm, path_ids)) {
    ROS_WARN("[ManifoldAStarPlanner] No path found between start and goal (check step height or slope)");
    return false;
  }

  // 路径重构
  out_path.poses.clear();
  out_path.header = start.header;
  for (uint32_t id : path_ids) {
    const auto & nd = graph_.getNode(id);
    geometry_msgs::PoseStamped ps;
    ps.header = start.header;
    ps.pose.position.x = nd.x;
    ps.pose.position.y = nd.y;
    ps.pose.position.z = nd.z;
    ps.pose.orientation.x = static_cast<double>(id); // 隐式透传 node_id 给 move_base 局部规划器 (方案 B)
    ps.pose.orientation.y = 0.0;
    ps.pose.orientation.z = 0.0;
    ps.pose.orientation.w = 1.0;
    out_path.poses.push_back(ps);
  }

  // ===== SC-LOS 贪心剪枝: 密集节点路径 → 稀疏航点 (保留原始密集路径供 debug 发布) =====
  last_dense_path_ = out_path;
  if (los_prune_enabled_ && out_path.poses.size() > 2) {
    std::vector<uint32_t> ids;
    ids.reserve(out_path.poses.size());
    for (const auto & ps : out_path.poses) {
      ids.push_back(static_cast<uint32_t>(std::max(0.0, std::round(ps.pose.orientation.x))));
    }
    const std::vector<uint32_t> sparse = los_simplifier_.simplifyPath(ids, los_max_segment_);
    if (sparse.size() >= 2 && sparse.size() < ids.size()) {
      nav_msgs::Path pruned;
      pruned.header = out_path.header;
      for (uint32_t nid : sparse) {
        const auto & nd = graph_.getNode(nid);
        geometry_msgs::PoseStamped ps;
        ps.header = out_path.header;
        ps.pose.position.x = nd.x;
        ps.pose.position.y = nd.y;
        ps.pose.position.z = nd.z;
        ps.pose.orientation.x = static_cast<double>(nid);
        ps.pose.orientation.y = 0.0;
        ps.pose.orientation.z = 0.0;
        ps.pose.orientation.w = 1.0;
        pruned.poses.push_back(ps);
      }
      ROS_INFO("[ManifoldAStarPlanner] SC-LOS pruned: %zu -> %zu waypoints", ids.size(), sparse.size());
      out_path = pruned;
    }
  }

  ROS_INFO("[ManifoldAStarPlanner] A* plan found successfully, path points: %zu", out_path.poses.size());
  return true;
}

} // namespace elevation_global_planner
