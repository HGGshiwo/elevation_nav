#include "elevation_global_planner/manifold_astar.hpp"
#include <ros/ros.h>
#include <tf2/utils.h>
#include <cmath>
#include <algorithm>
#include <limits>

namespace elevation_global_planner
{

ManifoldAStarPlanner::ManifoldAStarPlanner()
{
}

bool ManifoldAStarPlanner::initialize(const elevation_planner::ManifoldGraph & graph)
{
  graph_ = &graph;
  is_initialized_ = (graph_ && graph_->numNodes() > 0);
  if (is_initialized_)
  {
    los_simplifier_.build(*graph_);
  }
  return is_initialized_;
}

float ManifoldAStarPlanner::computeHeuristic(uint32_t a_id, uint32_t b_id) const
{
  const auto & a = graph_->getNode(a_id);
  const auto & b = graph_->getNode(b_id);
  float dxy = std::hypot(a.x - b.x, a.y - b.y);
  float dz = std::abs(a.z - b.z);
  return dxy + 1.8f * dz;
}

bool ManifoldAStarPlanner::findClosestNode(double x, double y, double z,
                                          elevation_planner::ManifoldNode & out_node) const
{
  if (!is_initialized_) return false;
  uint32_t nid = 0;
  if (graph_->findClosestNode(x, y, z, nid, 1.0, 1.2)) {
    out_node = graph_->getNode(nid);
    return true;
  }
  return false;
}

bool ManifoldAStarPlanner::plan(const geometry_msgs::PoseStamped & start,
                               const geometry_msgs::PoseStamped & goal,
                               nav_msgs::Path & out_path)
{
  if (!is_initialized_ || graph_ == nullptr || graph_->numNodes() == 0) {
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
  if (!graph_->findStartNode(start.pose.position.x, start.pose.position.y, start.pose.position.z,
                            hx, hy, start_id, 1.5, 0.35)) {
    ROS_ERROR("[ManifoldAStarPlanner] Cannot find valid node near start: (%.2f, %.2f, %.2f)",
              start.pose.position.x, start.pose.position.y, start.pose.position.z);
    return false;
  }
  if (!graph_->findClosestNode(goal.pose.position.x, goal.pose.position.y, goal.pose.position.z, goal_id, 1.5, 0.50)) {
    ROS_ERROR("[ManifoldAStarPlanner] Cannot find valid node near goal: (%.2f, %.2f, %.2f)",
              goal.pose.position.x, goal.pose.position.y, goal.pose.position.z);
    return false;
  }
  
  const auto & sn = graph_->getNode(start_id);
  const auto & gn = graph_->getNode(goal_id);
  ROS_INFO("[AStar] Start node %u (%.2f, %.2f, %.2f, trav=%.2f, dyn=%.2f, zone=%u, hardBlocked=%s), Goal node %u (%.2f, %.2f, %.2f, trav=%.2f, dyn=%.2f, zone=%u, hardBlocked=%s)",
           start_id, sn.x, sn.y, sn.z, sn.traversability, sn.dynamic_trav, sn.cost_zone, sn.hardBlocked() ? "true" : "false",
           goal_id, gn.x, gn.y, gn.z, gn.traversability, gn.dynamic_trav, gn.cost_zone, gn.hardBlocked() ? "true" : "false");

  // 统一 A* 内核: 采用反向搜索 (从固定 Goal 反向至动态 Start) + 上一帧路径节点复用 (防抖)
  elevation_planner::ManifoldAstarParams prm;
  prm.use_closed = true;
  prm.filter_single_step = false;
  prm.use_edge_cost = true;
  prm.h_use_3d = false;
  prm.h_dist_weight = 1.0;
  prm.h_z_weight = 1.8;
  prm.goal_tol_xy = graph_->getResolution();
  prm.goal_tol_z = 0.20;

  const std::vector<uint32_t>* prev_ptr = (goal_id == last_goal_id_ && !last_path_ids_.empty()) ? &last_path_ids_ : nullptr;

  std::vector<uint32_t> path_ids;
  if (!elevation_planner::manifoldAstarSearch(*graph_, start_id, goal_id, prm, path_ids, nullptr, prev_ptr, /*search_backward=*/true)) {
    ROS_WARN("[ManifoldAStarPlanner] No path found between start (%u: %.2f, %.2f) and goal (%u: %.2f, %.2f) (check step height or slope)",
             start_id, sn.x, sn.y, goal_id, gn.x, gn.y);
    last_path_ids_.clear();
    return false;
  }

  // 记录本轮成功路径与目标，供下一帧复用防抖
  last_goal_id_ = goal_id;
  last_path_ids_ = path_ids;

  // 路径重构
  out_path.poses.clear();
  out_path.header = start.header;
  for (uint32_t id : path_ids) {
    const auto & nd = graph_->getNode(id);
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
        const auto & nd = graph_->getNode(nid);
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
      out_path = pruned;
    }
  }

  ROS_INFO_THROTTLE(2.0, "[ManifoldAStarPlanner] A* plan found successfully, path points: %zu", out_path.poses.size());
  return true;
}

} // namespace elevation_global_planner
