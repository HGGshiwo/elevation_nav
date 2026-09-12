#include "elevation_global_planner/manifold_astar.hpp"
#include <ros/ros.h>
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
  if (!graph_.findClosestNode(start.pose.position.x, start.pose.position.y, start.pose.position.z, start_id, 2.5, 2.5)) {
    ROS_ERROR("[ManifoldAStarPlanner] Cannot find valid node near start: (%.2f, %.2f, %.2f)",
              start.pose.position.x, start.pose.position.y, start.pose.position.z);
    return false;
  }
  if (!graph_.findClosestNode(goal.pose.position.x, goal.pose.position.y, goal.pose.position.z, goal_id, 2.5, 2.5)) {
    ROS_ERROR("[ManifoldAStarPlanner] Cannot find valid node near goal: (%.2f, %.2f, %.2f)",
              goal.pose.position.x, goal.pose.position.y, goal.pose.position.z);
    return false;
  }

  const size_t num_nodes = graph_.numNodes();
  std::vector<float> g_scores(num_nodes, std::numeric_limits<float>::max());
  std::vector<uint32_t> came_from(num_nodes, UINT32_MAX);
  std::vector<bool> closed(num_nodes, false);

  std::priority_queue<SearchEntry, std::vector<SearchEntry>, std::greater<SearchEntry>> open_set;

  g_scores[start_id] = 0.0f;
  float h0 = computeHeuristic(start_id, goal_id);
  open_set.push({start_id, h0});

  bool reached = false;
  const auto & goal_node = graph_.getNode(goal_id);

  while (!open_set.empty()) {
    SearchEntry top = open_set.top();
    open_set.pop();

    uint32_t curr_id = top.node_id;
    if (closed[curr_id]) continue;
    closed[curr_id] = true;

    if (curr_id == goal_id) {
      reached = true;
      break;
    }

    const auto & curr_node = graph_.getNode(curr_id);

    // 容差接近目标检查
    if (std::hypot(curr_node.x - goal_node.x, curr_node.y - goal_node.y) < graph_.getResolution() &&
        std::abs(curr_node.z - goal_node.z) < 0.20f) {
      goal_id = curr_id;
      reached = true;
      break;
    }

    // CSR 连续边极速展开
    uint16_t edge_cnt = 0;
    const auto * edges = graph_.getEdges(curr_id, edge_cnt);

    float curr_g = g_scores[curr_id];
    for (uint16_t i = 0; i < edge_cnt; ++i) {
      uint32_t next_id = edges[i].target_id;
      if (closed[next_id]) continue;

      float step_cost = edges[i].cost;
      float tent_g = curr_g + step_cost;

      if (tent_g < g_scores[next_id]) {
        g_scores[next_id] = tent_g;
        came_from[next_id] = curr_id;
        float f = tent_g + computeHeuristic(next_id, goal_id);
        open_set.push({next_id, f});
      }
    }
  }

  if (!reached) {
    ROS_WARN("[ManifoldAStarPlanner] No path found between start and goal (check step height or slope)");
    return false;
  }

  // 路径重构
  out_path.poses.clear();
  out_path.header = start.header;
  uint32_t trace = goal_id;

  while (trace != UINT32_MAX) {
    const auto & nd = graph_.getNode(trace);
    geometry_msgs::PoseStamped ps;
    ps.header = start.header;
    ps.pose.position.x = nd.x;
    ps.pose.position.y = nd.y;
    ps.pose.position.z = nd.z;
    ps.pose.orientation.w = 1.0;
    out_path.poses.push_back(ps);

    if (trace == start_id) break;
    trace = came_from[trace];
  }

  std::reverse(out_path.poses.begin(), out_path.poses.end());
  ROS_INFO("[ManifoldAStarPlanner] A* plan found successfully, path points: %zu", out_path.poses.size());
  return true;
}

} // namespace elevation_global_planner
