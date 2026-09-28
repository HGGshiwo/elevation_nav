#pragma once

#include "manifold_graph.hpp"
#include <queue>
#include <vector>
#include <cmath>
#include <cstdint>
#include <limits>
#include <algorithm>

namespace elevation_planner
{

/**
 * @brief 统一 A* 搜索参数 (全局规划与局部绕障共用同一内核)
 *
 * 全局模式 (默认值): CSR 预编译边代价 + closed 严格去重 + 启发式 dxy + 1.8*|dz| + 目标容差早退
 * 局部绕障模式: 单步运动学邻域过滤 + 运动学代价重算 + 三点转角惩罚 + 陈旧堆项容限
 */
struct ManifoldAstarParams
{
  // ---- 邻域过滤 ----
  bool   filter_single_step{false};  ///< true: 在 CSR 边上追加单步运动学过滤 (步高/步距极限)
  double max_step_height{0.25};      ///< 单步垂直高度极限 (m)
  double max_stride_length{0.35};    ///< 单步水平跨步极限 (m)

  // ---- 代价模型 ----
  bool   use_edge_cost{true};        ///< true: 使用 CSR 预编译边代价; false: 运动学代价重算
  double weight_z{2.0};              ///< 运动学模式: 垂直爬升重力势能惩罚
  double weight_traversability{3.0}; ///< 运动学模式: 踏面阻尼/障碍代价
  double turn_weight{0.0};           ///< 三点转角惩罚权重 (0 关闭; 局部绕障 1.5)

  // ---- 启发式: h = h_dist_weight * (h_use_3d ? d3 : dxy) + h_z_weight * |dz| ----
  bool   h_use_3d{false};
  double h_dist_weight{1.0};
  double h_z_weight{1.8};

  // ---- 目标容差早退 (goal_tol_xy <= 0 关闭) ----
  double goal_tol_xy{-1.0};
  double goal_tol_z{0.0};

  // ---- 陈旧堆项处理 ----
  bool   use_closed{true};           ///< true: closed 数组严格去重 (全局模式)
  double stale_push_limit{200.0};    ///< use_closed=false 时陈旧 f 的跳变容限 (局部绕障原值 200)

  // ---- 局部子图边界 (0 = 不限; 动态避障 A* 必须设置, 防止阻挡时在全图穷举) ----
  double max_xy_radius{0.0};         ///< 扩展空间半径 (m): 距起点 XY 超出该半径的邻居不扩展
  int    max_expansions{0};          ///< 弹出节点数上限: 超限判定局部子图内无解 (返回 false)
};

/**
 * @brief CSR 流形图上的统一 A* 内核 (原 ManifoldAStarPlanner 与 KinematicAStar 的公共实现)
 * @param graph    流形拓扑图 (CSR 边已 finalize)
 * @param start_id 起点节点 ID
 * @param goal_id  目标节点 ID
 * @param prm      搜索参数
 * @param[out] out_ids start→reached 的节点 ID 序列 (含两端, 顺序 start→goal)
 * @return 是否到达目标 (精确匹配或容差早退)
 */
inline bool manifoldAstarSearch(const ManifoldGraph& graph,
                                uint32_t start_id,
                                uint32_t goal_id,
                                const ManifoldAstarParams& prm,
                                std::vector<uint32_t>& out_ids)
{
  out_ids.clear();
  const size_t n = graph.numNodes();
  if (n == 0 || start_id >= n || goal_id >= n) return false;

  if (start_id == goal_id)
  {
    out_ids.push_back(start_id);
    return true;
  }

  const double INF = std::numeric_limits<double>::max();
  const uint32_t INVALID = std::numeric_limits<uint32_t>::max();

  std::vector<double> g(static_cast<size_t>(n), INF);
  std::vector<uint32_t> came_from(static_cast<size_t>(n), INVALID);
  std::vector<uint8_t> closed(static_cast<size_t>(n), 0);

  auto heuristic = [&](uint32_t from_id) {
    const GraphNode& a = graph.getNode(from_id);
    const GraphNode& b = graph.getNode(goal_id);
    const double dx = static_cast<double>(a.x) - b.x;
    const double dy = static_cast<double>(a.y) - b.y;
    const double dz = std::abs(static_cast<double>(a.z) - b.z);
    const double base = prm.h_use_3d ? std::sqrt(dx * dx + dy * dy + dz * dz) : std::hypot(dx, dy);
    return prm.h_dist_weight * base + prm.h_z_weight * dz;
  };

  typedef std::pair<double, uint32_t> QEntry;
  std::priority_queue<QEntry, std::vector<QEntry>, std::greater<QEntry>> open;

  g[start_id] = 0.0;
  open.push({heuristic(start_id), start_id});

  const GraphNode& start_node = graph.getNode(start_id);
  const GraphNode& goal_node = graph.getNode(goal_id);
  bool reached = false;
  uint32_t reached_id = goal_id;
  int expansions = 0;

  while (!open.empty())
  {
    const QEntry top = open.top();
    open.pop();
    const uint32_t curr = top.second;

    if (prm.use_closed)
    {
      if (closed[curr]) continue;
      closed[curr] = 1;
    }
    else if (top.first > g[curr] + prm.stale_push_limit)
    {
      continue;
    }

    // 局部子图预算: 弹出节点数超限判定范围内无解 (阻断阻挡时的全图穷举)
    if (prm.max_expansions > 0 && ++expansions > prm.max_expansions)
    {
      return false;
    }

    if (curr == goal_id)
    {
      reached = true;
      reached_id = curr;
      break;
    }

    const GraphNode& curr_node = graph.getNode(curr);
    if (prm.goal_tol_xy > 0.0 &&
        std::hypot(curr_node.x - goal_node.x, curr_node.y - goal_node.y) < prm.goal_tol_xy &&
        std::abs(curr_node.z - goal_node.z) < prm.goal_tol_z)
    {
      reached = true;
      reached_id = curr;
      break;
    }

    const uint32_t parent = came_from[curr];

    uint16_t cnt = 0;
    const GraphEdge* es = graph.getEdges(curr, cnt);
    for (uint16_t i = 0; i < cnt; ++i)
    {
      const uint32_t nid = es[i].target_id;
      if (nid >= n) continue;
      const GraphNode& nv = graph.getNode(nid);

      // 禁行节点不可落足。先验图上禁行节点在建图期即无边, 此检查天然不触发;
      // 融合引擎原位刷新属性后边结构仍在, 动态障碍靠此检查排除
      if (nv.traversability >= 0.95f) continue;

      // 局部子图空间边界: 距起点 XY 超出半径的邻居不扩展 (动态避障限定在机器人邻域内)
      if (prm.max_xy_radius > 0.0 &&
          std::hypot(static_cast<double>(nv.x) - start_node.x,
                     static_cast<double>(nv.y) - start_node.y) > prm.max_xy_radius)
      {
        continue;
      }

      if (prm.filter_single_step &&
          !graph.isSingleStepNeighbor(curr_node, nv, prm.max_step_height, prm.max_stride_length))
      {
        continue;
      }

      const double dxy = std::hypot(static_cast<double>(nv.x) - curr_node.x,
                                    static_cast<double>(nv.y) - curr_node.y);
      const double dz = std::abs(static_cast<double>(nv.z) - curr_node.z);
      const double d3 = std::sqrt(dxy * dxy + dz * dz);

      double step_cost;
      if (prm.use_edge_cost)
      {
        step_cost = es[i].cost;
      }
      else
      {
        step_cost = d3 + prm.weight_z * dz + prm.weight_traversability * nv.traversability;
      }

      if (prm.turn_weight > 0.0 && parent != INVALID)
      {
        const GraphNode& pv = graph.getNode(parent);
        const double i0x = static_cast<double>(curr_node.x) - pv.x;
        const double i0y = static_cast<double>(curr_node.y) - pv.y;
        const double i1x = static_cast<double>(nv.x) - curr_node.x;
        const double i1y = static_cast<double>(nv.y) - curr_node.y;
        const double n0 = std::hypot(i0x, i0y);
        const double n1 = std::hypot(i1x, i1y);
        if (n0 > 1e-4 && n1 > 1e-4)
        {
          double cos_theta = (i0x * i1x + i0y * i1y) / (n0 * n1);
          cos_theta = std::max(-1.0, std::min(1.0, cos_theta));
          step_cost += prm.turn_weight * (1.0 - cos_theta) * d3;
        }
      }

      const double tent = g[curr] + step_cost;
      if (tent < g[nid])
      {
        g[nid] = tent;
        came_from[nid] = curr;
        open.push({tent + heuristic(nid), nid});
      }
    }
  }

  if (!reached) return false;

  for (uint32_t c = reached_id; c != INVALID; c = came_from[c])
  {
    out_ids.push_back(c);
    if (c == start_id) break;
  }
  std::reverse(out_ids.begin(), out_ids.end());
  return true;
}

} // namespace elevation_planner
