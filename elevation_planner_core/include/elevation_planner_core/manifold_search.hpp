#pragma once

#include "manifold_graph.hpp"
#include <queue>
#include <string>
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
 * @param[out] fail_reason 可选失败原因 (成功时不清空) —— 预算耗尽/开集耗尽可区分
 * @return 是否到达目标 (精确匹配或容差早退)
 */
inline bool manifoldAstarSearch(const ManifoldGraph& graph,
                                uint32_t start_id,
                                uint32_t goal_id,
                                const ManifoldAstarParams& prm,
                                std::vector<uint32_t>& out_ids,
                                std::string* fail_reason = nullptr,
                                const std::vector<uint32_t>* prev_path_ids = nullptr,
                                bool search_backward = false)
{
  out_ids.clear();
  const size_t n = graph.numNodes();
  if (n == 0 || start_id >= n || goal_id >= n)
  {
    if (fail_reason) *fail_reason = "invalid node id (start/goal out of range)";
    return false;
  }

  if (start_id == goal_id)
  {
    out_ids.push_back(start_id);
    return true;
  }

  // 若开启反向搜索 (从静态目标 Goal 反向搜索至动态起点 Start)，交换源与目标
  const uint32_t src_id = search_backward ? goal_id : start_id;
  const uint32_t dst_id = search_backward ? start_id : goal_id;

  const double INF = std::numeric_limits<double>::max();
  const uint32_t INVALID = std::numeric_limits<uint32_t>::max();

  std::vector<double> g(static_cast<size_t>(n), INF);
  std::vector<uint32_t> came_from(static_cast<size_t>(n), INVALID);
  std::vector<uint8_t> closed(static_cast<size_t>(n), 0);

  // 上一帧历史路径节点标记 (用于路径惯性折让与彻底消除路径震荡)
  std::vector<uint8_t> on_prev_path;
  if (prev_path_ids && !prev_path_ids->empty())
  {
    on_prev_path.assign(n, 0);
    for (uint32_t pid : *prev_path_ids)
    {
      if (pid < n) on_prev_path[pid] = 1;
    }
  }

  auto heuristic = [&](uint32_t from_id) {
    const GraphNode& a = graph.getNode(from_id);
    const GraphNode& b = graph.getNode(dst_id);
    const double dx = static_cast<double>(a.x) - b.x;
    const double dy = static_cast<double>(a.y) - b.y;
    const double dz = std::abs(static_cast<double>(a.z) - b.z);
    const double base = prm.h_use_3d ? std::sqrt(dx * dx + dy * dy + dz * dz) : std::hypot(dx, dy);
    return prm.h_dist_weight * base + prm.h_z_weight * dz;
  };

  typedef std::pair<double, uint32_t> QEntry;
  std::priority_queue<QEntry, std::vector<QEntry>, std::greater<QEntry>> open;

  g[src_id] = 0.0;
  open.push({heuristic(src_id), src_id});

  const GraphNode& src_node = graph.getNode(src_id);
  const GraphNode& dst_node = graph.getNode(dst_id);
  bool reached = false;
  uint32_t reached_id = dst_id;
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

    if (prm.max_expansions > 0 && ++expansions > prm.max_expansions)
    {
      if (fail_reason) *fail_reason = "expansion budget exhausted (max_expansions=" +
                                      std::to_string(prm.max_expansions) + ", subgraph too tight or obstacle too wide)";
      return false;
    }

    if (curr == dst_id)
    {
      reached = true;
      reached_id = curr;
      break;
    }

    const GraphNode& curr_node = graph.getNode(curr);
    const bool curr_dyn = curr_node.dynamic_trav > 0.f;
    if (prm.goal_tol_xy > 0.0 &&
        std::hypot(curr_node.x - dst_node.x, curr_node.y - dst_node.y) < prm.goal_tol_xy &&
        std::abs(curr_node.z - dst_node.z) < prm.goal_tol_z)
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

      if (nv.hardBlocked()) continue;

      if (prm.max_xy_radius > 0.0 &&
          std::hypot(static_cast<double>(nv.x) - src_node.x,
                     static_cast<double>(nv.y) - src_node.y) > prm.max_xy_radius)
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
      if (prm.use_edge_cost && !curr_dyn && nv.dynamic_trav <= 0.f)
      {
        step_cost = es[i].cost;
      }
      else
      {
        step_cost = d3 + prm.weight_z * dz
                  + 0.5 * prm.weight_traversability * (curr_node.traversability + nv.traversability);
      }

      // 上一帧历史路径惯性折扣 (0.85): 抑制等价分支间的随机震荡跳变
      if (!on_prev_path.empty() && on_prev_path[nid])
      {
        step_cost *= 0.85;
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

  if (!reached)
  {
    if (fail_reason) *fail_reason = "open set exhausted (no path within subgraph radius " +
                                    std::to_string(prm.max_xy_radius) + "m)";
    return false;
  }

  // 路径重构: 保证输出总是 start -> ... -> goal 顺序
  for (uint32_t c = reached_id; c != INVALID; c = came_from[c])
  {
    out_ids.push_back(c);
    if (c == src_id) break;
  }

  if (!search_backward)
  {
    // 正向搜索: 回溯是从 goal 到 start，需反转为 start -> goal
    std::reverse(out_ids.begin(), out_ids.end());
  }
  // 反向搜索: 回溯是从 reached (start_id) 到 goal_id，天然就是 start -> goal 顺序，无需反转!

  return true;
}

} // namespace elevation_planner
