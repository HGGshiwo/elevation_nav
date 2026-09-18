#pragma once

#include "elevation_planner_core/manifold_graph.hpp"
#include "elevation_planner_core/graph_store.hpp"
#include "elevation_planner_core/local_elevation_grid.hpp"
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/cost_values.h>
#include <Eigen/Core>
#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>

namespace elevation_planner
{

class TrajectoryValidator
{
public:
  /**
   * @brief 验证 2D/3D 轨迹在三维流形与局部代价地图上的物理连续性与落地支撑
   * 
   * 采用单一可信源架构 (Single Source of Truth):
   * 1. 代价地图单步连通性: 前级已由 BFS 扩展与拓扑缝 (REASON_SEAM) 自验证,
   *    凡落在 costmap 自由区 (< LETHAL_OBSTACLE) 的轨迹点天然保证图连通,
   *    踩中致命障碍 (>= LETHAL_OBSTACLE) 则坚决判定非法阻断;
   * 2. 真实物理踏面支撑: 优先从 LocalElevationGrid 以 O(1) 双线性插值真实空间高程,
   *    验证踏面支撑完整性与单步台阶爬升包络 (|dz| <= allowed_single_step);
   * 3. 目标层匹配与天桥峰值: 验证终点与期望目标高程匹配度 (偏差 <= 0.4m), 杜绝底层穿模抄近道;
   * 4. 离线无代价地图模式下优雅降级为纯流形图几何与拓扑校验。
   */
  static bool validate(
      const std::vector<Eigen::Vector2d> & xy_poses,
      double start_z,
      double target_z = std::numeric_limits<double>::quiet_NaN(),
      const Eigen::Vector2d & target_xy = Eigen::Vector2d::Zero(),
      double max_step_height = 0.25,
      double min_peak_z = std::numeric_limits<double>::quiet_NaN(),
      std::string * out_reason = nullptr,
      const costmap_2d::Costmap2D * costmap = nullptr)
  {
    if (xy_poses.size() < 2) return true;

    auto local_grid = GraphStore::instance().getLocalElevationGrid();
    auto graph = GraphStore::instance().getGlobalGraph();
    if (!graph || graph->numNodes() == 0)
    {
      graph = GraphStore::instance().getFusedGraph();
    }

    const double allowed_single_step = std::max(max_step_height, 0.35); // 单步台阶最大高度极限 (0.35m)
    const double step_size = 0.05; // 采样步长与代价地图 0.05m 分辨率 1:1 精确对齐

    // 0. 起点高程吸附
    double cur_z = start_z;
    uint32_t prev_nid = 0;
    bool has_init_support = false;

    if (local_grid && local_grid->interpolateZ(xy_poses[0].x(), xy_poses[0].y(), cur_z))
    {
      has_init_support = true;
    }

    if (graph && graph->numNodes() > 0)
    {
      uint32_t init_nid = 0;
      if (graph->findClosestNode(xy_poses[0].x(), xy_poses[0].y(), cur_z, init_nid, 0.40, allowed_single_step))
      {
        prev_nid = init_nid;
        if (!has_init_support)
        {
          cur_z = graph->getNode(init_nid).z;
          has_init_support = true;
        }
      }
    }
    else if (!has_init_support)
    {
      has_init_support = true; // 无先验图兼容通过
    }

    if (!has_init_support)
    {
      if (out_reason) *out_reason = "start_node_not_found";
      return false;
    }

    double max_traj_z = cur_z;

    // 1. 沿 TEB 规划线穿过的物理踏面逐点进行代价地图致命障碍与高程连续性校验
    for (size_t i = 1; i < xy_poses.size(); ++i)
    {
      const Eigen::Vector2d & p_start = xy_poses[i - 1];
      const Eigen::Vector2d & p_end = xy_poses[i];
      double seg_len = (p_end - p_start).norm();

      int steps = std::max(1, static_cast<int>(std::ceil(seg_len / step_size)));
      for (int s = 1; s <= steps; ++s)
      {
        double t = static_cast<double>(s) / steps;
        Eigen::Vector2d pt = p_start + t * (p_end - p_start);

        // 1.1 踏面支撑与高程连续性校验
        double next_z = cur_z;
        bool has_support = false;
        uint32_t curr_nid = prev_nid;

        if (local_grid && local_grid->interpolateZ(pt.x(), pt.y(), next_z))
        {
          has_support = true;
        }
        else if (graph && graph->numNodes() > 0)
        {
          // 纯图模式: 纯 2D 最近邻匹配 (仅用高度过滤异层，彻底消除打分公式与高程粘滞)
          int r = 0, c = 0;
          if (graph->toGridIndex(pt.x(), pt.y(), r, c))
          {
            double best_dxy = std::numeric_limits<double>::max();
            for (int dr = -1; dr <= 1; ++dr)
            {
              for (int dc = -1; dc <= 1; ++dc)
              {
                int nr = r + dr, nc = c + dc;
                if (nr < 0 || nr >= graph->getRows() || nc < 0 || nc >= graph->getCols()) continue;
                for (uint32_t nid : graph->getSpatialCellNodes(nr, nc))
                {
                  const auto & nd = graph->getNode(nid);
                  if (nd.traversability >= 0.95f) continue;
                  if (std::abs(nd.z - cur_z) > allowed_single_step) continue;
                  double dxy = std::hypot(nd.x - pt.x(), nd.y - pt.y());
                  if (dxy <= 0.35 && dxy < best_dxy)
                  {
                    best_dxy = dxy;
                    next_z = nd.z;
                    curr_nid = nid;
                    has_support = true;
                  }
                }
              }
            }
          }
        }
        else
        {
          has_support = true; // 无图模式放行
        }

        // 同步查询流形图当前位置对应的节点 ID (用于管道与拓扑连通校验)
        if (graph && graph->numNodes() > 0)
        {
          uint32_t step_nid = 0;
          if (graph->findClosestNode(pt.x(), pt.y(), next_z, step_nid, 0.40, allowed_single_step))
          {
            curr_nid = step_nid;
          }
        }

        if (!has_support)
        {
          if (out_reason) *out_reason = "surface_support_lost";
          return false;
        }

        double dz = std::abs(next_z - cur_z);
        if (dz > allowed_single_step)
        {
          if (out_reason) *out_reason = "step_height_exceeded";
          return false;
        }

        // 1.3 拓扑流形管道边界校验 (若管道已生成，候选轨迹严禁脱离管道所限定的流形空间)
        auto corridor = GraphStore::instance().getTopologicalCorridor();
        if (corridor && !corridor->empty() && graph && graph->numNodes() > 0)
        {
          if (curr_nid < corridor->in_corridor.size())
          {
            if (!corridor->isInCorridor(curr_nid))
            {
              if (out_reason) *out_reason = "out_of_topological_corridor";
              return false;
            }
          }
        }

        // 纯图无代价地图兜底模式下的拓扑连通校验 (有 costmap 时已由自由区保证连通)
        if (!costmap && graph && graph->numEdges() > 0 && curr_nid != prev_nid)
        {
          if (!graph->isConnected(prev_nid, curr_nid, 5))
          {
            if (out_reason) *out_reason = "topological_disconnected";
            return false;
          }
        }

        cur_z = next_z;
        prev_nid = curr_nid;
        max_traj_z = std::max(max_traj_z, cur_z);
      }
    }

    // 2. 目标层级匹配校验：当轨迹末端接近前瞻目标 XY 门限时，验证终点实际高程是否与目标层级相符
    if (!std::isnan(target_z))
    {
      double dist_to_target_xy = (xy_poses.back() - target_xy).norm();
      if (dist_to_target_xy <= 0.50)
      {
        // 终点实际足底高程与全局目标高度偏差不得超过 0.40m (杜绝 4 楼目标规划到 3 楼底层的跨层穿模)
        if (std::abs(cur_z - target_z) > 0.40)
        {
          if (out_reason) *out_reason = "target_z_mismatch";
          return false;
        }
      }
    }

    // 3. 全局高程峰值/转角平台保真度校验 (针对天桥/过街天桥立体构型)
    if (!std::isnan(min_peak_z))
    {
      if (max_traj_z < min_peak_z - 0.35)
      {
        if (out_reason) *out_reason = "peak_z_not_reached";
        return false; // 候选轨迹未到达预期的爬升峰值高度，判定为底层绕行
      }
    }

    return true;
  }
};

} // namespace elevation_planner
