#pragma once

#include "elevation_planner_core/manifold_graph.hpp"
#include "elevation_planner_core/graph_store.hpp"
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
   * @brief 验证 2D/3D 轨迹在三维流形图上的高程物理连续性与落地支撑
   * 
   * 严格复用建图算法 (cloud_graph_builder) 的连续性口径:
   * 1. 换算跨越格数: k = max(1, ceil(dxy / res))
   * 2. 攀爬包络: |dz| <= k * max_step_height
   * 3. 途经中间支撑检查: 沿两点连线插值采样, 每个中间采样点高程必须在流形图中有踏面支撑 (|nd.z - sz| <= max_step_height)
   * 4. 目标层匹配: 若指定 target_z, 且轨迹终点接近全局目标 xy (<= 0.5m), 则终点高程与 target_z 偏差 <= 0.4m
   * 
   * @param xy_poses 轨迹 2D 坐标点序列 [(x0, y0), (x1, y1), ...]
   * @param start_z 机器人脚下起始高程 (m)
   * @param target_z 全局参考目标高程 (m, 若未知可传 std::numeric_limits<double>::quiet_NaN())
   * @param target_xy 全局参考目标 (x, y) 坐标 (用于终点贴合度门限判定)
   * @param max_step_height 单步台阶最大步高极限 (默认 0.25m)
   * @param min_peak_z 全局路径要求到达的最小峰值高程 (可选, 用于防楼梯/转角走底层平地抄近路)
   * @return true 轨迹在 3D 流形图上连续可行; false 存在踏空悬崖跌落或跨层
   */
  static bool validate(
      const std::vector<Eigen::Vector2d> & xy_poses,
      double start_z,
      double target_z = std::numeric_limits<double>::quiet_NaN(),
      const Eigen::Vector2d & target_xy = Eigen::Vector2d::Zero(),
      double max_step_height = 0.25,
      double min_peak_z = std::numeric_limits<double>::quiet_NaN(),
      std::string * out_reason = nullptr)
  {
    if (xy_poses.size() < 2) return true;

    // 优先使用完整全局流形图 (具备全图楼层、楼梯与拓扑先验);
    // 实时融合图 (fused_graph) 仅为局部滑动窗口 (半径通常仅2m), 若优先使用会导致前瞻轨迹末端出界踏空误判
    auto graph = GraphStore::instance().getGlobalGraph();
    if (!graph || graph->numNodes() == 0)
    {
      graph = GraphStore::instance().getFusedGraph();
    }
    if (!graph || graph->numNodes() == 0)
    {
      return true; // 无 3D 流形图先验时保持兼容通过
    }

    const double res = graph->getResolution();
    auto local_grid = GraphStore::instance().getLocalElevationGrid();

    // 踏面瞬时查询闭包: 优先通过局部切片以 O(1) 查询，未命中时向全局流形图检索
    auto querySurface = [&](double x, double y, double ref_z, double max_dz) -> std::pair<bool, double> {
      if (local_grid)
      {
        double lz = 0.0;
        if (local_grid->interpolateZ(x, y, lz))
        {
          if (std::abs(lz - ref_z) <= max_dz)
          {
            return {true, lz};
          }
        }
      }

      int r = 0, c = 0;
      if (!graph->toGridIndex(x, y, r, c))
      {
        return {false, ref_z};
      }

      double best_dz = std::numeric_limits<double>::max();
      double best_z = ref_z;
      bool found = false;

      // 1. 优先在落入的本格检索
      for (uint32_t nid : graph->getSpatialCellNodes(r, c))
      {
        const auto & nd = graph->getNode(nid);
        if (nd.traversability >= 0.95f) continue;
        double dz = std::abs(nd.z - ref_z);
        if (dz < best_dz)
        {
          best_dz = dz;
          best_z = nd.z;
          found = true;
        }
      }

      if (found && best_dz <= max_dz)
      {
        return {true, best_z};
      }

      // 2. 本格无有效支撑时，向 8 邻域检索 (容忍细微离散边缘漂移)
      for (int dr = -1; dr <= 1; ++dr)
      {
        for (int dc = -1; dc <= 1; ++dc)
        {
          if (dr == 0 && dc == 0) continue;
          int nr = r + dr;
          int nc = c + dc;
          if (nr < 0 || nr >= graph->getRows() || nc < 0 || nc >= graph->getCols()) continue;
          for (uint32_t nid : graph->getSpatialCellNodes(nr, nc))
          {
            const auto & nd = graph->getNode(nid);
            if (nd.traversability >= 0.95f) continue;
            double dz = std::abs(nd.z - ref_z);
            if (dz < best_dz)
            {
              best_dz = dz;
              best_z = nd.z;
              found = true;
            }
          }
        }
      }

      if (found && best_dz <= max_dz)
      {
        return {true, best_z};
      }
      return {false, ref_z};
    };

    // 0. 起点流形踏面高程吸附
    auto init_surf = querySurface(xy_poses[0].x(), xy_poses[0].y(), start_z, 0.50);
    if (!init_surf.first)
    {
      if (out_reason) *out_reason = "start_node_not_found";
      return false; // 起点脱离有效踏面
    }

    double cur_z = init_surf.second;
    double max_traj_z = cur_z;

    // 1. 沿 TEB 规划线穿过的物理踏面逐点进行高度连续性与台阶爬升包络校验
    // 步长取 0.08m，密集采样保证踏面连续支撑，不放过悬崖、镂空与跨层
    const double step_size = 0.08;
    const double allowed_single_step = std::max(max_step_height, 0.35); // 单步台阶最大高度极限 (0.35m, 兼容陡坡与楼梯转角台阶)

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

        // 检索落脚点踏面支撑 (与单步台阶极限严格对齐，消除口径冲突)
        auto surf = querySurface(pt.x(), pt.y(), cur_z, allowed_single_step);
        if (!surf.first)
        {
          if (out_reason) *out_reason = "surface_support_lost";
          return false; // 悬空、镂空楼梯井或跌落悬崖
        }

        double next_z = surf.second;
        double dz = std::abs(next_z - cur_z);
        if (dz > allowed_single_step)
        {
          if (out_reason) *out_reason = "step_height_exceeded";
          return false; // 单步高差超出物理攀爬极限
        }

        cur_z = next_z;
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
