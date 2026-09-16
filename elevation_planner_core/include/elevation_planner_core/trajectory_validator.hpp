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
   * @return true 轨迹在 3D 流形图上连续可行; false 存在踏空悬崖跌落或跨层
   */
  static bool validate(
      const std::vector<Eigen::Vector2d> & xy_poses,
      double start_z,
      double target_z = std::numeric_limits<double>::quiet_NaN(),
      const Eigen::Vector2d & target_xy = Eigen::Vector2d::Zero(),
      double max_step_height = 0.25)
  {
    if (xy_poses.size() < 2) return true;

    auto graph = GraphStore::instance().getFusedGraph();
    if (!graph || graph->numNodes() == 0)
    {
      graph = GraphStore::instance().getGlobalGraph();
    }
    if (!graph || graph->numNodes() == 0)
    {
      return true; // 无 3D 流形图先验时保持兼容通过
    }

    const double res = graph->getResolution();
    double cur_z = start_z;

    // 0. 起点高程踏面吸附
    int r0 = 0, c0 = 0;
    if (graph->toGridIndex(xy_poses[0].x(), xy_poses[0].y(), r0, c0))
    {
      double best_dz = std::numeric_limits<double>::max();
      for (uint32_t nid : graph->getSpatialCellNodes(r0, c0))
      {
        const auto & nd = graph->getNode(nid);
        if (nd.traversability >= 0.95f) continue;
        double dz = std::abs(nd.z - cur_z);
        if (dz < best_dz && dz <= 0.40)
        {
          best_dz = dz;
          cur_z = nd.z;
        }
      }
    }

    // 1. 逐段检查高程连续性
    for (size_t i = 1; i < xy_poses.size(); ++i)
    {
      const Eigen::Vector2d & p_prev = xy_poses[i - 1];
      const Eigen::Vector2d & p_curr = xy_poses[i];

      double dxy = (p_curr - p_prev).norm();
      if (dxy < 1e-4) continue;

      int k = std::max(1, static_cast<int>(std::ceil(dxy / res)));
      double max_allowed_dz = k * max_step_height;

      int r_curr = 0, c_curr = 0;
      if (!graph->toGridIndex(p_curr.x(), p_curr.y(), r_curr, c_curr))
      {
        return false; // 越出地图边界
      }

      double next_z = cur_z;
      bool found_surface = false;
      double best_dz = std::numeric_limits<double>::max();

      for (uint32_t nid : graph->getSpatialCellNodes(r_curr, c_curr))
      {
        const auto & nd = graph->getNode(nid);
        if (nd.traversability >= 0.95f) continue;
        double dz = std::abs(nd.z - cur_z);
        if (dz <= max_allowed_dz && dz < best_dz)
        {
          best_dz = dz;
          next_z = nd.z;
          found_surface = true;
        }
      }

      // 若当前格无合法节点，向 8 邻域拓展搜索 (容忍离散化边缘漂移)
      if (!found_surface)
      {
        double min_dist_xy = res;
        for (int dr = -1; dr <= 1; ++dr)
        {
          for (int dc = -1; dc <= 1; ++dc)
          {
            if (dr == 0 && dc == 0) continue;
            for (uint32_t nid : graph->getSpatialCellNodes(r_curr + dr, c_curr + dc))
            {
              const auto & nd = graph->getNode(nid);
              if (nd.traversability >= 0.95f) continue;
              double dist_xy = std::hypot(nd.x - p_curr.x(), nd.y - p_curr.y());
              double dz = std::abs(nd.z - cur_z);
              if (dist_xy <= min_dist_xy && dz <= max_allowed_dz && dz < best_dz)
              {
                best_dz = dz;
                next_z = nd.z;
                found_surface = true;
              }
            }
          }
        }
      }

      if (!found_surface)
      {
        return false; // 悬崖断坎或跌落
      }

      // 途经中间支撑检查: 复用 cloud_graph_builder 的 hasIntermediateSupport
      int interp_steps = std::max(1, static_cast<int>(std::ceil(dxy / (res * 0.8))));
      if (interp_steps > 1)
      {
        for (int s = 1; s < interp_steps; ++s)
        {
          double t = static_cast<double>(s) / interp_steps;
          double sx = p_prev.x() + t * (p_curr.x() - p_prev.x());
          double sy = p_prev.y() + t * (p_curr.y() - p_prev.y());
          double sz = cur_z + t * (next_z - cur_z);

          int sr = 0, sc = 0;
          if (!graph->toGridIndex(sx, sy, sr, sc)) return false;

          bool supported = false;
          for (uint32_t nid : graph->getSpatialCellNodes(sr, sc))
          {
            const auto & nd = graph->getNode(nid);
            if (nd.traversability < 0.95f && std::abs(nd.z - sz) <= max_step_height)
            {
              supported = true;
              break;
            }
          }
          if (!supported)
          {
            return false; // 中间踩空，跨越悬崖/天井
          }
        }
      }

      cur_z = next_z;
    }

    // 2. 目标高度贴合度校验 (防规划到一楼去)
    if (!std::isnan(target_z))
    {
      const Eigen::Vector2d & p_end = xy_poses.back();
      double dist_to_goal = (p_end - target_xy).norm();
      if (dist_to_goal <= 0.50)
      {
        if (std::abs(cur_z - target_z) > 0.40)
        {
          return false; // 终点高度显著偏离全局目标所在高度
        }
      }
    }

    return true;
  }
};

} // namespace elevation_planner
