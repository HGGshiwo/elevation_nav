#include "elevation_map_loader/manifold_forest.hpp"
#include <queue>
#include <cmath>
#include <algorithm>
#include <ros/ros.h>

namespace elevation_map_loader
{

void ManifoldForestExtractor::extract(const grid_map::GridMapPclLoader & loader, grid_map::GridMap & out_map)
{
  const auto & raw_clusters = loader.getClusterHeightsWithingGridMapCell();
  const auto & size = out_map.getSize();
  const int rows = size(0);
  const int cols = size(1);
  const double res = out_map.getResolution();
  const float max_step = static_cast<float>(config_.max_step_height);
  const float max_slope = static_cast<float>(config_.max_slope_deg);
  const float max_stride = static_cast<float>(config_.max_stride_length);

  // 1. 整理每个单元格的纵向多层候选高程 (按 Z 升序排序，合并相近碎簇)
  std::vector<std::vector<std::vector<float>>> levels(rows, std::vector<std::vector<float>>(cols));
  int max_levels = 1;

  for (int r = 0; r < rows; ++r) {
    for (int c = 0; c < cols; ++c) {
      if (r < (int)raw_clusters.size() && c < (int)raw_clusters[r].size()) {
        auto h = raw_clusters[r][c];
        if (h.empty()) continue;
        std::sort(h.begin(), h.end());

        std::vector<float> cell_lvls;
        for (float z : h) {
          if (cell_lvls.empty() || (z - cell_lvls.back() > config_.vertical_cluster_gap)) {
            cell_lvls.push_back(z);
          }
        }
        levels[r][c] = cell_lvls;
        if ((int)cell_lvls.size() > max_levels) {
          max_levels = cell_lvls.size();
        }
      }
    }
  }

  max_levels = std::min(max_levels, config_.max_layers);
  if (max_levels < 1) max_levels = 1;

  // 2. 初始化 GridMap 独立图层
  for (int s = 0; s < max_levels; ++s) {
    std::string suf = "_" + std::to_string(s);
    out_map.add("elevation" + suf, NAN);
    out_map.add("step_height" + suf, 0.0f);
    out_map.add("slope" + suf, 0.0f);
    out_map.add("obstacle_mask" + suf, 3.0f);
    out_map.add("traversability" + suf, 1.0f);
  }
  out_map.add("elevation", NAN);
  out_map.add("clearance_0", NAN);

  // 3. 多源带权 Dijkstra 最小测地代价森林生长
  struct DijkstraNode {
    int layer;
    int r;
    int c;
    float z;
    float cost;
    bool operator>(const DijkstraNode & other) const { return cost > other.cost; }
  };
  std::priority_queue<DijkstraNode, std::vector<DijkstraNode>, std::greater<DijkstraNode>> pq;

  std::vector<std::vector<std::vector<float>>> layer_cost(
    max_levels, std::vector<std::vector<float>>(rows, std::vector<float>(cols, 1e9f)));

  std::vector<std::vector<int>> single_owner(rows, std::vector<int>(cols, -1));
  std::vector<std::vector<float>> single_best_cost(rows, std::vector<float>(cols, 1e9f));

  // A. 多层垂直重叠区锚定种子
  for (int r = 0; r < rows; ++r) {
    for (int c = 0; c < cols; ++c) {
      const auto & cell_lvls = levels[r][c];
      if (cell_lvls.size() >= 2) {
        for (int s = 0; s < std::min((int)cell_lvls.size(), max_levels); ++s) {
          std::string suf = "_" + std::to_string(s);
          out_map["elevation" + suf](r, c) = cell_lvls[s];
          layer_cost[s][r][c] = 0.0f;
          pq.push({s, r, c, cell_lvls[s], 0.0f});
        }
      }
    }
  }

  // 跨步邻域预计算
  const int stride_cells = std::max(1, static_cast<int>(std::ceil(max_stride / res)));
  struct NeighborOffset { int dr; int dc; float dist; };
  std::vector<NeighborOffset> nbr_offsets;
  for (int dr = -stride_cells; dr <= stride_cells; ++dr) {
    for (int dc = -stride_cells; dc <= stride_cells; ++dc) {
      if (dr == 0 && dc == 0) continue;
      float d = std::hypot(static_cast<float>(dr), static_cast<float>(dc)) * static_cast<float>(res);
      if (d <= max_stride) {
        nbr_offsets.push_back({dr, dc, d});
      }
    }
  }
  std::sort(nbr_offsets.begin(), nbr_offsets.end(), [](const NeighborOffset & a, const NeighborOffset & b) {
    return a.dist < b.dist;
  });

  // B. Dijkstra 最优流形生长与动态重划
  while (!pq.empty()) {
    DijkstraNode curr = pq.top();
    pq.pop();

    if (curr.cost > layer_cost[curr.layer][curr.r][curr.c]) continue;

    std::string suf = "_" + std::to_string(curr.layer);
    auto & elev = out_map["elevation" + suf];

    for (const auto & nbr : nbr_offsets) {
      int nr = curr.r + nbr.dr;
      int nc = curr.c + nbr.dc;
      if (nr < 0 || nr >= rows || nc < 0 || nc >= cols) continue;

      const auto & nbr_lvls = levels[nr][nc];
      if (nbr_lvls.empty()) continue;

      float best_z = NAN;
      float min_dz = max_step;
      for (float cz : nbr_lvls) {
        float dz = std::abs(cz - curr.z);
        float slope_deg = (nbr.dist > res * 1.5f) ? (std::atan(dz / nbr.dist) * 180.0f / M_PI) : 0.0f;
        if (dz <= min_dz && slope_deg <= max_slope) {
          min_dz = dz;
          best_z = cz;
        }
      }

      if (std::isnan(best_z)) continue;

      float edge_cost = std::sqrt(nbr.dist * nbr.dist + (2.0f * min_dz) * (2.0f * min_dz));
      float new_cost = curr.cost + edge_cost;

      if (new_cost < layer_cost[curr.layer][nr][nc]) {
        if (nbr_lvls.size() == 1) {
          int old_owner = single_owner[nr][nc];
          if (old_owner != -1 && old_owner != curr.layer) {
            if (new_cost < single_best_cost[nr][nc]) {
              out_map["elevation_" + std::to_string(old_owner)](nr, nc) = NAN;
              single_owner[nr][nc] = curr.layer;
              single_best_cost[nr][nc] = new_cost;
            } else {
              continue;
            }
          } else {
            single_owner[nr][nc] = curr.layer;
            single_best_cost[nr][nc] = new_cost;
          }
        }

        layer_cost[curr.layer][nr][nc] = new_cost;
        elev(nr, nc) = best_z;
        pq.push({curr.layer, nr, nc, best_z, new_cost});

        // 跨步微孔插值平滑
        if (nbr.dist > res * 1.5f) {
          int steps = std::max(std::abs(nbr.dr), std::abs(nbr.dc));
          for (int k = 1; k < steps; ++k) {
            int ir = curr.r + static_cast<int>(std::round(static_cast<float>(nbr.dr) * k / steps));
            int ic = curr.c + static_cast<int>(std::round(static_cast<float>(nbr.dc) * k / steps));
            if (ir >= 0 && ir < rows && ic >= 0 && ic < cols) {
              if (std::isnan(elev(ir, ic))) {
                float t = static_cast<float>(k) / steps;
                elev(ir, ic) = curr.z + t * (best_z - curr.z);
              }
            }
          }
        }
      }
    }
  }

  // C. 基础地面与开阔平原兜底填充
  auto & elev0 = out_map["elevation_0"];
  for (int r = 0; r < rows; ++r) {
    for (int c = 0; c < cols; ++c) {
      const auto & cell_lvls = levels[r][c];
      if (!cell_lvls.empty() && std::isnan(elev0(r, c))) {
        bool claimed_by_upper = false;
        for (int s = 1; s < max_levels; ++s) {
          if (!std::isnan(out_map["elevation_" + std::to_string(s)](r, c))) {
            claimed_by_upper = true;
            break;
          }
        }
        if (!claimed_by_upper) {
          elev0(r, c) = cell_lvls[0];
        }
      }
    }
  }

  // 4. 计算各层台阶、坡度与可通行性代价
  for (int s = 0; s < max_levels; ++s) {
    std::string suf = "_" + std::to_string(s);
    const auto & elev = out_map["elevation" + suf];
    auto & step_layer = out_map["step_height" + suf];
    auto & slope_layer = out_map["slope" + suf];
    auto & obs_layer = out_map["obstacle_mask" + suf];
    auto & trav_layer = out_map["traversability" + suf];

    for (int r = 0; r < rows; ++r) {
      for (int c = 0; c < cols; ++c) {
        float z0 = elev(r, c);
        if (std::isnan(z0)) continue;

        float max_diff = 0.0f;
        for (int dr = -1; dr <= 1; ++dr) {
          for (int dc = -1; dc <= 1; ++dc) {
            if (dr == 0 && dc == 0) continue;
            int nr = r + dr;
            int nc = c + dc;
            if (nr >= 0 && nr < rows && nc >= 0 && nc < cols) {
              float nz = elev(nr, nc);
              if (!std::isnan(nz)) {
                max_diff = std::max(max_diff, std::abs(nz - z0));
              }
            }
          }
        }
        step_layer(r, c) = max_diff;

        float gx = 0.0f, gy = 0.0f;
        if (c > 0 && c < cols - 1 && !std::isnan(elev(r, c - 1)) && !std::isnan(elev(r, c + 1))) {
          gx = (elev(r, c + 1) - elev(r, c - 1)) / (2.0f * res);
        }
        if (r > 0 && r < rows - 1 && !std::isnan(elev(r - 1, c)) && !std::isnan(elev(r + 1, c))) {
          gy = (elev(r + 1, c) - elev(r - 1, c)) / (2.0f * res);
        }
        float slope_deg = std::atan(std::sqrt(gx * gx + gy * gy)) * 180.0f / M_PI;
        slope_layer(r, c) = slope_deg;

        if (max_diff > max_step || slope_deg > max_slope) {
          obs_layer(r, c) = 1.0f;
          trav_layer(r, c) = 1.0f;
        } else {
          obs_layer(r, c) = 0.0f;
          float cost = (max_diff / max_step) * 0.5f + (slope_deg / max_slope) * 0.5f;
          trav_layer(r, c) = std::min(1.0f, std::max(0.0f, cost));
        }
      }
    }
  }

  // 5. 底层净空干涉检测
  if (out_map.exists("elevation_1")) {
    const auto & elev0 = out_map["elevation_0"];
    const auto & elev1 = out_map["elevation_1"];
    auto & clr = out_map["clearance_0"];
    auto & obs0 = out_map["obstacle_mask_0"];

    for (int r = 0; r < rows; ++r) {
      for (int c = 0; c < cols; ++c) {
        float z0 = elev0(r, c);
        float z1 = elev1(r, c);
        if (!std::isnan(z0) && !std::isnan(z1)) {
          float clearance = z1 - z0;
          clr(r, c) = clearance;
          if (clearance > 0.15f && clearance < config_.dog_height) {
            obs0(r, c) = 2.0f;
          }
        }
      }
    }
  }

  // 6. 向下兼容标准单层
  if (!out_map.exists("elevation")) out_map.add("elevation", out_map["elevation_0"]);
  else out_map["elevation"] = out_map["elevation_0"];
  if (!out_map.exists("step_height")) out_map.add("step_height", out_map["step_height_0"]);
  else out_map["step_height"] = out_map["step_height_0"];
  if (!out_map.exists("slope")) out_map.add("slope", out_map["slope_0"]);
  else out_map["slope"] = out_map["slope_0"];
  if (!out_map.exists("obstacle_mask")) out_map.add("obstacle_mask", out_map["obstacle_mask_0"]);
  else out_map["obstacle_mask"] = out_map["obstacle_mask_0"];
  if (!out_map.exists("traversability")) out_map.add("traversability", out_map["traversability_0"]);
  else out_map["traversability"] = out_map["traversability_0"];

  ROS_INFO("[ManifoldForestExtractor] Built %d non-conflicting elevation surfaces", max_levels);
}

} // namespace elevation_map_loader
