#include "elevation_planner_core/cloud_graph_builder.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sstream>
#include <algorithm>
#include <cmath>
#include <vector>

namespace elevation_planner
{

namespace
{
// 在查询点 3x3 邻域格内查找最近节点 (无视可通行性, 供诊断接口使用)
bool findNearestNodeAny(const ManifoldGraph & graph, double qx, double qy, double qz, uint32_t & out_id)
{
  int r = 0, c = 0;
  if (!graph.toGridIndex(qx, qy, r, c)) return false;
  double best_d = 1e9;
  bool found = false;
  for (int dr = -1; dr <= 1; ++dr) {
    int nr = r + dr;
    if (nr < 0 || nr >= graph.getRows()) continue;
    for (int dc = -1; dc <= 1; ++dc) {
      int nc = c + dc;
      if (nc < 0 || nc >= graph.getCols()) continue;
      for (uint32_t nid : graph.getSpatialCellNodes(nr, nc)) {
        const auto & nd = graph.getNode(nid);
        double d = std::hypot(nd.x - qx, nd.y - qy) + std::abs(nd.z - qz);
        if (d < best_d) {
          best_d = d;
          out_id = nid;
          found = true;
        }
      }
    }
  }
  return found;
}

// 机体胶囊扫掠硬碰撞: 沿 u->v 线段采样, 硬半径内出现 "本行走层" 的禁行节点即剐蹭。
// 垂直方向仅做选层切一刀: 只算 [sz - foot_clearance, sz + dog_height] 内的禁行节点——
// 脚平面以下的禁行节点是被踩的楼梯结构/其他层, 不参与水平胶囊判定
// (建边丢弃与诊断报因共用同一判据, 保证口径一致)
bool segmentHitsHardInflation(const ManifoldGraph & graph, const GraphNode & u, const GraphNode & v,
                              double hard_radius, double dog_height, double foot_clearance, double res)
{
  const int rad = std::max(1, static_cast<int>(std::ceil(hard_radius / res)));
  const float seg_len = std::hypot(v.x - u.x, v.y - u.y);
  const int steps = std::max(1, static_cast<int>(std::ceil(seg_len / (res * 0.5))));
  for (int s = 0; s <= steps; ++s) {
    float t = static_cast<float>(s) / steps;
    float sx = u.x + t * (v.x - u.x);
    float sy = u.y + t * (v.y - u.y);
    float sz = u.z + t * (v.z - u.z);
    int r = 0, c = 0;
    if (!graph.toGridIndex(sx, sy, r, c)) return true;
    for (int dr = -rad; dr <= rad; ++dr) {
      for (int dc = -rad; dc <= rad; ++dc) {
        if (std::hypot(dr, dc) * res > hard_radius) continue;
        for (uint32_t nid : graph.getSpatialCellNodes(r + dr, c + dc)) {
          const auto & nd = graph.getNode(nid);
          if (nd.traversability >= 0.95f &&
              nd.z >= sz - foot_clearance && nd.z <= sz + dog_height) return true;
        }
      }
    }
  }
  return false;
}

std::string fmt2(double v) { return std::to_string(v).substr(0, 4); }
} // namespace

bool CloudGraphBuilder::buildFromROSMsg(const sensor_msgs::PointCloud2 & cloud_msg, ManifoldGraph & out_graph)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(cloud_msg, *cloud);
  return buildFromPointCloud(cloud, out_graph);
}

bool CloudGraphBuilder::buildFromPointCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & cloud,
                                           ManifoldGraph & out_graph)
{
  ColumnTable table;
  if (!buildColumnTable(cloud, table)) return false;
  return buildGraphFromColumnTable(table, out_graph);
}

bool CloudGraphBuilder::buildColumnTable(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & cloud,
                                        ColumnTable & out_table,
                                        const GridExtent * aligned_extent)
{
  if (!cloud || cloud->empty()) return false;

  // 1. 体素降采样 (0.05m 网格保持几何边缘且去重)
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::VoxelGrid<pcl::PointXYZ> vox;
  vox.setInputCloud(cloud);
  vox.setLeafSize(0.05f, 0.05f, 0.05f);
  vox.filter(*filtered_cloud);

  if (filtered_cloud->empty()) return false;

  // 1.5 残影点过滤 (SOR): 悬浮稀疏点串的近邻距离远大于真实表面点, 统计上可分离。
  //     不过滤时残影串会被柱状聚类误判为水平表面, 撑爆上方净空判定
  if (config_.sor_std_mul < 1e6 && config_.sor_mean_k > 0) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr sor_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(filtered_cloud);
    sor.setMeanK(config_.sor_mean_k);
    sor.setStddevMulThresh(config_.sor_std_mul);
    sor.filter(*sor_cloud);
    filtered_cloud = sor_cloud;
  }

  if (filtered_cloud->empty()) return false;

  // 2. 栅格几何: 对齐模式直接沿用全局图几何 (格对齐融合的前提), 否则按点云包围盒自算
  double res = config_.resolution;
  double min_x = 0.0, min_y = 0.0;
  int rows = 0, cols = 0;

  if (aligned_extent && aligned_extent->valid()) {
    res = aligned_extent->resolution;
    min_x = aligned_extent->min_x;
    min_y = aligned_extent->min_y;
    rows = aligned_extent->rows;
    cols = aligned_extent->cols;
  } else {
    float bbox_min_x = std::numeric_limits<float>::max();
    float bbox_max_x = std::numeric_limits<float>::lowest();
    float bbox_min_y = std::numeric_limits<float>::max();
    float bbox_max_y = std::numeric_limits<float>::lowest();
    for (const auto & pt : filtered_cloud->points) {
      if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;
      bbox_min_x = std::min(bbox_min_x, pt.x);
      bbox_max_x = std::max(bbox_max_x, pt.x);
      bbox_min_y = std::min(bbox_min_y, pt.y);
      bbox_max_y = std::max(bbox_max_y, pt.y);
    }
    min_x = bbox_min_x;
    min_y = bbox_min_y;
    rows = static_cast<int>(std::ceil((bbox_max_x - bbox_min_x) / res)) + 1;
    cols = static_cast<int>(std::ceil((bbox_max_y - bbox_min_y) / res)) + 1;
  }
  if (rows <= 0 || cols <= 0) return false;

  out_table.extent = GridExtent{res, min_x, min_y, rows, cols};
  out_table.cells.assign(static_cast<size_t>(rows * cols), {});

  // 3. 空间栅格投影桶 (单个 O(N) 遍历)
  std::vector<std::vector<float>> column_heights(static_cast<size_t>(rows * cols));
  for (const auto & pt : filtered_cloud->points) {
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;
    int r = static_cast<int>(std::floor((pt.x - min_x) / res));
    int c = static_cast<int>(std::floor((pt.y - min_y) / res));
    if (r >= 0 && r < rows && c >= 0 && c < cols) {
      column_heights[static_cast<size_t>(r * cols + c)].push_back(pt.z);
    }
  }

  // 4. 单柱多曲面聚类 (提取一楼、二楼及楼梯踏面), 簇按高度升序写入柱表
  for (int r = 0; r < rows; ++r) {
    for (int c = 0; c < cols; ++c) {
      auto & heights = column_heights[static_cast<size_t>(r * cols + c)];
      if (heights.size() < static_cast<size_t>(config_.min_cluster_points)) continue;

      std::sort(heights.begin(), heights.end());

      std::vector<ColumnSurface> surfaces;
      float cur_bottom = heights[0];
      float cur_top = heights[0];
      int cur_count = 1;

      for (size_t k = 1; k < heights.size(); ++k) {
        if (heights[k] - heights[k - 1] <= config_.cluster_height_diff) {
          cur_top = heights[k];
          cur_count++;
        } else {
          surfaces.push_back({cur_top, cur_bottom, cur_count});
          cur_bottom = heights[k];
          cur_top = heights[k];
          cur_count = 1;
        }
      }
      surfaces.push_back({cur_top, cur_bottom, cur_count});

      out_table.cells[static_cast<size_t>(r * cols + c)] = std::move(surfaces);
    }
  }
  return true;
}

ColumnTable fuseColumnTables(const ColumnTable & prior,
                             const ColumnTable & observed,
                             double same_surface_tol,
                             int boundary_ring)
{
  // 先验缺失或分辨率不一致 (无法逐格对齐) 时, 退化为纯观测
  if (prior.empty() || observed.empty()) return observed;
  if (std::abs(prior.extent.resolution - observed.extent.resolution) > 1e-9) return observed;

  const GridExtent & oe = observed.extent;
  const GridExtent & pe = prior.extent;

  ColumnTable fused = observed; // extent 沿用观测表, 逐格覆写融合结果

  for (int r = 0; r < oe.rows; ++r) {
    for (int c = 0; c < oe.cols; ++c) {
      // 窗口最外 boundary_ring 圈强制先验, 保证出窗处与全局图衔接
      const bool boundary = (r < boundary_ring || r >= oe.rows - boundary_ring ||
                             c < boundary_ring || c >= oe.cols - boundary_ring);

      // 观测格中心世界坐标 -> 先验表格索引
      const double wx = oe.min_x + (r + 0.5) * oe.resolution;
      const double wy = oe.min_y + (c + 0.5) * oe.resolution;
      int pr = static_cast<int>(std::floor((wx - pe.min_x) / pe.resolution));
      int pc = static_cast<int>(std::floor((wy - pe.min_y) / pe.resolution));
      const bool has_prior = pr >= 0 && pr < pe.rows && pc >= 0 && pc < pe.cols &&
                             !prior.cells[static_cast<size_t>(pr * pe.cols + pc)].empty();
      if (!has_prior) continue; // 无先验: 保留观测原样

      const auto & pri = prior.cells[static_cast<size_t>(pr * pe.cols + pc)];
      if (boundary) {
        fused.cells[static_cast<size_t>(r * oe.cols + c)] = pri;
        continue;
      }

      // 双指针并集合并 (两列表均按 z 升序)
      const auto & obs = observed.cells[static_cast<size_t>(r * oe.cols + c)];
      std::vector<ColumnSurface> merged;
      merged.reserve(obs.size() + pri.size());
      size_t i = 0, j = 0;
      while (i < obs.size() || j < pri.size()) {
        if (j >= pri.size()) { merged.push_back(obs[i++]); continue; }
        if (i >= obs.size()) { merged.push_back(pri[j++]); continue; }
        const ColumnSurface & a = obs[i];
        const ColumnSurface & b = pri[j];
        if (std::abs(static_cast<double>(a.z_top) - static_cast<double>(b.z_top)) <= same_surface_tol) {
          ColumnSurface m = a; // 同一物理面: 几何取观测 (反映当前状态)
          m.count = std::max(a.count, b.count); // 一次稀疏观测不推翻先验支撑面
          merged.push_back(m);
          ++i; ++j;
        } else if (b.z_top < a.z_top) {
          merged.push_back(b); ++j; // 仅先验: 保留 (雷达扫不到的脚下支撑面)
        } else {
          merged.push_back(a); ++i; // 仅观测: 新增 (新踏面/新障碍)
        }
      }
      fused.cells[static_cast<size_t>(r * oe.cols + c)] = std::move(merged);
    }
  }
  return fused;
}

bool CloudGraphBuilder::buildGraphFromColumnTable(const ColumnTable & table, ManifoldGraph & out_graph)
{
  const GridExtent & ext = table.extent;
  if (!ext.valid() || table.cells.size() != static_cast<size_t>(ext.rows * ext.cols)) return false;

  const double res = ext.resolution;
  const double min_x = ext.min_x;
  const double min_y = ext.min_y;
  const int rows = ext.rows;
  const int cols = ext.cols;

  out_graph.initSpatialGrid(res, min_x, min_y, rows, cols); // 内部 clear, 支持重复建图
  const auto & cell_surfaces = table.cells;

  // 机体足印侧向碰撞膨胀: 扫描足印半径内 "垂直范围跨越踏步极限" 的结构 (墙体/台沿,
  // 即从本层踏面高度附近向上生长、高到迈不上去的竖直结构), 硬半径内硬阻挡, 外围软代价
  auto inflatedTraversability = [&](int r, int c, float tread_z, bool & lateral_hard) -> float {
    const int rad = std::max(1, static_cast<int>(std::ceil(config_.footprint_radius / res)));
    float worst = 0.0f;
    lateral_hard = false;
    const float soft_band = std::max(1e-3f, static_cast<float>(config_.footprint_radius - config_.body_hard_radius));
    for (int dr = -rad; dr <= rad; ++dr) {
      int nr = r + dr;
      if (nr < 0 || nr >= rows) continue;
      for (int dc = -rad; dc <= rad; ++dc) {
        int nc = c + dc;
        if (nc < 0 || nc >= cols) continue;
        // double 计算: float 乘法会让 "整 2 格 = 0.2m" 恰好落在硬半径边界时因舍入差 3e-9 漏成软代价
        const double d = std::hypot(dr, dc) * res;
        if (d > config_.footprint_radius) continue;

        for (const auto & S : cell_surfaces[static_cast<size_t>(nr * cols + nc)]) {
          if (S.count < config_.min_cluster_points) continue;
          // 侧向障碍判据: 结构底面贴近本层踏面 (根长在行走平面附近, 排除悬空板),
          // 顶面超过 "攀爬包络" 才判墙。攀爬包络: 从本踏面出发每前进一格 (resolution)
          // 最多再爬一级 max_step_height —— 与 8 邻域建边 `|dz| <= max_step_height` 的
          // 运动学口径保持一致, 因此格距 k = max(|dr|,|dc|) 的结构顶面在
          // tread_z + k * max_step_height 以内, 视为可逐级攀爬的楼梯本体, 不判墙。
          // 注意阈值必须随格距缩放: max_step_height 的物理含义是 "相邻两个落足点之间"
          // 的单步抬腿极限, 仅在 k=1 (贴身) 时等于踏步极限线 tread_z + 0.25;
          // 若对远处格子沿用固定 +0.25, 坡度 > atan(max_step_height / body_hard_radius)
          // (约 51°) 的楼梯段会因累计高差越线被误判成墙, 建边扫掠随之剪光周边全部边,
          // 整段楼梯断连 (实测 map-segment 二楼楼梯 A-B 两点度数归零)。k=1 时本判据
          // 与原单级踢面判据完全一致, 贴身墙/转角内切防护不受影响。
          const int k = std::max(std::abs(dr), std::abs(dc));
          const float top_limit = tread_z + static_cast<float>(k * config_.max_step_height);
          if (!(S.z_bottom < tread_z + config_.max_step_height &&
                S.z_top > top_limit)) continue;

          if (d <= config_.body_hard_radius) { lateral_hard = true; return 1.0f; } // 硬阻挡
          float soft = 0.7f * (config_.footprint_radius - d) / soft_band; // 距离衰减软代价 (上限0.7, 低于前端0.8禁行显示阈值)
          worst = std::max(worst, soft);
        }
      }
    }
    return worst;
  };

  for (int r = 0; r < rows; ++r) {
    for (int c = 0; c < cols; ++c) {
      const auto & surfaces = cell_surfaces[static_cast<size_t>(r * cols + c)];
      if (surfaces.empty()) continue;

      // 提取踏面节点并评估净空 (假设均可通行，不设踏面坡度截断)
      for (size_t s = 0; s < surfaces.size(); ++s) {
        if (surfaces[s].count < config_.min_cluster_points) continue;

        float headroom = 3.0f; // 默认室外无上顶
        if (s + 1 < surfaces.size()) {
          headroom = surfaces[s + 1].z_bottom - surfaces[s].z_top;
        }

        GraphNode node;
        node.x = static_cast<float>(min_x + (r + 0.5) * res);
        node.y = static_cast<float>(min_y + (c + 0.5) * res);
        node.z = surfaces[s].z_top;
        node.row = r;
        node.col = c;
        node.layer_id = static_cast<int>(s);
        node.headroom = headroom;

        // 顶棚净空不足判定 (满足身高净空即均可通行)
        if (headroom < config_.dog_height) {
          node.traversability = 1.0f; // 标记顶头不可通过
          node.flags |= node_flags::BLOCK_HEADROOM;
        } else {
          node.traversability = 0.0f;
        }

        // 机体足印膨胀: 贴墙节点标记阻挡/软代价, 使 A* 走走廊中线、转角外侧绕行
        if (node.traversability < 0.95f) {
          bool lateral_hard = false;
          float inf = inflatedTraversability(r, c, node.z, lateral_hard);
          node.traversability = std::max(node.traversability, inf);
          if (lateral_hard) node.flags |= node_flags::BLOCK_LATERAL;
        }

        out_graph.addNode(node);
      }
    }
  }

  // 5. 拓扑邻居建边: 两遍式 "短边主干 + 长边按需架桥"
  //    第一遍只建 8 邻域短边 (连通性主干); 第二遍在跨步窗口内为仍不连通的区域架长边桥,
  //    使长边仅出现在短边链无法连通的位置 (锯齿高差/点云缺失/窄缝), 消除平地冗余直连边
  size_t total_nodes = out_graph.numNodes();

  // 跨步搜索窗口: 按机器狗实际跨步极限换算为栅格半径, 让 max_stride_length 真正约束建边
  const int stride_cells = std::max(1, static_cast<int>(std::ceil(config_.max_stride_length / res)));

  // 并查集: 记录当前连通性, 长边只在两端尚不连通时才架设 (最小桥集)
  std::vector<uint32_t> uf_parent(total_nodes);
  std::vector<uint32_t> uf_size(total_nodes, 1);
  for (size_t i = 0; i < total_nodes; ++i) uf_parent[i] = static_cast<uint32_t>(i);
  auto uf_find = [&](uint32_t x) -> uint32_t {
    while (uf_parent[x] != x) { uf_parent[x] = uf_parent[uf_parent[x]]; x = uf_parent[x]; }
    return x;
  };
  auto uf_union = [&](uint32_t a, uint32_t b) {
    a = uf_find(a); b = uf_find(b);
    if (a == b) return;
    if (uf_size[a] < uf_size[b]) std::swap(a, b);
    uf_parent[b] = a;
    uf_size[a] += uf_size[b];
  };

  // 途经支撑检查: 跨多格的长边, 线段途经的每个栅格必须存在处于通行高度带内的可通行节点,
  // 防止 A* 借长边跨越楼梯井/空洞 (短邻接边不受影响, 保持原有拓扑)
  auto hasIntermediateSupport = [&](const GraphNode & u, const GraphNode & v) -> bool {
    const float seg_len = std::hypot(v.x - u.x, v.y - u.y);
    const int steps = std::max(1, static_cast<int>(std::ceil(seg_len / (res * 0.5))));
    for (int s = 1; s < steps; ++s) { // 端点即 u/v 自身, 只查中间点
      float t = static_cast<float>(s) / steps;
      float sx = u.x + t * (v.x - u.x);
      float sy = u.y + t * (v.y - u.y);
      float sz = u.z + t * (v.z - u.z);
      int r = 0, c = 0;
      if (!out_graph.toGridIndex(sx, sy, r, c)) return false;
      bool supported = false;
      for (uint32_t nid : out_graph.getSpatialCellNodes(r, c)) {
        const auto & nd = out_graph.getNode(nid);
        if (nd.traversability < 0.95f && std::abs(nd.z - sz) <= config_.max_step_height) { supported = true; break; }
      }
      if (!supported) return false;
    }
    return true;
  };

  // 机体胶囊扫掠检查: 沿 u->v 线段采样, 硬半径内出现本行走层禁行节点则该步会剐蹭, 丢弃此边
  auto sweepHardCollision = [&](const GraphNode & u, const GraphNode & v) -> bool {
    return segmentHitsHardInflation(out_graph, u, v,
                                    config_.body_hard_radius, config_.dog_height,
                                    config_.foot_clearance, res);
  };

  // 机体扫掠软代价: 线段附近足印半径内的软膨胀区越多, 该边代价越高 (引导路径远离墙体)
  auto sweepSoftCost = [&](const GraphNode & u, const GraphNode & v) -> float {
    const int rad = std::max(1, static_cast<int>(std::ceil(config_.footprint_radius / res)));
    const float seg_len = std::hypot(v.x - u.x, v.y - u.y);
    const int steps = std::max(1, static_cast<int>(std::ceil(seg_len / (res * 0.5))));
    float worst = 0.0f;
    for (int s = 0; s <= steps; ++s) {
      float t = static_cast<float>(s) / steps;
      float sx = u.x + t * (v.x - u.x);
      float sy = u.y + t * (v.y - u.y);
      float sz = u.z + t * (v.z - u.z);
      int r = 0, c = 0;
      if (!out_graph.toGridIndex(sx, sy, r, c)) continue;
      for (int dr = -rad; dr <= rad; ++dr) {
        for (int dc = -rad; dc <= rad; ++dc) {
          if (std::hypot(dr, dc) * res > config_.footprint_radius) continue;
          for (uint32_t nid : out_graph.getSpatialCellNodes(r + dr, c + dc)) {
            const auto & nd = out_graph.getNode(nid);
            if (nd.traversability > 0.05f && nd.traversability < 0.95f &&
                nd.z >= sz - config_.foot_clearance && nd.z <= sz + config_.dog_height) {
              worst = std::max(worst, nd.traversability);
            }
          }
        }
      }
    }
    return worst;
  };

  // 统一建边: 全部物理检查 + 代价计算, 写入双向边并同步并查集
  auto addEdgeChecked = [&](uint32_t u_id, uint32_t v_id) -> bool {
    const GraphNode & u = out_graph.getNode(u_id);
    const GraphNode & v = out_graph.getNode(v_id);

    if (sweepHardCollision(u, v)) return false;

    float dz = std::abs(u.z - v.z);
    float dxy = std::hypot(u.x - v.x, u.y - v.y);
    // 边代价：3D欧氏位移 + 高差势能惩罚 (鼓励走平地，但楼梯绝对连通)
    float edge_len = std::sqrt(dxy * dxy + dz * dz);
    float cost = edge_len + 2.0f * dz + 0.5f * (u.traversability + v.traversability);
    // 扫掠区软代价: 边身靠近墙体时提高代价, 引导 A* 居中绕行
    cost += static_cast<float>(config_.sweep_penalty_weight) * sweepSoftCost(u, v);

    out_graph.addEdge(u_id, v_id, cost);
    out_graph.addEdge(v_id, u_id, cost); // 双向边, A* 两个方向均可通行
    uf_union(u_id, v_id);
    return true;
  };

  // ---- 第一遍: 8 邻域短边 (连通性主干) ----
  for (uint32_t u_id = 0; u_id < total_nodes; ++u_id) {
    const auto & u = out_graph.getNode(u_id);
    if (u.traversability >= 0.95f) continue;

    for (int dr = -1; dr <= 1; ++dr) {
      for (int dc = -1; dc <= 1; ++dc) {
        if (dr == 0 && dc == 0) continue;
        int nr = u.row + dr;
        int nc = u.col + dc;
        if (nr < 0 || nr >= rows || nc < 0 || nc >= cols) continue;

        for (uint32_t v_id : out_graph.getSpatialCellNodes(nr, nc)) {
          if (v_id == u_id || v_id < u_id) continue; // 每无序对仅处理一次 (addEdgeChecked 已写双向)
          const auto & v = out_graph.getNode(v_id);
          if (v.traversability >= 0.95f) continue;

          // 仅约束四足物理极限: 垂直行程 (短边水平距离必然在跨步极限内)
          if (std::abs(u.z - v.z) > config_.max_step_height) continue;

          addEdgeChecked(u_id, v_id);
        }
      }
    }
  }

  // ---- 第二遍: 跨步窗口长边按需架桥 (仅当两端尚不连通, 省去无谓的扫掠检查) ----
  for (uint32_t u_id = 0; u_id < total_nodes; ++u_id) {
    const auto & u = out_graph.getNode(u_id);
    if (u.traversability >= 0.95f) continue;

    for (int dr = -stride_cells; dr <= stride_cells; ++dr) {
      for (int dc = -stride_cells; dc <= stride_cells; ++dc) {
        if (dr == 0 && dc == 0) continue;
        int nr = u.row + dr;
        int nc = u.col + dc;
        if (nr < 0 || nr >= rows || nc < 0 || nc >= cols) continue;

        for (uint32_t v_id : out_graph.getSpatialCellNodes(nr, nc)) {
          if (v_id == u_id) continue;
          const auto & v = out_graph.getNode(v_id);
          if (v.traversability >= 0.95f) continue;

          float dz = std::abs(u.z - v.z);
          if (dz > config_.max_step_height) continue;

          float dxy = std::hypot(u.x - v.x, u.y - v.y);
          if (dxy > config_.max_stride_length) continue;

          // 连通性门控: 两端已可达则长边无增量价值, 直接跳过 (最便宜的检查放最前)。
          // 不设 dxy 下限: 同格跨层 (dxy=0) 等短距离对在第一遍被扫掠/高差拒绝后,
          // 此处仍需作为桥候选参与连通
          if (uf_find(u_id) == uf_find(v_id)) continue;

          // 长边必须全线有落脚支撑, 防止借桥跨洞
          if (!hasIntermediateSupport(u, v)) continue;

          addEdgeChecked(u_id, v_id);
        }
      }
    }
  }

  // 6. 压平为连续 CSR 结构
  out_graph.finalizeCSR();
  return true;
}

void CloudGraphBuilder::toPointCloudMsg(const ManifoldGraph & graph,
                                        const std::string & frame_id,
                                        sensor_msgs::PointCloud2 & out_cloud)
{
  pcl::PointCloud<pcl::PointXYZI> pc;
  pc.points.reserve(graph.numNodes());

  for (size_t i = 0; i < graph.numNodes(); ++i) {
    const auto & nd = graph.getNode(static_cast<uint32_t>(i));
    pcl::PointXYZI pt;
    pt.x = nd.x;
    pt.y = nd.y;
    pt.z = nd.z;
    pt.intensity = nd.traversability;
    pc.points.push_back(pt);
  }

  pcl::toROSMsg(pc, out_cloud);
  out_cloud.header.frame_id = frame_id;
  out_cloud.header.stamp = ros::Time::now();
}

void CloudGraphBuilder::toMarkerArray(const ManifoldGraph & graph,
                                      const std::string & frame_id,
                                      visualization_msgs::MarkerArray & out_markers,
                                      size_t max_edges)
{
  out_markers.markers.clear();
  visualization_msgs::Marker line_list;
  line_list.header.frame_id = frame_id;
  line_list.header.stamp = ros::Time::now();
  line_list.ns = "manifold_edges";
  line_list.id = 0;
  line_list.type = visualization_msgs::Marker::LINE_LIST;
  line_list.action = visualization_msgs::Marker::ADD;
  line_list.scale.x = 0.02; // 线宽 2cm
  line_list.color.r = 0.1f;
  line_list.color.g = 0.8f;
  line_list.color.b = 0.2f;
  line_list.color.a = 0.6f;

  size_t added = 0;
  for (size_t i = 0; i < graph.numNodes() && added < max_edges; ++i) {
    uint16_t count = 0;
    const auto * edges = graph.getEdges(static_cast<uint32_t>(i), count);
    const auto & u = graph.getNode(static_cast<uint32_t>(i));
    for (uint16_t k = 0; k < count && added < max_edges; ++k) {
      if (edges[k].target_id < i) continue; // 无向图避免重复画两次
      const auto & v = graph.getNode(edges[k].target_id);

      geometry_msgs::Point p1, p2;
      p1.x = u.x; p1.y = u.y; p1.z = u.z;
      p2.x = v.x; p2.y = v.y; p2.z = v.z;
      line_list.points.push_back(p1);
      line_list.points.push_back(p2);
      added++;
    }
  }

  out_markers.markers.push_back(line_list);
}

std::string CloudGraphBuilder::diagnoseEdge(const ManifoldGraph & graph,
                                            double x1, double y1, double z1,
                                            double x2, double y2, double z2) const
{
  std::stringstream ss;

  uint32_t u_id = 0, v_id = 0;
  bool found_u = findNearestNodeAny(graph, x1, y1, z1, u_id);
  bool found_v = findNearestNodeAny(graph, x2, y2, z2, v_id);

  if (!found_u || !found_v) {
    ss << "{\"status\":\"error\",\"message\":\"Failed to find nearest graph nodes for one or both coordinates\"}";
    return ss.str();
  }

  const auto & u = graph.getNode(u_id);
  const auto & v = graph.getNode(v_id);

  bool connected = false;
  float edge_cost = 0.0f;
  uint16_t edge_cnt = 0;
  const GraphEdge * edges = graph.getEdges(u_id, edge_cnt);
  for (uint16_t i = 0; i < edge_cnt; ++i) {
    if (edges[i].target_id == v_id) {
      connected = true;
      edge_cost = edges[i].cost;
      break;
    }
  }

  double dxy = std::hypot(u.x - v.x, u.y - v.y);
  double dz = std::abs(u.z - v.z);
  double slope_deg = (dxy > 1e-4) ? (std::atan2(dz, dxy) * 180.0 / M_PI) : 90.0;

  int dr = std::abs(u.row - v.row);
  int dc = std::abs(u.col - v.col);

  // 节点禁行原因描述 (供两点诊断引用)
  auto nodeBlockReason = [&](const GraphNode & n) -> std::string {
    if (n.traversability < 0.8f) return "";
    if (n.flags & node_flags::BLOCK_HEADROOM) return "头顶净空不足 " + fmt2(n.headroom) + "m < 机体高度 " + fmt2(config_.dog_height) + "m";
    if (n.flags & node_flags::BLOCK_LATERAL) return "侧向墙体落入机体硬半径 " + fmt2(config_.body_hard_radius) + "m 内";
    return "禁行节点";
  };

  std::vector<std::string> reasons;
  const int stride_cells = std::max(1, static_cast<int>(std::ceil(config_.max_stride_length / config_.resolution)));
  if (!connected) {
    std::string ra = nodeBlockReason(u);
    std::string rb = nodeBlockReason(v);
    if (!ra.empty()) reasons.push_back("方块A禁行: " + ra);
    if (!rb.empty()) reasons.push_back("方块B禁行: " + rb);
    if (dr > stride_cells || dc > stride_cells) reasons.push_back("超出跨步搜索窗口 (栅格距离 dr=" + std::to_string(dr) + ", dc=" + std::to_string(dc) + " > 窗口半径 " + std::to_string(stride_cells) + " 格)");
    if (dz > config_.max_step_height) reasons.push_back("台阶高差超限 (Δz=" + fmt2(dz) + "m > 抬腿极限 " + fmt2(config_.max_step_height) + "m)");
    if (dxy > config_.max_stride_length) reasons.push_back("跨步距离超限 (Δxy=" + fmt2(dxy) + "m > 步长极限 " + fmt2(config_.max_stride_length) + "m)");
    if (reasons.empty()) {
      // 稀疏图常态: 两点无直连边但同连通域, 经多跳短边可达 (A* 可正常规划, 非异常)
      std::vector<uint8_t> vis(graph.numNodes(), 0);
      std::vector<uint32_t> stack{u_id};
      vis[u_id] = 1;
      bool reachable = false;
      while (!stack.empty() && !reachable) {
        uint32_t x = stack.back(); stack.pop_back();
        uint16_t ec = 0;
        const auto * es = graph.getEdges(x, ec);
        for (uint16_t k = 0; k < ec; ++k) {
          uint32_t t = es[k].target_id;
          if (t == v_id) { reachable = true; break; }
          if (!vis[t]) { vis[t] = 1; stack.push_back(t); }
        }
      }
      if (reachable) {
        reasons.push_back("两点在同一连通域内, 经多跳短边链可达 (稀疏图无直连边属正常, A* 可正常规划)");
      } else if (segmentHitsHardInflation(graph, u, v, config_.body_hard_radius, config_.dog_height,
                                          config_.foot_clearance, config_.resolution)) {
        reasons.push_back("机体扫掠碰撞: 两端虽可站立, 但连线剐蹭硬膨胀禁行区 (典型为转角内切)");
      } else {
        reasons.push_back("两踏面不属于同一层簇或不在搜索窗口内");
      }
    }
  } else {
    reasons.push_back("满足四足运动学工作空间约束, 流形图中存在活跃邻边");
  }

  ss << "{"
     << "\"status\":\"ok\","
     << "\"connected\":" << (connected ? "true" : "false") << ","
     << "\"cost\":" << edge_cost << ","
     << "\"node_a\":{\"id\":" << u_id << ",\"x\":" << u.x << ",\"y\":" << u.y << ",\"z\":" << u.z << ",\"row\":" << u.row << ",\"col\":" << u.col << ",\"layer\":" << u.layer_id << ",\"headroom\":" << u.headroom << ",\"traversability\":" << u.traversability << "},"
     << "\"node_b\":{\"id\":" << v_id << ",\"x\":" << v.x << ",\"y\":" << v.y << ",\"z\":" << v.z << ",\"row\":" << v.row << ",\"col\":" << v.col << ",\"layer\":" << v.layer_id << ",\"headroom\":" << v.headroom << ",\"traversability\":" << v.traversability << "},"
     << "\"metrics\":{\"dxy\":" << dxy << ",\"dz\":" << dz << ",\"slope_deg\":" << slope_deg << ",\"dr\":" << dr << ",\"dc\":" << dc << "},"
     << "\"limits\":{\"max_step_height\":" << config_.max_step_height << ",\"max_stride_length\":" << config_.max_stride_length << ",\"dog_height\":" << config_.dog_height << ",\"body_hard_radius\":" << config_.body_hard_radius << ",\"footprint_radius\":" << config_.footprint_radius << "},"
     << "\"reason\":\"";
  for (size_t i = 0; i < reasons.size(); ++i) {
    if (i > 0) ss << "; ";
    ss << reasons[i];
  }
  ss << "\"}";

  return ss.str();
}

std::string CloudGraphBuilder::diagnoseNode(const ManifoldGraph & graph,
                                            double x, double y, double z) const
{
  std::stringstream ss;
  uint32_t nid = 0;
  if (!findNearestNodeAny(graph, x, y, z, nid)) {
    ss << "{\"status\":\"error\",\"message\":\"坐标附近未找到流形图节点\"}";
    return ss.str();
  }
  const auto & u = graph.getNode(nid);

  // 扫描足印半径内的侧向障碍节点: 从踏面带向上生长 (超过抬腿极限) 的结构。
  // 楼梯上一级踏面 Δz ≤ max_step_height 不会命中; 窗口上限取 max(1.5m, 2倍机高),
  // 覆盖栏杆/矮墙 (顶面可高于机体), 排除上层楼板 (层高 ~3m)。
  // 注意: 图节点只存表面顶高, 与建图时基于表面底高的栅格级判定存在少量口径差,
  // 找不到精确墙体时如实回退到 flags 描述, 不编造方位。
  bool has_wall = false;
  double wall_d = 1e9, wall_dx = 0.0, wall_dy = 0.0, wall_z = 0.0;
  {
    int r0 = 0, c0 = 0;
    if (graph.toGridIndex(u.x, u.y, r0, c0)) {
      const double z_win_up = std::max(1.5, config_.dog_height * 2.0);
      const int rad = std::max(1, static_cast<int>(std::ceil(config_.footprint_radius / config_.resolution))) + 1;
      for (int dr = -rad; dr <= rad; ++dr) {
        for (int dc = -rad; dc <= rad; ++dc) {
          for (uint32_t wid : graph.getSpatialCellNodes(r0 + dr, c0 + dc)) {
            if (wid == nid) continue;
            const auto & w = graph.getNode(wid);
            double dz = w.z - u.z;
            if (dz <= config_.max_step_height || dz >= z_win_up) continue;
            double d = std::hypot(w.x - u.x, w.y - u.y);
            if (d < wall_d) {
              wall_d = d;
              wall_dx = w.x - u.x;
              wall_dy = w.y - u.y;
              wall_z = w.z;
              has_wall = true;
            }
          }
        }
      }
    }
  }

  // 侧向障碍方位 (8 方位, 供前端直读)
  auto dirName = [](double dx, double dy) -> std::string {
    if (std::hypot(dx, dy) < 1e-3) return "正上方";
    double ang = std::atan2(dy, dx) * 180.0 / M_PI;
    static const char * names[8] = {"+X 方向", "+X+Y 方向", "+Y 方向", "-X+Y 方向",
                                    "-X 方向", "-X-Y 方向", "-Y 方向", "+X-Y 方向"};
    int idx = static_cast<int>(std::floor((ang + 22.5) / 45.0));
    idx = ((idx % 8) + 8) % 8;
    return names[idx];
  };

  bool blocked = u.traversability >= 0.95f;
  const bool wall_in_hard = has_wall && wall_d <= config_.body_hard_radius + config_.resolution;
  std::string code, reason;
  if (blocked) {
    if (u.flags & node_flags::BLOCK_HEADROOM) {
      code = "headroom";
      reason = "禁行: 头顶净空 " + fmt2(u.headroom) + "m < 机体站立高度 " + fmt2(config_.dog_height) + "m, 通行会顶头";
    } else if (u.flags & node_flags::BLOCK_LATERAL) {
      code = "lateral_body";
      if (wall_in_hard) {
        reason = "禁行: 侧向墙体距踏面中心 " + fmt2(wall_d) + "m (" + dirName(wall_dx, wall_dy) + ", 高出踏面 " + fmt2(wall_z - u.z) + "m) < 机体硬半径 " + fmt2(config_.body_hard_radius) + "m, 机身无法通过";
      } else {
        reason = "禁行: 建图时栅格级判定足印硬半径 " + fmt2(config_.body_hard_radius) + "m 内存在竖直墙体 (细结构未形成独立图节点, 无法给出精确方位)";
      }
    } else {
      code = "blocked";
      reason = "禁行节点 (原因未记录)";
    }
  } else if (u.traversability > 0.05f) {
    code = "soft_inflation";
    if (has_wall && wall_d > config_.body_hard_radius + config_.resolution) {
      reason = "软膨胀减速区: 距侧向墙体 " + fmt2(wall_d) + "m (" + dirName(wall_dx, wall_dy) + ", 膨胀带 " + fmt2(config_.body_hard_radius) + "~" + fmt2(config_.footprint_radius) + "m), 可通行但代价升高";
    } else {
      reason = "软代价减速区 (trav=" + fmt2(u.traversability) + "), 位于机体膨胀带内, 可通行但代价升高";
    }
  } else {
    code = "free";
    reason = "可安全通行";
  }

  ss << "{"
     << "\"status\":\"ok\","
     << "\"node\":{\"id\":" << nid << ",\"x\":" << u.x << ",\"y\":" << u.y << ",\"z\":" << u.z
     << ",\"layer\":" << u.layer_id << ",\"headroom\":" << u.headroom
     << ",\"traversability\":" << u.traversability << ",\"flags\":" << u.flags << "},"
     << "\"blocked\":" << (blocked ? "true" : "false") << ","
     << "\"reason_code\":\"" << code << "\","
     << "\"reason\":\"" << reason << "\","
     << "\"nearest_obstacle\":"
     << (has_wall
          ? ("{\"dist\":" + fmt2(wall_d) + ",\"dx\":" + fmt2(wall_dx) + ",\"dy\":" + fmt2(wall_dy) + ",\"z\":" + fmt2(wall_z) + ",\"direction\":\"" + dirName(wall_dx, wall_dy) + "\"}")
          : std::string("null")) << ","
     << "\"limits\":{\"body_hard_radius\":" << config_.body_hard_radius
     << ",\"footprint_radius\":" << config_.footprint_radius
     << ",\"dog_height\":" << config_.dog_height
     << ",\"max_step_height\":" << config_.max_step_height << "}"
     << "}";
  return ss.str();
}

} // namespace elevation_planner
