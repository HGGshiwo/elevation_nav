#include "elevation_planner_core/manifold_fusion_engine.hpp"

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <tf2/exceptions.h>
#include <elevation_planner_core/graph_store.hpp>

#include <cmath>
#include <tuple>
#include <unordered_map>
#include <unordered_set>

namespace elevation_planner
{

void ManifoldFusionEngine::setConfig(const GraphBuildConfig & build_cfg, const Params & params)
{
  params_ = params;
  graph_builder_.setConfig(build_cfg);
  config_ready_ = true;
}

void ManifoldFusionEngine::ingestCloud(const sensor_msgs::PointCloud2::ConstPtr & msg)
{
  std::lock_guard<std::mutex> lock(cloud_mutex_);
  latest_cloud_ = msg;
  cloud_dirty_ = true;
}

void ManifoldFusionEngine::ingestAuxCloud(const sensor_msgs::PointCloud2::ConstPtr & msg)
{
  std::lock_guard<std::mutex> lock(aux_mutex_);
  latest_aux_ = msg;
}

void ManifoldFusionEngine::restoreMutations()
{
  if (!mutated_graph_ || !prev_valid_) return;
  const auto & cfg = graph_builder_.getConfig();
  const int inflate = static_cast<int>(std::ceil(cfg.inflation_radius / cfg.resolution)) + 1;
  clearDynamicLayer(mutated_graph_,
                    std::max(0, prev_r0_ - inflate), std::max(0, prev_c0_ - inflate),
                    std::min(mutated_graph_->getRows() - 1, prev_r1_ + inflate),
                    std::min(mutated_graph_->getCols() - 1, prev_c1_ + inflate));
  prev_valid_ = false;
  dynamic_nodes_.clear();
}

void ManifoldFusionEngine::clearDynamicLayer(const std::shared_ptr<ManifoldGraph> & graph,
                                             int r0, int c0, int r1, int c1)
{
  for (int r = r0; r <= r1; ++r)
    for (int c = c0; c <= c1; ++c)
      for (uint32_t nid : graph->getSpatialCellNodes(r, c))
      {
        if (nid >= graph->numNodes()) continue;
        auto & node = graph->nodeMutable(nid);
        node.dynamic_trav = 0.f;
        node.dynamic_headroom = 3.0f;
        node.dynamic_zone = static_cast<uint8_t>(CostZone::FREE);
        node.synthesize();
      }
}

ManifoldFusionEngine::~ManifoldFusionEngine()
{
  restoreMutations();
}

bool ManifoldFusionEngine::processLatestCloud(tf2_ros::Buffer * tf, const geometry_msgs::Pose & robot_pose)
{
  if (!config_ready_) return false;

  sensor_msgs::PointCloud2::ConstPtr msg;
  {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    if (cloud_dirty_ && latest_cloud_)
    {
      msg = latest_cloud_;
      cloud_dirty_ = false;
    }
  }
  sensor_msgs::PointCloud2::ConstPtr aux;
  {
    std::lock_guard<std::mutex> lock(aux_mutex_);
    aux = latest_aux_;
  }
  // 主云与叠加云都缺: 无观测, 保持现状 (已有 mutations 不清空)
  if (!msg && !aux) return false;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  try {
    if (msg) {
      if (msg->header.frame_id != params_.map_frame && tf) {
        pcl::PointCloud<pcl::PointXYZ> raw;
        pcl::fromROSMsg(*msg, raw);
        pcl_ros::transformPointCloud(params_.map_frame, raw, *cloud, *tf);
      } else {
        pcl::fromROSMsg(*msg, *cloud);
      }
    }
    // 叠加云 (编辑器障碍): frame 恒为 map, 直接入列 —— 与主云物理并存, 无交错竞态
    if (aux) {
      pcl::PointCloud<pcl::PointXYZ> aux_pts;
      pcl::fromROSMsg(*aux, aux_pts);
      *cloud += aux_pts;
    }
  } catch (const tf2::TransformException & ex) {
    ROS_WARN_THROTTLE(2.0, "[Fusion] TF transform exception: %s", ex.what());
    return false;
  }

  if (cloud->empty()) {
    ROS_WARN_THROTTLE(2.0, "[Fusion] Combined pointcloud (main + aux) is empty -> restoring mutations (dynamic layer cleared)");
    restoreMutations();
    return false;
  }

  // ROI 裁剪: 机器人周围方窗 + 垂直高度带 (融合只覆盖机体周边, 保证 10Hz 节拍)
  const double rx = robot_pose.position.x;
  const double ry = robot_pose.position.y;
  const double rz = robot_pose.position.z;
  const double band_low  = rz - params_.crop_height_below;
  const double band_high = rz + params_.crop_height_above;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cropped(new pcl::PointCloud<pcl::PointXYZ>);
  cropped->reserve(cloud->size());
  const double r2 = params_.crop_radius_xy * params_.crop_radius_xy;
  for (const auto & pt : cloud->points) {
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;
    const double dx = pt.x - rx, dy = pt.y - ry;
    if (dx * dx + dy * dy > r2) continue;
    if (pt.z < band_low || pt.z > band_high) continue;
    cropped->push_back(pt);
  }

  if (cropped->empty()) {
    ROS_WARN_THROTTLE(2.0, "[Fusion] Cropped ROI cloud is 0 points! Total raw points=%zu. Robot=(%.2f, %.2f, %.2f), crop_radius_xy=%.2fm, z_band=[%.2f, %.2f]. All obstacles outside crop window -> restoring mutations (cleared dynamic obstacles)",
                      cloud->size(), rx, ry, rz, params_.crop_radius_xy, band_low, band_high);
    restoreMutations();
    return false;
  }

  // 观测柱表: 与全局图共用分辨率, 范围为机器人居中方窗
  GridExtent local_extent;
  local_extent.resolution = graph_builder_.getConfig().resolution;
  local_extent.min_x = rx - params_.crop_radius_xy;
  local_extent.min_y = ry - params_.crop_radius_xy;
  local_extent.rows = static_cast<int>(std::ceil(2.0 * params_.crop_radius_xy / local_extent.resolution)) + 1;
  local_extent.cols = local_extent.rows;

  ColumnTable observed;
  if (!graph_builder_.buildColumnTable(cropped, observed, &local_extent) || observed.empty()) {
    ROS_WARN_THROTTLE(2.0, "[Fusion] Failed to build observed column table -> restoring mutations");
    restoreMutations();
    return false;
  }

  auto graph = GraphStore::instance().getGlobalGraphMutable();
  auto prior_table = GraphStore::instance().getGlobalTable();
  if (!graph || graph->numNodes() == 0 || !prior_table) return false;

  // 换图 (全局图重建设) 时, 上一帧窗口作废 (新图动态层本就为空)
  if (graph != mutated_graph_)
  {
    mutated_graph_ = graph;
    prev_valid_ = false;
  }

  const auto & cfg = graph_builder_.getConfig();
  const double res = observed.extent.resolution;
  const auto & oe = observed.extent;
  const auto & pe = prior_table->extent;
  const double tol = cfg.cluster_height_diff;

  // 曲面查询 (世界坐标): ROI 内 = 观测 ⊕ 先验 逐柱合并 (按格缓存); ROI 外 = 纯先验
  std::unordered_map<int64_t, std::vector<ColumnSurface>> merged_cache;
  auto surfacesAt = [&](double wx, double wy) -> const std::vector<ColumnSurface> * {
    const int orr = static_cast<int>(std::floor((wx - oe.min_x) / res));
    const int occ = static_cast<int>(std::floor((wy - oe.min_y) / res));
    if (orr < 0 || orr >= oe.rows || occ < 0 || occ >= oe.cols) {
      // ROI 外: 纯先验
      const int pr = static_cast<int>(std::floor((wx - pe.min_x) / pe.resolution));
      const int pc = static_cast<int>(std::floor((wy - pe.min_y) / pe.resolution));
      if (pr < 0 || pr >= pe.rows || pc < 0 || pc >= pe.cols) return nullptr;
      return &prior_table->cells[static_cast<size_t>(pr * pe.cols + pc)];
    }
    const int64_t key = static_cast<int64_t>(orr) * 1000003 + occ;
    auto it = merged_cache.find(key);
    if (it != merged_cache.end()) return &it->second;
    const int pr = static_cast<int>(std::floor((wx - pe.min_x) / pe.resolution));
    const int pc = static_cast<int>(std::floor((wy - pe.min_y) / pe.resolution));
    std::vector<ColumnSurface> merged;
    if (pr >= 0 && pr < pe.rows && pc >= 0 && pc < pe.cols) {
      merged = CloudGraphBuilder::mergeColumnSurfaces(
          observed.cells[static_cast<size_t>(orr * oe.cols + occ)],
          prior_table->cells[static_cast<size_t>(pr * pe.cols + pc)], tol);
    } else {
      merged = observed.cells[static_cast<size_t>(orr * oe.cols + occ)];
    }
    return &(merged_cache.emplace(key, std::move(merged)).first->second);
  };

  // ROI 覆盖的全局图栅格范围 (与图栅格求交)
  int r0 = 0, c0 = 0, r1 = graph->getRows() - 1, c1 = graph->getCols() - 1;
  {
    int tr = 0, tc = 0;
    if (graph->toGridIndex(oe.min_x, oe.min_y, tr, tc)) {
      r0 = std::max(r0, tr);
      c0 = std::max(c0, tc);
    }
    if (graph->toGridIndex(oe.min_x + oe.rows * res, oe.min_y + oe.cols * res, tr, tc)) {
      r1 = std::min(r1, tr);
      c1 = std::min(c1, tc);
    }
  }

  size_t changed = 0;
  // 层带过滤: 只刷新机器人当前层带内的节点。空间格每格含全部楼层叠层节点,
  // 不过滤会把其它楼层的甲板/天花板节点也摸一遍 (观测数据属于机器人所在层,
  // 跨层重算既无意义又会因柱面差异误改静态通行性)
  const double z_band_low = band_low - 0.3;
  const double z_band_high = band_high + 0.3;

  // ---- 无状态分层刷新 (无变化检测/无快照): ----
  //   清理区 = 上帧 ∪ 本帧 ROI (外扩膨胀半径): 动态层清零, 覆盖障碍出现/消失/移动
  //   应用区 = 本帧 ROI: 逐节点全量重算动态层, 加法叠加静态层 (封顶 1.0)
  //   两条膨胀带重叠的窄缝代数和越过禁行线 → 合成 1.0 = 机体真过不去, 语义自洽
  const int inflate_cells = static_cast<int>(std::ceil(cfg.inflation_radius / res)) + 1;

  // 1) 局部计算本帧 ROI 窗口内所有节点的最新动态状态 (无中间态, 存于局部 new_dyn)
  struct DynState {
    float dyn_trav{0.0f};
    float dyn_head{3.0f};
    uint8_t dyn_zone{static_cast<uint8_t>(CostZone::FREE)};
  };
  std::unordered_map<uint32_t, DynState> new_dyn;
  new_dyn.reserve(static_cast<size_t>((r1 - r0 + 1) * (c1 - c0 + 1) * 2));

  for (int r = r0; r <= r1; ++r) {
    for (int c = c0; c <= c1; ++c) {
      for (uint32_t nid : graph->getSpatialCellNodes(r, c)) {
        if (nid >= graph->numNodes()) continue;
        const GraphNode & nd = graph->getNode(nid);
        if (nd.z < z_band_low || nd.z > z_band_high) continue;

        auto fetch = [&](int dr, int dc) -> const std::vector<ColumnSurface> * {
          return surfacesAt(nd.x + dr * res, nd.y + dc * res);
        };
        const auto * self = fetch(0, 0);
        if (!self) continue;

        float dyn_soft = 0.0f;
        float dyn_head = 0.0f;
        const CostZone dyn_zone = CloudGraphBuilder::computeNodeZone(
            nd.z, *self, fetch, cfg, dyn_soft, dyn_head);

        DynState st;
        st.dyn_zone = static_cast<uint8_t>(dyn_zone);
        st.dyn_trav = (dyn_zone == CostZone::BODY_HARD || dyn_zone == CostZone::FORBIDDEN)
                          ? 1.0f : dyn_soft;
        st.dyn_head = dyn_head;
        new_dyn.emplace(nid, st);
      }
    }
  }

  // 2) 原子覆写与清理 (In-place Overwrite: 绝不在原图上先清零再重算产生裸 FREE 中间态)
  dynamic_nodes_.clear();
  std::unordered_set<uint32_t> touched_nids;
  touched_nids.reserve(new_dyn.size());

  // 2a) 覆写本帧计算出的全部节点状态
  for (const auto & kv : new_dyn) {
    uint32_t nid = kv.first;
    touched_nids.insert(nid);
    const auto & st = kv.second;
    auto & node = graph->nodeMutable(nid);

    // 状态实质改变时才写入, 避免频繁脏写与 cache line 颠簸
    if (std::fabs(node.dynamic_trav - st.dyn_trav) > 1e-6f ||
        std::fabs(node.dynamic_headroom - st.dyn_head) > 1e-4f ||
        node.dynamic_zone != st.dyn_zone)
    {
      node.dynamic_trav = st.dyn_trav;
      node.dynamic_headroom = st.dyn_head;
      node.dynamic_zone = st.dyn_zone;
      node.synthesize();
      ++changed;
    }

    if (node.dynamic_zone > static_cast<uint8_t>(CostZone::FREE) || node.dynamic_trav > 0.f) {
      dynamic_nodes_.push_back({node.x, node.y, node.z, node.traversability,
                                static_cast<float>(node.cost_zone)});
    }
  }

  // 2b) 清理上一帧窗口中有动态值但本帧不在 new_dyn 中的节点 (障碍消失/移出视野)
  if (prev_valid_) {
    const int wr0 = std::max(0, prev_r0_ - inflate_cells), wc0 = std::max(0, prev_c0_ - inflate_cells);
    const int wr1 = std::min(graph->getRows() - 1, prev_r1_ + inflate_cells);
    const int wc1 = std::min(graph->getCols() - 1, prev_c1_ + inflate_cells);
    for (int r = wr0; r <= wr1; ++r) {
      for (int c = wc0; c <= wc1; ++c) {
        for (uint32_t nid : graph->getSpatialCellNodes(r, c)) {
          if (nid >= graph->numNodes()) continue;
          if (touched_nids.count(nid) > 0) continue; // 本帧已原子覆写

          auto & node = graph->nodeMutable(nid);
          if (node.dynamic_trav > 0.f || node.dynamic_headroom < 3.0f ||
              node.dynamic_zone != static_cast<uint8_t>(CostZone::FREE))
          {
            node.dynamic_trav = 0.f;
            node.dynamic_headroom = 3.0f;
            node.dynamic_zone = static_cast<uint8_t>(CostZone::FREE);
            node.synthesize();
            ++changed;
          }
        }
      }
    }
  }

  // 记录本帧窗口供下一帧清理
  prev_r0_ = r0; prev_c0_ = c0; prev_r1_ = r1; prev_c1_ = c1;
  prev_valid_ = true;

  return changed > 0;
}

} // namespace elevation_planner
