#include "elevation_planner_core/manifold_fusion_engine.hpp"

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <tf2/exceptions.h>
#include <elevation_planner_core/graph_store.hpp>

#include <cmath>
#include <unordered_map>

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
  if (!mutated_graph_) return;
  for (const auto & kv : touched_nodes_)
  {
    auto & node = mutated_graph_->nodeMutable(kv.first);
    node.traversability = kv.second.first;
    node.headroom = kv.second.second;
  }
  touched_nodes_.clear();
  dynamic_nodes_.clear();
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
  } catch (const tf2::TransformException &) {
    return false;
  }

  if (cloud->empty()) {
    // 空观测: 无状态语义 = 纯先验, 恢复所有动态改写
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
    // 空观测: 无状态语义下融合结果 = 纯先验, 恢复所有动态改写
    // (注入器在无激活障碍时持续发空帧, 障碍消失后封锁随首帧空观测自动清除)
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
    restoreMutations();
    return false;
  }

  auto graph = GraphStore::instance().getGlobalGraphMutable();
  auto prior_table = GraphStore::instance().getGlobalTable();
  if (!graph || graph->numNodes() == 0 || !prior_table) return false;

  // 换图 (全局图重建设) 时, 旧图上的属性快照作废
  if (graph != mutated_graph_)
  {
    mutated_graph_ = graph;
    touched_nodes_.clear();
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
  for (int r = r0; r <= r1; ++r) {
    for (int c = c0; c <= c1; ++c) {
      for (uint32_t nid : graph->getSpatialCellNodes(r, c)) {
        const GraphNode nd = graph->getNode(nid); // 拷贝读, 避免写者-读者别名
        if (nd.z < z_band_low || nd.z > z_band_high) continue;

        auto fetch = [&](int dr, int dc) -> const std::vector<ColumnSurface> * {
          return surfacesAt(nd.x + dr * res, nd.y + dc * res);
        };
        const auto * self = fetch(0, 0);
        if (!self) continue;

        float headroom = 0.0f;
        bool lateral_hard = false;
        const float dyn_trav = CloudGraphBuilder::computeNodeTraversability(
            nd.z, *self, fetch, cfg, headroom, lateral_hard);

        auto it = touched_nodes_.find(nid);
        const float prior_trav = (it != touched_nodes_.end()) ? it->second.first : nd.traversability;
        const float prior_head = (it != touched_nodes_.end()) ? it->second.second : nd.headroom;

        // 永不解锁不变量: 注入障碍只会让通行性变差 (封锁/软代价/净空变小),
        // 不会让不可行走的节点变可走 —— 杜绝"走上天花板"类重算偏差
        const float eff_trav = std::max(prior_trav, dyn_trav);
        const float eff_head = std::min(prior_head, headroom);

        if (std::abs(eff_trav - prior_trav) > 1e-6f || std::abs(eff_head - prior_head) > 1e-4f) {
          // 动态值偏离先验: 原位写回 (首次改写时记录快照)
          auto & node = graph->nodeMutable(nid);
          node.traversability = eff_trav;
          node.headroom = eff_head;
          if (it == touched_nodes_.end()) {
            touched_nodes_.emplace(nid, std::make_pair(prior_trav, prior_head));
          }
          ++changed;
        } else if (it != touched_nodes_.end()) {
          // 动态值回到先验 (障碍离开): 恢复快照并移除记录
          auto & node = graph->nodeMutable(nid);
          node.traversability = prior_trav;
          node.headroom = prior_head;
          touched_nodes_.erase(it);
          ++changed;
        }
      }
    }
  }

  // 已改写但落出当前观测范围 (XY 超出 ROI 或 z 超出层带) 的节点:
  // 观测不再覆盖, 按无状态语义恢复先验 (与旧独立融合图 "障碍只存在于被观测到的当帧" 一致)
  for (auto it = touched_nodes_.begin(); it != touched_nodes_.end();) {
    const GraphNode & nd = graph->getNode(it->first);
    const int orr = static_cast<int>(std::floor((nd.x - oe.min_x) / res));
    const int occ = static_cast<int>(std::floor((nd.y - oe.min_y) / res));
    const bool in_scope = (orr >= 0 && orr < oe.rows && occ >= 0 && occ < oe.cols &&
                           nd.z >= z_band_low && nd.z <= z_band_high);
    if (!in_scope) {
      auto & node = graph->nodeMutable(it->first);
      node.traversability = it->second.first;
      node.headroom = it->second.second;
      it = touched_nodes_.erase(it);
      ++changed;
    } else {
      ++it;
    }
  }

  // 当前被动态改写节点的快照 (供宿主发布实时可视化)
  dynamic_nodes_.clear();
  dynamic_nodes_.reserve(touched_nodes_.size());
  for (const auto & kv : touched_nodes_)
  {
    const GraphNode & nd = graph->getNode(kv.first);
    dynamic_nodes_.push_back({nd.x, nd.y, nd.z, nd.traversability});
  }

  return changed > 0;
}

} // namespace elevation_planner
