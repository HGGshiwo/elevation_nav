#pragma once

#include "manifold_graph.hpp"
#include <vector>
#include <list>
#include <unordered_map>
#include <cmath>
#include <cstdint>
#include <algorithm>
#include <limits>

namespace elevation_planner
{

/**
 * @brief SC-LOS (Support-Chain Line-of-Sight) 直线连通检测与路径剪枝配置
 */
struct ScLosConfig
{
  double max_step_height{0.25};   ///< z 链单步高度容差 (与图构建一致)
  double min_headroom{0.45};      ///< 支撑节点净空下限 (与 isPathBlocked 一致)
  double sample_step{0.0};        ///< 直线采样步长 (m); 0 => build() 按 0.5*分辨率自动取, 下限 0.02
  double support_radius{0.0};     ///< 采样点支撑节点匹配半径 (m); 0 => build() 按 0.75*分辨率自动取, 下限 0.04
  double max_seg_length{0.60};    ///< 剪枝最大段长 (m, 2D)
  size_t cache_capacity{8192};    ///< LOS 结果 LRU 缓存容量
  bool   strict_adjacency{true};  ///< 相邻匹配节点必须在图上直接邻接 (单步连通)
  int    max_samples{400};        ///< 单次 LOS 检查的采样数上限 (保护)
};

/**
 * @brief 支撑链直线连通检测器 + 贪心路径剪枝
 *
 * sc_los(A,B) 判定: 沿 A→B 的 2D 直线以 sample_step 采样, 每个采样点在 z 链
 * (携带层身份, 单步高度容差) 约束下能匹配到同层可通行支撑节点, 且相邻匹配节点
 * 在图上直接邻接。z 链保证多层环境中不会把楼梯井两侧的不同层"直线接通"。
 *
 * 复用 ManifoldGraph 自带的空间桶索引 (spatial_grid_), 无需额外建哈希;
 * LOS 结果按节点对缓存 (LRU), 图实例变更时需重新 build()。
 */
class PathSimplifier
{
public:
  explicit PathSimplifier(const ScLosConfig& cfg = ScLosConfig()) : cfg_(cfg) {}

  /// @brief 绑定图实例 (图更新/换实例后必须重新调用; 内部清空缓存)
  void build(const ManifoldGraph& graph)
  {
    graph_ = &graph;
    const double res = graph.getResolution();
    if (res > 1e-6)
    {
      if (cfg_.sample_step <= 0.0) cfg_.sample_step = std::max(0.02, 0.5 * res);
      if (cfg_.support_radius <= 0.0) cfg_.support_radius = std::max(0.04, 0.75 * res);
    }
    if (cfg_.sample_step <= 0.0) cfg_.sample_step = 0.05;
    if (cfg_.support_radius <= 0.0) cfg_.support_radius = 0.075;
    bound_nodes_ = graph.numNodes();
    clearCache();
  }

  void setConfig(const ScLosConfig& cfg) { cfg_ = cfg; clearCache(); }
  const ScLosConfig& getConfig() const { return cfg_; }

  const ManifoldGraph* boundGraph() const { return graph_; }
  size_t boundNodeCount() const { return bound_nodes_; }

  /**
   * @brief 判定两节点是否"直线联通" (SC-LOS)
   */
  bool losConnected(uint32_t a_id, uint32_t b_id)
  {
    if (a_id == b_id) return true;
    if (!graph_ || a_id >= bound_nodes_ || b_id >= bound_nodes_) return false;

    const uint64_t key = pairKey(a_id, b_id);
    auto it = cache_index_.find(key);
    if (it != cache_index_.end())
    {
      lru_.splice(lru_.begin(), lru_, it->second);
      return it->second->second;
    }

    const bool value = losCompute(a_id, b_id);
    lru_.push_front({key, value});
    cache_index_[key] = lru_.begin();
    if (lru_.size() > cfg_.cache_capacity)
    {
      cache_index_.erase(lru_.back().first);
      lru_.pop_back();
    }
    return value;
  }

  /**
   * @brief 贪心剪枝: 从起点向后找最远直线联通节点, 输出稀疏航点序列
   * @param ids 原始节点序列 (连续重复 id 会被剔除)
   * @param max_seg_length 最大段长 (2D, m); <=0 使用配置默认值
   * @return 稀疏节点序列 (首尾保留, 保序)
   */
  std::vector<uint32_t> simplifyPath(const std::vector<uint32_t>& ids, double max_seg_length = -1.0)
  {
    const double max_seg = (max_seg_length > 0.0) ? max_seg_length : cfg_.max_seg_length;

    std::vector<uint32_t> p;
    p.reserve(ids.size());
    for (uint32_t id : ids)
    {
      if (p.empty() || p.back() != id) p.push_back(id);
    }
    if (p.size() <= 2) return p;

    std::vector<uint32_t> out;
    out.reserve(p.size());
    out.push_back(p.front());

    size_t i = 0;
    const size_t last = p.size() - 1;
    while (i < last)
    {
      size_t last_good = i + 1;  // 相邻节点必然可直接到达, 保底前进
      size_t step = 4;
      while (true)
      {
        // 探测点 = last_good + step, 超出段长上限时回退到上限内最远节点
        size_t cand = last_good + step;
        if (cand > last) cand = last;
        while (cand > last_good && segDist2D(p[i], p[cand]) > max_seg) --cand;
        if (cand <= last_good) break;  // 已吃满段长上限

        if (losConnected(p[i], p[cand]))
        {
          last_good = cand;
          step = std::min(step * 2, static_cast<size_t>(16));
        }
        else
        {
          // 二分回退: 在 (last_good, cand) 内找最远直线联通节点 (视 LOS 近似单调)
          size_t lo = last_good + 1, hi = cand - 1;
          while (lo <= hi)
          {
            const size_t mid = lo + (hi - lo) / 2;
            if (losConnected(p[i], p[mid]))
            {
              last_good = mid;
              lo = mid + 1;
            }
            else
            {
              if (mid == lo) break;
              hi = mid - 1;
            }
          }
          break;
        }
      }
      out.push_back(p[last_good]);
      i = last_good;
    }
    return out;
  }

  /**
   * @brief 输出 A→B 的 SC-LOS 支撑链: 沿线 z 链行走匹配到的节点序列 (含 A、B, 保序)
   *
   * 链上相邻节点要么相同要么图上直接邻接 (严格模式), 因此天然同层、单步连通——
   * 可直接作为段走廊的多源 BFS 种子, 无需任何额外高度过滤。
   */
  bool computeLosChain(uint32_t a_id, uint32_t b_id, std::vector<uint32_t>& out_chain)
  {
    out_chain.clear();
    if (!graph_ || a_id >= bound_nodes_ || b_id >= bound_nodes_) return false;

    if (a_id == b_id)
    {
      out_chain.push_back(a_id);
      return true;
    }

    const uint64_t key = pairKey(a_id, b_id);
    auto it = chain_index_.find(key);
    if (it != chain_index_.end())
    {
      chain_lru_.splice(chain_lru_.begin(), chain_lru_, it->second);
      out_chain = it->second->second;
      return true;
    }

    std::vector<uint32_t> chain;
    if (!losWalk(a_id, b_id, &chain)) return false;

    chain_lru_.push_front({key, chain});
    chain_index_[key] = chain_lru_.begin();
    if (chain_lru_.size() > cfg_.cache_capacity)
    {
      chain_index_.erase(chain_lru_.back().first);
      chain_lru_.pop_back();
    }
    out_chain = std::move(chain);
    return true;
  }

  void clearCache()
  {
    lru_.clear();
    cache_index_.clear();
    chain_lru_.clear();
    chain_index_.clear();
  }

private:
  struct Candidate
  {
    uint32_t id;
    double score;
  };

  static uint64_t pairKey(uint32_t a, uint32_t b)
  {
    return (a < b) ? ((static_cast<uint64_t>(a) << 32) | b)
                   : ((static_cast<uint64_t>(b) << 32) | a);
  }

  double segDist2D(uint32_t a_id, uint32_t b_id) const
  {
    const GraphNode& a = graph_->getNode(a_id);
    const GraphNode& b = graph_->getNode(b_id);
    const double dx = static_cast<double>(a.x) - b.x;
    const double dy = static_cast<double>(a.y) - b.y;
    return std::hypot(dx, dy);
  }

  bool losCompute(uint32_t a_id, uint32_t b_id)
  {
    return losWalk(a_id, b_id, nullptr);
  }

  /// @brief z 链行走主体: chain_out 非空时输出匹配节点序列 (含 A、B)
  bool losWalk(uint32_t a_id, uint32_t b_id, std::vector<uint32_t>* chain_out)
  {
    const GraphNode& A = graph_->getNode(a_id);
    const GraphNode& B = graph_->getNode(b_id);
    const double ax = A.x, ay = A.y;
    const double dist = std::hypot(B.x - ax, B.y - ay);

    int n = 1;
    if (cfg_.sample_step > 1e-9)
      n = static_cast<int>(std::ceil(dist / cfg_.sample_step));
    n = std::max(1, std::min(cfg_.max_samples, n));

    if (chain_out)
    {
      chain_out->clear();
      chain_out->push_back(a_id);
    }
    uint32_t prev = a_id;
    double z_prev = A.z;
    for (int k = 1; k < n; ++k)
    {
      const double t = static_cast<double>(k) / n;
      const double px = ax + (B.x - ax) * t;
      const double py = ay + (B.y - ay) * t;
      uint32_t matched = prev;
      if (!matchSupport(px, py, prev, z_prev, matched)) return false;
      if (chain_out && matched != prev) chain_out->push_back(matched);
      prev = matched;
      z_prev = graph_->getNode(matched).z;
    }

    if (prev == b_id) return true;
    if (!graph_->isConnected(prev, b_id, 1)) return false;
    if (chain_out) chain_out->push_back(b_id);
    return true;
  }

  /**
   * @brief 在 (px,py) 附近匹配支撑节点: z 链高度容差 + 净空 + 可通行度过滤,
   *        候选按 (距离 + 高差) 评分取最优; 严格模式下要求与前一匹配节点直接邻接
   */
  bool matchSupport(double px, double py, uint32_t prev, double z_prev, uint32_t& out_id)
  {
    int r = 0, c = 0;
    if (!graph_->toGridIndex(px, py, r, c))
    {
      // 地图边界外微量截断 (与 findClosestNode 行为一致)
      r = std::max(0, std::min(graph_->getRows() - 1,
                               static_cast<int>(std::floor((px - graph_->getMinX()) / graph_->getResolution()))));
      c = std::max(0, std::min(graph_->getCols() - 1,
                               static_cast<int>(std::floor((py - graph_->getMinY()) / graph_->getResolution()))));
    }

    const double res = std::max(1e-6, graph_->getResolution());
    const int rad = std::max(1, static_cast<int>(std::ceil(cfg_.support_radius / res)));

    std::vector<Candidate> cands;
    cands.reserve(8);
    for (int dr = -rad; dr <= rad; ++dr)
    {
      const int rr = r + dr;
      if (rr < 0 || rr >= graph_->getRows()) continue;
      for (int dc = -rad; dc <= rad; ++dc)
      {
        const int cc = c + dc;
        if (cc < 0 || cc >= graph_->getCols()) continue;
        for (uint32_t nid : graph_->getSpatialCellNodes(rr, cc))
        {
          const GraphNode& nd = graph_->getNode(nid);
          if (nd.traversability >= 0.95f) continue;
          if (nd.headroom < cfg_.min_headroom) continue;
          const double dz = std::abs(static_cast<double>(nd.z) - z_prev);
          if (dz > cfg_.max_step_height) continue;
          const double dxy = std::hypot(static_cast<double>(nd.x) - px,
                                        static_cast<double>(nd.y) - py);
          if (dxy > cfg_.support_radius) continue;
          cands.push_back({nid, dxy + 2.0 * dz});
        }
      }
    }
    if (cands.empty()) return false;

    std::sort(cands.begin(), cands.end(),
              [](const Candidate& a, const Candidate& b) { return a.score < b.score; });
    for (const auto& cd : cands)
    {
      if (!cfg_.strict_adjacency || cd.id == prev || graph_->isConnected(prev, cd.id, 1))
      {
        out_id = cd.id;
        return true;
      }
    }
    return false;
  }

  const ManifoldGraph* graph_{nullptr};
  size_t bound_nodes_{0};
  ScLosConfig cfg_;

  std::list<std::pair<uint64_t, bool>> lru_;
  std::unordered_map<uint64_t, std::list<std::pair<uint64_t, bool>>::iterator> cache_index_;

  std::list<std::pair<uint64_t, std::vector<uint32_t>>> chain_lru_;
  std::unordered_map<uint64_t, std::list<std::pair<uint64_t, std::vector<uint32_t>>>::iterator> chain_index_;
};

} // namespace elevation_planner
