#pragma once

#include "elevation_planner_core/manifold_graph.hpp"
#include "elevation_planner_core/manifold_search.hpp"
#include "elevation_planner_core/planner_interface.hpp"
#include "elevation_planner_core/path_simplifier.hpp"

#include <nav_msgs/Path.h>
#include <vector>
#include <queue>
#include <cstdint>

namespace elevation_global_planner
{

struct SearchEntry
{
  uint32_t node_id{0};
  float f_cost{0.0f};

  bool operator>(const SearchEntry & other) const { return f_cost > other.f_cost; }
};

class ManifoldAStarPlanner : public elevation_planner::GlobalPlannerInterface
{
public:
  ManifoldAStarPlanner();
  ~ManifoldAStarPlanner() override = default;

  bool initialize(const elevation_planner::ManifoldGraph & graph) override;

  /**
   * @brief SC-LOS 剪枝开关与最大段长 (2D, m); 需在 initialize 前调用
   */
  void setLosPruning(bool enabled, double max_segment)
  {
    los_prune_enabled_ = enabled;
    los_max_segment_ = max_segment;
  }

  bool plan(const geometry_msgs::PoseStamped & start,
            const geometry_msgs::PoseStamped & goal,
            nav_msgs::Path & out_path) override;

  bool findClosestNode(double x, double y, double z, elevation_planner::ManifoldNode & out_node) const;

  const elevation_planner::ManifoldGraph & getGraph() const { return *graph_; }

  /**
   * @brief 最近一次 plan() 剪枝前的原始密集路径 (供 debug 话题发布)
   */
  const nav_msgs::Path & getLastDensePath() const { return last_dense_path_; }

private:
  float computeHeuristic(uint32_t node_a, uint32_t node_b) const;

  // 零拷贝绑定: 只持指针, 指向 GraphStore 的融合活图 (makePlan 每次重绑定,
  // 融合引擎在另一线程原位刷新节点属性 —— 4 字节对齐 float 的良性竞争, 已接受)
  const elevation_planner::ManifoldGraph * graph_{nullptr};
  elevation_planner::PathSimplifier los_simplifier_;
  bool los_prune_enabled_{true};
  double los_max_segment_{0.60};
  nav_msgs::Path last_dense_path_;
  uint32_t last_goal_id_{std::numeric_limits<uint32_t>::max()};
  std::vector<uint32_t> last_path_ids_;
  bool is_initialized_{false};
};

} // namespace elevation_global_planner
