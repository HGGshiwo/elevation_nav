#pragma once

#include "elevation_planner_core/manifold_graph.hpp"
#include "elevation_planner_core/layer_portal.hpp"
#include "elevation_planner_core/cost_evaluator.hpp"
#include "elevation_planner_core/planner_interface.hpp"

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
  void setPortalManager(const elevation_planner::LayerPortalManager & portal_mgr);
  void setCostEvaluator(const elevation_planner::CostEvaluator & evaluator);

  bool plan(const geometry_msgs::PoseStamped & start,
            const geometry_msgs::PoseStamped & goal,
            nav_msgs::Path & out_path) override;

  bool findClosestNode(double x, double y, double z, elevation_planner::ManifoldNode & out_node) const;

  const elevation_planner::ManifoldGraph & getGraph() const { return graph_; }

private:
  float computeHeuristic(uint32_t node_a, uint32_t node_b) const;

  elevation_planner::ManifoldGraph graph_;
  elevation_planner::LayerPortalManager portal_mgr_;
  elevation_planner::CostEvaluator cost_evaluator_;
  bool is_initialized_{false};
};

} // namespace elevation_global_planner
