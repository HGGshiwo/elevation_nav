#include "elevation_planner_core/cost_evaluator.hpp"

namespace elevation_planner
{

double CostEvaluator::computeEdgeCost(double x1, double y1, double z1,
                                      double x2, double y2, double z2,
                                      double trav1, double trav2) const
{
  double dx = x2 - x1;
  double dy = y2 - y1;
  double dz = std::abs(z2 - z1);
  double dxy = std::hypot(dx, dy);

  double dist_cost = config_.weight_dist * dxy;
  double elevation_cost = config_.weight_elevation * (dz * 2.5);
  double trav_avg = 0.5 * (trav1 + trav2);
  double terrain_cost = config_.weight_traversability * trav_avg;

  return dist_cost + elevation_cost + terrain_cost;
}

bool CostEvaluator::isTraversable(double dz, double dxy, double clearance) const
{
  if (dz > config_.max_step_height) return false;
  if (dxy > config_.max_stride_length) return false;
  if (clearance > 0.15 && clearance < config_.dog_height) {
    return false; // 顶棚净空不足
  }
  return true;
}

} // namespace elevation_planner
