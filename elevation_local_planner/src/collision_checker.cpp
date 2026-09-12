#include "elevation_local_planner/collision_checker.hpp"
#include <cmath>

namespace elevation_local_planner
{

bool CollisionChecker::checkCollision(double x, double y, double /*z*/,
                                      int layer,
                                      const elevation_planner::ManifoldGraph & graph) const
{
  double res = graph.getResolution();
  int radius_cells = std::max(1, static_cast<int>(std::ceil(footprint_radius_ / res)));

  int center_r = 0, center_c = 0;
  if (!graph.toGridIndex(x, y, center_r, center_c)) {
    return false;
  }

  // 检查足印半径内的所有栅格是否有不可通行障碍
  for (int dr = -radius_cells; dr <= radius_cells; ++dr) {
    for (int dc = -radius_cells; dc <= radius_cells; ++dc) {
      if (std::hypot(dr, dc) * res > footprint_radius_) continue;
      int r = center_r + dr;
      int c = center_c + dc;
      elevation_planner::ManifoldNode node;
      if (graph.getNode(layer, r, c, node)) {
        if (node.traversability >= 0.9f) return true; // 碰撞
      }
    }
  }
  return false;
}

} // namespace elevation_local_planner
