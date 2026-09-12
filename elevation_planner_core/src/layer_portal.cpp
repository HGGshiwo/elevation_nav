#include "elevation_planner_core/layer_portal.hpp"
#include <cmath>

namespace elevation_planner
{

void LayerPortalManager::detectPortals(
  const std::vector<std::vector<std::vector<double>>> & elevations,
  double max_transition_dz)
{
  portals_.clear();
  int num_layers = elevations.size();
  if (num_layers < 2) return;

  int rows = elevations[0].size();
  int cols = elevations[0][0].size();

  for (int s1 = 0; s1 < num_layers - 1; ++s1) {
    for (int s2 = s1 + 1; s2 < num_layers; ++s2) {
      for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
          double z1 = elevations[s1][r][c];
          double z2 = elevations[s2][r][c];
          if (!std::isnan(z1) && !std::isnan(z2)) {
            double dz = std::abs(z1 - z2);
            // 接缝处高程极其接近 (例如坡脚落地接触带)
            if (dz <= max_transition_dz) {
              LayerPortal p1;
              p1.from_layer = s1;
              p1.to_layer = s2;
              p1.row = r;
              p1.col = c;
              p1.height_diff = dz;
              p1.transition_cost = 0.05 + dz * 0.5;
              portals_.push_back(p1);

              LayerPortal p2 = p1;
              p2.from_layer = s2;
              p2.to_layer = s1;
              portals_.push_back(p2);
            }
          }
        }
      }
    }
  }
}

bool LayerPortalManager::isPortalCell(int layer, int r, int c, int & out_target_layer) const
{
  for (const auto & p : portals_) {
    if (p.from_layer == layer && p.row == r && p.col == c) {
      out_target_layer = p.to_layer;
      return true;
    }
  }
  return false;
}

} // namespace elevation_planner
