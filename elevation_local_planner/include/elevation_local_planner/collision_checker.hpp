#pragma once

#include "elevation_planner_core/manifold_graph.hpp"

namespace elevation_local_planner
{

class CollisionChecker
{
public:
  CollisionChecker(double footprint_radius = 0.30, double clearance_min = 0.50)
    : footprint_radius_(footprint_radius), clearance_min_(clearance_min) {}

  bool checkCollision(double x, double y, double z,
                      int layer,
                      const elevation_planner::ManifoldGraph & graph) const;

private:
  double footprint_radius_{0.30};
  double clearance_min_{0.50};
};

} // namespace elevation_local_planner
