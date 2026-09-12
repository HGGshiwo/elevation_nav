#pragma once

#include <nav_msgs/Path.h>
#include "elevation_planner_core/manifold_graph.hpp"

namespace elevation_global_planner
{

class PathSmoother
{
public:
  PathSmoother(double weight_data = 0.5, double weight_smooth = 0.3, double tolerance = 0.01)
    : weight_data_(weight_data), weight_smooth_(weight_smooth), tolerance_(tolerance) {}

  void smooth(const nav_msgs::Path & raw_path,
              const elevation_planner::ManifoldGraph & graph,
              nav_msgs::Path & out_path);

private:
  double weight_data_{0.5};
  double weight_smooth_{0.3};
  double tolerance_{0.01};
};

} // namespace elevation_global_planner
