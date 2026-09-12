#include "elevation_global_planner/path_smoother.hpp"
#include <cmath>

namespace elevation_global_planner
{

void PathSmoother::smooth(const nav_msgs::Path & raw_path,
                          const elevation_planner::ManifoldGraph & /*graph*/,
                          nav_msgs::Path & out_path)
{
  out_path = raw_path;
  int n = raw_path.poses.size();
  if (n <= 2) return;

  double change = tolerance_;
  int max_iter = 100;
  int iter = 0;

  while (change >= tolerance_ && iter < max_iter) {
    change = 0.0;
    iter++;
    for (int i = 1; i < n - 1; ++i) {
      double ox = out_path.poses[i].pose.position.x;
      double oy = out_path.poses[i].pose.position.y;
      double oz = out_path.poses[i].pose.position.z;

      double rx = raw_path.poses[i].pose.position.x;
      double ry = raw_path.poses[i].pose.position.y;
      double rz = raw_path.poses[i].pose.position.z;

      double nx = ox + weight_data_ * (rx - ox) +
                  weight_smooth_ * (out_path.poses[i - 1].pose.position.x +
                                    out_path.poses[i + 1].pose.position.x - 2.0 * ox);
      double ny = oy + weight_data_ * (ry - oy) +
                  weight_smooth_ * (out_path.poses[i - 1].pose.position.y +
                                    out_path.poses[i + 1].pose.position.y - 2.0 * oy);
      double nz = oz + weight_data_ * (rz - oz) +
                  weight_smooth_ * (out_path.poses[i - 1].pose.position.z +
                                    out_path.poses[i + 1].pose.position.z - 2.0 * oz);

      change += std::abs(ox - nx) + std::abs(oy - ny) + std::abs(oz - nz);
      out_path.poses[i].pose.position.x = nx;
      out_path.poses[i].pose.position.y = ny;
      out_path.poses[i].pose.position.z = nz;
    }
  }
}

} // namespace elevation_global_planner
