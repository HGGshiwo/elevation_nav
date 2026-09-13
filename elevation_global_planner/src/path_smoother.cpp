#include "elevation_global_planner/path_smoother.hpp"
#include <cmath>

namespace elevation_global_planner
{

namespace
{
// 判断平滑后的落点是否进入阻挡区 (硬阻挡节点所在格 / 图外空白格)。
// 落点所在栅格无任何节点也视为阻挡: 防止平滑把路径拉出图外 (楼梯边缘/楼梯井悬空)。
bool inBlockedRegion(const elevation_planner::ManifoldGraph & graph, double x, double y, double z)
{
  int r = 0, c = 0;
  if (!graph.toGridIndex(x, y, r, c)) return true;

  const auto & cell_nodes = graph.getSpatialCellNodes(r, c);
  if (cell_nodes.empty()) return true;

  for (uint32_t nid : cell_nodes) {
    const auto & nd = graph.getNode(nid);
    // 贴墙软膨胀区 (traversability >= 0.40) 亦视为保护区, 严禁平滑算法内切内弯割角
    if (nd.traversability >= 0.40f && std::abs(nd.z - z) <= 0.35) return true;
  }
  return false;
}
} // namespace

void PathSmoother::smooth(const nav_msgs::Path & raw_path,
                          const elevation_planner::ManifoldGraph & graph,
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

      // 碰撞回退: 平滑点落入阻挡区则退回原始路径点, 防止平滑把路径拉进墙体或图外
      if (inBlockedRegion(graph, nx, ny, nz)) {
        change += std::abs(ox - rx) + std::abs(oy - ry) + std::abs(oz - rz);
        out_path.poses[i].pose.position.x = rx;
        out_path.poses[i].pose.position.y = ry;
        out_path.poses[i].pose.position.z = rz;
        continue;
      }

      change += std::abs(ox - nx) + std::abs(oy - ny) + std::abs(oz - nz);
      out_path.poses[i].pose.position.x = nx;
      out_path.poses[i].pose.position.y = ny;
      out_path.poses[i].pose.position.z = nz;
    }
  }
}

} // namespace elevation_global_planner
