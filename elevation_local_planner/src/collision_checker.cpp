#include "elevation_local_planner/collision_checker.hpp"
#include <cmath>

namespace elevation_local_planner
{

bool CollisionChecker::checkCollision(double x, double y, double z,
                                      int layer,
                                      const elevation_planner::ManifoldGraph & graph) const
{
  (void)layer;
  // 1. 查找机器人脚下最近的踏面节点
  uint32_t nid = 0;
  if (!graph.findClosestNode(x, y, z, nid, 0.45, 0.7)) {
    // 脚下完全脱离踏面（悬空或越界出图）
    return false; // 不阻塞初始化，由外部路径跟踪守卫约束
  }

  const auto & node = graph.getNode(nid);

  // 2. 节点本身即为致命硬阻挡 (建图已膨胀硬半径 / 顶头撞梁)
  if (node.traversability >= 0.98f || node.headroom < clearance_min_) {
    return true; // 发生实质性硬碰撞
  }

  // 3. 机身核心硬半径贴身安全检查 (避免大半径二次膨胀误判走廊边缘)
  if (footprint_radius_ > 0.05) {
    double res = graph.getResolution();
    int radius_cells = static_cast<int>(std::floor(footprint_radius_ / res));
    int center_r = node.row, center_c = node.col;
    for (int dr = -radius_cells; dr <= radius_cells; ++dr) {
      for (int dc = -radius_cells; dc <= radius_cells; ++dc) {
        if (dr == 0 && dc == 0) continue;
        if (std::hypot(dr, dc) * res > footprint_radius_) continue;
        int r = center_r + dr;
        int c = center_c + dc;
        elevation_planner::ManifoldNode nbr;
        if (graph.getNode(node.layer_id, r, c, nbr)) {
          if (nbr.traversability >= 0.99f) return true;
        }
      }
    }
  }

  return false;
}

} // namespace elevation_local_planner
