#pragma once

#include <vector>
#include <geometry_msgs/Point.h>

namespace elevation_planner
{

/**
 * @brief 层间转移网关接口（Portal / Gateway）
 * 机器狗在坡道起始处、楼梯接触面等高程几近相等的接合部切换图层
 */
struct LayerPortal
{
  int from_layer{0};
  int to_layer{1};
  int row{0};
  int col{0};
  geometry_msgs::Point position;
  double height_diff{0.0};   ///< 两层之间的高度差，接近 0 时允许平滑跨越
  double transition_cost{0.1}; ///< 层间转移转换基础代价
};

class LayerPortalManager
{
public:
  LayerPortalManager() = default;

  void detectPortals(const std::vector<std::vector<std::vector<double>>> & elevations,
                     double max_transition_dz = 0.15);

  const std::vector<LayerPortal> & getPortals() const { return portals_; }
  bool isPortalCell(int layer, int r, int c, int & out_target_layer) const;

private:
  std::vector<LayerPortal> portals_;
};

} // namespace elevation_planner
