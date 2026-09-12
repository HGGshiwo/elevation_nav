#pragma once

#include <cmath>
#include <algorithm>

namespace elevation_planner
{

struct QuadrupedDynamicsConfig
{
  double max_step_height{0.25};    ///< 机器狗最大跨台阶高度 (m)
  double max_stride_length{0.35};   ///< 机器狗物理跨步/跨沟宽度 (m)
  double dog_height{0.50};          ///< 机器狗站立高度/通行净空 (m)
  double weight_dist{1.0};          ///< 路径距离权重
  double weight_elevation{2.0};     ///< 高程跃迁惩罚权重
  double weight_traversability{3.0};///< 地形可通行性代价权重
};

class CostEvaluator
{
public:
  explicit CostEvaluator(const QuadrupedDynamicsConfig & config = QuadrupedDynamicsConfig())
    : config_(config) {}

  double computeEdgeCost(double x1, double y1, double z1,
                         double x2, double y2, double z2,
                         double trav1, double trav2) const;

  bool isTraversable(double dz, double dxy, double clearance) const;

  const QuadrupedDynamicsConfig & getConfig() const { return config_; }
  void setConfig(const QuadrupedDynamicsConfig & config) { config_ = config; }

private:
  QuadrupedDynamicsConfig config_;
};

} // namespace elevation_planner
