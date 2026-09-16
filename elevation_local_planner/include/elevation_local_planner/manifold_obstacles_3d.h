#pragma once

#include <teb_local_planner/obstacles.h>
#include <teb_local_planner/distance_calculations.h>
#include <elevation_planner_core/manifold_graph.hpp>
#include <elevation_planner_core/graph_store.hpp>
#include <Eigen/Core>
#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>

namespace elevation_local_planner
{

/**
 * @brief 在流形图中根据 (x, y) 和参考高度快速查询真实地表踏面高度 z
 */
inline double querySurfaceZ(double x, double y, double ref_z)
{
  auto graph = elevation_planner::GraphStore::instance().getFusedGraph();
  if (!graph || graph->numNodes() == 0)
  {
    graph = elevation_planner::GraphStore::instance().getGlobalGraph();
  }
  if (!graph || graph->numNodes() == 0)
  {
    return ref_z;
  }

  int r = 0, c = 0;
  if (!graph->toGridIndex(x, y, r, c))
  {
    return ref_z;
  }

  const auto & nodes = graph->getSpatialCellNodes(r, c);
  double best_dz = std::numeric_limits<double>::max();
  double best_z = ref_z;
  for (uint32_t nid : nodes)
  {
    const auto & nd = graph->getNode(nid);
    double dz = std::abs(nd.z - ref_z);
    if (dz < best_dz)
    {
      best_dz = dz;
      best_z = nd.z;
    }
  }

  if (best_dz < 1.0)
  {
    return best_z;
  }

  // 若本格未命中，在 8 邻域查找
  for (int dr = -1; dr <= 1; ++dr)
  {
    for (int dc = -1; dc <= 1; ++dc)
    {
      if (dr == 0 && dc == 0) continue;
      for (uint32_t nid : graph->getSpatialCellNodes(r + dr, c + dc))
      {
        const auto & nd = graph->getNode(nid);
        double dz = std::abs(nd.z - ref_z);
        if (dz < best_dz)
        {
          best_dz = dz;
          best_z = nd.z;
        }
      }
    }
  }

  return best_z;
}

/**
 * @class LineObstacle3D
 * @brief 真实三维空间线段障碍物（如楼梯断坎、边缘悬崖、护栏）
 * 
 * 计算机器狗位置到该三维空间线段的精确 3D 欧氏最短几何距离
 */
class LineObstacle3D : public teb_local_planner::LineObstacle
{
public:
  LineObstacle3D(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2)
    : teb_local_planner::LineObstacle(p1.head<2>(), p2.head<2>()),
      p1_3d_(p1), p2_3d_(p2)
  {
  }

  virtual ~LineObstacle3D() = default;

  virtual double getMinimumDistance(const Eigen::Vector2d& position) const override
  {
    double ref_z = (p1_3d_.z() + p2_3d_.z()) * 0.5;
    double z_r = querySurfaceZ(position.x(), position.y(), ref_z);
    Eigen::Vector3d pt(position.x(), position.y(), z_r);

    // 计算 3D 空间点到 3D 空间线段的最短距离
    Eigen::Vector3d diff = p2_3d_ - p1_3d_;
    double len_sq = diff.squaredNorm();
    double t = 0.0;
    if (len_sq > 1e-6)
    {
      t = (pt - p1_3d_).dot(diff) / len_sq;
      t = std::max(0.0, std::min(1.0, t));
    }
    Eigen::Vector3d closest = p1_3d_ + t * diff;
    return (pt - closest).norm();
  }

  virtual bool checkCollision(const Eigen::Vector2d& position, double min_dist) const override
  {
    return getMinimumDistance(position) <= min_dist;
  }

  const Eigen::Vector3d& getStart3D() const { return p1_3d_; }
  const Eigen::Vector3d& getEnd3D() const { return p2_3d_; }

private:
  Eigen::Vector3d p1_3d_;
  Eigen::Vector3d p2_3d_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/**
 * @class CylinderObstacle3D
 * @brief 真实三维空间圆柱体障碍物（如立柱、树干、垂直障碍）
 * 
 * 包含底面中心、半径与明确的纵向 [z_min, z_max] 高度范围，天然解决上下楼层重叠无干涉
 */
class CylinderObstacle3D : public teb_local_planner::CircularObstacle
{
public:
  CylinderObstacle3D(const Eigen::Vector3d& center, double radius, double height = 2.0)
    : teb_local_planner::CircularObstacle(center.head<2>(), radius),
      center_3d_(center), radius_(radius),
      z_min_(center.z() - 0.1), z_max_(center.z() + height)
  {
  }

  CylinderObstacle3D(const Eigen::Vector3d& center, double radius, double z_min, double z_max)
    : teb_local_planner::CircularObstacle(center.head<2>(), radius),
      center_3d_(center), radius_(radius),
      z_min_(z_min), z_max_(z_max)
  {
  }

  virtual ~CylinderObstacle3D() = default;

  virtual double getMinimumDistance(const Eigen::Vector2d& position) const override
  {
    double z_r = querySurfaceZ(position.x(), position.y(), center_3d_.z());

    // 水平到圆柱侧壁距离
    double dxy = std::max(0.0, (position - center_3d_.head<2>()).norm() - radius_);

    // 垂直距离
    double dz = 0.0;
    if (z_r < z_min_)
    {
      dz = z_min_ - z_r;
    }
    else if (z_r > z_max_)
    {
      dz = z_r - z_max_;
    }

    return std::hypot(dxy, dz);
  }

  virtual bool checkCollision(const Eigen::Vector2d& position, double min_dist) const override
  {
    return getMinimumDistance(position) <= min_dist;
  }

private:
  Eigen::Vector3d center_3d_;
  double radius_;
  double z_min_;
  double z_max_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/**
 * @class PolygonObstacle3D
 * @brief 真实三维多棱柱体障碍物（如箱体、异形墙面）
 */
class PolygonObstacle3D : public teb_local_planner::PolygonObstacle
{
public:
  PolygonObstacle3D(const std::vector<Eigen::Vector3d>& vertices, double z_min, double z_max)
    : teb_local_planner::PolygonObstacle(),
      vertices_3d_(vertices), z_min_(z_min), z_max_(z_max)
  {
    for (const auto & v : vertices)
    {
      pushBackVertex(v.head<2>());
    }
    finalizePolygon();
  }

  virtual ~PolygonObstacle3D() = default;

  virtual double getMinimumDistance(const Eigen::Vector2d& position) const override
  {
    double ref_z = (z_min_ + z_max_) * 0.5;
    double z_r = querySurfaceZ(position.x(), position.y(), ref_z);

    // 水平距离通过基类多边形计算
    double dxy = teb_local_planner::PolygonObstacle::getMinimumDistance(position);

    // 垂直距离
    double dz = 0.0;
    if (z_r < z_min_)
    {
      dz = z_min_ - z_r;
    }
    else if (z_r > z_max_)
    {
      dz = z_r - z_max_;
    }

    return std::hypot(dxy, dz);
  }

  virtual bool checkCollision(const Eigen::Vector2d& position, double min_dist) const override
  {
    return getMinimumDistance(position) <= min_dist;
  }

private:
  std::vector<Eigen::Vector3d> vertices_3d_;
  double z_min_;
  double z_max_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace elevation_local_planner
