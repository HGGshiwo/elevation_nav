#pragma once

#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>
#include <Eigen/Core>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include "elevation_planner_core/manifold_graph.hpp"
#include "elevation_local_planner/sfc_corridor.hpp"

namespace elevation_local_planner
{

/**
 * @brief 纯 2D 均匀三次 B 样条解析速度与走廊节点高程披覆引擎
 * 
 * 核心设计:
 * 1. 轨迹与导数在 2D 平面上纯解析求导 (无 Z 轴噪声，天然 C^2 平滑，输出精确 vx, vy, wz)；
 * 2. 轨迹 3D 高程完全通过走廊记录的图节点缓存查询，零全局图二次检索开销。
 */
class BSplineTrajectory
{
public:
  BSplineTrajectory() : dt_(0.1), total_time_(0.0), num_segments_(0), valid_(false) {}

  /**
   * @brief 从 2D 控制点序列初始化 B 样条
   * @param opt_control_points_2d 2D 最优控制点序列
   * @param dt 节点时间步长 (s)
   */
  bool initialize2D(const std::vector<Eigen::Vector2d>& opt_control_points_2d, double dt)
  {
    if (opt_control_points_2d.size() < 4)
    {
      valid_ = false;
      return false;
    }
    control_points_2d_ = opt_control_points_2d;
    dt_ = std::max(0.02, dt);
    num_segments_ = static_cast<int>(control_points_2d_.size()) - 3;
    total_time_ = num_segments_ * dt_;
    valid_ = true;
    return true;
  }

  /**
   * @brief 传统 2D 航点直接反解初始化 (兜底模式)
   */
  bool initialize(const std::vector<Eigen::Vector2d>& waypoints_2d, double target_speed = 0.6)
  {
    if (waypoints_2d.size() < 2)
    {
      valid_ = false;
      return false;
    }

    target_speed = std::max(0.1, target_speed);
    control_points_2d_.clear();
    control_points_2d_.reserve(waypoints_2d.size() + 2);

    Eigen::Vector2d c0 = 2.0 * waypoints_2d[0] - waypoints_2d[1];
    control_points_2d_.push_back(c0);
    for (const auto& pt : waypoints_2d)
    {
      control_points_2d_.push_back(pt);
    }
    size_t M = waypoints_2d.size() - 1;
    Eigen::Vector2d c_end = 2.0 * waypoints_2d[M] - waypoints_2d[M - 1];
    control_points_2d_.push_back(c_end);

    double total_dist = 0.0;
    for (size_t i = 1; i < waypoints_2d.size(); ++i)
    {
      total_dist += (waypoints_2d[i] - waypoints_2d[i - 1]).norm();
    }
    double avg_seg_len = total_dist / std::max(size_t(1), waypoints_2d.size() - 1);
    avg_seg_len = std::max(0.08, avg_seg_len);

    dt_ = avg_seg_len / target_speed;
    dt_ = std::max(0.05, std::min(0.30, dt_));

    num_segments_ = static_cast<int>(control_points_2d_.size()) - 3;
    total_time_ = num_segments_ * dt_;
    valid_ = true;
    return true;
  }

  bool isValid() const { return valid_; }
  double getTotalTime() const { return total_time_; }
  double getKnotInterval() const { return dt_; }

  /**
   * @brief 解析计算 t 时刻的 2D 位置 p(t) = [x, y]^T
   */
  Eigen::Vector2d evaluatePosition(double t) const
  {
    if (!valid_ || control_points_2d_.size() < 4) return Eigen::Vector2d::Zero();

    t = std::max(0.0, std::min(total_time_, t));
    int seg = static_cast<int>(t / dt_);
    if (seg >= num_segments_) seg = num_segments_ - 1;

    double u = (t - seg * dt_) / dt_;
    double u2 = u * u;
    double u3 = u2 * u;

    double b0 = (1.0 - 3.0 * u + 3.0 * u2 - u3) / 6.0;
    double b1 = (4.0 - 6.0 * u2 + 3.0 * u3) / 6.0;
    double b2 = (1.0 + 3.0 * u + 3.0 * u2 - 3.0 * u3) / 6.0;
    double b3 = u3 / 6.0;

    return b0 * control_points_2d_[seg] +
           b1 * control_points_2d_[seg + 1] +
           b2 * control_points_2d_[seg + 2] +
           b3 * control_points_2d_[seg + 3];
  }

  /**
   * @brief 解析计算 t 时刻的 2D 速度矢量 dp(t)/dt = [vx, vy]^T
   */
  Eigen::Vector2d evaluateVelocity(double t) const
  {
    if (!valid_ || control_points_2d_.size() < 4) return Eigen::Vector2d::Zero();

    t = std::max(0.0, std::min(total_time_, t));
    int seg = static_cast<int>(t / dt_);
    if (seg >= num_segments_) seg = num_segments_ - 1;

    double u = (t - seg * dt_) / dt_;
    double u2 = u * u;

    double db0 = (-3.0 + 6.0 * u - 3.0 * u2) / (6.0 * dt_);
    double db1 = (-12.0 * u + 9.0 * u2) / (6.0 * dt_);
    double db2 = (3.0 + 6.0 * u - 9.0 * u2) / (6.0 * dt_);
    double db3 = (3.0 * u2) / (6.0 * dt_);

    return db0 * control_points_2d_[seg] +
           db1 * control_points_2d_[seg + 1] +
           db2 * control_points_2d_[seg + 2] +
           db3 * control_points_2d_[seg + 3];
  }

  /**
   * @brief 解析计算 t 时刻的 2D 加速度矢量 d^2p(t)/dt^2 = [ax, ay]^T
   */
  Eigen::Vector2d evaluateAcceleration(double t) const
  {
    if (!valid_ || control_points_2d_.size() < 4) return Eigen::Vector2d::Zero();

    t = std::max(0.0, std::min(total_time_, t));
    int seg = static_cast<int>(t / dt_);
    if (seg >= num_segments_) seg = num_segments_ - 1;

    double u = (t - seg * dt_) / dt_;

    double ddb0 = (6.0 - 6.0 * u) / (6.0 * dt_ * dt_);
    double ddb1 = (-12.0 + 18.0 * u) / (6.0 * dt_ * dt_);
    double ddb2 = (6.0 - 18.0 * u) / (6.0 * dt_ * dt_);
    double ddb3 = (6.0 * u) / (6.0 * dt_ * dt_);

    return ddb0 * control_points_2d_[seg] +
           ddb1 * control_points_2d_[seg + 1] +
           ddb2 * control_points_2d_[seg + 2] +
           ddb3 * control_points_2d_[seg + 3];
  }

  /**
   * @brief 解析计算标量前向线速度 v = ||dp/dt||
   */
  double evaluateLinearSpeed(double t) const
  {
    return evaluateVelocity(t).norm();
  }

  /**
   * @brief 解析计算期望机头朝向角 yaw = atan2(vy, vx)
   */
  double evaluateTargetYaw(double t) const
  {
    Eigen::Vector2d v = evaluateVelocity(t);
    return std::atan2(v.y(), v.x());
  }

  /**
   * @brief 解析计算 2D 轨迹曲率 kappa = |x' y'' - y' x''| / (x'^2 + y'^2)^1.5
   */
  double evaluateCurvature(double t) const
  {
    Eigen::Vector2d v = evaluateVelocity(t);
    Eigen::Vector2d a = evaluateAcceleration(t);
    double v_sq = v.squaredNorm();
    if (v_sq < 1.0e-5) return 0.0;
    return std::abs(v.x() * a.y() - v.y() * a.x()) / std::pow(v_sq, 1.5);
  }

  /**
   * @brief 解析计算角速度 omega_z = (x' y'' - y' x'') / (x'^2 + y'^2)
   */
  double evaluateAngularVelocity(double t) const
  {
    Eigen::Vector2d v = evaluateVelocity(t);
    Eigen::Vector2d a = evaluateAcceleration(t);
    double v_sq = v.squaredNorm();
    if (v_sq < 1.0e-5) return 0.0;
    return (v.x() * a.y() - v.y() * a.x()) / v_sq;
  }

  /**
   * @brief 纯通过走廊已记录的图节点缓存计算高程生成 3D Path (零全局图查表)
   */
  nav_msgs::Path toPathMsgFromCorridors(
      const std::string& frame_id,
      const std::vector<ConvexCorridor2D>& corridors,
      double step_dt = 0.05) const
  {
    nav_msgs::Path path;
    path.header.frame_id = frame_id;
    path.header.stamp = ros::Time::now();

    if (!valid_ || total_time_ <= 0.0 || corridors.empty()) return path;

    std::vector<Eigen::Vector3d> draped_points;
    for (double t = 0.0; t <= total_time_; t += step_dt)
    {
      Eigen::Vector2d p2d = evaluatePosition(t);
      double s = (total_time_ > 1e-4) ? (t / total_time_) : 0.0;
      int corr_idx = static_cast<int>(std::round(s * (corridors.size() - 1)));
      corr_idx = std::max(0, std::min(static_cast<int>(corridors.size()) - 1, corr_idx));

      double z = corridors[corr_idx].queryZFromNodes(p2d.x(), p2d.y());
      draped_points.emplace_back(p2d.x(), p2d.y(), z);
    }

    if (draped_points.empty()) return path;

    path.poses.reserve(draped_points.size());
    ros::Time now = ros::Time::now();

    for (size_t i = 0; i < draped_points.size(); ++i)
    {
      geometry_msgs::PoseStamped ps;
      ps.header.frame_id = frame_id;
      ps.header.stamp = now;
      ps.pose.position.x = draped_points[i].x();
      ps.pose.position.y = draped_points[i].y();
      ps.pose.position.z = draped_points[i].z();

      double yaw = 0.0;
      if (i + 1 < draped_points.size())
      {
        double dx = draped_points[i + 1].x() - draped_points[i].x();
        double dy = draped_points[i + 1].y() - draped_points[i].y();
        if (std::hypot(dx, dy) > 1e-4) yaw = std::atan2(dy, dx);
      }
      else if (i > 0)
      {
        double dx = draped_points[i].x() - draped_points[i - 1].x();
        double dy = draped_points[i].y() - draped_points[i - 1].y();
        if (std::hypot(dx, dy) > 1e-4) yaw = std::atan2(dy, dx);
      }

      ps.pose.orientation.x = 0.0;
      ps.pose.orientation.y = 0.0;
      ps.pose.orientation.z = std::sin(yaw * 0.5);
      ps.pose.orientation.w = std::cos(yaw * 0.5);

      path.poses.push_back(ps);
    }

    return path;
  }

  /**
   * @brief 兼容接口
   */
  nav_msgs::Path toPathMsg(const std::string& frame_id,
                           const elevation_planner::ManifoldGraph* graph,
                           double start_z = 0.0,
                           double end_z = 0.0,
                           double step_dt = 0.05) const
  {
    nav_msgs::Path path;
    path.header.frame_id = frame_id;
    path.header.stamp = ros::Time::now();

    if (!valid_ || total_time_ <= 0.0) return path;

    std::vector<Eigen::Vector3d> draped_points;
    for (double t = 0.0; t <= total_time_; t += step_dt)
    {
      Eigen::Vector2d p2d = evaluatePosition(t);
      double s = (total_time_ > 1e-4) ? (t / total_time_) : 0.0;
      double expected_z = (1.0 - s) * start_z + s * end_z;
      double z = expected_z;
      if (graph)
      {
        int r = 0, c = 0;
        if (graph->toGridIndex(p2d.x(), p2d.y(), r, c))
        {
          const auto& cell_nodes = graph->getSpatialCellNodes(r, c);
          float min_dz = 999.0f;
          for (uint32_t nid : cell_nodes)
          {
            const auto& nd = graph->getNode(nid);
            float dz = std::abs(nd.z - static_cast<float>(expected_z));
            if (dz < min_dz && dz <= 0.30f)
            {
              min_dz = dz;
              z = nd.z;
            }
          }
        }
      }
      draped_points.emplace_back(p2d.x(), p2d.y(), z);
    }

    if (draped_points.empty()) return path;

    path.poses.reserve(draped_points.size());
    ros::Time now = ros::Time::now();

    for (size_t i = 0; i < draped_points.size(); ++i)
    {
      geometry_msgs::PoseStamped ps;
      ps.header.frame_id = frame_id;
      ps.header.stamp = now;
      ps.pose.position.x = draped_points[i].x();
      ps.pose.position.y = draped_points[i].y();
      ps.pose.position.z = draped_points[i].z();

      double yaw = 0.0;
      if (i + 1 < draped_points.size())
      {
        double dx = draped_points[i + 1].x() - draped_points[i].x();
        double dy = draped_points[i + 1].y() - draped_points[i].y();
        if (std::hypot(dx, dy) > 1e-4) yaw = std::atan2(dy, dx);
      }
      else if (i > 0)
      {
        double dx = draped_points[i].x() - draped_points[i - 1].x();
        double dy = draped_points[i].y() - draped_points[i - 1].y();
        if (std::hypot(dx, dy) > 1e-4) yaw = std::atan2(dy, dx);
      }

      ps.pose.orientation.x = 0.0;
      ps.pose.orientation.y = 0.0;
      ps.pose.orientation.z = std::sin(yaw * 0.5);
      ps.pose.orientation.w = std::cos(yaw * 0.5);

      path.poses.push_back(ps);
    }

    return path;
  }

private:
  std::vector<Eigen::Vector2d> control_points_2d_;
  double dt_{0.1};
  double total_time_{0.0};
  int num_segments_{0};
  bool valid_{false};
};

} // namespace elevation_local_planner
