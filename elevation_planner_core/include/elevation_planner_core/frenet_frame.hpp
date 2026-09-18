#pragma once

#include "elevation_planner_core/manifold_graph.hpp"
#include "elevation_planner_core/topological_corridor.hpp"
#include <geometry_msgs/PoseStamped.h>
#include <tf2/utils.h>
#include <angles/angles.h>
#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>
#include <memory>

namespace elevation_planner
{

/**
 * @brief Frenet 航点参数结构体
 */
struct FrenetWaypoint
{
  double s{0.0};          ///< 沿 3D 参考路径的真实空间累积弧长 (m)
  double x{0.0};          ///< 3D 参考点 X (世界坐标系)
  double y{0.0};          ///< 3D 参考点 Y (世界坐标系)
  double z{0.0};          ///< 3D 参考点 Z (世界坐标系，真实高程踏面)
  double yaw{0.0};        ///< 水平切线航向角 (rad)
  double pitch{0.0};      ///< 纵向切线坡度/俯仰角 (rad)
  double curvature{0.0};  ///< 水平切线曲率 kappa (1/m)
  double nx{0.0};         ///< 横向左法向量 X: -sin(yaw)
  double ny{0.0};         ///< 横向左法向量 Y: cos(yaw)
  double left_width{0.5}; ///< 当前断面左侧走廊半宽限值 (m)
  double right_width{0.5};///< 当前断面右侧走廊半宽限值 (m)
};

/**
 * @class FrenetFrame
 * @brief 沿 3D 流形参考路径的 Frenet (s, l) 参数化坐标转换与走廊约束引擎
 * 
 * 核心功能：
 * 1. 消除 3D 楼梯、坡道与多层重叠在 2D 投影中的多义性与重叠截断；
 * 2. 纵向弧长 s 严格按真实 3D 位移累积 (ds = sqrt(dx^2 + dy^2 + dz^2))，天然适应大坡度与阶梯；
 * 3. 横向偏距 l 沿踏面水平切线法向展开，正值代表左偏，负值代表右偏；
 * 4. 解析输出局部切线航向、坡度角及曲率，为 TEB 输出提供精确微分几何曲率前馈；
 * 5. 将 3D 拓扑管道断面直接映射为 Frenet 左右硬边界 LineObstacle。
 */
class FrenetFrame
{
public:
  FrenetFrame() = default;

  explicit FrenetFrame(const std::vector<geometry_msgs::PoseStamped>& path)
  {
    initialize(path);
  }

  /**
   * @brief 根据三维全局/局部参考路径初始化 Frenet 坐标系
   */
  bool initialize(const std::vector<geometry_msgs::PoseStamped>& path)
  {
    waypoints_.clear();
    total_length_ = 0.0;

    if (path.empty()) return false;

    // 1. 过滤相距极近的冗余重合点
    std::vector<geometry_msgs::PoseStamped> clean_path;
    clean_path.reserve(path.size());
    for (const auto& p : path)
    {
      if (clean_path.empty())
      {
        clean_path.push_back(p);
      }
      else
      {
        double dx = p.pose.position.x - clean_path.back().pose.position.x;
        double dy = p.pose.position.y - clean_path.back().pose.position.y;
        double dz = p.pose.position.z - clean_path.back().pose.position.z;
        if (dx * dx + dy * dy + dz * dz > 1e-4) // > 1cm 间距
        {
          clean_path.push_back(p);
        }
      }
    }

    if (clean_path.size() < 2)
    {
      // 若只有 1 点，沿航向延伸虚拟点以形成微元坐标系
      if (clean_path.size() == 1)
      {
        geometry_msgs::PoseStamped p2 = clean_path.front();
        double yaw = tf2::getYaw(p2.pose.orientation);
        p2.pose.position.x += 0.20 * std::cos(yaw);
        p2.pose.position.y += 0.20 * std::sin(yaw);
        clean_path.push_back(p2);
      }
      else
      {
        return false;
      }
    }

    // 2. 累积真实三维空间几何弧长
    const size_t N = clean_path.size();
    waypoints_.resize(N);

    waypoints_[0].s = 0.0;
    waypoints_[0].x = clean_path[0].pose.position.x;
    waypoints_[0].y = clean_path[0].pose.position.y;
    waypoints_[0].z = clean_path[0].pose.position.z;

    for (size_t i = 1; i < N; ++i)
    {
      double dx = clean_path[i].pose.position.x - clean_path[i - 1].pose.position.x;
      double dy = clean_path[i].pose.position.y - clean_path[i - 1].pose.position.y;
      double dz = clean_path[i].pose.position.z - clean_path[i - 1].pose.position.z;
      double ds = std::sqrt(dx * dx + dy * dy + dz * dz);

      waypoints_[i].s = waypoints_[i - 1].s + ds;
      waypoints_[i].x = clean_path[i].pose.position.x;
      waypoints_[i].y = clean_path[i].pose.position.y;
      waypoints_[i].z = clean_path[i].pose.position.z;
    }
    total_length_ = waypoints_.back().s;

    // 3. 计算切线航向、俯仰角、法向量与曲率
    for (size_t i = 0; i < N; ++i)
    {
      double dx = 0.0, dy = 0.0, dz = 0.0;
      if (i + 1 < N)
      {
        dx = waypoints_[i + 1].x - waypoints_[i].x;
        dy = waypoints_[i + 1].y - waypoints_[i].y;
        dz = waypoints_[i + 1].z - waypoints_[i].z;
      }
      else
      {
        dx = waypoints_[i].x - waypoints_[i - 1].x;
        dy = waypoints_[i].y - waypoints_[i - 1].y;
        dz = waypoints_[i].z - waypoints_[i - 1].z;
      }

      double dxy = std::hypot(dx, dy);
      waypoints_[i].yaw = std::atan2(dy, dx);
      waypoints_[i].pitch = std::atan2(dz, std::max(1e-4, dxy));
      waypoints_[i].nx = -std::sin(waypoints_[i].yaw);
      waypoints_[i].ny = std::cos(waypoints_[i].yaw);
    }

    // 4. 曲率 kappa 计算 (三点差分平滑)
    for (size_t i = 0; i < N; ++i)
    {
      if (i > 0 && i + 1 < N)
      {
        double dyaw = angles::shortest_angular_distance(waypoints_[i - 1].yaw, waypoints_[i + 1].yaw);
        double ds = waypoints_[i + 1].s - waypoints_[i - 1].s;
        waypoints_[i].curvature = (ds > 1e-3) ? (dyaw / ds) : 0.0;
      }
      else if (i == 0 && N > 1)
      {
        double dyaw = angles::shortest_angular_distance(waypoints_[0].yaw, waypoints_[1].yaw);
        double ds = waypoints_[1].s - waypoints_[0].s;
        waypoints_[i].curvature = (ds > 1e-3) ? (dyaw / ds) : 0.0;
      }
      else if (i + 1 == N && N > 1)
      {
        waypoints_[i].curvature = waypoints_[i - 1].curvature;
      }
      // 限幅保护防数值飞溅
      waypoints_[i].curvature = std::max(-3.0, std::min(3.0, waypoints_[i].curvature));
    }

    return true;
  }

  /**
   * @brief 获取参考路径总弧长 (m)
   */
  inline double totalLength() const { return total_length_; }

  /**
   * @brief 航点数量
   */
  inline size_t size() const { return waypoints_.size(); }

  /**
   * @brief 是否为空
   */
  inline bool empty() const { return waypoints_.empty(); }

  /**
   * @brief 航点只读访问
   */
  inline const std::vector<FrenetWaypoint>& waypoints() const { return waypoints_; }

  /**
   * @brief 将三维空间点 (x, y, z) 投影至 Frenet 坐标系 (s, l)
   * 
   * @param x, y, z 待投影的三维点
   * @param s_out 输出累积弧长 (m)
   * @param l_out 输出横向偏距 (m)，左正右负
   * @param z_proj_out 投影点在参考线上对应的高程 (m)
   * @param heading_error_out 可选输出参考线切线方向与给定方向的夹角
   * @return double 3D 空间到参考线的正交距离 (欧氏距离)
   */
  double toFrenet(double x, double y, double z,
                  double& s_out, double& l_out, double& z_proj_out,
                  double* heading_error_out = nullptr) const
  {
    s_out = 0.0;
    l_out = 0.0;
    z_proj_out = z;
    if (waypoints_.empty()) return 0.0;

    if (waypoints_.size() == 1)
    {
      s_out = 0.0;
      double dx = x - waypoints_[0].x;
      double dy = y - waypoints_[0].y;
      l_out = dx * waypoints_[0].nx + dy * waypoints_[0].ny;
      z_proj_out = waypoints_[0].z;
      return std::sqrt(dx * dx + dy * dy + std::pow(z - waypoints_[0].z, 2));
    }

    double min_dist_sq = std::numeric_limits<double>::max();
    size_t best_seg = 0;
    double best_t = 0.0;

    for (size_t i = 0; i + 1 < waypoints_.size(); ++i)
    {
      const auto& p1 = waypoints_[i];
      const auto& p2 = waypoints_[i + 1];

      double vx = p2.x - p1.x;
      double vy = p2.y - p1.y;
      double vz = p2.z - p1.z;
      double v_sq = vx * vx + vy * vy + vz * vz;
      if (v_sq < 1e-8) continue;

      double wx = x - p1.x;
      double wy = y - p1.y;
      double wz = z - p1.z;

      double t = (wx * vx + wy * vy + wz * vz) / v_sq;
      t = std::max(0.0, std::min(1.0, t));

      double px = p1.x + t * vx;
      double py = p1.y + t * vy;
      double pz = p1.z + t * vz;

      double d_sq = std::pow(x - px, 2) + std::pow(y - py, 2) + std::pow(z - pz, 2);
      if (d_sq < min_dist_sq)
      {
        min_dist_sq = d_sq;
        best_seg = i;
        best_t = t;
      }
    }

    const auto& p1 = waypoints_[best_seg];
    const auto& p2 = waypoints_[best_seg + 1];

    s_out = p1.s + best_t * (p2.s - p1.s);
    double px = p1.x + best_t * (p2.x - p1.x);
    double py = p1.y + best_t * (p2.y - p1.y);
    z_proj_out = p1.z + best_t * (p2.z - p1.z);

    // 采用插值切向法向
    double interp_yaw = p1.yaw + best_t * angles::shortest_angular_distance(p1.yaw, p2.yaw);
    double nx = -std::sin(interp_yaw);
    double ny = std::cos(interp_yaw);

    l_out = (x - px) * nx + (y - py) * ny;

    if (heading_error_out)
    {
      *heading_error_out = interp_yaw;
    }

    return std::sqrt(min_dist_sq);
  }

  /**
   * @brief 将 Frenet 坐标 (s, l) 严格反变换为三维世界坐标 (x, y, z, yaw)
   * 
   * @param s 纵向弧长 (m)
   * @param l 横向偏距 (m)
   * @param x_out, y_out, z_out 输出三维坐标
   * @param yaw_out 输出参考切向角
   */
  bool toCartesian(double s, double l,
                   double& x_out, double& y_out, double& z_out, double& yaw_out) const
  {
    if (waypoints_.empty()) return false;

    if (s <= waypoints_.front().s)
    {
      const auto& p = waypoints_.front();
      x_out = p.x + l * p.nx;
      y_out = p.y + l * p.ny;
      z_out = p.z;
      yaw_out = p.yaw;
      return true;
    }

    if (s >= waypoints_.back().s)
    {
      const auto& p = waypoints_.back();
      x_out = p.x + l * p.nx;
      y_out = p.y + l * p.ny;
      z_out = p.z;
      yaw_out = p.yaw;
      return true;
    }

    // 二分查找对应弧长线段
    auto it = std::upper_bound(waypoints_.begin(), waypoints_.end(), s,
                               [](double val, const FrenetWaypoint& wp) {
                                 return val < wp.s;
                               });
    size_t idx2 = std::distance(waypoints_.begin(), it);
    size_t idx1 = idx2 - 1;

    const auto& p1 = waypoints_[idx1];
    const auto& p2 = waypoints_[idx2];

    double ds = p2.s - p1.s;
    double t = (ds > 1e-6) ? ((s - p1.s) / ds) : 0.0;
    t = std::max(0.0, std::min(1.0, t));

    double ref_x = p1.x + t * (p2.x - p1.x);
    double ref_y = p1.y + t * (p2.y - p1.y);
    z_out = p1.z + t * (p2.z - p1.z);

    yaw_out = p1.yaw + t * angles::shortest_angular_distance(p1.yaw, p2.yaw);
    yaw_out = angles::normalize_angle(yaw_out);

    double nx = -std::sin(yaw_out);
    double ny = std::cos(yaw_out);

    x_out = ref_x + l * nx;
    y_out = ref_y + l * ny;
    return true;
  }

  /**
   * @brief 在指定弧长 s 处获取切线曲率 kappa (1/m)
   */
  double getCurvature(double s) const
  {
    if (waypoints_.empty()) return 0.0;
    if (s <= waypoints_.front().s) return waypoints_.front().curvature;
    if (s >= waypoints_.back().s) return waypoints_.back().curvature;

    auto it = std::upper_bound(waypoints_.begin(), waypoints_.end(), s,
                               [](double val, const FrenetWaypoint& wp) {
                                 return val < wp.s;
                               });
    size_t idx2 = std::distance(waypoints_.begin(), it);
    size_t idx1 = idx2 - 1;

    double t = (waypoints_[idx2].s - waypoints_[idx1].s > 1e-6)
                   ? (s - waypoints_[idx1].s) / (waypoints_[idx2].s - waypoints_[idx1].s)
                   : 0.0;
    return (1.0 - t) * waypoints_[idx1].curvature + t * waypoints_[idx2].curvature;
  }

  /**
   * @brief 在指定弧长 s 处获取切线坡度/俯仰角 alpha (rad)
   */
  double getSlope(double s) const
  {
    if (waypoints_.empty()) return 0.0;
    if (s <= waypoints_.front().s) return waypoints_.front().pitch;
    if (s >= waypoints_.back().s) return waypoints_.back().pitch;

    auto it = std::upper_bound(waypoints_.begin(), waypoints_.end(), s,
                               [](double val, const FrenetWaypoint& wp) {
                                 return val < wp.s;
                               });
    size_t idx2 = std::distance(waypoints_.begin(), it);
    size_t idx1 = idx2 - 1;

    double t = (waypoints_[idx2].s - waypoints_[idx1].s > 1e-6)
                   ? (s - waypoints_[idx1].s) / (waypoints_[idx2].s - waypoints_[idx1].s)
                   : 0.0;
    return (1.0 - t) * waypoints_[idx1].pitch + t * waypoints_[idx2].pitch;
  }

  /**
   * @brief 在指定弧长 s 处获取切线水平航向角 (rad)
   */
  double getTangentYaw(double s) const
  {
    if (waypoints_.empty()) return 0.0;
    if (s <= waypoints_.front().s) return waypoints_.front().yaw;
    if (s >= waypoints_.back().s) return waypoints_.back().yaw;

    auto it = std::upper_bound(waypoints_.begin(), waypoints_.end(), s,
                               [](double val, const FrenetWaypoint& wp) {
                                 return val < wp.s;
                               });
    size_t idx2 = std::distance(waypoints_.begin(), it);
    size_t idx1 = idx2 - 1;

    double t = (waypoints_[idx2].s - waypoints_[idx1].s > 1e-6)
                   ? (s - waypoints_[idx1].s) / (waypoints_[idx2].s - waypoints_[idx1].s)
                   : 0.0;
    return angles::normalize_angle(
        waypoints_[idx1].yaw + t * angles::shortest_angular_distance(waypoints_[idx1].yaw, waypoints_[idx2].yaw));
  }

  /**
   * @brief 获取指定弧长处的走廊左限宽
   */
  double getLeftWidth(double s) const
  {
    if (waypoints_.empty()) return 0.5;
    if (s <= waypoints_.front().s) return waypoints_.front().left_width;
    if (s >= waypoints_.back().s) return waypoints_.back().left_width;

    auto it = std::upper_bound(waypoints_.begin(), waypoints_.end(), s,
                               [](double val, const FrenetWaypoint& wp) {
                                 return val < wp.s;
                               });
    size_t idx2 = std::distance(waypoints_.begin(), it);
    size_t idx1 = idx2 - 1;

    double t = (waypoints_[idx2].s - waypoints_[idx1].s > 1e-6)
                   ? (s - waypoints_[idx1].s) / (waypoints_[idx2].s - waypoints_[idx1].s)
                   : 0.0;
    return (1.0 - t) * waypoints_[idx1].left_width + t * waypoints_[idx2].left_width;
  }

  /**
   * @brief 获取指定弧长处的走廊右限宽
   */
  double getRightWidth(double s) const
  {
    if (waypoints_.empty()) return 0.5;
    if (s <= waypoints_.front().s) return waypoints_.front().right_width;
    if (s >= waypoints_.back().s) return waypoints_.back().right_width;

    auto it = std::upper_bound(waypoints_.begin(), waypoints_.end(), s,
                               [](double val, const FrenetWaypoint& wp) {
                                 return val < wp.s;
                               });
    size_t idx2 = std::distance(waypoints_.begin(), it);
    size_t idx1 = idx2 - 1;

    double t = (waypoints_[idx2].s - waypoints_[idx1].s > 1e-6)
                   ? (s - waypoints_[idx1].s) / (waypoints_[idx2].s - waypoints_[idx1].s)
                   : 0.0;
    return (1.0 - t) * waypoints_[idx1].right_width + t * waypoints_[idx2].right_width;
  }

  /**
   * @brief 基于流形连通图与拓扑管道精确推导沿路径每一断面横向通行半宽 W_left(s) 与 W_right(s)
   */
  void computeCorridorWidth(const ManifoldGraph& graph,
                            const TopologicalCorridor& corridor,
                            double max_radius = 2.5,
                            double max_step_height = 0.25,
                            double dog_height = 0.45)
  {
    if (waypoints_.empty() || graph.numNodes() == 0) return;

    const double step = 0.06; // 6cm 射线步进
    const float max_dz = static_cast<float>(max_step_height + 0.05);

    for (auto& wp : waypoints_)
    {
      // 1. 左向射线检测 (+nx, +ny)
      double wl = max_radius;
      for (double r = 0.10; r <= max_radius; r += step)
      {
        double qx = wp.x + r * wp.nx;
        double qy = wp.y + r * wp.ny;
        double qz = wp.z;

        uint32_t nid = 0;
        if (!graph.findClosestNode(qx, qy, qz, nid, 0.15, max_dz))
        {
          wl = std::max(0.18, r - step * 0.5);
          break;
        }

        const auto& nd = graph.getNode(nid);
        if (std::abs(nd.z - wp.z) > max_dz ||
            nd.traversability >= 0.8f ||
            (nd.headroom > 0.0f && nd.headroom < dog_height) ||
            (!corridor.empty() && !corridor.isInCorridor(nid)))
        {
          wl = std::max(0.18, r - step * 0.5);
          break;
        }
      }
      wp.left_width = wl;

      // 2. 右向射线检测 (-nx, -ny)
      double wr = max_radius;
      for (double r = 0.10; r <= max_radius; r += step)
      {
        double qx = wp.x - r * wp.nx;
        double qy = wp.y - r * wp.ny;
        double qz = wp.z;

        uint32_t nid = 0;
        if (!graph.findClosestNode(qx, qy, qz, nid, 0.15, max_dz))
        {
          wr = std::max(0.18, r - step * 0.5);
          break;
        }

        const auto& nd = graph.getNode(nid);
        if (std::abs(nd.z - wp.z) > max_dz ||
            nd.traversability >= 0.8f ||
            (nd.headroom > 0.0f && nd.headroom < dog_height) ||
            (!corridor.empty() && !corridor.isInCorridor(nid)))
        {
          wr = std::max(0.18, r - step * 0.5);
          break;
        }
      }
      wp.right_width = wr;
    }

    // 3. 走廊限宽平滑 (防止阶梯侧缘单点噪点造成剧烈突变)
    if (waypoints_.size() >= 3)
    {
      std::vector<double> smooth_l(waypoints_.size());
      std::vector<double> smooth_r(waypoints_.size());
      smooth_l.front() = waypoints_.front().left_width;
      smooth_r.front() = waypoints_.front().right_width;
      smooth_l.back() = waypoints_.back().left_width;
      smooth_r.back() = waypoints_.back().right_width;

      for (size_t i = 1; i + 1 < waypoints_.size(); ++i)
      {
        smooth_l[i] = 0.25 * waypoints_[i - 1].left_width + 0.50 * waypoints_[i].left_width + 0.25 * waypoints_[i + 1].left_width;
        smooth_r[i] = 0.25 * waypoints_[i - 1].right_width + 0.50 * waypoints_[i].right_width + 0.25 * waypoints_[i + 1].right_width;
      }

      for (size_t i = 0; i < waypoints_.size(); ++i)
      {
        waypoints_[i].left_width = smooth_l[i];
        waypoints_[i].right_width = smooth_r[i];
      }
    }
  }

  /**
   * @brief 生成与 TEB 内部约束 100% 对应的 3D 走廊连续曲面与发光护栏 Marker
   */
  void toCorridorMarkers(const std::string& frame_id,
                         visualization_msgs::MarkerArray& out_markers,
                         double wall_height = 0.35) const
  {
    out_markers.markers.clear();
    if (waypoints_.size() < 2) return;

    ros::Time now = ros::Time::now();

    visualization_msgs::Marker line_marker;
    line_marker.header.frame_id = frame_id;
    line_marker.header.stamp = now;
    line_marker.ns = "corridor_boundaries";
    line_marker.id = 0;
    line_marker.type = visualization_msgs::Marker::LINE_LIST;
    line_marker.action = visualization_msgs::Marker::ADD;
    line_marker.scale.x = 0.025; // 2.5cm 线宽
    line_marker.color.r = 0.0f;
    line_marker.color.g = 0.95f;
    line_marker.color.b = 1.0f;
    line_marker.color.a = 0.90f;

    visualization_msgs::Marker wall_marker;
    wall_marker.header.frame_id = frame_id;
    wall_marker.header.stamp = now;
    wall_marker.ns = "corridor_walls";
    wall_marker.id = 1;
    wall_marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
    wall_marker.action = visualization_msgs::Marker::ADD;
    wall_marker.scale.x = 1.0;
    wall_marker.scale.y = 1.0;
    wall_marker.scale.z = 1.0;
    wall_marker.color.r = 0.0f;
    wall_marker.color.g = 0.80f;
    wall_marker.color.b = 0.95f;
    wall_marker.color.a = 0.25f;

    const size_t N = waypoints_.size();
    for (size_t i = 0; i + 1 < N; ++i)
    {
      const auto& p1 = waypoints_[i];
      const auto& p2 = waypoints_[i + 1];

      geometry_msgs::Point l1, r1, l2, r2;
      geometry_msgs::Point l1_top, r1_top, l2_top, r2_top;

      l1.x = p1.x + p1.left_width * p1.nx;
      l1.y = p1.y + p1.left_width * p1.ny;
      l1.z = p1.z;

      r1.x = p1.x - p1.right_width * p1.nx;
      r1.y = p1.y - p1.right_width * p1.ny;
      r1.z = p1.z;

      l2.x = p2.x + p2.left_width * p2.nx;
      l2.y = p2.y + p2.left_width * p2.ny;
      l2.z = p2.z;

      r2.x = p2.x - p2.right_width * p2.nx;
      r2.y = p2.y - p2.right_width * p2.ny;
      r2.z = p2.z;

      l1_top = l1; l1_top.z += wall_height;
      r1_top = r1; r1_top.z += wall_height;
      l2_top = l2; l2_top.z += wall_height;
      r2_top = r2; r2_top.z += wall_height;

      // 1. 边界线 (左底线, 右底线, 左顶轨, 右顶轨)
      line_marker.points.push_back(l1); line_marker.points.push_back(l2);
      line_marker.points.push_back(r1); line_marker.points.push_back(r2);
      line_marker.points.push_back(l1_top); line_marker.points.push_back(l2_top);
      line_marker.points.push_back(r1_top); line_marker.points.push_back(r2_top);

      // 每隔 3 个点加一条横向肋线和垂直立柱
      if (i % 3 == 0 || i + 2 == N)
      {
        line_marker.points.push_back(l1); line_marker.points.push_back(r1);
        line_marker.points.push_back(l1); line_marker.points.push_back(l1_top);
        line_marker.points.push_back(r1); line_marker.points.push_back(r1_top);
      }

      // 2. 踏面缎带底面 (Ribbon Floor)
      wall_marker.points.push_back(l1); wall_marker.points.push_back(r1); wall_marker.points.push_back(l2);
      wall_marker.points.push_back(r1); wall_marker.points.push_back(r2); wall_marker.points.push_back(l2);

      // 3. 左侧护栏侧壁
      wall_marker.points.push_back(l1); wall_marker.points.push_back(l1_top); wall_marker.points.push_back(l2);
      wall_marker.points.push_back(l1_top); wall_marker.points.push_back(l2_top); wall_marker.points.push_back(l2);

      // 4. 右侧护栏侧壁
      wall_marker.points.push_back(r1); wall_marker.points.push_back(r2); wall_marker.points.push_back(r1_top);
      wall_marker.points.push_back(r2); wall_marker.points.push_back(r2_top); wall_marker.points.push_back(r1_top);
    }

    out_markers.markers.push_back(line_marker);
    out_markers.markers.push_back(wall_marker);
  }

private:
  std::vector<FrenetWaypoint> waypoints_;
  double total_length_{0.0};
};

} // namespace elevation_planner
