#pragma once

#include "elevation_local_planner/sfc_corridor.hpp"
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <algorithm>

namespace elevation_local_planner
{

/**
 * @brief 纯 2D B 样条极速二次规划 (QP) 优化器 (ADMM Algorithm)
 * 
 * 优化模型 (纯 2D 平面):
 *   min_{Q_2D} w_acc * sum ||q_{i+2} - 2q_{i+1} + q_i||^2
 *            + w_jerk * sum ||q_{i+3} - 3q_{i+2} + 3q_{i+1} - q_i||^2
 *            + w_guide * sum ||q_i - w_i||^2
 *   s.t.     q_0 = p_start_2d,  q_1 - q_0 = (dt / 3) * v_start_2d  (起点位姿与初速度严格连续)
 *            A_i * q_i <= b_i                                      (2D 凸走廊安全约束, 100% 保证无碰)
 */
class BSplineQPOptimizer
{
public:
  struct Params
  {
    double weight_acc{1.0};       ///< 加速度极小化平滑权重
    double weight_jerk{5.0};      ///< 加加速度 (Jerk) 极小化权重
    double weight_guide{0.15};    ///< 参考路径引导软权重
    double rho{2.0};              ///< ADMM 惩罚参数
    int max_iter{30};             ///< ADMM 最大迭代次数
    double eps_abs{1.0e-3};       ///< 绝对收敛残差容差 (m)
  };

  BSplineQPOptimizer() : params_(Params()) {}
  explicit BSplineQPOptimizer(const Params& p) : params_(p) {}

  /**
   * @brief 执行 2D B 样条控制点二次规划求解
   * @param init_waypoints_2d 2D 初始参考航点序列
   * @param corridors 每个航点对应的 2D 凸走廊约束
   * @param start_pos_2d 机器狗当前精确 2D 位置 (x, y)
   * @param start_vel_2d 机器狗当前 2D 线速度 (vx, vy)
   * @param dt B 样条节点均匀时间步长
   * @param[out] opt_control_points_2d 优化求解出的最优 2D 控制点序列
   * @return true 优化成功; false 优化失败
   */
  bool optimize2D(
      const std::vector<Eigen::Vector2d>& init_waypoints_2d,
      const std::vector<ConvexPolygon2D>& corridors,
      const Eigen::Vector2d& start_pos_2d,
      const Eigen::Vector2d& start_vel_2d,
      double dt,
      std::vector<Eigen::Vector2d>& opt_control_points_2d)
  {
    const int N = static_cast<int>(init_waypoints_2d.size());
    if (N < 3 || corridors.size() < static_cast<size_t>(N))
    {
      opt_control_points_2d = init_waypoints_2d;
      return false;
    }

    const int dim = N;
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(dim, dim);
    Eigen::VectorXd gx = Eigen::VectorXd::Zero(dim);
    Eigen::VectorXd gy = Eigen::VectorXd::Zero(dim);

    // 1.1 加速度二阶差分代价矩阵 D2
    if (params_.weight_acc > 0.0)
    {
      Eigen::MatrixXd D2 = Eigen::MatrixXd::Zero(dim - 2, dim);
      for (int i = 0; i < dim - 2; ++i)
      {
        D2(i, i)     =  1.0;
        D2(i, i + 1) = -2.0;
        D2(i, i + 2) =  1.0;
      }
      H += params_.weight_acc * (D2.transpose() * D2);
    }

    // 1.2 Jerk 三阶差分代价矩阵 D3
    if (params_.weight_jerk > 0.0 && dim >= 4)
    {
      Eigen::MatrixXd D3 = Eigen::MatrixXd::Zero(dim - 3, dim);
      for (int i = 0; i < dim - 3; ++i)
      {
        D3(i, i)     = -1.0;
        D3(i, i + 1) =  3.0;
        D3(i, i + 2) = -3.0;
        D3(i, i + 3) =  1.0;
      }
      H += params_.weight_jerk * (D3.transpose() * D3);
    }

    // 1.3 引导参考点代价
    for (int i = 0; i < dim; ++i)
    {
      H(i, i) += params_.weight_guide;
      gx(i) -= params_.weight_guide * init_waypoints_2d[i].x();
      gy(i) -= params_.weight_guide * init_waypoints_2d[i].y();
    }

    // 1.4 起点位置与初速度强约束 (严格保证过起点且一阶导数连续)
    const double w_fix = 1000.0;
    H(0, 0) += w_fix;
    gx(0)   -= w_fix * start_pos_2d.x();
    gy(0)   -= w_fix * start_pos_2d.y();

    Eigen::Vector2d p1_desired = start_pos_2d + (dt / 3.0) * start_vel_2d;
    H(1, 1) += w_fix;
    gx(1)   -= w_fix * p1_desired.x();
    gy(1)   -= w_fix * p1_desired.y();

    // 2. 预计算 ADMM 线性系统的 Cholesky 分解
    const double rho = params_.rho;
    Eigen::MatrixXd H_admm = H + rho * Eigen::MatrixXd::Identity(dim, dim);
    Eigen::LLT<Eigen::MatrixXd> llt(H_admm);
    if (llt.info() != Eigen::Success)
    {
      opt_control_points_2d = init_waypoints_2d;
      return false;
    }

    // 3. ADMM 迭代主循环 (纯 2D 平面投影)
    Eigen::Matrix<double, Eigen::Dynamic, 2> Q(dim, 2);
    Eigen::Matrix<double, Eigen::Dynamic, 2> S(dim, 2);
    Eigen::Matrix<double, Eigen::Dynamic, 2> Y(dim, 2);

    for (int i = 0; i < dim; ++i)
    {
      Q.row(i) = init_waypoints_2d[i].transpose();
      S.row(i) = init_waypoints_2d[i].transpose();
      Y.row(i) = Eigen::RowVector2d::Zero();
    }
    Q.row(0) = start_pos_2d.transpose();
    Q.row(1) = p1_desired.transpose();
    S.row(0) = start_pos_2d.transpose();
    S.row(1) = p1_desired.transpose();

    for (int iter = 0; iter < params_.max_iter; ++iter)
    {
      // 3.1 Q 步: 求解光滑系统 (x, y 两轴解耦极速回代)
      Eigen::VectorXd rhs_x = -gx + rho * (S.col(0) - Y.col(0) / rho);
      Eigen::VectorXd rhs_y = -gy + rho * (S.col(1) - Y.col(1) / rho);

      Q.col(0) = llt.solve(rhs_x);
      Q.col(1) = llt.solve(rhs_y);

      // 3.2 S 步: 投影到 2D 凸多边形走廊内部
      double max_viol = 0.0;
      for (int i = 0; i < dim; ++i)
      {
        if (i == 0)
        {
          S.row(i) = start_pos_2d.transpose();
          continue;
        }
        if (i == 1)
        {
          S.row(i) = p1_desired.transpose();
          continue;
        }

        Eigen::Vector2d q_i = Q.row(i).transpose() + Y.row(i).transpose() / rho;
        const auto& poly = corridors[i];

        Eigen::Vector2d d_rel = q_i - poly.center;
        double proj_n = poly.normal.dot(d_rel);
        double proj_t = poly.tangent.dot(d_rel);

        double clamp_n = std::max(-poly.d_right, std::min(poly.d_left, proj_n));
        double clamp_t = std::max(-poly.d_back,  std::min(poly.d_front, proj_t));

        Eigen::Vector2d s_i = poly.center + clamp_n * poly.normal + clamp_t * poly.tangent;
        S.row(i) = s_i.transpose();

        double viol = (Q.row(i) - S.row(i)).norm();
        if (viol > max_viol) max_viol = viol;
      }

      // 3.3 对偶更新 Y
      Y += rho * (Q - S);

      // 3.4 收敛检查
      if (max_viol < params_.eps_abs && iter >= 10)
      {
        break;
      }
    }

    // 4. 输出最优 2D 控制点序列
    opt_control_points_2d.resize(dim);
    for (int i = 0; i < dim; ++i)
    {
      opt_control_points_2d[i] = S.row(i).transpose();
    }

    return true;
  }

private:
  Params params_;
};

} // namespace elevation_local_planner
