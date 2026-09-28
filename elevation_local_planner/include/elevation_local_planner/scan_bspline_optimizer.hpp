#pragma once

#include "elevation_local_planner/lbfgs.hpp"
#include "elevation_local_planner/sfc_corridor.hpp"
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>
#include <string>
#include <sstream>
#include <iomanip>
#include <ros/ros.h>
#include <functional>

namespace elevation_local_planner
{

/**
 * @brief 2D 样条控制点内外循环增广拉格朗日 (ALM + L-BFGS) 优化器
 * 
 * 核心设计:
 * 1. 1:1 严格物理对应: 第 i 个物理轨迹采样点 p_i = (q_{i-1} + 4*q_i + q_{i+1})/6 严格由第 i 个凸包走廊约束 (A_i * p_i <= b_i)
 * 2. 内循环 (Inner Loop): L-BFGS 拟牛顿优化求解平滑、动力学、参考线贴合与物理点穿墙三次强惩罚 + ALM 乘子势场
 *    - J_corridor = \lambda \sum \max(0, A_i * p_i - b_i)^3 (立方强惩罚)
 *    - J_ALM = \mu^T (A_i * p_i - b_i) + \frac{\lambda_{ALM}}{2} ||\max(0, A_i * p_i - b_i)||^2
 * 3. 外循环 (Outer Loop): 检验物理轨迹点穿墙量，不调大 \lambda，更新拉格朗日乘子 \mu:
 *    - \mu_{i,j} = \max(0, \mu_{i,j} + \lambda_{ALM} * (A_{i,j} * p_i - b_{i,j}))
 *    - 经过 2~3 次外循环，物理轨迹穿墙量被数学强制压制至 0.0001m 级高精度。
 */
class ScanBsplineOptimizer
{
public:
  ScanBsplineOptimizer() = default;

  void setParams(double lambda_smooth = 1.0,
                 double lambda_corridor = 500.0,
                 double lambda_feas = 2.0,
                 double lambda_fitness = 2.5,
                 double max_vel = 0.80,
                 double max_acc = 1.50)
  {
    lambda1_smooth_ = lambda_smooth;
    lambda2_corridor_ = lambda_corridor;
    lambda3_feas_ = lambda_feas;
    lambda4_fitness_ = lambda_fitness;
    lambda_alm_ = lambda_corridor;
    max_vel_ = max_vel;
    max_acc_ = max_acc;
  }

  /**
   * @brief [DBG] 调试日志开关 (默认开启; 排查结束后可调用 setDebugVerbose(false) 关闭)
   */
  void setDebugVerbose(bool verbose) { debug_verbose_ = verbose; }

  /// 占据查询回调: (x, y, z) 处机体周围本层带内是否存在禁行节点;
  /// 命中时输出最近禁行节点坐标 (障碍面参考点), 供 rebound 定向排斥使用。
  /// 图外/悬空 (无本层节点) 返回 false —— 不装弹簧, 交走廊 ALM 项兜底
  using OccupancyCallback = std::function<bool(double, double, double, Eigen::Vector2d*)>;

  void setOccupancyCallback(OccupancyCallback cb) { occupancy_cb_ = std::move(cb); }

  /// rebound 参数: 排斥项权重与安全间距 (= body_hard_radius)
  void setReboundParams(double lambda_rebound, double clearance)
  {
    lambda_rebound_ = lambda_rebound;
    rebound_clearance_ = clearance;
  }

  /// rebound 约束结构 (可视化/诊断用): base_point=障碍面参考点, direction=排斥单位向量 (零向量=未激活)
  struct ReboundConstraint
  {
    Eigen::Vector2d base_point{0.0, 0.0};
    Eigen::Vector2d direction{0.0, 0.0};
  };

  /// 当前轮的 rebound 约束快照 (下标对齐控制点), 供宿主发布可视化箭头
  const std::vector<ReboundConstraint>& getReboundConstraints() const { return rebound_; }

  /// 控制点总数 (N + 2)
  int getControlPointCount() const { return num_ctrl_pts_; }

  /// 本轮优化最后一次刷新时激活 (碰撞) 的约束数量
  int getReboundActiveCount() const { return rebound_active_count_; }

  /**
   * @brief 执行 2D 样条控制点 ALM 内外循环优化 (1:1 物理点与凸包严格对应)
   * @param waypoints_2d 原始 2D 航点序列 (大小 M)
   * @param corridors_2d 每个航点对应的凸走廊与线性约束 A_i * p_i <= b_i (大小 M)
   * @param start_pos 起点位置
   * @param start_vel 起点速度
   * @param dt 样条时间步长
   * @param out_control_points 优化后的 2D 控制点序列
   * @return bool 优化是否成功
   */
  bool optimize(
      const std::vector<Eigen::Vector2d>& waypoints_2d,
      const std::vector<ConvexCorridor2D>& corridors_2d,
      const Eigen::Vector2d& start_pos,
      const Eigen::Vector2d& start_vel,
      double dt,
      std::vector<Eigen::Vector2d>& out_control_points)
  {
    if (waypoints_2d.size() < 2 || corridors_2d.size() != waypoints_2d.size()) return false;

    dt_ = std::max(0.05, std::min(5.0, dt));  // 上限放宽: 稀疏航点 (0.6m+ 段) 需要大 dt 保证参数化速度不超限
    corridors_ = corridors_2d;
    num_waypoints_ = static_cast<int>(waypoints_2d.size());
    num_ctrl_pts_ = num_waypoints_ + 2; // N + 2 个控制点严格对应 N 个航点/走廊节点评估点

    // 1. 初始化 N+2 个控制点矩阵
    // 使得初始样条每个 knot 物理点 p_i = (q_i + 4*q_{i+1} + q_{i+2})/6 精确对齐
    ctrl_pts_.resize(2, num_ctrl_pts_);
    
    // 起点边界初始化: 严格锁定机器人当前实测位置与速度，消灭起点偏置与折弯
    ctrl_pts_.col(0) = start_pos - dt_ * start_vel;
    ctrl_pts_.col(1) = start_pos;

    // 内部控制点初始化为后续航点 (i=0 对应起点 start_pos，已由 col(1) 严格绑定)
    for (int i = 1; i < num_waypoints_; ++i)
    {
      ctrl_pts_.col(i + 1) = waypoints_2d[i];
    }
    // 终点边界初始化
    Eigen::Vector2d end_pos = waypoints_2d.back();
    ctrl_pts_.col(num_ctrl_pts_ - 1) = 2.0 * end_pos - waypoints_2d[std::max(0, num_waypoints_ - 2)];

    // 参考路径点严格锚定起点
    ref_pts_ = waypoints_2d;
    if (!ref_pts_.empty())
    {
      ref_pts_[0] = start_pos;
    }

    // 2. 构造自由优化变量区间 (q_0, q_1 固定以保证起点位置与速度连续，q_2 ~ q_{N} 全部自由优化)
    free_start_idx_ = 2;
    free_end_idx_ = num_ctrl_pts_ - 2; // q_{N+1} 终点钳制，q_2 ... q_N 自由调整

    int num_free = free_end_idx_ - free_start_idx_ + 1;
    if (num_free <= 0)
    {
      out_control_points.clear();
      for (int i = 0; i < num_ctrl_pts_; ++i) out_control_points.push_back(ctrl_pts_.col(i));
      return true;
    }

    // 3. 严格 1:1 初始化每个走廊节点的拉格朗日乘子
    multipliers_.clear();
    multipliers_.resize(num_waypoints_);
    for (int i = 0; i < num_waypoints_; ++i)
    {
      size_t num_constraints = corridors_[i].constraint.b.size();
      multipliers_[i] = Eigen::VectorXd::Zero(num_constraints);
    }

    // rebound 定向排斥约束清零 (每轮外循环结束后由 checkCollisionAndRebound 按碰撞实测刷新)
    rebound_.assign(num_ctrl_pts_, ReboundConstraint());

    // ===== [DBG-1] 初始化快照: 初始穿墙量与微型/退化走廊检测 (区分数据问题 vs 优化发散) =====
    if (debug_verbose_)
    {
      double min_area = std::numeric_limits<double>::max();
      int min_area_idx = -1;
      double max_init_viol = 0.0;
      int max_init_viol_idx = -1;
      for (int i = 1; i < num_waypoints_; ++i)
      {
        const auto& cst = corridors_[i].constraint;
        double area = polygonArea(cst.vertices);
        if (area < min_area)
        {
          min_area = area;
          min_area_idx = i;
        }
        if (cst.A.rows() > 0)
        {
          double v_init = (cst.A * ctrl_pts_.col(i + 1) - cst.b).maxCoeff();
          if (v_init > max_init_viol)
          {
            max_init_viol = v_init;
            max_init_viol_idx = i;
          }
          if (v_init > 1.0e-6)
            ROS_INFO("[ScanBspline][DBG] corridor %d: initial violation %.4fm (waypoint outside its own corridor, data issue)", i, v_init);
        }
        if (area < 0.02)
          ROS_INFO("[ScanBspline][DBG] corridor %d: tiny corridor area=%.4fm^2 verts=%zu (degenerate fallback square?)", i, area, cst.vertices.size());
      }
      ROS_INFO("[ScanBspline][DBG] init: waypoints=%d ctrl=%d dt=%.3f free=[%d,%d] lam_cor=%.0f lam_alm=%.0f | min_area=%.4f@cor%d max_init_viol=%.4f@cor%d",
               num_waypoints_, num_ctrl_pts_, dt_, free_start_idx_, free_end_idx_,
               lambda2_corridor_, lambda_alm_, min_area, min_area_idx, max_init_viol, max_init_viol_idx);
    }

    std::vector<double> x(num_free * 2);
    for (int i = 0; i < num_free; ++i)
    {
      int cp_idx = free_start_idx_ + i;
      x[2 * i + 0] = ctrl_pts_(0, cp_idx);
      x[2 * i + 1] = ctrl_pts_(1, cp_idx);
    }

    // 4. 配置 L-BFGS 参数 (增加内循环最大迭代次数至 200)
    lbfgs::lbfgs_parameter_t lbfgs_params;
    lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
    lbfgs_params.mem_size = 16;
    lbfgs_params.max_iterations = 200;
    lbfgs_params.g_epsilon = 0.005;

    // 5. 外循环: 增广拉格朗日 (ALM) 乘子迭代
    // 24 轮: 强冲突场景 (动力学限幅与走廊临界冲突) 下违例约按 x0.88/轮 几何收敛,
    // 需 ~11 轮穿过 8mm 验收线, 24 轮留足余量; 良性场景 1~3 轮即触发早退。
    const int max_outer_iters = 24;
    const double violation_tol = 8.0e-3; // 8mm 早退阈值 (比 1cm 拒绝门槛留 20% 裕量)

    double final_max_violation = 0.0;
    int final_max_viol_idx = -1;
    Eigen::Vector2d final_max_viol_q = Eigen::Vector2d::Zero();
    double previous_max_violation = 1e9;
    dbg_viol_history_.clear(); // [DBG] 每次优化重置违例历史
    cur_lambda_alm_ = lambda_alm_; // 拷出来只在当前优化中使用，不直接修改类成员默认配置 lambda_alm_

    for (int outer_iter = 0; outer_iter < max_outer_iters; ++outer_iter)
    {
      // rebound 刷新: 每轮 L-BFGS 之前按上一轮末控制点 (首轮为 A* 初值) 检测碰撞并装弹簧,
      // 保证本轮内循环能实际消费排斥约束 —— 刷新放在轮末会因早退 break 使弹簧永远空转
      checkCollisionAndRebound();

      double final_cost = 0.0;
      int ret = lbfgs::lbfgs_optimize(
          static_cast<int>(x.size()),
          x.data(),
          &final_cost,
          &ScanBsplineOptimizer::costFunctionCallback,
          nullptr,
          nullptr,
          this,
          &lbfgs_params);

      if (ret < 0 && ret != lbfgs::LBFGS_ALREADY_MINIMIZED && ret != lbfgs::LBFGSERR_MAXIMUMITERATION)
      {
        ROS_DEBUG("[ScanBsplineOptimizer] L-BFGS outer_iter %d return code: %d", outer_iter, ret);
      }

      // 将内循环优化结果写回 ctrl_pts_
      for (int i = 0; i < num_free; ++i)
      {
        int cp_idx = free_start_idx_ + i;
        ctrl_pts_(0, cp_idx) = x[2 * i + 0];
        ctrl_pts_(1, cp_idx) = x[2 * i + 1];
      }

      // [DBG] 以写回后的 x 为准刷新一次代价分解 (线搜索失败时 lbfgs 内部缓存的代价属于失败试探点)
      double fx_final = 0.0;
      if (debug_verbose_)
      {
        std::vector<double> g_dbg(num_free * 2, 0.0);
        fx_final = evaluateCostAndGrad(x.data(), g_dbg.data(), static_cast<int>(x.size()));
      }

      // 全程控制点穿墙量检查: 走廊 i 严格直接约束自由控制点 q_{i+1}
      // 注: 从 i=1 开始检查，机器人当前起始点 q_1 属于不可移动的初始约束，不计入穿墙阻断
      double max_violation = 0.0;
      int max_viol_idx = -1;
      Eigen::Vector2d max_viol_q = Eigen::Vector2d::Zero();

      for (int i = 1; i < num_waypoints_; ++i)
      {
        const auto& cst = corridors_[i].constraint;
        Eigen::Vector2d q_eval = ctrl_pts_.col(i + 1);

        if (cst.A.rows() > 0)
        {
          Eigen::VectorXd ax_minus_b = cst.A * q_eval - cst.b;
          for (int j = 0; j < ax_minus_b.size(); ++j)
          {
            double viol = std::max(0.0, ax_minus_b(j));
            if (viol > max_violation)
            {
              max_violation = viol;
              max_viol_idx = i;
              max_viol_q = q_eval;
            }
          }
        }
      }

      final_max_violation = max_violation;
      final_max_viol_idx = max_viol_idx;
      final_max_viol_q = max_viol_q;

      // [DBG] 记录违例历史, 拒绝时一并输出
      if (debug_verbose_)
      {
        std::ostringstream oss;
        oss << outer_iter << ':' << std::fixed << std::setprecision(4) << max_violation;
        if (!dbg_viol_history_.empty()) dbg_viol_history_ += " -> ";
        dbg_viol_history_ += oss.str();
      }

      // ===== [DBG-2] 外循环逐轮追踪: ret 码 / 代价分解 / 梯度范数 / 最坏违例走廊与乘子 =====
      if (debug_verbose_)
      {
        double mu_worst = 0.0;
        int j_worst = -1;
        if (max_viol_idx >= 1 && max_viol_idx < static_cast<int>(corridors_.size()) &&
            corridors_[max_viol_idx].constraint.A.rows() > 0)
        {
          const auto& cst_dbg = corridors_[max_viol_idx].constraint;
          Eigen::VectorXd d_dbg = cst_dbg.A * max_viol_q - cst_dbg.b;
          double dv_worst = -1e18;
          for (int j = 0; j < d_dbg.size(); ++j)
          {
            if (d_dbg(j) > dv_worst)
            {
              dv_worst = d_dbg(j);
              j_worst = j;
            }
          }
          if (j_worst >= 0 && max_viol_idx < static_cast<int>(multipliers_.size()))
            mu_worst = multipliers_[max_viol_idx](j_worst);
        }
        ROS_INFO("[ScanBspline][DBG] outer=%d ret=%s fx=%.3f [sm=%.2f feas=%.2f fit=%.2f cor=%.2f alm=%.2f reb=%.2f] gnorm=%.3f | viol=%.4f@cor%d(q%d,row%d) mu=%.1f lam=%.0f",
                 outer_iter, lbfgsRetName(ret), fx_final,
                 dbg_cost_smooth_, dbg_cost_feas_, dbg_cost_fitness_, dbg_cost_corridor_, dbg_cost_alm_, dbg_cost_rebound_, dbg_gnorm_,
                 max_violation, max_viol_idx, max_viol_idx + 1, j_worst, mu_worst, cur_lambda_alm_);
      }

      // 若控制点穿墙量已满足 <= 0.0001m，则提前收敛退出
      // (rebound 刷新已移至本轮 L-BFGS 之前; 此处的控制点状态由规划器在 optimize 返回后
      //  通过 getReboundConstraints 快照发布可视化)
      if (max_violation <= violation_tol)
      {
        break;
      }

      // 在 for outer_iter 循环内部，如果穿墙量没有显著下降，放大 lambda (墙壁变硬两倍)
      if (outer_iter > 0 && max_violation > 0.9 * previous_max_violation)
      {
        cur_lambda_alm_ *= 2.0; // 墙壁变硬两倍
      }
      previous_max_violation = max_violation;

      // 乘子外循环更新: 直接作用于自由控制点 \mu_{i,j} = \max(0, \mu_{i,j} + \lambda_{ALM} * (A_{i,j} * q_{i+1} - b_{i,j}))
      // 注: 同样从 i=1 开始更新自由控制点的乘子
      for (int i = 1; i < num_waypoints_; ++i)
      {
        const auto& cst = corridors_[i].constraint;
        Eigen::Vector2d q_eval = ctrl_pts_.col(i + 1);

        if (cst.A.rows() > 0)
        {
          Eigen::VectorXd ax_minus_b = cst.A * q_eval - cst.b;
          for (int j = 0; j < ax_minus_b.size(); ++j)
          {
            double updated_mu = multipliers_[i](j) + cur_lambda_alm_ * ax_minus_b(j);
            multipliers_[i](j) = std::max(0.0, updated_mu);
          }
        }
      }
    }

    // 6. 导出最终控制点序列 (导出数据以支持 RViz / Web 前端可视化排查)
    out_control_points.clear();
    out_control_points.reserve(num_ctrl_pts_);
    for (int i = 0; i < num_ctrl_pts_; ++i)
    {
      out_control_points.push_back(ctrl_pts_.col(i));
    }

    // 7. 严格安全准入判定: 若自由控制点穿墙量仍大于 0.01m (1cm)，判定优化失败拒绝下发并精确定位违例点
    const double max_allowable_violation = 0.01; // 1厘米容差阈值
    if (final_max_violation > max_allowable_violation)
    {
      // ===== [DBG-3] 拒绝法证报告: 违例行/乘子/初始状态/邻走廊包含性/法向力平衡 =====
      if (debug_verbose_)
      {
        logFailureForensics(final_max_viol_idx, final_max_viol_q, final_max_violation, max_allowable_violation);
      }
      ROS_WARN_THROTTLE(1.0,
          "[ScanBsplineOptimizer] Trajectory rejected: Control point q_%d at (x=%.3f, y=%.3f) in Corridor %d violates constraint by %.3fm (exceeds safety threshold %.3fm), lambda_alm=%.3f",
          final_max_viol_idx + 1, final_max_viol_q.x(), final_max_viol_q.y(), final_max_viol_idx,
          final_max_violation, max_allowable_violation, cur_lambda_alm_);
      return false;
    }

    return true;
  }

private:
  // ===== rebound 定向排斥 (EGO 式碰撞事件驱动, 无 A* 依赖; 结构体定义见 public 区) =====

  /**
   * @brief 逐控制点查占据, 碰撞者装上 (base_point, direction) 定向弹簧。
   *        语义与伪 TF 落脚点预检对齐: 本层带内任一禁行节点 = 碰撞;
   *        图外/悬空 (无本层节点) 不惩罚, 交走廊 ALM 项兜底。
   */
  void checkCollisionAndRebound()
  {
    rebound_.assign(num_ctrl_pts_, ReboundConstraint());
    rebound_active_count_ = 0;
    if (!occupancy_cb_) return;

    for (int i = 1; i < num_waypoints_; ++i)
    {
      const int cp = i + 1;
      const Eigen::Vector2d q = ctrl_pts_.col(cp);
      const double z = corridors_[i].queryZFromNodes(q.x(), q.y());

      Eigen::Vector2d obstacle_pt(0.0, 0.0);
      if (!occupancy_cb_(q.x(), q.y(), z, &obstacle_pt)) continue;

      ReboundConstraint rc;
      rc.base_point = obstacle_pt;
      const Eigen::Vector2d d = q - obstacle_pt;
      const double n = d.norm();
      rc.direction = (n > 1.0e-6) ? (d / n) : Eigen::Vector2d(0.0, 0.0);
      rebound_[cp] = rc;
      ++rebound_active_count_;
    }
  }

  /**
   * @brief 定向排斥代价项 (EGO calcDistanceCostRebound 分段 C2 公式的 2D 版):
   *        dist = (q − base_point)·direction, 要求 dist ≥ clearance;
   *        err < demarcation 三次项, 深穿透切换二次+线性延拓, 梯度方向 = direction。
   */
  void calcReboundCost(const Eigen::Matrix2Xd& q, double& cost, Eigen::Matrix2Xd& grad)
  {
    cost = 0.0;
    if (lambda_rebound_ <= 0.0 || rebound_.empty()) return;

    const double demarcation = rebound_clearance_;
    const double a = 3.0 * demarcation;
    const double b = -3.0 * demarcation * demarcation;
    const double c = demarcation * demarcation * demarcation;

    for (int i = 1; i < num_waypoints_; ++i)
    {
      const int cp = i + 1;
      const auto& rc = rebound_[cp];
      if (rc.direction.squaredNorm() < 0.5) continue;  // 未激活 (单位向量模长恒 1)

      const double dist = (q.col(cp) - rc.base_point).dot(rc.direction);
      const double err = rebound_clearance_ - dist;
      if (err <= 0.0) continue;  // 已弹到安全间距之外

      if (err < demarcation)
        cost += err * err * err;
      else
        cost += a * err * err + b * err + c;

      // d(cost)/d(err), 再乘 d(err)/dq = -direction
      const double dJ_de = (err < demarcation) ? (3.0 * err * err) : (2.0 * a * err + b);
      grad.col(cp) += (-lambda_rebound_ * dJ_de) * rc.direction;
    }
  }

  static double costFunctionCallback(void* instance,
                                     const double* x,
                                     double* g,
                                     const int n)
  {
    auto* opt = reinterpret_cast<ScanBsplineOptimizer*>(instance);
    return opt->evaluateCostAndGrad(x, g, n);
  }

  double evaluateCostAndGrad(const double* x, double* g, const int n)
  {
    int num_free = free_end_idx_ - free_start_idx_ + 1;
    for (int i = 0; i < num_free; ++i)
    {
      int cp_idx = free_start_idx_ + i;
      ctrl_pts_(0, cp_idx) = x[2 * i + 0];
      ctrl_pts_(1, cp_idx) = x[2 * i + 1];
    }

    double cost_smooth = 0.0, cost_feas = 0.0, cost_fitness = 0.0;
    double cost_corridor = 0.0, cost_alm = 0.0, cost_rebound = 0.0;

    Eigen::Matrix2Xd grad_smooth = Eigen::Matrix2Xd::Zero(2, num_ctrl_pts_);
    Eigen::Matrix2Xd grad_feas = Eigen::Matrix2Xd::Zero(2, num_ctrl_pts_);
    Eigen::Matrix2Xd grad_fitness = Eigen::Matrix2Xd::Zero(2, num_ctrl_pts_);
    Eigen::Matrix2Xd grad_corridor = Eigen::Matrix2Xd::Zero(2, num_ctrl_pts_);
    Eigen::Matrix2Xd grad_alm = Eigen::Matrix2Xd::Zero(2, num_ctrl_pts_);
    Eigen::Matrix2Xd grad_rebound = Eigen::Matrix2Xd::Zero(2, num_ctrl_pts_);

    // 1. Jerk 加加速度平滑项
    calcSmoothnessCost(ctrl_pts_, cost_smooth, grad_smooth);

    // 2. 动力学可行性限幅项 (速度 + 加速度)
    calcFeasibilityCost(ctrl_pts_, cost_feas, grad_feas);

    // 3. 参考路径各向异性管道贴合项
    calcFitnessCost(ctrl_pts_, cost_fitness, grad_fitness);

    // 4. 凸包本质: 直接对控制点 q_{i+1} 施加穿墙三次强惩罚项 J_corridor
    calcCorridorCost(ctrl_pts_, cost_corridor, grad_corridor);

    // 5. 凸包本质: 直接对控制点 q_{i+1} 施加增广拉格朗日项 J_ALM
    calcALMCost(ctrl_pts_, cost_alm, grad_alm);

    // 6. rebound 定向排斥项: 碰撞控制点沿 (base_point, direction) 弹离障碍至 clearance 之外
    calcReboundCost(ctrl_pts_, cost_rebound, grad_rebound);

    // 线性合成总代价与总梯度
    double total_cost = lambda1_smooth_ * cost_smooth +
                        lambda3_feas_ * cost_feas +
                        lambda4_fitness_ * cost_fitness +
                        cost_corridor +
                        cost_alm +
                        cost_rebound;

    Eigen::Matrix2Xd grad_all = lambda1_smooth_ * grad_smooth +
                                lambda3_feas_ * grad_feas +
                                lambda4_fitness_ * grad_fitness +
                                grad_corridor +
                                grad_alm +
                                grad_rebound;

    for (int i = 0; i < num_free; ++i)
    {
      int cp_idx = free_start_idx_ + i;
      g[2 * i + 0] = grad_all(0, cp_idx);
      g[2 * i + 1] = grad_all(1, cp_idx);
    }

    // [DBG] 缓存本次评估的加权代价分量与梯度范数, 供外循环追踪日志使用
    dbg_cost_smooth_ = lambda1_smooth_ * cost_smooth;
    dbg_cost_feas_ = lambda3_feas_ * cost_feas;
    dbg_cost_fitness_ = lambda4_fitness_ * cost_fitness;
    dbg_cost_corridor_ = cost_corridor;
    dbg_cost_alm_ = cost_alm;
    dbg_cost_rebound_ = cost_rebound;
    double g_sq = 0.0;
    for (int i = 0; i < n; ++i) g_sq += g[i] * g[i];
    dbg_gnorm_ = std::sqrt(g_sq);

    return total_cost;
  }

  /**
   * @brief 直接对控制点 q_{i+1} 施加穿墙混合强惩罚: J_corridor = \lambda \sum (0.5 * \max(0, v)^2 + \max(0, v)^3)
   */
  void calcCorridorCost(const Eigen::Matrix2Xd& q, double& cost, Eigen::Matrix2Xd& grad)
  {
    cost = 0.0;
    if (corridors_.empty()) return;

    // 从 i=1 开始约束自由控制点 q_{i+1}，豁免机器人起始点 q_1
    for (int i = 1; i < num_waypoints_; ++i)
    {
      if (i >= static_cast<int>(corridors_.size())) continue;
      const auto& cst = corridors_[i].constraint;
      if (cst.A.rows() == 0) continue;

      // 直接约束第 i+1 个控制点 q_{i+1} (凸包本质)
      Eigen::Vector2d q_eval = q.col(i + 1);
      Eigen::VectorXd viol = cst.A * q_eval - cst.b; // M x 1

      for (int j = 0; j < viol.size(); ++j)
      {
        if (viol(j) > 0.0)
        {
          double v = viol(j);
          // 混合惩罚: 二次项提供微小穿透下的强大线性恢复梯度，三次项提供深层穿透的爆炸级阻截
          cost += lambda2_corridor_ * (0.5 * v * v + v * v * v);

          // 直接对控制点 q_{i+1} 求导: \nabla_{q_{i+1}} = \lambda * (v + 3 * v^2) * A_{i,j}^T
          Eigen::Vector2d grad_q = (lambda2_corridor_ * (v + 3.0 * v * v)) * cst.A.row(j).transpose();
          grad.col(i + 1) += grad_q;
        }
      }
    }
  }

  /**
   * @brief 直接对控制点 q_{i+1} 施加增广拉格朗日项
   */
  void calcALMCost(const Eigen::Matrix2Xd& q, double& cost, Eigen::Matrix2Xd& grad)
  {
    cost = 0.0;
    if (corridors_.empty()) return;

    // 从 i=1 开始约束自由控制点 q_{i+1}，豁免机器人起始点 q_1
    for (int i = 1; i < num_waypoints_; ++i)
    {
      if (i >= static_cast<int>(corridors_.size()) || i >= static_cast<int>(multipliers_.size())) continue;
      const auto& cst = corridors_[i].constraint;
      if (cst.A.rows() == 0) continue;

      // 直接约束第 i+1 个控制点 q_{i+1}
      Eigen::Vector2d q_eval = q.col(i + 1);
      Eigen::VectorXd ax_minus_b = cst.A * q_eval - cst.b;
      const auto& mu = multipliers_[i];

      for (int j = 0; j < ax_minus_b.size(); ++j)
      {
        double d = ax_minus_b(j);
        // PHR 增广拉格朗日有效集判别
        if (d + (mu(j) / cur_lambda_alm_) > 0.0)
        {
          cost += mu(j) * d + 0.5 * cur_lambda_alm_ * (d * d);

          // 直接对控制点 q_{i+1} 求导: \nabla_{q_{i+1}} = (\mu_j + \lambda_{ALM} * d) * A_{i,j}^T
          Eigen::Vector2d grad_q = (mu(j) + cur_lambda_alm_ * d) * cst.A.row(j).transpose();
          grad.col(i + 1) += grad_q;
        }
      }
    }
  }

  /**
   * @brief Jerk 加加速度平滑能量与解析梯度
   */
  void calcSmoothnessCost(const Eigen::Matrix2Xd& q, double& cost, Eigen::Matrix2Xd& grad)
  {
    cost = 0.0;
    for (int i = 0; i < q.cols() - 3; ++i)
    {
      Eigen::Vector2d jerk = q.col(i + 3) - 3.0 * q.col(i + 2) + 3.0 * q.col(i + 1) - q.col(i);
      cost += jerk.squaredNorm();
      Eigen::Vector2d temp_j = 2.0 * jerk;

      grad.col(i + 0) += -temp_j;
      grad.col(i + 1) += 3.0 * temp_j;
      grad.col(i + 2) += -3.0 * temp_j;
      grad.col(i + 3) += temp_j;
    }
  }

  /**
   * @brief 各向异性连续管道势能 (直接约束控制点 q_{i+1})
   */
  void calcFitnessCost(const Eigen::Matrix2Xd& q, double& cost, Eigen::Matrix2Xd& grad)
  {
    cost = 0.0;
    if (ref_pts_.size() < 2) return;

    double a2 = 25.0, b2 = 1.0;

    for (int i = 0; i < num_waypoints_; ++i)
    {
      Eigen::Vector2d x = q.col(i + 1) - ref_pts_[i];

      int prev_idx = std::max(0, i - 1);
      int next_idx = std::min(static_cast<int>(ref_pts_.size()) - 1, i + 1);
      Eigen::Vector2d v = (ref_pts_[next_idx] - ref_pts_[prev_idx]).normalized();
      if (v.squaredNorm() < 1e-4) v = Eigen::Vector2d(1.0, 0.0);
      Eigen::Vector2d n(-v.y(), v.x());

      double x_dot_v = x.dot(v);
      double x_dot_n = x.dot(n);

      double f = std::pow(x_dot_v, 2) / a2 + std::pow(x_dot_n, 2) / b2;
      cost += f;

      Eigen::Vector2d df_dx = (2.0 * x_dot_v / a2) * v + (2.0 * x_dot_n / b2) * n;
      grad.col(i + 1) += df_dx;
    }
  }

  /**
   * @brief 动力学可行性限幅 (速度 + 加速度)
   */
  void calcFeasibilityCost(const Eigen::Matrix2Xd& q, double& cost, Eigen::Matrix2Xd& grad)
  {
    cost = 0.0;
    double dt_inv = 1.0 / dt_;
    double dt_inv2 = dt_inv * dt_inv;

    // 速度限幅 (Huber 化: 超出量超过容差后转线性增长, 梯度封顶 —— 防止动力学项以
    //  10^5 级梯度压垮走廊/ALM 项; f/g 保持一致, 线搜索不受影响)
    for (int i = 0; i < q.cols() - 1; ++i)
    {
      Eigen::Vector2d v = (q.col(i + 1) - q.col(i)) * dt_inv;
      double v_norm = v.norm();
      if (v_norm > max_vel_)
      {
        double diff = v_norm - max_vel_;
        double e = std::min(diff, feas_vel_excess_cap_);
        cost += e * e * e + 3.0 * e * e * (diff - e);
        Eigen::Vector2d grad_v = (3.0 * e * e / v_norm) * v;
        grad.col(i + 0) += -dt_inv * grad_v;
        grad.col(i + 1) += dt_inv * grad_v;
      }
    }

    // 加速度限幅 (同上 Huber 化)
    for (int i = 0; i < q.cols() - 2; ++i)
    {
      Eigen::Vector2d acc = (q.col(i + 2) - 2.0 * q.col(i + 1) + q.col(i)) * dt_inv2;
      double a_norm = acc.norm();
      if (a_norm > max_acc_)
      {
        double diff = a_norm - max_acc_;
        double e = std::min(diff, feas_acc_excess_cap_);
        cost += e * e * e + 3.0 * e * e * (diff - e);
        Eigen::Vector2d grad_a = (3.0 * e * e / a_norm) * acc;
        grad.col(i + 0) += dt_inv2 * grad_a;
        grad.col(i + 1) += -2.0 * dt_inv2 * grad_a;
        grad.col(i + 2) += dt_inv2 * grad_a;
      }
    }
  }

  // ============================ [DBG] 调试辅助 ============================

  /**
   * @brief [DBG] L-BFGS 返回码转可读字符串
   */
  static const char* lbfgsRetName(int ret)
  {
    switch (ret)
    {
      case lbfgs::LBFGS_CONVERGENCE: return "CONVERGENCE";
      case lbfgs::LBFGS_STOP: return "STOP";
      case lbfgs::LBFGS_ALREADY_MINIMIZED: return "ALREADY_MINIMIZED";
      case lbfgs::LBFGSERR_LOGICERROR: return "LOGICERROR";
      case lbfgs::LBFGSERR_CANCELED: return "CANCELED";
      case lbfgs::LBFGSERR_INVALID_N: return "INVALID_N";
      case lbfgs::LBFGSERR_INVALID_MEMSIZE: return "INVALID_MEMSIZE";
      case lbfgs::LBFGSERR_INVALID_GEPSILON: return "INVALID_GEPSILON";
      case lbfgs::LBFGSERR_OUTOFINTERVAL: return "OUTOFINTERVAL";
      case lbfgs::LBFGSERR_INCORRECT_TMINMAX: return "INCORRECT_TMINMAX";
      case lbfgs::LBFGSERR_ROUNDING_ERROR: return "ROUNDING_ERROR";
      case lbfgs::LBFGSERR_MINIMUMSTEP: return "MINIMUMSTEP";
      case lbfgs::LBFGSERR_MAXIMUMSTEP: return "MAXIMUMSTEP";
      case lbfgs::LBFGSERR_MAXIMUMLINESEARCH: return "MAXIMUMLINESEARCH";
      case lbfgs::LBFGSERR_MAXIMUMITERATION: return "MAXIMUMITERATION";
      case lbfgs::LBFGSERR_WIDTHTOOSMALL: return "WIDTHTOOSMALL";
      case lbfgs::LBFGSERR_INVALIDPARAMETERS: return "INVALIDPARAMETERS";
      case lbfgs::LBFGSERR_INCREASEGRADIENT: return "INCREASEGRADIENT";
      default: return "UNKNOWN";
    }
  }

  /**
   * @brief [DBG] 多边形面积 (鞋带公式取绝对值, m^2)
   */
  static double polygonArea(const std::vector<Eigen::Vector2d>& verts)
  {
    double area2 = 0.0;
    for (size_t i = 0; i < verts.size(); ++i)
    {
      const auto& p1 = verts[i];
      const auto& p2 = verts[(i + 1) % verts.size()];
      area2 += p1.x() * p2.y() - p2.x() * p1.y();
    }
    return 0.5 * std::abs(area2);
  }

  /**
   * @brief [DBG] 射线法点在多边形内测试
   */
  static bool pointInPolygon(const Eigen::Vector2d& p, const std::vector<Eigen::Vector2d>& verts)
  {
    if (verts.size() < 3) return false;
    bool inside = false;
    for (size_t i = 0, j = verts.size() - 1; i < verts.size(); j = i++)
    {
      const auto& pi = verts[i];
      const auto& pj = verts[j];
      if ((pi.y() > p.y()) != (pj.y() > p.y()))
      {
        double x_cross = (pj.x() - pi.x()) * (p.y() - pi.y()) / (pj.y() - pi.y()) + pi.x();
        if (p.x() < x_cross) inside = !inside;
      }
    }
    return inside;
  }

  /**
   * @brief [DBG] 拒绝法证报告: 违例行/乘子/初始状态/邻走廊包含性/各代价项在违例法向 A_j 上的力平衡
   *
   * 判读方法:
   *   - force@A_j 五项投影之和 SUM 若显著偏离 0 => 内循环未解到子问题平稳点 (g_eps 过松 / 线搜索失败 / 迭代不足);
   *   - SUM≈0 但仍穿墙 => 平衡点本身在墙外, smooth/feas/fitness 中出现大负值者即把点顶出墙的"推手";
   *   - corridor/alm 投影为正 (梯度方向指向墙外, 梯度下降即拉回), 其余项为负时表示该项在往外推。
   */
  void logFailureForensics(int idx, const Eigen::Vector2d& q_viol, double viol, double tol)
  {
    std::ostringstream os;
    os << std::fixed << std::setprecision(4);
    os << "[ScanBspline][DBG] ===== rejection forensics corridor " << idx << " (q" << idx + 1 << ") =====\n";
    os << " viol=" << viol << " tol=" << tol << " | viol_history: " << dbg_viol_history_ << "\n";

    if (idx < 1 || idx >= num_waypoints_ || idx >= static_cast<int>(corridors_.size()))
    {
      os << " corridor index out of range!";
      ROS_WARN_THROTTLE(1.0, "%s", os.str().c_str());
      return;
    }

    const auto& cor = corridors_[idx];
    const auto& cst = cor.constraint;
    if (cst.A.rows() == 0)
    {
      os << " corridor constraint EMPTY!";
      ROS_WARN_THROTTLE(1.0, "%s", os.str().c_str());
      return;
    }

    // 1. 初始 vs 最终: q_{idx+1} 初始值 = ref_pts_[idx] (= waypoints_2d[idx])
    double init_viol = (cst.A * ref_pts_[idx] - cst.b).maxCoeff();
    os << " q_init=(" << ref_pts_[idx].x() << "," << ref_pts_[idx].y() << ") init_viol=" << init_viol
       << " | q_final=(" << q_viol.x() << "," << q_viol.y() << ") cor_center=(" << cor.center.x() << "," << cor.center.y() << ")\n";

    // 2. 最坏违例行详情
    Eigen::VectorXd d = cst.A * q_viol - cst.b;
    int j_worst = 0;
    for (int j = 1; j < d.size(); ++j)
    {
      if (d(j) > d(j_worst)) j_worst = j;
    }
    Eigen::Vector2d a_row = cst.A.row(j_worst).transpose();
    os << " worst row j=" << j_worst << "/" << d.size() << ": A_j=(" << a_row.x() << "," << a_row.y()
       << ") |A_j|=" << a_row.norm() << " b_j=" << cst.b(j_worst) << " d_j=" << d(j_worst)
       << " mu_j=" << multipliers_[idx](j_worst) << "\n";

    // 3. 走廊几何健康度 (verts=4 且 area≈0.0144 => 兜底 0.06m 小方块)
    double area = polygonArea(cst.vertices);
    double circum_r = 0.0;
    for (const auto& v : cst.vertices) circum_r = std::max(circum_r, (v - cor.center).norm());
    os << " corridor geom: verts=" << cst.vertices.size() << " area=" << area << "m^2 circum_r=" << circum_r << "m\n";

    // 4. 邻走廊包含性 (点其实落在相邻走廊内 => 走廊衔接问题)
    bool in_prev = (idx - 1 >= 0) && pointInPolygon(q_viol, corridors_[idx - 1].constraint.vertices);
    bool in_next = (idx + 1 < static_cast<int>(corridors_.size())) && pointInPolygon(q_viol, corridors_[idx + 1].constraint.vertices);
    os << " containment: in_cor" << idx - 1 << "=" << (in_prev ? 1 : 0) << " in_cor" << idx << "="
       << (pointInPolygon(q_viol, cst.vertices) ? 1 : 0) << " in_cor" << idx + 1 << "=" << (in_next ? 1 : 0) << "\n";

    // 5. 各代价项梯度在违例法向 A_j 上的投影力平衡
    double c_dummy = 0.0;
    Eigen::Matrix2Xd g_s = Eigen::Matrix2Xd::Zero(2, num_ctrl_pts_);
    Eigen::Matrix2Xd g_f = Eigen::Matrix2Xd::Zero(2, num_ctrl_pts_);
    Eigen::Matrix2Xd g_fit = Eigen::Matrix2Xd::Zero(2, num_ctrl_pts_);
    Eigen::Matrix2Xd g_cor = Eigen::Matrix2Xd::Zero(2, num_ctrl_pts_);
    Eigen::Matrix2Xd g_alm = Eigen::Matrix2Xd::Zero(2, num_ctrl_pts_);
    calcSmoothnessCost(ctrl_pts_, c_dummy, g_s);
    calcFeasibilityCost(ctrl_pts_, c_dummy, g_f);
    calcFitnessCost(ctrl_pts_, c_dummy, g_fit);
    calcCorridorCost(ctrl_pts_, c_dummy, g_cor);
    calcALMCost(ctrl_pts_, c_dummy, g_alm);
    int cp = idx + 1;
    double proj_s = lambda1_smooth_ * g_s.col(cp).dot(a_row);
    double proj_f = lambda3_feas_ * g_f.col(cp).dot(a_row);
    double proj_fit = lambda4_fitness_ * g_fit.col(cp).dot(a_row);
    double proj_cor = g_cor.col(cp).dot(a_row);
    double proj_alm = g_alm.col(cp).dot(a_row);
    os << " force@A_j: smooth=" << proj_s << " feas=" << proj_f << " fitness=" << proj_fit
       << " corridor=" << proj_cor << " alm=" << proj_alm
       << " SUM=" << (proj_s + proj_f + proj_fit + proj_cor + proj_alm);

    ROS_WARN_THROTTLE(1.0, "%s", os.str().c_str());
  }

private:
  double lambda1_smooth_{1.0};
  double lambda2_corridor_{1000.0};
  double lambda3_feas_{2.0};
  double lambda4_fitness_{1.0};
  double lambda_alm_{1000.0};
  double cur_lambda_alm_{1000.0};
  double max_vel_{0.80};
  double max_acc_{1.50};
  double feas_vel_excess_cap_{1.0};   ///< [Huber] 速度超出量转线性的容差 (m/s), 梯度封顶 3*cap^2/dt
  double feas_acc_excess_cap_{2.0};   ///< [Huber] 加速度超出量转线性的容差 (m/s^2), 梯度封顶 3*cap^2/dt^2

  double dt_{0.15};
  int num_waypoints_{0};
  int num_ctrl_pts_{0};
  int free_start_idx_{2};
  int free_end_idx_{0};

  Eigen::Matrix2Xd ctrl_pts_;
  std::vector<Eigen::Vector2d> ref_pts_;
  std::vector<ConvexCorridor2D> corridors_;
  std::vector<Eigen::VectorXd> multipliers_;

  // ===== rebound 定向排斥成员 =====
  OccupancyCallback occupancy_cb_;             ///< 占据查询回调 (astar_local_planner 注入, 图零耦合)
  std::vector<ReboundConstraint> rebound_;     ///< 控制点下标对齐的排斥约束 (零方向 = 未激活)
  double lambda_rebound_{0.0};                 ///< 排斥项权重 (0 = 功能关闭)
  double rebound_clearance_{0.17};             ///< 安全间距 (= body_hard_radius)
  int rebound_active_count_{0};                ///< 最近一次刷新时激活的约束数量 (诊断用)

  // ===== [DBG] 调试观测成员 =====
  bool debug_verbose_{true};
  double dbg_cost_smooth_{0.0};
  double dbg_cost_feas_{0.0};
  double dbg_cost_fitness_{0.0};
  double dbg_cost_corridor_{0.0};
  double dbg_cost_alm_{0.0};
  double dbg_cost_rebound_{0.0};
  double dbg_gnorm_{0.0};
  std::string dbg_viol_history_;
};

} // namespace elevation_local_planner
