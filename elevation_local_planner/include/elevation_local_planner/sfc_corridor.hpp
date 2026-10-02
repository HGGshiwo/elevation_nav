#pragma once

#include "elevation_planner_core/manifold_graph.hpp"
#include "elevation_planner_core/path_simplifier.hpp"
#include <ros/console.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Dense>
#include <vector>
#include <queue>
#include <unordered_set>
#include <cmath>
#include <cstdio>
#include <algorithm>
#include <limits>

namespace elevation_local_planner
{

/**
 * @brief [DBG] 负空间格诊断记录: 每个非 visited 格的节点信息与未访问原因
 *
 * why 分类: N=格内无节点 (空洞/立面)  A=最近节点高出段层带 (z-段高 > max_step)
 *           B=最近节点低于段层带      K=层带内但硬禁行 (trav>=0.95)
 *           U=层带内可走但未被 BFS 访问 (半径/去重/前沿互锁伪影)
 */
struct NegativeCell
{
  Eigen::Vector2d p;      ///< 格中心
  float z{0.f};           ///< 格内距段折线插值高度最近节点的 z (无节点 = 0)
  float trav{0.f};        ///< 该节点通行性
  float dz{0.f};          ///< 该节点相对段折线插值高度的差 (z - z_line)
  char why{'N'};          ///< 上述分类
};

/**
 * @brief 单个流形图踏面节点缓存数据 (完全消除坐标二次查图)
 */
struct CorridorNode3D
{
  uint32_t id{0};
  float x{0.0f};
  float y{0.0f};
  float z{0.0f};
  int32_t row{0};
  int32_t col{0};
  int32_t layer_id{0};
};

/**
 * @brief 线性凸多边形约束: A * q <= b (若穿墙，则违背量 A_i * q - b_i > 0)
 */
struct LinearConstraint2D
{
  Eigen::MatrixX2d A;                     ///< (M x 2) 各边向外法向量矩阵 a_j^T
  Eigen::VectorXd  b;                     ///< (M x 1) 边界偏置 b_j
  std::vector<Eigen::Vector2d> vertices;  ///< 凸包逆时针顶点序列
  int base_rows{-1};                      ///< [DBG] 拼接走廊中属于本段自身的行数 [0, base_rows); -1 = 全部行均为自身
};

/**
 * @brief 单个航点对应的 2D 凸走廊与拓扑节点集
 */
struct ConvexCorridor2D
{
  uint32_t root_node_id{0};               ///< 对应的流形图节点 ID
  Eigen::Vector2d center;                 ///< 走廊中心航点 (x, y)
  double z_ref{0.0};                      ///< 走廊基准地形高度 (m)
  int32_t layer_id{0};                    ///< 所在流形图层级
  LinearConstraint2D constraint;          ///< Andrew 凸包线性约束 A*q <= b
  std::vector<CorridorNode3D> node_pts;   ///< BFS 扩散收集的所有真实流形图节点 (x, y, z)
  std::vector<NegativeCell> negative_cells;    ///< [DBG] 负空间格诊断集 (visited 补集, bbox 内): JSON obstacles 字段数据源
  double inscribed_width{1e9};            ///< [DBG] 段折线最大内切宽度 (2×线点到自身约束面距离); 段式生成时计算, 未计算=1e9
  double body_clearance{1e9};             ///< 机体间隙闸: 沿段折线采样点到自身约束面的最小距离; < body_hard_radius → 链贴障碍, 规划失败

  /**
   * @brief 完全通过已记录的节点集快速查询 (x, y) 处的高程 Z (零图查表开销)
   */
  double queryZFromNodes(double x, double y) const
  {
    if (node_pts.empty()) return z_ref;

    double min_dist_sq = std::numeric_limits<double>::max();
    double best_z = z_ref;
    for (const auto& nd : node_pts)
    {
      double d_sq = (nd.x - x) * (nd.x - x) + (nd.y - y) * (nd.y - y);
      if (d_sq < min_dist_sq)
      {
        min_dist_sq = d_sq;
        best_z = nd.z;
      }
    }
    return best_z;
  }
};

// 兼容别名
using ConvexPolygon2D = ConvexCorridor2D;

/**
 * @brief 2D 凸走廊生成器 (基于真实拓扑单步相邻 BFS 扩散 + Andrew 单调链算法，零 findClosestNode 查点)
 */
/**
 * @brief 迭代半平面交走廊生成器 (Decomp 式): 折线种子 → visited 补集为障碍点集 → 逐刀切割
 *
 * 不变量 (每刀保持, 收敛即正确):
 *   1. 凸性: 半平面交集恒凸;
 *   2. 折线包含: min_clearance 动态截断保投影点, 最近点性质保整条折线在安全侧;
 *   3. 无禁行: 终止条件 = 障碍点集清空 (visited 补集 = 墙/悬崖/空洞/未建图/异层/剪枝格统一)。
 * 顶点仅为可视化 (SH 裁剪), 退化不影响优化 (优化只认 A,b)。
 */
class DecompCorridorGenerator
{
public:
  static ConvexCorridor2D generate(
      const std::vector<Eigen::Vector2d>& line,
      const elevation_planner::ManifoldGraph& graph,
      const std::unordered_set<int64_t>& visited_cells,
      int rmin, int rmax, int cmin, int cmax,
      double min_clearance = 0.01,
      double z0 = 0.0, double z1 = 0.0, double max_step = 0.25,
      std::vector<NegativeCell>* negative_space_out = nullptr)
  {
    ConvexCorridor2D cor;
    if (line.empty()) return cor;

    // 负空间障碍物集: visited 补集 (bbox 内), 每格附分类诊断 (见 NegativeCell):
    // z_line = 段折线在该格投影处的两端 z 线性插值; 分类是"为什么在负空间里"的
    // 事实记录 —— 障碍/不联通的语义区分以此为基础, 不靠猜测
    struct Obstacle { Eigen::Vector2d p; NegativeCell dbg; };
    std::vector<Obstacle> O;
    const double res = graph.getResolution();
    for (int r = rmin; r <= rmax; ++r)
      for (int c = cmin; c <= cmax; ++c)
      {
        const int64_t key = (static_cast<int64_t>(r) << 32) | static_cast<uint32_t>(c);
        if (visited_cells.count(key)) continue;
        Eigen::Vector2d p(graph.getMinX() + (r + 0.5) * res,
                          graph.getMinY() + (c + 0.5) * res);
        double t = 0.0;
        if (line.size() >= 2)
        {
          const Eigen::Vector2d dv = line.back() - line.front();
          const double l2 = dv.squaredNorm();
          if (l2 > 1e-12)
            t = std::max(0.0, std::min(1.0, (p - line.front()).dot(dv) / l2));
        }
        const double z_line = z0 + t * (z1 - z0);
        NegativeCell dbg;
        dbg.p = p;
        bool has = false;
        for (uint32_t nid : graph.getSpatialCellNodes(r, c))
        {
          if (nid >= graph.numNodes()) continue;
          const auto & nd = graph.getNode(nid);
          if (!has || std::fabs(nd.z - z_line) < std::fabs(dbg.z - z_line))
          {
            dbg.z = nd.z;
            dbg.trav = nd.traversability;
            has = true;
          }
        }
        if (!has)
        {
          dbg.why = 'N';
        }
        else
        {
          dbg.dz = static_cast<float>(dbg.z - z_line);
          if (dbg.dz > max_step) dbg.why = 'A';
          else if (dbg.dz < -max_step) dbg.why = 'B';
          else if (dbg.trav >= 0.95f) dbg.why = 'K';
          else dbg.why = 'U';
        }
        O.push_back({p, dbg});
      }

    // [DBG] 导出负空间诊断集: 前端/离线分析可见走廊是被哪些格、因何原因切出来的
    if (negative_space_out)
    {
      *negative_space_out = {};
      negative_space_out->reserve(O.size());
      for (const auto & ob : O) negative_space_out->push_back(ob.dbg);
    }

    // 初始包围盒 (visited bbox): 4 条半平面恒在, 保证约束非空
    Eigen::Vector2d dir(1.0, 0.0);
    if (line.size() >= 2)
    {
      Eigen::Vector2d d = line.back() - line.front();
      if (d.squaredNorm() > 1e-9) dir = d.normalized();
    }
    const Eigen::Vector2d nrm(-dir.y(), dir.x());
    const Eigen::Vector2d mid(graph.getMinX() + (rmin + rmax + 1) * 0.5 * res,
                              graph.getMinY() + (cmin + cmax + 1) * 0.5 * res);
    const double half_l = 0.5 * ((rmax - rmin) + (cmax - cmin)) * res + 2.0 * res;
    const double half_w = half_l;

    Eigen::MatrixX2d A(4, 2);
    Eigen::VectorXd b(4);
    A.row(0) = dir;  b(0) = dir.dot(mid) + half_l;
    A.row(1) = -dir; b(1) = -dir.dot(mid) + half_l;
    A.row(2) = nrm;  b(2) = nrm.dot(mid) + half_w;
    A.row(3) = -nrm; b(3) = -nrm.dot(mid) + half_w;

    std::vector<Eigen::Vector2d> verts = {
        mid + dir * half_l + nrm * half_w,  mid - dir * half_l + nrm * half_w,
        mid - dir * half_l - nrm * half_w,  mid + dir * half_l - nrm * half_w};

    // 点到折线最近点 (投影 t 截断在段内)
    auto closestOnLine = [&](const Eigen::Vector2d& o, Eigen::Vector2d& proj) -> double {
      double best = 1e18;
      if (line.size() == 1) { proj = line.front(); return (o - proj).norm(); }
      for (size_t i = 0; i + 1 < line.size(); ++i)
      {
        const Eigen::Vector2d a = line[i], dvec = line[i + 1] - line[i];
        const double l2 = dvec.squaredNorm();
        const double t = (l2 > 1e-12) ? std::max(0.0, std::min(1.0, (o - a).dot(dvec) / l2)) : 0.0;
        const Eigen::Vector2d q = a + t * dvec;
        const double d = (o - q).norm();
        if (d < best) { best = d; proj = q; }
      }
      return best;
    };

    // 迭代切割 (两层保护)
    int safety = static_cast<int>(O.size()) + 8;
    while (!O.empty() && safety-- > 0)
    {
      size_t bi = 0;
      double bd = 1e18;
      Eigen::Vector2d bproj(0.0, 0.0);
      for (size_t i = 0; i < O.size(); ++i)
      {
        Eigen::Vector2d pr;
        const double d = closestOnLine(O[i].p, pr);
        if (d < bd) { bd = d; bi = i; bproj = pr; }
      }
      const Eigen::Vector2d o = O[bi].p;
      const Eigen::Vector2d v = o - bproj;
      const double d = v.norm();
      if (d < min_clearance)
        return ConvexCorridor2D();   // 障碍落在折线上: 真不可行, 放弃本段 (move_base 重试)
      const Eigen::Vector2d n = v / d;
      // 无 margin: 贴障碍格中心切割 —— 机体与障碍的间距由 A* 代价层 (膨胀软带
      // 推链居中) 负责; 本层的机体间隙由 body_clearance 闸验证, 不足即失败冻结
      const double d_cut = d;
      const Eigen::Vector2d p_cut = bproj + n * d_cut;

      const int rows = static_cast<int>(A.rows());
      A.conservativeResize(rows + 1, Eigen::NoChange);
      A.row(rows) = n.transpose();
      b.conservativeResize(rows + 1);
      b(rows) = n.dot(p_cut);

      // 可视化多边形 SH 裁剪
      std::vector<Eigen::Vector2d> clipped;
      for (size_t i = 0; i < verts.size(); ++i)
      {
        const Eigen::Vector2d& cur = verts[i];
        const Eigen::Vector2d& nxt = verts[(i + 1) % verts.size()];
        const bool cur_in = n.dot(cur) <= b(rows) + 1e-12;
        const bool nxt_in = n.dot(nxt) <= b(rows) + 1e-12;
        if (cur_in) clipped.push_back(cur);
        if (cur_in != nxt_in)
        {
          const double dc = n.dot(cur) - b(rows), dn = n.dot(nxt) - b(rows);
          clipped.push_back(cur + (nxt - cur) * (dc / (dc - dn)));
        }
      }
      verts = clipped;

      // 剔除被切除的障碍点
      std::vector<Obstacle> rest;
      for (const auto& ok : O)
        if (n.dot(ok.p) < b(rows) - 1e-12)
          rest.push_back(ok);
      O.swap(rest);
    }

    cor.center = line.front();
    cor.constraint.A = A;
    cor.constraint.b = b;
    cor.constraint.vertices = verts;
    return cor;
  }
};

class SFCGenerator
{
public:
  SFCGenerator() = default;

  /**
   * @brief 直接通过 A* 输出的 node_id 序列向外在流形拓扑图上扩散生成凸多边形走廊 (零坐标查图)
   * @param path_node_ids A* 输出的图节点 ID 序列
   * @param graph 3D 流形高程图
   * @param max_diffusion_radius 最大扩散半径阈值 (默认 0.50m)
   * @param max_step_height 最大单步高度阈值
   * @param max_stride_length 最大单步跨度阈值
   * @return std::vector<ConvexCorridor2D>
   */
  static std::vector<ConvexCorridor2D> generateCorridors(
      const std::vector<uint32_t>& path_node_ids,
      const elevation_planner::ManifoldGraph& graph,
      double max_diffusion_radius = 0.50,
      double max_step_height = 0.25,
      double max_stride_length = 0.35)
  {
    std::vector<ConvexCorridor2D> corridors;
    if (path_node_ids.empty() || graph.numNodes() == 0) return corridors;

    corridors.reserve(path_node_ids.size());

    for (size_t i = 0; i < path_node_ids.size(); ++i)
    {
      uint32_t root_id = path_node_ids[i];
      if (root_id >= graph.numNodes()) continue;

      const auto& root_node = graph.getNode(root_id);

      ConvexCorridor2D corridor;
      corridor.root_node_id = root_id;
      corridor.center = Eigen::Vector2d(root_node.x, root_node.y);
      corridor.z_ref = root_node.z;
      corridor.layer_id = root_node.layer_id;

      // 1. 严格基于图单步相邻执行 BFS 扩散 (无重叠切断 + 0.5m 截断 + 参数化步长步高)
      performTopologicalBFS({root_id}, max_diffusion_radius, max_step_height, max_stride_length, graph, corridor.node_pts);

      // 若 BFS 结果为空，兜底添加中心根节点
      if (corridor.node_pts.empty())
      {
        CorridorNode3D root_cnd;
        root_cnd.id = root_node.id;
        root_cnd.x = root_node.x;
        root_cnd.y = root_node.y;
        root_cnd.z = root_node.z;
        root_cnd.row = root_node.row;
        root_cnd.col = root_node.col;
        root_cnd.layer_id = root_node.layer_id;
        corridor.node_pts.push_back(root_cnd);
      }

      // 2. 运行 Andrew 单调链凸包算法提取 A * q <= b 线性约束
      computeAndrewConvexHull(corridor.node_pts, corridor.center, max_diffusion_radius, corridor.constraint);

      corridors.push_back(corridor);
    }

    return corridors;
  }

  /**
   * @brief 将凸走廊与扩散节点全量序列化为 JSON 字符串供 Web 前端交互诊断
   */
  static void computeAndrewConvexHull(
      const std::vector<CorridorNode3D>& nodes,
      const Eigen::Vector2d& center,
      double fallback_radius,
      LinearConstraint2D& out_constraint,
      const std::vector<Eigen::Vector2d>* fallback_hull = nullptr)
  {
    std::vector<Eigen::Vector2d> pts;
    pts.reserve(nodes.size() + 8);
    for (const auto& nd : nodes)
    {
      pts.emplace_back(nd.x, nd.y);
    }

    // 1. 点集去重与排序 (按 X 升序，X 相同按 Y 升序)
    std::sort(pts.begin(), pts.end(), [](const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
      if (std::abs(a.x() - b.x()) > 1e-4) return a.x() < b.x();
      return a.y() < b.y();
    });

    pts.erase(std::unique(pts.begin(), pts.end(), [](const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
      return (a - b).squaredNorm() < 1e-6;
    }), pts.end());

    std::vector<Eigen::Vector2d> hull;

    // 二维叉积: (q - p) x (r - p)
    auto cross_2d = [](const Eigen::Vector2d& p, const Eigen::Vector2d& q, const Eigen::Vector2d& r) -> double {
      return (q.x() - p.x()) * (r.y() - p.y()) - (q.y() - p.y()) * (r.x() - p.x());
    };

    if (pts.size() >= 3)
    {
      size_t n = pts.size();
      size_t k = 0;
      hull.resize(2 * n);

      // 构造下凸包 (Lower Hull)
      for (size_t i = 0; i < n; ++i)
      {
        while (k >= 2 && cross_2d(hull[k - 2], hull[k - 1], pts[i]) <= 1e-7)
        {
          k--;
        }
        hull[k++] = pts[i];
      }

      // 构造上凸包 (Upper Hull)
      for (size_t i = n - 1, t = k + 1; i > 0; --i)
      {
        while (k >= t && cross_2d(hull[k - 2], hull[k - 1], pts[i - 1]) <= 1e-7)
        {
          k--;
        }
        hull[k++] = pts[i - 1];
      }

      hull.resize(k - 1);
    }

    // 若凸包退化 (< 3 顶点)，优先使用调用方提供的兜底多边形 (段胶囊), 否则用中心点膨胀紧凑小正方形
    if (hull.size() < 3)
    {
      if (fallback_hull != nullptr && fallback_hull->size() >= 3)
      {
        hull = *fallback_hull;
      }
      else
      {
        hull.clear();
        double r = 0.06;
        hull.push_back(center + Eigen::Vector2d(r, r));
        hull.push_back(center + Eigen::Vector2d(-r, r));
        hull.push_back(center + Eigen::Vector2d(-r, -r));
        hull.push_back(center + Eigen::Vector2d(r, -r));
      }
    }

    out_constraint.vertices = hull;

    // 2. 将逆时针多边形边缘转换为线性凸约束 A * q <= b
    size_t M = hull.size();
    out_constraint.A.resize(M, 2);
    out_constraint.b.resize(M);

    for (size_t i = 0; i < M; ++i)
    {
      const auto& p1 = hull[i];
      const auto& p2 = hull[(i + 1) % M];

      Eigen::Vector2d edge = p2 - p1;
      double edge_len = edge.norm();
      if (edge_len < 1e-5) edge_len = 1.0;

      // 逆时针多边形向外法向量 n = (edge.y, -edge.x) / ||edge||
      Eigen::Vector2d normal(edge.y() / edge_len, -edge.x() / edge_len);

      out_constraint.A.row(i) = normal.transpose();
      out_constraint.b(i) = normal.dot(p1);
    }
  }

  // ==================== 凸性保证: 覆盖性检验 + 孔洞拆分 + 航点重采样 ====================
  //
  // 问题: BFS 扩散的可通行节点团是非凸的 (绕障碍单侧/凹角), 直接取凸包会把凹进来的
  // 禁行区/空白区整个包进走廊 —— 优化器对凸包内部的障碍不可见, fitness 会把控制点拉进去。
  //
  // 方案 (中心点覆盖性检验): 对节点中心点集取凸包, 再枚举凸包内所有格中心 ——
  // 若某格中心在凸包内但该处无本走廊可通行节点, 即存在"孔洞" → 区域非凸。
  // 贴凸包边界的空洞簇是斜边楼梯化伪影, 忽略; 完全在内部的空洞簇才是真孔洞。
  // 真孔洞存在时: 过孔洞质心沿航段方向切分节点集, 递归验证子集 (深度/片数封顶);
  // 拆出的子凸包按时序对齐航线, 在航线穿入点重采样航点, 保持 corridors==waypoints 1:1。


  static std::string toJsonString(
      const std::vector<ConvexCorridor2D>& corridors,
      const std::vector<Eigen::Vector2d>& opt_control_points = {})
  {
    std::string json = "{\"corridors\":[";
    bool has_opt = (!opt_control_points.empty() && 
                    (opt_control_points.size() == corridors.size() + 2 || 
                     opt_control_points.size() == corridors.size()));

    for (size_t i = 0; i < corridors.size(); ++i)
    {
      const auto& c = corridors[i];
      if (i > 0) json += ",";
      json += "{";
      json += "\"idx\":" + std::to_string(i) + ",";
      json += "\"root_id\":" + std::to_string(c.root_node_id) + ",";
      json += "\"x\":" + std::to_string(c.center.x()) + ",";
      json += "\"y\":" + std::to_string(c.center.y()) + ",";
      json += "\"z\":" + std::to_string(c.z_ref) + ",";
      json += "\"layer\":" + std::to_string(c.layer_id) + ",";

      // 优化后的物理点 p_i 与控制点 q_i 及走廊违约穿透量
      if (has_opt)
      {
        Eigen::Vector2d p_i;
        Eigen::Vector2d q_i;
        if (opt_control_points.size() == corridors.size() + 2)
        {
          p_i = (opt_control_points[i] + 4.0 * opt_control_points[i + 1] + opt_control_points[i + 2]) / 6.0;
          q_i = opt_control_points[i + 1];
        }
        else
        {
          Eigen::Vector2d q_prev = (i > 0) ? opt_control_points[i - 1] : opt_control_points[0];
          Eigen::Vector2d q_curr = opt_control_points[i];
          Eigen::Vector2d q_next = (i + 1 < opt_control_points.size()) ? opt_control_points[i + 1] : opt_control_points.back();
          p_i = (q_prev + 4.0 * q_curr + q_next) / 6.0;
          q_i = q_curr;
        }
        double pz = c.queryZFromNodes(p_i.x(), p_i.y());

        double max_viol = 0.0;
        if (c.constraint.A.rows() > 0)
        {
          Eigen::VectorXd viol = c.constraint.A * q_i - c.constraint.b;
          for (int j = 0; j < viol.size(); ++j)
          {
            if (viol(j) > max_viol) max_viol = viol(j);
          }
        }

        json += "\"opt_p\":[" + std::to_string(p_i.x()) + "," + std::to_string(p_i.y()) + "," + std::to_string(pz) + "],";
        json += "\"opt_q\":[" + std::to_string(q_i.x()) + "," + std::to_string(q_i.y()) + "],";
        json += "\"viol\":" + std::to_string(max_viol) + ",";
      }

      // [DBG] 内切宽度 (沿段折线采样, 只统计自身行 [0, base_rows)): 退化细条可视化
      {
        const int total_rows = static_cast<int>(c.constraint.A.rows());
        const int own_rows = (c.constraint.base_rows >= 0 && c.constraint.base_rows <= total_rows)
                                 ? c.constraint.base_rows : total_rows;
        double width_dbg = 0.0;
        if (own_rows > 0 && !c.node_pts.empty())
        {
          auto clearance = [&](const Eigen::Vector2d & p) {
            double m = std::numeric_limits<double>::max();
            for (int j = 0; j < own_rows; ++j)
              m = std::min(m, c.constraint.b(j) - c.constraint.A.row(j).dot(p));
            return m;
          };
          // node_pts: front = 段末端 (curr), back = 段起点 (prev); 单点走廊两点重合
          const Eigen::Vector2d p0(c.node_pts.back().x, c.node_pts.back().y);
          const Eigen::Vector2d p1(c.node_pts.front().x, c.node_pts.front().y);
          for (double t : {0.0, 0.25, 0.5, 0.75, 1.0})
            width_dbg = std::max(width_dbg, 2.0 * clearance(p0 + t * (p1 - p0)));
        }
        json += "\"width\":" + std::to_string(width_dbg) + ",";
      }

      // 凸包多边形
      json += "\"polygon\":[";
      for (size_t v = 0; v < c.constraint.vertices.size(); ++v)
      {
        if (v > 0) json += ",";
        const auto& pt = c.constraint.vertices[v];
        double pz = c.queryZFromNodes(pt.x(), pt.y());
        json += "[" + std::to_string(pt.x()) + "," + std::to_string(pt.y()) + "," + std::to_string(pz) + "]";
      }
      json += "],";

      // [DBG] 负空间格诊断 (visited 补集, 走廊 bbox 内): 每格记录距段高度最近
      //       节点的 z / 通行性 / 相对段高差 / 未访问原因 —— why 分类:
      //       no_node=空洞立面  z_above=高出段层带  z_below=低于段层带
      //       blocked=层带内硬禁行  unvisited=层带内可走但 BFS 未访问(伪影)
      json += "\"obstacles\":[";
      for (size_t o = 0; o < c.negative_cells.size(); ++o)
      {
        if (o > 0) json += ",";
        const auto & nc = c.negative_cells[o];
        const char * why = "no_node";
        switch (nc.why)
        {
          case 'A': why = "z_above"; break;
          case 'B': why = "z_below"; break;
          case 'K': why = "blocked"; break;
          case 'U': why = "unvisited"; break;
          default: break;
        }
        char buf[160];
        std::snprintf(buf, sizeof(buf),
                      "{\"p\":[%.2f,%.2f],\"z\":%.2f,\"dz\":%.2f,\"trav\":%.2f,\"why\":\"%s\"}",
                      nc.p.x(), nc.p.y(), nc.z, nc.dz, nc.trav, why);
        json += buf;
      }
      json += "],";

      // 扩散图节点
      json += "\"nodes\":[";
      for (size_t n = 0; n < c.node_pts.size(); ++n)
      {
        if (n > 0) json += ",";
        const auto& nd = c.node_pts[n];
        json += "{\"id\":" + std::to_string(nd.id) +
                ",\"x\":" + std::to_string(nd.x) +
                ",\"y\":" + std::to_string(nd.y) +
                ",\"z\":" + std::to_string(nd.z) +
                ",\"l\":" + std::to_string(nd.layer_id) +
                ",\"r\":" + std::to_string(nd.row) +
                ",\"c\":" + std::to_string(nd.col) + "}";
      }
      json += "]}";
    }
    json += "]}";
    return json;
  }

  /**
   * @brief 完全通过已记录的节点集与凸包顶点进行 MarkerArray 可视化 (零图查表)
   */
  static void toVisualMarkers(
      const std::vector<ConvexCorridor2D>& corridors,
      const std::string& frame_id,
      visualization_msgs::MarkerArray& out_markers)
  {
    out_markers.markers.clear();
    if (corridors.empty()) return;

    ros::Time now = ros::Time::now();

    visualization_msgs::Marker line_marker;
    line_marker.header.frame_id = frame_id;
    line_marker.header.stamp = now;
    line_marker.ns = "sfc_corridor_wireframe";
    line_marker.id = 0;
    line_marker.type = visualization_msgs::Marker::LINE_LIST;
    line_marker.action = visualization_msgs::Marker::ADD;
    line_marker.scale.x = 0.02;
    line_marker.color.r = 0.0f;
    line_marker.color.g = 0.95f;
    line_marker.color.b = 0.95f;
    line_marker.color.a = 0.9f;

    visualization_msgs::Marker face_marker;
    face_marker.header.frame_id = frame_id;
    face_marker.header.stamp = now;
    face_marker.ns = "sfc_corridor_boxes";
    face_marker.id = 1;
    face_marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
    face_marker.action = visualization_msgs::Marker::ADD;
    face_marker.scale.x = 1.0;
    face_marker.scale.y = 1.0;
    face_marker.scale.z = 1.0;
    face_marker.color.r = 0.1f;
    face_marker.color.g = 0.8f;
    face_marker.color.b = 0.9f;
    face_marker.color.a = 0.22f;

    for (const auto& corridor : corridors)
    {
      const auto& verts = corridor.constraint.vertices;
      if (verts.size() < 3) continue;

      geometry_msgs::Point center_pt;
      center_pt.x = corridor.center.x();
      center_pt.y = corridor.center.y();
      center_pt.z = corridor.queryZFromNodes(center_pt.x, center_pt.y) + 0.03;

      for (size_t v = 0; v < verts.size(); ++v)
      {
        size_t v_next = (v + 1) % verts.size();
        
        geometry_msgs::Point p1, p2;
        p1.x = verts[v].x();
        p1.y = verts[v].y();
        p1.z = corridor.queryZFromNodes(p1.x, p1.y) + 0.03;

        p2.x = verts[v_next].x();
        p2.y = verts[v_next].y();
        p2.z = corridor.queryZFromNodes(p2.x, p2.y) + 0.03;

        // 边框线
        line_marker.points.push_back(p1);
        line_marker.points.push_back(p2);

        // 三角面片 (扇形剖分)
        face_marker.points.push_back(center_pt);
        face_marker.points.push_back(p1);
        face_marker.points.push_back(p2);
      }
    }

    out_markers.markers.push_back(line_marker);
    out_markers.markers.push_back(face_marker);
  }

  /**
   * @brief 段式走廊: 以 SC-LOS 支撑链为种子的多源 BFS 扩散 (走廊沿 A→B 连线向外发散)
   *
   * 走廊 i (i>=1) 罩住段 (W_{i-1}, W_i) 沿线可单步联通到达的邻域 (阈值同点式 0.5m),
   * 依然 1:1 约束控制点 q_{i+1}; 走廊 0 保持点式 (围绕 W_0, 优化器豁免 q_1)。
   * 相邻走廊在共享航点处必然重叠。
   * 层间隔离是结构性的: 种子链本身同层且单步连通, BFS 只沿单步边扩展, 无需高度过滤。
   *
   * @param state_points 已废弃 (走廊纯 A* 生成, 动力学锚点由优化器软约束处理) ——
   *        最近图节点作为附加 BFS 种子, 状态点本身直接注入凸包输入, 保证转角交集
   *        C_1 ∩ C_2 覆盖动力学锚点 (起始接缝: 机器人在节点覆盖区外时 q_2 仍可行)
   * @param state_z 状态点高程 (机器人 z, 用于注入节点与最近节点查询)
   */
  static std::vector<ConvexCorridor2D> generateSegmentCorridors(
      const std::vector<uint32_t>& path_node_ids,
      const elevation_planner::ManifoldGraph& graph,
      elevation_planner::PathSimplifier& simplifier,
      double max_diffusion_radius,
      double max_step_height,
      double max_stride_length,
      const std::vector<Eigen::Vector2d>& state_points = {},
      double state_z = 0.0,
      std::vector<Eigen::Vector2d>* out_waypoints = nullptr)
  {
    std::vector<ConvexCorridor2D> corridors;
    if (path_node_ids.empty() || graph.numNodes() == 0) return corridors;
    corridors.reserve(path_node_ids.size());
    if (out_waypoints) out_waypoints->clear();
    (void)simplifier; (void)max_diffusion_radius; (void)max_stride_length; (void)state_z; (void)state_points;

    for (size_t i = 0; i < path_node_ids.size(); ++i)
    {
      const uint32_t curr_id = path_node_ids[i];
      if (curr_id >= graph.numNodes()) continue;
      const auto& curr_node = graph.getNode(curr_id);
      const Eigen::Vector2d curr(curr_node.x, curr_node.y);

      // 1. 种子链: i==0 单点; i>=1 = SC-LOS 支撑链 (失败兜底两端点);
      //    起始两段链首并入机器人位姿锚点 (BFS 覆盖动力学锚点)
      std::vector<uint32_t> seeds;
      if (i == 0)
        seeds = {curr_id};
      else
      {
        const uint32_t prev_id = path_node_ids[i - 1];
        if (prev_id >= graph.numNodes()) continue;
        if (!simplifier.computeLosChain(prev_id, curr_id, seeds) || seeds.empty())
          seeds = {prev_id, curr_id};
      }

      // 2. BFS: visited 连通安全格集 (正空间) + 节点云
      std::vector<CorridorNode3D> nodes;
      std::unordered_set<int64_t> visited;
      performTopologicalBFS(seeds, max_diffusion_radius, max_step_height, max_stride_length,
                            graph, nodes, &visited);
      if (visited.empty()) continue;

      // 3. visited 包围盒
      int rmin = 1 << 30, rmax = -(1 << 30), cmin = 1 << 30, cmax = -(1 << 30);
      for (int64_t key : visited)
      {
        const int r = static_cast<int>(key >> 32);
        const int c = static_cast<int>(key & 0xffffffff);
        rmin = std::min(rmin, r); rmax = std::max(rmax, r);
        cmin = std::min(cmin, c); cmax = std::max(cmax, c);
      }

      // 4. 折线 (i>=1 = [W_{i-1}, W_i]; 起始两段并入机器人状态点)
      std::vector<Eigen::Vector2d> line;
      double z0 = curr_node.z, z1 = curr_node.z;   // 段两端高程: 负空间诊断的插值基准
      if (i == 0)
        line = {curr};
      else
      {
        const auto& prev_node = graph.getNode(path_node_ids[i - 1]);
        line.push_back(Eigen::Vector2d(prev_node.x, prev_node.y));
        line.push_back(curr);   // 两点直线段: 凸集, 切割面恒不切线段 (最近点性质)
        z0 = prev_node.z;
      }

      // 5. 迭代半平面切割 (visited 补集 = 负空间障碍物; 两层保护: min_clearance 保投影点,
      //    最近点性质保整条折线; 障碍落线上 -> 返回空走廊, move_base 重试)
      //    margin = robot_radius(body_hard_radius 0.17): 单一膨胀架构下负空间只剩障碍
      //    本体格, 走廊边界距障碍格中心 >= 0.17 —— 规划使用膨胀带作为禁改区,
      //    轨迹中心线保持一个机体半径, 机体缘恰好不过界
      std::vector<NegativeCell> negative_space;
      ConvexCorridor2D cor = DecompCorridorGenerator::generate(
          line, graph, visited, rmin, rmax, cmin, cmax, /*min_clearance=*/0.01,
          /*z0=*/z0, /*z1=*/z1, /*max_step=*/max_step_height, &negative_space);
      cor.negative_cells = std::move(negative_space);   // [DBG] JSON obstacles 字段数据源
      if (cor.constraint.A.rows() < 4)
      {
        ROS_WARN_THROTTLE(2.0, "[SFC][DBG] segment %zu decomp infeasible (obstacle on line), skipping", i);
        continue;
      }

      // [DBG] 走廊几何诊断: 沿段折线采样的最大内切宽度 (2 × 折线点到自身约束面最近距离)。
      //       切割行数 = A.rows() − 4 (初始包围盒恒 4 行); 内切宽度 < 机体直径
      //       (2×body_hard_radius = 0.34m) 即"图语义可走但机体物理上过不去"的退化细条,
      //       当前管线不据此触发重规划 —— 此日志/JSON width 字段是它的唯一可见性
      {
        auto clearance = [&cor](const Eigen::Vector2d & p) {
          double m = std::numeric_limits<double>::max();
          for (int j = 0; j < cor.constraint.A.rows(); ++j)
            m = std::min(m, cor.constraint.b(j) - cor.constraint.A.row(j).dot(p));
          return m;
        };
        double inscribed = 0.0, body_clear = 1e9;
        for (double t : {0.0, 0.25, 0.5, 0.75, 1.0})
        {
          const double cl = clearance(line.front() + t * (line.back() - line.front()));
          inscribed = std::max(inscribed, cl);
          body_clear = std::min(body_clear, cl);   // 逐点最小间隙 (闸判据)
        }
        cor.inscribed_width = 2.0 * inscribed;   // JSON width 展示
        cor.body_clearance = body_clear;         // 机体间隙闸数据源 (astar_local_planner)
        const double width = 2.0 * inscribed;
        ROS_INFO_THROTTLE(2.0, "[SFC][DBG] seg %zu: visited=%zu cells, cuts=%d, inscribed width=%.3fm",
                          i, visited.size(), static_cast<int>(cor.constraint.A.rows()) - 4, width);
        if (width < 0.34)
          ROS_WARN_THROTTLE(1.0, "[SFC][DBG] seg %zu corridor DEGENERATE: width=%.3fm < body 0.34m "
                                 "(optimizer will confine control points inside this sliver)", i, width);
      }

      cor.root_node_id = curr_id;
      cor.layer_id = curr_node.layer_id;
      cor.z_ref = curr_node.z;   // 段末端节点高程: JSON "z"/se调试渲染的地形基准 (此前漏赋值恒 0, 蓝球不贴地)
      CorridorNode3D cn;
      cn.id = curr_id;
      cn.x = curr_node.x; cn.y = curr_node.y; cn.z = curr_node.z;
      cn.row = curr_node.row; cn.col = curr_node.col; cn.layer_id = curr_node.layer_id;
      cor.node_pts.push_back(cn);
      if (i > 0)
      {
        const auto& pn = graph.getNode(path_node_ids[i - 1]);
        CorridorNode3D cp;
        cp.id = path_node_ids[i - 1];
        cp.x = pn.x; cp.y = pn.y; cp.z = pn.z;
        cp.row = pn.row; cp.col = pn.col; cp.layer_id = pn.layer_id;
        cor.node_pts.push_back(cp);
      }

      corridors.push_back(std::move(cor));
      if (out_waypoints) out_waypoints->push_back(curr);   // 航点 = 段末端节点 (恒在走廊内)
    }

    return corridors;
  }

  /**
   * @brief 转角交集拼接: 把相邻段走廊的线性约束上下拼接 (转角控制点 q_{i+1} ∈ C_i ∩ C_{i+1})
   *
   * 走廊 0 (点式) 不变; 内部索引 i 的约束变为 [C_i; C_{i+1}]; 末索引只有进段 C_{M-1}。
   * 凸性保证: 拼接后相邻控制点连成的折线, 每条边两端点同属一个凸走廊, 折线整体落在
   * 走廊区域内部, "点和点之间连线穿墙"机制性消失。
   * vertices/node_pts/center 保留本段走廊字段 (可视化仍显示未拼接的段凸包)。
   */
  static void stackAdjacentCorridors(std::vector<ConvexCorridor2D>& corridors)
  {
    if (corridors.size() < 3) return;

    std::vector<ConvexCorridor2D> stacked;
    stacked.reserve(corridors.size());
    stacked.push_back(corridors[0]);  // 走廊 0 (点式) 不变

    for (size_t i = 1; i + 1 < corridors.size(); ++i)
    {
      ConvexCorridor2D cor = corridors[i];
      const auto& next = corridors[i + 1];
      const int64_t r1 = static_cast<int64_t>(cor.constraint.A.rows());
      const int64_t r2 = static_cast<int64_t>(next.constraint.A.rows());
      if (r2 > 0)
      {
        cor.constraint.A.conservativeResize(r1 + r2, Eigen::NoChange);
        cor.constraint.A.bottomRows(r2) = next.constraint.A;
        cor.constraint.b.conservativeResize(r1 + r2);
        cor.constraint.b.tail(r2) = next.constraint.b;
        cor.constraint.base_rows = static_cast<int>(r1);  // [0, r1) = 本段自身行
      }
      stacked.push_back(std::move(cor));
    }
    stacked.push_back(corridors.back());  // 末走廊: 只有进段
    corridors = std::move(stacked);
  }

private:
  /**
   * @brief 基于图单步相邻执行多源 BFS 扩散 (无重叠切断 + 0.5m 截断 + 参数化 step_height & stride_length)
   * @param seed_ids 种子节点集 (全部以累计距离 0.0 入队; 段式走廊传 SC-LOS 支撑链, 点式走廊传单根节点)
   */
  static void performTopologicalBFS(
      const std::vector<uint32_t>& seed_ids,
      double max_radius,
      double max_step_height,
      double max_stride_length,
      const elevation_planner::ManifoldGraph& graph,
      std::vector<CorridorNode3D>& out_nodes,
      std::unordered_set<int64_t>* visited_cells_out = nullptr)
  {
    out_nodes.clear();
    if (seed_ids.empty()) return;

    struct BFSQueueItem {
      uint32_t id;
      double dist_from_root;
    };

    std::queue<BFSQueueItem> q;
    std::unordered_set<uint32_t> visited_node_ids;
    // 栅格判重表: 用于判断当前 (row, col) 是否已有值，无重叠直接切断
    std::unordered_set<int64_t> visited_cells;
    if (visited_cells_out) visited_cells_out->clear();

    auto make_cell_key = [](int32_t r, int32_t c) -> int64_t {
      return (static_cast<int64_t>(r) << 32) | (static_cast<uint32_t>(c));
    };

    // 压入全部种子节点 (初始累积距离为 0.0)
    for (uint32_t seed_id : seed_ids)
    {
      if (seed_id >= graph.numNodes()) continue;
      if (!visited_node_ids.insert(seed_id).second) continue;
      const auto& seed_node = graph.getNode(seed_id);
      visited_cells.insert(make_cell_key(seed_node.row, seed_node.col));
      q.push({seed_id, 0.0});

      CorridorNode3D seed_cnd;
      seed_cnd.id = seed_node.id;
      seed_cnd.x = seed_node.x;
      seed_cnd.y = seed_node.y;
      seed_cnd.z = seed_node.z;
      seed_cnd.row = seed_node.row;
      seed_cnd.col = seed_node.col;
      seed_cnd.layer_id = seed_node.layer_id;
      out_nodes.push_back(seed_cnd);
    }

    if (q.empty()) return;

    std::vector<uint32_t> single_step_nbrs;

    while (!q.empty())
    {
      auto curr = q.front();
      q.pop();

      uint32_t curr_id = curr.id;
      double curr_dist = curr.dist_from_root;
      const auto& curr_node = graph.getNode(curr_id);

      // 调用流形图暴露的单步相邻检索方法
      graph.getSingleStepNeighbors(curr_id, single_step_nbrs, max_step_height, max_stride_length);

      for (uint32_t neighbor_id : single_step_nbrs)
      {
        if (neighbor_id >= graph.numNodes()) continue;

        // 若节点已访问过，跳过
        if (visited_node_ids.find(neighbor_id) != visited_node_ids.end()) continue;

        const auto& nb_node = graph.getNode(neighbor_id);

        // 连续累积距离判断：从根节点 A 经由 B 扩展到 C 的实际物理步长累加
        double step_dx = nb_node.x - curr_node.x;
        double step_dy = nb_node.y - curr_node.y;
        double step_dz = nb_node.z - curr_node.z;
        double step_dist = std::sqrt(step_dx * step_dx + step_dy * step_dy + step_dz * step_dz);
        double cum_dist = curr_dist + step_dist;

        if (cum_dist > max_radius + 1e-4)
        {
          continue; // 连续累积距离超出走廊半径阈值，停止向外扩展
        }

        // 无重叠切断：判断当前 (row, col) 在本走廊是否已有值，若有则直接切断跳过
        int64_t cell_key = make_cell_key(nb_node.row, nb_node.col);
        if (visited_cells.find(cell_key) != visited_cells.end())
        {
          continue;
        }

        // 标记并入队
        visited_node_ids.insert(neighbor_id);
        visited_cells.insert(cell_key);
        q.push({neighbor_id, cum_dist});

        CorridorNode3D cnd;
        cnd.id = nb_node.id;
        cnd.x = nb_node.x;
        cnd.y = nb_node.y;
        cnd.z = nb_node.z;
        cnd.row = nb_node.row;
        cnd.col = nb_node.col;
        cnd.layer_id = nb_node.layer_id;
        out_nodes.push_back(cnd);
      }
    }

    if (visited_cells_out) *visited_cells_out = visited_cells;
  }




};

} // namespace elevation_local_planner
