#pragma once

#include "elevation_planner_core/manifold_graph.hpp"
#include "elevation_planner_core/path_simplifier.hpp"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Dense>
#include <vector>
#include <queue>
#include <unordered_set>
#include <cmath>
#include <algorithm>
#include <limits>

namespace elevation_local_planner
{

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
   * @param state_points 机器人状态点 (位姿 + 一步锚点): 非空时并入前两条段走廊 (i=1,2) ——
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

    for (size_t i = 0; i < path_node_ids.size(); ++i)
    {
      const uint32_t curr_id = path_node_ids[i];
      if (curr_id >= graph.numNodes()) continue;
      const auto& curr_node = graph.getNode(curr_id);

      // 种子链: i==0 点式围绕 W_0; i>=1 = 该段 SC-LOS 支撑链 (失败兜底两端点)
      std::vector<uint32_t> seeds;
      if (i == 0)
      {
        seeds = {curr_id};
      }
      else
      {
        const uint32_t prev_id = path_node_ids[i - 1];
        if (prev_id >= graph.numNodes()) continue;

        if (!simplifier.computeLosChain(prev_id, curr_id, seeds) || seeds.empty())
        {
          seeds = {prev_id, curr_id};
        }

        // 起始两条段走廊并入机器人状态: 最近图节点作为附加 BFS 种子,
        // 使 C_1 / C_2 的凸包覆盖机器人位姿与一步锚点 (转角交集含动力学锚点)
        if (!state_points.empty() && (i == 1 || i == 2))
        {
          uint32_t near_id = 0;
          if (graph.findClosestNode(state_points[0].x(), state_points[0].y(), state_z,
                                    near_id, 0.8, 0.35))
          {
            seeds.push_back(near_id);
          }
        }
      }

      // 段级递归走廊构建: 覆盖性检验失败时在孔洞最近的 LOS 链节点处切分,
      // 子走廊共享真实切分节点 (拼接交集恒可行), 航点 = 各子链末端真实节点
      std::vector<ConvexCorridor2D> sub_corridors;
      std::vector<Eigen::Vector2d> sub_waypoints;
      bool inject_state = !state_points.empty() && (i == 1 || i == 2);
      buildSegmentPiecesRecursive(
          seeds, graph, max_diffusion_radius, max_step_height, max_stride_length,
          /*z_band=*/max_step_height, state_points, state_z, inject_state,
          /*depth=*/0, sub_corridors, sub_waypoints);

      for (auto& sc : sub_corridors)
        corridors.push_back(std::move(sc));
      if (out_waypoints)
        for (auto& w : sub_waypoints) out_waypoints->push_back(w);
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
      std::vector<CorridorNode3D>& out_nodes)
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
  }

  /**
   * @brief Andrew's Monotone Chain 2D 凸包算法并构建 Ax <= b 线性约束
   * @param fallback_hull 退化 (<3 顶点) 时的自定义兜底多边形 (须 CCW); 空则退回中心 0.06m 小方块
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

  struct HoleSplitResult
  {
    bool convex{true};                                  ///< 覆盖性检验通过 (无内部孔洞)
    Eigen::Vector2d hole_centroid{0.0, 0.0};            ///< 最大真孔洞簇质心 (拆分锚点)
    Eigen::Vector2d hole_dir{0.0, 0.0};                 ///< 建议分割方向 (垂直于航段方向)
  };

  /**
   * @brief 中心点覆盖性检验: 凸包内格中心是否全部被本走廊节点覆盖
   * @param nodes 本走廊 BFS 节点集 (即中心点集)
   * @param graph 流形图 (查任意格中心处是否有可通行节点; 含融合动态封锁过滤)
   * @param seg_dir 航段方向 (输出建议分割方向用)
   * @return convex=true 直接用; convex=false 时 hole_centroid/hole_dir 给出拆分依据
   */
  static HoleSplitResult checkHullCoverage(
      const std::vector<CorridorNode3D>& nodes,
      const elevation_planner::ManifoldGraph& graph,
      const Eigen::Vector2d& seg_dir,
      double z_band)
  {
    HoleSplitResult res;
    if (nodes.size() < 4) return res;  // 过小节点团不判定 (凸包必退化, 无凹余量)

    // 1. 节点中心点凸包 (Andrew 单调链, 复用叉积写法)
    std::vector<Eigen::Vector2d> pts;
    pts.reserve(nodes.size());
    for (const auto& nd : nodes) pts.emplace_back(nd.x, nd.y);
    std::sort(pts.begin(), pts.end(), [](const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
      if (std::abs(a.x() - b.x()) > 1e-4) return a.x() < b.x();
      return a.y() < b.y();
    });
    pts.erase(std::unique(pts.begin(), pts.end(), [](const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
      return (a - b).squaredNorm() < 1e-6;
    }), pts.end());
    if (pts.size() < 4) return res;

    auto cross2 = [](const Eigen::Vector2d& p, const Eigen::Vector2d& q, const Eigen::Vector2d& r) {
      return (q.x() - p.x()) * (r.y() - p.y()) - (q.y() - p.y()) * (r.x() - p.x());
    };
    std::vector<Eigen::Vector2d> hull;
    size_t n = pts.size(), k = 0;
    hull.resize(2 * n);
    for (size_t i = 0; i < n; ++i)
    {
      while (k >= 2 && cross2(hull[k - 2], hull[k - 1], pts[i]) <= 1e-7) k--;
      hull[k++] = pts[i];
    }
    for (size_t i = n - 1, t = k + 1; i > 0; --i)
    {
      while (k >= t && cross2(hull[k - 2], hull[k - 1], pts[i - 1]) <= 1e-7) k--;
      hull[k++] = pts[i - 1];
    }
    hull.resize(k - 1);
    if (hull.size() < 3) return res;

    // 2. bbox 内逐格中心做点在凸包内测试, 收集"凸包内但无本走廊节点"的空洞格
    double min_x = pts.front().x(), max_x = pts.front().x();
    double min_y = pts.front().y(), max_y = pts.front().y();
    for (const auto& p : pts)
    {
      min_x = std::min(min_x, p.x()); max_x = std::max(max_x, p.x());
      min_y = std::min(min_y, p.y()); max_y = std::max(max_y, p.y());
    }

    // 孔洞判据 = 凸包内本层带 (|z| <= z_band) 被禁行节点占据的格。
    // 凸包含 "BFS 未扩散到的自由/空白区" 无害 (优化器落点处 drape 照常取高程),
    // 危险的只有禁行区 —— 动态障碍经融合引擎刷新 traversability 后天然被此检验捕获
    double z_lo = std::numeric_limits<double>::max(), z_hi = -std::numeric_limits<double>::max();
    for (const auto& nd : nodes)
    {
      z_lo = std::min(z_lo, static_cast<double>(nd.z));
      z_hi = std::max(z_hi, static_cast<double>(nd.z));
    }
    z_lo -= z_band;
    z_hi += z_band;

    const double grid_res = graph.getResolution();
    struct HoleCell { int r; int c; bool on_rim; };
    std::vector<HoleCell> hole_cells;
    const int rim = 1;  // 边界带: 距凸包边缘一格内的空洞视为斜边伪影

    int r_lo = 0, c_lo = 0, r_hi = 0, c_hi = 0;
    graph.toGridIndex(min_x, min_y, r_lo, c_lo);
    graph.toGridIndex(max_x, max_y, r_hi, c_hi);
    for (int r = r_lo; r <= r_hi; ++r)
    {
      for (int c = c_lo; c <= c_hi; ++c)
      {
        const double cx = graph.getMinX() + (r + 0.5) * grid_res;
        const double cy = graph.getMinY() + (c + 0.5) * grid_res;
        if (cx < min_x || cx > max_x || cy < min_y || cy > max_y) continue;

        // 点在凸包内 (半平面法, 凸包 CCW: 全部叉积 >= -eps)
        bool inside = true;
        int inside_edge_count = 0;
        for (size_t i = 0; i < hull.size() && inside; ++i)
        {
          const auto& p1 = hull[i];
          const auto& p2 = hull[(i + 1) % hull.size()];
          const double cr = cross2(p1, p2, Eigen::Vector2d(cx, cy));
          if (cr < -1e-9) inside = false;
          (void)inside_edge_count;
        }
        if (!inside) continue;

        // 本层带被禁行 → 孔洞格 (格内无节点/节点全可通行则合格)
        bool blocked_here = false;
        for (uint32_t nid : graph.getSpatialCellNodes(r, c))
        {
          const auto& nd = graph.getNode(nid);
          if (nd.traversability >= 0.95f &&
              static_cast<double>(nd.z) >= z_lo && static_cast<double>(nd.z) <= z_hi)
          {
            blocked_here = true;
            break;
          }
        }
        if (!blocked_here) continue;

        // 判断是否贴凸包边缘 (bbox 边缘近似 —— 凸包 ⊆ bbox, bbox 边缘即外圈)
        const bool on_rim = (r <= r_lo + rim || r >= r_hi - rim || c <= c_lo + rim || c >= c_hi - rim);
        hole_cells.push_back({r, c, on_rim});
      }
    }

    // 3. 过滤边缘伪影后聚类取最大真孔洞簇
    std::vector<HoleCell> real_holes;
    for (const auto& hc : hole_cells)
    {
      if (!hc.on_rim) real_holes.push_back(hc);
    }
    if (real_holes.empty()) return res;

    // 真孔洞簇质心 (4 邻域 flood fill 找最大簇, 防孤立噪声格误导拆分方向)
    std::vector<char> used(real_holes.size(), 0);
    size_t best_cluster_pts = 0;
    double sx = 0.0, sy = 0.0;
    size_t best_count = 0;
    for (size_t i = 0; i < real_holes.size(); ++i)
    {
      if (used[i]) continue;
      // BFS 聚簇
      std::vector<size_t> stack = {i};
      used[i] = 1;
      double cx = 0.0, cy = 0.0;
      size_t cnt = 0;
      while (!stack.empty())
      {
        size_t idx = stack.back();
        stack.pop_back();
        cx += graph.getMinX() + (real_holes[idx].r + 0.5) * grid_res;
        cy += graph.getMinY() + (real_holes[idx].c + 0.5) * grid_res;
        ++cnt;
        for (size_t j = 0; j < real_holes.size(); ++j)
        {
          if (used[j]) continue;
          if (std::abs(real_holes[j].r - real_holes[idx].r) + std::abs(real_holes[j].c - real_holes[idx].c) == 1)
          {
            used[j] = 1;
            stack.push_back(j);
          }
        }
      }
      if (cnt > best_count)
      {
        best_count = cnt;
        best_cluster_pts = cnt;
        sx = cx; sy = cy;
      }
    }
    (void)best_cluster_pts;
    if (best_count == 0) return res;

    // 孔洞过小 (少于 3 格) 视为噪声, 不拆 —— 避免在临界覆盖上反复切分
    if (best_count < 3) return res;

    res.convex = false;
    res.hole_centroid = Eigen::Vector2d(sx / best_count, sy / best_count);
    // 分割方向 = 垂直于航段方向 (沿航段切, 保持时序性)
    Eigen::Vector2d d = seg_dir;
    if (d.squaredNorm() < 1e-8) d = Eigen::Vector2d(1.0, 0.0);
    d.normalize();
    res.hole_dir = Eigen::Vector2d(-d.y(), d.x());
    return res;
  }

  /**
   * @brief 递归构建凸性保证走廊: 覆盖性检验失败时沿孔洞质心切分节点集, 递归验证子集
   * @return 按时序 (沿分割轴坐标排序) 的子走廊节点集列表
   */
  /**
   * @brief 段级递归走廊构建 (凸性保证): 覆盖性检验失败时, 取孔洞质心向 LOS 支撑链的投影节点 M,
   *        把链切成 [首..M] 与 [M..尾] 两个子链各自递归 —— 相邻子走廊共享真实节点 M
   *        (两端点必为 BFS 种子, 必在凸包内), 拼接约束 C_i ∩ C_i+1 ∋ M 恒可行;
   *        航点 = 各子链末端节点 (真实 A* 路径踏面, 依次为 M, ..., W_i)
   * @param seeds 有序支撑链节点 id (LOS 链 / 两端点兜底)
   * @param inject_state 首个输出走廊并入状态点 (i==1/2 动力学锚点覆盖), 用后清零
   */
  static void buildSegmentPiecesRecursive(
      const std::vector<uint32_t>& seeds,
      const elevation_planner::ManifoldGraph& graph,
      double max_diffusion_radius,
      double max_step_height,
      double max_stride_length,
      double z_band,
      const std::vector<Eigen::Vector2d>& state_points,
      double state_z,
      bool& inject_state,
      int depth,
      std::vector<ConvexCorridor2D>& out_corridors,
      std::vector<Eigen::Vector2d>& out_waypoints)
  {
    if (seeds.empty()) return;
    const auto& first_nd = graph.getNode(seeds.front());
    const auto& last_nd = graph.getNode(seeds.back());
    const Eigen::Vector2d seg_a(first_nd.x, first_nd.y);
    const Eigen::Vector2d seg_b(last_nd.x, last_nd.y);

    // 1. 围绕本子链 BFS
    std::vector<CorridorNode3D> nodes;
    performTopologicalBFS(seeds, max_diffusion_radius, max_step_height, max_stride_length, graph, nodes);
    if (nodes.empty()) return;

    // 2. 覆盖性检验
    HoleSplitResult chk = checkHullCoverage(nodes, graph, seg_b - seg_a, z_band);

    // 3. 非凸且可继续切: 选切分节点 M = 链内部节点距孔洞质心最近者
    if (!chk.convex && depth < 3 && seeds.size() >= 3)
    {
      size_t best_k = 0;
      double best_d = 1e18;
      for (size_t k = 1; k + 1 < seeds.size(); ++k)
      {
        const auto& nd = graph.getNode(seeds[k]);
        const double d = (Eigen::Vector2d(nd.x, nd.y) - chk.hole_centroid).squaredNorm();
        if (d < best_d) { best_d = d; best_k = k; }
      }
      if (best_k > 0)
      {
        std::vector<uint32_t> left(seeds.begin(), seeds.begin() + best_k + 1);
        std::vector<uint32_t> right(seeds.begin() + best_k, seeds.end());
        buildSegmentPiecesRecursive(left, graph, max_diffusion_radius, max_step_height,
                                    max_stride_length, z_band, state_points, state_z,
                                    inject_state, depth + 1, out_corridors, out_waypoints);
        buildSegmentPiecesRecursive(right, graph, max_diffusion_radius, max_step_height,
                                    max_stride_length, z_band, state_points, state_z,
                                    inject_state, depth + 1, out_corridors, out_waypoints);
        return;
      }
    }

    // 4. 基例: 单走廊 (凸 / 封顶回退 / 链太短不可再切)
    ConvexCorridor2D cor;
    cor.center = 0.5 * (seg_a + seg_b);
    cor.z_ref = first_nd.z;
    cor.layer_id = first_nd.layer_id;
    cor.root_node_id = seeds.front();
    cor.node_pts = nodes;

    // 状态点注入 (仅首走廊, 保证动力学锚点在凸包内)
    if (inject_state && !state_points.empty())
    {
      for (const auto& sp : state_points)
      {
        CorridorNode3D cnd;
        cnd.id = std::numeric_limits<uint32_t>::max();
        cnd.x = static_cast<float>(sp.x());
        cnd.y = static_cast<float>(sp.y());
        cnd.z = static_cast<float>(state_z);
        cnd.row = -1;
        cnd.col = -1;
        cnd.layer_id = first_nd.layer_id;
        cor.node_pts.push_back(cnd);
      }
      inject_state = false;
    }

    computeAndrewConvexHull(cor.node_pts, cor.center, max_diffusion_radius, cor.constraint);
    // 逐面受限膨胀: 外推 0.15m (> 格距), 距禁行格留 0.05m (凸 + 不吞禁行格均为数学保证)
    expandCorridorConstrained(cor, graph, z_band, /*expand_r=*/0.15, /*margin=*/0.05);

    out_corridors.push_back(std::move(cor));
    out_waypoints.push_back(seg_b);   // 航点 = 子链末端真实节点
  }

  // ==================== 逐面受限于障碍的走廊膨胀 (per-face capped offset) ====================
  //
  // 数学保证 (无需任何事后复检):
  // 1. 凸性: 凸集 = 半空间交集, 把某些 b_j 外推不改变集合性质, 交集恒为凸;
  // 2. 无障碍: d_j 取所有禁行格对该面的有向余量最小值 —— 对每个禁行格 c,
  //    它原本至少违反一条边 k (a_k·c - b_k > 0), 而 d_k <= (a_k·c - b_k) - margin,
  //    故膨胀后 a_k·c - b_k' >= margin > 0, c 仍被挡在外面。每个禁行格自带一条永不失效的约束。

  /**
   * @brief 对凸走廊约束做逐面封顶外推: 每条边外推 min(r, 该方向最近禁行格余量) - margin
   * @param cor 目标走廊 (constraint.A/b 原位更新, 顶点重建)
   * @param graph 流形图 (查禁行节点)
   * @param z_band 禁行格判定的 z 层带 (与走廊节点 z 范围匹配)
   * @param expand_r 目标膨胀半径 (m)
   * @param margin 距禁行格的安全余量 (m)
   */
  static void expandCorridorConstrained(
      ConvexCorridor2D& cor,
      const elevation_planner::ManifoldGraph& graph,
      double z_band,
      double expand_r,
      double margin)
  {
    const auto& A = cor.constraint.A;
    const auto& b = cor.constraint.b;
    const int M = static_cast<int>(A.rows());
    if (M == 0) return;

    // 走廊 bbox 外扩 expand_r 范围内的本层带禁行节点
    double min_x = 1e18, max_x = -1e18, min_y = 1e18, max_y = -1e18;
    for (const auto& v : cor.constraint.vertices)
    {
      min_x = std::min(min_x, static_cast<double>(v.x()));
      max_x = std::max(max_x, static_cast<double>(v.x()));
      min_y = std::min(min_y, static_cast<double>(v.y()));
      max_y = std::max(max_y, static_cast<double>(v.y()));
    }

    const double z_lo = cor.z_ref - z_band, z_hi = cor.z_ref + z_band;
    std::vector<Eigen::Vector2d> blocked;
    int r_lo = 0, c_lo = 0, r_hi = 0, c_hi = 0;
    if (graph.toGridIndex(min_x - expand_r, min_y - expand_r, r_lo, c_lo) &&
        graph.toGridIndex(max_x + expand_r, max_y + expand_r, r_hi, c_hi))
    {
      const int pad = 1;
      for (int r = std::max(0, r_lo - pad); r <= r_hi + pad; ++r)
      {
        for (int c = std::max(0, c_lo - pad); c <= c_hi + pad; ++c)
        {
          for (uint32_t nid : graph.getSpatialCellNodes(r, c))
          {
            const auto& nd = graph.getNode(nid);
            if (nd.traversability >= 0.95f && nd.z >= z_lo && nd.z <= z_hi)
              blocked.emplace_back(nd.x, nd.y);
          }
        }
      }
    }

    if (blocked.empty())
    {
      cor.constraint.b = b + Eigen::VectorXd::Constant(M, expand_r);  // 周边无禁行: 各面扩满 r
    }
    else
    {
      Eigen::VectorXd b_new = b;
      for (int j = 0; j < M; ++j)
      {
        double min_slack = 1e18;
        for (const auto& c : blocked)
        {
          const double slack = A.row(j).dot(c) - b(j);   // >0: 禁行格在本面外侧的余量
          if (slack > -1e-9 && slack < min_slack) min_slack = slack;
        }
        const double d = std::min(expand_r, min_slack) - margin;
        b_new(j) = b(j) + std::max(0.0, d);
      }
      cor.constraint.b = b_new;
    }

    // 顶点重建: 相邻半平面两两求交
    std::vector<Eigen::Vector2d> verts;
    verts.reserve(M);
    for (int j = 0; j < M; ++j)
    {
      const int k = (j + 1) % M;
      Eigen::Matrix2d Ab;
      Ab.row(0) = A.row(j);
      Ab.row(1) = A.row(k);
      const Eigen::Vector2d bb(cor.constraint.b(j), cor.constraint.b(k));
      Eigen::FullPivLU<Eigen::Matrix2d> lu(Ab);
      if (lu.rank() < 2) continue;  // 平行退化边, 跳过
      verts.push_back(lu.solve(bb));
    }
    if (verts.size() >= 3) cor.constraint.vertices = verts;
    // 求交退化 (<3 顶点): 保留原顶点 (仅约束外推生效, 可视化滞后无碍)
  }

};

} // namespace elevation_local_planner
