#include "elevation_costmap/manifold_costmap_builder.h"
#include <tf2/utils.h>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <queue>
#include <unordered_set>

#include "elevation_costmap/manifold_obstacle_extractor.h"
#include "elevation_planner_core/graph_store.hpp"
#include "elevation_planner_core/topological_corridor.hpp"

namespace elevation_costmap
{

ManifoldCostmapBuilder::ManifoldCostmapBuilder(const ManifoldCostmapBuilderConfig & config)
: config_(config)
{
}

bool ManifoldCostmapBuilder::buildCostmap(const elevation_planner::ManifoldGraph & graph,
                                         const geometry_msgs::Pose & robot_pose,
                                         const std::vector<geometry_msgs::PoseStamped> & global_plan,
                                         nav_msgs::OccupancyGrid & out_grid,
                                         geometry_msgs::TransformStamped & out_tf,
                                         std::vector<int8_t> * out_reasons,
                                         std::vector<int32_t> * out_node_ids,
                                         costmap_converter::ObstacleArrayMsg * out_obstacles,
                                         elevation_planner::LocalElevationGrid * out_elevation_grid) const
{
  if (graph.numNodes() == 0) return false;

  // -------------------------------------------------------------
  // 第一步: 提取机器狗脚下真实高程锚点 P0
  // -------------------------------------------------------------
  Eigen::Vector3d p0(robot_pose.position.x, robot_pose.position.y, robot_pose.position.z);
  uint32_t n0_id = 0;
  bool found_n0 = graph.findClosestNode(p0.x(), p0.y(), p0.z(), n0_id, 1.0, 1.0);
  if (found_n0)
  {
    const auto & n0 = graph.getNode(n0_id);
    p0.z() = n0.z; // 基准高度以机器人当前所踩踏面为准
  }

  // -------------------------------------------------------------
  // 第二步: 建立平行于全局 map 系的局部栅格几何 (彻底消除旋转畸变)
  // -------------------------------------------------------------
  const double res = config_.resolution > 0.01 ? config_.resolution : 0.05;
  const int cols = static_cast<int>(std::round(config_.map_width / res));
  const int rows = static_cast<int>(std::round(config_.map_length / res));

  out_grid.header.stamp = ros::Time::now();
  out_grid.header.frame_id = config_.map_frame;
  out_grid.info.resolution = res;
  out_grid.info.width = cols;
  out_grid.info.height = rows;

  // 以机器人为中心对称铺设局部代价地图
  const double origin_x = p0.x() - config_.map_width * 0.5;
  const double origin_y = p0.y() - config_.map_length * 0.5;
  const double origin_z = p0.z();

  out_grid.info.origin.position.x = origin_x;
  out_grid.info.origin.position.y = origin_y;
  out_grid.info.origin.position.z = origin_z;
  out_grid.info.origin.orientation.x = 0.0;
  out_grid.info.origin.orientation.y = 0.0;
  out_grid.info.origin.orientation.z = 0.0;
  out_grid.info.origin.orientation.w = 1.0; // 严格平行于 map 系

  if (out_elevation_grid)
  {
    *out_elevation_grid = elevation_planner::LocalElevationGrid(cols, rows, res, origin_x, origin_y, config_.map_frame, std::numeric_limits<float>::quiet_NaN());
  }

  // -------------------------------------------------------------
  // 第三步: 提取局部 A* 路径参考面 (遇 2D 自重叠处截断，只保留从狗开始的单层平面)
  // -------------------------------------------------------------
  std::vector<Eigen::Vector3d> local_waypoints;
  local_waypoints.reserve(30);
  {
    // 从离当前机器人位置最近的航点开始截取，避免机器人远离起点后取到身后历史航点
    size_t start_idx = 0;
    double min_d_robot = std::numeric_limits<double>::max();
    for (size_t i = 0; i < global_plan.size(); ++i)
    {
      double d = std::hypot(global_plan[i].pose.position.x - p0.x(),
                            global_plan[i].pose.position.y - p0.y());
      if (d < min_d_robot)
      {
        min_d_robot = d;
        start_idx = i;
      }
    }
    if (start_idx > 1) start_idx -= 1; // 包含前一个点保持平滑连续

    double w_acc = 0.0;
    for (size_t i = start_idx; i < global_plan.size(); ++i)
    {
      const auto & pos = global_plan[i].pose.position;
      Eigen::Vector3d curr_wp(pos.x, pos.y, pos.z);

      // 检测 2D 自身重叠: 航点在水平投影上与较早航点非常接近 (< 0.25m)，但高差明显 (> 0.40m)
      bool self_overlap = false;
      for (size_t j = 0; j < local_waypoints.size(); ++j)
      {
        double dxy2 = std::pow(curr_wp.x() - local_waypoints[j].x(), 2) +
                      std::pow(curr_wp.y() - local_waypoints[j].y(), 2);
        if (dxy2 < 0.25 * 0.25 && std::abs(curr_wp.z() - local_waypoints[j].z()) > 0.40)
        {
          self_overlap = true;
          break;
        }
      }
      if (self_overlap)
      {
        break; // 截断重叠后的远端航点，确保局部代价地图在视野内永远是单层不重叠平面
      }

      local_waypoints.push_back(curr_wp);
      if (i + 1 < global_plan.size())
      {
        w_acc += std::hypot(global_plan[i+1].pose.position.x - pos.x, global_plan[i+1].pose.position.y - pos.y);
        if (w_acc > 4.5) break;
      }
    }
  }

  // 计算任意点 (x, y) 到局部 A* 路径的最短水平距离
  auto distToLocalPlan = [&](double x, double y) -> double {
    if (local_waypoints.empty()) return 0.0;
    double min_d2 = std::numeric_limits<double>::max();
    for (size_t i = 0; i < local_waypoints.size(); ++i)
    {
      if (i + 1 < local_waypoints.size())
      {
        const auto & p1 = local_waypoints[i];
        const auto & p2 = local_waypoints[i + 1];
        double dx = p2.x() - p1.x();
        double dy = p2.y() - p1.y();
        double len2 = dx * dx + dy * dy;
        if (len2 < 1e-6)
        {
          double d2 = std::pow(x - p1.x(), 2) + std::pow(y - p1.y(), 2);
          if (d2 < min_d2) min_d2 = d2;
        }
        else
        {
          double t = std::max(0.0, std::min(1.0, ((x - p1.x()) * dx + (y - p1.y()) * dy) / len2));
          double proj_x = p1.x() + t * dx;
          double proj_y = p1.y() + t * dy;
          double d2 = std::pow(x - proj_x, 2) + std::pow(y - proj_y, 2);
          if (d2 < min_d2) min_d2 = d2;
        }
      }
      else
      {
        double d2 = std::pow(x - local_waypoints[i].x(), 2) + std::pow(y - local_waypoints[i].y(), 2);
        if (d2 < min_d2) min_d2 = d2;
      }
    }
    return std::sqrt(min_d2);
  };

  // 局部航点参考高程场: 每个栅格以对应局部航点高度为参考深度基准
  std::vector<float> ref_z(rows * cols, static_cast<float>(p0.z()));
  if (!local_waypoints.empty())
  {
    for (int r = 0; r < rows; ++r)
    {
      for (int c = 0; c < cols; ++c)
      {
        double cell_wx = origin_x + (c + 0.5) * res;
        double cell_wy = origin_y + (r + 0.5) * res;
        double min_dxy = 999.0;
        double best_wz = p0.z();
        for (const auto & wp : local_waypoints)
        {
          double dxy = std::hypot(wp.x() - cell_wx, wp.y() - cell_wy);
          if (dxy < min_dxy)
          {
            min_dxy = dxy;
            best_wz = wp.z();
          }
        }
        if (min_dxy <= std::max(1.2, config_.corridor_radius + 0.3))
        {
          ref_z[static_cast<size_t>(r * cols + c)] = static_cast<float>(best_wz);
        }
        else
        {
          ref_z[static_cast<size_t>(r * cols + c)] = std::numeric_limits<float>::quiet_NaN();
        }
      }
    }
  }

  // 获取全局拓扑管道 (由 A* 规划生成并基于连通图向外膨胀至 corridor_radius)
  auto global_corridor = elevation_planner::GraphStore::instance().getTopologicalCorridor();
  const double max_lateral_dist = config_.corridor_radius > 0.1 ? config_.corridor_radius : 3.0;

  // -------------------------------------------------------------
  // 第四步: BFS 仅扩展【可通行 + 单步联通】节点 (多次扩展直到 costmap 边界)
  // -------------------------------------------------------------
  auto isPassableNode = [&](const elevation_planner::GraphNode & nd) -> bool {
    return (nd.traversability < 0.8f &&
            (nd.headroom <= 0.0f || nd.headroom >= config_.dog_height));
  };

  std::vector<uint32_t> candidate_nodes;
  candidate_nodes.reserve(2000);
  std::vector<char> is_candidate(graph.numNodes(), 0);

  const double win_x_min = origin_x - 0.3;
  const double win_x_max = origin_x + config_.map_width + 0.3;
  const double win_y_min = origin_y - 0.3;
  const double win_y_max = origin_y + config_.map_length + 0.3;

  const float max_step_dz = 0.25f;          // 单步台阶高差上限 (物理跨步极限)
  const double max_step_dxy2 = 0.35 * 0.35; // 单步平面相邻距离上限 (物理跨步极限)

  if (graph.numEdges() > 0)
  {
    std::vector<bool> visited(graph.numNodes(), false);
    std::queue<uint32_t> q;

    // 种子 1: 机器人脚下起始节点 N0 (若可通行)
    if (found_n0 && isPassableNode(graph.getNode(n0_id)))
    {
      q.push(n0_id);
      visited[n0_id] = true;
    }

    // 种子 2: A* 局部航点上的对应可通行节点 (确保局部规划走廊锚定在 A* 面层)
    for (const auto & wp : local_waypoints)
    {
      uint32_t wp_nid = 0;
      if (graph.findClosestNode(wp.x(), wp.y(), wp.z(), wp_nid, 0.35, 0.35))
      {
        if (!visited[wp_nid] && isPassableNode(graph.getNode(wp_nid)))
        {
          q.push(wp_nid);
          visited[wp_nid] = true;
        }
      }
    }

    // BFS 广度优先循环扩展: 像水流一样在当前物理踏面上漫延，直至边界/墙体/断崖
    while (!q.empty())
    {
      uint32_t curr = q.front();
      q.pop();
      const auto & curr_nd = graph.getNode(curr);

      if (curr_nd.x >= win_x_min && curr_nd.x <= win_x_max &&
          curr_nd.y >= win_y_min && curr_nd.y <= win_y_max)
      {
        candidate_nodes.push_back(curr);
        is_candidate[curr] = 1;
      }

      uint16_t edge_count = 0;
      const auto * edges = graph.getEdges(curr, edge_count);
      for (uint16_t e = 0; e < edge_count; ++e)
      {
        uint32_t nbr = edges[e].target_id;
        if (visited[nbr]) continue;

        const auto & nbr_nd = graph.getNode(nbr);

        // 1. 代价地图窗口边界过滤 (超出 costmap 窗口停止向外扩展)
        if (nbr_nd.x < win_x_min || nbr_nd.x > win_x_max ||
            nbr_nd.y < win_y_min || nbr_nd.y > win_y_max)
          continue;

        // 2. 只扩展【可通行】节点 (彻底阻断不可通行死角、墙体、低净空入队)
        if (!isPassableNode(nbr_nd))
          continue;

        // 3. x, y 相邻判定 (单步平面跨步 <= 0.35m)
        double dxy2 = std::pow(nbr_nd.x - curr_nd.x, 2) + std::pow(nbr_nd.y - curr_nd.y, 2);
        if (dxy2 > max_step_dxy2) continue;

        // 4. 单步物理连通高差判定 (|dz| <= 0.25m)
        float s_dz = std::abs(nbr_nd.z - curr_nd.z);
        if (s_dz > max_step_dz) continue;

        // 5. 与局部参考高程场一致性判定 (防止顺着坡道一路蔓延到多层楼上或楼下)
        int c_idx = static_cast<int>(std::floor((nbr_nd.x - origin_x) / res));
        int r_idx = static_cast<int>(std::floor((nbr_nd.y - origin_y) / res));
        if (r_idx >= 0 && r_idx < rows && c_idx >= 0 && c_idx < cols)
        {
          float target_ref_z = ref_z[static_cast<size_t>(r_idx * cols + c_idx)];
          if (std::isnan(target_ref_z) || std::abs(nbr_nd.z - target_ref_z) > 0.50f)
            continue; // 偏离 A* 所在的面层或超出走廊，停止蔓延
        }

        // 6. 拓扑流形管道范围过滤 (优先采用基于连通图生成的全局拓扑管道，遇悬空/断崖/不可达自然阻断)
        if (global_corridor && !global_corridor->empty())
        {
          if (!global_corridor->isInCorridor(nbr))
            continue;
        }
        else if (!local_waypoints.empty())
        {
          if (distToLocalPlan(nbr_nd.x, nbr_nd.y) > max_lateral_dist)
            continue;
        }

        visited[nbr] = true;
        q.push(nbr);
      }
    }
  }

  // 兜底策略: 若图无拓扑边(如轻量测试)或未锁定 N0, 则收集窗口内同层可通行几何节点
  if (candidate_nodes.empty())
  {
    for (size_t i = 0; i < graph.numNodes(); ++i)
    {
      const auto & nd = graph.getNode(i);
      if (nd.x >= win_x_min && nd.x <= win_x_max &&
          nd.y >= win_y_min && nd.y <= win_y_max &&
          isPassableNode(nd) &&
          std::abs(nd.z - static_cast<float>(p0.z())) <= 0.50f)
      {
        if (global_corridor && !global_corridor->empty())
        {
          if (!global_corridor->isInCorridor(static_cast<uint32_t>(i)))
            continue;
        }
        else if (!local_waypoints.empty() && distToLocalPlan(nd.x, nd.y) > max_lateral_dist)
        {
          continue;
        }
        is_candidate[static_cast<size_t>(i)] = 1;
        candidate_nodes.push_back(static_cast<uint32_t>(i));
      }
    }
  }

  // -------------------------------------------------------------
  // 第五步: 2D 栅格单层盖章 (只盖章可通行节点，其余位置天然保持 100 障碍)
  // -------------------------------------------------------------
  // 初始地图默认全为 100 (悬崖/墙体等不可通行区天然保持 100)
  out_grid.data.assign(rows * cols, 100);
  if (out_reasons) out_reasons->assign(rows * cols, REASON_NO_NODE);
  if (out_node_ids) out_node_ids->assign(rows * cols, -1);

  // 深度缓冲区: 记录每个 2D 栅格胜出节点距离局部参考高度的绝对高差 |z - z_ref|
  std::vector<float> min_dz(rows * cols, std::numeric_limits<float>::max());
  // 水平距离缓冲区: 同层节点裁决时，离节点中心更近的胜出 (泰森多边形自然边界)
  std::vector<float> min_d2(rows * cols, std::numeric_limits<float>::max());

  // 盖章半径: 确保节点间无孔洞缝隙铺满踏面
  const double stamp_radius = std::max(0.08, graph.getResolution() * 0.75);
  const int stamp_cells = std::max(1, static_cast<int>(std::ceil(stamp_radius / res)));
  const double stamp_r2 = stamp_radius * stamp_radius;

  for (uint32_t nid : candidate_nodes)
  {
    const auto & nd = graph.getNode(nid);

    int center_c = static_cast<int>(std::floor((nd.x - origin_x) / res));
    int center_r = static_cast<int>(std::floor((nd.y - origin_y) / res));

    int8_t cell_cost = static_cast<int8_t>(nd.traversability * 70.0f);
    int8_t node_reason = (nd.traversability > 0.05f) ? REASON_SOFT_NODE : REASON_FREE_NODE;

    for (int dr = -stamp_cells; dr <= stamp_cells; ++dr)
    {
      int nr = center_r + dr;
      if (nr < 0 || nr >= rows) continue;

      for (int dc = -stamp_cells; dc <= stamp_cells; ++dc)
      {
        int nc = center_c + dc;
        if (nc < 0 || nc >= cols) continue;

        double cell_wx = origin_x + (nc + 0.5) * res;
        double cell_wy = origin_y + (nr + 0.5) * res;
        double d2 = std::pow(cell_wx - nd.x, 2) + std::pow(cell_wy - nd.y, 2);
        if (d2 > stamp_r2) continue;

        size_t idx = static_cast<size_t>(nr * cols + nc);
        float dz = std::abs(nd.z - ref_z[idx]);

        // 深度测试：
        // 1. 明显更靠近参考面层 (差距 > 0.05m) -> 胜出
        // 2. 属于同一高度层 (|dz - min_dz| <= 0.05m) -> 水平距离离节点中心更近者胜出 (泰森多边形原则)
        bool update = false;
        if (dz < min_dz[idx] - 0.05f)
        {
          update = true;
        }
        else if (std::abs(dz - min_dz[idx]) <= 0.05f)
        {
          if (static_cast<float>(d2) < min_d2[idx])
          {
            update = true;
          }
        }

        if (update)
        {
          min_dz[idx] = dz;
          min_d2[idx] = static_cast<float>(d2);
          out_grid.data[idx] = cell_cost;
          if (out_reasons) (*out_reasons)[idx] = node_reason;
          if (out_node_ids) (*out_node_ids)[idx] = static_cast<int32_t>(nid);
          if (out_elevation_grid) out_elevation_grid->setZ(nr, nc, nd.z);
        }
      }
    }
  }

  // -------------------------------------------------------------
  // 第四步半: 拓扑缝绘制 —— 图上无边衔接的相邻踏面, 在交界处画出致命分隔
  // -------------------------------------------------------------
  // 地毯是 2D 投影: 相邻两格各自盖到可通行代价后, 投影上连成一片 —— 即使图上
  // 两格之间不存在任何边 (高差超限的错层/断坎, 或扫掠判定不可直穿)。TEB 据此
  // 会规划穿越图上禁止的边界。判据完全由连通图决定: 相邻两图格各自存在可通行
  // 候选节点, 且任意跨格节点对之间都没有边 -> 交界处画致命条带 (宽约一个地毯格)。
  // 注意配对必须覆盖两格的全部可通行节点而非仅深度测试保留层: 保留层是逐格
  // 独立做出的局部选择, 陡楼梯/错层处相邻格保留的可能是不相接的两级踏面,
  // 而其它层之间恰有真实可走的边。桥接长边不受影响: 桥途经格必有高度带内
  // 支撑节点且与两端有边, 该判据不会在桥的走廊上画缝。
  // -------------------------------------------------------------
  // 第四步半: 拓扑断层/断缝检测 —— 相邻地毯格若无图边直接连通, 插入致命障碍
  // -------------------------------------------------------------
  // 判据完全以地毯上实际胜出的盖章节点为准:
  // 遍历 2D 地毯上相邻的可通行格 (4 邻域), 取其盖章节点 u 与 v。
  // 若 u != v, 且在流形图中不存在 u <-> v 的直接连通边, 则判定两格间存在物理断层
  // (如楼梯边缘落入下层地面、未连通错层), 立即在偏离机器人高度的一侧 (或两侧)
  // 插入致命障碍 (Cost=100, REASON_SEAM), 彻底封死异层渗漏路径。
  if (!candidate_nodes.empty() && out_node_ids)
  {
    std::unordered_set<uint64_t> edge_keys;
    edge_keys.reserve(candidate_nodes.size() * 4);
    for (uint32_t nid : candidate_nodes)
    {
      uint16_t ec = 0;
      const auto * es = graph.getEdges(nid, ec);
      for (uint16_t k = 0; k < ec; ++k)
      {
        const uint32_t a = std::min(nid, es[k].target_id);
        const uint32_t b = std::max(nid, es[k].target_id);
        edge_keys.insert((static_cast<uint64_t>(a) << 32) | static_cast<uint64_t>(b));
      }
    }

    std::vector<size_t> seam_cells;
    for (int r = 0; r < rows; ++r)
    {
      for (int c = 0; c < cols; ++c)
      {
        const size_t idx = static_cast<size_t>(r * cols + c);
        if (out_grid.data[idx] >= 100) continue;
        int32_t u = (*out_node_ids)[idx];
        if (u < 0) continue;

        // 检查 4 邻域 (+X 和 +Y)
        const int nbs[2][2] = {{r + 1, c}, {r, c + 1}};
        for (int k = 0; k < 2; ++k)
        {
          int nr = nbs[k][0], nc = nbs[k][1];
          if (nr >= rows || nc >= cols) continue;
          const size_t nidx = static_cast<size_t>(nr * cols + nc);
          if (out_grid.data[nidx] >= 100) continue;
          int32_t v = (*out_node_ids)[nidx];
          if (v < 0 || u == v) continue;

          const uint64_t key = (static_cast<uint64_t>(std::min(static_cast<uint32_t>(u), static_cast<uint32_t>(v))) << 32) |
                               static_cast<uint64_t>(std::max(static_cast<uint32_t>(u), static_cast<uint32_t>(v)));
          if (edge_keys.count(key) != 0) continue; // 1. 直接边或跨步建桥边已连通

          // 2. 跨步建桥一致性连通判定:
          // 若两节点在四足物理跨步极限内 (水平距离 <= 0.35m, 台阶高差 <= 0.25m),
          // 且在图上通过短边链在小跳数内可达 (建图时因已连通而按规则未建冗余直连长边),
          // 则视为同一连续踏面, 不插入断缝障碍
          const auto & nu = graph.getNode(static_cast<uint32_t>(u));
          const auto & nv = graph.getNode(static_cast<uint32_t>(v));
          float dxy = std::hypot(nu.x - nv.x, nu.y - nv.y);
          float dz = std::abs(nu.z - nv.z);
          if (dxy <= 0.35f && dz <= 0.25f && graph.isConnected(static_cast<uint32_t>(u), static_cast<uint32_t>(v), 4))
          {
            continue; // 跨步与短边链连通, 视为连通
          }

          // 3. 确系物理断层/断坎 (如楼梯边缘悬崖直落底层):
          // 路径踏面绝对保护法则: 凡是落在 A* 全局路径走廊内的合法踏面，绝不标记为断缝
          float dzu = std::abs(nu.z - ref_z[idx]);
          float dzv = std::abs(nv.z - ref_z[nidx]);

          if (!local_waypoints.empty())
          {
            bool u_on_path = (distToLocalPlan(nu.x, nu.y) <= 0.20 && dzu <= 0.20f);
            bool v_on_path = (distToLocalPlan(nv.x, nv.y) <= 0.20 && dzv <= 0.20f);

            if (u_on_path && v_on_path)
            {
              // 两侧同属于规划路径走廊 (如双折转角前后段)，均受保护，不自相残杀涂黑踏面
              continue;
            }
            else if (u_on_path)
            {
              // u 在规划路径上，断缝只能落在非路径一侧的 v (如中间缝隙/外侧)
              seam_cells.push_back(nidx);
            }
            else if (v_on_path)
            {
              // v 在规划路径上，断缝只能落在非路径一侧的 u
              seam_cells.push_back(idx);
            }
            else
            {
              // 两者均不在核心规划路径上，将明显偏离期望参考高程的一侧标记为断缝
              if (dzv > dzu + 0.05f) seam_cells.push_back(nidx);
              else if (dzu > dzv + 0.05f) seam_cells.push_back(idx);
              else seam_cells.push_back(dzv >= dzu ? nidx : idx);
            }
          }
          else
          {
            // 无全局路径模式 (如离线单元测试): 偏离机器人高度的一侧插入致命障碍
            if (dzv >= dzu) seam_cells.push_back(nidx);
            if (dzu >= dzv) seam_cells.push_back(idx);
          }
        }
      }
    }

    for (size_t sidx : seam_cells)
    {
      out_grid.data[sidx] = 100;
      if (out_reasons) (*out_reasons)[sidx] = REASON_SEAM;
      if (out_node_ids) (*out_node_ids)[sidx] = -1;
    }
  }

  // -------------------------------------------------------------
  // 第四步 3/4: 形态学闭运算 —— 填充孤立致命补丁 (图节点缺失伪影)
  // -------------------------------------------------------------
  // 盖章半径 (0.08m) 小于图节点间距 (0.10m): 单个图节点缺失会在连续踏面上
  // 留下 1~2 格的孤立致命补丁。全局 A* 的跨步架桥会跨过该缺格 (路径照常穿过),
  // 而 TEB 眼中这是致命雷区 —— 层间表示不一致, 机器人会在踏面中央被卡死。
  // 真实空洞 (楼梯井/断崖) 是大片连片致命区; 判据: 任一致命格 8 邻域中
  // >= 7 格非致命即视为孤立补丁, 填为邻域最恶劣的可通行代价。
  // 单趟读原图写副本, 不级联传播 (迭代闭运算会侵蚀真实空洞的边缘)。
  {
    std::vector<int8_t> filled = out_grid.data;
    for (int r = 0; r < rows; ++r)
    {
      for (int c = 0; c < cols; ++c)
      {
        const size_t idx = static_cast<size_t>(r * cols + c);
        if (out_grid.data[idx] != 100) continue;
        if (out_reasons && (*out_reasons)[idx] == REASON_SEAM) continue; // 绝不填塞拓扑缝

        int lethal_nb = 0;
        int8_t fill_cost = 0;
        for (int dr = -1; dr <= 1; ++dr)
        {
          for (int dc = -1; dc <= 1; ++dc)
          {
            if (dr == 0 && dc == 0) continue;
            const int nr = r + dr;
            const int nc = c + dc;
            if (nr < 0 || nr >= rows || nc < 0 || nc >= cols)
            {
              ++lethal_nb; // 地图边界外保守视为致命
              continue;
            }
            const int8_t nb = out_grid.data[static_cast<size_t>(nr * cols + nc)];
            if (nb >= 100) ++lethal_nb;
            else fill_cost = std::max(fill_cost, nb);
          }
        }
        if (lethal_nb <= 1)
        {
          filled[idx] = fill_cost; // >= 7/8 邻域可通行 -> 伪影, 填充
          if (out_reasons) (*out_reasons)[idx] = REASON_CLOSING_FILLED;
        }
      }
    }
    out_grid.data.swap(filled);
  }

  // -------------------------------------------------------------
  // 第四步 7/8: 地毯连通性验证 —— 只保留与机器人所在格连通的自由区
  // -------------------------------------------------------------
  // 盖章候选是 "图上与机器人联通" 的节点, 但图联通 ≠ 一步可跨: 楼梯下方的主层
  // 地面、悬空楼梯投影等异层节点, 会把它们的自由透印到画在机器人当前高度的地毯
  // 上。判据 (地毯自验证): 从机器人所在格出发, 沿自由/软代价格 4 邻域泛洪;
  // 泛洪不可达的自由格 = 绕道图外到达的透印, 转为致命 (成因码 REASON_UNREACHED,
  // 保留盖章节点 id 供诊断)。泛洪在缝与闭运算之后运行: 缝阻断异层渗漏, 闭运算
  // 先填平孤立伪影, 泛洪只对最终形态做连通性裁决。
  {
    const int seed_r = std::max(0, std::min(rows - 1,
        static_cast<int>(std::floor((p0.y() - origin_y) / res))));
    const int seed_c = std::max(0, std::min(cols - 1,
        static_cast<int>(std::floor((p0.x() - origin_x) / res))));

    // 种子: 机器人所在格应为自由; 若被遮蔽则在 5x5 邻域内找代价最低的自由格
    int start = -1;
    {
      size_t sidx = static_cast<size_t>(seed_r * cols + seed_c);
      if (out_grid.data[sidx] < 100) start = static_cast<int>(sidx);
    }
    if (start < 0)
    {
      float best = 100;
      for (int dr = -2; dr <= 2 && start < 0; ++dr)
        for (int dc = -2; dc <= 2 && start < 0; ++dc)
        {
          const int nr = seed_r + dr, nc = seed_c + dc;
          if (nr < 0 || nr >= rows || nc < 0 || nc >= cols) continue;
          const int8_t v = out_grid.data[static_cast<size_t>(nr * cols + nc)];
          if (v < 100 && v < best) { best = v; start = nr * cols + nc; }
        }
    }

    if (start >= 0)
    {
      std::vector<uint8_t> visited(rows * cols, 0);
      std::queue<int> q;
      q.push(start);
      visited[static_cast<size_t>(start)] = 1;
      while (!q.empty())
      {
        const int cur = q.front(); q.pop();
        const int cr = cur / cols, cc = cur % cols;
        const int nb[4][2] = {{cr - 1, cc}, {cr + 1, cc}, {cr, cc - 1}, {cr, cc + 1}};
        for (int k = 0; k < 4; ++k)
        {
          const int nr = nb[k][0], nc = nb[k][1];
          if (nr < 0 || nr >= rows || nc < 0 || nc >= cols) continue;
          const size_t nidx = static_cast<size_t>(nr * cols + nc);
          if (visited[nidx] || out_grid.data[nidx] >= 100) continue;
          visited[nidx] = 1;
          q.push(static_cast<int>(nidx));
        }
      }
      for (size_t i = 0; i < out_grid.data.size(); ++i)
      {
        if (out_grid.data[i] < 100 && !visited[i])
        {
          out_grid.data[i] = 100;
          if (out_reasons) (*out_reasons)[i] = REASON_UNREACHED;
          if (out_node_ids) (*out_node_ids)[i] = -1;
          if (out_elevation_grid) {
            int r = static_cast<int>(i / cols);
            int c = static_cast<int>(i % cols);
            out_elevation_grid->setZ(r, c, std::numeric_limits<float>::quiet_NaN());
          }
        }
      }
    }
  }

  // -------------------------------------------------------------
  // 第五步: 发布与局部坐标系对齐 TF
  // -------------------------------------------------------------
  out_tf.header.stamp = out_grid.header.stamp;
  out_tf.header.frame_id = config_.map_frame;
  out_tf.child_frame_id = config_.output_frame;
  out_tf.transform.translation.x = p0.x();
  out_tf.transform.translation.y = p0.y();
  out_tf.transform.translation.z = p0.z();
  out_tf.transform.rotation = out_grid.info.origin.orientation;

  // -------------------------------------------------------------
  // 第六步: 提取稀疏 3D 几何障碍物与悬空边界 (供给 TEB 原生同伦规划)
  // -------------------------------------------------------------
  if (out_obstacles)
  {
    ObstacleExtractorConfig ext_cfg;
    ext_cfg.window_radius = std::max(config_.map_width, config_.map_length) * 0.5;
    ext_cfg.dog_height = config_.dog_height;
    ext_cfg.height_tolerance = config_.height_tolerance;
    ext_cfg.map_frame = config_.map_frame;
    ManifoldObstacleExtractor extractor(ext_cfg);
    extractor.extractObstacles(graph, robot_pose, candidate_nodes, is_candidate, *out_obstacles);
  }

  return true;
}

} // namespace elevation_costmap
