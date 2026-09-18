#pragma once

#include "elevation_planner_core/manifold_graph.hpp"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include <queue>
#include <cmath>
#include <algorithm>
#include <limits>
#include <cstdint>
#include <functional>

namespace elevation_planner
{

/**
 * @brief 拓扑流形管道：沿 A* 规划路径基于流形连通图拓扑展开的三维安全走廊
 * 
 * 核心特性：
 * 1. 仅保留拓扑连通曲面，严格受限于四足机器人运动学跨越极限 (|dz| <= max_step_height) 与头顶净空；
 * 2. 遇到悬空、断崖、不可达墙体自然截断收敛，在开阔区域以设定半径 (如 3.0m) 为边界；
 * 3. 内部动态剔除局部障碍物与禁行区，结合局部前瞻视野实时更新；
 * 4. TEB 规划器在此管道内部进行弹性避障与同伦优化。
 */
struct TopologicalCorridor
{
  std::vector<uint32_t> node_ids;      ///< 管道内包含的所有可通行流形节点 ID
  std::vector<uint8_t> in_corridor;    ///< 节点包含快查表 (按 node_id 索引，1 为在管道内，0 为管道外)
  double radius{3.0};                  ///< 管道最大展开半宽 (m)
  double lookahead{6.0};               ///< 管道前瞻长度 (m)
  uint64_t version{0};                 ///< 管道更新版本号

  inline bool isInCorridor(uint32_t node_id) const
  {
    if (node_id < in_corridor.size())
    {
      return in_corridor[node_id] != 0;
    }
    return false;
  }

  inline bool empty() const { return node_ids.empty(); }
  inline size_t size() const { return node_ids.size(); }
};

/**
 * @brief 拓扑流形管道生成器
 */
class TopologicalCorridorGenerator
{
public:
  /**
   * @brief 根据 A* 规划路径与流形拓扑连通图生成拓扑管道
   * 
   * @param graph 全局或局部流形拓扑图
   * @param path 参考路径 (geometry_msgs::PoseStamped 序列)
   * @param corridor_radius 管道向外扩展的最大半宽 (m)，默认 3.0m (遇悬空/断崖/不可达位置自然停止)
   * @param lookahead_dist 管道沿路径前瞻长度上限 (m)，默认 6.0m (<=0 表示不截断)
   * @param max_step_height 单步垂直高差跨越极限 (m)，默认 0.25m
   * @param dog_height 机器狗站立净空要求 (m)，默认 0.45m
   * @param is_blocked_fn 动态/静态障碍物阻挡判据回调，返回 true 表示该坐标处存在障碍物，管道不可进入
   * @return TopologicalCorridor 生成的拓扑管道
   */
  static TopologicalCorridor generate(
      const ManifoldGraph & graph,
      const std::vector<geometry_msgs::PoseStamped> & path,
      double corridor_radius = 3.0,
      double lookahead_dist = 6.0,
      double max_step_height = 0.25,
      double dog_height = 0.45,
      const std::function<bool(float, float, float)> & is_blocked_fn = nullptr)
  {
    TopologicalCorridor corridor;
    corridor.radius = corridor_radius;
    corridor.lookahead = lookahead_dist;

    if (graph.numNodes() == 0 || path.empty())
    {
      return corridor;
    }

    // 1. 沿路径截取前瞻区间 (若指定 lookahead_dist > 0)
    std::vector<geometry_msgs::PoseStamped> lookahead_path;
    lookahead_path.reserve(path.size());
    double acc_dist = 0.0;
    for (size_t i = 0; i < path.size(); ++i)
    {
      if (i > 0)
      {
        double d = std::hypot(path[i].pose.position.x - path[i - 1].pose.position.x,
                              path[i].pose.position.y - path[i - 1].pose.position.y);
        acc_dist += d;
        if (lookahead_dist > 0.0 && acc_dist > lookahead_dist)
        {
          break;
        }
      }
      lookahead_path.push_back(path[i]);
    }
    if (lookahead_path.empty())
    {
      lookahead_path.push_back(path.front());
    }

    const size_t num_nodes = graph.numNodes();
    std::vector<float> dist(num_nodes, std::numeric_limits<float>::max());
    corridor.in_corridor.assign(num_nodes, 0);

    // 小顶堆: (当前到 A* 路径的最短流形测地距离, node_id)
    using PQEntry = std::pair<float, uint32_t>;
    std::priority_queue<PQEntry, std::vector<PQEntry>, std::greater<PQEntry>> pq;

    // 2. 种子提取：将局部前瞻路径上所有航点匹配到的可通行踏面节点作为初始种子
    for (const auto & pose_stamped : lookahead_path)
    {
      const auto & pt = pose_stamped.pose.position;
      uint32_t nid = 0;
      if (graph.findClosestNode(pt.x, pt.y, pt.z, nid, 0.40, 0.50))
      {
        const auto & nd = graph.getNode(nid);
        if (nd.traversability < 0.8f && (nd.headroom <= 0.0f || nd.headroom >= dog_height))
        {
          if (!(nd.flags & node_flags::BLOCK_HEADROOM))
          {
            // 若种子点被外部实体障碍物占据则跳过
            if (!is_blocked_fn || !is_blocked_fn(nd.x, nd.y, nd.z))
            {
              if (dist[nid] > 0.0f)
              {
                dist[nid] = 0.0f;
                pq.push({0.0f, nid});
              }
            }
          }
        }
      }
    }

    // 若前瞻段内未直接搜到种子节点，尝试吸附前瞻段第一点
    if (pq.empty() && !lookahead_path.empty())
    {
      uint32_t nid = 0;
      if (graph.findClosestNode(lookahead_path.front().pose.position.x,
                                lookahead_path.front().pose.position.y,
                                lookahead_path.front().pose.position.z,
                                nid, 1.0, 1.0))
      {
        const auto & nd = graph.getNode(nid);
        if (!is_blocked_fn || !is_blocked_fn(nd.x, nd.y, nd.z))
        {
          dist[nid] = 0.0f;
          pq.push({0.0f, nid});
        }
      }
    }

    if (pq.empty())
    {
      return corridor;
    }

    // 3. 多源拓扑测地膨胀 (Multi-Source Geodesic Dijkstra on ManifoldGraph)
    // 沿着拓扑边漫延，遇到悬空(无边)、垂直高差超限(|dz| > max_step_height)、低净空阻挡、以及实体障碍物截断停止
    const float max_dz = static_cast<float>(max_step_height + 0.05);

    while (!pq.empty())
    {
      const auto top_entry = pq.top();
      float d_curr = top_entry.first;
      uint32_t u = top_entry.second;
      pq.pop();

      if (d_curr > dist[u]) continue;
      if (d_curr > corridor_radius) break; // 优先队列单调递增，超出最大半宽即终止

      const auto & u_nd = graph.getNode(u);

      uint16_t edge_count = 0;
      const auto * edges = graph.getEdges(u, edge_count);
      for (uint16_t i = 0; i < edge_count; ++i)
      {
        uint32_t v = edges[i].target_id;
        const auto & v_nd = graph.getNode(v);

        // 3.1 通行性检查: 过滤顶头净空不足与致命硬阻挡 (不可达位置停止)
        if (v_nd.traversability >= 0.8f) continue;
        if (v_nd.headroom > 0.0f && v_nd.headroom < dog_height) continue;
        if (v_nd.flags & node_flags::BLOCK_HEADROOM) continue;

        // 3.2 动态/静态障碍物阻挡拦截 (遇障碍物截断，不进入障碍物区域向外扩展)
        if (is_blocked_fn && is_blocked_fn(v_nd.x, v_nd.y, v_nd.z)) continue;

        // 3.3 运动学单步跨越极限检验 (悬崖/断坎处无边或高差超限停止)
        float dz = std::abs(v_nd.z - u_nd.z);
        if (dz > max_dz) continue;

        // 3.4 沿流形曲面累积步进测地距离
        float dxy = std::hypot(v_nd.x - u_nd.x, v_nd.y - u_nd.y);
        float d_next = d_curr + dxy;

        if (d_next <= corridor_radius && d_next < dist[v])
        {
          dist[v] = d_next;
          pq.push({d_next, v});
        }
      }
    }

    // 3. 结算管道内节点集合
    corridor.node_ids.reserve(num_nodes / 4);
    for (size_t i = 0; i < num_nodes; ++i)
    {
      if (dist[i] <= corridor_radius)
      {
        corridor.node_ids.push_back(static_cast<uint32_t>(i));
        corridor.in_corridor[i] = 1;
      }
    }

    return corridor;
  }

  /**
   * @brief 将拓扑管道节点转换为 PointCloud2 消息
   */
  static void toPointCloudMsg(
      const ManifoldGraph & graph,
      const TopologicalCorridor & corridor,
      const std::string & frame_id,
      sensor_msgs::PointCloud2 & out_msg)
  {
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    cloud.reserve(corridor.size());
    for (uint32_t nid : corridor.node_ids)
    {
      if (nid >= graph.numNodes()) continue;
      const auto & nd = graph.getNode(nid);
      pcl::PointXYZRGB pt;
      pt.x = nd.x;
      pt.y = nd.y;
      pt.z = nd.z;
      pt.r = 40;
      pt.g = 210;
      pt.b = 160;
      cloud.push_back(pt);
    }
    pcl::toROSMsg(cloud, out_msg);
    out_msg.header.frame_id = frame_id;
    out_msg.header.stamp = ros::Time::now();
  }

  /**
   * @brief 基于流形图三维拓扑结构精确提取管道物理边界线段与立体防护护栏
   */
  static void toBoundaryMarkers(
      const ManifoldGraph & graph,
      const TopologicalCorridor & corridor,
      const std::string & frame_id,
      visualization_msgs::MarkerArray & out_markers,
      double wall_height = 0.45,
      double max_step_height = 0.25)
  {
    out_markers.markers.clear();
    if (corridor.empty() || graph.numNodes() == 0) return;

    visualization_msgs::Marker line_marker;
    line_marker.header.frame_id = frame_id;
    line_marker.header.stamp = ros::Time::now();
    line_marker.ns = "corridor_boundaries";
    line_marker.id = 0;
    line_marker.type = visualization_msgs::Marker::LINE_LIST;
    line_marker.action = visualization_msgs::Marker::ADD;
    line_marker.scale.x = 0.02; // 2cm 线宽
    line_marker.color.r = 0.0f;
    line_marker.color.g = 0.9f;
    line_marker.color.b = 1.0f;
    line_marker.color.a = 0.85f;

    visualization_msgs::Marker wall_marker;
    wall_marker.header.frame_id = frame_id;
    wall_marker.header.stamp = line_marker.header.stamp;
    wall_marker.ns = "corridor_walls";
    wall_marker.id = 1;
    wall_marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
    wall_marker.action = visualization_msgs::Marker::ADD;
    wall_marker.scale.x = 1.0;
    wall_marker.scale.y = 1.0;
    wall_marker.scale.z = 1.0;
    wall_marker.color.r = 0.0f;
    wall_marker.color.g = 0.7f;
    wall_marker.color.b = 1.0f;
    wall_marker.color.a = 0.18f;

    const double res = graph.getResolution();
    const double hres = res * 0.5;
    const float max_dz = static_cast<float>(max_step_height + 0.05);

    struct DirInfo {
      int dr;
      int dc;
      double dx1, dy1;
      double dx2, dy2;
    };
    const DirInfo dirs[4] = {
      { 1,  0,  hres, -hres,  hres,  hres},
      {-1,  0, -hres,  hres, -hres, -hres},
      { 0,  1,  hres,  hres, -hres,  hres},
      { 0, -1, -hres, -hres,  hres, -hres}
    };

    for (uint32_t uid : corridor.node_ids)
    {
      if (uid >= graph.numNodes()) continue;
      const auto & u_nd = graph.getNode(uid);

      for (int k = 0; k < 4; ++k)
      {
        int nr = u_nd.row + dirs[k].dr;
        int nc = u_nd.col + dirs[k].dc;

        const auto & nbs = graph.getSpatialCellNodes(nr, nc);
        bool has_valid_corridor_neighbor = false;
        for (uint32_t vid : nbs)
        {
          if (!corridor.isInCorridor(vid)) continue;
          const auto & v_nd = graph.getNode(vid);
          if (std::abs(v_nd.z - u_nd.z) <= max_dz)
          {
            has_valid_corridor_neighbor = true;
            break;
          }
        }

        if (!has_valid_corridor_neighbor)
        {
          geometry_msgs::Point p1, p2, p1_top, p2_top;
          p1.x = u_nd.x + dirs[k].dx1;
          p1.y = u_nd.y + dirs[k].dy1;
          p1.z = u_nd.z;

          p2.x = u_nd.x + dirs[k].dx2;
          p2.y = u_nd.y + dirs[k].dy2;
          p2.z = u_nd.z;

          p1_top = p1;
          p1_top.z += wall_height;
          p2_top = p2;
          p2_top.z += wall_height;

          // 1. 边界线 (底线, 顶护栏线, 垂直立柱)
          line_marker.points.push_back(p1);
          line_marker.points.push_back(p2);

          line_marker.points.push_back(p1_top);
          line_marker.points.push_back(p2_top);

          line_marker.points.push_back(p1);
          line_marker.points.push_back(p1_top);

          // 2. 防护墙 (两三角形)
          wall_marker.points.push_back(p1);
          wall_marker.points.push_back(p2);
          wall_marker.points.push_back(p2_top);

          wall_marker.points.push_back(p1);
          wall_marker.points.push_back(p2_top);
          wall_marker.points.push_back(p1_top);
        }
      }
    }

    out_markers.markers.push_back(line_marker);
    out_markers.markers.push_back(wall_marker);
  }
};

} // namespace elevation_planner
