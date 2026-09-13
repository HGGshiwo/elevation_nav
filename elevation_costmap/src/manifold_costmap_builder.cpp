#include "elevation_costmap/manifold_costmap_builder.h"
#include <tf2/utils.h>
#include <algorithm>
#include <cmath>
#include <queue>

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
                                         geometry_msgs::TransformStamped & out_tf) const
{
  (void)global_plan;
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

  // -------------------------------------------------------------
  // 第三步: 拓扑连通优先遍历 (BFS 从脚下 N0 沿拓扑边辐射，不受垂直高差截断)
  // -------------------------------------------------------------
  std::vector<uint32_t> candidate_nodes;
  candidate_nodes.reserve(2000);

  const double win_x_min = origin_x - 0.3;
  const double win_x_max = origin_x + config_.map_width + 0.3;
  const double win_y_min = origin_y - 0.3;
  const double win_y_max = origin_y + config_.map_length + 0.3;

  if (found_n0 && graph.numEdges() > 0)
  {
    std::vector<bool> visited(graph.numNodes(), false);
    std::queue<uint32_t> q;
    q.push(n0_id);
    visited[n0_id] = true;

    while (!q.empty())
    {
      uint32_t curr = q.front();
      q.pop();
      const auto & nd = graph.getNode(curr);

      if (nd.x >= win_x_min && nd.x <= win_x_max &&
          nd.y >= win_y_min && nd.y <= win_y_max)
      {
        candidate_nodes.push_back(curr);

        uint16_t edge_count = 0;
        const auto * edges = graph.getEdges(curr, edge_count);
        for (uint16_t e = 0; e < edge_count; ++e)
        {
          uint32_t nbr = edges[e].target_id;
          if (!visited[nbr])
          {
            visited[nbr] = true;
            q.push(nbr);
          }
        }
      }
    }
  }

  // 兜底策略: 若图无拓扑边(如轻量测试)或未锁定 N0, 则收集窗口内所有几何节点
  if (candidate_nodes.empty())
  {
    for (size_t i = 0; i < graph.numNodes(); ++i)
    {
      const auto & nd = graph.getNode(i);
      if (nd.x >= win_x_min && nd.x <= win_x_max &&
          nd.y >= win_y_min && nd.y <= win_y_max)
      {
        candidate_nodes.push_back(static_cast<uint32_t>(i));
      }
    }
  }

  // -------------------------------------------------------------
  // 第四步: 2D 栅格深度测试 (Z-Buffer 去重，铺满连通踏面，重合时丢弃较远层)
  // -------------------------------------------------------------
  // 初始地图默认全为 100 (悬崖/墙体等不可通行区)
  out_grid.data.assign(rows * cols, 100);

  // 深度缓冲区: 记录每个 2D 栅格已记录的踏面距离机器人基准高度的绝对高差 |z - z0|
  std::vector<float> min_dz(rows * cols, std::numeric_limits<float>::max());

  // 盖章半径: 确保节点间无孔洞缝隙铺满踏面
  const double stamp_radius = std::max(0.08, graph.getResolution() * 0.75);
  const int stamp_cells = std::max(1, static_cast<int>(std::ceil(stamp_radius / res)));
  const double stamp_r2 = stamp_radius * stamp_radius;

  for (uint32_t nid : candidate_nodes)
  {
    const auto & nd = graph.getNode(nid);
    float dz = std::abs(nd.z - static_cast<float>(p0.z()));

    int center_c = static_cast<int>(std::floor((nd.x - origin_x) / res));
    int center_r = static_cast<int>(std::floor((nd.y - origin_y) / res));

    bool is_passable = (nd.traversability < 0.8f &&
                        (nd.headroom <= 0.0f || nd.headroom >= config_.dog_height));
    int8_t cell_cost = is_passable ? static_cast<int8_t>(nd.traversability * 70.0f) : 100;

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

        // 核心深度测试:
        // 1. 若当前节点高度比已记录节点明显更靠近机器人 (高度差差额 > 0.20m), 强行覆盖 (丢弃较远层)
        // 2. 若高度差在同一层内 (<= 0.20m), 属于同一踏面层, 代价取最恶劣值以确保安全, 并更新最小 dz
        // 3. 若当前节点明显更远 (dz > min_dz[idx] + 0.20m), 属于楼上或楼下层, 直接丢弃
        if (dz < min_dz[idx] - 0.20f)
        {
          min_dz[idx] = dz;
          out_grid.data[idx] = cell_cost;
        }
        else if (std::abs(dz - min_dz[idx]) <= 0.20f)
        {
          out_grid.data[idx] = std::max(out_grid.data[idx], cell_cost);
          if (dz < min_dz[idx])
          {
            min_dz[idx] = dz;
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

  return true;
}

} // namespace elevation_costmap
