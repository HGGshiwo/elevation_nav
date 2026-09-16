#include "elevation_costmap/manifold_obstacle_extractor.h"
#include <algorithm>
#include <cmath>
#include <map>
#include <unordered_map>
#include <unordered_set>

namespace elevation_costmap
{

ManifoldObstacleExtractor::ManifoldObstacleExtractor(const ObstacleExtractorConfig & config)
: config_(config)
{
}

bool ManifoldObstacleExtractor::extractObstacles(const elevation_planner::ManifoldGraph & graph,
                                                const geometry_msgs::Pose & robot_pose,
                                                const std::vector<uint32_t> & candidate_nodes,
                                                const std::vector<char> & is_candidate,
                                                costmap_converter::ObstacleArrayMsg & out_obstacles) const
{
  if (graph.numNodes() == 0 || candidate_nodes.empty()) return false;

  out_obstacles.obstacles.clear();
  out_obstacles.header.stamp = ros::Time::now();
  out_obstacles.header.frame_id = config_.map_frame;

  // 1. 确定机器人基准足端高度
  double center_x = robot_pose.position.x;
  double center_y = robot_pose.position.y;
  double center_z = robot_pose.position.z;

  uint32_t n0_id = 0;
  if (graph.findClosestNode(center_x, center_y, center_z, n0_id, 1.0, 1.0))
  {
    center_z = graph.getNode(n0_id).z;
  }

  // 2. 方案 B 核心：提取当前活动踏面的连续边界防护墙与断坎 (LineObstacle - 连续防穿模)
  extractBoundaryLineObstacles(graph, center_x, center_y, center_z, candidate_nodes, is_candidate, out_obstacles.obstacles);

  // 3. 方案 B 辅助：提取当前活动踏面内部的孤立立柱/悬挂阻挡节点 (CircularObstacle - 紧凑小圆)
  extractIsolatedCircularObstacles(graph, center_x, center_y, center_z, candidate_nodes, is_candidate, out_obstacles.obstacles);

  return true;
}

// 辅助线段结构 (支持携带实际踏面高程)
struct Seg1D
{
  double start{0.0};
  double end{0.0};
  double z{0.0};
};

static std::vector<Seg1D> mergeSegments1D(std::vector<Seg1D> & segs, double tol = 0.02)
{
  if (segs.empty()) return segs;
  std::sort(segs.begin(), segs.end(), [](const Seg1D & a, const Seg1D & b) {
    return a.start < b.start;
  });

  std::vector<Seg1D> merged;
  merged.push_back(segs.front());

  for (size_t i = 1; i < segs.size(); ++i)
  {
    auto & last = merged.back();
    // 空间相邻且高差不超过 35cm 时共线合并，平滑楼梯斜坡与长断坎
    if (segs[i].start <= last.end + tol && std::abs(segs[i].z - last.z) < 0.35)
    {
      last.end = std::max(last.end, segs[i].end);
      last.z = (last.z + segs[i].z) * 0.5;
    }
    else
    {
      merged.push_back(segs[i]);
    }
  }
  return merged;
}

void ManifoldObstacleExtractor::extractBoundaryLineObstacles(const elevation_planner::ManifoldGraph & graph,
                                                            double center_x, double center_y, double center_z,
                                                            const std::vector<uint32_t> & candidate_nodes,
                                                            const std::vector<char> & is_candidate,
                                                            std::vector<costmap_converter::ObstacleMsg> & out_obs) const
{
  const double R = config_.window_radius;
  const double graph_res = graph.getResolution();
  const double gx_min = graph.getMinX();
  const double gy_min = graph.getMinY();
  const int graph_rows = graph.getRows();
  const int graph_cols = graph.getCols();

  // 1. 提取所有属于当前活动连通图的可通行节点
  std::vector<uint32_t> passable_nodes;
  passable_nodes.reserve(candidate_nodes.size());
  std::vector<char> is_passable(graph.numNodes(), 0);
  for (uint32_t nid : candidate_nodes)
  {
    const auto & nd = graph.getNode(nid);
    if (nd.traversability < 0.8f && (nd.headroom <= 0.0f || nd.headroom >= config_.dog_height))
    {
      passable_nodes.push_back(nid);
      is_passable[nid] = 1;
    }
  }

  // 2. 保护直接连通走廊 (含正交与对角边)：这些走廊穿过的交界面绝对不插护栏
  // 垂直边界键: (r, c) 表示 (r, c) 与 (r+1, c) 之间的垂直交界线 (X = const)
  // 水平边界键: (r, c) 表示 (r, c) 与 (r, c+1) 之间的水平交界线 (Y = const)
  auto makeKey = [](int r, int c) -> uint64_t {
    return (static_cast<uint64_t>(static_cast<uint32_t>(r)) << 32) | static_cast<uint64_t>(static_cast<uint32_t>(c));
  };
  std::unordered_map<uint64_t, std::vector<float>> vert_protected;
  std::unordered_map<uint64_t, std::vector<float>> horiz_protected;

  for (uint32_t u_id : passable_nodes)
  {
    const auto & u = graph.getNode(u_id);
    uint16_t ec = 0;
    const auto * es = graph.getEdges(u_id, ec);
    for (uint16_t k = 0; k < ec; ++k)
    {
      uint32_t v_id = es[k].target_id;
      if (v_id <= u_id || !is_passable[v_id]) continue;
      const auto & v = graph.getNode(v_id);

      int r1 = u.row, c1 = u.col;
      int r2 = v.row, c2 = v.col;
      float z_edge = (u.z + v.z) * 0.5f;

      if (r1 == r2 && std::abs(c1 - c2) == 1)
      {
        // 正交 Y 方向相连：保护两格之间的水平交界线
        horiz_protected[makeKey(r1, std::min(c1, c2))].push_back(z_edge);
      }
      else if (c1 == c2 && std::abs(r1 - r2) == 1)
      {
        // 正交 X 方向相连：保护两格之间的垂直交界线
        vert_protected[makeKey(std::min(r1, r2), c1)].push_back(z_edge);
      }
      else if (std::abs(r1 - r2) == 1 && std::abs(c1 - c2) == 1)
      {
        // 对角直接连通！保护该对角走廊所穿过的 4 个单元格中心十字交界面
        int r_min = std::min(r1, r2);
        int c_min = std::min(c1, c2);
        int c_max = std::max(c1, c2);
        vert_protected[makeKey(r_min, c_min)].push_back(z_edge);
        vert_protected[makeKey(r_min, c_max)].push_back(z_edge);
        horiz_protected[makeKey(r_min, c_min)].push_back(z_edge);
        horiz_protected[makeKey(std::max(r1, r2), c_min)].push_back(z_edge);
      }
    }
  }

  // 3. 逐网格探测未受保护的交界分界面，检测真实悬崖断坎
  int r_start = std::max(0, static_cast<int>(std::floor((center_x - R - gx_min) / graph_res)));
  int r_end   = std::min(graph_rows - 1, static_cast<int>(std::ceil((center_x + R - gx_min) / graph_res)));
  int c_start = std::max(0, static_cast<int>(std::floor((center_y - R - gy_min) / graph_res)));
  int c_end   = std::min(graph_cols - 1, static_cast<int>(std::ceil((center_y + R - gy_min) / graph_res)));

  // 辅助函数: 检查两格交界处在当前高程 z 是否已有直接连通走廊保护
  const double max_step = 0.25; // 四足狗安全步高极限 (m)
  auto isHorizProtected = [&](int r, int c, double z) -> bool {
    auto it = horiz_protected.find(makeKey(r, c));
    if (it == horiz_protected.end()) return false;
    for (float ze : it->second)
    {
      if (std::abs(ze - z) <= max_step) return true;
    }
    return false;
  };

  auto isVertProtected = [&](int r, int c, double z) -> bool {
    auto it = vert_protected.find(makeKey(r, c));
    if (it == vert_protected.end()) return false;
    for (float ze : it->second)
    {
      if (std::abs(ze - z) <= max_step) return true;
    }
    return false;
  };

  // 辅助函数: 检查邻格内是否存在可与当前节点平缓过渡的踏面 (高差在单步极限内)
  auto hasPassableSurfaceWithinStep = [&](int r, int c, double cur_z) -> bool {
    if (r < 0 || r >= graph_rows || c < 0 || c >= graph_cols) return false;
    for (uint32_t nid : graph.getSpatialCellNodes(r, c))
    {
      if (!is_passable[nid]) continue;
      const auto & nd = graph.getNode(nid);
      if (std::abs(nd.z - cur_z) <= max_step) return true;
    }
    return false;
  };

  std::map<int, std::vector<Seg1D>> horiz_segments;
  std::map<int, std::vector<Seg1D>> vert_segments;

  for (int r = r_start; r <= r_end; ++r)
  {
    for (int c = c_start; c <= c_end; ++c)
    {
      uint32_t curr_nid = 0;
      bool has_curr = false;
      double min_dz = std::numeric_limits<double>::max();
      for (uint32_t nid : graph.getSpatialCellNodes(r, c))
      {
        if (!is_passable[nid]) continue;
        const auto & nd = graph.getNode(nid);
        double dz = std::abs(nd.z - center_z);
        if (dz < min_dz)
        {
          min_dz = dz;
          curr_nid = nid;
          has_curr = true;
        }
      }
      if (!has_curr) continue; // 当前格无当前层可行踏面

      const double x0 = gx_min + r * graph_res;
      const double x1 = gx_min + (r + 1) * graph_res;
      const double y0 = gy_min + c * graph_res;
      const double y1 = gy_min + (c + 1) * graph_res;
      const double z_curr = graph.getNode(curr_nid).z;

      // 1. 检查 +Y 邻格交界面 (水平线段)
      if (!isHorizProtected(r, c, z_curr))
      {
        if (!hasPassableSurfaceWithinStep(r, c + 1, z_curr))
        {
          int y_key = static_cast<int>(std::round(y1 / graph_res));
          horiz_segments[y_key].push_back({x0, x1, z_curr});
        }
      }

      // 2. 检查 -Y 邻格交界面 (水平线段)
      if (!isHorizProtected(r, c - 1, z_curr))
      {
        if (!hasPassableSurfaceWithinStep(r, c - 1, z_curr))
        {
          int y_key = static_cast<int>(std::round(y0 / graph_res));
          horiz_segments[y_key].push_back({x0, x1, z_curr});
        }
      }

      // 3. 检查 +X 邻格交界面 (垂直线段)
      if (!isVertProtected(r, c, z_curr))
      {
        if (!hasPassableSurfaceWithinStep(r + 1, c, z_curr))
        {
          int x_key = static_cast<int>(std::round(x1 / graph_res));
          vert_segments[x_key].push_back({y0, y1, z_curr});
        }
      }

      // 4. 检查 -X 邻格交界面 (垂直线段)
      if (!isVertProtected(r - 1, c, z_curr))
      {
        if (!hasPassableSurfaceWithinStep(r - 1, c, z_curr))
        {
          int x_key = static_cast<int>(std::round(x0 / graph_res));
          vert_segments[x_key].push_back({y0, y1, z_curr});
        }
      }
    }
  }

  // 4. 共线合并水平边界线段 (LineObstacle)
  for (auto & kv : horiz_segments)
  {
    double by = kv.first * graph_res;
    auto merged = mergeSegments1D(kv.second);
    for (const auto & seg : merged)
    {
      double len = seg.end - seg.start;
      if (len < std::min(config_.min_boundary_length, graph_res * 0.8)) continue;

      costmap_converter::ObstacleMsg msg;
      msg.header.stamp = ros::Time::now();
      msg.header.frame_id = config_.map_frame;
      msg.id = 2000 + static_cast<int64_t>(out_obs.size());
      msg.radius = 0.0;

      geometry_msgs::Point32 p1, p2;
      p1.x = seg.start;
      p1.y = by;
      p1.z = seg.z;

      p2.x = seg.end;
      p2.y = by;
      p2.z = seg.z;

      msg.polygon.points.push_back(p1);
      msg.polygon.points.push_back(p2);
      out_obs.push_back(msg);
    }
  }

  // 5. 共线合并垂直边界线段 (LineObstacle)
  for (auto & kv : vert_segments)
  {
    double bx = kv.first * graph_res;
    auto merged = mergeSegments1D(kv.second);
    for (const auto & seg : merged)
    {
      double len = seg.end - seg.start;
      if (len < std::min(config_.min_boundary_length, graph_res * 0.8)) continue;

      costmap_converter::ObstacleMsg msg;
      msg.header.stamp = ros::Time::now();
      msg.header.frame_id = config_.map_frame;
      msg.id = 2000 + static_cast<int64_t>(out_obs.size());
      msg.radius = 0.0;

      geometry_msgs::Point32 p1, p2;
      p1.x = bx;
      p1.y = seg.start;
      p1.z = seg.z;

      p2.x = bx;
      p2.y = seg.end;
      p2.z = seg.z;

      msg.polygon.points.push_back(p1);
      msg.polygon.points.push_back(p2);
      out_obs.push_back(msg);
    }
  }
}

void ManifoldObstacleExtractor::extractIsolatedCircularObstacles(const elevation_planner::ManifoldGraph & graph,
                                                                double center_x, double center_y, double center_z,
                                                                const std::vector<uint32_t> & candidate_nodes,
                                                                const std::vector<char> & is_candidate,
                                                                std::vector<costmap_converter::ObstacleMsg> & out_obs) const
{
  const double R = config_.window_radius;
  const double R_sq = R * R;
  const int graph_rows = graph.getRows();
  const int graph_cols = graph.getCols();
  std::unordered_set<uint64_t> visited_cells;

  auto isPassable = [&](const elevation_planner::GraphNode & nd) -> bool {
    return (nd.traversability < 0.8f) &&
           (nd.headroom <= 0.0f || nd.headroom >= config_.dog_height) &&
           !(nd.flags & elevation_planner::node_flags::BLOCK_HEADROOM);
  };

  // 纯净立柱提取规则：
  // 1. 排除仅因头顶净空不足标记禁行的踏面 (如楼梯下方、桌底是负向不可走区域，而非突起的实体立柱)
  // 2. 排除贴墙侧向膨胀标记的地面/台阶节点 (BLOCK_LATERAL 已由 LineObstacle3D 沿边界防护，严禁重复生成假立柱)
  auto isSolidObstacle = [&](const elevation_planner::GraphNode & nd) -> bool {
    if (nd.flags & elevation_planner::node_flags::BLOCK_HEADROOM) return false;
    if (nd.flags & elevation_planner::node_flags::BLOCK_LATERAL) return false;
    return (nd.traversability >= 0.8f);
  };

  // 方案 2：同格已有踏面遮盖屏蔽
  // 检查空间网格 (r, c) 在目标高程 z 附近或上方是否已存在合法连通的可通行踏面
  // 若存在，说明该格在当前行走层本身就是正常通道/台阶，被遮盖在下方的残余节点绝不能生成为立柱
  const double max_step = 0.25; // 四足狗安全步高极限 (m)
  auto cellHasPassableTreadAtOrAbove = [&](int r, int c, double z) -> bool {
    if (r < 0 || r >= graph_rows || c < 0 || c >= graph_cols) return false;
    for (uint32_t adj_nid : graph.getSpatialCellNodes(r, c))
    {
      const auto & nd = graph.getNode(adj_nid);
      if (isPassable(nd))
      {
        // 1. 高程在单步台阶范围内 (|nd.z - z| <= max_step)，同属于当前层正常踏面
        if (std::abs(nd.z - z) <= max_step) return true;
        // 2. 踏面高程在 z 上方且在机体净空内，说明 z 属于被踏面覆盖的下层残余结构
        if (nd.z >= z - 0.05 && nd.z <= z + config_.dog_height) return true;
      }
    }
    return false;
  };

  for (uint32_t nid : candidate_nodes)
  {
    const auto & u = graph.getNode(nid);

    // 1. 节点本身即为阻挡实体 (如活动层内孤立立柱、低矮管道)
    if (isSolidObstacle(u))
    {
      if (!cellHasPassableTreadAtOrAbove(u.row, u.col, u.z))
      {
        double dx = u.x - center_x;
        double dy = u.y - center_y;
        if (dx * dx + dy * dy <= R_sq && u.z >= center_z && u.z <= center_z + config_.dog_height)
        {
          uint64_t cell_key = (static_cast<uint64_t>(u.row) << 32) | static_cast<uint64_t>(u.col);
          if (visited_cells.insert(cell_key).second)
          {
            costmap_converter::ObstacleMsg msg;
            msg.header.stamp = ros::Time::now();
            msg.header.frame_id = config_.map_frame;
            msg.id = 1000 + static_cast<int64_t>(out_obs.size());
            msg.radius = config_.default_circle_radius;

            geometry_msgs::Point32 pt;
            pt.x = u.x;
            pt.y = u.y;
            pt.z = u.z;
            msg.polygon.points.push_back(pt);
            out_obs.push_back(msg);
          }
        }
      }
      continue;
    }

    if (!isPassable(u)) continue;

    // 2. 节点为连通可行踏面：检查其 8 邻域，若相邻点为阻挡实体且高度侵入机体空间，提取为障碍物
    for (int dr = -1; dr <= 1; ++dr)
    {
      for (int dc = -1; dc <= 1; ++dc)
      {
        if (dr == 0 && dc == 0) continue;
        int nr = u.row + dr;
        int nc = u.col + dc;
        if (nr < 0 || nr >= graph_rows || nc < 0 || nc >= graph_cols) continue;

        for (uint32_t adj_id : graph.getSpatialCellNodes(nr, nc))
        {
          const auto & adj_nd = graph.getNode(adj_id);
          // 方案 3：排除仅因头顶净空不足标记禁行的节点
          if (!isSolidObstacle(adj_nd)) continue;

          // 方案 1：必须高于当前踏面且在机体高度包络内，严格杜绝脚下低层节点"长上来"穿透踏面
          if (adj_nd.z < u.z || adj_nd.z > u.z + config_.dog_height) continue;

          // 方案 2：若目标网格在其高度附近或上方已有可通行踏面，说明该格是正常楼梯/走廊，严禁插柱
          if (cellHasPassableTreadAtOrAbove(nr, nc, adj_nd.z)) continue;

          double dx = adj_nd.x - center_x;
          double dy = adj_nd.y - center_y;
          if (dx * dx + dy * dy <= R_sq)
          {
            uint64_t cell_key = (static_cast<uint64_t>(nr) << 32) | static_cast<uint64_t>(nc);
            if (visited_cells.insert(cell_key).second)
            {
              costmap_converter::ObstacleMsg msg;
              msg.header.stamp = ros::Time::now();
              msg.header.frame_id = config_.map_frame;
              msg.id = 1000 + static_cast<int64_t>(out_obs.size());
              msg.radius = config_.default_circle_radius;

              geometry_msgs::Point32 pt;
              pt.x = adj_nd.x;
              pt.y = adj_nd.y;
              pt.z = adj_nd.z;
              msg.polygon.points.push_back(pt);
              out_obs.push_back(msg);
            }
          }
        }
      }
    }
  }
}

} // namespace elevation_costmap
