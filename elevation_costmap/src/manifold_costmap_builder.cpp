#include "elevation_costmap/manifold_costmap_builder.h"
#include <tf2/utils.h>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <queue>
#include <unordered_set>

#include "elevation_costmap/manifold_obstacle_extractor.h"

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
                                         costmap_converter::ObstacleArrayMsg * out_obstacles) const
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
  std::vector<char> is_candidate(graph.numNodes(), 0);

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
        is_candidate[curr] = 1;

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
        is_candidate[static_cast<size_t>(i)] = 1;
        candidate_nodes.push_back(static_cast<uint32_t>(i));
      }
    }
  }

  // -------------------------------------------------------------
  // 第四步: 2D 栅格深度测试 (Z-Buffer 去重，铺满连通踏面，重合时丢弃较远层)
  // -------------------------------------------------------------
  // 初始地图默认全为 100 (悬崖/墙体等不可通行区)
  out_grid.data.assign(rows * cols, 100);
  // 逐格成因追踪 (调试图层): 初始全为 "无节点盖章"
  if (out_reasons) out_reasons->assign(rows * cols, REASON_NO_NODE);
  // 逐格胜出节点 id: 初始 -1 (无节点)
  if (out_node_ids) out_node_ids->assign(rows * cols, -1);

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
    // 成因码: 依节点禁行标志细分, 软代价与自由分开记录
    int8_t node_reason = REASON_FREE_NODE;
    if (!is_passable) {
      if (nd.flags & elevation_planner::node_flags::BLOCK_HEADROOM) node_reason = REASON_BLOCK_HEADROOM;
      else if (nd.flags & elevation_planner::node_flags::BLOCK_LATERAL) node_reason = REASON_BLOCK_LATERAL;
      else node_reason = REASON_BLOCK_OTHER;
    } else if (nd.traversability > 0.05f) {
      node_reason = REASON_SOFT_NODE;
    }

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
          if (out_reasons) (*out_reasons)[idx] = node_reason;
          if (out_node_ids) (*out_node_ids)[idx] = static_cast<int32_t>(nid);
        }
        else if (std::abs(dz - min_dz[idx]) <= 0.20f)
        {
          const int8_t merged = std::max(out_grid.data[idx], cell_cost);
          out_grid.data[idx] = merged;
          // 本节点代价为合并后最恶劣值时, 成因归属本节点
          if (out_reasons && cell_cost >= merged) (*out_reasons)[idx] = node_reason;
          if (out_node_ids && cell_cost >= merged) (*out_node_ids)[idx] = static_cast<int32_t>(nid);
          if (dz < min_dz[idx])
          {
            min_dz[idx] = dz;
          }
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
  if (!candidate_nodes.empty())
  {
    const double graph_res = graph.getResolution();
    const double gx_min = graph.getMinX();
    const double gy_min = graph.getMinY();
    const int graph_rows = graph.getRows();
    const int graph_cols = graph.getCols();

    // 窗口内候选节点的无向边集合 (键: min_id<<32 | max_id)
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

    const int r_begin = std::max(0, static_cast<int>(std::floor((origin_x - gx_min) / graph_res)));
    const int r_end = std::min(graph_rows - 1,
        static_cast<int>(std::floor((origin_x + config_.map_width - gx_min) / graph_res)));
    const int c_begin = std::max(0, static_cast<int>(std::floor((origin_y - gy_min) / graph_res)));
    const int c_end = std::min(graph_cols - 1,
        static_cast<int>(std::floor((origin_y + config_.map_length - gy_min) / graph_res)));

    std::vector<uint32_t> cell_a, cell_b;
    auto collectPassable = [&](int r, int c, std::vector<uint32_t> & out) {
      out.clear();
      if (r < 0 || r >= graph_rows || c < 0 || c >= graph_cols) return;
      for (uint32_t nid : graph.getSpatialCellNodes(r, c))
      {
        if (!is_candidate[nid]) continue;
        const auto & nd = graph.getNode(nid);
        if (nd.traversability < 0.8f &&
            (nd.headroom <= 0.0f || nd.headroom >= config_.dog_height))
          out.push_back(nid);
      }
    };
    auto anyCrossEdge = [&](const std::vector<uint32_t> & a, const std::vector<uint32_t> & b) {
      for (uint32_t ua : a)
        for (uint32_t ub : b)
        {
          if (ua == ub) return true;
          const uint64_t key = ua < ub
            ? (static_cast<uint64_t>(ua) << 32) | static_cast<uint64_t>(ub)
            : (static_cast<uint64_t>(ub) << 32) | static_cast<uint64_t>(ua);
          if (edge_keys.count(key) != 0) return true;
        }
      return false;
    };
    auto paintLethal = [&](double wx0, double wy0, double wx1, double wy1) {
      const int ic0 = std::max(0, static_cast<int>(std::floor((wx0 - origin_x) / res)));
      const int ic1 = std::min(cols - 1, static_cast<int>(std::ceil((wx1 - origin_x) / res)) - 1);
      const int ir0 = std::max(0, static_cast<int>(std::floor((wy0 - origin_y) / res)));
      const int ir1 = std::min(rows - 1, static_cast<int>(std::ceil((wy1 - origin_y) / res)) - 1);
      for (int ir = ir0; ir <= ir1; ++ir)
        for (int ic = ic0; ic <= ic1; ++ic)
        {
          const size_t sidx = static_cast<size_t>(ir * cols + ic);
          out_grid.data[sidx] = 100;
          if (out_reasons) (*out_reasons)[sidx] = REASON_SEAM;
          if (out_node_ids) (*out_node_ids)[sidx] = -1; // 缝无单一归属节点
        }
    };

    for (int r = r_begin; r <= r_end; ++r)
    {
      for (int c = c_begin; c <= c_end; ++c)
      {
        collectPassable(r, c, cell_a);
        if (cell_a.empty()) continue;

        if (c + 1 < graph_cols)  // +Y 邻格: 交界线平行于 X 轴
        {
          collectPassable(r, c + 1, cell_b);
          if (!cell_b.empty() && !anyCrossEdge(cell_a, cell_b))
          {
            const double by = gy_min + (c + 1) * graph_res;
            paintLethal(gx_min + r * graph_res, by - res * 0.5,
                        gx_min + (r + 1) * graph_res, by + res * 0.5);
          }
        }
        if (r + 1 < graph_rows)  // +X 邻格: 交界线平行于 Y 轴
        {
          collectPassable(r + 1, c, cell_b);
          if (!cell_b.empty() && !anyCrossEdge(cell_a, cell_b))
          {
            const double bx = gx_min + (r + 1) * graph_res;
            paintLethal(bx - res * 0.5, gy_min + c * graph_res,
                        bx + res * 0.5, gy_min + (c + 1) * graph_res);
          }
        }
      }
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
