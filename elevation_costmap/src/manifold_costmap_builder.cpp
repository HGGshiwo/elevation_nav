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
                                         costmap_converter::ObstacleArrayMsg * out_obstacles,
                                         elevation_planner::LocalElevationGrid * out_elevation_grid) const
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

  if (out_elevation_grid)
  {
    *out_elevation_grid = elevation_planner::LocalElevationGrid(cols, rows, res, origin_x, origin_y, config_.map_frame, static_cast<float>(origin_z));
  }

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

  const float max_layer_dz = 1.5f;          // 局部规划切层相对高差上限 (排除异层/二楼天花板)
  const float max_step_dz = 0.25f;          // 单步台阶高差上限 (单步联通)
  const double max_step_dxy2 = 0.35 * 0.35; // 单步平面相邻距离上限 (x,y 空间相邻)

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
      const auto & curr_nd = graph.getNode(curr);

      if (curr_nd.x >= win_x_min && curr_nd.x <= win_x_max &&
          curr_nd.y >= win_y_min && curr_nd.y <= win_y_max &&
          std::abs(curr_nd.z - static_cast<float>(p0.z())) <= max_layer_dz)
      {
        candidate_nodes.push_back(curr);
        is_candidate[curr] = 1;

        uint16_t edge_count = 0;
        const auto * edges = graph.getEdges(curr, edge_count);
        for (uint16_t e = 0; e < edge_count; ++e)
        {
          uint32_t nbr = edges[e].target_id;
          if (visited[nbr]) continue;

          const auto & nbr_nd = graph.getNode(nbr);

          // 1. 窗口边界过滤
          if (nbr_nd.x < win_x_min || nbr_nd.x > win_x_max ||
              nbr_nd.y < win_y_min || nbr_nd.y > win_y_max)
            continue;

          // 2. x, y 相邻判定 (平面距离 <= 0.35m)
          double dxy2 = std::pow(nbr_nd.x - curr_nd.x, 2) + std::pow(nbr_nd.y - curr_nd.y, 2);
          if (dxy2 > max_step_dxy2) continue;

          // 3. 单步联通高差判定 (|dz| <= 0.25m)
          float s_dz = std::abs(nbr_nd.z - curr_nd.z);
          if (s_dz > max_step_dz) continue;

          // 4. 局部层高判定 (|z - p0.z| <= 1.5m, 排除二楼/天花板)
          if (std::abs(nbr_nd.z - static_cast<float>(p0.z())) > max_layer_dz)
            continue;

          visited[nbr] = true;
          q.push(nbr);
        }
      }
    }
  }

  // 兜底策略: 若图无拓扑边(如轻量测试)或未锁定 N0, 则收集窗口内同层几何节点
  if (candidate_nodes.empty())
  {
    for (size_t i = 0; i < graph.numNodes(); ++i)
    {
      const auto & nd = graph.getNode(i);
      if (nd.x >= win_x_min && nd.x <= win_x_max &&
          nd.y >= win_y_min && nd.y <= win_y_max &&
          std::abs(nd.z - static_cast<float>(p0.z())) <= max_layer_dz)
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
    if (dz > max_layer_dz) continue;

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
          if (out_elevation_grid) out_elevation_grid->setZ(nr, nc, nd.z);
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
            if (out_elevation_grid) out_elevation_grid->setZ(nr, nc, nd.z);
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

          // 3. 确系物理断层/断坎 (如楼梯边缘悬崖直落底层): 在偏离机器人高度的一侧 (或两侧) 插入致命障碍
          float dzu = std::abs(nu.z - static_cast<float>(p0.z()));
          float dzv = std::abs(nv.z - static_cast<float>(p0.z()));
          if (dzv >= dzu) seam_cells.push_back(nidx);
          if (dzu >= dzv) seam_cells.push_back(idx);
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
            out_elevation_grid->setZ(r, c, static_cast<float>(p0.z()));
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
