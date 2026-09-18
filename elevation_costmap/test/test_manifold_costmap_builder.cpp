#include "elevation_costmap/manifold_costmap_builder.h"
#include <elevation_planner_core/manifold_graph.hpp>
#include <elevation_planner_core/trajectory_validator.hpp>
#include <elevation_planner_core/topological_corridor.hpp>
#include <iostream>
#include <cassert>
#include <cmath>

int main()
{
  ros::Time::init();
  std::cout << "[TestManifoldCostmapBuilder] Starting unit tests..." << std::endl;

  // 1. 构造包含 35 度斜坡/楼梯踏面的 ManifoldGraph
  elevation_planner::ManifoldGraph graph;
  graph.initSpatialGrid(0.10, -5.0, -5.0, 100, 100);

  for (int r = 0; r <= 30; ++r)
  {
    double x = r * 0.10;
    double z = r * 0.08; // 梯级上升: dx=0.1, dz=0.08 -> 坡度约 38.6 度
    for (int c = -5; c <= 5; ++c)
    {
      double y = c * 0.10;
      elevation_planner::GraphNode node;
      node.x = static_cast<float>(x);
      node.y = static_cast<float>(y);
      node.z = static_cast<float>(z);
      node.traversability = 0.0f;
      node.headroom = 2.0f;
      node.row = 50 + r;
      node.col = 50 + c;
      node.layer_id = 0;
      graph.addNode(node);
    }
  }
  // 相邻节点补边: 楼梯逐级可达 (无边则缝绘制会在整幅地毯上刻出致命格线)
  for (int r = 0; r <= 30; ++r)
    for (int c = -5; c <= 5; ++c)
    {
      const int id = r * 11 + (c + 5);
      if (r < 30) { graph.addEdge(id, id + 11, 0.1f); graph.addEdge(id + 11, id, 0.1f); }
      if (c < 5)  { graph.addEdge(id, id + 1, 0.1f);  graph.addEdge(id + 1, id, 0.1f); }
    }
  graph.finalizeCSR();

  std::cout << "  Graph built with " << graph.numNodes() << " nodes." << std::endl;
  assert(graph.numNodes() > 0);

  // 2. 初始化 ManifoldCostmapBuilder
  elevation_costmap::ManifoldCostmapBuilderConfig config;
  config.resolution = 0.05;
  config.map_length = 3.0;
  config.map_width = 2.0;
  config.forward_offset = 0.5;
  config.lookahead_distance = 0.6;
  elevation_costmap::ManifoldCostmapBuilder builder(config);

  // 3. 模拟机器人位于 (0.2, 0.0, 0.16)
  geometry_msgs::Pose robot_pose;
  robot_pose.position.x = 0.20;
  robot_pose.position.y = 0.0;
  robot_pose.position.z = 0.16;
  robot_pose.orientation.w = 1.0;

  // 模拟全局路径: 沿台阶向上
  std::vector<geometry_msgs::PoseStamped> global_plan;
  for (int i = 0; i <= 20; ++i)
  {
    geometry_msgs::PoseStamped ps;
    ps.pose.position.x = i * 0.10;
    ps.pose.position.y = 0.0;
    ps.pose.position.z = i * 0.08;
    ps.pose.orientation.w = 1.0;
    global_plan.push_back(ps);
  }

  // 4. 执行代价地图构建
  nav_msgs::OccupancyGrid out_grid;
  geometry_msgs::TransformStamped out_tf;
  bool ok = builder.buildCostmap(graph, robot_pose, global_plan, out_grid, out_tf);
  assert(ok);

  std::cout << "  Manifold costmap build success! Grid dimensions: "
            << out_grid.info.width << " x " << out_grid.info.height
            << ", resolution: " << out_grid.info.resolution << "m" << std::endl;

  // 5. 验证法向量恒朝上 (N.z > 0)
  Eigen::Quaterniond q(out_grid.info.origin.orientation.w,
                       out_grid.info.origin.orientation.x,
                       out_grid.info.origin.orientation.y,
                       out_grid.info.origin.orientation.z);
  Eigen::Vector3d n = q.toRotationMatrix().col(2);
  std::cout << "  Estimated Terrain Normal N = ["
            << n.x() << ", " << n.y() << ", " << n.z() << "]" << std::endl;
  assert(n.z() > 0.0);

  // 6. 验证踏面通道通畅与悬崖阻断
  int free_count = 0;
  int obstacle_count = 0;
  for (int8_t cost : out_grid.data)
  {
    if (cost < 50) free_count++;
    else if (cost == 100) obstacle_count++;
  }
  std::cout << "  Grid cell statistics: Free cells = " << free_count
            << ", Obstacle/Cliff cells = " << obstacle_count << std::endl;
  assert(free_count > 0);
  assert(obstacle_count > 0);

  // 7. 测试多层上下重合 (Z-Buffer 深度测试):
  // 构造一楼踏面 (z = 0.0) 和二楼天花板/踏面 (z = 3.0), 机器人位于一楼 (z = 0.0)
  elevation_planner::ManifoldGraph multi_graph;
  multi_graph.initSpatialGrid(0.10, -5.0, -5.0, 100, 100);
  // 一楼踏面 (自由可通)
  elevation_planner::GraphNode n_floor1;
  n_floor1.x = 1.0f; n_floor1.y = 1.0f; n_floor1.z = 0.0f;
  n_floor1.traversability = 0.0f; n_floor1.headroom = 2.0f;
  n_floor1.row = 60; n_floor1.col = 60; n_floor1.layer_id = 0;
  multi_graph.addNode(n_floor1);

  // 二楼踏面 (假设不可通行或者代价不同)
  elevation_planner::GraphNode n_floor2;
  n_floor2.x = 1.0f; n_floor2.y = 1.0f; n_floor2.z = 3.0f;
  n_floor2.traversability = 0.9f; n_floor2.headroom = 2.0f;
  n_floor2.row = 60; n_floor2.col = 60; n_floor2.layer_id = 1;
  multi_graph.addNode(n_floor2);
  multi_graph.finalizeCSR();

  elevation_costmap::ManifoldCostmapBuilderConfig cfg_6m;
  cfg_6m.map_width = 6.0;
  cfg_6m.map_length = 6.0;
  cfg_6m.resolution = 0.05;
  elevation_costmap::ManifoldCostmapBuilder builder_6m(cfg_6m);

  geometry_msgs::Pose bot_1f;
  bot_1f.position.x = 1.0; bot_1f.position.y = 1.0; bot_1f.position.z = 0.0;
  bot_1f.orientation.w = 1.0;

  nav_msgs::OccupancyGrid grid_1f;
  geometry_msgs::TransformStamped tf_1f;
  bool ok_1f = builder_6m.buildCostmap(multi_graph, bot_1f, {}, grid_1f, tf_1f);
  assert(ok_1f);
  assert(grid_1f.info.width == 120 && grid_1f.info.height == 120);

  // 验证在 (1.0, 1.0) 对应的中心栅格处，保留了一楼的自由代价 (0)，丢弃了二楼的不可通行 (100)
  int c_center = static_cast<int>(std::floor((1.0 - grid_1f.info.origin.position.x) / 0.05));
  int r_center = static_cast<int>(std::floor((1.0 - grid_1f.info.origin.position.y) / 0.05));
  size_t idx_center = static_cast<size_t>(r_center * grid_1f.info.width + c_center);
  std::cout << "  Z-Buffer overlap test at (1.0, 1.0): cell cost = "
            << static_cast<int>(grid_1f.data[idx_center])
            << " (Expected 0 for 1F floor)" << std::endl;
  assert(grid_1f.data[idx_center] == 0);

  // 8. 形态学闭运算: 图节点缺失 1~2 格应被填充为自由, 大片真实空洞必须保留
  //    (满铺踏面上挖掉 (50,50) 单节点 -> 伪影补丁; 挖掉 3x3 连片 -> 真实空洞)
  {
    elevation_planner::ManifoldGraph g2;
    g2.initSpatialGrid(0.10, -5.0, -5.0, 100, 100);
    for (int r = 40; r <= 60; ++r)
      for (int c = 40; c <= 60; ++c)
      {
        if (r == 50 && c == 50) continue;                       // 单点缺失
        if (r >= 55 && r <= 57 && c >= 55 && c <= 57) continue; // 3x3 连片缺失
        elevation_planner::GraphNode n;
        n.x = static_cast<float>(r * 0.10); n.y = static_cast<float>(c * 0.10);
        n.z = 0.0f; n.traversability = 0.0f; n.headroom = 2.0f;
        n.row = r; n.col = c; n.layer_id = 0;
        g2.addNode(n);
      }
    // 相邻节点补边 (可通行点阵内部全部连通, 缝绘制只应由缺格边界触发)
    for (int r = 40; r <= 60; ++r)
      for (int c = 40; c <= 60; ++c)
      {
        if (r == 50 && c == 50) continue;
        if (r >= 55 && r <= 57 && c >= 55 && c <= 57) continue;
        const int id = (r - 40) * 21 + (c - 40);
        auto present = [](int rr, int cc) {
          return rr >= 40 && rr <= 60 && cc >= 40 && cc <= 60 &&
                 !(rr == 50 && cc == 50) &&
                 !(rr >= 55 && rr <= 57 && cc >= 55 && cc <= 57);
        };
        if (present(r + 1, c)) { g2.addEdge(id, id + 21, 0.1f); g2.addEdge(id + 21, id, 0.1f); }
        if (present(r, c + 1)) { g2.addEdge(id, id + 1, 0.1f);  g2.addEdge(id + 1, id, 0.1f); }
      }
    g2.finalizeCSR();

    elevation_costmap::ManifoldCostmapBuilderConfig cfg_h;
    cfg_h.map_width = 3.0; cfg_h.map_length = 3.0; cfg_h.resolution = 0.05;
    elevation_costmap::ManifoldCostmapBuilder builder_h(cfg_h);

    geometry_msgs::Pose bot_h;
    bot_h.position.x = 5.0; bot_h.position.y = 5.0; bot_h.position.z = 0.0;
    bot_h.orientation.w = 1.0;

    nav_msgs::OccupancyGrid grid_h;
    geometry_msgs::TransformStamped tf_h;
    bool ok_h = builder_h.buildCostmap(g2, bot_h, {}, grid_h, tf_h);
    assert(ok_h);

    auto cellCostAt = [&](double wx, double wy) -> int {
      int cc = static_cast<int>(std::floor((wx - grid_h.info.origin.position.x) / 0.05));
      int rr = static_cast<int>(std::floor((wy - grid_h.info.origin.position.y) / 0.05));
      if (cc < 0 || cc >= static_cast<int>(grid_h.info.width) ||
          rr < 0 || rr >= static_cast<int>(grid_h.info.height)) return -1;
      return grid_h.data[static_cast<size_t>(rr * grid_h.info.width + cc)];
    };
    int single_hole_cost = cellCostAt(5.05, 5.05); // 单节点缺失中心
    int real_void_cost = cellCostAt(5.60, 5.60);   // 3x3 连片缺失中心
    std::cout << "  Closing test: single-hole cell = " << single_hole_cost
              << " (expect <100, filled), 3x3 void cell = " << real_void_cost
              << " (expect 100, preserved)" << std::endl;
    assert(single_hole_cost >= 0 && single_hole_cost < 100);
    assert(real_void_cost == 100);
  }

  // 9. 拓扑缝绘制: 相邻图格可通行但无边 -> 交界致命; 有边 -> 连续自由
  {
    auto buildPairGraph = [](bool with_edge) {
      elevation_planner::ManifoldGraph g;
      g.initSpatialGrid(0.10, -5.0, -5.0, 100, 100);
      for (int k = 0; k < 2; ++k)
      {
        elevation_planner::GraphNode n;
        n.x = 0.25f; n.y = 0.25f + 0.10f * k;  // 相邻两格 (52,52) 与 (52,53)
        n.z = 0.0f; n.traversability = 0.0f; n.headroom = 2.0f;
        n.row = 52; n.col = 52 + k; n.layer_id = 0;
        g.addNode(n);
      }
      if (with_edge) { g.addEdge(0, 1, 0.1f); g.addEdge(1, 0, 0.1f); }
      g.finalizeCSR();
      return g;
    };

    elevation_costmap::ManifoldCostmapBuilderConfig cfg_s;
    cfg_s.map_width = 3.0; cfg_s.map_length = 3.0; cfg_s.resolution = 0.05;
    elevation_costmap::ManifoldCostmapBuilder builder_s(cfg_s);
    geometry_msgs::Pose bot_s;
    bot_s.position.x = 0.25; bot_s.position.y = 0.30; bot_s.position.z = 0.0;
    bot_s.orientation.w = 1.0;

    auto seamCellCost = [&](nav_msgs::OccupancyGrid & grid) -> int {
      // 交界线 y = -5 + 53*0.1 = 0.3, 条带覆盖 y in [0.275, 0.325)
      int ic = static_cast<int>(std::floor((0.25 - grid.info.origin.position.x) / 0.05));
      int ir = static_cast<int>(std::floor((0.28 - grid.info.origin.position.y) / 0.05));
      return grid.data[static_cast<size_t>(ir * grid.info.width + ic)];
    };

    nav_msgs::OccupancyGrid g_no_edge, g_with_edge;
    geometry_msgs::TransformStamped tf_s;
    std::vector<int8_t> reasons_no_edge, reasons_with_edge;
    std::vector<int32_t> ids_no_edge, ids_with_edge;
    assert(builder_s.buildCostmap(buildPairGraph(false), bot_s, {}, g_no_edge, tf_s, &reasons_no_edge, &ids_no_edge));
    assert(builder_s.buildCostmap(buildPairGraph(true), bot_s, {}, g_with_edge, tf_s, &reasons_with_edge, &ids_with_edge));
    const int no_edge_cost = seamCellCost(g_no_edge);
    const int with_edge_cost = seamCellCost(g_with_edge);
    std::cout << "  Seam test: no-edge boundary cell = " << no_edge_cost
              << " (expect 100), with-edge cell = " << with_edge_cost
              << " (expect 0)" << std::endl;
    assert(no_edge_cost == 100);
    assert(with_edge_cost == 0);

    // 成因码追踪: 缝格=REASON_SEAM, 有边同格=REASON_FREE_NODE, 地图远角=REASON_NO_NODE
    auto reasonAt = [&](std::vector<int8_t> & reasons, nav_msgs::OccupancyGrid & grid,
                        double wx, double wy) -> int8_t {
      int ic = static_cast<int>(std::floor((wx - grid.info.origin.position.x) / 0.05));
      int ir = static_cast<int>(std::floor((wy - grid.info.origin.position.y) / 0.05));
      return reasons[static_cast<size_t>(ir * grid.info.width + ic)];
    };
    const int8_t seam_reason = reasonAt(reasons_no_edge, g_no_edge, 0.25, 0.28);
    const int8_t free_reason = reasonAt(reasons_with_edge, g_with_edge, 0.25, 0.28);
    const int8_t far_reason = reasonAt(reasons_no_edge, g_no_edge, -1.2, -1.2);
    std::cout << "  Reason test: seam=" << static_cast<int>(seam_reason)
              << " (expect " << static_cast<int>(elevation_costmap::REASON_SEAM) << "), free=" << static_cast<int>(free_reason)
              << " (expect " << static_cast<int>(elevation_costmap::REASON_FREE_NODE) << "), far=" << static_cast<int>(far_reason)
              << " (expect " << static_cast<int>(elevation_costmap::REASON_NO_NODE) << ")" << std::endl;
    assert(seam_reason == elevation_costmap::REASON_SEAM);
    assert(free_reason == elevation_costmap::REASON_FREE_NODE);
    assert(far_reason == elevation_costmap::REASON_NO_NODE);

    // 胜出节点 id: 自由格=实际盖章节点(0或1), 缝格=-1, 远角=-1
    auto idAt = [&](std::vector<int32_t> & ids, nav_msgs::OccupancyGrid & grid,
                    double wx, double wy) -> int32_t {
      int ic = static_cast<int>(std::floor((wx - grid.info.origin.position.x) / 0.05));
      int ir = static_cast<int>(std::floor((wy - grid.info.origin.position.y) / 0.05));
      return ids[static_cast<size_t>(ir * grid.info.width + ic)];
    };
    const int32_t free_id = idAt(ids_with_edge, g_with_edge, 0.25, 0.28);
    const int32_t seam_id = idAt(ids_no_edge, g_no_edge, 0.25, 0.28);
    const int32_t far_id = idAt(ids_no_edge, g_no_edge, -1.2, -1.2);
    std::cout << "  NodeId test: free_id=" << free_id << " (expect 0/1), seam_id=" << seam_id
              << " (expect -1), far_id=" << far_id << " (expect -1)" << std::endl;
    assert(free_id == 0 || free_id == 1);
    assert(seam_id == -1);
    assert(far_id == -1);
  }

  // 10. 地毯连通性验证: 图上联通但地毯上不连通的自由区 (绕道图外到达的透印) 转致命
  {
    // 机器人踩在节点 0 (0,0,0); 节点 1 (1.5,0,0) 与节点 0 有边 (桥),
    // 但两者之间的地毯格无任何节点覆盖 —— 自由区在地毯上互不连通
    elevation_planner::ManifoldGraph g3;
    g3.initSpatialGrid(0.10, -5.0, -5.0, 100, 100);
    for (int k = 0; k < 2; ++k)
    {
      elevation_planner::GraphNode n;
      n.x = k == 0 ? 0.0f : 1.5f;
      n.y = 0.0f; n.z = 0.0f;
      n.traversability = 0.0f; n.headroom = 2.0f;
      n.row = 50; n.col = 50 + k * 15; n.layer_id = 0;
      g3.addNode(n);
    }
    g3.addEdge(0, 1, 0.1f); g3.addEdge(1, 0, 0.1f);
    g3.finalizeCSR();

    elevation_costmap::ManifoldCostmapBuilderConfig cfg_u;
    cfg_u.map_width = 3.0; cfg_u.map_length = 3.0; cfg_u.resolution = 0.05;
    elevation_costmap::ManifoldCostmapBuilder builder_u(cfg_u);
    geometry_msgs::Pose bot_u;
    bot_u.position.x = 0.0; bot_u.position.y = 0.0; bot_u.position.z = 0.0;
    bot_u.orientation.w = 1.0;

    nav_msgs::OccupancyGrid grid_u;
    geometry_msgs::TransformStamped tf_u;
    std::vector<int8_t> reasons_u;
    assert(builder_u.buildCostmap(g3, bot_u, {}, grid_u, tf_u, &reasons_u));

    auto costAtU = [&](double wx, double wy) -> int {
      int ic = static_cast<int>(std::floor((wx - grid_u.info.origin.position.x) / 0.05));
      int ir = static_cast<int>(std::floor((wy - grid_u.info.origin.position.y) / 0.05));
      return grid_u.data[static_cast<size_t>(ir * grid_u.info.width + ic)];
    };
    const int near_robot = costAtU(0.05, 0.0);   // 机器人区域: 保持自由
    const int far_island = costAtU(1.5, 0.0);    // 桥对端孤立区: 透印, 转致命
    std::cout << "  Unreached test: robot-region cell = " << near_robot
              << " (expect 0), isolated bridge-end cell = " << far_island
              << " (expect 100)" << std::endl;
    assert(near_robot == 0);
    assert(far_island == 100);
  }

  // ---------------------------------------------------------------------------
  // 6. 拓扑图真实连通性与走廊校验 (TrajectoryValidator)
  // ---------------------------------------------------------------------------
  elevation_planner::ManifoldGraph g_cliff;
  {
    // 将 graph 注册到 GraphStore
    auto shared_g = std::make_shared<elevation_planner::ManifoldGraph>(graph);
    elevation_planner::GraphStore::instance().setGlobalGraph(shared_g);

    // 测试 1: 沿楼梯正面逐级攀爬轨迹 (节点有拓扑边连通) -> 必须合法通过
    std::vector<Eigen::Vector2d> valid_traj = {
      Eigen::Vector2d(0.0, 0.0),
      Eigen::Vector2d(0.1, 0.0),
      Eigen::Vector2d(0.2, 0.0),
      Eigen::Vector2d(0.4, 0.0)
    };
    std::string reason_valid;
    bool res_valid = elevation_planner::TrajectoryValidator::validate(
        valid_traj, 0.0, std::numeric_limits<double>::quiet_NaN(),
        Eigen::Vector2d::Zero(), 0.25, std::numeric_limits<double>::quiet_NaN(), &reason_valid);
    std::cout << "  TrajectoryValidator valid stair climb: res = " << res_valid
              << ", reason = " << reason_valid << " (expect 1)" << std::endl;
    assert(res_valid == true);

    // 构造一个楼梯侧面场景: 楼梯台阶相互连通，而在侧方 y=0.8 处添加平地孤立节点 (无拓扑边连到楼梯台阶)
    g_cliff.initSpatialGrid(0.10, -5.0, -5.0, 100, 100);
    for (int r = 0; r <= 30; ++r)
    {
      double x = r * 0.10;
      double z = r * 0.08;
      for (int c = -5; c <= 5; ++c)
      {
        double y = c * 0.10;
        elevation_planner::GraphNode node;
        node.x = static_cast<float>(x);
        node.y = static_cast<float>(y);
        node.z = static_cast<float>(z);
        node.traversability = 0.0f;
        node.headroom = 2.0f;
        node.row = 50 + r;
        node.col = 50 + c;
        node.layer_id = 0;
        g_cliff.addNode(node);
      }
    }
    // 添加侧方平地孤立节点
    elevation_planner::GraphNode side_ground;
    side_ground.x = 0.2f;
    side_ground.y = 0.8f;
    side_ground.z = 0.0f; // 平地
    side_ground.traversability = 0.0f;
    side_ground.headroom = 2.0f;
    side_ground.row = 52;
    side_ground.col = 58;
    side_ground.layer_id = 0;
    g_cliff.addNode(side_ground);

    for (int r = 0; r <= 30; ++r)
      for (int c = -5; c <= 5; ++c)
      {
        const int id = r * 11 + (c + 5);
        if (r < 30) { g_cliff.addEdge(id, id + 11, 0.1f); g_cliff.addEdge(id + 11, id, 0.1f); }
        if (c < 5)  { g_cliff.addEdge(id, id + 1, 0.1f);  g_cliff.addEdge(id + 1, id, 0.1f); }
      }
    g_cliff.finalizeCSR();
    elevation_planner::GraphStore::instance().setGlobalGraph(std::make_shared<elevation_planner::ManifoldGraph>(g_cliff));

    // 测试 2: 从平地侧面跨向楼梯台阶 (0.2, 0.8, z=0.0) -> (0.2, 0.4, z=0.16)
    // 空间高差仅 0.16m <= 0.25m, 但拓扑无边 -> 必须被拒绝且返回 topological_disconnected
    std::vector<Eigen::Vector2d> side_climb_traj = {
      Eigen::Vector2d(0.2, 0.8),
      Eigen::Vector2d(0.2, 0.4)
    };
    std::string reason_invalid;
    bool res_invalid = elevation_planner::TrajectoryValidator::validate(
        side_climb_traj, 0.0, std::numeric_limits<double>::quiet_NaN(),
        Eigen::Vector2d::Zero(), 0.25, std::numeric_limits<double>::quiet_NaN(), &reason_invalid);
    std::cout << "  TrajectoryValidator side cliff jump: res = " << res_invalid
              << ", reason = " << reason_invalid << " (expect 0, topological_disconnected)" << std::endl;
    assert(res_invalid == false);
    assert(reason_invalid == "topological_disconnected");
  }

  // -------------------------------------------------------------
  // 测试 8: TopologicalCorridorGenerator 拓扑管道生成测试
  // -------------------------------------------------------------
  {
    // 路径为中轴线: x: 0.0 -> 2.0, y = 0.0, z: 0.0 -> 1.6
    std::vector<geometry_msgs::PoseStamped> path;
    for (int i = 0; i <= 20; ++i)
    {
      geometry_msgs::PoseStamped ps;
      ps.pose.position.x = i * 0.10;
      ps.pose.position.y = 0.0;
      ps.pose.position.z = i * 0.08;
      path.push_back(ps);
    }

    // 生成半宽为 3.0m 的管道 (楼梯总宽 1.0m, 到悬空边缘自然截断)
    double corridor_r = 3.0;
    auto corridor = elevation_planner::TopologicalCorridorGenerator::generate(graph, path, corridor_r);
    std::cout << "  TopologicalCorridor generated with " << corridor.size()
              << " nodes (radius=" << corridor.radius << "m)" << std::endl;
    assert(corridor.size() > 0);
    assert(corridor.radius == 3.0);

    // 验证中轴线上所有节点都在管道内
    uint32_t center_id = 0;
    assert(graph.findClosestNode(0.5, 0.0, 0.4, center_id, 0.2, 0.2));
    assert(corridor.isInCorridor(center_id));

    // 验证楼梯边缘节点 (y = 0.5) 也在管道内 (因为 0.5m <= 3.0m 且拓扑连通)
    uint32_t edge_id = 0;
    assert(graph.findClosestNode(0.5, 0.5, 0.4, edge_id, 0.2, 0.2));
    assert(corridor.isInCorridor(edge_id));

    // 验证管道遇悬空自然停止: 在 g_cliff 中，side_ground (y=0.8) 无连通边
    auto corridor_cliff = elevation_planner::TopologicalCorridorGenerator::generate(g_cliff, path, corridor_r);
    uint32_t side_ground_id = 0;
    assert(g_cliff.findClosestNode(0.2, 0.8, 0.0, side_ground_id, 0.2, 0.2));
    // side_ground 虽然在 0.8m 水平距离内 (< 3.0m)，但由于无拓扑连通边 (悬空隔离)，绝不能进入管道！
    bool side_in = corridor_cliff.isInCorridor(side_ground_id);
    std::cout << "  TopologicalCorridor cliff isolation test: side_ground in corridor = "
              << side_in << " (expect 0)" << std::endl;
    assert(!side_in);
  }

  // -------------------------------------------------------------------------
  // Test 9: 管道动态结合障碍物阻断测试 (Obstacle-Carved Corridor Test)
  // -------------------------------------------------------------------------
  {
    std::vector<geometry_msgs::PoseStamped> path;
    for (int i = 0; i <= 20; ++i)
    {
      geometry_msgs::PoseStamped ps;
      ps.pose.position.x = i * 0.10;
      ps.pose.position.y = 0.0;
      ps.pose.position.z = i * 0.08;
      path.push_back(ps);
    }
    // 模拟在 x in [0.4, 0.6] 区域存在障碍物
    auto is_obstacle = [](float x, float y, float z) -> bool {
      return (x >= 0.40f && x <= 0.60f);
    };

    auto corridor_obs = elevation_planner::TopologicalCorridorGenerator::generate(
        graph, path, 3.0, 6.0, 0.25, 0.45, is_obstacle);
    
    // 验证处于障碍物内部的节点绝不能进入管道
    uint32_t obs_nid = 0;
    assert(graph.findClosestNode(0.50, 0.0, 0.40, obs_nid, 0.2, 0.2));
    bool obs_in = corridor_obs.isInCorridor(obs_nid);
    std::cout << "  TopologicalCorridor obstacle blocking test: obstacle cell in corridor = "
              << obs_in << " (expect 0)" << std::endl;
    assert(!obs_in);

    // 验证起点处的非障碍物节点正常在管道内
    uint32_t start_nid = 0;
    assert(graph.findClosestNode(0.10, 0.0, 0.08, start_nid, 0.2, 0.2));
    assert(corridor_obs.isInCorridor(start_nid));
  }

  // -------------------------------------------------------------------------
  // Test 10: 管道局部前瞻截断测试 (Lookahead-Limited Corridor Test)
  // -------------------------------------------------------------------------
  {
    // 构建一条 5 米长的路径 (50 个航点)
    std::vector<geometry_msgs::PoseStamped> long_path;
    for (int i = 0; i <= 50; ++i)
    {
      geometry_msgs::PoseStamped ps;
      ps.pose.position.x = i * 0.10;
      ps.pose.position.y = 0.0;
      ps.pose.position.z = i * 0.08;
      long_path.push_back(ps);
    }
    // 前瞻限制为 0.5 米, 展开半径 0.2 米
    double lookahead = 0.5;
    double radius = 0.2;
    auto corridor_short = elevation_planner::TopologicalCorridorGenerator::generate(
        graph, long_path, radius, lookahead);
    
    // 远端航点 (x = 1.5m > lookahead + radius) 对应的节点绝不能在管道内
    uint32_t far_nid = 0;
    assert(graph.findClosestNode(1.50, 0.0, 1.20, far_nid, 0.2, 0.2));
    bool far_in = corridor_short.isInCorridor(far_nid);
    std::cout << "  TopologicalCorridor lookahead limit test: far node in corridor = "
              << far_in << " (expect 0)" << std::endl;
    assert(!far_in);

    // 前瞻范围内航点 (x = 0.2m) 对应的节点正常在管道内
    uint32_t near_nid = 0;
    assert(graph.findClosestNode(0.20, 0.0, 0.16, near_nid, 0.2, 0.2));
    assert(corridor_short.isInCorridor(near_nid));
  }

  // -------------------------------------------------------------------------
  // Test 11: 轨迹校验器在有效管道内的放行验证 (TrajectoryValidator Corridor Test)
  // -------------------------------------------------------------------------
  {
    std::vector<geometry_msgs::PoseStamped> path;
    for (int i = 0; i <= 20; ++i)
    {
      geometry_msgs::PoseStamped ps;
      ps.pose.position.x = i * 0.10;
      ps.pose.position.y = 0.0;
      ps.pose.position.z = i * 0.08;
      path.push_back(ps);
    }
    auto corridor = std::make_shared<elevation_planner::TopologicalCorridor>(
        elevation_planner::TopologicalCorridorGenerator::generate(graph, path, 3.0, 6.0));
    elevation_planner::GraphStore::instance().setTopologicalCorridor(corridor);

    // 沿中轴线正常上楼梯轨迹
    std::vector<Eigen::Vector2d> traj_poses;
    for (int i = 0; i <= 10; ++i)
    {
      traj_poses.emplace_back(i * 0.10, 0.0);
    }
    std::string reason;
    bool valid = elevation_planner::TrajectoryValidator::validate(
        traj_poses, 0.0, 0.8, Eigen::Vector2d(1.0, 0.0), 0.25, std::numeric_limits<double>::quiet_NaN(), &reason);
    std::cout << "  TrajectoryValidator inside corridor test: valid = " << valid
              << ", reason = " << reason << " (expect valid=1)" << std::endl;
    assert(valid);
  }

  std::cout << "[TestManifoldCostmapBuilder] ALL ASSERTIONS PASSED!" << std::endl;
  return 0;
}
