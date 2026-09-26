#include <gtest/gtest.h>
#include "elevation_planner_core/manifold_graph.hpp"
#include "elevation_planner_core/path_simplifier.hpp"
#include "elevation_local_planner/sfc_corridor.hpp"
#include <cmath>
#include <functional>
#include <vector>

namespace
{

struct Fixture
{
  elevation_planner::ManifoldGraph g;
  std::vector<std::vector<int32_t>> id;
};

/// @brief 规则网格图 (图的空间桶约定: row = x 栅格索引, col = y 栅格索引)
Fixture makeGrid(int rows, int cols, double spacing,
                 const std::function<double(int, int)>& zfn,
                 const std::function<bool(int, int)>& valid = nullptr)
{
  Fixture fx;
  fx.g.initSpatialGrid(spacing, 0.0, 0.0, cols, rows);
  fx.id.assign(rows, std::vector<int32_t>(cols, -1));

  elevation_planner::GraphNode nd;
  nd.traversability = 0.0f;
  nd.headroom = 2.0f;
  for (int r = 0; r < rows; ++r)
  {
    for (int c = 0; c < cols; ++c)
    {
      if (valid && !valid(r, c)) continue;
      nd.x = static_cast<float>(c * spacing);
      nd.y = static_cast<float>(r * spacing);
      nd.z = static_cast<float>(zfn(r, c));
      nd.row = c;
      nd.col = r;
      fx.id[r][c] = static_cast<int32_t>(fx.g.addNode(nd));
    }
  }
  for (int r = 0; r < rows; ++r)
  {
    for (int c = 0; c < cols; ++c)
    {
      if (fx.id[r][c] < 0) continue;
      for (int dr = -1; dr <= 1; ++dr)
      {
        for (int dc = -1; dc <= 1; ++dc)
        {
          if (dr == 0 && dc == 0) continue;
          const int nr = r + dr, nc = c + dc;
          if (nr < 0 || nr >= rows || nc < 0 || nc >= cols || fx.id[nr][nc] < 0) continue;
          const double dz = zfn(nr, nc) - zfn(r, c);
          const double dxy = spacing * ((dr != 0 && dc != 0) ? 1.41421356 : 1.0);
          fx.g.addEdge(fx.id[r][c], fx.id[nr][nc], static_cast<float>(std::sqrt(dxy * dxy + dz * dz)));
        }
      }
    }
  }
  fx.g.finalizeCSR();
  return fx;
}

/// @brief 同一 (x,y) 柱面叠两层楼板 (楼梯间多层投影)
Fixture makeTwoLayer(int cols, double spacing, double z0, double z1)
{
  Fixture fx;
  fx.g.initSpatialGrid(spacing, 0.0, 0.0, cols, 1);
  fx.id.assign(2, std::vector<int32_t>(cols, -1));

  elevation_planner::GraphNode nd;
  nd.traversability = 0.0f;
  nd.headroom = 2.0f;
  nd.col = 0;
  nd.y = 0.0f;
  for (int c = 0; c < cols; ++c)
  {
    for (int layer = 0; layer < 2; ++layer)
    {
      nd.x = static_cast<float>(c * spacing);
      nd.z = static_cast<float>(layer == 0 ? z0 : z1);
      nd.row = c;
      fx.id[layer][c] = static_cast<int32_t>(fx.g.addNode(nd));
    }
  }
  for (int layer = 0; layer < 2; ++layer)
  {
    for (int c = 0; c + 1 < cols; ++c)
    {
      fx.g.addEdge(fx.id[layer][c], fx.id[layer][c + 1], static_cast<float>(spacing));
      fx.g.addEdge(fx.id[layer][c + 1], fx.id[layer][c], static_cast<float>(spacing));
    }
  }
  fx.g.finalizeCSR();
  return fx;
}

/// @brief 节点是否满足走廊全部线性约束 (A*q <= b)
bool satisfiesConstraint(const elevation_local_planner::ConvexCorridor2D& cor, double x, double y)
{
  if (cor.constraint.A.rows() == 0) return true;
  Eigen::Vector2d q(x, y);
  Eigen::VectorXd v = cor.constraint.A * q - cor.constraint.b;
  for (int j = 0; j < v.size(); ++j)
    if (v(j) > 1e-6) return false;
  return true;
}

} // namespace

/// 平地段式走廊: 香肠形状 (沿线扩散), 端点与中点满足线性约束, 顶点数正常
TEST(SfcSegment, FlatSausageShape)
{
  Fixture fx = makeGrid(30, 40, 0.1, [](int, int) { return 0.0; });
  elevation_planner::PathSimplifier sim;
  sim.build(fx.g);

  std::vector<uint32_t> ids;
  for (int c = 0; c <= 20; ++c) ids.push_back(static_cast<uint32_t>(fx.id[15][c]));

  auto corridors = elevation_local_planner::SFCGenerator::generateSegmentCorridors(
      ids, fx.g, sim, 0.50, 0.25, 0.35);
  ASSERT_EQ(corridors.size(), ids.size());

  // 走廊 1 = 段 (n00 -> n01) 的链种子扩散; 检查其约束覆盖两端与中点
  const auto& cor1 = corridors[1];
  EXPECT_GE(cor1.constraint.vertices.size(), 3u);
  EXPECT_TRUE(satisfiesConstraint(cor1, 0.0, 1.5));    // W_0
  EXPECT_TRUE(satisfiesConstraint(cor1, 0.1, 1.5));    // W_1
  EXPECT_TRUE(satisfiesConstraint(cor1, 0.05, 1.5));   // 中点

  // BFS 预算 0.5m: 沿线远端 (距段 >0.5m) 不应被吸入走廊 1 的节点集
  for (const auto& nd : cor1.node_pts)
  {
    const double d = std::hypot(nd.x - 0.05, nd.y - 1.5);
    EXPECT_LE(d, 0.55 + 1e-6) << "node (" << nd.x << "," << nd.y << ") beyond budget";
  }
}

/// 豁口: 段在平台内部, 扩散不跨过楼梯井 (节点全部属于本平台)
TEST(SfcSegment, NoLeakAcrossGap)
{
  Fixture fx = makeGrid(40, 20, 0.1,
                        [](int, int) { return 0.0; },
                        [](int r, int) { return r < 10 || r >= 30; });
  elevation_planner::PathSimplifier sim;
  sim.build(fx.g);

  std::vector<uint32_t> ids;
  for (int c = 0; c <= 6; ++c) ids.push_back(static_cast<uint32_t>(fx.id[5][c]));

  auto corridors = elevation_local_planner::SFCGenerator::generateSegmentCorridors(
      ids, fx.g, sim, 0.50, 0.25, 0.35);
  ASSERT_EQ(corridors.size(), ids.size());

  // 平台 A 的 y 范围 [0, 0.9]; 走廊节点不得跨过豁口 (y >= 2.9 为平台 B)
  for (size_t i = 1; i < corridors.size(); ++i)
  {
    for (const auto& nd : corridors[i].node_pts)
    {
      EXPECT_LT(nd.y, 2.5) << "corridor " << i << " leaked across the gap";
    }
  }
}

/// 同柱双层: 种子链锁定 z=0 层, BFS 沿单步边扩散, 走廊不含另一层节点 (无需高度过滤)
TEST(SfcSegment, LayerIsolatedByConnectivity)
{
  Fixture fx = makeTwoLayer(30, 0.1, 0.0, 0.5);
  elevation_planner::PathSimplifier sim;
  sim.build(fx.g);

  std::vector<uint32_t> ids;
  for (int c = 0; c <= 20; ++c) ids.push_back(static_cast<uint32_t>(fx.id[0][c]));

  auto corridors = elevation_local_planner::SFCGenerator::generateSegmentCorridors(
      ids, fx.g, sim, 0.50, 0.25, 0.35);
  ASSERT_EQ(corridors.size(), ids.size());

  for (size_t i = 1; i < corridors.size(); ++i)
  {
    for (const auto& nd : corridors[i].node_pts)
    {
      EXPECT_NEAR(nd.z, 0.0, 1e-6) << "corridor " << i << " leaked to z=0.5 layer";
    }
  }
}

/// 楼梯段: 沿爬升方向的段式走廊覆盖各级踏面, 端点在约束内
TEST(SfcSegment, StairSegmentCoversTreads)
{
  // 每级进深 0.3m (3 列), 高 0.15m
  Fixture fx = makeGrid(10, 30, 0.1, [](int, int c) { return 0.15 * (c / 3); });
  elevation_planner::PathSimplifier sim;
  sim.build(fx.g);

  std::vector<uint32_t> ids;
  for (int c = 0; c <= 24; ++c) ids.push_back(static_cast<uint32_t>(fx.id[5][c]));

  auto corridors = elevation_local_planner::SFCGenerator::generateSegmentCorridors(
      ids, fx.g, sim, 0.50, 0.25, 0.35);
  ASSERT_EQ(corridors.size(), ids.size());

  // 每条段走廊的约束必须罩住该段两端 (z 由各自踏面高度附近取)
  for (size_t i = 1; i < corridors.size(); ++i)
  {
    const auto& prev = fx.g.getNode(ids[i - 1]);
    const auto& curr = fx.g.getNode(ids[i]);
    EXPECT_TRUE(satisfiesConstraint(corridors[i], prev.x, prev.y));
    EXPECT_TRUE(satisfiesConstraint(corridors[i], curr.x, curr.y));
    EXPECT_GE(corridors[i].constraint.vertices.size(), 3u);
  }
}

/// 转角交集拼接: 内部走廊约束 = 相邻两段行拼接; 切角线上的点被交集拒绝; 转角点始终可行
TEST(SfcSegment, CornerIntersectionStacking)
{
  Fixture fx = makeGrid(30, 30, 0.1, [](int, int) { return 0.0; });
  elevation_planner::PathSimplifier sim;
  sim.build(fx.g);

  // L 形稀疏航点: W_0=(1.0,1.0) -> W_1=(2.0,1.0) -> W_2=(2.0,2.0)
  std::vector<uint32_t> ids;
  ids.push_back(static_cast<uint32_t>(fx.id[10][10]));
  ids.push_back(static_cast<uint32_t>(fx.id[10][20]));
  ids.push_back(static_cast<uint32_t>(fx.id[20][20]));

  auto corridors = elevation_local_planner::SFCGenerator::generateSegmentCorridors(
      ids, fx.g, sim, 0.50, 0.25, 0.35);
  ASSERT_EQ(corridors.size(), 3u);
  const int rows_c1 = corridors[1].constraint.A.rows();
  const int rows_c2 = corridors[2].constraint.A.rows();

  // 各自单独可行: W_0 在进段走廊 C_1 内, W_2 在出段走廊 C_2 内
  EXPECT_TRUE(satisfiesConstraint(corridors[1], 1.0, 1.0));
  EXPECT_TRUE(satisfiesConstraint(corridors[2], 2.0, 2.0));

  // 切角对角线 W_0 -> W_2 上的点 (1.3,1.3): 只满足 C_1, 不满足 C_2 —— 老做法在此放行
  EXPECT_TRUE(satisfiesConstraint(corridors[1], 1.3, 1.3));
  EXPECT_FALSE(satisfiesConstraint(corridors[2], 1.3, 1.3));

  // 拼接后: 该点被交集拒绝, 转角 W_1 始终在交集内
  elevation_local_planner::SFCGenerator::stackAdjacentCorridors(corridors);
  EXPECT_EQ(corridors[1].constraint.A.rows(), rows_c1 + rows_c2);
  EXPECT_FALSE(satisfiesConstraint(corridors[1], 1.3, 1.3));

  const auto& w1 = fx.g.getNode(ids[1]);
  EXPECT_TRUE(satisfiesConstraint(corridors[1], w1.x, w1.y));
  EXPECT_TRUE(satisfiesConstraint(corridors[1], 2.0, 1.0));
}

/// 机器人状态种子: 状态点在第二段 BFS 预算之外, 并入种子/凸包点后被 C_2 覆盖 (转角交集可行)
TEST(SfcSegment, RobotStateSeedingCoversAnchor)
{
  Fixture fx = makeGrid(10, 40, 0.1, [](int, int) { return 0.0; });
  elevation_planner::PathSimplifier sim;
  sim.build(fx.g);

  // 稀疏航点: W_0=(1.0,1.0) -> W_1=(2.0,1.0) -> W_2=(3.0,1.0)
  std::vector<uint32_t> ids;
  ids.push_back(static_cast<uint32_t>(fx.id[1][10]));
  ids.push_back(static_cast<uint32_t>(fx.id[1][20]));
  ids.push_back(static_cast<uint32_t>(fx.id[1][30]));

  // 机器人状态点 (0.9,1.0) 附近: 距第二段 (2,1)->(3,1) 的链 ~1.1m, 正常 BFS 够不到
  std::vector<Eigen::Vector2d> state_points = {Eigen::Vector2d(0.9, 1.0), Eigen::Vector2d(0.82, 1.0)};

  auto corridors = elevation_local_planner::SFCGenerator::generateSegmentCorridors(
      ids, fx.g, sim, 0.50, 0.25, 0.35, state_points, 0.0);
  ASSERT_EQ(corridors.size(), 3u);

  // 对照组: 无状态种子时 C_2 覆盖不到 0.9
  auto corridors_plain = elevation_local_planner::SFCGenerator::generateSegmentCorridors(
      ids, fx.g, sim, 0.50, 0.25, 0.35);
  EXPECT_FALSE(satisfiesConstraint(corridors_plain[2], 0.9, 1.0));

  // 并入状态种子 + 凸包点后: C_2 / C_1 均覆盖状态点 (q_2 转角交集可行的前提)
  EXPECT_TRUE(satisfiesConstraint(corridors[2], 0.9, 1.0));
  EXPECT_TRUE(satisfiesConstraint(corridors[2], 0.82, 1.0));
  EXPECT_TRUE(satisfiesConstraint(corridors[1], 0.9, 1.0));
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
