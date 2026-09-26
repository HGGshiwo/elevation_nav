#include <gtest/gtest.h>
#include "elevation_planner_core/path_simplifier.hpp"
#include "elevation_planner_core/manifold_search.hpp"
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

/// @brief 规则网格图: rows x cols, 间距 spacing, z = zfn(r,c), valid 过滤格子, 8 邻域连边
/// @note  图的空间桶约定: row = x 栅格索引, col = y 栅格索引 (见 toGridIndex)
Fixture makeGrid(int rows, int cols, double spacing,
                 const std::function<double(int, int)>& zfn,
                 const std::function<bool(int, int)>& valid = nullptr)
{
  Fixture fx;
  fx.g.initSpatialGrid(spacing, 0.0, 0.0, cols, rows);  // rows_ 覆盖 x, cols_ 覆盖 y
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
      nd.row = c;  // x 栅格索引
      nd.col = r;  // y 栅格索引
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
          const float cost = static_cast<float>(std::sqrt(dxy * dxy + dz * dz));
          fx.g.addEdge(fx.id[r][c], fx.id[nr][nc], cost);
        }
      }
    }
  }
  fx.g.finalizeCSR();
  return fx;
}

/// @brief 同一 (x,y) 柱面叠两层节点的楼板 (模拟楼梯间多层投影)
Fixture makeTwoLayer(int cols, double spacing, double z0, double z1)
{
  Fixture fx;
  fx.g.initSpatialGrid(spacing, 0.0, 0.0, cols, 1);  // rows_ 覆盖 x
  fx.id.assign(2, std::vector<int32_t>(cols, -1));

  elevation_planner::GraphNode nd;
  nd.traversability = 0.0f;
  nd.headroom = 2.0f;
  nd.col = 0;  // y 栅格索引
  nd.y = 0.0f;
  for (int c = 0; c < cols; ++c)
  {
    for (int layer = 0; layer < 2; ++layer)
    {
      nd.x = static_cast<float>(c * spacing);
      nd.z = static_cast<float>(layer == 0 ? z0 : z1);
      nd.row = c;  // x 栅格索引
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

std::vector<uint32_t> toIds(const Fixture& fx, int r, int c0, int c1)
{
  std::vector<uint32_t> ids;
  const int dir = (c1 >= c0) ? 1 : -1;
  for (int c = c0; c != c1 + dir; c += dir)
    ids.push_back(static_cast<uint32_t>(fx.id[r][c]));
  return ids;
}

} // namespace

/// 平地: 直线联通, 长直线被大幅剪枝
TEST(ScLos, FlatGroundCollapses)
{
  Fixture fx = makeGrid(30, 60, 0.1, [](int, int) { return 0.0; });
  elevation_planner::PathSimplifier sim;
  sim.build(fx.g);

  auto path = toIds(fx, 10, 0, 55);  // 5.5m 直线
  EXPECT_TRUE(sim.losConnected(path.front(), path.back()));

  auto sparse = sim.simplifyPath(path, 0.8);
  EXPECT_LT(sparse.size(), 10u);
  EXPECT_EQ(sparse.front(), path.front());
  EXPECT_EQ(sparse.back(), path.back());
}

/// 沿楼梯方向: z 链逐级连续爬升, 直线联通且可剪枝
TEST(ScLos, AlongStairsConnected)
{
  // 每级进深 0.3m (3 列), 高 0.15m; 相邻列最大 dz=0.15 <= max_step_height
  Fixture fx = makeGrid(6, 60, 0.1, [](int, int c) { return 0.15 * (c / 3); });
  elevation_planner::PathSimplifier sim;
  sim.build(fx.g);

  auto path = toIds(fx, 3, 0, 59);
  EXPECT_TRUE(sim.losConnected(path.front(), path.back()));

  auto sparse = sim.simplifyPath(path, 0.8);
  EXPECT_LT(sparse.size(), 15u);
}

/// 横穿楼梯井: 中段无同层支撑, 直线不联通 (不可剪枝)
TEST(ScLos, AcrossStairwellRejected)
{
  Fixture fx = makeGrid(40, 20, 0.1,
                        [](int, int) { return 0.0; },
                        [](int r, int) { return r < 10 || r >= 30; });
  elevation_planner::PathSimplifier sim;
  sim.build(fx.g);

  auto within = toIds(fx, 5, 0, 9);
  EXPECT_TRUE(sim.losConnected(within.front(), within.back()));

  const uint32_t a = static_cast<uint32_t>(fx.id[9][10]);
  const uint32_t b = static_cast<uint32_t>(fx.id[30][10]);
  EXPECT_FALSE(sim.losConnected(a, b));
}

/// 同一 (x,y) 柱面叠两层: z 链锁定层身份, 层内联通, 跨层不可直线接通
TEST(ScLos, LayerChainDisambiguates)
{
  Fixture fx = makeTwoLayer(30, 0.1, 0.0, 0.5);
  elevation_planner::PathSimplifier sim;
  sim.build(fx.g);

  auto lower = toIds(fx, 0, 0, 25);
  EXPECT_TRUE(sim.losConnected(lower.front(), lower.back()));

  // 跨层: 2D 直线存在但 z 跳变 0.5 > 0.25, 中段无中间层支撑, 末端无跨层边
  const uint32_t a = static_cast<uint32_t>(fx.id[0][10]);
  const uint32_t b = static_cast<uint32_t>(fx.id[1][25]);
  EXPECT_FALSE(sim.losConnected(a, b));
}

/// 支撑豁口: 直线穿过无节点区被判不联通
TEST(ScLos, HoleBlocksLine)
{
  Fixture fx = makeGrid(20, 20, 0.1,
                        [](int, int) { return 0.0; },
                        [](int r, int c) { return !(r == 10 && c >= 8 && c <= 12); });
  elevation_planner::PathSimplifier sim;
  sim.build(fx.g);

  const uint32_t a = static_cast<uint32_t>(fx.id[10][5]);
  const uint32_t b = static_cast<uint32_t>(fx.id[10][15]);
  EXPECT_FALSE(sim.losConnected(a, b));
}

/// 连续重复 id 被剔除, 输出保序且首尾保留
TEST(ScLos, DedupAndEndpoints)
{
  Fixture fx = makeGrid(10, 20, 0.1, [](int, int) { return 0.0; });
  elevation_planner::PathSimplifier sim;
  sim.build(fx.g);

  auto path = toIds(fx, 5, 0, 15);
  path.insert(path.begin() + 5, path[5]);
  path.insert(path.begin() + 6, path[5]);
  auto sparse = sim.simplifyPath(path, 0.5);
  EXPECT_EQ(sparse.front(), path.front());
  EXPECT_EQ(sparse.back(), path.back());
  for (size_t i = 1; i < sparse.size(); ++i)
    EXPECT_NE(sparse[i], sparse[i - 1]);
}

/// 统一 A* 内核冒烟: 平地对角最短路 (边代价模式, 与全局配置一致)
TEST(AstarKernel, FlatDiagonalShortest)
{
  Fixture fx = makeGrid(20, 20, 0.1, [](int, int) { return 0.0; });
  elevation_planner::ManifoldAstarParams prm;  // 默认 = 全局模式
  std::vector<uint32_t> ids;
  ASSERT_TRUE(elevation_planner::manifoldAstarSearch(fx.g, fx.id[0][0], fx.id[15][15], prm, ids));
  EXPECT_EQ(ids.front(), static_cast<uint32_t>(fx.id[0][0]));
  EXPECT_EQ(ids.back(), static_cast<uint32_t>(fx.id[15][15]));
  EXPECT_EQ(ids.size(), 16u);  // 对角线 15 步最短
}

/// 统一 A* 内核: 局部绕障模式 (单步过滤 + 转角惩罚) 仍可达
TEST(AstarKernel, KinematicModeReachable)
{
  Fixture fx = makeGrid(20, 20, 0.1, [](int, int) { return 0.0; });
  elevation_planner::ManifoldAstarParams prm;
  prm.filter_single_step = true;
  prm.use_edge_cost = false;
  prm.turn_weight = 1.5;
  prm.h_use_3d = true;
  prm.h_z_weight = 2.0;
  prm.use_closed = false;
  prm.stale_push_limit = 200.0;
  std::vector<uint32_t> ids;
  ASSERT_TRUE(elevation_planner::manifoldAstarSearch(fx.g, fx.id[2][2], fx.id[15][17], prm, ids));
  EXPECT_EQ(ids.back(), static_cast<uint32_t>(fx.id[15][17]));
  EXPECT_EQ(ids.front(), static_cast<uint32_t>(fx.id[2][2]));
}

/// 支撑链: 平地上链沿直线, 相邻节点直接邻接, 首尾正确
TEST(ScLos, LosChainFlat)
{
  Fixture fx = makeGrid(10, 30, 0.1, [](int, int) { return 0.0; });
  elevation_planner::PathSimplifier sim;
  sim.build(fx.g);

  std::vector<uint32_t> chain;
  ASSERT_TRUE(sim.computeLosChain(fx.id[5][2], fx.id[5][20], chain));
  EXPECT_EQ(chain.front(), static_cast<uint32_t>(fx.id[5][2]));
  EXPECT_EQ(chain.back(), static_cast<uint32_t>(fx.id[5][20]));
  for (size_t i = 1; i < chain.size(); ++i)
    EXPECT_TRUE(chain[i] == chain[i - 1] || fx.g.isConnected(chain[i - 1], chain[i], 1));
}

/// 支撑链: 同柱双层上锁定正确的层 (链内节点 z 全为本层), 跨层无链
TEST(ScLos, LosChainLayerLocked)
{
  Fixture fx = makeTwoLayer(30, 0.1, 0.0, 0.5);
  elevation_planner::PathSimplifier sim;
  sim.build(fx.g);

  std::vector<uint32_t> chain;
  ASSERT_TRUE(sim.computeLosChain(fx.id[0][2], fx.id[0][20], chain));
  for (uint32_t nid : chain)
    EXPECT_NEAR(fx.g.getNode(nid).z, 0.0, 1e-6);  // 不混入 0.5 层

  std::vector<uint32_t> bad;
  EXPECT_FALSE(sim.computeLosChain(fx.id[0][2], fx.id[1][20], bad));
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
