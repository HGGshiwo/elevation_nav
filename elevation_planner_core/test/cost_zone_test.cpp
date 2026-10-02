#include <gtest/gtest.h>
#include "elevation_planner_core/cloud_graph_builder.hpp"
#include "elevation_planner_core/manifold_search.hpp"
#include <cmath>
#include <vector>

namespace
{

using elevation_planner::CloudGraphBuilder;
using elevation_planner::ColumnSurface;
using elevation_planner::ColumnTable;
using elevation_planner::CostZone;
using elevation_planner::GraphBuildConfig;
using elevation_planner::GraphNode;
using elevation_planner::ManifoldGraph;

constexpr double kRes = 0.10;

/// 10x10 全地面 (z=0) 柱表, 叠加特征:
///  - (5,5) 高墙: [0, 1.2] 单曲面 (行走层无节点, 墙顶节点 z=1.2)
///  - (2,7) 低平台: [0, 0.2] 单曲面 (攀爬包络内的台阶, 不判墙)
///  - (8,2) 低顶: 地面 + 天花板 [0.25, 0.3] (净空 0.25 < dog_height)
ColumnTable makeTable()
{
  ColumnTable t;
  t.extent.resolution = kRes;
  t.extent.min_x = 0.0;
  t.extent.min_y = 0.0;
  t.extent.rows = 10;
  t.extent.cols = 10;
  t.cells.assign(static_cast<size_t>(t.extent.rows * t.extent.cols), {});

  for (int r = 0; r < 10; ++r)
    for (int c = 0; c < 10; ++c)
      t.cells[static_cast<size_t>(r * 10 + c)] = {{0.0f, -0.05f, 10}};

  t.cells[static_cast<size_t>(5 * 10 + 5)] = {{1.2f, 0.0f, 10}};   // 高墙
  t.cells[static_cast<size_t>(2 * 10 + 7)] = {{0.2f, 0.0f, 10}};   // 低平台
  t.cells[static_cast<size_t>(8 * 10 + 2)] = {{0.0f, -0.05f, 10},
                                              {0.3f, 0.25f, 5}};   // 低顶
  return t;
}

GraphBuildConfig makeConfig(bool ring_enabled = true)
{
  GraphBuildConfig cfg;
  cfg.resolution = kRes;
  cfg.max_step_height = 0.25;
  cfg.max_stride_length = 0.35;
  cfg.dog_height = 0.45;
  cfg.cluster_height_diff = 0.08;
  cfg.min_cluster_points = 2;
  cfg.footprint_radius = 0.26;
  cfg.body_hard_radius = 0.17;
  cfg.inflation_radius = 0.50;
  cfg.body_hard_ring_enabled = ring_enabled;
  return cfg;
}

const GraphNode * nodeAt(const ManifoldGraph & g, int r, int c, int layer = 0)
{
  for (uint32_t nid : g.getSpatialCellNodes(r, c)) {
    const auto & nd = g.getNode(nid);
    if (nd.layer_id == layer) return &g.getNode(nid);
  }
  return nullptr;
}

TEST(CostZoneTest, WallProducesBodyHardRingAndSoftBand)
{
  auto table = makeTable();
  ManifoldGraph g;
  CloudGraphBuilder builder(makeConfig());
  ASSERT_TRUE(builder.buildGraphFromColumnTable(table, g));

  // 机体硬禁行环: 墙邻 8 格 (格心距 0.10/0.141 < 0.17)
  static const int ring_cells[8][2] = {{4,4},{4,5},{4,6},{5,4},{5,6},{6,4},{6,5},{6,6}};
  for (const auto & rc : ring_cells) {
    const GraphNode * nd = nodeAt(g, rc[0], rc[1]);
    ASSERT_NE(nd, nullptr) << "cell (" << rc[0] << "," << rc[1] << ")";
    EXPECT_EQ(nd->cost_zone, static_cast<uint8_t>(CostZone::BODY_HARD)) << "(" << rc[0] << "," << rc[1] << ")";
    EXPECT_NEAR(nd->traversability, 1.0f, 1e-6f);
    EXPECT_TRUE(nd->hardBlocked());
    EXPECT_NE(nd->flags & elevation_planner::node_flags::BLOCK_LATERAL, 0);
    EXPECT_EQ(nd->edge_count, 0u);  // 建边要求两端 trav<0.95, 环节点天然无边
  }

  // 软代价带: 距墙 0.2m / 0.4m 衰减单调
  const GraphNode * soft1 = nodeAt(g, 5, 3);  // d=0.2
  ASSERT_NE(soft1, nullptr);
  EXPECT_EQ(soft1->cost_zone, static_cast<uint8_t>(CostZone::SOFT));
  EXPECT_NEAR(soft1->traversability, 0.9f * (0.5f - 0.2f) / 0.33f, 0.02f);
  EXPECT_FALSE(soft1->hardBlocked());

  const GraphNode * soft2 = nodeAt(g, 5, 1);  // d=0.4
  ASSERT_NE(soft2, nullptr);
  EXPECT_EQ(soft2->cost_zone, static_cast<uint8_t>(CostZone::SOFT));
  EXPECT_LT(soft2->traversability, soft1->traversability);

  // 自由区: 距墙 0.707m, 无任何叠加
  const GraphNode * far = nodeAt(g, 0, 0);
  ASSERT_NE(far, nullptr);
  EXPECT_EQ(far->cost_zone, static_cast<uint8_t>(CostZone::FREE));
  EXPECT_NEAR(far->traversability, 0.0f, 1e-6f);
}

TEST(CostZoneTest, WallTopIsClimbableFreeNode)
{
  auto table = makeTable();
  ManifoldGraph g;
  CloudGraphBuilder builder(makeConfig());
  ASSERT_TRUE(builder.buildGraphFromColumnTable(table, g));

  // 墙顶节点是可行踏面 (攀爬包络判据 k=0 不判自身为墙)
  const GraphNode * top = nodeAt(g, 5, 5);
  ASSERT_NE(top, nullptr);
  EXPECT_NEAR(top->z, 1.2f, 1e-6f);
  EXPECT_EQ(top->cost_zone, static_cast<uint8_t>(CostZone::FREE));
}

TEST(CostZoneTest, LowPlatformDoesNotProduceRing)
{
  auto table = makeTable();
  ManifoldGraph g;
  CloudGraphBuilder builder(makeConfig());
  ASSERT_TRUE(builder.buildGraphFromColumnTable(table, g));

  // 0.2m 平台在攀爬包络内 (z_top 0.2 < 1*max_step_height 0.25), 周边不产生机体硬环。
  // (注: 场地另一端的墙的软代价带会合法延伸到这些格子, 故只断言无硬环, 不断言 FREE)
  static const int nb[8][2] = {{1,7},{3,7},{2,6},{2,8},{1,6},{1,8},{3,6},{3,8}};
  for (const auto & rc : nb) {
    const GraphNode * nd = nodeAt(g, rc[0], rc[1]);
    ASSERT_NE(nd, nullptr) << "cell (" << rc[0] << "," << rc[1] << ")";
    EXPECT_LT(nd->cost_zone, static_cast<uint8_t>(CostZone::BODY_HARD)) << "(" << rc[0] << "," << rc[1] << ")";
    EXPECT_LT(nd->traversability, 0.95f);
    EXPECT_FALSE(nd->hardBlocked());
  }
  // 平台顶可通行且与邻格地面在单步极限内 (平台自身不判墙; 软带叠加不影响可通行性)
  const GraphNode * plat = nodeAt(g, 2, 7);
  ASSERT_NE(plat, nullptr);
  EXPECT_EQ(plat->cost_zone, static_cast<uint8_t>(CostZone::SOFT)); // 仅墙的软带叠加, 非环
  EXPECT_FALSE(plat->hardBlocked());
  const GraphNode * beside = nodeAt(g, 2, 6);
  ASSERT_NE(beside, nullptr);
  EXPECT_TRUE(g.isConnected(beside->id, plat->id, 1));
}

TEST(CostZoneTest, LowHeadroomIsForbiddenNotBodyHard)
{
  auto table = makeTable();
  ManifoldGraph g;
  CloudGraphBuilder builder(makeConfig());
  ASSERT_TRUE(builder.buildGraphFromColumnTable(table, g));

  const GraphNode * nd = nodeAt(g, 8, 2);  // 天花板下地面节点
  ASSERT_NE(nd, nullptr);
  EXPECT_EQ(nd->cost_zone, static_cast<uint8_t>(CostZone::FORBIDDEN));
  EXPECT_NEAR(nd->traversability, 1.0f, 1e-6f);
  EXPECT_NE(nd->flags & elevation_planner::node_flags::BLOCK_HEADROOM, 0);
}

TEST(CostZoneTest, RingDisabledRevertsToSoftOnly)
{
  auto table = makeTable();
  ManifoldGraph g;
  CloudGraphBuilder builder(makeConfig(/*ring_enabled=*/false));
  ASSERT_TRUE(builder.buildGraphFromColumnTable(table, g));

  // 关闭硬环: 墙邻格退回贴墙软代价上限 (0.9), 不再硬排除
  const GraphNode * nd = nodeAt(g, 5, 4);
  ASSERT_NE(nd, nullptr);
  EXPECT_EQ(nd->cost_zone, static_cast<uint8_t>(CostZone::SOFT));
  EXPECT_NEAR(nd->traversability, 0.9f, 0.02f);
  EXPECT_FALSE(nd->hardBlocked());
}

TEST(CostZoneTest, AstarHardExcludesRingNodes)
{
  auto table = makeTable();
  ManifoldGraph g;
  CloudGraphBuilder builder(makeConfig());
  ASSERT_TRUE(builder.buildGraphFromColumnTable(table, g));

  const GraphNode * start = nodeAt(g, 0, 0);
  const GraphNode * ring = nodeAt(g, 5, 4);
  ASSERT_NE(start, nullptr);
  ASSERT_NE(ring, nullptr);

  // 环节点无边且被内核硬排除: 搜索不可达 (开集耗尽)
  elevation_planner::ManifoldAstarParams prm;
  std::vector<uint32_t> path;
  std::string fail_reason;
  EXPECT_FALSE(elevation_planner::manifoldAstarSearch(g, start->id, ring->id, prm, path, &fail_reason));
  EXPECT_NE(fail_reason.find("open set exhausted"), std::string::npos);
}

TEST(CostZoneTest, DiagnoseNodeReportsDynamicBlocking)
{
  auto table = makeTable();
  ManifoldGraph g;
  CloudGraphBuilder builder(makeConfig());
  ASSERT_TRUE(builder.buildGraphFromColumnTable(table, g));

  // 模拟融合引擎动态封锁一个静态自由的节点 (注入障碍/编辑器障碍落在 (0,0) 格)
  const uint32_t nid0 = nodeAt(g, 0, 0)->id;
  const float nx = nodeAt(g, 0, 0)->x, ny = nodeAt(g, 0, 0)->y, nz = nodeAt(g, 0, 0)->z;
  {
    GraphNode & nd = g.nodeMutable(nid0);
    nd.dynamic_trav = 1.0f;
    nd.dynamic_zone = static_cast<uint8_t>(CostZone::FORBIDDEN);
    nd.synthesize();
  }

  // 诊断必须命中 dynamic_blocked 分支并透出分层区划 (活图语义)
  const std::string json = builder.diagnoseNode(g, nx, ny, nz);
  EXPECT_NE(json.find("\"reason_code\":\"dynamic_blocked\""), std::string::npos);
  EXPECT_NE(json.find("\"dynamic_zone\":3"), std::string::npos);
  EXPECT_NE(json.find("\"static_zone\":0"), std::string::npos);
  EXPECT_NE(json.find("\"dynamic_trav\":1"), std::string::npos);
}

/// 编辑器/注入器规格圆柱 (H=0.5) 的动态带宽度: 攀爬包络判据 S.z_top > k*max_step_height
/// 在 k=2 处恰好卡边界 (0.5 > 0.5 为假), 软带为空 —— 全部带宽 = BODY_HARD 薄环
TEST(CostZoneTest, EditorCylinderHalfMeterHasNoSoftBand)
{
  // 干净场地: 全地面 + 单个 0.5m 圆柱列 (无其他结构, 避免墙的软带污染)
  ColumnTable table;
  table.extent.resolution = kRes;
  table.extent.min_x = 0.0;
  table.extent.min_y = 0.0;
  table.extent.rows = 10;
  table.extent.cols = 10;
  table.cells.assign(static_cast<size_t>(table.extent.rows * table.extent.cols), {});
  for (int r = 0; r < 10; ++r)
    for (int c = 0; c < 10; ++c)
      table.cells[static_cast<size_t>(r * 10 + c)] = {{0.0f, -0.05f, 10}};
  table.cells[static_cast<size_t>(8 * 10 + 8)] = {{0.5f, 0.0f, 10}};  // 圆柱规格: [0, 0.5]

  ManifoldGraph g;
  CloudGraphBuilder builder(makeConfig());
  ASSERT_TRUE(builder.buildGraphFromColumnTable(table, g));

  // 圆柱列的节点在其顶面 (z=0.5, 静态语义为可攀爬顶盖);
  // 融合场景下地面节点 (tread=0) 因 self 列含圆柱面 → 净空 0 → 动态 FORBIDDEN
  const GraphNode * near = nodeAt(g, 8, 7);   // k=1, d=0.1
  ASSERT_NE(near, nullptr);
  EXPECT_EQ(near->cost_zone, static_cast<uint8_t>(CostZone::BODY_HARD));

  // 单步拓扑传染: (8,7) 沿地面单步边向 (8,6) 传染, 累积距离 d=0.2m -> 正常生成 SOFT 软代价带 (修复历史假想包络漏判问题)
  const GraphNode * far = nodeAt(g, 8, 6);
  ASSERT_NE(far, nullptr);
  EXPECT_EQ(far->cost_zone, static_cast<uint8_t>(CostZone::SOFT));
  EXPECT_NEAR(far->traversability, 0.9f * (0.5f - 0.2f) / 0.33f, 0.02f);
  const GraphNode * diag = nodeAt(g, 6, 6);   // k=2 对角, d=0.283
  ASSERT_NE(diag, nullptr);
  EXPECT_EQ(diag->cost_zone, static_cast<uint8_t>(CostZone::SOFT));
}

/// 对照: 圆柱高于 2 级包络 (H=0.55 > 2*0.25) 时, k=2 环产生软代价
TEST(CostZoneTest, TallerCylinderProducesSoftBand)
{
  ColumnTable table;
  table.extent.resolution = kRes;
  table.extent.min_x = 0.0;
  table.extent.min_y = 0.0;
  table.extent.rows = 10;
  table.extent.cols = 10;
  table.cells.assign(static_cast<size_t>(table.extent.rows * table.extent.cols), {});
  for (int r = 0; r < 10; ++r)
    for (int c = 0; c < 10; ++c)
      table.cells[static_cast<size_t>(r * 10 + c)] = {{0.0f, -0.05f, 10}};
  table.cells[static_cast<size_t>(8 * 10 + 8)] = {{0.55f, 0.0f, 10}}; // 高于 2 级包络

  ManifoldGraph g;
  CloudGraphBuilder builder(makeConfig());
  ASSERT_TRUE(builder.buildGraphFromColumnTable(table, g));

  const GraphNode * far = nodeAt(g, 8, 6);  // k=2, d=0.2
  ASSERT_NE(far, nullptr);
  EXPECT_EQ(far->cost_zone, static_cast<uint8_t>(CostZone::SOFT));
  EXPECT_NEAR(far->traversability, 0.9f * (0.5f - 0.2f) / 0.33f, 0.02f);
}

} // namespace

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
