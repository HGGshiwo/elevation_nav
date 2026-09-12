// 离线验证工具: 加载 PCD -> 建流形图(机体建模 OFF/ON 对比) -> 连通域/膨胀诊断 -> A* 规划 -> 路径侧向安全距离
// 用法: rosrun elevation_global_planner offline_test <pcd_file> [sx sy sz gx gy gz]
#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>
#include <map>
#include <algorithm>

#include "elevation_planner_core/cloud_graph_builder.hpp"
#include "elevation_global_planner/manifold_astar.hpp"
#include "elevation_global_planner/path_smoother.hpp"

namespace
{
struct Diag
{
  std::vector<int32_t> comp;      // 每节点所属连通域编号
  std::vector<size_t> comp_size;  // 各连通域节点数
};

Diag analyzeComponents(const elevation_planner::ManifoldGraph & graph)
{
  Diag d;
  d.comp.assign(graph.numNodes(), -1);
  for (size_t s = 0; s < graph.numNodes(); ++s) {
    if (d.comp[s] >= 0) continue;
    int32_t cid = static_cast<int32_t>(d.comp_size.size());
    size_t cnt = 0;
    std::vector<uint32_t> stack{static_cast<uint32_t>(s)};
    d.comp[s] = cid;
    while (!stack.empty()) {
      uint32_t u = stack.back(); stack.pop_back();
      cnt++;
      uint16_t ec = 0;
      const auto * es = graph.getEdges(u, ec);
      for (uint16_t k = 0; k < ec; ++k) {
        if (d.comp[es[k].target_id] < 0) { d.comp[es[k].target_id] = cid; stack.push_back(es[k].target_id); }
      }
    }
    d.comp_size.push_back(cnt);
  }
  return d;
}

void printZHistogram(const elevation_planner::ManifoldGraph & graph,
                     const std::vector<uint32_t> & ids, const char * tag)
{
  std::map<int, size_t> buckets;
  for (uint32_t id : ids) {
    const auto & nd = graph.getNode(id);
    if (nd.traversability >= 0.95f) continue;
    buckets[static_cast<int>(std::floor(nd.z / 0.5))]++;
  }
  printf("  %s free-node z histogram (0.5m buckets):\n", tag);
  for (const auto & kv : buckets)
    printf("    z [%+5.1f, %+5.1f): %zu\n", kv.first * 0.5, kv.first * 0.5 + 0.5, kv.second);
}

// 路径点同层(±0.12m)硬阻挡节点的最近水平距离 —— 近似 "离墙距离"
double minWallDist(const elevation_planner::ManifoldGraph & graph, const nav_msgs::Path & path)
{
  double worst = 1e9;
  for (const auto & ps : path.poses) {
    double px = ps.pose.position.x, py = ps.pose.position.y, pz = ps.pose.position.z;
    int r = 0, c = 0, rad = 8;
    if (!graph.toGridIndex(px, py, r, c)) { return 0.0; }
    double best = 1e9;
    for (int dr = -rad; dr <= rad; ++dr)
      for (int dc = -rad; dc <= rad; ++dc)
        for (uint32_t nid : graph.getSpatialCellNodes(r + dr, c + dc)) {
          const auto & nd = graph.getNode(nid);
          if (nd.traversability >= 0.95f && std::abs(nd.z - pz) <= 0.12)
            best = std::min(best, std::hypot(nd.x - px, nd.y - py));
        }
    if (best < worst) worst = best;
  }
  return worst >= 1e9 ? 0.80 : worst;
}
} // namespace

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "offline_test", ros::init_options::AnonymousName);
  ros::Time::init(); // toMarkerArray 等使用 ros::Time::now(), 无 NodeHandle 时需手动初始化

  if (argc < 2) {
    printf("usage: offline_test <pcd> [sx sy sz gx gy gz]\n");
    return 1;
  }
  std::string pcd_path = argv[1];

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path, *cloud) == -1) {
    printf("ERROR: cannot load %s\n", pcd_path.c_str());
    return 1;
  }
  printf("loaded %zu points from %s\n", cloud->size(), pcd_path.c_str());

  double sx = 0, sy = 0, sz = 0, gx = 0, gy = 0, gz = 0;
  bool has_query = (argc >= 8);

  // cloud 模式: offline_test <pcd> cloud x y [radius] [zmin zmax] —— 打印原始点云在该 (x,y) 圆柱内的 z 分布,
  // 并模拟建图器 0.05m 体素降采样后的视图; 给定 zmin/zmax 时逐点输出 (x,y,z) 用于判断结构真伪
  if (argc >= 5 && std::string(argv[2]) == "cloud") {
    double qx = atof(argv[3]), qy = atof(argv[4]);
    double rad = (argc >= 6) ? atof(argv[5]) : 0.15;
    double zmin = (argc >= 7) ? atof(argv[6]) : -1e9;
    double zmax = (argc >= 8) ? atof(argv[7]) : 1e9;
    struct Pt { double x, y, z; };
    std::vector<Pt> pts;
    for (const auto & pt : cloud->points) {
      if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;
      if (std::hypot(pt.x - qx, pt.y - qy) <= rad && pt.z >= zmin && pt.z <= zmax)
        pts.push_back({pt.x, pt.y, pt.z});
    }
    std::sort(pts.begin(), pts.end(), [](const Pt & a, const Pt & b) { return a.z < b.z; });
    printf("raw points within %.2fm of (%.2f, %.2f), z in [%.2f, %.2f]: %zu\n",
           rad, qx, qy, zmin, zmax, pts.size());
    if (!pts.empty()) {
      std::map<int, size_t> buckets; // 0.1m 桶
      for (const auto & p : pts) buckets[static_cast<int>(std::floor(p.z / 0.1))]++;
      printf("z histogram (0.1m buckets):\n");
      for (const auto & kv : buckets)
        printf("  z [%+5.2f, %+5.2f): %zu %s\n", kv.first * 0.1, kv.first * 0.1 + 0.1, kv.second,
               kv.second >= 5 ? "<-- dense" : "");
      printf("points (x, y, z):\n");
      for (const auto & p : pts) printf("  (%.3f, %.3f, %.3f)\n", p.x, p.y, p.z);
      // XY 包围盒: 判断是水平铺开的板还是竖直细杆
      double mnx = 1e9, mxx = -1e9, mny = 1e9, mxy = -1e9;
      for (const auto & p : pts) {
        mnx = std::min(mnx, p.x); mxx = std::max(mxx, p.x);
        mny = std::min(mny, p.y); mxy = std::max(mxy, p.y);
      }
      printf("xy extent: %.3f x %.3f m %s\n", mxx - mnx, mxy - mny,
             (mxx - mnx < 0.08 && mxy - mny < 0.08) ? "<-- 细竖直结构/噪点 (非水平板)" : "");
    }
    // 体素降采样 + SOR 视图 (与建图器完全一致)
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> vox;
    vox.setInputCloud(cloud);
    vox.setLeafSize(0.05f, 0.05f, 0.05f);
    vox.filter(*filtered);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(filtered);
    sor.setMeanK(16);
    sor.setStddevMulThresh(1.5);
    sor.filter(*filtered);
    std::vector<Pt> vpts;
    for (const auto & pt : filtered->points) {
      if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;
      if (std::hypot(pt.x - qx, pt.y - qy) <= rad && pt.z >= zmin && pt.z <= zmax)
        vpts.push_back({pt.x, pt.y, pt.z});
    }
    std::sort(vpts.begin(), vpts.end(), [](const Pt & a, const Pt & b) { return a.z < b.z; });
    printf("after voxel 0.05 + SOR (builder's view): %zu points\n", vpts.size());
    for (const auto & p : vpts) printf("  (%.3f, %.3f, %.3f)\n", p.x, p.y, p.z);
    return 0;
  }

  // check 模式: offline_test <pcd> check x1 y1 z1 x2 y2 z2 —— 检查两点间是否建边及 C++ 权威原因
  if (argc == 10 && std::string(argv[2]) == "check") {
    double ax = atof(argv[3]), ay = atof(argv[4]), az = atof(argv[5]);
    double bx = atof(argv[6]), by = atof(argv[7]), bz = atof(argv[8]);
    double query_z = atof(argv[9]);
    elevation_planner::CloudGraphBuilder builder;
    elevation_planner::ManifoldGraph graph;
    if (!builder.buildFromPointCloud(cloud, graph)) { printf("ERROR: graph build failed\n"); return 1; }
    std::string edge_res = builder.diagnoseEdge(graph, ax, ay, az, bx, by, bz);
    printf("EDGE DIAG: %s\n", edge_res.c_str());
    std::string node_res = builder.diagnoseNode(graph, ax, ay, query_z);
    printf("NODE A DIAG: %s\n", node_res.c_str());
    return 0;
  }

  // dump 模式: offline_test <pcd> dump x y z —— 打印 ON 图内该坐标 0.5m 邻域的全部节点 (排查扫掠触发源)
  if (argc == 6 && std::string(argv[2]) == "dump") {
    double qx = atof(argv[3]), qy = atof(argv[4]), qz = atof(argv[5]);
    elevation_planner::CloudGraphBuilder builder;
    elevation_planner::ManifoldGraph graph;
    if (!builder.buildFromPointCloud(cloud, graph)) { printf("ERROR: graph build failed\n"); return 1; }
    elevation_planner::GraphBuildConfig cfg = builder.getConfig();
    printf("dump nodes within 0.5m of (%.2f, %.2f, %.2f)\n", qx, qy, qz);
    printf("flags: H=headroom-blocked(0x10) L=lateral-blocked(0x20)\n");
    for (size_t i = 0; i < graph.numNodes(); ++i) {
      const auto & nd = graph.getNode(static_cast<uint32_t>(i));
      double d = std::hypot(nd.x - qx, nd.y - qy);
      if (d > 0.5) continue;
      char tag = (nd.traversability >= 0.95f) ? '#' : (nd.traversability > 0.05f ? 's' : '.');
      char fl = ' ';
      if (nd.flags & elevation_planner::node_flags::BLOCK_HEADROOM) fl = 'H';
      if (nd.flags & elevation_planner::node_flags::BLOCK_LATERAL) fl = (fl == 'H') ? '*' : 'L';
      printf("  %c%c d=%.2f (%.2f, %.2f, %+.3f) layer=%d trav=%.2f headroom=%.2f\n",
             tag, fl, d, nd.x, nd.y, nd.z, nd.layer_id, nd.traversability, nd.headroom);
    }
    return 0;
  }

  auto buildGraph = [&](bool body_model_on, elevation_planner::ManifoldGraph & graph,
                        elevation_planner::GraphBuildConfig & out_cfg) -> bool {
    elevation_planner::GraphBuildConfig cfg;
    if (!body_model_on) {
      cfg.footprint_radius = 1e-3;   // 等效关闭机体建模
      cfg.body_hard_radius = 1e-3;
      cfg.sweep_penalty_weight = 0.0;
    }
    out_cfg = cfg;
    elevation_planner::CloudGraphBuilder builder(cfg);
    return builder.buildFromPointCloud(cloud, graph);
  };

  elevation_planner::ManifoldGraph graph_off, graph_on;
  elevation_planner::GraphBuildConfig cfg_off, cfg_on;
  if (!buildGraph(false, graph_off, cfg_off) || !buildGraph(true, graph_on, cfg_on)) {
    printf("ERROR: graph build failed\n");
    return 1;
  }

  // 两张图节点一一对应 (同一建图确定性流程, 仅 traversability/边不同)
  size_t hard_off = 0, hard_on = 0, soft_on = 0, newly_blocked = 0;
  std::vector<uint32_t> newly_blocked_ids;
  for (size_t i = 0; i < graph_off.numNodes(); ++i) {
    float t1 = graph_off.getNode(static_cast<uint32_t>(i)).traversability;
    float t2 = graph_on.getNode(static_cast<uint32_t>(i)).traversability;
    if (t1 >= 0.95f) hard_off++;
    if (t2 >= 0.95f) hard_on++;
    if (t2 > 0.05f && t2 < 0.95f) soft_on++;
    if (t1 < 0.95f && t2 >= 0.95f) { newly_blocked++; newly_blocked_ids.push_back(static_cast<uint32_t>(i)); }
  }
  printf("\nOFF: nodes=%zu edges=%zu hard_blocked=%zu\n", graph_off.numNodes(), graph_off.numEdges(), hard_off);
  printf("ON : nodes=%zu edges=%zu hard_blocked=%zu soft_inflated=%zu\n", graph_on.numNodes(), graph_on.numEdges(), hard_on, soft_on);
  printf("newly hard-blocked by body model: %zu\n", newly_blocked);
  {
    std::map<int, size_t> buckets;
    for (uint32_t id : newly_blocked_ids)
      buckets[static_cast<int>(std::floor(graph_off.getNode(id).z / 0.5))]++;
    printf("newly blocked z histogram (0.5m buckets):\n");
    for (const auto & kv : buckets)
      printf("    z [%+5.1f, %+5.1f): %zu\n", kv.first * 0.5, kv.first * 0.5 + 0.5, kv.second);
    printf("sample newly blocked nodes:\n");
    for (size_t k = 0; k < newly_blocked_ids.size() && k < 8; ++k) {
      const auto & nd = graph_on.getNode(newly_blocked_ids[k]);
      printf("    (%.2f, %.2f, %.2f) layer=%d headroom=%.2f\n", nd.x, nd.y, nd.z, nd.layer_id, nd.headroom);
    }
  }

  Diag diag_off = analyzeComponents(graph_off);
  Diag diag_on = analyzeComponents(graph_on);
  size_t best_off = std::max_element(diag_off.comp_size.begin(), diag_off.comp_size.end()) - diag_off.comp_size.begin();
  size_t best_on = std::max_element(diag_on.comp_size.begin(), diag_on.comp_size.end()) - diag_on.comp_size.begin();
  printf("components: OFF=%zu (largest %zu), ON=%zu (largest %zu)\n",
         diag_off.comp_size.size(), diag_off.comp_size[best_off],
         diag_on.comp_size.size(), diag_on.comp_size[best_on]);

  // 碎裂定位: OFF 最大连通域的节点在 ON 图中散落到了哪些连通域
  {
    std::map<int32_t, std::vector<uint32_t>> groups;
    for (size_t i = 0; i < graph_off.numNodes(); ++i) {
      uint32_t id = static_cast<uint32_t>(i);
      if (diag_off.comp[id] != static_cast<int32_t>(best_off)) continue;
      if (graph_on.getNode(id).traversability >= 0.95f) continue; // 只看 ON 中仍可通行的节点
      groups[diag_on.comp[id]].push_back(id);
    }
    std::vector<std::pair<int32_t, std::vector<uint32_t>>> sorted(groups.begin(), groups.end());
    std::sort(sorted.begin(), sorted.end(),
              [](const auto & a, const auto & b) { return a.second.size() > b.second.size(); });
    printf("OFF-largest-comp fragments in ON graph (top 6):\n");
    for (size_t g = 0; g < sorted.size() && g < 6; ++g) {
      float min_x = 1e9f, min_y = 1e9f, min_z = 1e9f, max_x = -1e9f, max_y = -1e9f, max_z = -1e9f;
      for (uint32_t id : sorted[g].second) {
        const auto & nd = graph_on.getNode(id);
        min_x = std::min(min_x, nd.x); max_x = std::max(max_x, nd.x);
        min_y = std::min(min_y, nd.y); max_y = std::max(max_y, nd.y);
        min_z = std::min(min_z, nd.z); max_z = std::max(max_z, nd.z);
      }
      printf("  ON-comp %d: %zu nodes, bbox x[%.1f, %.1f] y[%.1f, %.1f] z[%.1f, %.1f]\n",
             sorted[g].first, sorted[g].second.size(), min_x, max_x, min_y, max_y, min_z, max_z);
    }
  }

  // 起终点: 未指定时取 OFF 图最大连通域内 x+y 最小/最大的两个自由节点
  if (!has_query) {
    uint32_t a = 0, b = 0;
    float best_min = 1e9f, best_max = -1e9f;
    for (size_t i = 0; i < graph_off.numNodes(); ++i) {
      uint32_t id = static_cast<uint32_t>(i);
      if (diag_off.comp[id] != static_cast<int32_t>(best_off)) continue;
      const auto & nd = graph_off.getNode(id);
      if (nd.traversability >= 0.95f) continue;
      float s = nd.x + nd.y;
      if (s < best_min) { best_min = s; a = id; }
      if (s > best_max) { best_max = s; b = id; }
    }
    const auto & na = graph_off.getNode(a);
    const auto & nb = graph_off.getNode(b);
    sx = na.x; sy = na.y; sz = na.z;
    gx = nb.x; gy = nb.y; gz = nb.z;
  }
  printf("query start=(%.2f, %.2f, %.2f) goal=(%.2f, %.2f, %.2f)\n", sx, sy, sz, gx, gy, gz);

  auto runPlan = [&](bool body_model_on, const elevation_planner::ManifoldGraph & graph,
                     const Diag & diag) {
    printf("\n--- plan with body_model=%s ---\n", body_model_on ? "ON" : "OFF");
    elevation_global_planner::ManifoldAStarPlanner planner;
    planner.initialize(graph);
    geometry_msgs::PoseStamped start, goal;
    start.pose.position.x = sx; start.pose.position.y = sy; start.pose.position.z = sz;
    goal.pose.position.x = gx; goal.pose.position.y = gy; goal.pose.position.z = gz;
    nav_msgs::Path raw, smoothed;
    if (!planner.plan(start, goal, raw)) {
      printf("plan FAILED (no path)\n");
      return;
    }
    elevation_global_planner::PathSmoother smoother;
    smoother.smooth(raw, graph, smoothed);

    float z_min = 1e9f, z_max = -1e9f;
    for (const auto & ps : raw.poses) {
      z_min = std::min(z_min, static_cast<float>(ps.pose.position.z));
      z_max = std::max(z_max, static_cast<float>(ps.pose.position.z));
    }
    printf("path points: raw=%zu smoothed=%zu, z range [%.2f, %.2f]\n",
           raw.poses.size(), smoothed.poses.size(), z_min, z_max);
    printf("min wall clearance (same-level band): raw=%.3f m, smoothed=%.3f m\n",
           minWallDist(graph, raw), minWallDist(graph, smoothed));

    // 找到爬梯段 (相邻路径点 xy 距离 < 0.5m 内 z 变化最大的位置), 供 ON 图诊断阻挡情况
    if (body_model_on == false) {
      float best_dz = 0.0f;
      size_t best_i = 0;
      for (size_t i = 0; i + 3 < raw.poses.size(); ++i) {
        const auto & p0 = raw.poses[i].pose.position;
        const auto & p3 = raw.poses[i + 3].pose.position;
        if (std::hypot(p3.x - p0.x, p3.y - p0.y) > 0.5) continue;
        float dz = std::abs(static_cast<float>(p3.z - p0.z));
        if (dz > best_dz) { best_dz = dz; best_i = i + 1; }
      }
      if (best_dz > 0.05f) {
        const auto & mid = raw.poses[best_i].pose.position;
        printf("stair segment center: (%.2f, %.2f, %.2f), dz/3pts=%.2f\n", mid.x, mid.y, mid.z, best_dz);
        printf("ON-graph nodes within 1.0m of stair center (trav: .=free s=soft #=hard):\n");
        int r0 = 0, c0 = 0;
        if (graph_on.toGridIndex(mid.x, mid.y, r0, c0)) {
          for (int dr = -10; dr <= 10; ++dr) {
            for (int dc = -10; dc <= 10; ++dc) {
              for (uint32_t nid : graph_on.getSpatialCellNodes(r0 + dr, c0 + dc)) {
                const auto & nd = graph_on.getNode(nid);
                double dxy = std::hypot(nd.x - mid.x, nd.y - mid.y);
                if (dxy > 1.0) continue;
                char tag = (nd.traversability >= 0.95f) ? '#' : (nd.traversability > 0.05f ? 's' : '.');
                printf("    %c (%.2f, %.2f, %+0.2f) trav=%.2f\n", tag, nd.x, nd.y, nd.z, nd.traversability);
              }
            }
          }
        }
      }
    }
  };

  runPlan(false, graph_off, diag_off);
  runPlan(true, graph_on, diag_on);

  // 边可视化完整性: MarkerArray 线数必须覆盖全部无向边, 否则前端边显示被截断
  {
    visualization_msgs::MarkerArray ma;
    elevation_planner::CloudGraphBuilder::toMarkerArray(graph_on, "map", ma);
    size_t lines = ma.markers.empty() ? 0 : ma.markers[0].points.size() / 2;
    printf("\nedge marker lines: %zu (graph edges %zu, undirected %zu) %s\n",
           lines, graph_on.numEdges(), graph_on.numEdges() / 2,
           lines >= graph_on.numEdges() / 2 ? "COMPLETE" : "TRUNCATED!");
  }

  // 单节点禁行原因诊断 (与前端 /api/nav/diagnose_node 同一入口): 抽样禁行/软膨胀/自由节点各打印若干
  {
    elevation_planner::CloudGraphBuilder builder_on(cfg_on);
    printf("\n--- diagnoseNode samples (body model ON) ---\n");
    int shown_blocked = 0, shown_soft = 0, shown_free = 0;
    for (size_t i = 0; i < graph_on.numNodes() && (shown_blocked < 3 || shown_soft < 3 || shown_free < 2); ++i) {
      uint32_t id = static_cast<uint32_t>(i);
      const auto & nd = graph_on.getNode(id);
      std::string res = builder_on.diagnoseNode(graph_on, nd.x, nd.y, nd.z);
      // 只打印代表性样本
      bool is_hard = nd.traversability >= 0.95f;
      bool is_soft = nd.traversability > 0.05f && nd.traversability < 0.95f;
      bool is_free = nd.traversability <= 0.05f;
      if ((is_hard && shown_blocked < 3) || (is_soft && shown_soft < 3) || (is_free && shown_free < 2)) {
        printf("[%s] (%.2f, %.2f, %+.2f) -> %s\n",
               is_hard ? "HARD" : (is_soft ? "SOFT" : "FREE"), nd.x, nd.y, nd.z, res.c_str());
        if (is_hard) shown_blocked++;
        else if (is_soft) shown_soft++;
        else shown_free++;
      }
    }
  }

  // 连通性保底验证: 在 ON 图最大连通域内选一低一高两点, 确认正常宽度的楼梯/楼层仍可跨层规划
  {
    double bx = sx, by = sy, bz = sz, ex = gx, ey = gy, ez = gz;
    uint32_t a = 0, b = 0;
    float best_min = 1e9f, best_max = -1e9f;
    for (size_t i = 0; i < graph_on.numNodes(); ++i) {
      uint32_t id = static_cast<uint32_t>(i);
      if (diag_on.comp[id] != static_cast<int32_t>(best_on)) continue;
      const auto & nd = graph_on.getNode(id);
      if (nd.traversability >= 0.95f) continue;
      float s = nd.x + nd.y;
      if (s < best_min) { best_min = s; a = id; }
      if (s > best_max) { best_max = s; b = id; }
    }
    const auto & na = graph_on.getNode(a);
    const auto & nb = graph_on.getNode(b);
    sx = na.x; sy = na.y; sz = na.z;
    gx = nb.x; gy = nb.y; gz = nb.z;
    printf("\nON-graph cross-level query: start=(%.2f, %.2f, %.2f) goal=(%.2f, %.2f, %.2f)\n", sx, sy, sz, gx, gy, gz);
    runPlan(true, graph_on, diag_on);
    // 反向验证: 桥必须双向可通行
    std::swap(sx, gx); std::swap(sy, gy); std::swap(sz, gz);
    printf("ON-graph reverse query: start=(%.2f, %.2f, %.2f) goal=(%.2f, %.2f, %.2f)\n", sx, sy, sz, gx, gy, gz);
    runPlan(true, graph_on, diag_on);
    sx = bx; sy = by; sz = bz; gx = ex; gy = ey; gz = ez;
  }

  // 断点追踪: 把 OFF 原始路径逐边映射到 ON 图 (两图节点 ID 一一对应), 定位被机体模型切断的第一条边
  {
    elevation_global_planner::ManifoldAStarPlanner planner_off;
    planner_off.initialize(graph_off);
    geometry_msgs::PoseStamped start, goal;
    start.pose.position.x = sx; start.pose.position.y = sy; start.pose.position.z = sz;
    goal.pose.position.x = gx; goal.pose.position.y = gy; goal.pose.position.z = gz;
    nav_msgs::Path raw;
    if (!planner_off.plan(start, goal, raw)) return 0;

    // 由 OFF 图还原路径节点 id (坐标匹配)
    std::vector<uint32_t> path_ids;
    for (const auto & ps : raw.poses) {
      uint32_t nid = 0;
      if (graph_off.findClosestNode(ps.pose.position.x, ps.pose.position.y, ps.pose.position.z, nid, 0.15, 0.15))
        path_ids.push_back(nid);
      else path_ids.push_back(UINT32_MAX);
    }

    printf("\n--- OFF path traced on ON graph ---\n");
    int reported = 0;
    for (size_t i = 0; i + 1 < path_ids.size() && reported < 10; ++i) {
      uint32_t u = path_ids[i], v = path_ids[i + 1];
      if (u == UINT32_MAX || v == UINT32_MAX) continue;
      const auto & nu = graph_on.getNode(u);
      const auto & nv = graph_on.getNode(v);
      bool u_free = nu.traversability < 0.95f;
      bool v_free = nv.traversability < 0.95f;
      bool has_edge = false;
      float edge_cost_on = -1.0f, edge_cost_off = -1.0f;
      uint16_t ec = 0;
      const auto * es = graph_on.getEdges(u, ec);
      for (uint16_t k = 0; k < ec; ++k) if (es[k].target_id == v) { has_edge = true; edge_cost_on = es[k].cost; }
      ec = 0;
      const auto * es2 = graph_off.getEdges(u, ec);
      for (uint16_t k = 0; k < ec; ++k) if (es2[k].target_id == v) { edge_cost_off = es2[k].cost; }
      if (u_free && v_free && has_edge) continue;
      printf("edge %3zu: (%.2f,%.2f,%+.2f) -> (%.2f,%.2f,%+.2f) | ON: u=%s v=%s edge=%s (cost %.2f -> %.2f)\n",
             i, nu.x, nu.y, nu.z, nv.x, nv.y, nv.z,
             u_free ? "free" : "BLOCKED", v_free ? "free" : "BLOCKED", has_edge ? "yes" : "DROPPED",
             edge_cost_off, edge_cost_on);
      reported++;
    }
    if (reported == 0) printf("all OFF-path edges survive in ON graph\n");
  }
  return 0;
}
