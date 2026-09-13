#pragma once

#include "elevation_planner_core/manifold_graph.hpp"
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <memory>
#include <string>

namespace elevation_planner
{

/**
 * @brief 2D 空间栅格几何参数 (用于柱表与流形图之间的格对齐)
 */
struct GridExtent
{
  double resolution{0.10};
  double min_x{0.0};
  double min_y{0.0};
  int rows{0};
  int cols{0};

  bool valid() const { return rows > 0 && cols > 0 && resolution > 1e-6; }
};

/**
 * @brief 单根垂直柱内的一个聚类曲面 (踏面候选)
 */
struct ColumnSurface
{
  float z_top{0.0f};     ///< 曲面踩踏高程 (簇内最高点)
  float z_bottom{0.0f}; ///< 曲面底板高程 (簇内最低点)
  int count{0};         ///< 簇内点数 (低于 min_cluster_points 的面不生成节点)
};

/**
 * @brief 柱表: 建图管线步骤 1~4 的中间产物, 每格存储按高度排序的多层曲面簇。
 *        全局先验柱表与实时观测柱表按格 (row, col) 对齐后即可逐柱融合
 */
struct ColumnTable
{
  GridExtent extent;
  std::vector<std::vector<ColumnSurface>> cells; ///< 大小 rows*cols, 空格为空向量

  bool empty() const { return cells.empty(); }
};

/**
 * @brief 逐柱并集融合: 先验柱表 + 实时观测柱表 -> 融合柱表 (无时间维度)
 *
 * 融合规则 (每格独立, 两表须同 resolution):
 *  1. 同一物理面 (两侧 z_top 差 <= same_surface_tol): 合并为一层, 几何取观测值,
 *     点数取 max —— 当前状态由观测反映, 但一次稀疏观测不推翻先验已确认的支撑面;
 *  2. 仅存在于先验: 原样保留 —— 实时雷达被机体遮挡扫不到脚下时, 先验地面层仍在;
 *  3. 仅存在于观测: 新增一层 (实时发现的新踏面/新障碍);
 *  4. 窗口最外 boundary_ring 圈强制采用先验 (先验缺失才回退观测), 保证出窗衔接;
 *
 * 观测柱表每帧从零重建、先验柱表永不被写入, 动态障碍只存在于当帧, 无需衰减。
 * 净空/膨胀/建边不在融合层处理, 统一交由 buildGraphFromColumnTable 重算,
 * 保证融合图与全局图的通行性判定口径完全一致。
 *
 * @return 融合柱表, extent 沿用观测表; 两表分辨率不一致时原样返回观测表
 */
ColumnTable fuseColumnTables(const ColumnTable & prior,
                             const ColumnTable & observed,
                             double same_surface_tol = 0.08,
                             int boundary_ring = 1);

struct GraphBuildConfig
{
  double resolution{0.10};          ///< 2D 空间栅格投影分辨率 (m)
  double max_step_height{0.25};     ///< 四足狗台阶最大踏步高差 (m)
  double max_stride_length{0.35};   ///< 最大跨步步长 (m)
  double dog_height{0.45};          ///< 机器狗身高净空阈值 (m)
  double cluster_height_diff{0.08}; ///< 垂直单柱点云聚类厚度间距 (m)
  int min_cluster_points{2};        ///< 形成踏面的最少激光点数

  // ---- 残影点过滤 (SOR 统计离点滤): 悬浮稀疏点串 (动态物体残留/配准抖动) 会伪装成
  //      水平表面, 撑爆上方净空判定; 其近邻距离远大于真实表面点, 统计上可分离 ----
  int sor_mean_k{16};               ///< SOR 近邻统计点数
  double sor_std_mul{1.5};          ///< SOR 标准差倍数阈值 (越大越保守, 极大值等效关闭)

  // ---- 机体碰撞建模 (足印膨胀 + 建边扫掠) ----
  double footprint_radius{0.30};    ///< 机体足印外接半径 (m): 软代价膨胀边界
  double body_hard_radius{0.15};    ///< 机体硬阻挡半径 (m): 约半身宽+安全余量, 侧向障碍进入此范围则节点不可通行
  double sweep_penalty_weight{1.0}; ///< 建边时机体扫掠区软代价权重
  double foot_clearance{0.05};      ///< 足底容差 (m): 行走面下方此深度内的禁行节点仍保守视为剐蹭, 更深的视为脚下楼梯结构/其他层
};

class CloudGraphBuilder
{
public:
  CloudGraphBuilder() = default;
  explicit CloudGraphBuilder(const GraphBuildConfig & config) : config_(config) {}

  void setConfig(const GraphBuildConfig & config) { config_ = config; }
  const GraphBuildConfig & getConfig() const { return config_; }

  /// @brief 从 ROS PointCloud2 直接端到端构建流形拓扑图
  bool buildFromROSMsg(const sensor_msgs::PointCloud2 & cloud_msg, ManifoldGraph & out_graph);

  /// @brief 从 PCL PointXYZ 点云直接构建流形拓扑图 (等价于柱表生成 + 建边压平两段串联)
  bool buildFromPointCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & cloud, ManifoldGraph & out_graph);

  /// @brief 建图管线步骤 1~4: 点云 -> 柱表 (滤波 + 投影 + 单柱多层聚类)
  /// @param aligned_extent 非空时强制柱表栅格与该几何对齐 (用于和全局图逐格融合),
  ///                      落在范围外的点被丢弃; 为空时按点云包围盒自算
  bool buildColumnTable(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & cloud,
                        ColumnTable & out_table,
                        const GridExtent * aligned_extent = nullptr);

  /// @brief 建图管线步骤 5~6: 柱表 -> 流形图 (节点生成 + 净空/膨胀评估 + 建边 + CSR)
  bool buildGraphFromColumnTable(const ColumnTable & table, ManifoldGraph & out_graph);

  /// @brief 将图中的踏面节点导出为点云消息 (用于 RViz 彩色显示)
  static void toPointCloudMsg(const ManifoldGraph & graph,
                              const std::string & frame_id,
                              sensor_msgs::PointCloud2 & out_cloud);

  /// @brief 将图中的拓扑边导出为 RViz MarkerArray (用于三维连通性可视化)
  /// @note max_edges 须大于无向边总数, 否则地图后半段 (按栅格行序) 的边会被截断,
  ///       导致前端边可视化与权威诊断不一致; 20万约对应千万级节点的大图
  static void toMarkerArray(const ManifoldGraph & graph,
                            const std::string & frame_id,
                            visualization_msgs::MarkerArray & out_markers,
                            size_t max_edges = 200000);

  /// @brief 权威诊断两点之间的拓扑连通性及未建边物理原因 (输出 JSON 字符串)
  std::string diagnoseEdge(const ManifoldGraph & graph,
                           double x1, double y1, double z1,
                           double x2, double y2, double z2) const;

  /// @brief 权威诊断单个踏面节点的通行状态与禁行原因 (净空不足/侧向机体膨胀/软代价区)
  std::string diagnoseNode(const ManifoldGraph & graph,
                           double x, double y, double z) const;

private:
  GraphBuildConfig config_;
};

} // namespace elevation_planner
