#pragma once

#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <elevation_planner_core/manifold_graph.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <string>

namespace elevation_costmap
{

/// 逐格成因码 (调试图层 /elevation_local_costmap_debug 每格一字节)
enum CellReason : int8_t
{
  REASON_NO_NODE = 0,        ///< 从未被节点盖章 (无候选节点进入盖章半径), 默认致命 —— "看不见的障碍"主因
  REASON_FREE_NODE = 1,      ///< 可通行节点盖章 (traversability≈0)
  REASON_SOFT_NODE = 2,      ///< 软代价节点盖章 (0 < traversability < 0.8, 贴墙减速带)
  REASON_BLOCK_HEADROOM = 3, ///< 胜出节点头顶净空不足
  REASON_BLOCK_LATERAL = 4,  ///< 胜出节点侧向墙体硬阻挡
  REASON_BLOCK_OTHER = 5,    ///< 其它禁行节点
  REASON_SEAM = 6,           ///< 拓扑缝 (相邻踏面格间无任何连通边)
  REASON_CLOSING_FILLED = 7, ///< 原为致命, 被形态学闭运算填充为可通行
  REASON_UNREACHED = 8,      ///< 自由来自与机器人所在格在地毯上不连通的区域 (绕道图外到达的透印), 已转为致命
};

/// 成因码对应的人类可读解释 (调试面板用)
inline const char * cellReasonText(int8_t reason)
{
  switch (reason) {
    case REASON_NO_NODE: return "无节点盖章: 附近无可通行踏面节点进入盖章半径, 默认致命 (点云空缺/节点缺失)";
    case REASON_FREE_NODE: return "可通行踏面节点盖章";
    case REASON_SOFT_NODE: return "软代价节点: 位于贴墙膨胀减速带";
    case REASON_BLOCK_HEADROOM: return "节点头顶净空不足 (上层结构过低)";
    case REASON_BLOCK_LATERAL: return "节点侧向墙体落入机体硬半径";
    case REASON_BLOCK_OTHER: return "节点被判定禁行";
    case REASON_SEAM: return "拓扑缝: 相邻踏面格在流形图中无任何连通边, 禁止直接跨越";
    case REASON_CLOSING_FILLED: return "闭运算填充: 原为孤立致命补丁 (图节点缺失伪影), 已填为可通行";
    case REASON_UNREACHED: return "透印隔离: 该格的自由来自图上联通、但在地毯上与机器人所在区域不连通的节点 (绕道图外到达), 已转为致命";
    default: return "未知成因";
  }
}

struct ManifoldCostmapBuilderConfig
{
  double resolution{0.05};             ///< 局部栅格分辨率 (m)
  double map_width{6.0};               ///< 局部代价地图横向总宽度 (m)
  double map_length{6.0};              ///< 局部代价地图纵向总长度 (m)
  double forward_offset{3.0};          ///< 机器人到地图后边缘的偏移 (m), 3.0m 居中对齐
  double lookahead_distance{0.6};      ///< 全局路径前瞻采样距离 (m), 用于确定切向前进方向
  double height_tolerance{0.30};       ///< 贴地离面高度容差 (m)
  double dog_height{0.45};             ///< 净空高度阈值 (m)
  std::string map_frame{"map"};        ///< 全局参考系
  std::string base_frame{"base_link"}; ///< 机器人机身参考系
  std::string output_frame{"local_manifold_frame"}; ///< 局部流形切空间参考系
};

/**
 * @class ManifoldCostmapBuilder
 * @brief 基于三维多层流形图构建 1:1 无畸变局部代价地图的核心构建器
 * 
 * 算法原理:
 * 1. 从流形图中提取机器狗脚底踏面点 P0, 前进切向点 P_front, 侧向踏面点 P_side;
 * 2. 叉乘构建贴地法向量 N 与正交切空间主轴 [X_axis, Y_axis];
 * 3. 对视野内节点点乘降维投影, 严格保距, 消除坡度速度失真与横向拉伸畸变.
 */
class ManifoldCostmapBuilder
{
public:
  explicit ManifoldCostmapBuilder(const ManifoldCostmapBuilderConfig & config = ManifoldCostmapBuilderConfig());

  void setConfig(const ManifoldCostmapBuilderConfig & config) { config_ = config; }
  const ManifoldCostmapBuilderConfig & getConfig() const { return config_; }

  /**
   * @brief 从流形图构建局部 1:1 代价地图
   * @param graph 3D流形图 (融合图或全局先验图)
   * @param robot_pose 机器人当前3D世界位姿
   * @param global_plan 全局参考规划路径 (3D)
   * @param out_grid 输出的标准局部代价地图 (OccupancyGrid)
   * @param out_tf 输出的局部切空间坐标系到世界坐标系的变换
   * @param out_reasons 可选输出: 与 out_grid.data 同布局的逐格成因码 (CellReason), 供调试图层
   * @param out_node_ids 可选输出: 与 out_grid.data 同布局的逐格胜出节点 id (-1=无, 如默认致命/拓扑缝)
   * @return true 构建成功, false 失败
   */
  bool buildCostmap(const elevation_planner::ManifoldGraph & graph,
                    const geometry_msgs::Pose & robot_pose,
                    const std::vector<geometry_msgs::PoseStamped> & global_plan,
                    nav_msgs::OccupancyGrid & out_grid,
                    geometry_msgs::TransformStamped & out_tf,
                    std::vector<int8_t> * out_reasons = nullptr,
                    std::vector<int32_t> * out_node_ids = nullptr) const;

private:
  ManifoldCostmapBuilderConfig config_;
};

} // namespace elevation_costmap
