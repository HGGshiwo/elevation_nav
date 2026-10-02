#pragma once

#include <grid_map_core/grid_map_core.hpp>
#include <geometry_msgs/Point.h>
#include <vector>
#include <string>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace elevation_planner
{

/**
 * @brief 紧凑图边定义 (仅 8 字节)
 */
struct GraphEdge
{
  uint32_t target_id{0};   ///< 目标节点在全局节点数组中的下标
  float cost{0.0f};        ///< 边权重 (3D欧氏距离 + 台阶高差惩罚 + 坡度代价)
};

/// GraphNode.flags 状态位定义
namespace node_flags
{
constexpr uint16_t STAIR = 0x01;          ///< 楼梯踏面
constexpr uint16_t BRIDGE = 0x02;         ///< 桥梁连廊
constexpr uint16_t GATEWAY = 0x04;        ///< 层间网关
constexpr uint16_t BLOCK_HEADROOM = 0x10; ///< 禁行原因: 头顶净空不足
constexpr uint16_t BLOCK_LATERAL = 0x20;  ///< 禁行原因: 侧向障碍落入机体硬半径
}

/**
 * @brief 三档代价区划: 图上每个节点显式归属的通行语义档位。
 *
 * 区划由建图 (静态层) 与融合 (动态层) 分别计算, 合成取严 (max)。
 * 不变量: cost_zone >= BODY_HARD 的节点合成 traversability 恒为 1.0,
 * 因此所有 trav>=0.95 的历史消费者自动尊重机体硬环, 无需逐一改造。
 */
enum class CostZone : uint8_t
{
  FREE = 0,       ///< 自由区: 无侧向障碍影响, 可通行
  SOFT = 1,       ///< 软代价带: body_hard_radius < d <= inflation_radius, 可通行代价递增
  BODY_HARD = 2,  ///< 机体硬禁行环: d < body_hard_radius, 机体不可入 (非障碍本体), A* 硬排除
  FORBIDDEN = 3   ///< 绝对禁行: 障碍本体 (行走层无节点) / 顶头净空不足 / 动态封锁
};

/**
 * @brief 紧凑流形拓扑踏面节点 (32字节对齐，硬件缓存友好)
 */
struct alignas(32) GraphNode
{
  uint32_t id{0};           ///< 全局连续节点唯一编号
  float x{0.0f};            ///< 真实空间世界坐标 X
  float y{0.0f};            ///< 真实空间世界坐标 Y
  float z{0.0f};            ///< 真实空间脚掌踏面高程 Z
  float traversability{0.0f};///< 可通行度代价合成值 = min(1.0, 静态层+动态层), 所有消费者只读
  float static_trav{0.0f};   ///< 静态层通行性: 建图期烘焙 (墙体/楼梯/静态软带), 融合不修改
  float dynamic_trav{0.0f};  ///< 动态层通行性: 融合每帧全量重算 (0=无叠加; <=0.9 膨胀; 1.0 禁行)
  float headroom{2.0f};      ///< 上方净空合成值 = min(静态层, 动态层)
  float static_headroom{2.0f};  ///< 静态层净空: 建图期烘焙, 融合不修改
  float dynamic_headroom{3.0f}; ///< 动态层净空: 融合每帧重算 (3.0 = 无动态顶盖)
  uint8_t static_zone{0};    ///< 静态层区划 (CostZone): 建图期烘焙, 融合不修改
  uint8_t dynamic_zone{0};   ///< 动态层区划 (CostZone): 融合每帧全量重算 (0=FREE 无叠加)
  uint8_t cost_zone{0};      ///< 区划合成值 = max(静态层, 动态层), 消费者只读 (CostZone)
  int32_t row{0};           ///< 空间栅格行
  int32_t col{0};           ///< 空间栅格列
  int32_t layer_id{0};      ///< 曲面层级编号 (0: 最底层, 1, 2...)
  uint32_t edge_offset{0};  ///< 在全局 CSR 边数组中的起始偏移
  uint16_t edge_count{0};   ///< 邻接边总数
  uint16_t flags{0};        ///< 状态标记 (见 node_flags 常量: 楼梯/连廊/网关/禁行原因)

  int64_t getKey() const
  {
    return (static_cast<int64_t>(layer_id) << 48) ^
           (static_cast<int64_t>(row) << 24) ^
           (static_cast<int64_t>(col));
  }

  /// 硬排除判定: 机体硬环与绝对禁行 (zone 优先, trav 阈值兜底兼容旧构造路径)
  bool hardBlocked() const
  {
    return cost_zone >= static_cast<uint8_t>(CostZone::BODY_HARD) || traversability >= 0.95f;
  }

  /// 三层属性合成 (唯一写入点): 区划取严 (动态只许变差), 机体环/禁行强制 trav=1.0
  void synthesize()
  {
    cost_zone = std::max(static_zone, dynamic_zone);
    traversability = (cost_zone >= static_cast<uint8_t>(CostZone::BODY_HARD))
                         ? 1.0f
                         : std::min(1.0f, static_trav + dynamic_trav);
    headroom = std::min(static_headroom, dynamic_headroom);
  }
};

// 保持历史兼容别名
using ManifoldNode = GraphNode;

/**
 * @brief 多层连续流形拓扑图：基于 CSR (Compressed Sparse Row) 与 2.5D 空间桶的极速存储结构
 */
class ManifoldGraph
{
public:
  ManifoldGraph() = default;
  ~ManifoldGraph() = default;

  void clear();

  /// @brief 初始化空间哈希网格几何参数
  void initSpatialGrid(double resolution, double min_x, double min_y, int rows, int cols);

  /// @brief 添加节点，返回其分配的唯一全局 node_id
  uint32_t addNode(const GraphNode & node);

  /// @brief 添加邻接边 (在构建阶段缓存)
  void addEdge(uint32_t from_id, uint32_t to_id, float cost);

  /// @brief 构建完成，压平所有边为连续 CSR 内存布局
  void finalizeCSR();

  /// @brief 极速 O(1) 查找给定三维位置附近最近的可行踏面节点
  /// @brief 查询最近节点 (默认跳过禁行节点, 用于起终点吸附;
  ///        include_blocked=true 时返回真正的最近节点, 供阻挡检测/路径锚定感知动态障碍)
  bool findClosestNode(double x, double y, double z,
                       uint32_t & out_node_id,
                       double max_dist_xy = 0.5,
                       double max_dist_z = 0.6,
                       bool include_blocked = false) const;

  /// @brief 结合车头朝向约束查找起点踏面节点 (严格防止向身后倒退吸附)
  bool findStartNode(double x, double y, double z,
                     double heading_x, double heading_y,
                     uint32_t & out_node_id,
                     double max_dist_xy = 2.5,
                     double max_dist_z = 2.5) const;

  /// @brief 访问节点与邻居边 (零开销)
  inline const GraphNode & getNode(uint32_t id) const { return nodes_[id]; }
  /// 融合引擎专用: 原位刷新节点属性 (traversability/headroom)。
  /// 仅允许改属性, 节点集合/id/边结构不可动 (全局-局部 id 透传的前提)。
  inline GraphNode & nodeMutable(uint32_t id) { return nodes_[id]; }
  inline const GraphEdge * getEdges(uint32_t id, uint16_t & count) const
  {
    count = nodes_[id].edge_count;
    return &edges_[nodes_[id].edge_offset];
  }

  /// @brief 检查两节点在拓扑图中是否连通 (基于有限跳数的 BFS 极速探测)
  bool isConnected(uint32_t from_id, uint32_t to_id, int max_hops = 3) const;

  inline size_t numNodes() const { return nodes_.size(); }
  inline size_t numEdges() const { return edges_.size(); }
  inline double getResolution() const { return resolution_; }
  inline int getRows() const { return rows_; }
  inline int getCols() const { return cols_; }
  inline double getMinX() const { return min_x_; }
  inline double getMinY() const { return min_y_; }

  inline const std::vector<uint32_t> & getSpatialCellNodes(int r, int c) const
  {
    static const std::vector<uint32_t> empty_vec;
    if (r < 0 || r >= rows_ || c < 0 || c >= cols_) return empty_vec;
    return spatial_grid_[static_cast<size_t>(r * cols_ + c)];
  }

  bool toGridIndex(double x, double y, int & r, int & c) const;

  // 兼容 ETH GridMap 接口
  void initializeFromGridMap(const grid_map::GridMap & map, int num_layers);
  bool getNode(int layer, int r, int c, ManifoldNode & out_node) const;
  bool getNeighbors(const ManifoldNode & curr, std::vector<ManifoldNode> & neighbors) const;

  /// @brief 判定两节点是否满足单步运动学相邻 (单步 8 邻域、步高与步长极限内)
  bool isSingleStepNeighbor(const GraphNode & u, const GraphNode & v,
                            double max_step_height = 0.25,
                            double max_stride_length = 0.35) const;

  /// @brief 获取给定节点在图上所有合法的单步相邻邻居节点 ID
  void getSingleStepNeighbors(uint32_t node_id,
                              std::vector<uint32_t> & out_neighbor_ids,
                              double max_step_height = 0.25,
                              double max_stride_length = 0.35) const;

private:
  std::vector<GraphNode> nodes_;
  std::vector<GraphEdge> edges_;
  // 构建阶段临时边缓存：temp_edges_[from_id] = {GraphEdge...}
  std::vector<std::vector<GraphEdge>> temp_edges_;

  // 空间桶索引：大小为 rows_ * cols_，存储落入该栅格的所有多层踏面节点 ID
  std::vector<std::vector<uint32_t>> spatial_grid_;

  double resolution_{0.10};
  double min_x_{0.0};
  double min_y_{0.0};
  int rows_{0};
  int cols_{0};
};

} // namespace elevation_planner
