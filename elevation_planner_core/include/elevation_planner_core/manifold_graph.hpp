#pragma once

#include <grid_map_core/grid_map_core.hpp>
#include <geometry_msgs/Point.h>
#include <vector>
#include <string>
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
 * @brief 紧凑流形拓扑踏面节点 (32字节对齐，硬件缓存友好)
 */
struct alignas(32) GraphNode
{
  uint32_t id{0};           ///< 全局连续节点唯一编号
  float x{0.0f};            ///< 真实空间世界坐标 X
  float y{0.0f};            ///< 真实空间世界坐标 Y
  float z{0.0f};            ///< 真实空间脚掌踏面高程 Z
  float traversability{0.0f};///< 可通行度代价 (0.0: 完全平坦, 1.0: 致命障碍)
  float headroom{2.0f};     ///< 上方净空高度 (距上一层底板距离，四足机器人需大于自身身高)
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
  bool findClosestNode(double x, double y, double z,
                       uint32_t & out_node_id,
                       double max_dist_xy = 0.5,
                       double max_dist_z = 0.6) const;

  /// @brief 访问节点与邻居边 (零开销)
  inline const GraphNode & getNode(uint32_t id) const { return nodes_[id]; }
  inline const GraphEdge * getEdges(uint32_t id, uint16_t & count) const
  {
    count = nodes_[id].edge_count;
    return &edges_[nodes_[id].edge_offset];
  }

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
