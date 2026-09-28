#pragma once

#include <string>
#include <mutex>
#include <memory>
#include <unordered_map>
#include <array>
#include <vector>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose.h>
#include <tf2_ros/buffer.h>
#include <elevation_planner_core/cloud_graph_builder.hpp>
#include <elevation_planner_core/manifold_graph.hpp>

namespace elevation_planner
{

/**
 * @class ManifoldFusionEngine
 * @brief 实时点云 → 全局流形图属性原位刷新的编排引擎 (无线程, 同步逐帧处理)
 *
 * 数据流: 原始点云 → TF 变换到 map 系 → 机器人 ROI 裁剪 (方窗 + 垂直高度带)
 *        → 观测柱表 → 与先验柱表逐柱合并 → 对 ROI 内的全局图节点重算
 *        traversability/headroom → 原位写回全局图 (GraphStore 中同一份)。
 *
 * 节点集合与 id 永不改变 (全局-局部 node_id 透传的前提), 点云只改属性:
 *  - 障碍脚下净空归零 / 侧向结构跨越攀爬包络 → traversability 封锁 (原位写 1.0/软代价);
 *  - 障碍离开观测范围 → 重算值回到先验值 → 恢复快照;
 *  - 局部规划器/绕障/走廊/LOS 直接读图节点, 动态障碍天然生效;
 *  - 全局规划器 A* 读的是自己持有的先验图副本, 不受影响 (结构纯先验不变)。
 *
 * 快照语义: 首次改写某节点时记录 {先验 traversability, headroom};
 * 换图 (全局图重建) 时旧快照作废清空; restoreMutations() 销毁时恢复。
 *
 * 线程约定: ingestCloud 由宿主的 ROS 订阅回调调用; processLatestCloud 由宿主的
 * 工作线程周期调用。属性写入与控制器线程的读取为对齐 4 字节 float 的良性竞争
 * (x86/ARM 保证单字原子性), 相比整图拷贝换指针省去每帧 16MB 的复制。
 */
class ManifoldFusionEngine
{
public:
  struct Params
  {
    std::string map_frame{"map"};
    std::string base_frame{"base_link"};
    double crop_radius_xy{2.0};       ///< ROI 方窗半径 (m), 融合覆盖边长 = 2 × crop_radius_xy
    double crop_height_above{2.0};    ///< 机体所在平面以上保留高度 (m)
    double crop_height_below{1.0};    ///< 机体所在平面以下保留高度 (m)
  };

  /// 注入建图判据 (与全局图同源的 GraphBuildConfig) 与 ROI 参数
  void setConfig(const GraphBuildConfig & build_cfg, const Params & params);
  ~ManifoldFusionEngine();

  /// ROS 订阅回调入口: 暂存最新一帧点云 (仅保留最新, 旧帧直接覆盖)
  void ingestCloud(const sensor_msgs::PointCloud2::ConstPtr & msg);

  /// 第二输入源 (编辑器障碍等叠加云): 与主云合并处理; 语义为"持续存在", 不参与空帧清空
  void ingestAuxCloud(const sensor_msgs::PointCloud2::ConstPtr & msg);

  /**
   * @brief 处理暂存的最新点云帧, 原位刷新 ROI 内全局图节点属性
   * @param tf TF buffer (用于将点云变换到 map 系)
   * @param robot_pose 机器人当前 map 系位姿 (ROI 中心)
   * @return true 本帧有节点属性发生变化
   */
  bool processLatestCloud(tf2_ros::Buffer * tf, const geometry_msgs::Pose & robot_pose);

  /// 恢复所有被动态改写的节点属性 (引擎销毁/停止时调用)
  void restoreMutations();

  /// 当前被动态改写属性的节点快照 [{x, y, z, traversability}] (供宿主实时可视化;
  /// 封锁=1.0 红, 软代价 0~1 橙; 障碍离开后自动清空)。仅 worker 线程访问。
  const std::vector<std::array<float, 4>> & getDynamicNodes() const { return dynamic_nodes_; }

private:
  Params params_;
  CloudGraphBuilder graph_builder_;
  bool config_ready_{false};

  std::mutex cloud_mutex_;
  sensor_msgs::PointCloud2::ConstPtr latest_cloud_;
  bool cloud_dirty_{false};

  std::mutex aux_mutex_;
  sensor_msgs::PointCloud2::ConstPtr latest_aux_;

  // 属性改写快照: node_id -> {先验 traversability, 先验 headroom}
  std::unordered_map<uint32_t, std::pair<float, float>> touched_nodes_;
  std::shared_ptr<ManifoldGraph> mutated_graph_;  // 当前绑定的全局图 (换图时快照作废)

  // 当前被动态改写节点快照 [{x, y, z, traversability}] (供宿主发布可视化)
  std::vector<std::array<float, 4>> dynamic_nodes_;
};

} // namespace elevation_planner
