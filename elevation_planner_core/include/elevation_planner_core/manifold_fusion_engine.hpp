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
 *  - 分层叠加: 节点保存 static_trav/static_headroom/static_zone (建图静态层) 与
 *    dynamic_trav/dynamic_headroom/dynamic_zone (融合动态层, 含禁行 1.0、
 *    机体硬环 BODY_HARD 与膨胀 <=0.9); 合成规则见 GraphNode::synthesize():
 *    zone 取严 (max), trav = 环/禁行 ? 1.0 : min(1.0, 静态+动态), headroom = min;
 *  - 动态层每帧全量重算: 清理区 (上帧 ∪ 本帧 ROI 外扩膨胀半径) 清零后,
 *    按当前观测全量重写 —— 无快照、无变化检测, 障碍移除随清理区天然复原;
 *  - 局部规划器/绕障/走廊/LOS 直接读合成 traversability, 动态障碍天然生效。
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

  /// 当前被动态改写属性的节点快照 [{x, y, z, traversability, zone}] (供宿主实时可视化;
  /// zone = CostZone 枚举: FORBIDDEN 禁行 / BODY_HARD 机体硬环 / SOFT 软代价, 配合 trav
  /// 供前端三档配色; 障碍离开后自动清空)。仅 worker 线程访问。
  const std::vector<std::array<float, 5>> & getDynamicNodes() const { return dynamic_nodes_; }

private:
  Params params_;
  CloudGraphBuilder graph_builder_;
  bool config_ready_{false};

  std::mutex cloud_mutex_;
  sensor_msgs::PointCloud2::ConstPtr latest_cloud_;
  bool cloud_dirty_{false};

  std::mutex aux_mutex_;
  sensor_msgs::PointCloud2::ConstPtr latest_aux_;

  // 无状态分层刷新: 不做变化检测/快照, 每帧清理区(上帧∪本帧 ROI 外扩膨胀半径)
  // 清零动态层后全量重写。仅记录上一帧窗口供清理, 换图时作废
  std::shared_ptr<ManifoldGraph> mutated_graph_;  // 当前绑定的全局图 (换图时旧窗口作废)
  bool prev_valid_{false};
  int prev_r0_{0}, prev_c0_{0}, prev_r1_{0}, prev_c1_{0};

  // 动态层清零: 窗口内层带节点 dynamic_trav=0 / dynamic_headroom=3.0 / dynamic_zone=FREE 并重合成
  void clearDynamicLayer(const std::shared_ptr<ManifoldGraph> & graph,
                         int r0, int c0, int r1, int c1);

  // 当前被动态层叠加的节点快照 [{x, y, z, 合成 traversability, 合成 zone}] (供宿主发布可视化)
  std::vector<std::array<float, 5>> dynamic_nodes_;
};

} // namespace elevation_planner
