#pragma once

#include <costmap_converter/ObstacleArrayMsg.h>
#include <costmap_converter/ObstacleMsg.h>
#include <elevation_planner_core/manifold_graph.hpp>
#include <geometry_msgs/Pose.h>
#include <vector>
#include <string>
#include <memory>

namespace elevation_costmap
{

struct ObstacleExtractorConfig
{
  double window_radius{3.5};           ///< 局部提取视野半径 (m)
  double height_tolerance{0.30};       ///< 贴地高程匹配容差 (m)
  double dog_height{0.45};             ///< 机体垂直通过高度 (m)
  double min_boundary_length{0.15};    ///< 悬空边界线最小长度过滤 (m)
  double default_circle_radius{0.15};  ///< 孤立正障碍圆柱默认半径 (m)
  std::string map_frame{"map"};        ///< 目标坐标系
};

/**
 * @class ManifoldObstacleExtractor
 * @brief 直接基于流形拓扑连通图提取稀疏高质量局部几何障碍物
 * 
 * 核心设计 (方案 B - 连通图直出):
 * 1. 连续障碍物与边界墙 (LineObstacle):
 *    直接利用可通行连通分支 (candidate_nodes) 与外部世界/墙体的交界面，合并为不可穿透的连续刚性防护线段。
 *    彻底杜绝多圆柱间的空隙钻缝 (穿模) 风险，严密保护台阶断坎与实体墙面。
 * 2. 孤立正障碍物 (CircularObstacle):
 *    仅针对活动层内部的孤立立柱/悬挂阻挡节点输出紧凑圆柱避障原语，方便 TEB 快速进行同伦类分支探索。
 * 3. 彻底废除点云凸包与空间切片 (PolygonObstacle):
 *    杜绝多边形算力消耗与台阶交接处误框踏面的历史缺陷。
 */
class ManifoldObstacleExtractor
{
public:
  explicit ManifoldObstacleExtractor(const ObstacleExtractorConfig & config = ObstacleExtractorConfig());

  void setConfig(const ObstacleExtractorConfig & config) { config_ = config; }
  const ObstacleExtractorConfig & getConfig() const { return config_; }

  /**
   * @brief 从三维流形图提取当前层几何障碍物并组装为 ObstacleArrayMsg
   * @param graph 3D流形图
   * @param robot_pose 机器人当前世界位姿
   * @param candidate_nodes 经BFS确认属于当前连通踏面的节点列表
   * @param is_candidate 节点是否属于当前踏面的布尔掩码
   * @param out_obstacles 输出的 ObstacleArrayMsg (供 TEB 消费)
   * @return true 提取成功, false 提取失败
   */
  bool extractObstacles(const elevation_planner::ManifoldGraph & graph,
                        const geometry_msgs::Pose & robot_pose,
                        const std::vector<uint32_t> & candidate_nodes,
                        const std::vector<char> & is_candidate,
                        costmap_converter::ObstacleArrayMsg & out_obstacles) const;

private:
  ObstacleExtractorConfig config_;

  // 辅助 1: 提取连续连通图边界墙与断坎防护线 (LineObstacle - 防穿模)
  void extractBoundaryLineObstacles(const elevation_planner::ManifoldGraph & graph,
                                    double center_x, double center_y, double center_z,
                                    const std::vector<uint32_t> & candidate_nodes,
                                    const std::vector<char> & is_candidate,
                                    std::vector<costmap_converter::ObstacleMsg> & out_obs) const;

  // 辅助 2: 提取当前活动层孤立立柱/悬挂阻挡节点 (CircularObstacle)
  void extractIsolatedCircularObstacles(const elevation_planner::ManifoldGraph & graph,
                                        double center_x, double center_y, double center_z,
                                        const std::vector<uint32_t> & candidate_nodes,
                                        const std::vector<char> & is_candidate,
                                        std::vector<costmap_converter::ObstacleMsg> & out_obs) const;
};

} // namespace elevation_costmap
