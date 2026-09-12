#include "elevation_planner_core/cloud_graph_builder.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <algorithm>
#include <cmath>
#include <vector>

namespace elevation_planner
{

bool CloudGraphBuilder::buildFromROSMsg(const sensor_msgs::PointCloud2 & cloud_msg, ManifoldGraph & out_graph)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(cloud_msg, *cloud);
  return buildFromPointCloud(cloud, out_graph);
}

bool CloudGraphBuilder::buildFromPointCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & cloud,
                                           ManifoldGraph & out_graph)
{
  if (!cloud || cloud->empty()) return false;

  // 1. 体素降采样 (0.05m 网格保持几何边缘且去重)
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::VoxelGrid<pcl::PointXYZ> vox;
  vox.setInputCloud(cloud);
  vox.setLeafSize(0.05f, 0.05f, 0.05f);
  vox.filter(*filtered_cloud);

  if (filtered_cloud->empty()) return false;

  // 2. 计算 XY 空间包围盒
  float min_x = std::numeric_limits<float>::max();
  float max_x = std::numeric_limits<float>::lowest();
  float min_y = std::numeric_limits<float>::max();
  float max_y = std::numeric_limits<float>::lowest();

  for (const auto & pt : filtered_cloud->points) {
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;
    min_x = std::min(min_x, pt.x);
    max_x = std::max(max_x, pt.x);
    min_y = std::min(min_y, pt.y);
    max_y = std::max(max_y, pt.y);
  }

  double res = config_.resolution;
  int rows = static_cast<int>(std::ceil((max_x - min_x) / res)) + 1;
  int cols = static_cast<int>(std::ceil((max_y - min_y) / res)) + 1;

  out_graph.initSpatialGrid(res, min_x, min_y, rows, cols);

  // 3. 空间栅格投影桶 (单个 O(N) 遍历)
  std::vector<std::vector<float>> column_heights(static_cast<size_t>(rows * cols));
  for (const auto & pt : filtered_cloud->points) {
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;
    int r = static_cast<int>(std::floor((pt.x - min_x) / res));
    int c = static_cast<int>(std::floor((pt.y - min_y) / res));
    if (r >= 0 && r < rows && c >= 0 && c < cols) {
      column_heights[static_cast<size_t>(r * cols + c)].push_back(pt.z);
    }
  }

  // 4. 单柱多曲面聚类与垂直净空计算 (提取一楼、二楼及楼梯踏面)
  struct RawSurface {
    float z_top;
    float z_bottom;
    int count;
  };

  for (int r = 0; r < rows; ++r) {
    for (int c = 0; c < cols; ++c) {
      auto & heights = column_heights[static_cast<size_t>(r * cols + c)];
      if (heights.size() < static_cast<size_t>(config_.min_cluster_points)) continue;

      std::sort(heights.begin(), heights.end());

      std::vector<RawSurface> surfaces;
      float cur_bottom = heights[0];
      float cur_top = heights[0];
      int cur_count = 1;

      for (size_t k = 1; k < heights.size(); ++k) {
        if (heights[k] - heights[k - 1] <= config_.cluster_height_diff) {
          cur_top = heights[k];
          cur_count++;
        } else {
          surfaces.push_back({cur_top, cur_bottom, cur_count});
          cur_bottom = heights[k];
          cur_top = heights[k];
          cur_count = 1;
        }
      }
      surfaces.push_back({cur_top, cur_bottom, cur_count});

      // 提取踏面节点并评估净空 (假设均可通行，不设踏面坡度截断)
      for (size_t s = 0; s < surfaces.size(); ++s) {
        if (surfaces[s].count < config_.min_cluster_points) continue;

        float headroom = 3.0f; // 默认室外无上顶
        if (s + 1 < surfaces.size()) {
          headroom = surfaces[s + 1].z_bottom - surfaces[s].z_top;
        }

        GraphNode node;
        node.x = static_cast<float>(min_x + (r + 0.5) * res);
        node.y = static_cast<float>(min_y + (c + 0.5) * res);
        node.z = surfaces[s].z_top;
        node.row = r;
        node.col = c;
        node.layer_id = static_cast<int>(s);
        node.headroom = headroom;

        // 顶棚净空不足判定 (满足身高净空即均可通行)
        if (headroom < config_.dog_height) {
          node.traversability = 1.0f; // 标记顶头不可通过
        } else {
          node.traversability = 0.0f;
        }

        out_graph.addNode(node);
      }
    }
  }

  // 5. 拓扑邻居建边 (四足运动学触足工作空间：垂直行程 + 水平步长 + 净空，假设台阶与坡度均可通行)
  size_t total_nodes = out_graph.numNodes();

  for (uint32_t u_id = 0; u_id < total_nodes; ++u_id) {
    const auto & u = out_graph.getNode(u_id);
    if (u.traversability >= 0.95f) continue;

    for (int dr = -1; dr <= 1; ++dr) {
      for (int dc = -1; dc <= 1; ++dc) {
        if (dr == 0 && dc == 0) continue;
        int nr = u.row + dr;
        int nc = u.col + dc;
        if (nr < 0 || nr >= rows || nc < 0 || nc >= cols) continue;

        // 在 nr, nc 查找处于触足工作空间包络内的踏面
        for (uint32_t v_id : out_graph.getSpatialCellNodes(nr, nc)) {
          if (v_id == u_id) continue;
          const auto & v = out_graph.getNode(v_id);
          if (v.traversability >= 0.95f) continue;

          // 仅约束四足物理极限：垂直行程与水平步长
          float dz = std::abs(u.z - v.z);
          if (dz > config_.max_step_height) continue;

          float dxy = std::hypot(u.x - v.x, u.y - v.y);
          if (dxy > config_.max_stride_length) continue;

          // 边代价：3D欧氏位移 + 高差势能惩罚 (鼓励走平地，但楼梯绝对连通)
          float edge_len = std::sqrt(dxy * dxy + dz * dz);
          float cost = edge_len + 2.0f * dz + 0.5f * (u.traversability + v.traversability);
          out_graph.addEdge(u_id, v_id, cost);
        }
      }
    }
  }

  // 6. 压平为连续 CSR 结构
  out_graph.finalizeCSR();
  return true;
}

void CloudGraphBuilder::toPointCloudMsg(const ManifoldGraph & graph,
                                        const std::string & frame_id,
                                        sensor_msgs::PointCloud2 & out_cloud)
{
  pcl::PointCloud<pcl::PointXYZI> pc;
  pc.points.reserve(graph.numNodes());

  for (size_t i = 0; i < graph.numNodes(); ++i) {
    const auto & nd = graph.getNode(static_cast<uint32_t>(i));
    pcl::PointXYZI pt;
    pt.x = nd.x;
    pt.y = nd.y;
    pt.z = nd.z;
    pt.intensity = nd.traversability;
    pc.points.push_back(pt);
  }

  pcl::toROSMsg(pc, out_cloud);
  out_cloud.header.frame_id = frame_id;
  out_cloud.header.stamp = ros::Time::now();
}

void CloudGraphBuilder::toMarkerArray(const ManifoldGraph & graph,
                                      const std::string & frame_id,
                                      visualization_msgs::MarkerArray & out_markers,
                                      size_t max_edges)
{
  out_markers.markers.clear();
  visualization_msgs::Marker line_list;
  line_list.header.frame_id = frame_id;
  line_list.header.stamp = ros::Time::now();
  line_list.ns = "manifold_edges";
  line_list.id = 0;
  line_list.type = visualization_msgs::Marker::LINE_LIST;
  line_list.action = visualization_msgs::Marker::ADD;
  line_list.scale.x = 0.02; // 线宽 2cm
  line_list.color.r = 0.1f;
  line_list.color.g = 0.8f;
  line_list.color.b = 0.2f;
  line_list.color.a = 0.6f;

  size_t added = 0;
  for (size_t i = 0; i < graph.numNodes() && added < max_edges; ++i) {
    uint16_t count = 0;
    const auto * edges = graph.getEdges(static_cast<uint32_t>(i), count);
    const auto & u = graph.getNode(static_cast<uint32_t>(i));
    for (uint16_t k = 0; k < count && added < max_edges; ++k) {
      if (edges[k].target_id < i) continue; // 无向图避免重复画两次
      const auto & v = graph.getNode(edges[k].target_id);

      geometry_msgs::Point p1, p2;
      p1.x = u.x; p1.y = u.y; p1.z = u.z;
      p2.x = v.x; p2.y = v.y; p2.z = v.z;
      line_list.points.push_back(p1);
      line_list.points.push_back(p2);
      added++;
    }
  }

  out_markers.markers.push_back(line_list);
}

std::string CloudGraphBuilder::diagnoseEdge(const ManifoldGraph & graph,
                                            double x1, double y1, double z1,
                                            double x2, double y2, double z2) const
{
  std::stringstream ss;
  auto findAnyClosestNode = [&](double qx, double qy, double qz, uint32_t & out_id) -> bool {
    int r = 0, c = 0;
    if (!graph.toGridIndex(qx, qy, r, c)) return false;
    double best_d = 1e9;
    bool found = false;
    for (int dr = -1; dr <= 1; ++dr) {
      int nr = r + dr;
      for (int dc = -1; dc <= 1; ++dc) {
        int nc = c + dc;
        for (uint32_t nid : graph.getSpatialCellNodes(nr, nc)) {
          const auto & nd = graph.getNode(nid);
          double d = std::hypot(nd.x - qx, nd.y - qy) + std::abs(nd.z - qz);
          if (d < best_d) {
            best_d = d;
            out_id = nid;
            found = true;
          }
        }
      }
    }
    return found;
  };

  uint32_t u_id = 0, v_id = 0;
  bool found_u = findAnyClosestNode(x1, y1, z1, u_id);
  bool found_v = findAnyClosestNode(x2, y2, z2, v_id);

  if (!found_u || !found_v) {
    ss << "{\"status\":\"error\",\"message\":\"Failed to find nearest graph nodes for one or both coordinates\"}";
    return ss.str();
  }

  const auto & u = graph.getNode(u_id);
  const auto & v = graph.getNode(v_id);

  bool connected = false;
  float edge_cost = 0.0f;
  uint16_t edge_cnt = 0;
  const GraphEdge * edges = graph.getEdges(u_id, edge_cnt);
  for (uint16_t i = 0; i < edge_cnt; ++i) {
    if (edges[i].target_id == v_id) {
      connected = true;
      edge_cost = edges[i].cost;
      break;
    }
  }

  double dxy = std::hypot(u.x - v.x, u.y - v.y);
  double dz = std::abs(u.z - v.z);
  double slope_deg = (dxy > 1e-4) ? (std::atan2(dz, dxy) * 180.0 / M_PI) : 90.0;

  int dr = std::abs(u.row - v.row);
  int dc = std::abs(u.col - v.col);

  std::vector<std::string> reasons;
  if (!connected) {
    if (u.traversability >= 0.8f) {
      reasons.push_back("Node A low headroom (" + std::to_string(u.headroom).substr(0,4) + "m < " + std::to_string(config_.dog_height).substr(0,4) + "m)");
    }
    if (v.traversability >= 0.8f) {
      reasons.push_back("Node B low headroom (" + std::to_string(v.headroom).substr(0,4) + "m < " + std::to_string(config_.dog_height).substr(0,4) + "m)");
    }
    if (dr > 1 || dc > 1) reasons.push_back("Not 8-connected grid neighbors (dr=" + std::to_string(dr) + ", dc=" + std::to_string(dc) + ")");
    if (dz > config_.max_step_height) reasons.push_back("Step height dz exceeds max_step_height (" + std::to_string(dz).substr(0,4) + "m > " + std::to_string(config_.max_step_height).substr(0,4) + "m)");
    if (dxy > config_.max_stride_length) reasons.push_back("Stride length dxy exceeds max_stride_length (" + std::to_string(dxy).substr(0,4) + "m > " + std::to_string(config_.max_stride_length).substr(0,4) + "m)");
    if (reasons.empty()) reasons.push_back("Nodes belong to different layer clusters or not in search window");
  } else {
    reasons.push_back("Satisfies all quadruped kinematic workspace constraints; active edge in manifold graph");
  }

  ss << "{"
     << "\"status\":\"ok\","
     << "\"connected\":" << (connected ? "true" : "false") << ","
     << "\"cost\":" << edge_cost << ","
     << "\"node_a\":{\"id\":" << u_id << ",\"x\":" << u.x << ",\"y\":" << u.y << ",\"z\":" << u.z << ",\"row\":" << u.row << ",\"col\":" << u.col << ",\"layer\":" << u.layer_id << ",\"headroom\":" << u.headroom << ",\"traversability\":" << u.traversability << "},"
     << "\"node_b\":{\"id\":" << v_id << ",\"x\":" << v.x << ",\"y\":" << v.y << ",\"z\":" << v.z << ",\"row\":" << v.row << ",\"col\":" << v.col << ",\"layer\":" << v.layer_id << ",\"headroom\":" << v.headroom << ",\"traversability\":" << v.traversability << "},"
     << "\"metrics\":{\"dxy\":" << dxy << ",\"dz\":" << dz << ",\"slope_deg\":" << slope_deg << ",\"dr\":" << dr << ",\"dc\":" << dc << "},"
     << "\"limits\":{\"max_step_height\":" << config_.max_step_height << ",\"max_stride_length\":" << config_.max_stride_length << ",\"dog_height\":" << config_.dog_height << "},"
     << "\"reason\":\"";
  for (size_t i = 0; i < reasons.size(); ++i) {
    if (i > 0) ss << "; ";
    ss << reasons[i];
  }
  ss << "\"}";

  return ss.str();
}

} // namespace elevation_planner
