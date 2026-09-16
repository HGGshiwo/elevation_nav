#include "elevation_planner_core/manifold_graph.hpp"
#include <algorithm>
#include <cmath>

namespace elevation_planner
{

void ManifoldGraph::clear()
{
  nodes_.clear();
  edges_.clear();
  temp_edges_.clear();
  spatial_grid_.clear();
  rows_ = 0;
  cols_ = 0;
}

void ManifoldGraph::initSpatialGrid(double resolution, double min_x, double min_y, int rows, int cols)
{
  clear();
  resolution_ = resolution;
  min_x_ = min_x;
  min_y_ = min_y;
  rows_ = rows;
  cols_ = cols;
  spatial_grid_.resize(static_cast<size_t>(rows_ * cols_));
}

bool ManifoldGraph::toGridIndex(double x, double y, int & r, int & c) const
{
  if (resolution_ <= 1e-6 || rows_ <= 0 || cols_ <= 0) return false;
  r = static_cast<int>(std::floor((x - min_x_) / resolution_));
  c = static_cast<int>(std::floor((y - min_y_) / resolution_));
  return (r >= 0 && r < rows_ && c >= 0 && c < cols_);
}

uint32_t ManifoldGraph::addNode(const GraphNode & node)
{
  uint32_t new_id = static_cast<uint32_t>(nodes_.size());
  GraphNode n = node;
  n.id = new_id;
  nodes_.push_back(n);
  temp_edges_.emplace_back();

  if (n.row >= 0 && n.row < rows_ && n.col >= 0 && n.col < cols_) {
    spatial_grid_[static_cast<size_t>(n.row * cols_ + n.col)].push_back(new_id);
  }
  return new_id;
}

void ManifoldGraph::addEdge(uint32_t from_id, uint32_t to_id, float cost)
{
  if (from_id >= temp_edges_.size() || to_id >= nodes_.size() || from_id == to_id) return;
  temp_edges_[from_id].push_back({to_id, cost});
}

void ManifoldGraph::finalizeCSR()
{
  edges_.clear();
  size_t total_edges = 0;
  for (const auto & vec : temp_edges_) {
    total_edges += vec.size();
  }
  edges_.reserve(total_edges);

  for (size_t i = 0; i < nodes_.size(); ++i) {
    nodes_[i].edge_offset = static_cast<uint32_t>(edges_.size());
    nodes_[i].edge_count = static_cast<uint16_t>(temp_edges_[i].size());
    edges_.insert(edges_.end(), temp_edges_[i].begin(), temp_edges_[i].end());
  }
  temp_edges_.clear();
  temp_edges_.shrink_to_fit();
}

bool ManifoldGraph::findClosestNode(double x, double y, double z,
                                    uint32_t & out_node_id,
                                    double max_dist_xy,
                                    double max_dist_z) const
{
  if (nodes_.empty() || rows_ <= 0 || cols_ <= 0) return false;

  int center_r = 0, center_c = 0;
  if (!toGridIndex(x, y, center_r, center_c)) {
    // 若在地图微量边界外，截断至边界
    center_r = std::max(0, std::min(rows_ - 1, static_cast<int>(std::floor((x - min_x_) / resolution_))));
    center_c = std::max(0, std::min(cols_ - 1, static_cast<int>(std::floor((y - min_y_) / resolution_))));
  }

  int search_radius = std::max(1, static_cast<int>(std::ceil(max_dist_xy / resolution_)));
  double best_cost = std::numeric_limits<double>::max();
  bool found = false;

  for (int dr = -search_radius; dr <= search_radius; ++dr) {
    int r = center_r + dr;
    if (r < 0 || r >= rows_) continue;
    for (int dc = -search_radius; dc <= search_radius; ++dc) {
      int c = center_c + dc;
      if (c < 0 || c >= cols_) continue;

      const auto & cell_nodes = spatial_grid_[static_cast<size_t>(r * cols_ + c)];
      for (uint32_t nid : cell_nodes) {
        const auto & nd = nodes_[nid];
        if (nd.traversability >= 0.95f) continue; // 忽略不可通行障碍
        double dxy = std::hypot(nd.x - x, nd.y - y);
        if (dxy > max_dist_xy) continue;
        double dz = std::abs(nd.z - z);
        if (dz > max_dist_z) continue;

        // 优先距离近且可通行度好的踏面
        double score = dxy + 2.0 * dz + nd.traversability * 2.0;
        if (score < best_cost) {
          best_cost = score;
          out_node_id = nid;
          found = true;
        }
      }
    }
  }
  return found;
}

bool ManifoldGraph::findStartNode(double x, double y, double z,
                                 double heading_x, double heading_y,
                                 uint32_t & out_node_id,
                                 double max_dist_xy,
                                 double max_dist_z) const
{
  if (nodes_.empty() || rows_ <= 0 || cols_ <= 0) return false;

  int center_r = 0, center_c = 0;
  if (!toGridIndex(x, y, center_r, center_c)) {
    center_r = std::max(0, std::min(rows_ - 1, static_cast<int>(std::floor((x - min_x_) / resolution_))));
    center_c = std::max(0, std::min(cols_ - 1, static_cast<int>(std::floor((y - min_y_) / resolution_))));
  }

  int search_radius = std::max(1, static_cast<int>(std::ceil(max_dist_xy / resolution_)));
  double best_cost = std::numeric_limits<double>::max();
  bool found = false;
  bool check_heading = (std::hypot(heading_x, heading_y) > 0.1);

  // 第一轮：严格前向搜索 (proj >= -0.05m)，严禁把起点吸附到机器人屁股身后
  if (check_heading) {
    for (int dr = -search_radius; dr <= search_radius; ++dr) {
      int r = center_r + dr;
      if (r < 0 || r >= rows_) continue;
      for (int dc = -search_radius; dc <= search_radius; ++dc) {
        int c = center_c + dc;
        if (c < 0 || c >= cols_) continue;

        const auto & cell_nodes = spatial_grid_[static_cast<size_t>(r * cols_ + c)];
        for (uint32_t nid : cell_nodes) {
          const auto & nd = nodes_[nid];
          if (nd.traversability >= 0.95f) continue;
          double dxy = std::hypot(nd.x - x, nd.y - y);
          if (dxy > max_dist_xy) continue;
          double dz = std::abs(nd.z - z);
          if (dz > max_dist_z) continue;

          double proj = (nd.x - x) * heading_x + (nd.y - y) * heading_y;
          if (proj < -0.05) continue; // 坚决剔除身后的倒退节点

          double score = dxy + 1.0 * dz + nd.traversability * 2.0;
          if (score < best_cost) {
            best_cost = score;
            out_node_id = nid;
            found = true;
          }
        }
      }
    }
  }

  // 第二轮兜底：若前方无合法节点，回退到普通就近查找
  if (!found) {
    return findClosestNode(x, y, z, out_node_id, max_dist_xy, max_dist_z);
  }

  return true;
}

void ManifoldGraph::initializeFromGridMap(const grid_map::GridMap & map, int num_layers)
{
  clear();
  const auto & size = map.getSize();
  rows_ = size(0);
  cols_ = size(1);
  resolution_ = map.getResolution();
  auto pos = map.getPosition();
  double length_x = map.getLength()(0);
  double length_y = map.getLength()(1);
  min_x_ = pos(0) - length_x / 2.0;
  min_y_ = pos(1) - length_y / 2.0;

  initSpatialGrid(resolution_, min_x_, min_y_, rows_, cols_);

  for (int s = 0; s < num_layers; ++s) {
    std::string suf = "_" + std::to_string(s);
    std::string elev_layer = (s == 0 && !map.exists("elevation_0")) ? "elevation" : "elevation" + suf;
    std::string trav_layer = (s == 0 && !map.exists("traversability_0")) ? "traversability" : "traversability" + suf;

    if (!map.exists(elev_layer)) continue;
    const auto & elev_data = map[elev_layer];
    const auto * trav_data = map.exists(trav_layer) ? &map[trav_layer] : nullptr;

    for (int r = 0; r < rows_; ++r) {
      for (int c = 0; c < cols_; ++c) {
        float z = elev_data(r, c);
        if (std::isnan(z)) continue;
        float trav = (trav_data != nullptr) ? (*trav_data)(r, c) : 0.0f;
        if (std::isnan(trav)) trav = 0.0f;

        grid_map::Index idx(r, c);
        grid_map::Position p;
        map.getPosition(idx, p);

        GraphNode node;
        node.layer_id = s;
        node.row = r;
        node.col = c;
        node.x = static_cast<float>(p(0));
        node.y = static_cast<float>(p(1));
        node.z = z;
        node.traversability = trav;
        node.headroom = 2.0f;
        addNode(node);
      }
    }
  }

  // 默认同层 8-邻域建边
  for (uint32_t i = 0; i < nodes_.size(); ++i) {
    const auto & u = nodes_[i];
    for (int dr = -1; dr <= 1; ++dr) {
      for (int dc = -1; dc <= 1; ++dc) {
        if (dr == 0 && dc == 0) continue;
        int nr = u.row + dr;
        int nc = u.col + dc;
        if (nr < 0 || nr >= rows_ || nc < 0 || nc >= cols_) continue;

        const auto & nbr_ids = spatial_grid_[static_cast<size_t>(nr * cols_ + nc)];
        for (uint32_t nid : nbr_ids) {
          const auto & v = nodes_[nid];
          if (v.layer_id != u.layer_id) continue;
          float dz = std::abs(u.z - v.z);
          if (dz > 0.25f) continue;
          float dxy = std::hypot(u.x - v.x, u.y - v.y);
          float cost = std::sqrt(dxy * dxy + dz * dz) + 0.5f * (u.traversability + v.traversability);
          addEdge(i, nid, cost);
        }
      }
    }
  }
  finalizeCSR();
}

bool ManifoldGraph::getNode(int layer, int r, int c, ManifoldNode & out_node) const
{
  if (r < 0 || r >= rows_ || c < 0 || c >= cols_) return false;
  const auto & ids = spatial_grid_[static_cast<size_t>(r * cols_ + c)];
  for (uint32_t id : ids) {
    if (nodes_[id].layer_id == layer) {
      out_node = nodes_[id];
      return true;
    }
  }
  return false;
}

bool ManifoldGraph::getNeighbors(const ManifoldNode & curr, std::vector<ManifoldNode> & neighbors) const
{
  neighbors.clear();
  if (curr.id >= nodes_.size()) return false;
  uint16_t cnt = 0;
  const GraphEdge * edges = getEdges(curr.id, cnt);
  for (uint16_t i = 0; i < cnt; ++i) {
    neighbors.push_back(nodes_[edges[i].target_id]);
  }
  return !neighbors.empty();
}

bool ManifoldGraph::isConnected(uint32_t from_id, uint32_t to_id, int max_hops) const
{
  if (from_id == to_id) return true;
  if (from_id >= nodes_.size() || to_id >= nodes_.size()) return false;

  if (max_hops <= 1) {
    uint16_t count = 0;
    const auto * es = getEdges(from_id, count);
    for (uint16_t i = 0; i < count; ++i) {
      if (es[i].target_id == to_id) return true;
    }
    return false;
  }

  // 小半径广度优先搜索 (max_hops 2~4, 探索节点数通常 < 50)
  std::vector<uint32_t> frontier{from_id};
  std::vector<uint32_t> next_frontier;
  std::vector<uint32_t> visited{from_id};

  for (int h = 0; h < max_hops; ++h) {
    next_frontier.clear();
    for (uint32_t curr : frontier) {
      uint16_t count = 0;
      const auto * es = getEdges(curr, count);
      for (uint16_t i = 0; i < count; ++i) {
        uint32_t next = es[i].target_id;
        if (next == to_id) return true;
        if (std::find(visited.begin(), visited.end(), next) == visited.end()) {
          visited.push_back(next);
          next_frontier.push_back(next);
        }
      }
    }
    if (next_frontier.empty()) break;
    frontier = next_frontier;
  }
  return false;
}

} // namespace elevation_planner
