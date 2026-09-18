#pragma once

#include "elevation_planner_core/cloud_graph_builder.hpp"
#include "elevation_planner_core/manifold_graph.hpp"
#include "elevation_planner_core/local_elevation_grid.hpp"
#include "elevation_planner_core/topological_corridor.hpp"

#include <memory>
#include <mutex>

namespace elevation_planner
{

/**
 * @brief 进程级共享存储: move_base 单进程内全局插件与局部插件之间传递
 *        全局先验柱表与全局流形图, 免去自定义消息的全量序列化。
 *
 * 全局插件在 initialize()/重载地图时写入; 局部插件每帧读取柱表做 ROI 融合。
 * 读写均返回/接收 shared_ptr, 取拷贝后无锁使用, 写入端整表替换不原位修改。
 */
class GraphStore
{
public:
  static GraphStore & instance()
  {
    static GraphStore store;
    return store;
  }

  void setGlobalTable(std::shared_ptr<const ColumnTable> table)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    global_table_ = std::move(table);
  }

  std::shared_ptr<const ColumnTable> getGlobalTable() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return global_table_;
  }

  void setGlobalGraph(std::shared_ptr<const ManifoldGraph> graph)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    global_graph_ = std::move(graph);
  }

  std::shared_ptr<const ManifoldGraph> getGlobalGraph() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return global_graph_;
  }

  void setFusedGraph(std::shared_ptr<const ManifoldGraph> graph)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    fused_graph_ = std::move(graph);
  }

  std::shared_ptr<const ManifoldGraph> getFusedGraph() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return fused_graph_;
  }

  void setLocalElevationGrid(std::shared_ptr<const LocalElevationGrid> grid)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    local_elevation_grid_ = std::move(grid);
  }

  std::shared_ptr<const LocalElevationGrid> getLocalElevationGrid() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return local_elevation_grid_;
  }

  void setTopologicalCorridor(std::shared_ptr<const TopologicalCorridor> corridor)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    topological_corridor_ = std::move(corridor);
  }

  std::shared_ptr<const TopologicalCorridor> getTopologicalCorridor() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return topological_corridor_;
  }

private:
  GraphStore() = default;
  GraphStore(const GraphStore &) = delete;
  GraphStore & operator=(const GraphStore &) = delete;

  mutable std::mutex mutex_;
  std::shared_ptr<const ColumnTable> global_table_;
  std::shared_ptr<const ManifoldGraph> global_graph_;
  std::shared_ptr<const ManifoldGraph> fused_graph_;
  std::shared_ptr<const LocalElevationGrid> local_elevation_grid_;
  std::shared_ptr<const TopologicalCorridor> topological_corridor_;
};

} // namespace elevation_planner
