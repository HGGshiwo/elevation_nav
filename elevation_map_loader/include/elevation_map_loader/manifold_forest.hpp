#pragma once

#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_pcl/GridMapPclLoader.hpp>
#include <vector>
#include <string>

namespace elevation_map_loader
{

struct ExtractorConfig
{
  double resolution{0.10};
  double max_step_height{0.25};
  double max_slope_deg{30.0};
  double max_stride_length{0.35};
  double dog_height{0.50};
  int max_layers{3};
  float vertical_cluster_gap{0.35f};
};

class ManifoldForestExtractor
{
public:
  explicit ManifoldForestExtractor(const ExtractorConfig & config = ExtractorConfig())
    : config_(config) {}

  void extract(const grid_map::GridMapPclLoader & loader, grid_map::GridMap & out_map);

  const ExtractorConfig & getConfig() const { return config_; }
  void setConfig(const ExtractorConfig & config) { config_ = config; }

private:
  ExtractorConfig config_;
};

} // namespace elevation_map_loader
