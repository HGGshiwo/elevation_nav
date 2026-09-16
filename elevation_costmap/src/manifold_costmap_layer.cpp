#include "elevation_costmap/manifold_costmap_layer.h"
#include <pluginlib/class_list_macros.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf2/utils.h>

#include <cmath>
#include <unordered_map>

PLUGINLIB_EXPORT_CLASS(elevation_costmap::ManifoldCostmapLayer, costmap_2d::Layer)

namespace elevation_costmap
{

ManifoldCostmapLayer::ManifoldCostmapLayer()
{
  costmap_ = nullptr;
}

ManifoldCostmapLayer::~ManifoldCostmapLayer()
{
  if (worker_running_)
  {
    worker_running_ = false;
    if (worker_thread_.joinable()) worker_thread_.join();
  }
}

void ManifoldCostmapLayer::onInitialize()
{
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~/" + name_);

  private_nh.param<std::string>("map_frame",   map_frame_,   "map");
  private_nh.param<std::string>("base_frame",  base_frame_,  "base_link");
  private_nh.param<std::string>("cloud_topic",     cloud_topic_,     "/lidar_points");
  private_nh.param<std::string>("plan_topic",      plan_topic_,      "/move_base/ElevationGlobalPlanner/global_plan");
  private_nh.param<std::string>("obstacles_topic", obstacles_topic_, "/move_base/TebLocalPlannerROS/obstacles");
  private_nh.param<double>("fusion_rate",        fusion_rate_, 10.0);
  private_nh.param<double>("crop_radius_xy",     crop_radius_xy_, 2.0);
  private_nh.param<double>("crop_height_above",  crop_height_above_, 2.0);
  private_nh.param<double>("crop_height_below",  crop_height_below_, 1.0);

  // 1:1 代价地图构建器配置
  ManifoldCostmapBuilderConfig builder_cfg;
  private_nh.param<double>("resolution",         builder_cfg.resolution, 0.05);
  private_nh.param<double>("map_width",          builder_cfg.map_width, 3.5);
  private_nh.param<double>("map_length",         builder_cfg.map_length, 3.5);
  private_nh.param<double>("forward_offset",     builder_cfg.forward_offset, 1.0);
  private_nh.param<double>("lookahead_distance", builder_cfg.lookahead_distance, 0.6);
  private_nh.param<double>("height_tolerance",   builder_cfg.height_tolerance, 0.30);
  private_nh.param<double>("dog_height",         builder_cfg.dog_height, 0.45);
  builder_cfg.map_frame = map_frame_;
  builder_cfg.base_frame = base_frame_;
  costmap_builder_.setConfig(builder_cfg);

  // 建图配置 (与全局同源)
  elevation_planner::GraphBuildConfig build_cfg;
  private_nh.param<double>("resolution",         build_cfg.resolution, 0.10);
  private_nh.param<double>("max_step_height",    build_cfg.max_step_height, 0.25);
  private_nh.param<double>("max_stride_length",  build_cfg.max_stride_length, 0.35);
  private_nh.param<double>("dog_height",         build_cfg.dog_height, 0.45);
  private_nh.param<double>("footprint_radius",   build_cfg.footprint_radius, 0.30);
  private_nh.param<double>("body_hard_radius",   build_cfg.body_hard_radius, 0.15);
  private_nh.param<double>("sweep_penalty_weight", build_cfg.sweep_penalty_weight, 1.0);
  private_nh.param<int>   ("sor_mean_k",         build_cfg.sor_mean_k, 16);
  private_nh.param<double>("sor_std_mul",        build_cfg.sor_std_mul, 1.5);
  private_nh.param<double>("cluster_height_diff", build_cfg.cluster_height_diff, 0.08);
  private_nh.param<int>   ("min_cluster_points", build_cfg.min_cluster_points, 2);
  graph_builder_.setConfig(build_cfg);

  current_ = true;
  enabled_ = true;
  default_value_ = costmap_2d::FREE_SPACE;

  matchSize();

  // 话题订阅与调试可视化发布
  cloud_sub_ = nh.subscribe(cloud_topic_, 1, &ManifoldCostmapLayer::cloudCallback, this);
  plan_sub_  = nh.subscribe(plan_topic_,  1, &ManifoldCostmapLayer::planCallback,  this);
  costmap_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("/elevation_local_costmap", 1, /*latch=*/true);
  obstacles_pub_ = nh.advertise<costmap_converter::ObstacleArrayMsg>(obstacles_topic_, 1, /*latch=*/false);
  debug_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("/elevation_local_costmap_debug", 1, /*latch=*/true);
  debug_nodes_pub_ = nh.advertise<std_msgs::Int32MultiArray>("/elevation_local_costmap_debug_nodes", 1, /*latch=*/true);

  // 启动后台高频融合与地毯构建线程 (10Hz)
  worker_running_ = true;
  worker_thread_ = std::thread(&ManifoldCostmapLayer::fusionAndCarpetLoop, this);

  ROS_INFO("[ManifoldCostmapLayer] Initialized in-process 1:1 Manifold Costmap Layer (rate=%.1fHz, zero-copy GraphStore)", fusion_rate_);
}

void ManifoldCostmapLayer::activate()
{
}

void ManifoldCostmapLayer::deactivate()
{
}

void ManifoldCostmapLayer::reset()
{
  std::lock_guard<std::mutex> lock(grid_mutex_);
  has_cached_grid_ = false;
  current_ = true;
}

void ManifoldCostmapLayer::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr & msg)
{
  std::lock_guard<std::mutex> lock(cloud_mutex_);
  latest_cloud_ = msg;
  cloud_dirty_ = true;
}

void ManifoldCostmapLayer::planCallback(const nav_msgs::Path::ConstPtr & msg)
{
  std::lock_guard<std::mutex> lock(plan_mutex_);
  latest_plan_ = msg->poses;
}

bool ManifoldCostmapLayer::lookupRobotPose(geometry_msgs::Pose & pose)
{
  if (!tf_) return false;
  try {
    geometry_msgs::TransformStamped tf_stamped = tf_->lookupTransform(
      map_frame_, base_frame_, ros::Time(0), ros::Duration(0.05));
    pose.position.x = tf_stamped.transform.translation.x;
    pose.position.y = tf_stamped.transform.translation.y;
    pose.position.z = tf_stamped.transform.translation.z;
    pose.orientation = tf_stamped.transform.rotation;
    return true;
  } catch (const tf2::TransformException &) {
    return false;
  }
}

void ManifoldCostmapLayer::fusionAndCarpetLoop()
{
  ros::Rate rate(fusion_rate_ > 0.1 ? fusion_rate_ : 10.0);
  while (worker_running_ && ros::ok())
  {
    rate.sleep();

    geometry_msgs::Pose robot_pose;
    bool has_pose = lookupRobotPose(robot_pose);

    // 1. 点云融合处理 (若有新点云且定位就绪)
    if (has_pose)
    {
      sensor_msgs::PointCloud2::ConstPtr msg;
      {
        std::lock_guard<std::mutex> lock(cloud_mutex_);
        if (cloud_dirty_ && latest_cloud_)
        {
          msg = latest_cloud_;
          cloud_dirty_ = false;
        }
      }

      if (msg)
      {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        try {
          if (msg->header.frame_id != map_frame_ && tf_) {
            pcl::PointCloud<pcl::PointXYZ> raw;
            pcl::fromROSMsg(*msg, raw);
            pcl_ros::transformPointCloud(map_frame_, raw, *cloud, *tf_);
          } else {
            pcl::fromROSMsg(*msg, *cloud);
          }
        } catch (const tf2::TransformException &) {
        }

        if (!cloud->empty())
        {
          const double rx = robot_pose.position.x;
          const double ry = robot_pose.position.y;
          const double rz = robot_pose.position.z;
          const double band_low  = rz - crop_height_below_;
          const double band_high = rz + crop_height_above_;

          pcl::PointCloud<pcl::PointXYZ>::Ptr cropped(new pcl::PointCloud<pcl::PointXYZ>);
          cropped->reserve(cloud->size());
          const double r2 = crop_radius_xy_ * crop_radius_xy_;
          for (const auto & pt : cloud->points) {
            if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;
            const double dx = pt.x - rx, dy = pt.y - ry;
            if (dx * dx + dy * dy > r2) continue;
            if (pt.z < band_low || pt.z > band_high) continue;
            cropped->push_back(pt);
          }

          if (!cropped->empty())
          {
            elevation_planner::GridExtent local_extent;
            local_extent.resolution = graph_builder_.getConfig().resolution;
            local_extent.min_x = rx - crop_radius_xy_;
            local_extent.min_y = ry - crop_radius_xy_;
            local_extent.rows = static_cast<int>(std::ceil(2.0 * crop_radius_xy_ / local_extent.resolution)) + 1;
            local_extent.cols = local_extent.rows;

            elevation_planner::ColumnTable observed;
            if (graph_builder_.buildColumnTable(cropped, observed, &local_extent) && !observed.empty())
            {
              // 同进程零拷贝获取先验柱表
              auto prior = elevation_planner::GraphStore::instance().getGlobalTable();
              elevation_planner::ColumnTable fused = prior
                ? elevation_planner::fuseColumnTables(*prior, observed, graph_builder_.getConfig().cluster_height_diff, 1)
                : observed;

              auto graph = std::make_shared<elevation_planner::ManifoldGraph>();
              if (graph_builder_.buildGraphFromColumnTable(fused, *graph))
              {
                std::lock_guard<std::mutex> lock(fused_graph_mutex_);
                fused_graph_ = graph;
                elevation_planner::GraphStore::instance().setFusedGraph(graph);
              }
            }
          }
        }
      }
    }

    // 2. 获取有效图源: 优先实时融合图，无则同进程零拷贝获取全局先验图
    std::shared_ptr<const elevation_planner::ManifoldGraph> active_graph;
    {
      std::lock_guard<std::mutex> lock(fused_graph_mutex_);
      active_graph = fused_graph_;
    }
    if (!active_graph || active_graph->numNodes() == 0)
    {
      active_graph = elevation_planner::GraphStore::instance().getGlobalGraph();
    }

    // 3. 构建 1:1 地毯地图并缓存
    if (active_graph && active_graph->numNodes() > 0 && has_pose)
    {
      std::vector<geometry_msgs::PoseStamped> plan;
      {
        std::lock_guard<std::mutex> lock(plan_mutex_);
        plan = latest_plan_;
      }

      nav_msgs::OccupancyGrid grid;
      std::vector<int8_t> reasons;
      std::vector<int32_t> winner_ids;
      geometry_msgs::TransformStamped dummy_tf;
      costmap_converter::ObstacleArrayMsg obstacles_msg;
      if (costmap_builder_.buildCostmap(*active_graph, robot_pose, plan, grid, dummy_tf, &reasons, &winner_ids, &obstacles_msg))
      {
        // 成因码调试图层: 与地毯同几何, data 逐格 CellReason (供 Web 点击诊断)
        nav_msgs::OccupancyGrid debug_grid = grid;
        debug_grid.data.assign(reasons.begin(), reasons.end());

        // 逐格胜出节点 id + 胜出节点坐标表 (供前端精确显示盖章节点):
        // [w, h, 表条数T, 胜出id×(w*h), (id, x_mm, y_mm, z_mm, trav_x100)×T]
        // 坐标以毫米整数编码, 融合图/全局图模式下均可精确对照
        std::unordered_map<int32_t, const elevation_planner::GraphNode *> winner_table;
        for (int32_t id : winner_ids)
        {
          if (id >= 0) winner_table.emplace(id, &active_graph->getNode(static_cast<uint32_t>(id)));
        }
        std_msgs::Int32MultiArray debug_nodes;
        debug_nodes.data.reserve(3 + winner_ids.size() + 5 * winner_table.size());
        debug_nodes.data.push_back(static_cast<int32_t>(grid.info.width));
        debug_nodes.data.push_back(static_cast<int32_t>(grid.info.height));
        debug_nodes.data.push_back(static_cast<int32_t>(winner_table.size()));
        for (int32_t id : winner_ids) debug_nodes.data.push_back(id);
        for (const auto & kv : winner_table)
        {
          const auto & nd = *kv.second;
          debug_nodes.data.push_back(kv.first);
          debug_nodes.data.push_back(static_cast<int32_t>(std::lround(nd.x * 1000.0)));
          debug_nodes.data.push_back(static_cast<int32_t>(std::lround(nd.y * 1000.0)));
          debug_nodes.data.push_back(static_cast<int32_t>(std::lround(nd.z * 1000.0)));
          debug_nodes.data.push_back(static_cast<int32_t>(std::lround(nd.traversability * 100.0f)));
        }
        {
          std::lock_guard<std::mutex> lock(grid_mutex_);
          cached_grid_ = grid;
          cached_obstacles_ = obstacles_msg;
          cached_debug_ = debug_grid;
          cached_debug_nodes_ = debug_nodes;
          has_cached_grid_ = true;
        }
        costmap_pub_.publish(grid);
        obstacles_pub_.publish(obstacles_msg);
        debug_pub_.publish(debug_grid);
        debug_nodes_pub_.publish(debug_nodes);
      }
    }
  }
}

void ManifoldCostmapLayer::updateBounds(double robot_x, double robot_y, double robot_yaw,
                                        double* min_x, double* min_y,
                                        double* max_x, double* max_y)
{
  (void)robot_x;
  (void)robot_y;
  (void)robot_yaw;
  if (!enabled_) return;

  std::lock_guard<std::mutex> lock(grid_mutex_);
  if (!has_cached_grid_) return;

  double map_min_x = cached_grid_.info.origin.position.x;
  double map_min_y = cached_grid_.info.origin.position.y;
  double map_max_x = map_min_x + cached_grid_.info.width * cached_grid_.info.resolution;
  double map_max_y = map_min_y + cached_grid_.info.height * cached_grid_.info.resolution;

  *min_x = std::min(*min_x, map_min_x);
  *min_y = std::min(*min_y, map_min_y);
  *max_x = std::max(*max_x, map_max_x);
  *max_y = std::max(*max_y, map_max_y);
}

void ManifoldCostmapLayer::updateCosts(costmap_2d::Costmap2D& master_grid,
                                       int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) return;

  std::lock_guard<std::mutex> lock(grid_mutex_);
  if (!has_cached_grid_) return;

  const auto& grid_msg = cached_grid_;
  const double map_res = grid_msg.info.resolution;
  const unsigned int map_w = grid_msg.info.width;
  const unsigned int map_h = grid_msg.info.height;

  for (int j = min_j; j < max_j; ++j)
  {
    for (int i = min_i; i < max_i; ++i)
    {
      double wx = 0.0, wy = 0.0;
      master_grid.mapToWorld(i, j, wx, wy);

      double dx = wx - grid_msg.info.origin.position.x;
      double dy = wy - grid_msg.info.origin.position.y;

      if (dx >= 0.0 && dy >= 0.0)
      {
        int cell_x = static_cast<int>(dx / map_res);
        int cell_y = static_cast<int>(dy / map_res);

        if (cell_x >= 0 && cell_x < static_cast<int>(map_w) &&
            cell_y >= 0 && cell_y < static_cast<int>(map_h))
        {
          size_t idx = static_cast<size_t>(cell_y * map_w + cell_x);
          int8_t val = grid_msg.data[idx];

          unsigned char cost = costmap_2d::FREE_SPACE;
          if (val == 100)
          {
            cost = costmap_2d::LETHAL_OBSTACLE;
          }
          else if (val > 0)
          {
            cost = static_cast<unsigned char>(val * 2.5);
          }
          else if (val == 0)
          {
            cost = costmap_2d::FREE_SPACE;
          }
          else
          {
            cost = costmap_2d::NO_INFORMATION;
          }

          master_grid.setCost(i, j, cost);
        }
      }
    }
  }
}

} // namespace elevation_costmap
