#include "elevation_costmap/manifold_costmap_layer.h"
#include <pluginlib/class_list_macros.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

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
  private_nh.param<std::string>("cloud_topic", cloud_topic_, "/lidar_points");
  private_nh.param<double>("fusion_rate", fusion_rate_, 10.0);
  double crop_radius_xy = 2.0, crop_height_above = 2.0, crop_height_below = 1.0;
  private_nh.param<double>("crop_radius_xy",    crop_radius_xy, 2.0);
  private_nh.param<double>("crop_height_above", crop_height_above, 2.0);
  private_nh.param<double>("crop_height_below", crop_height_below, 1.0);

  // 建图判据 (与全局先验图同源, 保证融合图运动学口径一致)
  elevation_planner::GraphBuildConfig build_cfg;
  private_nh.param<double>("resolution",         build_cfg.resolution, 0.10);
  private_nh.param<double>("max_step_height",    build_cfg.max_step_height, 0.25);
  private_nh.param<double>("max_stride_length",  build_cfg.max_stride_length, 0.35);
  private_nh.param<double>("dog_height",         build_cfg.dog_height, 0.45);
  private_nh.param<double>("footprint_radius",   build_cfg.footprint_radius, 0.26);
  private_nh.param<double>("body_hard_radius",   build_cfg.body_hard_radius, 0.17);
  private_nh.param<double>("inflation_radius",   build_cfg.inflation_radius, 0.50);
  private_nh.param<double>("sweep_penalty_weight", build_cfg.sweep_penalty_weight, 1.0);
  private_nh.param<int>   ("sor_mean_k",         build_cfg.sor_mean_k, 16);
  private_nh.param<double>("sor_std_mul",        build_cfg.sor_std_mul, 1.5);
  private_nh.param<double>("cluster_height_diff", build_cfg.cluster_height_diff, 0.08);
  private_nh.param<int>   ("min_cluster_points", build_cfg.min_cluster_points, 2);

  double robot_length = 0.0, robot_width = 0.0, margin = 0.04;
  private_nh.param<double>("obstacle_safety_margin", margin, 0.04);
  if (private_nh.getParam("robot_width", robot_width) && robot_width > 0.0) {
    build_cfg.body_hard_radius = robot_width * 0.5 + margin;
    if (private_nh.getParam("robot_length", robot_length) && robot_length > 0.0) {
      build_cfg.footprint_radius = std::hypot(robot_length * 0.5, robot_width * 0.5) + margin;
    }
    ROS_INFO("[ManifoldCostmapLayer] Unified robot geometry: body_hard_radius=%.3fm, footprint_radius=%.3fm (W=%.2f, L=%.2f, margin=%.2f)",
             build_cfg.body_hard_radius, build_cfg.footprint_radius, robot_width, robot_length, margin);
  }

  elevation_planner::ManifoldFusionEngine::Params fusion_params;
  fusion_params.map_frame = map_frame_;
  fusion_params.base_frame = base_frame_;
  fusion_params.crop_radius_xy = crop_radius_xy;
  fusion_params.crop_height_above = crop_height_above;
  fusion_params.crop_height_below = crop_height_below;
  fusion_engine_.setConfig(build_cfg, fusion_params);

  enabled_ = true;
  current_ = true;
  default_value_ = costmap_2d::FREE_SPACE;
  matchSize();

  cloud_sub_ = nh.subscribe(cloud_topic_, 1, &ManifoldCostmapLayer::cloudCallback, this);
  aux_cloud_sub_ = nh.subscribe("/elevation_editor_obstacles", 1,
                                &ManifoldCostmapLayer::auxCloudCallback, this);
  dynamic_nodes_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/elevation_dynamic_nodes", 1, /*latch=*/true);

  worker_running_ = true;
  worker_thread_ = std::thread(&ManifoldCostmapLayer::fusionLoop, this);

  ROS_INFO("[ManifoldCostmapLayer] Fusion host initialized (rate=%.1fHz, zero-copy GraphStore)", fusion_rate_);
}

void ManifoldCostmapLayer::activate() {}

void ManifoldCostmapLayer::deactivate() {}

void ManifoldCostmapLayer::reset()
{
  current_ = true;
}

void ManifoldCostmapLayer::updateBounds(double, double, double, double*, double*, double*, double*) {}

void ManifoldCostmapLayer::updateCosts(costmap_2d::Costmap2D &, int, int, int, int) {}

void ManifoldCostmapLayer::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr & msg)
{
  fusion_engine_.ingestCloud(msg);
}

void ManifoldCostmapLayer::auxCloudCallback(const sensor_msgs::PointCloud2::ConstPtr & msg)
{
  fusion_engine_.ingestAuxCloud(msg);
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

void ManifoldCostmapLayer::fusionLoop()
{
  ros::Rate rate(fusion_rate_ > 0.1 ? fusion_rate_ : 10.0);
  while (worker_running_ && ros::ok())
  {
    rate.sleep();

    geometry_msgs::Pose robot_pose;
    if (lookupRobotPose(robot_pose))
    {
      fusion_engine_.processLatestCloud(tf_, robot_pose);
    }

    // 动态改写节点实时可视化 (前端按 intensity 着色: 封锁=红, 软代价=橙; 空帧即清除)
    // z 必须发布真实节点高程: 桥接端以 (x,y,z) 精确匹配建立 dynamic_trav 覆盖层,
    // 任何可视化偏移都会导致查询端键失配、动态封锁静默失效 (渲染抬升由前端 +0.02 自理)
    {
      pcl::PointCloud<pcl::PointXYZI> dyn_cloud;
      for (const auto & n : fusion_engine_.getDynamicNodes()) {
        pcl::PointXYZI pt;
        pt.x = n[0];
        pt.y = n[1];
        pt.z = n[2];
        pt.intensity = n[3];
        dyn_cloud.push_back(pt);
      }
      sensor_msgs::PointCloud2 msg;
      pcl::toROSMsg(dyn_cloud, msg);
      msg.header.frame_id = map_frame_;
      msg.header.stamp = ros::Time::now();
      dynamic_nodes_pub_.publish(msg);
    }
  }
}

} // namespace elevation_costmap
