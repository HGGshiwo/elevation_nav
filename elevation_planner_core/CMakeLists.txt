cmake_minimum_required(VERSION 3.0.2)
project(elevation_planner_core)

add_compile_options(-std=c++14 -O3 -Wall)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  grid_map_core
  grid_map_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  pcl_ros
  pcl_conversions
  costmap_2d
)

find_package(PCL REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES elevation_planner_core
  CATKIN_DEPENDS roscpp grid_map_core grid_map_msgs geometry_msgs nav_msgs sensor_msgs visualization_msgs pcl_ros pcl_conversions costmap_2d
  DEPENDS PCL
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_library(elevation_planner_core
  src/manifold_graph.cpp
  src/cloud_graph_builder.cpp
  src/layer_portal.cpp
  src/cost_evaluator.cpp
)
target_link_libraries(elevation_planner_core
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
