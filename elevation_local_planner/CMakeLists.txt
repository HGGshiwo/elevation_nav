cmake_minimum_required(VERSION 3.0.2)
project(elevation_local_planner)

add_compile_options(-std=c++14 -O3 -Wall)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  elevation_planner_core
  grid_map_core
  grid_map_ros
  grid_map_msgs
  geometry_msgs
  nav_msgs
  tf2_ros
  nav_core
  costmap_2d
  pluginlib
  pcl_ros
  pcl_conversions
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES elevation_local_planner_lib
  CATKIN_DEPENDS roscpp elevation_planner_core grid_map_core grid_map_ros grid_map_msgs geometry_msgs nav_msgs tf2_ros nav_core costmap_2d pluginlib pcl_ros pcl_conversions
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(elevation_local_planner_lib
  src/collision_checker.cpp
  src/d1_velocity_smoother.cpp
  src/elevation_local_planner_plugin.cpp
)
target_link_libraries(elevation_local_planner_lib
  ${catkin_LIBRARIES}
)
