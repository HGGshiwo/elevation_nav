cmake_minimum_required(VERSION 3.0.2)
project(elevation_costmap)

add_compile_options(-std=c++17 -O3 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_ros
  pcl_conversions
  costmap_2d
  elevation_planner_core
)

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES manifold_costmap_layer
  CATKIN_DEPENDS
    roscpp
    std_msgs
    geometry_msgs
    nav_msgs
    sensor_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    costmap_2d
    elevation_planner_core
  DEPENDS PCL EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# move_base 内部插件库 (实时点云融合宿主)
add_library(manifold_costmap_layer
  src/manifold_costmap_layer.cpp
)
target_link_libraries(manifold_costmap_layer
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
