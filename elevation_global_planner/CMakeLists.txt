cmake_minimum_required(VERSION 3.0.2)
project(elevation_global_planner)

add_compile_options(-std=c++14 -O3 -Wall)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  elevation_planner_core
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  pcl_ros
  pcl_conversions
  tf2_ros
)

find_package(PCL REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES elevation_global_planner_lib
  CATKIN_DEPENDS roscpp elevation_planner_core geometry_msgs nav_msgs sensor_msgs visualization_msgs pcl_ros pcl_conversions tf2_ros
  DEPENDS PCL
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_library(elevation_global_planner_lib
  src/manifold_astar.cpp
  src/path_smoother.cpp
)
target_link_libraries(elevation_global_planner_lib
  ${catkin_LIBRARIES}
)

add_executable(elevation_global_planner_node
  src/elevation_global_planner_node.cpp
)
target_link_libraries(elevation_global_planner_node
  elevation_global_planner_lib
  ${catkin_LIBRARIES}
)
