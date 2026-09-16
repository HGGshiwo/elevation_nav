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
  nav_core
  costmap_2d
  pluginlib
)

find_package(PCL REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES elevation_global_planner_lib
  CATKIN_DEPENDS roscpp elevation_planner_core geometry_msgs nav_msgs sensor_msgs visualization_msgs pcl_ros pcl_conversions tf2_ros nav_core costmap_2d pluginlib
  DEPENDS PCL
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

add_library(elevation_global_planner_lib
  src/manifold_astar.cpp
  src/path_smoother.cpp
)
target_link_libraries(elevation_global_planner_lib
  ${catkin_LIBRARIES}
)

# move_base nav_core 全局规划器插件库
add_library(elevation_global_planner_plugin
  src/elevation_global_planner_plugin.cpp
)
target_link_libraries(elevation_global_planner_plugin
  elevation_global_planner_lib
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

# ---- 离线验证工具 (test/) ----
add_executable(offline_test
  test/offline_test.cpp
)
target_link_libraries(offline_test
  elevation_global_planner_lib
  ${catkin_LIBRARIES}
)
