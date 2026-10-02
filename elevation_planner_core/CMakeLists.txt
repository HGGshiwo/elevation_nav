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
  tf2_ros
  message_generation
)

find_package(PCL REQUIRED)

add_service_files(
  FILES
  DiagnoseQuery.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES elevation_planner_core
  CATKIN_DEPENDS roscpp grid_map_core grid_map_msgs geometry_msgs nav_msgs sensor_msgs visualization_msgs pcl_ros pcl_conversions costmap_2d tf2_ros message_runtime
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
  src/pcd_map_io.cpp
  src/manifold_fusion_engine.cpp
)
add_dependencies(elevation_planner_core ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(elevation_planner_core
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

if (CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_path_simplifier test/path_simplifier_test.cpp)
  target_link_libraries(test_path_simplifier elevation_planner_core ${catkin_LIBRARIES})
  catkin_add_gtest(test_cost_zone test/cost_zone_test.cpp)
  target_link_libraries(test_cost_zone elevation_planner_core ${catkin_LIBRARIES})
endif()
