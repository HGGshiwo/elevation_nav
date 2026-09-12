cmake_minimum_required(VERSION 3.0.2)
project(elevation_map_loader)

add_compile_options(-std=c++14 -O3 -Wall)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  grid_map_core
  grid_map_ros
  grid_map_pcl
  grid_map_msgs
  std_msgs
  sensor_msgs
  pcl_ros
  rospy
)

find_package(PCL REQUIRED)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES elevation_map_loader_lib
  CATKIN_DEPENDS roscpp roslib rospy grid_map_core grid_map_ros grid_map_pcl grid_map_msgs std_msgs sensor_msgs pcl_ros
  DEPENDS PCL
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_library(elevation_map_loader_lib
  src/manifold_forest.cpp
)
target_link_libraries(elevation_map_loader_lib
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(pcd_to_grid_map_node
  src/pcd_to_grid_map_node.cpp
)
target_link_libraries(pcd_to_grid_map_node
  elevation_map_loader_lib
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

catkin_install_python(PROGRAMS
  scripts/elevation_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
