cmake_minimum_required(VERSION 3.0.2)
project(path_planning)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
  utils
  communication
  yaml-cpp
  alglib_pkg
  serial
)

find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

# graph system and thread components for boost::asio

find_package(Boost REQUIRED COMPONENTS graph system thread)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES alglib_pkg
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  
)

add_library(track
  src/map/Track.cpp
  src/utils/PathUtils.cpp
  src/utils/SplineUtils.cpp
  src/utils/Utility.cpp
  src/utils/TrafficClient.cpp  
)

add_library(path_planner
  src/planning/PathPlanner.cpp
)

target_link_libraries(track
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  communication
)

target_link_libraries(path_planner
  track
)

add_executable(path_manager_node
  src/path_manager_node.cpp
)
add_executable(sensing_node 
  src/utils/sensing_node.cpp
)

# Dedicated block for sensing_node

target_link_libraries(sensing_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  pthread
)

# Link everything required for sensing_node

target_link_libraries(path_manager_node
  path_planner
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

