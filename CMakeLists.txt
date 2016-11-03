cmake_minimum_required(VERSION 2.8.3)
project(homer_navigation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  homer_robbie_architecture
  homer_robot_platform
  nav_msgs
  sensor_msgs
  homer_mapnav_msgs
  homer_nav_libs
  tf
  cmake_modules
  homer_tools
)

find_package(Eigen3 REQUIRED)

set(CMAKE_BUILD_TYPE Release)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

add_executable(homer_navigation src/homer_navigation_node.cpp)
add_dependencies(homer_navigation ${catkin_EXPORTED_TARGETS})

target_link_libraries(homer_navigation
   ${catkin_LIBRARIES}
   ${EIGEN3_LIBRARIES}
 )
