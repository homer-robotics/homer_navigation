cmake_minimum_required(VERSION 2.8.3)
project(homer_navigation)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  nav_msgs
  sensor_msgs
  homer_mapnav_msgs
  homer_nav_libs
  tf
  cmake_modules
  homer_ptu_msgs
  actionlib
  homer_robbie_architecture
  trajectory_msgs
  homer_tts
  homer_tools
)

find_package(Eigen3 REQUIRED)

# eigen 3.2 (wily) only provdies EIGEN3_INCLUDE_DIR, not EIGEN3_INCLUDE_DIRS
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

set(CMAKE_BUILD_TYPE Release)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
  	roscpp
	roslib
	nav_msgs
	sensor_msgs
	homer_mapnav_msgs
	homer_nav_libs
  	trajectory_msgs
	tf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(homer_navigation src/homer_navigation_node.cpp)
add_dependencies(homer_navigation ${catkin_EXPORTED_TARGETS})

target_link_libraries(homer_navigation
   ${catkin_LIBRARIES}
   ${EIGEN3_LIBRARIES}
 )

add_executable(drive_to src/drive_to_node.cpp)
add_dependencies(drive_to ${catkin_EXPORTED_TARGETS})
target_link_libraries(drive_to ${catkin_LIBRARIES})

add_executable(depth_occupancy_map src/depth_occupancy_map.cpp)
add_dependencies(depth_occupancy_map ${catkin_EXPORTED_TARGETS})
target_link_libraries(depth_occupancy_map ${catkin_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS homer_navigation drive_to depth_occupancy_map
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
	nodes/detect_obstacle.py
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
	  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
	  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
