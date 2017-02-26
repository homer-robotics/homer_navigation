cmake_minimum_required(VERSION 2.8.3)
project(homer_navigation)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  homer_robbie_architecture
  nav_msgs
  sensor_msgs
  homer_mapnav_msgs
  homer_nav_libs
  tf
  cmake_modules
  homer_ptu_msgs
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
	homer_robbie_architecture
	nav_msgs
	sensor_msgs
	homer_mapnav_msgs
	homer_nav_libs
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

install(DIRECTORY include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS homer_navigation
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
	  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
	  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
