cmake_minimum_required(VERSION 3.0.2)
project(pbtm)

# set(CMAKE_BUILD_TYPE "Release")
# ADD_COMPILE_OPTIONS(-std=c++11 )
# ADD_COMPILE_OPTIONS(-std=c++14 )
# set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  mavros_msgs
  trajectory_msgs
  pcl_ros
  dynamic_reconfigure
)

# Add offboard_controller_lib directory
add_subdirectory(offboard_controller_lib)

find_package(Eigen3 REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/pbtm.cfg
)

catkin_package(
  CATKIN_DEPENDS 
  roscpp
  mavros_msgs
  trajectory_msgs
  pcl_ros

  DEPENDS
  Eigen3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  libbspline/cpp/include
)

add_executable (${PROJECT_NAME}_node 
  src/main.cpp
  src/pbtm.cpp
)


target_link_libraries(${PROJECT_NAME}_node 
  ${catkin_LIBRARIES}
  offboard_controller_lib # link offboard_controller_lib
)

