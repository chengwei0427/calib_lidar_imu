cmake_minimum_required(VERSION 3.0.2)
project(calib_lidar_imu)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
add_compile_options(-std=c++11)
add_compile_options(-o3)
set( CMAKE_CXX_FLAGS_RELEASE "-fpermissive -fPIC")
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  tf
  rosbag
  pcl_ros
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)

set(3RDPARTY_DIR ${PROJECT_SOURCE_DIR}/../../../workspace/3rdparty)
set(PCL_INCLUDE_DIRS ${3RDPARTY_DIR}/pcl_1.9.1/include/pcl-1.9)
set(PCL_LIBRARY_DIRS ${3RDPARTY_DIR}/pcl_1.9.1/lib)
file(GLOB PCL_LIBRARIES ${PCL_LIBRARY_DIRS}/*.so*)
message(STATUS "PCL_INCLUDE_DIRS:" ${PCL_INCLUDE_DIRS})
message(STATUS "PCL_LIBRARY_DIRS:" ${PCL_LIBRARY_DIRS})

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES lidar_imu_calib
#  CATKIN_DEPENDS roscpp rospy sensor_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${PROJECT_SOURCE_DIR}/../ndt_omp-master/include
  ${catkin_INCLUDE_DIRS}
)

include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN3_DEFINITIONS})

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

link_directories(${PROJECT_SOURCE_DIR}/../../devel/lib)

add_library(calib_lidar_imu src/calib_lidar_imu.cpp)
target_link_libraries(calib_lidar_imu ${catkin_LIBRARIES} ${PCL_LIBRARIES} ndt_omp)

add_executable(calib_lidar_imu_node apps/calib_warp.cpp)
target_link_libraries(calib_lidar_imu_node calib_lidar_imu)

add_executable(calib_sim apps/test_sim.cpp)