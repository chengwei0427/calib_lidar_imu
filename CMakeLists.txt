cmake_minimum_required(VERSION 3.5.0)
project(calib_lidar_imu)

add_definitions(-msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2)
## Compile as C++11, supported in ROS Kinetic and newer
SET( CMAKE_CXX_FLAGS "-std=c++11 -O3")
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
set(CMAKE_BUILD_TYPE "RELEASE")


# set( CMAKE_CXX_FLAGS_RELEASE "-fpermissive -fPIC ")
# set(CMAKE_CXX_FLAGS "-O3  -w -fopenmp -pthread")
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

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

# find_package(Boost REQUIRED COMPONENTS system)
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(Ceres REQUIRED)
# find_package(GTSAM REQUIRED)
set(3RDPARTY_DIR /home/cc/workspace/3rdparty)
set(GTSAM_INCLUDE_DIR ${3RDPARTY_DIR}/gtsam4/include)
set(GTSAM_LIBRARY_DIRS ${3RDPARTY_DIR}/gtsam4/lib)

# find_package(PCL REQUIRED)

set(PCL_INCLUDE_DIRS /home/cc/workspace/3rdparty/pcl1.7.2/include/pcl-1.7)
set(PCL_LIBRARY_DIRS /home/cc/workspace/3rdparty/pcl1.7.2/lib)
file(GLOB PCL_LIBRARIES ${PCL_LIBRARY_DIRS}/*.so)
message(STATUS "PCL_INCLUDE_DIRS:" ${PCL_INCLUDE_DIRS})
message(STATUS "PCL_LIBRARY_DIRS:" ${PCL_LIBRARY_DIRS})

catkin_package(
 INCLUDE_DIRS include thirdparty
#  LIBRARIES lidar_imu_calib
 CATKIN_DEPENDS roscpp rospy sensor_msgs rosbag
#  DEPENDS system_lib
)

include_directories(
  include
  thirdparty
  ${catkin_INCLUDE_DIRS}
)

include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN3_DEFINITIONS})

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

link_directories(${GTSAM_LIBRARY_DIRS})
include_directories(${GTSAM_INCLUDE_DIR} ${CERES_INCLUDE_DIRS})


# add_library(calib_lidar_imu src/calib_lidar_imu.cpp src/registrations.cpp)
# target_link_libraries(calib_lidar_imu ${catkin_LIBRARIES} ${PCL_LIBRARIES} ndt_omp fast_gicp)

add_executable(calib_lidar_imu_node 
                    apps/calib_warp.cpp
                    src/calib_lidar_imu.cpp 
                    src/registrations.cpp
                    # thirdparty/fast_gicp/gicp/lsq_registration.cpp
                    # thirdparty/fast_gicp/gicp/fast_gicp.cpp
                    # thirdparty/fast_gicp/gicp/fast_gicp_st.cpp
                    # thirdparty/fast_gicp/gicp/fast_vgicp.cpp
                    thirdparty/pclomp/voxel_grid_covariance_omp.cpp
                    thirdparty/pclomp/ndt_omp.cpp
                    thirdparty/pclomp/gicp_omp.cpp)
target_link_libraries(calib_lidar_imu_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES} libgtsam.so)

add_executable(calib_sim apps/test_sim.cpp)