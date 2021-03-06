<a href="#"><img src="https://img.shields.io/badge/c++-%2300599C.svg?style=flat&logo=c%2B%2B&logoColor=white"></img></a>
  <a href="#"><img src="https://img.shields.io/github/stars/chengwei0427/calib_lidar_imu"></img></a>
  <a href="#"><img src="https://img.shields.io/github/forks/chengwei0427/calib_lidar_imu"></img></a>
  <a href="#"><img src="https://img.shields.io/github/repo-size/chengwei0427/calib_lidar_imu"></img></a>
  <a href="https://github.com/chengwei0427/calib_lidar_imu/issues"><img src="https://img.shields.io/github/issues/chengwei0427/calib_lidar_imu"></img></a>
  <a href="https://github.com/chengwei0427/calib_lidar_imu/graphs/contributors"><img src="https://img.shields.io/github/contributors/chengwei0427/calib_lidar_imu?color=blue"></img></a>


# calib_lidar_imu
Linear Rotation Calibration about Lidar-Imu
![image](https://github.com/chengwei0427/calib_lidar_imu/blob/main/doc/flow.png)

## Dependencies

- ROS (tested on Kinetic16.04)

- PCL1.9.1

- Eigen

- Ceres

- Gtsam


## Build

1. install and build ROS,PCL,ceres,gtsam

2. run

   ```
   roslaunch calib_lidar_imu calib_lidar_imu.launch
   ```

## calib result
<p align='center'>
    <img src="./doc/res.png" alt="drawing" width="800"/>
</p>

<p align='center'>
    <img src="./doc/cloud_map.png" alt="drawing" width="800"/>
</p>

## TODO

  - [ ]	Use scan-to-map[like loam] to solve lidar odometry pose instead of icp/ndt.
  - [ ]	Use preintegration to solve the imu pose.
## reference
[https://blog.csdn.net/u012700322/article/details/117021836]