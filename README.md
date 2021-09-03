# calib_lidar_imu
Linear Rotation Calibration about Lidar-Imu
![image](https://github.com/chengwei0427/calib_lidar_imu/blob/main/doc/flow.png)

## Dependencies

- ROS (tested on Kinetic16.04)

- PCL1.9.1

- Eigen

- ndt_omp and fast_gicp (https://github.com/koide3)

## Build

1. install and build ROS,PCL

2. run

   ```
   roslaunch calib_lidar_imu calib_lidar_imu.launch
   ```

## calib result
![image](https://github.com/chengwei0427/calib_lidar_imu/blob/main/doc/res.png)


![image](https://github.com/chengwei0427/calib_lidar_imu/blob/main/doc/cloud_map.png)

## reference
[https://blog.csdn.net/u012700322/article/details/117021836]