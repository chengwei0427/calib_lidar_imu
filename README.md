# calib_lidar_imu
Linear Rotation Calibration about Lidar-Imu
![alt text](https://github.com/chengwei0427/calib_lidar_imu/tree/main/doc/flow.png?raw=true)

## Dependencies

- ROS (tested on Kinetic16.04)

- PCL1.9.1

- Eigen


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