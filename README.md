# DPLR_SLAM2
## Dandelion detection and 3D LIDAR SLAM using intelrealsense L515 camera for the Dandelion picking legged robot project at BIRDS lab UMICH

### This repo is an extension work of [SSL_SLAM](https://github.com/wh200720041/SSL_SLAM). Similar to RTABMAP, dplr_SLAM separates the mapping module and localization module. Map saving and map optimization is enabled in the mapping unit. Map loading and localization is enabled in the localziation unit.

Running speed: 20 Hz on Intel NUC, 30 Hz on PC

## 1. DPLR_SLAM example
### 1.1 Dandelion detection and map building example
<p align='center'>
<a href="https://www.youtube.com/watch?v=4HYE_WGkvp8">
<img width="65%" src="/img/lc_test5_v3.gif"/>
</a>
</p>


## 2. Prerequisites
### 2.1 **Ubuntu** and **ROS**
Ubuntu 64-bit 18.04.

ROS Melodic. [ROS Installation](http://wiki.ros.org/ROS/Installation)

### 2.2. **Ceres Solver**
Follow [Ceres Installation](http://ceres-solver.org/installation.html).

### 2.3. **PCL**
Follow [PCL Installation](http://www.pointclouds.org/downloads/linux.html).

Tested with 1.8.1

### 2.4. **GTSAM**
Follow [GTSAM Installation](https://gtsam.org/get_started/).

### 2.5. **Trajectory visualization**
For visualization purpose, this package uses hector trajectory sever, you may install the package by 
```
sudo apt-get install ros-melodic-hector-trajectory-server
```
Alternatively, you may remove the hector trajectory server node if trajectory visualization is not needed

## 3. Sensor Setup
If you have new Realsense L515 sensor, you may follow the below setup instructions

### 3.1 L515
<p align='center'>
<img width="35%" src="/img/realsense_L515.jpg"/>
</p>

### 3.2 Librealsense
Follow [Librealsense Installation](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md)

### 3.3 Realsense_ros
Copy [realsense_ros](https://github.com/IntelRealSense/realsense-ros) package to your catkin folder
```
    cd ~/catkin_ws/src
    git clone https://github.com/IntelRealSense/realsense-ros.git
    cd ..
    catkin_make
```

## 4. Build 
### 4.1 Clone repository:
```
    cd ~/catkin_ws/src
    git clone https://github.com/srirampr/dplr_slam2.git
    cd ..
    catkin_make
    source ~/catkin_ws/devel/setup.bash
```

### 4.2 Download test rosbag
You may download our recorded data: [MappingTest.bag](https://drive.google.com/file/d/1XRXKkq3TsUiM4o9_bWL8t9HqWHswfgWo/view?usp=sharing) (3G) and [LocalizationTest.bag](https://drive.google.com/file/d/1-5j_jgraus0gJkpFRZS5hFUiKlT7aQtG/view?usp=sharing) (6G)if you dont have realsense L515, and by defult the file should be under home/user/Downloads

unzip the file (it may take a while to unzip) 
```
cd ~/Downloads
unzip LocalizationTest.zip
unzip MappingTest.zip
```


### 4.4 Mapping and Localization

Type
```
    roslaunch dplr_slam2 dplr_slam2_mapping.launch
```
]

### 4.5 Parameters Explanation
The map size depends on number of keyframes used. The more keyframes used for map buildin, the larger map will be. 

min_map_update_distance: distance threshold to add a keyframe. higher means lower update rate. 
min_map_update_angle: angle threshold to add a keyframe. higher means lower update rate. 
min_map_update_frame: time threshold to add a keyframe. higher means lower update rate. 




