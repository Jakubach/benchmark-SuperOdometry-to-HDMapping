# [SuperOdom](https://github.com/superxslam/SuperOdom) converter to [HDMapping](https://github.com/MapsHD/HDMapping)

## Hint

Please change branch to [Bunker-DVI-Dataset-reg-1](https://github.com/MapsHD/benchmark-SuperOdom-to-HDMapping/tree/Bunker-DVI-Dataset-reg-1) for quick experiment.


## Example Dataset: 

Download the dataset from [Bunker DVI Dataset](https://charleshamesse.github.io/bunker-dvi-dataset/)  

## Intended use 

This small toolset allows to integrate SLAM solution provided by [superOdom](https://github.com/superxslam/SuperOdom) with [HDMapping](https://github.com/MapsHD/HDMapping).
This repository contains ROS 1 workspace that :
  - submodule to tested revision of superOdom
  - a converter that listens to topics advertised from odometry node and save data in format compatible with HDMapping.

## Dependecies
```shell
sudo apt install libgoogle-glog-dev libtbb-dev
```


## livox_ros_driver

```shell
git clone https://github.com/Livox-SDK/livox_ros_driver.git ws_livox/src
cd ws_livox
catkin_make

Use the following command to update the current ROS package environment :

source ./devel/setup.sh
```
## Building

Clone the repo
```shell
mkdir -p /test_ws/src
cd /test_ws/src
git clone https://github.com/marcinmatecki/SuperOdom-to-HDMapping.git --recursive
cd ..
catkin_make
```

## Usage - data SLAM:

Prepare recorded bag with estimated odometry:

In first terminal record bag:
```shell
rosbag record /lio/odom /lio/cloud_world
```

and start odometry:
```shell 
cd /test_ws/
source ./devel/setup.sh # adjust to used shell
roslaunch super_lio Livox_mid360.launch
rosbag play your bag file
```

## Usage - conversion:

```shell
cd /test_ws/
source ./devel/setup.sh # adjust to used shell
rosrun superOdom-to-hdmapping listener <recorded_bag> <output_dir>
```

## Record the bag file:

```shell
rosbag record /lio/odom /lio/cloud_world
```

## Super LIO Launch:

```shell
cd /test_ws/
source ./devel/setup.sh # adjust to used shell
roslaunch super_lio Livox_mid360.launch
rosbag play your bag file
```

## During the record (if you want to stop recording earlier) / after finishing the bag:

```shell
In the terminal where the ros record is, interrupt the recording by CTRL+C
Do it also in ros launch terminal by CTRL+C.
```

## Usage - Conversion (ROS bag to HDMapping, after recording stops):

```shell
cd /test_ws/
source ./devel/setup.sh # adjust to used shell
rosrun superOdom-to-hdmapping listener <recorded_bag> <output_dir>
```
