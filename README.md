# [SuperOdom](https://github.com/superxslam/SuperOdom) converter to [HDMapping](https://github.com/MapsHD/HDMapping)

## Hint

Please change branch to [Bunker-DVI-Dataset-reg-1](https://github.com/MapsHD/benchmark-SuperOdometry-to-HDMapping/tree/Bunker-DVI-Dataset-reg-1) for quick experiment.


## Example Dataset:

Download the dataset from [Bunker DVI Dataset](https://charleshamesse.github.io/bunker-dvi-dataset/)

## Intended use

This small toolset allows to integrate SLAM solution provided by [SuperOdom](https://github.com/superxslam/SuperOdom) with [HDMapping](https://github.com/MapsHD/HDMapping).
This repository contains ROS 2 workspace that :
  - submodule to tested revision of superOdom
  - a converter that listens to topics advertised from odometry node and save data in format compatible with HDMapping.

## Dependecies
```shell
sudo apt install libgoogle-glog-dev libtbb-dev
```


## Building

Clone the repo
```shell
mkdir -p ~/superodom_ws/src
cd ~/superodom_ws/src
git clone https://github.com/MapsHD/benchmark-SuperOdometry-to-HDMapping.git . --recursive
cd ~/superodom_ws
colcon build
source install/setup.bash
```

## Usage - data SLAM:

Prepare recorded bag with estimated odometry:

In first terminal launch odometry:
```shell
source install/setup.bash
ros2 launch super_odometry livox_mid360.launch.py
```

In second terminal play bag:
```shell
source install/setup.bash
ros2 bag play /path/to/your/bag.mcap --clock
```

In third terminal record bag:
```shell
source install/setup.bash
ros2 bag record /registered_scan /state_estimation -o my_recording --storage sqlite3
```

## Usage - conversion:

```shell
source install/setup.bash
ros2 run superOdom-to-hdmapping listener my_recording output_hdmapping
```

## Record the bag file:

```shell
ros2 bag record /registered_scan /state_estimation -o my_recording --storage sqlite3
```

## SuperOdom Launch:

```shell
source install/setup.bash
ros2 launch super_odometry livox_mid360.launch.py
```

## During the record (if you want to stop recording earlier) / after finishing the bag:

```shell
In the terminal where the ros record is, interrupt the recording by CTRL+C
Do it also in ros launch terminal by CTRL+C.
```

## Usage - Conversion (ROS bag to HDMapping, after recording stops):

```shell
source install/setup.bash
ros2 run superOdom-to-hdmapping listener my_recording output_hdmapping
```
