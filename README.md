# [SuperOdom](https://github.com/superxslam/SuperOdom) ROS 2 to [HDMapping](https://github.com/MapsHD/HDMapping)

## Quick Start

For automated ROS 2 workflow with Bunker DVI Dataset, use branch [Bunker-DVI-Dataset-reg-1](https://github.com/MapsHD/benchmark-SuperOdom-to-HDMapping/tree/Bunker-DVI-Dataset-reg-1)

## Overview

This repository integrates [SuperOdom](https://github.com/superxslam/SuperOdom) (ROS 2 LiDAR-Inertial Odometry) with [HDMapping](https://github.com/MapsHD/HDMapping).

**Components:**
- SuperOdom: ROS 2 Humble LiDAR-Inertial odometry (SLAM)
- superOdom-to-hdmapping: Listener that converts odometry topics to HDMapping format

## Requirements

- ROS 2 Humble
- Docker (for containerized build)
- Colcon build tools
- Data: Livox bag format (use Bunker-DVI-Dataset-reg-1 branch for automated setup)

## Building with Docker

```bash
docker build -t superodom_humble .
```

## Building Native ROS 2 Workspace

```bash
mkdir -p ~/superodom_ws/src
cd ~/superodom_ws/src
git clone https://github.com/MapsHD/benchmark-SuperOdom-to-HDMapping.git --recursive
cd ~/superodom_ws
colcon build
source install/setup.bash
```

## Running SuperOdom with ROS Bags

### Step 1: Launch SuperOdom

Terminal 1:
```bash
source install/setup.bash
ros2 launch super_odometry livox_mid360.launch.py
```

### Step 2: Play recorded bag

Terminal 2:
```bash
source install/setup.bash
ros2 bag play /path/to/your/bag.mcap --clock
```

### Step 3: Record odometry output (run in parallel with Steps 1-2)

Terminal 3:
```bash
source install/setup.bash
ros2 bag record /lio/odom /lio/cloud_world -o my_recording
```

### Step 4: Convert to HDMapping format

After recording completes, run:
```bash
source install/setup.bash
ros2 run superOdom-to-hdmapping listener my_recording output_hdmapping
```

## Output Files

The converter generates:
- `output_hdmapping/session.json`
- `output_hdmapping/poses.reg`
- `output_hdmapping/scan_lio_*.laz`
- `output_hdmapping/trajectory_lio_*.csv`

Open in HDMapping using `multi_view_tls_registration_step_2` tool.

## Supported Sensors

- Livox Mid360 (configured by default in launch files)
- Velodyne VLP-16
- Ouster OS1-128

See `src/SuperOdom/super_odometry/launch/` for other sensor configurations.
