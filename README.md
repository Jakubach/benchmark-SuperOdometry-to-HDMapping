# SuperOdometry to HDMapping simplified instruction

## Step 1 (prepare data)
Download the dataset `reg-1.bag` by clicking [link](https://cloud.cylab.be/public.php/dav/files/7PgyjbM2CBcakN5/reg-1.bag) (it is part of [Bunker DVI Dataset](https://charleshamesse.github.io/bunker-dvi-dataset)) and convert with [tool](https://github.com/MapsHD/mandeye_to_bag). Data conversion workflow has 3 steps:
1. ros1-to-hdmapping
2. hdmapping-to-ros1 (creating `reg-1-convert.bag`)
3. rosbags-convert (ros1 to ros2 - done in Step 3)

Step 1 performs the first two conversions using mandeye_to_bag:

 ```shell
cd ~/hdmapping-benchmark/mandeye_to_bag
./mandeye-convert.sh ~/hdmapping-benchmark/data/reg-1.bag ~/hdmapping-benchmark/data/reg-1-convert ros1-to-hdmapping
./mandeye-convert.sh ~/hdmapping-benchmark/data/reg-1-convert ~/hdmapping-benchmark/data/reg-1-convert.bag hdmapping-to-ros1
```

The final conversion (rosbags-convert) happens in **Step 3**, which creates `reg-1-ros2-lidar` folder.
This folder will be your input for **Step 4**.

## Step 2 (prepare docker)
Run following commands in terminal

```shell
mkdir -p ~/hdmapping-benchmark
cd ~/hdmapping-benchmark
git clone https://github.com/MapsHD/benchmark-SuperOdometry-to-HDMapping.git --recursive
cd benchmark-SuperOdometry-to-HDMapping
git checkout Bunker-DVI-Dataset-reg-1
git submodule update --init --recursive
docker build -t superodom_humble .
```

## Step 3 (Convert data)
We now convert data from ROS1 to ROS2

```shell
docker run -it -v ~/hdmapping-benchmark/data:/data --user 1000:1000 superodom_humble /bin/bash
cd /data
rosbags-convert --src reg-1-convert.bag --dst reg-1-ros2-lidar
exit
```

The conversion produces a bag with message type `livox_ros_driver/msg/CustomMsg`,
but SuperOdometry expects `livox_ros_driver2/msg/CustomMsg`. Fix the type name:

```shell
python3 -c "
import sqlite3, pathlib

bag_dir = pathlib.Path('$HOME/hdmapping-benchmark/data/reg-1-ros2-lidar')

# Fix SQLite database
for db in bag_dir.glob('*.db3'):
    conn = sqlite3.connect(str(db))
    conn.execute(\"UPDATE topics SET type='livox_ros_driver2/msg/CustomMsg' WHERE type='livox_ros_driver/msg/CustomMsg'\")
    conn.commit()
    conn.close()

# Fix metadata.yaml
meta = bag_dir / 'metadata.yaml'
meta.write_text(meta.read_text().replace('livox_ros_driver/msg/CustomMsg', 'livox_ros_driver2/msg/CustomMsg'))

print('Type fixed: livox_ros_driver -> livox_ros_driver2')
"
```

## Step 4 (run docker, file 'reg-1-ros2-lidar' should be in '~/hdmapping-benchmark/data')
You are now back on the HOST (after `exit` in Step 3)

```shell
cd ~/hdmapping-benchmark/benchmark-SuperOdometry-to-HDMapping
chmod +x docker_session_run-ros2-superOdom.sh
./docker_session_run-ros2-superOdom.sh ~/hdmapping-benchmark/data/reg-1-ros2-lidar ~/hdmapping-benchmark/data
```

## Step 5 (Open and visualize data)
Expected data should appear in ~/hdmapping-benchmark/data/output_hdmapping-superOdom
Use tool [multi_view_tls_registration_step_2](https://github.com/MapsHD/HDMapping) to open session.json from ~/hdmapping-benchmark/data/output_hdmapping-superOdom.

You should see following data

lio_initial_poses.reg

poses.reg

scan_lio_*.laz

session.json

trajectory_lio_*.csv

## Contact email
januszbedkowski@gmail.com
