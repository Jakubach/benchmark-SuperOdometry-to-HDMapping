# SuperOdometry to HDMapping simplified instruction

## Step 1 (prepare data)
Download the dataset `reg-1.bag` by clicking [link](https://cloud.cylab.be/public.php/dav/files/7PgyjbM2CBcakN5/reg-1.bag) (it is part of [Bunker DVI Dataset](https://charleshamesse.github.io/bunker-dvi-dataset)) and convert with [tool](https://github.com/MapsHD/mandeye_to_bag). Data conversion is performed using the mandeye_to_bag tool, with the workflow:
ros1-to-hdmapping → hdmapping-to-ros1(name it reg-1-convert.bag) → rosbags-convert (ros1 to ros2)

 Click [here](https://github.com/MapsHD/mandeye_to_bag) for tool and use these commands:
 ```shell
cd ~/hdmapping-benchmark/mandeye_to_bag
./mandeye-convert.sh ~/hdmapping-benchmark/data/reg-1.bag ~/hdmapping-benchmark/data/reg-1-convert ros1-to-hdmapping
./mandeye-convert.sh ~/hdmapping-benchmark/data/reg-1-convert ~/hdmapping-benchmark/data/reg-1-convert.bag hdmapping-to-ros1
```


Folder 'reg-1-ros2-lidar' is an input for further calculations.
It should be located in '~/hdmapping-benchmark/data'.

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
rosbags-convert --src reg-1-convert.bag/reg-1-convert --dst reg-1-ros2-lidar
```

close terminal

## Step 4 (run docker, file 'reg-1-ros2-lidar' should be in '~/hdmapping-benchmark/data')
open new terminal

```shell
cd ~/hdmapping-benchmark/benchmark-SuperOdometry-to-HDMapping
chmod +x docker_session_run-ros2-superOdom.sh
cd ~/hdmapping-benchmark/data
~/hdmapping-benchmark/benchmark-SuperOdometry-to-HDMapping/docker_session_run-ros2-superOdom.sh reg-1-ros2-lidar .
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
