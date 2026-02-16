FROM ubuntu:22.04

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    curl gnupg lsb-release software-properties-common sudo \
    build-essential git cmake \
    python3-pip \
    libceres-dev libeigen3-dev libtbb-dev libomp-dev \
    nlohmann-json3-dev \
    libpcl-dev \
    libboost-all-dev \
    libatlas-base-dev \
    libsuitesparse-dev \
    libgoogle-glog-dev \
    libglew-dev \
    ca-certificates \
    tmux wget unzip \
    && rm -rf /var/lib/apt/lists/*

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/ros2.list

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-desktop \
    python3-rosdep \
    ros-humble-tf2 \
    ros-humble-cv-bridge \
    ros-humble-pcl-conversions \
    ros-humble-image-transport \
    ros-humble-image-transport-plugins \
    ros-humble-pcl-ros \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --no-cache-dir colcon-common-extensions

# Install Sophus
RUN git clone http://github.com/strasdat/Sophus.git /tmp/sophus && \
    cd /tmp/sophus && git checkout 97e7161 && \
    mkdir build && cd build && \
    cmake .. -DBUILD_TESTS=OFF && \
    make -j$(nproc) && make install && \
    rm -rf /tmp/sophus

# Install GTSAM
RUN git clone https://github.com/borglab/gtsam.git /tmp/gtsam && \
    cd /tmp/gtsam && git checkout 4abef92 && \
    mkdir build && cd build && \
    cmake .. \
      -DGTSAM_USE_SYSTEM_EIGEN=ON \
      -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
      -DGTSAM_BUILD_TESTS=OFF \
      -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF && \
    make -j$(nproc) && make install && \
    ldconfig && \
    rm -rf /tmp/gtsam

WORKDIR /ros2_ws

COPY ./src ./src

# Clone LASzip v3.4.3 (submodule may be empty after branch checkout)
RUN git clone --branch 3.4.3 --depth 1 https://github.com/LASzip/LASzip.git \
    src/superOdom-to-hdmapping/src/3rdparty/LASzip

RUN ldconfig

# Replace livox_ros_driver2 -> livox_ros_driver (bag has v1 CustomMsg type)
RUN sed -i \
  -e 's|livox_ros_driver2|livox_ros_driver|g' \
  src/SuperOdom/super_odometry/CMakeLists.txt \
  src/SuperOdom/super_odometry/package.xml \
  src/SuperOdom/super_odometry/src/FeatureExtraction/featureExtraction.cpp \
  src/SuperOdom/super_odometry/include/super_odometry/FeatureExtraction/featureExtraction.h

# Configure topics for Livox MID360 bag
RUN sed -i \
  -e 's|imu_topic: "/imu/data"|imu_topic: "/livox/imu"|' \
  -e 's|laser_topic: "/lidar/scan"|laser_topic: "/livox/lidar"|' \
  src/SuperOdom/super_odometry/config/livox_mid360.yaml

# Enable use_sim_time for bag playback
RUN sed -i \
  -e "s|value='false'|value='true'|" \
  src/SuperOdom/super_odometry/launch/livox_mid360.launch.py

RUN source /opt/ros/humble/setup.bash && \
    colcon build

ARG UID=1000
ARG GID=1000
RUN groupadd -g $GID ros && \
    useradd -m -u $UID -g $GID -s /bin/bash ros

RUN python3 -m pip install "rosbags==0.10.5"

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

CMD ["bash"]
