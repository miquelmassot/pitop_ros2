FROM ubuntu:jammy

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV ROS_DISTRO humble

# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && \
    apt-get install -q -y --no-install-recommends \
    tzdata \
    dirmngr \
    gnupg2 \
    build-essential \
    git \
    && rm -rf /var/lib/apt/lists/*

# setup sources.list
RUN echo "deb http://packages.ros.org/ros2/ubuntu jammy main" > /etc/apt/sources.list.d/ros2-latest.list \
    && apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 \
    # install packages
    && apt-get update && apt-get install -q -y --no-install-recommends \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool \
    python3-pip \
    ros-humble-ros-base=0.10.0-1* \
    && rm -rf /var/lib/apt/lists/* \
    && pip install --no-cache-dir setuptools==58.2.0

# bootstrap rosdep
RUN rosdep init && \
    rosdep update --rosdistro $ROS_DISTRO

# setup colcon mixin and metadata
RUN colcon mixin add default \
    https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && \
    colcon mixin update && \
    colcon metadata add default \
    https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml && \
    colcon metadata update

RUN mkdir -p /ros2_ws/src/pitop_ros2 \
    && git clone https://github.com/babakhani/rplidar_ros2.git /ros2_ws/src/rplidar_ros

WORKDIR /ros2_ws
COPY . /ros2_ws/src/pitop_ros2
RUN /bin/bash -c 'source /opt/ros/$ROS_DISTRO/setup.bash; colcon build'
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc
WORKDIR /ros2_ws/src

# setup entrypoint
COPY ./ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]