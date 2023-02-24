FROM ubuntu:jammy
MAINTAINER Miquel Massot miquel.massot-campos@soton.ac.uk

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV ROS_DISTRO humble

# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && \
    apt-get install -q -y --no-install-recommends \
    curl \
    gnupg2 \
    lsb-release \
    tzdata \
    dirmngr \
    gnupg2 \
    build-essential \
    git \
    openssh-client \
    ca-certificates \
    && rm -rf /var/lib/apt/lists/*

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# install packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool \
    python3-pip \
    python3-dev \
    ros-humble-ros-base=0.10.0-1* \
    && rm -rf /var/lib/apt/lists/* \
    && pip install --no-cache-dir setuptools==58.2.0

# get pi-top SDK libs
RUN apt-get update && apt-get install libsystemd-journal
RUN git clone https://github.com/pi-top/pi-top-Python-SDK.git /opt/pi-top-SDK
RUN cd /opt/pi-top-SDK && bash dev-install.sh

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
