FROM ubuntu:22.04

SHELL ["/bin/bash", "-o", "pipefail", "-o", "errexit", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# Set locale to GB UTF-8
RUN apt-get update && apt-get install -y locales && \
    locale-gen en_GB en_GB.UTF-8 && \
    update-locale LC_ALL=en_GB.UTF-8 LANG=en_GB.UTF-8

ENV LANG=en_GB.UTF-8

# Install dependencies
RUN apt-get update; \
    apt-get install -y \
        sudo \
        lsb-release \
        cmake \
        git \
        build-essential \
        software-properties-common \
        curl \
        wget \
        gnupg

# Install ROS 2 Humble Desktop and development tools
RUN add-apt-repository universe && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt-get update && apt-get upgrade -y && \
    apt-get install -y \
        ros-humble-desktop \
        ros-dev-tools \
        ros-humble-cv-bridge \
        ros-humble-ros-gz-bridge \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-vcstool \
        ros-humble-ament-cmake \
        ros-humble-ament-cmake-core \
        ros-humble-ament-package \
        ros-humble-rosidl-generator-py \
        ros-humble-rosidl-adapter \
        build-essential \
        cmake && \
    rm -rf /var/lib/apt/lists/*

RUN echo 'export PYTHONPATH=$PYTHONPATH:/opt/ros/humble/local/lib/python3.10/dist-packages' >> /etc/bash.bashrc

# Initialize rosdep
RUN rosdep init || true && \
    rosdep update


# Set up environment
RUN bash -c 'echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc'

# Set environment variables
ENV AMENT_PREFIX_PATH=/opt/ros/humble
ENV CMAKE_PREFIX_PATH=/opt/ros/humble
ENV COLCON_PREFIX_PATH=/opt/ros/humble
ENV LD_LIBRARY_PATH=/opt/ros/humble/lib
ENV PATH=/opt/ros/humble/bin:$PATH
ENV PYTHONPATH=/opt/ros/humble/lib/python3.10/site-packages
ENV ROS_DISTRO=humble
ENV ROS_PYTHON_VERSION=3
ENV ROS_VERSION=2
ENV GZ_SIM_RESOURCE_PATH=/usr/share/gazebo
ENV GZ_SIM_SYSTEM_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/gazebo/plugins

# Install Gazebo Harmonic and dependencies
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list && \
    apt-get update && \
    apt-get install -y \
        gz-harmonic \
        libgz-sim8-dev \
        libgz-math8-dev \
        libgz-plugin2-dev \
        libgz-common5-dev \
        libgz-transport13-dev \
        ros-humble-ros-gz-interfaces \
        ros-humble-ros-gz-bridge \
        ros-humble-ros-gz-sim \
    && rm -rf /var/lib/apt/lists/*

# Install MAVROS
RUN apt-get update; \
    apt-get install -y ros-humble-mavros; \
    source /opt/ros/humble/setup.bash; \
    ros2 run mavros install_geographiclib_datasets.sh

# Configure User
ENV USER=ubuntu
RUN adduser --gecos '' ${USER} --disabled-password
RUN adduser ${USER} sudo
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
RUN usermod -a -G video ${USER}
RUN usermod -a -G render ${USER}
RUN chmod a+rwx /home
ENV PATH="${PATH}:/home/${USER}/.local/bin"

USER ${USER}

WORKDIR /home/${USER}

# Setting Gazebo version
ENV GZ_VERSION=harmonic

# Set Gazebo environment variables
ENV GZ_SIM_SYSTEM_PLUGIN_PATH=/home/${USER}/asv_sim_ws/install/lib:${GZ_SIM_SYSTEM_PLUGIN_PATH}


# Clone PX4 Autopilot
RUN git clone https://github.com/PX4/PX4-Autopilot.git --recursive

# Build PX4
WORKDIR /home/${USER}/PX4-Autopilot

RUN ./Tools/setup/ubuntu.sh && \
    DONT_RUN=1 make px4_sitl_default

ENV GZ_SIM_RESOURCE_PATH=/home/ubuntu/PX4-Autopilot-sim/Tools/simulation/gz:${GZ_SIM_RESOURCE_PATH}

# Install necessary python packages
RUN pip install opencv-contrib-python==4.6.0.66 &&\
    pip install "numpy<2" &&\
    pip install transforms3d &&\
    pip install seaborn
