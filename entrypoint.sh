#!/bin/bash
set -e

# Build the acton workspace
cd ~/ros_ws && \
    colcon build &&\
    source ~/ros_ws/install/setup.bash

# Source ROS 2 setup
source /opt/ros/humble/setup.bash

# Source PX4
cd ~/PX4-Autopilot

# Start PX4 SITL
make px4_sitl gz_x500 &

# Wait for PX4 to start
sleep 10

# Start MAVROS talking over UDP to PX4
ros2 launch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557" &

# Wait for MAVROS to start
sleep 10

ros2 param set /mavros/param  NAV_DLL_ACT 0 &
ros2 param set /mavros/param  COM_RC_IN_MODE 4 &
ros2 param set /mavros/param  COM_RCL_EXCEPT 4 &

# Start mission node
ros2 run mission_control mission_control &

# Keep the container running
wait
