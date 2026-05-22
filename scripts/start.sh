#!/bin/bash
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-1}
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}
export CYCLONEDDS_URI=${CYCLONEDDS_URI:-file:///home/windshape/.ros/cyclonedds.xml}

SCRIPT_DIR=$(cd $(dirname $0); pwd)

echo "Start robot system..."

# 1. 清理旧系统
$SCRIPT_DIR/clean.sh

# 2. source ros
source /opt/ros/humble/setup.bash

# 4. 启动系统
ros2 launch manipulator demo.launch.py