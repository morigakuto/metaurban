#!/bin/bash
set -e

# Source ROS2
source /opt/ros/humble/setup.bash

# Source the ROS2 bridge workspace
if [ -f /metaurban/bridges/ros_bridge/install/setup.bash ]; then
    source /metaurban/bridges/ros_bridge/install/setup.bash
fi

exec "$@"
