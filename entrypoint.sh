#!/bin/bash

# Check if the volume is empty
if [ -z "$(ls -A /root/ros_ws)" ]; then
    echo "Initializing ros_workspace volume..."
    cp -R /tmp/ros_ws/* /root/ros_ws/
else
    echo "ros_workspace volume already initialized."
fi

# Source ROS setup
source /opt/ros/noetic/setup.bash

# Check if the devel/setup.bash exists, if not, build the workspace
if [ ! -f /root/ros_ws/devel/setup.bash ]; then
    cd /root/ros_ws
    catkin_make
fi


source /root/ros_ws/devel/setup.bash

# Execute the command passed to docker run
exec "$@"