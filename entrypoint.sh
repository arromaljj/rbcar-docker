#!/bin/bash


cp -R /tmp/ros_ws/* /root/ros_ws/


# Source ROS setup
source /opt/ros/noetic/setup.bash

# Copy files from /root/src to /root/ros_ws/src
echo "Copying files from /root/src to /root/ros_ws/src..."
# Check if the directory is empty, if not copy 
if [ "$(ls -A /root/src)" ]; then
    cp -R /root/src/. /root/ros_ws/src/
else
    echo "Source directory is empty. Nothing to copy."
fi

echo "Building the project"

cd /root/ros_ws
catkin_make

source /root/ros_ws/devel/setup.bash

# # Execute the command passed to docker run
# exec "$@"
# Drop into a bash shell
exec /bin/bash