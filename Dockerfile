# Use the official ROS Noetic base image
FROM --platform=linux/amd64 ros:noetic


# Install necessary packages
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-wstool \
    git \
    ros-noetic-costmap-2d \
    ros-noetic-rviz \
    ros-noetic-controller-manager \
    ros-noetic-gazebo-ros \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-gazebo-ros-control \
    python3-rospkg \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    x11-apps 
    # && rm -rf /var/lib/apt/lists/*

RUN apt-get install -y ros-noetic-robot-state-publisher

# Set the default shell to bash
SHELL ["/bin/bash", "-c"]

# Source ROS setup.bash
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# Create and set the working directory
WORKDIR /root/ros_ws

# Initialize a catkin workspace
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && \
    mkdir -p src && \
    cd src && \
    catkin_init_workspace && \
    cd .. && \
    catkin_make"

# Source the workspace setup.bash
RUN echo "source /root/ros_ws/devel/setup.bash" >> ~/.bashrc






RUN echo "hello"
RUN wstool init src
COPY ./.rosinstall ./src/.rosinstall

RUN git clone https://github.com/arromaljj/camera_info_manager_py.git src/camera_info_manager_py
RUN rosdep install --from-paths . -i --rosdistro noetic
RUN cp -r src/camera_info_manager_py/src/camera_info_manager /opt/ros/noetic/lib/python3/dist-packages/
RUN wstool update -t src
RUN rosdep install --from-paths src -i --ignore-src -y

RUN sudo dpkg -i src/rbcar_common/rbcar_control/lib/ros-noetic-robotnik-msgs_2.2.0-0focal_amd64.deb
RUN sudo dpkg -i src/rbcar_common/rbcar_control/lib/ros-noetic-ackermann-drive-controller_0.0.0-0focal_amd64.deb 

# # Create XDG_RUNTIME_DIR directory with proper permissions
RUN mkdir -p /tmp/runtime-root && chmod 0700 /tmp/runtime-root

# Copy the entrypoint script
COPY entrypoint.sh /root/entrypoint.sh
RUN chmod +x /root/entrypoint.sh

# Copy the workspace to a temporary location
RUN cp -R /root/ros_ws /tmp/ros_ws

# Set the entrypoint
ENTRYPOINT ["/root/entrypoint.sh"]
# # Set the entrypoint to the launch file
# ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/noetic/setup.bash && source /root/ros_ws/devel/setup.bash"]

