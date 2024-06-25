ARG ROS_DISTRO # override with --build-arg ROS_DISTRO=<something>
FROM ros:${ROS_DISTRO}-ros-base-focal

# install dependencies package
RUN apt-get update && apt-get install -y git

WORKDIR /rplidar_ws

# Setup workspace
RUN git clone -b ros2 https://github.com/Slamtec/rplidar_ros.git /rplidar_ws/src/rplidar_ros && \
    rosdep update --rosdistro ${ROS_DISTRO} && \
    rosdep install --from-paths src --ignore-src -y

# Build 
SHELL ["/bin/bash","-c"]
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# launch ros package
CMD ["ros2", "launch", "rplidar_ros", "rplidar_a1_launch.py"]