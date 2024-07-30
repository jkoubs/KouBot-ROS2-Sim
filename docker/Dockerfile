# FROM galactic_env
FROM jkoubi/galactic_env:v1
# ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

# Setup colcon_cd
RUN echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc
RUN echo "export _colcon_cd_root=/opt/ros/galactic/" >> ~/.bashrc

# Setup colcon tab completion
RUN echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc


COPY /ros2_ws/ /ros2_ws/
# Install ultralytics for YOLOv8
#RUN pip3 install ultralytics
# Update rosdep database and install dependencies
RUN source /opt/ros/galactic/setup.bash && \
    apt-get update --fix-missing && \
    cd /ros2_ws/src  && \
    git clone -b ros2 https://github.com/Slamtec/rplidar_ros.git && \
    # This installs dpdencies declared in package.xml from all pkgs in the src folder for ROS 2 Galactic
    cd /ros2_ws/ && \
    rosdep install -i --from-path src --rosdistro galactic -y --skip-keys rviz && \
    # Build env_ws
    colcon build --symlink-install 

# Set Working directory
WORKDIR '/ros2_ws'
# ENTRYPOINT ["/ros_entrypoint.sh"]

# Image name: jkoubi/koubot_ros2:jk_rplidar