FROM galactic_env:latest
# ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

# Setup colcon_cd
RUN echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc
RUN echo "export _colcon_cd_root=/opt/ros/galactic/" >> ~/.bashrc

# Setup colcon tab completion
RUN echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc


COPY /ros2_ws/ /ros2_ws/
RUN source /opt/ros/galactic/setup.bash && \
    apt-get update --fix-missing && \
    cd /ros2_ws  && \
    # This installs dpdencies declared in package.xml from all pkgs in the src folder for ROS 2 Galactic
    rosdep install -i --from-path src --rosdistro galactic -y && \
    # Build env_ws
    colcon build --symlink-install 

# Set Working directory
WORKDIR '/ros2_ws'
# ENTRYPOINT ["/ros_entrypoint.sh"]
