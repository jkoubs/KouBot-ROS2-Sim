# This repository holds all the development work related to the KouBot Project done by me.

# Table of Contents
 - [About](#about)
 - [Packages](#packages)
 - [Install](#installation-instructions)
 - [System at a glance](#system-at-a-glance)
 - [Simulation](#simulation)
   - [Autonomous Navigation](#autonomous-navigation)
   - [Manipulation Pipeline](#manipulation-pipeline)
   - [Perception Pipeline](#perception-pipeline)
   - [Fetch-and-Carry Mission](#fetch-and-carry-mission)
 - [Real Robot](#real-robot)
 - [Docker](#docker)

# About 

In progress ...

# Packages

In progress ...

# Install

## Launch container

We will build 2 images:
- <strong>galactic_tb_env</strong>: Allows to run turtlebot3 simulations in ROS 2 from a linux computer
- <strong>koubot_ros2</strong>: Built on top of <strong>galactic_tb_env</strong> image to set up the ros2_ws for developing the KouBot project.

Open a new terminal and git clone the following repositories:
```bash
git clone https://github.com/jkoubs/ros2_galactic.git
git clone https://github.com/jkoubs/KouBot-ROS2.git
```
Then build the images:
```bash
cd ros2_galactic/docker
docker build -f Dockerfile -t galactic_env .
cd ../..
cd koubot_ros2/docker
docker build -f Dockerfile -t koubot_ros2 ../
```
<strong><em>Note</em></strong>: <strong>../</strong> represents the PATH context which sets the target context one level above to the <strong>koubot_ros2</strong> directory in order to successfully execute the COPY command from the Dockerfile which will copy the <strong>ros2_ws</strong> inside the container.


Next we will create the container:

<strong><em>Requirement</em></strong>: To run GUI applications in Docker on Linux hosts, you have to run <strong>"xhost +local:root"</strong>. To disallow, <strong>"xhost -local:root"</strong>. For Windows and Mac hosts please check : [Running GUI applications in Docker on Windows, Linux and Mac hosts](https://cuneyt.aliustaoglu.biz/en/running-gui-applications-in-docker-on-windows-linux-mac-hosts/). Can also found some more information about [Using GUI's with Docker](http://wiki.ros.org/docker/Tutorials/GUI).

```bash
xhost +local:root
```

We can now run the image as a container named <strong>koubot_ros2_container</strong> using docker-compose :

```bash
docker-compose up
```

We are now <strong>inside the container</strong> and ready for executing our codes.

<u><strong><em>Note:</em></strong></u> For the next tasks we will consider that we are working from inside our container, in the <strong>ros2_ws</strong> workspace.

# System at a glance

In progress ...

# Simulation

## Autonomous Navigation

### 1) Teleop

Spawn the robot in Gazebo (shell # 1):
```
ros2 launch koubot_gazebo spawn_robot.launch.xml
```

Launch teleop node (shell # 2):

```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```


### 2) Mapping
### 3) Localization
### 4) Autonomous Navigation

# Real Robot

In progress ..

# Docker

In progress ..