# This repository holds all the development work related to the KouBot Project done by me.

# Table of Contents
 - [About](#about)
    - [Project Goals](#project-goals)
    - [Key Features](#key-features)
    - [Power System](#power-system)
 - [Install](#install)
 - [Simulation](#simulation)
    - [Autonomous Navigation](#autonomous-navigation)
      - [Teleoperation](#teleoperation)
      - [Mapping](#mapping)
      - [Localization](#localization)
      - [Path Planning](#path-planning)
 - [Real Robot](#real-robot)
 - [What has been done](#what-has-been-done)
 - [Challenges](#challenges)

# About 

<div align="center">
  <img src="doc/base_v2.png" alt="base" width="400"/>
</div>

<br>

Welcome to the <em>Koubot</em> project repository! <em>Koubot</em>  is an ambitious project aimed at developing an autonomous mobile robot equipped with **four mecanum wheels**, each powered by its own motor. This unique wheel configuration allows for omnidirectional movement, making <em>Koubot</em> highly maneuverable and capable of navigating complex environments with ease.

<br>

<div align="center">
  <img src="doc/koubot_rviz_v2.png" alt="base" width="400"/>
</div>

## Project Goals

One of the primary goals of the <em>Koubot</em> project is to achieve advanced **sensor fusion**, integrating data from multiple sensors to enhance the robot's odometry. By fusing data from an IMU (Inertial Measurement Unit) and GPS, <em>Koubot</em> will be able to:


* Accurately detect and avoid obstacles
* Map and navigate unfamiliar environments
* Perform tasks autonomously with high precision

## Key Features


* **Computer**: Raspberry Pi Model B.

* **Software**: ROS 2 Galactic, Docker container, Gazebo, RViz.

* **Sensors**: LiDAR (RPLIDAR-A1M8 by Slamtec), Depth camera (OAK-D Lite), IMU (Adafruit - BNO055), GPS (Adafruit - Ultimate GPS Breakout v3).

* **Mecanum Wheel Configuration**: Each of the 4 wheels is driven by its own motor, enabling omnidirectional movement for versatile navigation.

* **Sensor Fusion**: Integrates data from IMU and GPS to improve odometry and overall navigation accuracy.



## Power System



![power_system](doc/power_diagram_full.png)

<br>

The power system of <em>Koubot</em> is divided into two sections to ensure efficient and reliable operation:

* **Motor Power Supply**: This section is dedicated to powering the 4 motors that drive the mecanum wheels, providing the necessary torque and control for movement.

* **System Power Supply**: This section powers the computer and sensors, ensuring stable and continuous operation of the robot's processing and sensing capabilities.

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
cd KouBot-ROS2/docker
docker build -f Dockerfile -t koubot_ros2 ../
```
<strong><em>Note</em></strong>: <strong>../</strong> represents the PATH context which sets the target context one level above to the <strong>koubot_ros2</strong> directory in order to successfully execute the COPY command from the Dockerfile which will copy the <strong>ros2_ws</strong> inside the container.


Next we will create the container:

<strong><em>Requirement</em></strong>: To run GUI applications in Docker on Linux hosts, you have to run <strong>"xhost +local:root"</strong>. To disallow, <strong>"xhost -local:root"</strong>. For Windows and Mac hosts please check : [Running GUI applications in Docker on Windows, Linux and Mac hosts](https://cuneyt.aliustaoglu.biz/en/running-gui-applications-in-docker-on-windows-linux-mac-hosts/). Can also found some more information about [Using GUI's with Docker](http://wiki.ros.org/docker/Tutorials/GUI).

```bash
xhost +local:root
```

**IMPORTANT NOTE:** Before running the container be sure to **edit the docker-compose.yml file and rename the path according to your local environment to properly mount your host directory into the container**.

<div align="center">
  <img src="doc/docker_instructions.png" alt="base"/>
</div>

We can now run the image as a container named <strong>koubot_ros2_container</strong> using docker-compose :

```bash
docker-compose up
```

We are now <strong>inside the container</strong> and ready for executing our codes.

<u><strong><em>Note:</em></strong></u> For the next tasks we will consider that we are working from inside our container, in the <strong>ros2_ws</strong> workspace.


# Simulation

## Autonomous Navigation

### Teleoperation

Spawn the robot in Gazebo (shell # 1):
```bash
cd /ros2_ws/
ros2 launch koubot_gazebo spawn_robot.launch.xml
```

Launch teleop node (shell # 2):

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Launch RViz (shell # 3):

```bash
rviz2
```
**Note:** In RViz add the **Image** Display with the `/camera/image_raw` topic and choose **Best Effort** for the **Reliability Policy**. Also add the **PointCloud2** Display with the `/point_cloud_sensor/points` topic.
### Mapping
### Localization
### Path Planning

# Real Robot

In progress ..

# What has been done

## 1. Robot modeling (Using FreeCAD) + URDF

<div align="center">
  <img src="doc/koubot_rviz_v2.png" alt="base"/>
</div>

## 2. Added teleoperation (able to teleop in simulation)


<div align="center">
  <img src="doc/teleop.gif" alt="Demo"/>
</div>

## 3. Communicate with robot wheels using Arduino and Platformio

<div align="center">
  <img src="doc/control_wheels.gif" alt="Demo"/>
</div>

## 4. Installed sensors on real robot (LiDAR + Camera)

# Challenges

## 1. Docker for real robot integration? Maybe better to directly install ROS 2 Galactic on Raspberry Pi (Preference toward installing Docker instead and use Docker container)

## 2. Communicate with motors and ROS 2

## 3. Install IMU