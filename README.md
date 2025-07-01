# This repository holds all the development work related to the KouBot Project done by me.

# Table of Contents
 - [About](#about)
    - [Project Goal](#project-goal)
    - [Key Features](#key-features)
    - [Power System](#power-system)
 - [Install](#install)
 - [Simulation](#simulation)
    - [Teleoperation](#teleoperation)
    - [Mapping](#mapping)
    - [Localization](#localization)
    - [Autonomous Navigation](#autonomous-navigation)
 - [Real Robot](#real-robot)


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

## Project Goal

Design and build an affordable, ROS 2-based indoor robot with full autonomous navigation capabilities, leveraging Docker for reproducibility and portability.


## Key Features


* **Computer**: Raspberry Pi Model B.

* **Software**: ROS 2 Galactic, Docker container, Gazebo, RViz.

* **Sensors**: LiDAR (RPLIDAR-A1M8 by Slamtec), Depth camera (OAK-D Lite), IMU (Adafruit - BNO055).

* **Mecanum Wheel Configuration**: Each of the 4 wheels is driven by its own motor, enabling omnidirectional movement for versatile navigation.

* **Sensor Fusion**: Integrates data from IMU and wheel encoders to improve odometry and overall navigation accuracy.



## Power System



![power_system](doc/power_diagram_full.png)

<br>

The power system of <em>Koubot</em> is divided into two sections to ensure efficient and reliable operation:

* **Motor Power Supply**: This section is dedicated to powering the 4 motors that drive the mecanum wheels, providing the necessary torque and control for movement.

* **System Power Supply**: This section powers the computer and sensors, ensuring stable and continuous operation of the robot's processing and sensing capabilities.

# Install


We will build 2 images:
- <strong>galactic_tb_env</strong>: Allows to run turtlebot3 simulations in ROS 2 from a linux computer.
- <strong>koubot_ros2</strong>: Built on top of <strong>galactic_tb_env</strong> image to set up the ros2_ws for developing the KouBot project.

Open a new terminal and git clone the following repositories:
```bash
git clone https://github.com/jkoubs/ros2_galactic.git
git clone https://github.com/jkoubs/KouBot-ROS2-Sim.git
```
Then build the images:
```bash
cd ros2_galactic/docker
docker build -f Dockerfile -t galactic_env .
cd ../..
cd KouBot-ROS2-Sim/docker
docker build -f Dockerfile -t koubot_ros2 ../
```
<strong><em>Note</em></strong>: <strong>../</strong> represents the PATH context which sets the target context one level above to the <strong>koubot_ros2</strong> directory in order to successfully execute the COPY command from the Dockerfile which will copy the <strong>ros2_ws</strong> inside the container.


Next we will create the container:

<strong><em>Requirement</em></strong>: To run GUI applications in Docker on Linux hosts, you have to run <strong>"xhost +local:root"</strong>. To disallow, <strong>"xhost -local:root"</strong>. For Windows and Mac hosts please check : [Running GUI applications in Docker on Windows, Linux and Mac hosts](https://cuneyt.aliustaoglu.biz/en/running-gui-applications-in-docker-on-windows-linux-mac-hosts/). Can also found some more information about [Using GUI's with Docker](http://wiki.ros.org/docker/Tutorials/GUI).

```bash
xhost +local:root
```

**IMPORTANT NOTE:** Before running the container be sure to **edit the docker-compose.yml file and rename the path according to your local environment to properly mount your host directory into the container. Thus, you need to edit `$HOME/Projects/KouBot-ROS2-Sim/ros2_ws/src` to your own local path.**

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


## Teleoperation

<div align="center">
  <img src="doc/teleop.gif" alt="Demo"/>
</div>

Spawn the robot in Gazebo (Terminal 1):

```bash
cd /ros2_ws/
ros2 launch koubot_gazebo spawn_robot.launch.xml
```

Launch teleop node (Terminal 2):

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```


Launch RViz (Terminal 3):


```bash
rviz2
```
**Note:** In RViz add the **Image** Display with the `/camera/image_raw` topic and choose **Best Effort** for the **Reliability Policy**. Also add the **PointCloud2** Display with the `/point_cloud_sensor/points` topic.


## Mapping

<div align="center">
  <img src="doc/mapping.gif" alt="Demo"/>
</div>

Launch the simulation (Terminal 1):

```bash
ros2 launch koubot_gazebo ttbot3_spawn_robot.launch.xml
```

Launch the slam node (Terminal 2):

```bash
ros2 launch koubot_slam slam.launch.py
```
**IMPORTANT:** In RViz, change the **Fixed Frame** to `/map` and add a `Map` Display with the appropriate topic named `/map`.


Now, using teleop, move the robot around to perform the map of the environment (Terminal 3):

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Once you are done creating the map, save it (Terminal 4):

```bash
cd /ros2_ws/src/koubot_slam/maps/
ros2 run nav2_map_server map_saver_cli -f map
```

This will create two files (`map.yaml` & `map.pgm`) to represent the map. Those files are located inside the koubot_slam package:

```bash
koubot_slam/
  └── maps/
    ├── map.pgm
    └── map.yaml
```

## Localization

<div align="center">
  <img src="doc/localization.gif" alt="Demo"/>
</div>

Launch the simulation (Terminal 1):

```bash
ros2 launch koubot_gazebo ttbot3_spawn_robot.launch.xml
```

Launch the amcl node (Terminal 2):

```bash
ros2 launch koubot_slam amcl.launch.py
```
**IMPORTANT:** Once our localization node is launched, we need to set a `2D Pose Estimate` using Rviz. 



Now, using teleop, move the robot around and check how the localization pipeline works (Terminal 3):

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

We can see that the particle filter (small purple dots)  shrinks over time which confirms that our localization is working properly.


## Autonomous Navigation

# Real Robot

You can find the results on the real robot [here](https://github.com/jkoubs/KouBot-ROS2).



