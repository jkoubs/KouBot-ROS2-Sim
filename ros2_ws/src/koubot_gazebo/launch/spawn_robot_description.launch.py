#!/usr/bin/python3
# -*- coding: utf-8 -*-
from launch_ros.actions import Node
from launch import LaunchDescription
# Read the robot_description topic and spawn that robot mdl

def generate_launch_description():
    position = [0.0, 0.0, 0.2]
    orientation = [0.0, 0.0, 0.0]
    robot_name = "koubot"

    # Spawn ROBOT Set Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=['-entity',
                   robot_name,
                   '-x', str(position[0]), '-y', str(position[1]), '-z', str(position[2]),
                   '-R', str(orientation[0]), '-P', str(orientation[1]), '-Y', str(orientation[2]),
                   '-topic', '/robot_description'
                   ]
    )

    # create and return launch description object
    return LaunchDescription(
        [
            spawn_robot,
        ]
    )