#!/usr/bin/python3
# -*- coding: utf-8 -*-
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import os
# Read the robot_description topic and spawn that robot mdl

def generate_launch_description():
    position = [-2.0, 0.0, 0.2]
    orientation = [0.0, 0.0, 0.0]
    robot_name = "koubot"

    # Gazebo models
    models_dir = get_package_share_directory('koubot_gazebo') + '/models'


    # Spawn ROBOT 
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
                   ],
        parameters=[{'use_sim_time': True}]
    )
    

    # Static transform from base_link to odom
    static_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_odom_base_link',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
        parameters=[{'use_sim_time': True}]
    )

    # create and return launch description object
    return LaunchDescription(
        [
            spawn_robot,
            # static_transform_publisher,
        ]
    )
