#!/usr/bin/python3
# -*- coding: utf-8 -*-
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import os
# Read the robot_description topic and spawn that robot mdl

def generate_launch_description():
    position = [0.0, 0.0, 0.2]
    orientation = [0.0, 0.0, 0.0]
    robot_name = "koubot"

    # Gazebo models
    models_dir = get_package_share_directory('koubot_gazebo') + '/models'
    cube_model_path = os.path.join(models_dir, 'demo_cube', 'model.sdf')
    banana_model_path = os.path.join(models_dir, 'banana', 'banana.sdf')
    cube_position = [2.0, 0.0, 1.2] # bench orange side
    banana_position = [1.0, 0.0, 0.3] # bench orange side
    banana_orientation = [-1.9, 0.0, 1.57]
 


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
                   ]
    )

    # Spawn ROBOT cube
    spawn_cube = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_entity',
            arguments=['-entity', 'cube', '-file', cube_model_path, '-x', str(cube_position[0]), '-y', str(cube_position[1]), '-z', str(cube_position[2])],
            output='screen'
        )

    # Spawn ROBOT cube
    spawn_banana = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_entity',
            arguments=['-entity', 'banana', '-file', banana_model_path, '-x', str(banana_position[0]), '-y', str(banana_position[1]), '-z', str(banana_position[2]),
            '-R', str(banana_orientation[0]), '-P', str(banana_orientation[1]), '-Y', str(banana_orientation[2])],
            output='screen'
        )
    
    # create and return launch description object
    return LaunchDescription(
        [
            spawn_robot,
            spawn_cube,
            spawn_banana,
        ]
    )
