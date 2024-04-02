#!/usr/bin/python3
# -*- coding: utf-8 -*-
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import os
# Read the robot_description topic and spawn that robot mdl

def generate_launch_description():
    # Declare launch arguments for model path and spawn position
    models_dir = get_package_share_directory('koubot_gazebo') + '/models'
    cube_model_path = os.path.join(models_dir, 'demo_cylinder', 'model.sdf')
    cube_position = [1.0, 1.0, 1.2] # bench orange side
    #cube_position = [0.0, 0.7, 1.2] # middle bench
    cube_orientation = [0.0, 0.0, 0.0]


    # Spawn ROBOT cube
    spawn_cube = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_entity',
            arguments=['-entity', 'cylinder', '-file', cube_model_path, '-x', str(cube_position[0]), '-y', str(cube_position[1]), '-z', str(cube_position[2])],
            output='screen'
        )


    # create and return launch description object
    return LaunchDescription(
        [
            spawn_cube,
        ]
    )
