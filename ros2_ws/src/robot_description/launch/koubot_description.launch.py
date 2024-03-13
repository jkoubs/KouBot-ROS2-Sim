import os

import launch_ros.descriptions

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node

# Launch file launching: Robot State Pub, Joint State Pub, Joint State Pub GUI, Rviz

def generate_launch_description():

    ####### DATA INPUT ##########
    urdf_file = 'koubot_base.xacro'
    package_description = "robot_description"

    ####### DATA INPUT END ##########
    print("Fetching URDF ==>")
    robot_desc_path = os.path.join(get_package_share_directory(
        package_description), "urdf", "base", urdf_file)

    # 1) Robot State Publisher
    # Publish the contents of the URDF in the 'robot_description' topic using the robot_state_publish_node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_node',
        emulate_tty=True,
        # parameters=[{'use_sim_time': True, 'robot_description': Command(
        #     ['xacro ', robot_desc_path])}],
        parameters=[{'robot_description': launch_ros.descriptions.ParameterValue(Command(['xacro ',os.path.join('src',package_description,'urdf/base/koubot_base.xacro')]), value_type=str)  }],
        output="screen"
    )

    # 2) Joint State Publisher
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher_node',
        output="screen"
    )

    # 3) Joint State Publisher Gui
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui_node',
        output="screen"
    )

    # 4) RVIZ Configuration
    rviz_config_dir = os.path.join(get_package_share_directory(
        package_description), 'rviz', 'camera.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        name='rviz_node',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', rviz_config_dir]
    )

    # create and return launch description object
    return LaunchDescription(
        [
            robot_state_publisher_node,
            rviz_node,
            joint_state_publisher_node,
            joint_state_publisher_gui_node
        ]
    )
