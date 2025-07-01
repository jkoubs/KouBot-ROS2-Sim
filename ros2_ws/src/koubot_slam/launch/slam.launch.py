from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    slam_params_file = LaunchConfiguration('slam_params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    slam_params_file_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value=PathJoinSubstitution(
            [FindPackageShare("koubot_slam"), 'config', 'slam.yaml']
        ),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node',
    )

    rviz_config_file = os.path.join(get_package_share_directory(
        'koubot_slam'), 'rviz', 'slam.rviz')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation/Gazebo clock'
    )

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam',
        parameters=[slam_params_file, {'use_sim_time': use_sim_time}],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        name='rviz_node',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', rviz_config_file]
    )

    return LaunchDescription([
        use_sim_time_arg,
        slam_params_file_arg,
        rviz,
        slam_node
    ])