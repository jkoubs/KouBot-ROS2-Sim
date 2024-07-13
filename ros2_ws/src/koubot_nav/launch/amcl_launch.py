from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_dir = get_package_share_directory('koubot_nav')

    return LaunchDescription([
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[
                os.path.join(package_dir, 'config', 'amcl_config.yaml'),
                {'use_sim_time': True}
            ]
        ),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[
                {'yaml_filename': os.path.join(package_dir, 'maps', 'my_map.yaml')},
                {'use_sim_time': True}
            ]
        ),
    ])
