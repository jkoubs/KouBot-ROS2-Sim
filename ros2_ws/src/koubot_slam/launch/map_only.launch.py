from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    map_file = LaunchConfiguration("map_file")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="Use simulation time"
    )

    map_file_arg = DeclareLaunchArgument(
        "map_file",
        default_value=PathJoinSubstitution([FindPackageShare("koubot_slam"), "maps", "map2.yaml"]),
        description="Path to map YAML file",
    )

    rviz_config_file = os.path.join(
        FindPackageShare("koubot_slam").find("koubot_slam"),
        "rviz",
        "amcl.rviz"
    )

    # Map server node
    map_server_node = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        parameters=[{"use_sim_time": use_sim_time, "yaml_filename": map_file}],
        output="screen"
    )

    # Lifecycle manager to auto-activate map_server
    lifecycle_manager_node = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"autostart": True},
            {"node_names": ["map_server"]}
        ]
    )

    # Static transform from map -> base_link
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_map_to_base",
        arguments=["0", "0", "0", "0", "0", "0", "map", "base_link"],
        output="screen"
    )

    static_map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_map_to_odom",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        output="screen"
    )

    # RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[{"use_sim_time": use_sim_time}]
    )

    return LaunchDescription([
        use_sim_time_arg,
        map_file_arg,
        LogInfo(msg=["Launching map_server with: ", map_file]),
        map_server_node,
        lifecycle_manager_node,
        #static_tf_node,
        static_map_to_odom,
        rviz_node
    ])
