from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    amcl_params_file = LaunchConfiguration("amcl_params_file")
    map_file = LaunchConfiguration("map_file")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="Use simulation/Gazebo clock"
    )

    map_file_arg = DeclareLaunchArgument(
        "map_file",
        default_value=PathJoinSubstitution([FindPackageShare("koubot_slam"), "maps", "map.yaml"]),
        description="Full path to the yaml map file",
    )

    amcl_params_file_arg = DeclareLaunchArgument(
        "amcl_params_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("koubot_slam"), "config", "amcl.yaml"]
        ),
        description="Full path to the ROS2 parameters file to use for the amcl node",
    )

    # Loads map & pub map data to /map topic
    map_server_node = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        parameters=[{"use_sim_time": use_sim_time, "yaml_filename": map_file}],
        output='screen',
    )

    # Localization node
    amcl_node = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        parameters=[amcl_params_file, {"use_sim_time": use_sim_time}],
        output='screen',
    )

    # Manages the lifecycle of the navigation-related nodes.
    # Ensures that they are started and stopped in the correct order
    # Maintains state and config of nav stack
    # Providing controlled & reliable startup and shutdown process

    nav_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="nav_manager",
        output='screen',
        parameters=[
            {"use_sim_time": use_sim_time},
            {"autostart": True},
            {"node_names": ["map_server", "amcl"]},
        ],
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            amcl_params_file_arg,
            map_file_arg,
            LogInfo(msg=["Launching AMCL with map: ", map_file]),
            LogInfo(msg=["Using AMCL parameters file: ", amcl_params_file]),
            map_server_node,
            amcl_node,
            nav_manager,
        ]
    )