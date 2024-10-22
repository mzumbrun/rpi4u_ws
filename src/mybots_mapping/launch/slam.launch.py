import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    
    mybots_mapping_pkg = get_package_share_directory('mybots_mapping')

    use_sim_time = LaunchConfiguration("use_sim_time")
    lifecycle_nodes = ("map_saver_server")
    free_thresh_default = 0.25
    occupied_thresh_default = 0.65

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true"
    )
  
    nav2_map_saver = Node(
        package="nav2_map_server",
        executable="map_saver_server",
        name="map_saver_server",
        output="screen",
        parameters=[
            {"save_map_timeout": 5.0},
            {"use_sim_time": use_sim_time},
            {"free_thresh_default": free_thresh_default},
            {"occupied_thresh_default": occupied_thresh_default},
        ],
    )

   
    jazzy_slam_toolbox_launch = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("slam_toolbox"),
            "launch",
            "online_async_launch.py"
        ),
        launch_arguments={
            "slam_params_file": os.path.join(get_package_share_directory('mybots_mapping'), "config", "slam_toolbox.yaml"),
            "use_sim_time": use_sim_time,
        }.items(),
    )

    nav2_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_slam",
        output="screen",
        parameters=[
            {"node_names": lifecycle_nodes},
            {"use_sim_time": use_sim_time},
            {"autostart": True}
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        nav2_map_saver,
        jazzy_slam_toolbox_launch,
        nav2_lifecycle_manager,
    ])