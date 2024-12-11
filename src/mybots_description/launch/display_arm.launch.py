import os
from ament_index_python.packages import get_package_share_directory
import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    mybots_description_dir = get_package_share_directory("mybots_description")

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')
    
    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="true")
    use_ros2_control_arg = DeclareLaunchArgument("use_ros2_control", default_value='true')
    
    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('mybots_description'))
    xacro_file = os.path.join(pkg_path,'urdf','little_arm.urdf.xacro')
    doc = xacro.process_file(xacro_file, mappings={'is_sim' : 'false', 'use_ros2_control' : 'true'})
    robot_description = doc.toprettyxml(indent='  ')
    
    # Create a robot_state_publisher node
    params = {'robot_description': robot_description, 'use_sim_time': use_sim_time}
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(mybots_description_dir, "rviz", "display_arm.rviz")],
    )

    return LaunchDescription([
       # model_arg,
        use_sim_time_arg,
        use_ros2_control_arg,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node
    ])