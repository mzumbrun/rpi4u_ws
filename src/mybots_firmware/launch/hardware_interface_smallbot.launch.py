import os
from os import pathsep
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    mybots_description = get_package_share_directory("mybots_description")
   # mybots_description_prefix = get_package_prefix("mybots_description")

    model_arg = DeclareLaunchArgument(name="model", default_value=os.path.join(
                                      mybots_description, "urdf", "smallbot.urdf.xacro"),
                                      description="Absolute path to robot urdf file"
    )

    model_path = str(Path(mybots_description).parent.resolve())
    model_path += pathsep + os.path.join(get_package_share_directory("mybots_description"), 'models')
  
    
    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model"),
                                                " is_sim:=False"]),
                                       value_type=str)
    
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description,
             "use_sim_time": False},
            os.path.join(
                get_package_share_directory("mybots_controller"),
                "config",
                "mybots_controllers.yaml",
            ),
        ],
    )


    return LaunchDescription(
        [
            model_arg,
            robot_state_publisher_node,
            controller_manager,
 
        ]
    )