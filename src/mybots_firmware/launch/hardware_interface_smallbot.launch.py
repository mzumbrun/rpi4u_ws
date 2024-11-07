import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    
      
    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')
    
    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="false")
    use_ros2_control_arg = DeclareLaunchArgument("use_ros2_control", default_value='true')
    
    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('mybots_description'))
    xacro_file = os.path.join(pkg_path,'urdf','smallbot.urdf.xacro')
    # robot_description_config = xacro.process_file(xacro_file).toxml()
    robot_description_config = Command(['xacro ', xacro_file, ' use_ros2_control:=', use_ros2_control, ' is_sim:=', use_sim_time])
    
    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config, 'use_sim_time': use_sim_time}
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )
    
    model_arg = DeclareLaunchArgument(name="model", default_value=os.path.join(
                                      get_package_share_directory("mybots_description"), "urdf", "smallbot.urdf.xacro"),
                                      description="Absolute path to robot urdf file"
    )
    
    #robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]),
    #                                   value_type=str)
    
    #delay_controller_manager = TimerAction(period=3.0, actions=[controller_manager])
    
    
    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])

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
            use_sim_time_arg,
            use_ros2_control_arg,
            robot_state_publisher_node,
           # delay_controller_manager,
            model_arg,
            controller_manager,
 
        ]
    )