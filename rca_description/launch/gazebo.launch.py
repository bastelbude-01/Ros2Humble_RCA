from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(get_package_share_directory("rca_description"),"urdf","rca.urdf.xacro"),
        description="Absolute path to the URDF File"
    )

    env_var = SetEnvironmentVariable("GAZEBO_MODEL_PATH", os.path.join(get_package_prefix("rca_description"), "share"))

    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]))

    rsp = Node(        
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )

    start_gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("gazebo_ros"),"launch", "gzserver.launch.py"))
        )

    start_gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("gazebo_ros"),"launch", "gzclient.launch.py"))
    )
    start_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity","rca", "-topic", "robot_description"]
    )

    

    return LaunchDescription([
        model_arg,
        env_var,
        rsp,
        start_gazebo_server,
        start_gazebo_client,
        start_robot
        


    ])