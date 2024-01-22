from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    
    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("rca_controller"),
            "launch",
            "controller.launch.py"
        ),
        launch_arguments={"is_sim" : "False"}.items()
    )

    moveit = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("rca_moveit"),
            "launch",
            "moveit.launch.py"
        ),
        launch_arguments={"is_sim" : "False"}.items()
    )

    remote = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("rca_remote"),
            "launch",
            "remote_interface.launch.py"
        ),
    )

    return LaunchDescription([
        controller,
        moveit,
        remote,


    ])