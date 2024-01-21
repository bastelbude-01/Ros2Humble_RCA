from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    task_server_ = Node(
        package="rca_remote",
        executable="task_server_node"
    )

    own_interface_node = Node(
        package="rca_remote",
        executable="own_interface.py"
    )


    return LaunchDescription([

        task_server_,
        own_interface_node
    ])