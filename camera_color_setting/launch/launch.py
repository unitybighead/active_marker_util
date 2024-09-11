from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="camera_color_setting",
                namespace="/am16",
                executable="camera_gui",
            )
        ]
    )
