from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import Shutdown


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="pattern_calibrator",
                namespace="/am16",
                executable="camera_gui",
                on_exit=Shutdown(),
            ),
            Node(
                package="pattern_calibrator",
                namespace="/am16",
                executable="calibrator",
                on_exit=Shutdown(),
            ),
            Node(
                package="pattern_calibrator",
                namespace="/am16",
                executable="logger",
                on_exit=Shutdown(),
            ),
        ]
    )
