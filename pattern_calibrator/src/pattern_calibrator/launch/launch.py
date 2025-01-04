from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import Shutdown


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="pattern_calibrator",
                namespace="/am10",
                executable="camera_gui",
                on_exit=Shutdown(),
            ),
            Node(
                package="pattern_calibrator",
                namespace="/am10",
                executable="calibrator",
                on_exit=Shutdown(),
            ),
            Node(
                package="pattern_calibrator",
                namespace="/am10",
                executable="logger",
                parameters=[{"ID": 10, "team_color": "blue"}],
                on_exit=Shutdown(),
            ),
            Node(
                package="pattern_calibrator",
                namespace="/am10",
                executable="vision_receiver",
                on_exit=Shutdown(),
            ),
        ]
    )
