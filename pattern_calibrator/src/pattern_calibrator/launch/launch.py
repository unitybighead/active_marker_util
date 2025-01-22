from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import Shutdown


def generate_launch_description():
    ns = "/am10"
    return LaunchDescription(
        [
            Node(
                package="pattern_calibrator",
                namespace=ns,
                executable="camera_gui",
                on_exit=Shutdown(),
            ),
            Node(
                package="pattern_calibrator",
                namespace=ns,
                executable="calibrator",
                on_exit=Shutdown(),
            ),
            Node(
                package="pattern_calibrator",
                namespace=ns,
                executable="logger",
                parameters=[{"ID": 10, "team_color": "blue"}],
                on_exit=Shutdown(),
            ),
            Node(
                package="pattern_calibrator",
                namespace=ns,
                executable="vision_receiver",
                on_exit=Shutdown(),
            ),
        ]
    )
