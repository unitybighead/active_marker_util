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
                executable="predictor",
                parameters=[
                    {
                        "update_hz": 1,
                        "ID": 10,
                        "team_color": "blue",
                        "camera_height": 2500.0,
                    }
                ],
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
