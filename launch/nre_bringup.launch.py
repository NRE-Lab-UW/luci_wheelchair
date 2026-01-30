# This file will serve as a starting point to bring up the rest of the autonomy stack for LUCI

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="uwnre_base",
            executable="twist2joy_node",
            name="twist2joy_node",
            output="screen",
            parameters=[],
        ),
    ])
