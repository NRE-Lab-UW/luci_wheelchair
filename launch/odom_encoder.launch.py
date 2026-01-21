from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # 1) Encoder-based odometry node
        Node(
            package="uwnre_base",
            executable="odom_node",
            name="odom_node",
            output="screen",
            # namespace is already set to "uwnre" *inside* the node class,
            # so we don't set it here again
            parameters=[{
                # tune these for your wheelchair
                "wheel_radius": 0.15,      # meters
                "wheel_base": 0.60,        # meters
                "ticks_per_rev": 4096,
                "publish_rate": 25.0,
            }],
        ),

        # 2) Twist -> LuciJoystick bridge
        Node(
            package="uwnre_base",
            executable="wheel_angle_scaler",
            name="wheel_angle_scaler",
            output="screen",
            parameters=[],
        ),
        Node(
            package="uwnre_base",
            executable="twist2joy_node",
            name="twist2joy_node",
            output="screen",
            parameters=[],
        ),
        Node(
            package="uwnre_calib",
            executable="test_cmd_vel_pub",
            name="test_cmd_vel_pub",
            output="screen",
            parameters=[],
        ),
    ])
