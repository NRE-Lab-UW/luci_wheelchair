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
                "ticks_per_rev": 2048,
                "publish_rate": 25.0,
            }],
        ),

        # 2) Twist -> LuciJoystick bridge
        Node(
            package="uwnre_base",
            executable="twist2joy_node",
            name="twist2joy_node",
            output="screen",
            parameters=[{
                "cmd_vel_topic": "/uwnre/cmd_vel",
                "max_forward_speed": 0.8,   # m/s, adjust from calibration results
                "max_reverse_speed": 0.5,   # m/s (magnitude)
                "max_angular_speed": 1.2,   # rad/s
                "publish_rate": 20.0,       # Hz
                "deadman_timeout": 0.5,     # seconds
            }],
        ),

        # 3) Simple cmd_vel generator (fake Nav2)
        Node(
            package="uwnre_calib",
            executable="test_cmd_vel_publisher",
            name="test_cmd_vel_publisher",
            output="screen",
            parameters=[{
                "cmd_vel_topic": "/uwnre/cmd_vel",
                "linear_x": 0.2,    # m/s
                "angular_z": 0.0,   # rad/s
                "duration": 0.0,    # 0 = run forever
                "rate": 10.0,       # Hz
            }],
        ),

        # 4) Joystick + velocity logger for mapping
        Node(
            package="uwnre_calib",
            executable="joystick_speed_calib",
            name="joystick_speed_calib",
            output="screen",
            parameters=[{
                "odom_topic": "/uwnre/odom",
                "joystick_topic": "/luci/remote_joystick",
                "log_rate": 10.0,   # Hz
            }],
        ),
    ])
