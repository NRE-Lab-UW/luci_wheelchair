from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # 1) Encoder-based odometry node
        Node(
            package="uwnre_base",
            executable="encoder_odom_node",
            name="encoder_odom_node",
            output="screen",
            parameters=[{
                # Tune these to your chair:
                "wheel_radius": 0.15,      # meters
                "wheel_base": 0.60,        # meters
                "ticks_per_rev": 2048,
                "publish_rate": 25.0,
            }],
        ),

        # 2) Direct joystick publisher + odom logger
        Node(
            package="uwnre_calib",
            executable="direct_joystick_odom_logger",
            name="direct_joystick_odom_logger",
            output="screen",
            parameters=[{
                "joystick_topic": "/luci/remote_joystick",
                "odom_topic": "/uwnre/odom",
                "forward_back": 40,              # joystick % forward
                "left_right": 0,
                "duration": 5.0,                 # seconds
                "rate": 10.0,                    # Hz
                "output_csv": "joystick_odom_log.csv",
            }],
        ),
    ])
