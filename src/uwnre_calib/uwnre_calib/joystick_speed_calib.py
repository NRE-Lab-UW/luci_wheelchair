import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from luci_messages.msg import LuciJoystick

import math


class JoystickSpeedCalib(Node):
    def __init__(self):
        super().__init__("joystick_speed_calib", namespace="uwnre")

        # Parameters
        self.declare_parameter("odom_topic", "/uwnre/odom")
        self.declare_parameter("joystick_topic", "/luci/remote_joystick")
        self.declare_parameter("log_rate", 10.0)   # Hz

        self.odom_topic = self.get_parameter("odom_topic").value
        self.joystick_topic = self.get_parameter("joystick_topic").value
        self.log_rate = float(self.get_parameter("log_rate").value)

        # State
        self.latest_lin_vel = 0.0
        self.latest_ang_vel = 0.0
        self.latest_fb = 0       # forward_back ([-100, 100])
        self.latest_lr = 0       # left_right   ([-100, 100])
        self.have_odom = False
        self.have_joy = False

        # Subscriptions
        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10,
        )

        self.joy_sub = self.create_subscription(
            LuciJoystick,
            self.joystick_topic,
            self.joy_callback,
            10,
        )

        # Timer for periodic logging
        dt = 1.0 / self.log_rate if self.log_rate > 0.0 else 0.1
        self.timer = self.create_timer(dt, self.timer_callback)

        # Print CSV header once
        self.get_logger().info(
            f"JoystickSpeedCalib started.\n"
            f"  odom_topic: {self.odom_topic}\n"
            f"  joystick_topic: {self.joystick_topic}\n"
            f"  log_rate: {self.log_rate} Hz\n"
            "CSV output format:\n"
            "time_sec,forward_back,left_right,lin_vel_x,ang_vel_z"
        )

    def odom_callback(self, msg: Odometry):
        self.latest_lin_vel = msg.twist.twist.linear.x
        self.latest_ang_vel = msg.twist.twist.angular.z
        self.have_odom = True

    def joy_callback(self, msg: LuciJoystick):
        self.latest_fb = msg.forward_back
        self.latest_lr = msg.left_right
        self.have_joy = True

    def timer_callback(self):
        # Only log when we have both joystick + odom
        if not (self.have_odom and self.have_joy):
            return

        # Current time in seconds (float) for easy plotting
        now = self.get_clock().now()
        t = now.nanoseconds / 1e9

        # CSV-style single line
        line = f"{t:.3f},{self.latest_fb},{self.latest_lr},{self.latest_lin_vel:.4f},{self.latest_ang_vel:.4f}"
        # Use print so you can pipe to a file easily
        print(line)


def main(args=None):
    rclpy.init(args=args)
    node = JoystickSpeedCalib()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down joystick_speed_calib (KeyboardInterrupt)")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
