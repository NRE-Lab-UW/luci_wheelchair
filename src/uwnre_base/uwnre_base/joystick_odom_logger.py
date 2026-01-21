#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from luci_messages.msg import LuciJoystick  # adjust if needed
import csv
import os
from datetime import datetime


class JoystickOdomLogger(Node):
    def __init__(self):
        super().__init__("joystick_odom_logger", namespace="uwnre")

        # Parameters
        self.declare_parameter("csv_dir", "/tmp")
        self.declare_parameter("odom_topic", "/luci/odom")
        self.declare_parameter("joy_topic", "/luci/joystick_position")

        csv_dir = self.get_parameter("csv_dir").value
        odom_topic = self.get_parameter("odom_topic").value
        joy_topic = self.get_parameter("joy_topic").value

        os.makedirs(csv_dir, exist_ok=True)
        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_path = os.path.join(csv_dir, f"joyodom_speed2.csv")

        # Open CSV file + header
        self.csv_file = open(self.csv_path, mode="w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            "stamp_sec", "stamp_nanosec",

            # full linear twist
            "linear_x", "linear_y", "linear_z",

            # full angular twist
            "angular_x", "angular_y", "angular_z",

            # joystick state
            "forward_back", "left_right",
            "joystick_zone", "input_source"
        ])
        self.csv_file.flush()

        # Latest joystick values
        self.latest_forward_back = 0
        self.latest_left_right = 0
        self.latest_joystick_zone = 0
        self.latest_input_source = 0
        self.have_joystick = False

        # Subscribers
        self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            50
        )

        self.create_subscription(
            LuciJoystick,
            joy_topic,
            self.joystick_callback,
            50
        )

        self.get_logger().info(f"JoystickOdomLogger started, logging to: {self.csv_path}")

    def joystick_callback(self, msg: LuciJoystick):
        """Store latest joystick values."""
        self.latest_forward_back = int(msg.forward_back)
        self.latest_left_right = int(msg.left_right)
        self.latest_joystick_zone = int(msg.joystick_zone)
        self.latest_input_source = int(msg.input_source)
        self.have_joystick = True

    def odom_callback(self, msg: Odometry):
        """Log full twist + latest joystick state."""
        if not self.have_joystick:
            return  # skip until we receive joystick data

        stamp = msg.header.stamp

        # Extract full twist
        lin = msg.twist.twist.linear
        ang = msg.twist.twist.angular

        row = [
            stamp.sec,
            stamp.nanosec,

            lin.x, lin.y, lin.z,
            ang.x, ang.y, ang.z,

            self.latest_forward_back,
            self.latest_left_right,
            self.latest_joystick_zone,
            self.latest_input_source,
        ]

        self.csv_writer.writerow(row)
        self.csv_file.flush()

    def destroy_node(self):
        try:
            self.csv_file.close()
        except:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = JoystickOdomLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
