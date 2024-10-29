#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from luci_messages.msg import LuciJoystick, LuciImu, LuciDriveMode
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points

FORWARD_MAX = 100
BACKWARD_MAX = -100
LEFT_MAX = -100
RIGHT_MAX = 100

class CenterDrive(Node):
    def __init__(self):
        super().__init__('center_drive_node')

        self.joystick_publisher = self.create_publisher(LuciJoystick, 'luci/remote_joystick', 10)

        self.mode_publisher = self.create_publisher(LuciDriveMode, 'luci/drive_mode', 1)

        self.collision_subscriber = self.create_subscription(PointCloud2, "/collision", self.collision_callback, 10)

        self.kp, self.kd, self.ki = 0.1, 0 ,0




    def collision_callback(self, msg):
        left_points = []
        right_points = []

        joystick_msg = LuciJoystick()

        points = read_points(msg, field_names=("x", "y", "z"), skip_nans=True)

        for point in points:
            x, y, z = point[:3]
            print(f"Point: x={x}, y={y}, z={z}")

            if x < 0:  # Left of the wheelchair
                left_points.append(abs(x))
            elif x > 0:  # Right of the wheelchair
                right_points.append(abs(x))

            if y < 1:
                joystick_msg.forward_back = 0
                joystick_msg.left_right = 0

                self.get_logger().error("Stopping Wheelchair, too close to forward wall!")

                self.joystick_publisher.publish(joystick_msg)
                return




        if not left_points or not right_points:
            self.get_logger().warn("Could not detect both walls!")
            return

        left_dist = sum(left_points) / len(left_points)
        right_dist = sum(right_points) / len(right_points)

        error = right_dist - left_dist
        control_signal = self.kp * error

        self.get_logger().info(f"Error: {error}")

        joystick_msg.forward_back = FORWARD_MAX/2
        joystick_msg.left_right = control_signal

        self.joystick_publisher.publish(joystick_msg)

def main(args=None):
    rclpy.init(args=args)
    center_drive = CenterDrive()
    mode_msg = LuciDriveMode()
    mode_msg.mode = LuciDriveMode.AUTO
    center_drive.mode_publisher.publish(mode_msg)

    rclpy.spin(center_drive)

    mode_msg.mode = LuciDriveMode.USER
    center_drive.mode_publisher.publish(mode_msg)

    center_drive.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    