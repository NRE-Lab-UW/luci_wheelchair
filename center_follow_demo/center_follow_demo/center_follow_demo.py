#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from luci_messages.msg import LuciJoystick
from luci_messages.msg import LuciImu
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points

class CenterDrive(Node):
    def __init__(self):
        super().__init__('center_drive_node')

        # TODO: What name do I publish to?
        self.joystick_publisher = self.create_publisher(LuciJoystick, '/', 10)

        self.collision_subscriber = self.create_subscription(PointCloud2, "/collision", self.collision_callback, 10)


    def collision_callback(self, msg):
        points = read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        for point in points:
            x, y, z = point[:3]
            print(f"Point: x={x}, y={y}, z={z}")

if __name__ == "__main__":
    rclpy.init(args=args)
    center_drive = CenterDrive()
    rclpy.spin(center_drive)
    center_drive.destroy_node()
    rclpy.shutdown()