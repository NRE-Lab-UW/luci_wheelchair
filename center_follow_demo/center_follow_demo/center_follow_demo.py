#!/usr/bin/env python3

from rclpy.node import Node
from luci_messages.msg import LuciJoystick
from luci_messages.msg import LuciImu
from #IMPORT THE OCCUPANCY GRID

class CenterDrive(Node):
    def __init__(self):
        super().__init__('center_drive_node')

        # TODO: What name do I publish to?
        self.joystick_publisher = self.create_publisher(LuciJoystick, '/', 10)

        self.lidar_subscriber = self.create_subscription()