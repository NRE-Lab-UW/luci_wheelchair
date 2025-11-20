#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math

from sensor_msgs.msg import JointState
from luci_messages.msg import LuciEncoders  


class WheelAngleScaler(Node):
    def __init__(self):
        super().__init__('wheel_angle_scaler', namespace='uwnre')

        # Scale factor derived from your data
        self.scale = 2.37  # sensor_deg = scale * wheel_deg

        # For unwrapping
        self.prev_left_raw = None
        self.prev_right_raw = None
        self.left_unwrapped = 0.0
        self.right_unwrapped = 0.0

        # Subscriber
        self.subscription = self.create_subscription(
            LuciEncoders,
            '/luci/encoders',
            self.encoder_callback,
            10
        )

        # Publisher (under uwnre namespace)
        self.publisher = self.create_publisher(
            JointState,
            'scaled_wheel_angles',
            10
        )

        self.get_logger().info("WheelAngleScaler node started.")

    # -----------------------------
    # Unwrap helper
    # -----------------------------
    def unwrap(self, current, previous):
        delta = current - previous

        # Handle wrap-around
        if delta > 180.0:
            delta -= 360.0
        elif delta < -180.0:
            delta += 360.0

        return delta

    # -----------------------------
    # Main callback
    # -----------------------------
    def encoder_callback(self, msg: LuciEncoders):

        left_raw = float(msg.left_angle)
        right_raw = float(msg.right_angle)

        # Initialize unwrap state
        if self.prev_left_raw is None:
            self.prev_left_raw = left_raw
            self.prev_right_raw = right_raw
            return

        # Unwrap raw sensor angles
        d_left = self.unwrap(left_raw, self.prev_left_raw)
        d_right = self.unwrap(right_raw, self.prev_right_raw)

        self.left_unwrapped += d_left
        self.right_unwrapped += d_right

        self.prev_left_raw = left_raw
        self.prev_right_raw = right_raw

        # Scale sensor → wheel degrees
        left_wheel_deg = self.left_unwrapped / self.scale
        right_wheel_deg = self.right_unwrapped / self.scale

        # Publish JointState
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ['left_wheel_degs', 'right_wheel_degs']
        js.position = [
            left_wheel_deg,
            right_wheel_deg
        ]

        self.publisher.publish(js)


def main(args=None):
    rclpy.init(args=args)
    node = WheelAngleScaler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
