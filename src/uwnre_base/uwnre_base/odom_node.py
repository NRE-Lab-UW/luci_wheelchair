#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
import tf2_ros
import math


class OdomNode(Node):
    def __init__(self):
        super().__init__("odom_node", namespace="uwnre")

        # ---- Parameters ----
        self.declare_parameter("wheel_radius", 0.16)   # m
        self.declare_parameter("wheel_base", 0.60)     # m

        self.wheel_radius = self.get_parameter("wheel_radius").value
        self.wheel_base = self.get_parameter("wheel_base").value

        # Pose state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Previous wheel angles (in degrees, as published by wheel_angle_scaler)
        self.prev_left_deg = None
        self.prev_right_deg = None

        # Time for velocity calc
        self.last_time = None

        # Subscriber for scaled wheel angles
        self.angle_sub = self.create_subscription(
            JointState,
            "scaled_wheel_angles",
            self.angle_callback,
            10
        )

        # Odometry publisher
        self.odom_pub = self.create_publisher(Odometry, "odom", 10)

        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Reset service
        self.reset_service = self.create_service(
            Trigger,
            "reset_odometry",
            self.handle_reset_odometry
        )

        self.get_logger().info("uwnre odom_node started")

    # Normalize angle to [-pi, pi]
    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def unwrap_delta_deg(self, current_deg, prev_deg):
        delta = current_deg - prev_deg
        if delta > 180.0:
            delta -= 360.0
        elif delta < -180.0:
            delta += 360.0
        return delta

    # Reset service
    def handle_reset_odometry(self, request, response):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        # Don't reset prev_* so we don't get a giant jump; let next callback handle it
        self.last_time = self.get_clock().now()
        self.get_logger().info("Odometry reset via service 'reset_odometry'")
        response.success = True
        response.message = "Odometry reset"
        return response

    # Main encoder callback → compute odom and publish
    def angle_callback(self, msg: JointState):
        # Expect [left_wheel_degs, right_wheel_degs]
        if len(msg.position) < 2:
            self.get_logger().warn("scaled_wheel_angles has fewer than 2 positions")
            return

        left_deg = float(msg.position[0])
        right_deg = float(msg.position[1])

        now = self.get_clock().now()

        # First message: just initialize state
        if self.prev_left_deg is None:
            self.prev_left_deg = left_deg
            self.prev_right_deg = right_deg
            self.last_time = now
            return

        # Time delta
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0.0:
            dt = 1e-6  # avoid division by zero

        # Wheel angle deltas in degrees, unwrapped
        d_left_deg = self.unwrap_delta_deg(left_deg, self.prev_left_deg)
        d_right_deg = self.unwrap_delta_deg(right_deg, self.prev_right_deg)

        self.prev_left_deg = left_deg
        self.prev_right_deg = right_deg
        self.last_time = now

        # Convert to radians
        d_left_rad = math.radians(d_left_deg)
        d_right_rad = math.radians(d_right_deg)

        # Convert to linear distances for each wheel
        d_left = d_left_rad * self.wheel_radius
        d_right = d_right_rad * self.wheel_radius

        # Diff-drive kinematics
        ds = (d_right + d_left) / 2.0
        dtheta = (d_right - d_left) / self.wheel_base

        # Update pose (using current heading + half rotation for better accuracy)
        theta_mid = self.theta + dtheta / 2.0
        self.x += ds * math.cos(theta_mid)
        self.y += ds * math.sin(theta_mid)
        self.theta = self.normalize_angle(self.theta + dtheta)

        # Velocities
        v = ds / dt
        omega = dtheta / dt

        # Publish odom + tf
        self.publish_odom(now, v, omega)

    def publish_odom(self, stamp, v, omega):
        # Odometry msg
        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        # Yaw-only orientation
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)

        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = omega

        self.odom_pub.publish(odom)

        # TF
        t = TransformStamped()
        t.header.stamp = stamp.to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        t.transform.rotation.z = math.sin(self.theta / 2.0)
        t.transform.rotation.w = math.cos(self.theta / 2.0)
    
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
