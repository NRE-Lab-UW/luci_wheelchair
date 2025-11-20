asdf
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Int32
import tf2_ros
import math
import time


class OdomNode(Node):
    def __init__(self):
        super().__init__("odom_node", namespace="uwnre")

        # ---- Declare parameters ----
        # TODO: Find the correct values and update in the launch file
        self.declare_parameter("wheel_radius", 0.15)          # meters
        self.declare_parameter("wheel_base", 0.60)            # meters
        self.declare_parameter("ticks_per_rev", 2048)
        self.declare_parameter("publish_rate", 25.0)          # Hz

        self.wheel_radius = self.get_parameter("wheel_radius").value
        self.wheel_base = self.get_parameter("wheel_base").value
        self.ticks_per_rev = self.get_parameter("ticks_per_rev").value

        # Encoder state
        self.left_ticks_prev = None
        self.right_ticks_prev = None

        self.left_ticks = 0
        self.right_ticks = 0

        # Robot pose state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Subscribers for encoder ticks
        # TODO - Update subscription
        self.left_sub = self.create_subscription(
            Int32, "left_wheel_ticks", self.left_callback, 10
        )
        self.right_sub = self.create_subscription(
            Int32, "right_wheel_ticks", self.right_callback, 10
        )

        # Publisher for odom
        self.odom_pub = self.create_publisher(Odometry, "odom", 10)

        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Timer for odom publishing
        dt = 1.0 / self.get_parameter("publish_rate").value
        self.timer = self.create_timer(dt, self.update)

        self.last_time = self.get_clock().now()

        self.get_logger().info("uwnre odom_node started")

    # ------------------- Encoder callbacks -------------------
    def left_callback(self, msg: Int32):
        self.left_ticks = msg.data

    def right_callback(self, msg: Int32):
        self.right_ticks = msg.data

    # ------------------- Main Odom Update Loop -------------------
    def update(self):
        if self.left_ticks_prev is None or self.right_ticks_prev is None:
            self.left_ticks_prev = self.left_ticks
            self.right_ticks_prev = self.right_ticks
            return

        # Compute tick deltas
        delta_left = self.left_ticks - self.left_ticks_prev
        delta_right = self.right_ticks - self.right_ticks_prev

        self.left_ticks_prev = self.left_ticks
        self.right_ticks_prev = self.right_ticks

        # Convert ticks → meters
        meters_per_tick = (2.0 * math.pi * self.wheel_radius) / self.ticks_per_rev
        dl = delta_left * meters_per_tick
        dr = delta_right * meters_per_tick

        # Compute linear and angular displacement
        dc = (dl + dr) / 2.0
        dtheta = (dr - dl) / self.wheel_base

        # Update pose
        self.theta += dtheta
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))  # normalize

        self.x += dc * math.cos(self.theta)
        self.y += dc * math.sin(self.theta)

        # Compute current velocity
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        v = dc / dt if dt > 0 else 0.0
        omega = dtheta / dt if dt > 0 else 0.0

        # Publish Odometry
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)

        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = omega

        self.odom_pub.publish(odom)

        # Publish odom → base_link TF
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
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
