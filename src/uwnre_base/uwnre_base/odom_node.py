import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Int32
import tf2_ros
import math
import time
from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


class OdomNode(Node):
    def __init__(self):
        super().__init__("odom_node", namespace="uwnre")

        # ---- Declare parameters ----
        # TODO: Find the correct values and update in the launch file
        self.declare_parameter("wheel_radius", 0.16)          # meters
        self.declare_parameter("wheel_base", 0.60)            # meters
        self.declare_parameter("ticks_per_rev", 4096)
        self.declare_parameter("publish_rate", 25.0)          # Hz

        self.wheel_radius = self.get_parameter("wheel_radius").value
        self.wheel_base = self.get_parameter("wheel_base").value
        self.ticks_per_rev = self.get_parameter("ticks_per_rev").value

        # Robot pose state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Previous angles for left and right wheels
        self.prev_left_angle = 0.0
        self.prev_right_angle = 0.0

        # Initialize last_x and last_theta to avoid AttributeError
        self.last_x = 0.0
        self.last_theta = 0.0



        # Subscriber for scaled wheel angles
        self.angle_sub = self.create_subscription(
            JointState, "scaled_wheel_angles", self.angle_callback, 10
        )

        # Publisher for odom
        self.odom_pub = self.create_publisher(Odometry, "odom", 10)

        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Service to reset odometry: call 'reset_odometry' (std_srvs/Trigger)
        # The service clears x,y,theta and aligns previous tick samples so
        # no large jump occurs after reset.
        self.reset_service = self.create_service(
            Trigger, "reset_odometry", self.handle_reset_odometry
        )

        # Timer for odom publishing
        dt = 1.0 / self.get_parameter("publish_rate").value
        self.timer = self.create_timer(dt, self.update)

        self.last_time = self.get_clock().now()

        self.get_logger().info("uwnre odom_node started")

    # Function to normalize angles to the range [-pi, pi]
    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    # ------------------- Service to reset odometry -------------------
    def handle_reset_odometry(self, request, response):
        """Service callback to reset odometry pose/state.

        Uses std_srvs/Trigger: no request data. Returns success True and a message.
        """
        # Reset pose
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Reset timing reference
        self.last_time = self.get_clock().now()

        self.get_logger().info("Odometry reset via service 'reset_odometry'")

        response.success = True
        response.message = "Odometry reset"
        return response

    # ------------------- Main Odom Update Loop -------------------
    def update(self):
        # Compute current velocity
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        v = (self.x - self.last_x) / dt if dt > 0 else 0.0
        omega = (self.theta - self.last_theta) / dt if dt > 0 else 0.0

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

    # Update the odometry calculation logic
    def angle_callback(self, msg):
        left_angle = msg.position[0]
        right_angle = msg.position[1]

        # Calculate odometry based on wheel angles
        dt = (self.get_clock().now() - self.last_time).nanoseconds / 1e9
        self.last_time = self.get_clock().now()

        # Calculate the change in position and orientation
        delta_left = left_angle * self.wheel_radius
        delta_right = right_angle * self.wheel_radius
        delta_theta = (delta_right - delta_left) / self.wheel_base

        # Update the robot's pose
        self.theta += delta_theta
        self.x += (delta_left + delta_right) / 2 * math.cos(self.theta)
        self.y += (delta_left + delta_right) / 2 * math.sin(self.theta)

        # Normalize the angle to the range [-pi, pi]
        self.theta = self.normalize_angle(self.theta)

        # Publish odometry
        self.publish_odometry()

    def publish_odometry(self):
        # Compute current velocity
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        v = (self.x - self.last_x) / dt if dt > 0 else 0.0
        omega = (self.theta - self.last_theta) / dt if dt > 0 else 0.0

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
