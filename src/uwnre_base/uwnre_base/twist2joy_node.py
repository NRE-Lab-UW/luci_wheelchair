import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import Twist
from luci_messages.msg import LuciJoystick
from std_srvs.srv import Empty

# Joystick zones – must match LUCI’s definitions
JS_FRONT = 0
JS_FRONT_LEFT = 1
JS_FRONT_RIGHT = 2
JS_LEFT = 3
JS_RIGHT = 4
JS_BACK_RIGHT = 5
JS_BACK_LEFT = 6
JS_BACK = 7
JS_ORIGIN = 8

# Input source enum – must match proto
REMOTE = 1


class TwistToJoystickNode(Node):
    def __init__(self):
        super().__init__("twist2joy_node", namespace="uwnre")

        # ----- Parameters -----
        # Max speeds used for scaling Twist -> joystick [-100, 100]
        # TODO: Determine what the max values are
        self.declare_parameter("cmd_vel_topic", "/uwnre/cmd_vel")
        self.declare_parameter("max_forward_speed", 10)      # m/s
        self.declare_parameter("max_reverse_speed", -10)      # m/s (magnitude)
        self.declare_parameter("max_angular_speed", 10)      # rad/s
        self.declare_parameter("publish_rate", 20.0)          # Hz
        self.declare_parameter("deadman_timeout", 0.5)        # seconds

        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.max_forward_speed = float(self.get_parameter("max_forward_speed").value)
        self.max_reverse_speed = float(self.get_parameter("max_reverse_speed").value)
        self.max_angular_speed = float(self.get_parameter("max_angular_speed").value)
        self.publish_rate = float(self.get_parameter("publish_rate").value)
        self.deadman_timeout = float(self.get_parameter("deadman_timeout").value)

        # ----- State -----
        self.last_cmd_vel = Twist()
        self.last_cmd_time = self.get_clock().now()
        self.have_cmd = False  # have we ever seen a cmd_vel yet?

        # ----- Publishers / Subscribers -----
        # LUCI joystick topic – keep absolute so it matches your existing teleop
        self.joy_pub = self.create_publisher(LuciJoystick, "/luci/remote_joystick", 10)

        # Subscribe to cmd_vel (from Nav2 or anything else)
        self.cmd_sub = self.create_subscription(
            Twist,
            self.cmd_vel_topic,
            self.cmd_vel_callback,
            10,
        )

        # Services to enable / disable auto remote input (like your teleop node)
        self.set_auto_input_client = self.create_client(Empty, "/luci/set_auto_remote_input")
        self.rm_auto_input_client = self.create_client(Empty, "/luci/remove_auto_remote_input")

        # Wait for the auto input service once on startup
        while not self.set_auto_input_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /luci/set_auto_remote_input service...")

        self.set_auto_service()  # enable auto remote input

        # Timer to publish joystick at fixed rate
        dt = 1.0 / self.publish_rate
        self.timer = self.create_timer(dt, self.timer_callback)

        self.get_logger().info(
            f"twist2joy_node started. Listening on {self.cmd_vel_topic}, "
            "publishing to /luci/remote_joystick"
        )
        
        self.get_logger().info("testing bruh")

    # ----- Service helpers -----
    def set_auto_service(self):
        req = Empty.Request()
        future = self.set_auto_input_client.call_async(req)
        future.add_done_callback(self.handle_service_response)

    def rm_auto_service(self):
        req = Empty.Request()
        future = self.rm_auto_input_client.call_async(req)
        future.add_done_callback(self.handle_service_response)

    def handle_service_response(self, future):
        try:
            future.result()
            self.get_logger().info("Auto remote input service call succeeded")
        except Exception as e:
            self.get_logger().error(f"Auto remote input service call failed: {e}")

    # ----- Subscribers -----
    def cmd_vel_callback(self, msg: Twist):
        self.last_cmd_vel = msg
        self.last_cmd_time = self.get_clock().now()
        self.have_cmd = True

    # ----- Core mapping logic -----
    def timer_callback(self):
        now = self.get_clock().now()
        time_since_cmd = (now - self.last_cmd_time).nanoseconds / 1e9

        # Deadman: if no cmd_vel recently, send zero joystick
        if (not self.have_cmd) or (time_since_cmd > self.deadman_timeout):
            msg = self.zero_joystick()
            self.joy_pub.publish(msg)
            # don’t spam logs every tick – comment this in if you want
            # self.get_logger().debug("Deadman timeout – sending zero joystick")
            return

        # Map last cmd_vel → joystick
        v = self.last_cmd_vel.linear.x
        omega = self.last_cmd_vel.angular.z

        joy_msg = LuciJoystick()
        joy_msg.input_source = REMOTE

        # Forward/back mapping
        fb = self.map_linear_to_joystick(v)
        fb = 10
        self.get_logger().info(f"FORWARD BACK: {fb}")
        lr = self.map_angular_to_joystick(omega)

        joy_msg.forward_back = 10
        joy_msg.left_right = lr
        joy_msg.joystick_zone = self.compute_zone(fb, lr)

        self.joy_pub.publish(joy_msg)

        # Debug print (tune / remove later)
        self.get_logger().debug(
            f"cmd_vel: v={v:.3f} m/s, omega={omega:.3f} rad/s "
            f"-> joystick fb={fb}, lr={lr}, zone={joy_msg.joystick_zone}"
        )

    def zero_joystick(self) -> LuciJoystick:
        msg = LuciJoystick()
        msg.input_source = REMOTE
        msg.forward_back = 0
        msg.left_right = 0
        msg.joystick_zone = JS_ORIGIN
        return msg

    def map_linear_to_joystick(self, v: float) -> int:
        """
        Map linear.x (m/s) to [-100, 100] for forward_back.
        Uses separate limits for forward / reverse so you can
        be slower in reverse if you want.
        """
        if v > 0.0:
            if self.max_forward_speed <= 0.0:
                return 0
            ratio = v / self.max_forward_speed
        elif v < 0.0:
            if self.max_reverse_speed <= 0.0:
                return 0
            ratio = v / self.max_reverse_speed  # note: max_reverse_speed is positive magnitude
        else:
            return 0

        ratio = max(min(ratio, 1.0), -1.0)  # clamp to [-1, 1]
        return int(round(ratio * 100.0))

    def map_angular_to_joystick(self, omega: float) -> int:
        """
        Map angular.z (rad/s) to [-100, 100] for left_right.
        Positive omega = turn left, negative = turn right.
        """
        if self.max_angular_speed <= 0.0:
            return 0

        ratio = omega / self.max_angular_speed
        ratio = max(min(ratio, 1.0), -1.0)
        return int(round(ratio * 100.0))

    def compute_zone(self, fb: int, lr: int) -> int:
        """
        Rough joystick zone classification based on forward/back and left/right.
        You can refine this later; for now we just pick a basic direction.
        """
        # small threshold so tiny values count as zero
        fb_abs = abs(fb)
        lr_abs = abs(lr)
        thresh = 5

        # TODO: There are combination zones to test/add later

        if fb_abs < thresh and lr_abs < thresh:
            return JS_ORIGIN

        if fb_abs >= lr_abs:
            # more forward/back than turning
            if fb > 0:
                return JS_FRONT
            else:
                return JS_BACK
        else:
            # more turning than forward/back
            if lr > 0:
                return JS_RIGHT
            else:
                return JS_LEFT


def main(args=None):
    rclpy.init(args=args)
    node = TwistToJoystickNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down twist2joy_node (KeyboardInterrupt)")
    finally:
        # make sure we release auto remote input
        node.rm_auto_service()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
