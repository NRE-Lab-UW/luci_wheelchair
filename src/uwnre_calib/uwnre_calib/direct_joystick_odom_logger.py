import os
import csv

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from nav_msgs.msg import Odometry
from luci_messages.msg import LuciJoystick
from std_srvs.srv import Empty


# Joystick zones – should match LUCI’s enums
JS_FRONT = 0
JS_FRONT_LEFT = 1
JS_FRONT_RIGHT = 2
JS_LEFT = 3
JS_RIGHT = 4
JS_BACK_LEFT = 5
JS_BACK_RIGHT = 6
JS_BACK = 7
JS_ORIGIN = 8

REMOTE = 1  # InputSource.Remote


class DirectJoystickOdomLogger(Node):
    def __init__(self):
        super().__init__("direct_joystick_odom_logger")

        # ---------- Parameters ----------
        self.declare_parameter("joystick_topic", "/luci/remote_joystick")
        self.declare_parameter("odom_topic", "/uwnre/odom")
        self.declare_parameter("forward_back", 40)      # [-100, 100]
        self.declare_parameter("left_right", 0)         # [-100, 100]
        self.declare_parameter("duration", 5.0)         # seconds; <=0 -> run forever
        self.declare_parameter("rate", 10.0)            # Hz
        self.declare_parameter("output_csv", "joystick_odom_log.csv")

        self.joystick_topic = self.get_parameter("joystick_topic").value
        self.odom_topic = self.get_parameter("odom_topic").value
        self.forward_back = int(self.get_parameter("forward_back").value)
        self.left_right = int(self.get_parameter("left_right").value)
        self.duration = float(self.get_parameter("duration").value)
        self.rate = float(self.get_parameter("rate").value)
        self.output_csv = self.get_parameter("output_csv").value

        # ---------- State ----------
        self.latest_odom = None
        self.have_odom = False

        # ---------- Publisher / Subscriber ----------
        self.joy_pub = self.create_publisher(LuciJoystick, self.joystick_topic, 10)

        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10,
        )

        # Services to enable/disable auto remote input (same as teleop)
        self.set_auto_input_client = self.create_client(Empty, "/luci/set_auto_remote_input")
        self.rm_auto_input_client = self.create_client(Empty, "/luci/remove_auto_remote_input")

        while not self.set_auto_input_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /luci/set_auto_remote_input service...")

        self.set_auto_service()

        # ---------- Timing ----------
        self.start_time = self.get_clock().now()
        if self.duration > 0.0:
            self.end_time = self.start_time + Duration(seconds=self.duration)
        else:
            self.end_time = None

        dt = 1.0 / self.rate if self.rate > 0.0 else 0.1
        self.timer = self.create_timer(dt, self.timer_callback)

        # ---------- CSV Setup ----------
        # Open file in CWD by default; you can pass an absolute path via parameter
        self.csv_file = open(self.output_csv, "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            "time_sec",
            "x",
            "y",
            "theta",
            "lin_vel_x",
            "ang_vel_z",
            "forward_back",
            "left_right",
        ])
        self.csv_file.flush()

        self.sent_stop = False

        self.get_logger().info(
            "DirectJoystickOdomLogger started.\n"
            f"  joystick_topic: {self.joystick_topic}\n"
            f"  odom_topic: {self.odom_topic}\n"
            f"  fb: {self.forward_back}, lr: {self.left_right}\n"
            f"  duration: {'infinite' if self.end_time is None else self.duration} s\n"
            f"  rate: {self.rate} Hz\n"
            f"  output_csv: {os.path.abspath(self.output_csv)}"
        )

    # ---------- Service helpers ----------
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

    # ---------- Callbacks ----------
    def odom_callback(self, msg: Odometry):
        self.latest_odom = msg
        self.have_odom = True

    def timer_callback(self):
        now = self.get_clock().now()

        # Handle duration timeout: send one zero, then stop publishing
        if self.end_time is not None and now >= self.end_time:
            if not self.sent_stop:
                self.get_logger().info("Duration elapsed – sending zero joystick and stopping publisher.")
                zero_msg = self.zero_joystick()
                self.joy_pub.publish(zero_msg)
                self.sent_stop = True
            self.timer.cancel()
            return

        # Build joystick message
        joy_msg = LuciJoystick()
        joy_msg.input_source = REMOTE
        joy_msg.forward_back = self.forward_back
        joy_msg.left_right = self.left_right
        joy_msg.joystick_zone = self.compute_zone(self.forward_back, self.left_right)

        # Publish joystick command
        self.joy_pub.publish(joy_msg)

        # Log odom + joystick to CSV if we have odom
        if self.have_odom and self.latest_odom is not None:
            t = now.nanoseconds / 1e9
            x = self.latest_odom.pose.pose.position.x
            y = self.latest_odom.pose.pose.position.y

            # Recover yaw from quaternion (we only care about planar)
            qz = self.latest_odom.pose.pose.orientation.z
            qw = self.latest_odom.pose.pose.orientation.w
            theta = 2.0 * math.atan2(qz, qw)

            vx = self.latest_odom.twist.twist.linear.x
            wz = self.latest_odom.twist.twist.angular.z

            self.csv_writer.writerow([
                f"{t:.3f}",
                f"{x:.6f}",
                f"{y:.6f}",
                f"{theta:.6f}",
                f"{vx:.6f}",
                f"{wz:.6f}",
                self.forward_back,
                self.left_right,
            ])
            self.csv_file.flush()

    def zero_joystick(self) -> LuciJoystick:
        msg = LuciJoystick()
        msg.input_source = REMOTE
        msg.forward_back = 0
        msg.left_right = 0
        msg.joystick_zone = JS_ORIGIN
        return msg

    def compute_zone(self, fb: int, lr: int) -> int:
        """Simple zone classification similar to twist2joy."""
        fb_abs = abs(fb)
        lr_abs = abs(lr)
        thresh = 5

        if fb_abs < thresh and lr_abs < thresh:
            return JS_ORIGIN

        if fb_abs >= lr_abs:
            if fb > 0:
                return JS_FRONT
            else:
                return JS_BACK
        else:
            if lr > 0:
                return JS_RIGHT
            else:
                return JS_LEFT

    def destroy_node(self):
        # on shutdown, try to send a final zero and close CSV
        try:
            zero_msg = self.zero_joystick()
            self.joy_pub.publish(zero_msg)
        except Exception:
            pass

        try:
            self.rm_auto_service()
        except Exception:
            pass

        try:
            self.csv_file.close()
        except Exception:
            pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DirectJoystickOdomLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down direct_joystick_odom_logger (KeyboardInterrupt)")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
