import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import Twist


class TestCmdVelPublisher(Node):
    def __init__(self):
        super().__init__("test_cmd_vel_pub", namespace="uwnre")

        # ----- Parameters -----
        # Topic to publish to (should match twist2joy's input)
        self.declare_parameter("cmd_vel_topic", "/uwnre/cmd_vel")
        self.declare_parameter("linear_x", 1.5)      # m/s
        self.declare_parameter("angular_z", 0.0)     # rad/s
        self.declare_parameter("duration", 0.0)      # seconds; <=0 -> run forever
        self.declare_parameter("rate", 10.0)         # Hz

        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.linear_x = float(self.get_parameter("linear_x").value)
        self.angular_z = float(self.get_parameter("angular_z").value)
        self.duration = float(self.get_parameter("duration").value)
        self.rate = float(self.get_parameter("rate").value)

        # ----- Publisher -----
        self.pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        # Compute stop time if we have a finite duration
        self.start_time = self.get_clock().now()
        if self.duration > 0.0:
            self.end_time = self.start_time + Duration(seconds=self.duration)
        else:
            self.end_time = None  # run forever

        # Timer
        dt = 1.0 / self.rate if self.rate > 0.0 else 0.1
        self.timer = self.create_timer(dt, self.timer_callback)

        self.sent_stop = False

        self.get_logger().info(
            f"test_cmd_vel_pub started.\n"
            f"  topic: {self.cmd_vel_topic}\n"
            f"  linear_x: {self.linear_x} m/s\n"
            f"  angular_z: {self.angular_z} rad/s\n"
            f"  duration: {'infinite' if self.end_time is None else self.duration} s\n"
            f"  rate: {self.rate} Hz"
        )

    def timer_callback(self):
        now = self.get_clock().now()

        # If we have a duration and it's passed, send zero once and stop publishing
        if self.end_time is not None and now >= self.end_time:
            if not self.sent_stop:
                self.get_logger().info("Duration elapsed – sending zero Twist and stopping publishing.")
                stop_msg = Twist()
                self.pub.publish(stop_msg)
                self.sent_stop = True

            # stop the timer; node will keep running until Ctrl+C
            self.timer.cancel()
            return

        # Publish the commanded Twist
        msg = Twist()
        msg.linear.x = self.linear_x
        msg.angular.z = self.angular_z
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TestCmdVelPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down test_cmd_vel_pub (KeyboardInterrupt)")
        # send a final zero Twist on exit
        stop_msg = Twist()
        node.pub.publish(stop_msg)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
