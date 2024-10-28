import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # You can change this to the type of message you expect
from luci_messages.msg import LuciEncoders

class EncoderNode(Node):

    def __init__(self):
        super().__init__('encoder_node')
        
        # Publisher
        self.publisher = self.create_publisher(String, '/encoder_stuff', 10)
        
        # Subscriber
        self.subscription = self.create_subscription(LuciEncoders, '/luci/encoders', self.listener_callback, 10)
        
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        # self.get_logger().info(f'Received: "{msg.header.stamp.sec}"')
        right_angle = msg.right_angle
        left_angle = msg.left_angle
        seconds = msg.header.stamp.sec
        nanosec = msg.header.stamp.nanosec
        timestamp = float(str(seconds)+'.'+str(nanosec))
        pub_string = 'Time {:<20} Right: {:<10}   Left: {:<10}'.format(timestamp, right_angle, left_angle)
        self.get_logger().info(pub_string)
        self.publish_stuff(pub_string)

    def publish_stuff(self, string_to_pub):
        # Do something with the received message and publish a new one
        msg = String()
        msg.data = 'Processed encoder data: '
        self.publisher.publish(msg)  # Publish the message
        self.get_logger().info(f'Published: {msg.data}  {string_to_pub}')

def main(args=None):
    rclpy.init(args=args)
    
    # Create node instance
    node = EncoderNode()

    # Spin node to keep it active and process messages
    rclpy.spin(node)

    # Clean up when the node is stopped
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


# [INFO] [1728677252.995560804] [encoder_node]: Received: "luci_messages.msg.LuciEncoders(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1728677252, nanosec=994566903), frame_id='base_link'), left_angle=153.19000244140625, right_angle=86.91999816894531, fl_caster_degrees=0.0, bl_caster_degrees=0.0, fr_caster_degrees=0.0, br_caster_degrees=0.0)"
