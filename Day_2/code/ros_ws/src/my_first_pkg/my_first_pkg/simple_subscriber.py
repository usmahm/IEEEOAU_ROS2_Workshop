import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__('simple_subscriber')
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)

    simple_subscriber_node = SimpleSubscriber()
    rclpy.spin(simple_subscriber_node)
    
    simple_subscriber_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()