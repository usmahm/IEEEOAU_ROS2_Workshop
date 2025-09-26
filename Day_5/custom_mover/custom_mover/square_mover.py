import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class SquareMover(Node):
    def __init__(self):
        super().__init__('square_mover')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        # State initialization
        self.state = "forward"   # "forward" or "turn"
        self.step_count = 0      # counts ticks
        self.edge_count = 0      # counts edges of square

        # Parameters
        self.forward_steps = 50   # ~5s forward
        self.turn_steps = 27      # ~2.7s turn
        
        self.timer = self.create_timer(0.1, self.move_robot)

        
    def move_robot(self):
        msg = Twist()

        # Stop if max loops reached
        if self.edge_count >= 4:
            self.publisher_.publish(msg)
            self.get_logger().info("Completed square pattern movement ✅")
            rclpy.shutdown()
            return

        if self.state == "forward":
            if self.step_count == 0:
                self.get_logger().info(f"Moving turtlebot in a straight line")

            msg.linear.x = 0.2
            msg.angular.z = 0.0
            self.step_count += 1

            if self.step_count >= self.forward_steps:
                self.edge_count += 1
                self.state = "turn"
                self.step_count = 0

        elif self.state == "turn":
            if self.step_count == 0:
                self.get_logger().info(f"Rotating turtlebot3")

            msg.linear.x = 0.0
            msg.angular.z = 0.7
            self.step_count += 1

            if self.step_count >= self.turn_steps:
                self.state = "forward"
                self.step_count = 0

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SquareMover()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop_msg = Twist()
        node.publisher_.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
