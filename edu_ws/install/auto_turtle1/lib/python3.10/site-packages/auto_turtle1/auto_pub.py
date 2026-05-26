"""Drive turtlesim in a square until a stop signal is received."""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool


class AutoPublisher(Node):
    """Publish velocity commands for square movement."""

    def __init__(self):
        super().__init__('square_move')

        self.is_stopped = False

        self.pub = self.create_publisher(
            Twist,
            'turtle1/cmd_vel',
            10,
        )
        self.sub = self.create_subscription(
            Bool,
            '/stop_signal',
            self.stop_callback,
            10,
        )

        self.move_square()

    def stop_callback(self, msg):
        """Stop the turtle when a True stop signal arrives."""
        if msg.data:
            self.get_logger().info('Stop signal received. Stopping the turtle.')
            self.is_stopped = True

            stop_msg = Twist()
            stop_msg.linear.x = 0.0
            stop_msg.angular.z = 0.0
            self.pub.publish(stop_msg)

    def move_square(self):
        """Move forward and turn repeatedly to approximate a square."""
        msg = Twist()

        for _ in range(100):
            if self.is_stopped:
                self.get_logger().info(
                    'Turtle has been stopped. Exiting move loop.'
                )
                break

            msg.linear.x = 1.0
            msg.angular.z = 0.0
            self.pub.publish(msg)
            self.get_logger().info('Moving forward')

            for _ in range(20):
                if self.is_stopped:
                    break
                rclpy.spin_once(self, timeout_sec=0.1)

            if self.is_stopped:
                break

            msg.linear.x = 0.0
            msg.angular.z = 1.57
            self.pub.publish(msg)
            self.get_logger().info('Turning')

            for _ in range(20):
                if self.is_stopped:
                    break
                rclpy.spin_once(self, timeout_sec=0.1)

            if self.is_stopped:
                break


def main(args=None):
    """Run the square-movement turtlesim node."""
    rclpy.init(args=args)
    node = AutoPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
