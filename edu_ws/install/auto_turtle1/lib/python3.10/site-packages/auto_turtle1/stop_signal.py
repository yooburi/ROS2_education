"""Publish a one-shot stop signal for the turtlesim demo."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class StopSignal(Node):
    """Publish a stop signal on startup."""

    def __init__(self):
        super().__init__('stop_signal')
        self.publisher_ = self.create_publisher(
            Bool,
            '/stop_signal',
            10,
        )

        self.publish_stop_signal()

    def publish_stop_signal(self):
        """Publish a single True stop message."""
        msg = Bool()
        msg.data = True
        self.publisher_.publish(msg)
        self.get_logger().info('Published stop signal: True')


def main(args=None):
    """Run the stop-signal publisher node."""
    rclpy.init(args=args)
    node = StopSignal()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
