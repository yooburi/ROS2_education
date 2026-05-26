"""Publish the turtle's distance from the turtlesim map center."""

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from turtlesim.msg import Pose


class AutoSubscriber(Node):
    """Compute and publish the turtle's distance to the map center."""

    def __init__(self):
        super().__init__('pose_calc_distance')

        self.center_x = 5.544445
        self.center_y = 5.544445

        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10,
        )
        self.publisher = self.create_publisher(
            Float32,
            '/distance_to_center',
            10,
        )

    def pose_callback(self, msg):
        """Publish the current distance between the turtle and the center."""
        distance = math.sqrt(
            (msg.x - self.center_x) ** 2 + (msg.y - self.center_y) ** 2
        )
        self.get_logger().info(f'거리: {distance:.2f}')

        distance_msg = Float32()
        distance_msg.data = float(distance)
        self.publisher.publish(distance_msg)


def main(args=None):
    """Run the turtlesim distance publisher node."""
    rclpy.init(args=args)
    node = AutoSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
