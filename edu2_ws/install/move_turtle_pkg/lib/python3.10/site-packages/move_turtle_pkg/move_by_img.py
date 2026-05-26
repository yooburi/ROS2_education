#!/usr/bin/env python3
"""Control turtlesim with optical flow measured from an image stream."""

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Image


class OpticalFlowTurtle(Node):
    """Steer a turtlesim turtle from the average horizontal optical flow."""

    def __init__(self):
        super().__init__('optical_flow_turtle')

        self.declare_parameter('image_topic', '/usb_cam_1/image_raw')
        self.declare_parameter('cmd_vel_topic', '/turtle1/cmd_vel')

        image_topic = self.get_parameter('image_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

        self.prev_gray = None

        self.sub = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10
        )

        self.pub = self.create_publisher(
            Twist,
            cmd_vel_topic,
            10
        )

        self.base_speed = 0.3
        self.k_angular = 0.15
        self.flow_mag_th = 0.3

        self.get_logger().info(
            f'Listening on {image_topic} and publishing to {cmd_vel_topic}.'
        )

    def image_to_gray(self, msg: Image):
        """Convert a sensor_msgs/Image message into an 8-bit grayscale frame."""
        encoding = msg.encoding.lower()

        try:
            row_data = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height,
                msg.step,
            )
        except ValueError:
            self.get_logger().error(
                f'Failed to reshape image buffer for encoding {msg.encoding}.'
            )
            return None

        if encoding in ('mono8', '8uc1'):
            if msg.step < msg.width:
                self.get_logger().error(
                    f'Image step {msg.step} is smaller than width {msg.width}.'
                )
                return None
            return row_data[:, :msg.width].copy()

        channel_map = {
            'bgr8': (3, cv2.COLOR_BGR2GRAY),
            'rgb8': (3, cv2.COLOR_RGB2GRAY),
            'bgra8': (4, cv2.COLOR_BGRA2GRAY),
            'rgba8': (4, cv2.COLOR_RGBA2GRAY),
        }
        if encoding not in channel_map:
            self.get_logger().error(
                f'Unsupported image encoding: {msg.encoding}.'
            )
            return None

        channels, color_code = channel_map[encoding]
        expected_row_width = msg.width * channels
        if msg.step < expected_row_width:
            self.get_logger().error(
                f'Image step {msg.step} is smaller than expected '
                f'row width {expected_row_width} for {msg.encoding}.'
            )
            return None

        image = row_data[:, :expected_row_width].reshape(
            msg.height,
            msg.width,
            channels,
        )
        return cv2.cvtColor(np.ascontiguousarray(image), color_code)

    def image_callback(self, msg: Image):
        """Compute dense optical flow and publish a turtlesim velocity."""
        gray = self.image_to_gray(msg)
        if gray is None:
            return

        if self.prev_gray is None:
            self.prev_gray = gray
            return

        flow = cv2.calcOpticalFlowFarneback(
            self.prev_gray, gray,
            None,
            0.5,
            3,
            15,
            3,
            5,
            1.2,
            0
        )

        self.prev_gray = gray

        fx = flow[..., 0]
        fy = flow[..., 1]
        mag = np.sqrt(fx ** 2 + fy ** 2)

        h, _ = gray.shape
        y1, y2 = h // 3, 2 * h // 3
        center_fx = fx[y1:y2, :]
        center_mag = mag[y1:y2, :]

        mean_fx = float(np.mean(center_fx))
        mean_mag = float(np.mean(center_mag))

        twist = Twist()

        if mean_mag > self.flow_mag_th:
            twist.linear.x = self.base_speed
            twist.angular.z = self.k_angular * mean_fx

            self.get_logger().info(
                f"flow: mean_fx={mean_fx:.3f}, mean_mag={mean_mag:.3f} "
                f"=> v={twist.linear.x:.2f}, w={twist.angular.z:.2f}"
            )
        else:
            # 흐름이 거의 없으면 일단 정지
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.get_logger().info(
                f"flow too small (mean_mag={mean_mag:.3f}), stop."
            )

        self.pub.publish(twist)


def main(args=None):
    """Run the optical-flow turtlesim controller node."""
    rclpy.init(args=args)
    node = OpticalFlowTurtle()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
