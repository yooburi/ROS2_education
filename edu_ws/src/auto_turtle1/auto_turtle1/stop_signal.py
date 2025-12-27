import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool  # std_msgs 패키지에서 Bool 메시지 타입을 임포트


class StopSignal(Node):
    def __init__(self):
        super().__init__('stop_signal')  # 노드 이름 설정
        self.publisher_ = self.create_publisher(
            Bool,
            '/stop_signal',
            10
        )

        self.publish_stop_signal()

    def publish_stop_signal(self):
        msg = Bool()
        msg.data = True  # 정지 신호 설정
        self.publisher_.publish(msg)
        self.get_logger().info('Published stop signal: True')


def main(args=None):
    rclpy.init(args=args)
    node = StopSignal()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()