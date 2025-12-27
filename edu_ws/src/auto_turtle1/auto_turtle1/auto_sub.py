import rclpy
from rclpy.node import Node

#HW1 FLOAt메시지 타입 임포트
from std_msgs.msg import Float32 # std_msgs 패키지에서 Float32 메시지 타입을 임포트
from turtlesim.msg import Pose # turtlesim 패키지에서 Pose 메시지 타입을 임포트
import math # 수학 모듈 임포트

class AutoSubscriber(Node):
    def __init__(self):
        super().__init__('pose_calc_distance')  # 노드 이름 설정
        self.center_x = 5.544445
        self.center_y = 5.544445
        
        # 구독자 생성
        self.subscription = self.create_subscription( 
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10)
        
        #HW1 퍼블리셔 생성
        self.publisher = self.create_publisher(
            Float32,
            '/distance_to_center',
            10)
        
    def pose_callback(self, msg):
        #현재 좌표
        turtle_x = msg.x
        turtle_y = msg.y
        
        # 중심과의 거리 계산
        distance = math.sqrt((turtle_x - self.center_x) ** 2 + (turtle_y - self.center_y) ** 2)
        self.get_logger().info(f'거리: {distance:.2f}')
        
        #Hw1 거리 퍼블리시
        distance_msg = Float32()
        distance_msg.data = float(distance)
        self.publisher.publish(distance_msg)


def main(args=None):
    rclpy.init(args=args)
    node = AutoSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()