import rclpy
from rclpy.node import Node

#HW2 Bool메시지 타입 임포트
from std_msgs.msg import Bool # std_msgs 패키지에서 Bool 메시지 타입을 임포트
from geometry_msgs.msg import Twist # geometry_msgs 패키지에서 Twist 메시지 타입을 임포트
import time # time 모듈 임포트

class AutoPublisher(Node):
    def __init__(self):
        super().__init__('square_move') # 노드 이름 설정
        
        #HW2 정지 상태 변수
        self.is_stopped = False

        self.pub = self.create_publisher(
            Twist,
            'turtle1/cmd_vel',
            10
            ) # 퍼블리셔 생성

        #HW2 구독자 생성
        self.sub = self.create_subscription(
            Bool,
            '/stop_signal',
            self.stop_callback,
            10
            )

        self.move_square() # 사각형 이동 함수 호출
    
    #HW2 정지 콜백 함수
    def stop_callback(self, msg):
        if msg.data: # 메시지 데이터가 True
            self.get_logger().info('Stop signal received. Stopping the turtle.')
            
            # 정지 상태 업데이트
            self.is_stopped = True

            stop_msg = Twist() # 정지 명령을 위한 Twist 메시지 생성
            stop_msg.linear.x = 0.0
            stop_msg.angular.z = 0.0
            self.pub.publish(stop_msg) # 정지 명령 퍼블리시
    
    def move_square(self):
        msg = Twist() # Twist 메시지 객체 생성
        for _ in range(100):
            # 정지 신호 수신 시 루프 종료
            if self.is_stopped:
                self.get_logger().info('Turtle has been stopped. Exiting move loop.')
                break

            # 직진
            msg.linear.x = 1.0
            msg.angular.z = 0.0
            self.pub.publish(msg) # 메시지 퍼블리시
            self.get_logger().info('Moving forward')
            #time.sleep(2)

            #sleep만 쓰면 콜백함수가 작동하지 않음
            for _ in range(20):
                if self.is_stopped:
                    break
                rclpy.spin_once(self, timeout_sec=0.1) # 콜백 함수 처리
            
            if self.is_stopped:
                break

            # 회전
            msg.linear.x = 0.0
            msg.angular.z = 1.57 # 약 90도 회전
            self.pub.publish(msg) # 메시지 퍼블리시
            self.get_logger().info('Turning')

            for _ in range(20):
                if self.is_stopped:
                    break
                rclpy.spin_once(self, timeout_sec=0.1) # 콜백 함수 처리 

            if self.is_stopped:
                break

def main(args=None):
    rclpy.init(args=args)
    node = AutoPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()