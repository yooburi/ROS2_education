#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

import cv2
from cv_bridge import CvBridge
import numpy as np


class OpticalFlowTurtle(Node):
    def __init__(self):
        super().__init__('optical_flow_turtle')

        self.bridge = CvBridge()
        self.prev_gray = None  # 이전 프레임 저장

        # 백파일에서 나오는 이미지 토픽 (이름 맞춰서 수정)
        self.sub = self.create_subscription(
            Image,
            '',
            self.image_callback,
            10
        )

        # turtlesim 제어 토픽
        self.pub = self.create_publisher(
            Twist,
            '',
            10
        )

        # 간단한 파라미터들 (강의 중 튜닝해보기 좋음)
        self.base_speed = 0.3     # 전진 속도
        self.k_angular = 0.15      # 회전 강도 gain
        self.flow_mag_th = 0.3    # “움직임 있음”으로 볼 최소 평균 크기

    def image_callback(self, msg: Image):
        # ROS Image → OpenCV BGR
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # 흑백으로 변환
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # 첫 프레임이면 prev_gray만 저장하고 종료
        if self.prev_gray is None:
            self.prev_gray = gray
            return

        # Farneback dense optical flow 계산
        flow = cv2.calcOpticalFlowFarneback(
            self.prev_gray, gray,
            None,
            0.5,   # pyr_scale
            3,     # levels
            15,    # winsize
            3,     # iterations
            5,     # poly_n
            1.2,   # poly_sigma
            0
        )

        # 다음 계산을 위해 현재 프레임을 prev로 저장
        self.prev_gray = gray

        # 흐름 벡터 분리
        fx = flow[..., 0]
        fy = flow[..., 1]
        mag = np.sqrt(fx**2 + fy**2)

        # 너무 귀퉁이 노이즈는 빼고 중앙 영역만 사용 (상하 1/3 잘라내기)
        h, w = gray.shape
        y1, y2 = h // 3, 2 * h // 3
        center_fx = fx[y1:y2, :]
        center_mag = mag[y1:y2, :]

        mean_fx = float(np.mean(center_fx))
        mean_mag = float(np.mean(center_mag))

        twist = Twist()

        if mean_mag > self.flow_mag_th:
            # 움직임이 어느 정도 있다 → 앞으로 가면서 방향 조절
            twist.linear.x = self.base_speed
            # 평균 흐름이 오른쪽(-면) → 배경이 오른쪽으로 움직이므로
            # 카메라는 왼쪽으로 돌고 있다는 느낌 → 반대로 보정
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

        # 거북이에 속도 명령 전송
        self.pub.publish()


def main(args=None):
    rclpy.init(args=args)
    node = OpticalFlowTurtle()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
