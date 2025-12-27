# ROS 교육 자료 (ROS 2 Humble)

ROS 2 기초/응용 교육에서 사용한 예제 코드와 슬라이드(PDF)를 포함합니다. 두 개의 독립적인 워크스페이스(1차시, 2차시)가 있으며 각 워크스페이스는 개별적으로 빌드·실행합니다.

<table>
  <tr>
    <td>
      <img src="https://github.com/user-attachments/assets/67ea5bd6-af5d-43c5-9afb-01e7bea46a3c" alt="첫 번째 GIF 이미지" width="400">
    </td>
    <td>
      <img src="https://github.com/user-attachments/assets/2546a693-0db9-4d4c-a991-ea00ebed739c" alt="두 번째 GIF 이미지" width="400">
    </td>
  </tr>
</table>


## 사전 준비

- 권장 환경: Ubuntu 22.04 + ROS 2 Humble
- 기본 도구: `colcon`, `rosdep`
- 데모 패키지별 의존성
  - 공통: `rclpy`, `geometry_msgs`, `std_msgs`
  - 1차시(turtlesim): `turtlesim`
  - 2차시(이미지 처리): `sensor_msgs`, `cv_bridge`, `OpenCV`, `numpy`

의존성 설치 명령:

```bash
sudo apt update
sudo apt install -y \
  ros-humble-desktop \
  python3-colcon-common-extensions \
  ros-humble-turtlesim \
  ros-humble-cv-bridge \
  python3-opencv python3-numpy

# ROS 환경 설정(새 터미널마다 적용)
source /opt/ros/humble/setup.bash
```

## 1차시: auto_turtle1

퍼블리셔/서브스크라이버 기초를 다루며 `turtlesim`을 제어합니다.

- 노드 개요
  - `auto_sub`: `/turtle1/pose` 구독 → 거북이의 현재 위치로부터 맵 중앙까지의 거리 계산 → `/distance_to_center`(Float32) 퍼블리시
  - `auto_pub`: `turtle1/cmd_vel` 퍼블리시로 사각형 경로 이동. `/stop_signal`(Bool) 수신 시 정지
  - `stop_signal`: `/stop_signal` True를 1회 퍼블리시(정지 트리거)

- 실행 예시(터미널 3개 권장)
  1) turtlesim 실행
     ```bash
     source /opt/ros/humble/setup.bash
     ros2 run turtlesim turtlesim_node
     ```
  2) 거리 계산/퍼블리시 노드
     ```bash
     source edu_ws/install/setup.bash
     ros2 run auto_turtle1 auto_sub
     # 확인(옵션):
     ros2 topic echo /distance_to_center
     ```
  3) 사각형 이동 및 정지
     ```bash
     source edu_ws/install/setup.bash
     ros2 run auto_turtle1 auto_pub
     # 정지시키기(필요 시 별도 터미널에서):
     ros2 run auto_turtle1 stop_signal
     ```

- 확인/디버깅
  - 토픽 목록: `ros2 topic list`
  - 토픽 정보: `ros2 topic info /turtle1/pose`
  - 메시지 보기: `ros2 topic echo /distance_to_center`

---

## 2차시: move_turtle_pkg

OpenCV Farneback Dense Optical Flow를 이용해 이미지 흐름으로 거북이 회전을 보정하는 데모입니다. 실습용으로 일부 항목은 TODO 상태입니다.

- 주요 스크립트: `edu2_ws/src/move_turtle_pkg/move_turtle_pkg/move_by_img.py`
- 튜닝 파라미터: `base_speed`, `k_angular`, `flow_mag_th`

### 실습 TODO(필수 수정 포인트)

1) `package.xml` 의존성 보강
   - `sensor_msgs`, `geometry_msgs`, `cv_bridge` 등을 `<depend>`에 추가
2) `setup.py` 엔트리 포인트 수정
   - `move_by_img = move_turtle_pkg.move_by_img:main`
3) 코드 내 토픽 이름 지정
   - 이미지 구독 토픽: 예) `'/camera/image_raw'` (카메라/백(bag) 파일에 맞춰 수정)
   - 제어 퍼블리시 토픽: `'/turtle1/cmd_vel'`
4) 퍼블리시 호출 인자 보완
   - `self.pub.publish(twist)` 형태로 메시지 객체를 전달

### 실행 예시(토픽 연결 후)

- turtlesim 실행
  ```bash
  source /opt/ros/humble/setup.bash
  ros2 run turtlesim turtlesim_node
  ```
- 이미지 소스 제공(택 1)
  - 카메라 드라이버 실행 또는 bag 파일 재생: `ros2 bag play <YOUR_BAG>`
- Optical Flow 노드 실행
  ```bash
  source edu2_ws/install/setup.bash
  ros2 run move_turtle_pkg move_by_img
  ```

- 확인/디버깅
  - 토픽 확인: `ros2 topic list`
  - 이미지 타입: `ros2 interface show sensor_msgs/msg/Image`
  - 메시지 보기: `ros2 topic echo /camera/image_raw`
  - 그래프: `rqt_graph`

---

## 참고

- 슬라이드: `ROS 교육 PPT v1.pdf`, `ROS 교육 2차시 PPT.pdf`
- Maintainer: yoo <smzzang21@konkuk.ac.kr>

