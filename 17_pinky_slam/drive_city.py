"""강의 17: 핑크시티를 자동으로 한 바퀴 도는 운전 노드 (지도 만들 때 손으로 몰기 번거로우면 사용).

    source /opt/ros/jazzy/setup.bash
    python3 ~/nvidia-dli-lecture/17_pinky_slam/drive_city.py

/odom 으로 지금 위치를 보고, 정해 둔 점들을 차례로 향해 /cmd_vel 을 보냅니다.
길: 순환 도로 한 바퀴 → 가로 도로 → 세로 도로 → 출발점. 약 100초 걸립니다.
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

START = (-1.05, -0.5)          # 로봇의 시작 위치 (도시 좌표). 시작 방향은 +Y — pink_city.py 의 ROBOT_START 와 같아야 한다
R = 1.05                       # 순환 도로 중심선
# 지나갈 점 (도시 좌표)
CITY_POINTS = [(-R, 0.75), (-0.75, R), (0.75, R), (R, 0.75), (R, -0.75), (0.75, -R), (-0.75, -R), (-R, -0.75), (-R, -0.1),   # 순환 도로
               (-0.9, 0.0), (0.9, 0.0), (R, 0.2), (R, 0.75), (0.75, R), (0.2, R),                                            # 가로 도로
               (0.0, 0.85), (0.0, -0.85), (-0.2, -R), (-0.75, -R), (-R, -0.75), (-R, -0.5)]                                  # 세로 도로 → 출발점
# /odom 은 "시작 위치가 원점, 시작할 때 보던 쪽이 +x" 인 좌표다. 도시 좌표의 점을 그 좌표로 바꾼다.
POINTS = [(y - START[1], -(x - START[0])) for x, y in CITY_POINTS]
SPEED = 0.18                   # 직진 속도 [m/s]
REACHED = 0.08                 # 이 거리 안에 들어오면 다음 점으로 [m]


class DriveCity(Node):
    def __init__(self):
        super().__init__("drive_city")
        self.index = 0
        self.done = False
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)

    def on_odom(self, message):
        command = Twist()
        if self.index >= len(POINTS):
            self.done = True
        else:
            p, q = message.pose.pose.position, message.pose.pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            dx, dy = POINTS[self.index][0] - p.x, POINTS[self.index][1] - p.y
            if math.hypot(dx, dy) < REACHED:
                self.index += 1
                print(f"점 {self.index} / {len(POINTS)} 통과", flush=True)
            else:
                error = (math.atan2(dy, dx) - yaw + math.pi) % (2 * math.pi) - math.pi      # 목표 방향과의 차이
                command.angular.z = max(-1.5, min(1.5, 2.5 * error))
                command.linear.x = SPEED * max(0.0, 1 - abs(error) / 0.6)                   # 많이 틀어져 있으면 먼저 제자리에서 돈다
        self.publisher.publish(command)


def main():
    rclpy.init()
    node = DriveCity()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass
    for _ in range(10):                                   # 멈춤 명령
        node.publisher.publish(Twist())
        rclpy.spin_once(node, timeout_sec=0.05)
    print("도착. 로봇을 세웠습니다.", flush=True)


if __name__ == "__main__":
    main()
