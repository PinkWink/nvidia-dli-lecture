"""강의 54: 이름을 말하면 찾아서 다가갑니다. 지도도, 학습한 모델도 쓰지 않고 머리 카메라와 미리 학습된 모델만 씁니다.

    source /opt/ros/jazzy/setup.bash
    source ~/pinklab_robotworld2026/install/setup.bash
    ~/yolo_venv/bin/python 54_pl2m_models/find_go.py "a round white table"                 # 둥근 흰 테이블(상담 테이블)을 찾아 1 m 앞까지
    ~/yolo_venv/bin/python 54_pl2m_models/find_go.py "a round white table" --stop 1.2      # 1.2 m 앞에서 선다
    ~/yolo_venv/bin/python 54_pl2m_models/find_go.py "a round white table" --no-depth      # 깊이 센서 없이: 거리를 색 영상에서 어림한다
    ~/yolo_venv/bin/python 54_pl2m_models/find_go.py "a round white table" --look-only     # 움직이지 않고 지금 보이는지만
    ~/yolo_venv/bin/python 54_pl2m_models/find_go.py "a pink table"                        # 분홍 상판의 음료대: 엉뚱한 것을 찾아간다 (강의 54 의 5절)

하는 일 (보고 → 움직이고 → 다시 보기를 되풀이한다. 움직이는 동안에는 보지 않는다)
    SEARCH    서서 영상 한 장에서 이름으로 찾는다. 없으면 왼쪽으로 25° 돌고 다시 본다
    AIM       찾은 상자의 가운데가 화면 가운데에 오게 돈다
    APPROACH  거리를 재고, 남은 거리의 절반만큼(많아야 0.5 m) 나아간 뒤 다시 본다
장애물은 피하지 않는다. 가는 길에 무엇이 있으면 부딪힌다 (강의 44 의 Nav2 와 다른 점).
"""
import argparse
import math
import os
import sys

import numpy as np
import rclpy
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from try_models import HeadCamera

TURN_SPEED, DRIVE_SPEED = 0.35, 0.15            # 도는 속도 [rad/s], 나아가는 속도 [m/s]
SEARCH_TURN, SEARCH_LIMIT = math.radians(25.0), 15      # 못 찾으면 이만큼 돌고 다시 본다. 한 바퀴 넘게 돌아도 없으면 그만둔다
AIM_TOLERANCE = math.radians(3.0)               # 목표가 화면 가운데에서 이 각 안에 있으면 돌지 않는다
STEP_LIMIT = 0.5                                # 한 번에 나아가는 거리의 한계 [m]
NEAR = 20                                       # 상자 안의 거리들 가운데 가까운 쪽에서 이 % 자리의 값을 그 물체까지의 거리로 본다
# 채점에 쓰는 참값: 이름 → [(무엇, 가운데 (x, y), 가운데에서 가장자리까지 [m])]. 로봇의 프로그램은 이것을 쓰지 않는다
TARGETS = {"a round white table": [("상담 테이블 1", (6.28, -1.90), 0.30), ("상담 테이블 2", (6.38, -4.43), 0.30)],
           "a pink table": [("음료대", (7.65, -1.60), 0.21)]}


class Seeker(HeadCamera):
    """머리 카메라(강의 54 의 HeadCamera)에 바퀴를 더한 것."""

    def __init__(self):
        super().__init__()
        self.odom = None
        self.commands = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Odometry, "/odom", self.on_odom, 20)

    def on_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.odom = (p.x, p.y, math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)))

    def move(self, linear, angular, done):
        """done() 이 참이 될 때까지 그 속도로 움직이고 선다. 오도메트리로 잰다."""
        while not done():
            self.commands.publish(Twist(linear=Vector3(x=linear), angular=Vector3(z=angular)))
            rclpy.spin_once(self, timeout_sec=0.02)
        for _ in range(10):                                           # 멈춤을 몇 번 보내고 잠깐 기다린다 (서서 찍으려고)
            self.commands.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.05)
        end = self.get_clock().now().nanoseconds + int(0.6e9)
        while self.get_clock().now().nanoseconds < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_odom(self):
        while self.odom is None:                                      # 오도메트리가 올 때까지
            rclpy.spin_once(self, timeout_sec=0.1)

    def turn(self, angle):
        self.wait_odom()
        start = self.odom[2]
        turned = lambda: abs(math.atan2(math.sin(self.odom[2] - start), math.cos(self.odom[2] - start)))
        self.move(0.0, math.copysign(TURN_SPEED, angle), lambda: turned() >= abs(angle) - 0.02)

    def drive(self, distance):
        self.wait_odom()
        x, y = self.odom[:2]
        self.move(DRIVE_SPEED, 0.0, lambda: math.hypot(self.odom[0] - x, self.odom[1] - y) >= distance - 0.01)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("name", help='찾을 것의 이름 (영어로). 예: "a round white table"')
    parser.add_argument("--stop", type=float, default=1.0, help="이 거리 앞에서 선다 [m]")
    parser.add_argument("--score", type=float, default=0.25, help="이 점수보다 낮은 것은 찾은 것으로 치지 않는다")
    parser.add_argument("--no-depth", action="store_true", help="깊이 센서를 쓰지 않고, 거리를 색 영상에서 어림한다")
    parser.add_argument("--median", action="store_true", help="비교용: 거리를 상자 가운데 부분의 중앙값으로 잰다 (강의 47 의 컵처럼)")
    parser.add_argument("--look-only", action="store_true", help="움직이지 않고 지금 보이는지만 출력한다")
    args = parser.parse_args()
    global NEAR
    NEAR = 50 if args.median else NEAR
    rclpy.init()
    robot = Seeker()
    import models
    find = models.Find()
    guess = models.Distance() if args.no_depth else None
    now = lambda: robot.get_clock().now().nanoseconds * 1e-9

    def look():
        """서서 한 장 찍고 찾는다: (점수, 왼쪽으로 틀어진 각 [rad], 거리 [m]) 또는 None."""
        color, depth, matrix, _ = robot.frame()
        found = [row for row in find(color, [args.name], threshold=args.score)]
        if not found:
            return None
        _, score, (x1, y1, x2, y2) = found[0]
        u, w, h = (x1 + x2) / 2.0, x2 - x1, y2 - y1
        field = guess(color) if guess else depth
        patch = field[int(y1 + h / 4):int(y2 - h / 4) + 1, int(x1 + w / 4):int(x2 - w / 4) + 1]                # 상자의 가운데 부분
        patch = patch[np.isfinite(patch) & (patch > 0.05)]
        # 테이블처럼 속이 빈 물체는 상자 안에 뒤의 배경이 많이 섞인다. 중앙값(강의 47) 대신 가까운 쪽 20 % 자리의 값을 쓴다
        return score, math.atan2(matrix[0, 2] - u, matrix[0, 0]), float(np.percentile(patch, NEAR)) if patch.size else float("nan")

    started, turns = now(), 0
    print(f"찾을 것: {args.name!r}.  거리는 {'색 영상에서 어림 (Distance 모델)' if guess else '깊이 센서'}로 잼", flush=True)
    while True:
        seen = look()
        if seen is None:
            print(f"[{now() - started:5.1f}초] SEARCH    보이지 않음" + ("" if args.look_only else f": 왼쪽으로 {math.degrees(SEARCH_TURN):.0f}° 돎"), flush=True)
            turns += 1
            if args.look_only or turns > SEARCH_LIMIT:
                break
            robot.turn(SEARCH_TURN)
            continue
        score, bearing, distance = seen
        print(f"[{now() - started:5.1f}초] 보임      점수 {score:.2f} · 왼쪽으로 {math.degrees(bearing):+.1f}° · 거리 {distance:.2f} m", flush=True)
        if args.look_only:
            break
        if abs(bearing) > AIM_TOLERANCE:
            print(f"[{now() - started:5.1f}초] AIM       {math.degrees(bearing):+.1f}° 돎", flush=True)
            robot.turn(bearing)
            continue
        if not math.isfinite(distance) or distance <= args.stop + 0.03:
            print(f"[{now() - started:5.1f}초] 도착      {distance:.2f} m 앞에 섬", flush=True)
            break
        step = min((distance - args.stop) * (0.5 if distance - args.stop > 0.2 else 1.0), STEP_LIMIT)
        print(f"[{now() - started:5.1f}초] APPROACH  {step:.2f} m 나아감", flush=True)
        robot.drive(step)
    for _ in range(5):
        rclpy.spin_once(robot, timeout_sec=0.1)
    if robot.robot is not None and args.name in TARGETS:                  # 채점 (참값): 그 이름에 맞는 것 가운데 로봇이 보고 있는 쪽의 것까지
        x, y, yaw = robot.robot
        off = lambda point: abs(math.atan2(math.sin(math.atan2(point[1] - y, point[0] - x) - yaw), math.cos(math.atan2(point[1] - y, point[0] - x) - yaw)))
        label, point, radius = min(TARGETS[args.name], key=lambda target: off(target[1]))
        print(f"채점 (참값): 로봇 ({x:.2f}, {y:.2f}).  보고 있는 쪽의 {label}: 가장자리까지 {math.hypot(point[0] - x, point[1] - y) - radius:.2f} m,  그쪽에서 {math.degrees(off(point)):.1f}° 틀어져 있음", flush=True)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
