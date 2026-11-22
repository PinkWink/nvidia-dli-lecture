"""강의 47: 카메라로 구한 컵의 자리(/vision/detections)를 시뮬레이터의 참값과 견줍니다.

    source /opt/ros/jazzy/setup.bash
    source ~/pinklab_robotworld2026/install/setup.bash
    python3 47_pl2m_vision/check_cups.py                    # 영상 20장을 모아 컵마다 평균 오차를 출력
    python3 47_pl2m_vision/check_cups.py --frames 50 --json result.json

참값은 /cups/truth (컵, 부스 좌표) 와 /ground_truth (로봇, 부스 좌표) 에서 읽어 로봇 기준으로 바꾼다. 채점에만 쓴다.
"""
import argparse
import json
import math

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from tf2_msgs.msg import TFMessage
from warehouse_interfaces.msg import Detection3DArray


class Checker(Node):
    def __init__(self):
        super().__init__("check_cups", parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.robot, self.cups, self.rows = None, {}, []
        self.create_subscription(Odometry, "/ground_truth", self.on_robot, 50)
        self.create_subscription(TFMessage, "/cups/truth", self.on_cups, 50)
        self.create_subscription(Detection3DArray, "/vision/detections", self.on_detections, 50)

    def on_robot(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.robot = (p.x, p.y, math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)))

    def on_cups(self, msg):
        for item in msg.transforms:
            t = item.transform.translation
            self.cups[item.child_frame_id] = (t.x, t.y, t.z)

    def truth(self, name):
        """컵의 참값을 로봇 기준 (앞, 왼쪽, 높이) 으로."""
        (x, y, z), (rx, ry, yaw) = self.cups[name], self.robot
        dx, dy = x - rx, y - ry
        return np.array([dx * math.cos(yaw) + dy * math.sin(yaw), -dx * math.sin(yaw) + dy * math.cos(yaw), z])

    def on_detections(self, msg):
        if self.robot is None or not self.cups:
            return
        row = {}
        for item in msg.detections:
            if item.has_position and item.label in self.cups:
                p = item.position.point
                row[item.label] = (item.score, np.array([p.x, p.y, p.z]) - self.truth(item.label))
        self.rows.append(row)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--frames", type=int, default=20, help="모을 영상 수")
    parser.add_argument("--json", metavar="FILE", help="결과를 이 파일에 저장한다")
    args = parser.parse_args()
    rclpy.init()
    node = Checker()
    while rclpy.ok() and len(node.rows) < args.frames:
        rclpy.spin_once(node, timeout_sec=0.2)
    print(f"영상 {len(node.rows)}장.  오차 = 카메라로 구한 자리 - 참값 [mm] (로봇 기준: 앞 · 왼쪽 · 높이)")
    print(f"{'컵':<12}{'찾은 횟수':>8}{'점수':>7}{'참값: 앞':>10}{'왼쪽':>8}{'오차: 앞':>10}{'왼쪽':>7}{'높이':>7}{'수평 거리':>9}")
    result = {}
    for name in sorted(node.cups):
        hits = [row[name] for row in node.rows if name in row]
        truth = node.truth(name)
        if not hits:
            print(f"{name:<12}{0:>10}        {truth[0]:>10.3f}{truth[1]:>+9.3f}   찾지 못함")
            result[name] = {"found": 0, "truth": truth.tolist()}
            continue
        error = np.mean([e for _, e in hits], axis=0) * 1000.0
        score = float(np.mean([s for s, _ in hits]))
        print(f"{name:<12}{len(hits):>10}{score:>9.2f}{truth[0]:>10.3f}{truth[1]:>+9.3f}{error[0]:>+11.1f}{error[1]:>+8.1f}{error[2]:>+8.1f}{math.hypot(error[0], error[1]):>10.1f}")
        result[name] = {"found": len(hits), "score": score, "truth": truth.tolist(), "error_mm": error.tolist()}
    if args.json:
        with open(args.json, "w") as file:
            json.dump({"frames": len(node.rows), "cups": result}, file, indent=1)
        print("저장:", args.json)


if __name__ == "__main__":
    main()
