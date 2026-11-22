"""강의 54: PL-2M 의 머리 카메라 영상 한 장에 AI 모델 넷을 돌려 보고, 시뮬레이터의 참값과 견줍니다.

    source /opt/ros/jazzy/setup.bash
    source ~/pinklab_robotworld2026/install/setup.bash
    ~/yolo_venv/bin/python 54_pl2m_models/try_models.py                          # 넷 다
    ~/yolo_venv/bin/python 54_pl2m_models/try_models.py --only find distance     # 고른 것만 (ask · segment · distance · find)
    ~/yolo_venv/bin/python 54_pl2m_models/try_models.py --ask "Is the table pink? Answer yes or no."
    ~/yolo_venv/bin/python 54_pl2m_models/try_models.py --only find --find "a pink table" "a person"     # 아무 이름이나 찾아 보기
    ~/yolo_venv/bin/python 54_pl2m_models/try_models.py --save models.png        # 결과를 그림 한 장으로

시뮬레이터(강의 47 의 pl2m_vision.py)와 robot_state_publisher(강의 45 의 dock.launch.py)가 떠 있어야 한다.
모델이 쓰는 것은 색 영상 한 장뿐이다. 깊이 영상과 참값은 모델의 답을 채점하는 데만 쓴다.
"""
import argparse
import math
import os
import sys

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "47_pl2m_vision"))
from cup_detector import matrix_from                    # 강의 47: /tf 의 변환 → 4 × 4 행렬
from locate import cup_center                           # 강의 47: 상자 + 깊이 → 로봇 기준의 자리

CUPS = {"cup_pink": "a pink cup", "cup_blue": "a blue cup", "cup_yellow": "a yellow cup"}       # 컵의 이름 → 찾을 때 적는 말
QUESTIONS = ["How many cups are on the pink table? Answer with a number.",
             "What colors are the cups on the table, from left to right?",
             "Is there a person in the image? Answer yes or no."]


class HeadCamera(Node):
    """머리 카메라의 영상 한 쌍(색 · 깊이)과, 채점에 쓸 참값을 받는다."""

    def __init__(self):
        super().__init__("try_models", parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.matrix = self.robot = None
        self.cups, self.waiting, self.pairs = {}, {}, []
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.create_subscription(CameraInfo, "/head_camera/camera_info", lambda msg: setattr(self, "matrix", np.array(msg.k).reshape(3, 3)), qos_profile_sensor_data)
        self.create_subscription(Image, "/head_camera/image_raw", lambda msg: self.pair(msg, "color"), qos_profile_sensor_data)
        self.create_subscription(Image, "/head_camera/depth", lambda msg: self.pair(msg, "depth"), qos_profile_sensor_data)
        self.create_subscription(Odometry, "/ground_truth", self.on_robot, 10)
        self.create_subscription(TFMessage, "/cups/truth", self.on_cups, 10)

    def pair(self, msg, kind):
        """색과 깊이는 따로 온다. 시각이 같은 둘이 모이면 한 쌍이다 (강의 47)."""
        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        self.waiting.setdefault(stamp, {})[kind] = msg
        if len(self.waiting[stamp]) == 2:
            both = self.waiting.pop(stamp)
            color = np.frombuffer(both["color"].data, np.uint8).reshape(msg.height, msg.width, -1)[:, :, :3].copy()
            self.pairs.append((color, np.frombuffer(both["depth"].data, np.float32).reshape(msg.height, msg.width).copy()))

    def on_robot(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.robot = (p.x, p.y, math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)))

    def on_cups(self, msg):
        for item in msg.transforms:
            self.cups[item.child_frame_id] = (item.transform.translation.x, item.transform.translation.y, item.transform.translation.z)

    def frame(self):
        """새로 찍은 한 쌍과 카메라가 달린 자리: (색, 깊이, 렌즈 값, base_from_camera)."""
        self.pairs.clear()
        while not self.pairs or self.matrix is None or not self.tf.can_transform("base_footprint", "head_camera_optical_frame", Time()):
            rclpy.spin_once(self, timeout_sec=0.2)
        color, depth = self.pairs[-1]
        return color, depth, self.matrix, matrix_from(self.tf.lookup_transform("base_footprint", "head_camera_optical_frame", Time()).transform)

    def truth(self, name):
        """컵의 참값을 로봇 기준 (앞, 왼쪽, 높이) 으로. 참값이 아직 오지 않았으면 None."""
        if self.robot is None or name not in self.cups:
            return None
        (x, y, z), (rx, ry, yaw) = self.cups[name], self.robot
        dx, dy = x - rx, y - ry
        return np.array([dx * math.cos(yaw) + dy * math.sin(yaw), -dx * math.sin(yaw) + dy * math.cos(yaw), z])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--only", nargs="+", choices=["ask", "segment", "distance", "find"], default=["ask", "segment", "distance", "find"])
    parser.add_argument("--ask", nargs="+", metavar="QUESTION", default=QUESTIONS, help="물어볼 말 (영어로)")
    parser.add_argument("--find", nargs="+", metavar="NAME", help='컵 대신 찾을 것들의 이름 (영어로). 예: "a pink table" "a person"')
    parser.add_argument("--save", metavar="PNG", help="결과를 그림 한 장으로 저장한다")
    args = parser.parse_args()
    rclpy.init()
    camera = HeadCamera()
    color, depth, matrix, base_from_camera = camera.frame()
    for _ in range(10):                                           # 참값이 올 틈을 준다
        rclpy.spin_once(camera, timeout_sec=0.1)
    import models                                                 # 여기서 PyTorch 와 transformers 를 읽는다 (몇 초 걸린다)
    shown = {"color": color, "depth": depth}

    if "find" in args.only:
        find = models.Find()
        found = find(color, args.find or list(CUPS.values()))
        print(f"\n[이름으로 찾기] 올리는 데 {find.loaded:.1f}초, 한 장에 {find.took * 1000:.0f} ms.  찾은 것 {len(found)}개", flush=True)
        shown["found"] = found
        for words, score, (x1, y1, x2, y2) in found if args.find else []:
            print(f"  {words:<22} 점수 {score:.2f}  상자 ({x1:.0f}, {y1:.0f}) ~ ({x2:.0f}, {y2:.0f})", flush=True)
        for name, words in ({} if args.find else CUPS).items():
            best = next((row for row in found if row[0] == words), None)
            if best is None:
                print(f"  {words:<14} 찾지 못함", flush=True)
                continue
            place, actual = cup_center(best[2], depth, matrix, base_from_camera), camera.truth(name)
            text = f"  {words:<14} 점수 {best[1]:.2f}  상자 ({best[2][0]:.0f}, {best[2][1]:.0f}) ~ ({best[2][2]:.0f}, {best[2][3]:.0f})"
            if place is not None:
                text += f"  →  로봇 기준 앞 {place[0]:.3f} · 왼쪽 {place[1]:+.3f} m"
                if actual is not None:
                    text += f"  (참값과 {np.hypot(*(place - actual)[:2]) * 1000:.1f} mm)"
            print(text, flush=True)

    if "distance" in args.only:
        distance = models.Distance()
        guessed = distance(color)
        valid = np.isfinite(depth) & (depth > 0.1) & (depth < 10.0)
        ratio = np.abs(guessed[valid] - depth[valid]) / depth[valid]
        print(f"\n[영상 한 장으로 거리 어림하기] 올리는 데 {distance.loaded:.1f}초, 한 장에 {distance.took * 1000:.0f} ms", flush=True)
        print(f"  깊이 영상과 견줌: 오차의 중앙값 {np.median(ratio) * 100:.0f} %, 10 % 안에 든 픽셀 {(ratio < 0.1).mean() * 100:.0f} %", flush=True)
        shown["guessed"] = guessed
        for name, words in CUPS.items():
            best = next((row for row in shown.get("found", []) if row[0] == words), None)
            if best is not None:
                u, v = int((best[2][0] + best[2][2]) / 2), int((best[2][1] + best[2][3]) / 2)
                print(f"  {words:<14} 까지: 깊이 영상 {depth[v, u]:.3f} m,  어림 {guessed[v, u]:.3f} m  ({(guessed[v, u] - depth[v, u]) * 100:+.1f} cm)", flush=True)

    if "segment" in args.only:
        segment = models.Segment()
        kinds = segment(color)
        print(f"\n[픽셀마다 무엇인지] 올리는 데 {segment.loaded:.1f}초, 한 장에 {segment.took * 1000:.0f} ms", flush=True)
        print("  " + ",  ".join(f"{name} {share * 100:.0f} %" for name, share in kinds[:7]), flush=True)
        shown["mask"] = segment.mask

    if "ask" in args.only:
        ask = models.Ask()
        print(f"\n[말로 묻기] 올리는 데 {ask.loaded:.1f}초", flush=True)
        for question in args.ask:
            answer = ask(color, question)
            print(f"  Q: {question}\n  A: {' '.join(answer.split())}   ({ask.took:.1f}초)", flush=True)

    if args.save:
        save_picture(shown, args.save)
    rclpy.shutdown()


def save_picture(shown, path):
    """색 영상 + 찾은 상자, 무엇인지(마스크), 깊이 영상, 어림한 거리를 한 장에."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    panels = [("color", "color + boxes (Find)"), ("mask", "what is it (Segment)"), ("depth", "depth sensor [m]"), ("guessed", "guessed distance [m] (Distance)")]
    panels = [(key, title) for key, title in panels if key in shown]
    figure, axes = plt.subplots(1, len(panels), figsize=(5.2 * len(panels), 4.2), dpi=110)
    for ax, (key, title) in zip(np.atleast_1d(axes), panels):
        if key == "color":
            ax.imshow(shown["color"])
            for name, score, (x1, y1, x2, y2) in shown.get("found", []):
                ax.add_patch(plt.Rectangle((x1, y1), x2 - x1, y2 - y1, fill=False, ec="white", lw=1.5))
                ax.text(x1, y1 - 4, f"{name} {score:.2f}", color="white", fontsize=7)
        elif key == "mask":
            ax.imshow(shown["mask"], cmap="tab20")
        else:
            ax.imshow(np.clip(shown[key], 0.3, 3.0), cmap="viridis_r", vmin=0.3, vmax=3.0)
        ax.set_title(title, fontsize=10)
        ax.axis("off")
    figure.tight_layout()
    figure.savefig(path)
    print("\n저장:", path, flush=True)


if __name__ == "__main__":
    main()
