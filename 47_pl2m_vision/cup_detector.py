"""강의 47: 머리 카메라의 영상에서 컵을 찾고(YOLO), 깊이로 로봇 기준의 자리를 구해 /vision/detections 로 냅니다.

    source /opt/ros/jazzy/setup.bash
    source ~/pinklab_robotworld2026/install/setup.bash            # 메시지 형식 warehouse_interfaces/Detection3DArray
    ~/yolo_venv/bin/python 47_pl2m_vision/cup_detector.py                     # 찾는 대로 낸다
    ~/yolo_venv/bin/python 47_pl2m_vision/cup_detector.py --print             # 1초마다 찾은 컵을 출력
    ~/yolo_venv/bin/python 47_pl2m_vision/cup_detector.py --save cups.png     # 찾은 컵에 상자를 그린 영상 한 장을 저장
    ~/yolo_venv/bin/python 47_pl2m_vision/cup_detector.py --window            # 카메라 영상을 창으로 보기
    ~/yolo_venv/bin/python 47_pl2m_vision/cup_detector.py --timing            # 한 장을 처리하는 시간을 단계별로 출력 (강의 49)
    ~/yolo_venv/bin/python 47_pl2m_vision/cup_detector.py --surface           # 비교용: 반지름만큼 더 들어가지 않고 보이는 면의 점을 낸다

YOLO 가 설치된 가상환경의 파이썬(강의 28)으로 실행하고, ROS 2 는 그 터미널에서 source 해 둔다.
쓰는 것은 색 영상, 깊이 영상, 렌즈 값(camera_info), 카메라가 로봇에 달린 자리(/tf) 뿐이다. 시뮬레이터의 참값은 쓰지 않는다.
    색 영상 → 상자와 이름         YOLO (make_dataset.py 의 데이터로 학습한 것)
    상자 + 깊이 → 카메라 기준의 점  locate.py
    카메라 기준 → 로봇 기준        /tf 의 base_footprint → head_camera_optical_frame
"""
import argparse
import os
import sys
import time

import cv2
import numpy as np
import rclpy
import rclpy.executors
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from warehouse_interfaces.msg import Detection3D, Detection3DArray

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from locate import CUP_RADIUS, cup_center

BASE, OPTICAL = "base_footprint", "head_camera_optical_frame"
WEIGHTS = os.path.join(HERE, "..", "work", "cup_train", "cups", "weights", "best.pt")
COLORS = {"cup_pink": (200, 90, 255), "cup_blue": (255, 160, 60), "cup_yellow": (60, 220, 255)}       # 그릴 때의 색 (B, G, R)


def matrix_from(transform):
    """/tf 의 변환 → 4 × 4 행렬."""
    q, t = transform.rotation, transform.translation
    x, y, z, w = q.x, q.y, q.z, q.w
    matrix = np.eye(4)
    matrix[:3, :3] = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                      [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                      [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
    matrix[:3, 3] = [t.x, t.y, t.z]
    return matrix


class CupDetector(Node):
    def __init__(self, weights, push=CUP_RADIUS, confidence=0.5, device=None, show=False, save=None, window=False, timing=False):
        super().__init__("cup_detector", parameter_overrides=[Parameter("use_sim_time", value=True)])
        from ultralytics import YOLO
        self.model, self.confidence, self.device = YOLO(weights, task="detect"), confidence, device
        self.model.predict(np.zeros((480, 640, 3), np.uint8), device=device, verbose=False)      # 처음 한 번은 오래 걸리므로 미리 돌려 둔다
        self.push, self.show, self.save, self.window = push, show, save, window
        self.matrix = self.base_from_camera = None
        self.waiting = {}                                                      # 시각 → 그 시각의 색 영상 · 깊이 영상
        self.frames, self.spent, self.shown, self.found = 0, 0.0, None, []
        self.timing, self.stages = timing, np.zeros(6)                         # 단계별로 쓴 시간의 합 [s]
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.publisher = self.create_publisher(Detection3DArray, "/vision/detections", 10)
        self.picture = self.create_publisher(Image, "/vision/cups_image", 2)   # 찾은 컵에 상자를 그린 영상 (rqt_image_view 로 본다)
        self.create_subscription(CameraInfo, "/head_camera/camera_info", self.on_info, qos_profile_sensor_data)
        self.create_subscription(Image, "/head_camera/depth", self.on_depth, qos_profile_sensor_data)
        self.create_subscription(Image, "/head_camera/image_raw", self.on_image, qos_profile_sensor_data)

    def on_info(self, msg):
        self.matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)          # 렌즈 값: 초점 거리와 영상의 중심 [픽셀]

    def on_depth(self, msg):
        self.pair((msg.header.stamp.sec, msg.header.stamp.nanosec), depth=np.frombuffer(msg.data, np.float32).reshape(msg.height, msg.width))

    def on_image(self, msg):
        self.pair((msg.header.stamp.sec, msg.header.stamp.nanosec), color=msg)

    def pair(self, stamp, **part):
        """색과 깊이는 따로 온다. 시각이 같은 둘이 모이면 그때 처리한다 (어느 쪽이 먼저 올지는 정해져 있지 않다)."""
        self.waiting.setdefault(stamp, {}).update(part)
        for old in sorted(self.waiting)[:-5]:                                  # 짝을 못 찾은 것은 최근 다섯 장만 둔다
            del self.waiting[old]
        if len(self.waiting[stamp]) == 2:
            both = self.waiting.pop(stamp)
            self.detect(both["color"], both["depth"])

    def detect(self, msg, depth):
        if self.matrix is None:
            return
        if self.base_from_camera is None:                                      # 카메라가 달린 자리는 바뀌지 않으므로 한 번만 읽는다
            if not self.tf.can_transform(BASE, OPTICAL, Time()):
                return
            self.base_from_camera = matrix_from(self.tf.lookup_transform(BASE, OPTICAL, Time()).transform)
        started = time.monotonic()
        image = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, -1)[:, :, :3]
        picture = np.ascontiguousarray(image[:, :, ::-1])                      # YOLO 와 OpenCV 는 B, G, R 순서로 받는다
        converted = time.monotonic()
        result = self.model.predict(picture, conf=self.confidence, device=self.device, verbose=False)[0]
        predicted = time.monotonic()
        out = Detection3DArray()
        out.header.stamp, out.header.frame_id = msg.header.stamp, BASE         # 시각은 영상을 찍은 때
        self.found = []
        for box, number, score in zip(result.boxes.xyxy.cpu().numpy(), result.boxes.cls.int().tolist(), result.boxes.conf.tolist()):
            item = Detection3D(label=self.model.names[number], score=float(score), track_id=-1, camera="head_camera")
            item.bbox_xyxy = [float(v) for v in box]
            center = cup_center(box, depth, self.matrix, self.base_from_camera, self.push)
            if center is not None:
                item.has_position = True
                item.position = PointStamped()
                item.position.header.stamp, item.position.header.frame_id = msg.header.stamp, BASE
                item.position.point.x, item.position.point.y, item.position.point.z = (float(v) for v in center)
            out.detections.append(item)
            self.found.append((item.label, float(score), box, center))
        self.publisher.publish(out)
        self.frames, self.spent = self.frames + 1, self.spent + time.monotonic() - started
        speed = result.speed                                                   # YOLO 가 잰 것 [ms]: 전처리 · 추론 · 후처리
        self.stages += [converted - started, speed["preprocess"] / 1000.0, speed["inference"] / 1000.0, speed["postprocess"] / 1000.0,
                        predicted - converted, time.monotonic() - predicted]
        watched = self.picture.get_subscription_count() > 0                    # 보는 쪽이 있을 때만 그려서 낸다
        if self.window or self.save or watched:
            for label, score, box, center in self.found:
                x1, y1, x2, y2 = (int(v) for v in box)
                cv2.rectangle(picture, (x1, y1), (x2, y2), COLORS.get(label, (255, 255, 255)), 2)
                color = COLORS.get(label, (255, 255, 255))
                cv2.putText(picture, f"{label.replace('cup_', '')} {score:.2f}", (x1 - 12, y1 - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)
                if center is not None:                                         # 상자 아래에 로봇 기준의 자리 (앞, 왼쪽) [m]
                    cv2.putText(picture, f"{center[0]:.2f}, {center[1]:+.2f}", (x1 - 22, y2 + 16), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)
            if watched:
                self.picture.publish(Image(header=msg.header, height=msg.height, width=msg.width, encoding="bgr8", step=3 * msg.width, data=picture.tobytes()))
            if self.window:
                cv2.imshow("head_camera", picture)
                cv2.waitKey(1)
            if self.save and self.found:
                cv2.imwrite(self.save, picture)
                print(f"저장: {self.save} (컵 {len(self.found)}개)", flush=True)
                self.save = None
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.show and (self.shown is None or now - self.shown >= 1.0):
            found = ",  ".join(f"{label} {score:.2f} 앞 {center[0]:.3f} · 왼쪽 {center[1]:+.3f} · 높이 {center[2]:.3f} m" if center is not None else f"{label} {score:.2f} (깊이 없음)"
                               for label, score, _, center in sorted(self.found, key=lambda item: item[0])) or "컵 없음"
            print(f"[{now:7.1f}초] 영상 {self.frames}장, 한 장에 {self.spent / self.frames * 1000:.0f} ms:  {found}", flush=True)
            if self.timing:
                convert, before, inference, after, predict, rest = self.stages / self.frames * 1000.0
                print(f"            영상 변환 {convert:.1f} ms · YOLO {predict:.1f} ms (전처리 {before:.1f} · 추론 {inference:.1f} · 후처리 {after:.1f} · 그 밖 {predict - before - inference - after:.1f}) · "
                      f"자리 구하기와 내보내기 {rest:.1f} ms", flush=True)
            self.shown, self.frames, self.spent, self.stages = now, 0, 0.0, np.zeros(6)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--weights", default=WEIGHTS, help="학습한 모델 파일")
    parser.add_argument("--surface", action="store_true", help="비교용: 보이는 면의 점을 그대로 낸다 (반지름만큼 더 들어가지 않는다)")
    parser.add_argument("--confidence", type=float, default=0.5, help="이 점수보다 낮은 것은 버린다")
    parser.add_argument("--device", help="YOLO 를 돌릴 곳: cpu 또는 0 (GPU). 주지 않으면 GPU 가 있을 때 GPU")
    parser.add_argument("--print", action="store_true", dest="show", help="1초마다 찾은 컵을 출력")
    parser.add_argument("--timing", action="store_true", help="한 장을 처리하는 시간을 단계별로 출력 (--print 와 함께 나온다)")
    parser.add_argument("--save", metavar="PNG", help="찾은 컵에 상자를 그린 영상 한 장을 저장")
    parser.add_argument("--window", action="store_true", help="카메라 영상을 창으로 띄운다")
    args = parser.parse_args()
    rclpy.init()
    node = CupDetector(os.path.abspath(args.weights), 0.0 if args.surface else CUP_RADIUS, args.confidence, args.device, args.show or args.timing, args.save, args.window, args.timing)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException, rclpy._rclpy_pybind11.RCLError):
        pass                                             # Ctrl+C 로 끝낼 때


if __name__ == "__main__":
    main()
