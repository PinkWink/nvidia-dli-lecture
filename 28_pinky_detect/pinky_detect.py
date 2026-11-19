#!/usr/bin/env python3
"""강의 28: 로봇 카메라 영상에서 다른 Pinky Pro 를 찾는 ROS 2 노드.

    source /opt/ros/jazzy/setup.bash
    ~/yolo_venv/bin/python 28_pinky_detect/pinky_detect.py
    ~/yolo_venv/bin/python 28_pinky_detect/pinky_detect.py --ros-args -p model:=/경로/best.pt -p confidence:=0.5

받는 것   /camera/image_raw   (sensor_msgs/Image)
내는 것   /pinky/debug        상자를 그린 영상 (rqt_image_view 로 본다)
          /pinky/nearest      가장 가까운(상자가 가장 큰) Pinky Pro: x = 화면 가로 위치 (-1 왼쪽 끝 ~ +1 오른쪽 끝),
                              y = 상자 높이 / 화면 높이, z = 믿는 정도. 아무것도 없으면 z = 0 (geometry_msgs/Point)

YOLO 를 설치한 가상환경의 파이썬으로 실행한다. ROS 2 를 먼저 source 하면 그 파이썬에서도 rclpy 를 쓸 수 있다.
"""
import os
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from sensor_msgs.msg import Image
from ultralytics import YOLO

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_MODEL = os.path.abspath(os.path.join(HERE, "..", "work", "pinky_yolo", "runs", "pinky", "weights", "best.pt"))


def to_bgr(msg):
    """sensor_msgs/Image → OpenCV 영상 (cv_bridge 없이)."""
    channels = 4 if msg.encoding in ("rgba8", "bgra8") else 3
    image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)[:, :msg.width * channels]
    image = image.reshape(msg.height, msg.width, channels)[:, :, :3]
    return image if msg.encoding.startswith("bgr") else image[:, :, ::-1]


class PinkyDetect(Node):
    def __init__(self):
        super().__init__("pinky_detect")
        model = self.declare_parameter("model", DEFAULT_MODEL).value
        self.confidence = self.declare_parameter("confidence", 0.5).value
        self.model = YOLO(model)
        self.model.predict(np.zeros((360, 640, 3), dtype=np.uint8), verbose=False)       # 첫 실행은 느리므로 미리 한 번
        self.debug = self.create_publisher(Image, "/pinky/debug", 2)
        self.nearest = self.create_publisher(Point, "/pinky/nearest", 2)
        self.create_subscription(Image, "/camera/image_raw", self.on_image, 2)
        self.frames, self.spent, self.last_report = 0, 0.0, time.monotonic()
        self.found = 0
        self.get_logger().info(f"모델: {model}")

    def on_image(self, msg):
        image = np.ascontiguousarray(to_bgr(msg))
        start = time.monotonic()
        result = self.model.predict(image, conf=self.confidence, verbose=False)[0]
        self.spent += time.monotonic() - start
        self.frames += 1
        height, width = image.shape[:2]
        best = Point()
        for (left, top, right, bottom), score in zip(result.boxes.xyxy.cpu().numpy(), result.boxes.conf.cpu().numpy()):
            cv2.rectangle(image, (int(left), int(top)), (int(right), int(bottom)), (160, 60, 255), 2)
            cv2.putText(image, f"pinky {score:.2f}", (int(left), max(12, int(top) - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (160, 60, 255), 1)
            size = (bottom - top) / height
            if size > best.y:
                best.x, best.y, best.z = float(((left + right) / 2 - width / 2) / (width / 2)), float(size), float(score)
        self.found = len(result.boxes)
        self.nearest.publish(best)
        out = Image(header=msg.header, height=height, width=width, encoding="bgr8", step=width * 3, data=image.tobytes())
        self.debug.publish(out)
        now = time.monotonic()
        if now - self.last_report >= 1.0:
            self.get_logger().info(f"Pinky Pro {self.found}대  (가장 가까운 것: 가로 {best.x:+.2f}, 높이 {best.y:.2f}, 믿는 정도 {best.z:.2f})  "
                                   f"1초 동안 {self.frames}장, 한 장 {1000 * self.spent / max(1, self.frames):.0f} ms")
            self.frames, self.spent, self.last_report = 0, 0.0, now


def main():
    rclpy.init()
    node = PinkyDetect()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
