#!/usr/bin/env python3
"""강의 25: 영상 분할 결과를 읽어 "사람이 화면의 몇 % 인지" 를 알려 주는 ROS 2 노드.

Isaac ROS 컨테이너 안에서 실행한다.
    python3 person_watch.py

/unet/raw_segmentation_mask 는 화소마다 종류 번호가 든 영상이다 (PeopleSemSegNet 은 0 = 그 밖, 1 = 사람).
1초에 한 번, 사람 화소의 비율 · 사람이 있는 가로 범위 · 받은 영상 수를 출력한다.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

PERSON = 1          # PeopleSemSegNet 의 사람 종류 번호


class PersonWatch(Node):
    def __init__(self):
        super().__init__("person_watch")
        self.count = 0
        self.last = None
        self.create_subscription(Image, "/unet/raw_segmentation_mask", self.on_mask, 10)
        self.create_timer(1.0, self.report)

    def on_mask(self, msg):
        self.count += 1
        self.last = msg

    def report(self):
        if self.last is None:
            self.get_logger().info("분할 결과를 기다리는 중...")
            return
        msg = self.last
        mask = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)[:, :msg.width]
        person = mask == PERSON
        ratio = 100.0 * person.mean()
        columns = np.flatnonzero(person.any(axis=0))
        where = f"가로 {columns[0]} ~ {columns[-1]} 화소" if len(columns) else "없음"
        self.get_logger().info(f"사람 {ratio:5.1f} %  ({where}, 영상 {msg.width} x {msg.height})  1초 동안 {self.count}장")
        self.count = 0


def main():
    rclpy.init()
    node = PersonWatch()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
