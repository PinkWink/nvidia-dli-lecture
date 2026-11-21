"""강의 45: 앞 카메라 영상에서 ArUco 마커를 찾아, 로봇 기준의 위치를 /vision/markers 로 냅니다.

    source /opt/ros/jazzy/setup.bash
    source ~/pinklab_robotworld2026/install/setup.bash            # 메시지 형식 warehouse_interfaces/ArucoMarkerArray
    python3 45_pl2m_dock/aruco_node.py                            # 찾는 대로 낸다
    python3 45_pl2m_dock/aruco_node.py --print                    # 1초마다 찾은 마커를 출력
    python3 45_pl2m_dock/aruco_node.py --save markers.png         # 마커에 테두리를 그린 영상 한 장을 저장
    python3 45_pl2m_dock/aruco_node.py --window                   # 카메라 영상을 창으로 보기 (찾은 마커에 테두리)

쓰는 것은 영상, 렌즈 값(camera_info), 카메라가 로봇에 달린 자리(/tf) 뿐이다. 시뮬레이터의 참값은 쓰지 않으므로 실제 로봇에서도 같은 코드가 돈다.
    영상에서 네 꼭짓점 찾기      cv2.aruco
    꼭짓점 → 카메라 기준 자세    cv2.solvePnP (마커의 한 변이 0.15 m 라는 것을 안다)
    카메라 기준 → 로봇 기준      /tf 의 base_footprint → base_camera_optical_frame
"""
import argparse
import math
import os
import sys

import cv2
import numpy as np
import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from warehouse_interfaces.msg import ArucoMarker, ArucoMarkerArray

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from markers import MARKER_SIZE, MARKERS, aruco_dictionary

BASE, OPTICAL = "base_footprint", "base_camera_optical_frame"


def make_detector():
    """영상 → (꼭짓점들, id 들). OpenCV 4.7 부터 쓰는 방법이 바뀌어 둘 다 받는다."""
    dictionary = aruco_dictionary()
    if hasattr(cv2.aruco, "ArucoDetector"):
        detector = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())
        return lambda gray: detector.detectMarkers(gray)[:2]
    parameters = cv2.aruco.DetectorParameters_create()
    return lambda gray: cv2.aruco.detectMarkers(gray, dictionary, parameters=parameters)[:2]


def marker_pose(corners, matrix):
    """네 꼭짓점(픽셀) → 카메라(optical) 기준의 (위치, 회전 행렬). 마커의 z 축이 마커가 바라보는 쪽이다."""
    half = MARKER_SIZE / 2.0
    square = np.array([[-half, half, 0], [half, half, 0], [half, -half, 0], [-half, -half, 0]], np.float32)   # 왼쪽 위부터 시계 방향
    ok, rotation, position = cv2.solvePnP(square, corners, matrix, None, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    return (position.ravel(), cv2.Rodrigues(rotation)[0]) if ok else None


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


class MarkerDetector(Node):
    def __init__(self, show=False, save=None, window=False):
        super().__init__("aruco_node", parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.show, self.save, self.window = show, save, window
        self.detect = make_detector()
        self.matrix = self.base_from_camera = None
        self.frames, self.latency, self.seen, self.shown = 0, 0.0, {}, None
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.publisher = self.create_publisher(ArucoMarkerArray, "/vision/markers", 10)
        self.create_subscription(CameraInfo, "/base_camera/camera_info", self.on_info, qos_profile_sensor_data)
        self.create_subscription(Image, "/base_camera/image_raw", self.on_image, qos_profile_sensor_data)

    def on_info(self, msg):
        self.matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)          # 렌즈 값: 초점 거리와 영상의 중심 [픽셀]

    def on_image(self, msg):
        if self.matrix is None:
            return
        if self.base_from_camera is None:                                      # 카메라가 달린 자리는 바뀌지 않으므로 한 번만 읽는다
            if not self.tf.can_transform(BASE, OPTICAL, Time()):
                return
            self.base_from_camera = matrix_from(self.tf.lookup_transform(BASE, OPTICAL, Time()).transform)
        image = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, -1)[:, :, :3]
        corners, ids = self.detect(cv2.cvtColor(image, cv2.COLOR_RGB2GRAY))
        out = ArucoMarkerArray()
        out.header.stamp, out.header.frame_id, out.camera = msg.header.stamp, BASE, "base_camera"     # 시각은 영상을 찍은 때
        for quad, marker in zip(corners, ids.ravel() if ids is not None else []):
            pose = marker_pose(quad[0], self.matrix) if int(marker) in MARKERS else None
            if pose is None:
                continue
            position = (self.base_from_camera @ np.r_[pose[0], 1.0])[:3]
            normal = (self.base_from_camera[:3, :3] @ pose[1])[:, 2]           # 마커가 바라보는 쪽 (로봇 기준)
            item = ArucoMarker(id=int(marker), size=MARKER_SIZE, distance=float(np.linalg.norm(pose[0])))
            item.pose.position.x, item.pose.position.y, item.pose.position.z = (float(v) for v in position)
            item.normal_yaw = math.atan2(normal[1], normal[0])
            item.corners = [float(v) for v in quad.reshape(-1)]
            out.markers.append(item)
        self.publisher.publish(out)
        now = self.get_clock().now().nanoseconds * 1e-9
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.frames, self.latency = self.frames + 1, self.latency + max(0.0, now - stamp)
        self.seen = {m.id: m for m in out.markers}
        if self.window:
            picture = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            cv2.aruco.drawDetectedMarkers(picture, corners, ids)
            cv2.imshow("base_camera", picture)
            cv2.waitKey(1)
        if self.save and out.markers:
            picture = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            cv2.aruco.drawDetectedMarkers(picture, corners, ids)
            cv2.imwrite(self.save, picture)
            print(f"저장: {self.save} (마커 {sorted(self.seen)})", flush=True)
            self.save = None
        if self.show and (self.shown is None or now - self.shown >= 1.0):
            found = ",  ".join(f"{m.id}번 앞 {m.pose.position.x:.3f} · 왼쪽 {m.pose.position.y:+.3f} · 높이 {m.pose.position.z:.3f} m, 법선 {math.degrees(m.normal_yaw):+.1f}°"
                               for m in sorted(out.markers, key=lambda m: m.id)) or "마커 없음"
            print(f"[{now:7.1f}초] 영상 {self.frames}장, 늦음 {self.latency / self.frames * 1000:.0f} ms:  {found}", flush=True)
            self.shown, self.frames, self.latency = now, 0, 0.0


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--print", action="store_true", dest="show", help="1초마다 찾은 마커를 출력")
    parser.add_argument("--save", metavar="PNG", help="마커에 테두리를 그린 영상 한 장을 저장")
    parser.add_argument("--window", action="store_true", help="카메라 영상을 창으로 띄운다")
    args = parser.parse_args()
    rclpy.init()
    node = MarkerDetector(args.show, args.save, args.window)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException, rclpy._rclpy_pybind11.RCLError):
        pass                                             # Ctrl+C 로 끝낼 때


if __name__ == "__main__":
    main()
