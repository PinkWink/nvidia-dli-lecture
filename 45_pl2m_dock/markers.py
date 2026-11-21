"""도킹에 쓰는 ArUco 마커: 어디에 붙어 있는지, 그림을 어떻게 만드는지. Isaac Sim 쪽과 ROS 2 쪽이 같이 쓴다.

값은 원본 저장소의 src/mobile_openarm_mujoco/worlds/booth.yaml (markers) 와 warehouse_skills/config/skills.yaml (dock) 에서 가져왔다.
마커는 스테이션마다 두 장이다. 테이블 앞면, 바닥에서 0.3 m 높이에 좌우로 벌려 붙였다.
"""
import math

DICTIONARY = "DICT_4X4_50"            # 4 × 4 칸짜리 마커 50종
MARKER_SIZE, PLATE_SIZE = 0.15, 0.19  # 검은 네모의 한 변 · 흰 판의 한 변 [m]
MARKER_HEIGHT = 0.3                   # 마커 가운데의 높이 [m]

# id → (스테이션, 가운데 (x, y), 마커가 바라보는 방향 [rad], 스테이션 축에서 옆으로 벗어난 거리 [m]: 도킹한 로봇이 볼 때 왼쪽이 +)
MARKERS = {
    0: ("drinks",    (7.82, -1.805),  -math.pi / 2, -0.17),
    1: ("drinks",    (7.48, -1.805),  -math.pi / 2, +0.17),
    2: ("consult_1", (5.985, -2.05),  math.pi,      -0.15),
    3: ("consult_1", (5.985, -1.75),  math.pi,      +0.15),
    4: ("consult_2", (6.085, -4.58),  math.pi,      -0.15),
    5: ("consult_2", (6.085, -4.28),  math.pi,      +0.15),
}

DOCK = {                              # 원본 skills.yaml 의 dock
    "target_dx": 0.43,                # 도킹을 마쳤을 때 로봇 중심에서 마커 면까지의 거리 [m]. 로봇 앞면이 0.30 m 라 13 cm 가 남는다
    "visual_end_dx": 0.80,            # 이 거리부터는 마커가 화면을 벗어나므로 카메라를 믿지 않고 오도메트리로 직진한다
    "max_speed": 0.10, "blind_speed": 0.06,
    "gain": 0.6, "lateral_gain": 1.2, "heading_gain": 0.4,
    "max_lateral_error": 0.05,        # 축에서 이보다 많이 벗어나 있으면 먼저 옆걸음으로 맞춘다 [m]
    "undock_distance": 0.75,          # 작업을 마치고 뒤로 물러나는 거리 [m]
    "timeout": 180.0,
}


def station_markers(station):
    """스테이션의 마커들: {id: 옆으로 벗어난 거리}."""
    return {marker: spec[3] for marker, spec in MARKERS.items() if spec[0] == station}


def docked_pose(station):
    """도킹을 마쳤을 때 로봇이 있어야 할 자리 (x, y, 방향): 두 마커의 가운데에서 target_dx 만큼 물러난 곳. 채점에 쓴다."""
    spots = [spec for spec in MARKERS.values() if spec[0] == station]
    x = sum(spec[1][0] for spec in spots) / len(spots)
    y = sum(spec[1][1] for spec in spots) / len(spots)
    facing = spots[0][2] + math.pi                                    # 로봇은 마커가 바라보는 쪽의 반대를 본다
    yaw = math.atan2(math.sin(facing), math.cos(facing))
    return x - DOCK["target_dx"] * math.cos(yaw), y - DOCK["target_dx"] * math.sin(yaw), yaw


def aruco_dictionary():
    import cv2
    return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, DICTIONARY))


def marker_image(marker, pixels=400):
    """흰 판 가운데에 마커를 그린 그림 (흑백)."""
    import cv2
    import numpy as np
    inner = int(round(pixels * MARKER_SIZE / PLATE_SIZE))
    border = (pixels - inner) // 2
    draw = getattr(cv2.aruco, "generateImageMarker", None) or cv2.aruco.drawMarker      # OpenCV 4.7 부터 이름이 바뀌었다
    canvas = np.full((pixels, pixels), 255, np.uint8)
    canvas[border:border + inner, border:border + inner] = draw(aruco_dictionary(), int(marker), inner)
    return canvas
