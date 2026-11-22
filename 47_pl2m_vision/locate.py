"""상자(픽셀)와 깊이 영상에서 컵의 가운데를 구한다. ROS 2 도 YOLO 도 쓰지 않는 계산만 모았다 (cup_detector.py 가 쓴다).

좌표계
    카메라(optical)   +X 영상의 오른쪽, +Y 영상의 아래, +Z 카메라가 보는 쪽
    로봇(base)        +X 앞, +Y 왼쪽, +Z 위. base_footprint (바닥에 붙은 로봇의 가운데)
"""
import numpy as np

CUP_RADIUS, CUP_HEIGHT = 0.025, 0.09          # 컵의 반지름 · 높이 [m]


def back_project(us, vs, depths, matrix):
    """픽셀 (u, v) 와 깊이 z → 카메라 기준의 점 (x, y, z). 깊이는 카메라 앞쪽(+Z)으로 잰 거리다.

        x = (u - cx) z / fx        y = (v - cy) z / fy
    """
    fx, fy, cx, cy = matrix[0, 0], matrix[1, 1], matrix[0, 2], matrix[1, 2]
    return np.stack([(us - cx) * depths / fx, (vs - cy) * depths / fy, depths], axis=-1)


def to_base(points, base_from_camera):
    """카메라 기준의 점들 → 로봇 기준. base_from_camera 는 4 × 4 행렬 (/tf 에서 읽은, 카메라가 달린 자리)."""
    return points @ base_from_camera[:3, :3].T + base_from_camera[:3, 3]


def cup_center(box, depth, matrix, base_from_camera, push=CUP_RADIUS):
    """상자 (x1, y1, x2, y2) 와 깊이 영상 → 로봇 기준의 컵 가운데 (앞, 왼쪽, 높이). 깊이를 읽지 못하면 None.

    1. 깊이: 상자 가운데 부분(가로 · 세로의 가운데 절반)의 중앙값. 가장자리에는 컵 뒤의 배경이 섞인다
    2. 상자의 가운데 픽셀과 그 깊이 → 카메라 기준의 점. 컵의 보이는 면 위에 있다
    3. 그 점을 같은 선 위에서 push 만큼 더 멀리 보낸다. 깊이는 보이는 면까지의 거리고, 컵의 가운데는 반지름만큼 더 안쪽이다
       (원본 저장소의 방법. push=0 이면 보이는 면 그대로다)
    4. 로봇 기준으로 옮긴다
    """
    x1, y1, x2, y2 = box
    w, h = x2 - x1, y2 - y1
    patch = depth[int(y1 + h / 4):int(y2 - h / 4) + 1, int(x1 + w / 4):int(x2 - w / 4) + 1]
    patch = patch[np.isfinite(patch) & (patch > 0.05)]
    if patch.size == 0:
        return None
    point = back_project(np.array((x1 + x2) / 2.0), np.array((y1 + y2) / 2.0), np.array(float(np.median(patch))), matrix)
    point = point * (1.0 + push / np.linalg.norm(point))
    return to_base(point, base_from_camera)
