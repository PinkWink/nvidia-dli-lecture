"""부스의 자리들: 로봇이 서는 곳과 가는 곳. Isaac Sim 쪽(pl2m_nav.py)과 ROS 2 쪽(go_to.py)이 같이 쓴다.

값은 원본 저장소의 src/mobile_openarm_mujoco/worlds/booth.yaml 에서 가져왔다.
좌표는 부스 좌표계 = 지도 좌표계(map): x 0 ~ 9 m, y -6 ~ 0 m 가 부스다.
"""
import math
import os

REPO = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
SOURCE_MAP = os.path.expanduser("~/pinklab_robotworld2026/src/mobile_openarm_navigation/maps/booth.yaml")   # 원본이 자기 MuJoCo 장면에서 만든 지도
MAP = os.path.join(REPO, "work", "booth", "booth_map.yaml")                                                  # make_map.py 가 만드는 지도
AREA = (0.0, -6.0, 9.0, 0.0)                 # 부스의 범위: x 최소, y 최소, x 최대, y 최대 [m]. 로봇은 이 밖(통로)으로 나가지 않는다
SPAWN = (5.3, -3.2, 0.0)                     # 로봇이 처음 서는 자리 (x, y, 방향[rad]): 상담 테이블 둘 사이

# 스테이션: goal = Nav2 로 가는 자리 (x, y, 방향). 테이블 가장자리에서 1 m 쯤 떨어져 테이블을 바라본다
#           edge = 로봇이 마주 보는 테이블 가장자리의 가운데,  top = 상판 높이 [m]
STATIONS = {
    "drinks":    {"goal": (7.65, -2.82, math.pi / 2), "edge": (7.65, -1.80), "top": 0.75, "label": "음료대"},
    "consult_1": {"goal": (4.96, -1.90, 0.0),         "edge": (5.98, -1.90), "top": 0.72, "label": "상담 테이블 1"},
    "consult_2": {"goal": (5.06, -4.43, 0.0),         "edge": (6.08, -4.43), "top": 0.72, "label": "상담 테이블 2"},
    "home":      {"goal": SPAWN,                      "edge": None,          "top": None, "label": "처음 자리"},
}


def angle_difference(a, b):
    """두 방향의 차이 a - b 를 -π ~ π 로."""
    return math.atan2(math.sin(a - b), math.cos(a - b))
