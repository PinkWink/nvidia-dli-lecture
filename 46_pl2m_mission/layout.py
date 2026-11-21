"""스테이션에 무엇이 어디 있는지: 컵 · 놓을 자리 · 팔이 피해야 할 가구와 사람. ROS 2 쪽(mission.py)이 쓴다.

값은 원본 저장소의 worlds/booth.yaml (objects · scene) 과 warehouse_skills/config/skills.yaml (grasp · place · carry) 에서 가져왔다.
좌표는 부스 좌표계(지도와 같음)다. 로봇은 도킹한 뒤 이 좌표를 자기 기준으로 바꿔 쓴다 (to_base).
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
for folder in ("44_pl2m_nav", "45_pl2m_dock"):
    sys.path.insert(0, os.path.join(HERE, "..", folder))
from booth import STATIONS
from markers import docked_pose

DECK_TOP = 0.02                               # 바닥판의 높이 [m]. 로봇이 이 위에 서 있으므로, 로봇 기준 높이 = 부스 좌표의 높이 - 0.02
CUP_DIAMETER, CUP_HEIGHT, CUP_MASS = 0.05, 0.09, 0.2
CUPS = {                                      # 음료대 위의 컵: 가운데 (x, y, z)
    "cup_pink":   (7.50, -1.72, 0.795),
    "cup_blue":   (7.65, -1.72, 0.795),
    "cup_yellow": (7.80, -1.72, 0.795),
}
# 놓는 자리: 든 팔의 정면. 축에서 옆으로 15 cm (왼팔이면 왼쪽), 가장자리에서 9 cm 안쪽. 상판 4 mm 위에서 놓는다.
# 원본은 가운데부터 채우고(옆으로 0 · ±10 cm) 가장자리에서 5.5 cm 안쪽에 놓는다. 그 자리는 이 강의의 설정에서 팔이 닿지 않았다 (강의 46 의 5절).
PLACE = {"from_edge": 0.09, "lateral": 0.15, "drop_z": 0.004}
GRASP = {"pre_offset_x": -0.10, "z_offset": 0.02, "lift_z": 0.04, "retreat_x": -0.14,
         "open": 0.044, "close": 0.012, "close_min": 0.016, "close_max": 0.028}                       # 강의 43 과 같은 값
CARRY_TIP = (0.34, 0.153, 0.86)               # 운반 자세(carry)일 때 왼손 손끝의 자리 (로봇 기준). 오른손은 y 의 부호가 반대

REACH = 0.12                                  # 팔이 컵을 집어 운반 자세까지 곧게 가려면, 컵이 로봇의 가운데에서 그 팔 쪽으로 이만큼은 떨어져 있어야 한다 [m]
                                              # 재 본 한계는 8 cm 다 (7 cm 에서는 직선이 15 % 에서 끊겼다). 도킹 오차 2 cm 와 여유를 더했다 (강의 46 의 6절)

# 팔이 피해야 할 가구: 스테이션 → [(이름, 모양, 가운데, 크기)]. 상자의 크기는 (x, y, z), 원기둥은 (반지름, 높이), 공은 (반지름,)
FURNITURE = {
    "drinks":    [("drink_stand", "box", (7.65, -1.60, 0.375), (0.62, 0.42, 0.75))],
    "consult_1": [("consult_1_top", "cylinder", (6.28, -1.90, 0.705), (0.30, 0.03)),
                  ("consult_1_post", "cylinder", (6.28, -1.90, 0.35), (0.05, 0.70)),
                  ("consult_1_board", "box", (6.00, -1.90, 0.30), (0.02, 0.50, 0.44))],
    "consult_2": [("consult_2_top", "cylinder", (6.38, -4.43, 0.705), (0.30, 0.03)),
                  ("consult_2_post", "cylinder", (6.38, -4.43, 0.35), (0.05, 0.70)),
                  ("consult_2_board", "box", (6.10, -4.43, 0.30), (0.02, 0.50, 0.44))],
}


BOOTH_XML = os.path.join(HERE, "..", "work", "booth", "booth.xml")     # 강의 38 이 만든 부스의 MuJoCo 장면 파일. 시뮬레이터의 부스도 이것으로 만든다


def people_near(station, within=1.0, margin=0.02, path=BOOTH_XML):
    """도킹한 자리에서 within [m] 안에 선 사람 모형의 도형들: [(이름, 모양, 가운데, 크기)], 부스 좌표.

    시뮬레이터에서는 사람도 충돌체라(강의 38) 팔이 닿으면 로봇이 밀린다. MoveIt 은 알려 준 것만 피하므로 팔이 닿을 만한 사람은 알려야 한다.
    캡슐(다리 · 몸통 · 팔)은 서 있는 원기둥으로, 공(머리)은 공으로 옮기고 margin 만큼 키운다.
    """
    import xml.etree.ElementTree as ET
    x0, y0, _ = docked_pose(station)
    shapes = []
    for body in ET.parse(path).getroot().find("worldbody").findall("body"):
        bx, by, bz = (float(v) for v in body.get("pos").split())
        yaw = float(body.get("euler", "0 0 0").split()[2])
        if body.get("name").startswith("pl2m") or math.hypot(bx - x0, by - y0) > within:
            continue
        for geom in body.findall("geom"):
            radius = float(geom.get("size").split()[0])
            ends = [float(v) for v in (geom.get("fromto") or geom.get("pos") + " " + geom.get("pos")).split()]
            (ax, ay, az), (cx, cy, cz) = ends[:3], ends[3:]
            mx, my, mz = (ax + cx) / 2.0, (ay + cy) / 2.0, (az + cz) / 2.0                  # 몸 기준의 가운데 → 부스 좌표
            center = (bx + mx * math.cos(yaw) - my * math.sin(yaw), by + mx * math.sin(yaw) + my * math.cos(yaw), bz + mz)
            name = f"{body.get('name')}_{geom.get('name')}"
            if geom.get("type") == "sphere":
                shapes.append((name, "sphere", center, (radius + margin,)))
            else:                                                                           # 기운 캡슐은 그만큼 굵은 원기둥으로 감싼다
                shapes.append((name, "cylinder", center, (radius + math.hypot(cx - ax, cy - ay) / 2.0 + margin, abs(cz - az) + 2.0 * radius)))
    return shapes


def to_base(station, point, side=0.0, turn=0.0):
    """부스 좌표의 점 → 도킹한 로봇 기준의 (앞, 왼쪽, 높이).

    로봇은 마커 면에서 0.43 m 떨어져 서 있다고 본다. side · turn 은 도킹이 알려 준 값이다:
    축이 로봇의 왼쪽으로 벗어난 거리 [m], 축과 나란해지려면 로봇이 돌아야 하는 각 [rad].
    """
    x0, y0, yaw = docked_pose(station)
    dx, dy = point[0] - x0, point[1] - y0
    ahead = dx * math.cos(yaw) + dy * math.sin(yaw)                  # 축을 따라 앞으로
    left = -dx * math.sin(yaw) + dy * math.cos(yaw) + side           # 축에서 왼쪽으로 + 로봇이 축에서 벗어난 만큼
    return ahead * math.cos(turn) - left * math.sin(turn), ahead * math.sin(turn) + left * math.cos(turn), point[2] - DECK_TOP


def arm_and_aim(station, cup):
    """그 컵을 집을 팔과, 도킹할 때 비켜 설 거리 aim [m] (축이 로봇의 왼쪽으로 aim 만큼 떨어지게 선다. - 면 로봇이 왼쪽으로 비켜 선 것).

    컵이 축에서 REACH 넘게 떨어져 있으면 그쪽 팔로, 축 위에 서서 집는다.
    가운데 가까이 있는 컵은 어느 팔로도 곧게 닿지 않는다. 가까운 사람이 없는 쪽으로 비켜 서고, 그 반대쪽 팔로 집는다.
    """
    left = to_base(station, CUPS[cup])[1]                             # 축 위에 섰을 때 컵이 로봇의 왼쪽으로 떨어진 거리
    if abs(left) >= REACH:
        return ("left" if left > 0 else "right"), 0.0
    people = [to_base(station, center)[1] for _, _, center, _ in people_near(station)]
    step_left = not people or min(people, key=abs) < 0               # 가장 가까운 사람이 오른쪽에 있으면 왼쪽으로 비켜 선다
    return ("right", -REACH - left) if step_left else ("left", REACH - left)


def slot(station, side):
    """side("left" · "right") 팔이 놓을 자리의 부스 좌표: 컵의 가운데 (x, y, z)."""
    (ex, ey), yaw, top = STATIONS[station]["edge"], STATIONS[station]["goal"][2], STATIONS[station]["top"]
    inside, left = PLACE["from_edge"], PLACE["lateral"] if side == "left" else -PLACE["lateral"]
    return (ex + inside * math.cos(yaw) - left * math.sin(yaw), ey + inside * math.sin(yaw) + left * math.cos(yaw), top + CUP_HEIGHT / 2)
