"""강의 48: 컵의 자리를 좌표표에서 읽지 않고 머리 카메라로 찾아서 집어 나릅니다. 강의 46 의 임무에 "보기(LOOK)" 를 더한 것입니다.

    source /opt/ros/jazzy/setup.bash
    source ~/pinklab_robotworld2026/install/setup.bash
    python3 48_pl2m_find_pick/find_pick.py                                  # 분홍 컵을 찾아 음료대 → 상담 테이블 1 로
    python3 48_pl2m_find_pick/find_pick.py --cup cup_yellow --to consult_2  # 노란 컵을 상담 테이블 2 로
    python3 48_pl2m_find_pick/find_pick.py --only look                      # 서 있는 자리에서 보이는 컵을 출력만
    python3 48_pl2m_find_pick/find_pick.py --only pick                      # 음료대에 도킹한 자리에서 찾아 집기만 (시뮬레이터를 --start drinks --docked 로)
    python3 48_pl2m_find_pick/find_pick.py --save run.json                  # 모드의 기록을 저장

컵 찾는 노드(강의 47 의 cup_detector.py)가 떠 있어야 한다. 이 프로그램은 그 노드가 내는 /vision/detections 만 읽는다.

모드가 바뀌는 순서 (강의 46 과 다른 곳에 ★)
    ARM      두 팔을 이동 자세로
    NAV2     음료대 앞으로
  ★ LOOK     서서 마커와 컵을 봄            (멈춘 뒤에 찍은 영상 5장에서 컵을 찾음)  → 집을 팔과 비켜 설 거리를 정함. 닿지 않는 자리면 여기서 그만둠
    SETTLE → SEARCH → (ALIGN) → SERVO → BLIND → DOCKED      강의 45 의 도킹
  ★ LOOK     도킹한 자리에서 다시 봄        (영상 5장)  → 컵의 자리 (로봇 기준). 다른 컵은 피할 물체로 MoveIt 에 알림
    ARM      컵을 집어 운반 자세로
    UNDOCK → NAV2 상담 테이블 앞으로 → SETTLE → … → DOCKED
  ★ LOOK     놓을 자리가 비었는지 봄
    ARM      컵을 내려놓고 이동 자세로
    UNDOCK → NAV2 처음 자리로 → IDLE

모드 사이에 건네는 것 (강의 46 과 다른 것)
    LOOK → 도킹      비켜 설 거리(aim). 좌표표가 아니라 방금 본 컵의 자리에서 셈한다
    LOOK → ARM       컵의 자리 (로봇 기준의 앞, 왼쪽, 높이). 도킹이 알려 준 값(side, turn)은 가구의 자리를 셈하는 데만 쓴다
"""
import argparse
import math
import os
import sys

import numpy as np
import rclpy
from warehouse_interfaces.msg import Detection3DArray

HERE = os.path.dirname(os.path.abspath(__file__))
for folder in ("42_pl2m_moveit", "43_pl2m_pick", "44_pl2m_nav", "45_pl2m_dock", "46_pl2m_mission"):
    sys.path.insert(0, os.path.join(HERE, "..", folder))
import dock
from booth import STATIONS
from dock import DockError, station_geometry
from layout import CUP_DIAMETER, CUP_HEIGHT, CUPS, people_near, slot, to_base
from markers import DOCK, station_markers
from mission import ARM_TOLERANCE, ArmError, Mission, shape

dock.LABELS["LOOK"] = "컵 찾기"
LOOK_FRAMES = 5                    # 멈춘 뒤에 찍은 영상 이만큼의 평균을 쓴다
MIN_SCORE = 0.7                    # 점수가 이보다 낮은 컵은 집지 않는다. 다른 컵에 가려졌을 수 있다 (강의 47)
HEIGHT_TOLERANCE = 0.015           # 컵의 높이가 "상판 위에 선 컵" 과 이보다 많이 다르면 집지 않는다 [m]. 가려진 컵은 3.4 cm 높게 나왔다
# 팔이 닿는 범위. 도킹한 자리에서 컵을 집어 운반 자세까지 직선으로 이어지는 자리를 잰 것이다 (강의 48 의 3절)
REACH_DEPTH = (0.05, 0.12)         # 컵의 가운데가 테이블 앞면(마커 면)에서 안쪽으로 이만큼 [m]
REACH_SIDE = (0.08, 0.24)          # 컵이 로봇의 가운데에서 집는 팔 쪽으로 이만큼 [m]. 잰 값은 0.06 ~ 0.26 이고 도킹의 오차 2 cm 를 뺐다
MAX_STEP = 0.12                    # 옆으로 비켜 서는 거리의 한계 [m]. 강의 45 에서 12 cm 까지 해 봤다. 더 비키면 다가가는 동안 마커가 화면을 벗어난다
BODY_HALF, PERSON_GAP = 0.25, 0.06  # 베이스의 폭의 절반, 비켜 섰을 때 옆 사람과 남길 틈 [m]
HEAD_CAMERA = (0.06, 0.0)          # 머리 카메라의 자리 (로봇 기준의 앞, 왼쪽) [m]. URDF 의 head_camera_link
FAR_SLACK = 0.02                   # 1 m 밖에서 잰 깊이는 1 cm 쯤 어긋난다 (마커 면까지의 거리가 그만큼 틀린다). 멀리서는 이만큼 넉넉하게 본다


def step_range(station):
    """도킹할 때 비켜 설 수 있는 범위 (aim 의 최소, 최대). aim > 0 이면 로봇이 축의 오른쪽에 선다.

    옆걸음은 오도메트리만 보고 움직이므로 옆에 선 사람을 피하지 못한다. 사람이 선 쪽으로는 틈이 남는 만큼만 비킨다.
    """
    room = {"left": MAX_STEP, "right": MAX_STEP}
    for _, kind, center, size in people_near(station):
        ahead, left, _ = to_base(station, center)
        if kind == "cylinder" and -0.4 < ahead < 0.45:                                # 도킹한 베이스의 옆에 있는 것만
            side = "left" if left > 0 else "right"
            room[side] = min(room[side], max(0.0, abs(left) - size[0] - BODY_HALF - PERSON_GAP))
    return -room["left"], room["right"]


def arm_and_aim(station, left, depth):
    """컵이 스테이션의 축에서 왼쪽으로 left, 테이블 앞면에서 안쪽으로 depth 에 있을 때: (집을 팔, 비켜 설 거리 aim).

    도킹한 뒤 컵은 로봇의 가운데에서 왼쪽으로 left + aim 에 온다. 이것이 그 팔의 범위(REACH_SIDE)에 들어오는 aim 가운데 가장 작게 움직이는 것을 고른다.
    """
    if not REACH_DEPTH[0] - FAR_SLACK <= depth <= REACH_DEPTH[1] + FAR_SLACK:        # 확실히 벗어난 것만 여기서 거른다. 아슬아슬한 것은 도킹한 뒤 집기 전 확인이 가른다
        raise ArmError(f"컵이 테이블 앞면에서 {depth * 100:.1f} cm 안쪽에 있음: 팔이 닿는 것은 {REACH_DEPTH[0] * 100:.0f} ~ {REACH_DEPTH[1] * 100:.0f} cm")
    low, high = step_range(station)
    choices = []
    for arm, sign in (("left", 1.0), ("right", -1.0)):
        toward = sign * left                                                          # 축 위에 섰을 때 컵이 그 팔 쪽으로 떨어진 거리
        aim = sign * min(max(toward, REACH_SIDE[0]), REACH_SIDE[1]) - left
        if low - 1e-6 <= aim <= high + 1e-6:
            choices.append((abs(aim), arm, aim))
    if not choices:
        raise ArmError(f"컵이 축에서 왼쪽으로 {left * 100:+.1f} cm 에 있음: 비켜 설 수 있는 범위({low * 100:+.0f} ~ {high * 100:+.0f} cm) 안에서는 어느 팔도 닿지 않음")
    _, arm, aim = min(choices)
    return arm, (0.0 if abs(aim) < 0.005 else aim)


class FindPick(Mission):
    """강의 46 의 임무(Mission)에서 컵의 자리를 정하는 두 곳만 카메라로 바꾼다: choose_arm · cup_position."""

    def __init__(self):
        super().__init__()
        self.frames = []                     # 받은 /vision/detections
        self.plan = None                     # LOOK 이 정한 (팔, aim)
        self.target = None                   # 도킹한 뒤에 본 컵의 자리 (로봇 기준)
        self.others = []                     # 같은 테이블의 다른 컵: MoveIt 에 알릴 도형
        self.base.create_subscription(Detection3DArray, "/vision/detections", self.frames.append, 10)

    # --- 보기 -----------------------------------------------------------
    def look(self, timeout=8.0):
        """서 있는 채로, 지금부터 찍은 영상 LOOK_FRAMES 장에서 컵을 모은다: {이름: (자리의 평균 [앞, 왼쪽, 높이], 점수의 평균)}.

        로봇이 멈춘 뒤에 불러야 한다. 움직이는 동안 찍은 영상의 자리는 지금의 로봇 기준이 아니다.
        """
        base = self.base
        since, end = base.now(), base.now() + timeout
        self.frames.clear()
        fresh = []
        while len(fresh) < LOOK_FRAMES and base.now() < end:
            base.spin(0.05)
            fresh = [m for m in self.frames if m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 >= since]
        if not fresh:
            raise ArmError("/vision/detections 가 오지 않음: 컵 찾는 노드(cup_detector.py)가 떠 있는지 볼 것")
        seen = {}
        for message in fresh:
            for item in message.detections:
                if item.has_position:
                    seen.setdefault(item.label, []).append((item.position.point.x, item.position.point.y, item.position.point.z, item.score))
        return {label: (np.mean(rows, axis=0)[:3], float(np.mean(rows, axis=0)[3])) for label, rows in seen.items() if len(rows) > len(fresh) / 2}

    def checked(self, seen, cup, top):
        """본 것 가운데 cup 의 자리. 보이지 않거나, 점수가 낮거나, 높이가 상판 위에 선 컵과 다르면 그만둔다."""
        if cup not in seen:
            raise ArmError(f"{cup} 이(가) 보이지 않음 (보인 것: {', '.join(sorted(seen)) or '없음'})")
        position, score = seen[cup]
        off = position[2] - (top + CUP_HEIGHT / 2.0)
        if score < MIN_SCORE or abs(off) > HEIGHT_TOLERANCE:
            raise ArmError(f"{cup} 의 자리를 믿을 수 없음: 점수 {score:.2f} (한계 {MIN_SCORE}), 높이가 상판 위에 선 컵보다 {off * 1000:+.0f} mm (한계 ±{HEIGHT_TOLERANCE * 1000:.0f} mm). 가려졌을 수 있음")
        return position, score

    def look_from_afar(self, station, cup):
        """Nav2 로 도착한 자리에서: 마커로 내가 선 자리를, 머리 카메라로 컵을 봄 → 컵이 스테이션의 어디에 있나 (축에서 왼쪽으로, 앞면에서 안쪽으로)."""
        base, offsets = self.base, station_markers(station)
        base.wait_still()
        along, side, turn = station_geometry(base.find_markers(offsets), offsets)       # 마커 면까지, 축이 왼쪽으로, 돌아야 할 각 (강의 45)
        (x, y, _), score = self.checked(self.look(), cup, STATIONS[station]["top"])
        ahead, left = x * math.cos(turn) + y * math.sin(turn), -x * math.sin(turn) + y * math.cos(turn)     # 축과 나란한 방향으로 돌려서
        left, depth = left - side, ahead - along
        print(f"            {cup}: 점수 {score:.2f}, 로봇 기준 앞 {x:.3f} · 왼쪽 {y:+.3f} m.  내가 선 자리: 면까지 {along:.3f} m · 축이 왼쪽으로 {side:+.3f} m · 틀어진 각 {math.degrees(turn):+.1f}°", flush=True)
        print(f"            → 컵은 축에서 왼쪽으로 {left * 100:+.1f} cm, 테이블 앞면에서 안쪽으로 {depth * 100:.1f} cm", flush=True)
        return left, depth

    def untangle(self, seen, cup, top):
        """다른 컵의 자리가 집을 컵과 겹쳐 있으면 고쳐 놓는다. 컵은 서로 겹칠 수 없으므로 겹쳐 보이면 한쪽이 틀린 것이다.

        집을 컵은 checked 를 통과했다. 겹친 쪽은 그 뒤에 가려진 컵으로 보고, 카메라에서 본 방향을 따라 겹치지 않는 데까지 뒤로 민다. 높이는 상판 위로 둔다.
        """
        (tx, ty, _), _ = seen[cup]
        fixed = {}
        for name, (position, score) in seen.items():
            x, y, z = (float(v) for v in position)
            gap = math.hypot(x - tx, y - ty)
            if name != cup and gap < CUP_DIAMETER + 0.005:
                ux, uy = x - HEAD_CAMERA[0], y - HEAD_CAMERA[1]
                far = math.hypot(ux, uy)
                ux, uy = ux / far, uy / far                                           # 카메라에서 그 컵을 보는 방향
                bx, by = HEAD_CAMERA[0] - tx, HEAD_CAMERA[1] - ty
                along = bx * ux + by * uy                                             # 그 선 위에서 집을 컵과 CUP_DIAMETER + 5 mm 떨어지는 먼 쪽의 점
                reach = -along + math.sqrt(max(0.0, along * along - (bx * bx + by * by) + (CUP_DIAMETER + 0.005) ** 2))
                x, y, z = HEAD_CAMERA[0] + reach * ux, HEAD_CAMERA[1] + reach * uy, top + CUP_HEIGHT / 2.0
                print(f"            {name} 의 자리가 {cup} 와 겹쳐 보임 (가운데끼리 {gap * 100:.1f} cm, 점수 {score:.2f}): 뒤에 가려진 것으로 보고 {(reach - far) * 100:.1f} cm 뒤로 옮겨 알림", flush=True)
            fixed[name] = (x, y, z)
        return fixed

    # --- 강의 46 의 두 곳을 바꿈 ------------------------------------------
    def choose_arm(self, station, cup):
        return self.plan

    def cup_position(self, station, cup, where):
        return tuple(float(v) for v in self.target)

    def add_furniture(self, station, where, more=()):
        """가구 · 사람에 더해, 같은 테이블에서 본 다른 컵도 피할 물체로 넣는다."""
        names = super().add_furniture(station, where, list(more) + self.others)
        return names + [item.id for item in self.others]

    def look_docked(self, station, cup):
        """도킹한 자리에서 다시 봄: 집을 컵의 자리(self.target)와 다른 컵들(self.others)."""
        seen = self.look()
        self.target, score = self.checked(seen, cup, STATIONS[station]["top"])
        x, y, z = self.target
        places = self.untangle(seen, cup, STATIONS[station]["top"])
        self.others = [shape(name, "cylinder", place, (CUP_DIAMETER / 2, CUP_HEIGHT)) for name, place in places.items() if name != cup and name in CUPS]
        print(f"            {cup}: 점수 {score:.2f}, 로봇 기준 앞 {x:.3f} · 왼쪽 {y:+.3f} · 높이 {z:.3f} m.  다른 컵 {len(self.others)}개를 피할 물체로 알림", flush=True)
        if self.plan is None:                                                          # 도킹한 자리에서 시작했을 때 (--only pick): 컵이 있는 쪽 팔로
            self.plan = ("left" if y > 0 else "right", 0.0)
        toward = y if self.plan[0] == "left" else -y
        if not REACH_SIDE[0] - 0.02 <= toward <= REACH_SIDE[1] + 0.02 or not REACH_DEPTH[0] - 0.005 <= x - DOCK["target_dx"] <= REACH_DEPTH[1] + 0.005:
            print(f"            (팔이 닿는 범위의 밖일 수 있음: 팔 쪽으로 {toward * 100:.1f} cm, 앞면에서 {(x - DOCK['target_dx']) * 100:.1f} cm. 집기 전 확인에서 가려냄)", flush=True)

    def slot_is_free(self, station, where, side):
        """놓을 자리에 이미 컵이 있나 본다. 손에 든 컵도 보이지만 놓을 자리에서 멀다."""
        x, y, _ = to_base(station, slot(station, side), *where)
        for name, (position, _) in self.look().items():
            if math.hypot(position[0] - x, position[1] - y) < CUP_DIAMETER + 0.03:
                raise ArmError(f"놓을 자리에 {name} 이(가) 있음 (놓을 자리에서 {math.hypot(position[0] - x, position[1] - y) * 100:.1f} cm): 내려놓지 않음")
        print("            놓을 자리가 비어 있음", flush=True)

    # --- 모드를 잇기 ----------------------------------------------------
    def run(self, cup, source, target, tries=2):
        base = self.base
        self.mode("ARM", "두 팔을 이동 자세(transport)로")
        for side in ("left", "right"):
            self.arm_to(side, "transport")
        # 1. 음료대 앞으로 가서 보고, 설 자리를 정해 도킹
        for attempt in range(1, tries + 1):
            base.navigate(source)
            self.mode("LOOK", f"서서 마커와 {cup} 을(를) 봄")
            left, depth = self.look_from_afar(source, cup)
            self.plan = arm_and_aim(source, left, depth)
            arm, aim = self.plan
            print(f"            → {'왼팔' if arm == 'left' else '오른팔'}로 집음. " + (f"축이 왼쪽으로 {aim * 100:+.0f} cm 인 자리에 비켜 서서 도킹함" if aim else "축 위에 서서 도킹함"), flush=True)
            try:
                _, *where = base.dock(source, aim=aim, tolerance=ARM_TOLERANCE)       # 도킹이 건네는 값: (side, turn)
                break
            except DockError as error:                                                # 강의 45 의 go_and_dock 과 같다: 물러났다가 한 번 더
                base.halt()
                self.mode("FAILED", f"{error} ({attempt}/{tries}번째)")
                if attempt == tries:
                    raise
                base.drive(-0.3)
        forward, side_error, yaw = base.score(source)
        print(f"            채점 (참값): 서 있는 자리가 앞뒤 {forward * 1000:+.1f} mm · 좌우 {side_error * 1000:+.1f} mm · 방향 {math.degrees(yaw):+.2f}° 어긋남", flush=True)
        # 2. 도킹한 자리에서 다시 보고 집기
        self.mode("LOOK", "도킹한 자리에서 다시 봄")
        self.look_docked(source, cup)
        self.mode("ARM", f"베이스는 서 있음. {cup} 을(를) 집음")
        self.mark_base()
        side = self.pick(source, cup, where)
        self.others = []
        self.leave(side, "carry")
        # 3. 상담 테이블로 가서 도킹, 놓을 자리를 보고 놓기
        _, (_, *where) = base.go_and_dock(target, tolerance=ARM_TOLERANCE)
        forward, side_error, yaw = base.score(target)
        print(f"            채점 (참값): 서 있는 자리가 앞뒤 {forward * 1000:+.1f} mm · 좌우 {side_error * 1000:+.1f} mm · 방향 {math.degrees(yaw):+.2f}° 어긋남", flush=True)
        self.mode("LOOK", "놓을 자리가 비었는지 봄")
        self.slot_is_free(target, where, side)
        self.mode("ARM", f"베이스는 서 있음. {cup} 을(를) {'왼팔' if side == 'left' else '오른팔'}의 정면에 놓음")
        self.mark_base()
        self.place(target, where)
        self.stood = None
        self.leave(side, "transport")
        # 4. 처음 자리로
        base.navigate("home")
        self.mode("IDLE", "임무 끝")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--cup", choices=list(CUPS), default="cup_pink")
    parser.add_argument("--from", dest="source", default="drinks", choices=["drinks"])
    parser.add_argument("--to", dest="target", default="consult_1", choices=["consult_1", "consult_2"])
    parser.add_argument("--only", choices=["look", "pick"], help="look: 보이는 컵을 출력만, pick: 도킹한 자리에서 찾아 집기만")
    parser.add_argument("--save", metavar="JSON", help="모드의 기록을 저장")
    args = parser.parse_args()

    rclpy.init()
    mission = FindPick()
    base = mission.base
    base.wait_for(lambda: base.odom_msg is not None and base.truth_msg is not None, 60.0, "/odom · /ground_truth (시뮬레이터)")
    try:
        if args.only == "look":
            for name, (position, score) in sorted(mission.look().items()):
                print(f"{name:<11} 점수 {score:.2f}  로봇 기준 앞 {position[0]:.3f} · 왼쪽 {position[1]:+.3f} · 높이 {position[2]:.3f} m", flush=True)
        elif args.only == "pick":
            mission.mode("LOOK", "도킹한 자리에서 봄")
            mission.look_docked(args.source, args.cup)
            mission.mode("ARM", f"{args.cup} 을(를) 집음")
            mission.mark_base()
            mission.pick(args.source, args.cup, (0.0, 0.0))
            mission.mode("IDLE", "집어서 운반 자세")
        else:
            base.ready()
            mission.run(args.cup, args.source, args.target)
    except (DockError, ArmError) as error:
        if base.mode != "FAILED":
            mission.mode("FAILED", str(error))
        base.halt()
    if args.save:
        base.save(args.save)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
