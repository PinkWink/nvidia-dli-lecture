"""강의 45: Nav2 모드와 도킹 모드를 오가며 스테이션에 정확히 섭니다.

    python3 45_pl2m_dock/dock.py drinks                  # 지금 자리에서 음료대에 도킹 (시뮬레이터를 --start drinks 로 띄웠을 때)
    python3 45_pl2m_dock/dock.py drinks --nav            # Nav2 로 음료대 앞까지 간 뒤 도킹
    python3 45_pl2m_dock/dock.py drinks --nav --undock   # 도킹한 뒤 뒤로 물러나 Nav2 모드로 돌아옴
    python3 45_pl2m_dock/dock.py --tour 2                # 스테이션 셋을 두 바퀴: Nav2 → 도킹 → 물러나기, 오차를 표로
    python3 45_pl2m_dock/dock.py --tour 2 --strict-nav   # 비교용: Nav2 가 도착이라고 할 때까지 기다림
    python3 45_pl2m_dock/dock.py drinks --aim 0.05       # 축이 로봇의 왼쪽으로 5 cm 인 자리에 선다 (오른쪽으로 5 cm 비켜 서기)
    python3 45_pl2m_dock/dock.py drinks --no-align       # 비교용: 옆으로 벗어나 있어도 옆걸음을 하지 않음
    python3 45_pl2m_dock/dock.py drinks --single         # 비교용: 마커 한 장만 보고 도킹
    python3 45_pl2m_dock/dock.py drinks --visual-end 0.45  # 비교용: 마커를 보며 0.45 m 까지 가려고 해 본다 (어디서 마커를 놓치나)
    python3 45_pl2m_dock/dock.py drinks --save run.json  # 모드와 잰 값의 기록을 저장
    python3 45_pl2m_dock/dock.py drinks --measure 100    # 움직이지 않고 영상 100장을 재어, 마커 두 장과 한 장을 견줌

모드 (한 번에 하나만 켜진다)                  무엇을 보고 움직이나            누가 /cmd_vel 을 내나
    NAV2     Nav2 로 스테이션 앞까지          지도 + 라이다 (AMCL)            Nav2
    SETTLE   멈췄는지 확인                    오도메트리의 속도               이 프로그램 (0 만 냄)
    SEARCH   마커 두 장 찾기                  카메라                          이 프로그램
    ALIGN    옆으로 많이 벗어났으면 옆걸음    오도메트리                      이 프로그램
    SERVO    마커를 보며 다가가기             카메라 (마커 두 장)             이 프로그램
    BLIND    마커가 화면을 벗어난 뒤의 직진   오도메트리                      이 프로그램
    DOCKED   도킹 끝. 베이스는 서 있고 팔이 일한다                            아무도 (0 을 냄)
    UNDOCK   뒤로 물러나기                    오도메트리                      이 프로그램

도킹의 절차와 값은 원본 warehouse_skills/nav_client.py · pick_place_server.py 를 따랐다.
"""
import argparse
import collections
import json
import math
import os
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import Twist
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from warehouse_interfaces.msg import ArucoMarkerArray

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "44_pl2m_nav"))
from booth import STATIONS, angle_difference
from go_to import Base, yaw_of
from markers import DOCK, docked_pose, station_markers

LABELS = {"NAV2": "Nav2 이동", "SETTLE": "멈춤 확인", "SEARCH": "마커 찾기", "ALIGN": "옆걸음", "SERVO": "마커 보며 접근",
          "BLIND": "오도메트리 직진", "DOCKED": "도킹됨", "UNDOCK": "물러나기", "IDLE": "대기", "FAILED": "실패"}
STILL_LINEAR, STILL_ANGULAR, STILL_TIME = 0.005, 0.01, 0.5      # 멈췄다고 보는 속도 [m/s] · [rad/s] 와 그 상태가 이어져야 하는 시간 [s]
AIM_TOLERANCE = 0.02                                           # 비켜 설 때(aim)는 옆으로 2 cm 넘게 어긋나면 옆걸음을 한다: 마커를 보며 다가가는 동안에는 옆 오차가 거의 줄지 않는다
NEAR = (0.25, 5.0)                                             # Nav2 가 목표의 25 cm 안에서 5초 동안 나아가지 않으면 도착으로 본다: 도킹이 받아 줄 수 있는 범위


class DockError(Exception):
    """도킹을 계속할 수 없는 까닭."""


def station_geometry(fixes, offsets):
    """마커가 보인 자리(로봇 기준) → 스테이션과 로봇의 관계 (along, side, turn).

    fixes    {id: (앞 [m], 왼쪽 [m], 법선 방향 [rad])}: 마커의 가운데와 마커가 바라보는 쪽
    offsets  {id: 스테이션 축에서 옆으로 벗어난 거리 [m]}
    along    로봇에서 마커 면까지의 거리 (면에 수직으로)
    side     스테이션 축이 로봇의 왼쪽으로 얼마나 떨어져 있나 (+ 면 로봇이 축의 오른쪽에 있다)
    turn     로봇이 축과 나란해지려면 돌아야 하는 각 (+ 면 왼쪽으로)
    두 장이 보이면 두 가운데를 잇는 선에서 방향을 얻는다. 한 장이면 그 마커의 법선을 쓴다 (작은 영상에서는 많이 흔들린다).
    """
    seen = sorted((marker for marker in offsets if marker in fixes), key=lambda marker: offsets[marker])
    if not seen:
        raise DockError("마커가 보이지 않음")
    if len(seen) >= 2:
        a, b = seen[0], seen[-1]
        ux, uy = fixes[b][0] - fixes[a][0], fixes[b][1] - fixes[a][1]              # 오른쪽 마커 → 왼쪽 마커
        length = math.hypot(ux, uy)
        ux, uy = ux / length, uy / length
        scale = length / (offsets[b] - offsets[a])                                 # 잰 간격 ÷ 실제 간격 (1 에 가까워야 한다)
        ox, oy = fixes[a][0] - ux * offsets[a] * scale, fixes[a][1] - uy * offsets[a] * scale   # 축이 마커 면과 만나는 점
    else:
        a = seen[0]
        normal = fixes[a][2]
        ux, uy = math.sin(normal), -math.cos(normal)
        ox, oy = fixes[a][0] - ux * offsets[a], fixes[a][1] - uy * offsets[a]
    nx, ny = -uy, ux                                                               # 마커 면의 법선 (로봇 쪽을 향함)
    return -(ox * nx + oy * ny), ox * ux + oy * uy, -math.atan2(ny, -nx)


class Docker(Base):
    """모드를 하나씩 바꿔 가며 베이스를 스테이션에 세운다."""

    def __init__(self, name="dock"):
        super().__init__(name)
        self.mode, self.history, self.samples = "IDLE", [], []
        self.mode_pub = self.create_publisher(String, "/mode", 10)
        self.markers = None
        self.poses = collections.deque(maxlen=400)        # (시각, x, y, 방향) : 영상을 찍은 순간의 오도메트리를 찾는 데 쓴다
        self.pending, self.foreign = collections.deque(maxlen=200), 0     # 내가 낸 /cmd_vel 가운데 아직 되돌아오지 않은 것, 남이 낸 것의 수
        self.create_subscription(ArucoMarkerArray, "/vision/markers", self._on_markers, 10)
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd, 50)
        self.nav_goals = []                               # Nav2 가 받은 목표들의 상태 (액션 서버가 알려 준다)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_sub = self.create_subscription(GoalStatusArray, "/navigate_to_pose/_action/status",
                                                   lambda msg: setattr(self, "nav_goals", [goal.status for goal in msg.status_list]), latched)
        self.options = {"align": True, "settle": True, "single": False}

    # --- 듣기 -----------------------------------------------------------
    def _on_odom(self, msg):
        super()._on_odom(msg)
        self.poses.append((msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, msg.pose.pose.position.x, msg.pose.pose.position.y, yaw_of(msg.pose.pose.orientation)))

    def _on_markers(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        near = min(self.poses, key=lambda pose: abs(pose[0] - stamp)) if self.poses else None     # 찍은 순간의 오도메트리
        self.markers = (stamp, {m.id: (m.pose.position.x, m.pose.position.y, m.normal_yaw) for m in msg.markers}, near)

    def _on_cmd(self, msg):
        """/cmd_vel 을 듣는다. 0 이 아닌 명령인데 내가 낸 것이 아니면 다른 노드가 로봇을 움직이고 있는 것이다."""
        value = (msg.linear.x, msg.angular.z)
        if value == (0.0, 0.0):
            return
        if value in self.pending:
            self.pending.remove(value)
        else:
            self.foreign += 1

    # --- 모드 -----------------------------------------------------------
    def set_mode(self, mode, why=""):
        """모드를 바꾼다. 바뀐 때와 까닭을 적고 /mode 로 알린다."""
        self.spin()
        now = self.now()
        print(f"[{now:7.1f}초] {self.mode:<6} → {mode:<6} ({LABELS[mode]})  {why}", flush=True)
        self.history.append({"time": round(now, 2), "from": self.mode, "mode": mode, "why": why, "truth": [round(v, 4) for v in self.truth()]})
        self.mode = mode
        self.mode_pub.publish(String(data=mode))

    def note(self, **values):
        """그래프용 기록 한 줄: 지금 모드 · 시각 · 참값과 잰 값."""
        truth = self.truth()
        self.samples.append({"time": round(self.now(), 3), "mode": self.mode, "truth": [round(v, 4) for v in truth],
                             **{key: round(value, 4) for key, value in values.items()}})

    # --- /cmd_vel -------------------------------------------------------
    def command(self, linear, angular):
        """이 프로그램이 /cmd_vel 을 낸다. Nav2 모드에서는 부르지 않는다."""
        if self.mode == "NAV2":
            raise DockError("Nav2 모드에서는 이 프로그램이 /cmd_vel 을 내지 않음")
        self.publish(linear, angular)
        if (float(linear), float(angular)) != (0.0, 0.0):
            self.pending.append((float(linear), float(angular)))

    def halt(self):
        for _ in range(3):
            self.command(0.0, 0.0)
            self.spin(0.05)

    def nav_busy(self, wait=2.0):
        """Nav2 에 아직 끝나지 않은 목표가 있나. 방금 끝난 목표의 상태가 늦게 올 수 있어 wait 초까지 기다려 본다."""
        if not self.nav.wait_for_server(timeout_sec=wait):                     # Nav2 가 떠 있지 않다 (도킹만 해 볼 때)
            return False
        end = time.monotonic() + wait
        while self.status_sub.get_publisher_count() == 0 and time.monotonic() < end:
            self.spin(0.05)                                                    # 막 뜬 노드는 상태 토픽과 아직 이어지지 않았을 수 있다
        busy = (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING, GoalStatus.STATUS_CANCELING)
        end = time.monotonic() + wait
        while True:
            self.spin(0.05)
            if not any(status in busy for status in self.nav_goals):
                return False
            if time.monotonic() > end:
                return True

    def others_moving(self):
        """내가 낸 것 말고 0 이 아닌 /cmd_vel 이 몇 개 들렸나 (= 다른 노드가 로봇을 움직이고 있나)."""
        self.spin()
        return self.foreign

    def wait_still(self, timeout=10.0):
        """오도메트리의 속도가 0 에 가까운 상태가 0.5초 이어질 때까지 기다린다. 걸린 시간 [s]."""
        start = since = self.now()
        while self.now() - start < timeout:
            self.command(0.0, 0.0)
            self.spin(0.05)
            linear, angular = self.speed()
            if abs(linear) > STILL_LINEAR or abs(angular) > STILL_ANGULAR:
                since = self.now()
            elif self.now() - since >= STILL_TIME:
                return self.now() - start
        raise DockError("로봇이 멈추지 않음")

    # --- 오도메트리로 움직이기 ------------------------------------------
    def turn(self, angle, speed=0.25):
        """제자리에서 angle [rad] 만큼 돈다 (+ 왼쪽). 돈 각은 오도메트리로 잰다."""
        begin, sign = self.odom()[2], 1.0 if angle >= 0 else -1.0
        deadline, done = self.now() + abs(angle) / speed + 3.0, 0.0
        while self.now() < deadline:
            done = angle_difference(self.odom()[2], begin)
            if sign * done >= abs(angle) - 0.01:
                break
            self.command(0.0, sign * speed)
            self.note()
            self.spin(0.05)
        self.halt()
        return done

    def drive(self, distance, speed=0.08):
        """곧게 distance [m] 만큼 간다 (- 면 뒤로). 간 거리는 오도메트리로 잰다."""
        if abs(distance) < 0.005:
            return 0.0
        x0, y0, _ = self.odom()
        sign, deadline, travelled = 1.0 if distance >= 0 else -1.0, self.now() + abs(distance) / speed + 5.0, 0.0
        while self.now() < deadline:
            x, y, _ = self.odom()
            travelled = math.hypot(x - x0, y - y0)
            if travelled >= abs(distance) - 0.004:
                break
            slow = abs(distance) - travelled <= 0.05                       # 마지막 5 cm 는 천천히
            self.command(sign * (max(0.03, speed * 0.4) if slow else speed), 0.0)
            self.note()
            self.spin(0.05)
        self.halt()
        return sign * travelled

    def sidestep(self, lateral):
        """옆으로 lateral [m] 만큼 옮긴다 (+ 왼쪽): 90° 돌고, 직진하고, 다시 90° 돌아온다."""
        sign = 1.0 if lateral >= 0 else -1.0
        self.turn(sign * math.pi / 2)
        self.drive(abs(lateral))
        self.turn(-sign * math.pi / 2)

    # --- 마커 -----------------------------------------------------------
    def fixes(self, offsets, max_age=0.8, since=None):
        """지금 쓸 수 있는 마커들 {id: (앞, 왼쪽, 법선)}. 찍은 지 max_age 초가 넘었거나 since 보다 먼저 찍은 영상은 버린다."""
        self.spin()
        if self.markers is None:
            return {}
        stamp, found, _ = self.markers
        if self.now() - stamp > max_age or (since is not None and stamp < since):
            return {}
        wanted = list(offsets)[:1] if self.options["single"] else list(offsets)
        return {marker: found[marker] for marker in wanted if marker in found}

    def find_markers(self, offsets, timeout=6.0):
        """서 있는 채로 마커를 기다린다. 안 보이면 제자리에서 좌우로 돌며 찾는다. 선 뒤에 찍은 영상만 받는다."""
        need = 1 if self.options["single"] else 2
        since, end = self.now(), self.now() + timeout
        while self.now() < end:
            found = self.fixes(offsets, since=since)
            if len(found) >= need:
                return found
            self.spin(0.05)
        print(f"            마커 {list(offsets)} 가 안 보임: 제자리에서 돌며 찾음", flush=True)
        for speed, seconds in ((0.25, 2.5), (-0.25, 5.0), (0.25, 2.5)):         # 왼쪽 36° → 오른쪽 72° → 가운데로
            end = self.now() + seconds
            while self.now() < end:
                if len(self.fixes(offsets)) >= need:
                    self.wait_still()                                          # 돌다가 찾았으면 서서 다시 잰다
                    return self.find_markers(offsets, timeout)
                self.command(0.0, speed)
                self.spin(0.05)
        self.halt()
        raise DockError(f"마커 {list(offsets)} 를 찾지 못함")

    # --- 도킹 -----------------------------------------------------------
    def dock(self, station, aim=0.0, tolerance=None):
        """지금 자리(스테이션 앞)에서 도킹한다. 선 자리를 (마커 면까지의 거리, 축이 왼쪽으로 벗어난 거리, 축과 나란해지려면 돌 각) 으로 돌려준다.

        이 세 값이 도킹 모드가 다음 모드(팔 작업)에 건네는 것이다. 팔은 이것으로 테이블 위 물체의 자리를 셈한다.
        aim: 축이 로봇의 왼쪽으로 이만큼 [m] 떨어지게 선다 (0 이면 축 위). 물체가 팔이 닿는 범위에 오도록 비켜 설 때 쓴다.
        tolerance: 옆으로 이보다 더 어긋나 있으면 옆걸음을 한다. 주지 않으면 DOCK 의 max_lateral_error.
        """
        tolerance = DOCK["max_lateral_error"] if tolerance is None else tolerance
        offsets = station_markers(station)
        # 1. 멈춤 확인: 움직이는 중에 찍은 영상으로 자리를 재면 안 된다
        self.set_mode("SETTLE", "Nav2 가 끝났는지, 베이스가 멈췄는지 봄")
        if self.nav_busy():                                                    # 아직 아무것도 내지 않는다
            raise DockError("Nav2 에 끝나지 않은 목표가 있음: Nav2 가 끝난 뒤에 도킹할 것")
        self.foreign = 0
        self.spin(0.5)                                                         # 0.5초 동안 /cmd_vel 을 듣기만 한다
        if self.foreign > 0:
            raise DockError(f"다른 노드가 /cmd_vel 을 내고 있음 (0.5초에 {self.foreign}개)")
        if self.options["settle"]:
            waited = self.wait_still()
            print(f"            멈춤: {waited:.1f}초 걸림", flush=True)
        self.spin()
        self.foreign = 0                                                       # 여기부터 다른 노드가 /cmd_vel 을 내면 도킹을 멈춘다
        # 2. 마커 찾기 → 옆으로 벗어난 만큼 옆걸음
        self.set_mode("SEARCH", f"마커 {list(offsets)} 를 찾음")
        for attempt in range(3):
            found = self.find_markers(offsets)
            along, side, turn = station_geometry(found, offsets)
            print(f"            마커 {sorted(found)}: 면까지 {along:.3f} m, 축이 왼쪽으로 {side:+.3f} m, 돌아야 할 각 {math.degrees(turn):+.1f}°", flush=True)
            self.note(along=along, side=side, turn=turn)
            if abs(side - aim) <= tolerance or not self.options["align"] or attempt == 2:
                break
            where = f"설 자리(축이 왼쪽으로 {aim * 100:+.0f} cm 인 곳)에서" if aim else "축에서"
            self.set_mode("ALIGN", f"{where} {abs(side - aim) * 100:.1f} cm 벗어남 (한계 {tolerance * 100:.0f} cm): 옆걸음")
            if abs(turn) > 0.03:
                self.turn(turn)
            self.sidestep(side - aim)
            self.wait_still()
            self.set_mode("SEARCH", "옆걸음 뒤 다시 잼")
        if abs(side - aim) > 2 * tolerance and self.options["align"]:
            raise DockError(f"옆걸음을 하고도 {'설 자리' if aim else '축'}에서 {abs(side - aim) * 100:.1f} cm 벗어나 있음")
        # 3. 마커를 보며 다가가기
        self.set_mode("SERVO", f"마커를 보며 면까지 {DOCK['visual_end_dx']:.2f} m 가 될 때까지")
        last, last_pose, end, shown, missed, lost = None, None, self.now() + DOCK["timeout"], -1.0, 0, None
        while True:
            if self.now() > end:
                self.halt()
                raise DockError("시간 안에 다가가지 못함")
            if self.others_moving() > 0:
                self.halt()
                raise DockError("도킹 중에 다른 노드가 /cmd_vel 을 냄")
            found = self.fixes(offsets)
            if len(found) < (1 if self.options["single"] else 2):
                if last is not None and last[0] <= DOCK["visual_end_dx"] + 0.15:   # 거의 다 와서 한 장이 화면을 벗어남: 마지막으로 잰 값으로 간다
                    why = "마커 한 장이 화면을 벗어남"
                    break
                if lost is None:
                    lost = self.now()
                    if last is not None:
                        print(f"            마커가 다 보이지 않게 됨: 마지막으로 잰 면까지의 거리 {last[0]:.3f} m", flush=True)
                if self.now() - lost > 5.0:
                    self.halt()
                    raise DockError("마커가 5초째 다 보이지 않음" + (f" (마지막으로 잰 면까지의 거리 {last[0]:.3f} m)" if last else ""))
                missed += 1                                                    # 마커가 다 보이지 않으면 서서 다음 영상을 기다린다
                self.command(0.0, 0.0)
                self.spin(0.1)
                continue
            lost = None
            along, side, turn = station_geometry(found, offsets)
            if last is not None and abs(side - last[1]) > 0.08:
                side = last[1]                                                 # 한 번 튄 값은 버린다
            last, last_pose = (along, side, turn), self.markers[2]
            if along <= DOCK["visual_end_dx"]:
                why = f"면까지 {along:.3f} m"
                break
            linear = min(DOCK["max_speed"], max(0.03, DOCK["gain"] * (along - DOCK["target_dx"])))
            angular = DOCK["lateral_gain"] * (side - aim) / max(along, 0.3) + DOCK["heading_gain"] * turn
            angular = max(-0.3, min(0.3, angular))
            self.command(linear, angular)
            self.note(along=along, side=side, turn=turn, linear=linear, angular=angular)
            if self.now() - shown >= 0.5:
                shown = self.now()
                print(f"            면까지 {along:.3f} m · 축 {side:+.3f} m · 각 {math.degrees(turn):+5.1f}°  →  전진 {linear:.2f} m/s · 회전 {angular:+.2f} rad/s"
                      + (f"   (마커를 기다린 횟수 {missed})" if missed else ""), flush=True)
            self.spin(0.1)
        self.halt()
        # 4. 마커를 믿지 않고 오도메트리로 마지막 구간
        along, side, turn = last
        x, y, _ = self.odom()
        travelled = math.hypot(x - last_pose[1], y - last_pose[2]) if last_pose else 0.0     # 그 영상을 찍은 뒤 이미 간 거리
        remaining = along - travelled - DOCK["target_dx"]
        self.set_mode("BLIND", f"{why}. 마지막으로 잰 값: 면까지 {along:.3f} m · 축 {side:+.3f} m · 각 {math.degrees(turn):+.1f}°. "
                      f"그 뒤 {travelled * 100:.1f} cm 를 이미 갔으므로 {remaining * 100:.1f} cm 를 오도메트리로 감")
        if abs(turn) > 0.02:                                   # 1.1° 넘게 틀어져 있으면 제자리에서 돌아 축과 나란히 선다
            self.turn(turn)
        self.drive(remaining, DOCK["blind_speed"])
        self.wait_still()
        # 5. 건넬 값: 마지막 영상을 찍은 뒤 로봇이 움직인 만큼(오도메트리)을 그때 잰 값에 더한다
        if last_pose:
            x, y, yaw = self.odom()
            dx, dy = x - last_pose[1], y - last_pose[2]
            ahead = dx * math.cos(last_pose[3]) + dy * math.sin(last_pose[3])        # 찍을 때의 로봇 기준으로 앞 · 왼쪽으로 간 거리
            left = -dx * math.sin(last_pose[3]) + dy * math.cos(last_pose[3])
            side = side - (-ahead * math.sin(turn) + left * math.cos(turn))        # 축에서 틀어진 채(-turn) 간 만큼 옆으로 옮겨졌다
            turn = turn - angle_difference(yaw, last_pose[3])                         # 그동안 돈 만큼 뺀다
        self.set_mode("DOCKED", f"면까지 {DOCK['target_dx']:.2f} m 로 봄, 축이 왼쪽으로 {side:+.3f} m, 축과 나란해지려면 {math.degrees(turn):+.1f}°")
        return DOCK["target_dx"], side, turn

    def undock(self):
        """뒤로 물러난다. 테이블 바로 앞은 Nav2 의 지도에서 막힌 곳 둘레라, 여기서 Nav2 를 부르지 않는다."""
        self.set_mode("UNDOCK", f"오도메트리로 {DOCK['undock_distance']:.2f} m 뒤로")
        moved = self.drive(-DOCK["undock_distance"])
        self.wait_still()
        return moved

    def navigate(self, station):
        """Nav2 모드: 스테이션 앞까지. 이 동안 이 프로그램은 /cmd_vel 을 내지 않는다."""
        goal = STATIONS[station]["goal"]
        label = STATIONS[station]["label"] + ("" if station == "home" else " 앞")
        self.set_mode("NAV2", f"{label} ({goal[0]:.2f}, {goal[1]:.2f}, {math.degrees(goal[2]):.0f}°) 으로")
        result, seconds = self.go_to(*goal, near=NEAR if self.options.get("near", True) else None)
        truth, _, arrival, _ = self.arrival(goal)
        print(f"            Nav2: {result} ({seconds:.1f}초). 도착 오차 {arrival[0] * 100:.1f} cm · {math.degrees(arrival[1]):+.1f}° (참값으로 채점)", flush=True)
        if result not in ("도착", "가까이서 멈춤"):
            raise DockError(f"Nav2 가 도착하지 못함: {result}")
        if result == "가까이서 멈춤":                                           # Nav2 의 목표는 거뒀다. 방향이 많이 틀어져 있으면 오도메트리로 돌려 스테이션을 본다
            error = angle_difference(goal[2], self.estimate()[2])
            if abs(error) > math.radians(10.0):
                self.set_mode("ALIGN", f"Nav2 가 목표 가까이에서 나아가지 못해 목표를 거둠. 방향을 {math.degrees(error):+.0f}° 돌림")
                self.turn(error)
                self.halt()
        return arrival

    def go_and_dock(self, station, tries=2, aim=0.0, tolerance=None):
        """Nav2 → 도킹. 도킹에 실패하면 물러났다가 Nav2 로 다시 앞에 선 뒤 한 번 더 한다. aim · tolerance 는 dock 에 그대로 건넨다."""
        for attempt in range(1, tries + 1):
            arrival = self.navigate(station)
            try:
                return arrival, self.dock(station, aim, tolerance)
            except DockError as error:
                self.halt()
                self.set_mode("FAILED", f"{error} ({attempt}/{tries}번째)")
                if attempt == tries:
                    raise
                self.drive(-0.3)

    def measure(self, station, count=100):
        """서 있는 채로 영상 count 장을 재어, 마커 두 장을 쓸 때와 한 장만 쓸 때의 값을 견준다."""
        offsets = station_markers(station)
        first, seen, rows = list(offsets)[0], None, {"두 장": [], "한 장": []}
        while len(rows["두 장"]) < count:
            self.spin(0.05)
            if self.markers is None or self.markers[0] == seen:
                continue
            seen, found = self.markers[0], self.markers[1]
            if all(marker in found for marker in offsets):
                rows["두 장"].append(station_geometry(found, offsets))
                rows["한 장"].append(station_geometry({first: found[first]}, offsets))
        forward, left, yaw = self.score(station)
        truth = (DOCK["target_dx"] - forward, -left, -yaw)
        scale = (1000.0, 1000.0, math.degrees(1.0))
        print(f"영상 {count}장 (서 있는 채로).  {'':<8}{'면까지 [mm]':>16}{'축 [mm]':>18}{'각 [°]':>16}", flush=True)
        print(f"  {'참값':<26}" + "".join(f"{value * k:>18.1f}" for value, k in zip(truth, scale)), flush=True)
        for label, values in rows.items():
            columns = list(zip(*values))
            mean = [sum(column) / len(column) for column in columns]
            spread = [math.sqrt(sum((v - m) ** 2 for v in column) / len(column)) for column, m in zip(columns, mean)]
            print(f"  마커 {label}: 평균 ± 흩어짐   " + "".join(f"{m * k:>10.1f} ± {d * k:<5.1f}" for m, d, k in zip(mean, spread, scale)), flush=True)
        return rows

    def score(self, station):
        """도킹한 자리의 채점 (참값): (앞뒤 오차 [m], 좌우 오차 [m], 방향 오차 [rad]). + 는 더 들어감 · 왼쪽 · 왼쪽으로 돎."""
        self.spin(0.3)
        truth, ideal = self.truth(), docked_pose(station)
        dx, dy = truth[0] - ideal[0], truth[1] - ideal[1]
        cos, sin = math.cos(ideal[2]), math.sin(ideal[2])
        return dx * cos + dy * sin, -dx * sin + dy * cos, angle_difference(truth[2], ideal[2])

    def save(self, path, **more):
        with open(path, "w") as file:
            json.dump({"history": self.history, "samples": self.samples, **more}, file, ensure_ascii=False)
        print("저장:", path, flush=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("station", nargs="?", choices=[name for name in STATIONS if name != "home"])
    parser.add_argument("--nav", action="store_true", help="먼저 Nav2 로 스테이션 앞까지 감")
    parser.add_argument("--undock", action="store_true", help="도킹한 뒤 뒤로 물러남")
    parser.add_argument("--tour", type=int, metavar="N", help="스테이션 셋을 N 바퀴: Nav2 → 도킹 → 물러나기")
    parser.add_argument("--aim", type=float, default=0.0, metavar="M", help="축이 로봇의 왼쪽으로 M [m] 떨어지게 선다 (기본 0 = 축 위). 이때는 옆으로 2 cm 넘게 어긋나면 옆걸음을 함")
    parser.add_argument("--no-align", action="store_true", help="비교용: 옆걸음을 하지 않음")
    parser.add_argument("--no-settle", action="store_true", help="비교용: 멈췄는지 확인하지 않음")
    parser.add_argument("--strict-nav", action="store_true", help="비교용: Nav2 가 도착이라고 할 때까지 기다림 (목표 가까이에서 서 있어도 넘어가지 않음)")
    parser.add_argument("--single", action="store_true", help="비교용: 마커 한 장만 씀")
    parser.add_argument("--visual-end", type=float, metavar="M", help="비교용: 마커를 보며 다가가는 것을 끝내는 거리 [m] (기본 0.80)")
    parser.add_argument("--save", metavar="JSON", help="모드와 잰 값의 기록을 저장")
    parser.add_argument("--measure", type=int, metavar="N", help="움직이지 않고 영상 N 장을 재어 마커 두 장과 한 장을 견줌")
    args = parser.parse_args()
    if (args.station is None) == (args.tour is None):
        parser.error("스테이션 이름이나 --tour 가운데 하나를 줄 것")

    rclpy.init()
    robot = Docker()
    robot.options = {"align": not args.no_align, "settle": not args.no_settle, "single": args.single, "near": not args.strict_nav}
    if args.visual_end is not None:
        DOCK["visual_end_dx"] = args.visual_end
    robot.wait_for(lambda: robot.odom_msg is not None and robot.truth_msg is not None, 60.0, "/odom · /ground_truth (시뮬레이터)")
    rows = []
    try:
        if args.measure:
            measured = robot.measure(args.station, args.measure)
            rows.append({"station": args.station, "measure": {key: values for key, values in measured.items()}})
        elif args.tour is None:
            before = robot.score(args.station)
            if args.nav:
                robot.ready()
                _, (_, side, turn) = robot.go_and_dock(args.station, aim=args.aim, tolerance=AIM_TOLERANCE if args.aim else None)   # 실패하면 Nav2 로 다시 앞에 서서 한 번 더
            else:
                _, side, turn = robot.dock(args.station, args.aim, AIM_TOLERANCE if args.aim else None)
            forward, left, yaw = robot.score(args.station)
            print(f"채점 (참값 - 도킹 자리): 앞뒤 {forward * 1000:+.1f} mm, 좌우 {left * 1000:+.1f} mm, 방향 {math.degrees(yaw):+.2f}°", flush=True)
            print(f"     로봇이 아는 값:      좌우 {-side * 1000:+.1f} mm, 방향 {math.degrees(-turn):+.2f}°   "
                  f"(참값과의 차이 {(-side - left) * 1000:+.1f} mm, {math.degrees(-turn - yaw):+.2f}°)"
                  + (f"   설 자리는 좌우 {-args.aim * 1000:+.0f} mm" if args.aim else ""), flush=True)
            rows.append({"station": args.station, "before": before, "after": (forward, left, yaw), "side": side, "turn": turn})
            if args.undock:
                robot.undock()
                robot.set_mode("IDLE", "물러남. 이제 Nav2 에 다음 목적지를 줄 수 있음")
        else:
            robot.ready()
            print("Nav2 로 도착했을 때와 도킹을 마쳤을 때의 오차 (참값 - 도킹 자리 기준. Nav2 는 앞뒤로 0.585 m 덜 간 자리가 목표)", flush=True)
            for lap in range(1, args.tour + 1):
                for station in ("drinks", "consult_1", "consult_2"):
                    arrival, (_, side, turn) = robot.go_and_dock(station)
                    forward, left, yaw = robot.score(station)
                    rows.append({"lap": lap, "station": station, "nav": [arrival[0], arrival[1]], "after": [forward, left, yaw], "side": side, "turn": turn})
                    print(f"  {lap}바퀴 {station:<10} Nav2 도착 오차 {arrival[0] * 100:5.1f} cm · {math.degrees(arrival[1]):+5.1f}°   →   "
                          f"도킹 뒤 앞뒤 {forward * 1000:+6.1f} mm · 좌우 {left * 1000:+6.1f} mm (로봇이 아는 값 {-side * 1000:+6.1f}) · "
                          f"방향 {math.degrees(yaw):+5.2f}° (아는 값 {math.degrees(-turn):+5.2f})", flush=True)
                    robot.undock()
            robot.set_mode("IDLE", "끝")
    except DockError as error:
        if robot.mode != "FAILED":                       # go_and_dock 은 실패를 이미 적었다
            robot.set_mode("FAILED", str(error))
        robot.halt()
    if args.save:
        robot.save(args.save, rows=rows)
    robot.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
