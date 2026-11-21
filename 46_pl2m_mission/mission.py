"""강의 46: 음료대의 컵을 집어 상담 테이블에 가져다 놓습니다. Nav2 · 도킹 · 팔 작업을 모드로 이어 붙인 임무입니다.

    source /opt/ros/jazzy/setup.bash
    source ~/pinklab_robotworld2026/install/setup.bash
    python3 46_pl2m_mission/mission.py                                  # 분홍 컵을 음료대 → 상담 테이블 1 로
    python3 46_pl2m_mission/mission.py --cup cup_yellow --to consult_2  # 노란 컵을 상담 테이블 2 로
    python3 46_pl2m_mission/mission.py --only pick                      # 음료대에 도킹한 자리에서 집기만 (시뮬레이터를 --start drinks --docked 로)
    python3 46_pl2m_mission/mission.py --only place                     # 상담 테이블에 도킹한 자리에서 놓는 동작만 (집지 않은 채)
    python3 46_pl2m_mission/mission.py --no-dock                        # 비교용: 도킹하지 않고 Nav2 로 도착한 자리에서 바로 집는다
    python3 46_pl2m_mission/mission.py --save run.json                  # 모드의 기록을 저장

모드가 바뀌는 순서 (괄호 안은 그 모드를 끝내는 조건)
    ARM      두 팔을 이동 자세로            (관절이 이동 자세에 와 있음)
    NAV2     음료대 앞으로                  (Nav2 가 도착이라고 함)
    SETTLE → SEARCH → (ALIGN) → SERVO → BLIND → DOCKED      강의 45 의 도킹
    ARM      컵을 집어 운반 자세로          (손가락이 컵에 막혀 멈춤, 팔이 운반 자세에 와 있음)
    UNDOCK   뒤로 물러남                    (오도메트리로 0.75 m)
    NAV2     상담 테이블 앞으로
    SETTLE → … → DOCKED
    ARM      컵을 내려놓고 이동 자세로
    UNDOCK → NAV2 처음 자리로 → IDLE

모드 사이에 건네는 것
    NAV2 → 도킹      없음. 도킹은 지도를 쓰지 않고 마커만 본다
    도킹 → ARM       마커 면까지의 거리(0.43 m), 축이 옆으로 벗어난 거리(side), 축과 틀어진 각(turn). 팔은 이 셋으로 컵과 놓을 자리의 위치를 셈한다
    ARM → UNDOCK     팔이 운반 자세인지. 아니면 베이스를 움직이지 않는다
"""
import argparse
import math
import os
import sys

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import CollisionObject, Constraints, MoveItErrorCodes
from moveit_msgs.srv import GetCartesianPath
from shape_msgs.msg import SolidPrimitive

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
for folder in ("42_pl2m_moveit", "43_pl2m_pick", "44_pl2m_nav", "45_pl2m_dock"):
    sys.path.insert(0, os.path.join(HERE, "..", folder))
import dock
from booth import STATIONS
from dock import DockError, Docker
from layout import CARRY_TIP, CUP_DIAMETER, CUP_HEIGHT, CUP_MASS, CUPS, FURNITURE, GRASP, PLACE, slot, to_base
from markers import DOCK, docked_pose
from move_arm import BASE, ERRORS, named_state, tip
from pick_cup import Picker, pose_at

dock.LABELS["ARM"] = "팔 작업"


class ArmError(Exception):
    """팔 작업을 계속할 수 없는 까닭."""


def shape(name, kind, center, size, yaw=0.0):
    """MoveIt 에 알릴 장애물 하나. 로봇 기준(base_footprint)의 자리로 넣는다."""
    item = CollisionObject(id=name, operation=CollisionObject.ADD)
    item.header.frame_id = BASE
    if kind == "box":
        item.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[float(v) for v in size])]
    else:
        item.primitives = [SolidPrimitive(type=SolidPrimitive.CYLINDER, dimensions=[float(size[1]), float(size[0])])]     # (높이, 반지름)
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = (float(v) for v in center)
    pose.orientation.z, pose.orientation.w = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
    item.primitive_poses = [pose]
    return item


class Mission:
    """베이스(Docker)와 팔(Picker)을 모드에 따라 번갈아 부린다. 한 번에 한쪽만 움직인다."""

    def __init__(self):
        self.base, self.arm = Docker("mission_base"), Picker()
        self.holding = None                     # 손에 든 컵: (팔, 이름)

    def mode(self, name, why=""):
        self.base.set_mode(name, why)

    # --- 팔 -------------------------------------------------------------
    def joints(self, side, pose):
        """이름 붙은 자세의 관절 값 {관절 이름: 각도}."""
        return {c.joint_name: c.position for c in named_state(side, pose)}

    def arm_at(self, side, pose, tolerance=0.06):
        """팔이 이름 붙은 자세에 와 있나 (잰 관절 각도로 확인)."""
        self.arm.settle(0.3)
        return all(abs(self.arm.measured.get(name, 9.0) - value) <= tolerance for name, value in self.joints(side, pose).items())

    def arm_to(self, side, pose, tries=3):
        """팔을 이름 붙은 자세로 보낸다."""
        if self.arm_at(side, pose):
            return
        constraints = Constraints(joint_constraints=named_state(side, pose))
        for _ in range(tries):
            code = self.arm.move(f"{side}_arm", constraints).error_code.val
            if code == MoveItErrorCodes.SUCCESS:
                self.arm.settle(0.5)
                return
        raise ArmError(f"{side} 팔을 {pose} 자세로 보내지 못함 ({ERRORS.get(code, code)})")

    def straight(self, side, x, y, z, what):
        fraction = self.arm.straight(side, x, y, z)
        if fraction < 0.9:
            raise ArmError(f"{what}: 직선의 {fraction * 100:.0f} % 만 갈 수 있음")

    def lower(self, side, x, y, z):
        """컵을 상판까지 내리는 마지막 구간. 컵이 상판에 닿으러 가는 길이라 MoveIt 의 충돌 검사를 끄고 직선으로 간다.

        충돌 검사를 켠 채로는 상판 4 mm 위까지 가는 길이 마지막 1 cm 에서 끊길 때가 있었다 (아홉 번에 두 번).
        """
        request = GetCartesianPath.Request()
        request.header.frame_id = BASE
        request.start_state.is_diff = True
        request.group_name, request.link_name = f"{side}_arm", tip(side)
        request.waypoints = [pose_at(x, y, z)]
        request.max_step = 0.01
        request.avoid_collisions = False
        request.max_velocity_scaling_factor = request.max_acceleration_scaling_factor = 0.2
        answer = self.arm.call(self.arm.cartesian, request)
        if answer.error_code.val != MoveItErrorCodes.SUCCESS or answer.fraction < 0.9:
            raise ArmError(f"내리기: 직선의 {answer.fraction * 100:.0f} % 만 갈 수 있음")
        sent = self.arm.execute.send_goal_async(ExecuteTrajectory.Goal(trajectory=answer.solution))
        rclpy.spin_until_future_complete(self.arm, sent)
        done = sent.result().get_result_async()
        rclpy.spin_until_future_complete(self.arm, done)
        if done.result().result.error_code.val != MoveItErrorCodes.SUCCESS:
            raise ArmError("내리기: 궤적을 끝까지 따라가지 못함")

    def add_furniture(self, station, where, more=()):
        """그 스테이션의 가구를 로봇 기준 자리로 MoveIt 에 넣는다. where = 도킹이 알려 준 (side, turn)."""
        items = []
        for name, kind, center, size in FURNITURE[station]:
            turned = kind == "box" and abs(math.sin(STATIONS[station]["goal"][2])) > 0.7       # 로봇이 ±y 쪽을 보면 상자의 가로세로가 바뀐다
            items.append(shape(name, kind, to_base(station, center, *where), (size[1], size[0], size[2]) if turned else size, yaw=where[1]))
        self.arm.apply(world=items + list(more))
        return [item.id for item in items]

    def remove(self, names):
        self.arm.apply(world=[CollisionObject(id=name, operation=CollisionObject.REMOVE) for name in names])

    # --- 집기 · 놓기 ----------------------------------------------------
    def pick(self, station, cup, where):
        """도킹한 자리에서 컵을 집어 운반 자세까지. 강의 43 의 순서 그대로, 자리만 도킹이 알려 준 값 where = (side, turn) 으로 셈한다."""
        x, y, z = to_base(station, CUPS[cup], *where)
        side = "left" if to_base(station, CUPS[cup])[1] >= -0.01 else "right"      # 축의 왼쪽 컵과 가운데 컵은 왼팔, 오른쪽 컵은 오른팔
        tip_z = z + GRASP["z_offset"]
        print(f"            {cup}: 로봇 기준 앞 {x:.3f} · 왼쪽 {y:+.3f} · 높이 {z:.3f} m  →  {'왼팔' if side == 'left' else '오른팔'}", flush=True)
        names = self.add_furniture(station, where, [shape(cup, "cylinder", (x, y, z), (CUP_DIAMETER / 2, CUP_HEIGHT))])
        self.arm.grip(side, GRASP["open"])
        for attempt in range(1, 4):
            # 같은 손끝 자세에도 팔 모양은 여럿이다 (강의 43). MoveIt 이 고른 팔 모양에서 직선 접근이 안 되면, 접근 시작 자리부터 다시 잡는다
            if not self.arm.to_pose(side, x + GRASP["pre_offset_x"], y, tip_z):
                raise ArmError("접근 시작 자리로 가지 못함")
            fraction = self.arm.straight(side, x, y, tip_z)
            if fraction >= 0.9:
                break
            print(f"            이 팔 모양에서는 직선 접근의 {fraction * 100:.0f} % 만 갈 수 있음: 접근 시작 자리를 다시 잡음 ({attempt}/3)", flush=True)
            self.arm_to(side, "transport")
        else:
            raise ArmError("컵으로 곧게 접근할 수 있는 팔 모양을 찾지 못함")
        position, _, _ = self.arm.grip(side, GRASP["close"])
        if not GRASP["close_min"] <= position <= GRASP["close_max"]:
            self.arm.grip(side, GRASP["open"])
            raise ArmError(f"컵을 쥐지 못함: 손가락이 {position * 1000:.1f} mm 에서 멈춤")
        print(f"            쥐었음: 손가락이 {position * 1000:.1f} mm 에서 멈춤. 로봇에 {CUP_MASS:g} kg 을 알림", flush=True)
        self.arm.tell_payload(side, CUP_MASS)
        self.arm.attach(side, cup)
        self.holding = (side, cup)
        self.straight(side, x, y, tip_z + GRASP["lift_z"], "들기")
        back = max(x + GRASP["retreat_x"], CARRY_TIP[0] + 0.03)                       # 운반 자리보다 몸 쪽으로는 가지 않는다
        self.straight(side, back, y, tip_z + GRASP["lift_z"], "뒤로 빼기")
        carry_y = CARRY_TIP[1] if side == "left" else -CARRY_TIP[1]
        try:
            self.straight(side, CARRY_TIP[0], carry_y, CARRY_TIP[2], "운반 자리로 당기기")   # 직선이면 손의 방향이 그대로라 컵이 서 있다
        except ArmError as error:
            print(f"            {error}: 운반 자세로 바로 감", flush=True)
        self.arm_to(side, "carry")
        self.remove(names)
        return side

    def place(self, station, where, side=None):
        """도킹한 자리에서 컵을 든 팔의 정면에 내려놓고 이동 자세까지. where = 도킹이 알려 준 (side, turn)."""
        side = side or (self.holding[0] if self.holding else "left")
        x, y, z = to_base(station, slot(station, side), *where)
        tip_z = z + PLACE["drop_z"] + GRASP["z_offset"]
        print(f"            놓을 자리: 로봇 기준 앞 {x:.3f} · 왼쪽 {y:+.3f} · 컵 가운데의 높이 {z:.3f} m", flush=True)
        names = self.add_furniture(station, where)
        # 집을 때 온 길을 거꾸로 간다: 운반 자리 → 뒤쪽 위 → 놓을 자리 위 → 아래. 모두 직선이라 손의 방향이 그대로고 컵이 서 있다
        self.arm_to(side, "carry")
        back = max(x + GRASP["retreat_x"], CARRY_TIP[0] + 0.03)
        self.straight(side, back, y, tip_z + GRASP["lift_z"], "놓을 자리의 뒤로")
        self.straight(side, x, y, tip_z + GRASP["lift_z"], "놓을 자리 위로")
        self.lower(side, x, y, tip_z)
        self.arm.grip(side, GRASP["open"])
        if self.holding:
            self.arm.attach(side, self.holding[1], attach=False)
            self.remove([self.holding[1]])
        self.arm.tell_payload(side, 0.0)
        print("            놓았음: 그리퍼를 열고, 로봇에 알린 무게를 0 kg 으로 되돌림", flush=True)
        self.holding = None
        carry_y = CARRY_TIP[1] if side == "left" else -CARRY_TIP[1]
        try:                                                                          # 손을 뒤로 빼고 운반 자리로
            self.straight(side, back, y, tip_z, "손 빼기")
            self.straight(side, CARRY_TIP[0], carry_y, CARRY_TIP[2], "운반 자리로")
        except ArmError as error:                                                     # 컵은 이미 놓았다. 직선이 안 되면 계획으로 돌아간다
            print(f"            {error}: 이동 자세로 바로 감", flush=True)
        self.arm_to(side, "transport")
        self.remove(names)
        return side

    # --- 모드를 잇기 ----------------------------------------------------
    def leave(self, side, pose):
        """도킹한 자리를 떠난다. 팔이 정해진 자세가 아니면 베이스를 움직이지 않는다."""
        if not self.arm_at(side, pose):
            raise ArmError(f"{side} 팔이 {pose} 자세가 아님: 베이스를 움직이지 않음")
        self.base.undock()

    def run(self, cup, source, target, use_dock=True):
        base = self.base
        self.mode("ARM", "두 팔을 이동 자세(transport)로")
        for side in ("left", "right"):
            self.arm_to(side, "transport")
        # 1. 음료대로 가서 도킹, 집기
        if use_dock:
            _, (_, *where) = base.go_and_dock(source)          # 도킹이 건네는 값: (side, turn)
        else:
            base.navigate(source)
            base.set_mode("SETTLE", "도킹하지 않음: 멈춘 것만 확인")
            base.wait_still()
            # 도킹이 알려 줄 값이 없으므로, 도착한 자리가 도킹한 자리보다 0.585 m 뒤일 것이라고만 보고 그만큼 오도메트리로 나아간다
            base.set_mode("BLIND", f"마커를 보지 않고 {STATIONS[source]['label']} 쪽으로 오도메트리 직진")
            ideal, goal = docked_pose(source), STATIONS[source]["goal"]
            base.drive(math.hypot(ideal[0] - goal[0], ideal[1] - goal[1]), DOCK["blind_speed"])
            base.wait_still()
            where = (0.0, 0.0)
        forward, left, yaw = base.score(source)
        print(f"            채점 (참값): 서 있는 자리가 앞뒤 {forward * 1000:+.1f} mm · 좌우 {left * 1000:+.1f} mm · 방향 {math.degrees(yaw):+.2f}° 어긋남", flush=True)
        self.mode("ARM", f"베이스는 서 있음. {cup} 을(를) 집음")
        side = self.pick(source, cup, where)
        self.leave(side, "carry")
        # 2. 상담 테이블로 가서 도킹, 놓기
        _, (_, *where) = base.go_and_dock(target)
        forward, left, yaw = base.score(target)
        print(f"            채점 (참값): 서 있는 자리가 앞뒤 {forward * 1000:+.1f} mm · 좌우 {left * 1000:+.1f} mm · 방향 {math.degrees(yaw):+.2f}° 어긋남", flush=True)
        self.mode("ARM", f"베이스는 서 있음. {cup} 을(를) {'왼팔' if side == 'left' else '오른팔'}의 정면에 놓음")
        self.place(target, where)
        self.leave(side, "transport")
        # 3. 처음 자리로
        base.navigate("home")
        self.mode("IDLE", "임무 끝")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--cup", choices=list(CUPS), default="cup_pink")
    parser.add_argument("--from", dest="source", default="drinks", choices=["drinks"])
    parser.add_argument("--to", dest="target", default="consult_1", choices=["consult_1", "consult_2"])
    parser.add_argument("--arm", choices=["left", "right"], default="left", help="--only place 일 때: 놓는 동작을 할 팔")
    parser.add_argument("--only", choices=["pick", "place"], help="도킹한 자리에서 팔 동작만 함")
    parser.add_argument("--side-offset", type=float, default=0.0, help="--only 일 때: 도킹이 알려 줬다고 볼 옆 거리 [m]")
    parser.add_argument("--no-dock", action="store_true", help="비교용: 음료대에서 도킹하지 않고 집는다")
    parser.add_argument("--save", metavar="JSON", help="모드의 기록을 저장")
    args = parser.parse_args()

    rclpy.init()
    mission = Mission()
    base = mission.base
    base.wait_for(lambda: base.odom_msg is not None and base.truth_msg is not None, 60.0, "/odom · /ground_truth (시뮬레이터)")
    try:
        if args.only == "pick":
            mission.mode("ARM", f"{args.cup} 을(를) 집음")
            mission.pick(args.source, args.cup, (args.side_offset, 0.0))
            mission.mode("IDLE", "집어서 운반 자세")
        elif args.only == "place":
            mission.mode("ARM", "놓는 동작")
            mission.place(args.target, (args.side_offset, 0.0), args.arm)
            mission.mode("IDLE", "놓고 이동 자세")
        else:
            base.ready()
            mission.run(args.cup, args.source, args.target, use_dock=not args.no_dock)
    except (DockError, ArmError) as error:
        if base.mode != "FAILED":
            mission.mode("FAILED", str(error))
        base.halt()
    if args.save:
        base.save(args.save)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
