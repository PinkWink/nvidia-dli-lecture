#!/usr/bin/env python3
"""강의 43: 탁자 위의 컵을 집어 듭니다. 원본의 집기 절차(warehouse_skills 의 grasp)를 한 단계씩 풀어 쓴 것입니다.

    source /opt/ros/jazzy/setup.bash
    source ~/pinklab_robotworld2026/install/setup.bash
    python3 43_pl2m_pick/pick_cup.py left                 # 왼팔로 분홍 컵
    python3 43_pl2m_pick/pick_cup.py right                # 오른팔로 파란 컵
    python3 43_pl2m_pick/pick_cup.py left --no-payload    # 집은 뒤 무게를 로봇에 알리지 않음
    python3 43_pl2m_pick/pick_cup.py left --mass 1.0      # 컵이 1 kg 이라고 알림 (시뮬레이터의 --cup-mass 와 맞출 것)
    python3 43_pl2m_pick/pick_cup.py left --put-back      # 들었다가 제자리에 다시 내려놓음
    python3 43_pl2m_pick/pick_cup.py left --mass 1.0 --compare   # 든 채로 무게 알림을 껐다 켜서 손끝의 높이를 비교

순서
    1. 장면 알리기      탁자와 컵을 MoveIt 의 계획 장면에 넣는다
    2. 그리퍼 열기
    3. 접근 시작 자리   컵의 뒤 10 cm, 손은 앞을 보고 옆에서 쥐는 방향 (자세 목표, MoveIt 이 경로를 계획)
    4. 곧게 접근        손끝을 직선으로 컵까지 (직선 경로)
    5. 그리퍼 닫기      12 mm 까지 닫으라고 한다. 컵에 막혀 25 mm 쯤에서 멈추면 쥔 것
    6. 무게 알리기      로봇(시뮬레이터)에 손에 든 무게를 알린다. MoveIt 에는 컵이 손에 붙었다고 알린다
    7. 들기 · 빼기      4 cm 들고, 14 cm 뒤로 뺀다 (직선 경로)
"""
import argparse
import os
import sys
import time

import rclpy
from control_msgs.action import GripperCommand
from geometry_msgs.msg import Pose, Quaternion
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, Constraints, MoveItErrorCodes, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "42_pl2m_moveit"))
from move_arm import BASE, ERRORS, Arm, pose_goal, tip
from scene import CUP_DIAMETER, CUP_HEIGHT, CUP_MASS, CUPS, GRASP, GRASP_FORWARD, TABLE_LEG, TABLE_TOP

FORWARD = Quaternion(x=GRASP_FORWARD[0], y=GRASP_FORWARD[1], z=GRASP_FORWARD[2], w=GRASP_FORWARD[3])


def pose_at(x, y, z):
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = float(x), float(y), float(z)
    pose.orientation = FORWARD
    return pose


def box(name, center, size):
    item = CollisionObject(id=name, operation=CollisionObject.ADD)
    item.header.frame_id = "odom"
    item.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[float(v) for v in size])]
    item.primitive_poses = [Pose()]
    item.primitive_poses[0].position.x, item.primitive_poses[0].position.y, item.primitive_poses[0].position.z = center
    item.primitive_poses[0].orientation.w = 1.0
    return item


def cylinder(name, center, diameter, height):
    item = box(name, center, (0, 0, 0))
    item.primitives = [SolidPrimitive(type=SolidPrimitive.CYLINDER, dimensions=[float(height), float(diameter) / 2])]
    return item


class Picker(Arm):
    def __init__(self):
        super().__init__()
        self.scene = self.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self.cartesian = self.create_client(GetCartesianPath, "/compute_cartesian_path")
        self.execute = ActionClient(self, ExecuteTrajectory, "/execute_trajectory")
        self.grippers = {side: ActionClient(self, GripperCommand, f"/{side}_gripper_controller/gripper_cmd") for side in ("left", "right")}
        self.payload = self.create_publisher(JointState, "/payload", 10)
        self.measured, self.effort = {}, {}
        self.create_subscription(JointState, "/joint_states", self.on_joint_states, 10)

    def on_joint_states(self, msg):
        self.measured.update(zip(msg.name, msg.position))
        self.effort.update(zip(msg.name, msg.effort))

    def call(self, client, request):
        if not client.wait_for_service(timeout_sec=20.0):
            sys.exit("MoveIt 이 없음: ros2 launch 42_pl2m_moveit/moveit.launch.py 를 먼저 띄울 것")
        done = client.call_async(request)
        rclpy.spin_until_future_complete(self, done)
        return done.result()

    def apply(self, world=(), attached=()):
        scene = PlanningScene(is_diff=True)
        scene.world.collision_objects = list(world)
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects = list(attached)
        return self.call(self.scene, ApplyPlanningScene.Request(scene=scene)).success

    def add_scene(self):
        """탁자와 컵 둘을 MoveIt 에 알린다."""
        items = [box("table_top", TABLE_TOP["center"], TABLE_TOP["size"]), box("table_leg", TABLE_LEG["center"], TABLE_LEG["size"])]
        items += [cylinder(name, center, CUP_DIAMETER, CUP_HEIGHT) for name, center, _ in CUPS.values()]
        return self.apply(world=items)

    def attach(self, side, name, attach=True):
        """컵이 손에 붙었다고(또는 떨어졌다고) MoveIt 에 알린다. 붙은 뒤에는 손가락과 닿아 있어도 충돌로 보지 않는다."""
        item = AttachedCollisionObject(link_name=f"openarm_{side}_hand")
        item.object.id = name
        item.object.operation = CollisionObject.ADD if attach else CollisionObject.REMOVE
        item.touch_links = [f"openarm_{side}_hand", f"openarm_{side}_left_finger", f"openarm_{side}_right_finger", tip(side)]
        return self.apply(attached=[item])

    def grip(self, side, opening):
        """그리퍼를 opening [m] 까지 움직인다. (멈춘 자리, 목표에 닿았는지, 막혀서 멈췄는지) 를 돌려준다."""
        goal = GripperCommand.Goal()
        goal.command.position = float(opening)
        self.grippers[side].wait_for_server(timeout_sec=10.0)
        sent = self.grippers[side].send_goal_async(goal)
        rclpy.spin_until_future_complete(self, sent)
        done = sent.result().get_result_async()
        rclpy.spin_until_future_complete(self, done)
        result = done.result().result
        return result.position, result.reached_goal, result.stalled

    def straight(self, side, x, y, z, speed=0.3):
        """손끝을 지금 자리에서 (x, y, z) 까지 직선으로 옮긴다. 직선의 몇 % 를 갈 수 있었는지 돌려준다."""
        request = GetCartesianPath.Request()
        request.header.frame_id = BASE
        request.start_state.is_diff = True
        request.group_name, request.link_name = f"{side}_arm", tip(side)
        request.waypoints = [pose_at(x, y, z)]
        request.max_step = 0.01                                   # 1 cm 마다 역기구학을 푼다
        request.avoid_collisions = True
        request.max_velocity_scaling_factor = request.max_acceleration_scaling_factor = speed
        answer = self.call(self.cartesian, request)
        if answer.error_code.val != MoveItErrorCodes.SUCCESS or answer.fraction < 0.9:
            return answer.fraction if answer.error_code.val == MoveItErrorCodes.SUCCESS else 0.0
        self.execute.wait_for_server(timeout_sec=10.0)
        sent = self.execute.send_goal_async(ExecuteTrajectory.Goal(trajectory=answer.solution))
        rclpy.spin_until_future_complete(self, sent)
        done = sent.result().get_result_async()
        rclpy.spin_until_future_complete(self, done)
        return answer.fraction if done.result().result.error_code.val == MoveItErrorCodes.SUCCESS else 0.0

    def to_pose(self, side, x, y, z, tries=3):
        """손끝을 (x, y, z) 로, 앞을 보고 옆에서 쥐는 방향으로 보낸다 (MoveIt 이 경로를 계획)."""
        where, facing = pose_goal(side, (x, y, z), FORWARD)
        constraints = Constraints(position_constraints=[where], orientation_constraints=[facing])
        for _ in range(tries):
            code = self.move(f"{side}_arm", constraints).error_code.val
            if code == MoveItErrorCodes.SUCCESS:
                return True
        print(f"  실패: {ERRORS.get(code, code)}")
        return False

    def tell_payload(self, side, mass):
        """로봇(시뮬레이터)에 손에 든 무게를 알린다. 몇 번 되풀이해 보낸다."""
        msg = JointState(name=[side], position=[float(mass)])
        for _ in range(5):
            self.payload.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)

    def settle(self, seconds=1.0):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("side", choices=["left", "right"])
    parser.add_argument("--mass", type=float, default=CUP_MASS, help="로봇에 알릴 컵의 무게 [kg]")
    parser.add_argument("--no-payload", action="store_true", help="집은 뒤 무게를 로봇에 알리지 않음")
    parser.add_argument("--put-back", action="store_true", help="들었다가 제자리에 다시 내려놓음")
    parser.add_argument("--compare", action="store_true", help="든 채로 무게 알림을 껐다 켜서 손끝의 높이를 비교함")
    args = parser.parse_args()
    side = args.side
    name, (cx, cy, cz), _ = CUPS[side]
    z = cz + GRASP["z_offset"]                                    # 손끝의 높이
    finger, follower = f"openarm_{side}_finger_joint1", f"openarm_{side}_finger_joint2"

    rclpy.init()
    arm = Picker()
    print(f"1. 장면 알리기: 탁자와 컵  →  {'됨' if arm.add_scene() else '안 됨'}")
    position, _, _ = arm.grip(side, GRASP["open"])
    print(f"2. 그리퍼 열기  →  손가락 {position * 1000:.1f} mm (둘째 손가락 {arm.measured.get(follower, 0.0) * 1000:.1f} mm)")
    print(f"3. 접근 시작 자리 ({cx + GRASP['pre_offset_x']:.3f}, {cy:.3f}, {z:.3f})  →  ", end="", flush=True)
    if not arm.to_pose(side, cx + GRASP["pre_offset_x"], cy, z):
        sys.exit(1)
    print("도착")
    fraction = arm.straight(side, cx, cy, z)
    arm.settle(0.5)
    at = arm.tip_pose(side)[0]
    print(f"4. 곧게 접근 {-GRASP['pre_offset_x'] * 100:.0f} cm  →  직선의 {fraction * 100:.0f} % 를 감, 손끝 ({at[0]:.3f}, {at[1]:.3f}, {at[2]:.3f})")
    if fraction < 0.9:
        sys.exit("  직선으로 갈 수 없음")
    position, reached, stalled = arm.grip(side, GRASP["close"])
    held = GRASP["close_min"] <= position <= GRASP["close_max"]
    print(f"5. 그리퍼 닫기  →  손가락이 {position * 1000:.1f} mm 에서 멈춤 (둘째 손가락 {arm.measured.get(follower, 0.0) * 1000:.1f} mm): {'쥐었음' if held else '놓쳤음'}")
    arm.settle(0.5)
    print(f"   손가락 관절에 걸린 힘: 첫째 {arm.effort.get(finger, 0.0):+.1f} N, 둘째 {arm.effort.get(follower, 0.0):+.1f} N")
    if not held:
        arm.grip(side, GRASP["open"])
        sys.exit(1)
    if not args.no_payload:
        arm.tell_payload(side, args.mass)
    arm.attach(side, name)
    print(f"6. 무게 알리기  →  로봇에 {'알리지 않음' if args.no_payload else f'{args.mass:g} kg'}, MoveIt 에는 컵이 손에 붙었다고 알림")
    up = arm.straight(side, cx, cy, z + GRASP["lift_z"])
    back = arm.straight(side, cx + GRASP["retreat_x"], cy, z + GRASP["lift_z"])
    print(f"7. 들기 {GRASP['lift_z'] * 100:.0f} cm · 빼기 {-GRASP['retreat_x'] * 100:.0f} cm  →  직선의 {up * 100:.0f} % · {back * 100:.0f} %")
    arm.settle(1.5)
    reached, _ = arm.tip_pose(side)
    wanted = (cx + GRASP["retreat_x"], cy, z + GRASP["lift_z"])
    print(f"   손끝의 높이 {reached[2]:.4f} m (목표 {wanted[2]:.4f}, 차이 {(reached[2] - wanted[2]) * 1000:+.1f} mm),  손가락 {arm.measured.get(finger, 0.0) * 1000:.1f} mm")
    if args.compare:                                              # 같은 팔 자세에서, 알리는 무게만 바꿔 본다
        import math
        print("   팔 관절 1 ~ 7 [°]: " + " ".join(f"{math.degrees(arm.measured[f'openarm_{side}_joint{i}']):.1f}" for i in range(1, 8)))
        print(f"   {'알린 무게[kg]':>12} {'손끝의 높이[m]':>13} {'내려간 거리[mm]':>13}")
        first = None
        for mass in (args.mass, args.mass / 2, 0.0, args.mass):
            arm.tell_payload(side, mass)
            arm.settle(2.0)
            height = arm.tip_pose(side)[0][2]
            first = height if first is None else first
            print(f"   {mass:>12.2f} {height:>13.4f} {(first - height) * 1000:>13.1f}")
    if args.put_back:
        arm.straight(side, cx, cy, z + GRASP["lift_z"])
        arm.straight(side, cx, cy, z + 0.004)                     # 4 mm 위에서 놓는다
        position, _, _ = arm.grip(side, GRASP["open"])
        arm.attach(side, name, attach=False)
        if not args.no_payload:
            arm.tell_payload(side, 0.0)
        arm.straight(side, cx + GRASP["pre_offset_x"], cy, z)
        print(f"8. 제자리에 내려놓기  →  손가락 {position * 1000:.1f} mm, 무게 0 kg 으로 되돌림")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
