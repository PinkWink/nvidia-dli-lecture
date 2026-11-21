#!/usr/bin/env python3
"""강의 42: MoveIt 에 목표를 주고, 계획한 경로를 실행시킵니다.

    source /opt/ros/jazzy/setup.bash
    source ~/pinklab_robotworld2026/install/setup.bash
    python3 42_pl2m_moveit/move_arm.py left ready                 # 이름 붙은 자세로 (SRDF 의 group_state)
    python3 42_pl2m_moveit/move_arm.py both carry                 # 두 팔 함께
    python3 42_pl2m_moveit/move_arm.py left --shift 0.10 0 0.05   # 손끝을 지금 자리에서 앞으로 10 cm, 위로 5 cm
    python3 42_pl2m_moveit/move_arm.py left --xyz 0.45 0.20 0.80  # 손끝을 베이스 기준 이 자리로
    python3 42_pl2m_moveit/move_arm.py left ready --plan-only     # 계획만 하고 움직이지 않음

목표의 두 종류
    관절 목표   "관절 7개가 각각 몇 도" 로 준다. 이름 붙은 자세가 이것이다
    자세 목표   "손끝(tcp)이 어디에, 어느 방향으로" 로 준다. 관절 각도는 MoveIt 이 역기구학으로 찾는다
손끝의 방향은 지금 방향을 지키게 합니다. 좌표는 로봇 베이스(base_footprint) 기준이고 x 앞 · y 왼쪽 · z 위 입니다.
"""
import argparse
import math
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import BoundingVolume, Constraints, JointConstraint, MoveItErrorCodes, OrientationConstraint, PositionConstraint
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformListener

BASE = "base_footprint"
ERRORS = {value: name for name, value in vars(MoveItErrorCodes).items() if name.isupper() and isinstance(value, int)}


def tip(side):
    return f"openarm_{side}_hand_tcp"                        # 손끝: 두 손가락 사이의 점


def named_state(side, pose):
    """SRDF 에 적힌 이름 붙은 자세 → 관절 목표."""
    srdf = ET.parse(Path(get_package_share_directory("mobile_openarm_moveit_config")) / "config/mobile_openarm.srdf")
    state = srdf.find(f"group_state[@group='{side}_arm'][@name='{pose}']")
    if state is None:
        names = sorted({s.get("name") for s in srdf.findall("group_state") if s.get("group") == f"{side}_arm"})
        sys.exit(f"SRDF 에 '{pose}' 자세가 없음. 있는 것: {', '.join(names)}")
    return [JointConstraint(joint_name=j.get("name"), position=float(j.get("value")), tolerance_above=0.01, tolerance_below=0.01, weight=1.0)
            for j in state.findall("joint")]


def pose_goal(side, position, orientation):
    """손끝이 position 의 1 mm 안에, 방향은 orientation 에서 0.02 rad 안에 오게 하는 목표."""
    where = PositionConstraint(link_name=tip(side), weight=1.0)
    where.header.frame_id = BASE
    center = Pose()
    center.position.x, center.position.y, center.position.z = position
    center.orientation.w = 1.0
    where.constraint_region = BoundingVolume(primitives=[SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.001])], primitive_poses=[center])
    facing = OrientationConstraint(link_name=tip(side), orientation=orientation, weight=1.0,
                                   absolute_x_axis_tolerance=0.02, absolute_y_axis_tolerance=0.02, absolute_z_axis_tolerance=0.02)
    facing.header.frame_id = BASE
    return where, facing


class Arm(Node):
    def __init__(self):
        super().__init__("move_arm", parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.client = ActionClient(self, MoveGroup, "/move_action")
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)

    def tip_pose(self, side, wait=5.0):
        """손끝의 지금 위치와 방향 (베이스 기준). /tf 에서 읽는다 = 잰 관절 각도로 계산한 값."""
        end = time.time() + wait
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.tf.can_transform(BASE, tip(side), Time()):
                t = self.tf.lookup_transform(BASE, tip(side), Time()).transform
                return (t.translation.x, t.translation.y, t.translation.z), t.rotation
        sys.exit("/tf 에서 손끝의 위치를 읽지 못함: moveit.launch.py 와 시뮬레이터가 떠 있는지 볼 것")

    def move(self, group, constraints, plan_only=False, speed=0.3):
        goal = MoveGroup.Goal()
        request = goal.request
        request.group_name = group
        request.allowed_planning_time = 8.0
        request.num_planning_attempts = 5
        request.max_velocity_scaling_factor = speed           # 관절 속도 한계의 몇 배로 움직일지
        request.max_acceleration_scaling_factor = speed
        request.start_state.is_diff = True                    # 지금 자세에서 출발
        request.goal_constraints = [constraints]
        goal.planning_options.plan_only = plan_only
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        if not self.client.wait_for_server(timeout_sec=30.0):
            sys.exit("MoveIt 이 없음: ros2 launch 42_pl2m_moveit/moveit.launch.py 를 먼저 띄울 것")
        sent = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, sent)
        if not sent.result().accepted:
            sys.exit("MoveIt 이 요청을 받지 않았음")
        done = sent.result().get_result_async()
        rclpy.spin_until_future_complete(self, done)
        return done.result().result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("side", choices=["left", "right", "both"])
    parser.add_argument("pose", nargs="?", help="이름 붙은 자세 (home · transport · ready · carry · hands_up)")
    parser.add_argument("--xyz", type=float, nargs=3, metavar=("X", "Y", "Z"), help="손끝을 베이스 기준 이 자리로 [m]")
    parser.add_argument("--shift", type=float, nargs=3, metavar=("DX", "DY", "DZ"), help="손끝을 지금 자리에서 이만큼 옮김 [m]")
    parser.add_argument("--plan-only", action="store_true", help="계획만 하고 움직이지 않음")
    parser.add_argument("--speed", type=float, default=0.3, help="속도 · 가속도 한계의 몇 배로 움직일지 (0 ~ 1)")
    parser.add_argument("--tries", type=int, default=3, help="계획이 실패하면 다시 해 보는 횟수 (OMPL 은 무작위로 찾아서 가끔 실패한다)")
    args = parser.parse_args()
    if sum(value is not None for value in (args.pose, args.xyz, args.shift)) != 1:
        parser.error("자세 이름, --xyz, --shift 가운데 하나만 줄 것")
    if args.side == "both" and args.pose is None:
        parser.error("--xyz · --shift 는 한 팔씩만 됨")

    rclpy.init()
    arm = Arm()
    sides = ["left", "right"] if args.side == "both" else [args.side]
    constraints, wanted = Constraints(), None
    if args.pose:
        for side in sides:
            constraints.joint_constraints += named_state(side, args.pose)
        what = f"{args.pose} 자세"
    else:
        position, orientation = arm.tip_pose(args.side)
        wanted = tuple(args.xyz) if args.xyz else tuple(p + d for p, d in zip(position, args.shift))
        where, facing = pose_goal(args.side, wanted, orientation)
        constraints.position_constraints, constraints.orientation_constraints = [where], [facing]
        what = "손끝을 (" + ", ".join(f"{value:.3f}" for value in wanted) + ") 로"
    group = "both_arms" if args.side == "both" else f"{args.side}_arm"
    for attempt in range(1, args.tries + 1):
        result = arm.move(group, constraints, args.plan_only, args.speed)
        code = result.error_code.val
        if code == MoveItErrorCodes.SUCCESS:
            break
        print(f"{group}: {attempt}번째 시도 실패 ({ERRORS.get(code, str(code))})" + (", 다시 계획함" if attempt < args.tries else ""))
    points = result.planned_trajectory.joint_trajectory.points
    print(f"{group}: {what}  →  {'성공' if code == MoveItErrorCodes.SUCCESS else '실패 (' + ERRORS.get(code, str(code)) + ')'}")
    if points:
        last = points[-1].time_from_start
        print(f"  궤적의 점 {len(points)}개, 궤적의 길이 {last.sec + last.nanosec * 1e-9:.2f}초"
              + (", 점마다 속도 있음" if len(points[0].velocities) else ", 속도 없음")
              + (f", 계획에 걸린 시간 {result.planning_time:.2f}초" if result.planning_time > 0 else ""))
    if wanted and code == MoveItErrorCodes.SUCCESS and not args.plan_only:
        for _ in range(20):
            rclpy.spin_once(arm, timeout_sec=0.05)            # 멈춘 뒤의 /tf 를 받는다
        reached, _ = arm.tip_pose(args.side)
        error = [(a - b) * 1000 for a, b in zip(reached, wanted)]
        print(f"  손끝이 간 곳 ({reached[0]:.3f}, {reached[1]:.3f}, {reached[2]:.3f}),  목표와의 차이 x {error[0]:+.1f} · y {error[1]:+.1f} · z {error[2]:+.1f} mm"
              f"  (거리 {math.sqrt(sum(e * e for e in error)):.1f} mm)")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
