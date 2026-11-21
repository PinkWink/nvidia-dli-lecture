#!/usr/bin/env python3
"""강의 41: 두 팔을 정해 둔 자세로 보내는 궤적을 궤적 서버에 보냅니다 (MoveIt 이 할 일을 손으로 해 보는 것).

    source /opt/ros/jazzy/setup.bash
    python3 41_pl2m_ros2/send_pose.py forward            # 두 팔을 앞으로 뻗기, 3초에
    python3 41_pl2m_ros2/send_pose.py side --seconds 2
    python3 41_pl2m_ros2/send_pose.py carry --grip 0.03  # 운반 자세 + 그리퍼를 30 mm 벌림
    python3 41_pl2m_ros2/send_pose.py down
"""
import argparse
import math
import sys

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

POSES = {  # 왼팔 관절 1 ~ 7 [rad]. 강의 39 의 POSES 와 강의 40 의 REACH 에서 가져옴
    "down": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "transport": [0.0, 0.0, 0.0, 1.2, 0.0, 0.0, 0.0],
    "carry": [0.93, -2.3726, -0.4355, 2.3729, 0.387, -0.7903, 1.3904],
    "forward": [-1.5708, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "side": [0.0, -1.5708, 0.0, 0.0, 0.0, 0.0, 0.0],
}
MIRROR = [-1, -1, -1, 1, -1, -1, -1]          # 오른팔 = 왼팔 × 이 부호
POINTS = 20                                   # 궤적을 이만큼의 점으로 나눠 보냄


def smooth(ratio):
    return ratio * ratio * (3.0 - 2.0 * ratio)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("pose", choices=list(POSES))
    parser.add_argument("--seconds", type=float, default=3.0, help="궤적의 길이")
    parser.add_argument("--grip", type=float, help="그리퍼를 이만큼 벌림 [m] (0 ~ 0.044)")
    args = parser.parse_args()

    rclpy.init()
    node = Node("send_pose")
    measured = {}
    node.create_subscription(JointState, "/joint_states", lambda msg: measured.update(zip(msg.name, msg.position)), 10)
    goals = {"left": POSES[args.pose], "right": [value * sign for value, sign in zip(POSES[args.pose], MIRROR)]}
    names = {side: [f"openarm_{side}_joint{i}" for i in range(1, 8)] for side in goals}
    clients = {side: ActionClient(node, FollowJointTrajectory, f"/{side}_joint_trajectory_controller/follow_joint_trajectory") for side in goals}
    for client in clients.values():
        if not client.wait_for_server(timeout_sec=5.0):
            sys.exit("궤적 서버가 없음: trajectory_server.py 를 먼저 띄울 것")
    while not all(name in measured for side in names for name in names[side]):
        rclpy.spin_once(node, timeout_sec=0.1)                # 지금 자세를 알아야 궤적을 만든다

    futures = {}
    for side, client in clients.items():
        begin = [measured[name] for name in names[side]]
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = names[side]
        for k in range(1, POINTS + 1):
            ratio = smooth(k / POINTS)
            t = args.seconds * k / POINTS
            point = JointTrajectoryPoint(positions=[a + (b - a) * ratio for a, b in zip(begin, goals[side])])
            point.time_from_start = Duration(sec=int(t), nanosec=int((t % 1.0) * 1e9))
            goal.trajectory.points.append(point)
        futures[side] = client.send_goal_async(goal)
    results = {}
    for side, future in futures.items():
        rclpy.spin_until_future_complete(node, future)
        if not future.result().accepted:
            sys.exit(f"{side}: 궤적을 받아 주지 않았음")
        results[side] = future.result().get_result_async()
    for side, future in results.items():
        rclpy.spin_until_future_complete(node, future)
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.05)               # 멈춘 뒤의 위치를 받는다
    print(f"{'팔':<6} {'결과':<8} {'가장 큰 오차[°]':>12} {'1번 관절 오차[°]':>14} {'2번 관절 오차[°]':>14}")
    for side, future in results.items():
        result = future.result().result
        errors = [math.degrees(measured[name] - goal) for name, goal in zip(names[side], goals[side])]
        sign = 1 if side == "left" else -1                    # 1 · 2번 관절은 오른팔의 부호가 반대
        print(f"{'왼팔' if side == 'left' else '오른팔':<6} {'성공' if result.error_code == result.SUCCESSFUL else '실패':<8} "
              f"{max(abs(e) for e in errors):>12.2f} {errors[0] * sign:>14.2f} {errors[1] * sign:>14.2f}")

    if args.grip is not None:
        grips = {side: ActionClient(node, GripperCommand, f"/{side}_gripper_controller/gripper_cmd") for side in goals}
        waits = {}
        for side, client in grips.items():
            client.wait_for_server(timeout_sec=5.0)
            goal = GripperCommand.Goal()
            goal.command.position = args.grip
            waits[side] = client.send_goal_async(goal)
        for side, future in waits.items():
            rclpy.spin_until_future_complete(node, future)
            done = future.result().get_result_async()
            rclpy.spin_until_future_complete(node, done)
            result = done.result().result
            print(f"{'왼손' if side == 'left' else '오른손'} 그리퍼: 위치 {result.position * 1000:.1f} mm, 도착 {'예' if result.reached_goal else '아니오'}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
