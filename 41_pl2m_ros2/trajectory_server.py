#!/usr/bin/env python3
"""강의 41: 궤적 서버. MoveIt 이 보내는 궤적(액션)을 받아, 시간에 맞춰 관절의 목표 위치를 /joint_command 로 내보냅니다.

목표는 /joint_states 가 올 때마다 하나씩 보냅니다. 시뮬레이터가 한 스텝 나아갈 때마다 목표도 한 번 바뀌게 하려는 것입니다.
(벽시계로 50 Hz 씩 보내면 60 Hz 로 도는 시뮬레이터와 박자가 어긋나 관절 토크가 출렁였습니다. --wall 로 그때의 방법을 볼 수 있습니다.)

    source /opt/ros/jazzy/setup.bash
    python3 41_pl2m_ros2/trajectory_server.py
    python3 41_pl2m_ros2/trajectory_server.py --wall     # 비교용: 벽시계로 50 Hz 씩 보냄

액션 (원본 MuJoCo 시뮬레이터가 내는 것과 같은 이름. MoveIt 설정 moveit_controllers.yaml 이 이 이름을 찾는다)
    /left_joint_trajectory_controller/follow_joint_trajectory     control_msgs/action/FollowJointTrajectory
    /right_joint_trajectory_controller/follow_joint_trajectory
    /left_gripper_controller/gripper_cmd                          control_msgs/action/GripperCommand
    /right_gripper_controller/gripper_cmd

이 노드는 위치만 보냅니다. 팔의 무게를 버티는 토크(중력 보상)는 시뮬레이터가 넣습니다.
"""
import math
import sys
import threading
import time

import numpy as np
import rclpy
from control_msgs.action import FollowJointTrajectory, GripperCommand
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

WALL_RATE = 50.0            # --wall 일 때 /joint_command 를 내보내는 빠르기 [Hz]
GOAL_TOLERANCE = 0.035      # 궤적이 끝났을 때 목표와 이만큼 안이면 성공 [rad] (원본 설정 arm.goal_tolerance)
GOAL_TIME = 3.0             # 궤적이 끝난 뒤 이 시간 안에 못 들어오면 실패 [s] (원본 설정 arm.goal_time_tolerance)
GRIP_TOLERANCE = 0.002      # 그리퍼가 목표와 이만큼 안이면 도착 [m]
GRIP_OPEN = 0.044           # 손가락 하나가 벌어지는 거리의 한계 [m]


def seconds(duration):
    return duration.sec + duration.nanosec * 1e-9


class Trajectory:
    """궤적의 점들 사이를 잇는다. 점에 속도가 있으면 3차 곡선으로, 없으면 직선으로 (원본 trajectory.py 와 같은 방법)."""

    def __init__(self, trajectory, current):
        self.times = np.array([seconds(point.time_from_start) for point in trajectory.points])
        self.positions = np.array([point.positions for point in trajectory.points])
        velocities = [point.velocities for point in trajectory.points]
        self.velocities = np.array(velocities) if all(len(v) == len(current) for v in velocities) else None
        if self.times[0] > 0:                                  # 첫 점이 0초가 아니면 지금 자세에서 출발
            self.times = np.r_[0.0, self.times]
            self.positions = np.vstack([current, self.positions])
            if self.velocities is not None:
                self.velocities = np.vstack([np.zeros(len(current)), self.velocities])

    def sample(self, t):
        if t <= self.times[0]:
            return self.positions[0].copy()
        if t >= self.times[-1]:
            return self.positions[-1].copy()
        i = int(np.searchsorted(self.times, t, side="right")) - 1
        dt = self.times[i + 1] - self.times[i]
        u = (t - self.times[i]) / dt
        a, b = self.positions[i], self.positions[i + 1]
        if self.velocities is None:
            return a + (b - a) * u
        va, vb = self.velocities[i], self.velocities[i + 1]
        return ((2 * u**3 - 3 * u**2 + 1) * a + (u**3 - 2 * u**2 + u) * dt * va
                + (-2 * u**3 + 3 * u**2) * b + (u**3 - u**2) * dt * vb)


class TrajectoryServer(Node):
    def __init__(self, wall=False):
        super().__init__("pl2m_trajectory_server", parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.lock = threading.Lock()
        self.measured, self.effort = {}, {}        # /joint_states 에서 읽은 지금 위치 · 힘
        self.targets = {}                          # 내보내는 목표 위치 (한 번이라도 명령한 관절만)
        self.active = {}                           # 실행 중인 궤적: 제어기 이름 → 상태
        self.stamp = 0.0                           # 마지막 /joint_states 의 시각 (시뮬레이션 시간)
        self.wall = wall
        group = ReentrantCallbackGroup()
        self.command = self.create_publisher(JointState, "/joint_command", 10)
        self.create_subscription(JointState, "/joint_states", self.on_joint_states, 10, callback_group=group)
        if wall:                                   # 예전 방법: 시뮬레이터의 박자와 상관없이 벽시계로 보냄
            self.create_timer(1.0 / WALL_RATE, self.step, callback_group=group, clock=rclpy.clock.Clock())
        self.servers = []
        for side in ("left", "right"):
            arm = [f"openarm_{side}_joint{i}" for i in range(1, 8)]
            finger = f"openarm_{side}_finger_joint1"
            self.servers.append(ActionServer(
                self, FollowJointTrajectory, f"/{side}_joint_trajectory_controller/follow_joint_trajectory",
                execute_callback=lambda handle, key=side + "_arm": self.run_trajectory(handle, key),
                goal_callback=lambda request, names=arm, key=side + "_arm": self.check_trajectory(request, names, key),
                cancel_callback=lambda handle: CancelResponse.ACCEPT, callback_group=group))
            self.servers.append(ActionServer(
                self, GripperCommand, f"/{side}_gripper_controller/gripper_cmd",
                execute_callback=lambda handle, name=finger, key=side + "_gripper": self.run_gripper(handle, name, key),
                goal_callback=lambda request, key=side + "_gripper": self.check_gripper(request, key),
                cancel_callback=lambda handle: CancelResponse.ACCEPT, callback_group=group))
        self.busy = set()
        self.get_logger().info("궤적 서버 준비됨: 팔 2 · 그리퍼 2" + (" (벽시계 50 Hz)" if wall else " (/joint_states 에 맞춰 보냄)"))

    def on_joint_states(self, msg):
        with self.lock:
            self.measured.update(zip(msg.name, msg.position))
            self.effort.update(zip(msg.name, msg.effort))
            self.stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if not self.wall:
            self.step()                            # 시뮬레이터가 한 스텝 나아갈 때마다 목표를 한 번 보낸다

    def step(self):
        """실행 중인 궤적마다 지금 시각의 목표를 구해, 한 메시지에 모아 보낸다. 팔 둘이 따로 보내면 한쪽이 묻힐 수 있다."""
        with self.lock:
            now = self.stamp
            for state in self.active.values():
                self.advance(state, now)
            if not self.targets:
                return
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = list(self.targets)
            msg.position = [float(value) for value in self.targets.values()]
        self.command.publish(msg)                  # velocity · effort 는 비워 둔다. 위치만 보낸다

    def advance(self, state, now):
        """궤적 하나를 지금 시각까지 진행시킨다 (lock 을 잡은 채 불린다)."""
        if state["start"] is None:
            state["start"] = now
        trajectory, order, handle = state["trajectory"], state["order"], state["handle"]
        t = now - state["start"]
        desired = trajectory.sample(t)
        self.targets.update(zip(order, desired))
        actual = np.array([self.measured[name] for name in order])
        if t - state["feedback_at"] >= 0.05:
            feedback = FollowJointTrajectory.Feedback()
            feedback.header.stamp = self.get_clock().now().to_msg()
            feedback.joint_names = order
            feedback.desired = JointTrajectoryPoint(positions=desired.tolist())
            feedback.actual = JointTrajectoryPoint(positions=actual.tolist())
            feedback.error = JointTrajectoryPoint(positions=(desired - actual).tolist())
            handle.publish_feedback(feedback)
            state["feedback_at"] = t
        if t >= trajectory.times[-1] and not state["done"].is_set():
            error = np.abs(actual - trajectory.positions[-1])
            if all(error[i] <= state["tolerance"].get(name, GOAL_TOLERANCE) for i, name in enumerate(order)):
                state["outcome"] = ("succeed", "")
                state["done"].set()
            elif t > trajectory.times[-1] + state["timeout"]:
                state["outcome"] = ("abort", f"목표에 들어오지 못함: 가장 큰 오차 {math.degrees(error.max()):.2f}°")
                state["done"].set()

    # 팔: FollowJointTrajectory
    def check_trajectory(self, request, names, key):
        trajectory = request.trajectory
        problem = None
        if key in self.busy:
            problem = "앞의 궤적이 아직 끝나지 않았음"
        elif sorted(trajectory.joint_names) != sorted(names):
            problem = "관절 이름이 이 제어기의 것과 다름"
        elif not trajectory.points or any(len(point.positions) != len(names) for point in trajectory.points):
            problem = "점이 없거나 위치의 개수가 맞지 않음"
        elif any(name not in self.measured for name in names):
            problem = "/joint_states 를 아직 받지 못했음"
        else:
            times = [seconds(point.time_from_start) for point in trajectory.points]
            if any(b <= a for a, b in zip(times, times[1:])) or times[0] < 0:
                problem = "점의 시간이 차례대로가 아님"
        if problem:
            self.get_logger().warning(f"{key}: 궤적을 받지 않음 — {problem}")
            return GoalResponse.REJECT
        self.busy.add(key)
        return GoalResponse.ACCEPT

    def run_trajectory(self, handle, key):
        request, result = handle.request, FollowJointTrajectory.Result()
        order = list(request.trajectory.joint_names)
        with self.lock:
            current = np.array([self.measured[name] for name in order])
            state = {"trajectory": Trajectory(request.trajectory, current), "order": order, "handle": handle, "start": None,
                     "feedback_at": -1.0, "done": threading.Event(), "outcome": None,
                     "tolerance": {t.name: t.position for t in request.goal_tolerance if t.position > 0},
                     "timeout": seconds(request.goal_time_tolerance) or GOAL_TIME}
            self.active[key] = state
        try:
            while rclpy.ok() and not state["done"].wait(0.02):
                if handle.is_cancel_requested:
                    handle.canceled()
                    return result
            outcome, text = state["outcome"] or ("abort", "서버가 끝남")
            if outcome == "succeed":
                result.error_code = result.SUCCESSFUL
                handle.succeed()
            else:
                result.error_code = result.GOAL_TOLERANCE_VIOLATED
                result.error_string = text
                handle.abort()
            return result
        finally:
            with self.lock:
                self.active.pop(key, None)
            self.busy.discard(key)

    # 그리퍼: GripperCommand
    def check_gripper(self, request, key):
        position = request.command.position
        if key in self.busy or not math.isfinite(position) or not 0.0 <= position <= GRIP_OPEN:
            self.get_logger().warning(f"{key}: 명령을 받지 않음 (0 ~ {GRIP_OPEN} m 사이여야 함)")
            return GoalResponse.REJECT
        self.busy.add(key)
        return GoalResponse.ACCEPT

    def run_gripper(self, handle, name, key):
        goal, result = handle.request.command.position, GripperCommand.Result()
        with self.lock:
            self.targets[name] = goal
            start = moved_at = self.stamp
        last = None
        try:
            while rclpy.ok():
                with self.lock:
                    position, now = float(self.measured[name]), self.stamp
                    result.effort = float(self.effort.get(name, 0.0))
                if last is None or abs(position - last) > 0.0001:
                    moved_at, last = now, position
                result.position = position
                result.reached_goal = abs(position - goal) < GRIP_TOLERANCE
                result.stalled = now - moved_at > 1.0 and not result.reached_goal      # 물체를 쥐어 더 못 닫는 경우
                if handle.is_cancel_requested:
                    handle.canceled()
                    return result
                if result.reached_goal or result.stalled:
                    handle.succeed()
                    return result
                if now - start > 6.0:
                    handle.abort()
                    return result
                time.sleep(0.02)
            handle.abort()
            return result
        finally:
            self.busy.discard(key)


def main():
    rclpy.init()
    node = TrajectoryServer(wall="--wall" in sys.argv)
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
