"""강의 44: Nav2 에 목적지를 주고, 도착한 자리가 목표와 얼마나 다른지 잽니다.

    python3 44_pl2m_nav/go_to.py drinks              # 음료대 앞으로 (스테이션 이름: drinks · consult_1 · consult_2 · home)
    python3 44_pl2m_nav/go_to.py 6.0 -3.0 90         # 지도의 (x, y) 로 가서 90° 방향을 봄
    python3 44_pl2m_nav/go_to.py --tour 3            # 음료대 → 상담 1 → 상담 2 → 처음 자리를 세 바퀴 돌며 도착 오차를 잰다
    python3 44_pl2m_nav/go_to.py --tour 3 --save tour.json

재는 것
    도착 오차   참값 - 목표.  Nav2 가 "도착" 이라고 한 순간 로봇이 실제로 서 있는 자리
    추정 오차   추정 - 참값.  AMCL 이 생각하는 자리(/tf 의 map → base_footprint)가 실제와 얼마나 다른가
참값(/ground_truth)은 시뮬레이터만 아는 값이다. 여기서는 채점에만 쓴다.
"""
import argparse
import json
import math
import os
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from booth import STATIONS, angle_difference

STATUS = {GoalStatus.STATUS_SUCCEEDED: "도착", GoalStatus.STATUS_ABORTED: "중단됨", GoalStatus.STATUS_CANCELED: "취소됨"}


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def pose_error(pose, reference):
    """(거리 [m], 방향 차이 [rad]): pose 가 reference 와 얼마나 다른가."""
    return math.hypot(pose[0] - reference[0], pose[1] - reference[1]), angle_difference(pose[2], reference[2])


class Base(Node):
    """베이스(이동부)를 움직이고 그 위치를 읽는다. 뒤 강의의 도킹 · 임무 프로그램이 이 클래스를 이어 쓴다."""

    def __init__(self, name="base"):
        super().__init__(name, parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.nav = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.odom_msg = self.truth_msg = None
        self._ticked = [None, time.monotonic(), time.monotonic()]             # (마지막으로 본 시각, 그 값이 된 때, 마지막으로 물어본 때)
        self.create_subscription(Odometry, "/odom", self._on_odom, 50)
        self.create_subscription(Odometry, "/ground_truth", lambda msg: setattr(self, "truth_msg", msg), 50)
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)

    def _on_odom(self, msg):
        self.odom_msg = msg

    # --- 시간 · 기다리기 ------------------------------------------------
    def now(self):
        """시뮬레이터의 시각 [s] (/clock). 계속 물어보는데도 시계가 10초째 그대로면 시뮬레이터가 멈춘 것이므로 끝낸다."""
        value, wall = self.get_clock().now().nanoseconds * 1e-9, time.monotonic()
        if value != self._ticked[0] or wall - self._ticked[2] > 2.0:        # 시계가 갔거나, 한동안 묻지 않았다 (팔이 일하는 동안)
            self._ticked = [value, wall, wall]
        else:
            self._ticked[2] = wall
            if wall - self._ticked[1] > 10.0:
                sys.exit("시뮬레이터의 시계(/clock)가 10초째 멈춰 있음: 시뮬레이터가 떠 있는지 볼 것")
        return value

    def spin(self, seconds=0.0):
        """seconds 동안(벽시계) 들어오는 메시지를 처리한다. 0 이면 밀린 것만 처리한다."""
        end = time.monotonic() + seconds
        for _ in range(300):                             # 밀린 것: 한 번에 하나씩 처리되므로 여러 번 부른다 (빈 호출은 0.03 ms)
            rclpy.spin_once(self, timeout_sec=0.0)
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def wait_for(self, condition, timeout, what):
        end = time.monotonic() + timeout
        while not condition():
            if time.monotonic() > end:
                sys.exit(f"{what} 을(를) 기다리다 포기함")
            rclpy.spin_once(self, timeout_sec=0.05)

    def ready(self, timeout=60.0):
        """Nav2 와 위치 추정이 준비될 때까지 기다린다."""
        if not self.nav.wait_for_server(timeout_sec=timeout):
            sys.exit("Nav2 가 없음: ros2 launch 44_pl2m_nav/nav2.launch.py 를 먼저 띄울 것")
        self.wait_for(lambda: self.odom_msg is not None and self.truth_msg is not None, timeout, "/odom · /ground_truth (시뮬레이터)")
        self.wait_for(lambda: self.tf.can_transform("map", "base_footprint", Time()), timeout, "/tf 의 map → base_footprint (AMCL)")

    # --- 위치 읽기 ------------------------------------------------------
    def odom(self):
        """오도메트리: 처음 자리 기준 (x, y, 방향[rad])."""
        p = self.odom_msg.pose.pose
        return p.position.x, p.position.y, yaw_of(p.orientation)

    def speed(self):
        """오도메트리의 (전진 속도 [m/s], 회전 속도 [rad/s])."""
        return self.odom_msg.twist.twist.linear.x, self.odom_msg.twist.twist.angular.z

    def truth(self):
        """참값: 지도 좌표계의 (x, y, 방향[rad]). 채점에만 쓴다."""
        p = self.truth_msg.pose.pose
        return p.position.x, p.position.y, yaw_of(p.orientation)

    def estimate(self):
        """추정: AMCL 이 생각하는 지도 위의 (x, y, 방향[rad]). /tf 의 map → odom → base_footprint."""
        t = self.tf.lookup_transform("map", "base_footprint", Time()).transform
        return t.translation.x, t.translation.y, yaw_of(t.rotation)

    # --- 움직이기 -------------------------------------------------------
    def publish(self, linear, angular):
        msg = Twist()
        msg.linear.x, msg.angular.z = float(linear), float(angular)
        self.cmd.publish(msg)

    def go_to(self, x, y, yaw, timeout=180.0, tries=2, near=None):
        """Nav2 에 목적지를 주고 끝날 때까지 기다린다. (결과, 걸린 시간 [s]). 중단되면 한 번 더 보낸다 (같은 자리라 다시 보내도 됨).

        near = (거리 [m], 시간 [s]) 을 주면: 추정한 위치가 목표에서 그 거리 안에 든 채 그 시간 동안 2 cm 도 나아가지 않을 때 (제자리에서 도는 것은 됨),
        Nav2 가 도착이라고 하지 않아도 목표를 거두고 "가까이서 멈춤" 으로 끝낸다 (강의 45: 다음 모드가 받아 줄 수 있는 범위).
        """
        start = self.now()
        for attempt in range(tries):
            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = "map"
            goal.pose.header.stamp = self.get_clock().now().to_msg()
            goal.pose.pose.position.x, goal.pose.pose.position.y = float(x), float(y)
            goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
            sent = self.nav.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, sent, timeout_sec=10.0)
            if sent.result() is None or not sent.result().accepted:
                return "받지 않음", self.now() - start
            done = sent.result().get_result_async()
            anchor = None                                   # 목표 가까이에서 나아가지 않기 시작한 (때, 오도메트리의 x, y)
            while not done.done():
                rclpy.spin_once(self, timeout_sec=0.05)
                if self.now() - start > timeout:
                    sent.result().cancel_goal_async()
                    return "시간 초과", self.now() - start
                if near and self.tf.can_transform("map", "base_footprint", Time()):
                    distance, _ = pose_error(self.estimate(), (x, y, yaw))
                    here = self.odom()
                    if distance > near[0]:
                        anchor = None
                    elif anchor is None or math.hypot(here[0] - anchor[1], here[1] - anchor[2]) > 0.02:
                        anchor = (self.now(), here[0], here[1])
                    elif self.now() - anchor[0] >= near[1]:
                        print(f"            Nav2 가 목표에서 {distance * 100:.0f} cm 떨어진 곳(추정)에서 {near[1]:g}초째 나아가지 않음: 목표를 거둠", flush=True)
                        cancel = sent.result().cancel_goal_async()          # 목표를 거두고, 거둬졌다는 답을 기다린다
                        rclpy.spin_until_future_complete(self, cancel, timeout_sec=5.0)
                        rclpy.spin_until_future_complete(self, done, timeout_sec=5.0)
                        return "가까이서 멈춤", self.now() - start
            status = done.result().status
            if status == GoalStatus.STATUS_SUCCEEDED or attempt == tries - 1:
                return STATUS.get(status, f"상태 {status}"), self.now() - start
            self.get_logger().warning("Nav2 가 중단함: 한 번 더 보냄")
            self.spin(1.0)

    def arrival(self, goal):
        """도착한 자리의 채점: (참값, 추정, 도착 오차, 추정 오차)."""
        self.spin(0.3)                                  # 가장 새 /tf · 참값을 받는다
        truth, estimate = self.truth(), self.estimate()
        return truth, estimate, pose_error(truth, goal), pose_error(estimate, truth)


def show_arrival(goal, truth, estimate, arrival, guess):
    row = lambda name, pose: f"    {name:<12}{pose[0]:8.3f}{pose[1]:8.3f}{math.degrees(pose[2]):8.1f}"
    print(f"    {'':<12}{'x[m]':>8}{'y[m]':>8}{'방향[°]':>7}")
    print(row("목표", goal))
    print(row("추정 (AMCL)", estimate))
    print(row("참값", truth))
    print(f"    도착 오차 (참값 - 목표): 거리 {arrival[0] * 100:.1f} cm, 방향 {math.degrees(arrival[1]):+.1f}°")
    print(f"    추정 오차 (추정 - 참값): 거리 {guess[0] * 100:.1f} cm, 방향 {math.degrees(guess[1]):+.1f}°", flush=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("target", nargs="*", help="스테이션 이름, 또는 x y 방향[°]")
    parser.add_argument("--tour", type=int, metavar="N", help="음료대 → 상담 1 → 상담 2 → 처음 자리를 N 바퀴")
    parser.add_argument("--save", metavar="JSON", help="--tour 의 결과를 이 파일에 저장")
    args = parser.parse_args()
    if args.tour is None and len(args.target) not in (1, 3):
        parser.error("스테이션 이름 하나, 또는 x y 방향[°] 셋을 줄 것")
    if len(args.target) == 1 and args.target[0] not in STATIONS:
        parser.error(f"스테이션은 {', '.join(STATIONS)} 가운데 하나")

    rclpy.init()
    base = Base()
    base.ready()
    if args.tour is None:
        if len(args.target) == 1:
            goal, label = STATIONS[args.target[0]]["goal"], STATIONS[args.target[0]]["label"]
        else:
            goal, label = (float(args.target[0]), float(args.target[1]), math.radians(float(args.target[2]))), "지도 위의 자리"
        print(f"Nav2 에 목적지를 줌: {label} ({goal[0]:.2f}, {goal[1]:.2f}, {math.degrees(goal[2]):.0f}°)", flush=True)
        result, seconds = base.go_to(*goal)
        print(f"  →  {result} ({seconds:.1f}초)", flush=True)
        show_arrival(goal, *base.arrival(goal))
    else:
        rows = []
        print(f"{'바퀴':>4} {'목적지':<10}{'결과':<6}{'시간[s]':>7}{'도착 오차[cm]':>12}{'방향[°]':>8}{'추정 오차[cm]':>12}{'방향[°]':>8}", flush=True)
        for lap in range(1, args.tour + 1):
            for name in ("drinks", "consult_1", "consult_2", "home"):
                goal = STATIONS[name]["goal"]
                result, seconds = base.go_to(*goal)
                truth, estimate, arrival, guess = base.arrival(goal)
                rows.append({"lap": lap, "station": name, "result": result, "seconds": round(seconds, 2), "goal": list(goal),
                             "truth": [round(v, 4) for v in truth], "estimate": [round(v, 4) for v in estimate]})
                print(f"{lap:>4} {name:<13}{result:<6}{seconds:>8.1f}{arrival[0] * 100:>13.1f}{math.degrees(arrival[1]):>+10.1f}"
                      f"{guess[0] * 100:>14.1f}{math.degrees(guess[1]):>+10.1f}", flush=True)
        good = [row for row in rows if row["result"] == "도착"]
        errors = [pose_error(row["truth"], row["goal"]) for row in good]
        if errors:
            print(f"도착 {len(good)} / {len(rows)}번.  도착 오차: 거리 평균 {sum(e[0] for e in errors) / len(errors) * 100:.1f} cm · 최대 {max(e[0] for e in errors) * 100:.1f} cm, "
                  f"방향 최대 {max(abs(math.degrees(e[1])) for e in errors):.1f}°", flush=True)
        if args.save:
            with open(args.save, "w") as file:
                json.dump(rows, file, ensure_ascii=False)
            print("저장:", args.save)
    base.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
