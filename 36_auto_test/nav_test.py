"""강의 36: 같은 주행 시험을 사람 손 없이 여러 번 돌리고 통과율을 냅니다.

    python3 36_auto_test/nav_test.py --trials 6

시뮬레이터(pink_city.py --ros2 --sim-control)와 Nav2(강의 18)가 켜져 있어야 합니다.

시험 한 번
    1. 옮기기   /set_entity_state 로 로봇을 출발점에 순간 이동
    2. 알리기   Nav2 에게 새 위치를 알리고(initialpose), 지난 시험의 흔적을 비용 지도에서 지움
    3. 달리기   navigate_to_pose 로 목적지까지
    4. 판정     /get_entity_state 로 읽은 "실제 위치" 가 목적지에서 --tolerance 안이면 통과
"""
import argparse
import json
import math
import random

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from simulation_interfaces.srv import GetEntityState, SetEntityState

BODY = "/World/pinky/base_footprint"       # 로봇의 몸체. 위치를 읽을 때도 옮길 때도 이것을 씀 (/World/pinky 는 주행해도 값이 그대로)
# 시험할 길: 이름, 출발 (x, y, 방향[도]), 목적지 (x, y). 모두 지도 좌표 [m]
ROUTES = [("출발점 → 본사 건너편", (0.0, 0.0, 0.0), (1.0, -2.1)),
          ("본사 건너편 → 출발점", (1.0, -2.1, 180.0), (0.0, 0.0)),
          ("출발점 → 사거리", (0.0, 0.0, 0.0), (0.5, -1.05))]
STATUS = {4: "도착", 5: "취소", 6: "중단"}   # navigate_to_pose 가 끝났을 때의 상태 번호


def map_to_city(x, y, yaw_deg=0.0):
    """지도 좌표 → 시뮬레이터(도시) 좌표. 강의 18: 지도 (x, y) = 도시 (Y + 0.5, −X − 1.05)."""
    return -y - 1.05, x - 0.5, yaw_deg + 90.0


def city_to_map(x, y):
    return y + 0.5, -x - 1.05


class NavTest(Node):
    def __init__(self):
        super().__init__("nav_test", parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.get_state = self.create_client(GetEntityState, "/get_entity_state")
        self.set_state = self.create_client(SetEntityState, "/set_entity_state")
        self.clear = [self.create_client(ClearEntireCostmap, f"/{name}/clear_entirely_{name}")
                      for name in ("local_costmap", "global_costmap")]
        self.nav = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.initial = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self.on_amcl, latched)
        self.estimate = None                 # Nav2(AMCL)가 생각하는 로봇 위치 (지도 좌표)

    def on_amcl(self, message):
        self.estimate = (message.pose.pose.position.x, message.pose.pose.position.y)

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9      # 시뮬레이션 시간 [s]

    def wait(self, seconds):
        end = self.now() + seconds
        while rclpy.ok() and self.now() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def call(self, client, request, limit=10.0):
        if not client.wait_for_service(timeout_sec=limit):
            raise RuntimeError(f"서비스 {client.srv_name} 이 없습니다. 시뮬레이터와 Nav2 가 켜져 있는지 확인하세요.")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=limit)
        if future.result() is None:
            raise RuntimeError(f"서비스 {client.srv_name} 이 {limit:.0f}초 안에 답하지 않았습니다.")
        return future.result()

    def true_position(self):
        """시뮬레이터가 아는 로봇의 실제 위치를 지도 좌표로."""
        answer = self.call(self.get_state, GetEntityState.Request(entity=BODY))
        if answer.result.result != 1:                        # 1 = 성공
            raise RuntimeError(f"위치 읽기 실패: {answer.result.error_message}")
        return city_to_map(answer.state.pose.position.x, answer.state.pose.position.y)

    def move_robot(self, x, y, yaw_deg):
        """1. 옮기기: 로봇을 지도 좌표 (x, y) 로 순간 이동하고, 정말 그 자리에 있는지 읽어서 확인한다."""
        city_x, city_y, city_yaw = map_to_city(x, y, yaw_deg)
        request = SetEntityState.Request(entity=BODY)
        request.state.pose.position.x, request.state.pose.position.y = city_x, city_y
        request.state.pose.orientation.z = math.sin(math.radians(city_yaw) / 2)
        request.state.pose.orientation.w = math.cos(math.radians(city_yaw) / 2)
        answer = self.call(self.set_state, request)             # 속도(twist)는 0 으로 간다 = 멈춘 채로 옮겨짐
        if answer.result.result != 1:
            raise RuntimeError(f"순간 이동 실패: {answer.result.error_message}")
        self.wait(1.0)
        if math.dist(self.true_position(), (x, y)) > 0.05:
            raise RuntimeError("순간 이동을 했는데 로봇이 그 자리에 없습니다.")

    def tell_nav2(self, x, y, yaw_deg):
        """2. 알리기: Nav2 의 위치 추정이 새 자리로 옮겨질 때까지 알려 주고, 비용 지도를 비운다. 성공하면 True."""
        message = PoseWithCovarianceStamped()
        message.header.frame_id = "map"
        message.pose.pose.position.x, message.pose.pose.position.y = x, y
        message.pose.pose.orientation.z = math.sin(math.radians(yaw_deg) / 2)
        message.pose.pose.orientation.w = math.cos(math.radians(yaw_deg) / 2)
        message.pose.covariance[0] = message.pose.covariance[7] = 0.02      # x, y 의 불확실한 정도
        message.pose.covariance[35] = 0.02                                  # 방향의 불확실한 정도
        self.estimate, end = None, self.now() + 10.0
        while self.now() < end:
            if self.estimate is not None and math.dist(self.estimate, (x, y)) < 0.10:
                break
            self.initial.publish(message)
            self.wait(0.5)
        else:
            return False
        for client in self.clear:
            self.call(client, ClearEntireCostmap.Request())
        self.wait(1.0)                       # 비운 비용 지도가 새 라이다 값으로 다시 채워지는 시간
        return True

    def drive(self, x, y, limit):
        """3. 달리기: 목적지를 주고 끝날 때까지 기다린다. (상태 이름, 걸린 시간[s]) 을 돌려준다."""
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.pose.position.x, goal.pose.pose.position.y = x, y
        goal.pose.pose.orientation.w = 1.0
        if not self.nav.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("navigate_to_pose 가 없습니다. Nav2 가 켜져 있는지 확인하세요.")
        sent = self.nav.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, sent, timeout_sec=10.0)
        handle = sent.result()
        if handle is None or not handle.accepted:
            return "거절", 0.0
        start, result = self.now(), handle.get_result_async()
        while not result.done() and self.now() - start < limit:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not result.done():
            handle.cancel_goal_async()
            self.wait(1.0)
            return "시간 초과", self.now() - start
        return STATUS.get(result.result().status, str(result.result().status)), self.now() - start

    def trial(self, number, route, jitter, tolerance, limit, rng):
        name, (start_x, start_y, start_yaw), (goal_x, goal_y) = route
        start_x += rng.uniform(-jitter, jitter)
        start_y += rng.uniform(-jitter, jitter)
        self.move_robot(start_x, start_y, start_yaw)
        if self.tell_nav2(start_x, start_y, start_yaw):
            status, seconds = self.drive(goal_x, goal_y, limit)
        else:
            status, seconds = "위치 추정 실패", 0.0
        self.wait(1.0)
        error = math.dist(self.true_position(), (goal_x, goal_y))                       # 4. 판정
        estimate_error = math.dist(self.true_position(), self.estimate) if self.estimate else float("nan")
        passed = status == "도착" and error <= tolerance
        print(f"{number:>3}  {name:<14} {status:<8} {seconds:5.1f}초  목적지까지 {error * 100:5.1f} cm  "
              f"추정 오차 {estimate_error * 100:4.1f} cm  →  {'통과' if passed else '실패'}", flush=True)
        return {"trial": number, "route": name, "start": [round(start_x, 3), round(start_y, 3)], "goal": [goal_x, goal_y],
                "status": status, "seconds": round(seconds, 1), "error_m": round(error, 3),
                "estimate_error_m": round(estimate_error, 3), "pass": passed}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--trials", type=int, default=6, help="시험 횟수 (길 3개를 차례로 돈다)")
    parser.add_argument("--tolerance", type=float, default=0.10, help="목적지에서 이 거리 안에 서면 통과 [m]")
    parser.add_argument("--jitter", type=float, default=0.0, help="출발 위치를 매번 최대 이만큼 흔든다 [m]")
    parser.add_argument("--limit", type=float, default=90.0, help="한 번의 주행을 기다리는 최대 시간 [s]")
    parser.add_argument("--seed", type=int, default=0, help="--jitter 의 난수 씨앗")
    parser.add_argument("--out", metavar="JSON", help="결과를 이 파일에 저장한다")
    args = parser.parse_args()

    rclpy.init()
    node = NavTest()
    rng = random.Random(args.seed)
    print(f"시험 {args.trials}번, 통과 기준: 도착 + 목적지에서 {args.tolerance * 100:.0f} cm 안", flush=True)
    results = [node.trial(k + 1, ROUTES[k % len(ROUTES)], args.jitter, args.tolerance, args.limit, rng)
               for k in range(args.trials)]
    passed = sum(r["pass"] for r in results)
    print(f"통과 {passed}/{len(results)} ({100 * passed / len(results):.0f}%)", flush=True)
    for name in dict.fromkeys(r["route"] for r in results):          # 길마다 한 줄씩
        rows = [r for r in results if r["route"] == name]
        arrived = [r for r in rows if r["status"] == "도착"]
        line = f"  {name:<14} 통과 {sum(r['pass'] for r in rows)}/{len(rows)}  도착 {len(arrived)}/{len(rows)}"
        if arrived:
            line += (f"  평균 {sum(r['seconds'] for r in arrived) / len(arrived):.1f}초"
                     f"  목적지까지 평균 {100 * sum(r['error_m'] for r in arrived) / len(arrived):.1f} cm")
        print(line, flush=True)
    if args.out:
        with open(args.out, "w") as file:
            json.dump({"tolerance": args.tolerance, "jitter": args.jitter, "seed": args.seed, "results": results},
                      file, ensure_ascii=False, indent=1)
        print("결과 저장:", args.out, flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
