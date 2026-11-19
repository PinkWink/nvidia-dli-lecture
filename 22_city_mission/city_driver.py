"""강의 22: 장소 이름으로 명령하면, 차선을 지키며 찾아가는 ROS 2 노드.

    source /opt/ros/jazzy/setup.bash
    python3 ~/nvidia-dli-lecture/22_city_mission/city_driver.py 핑크랩 공원          # 핑크랩에 들렀다가 공원으로
    python3 ~/nvidia-dli-lecture/22_city_mission/city_driver.py 아파트
    python3 ~/nvidia-dli-lecture/22_city_mission/city_driver.py --list               # 장소 목록

세 가지를 합친 것입니다.
    어디에 있나     AMCL (강의 18) 이 내보내는 map → base_footprint 좌표
    어느 길로 갈까   도로망(교차로 5개 · 도로 8개)에서 가장 짧은 길 찾기
    어떻게 달릴까    교차로 사이는 카메라로 차선 추종 (강의 21), 교차로 안은 지도상 위치를 보고 정해진 곡선을 따라 통과

받기     /camera/image_raw, tf (map → base_footprint)
내보내기  /cmd_vel, /lane/debug
"""
import heapq
import math
import os
import sys

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image
from tf2_ros import Buffer, TransformListener

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "21_lane_follow"))
import cv2
from lane_follow import HEIGHT, MODEL, PALETTE, ROWS, WIDTH, lane_target, segment

# ---------------------------------------------------------------- 도시의 도로망 (도시 좌표, 단위 m)
RING, CORNER, LANE = 1.05, 0.30, 0.10          # 순환 도로 중심선까지의 거리, 모퉁이 반지름, 차로 가운데까지의 거리
START = (-1.05, -0.5, 90.0)                    # 지도를 만들 때 로봇이 있던 자리 = 지도 좌표의 원점 (강의 17)
NODES = {"가운데": (0.0, 0.0), "북": (0.0, RING), "남": (0.0, -RING), "동": (RING, 0.0), "서": (-RING, 0.0)}      # 교차로
ZONE = {"가운데": 0.48, "북": 0.36, "남": 0.36, "동": 0.36, "서": 0.36}                                          # 교차로로 치는 반지름
PLACES = {            # 장소 이름 → 그 앞 도로 위의 점
    "핑크랩": (0.525, 0.0),            # 본사 앞 (가운데 ~ 동 도로)
    "아카데미": (-0.655, 0.0),         # 핑크 아카데미 앞 (가운데 ~ 서 도로)
    "공원": (0.0, -0.525),            # 공원 옆 (가운데 ~ 남 도로)
    "아파트": (RING, -0.55),           # 아파트 옆 (동 ~ 남 순환 도로)
}


def _ring_edge(a, b, corner, start_angle):
    """순환 도로의 한 구간: 교차로 a → 직선 → 모퉁이(1/4 원) → 직선 → 교차로 b."""
    points = [NODES[a]]
    for k in range(0, 19):
        r = math.radians(start_angle - 90.0 * k / 18)                # 시계 방향으로 돈다
        points.append((corner[0] + CORNER * math.cos(r), corner[1] + CORNER * math.sin(r)))
    points.append(NODES[b])
    return points


s = RING - CORNER
EDGES = {             # 도로 이름 → (시작 교차로, 끝 교차로, 중심선의 점들)
    "가운데-서": ("가운데", "서", [NODES["가운데"], NODES["서"]]),
    "가운데-북": ("가운데", "북", [NODES["가운데"], NODES["북"]]),
    "가운데-동": ("가운데", "동", [NODES["가운데"], NODES["동"]]),
    "가운데-남": ("가운데", "남", [NODES["가운데"], NODES["남"]]),
    "서-북": ("서", "북", _ring_edge("서", "북", (-s, s), 180.0)),
    "북-동": ("북", "동", _ring_edge("북", "동", (s, s), 90.0)),
    "동-남": ("동", "남", _ring_edge("동", "남", (s, -s), 0.0)),
    "남-서": ("남", "서", _ring_edge("남", "서", (-s, -s), -90.0)),
}


def _dense(points, step=0.01):
    """점들 사이를 1 cm 간격으로 채운다 → (N, 2) 배열과 각 점까지의 누적 거리."""
    out = [np.array(points[0], float)]
    for p, q in zip(points[:-1], points[1:]):
        p, q = np.array(p, float), np.array(q, float)
        n = max(1, int(np.linalg.norm(q - p) / step))
        out += [p + (q - p) * k / n for k in range(1, n + 1)]
    out = np.array(out)
    return out, np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(out, axis=0), axis=1))])


LINES = {name: _dense(points) for name, (_, _, points) in EDGES.items()}
LENGTH = {name: float(LINES[name][1][-1]) for name in EDGES}


def right_of(d):
    """진행 방향 d 의 오른쪽 방향."""
    return np.array([d[1], -d[0]])


def locate(xy, edge=None):
    """점에서 가장 가까운 도로와, 그 도로 시작점에서의 거리 → (도로 이름, 거리, 떨어진 정도)."""
    best = None
    for name in ([edge] if edge else EDGES):
        line, dist = LINES[name]
        k = int(np.argmin(np.linalg.norm(line - np.array(xy), axis=1)))
        off = float(np.linalg.norm(line[k] - np.array(xy)))
        if best is None or off < best[2]:
            best = (name, float(dist[k]), off)
    return best


def tangent(edge, along, direction):
    """도로 위 한 점에서의 진행 방향 (단위 벡터). direction: +1 시작 → 끝, −1 끝 → 시작."""
    line, dist = LINES[edge]
    k = int(np.clip(np.searchsorted(dist, along), 1, len(line) - 1))
    t = line[k] - line[k - 1]
    return direction * t / np.linalg.norm(t)


def end_node(edge, direction):
    return EDGES[edge][1] if direction > 0 else EDGES[edge][0]


def plan(edge, along, direction, place):
    """지금 (도로, 위치, 방향) 에서 장소까지 가장 짧은 길 (유턴 없음) → [(도로, 방향), …]. 첫 항목은 지금 달리는 도로."""
    goal_edge, goal_along, _ = locate(PLACES[place])
    remaining = LENGTH[edge] - along if direction > 0 else along
    ahead = (goal_along - along) * direction
    if goal_edge == edge and ahead > 0.02:
        return [(edge, direction)]                                    # 같은 도로의 앞쪽에 있다
    queue, seen, count = [(remaining, 0, [(edge, direction)], False)], set(), 0
    while queue:
        cost, _, path, done = heapq.heappop(queue)
        if done:
            return path                                               # 가장 먼저 꺼내진 것이 가장 짧은 길
        last_edge, last_dir = path[-1]
        if (last_edge, last_dir) in seen:
            continue
        seen.add((last_edge, last_dir))
        node = end_node(last_edge, last_dir)
        for name, (a, b, _) in EDGES.items():
            if name == last_edge or node not in (a, b):
                continue                                              # 왔던 길로 되돌아가지 않는다
            d = 1 if a == node else -1
            count += 1
            if name == goal_edge:
                reach = goal_along if d > 0 else LENGTH[name] - goal_along
                heapq.heappush(queue, (cost + reach, count, path + [(name, d)], True))
            heapq.heappush(queue, (cost + LENGTH[name], count + 1000000, path + [(name, d)], False))
    return None


def crossing(node, position, d_in, d_out):
    """교차로를 지나는 곡선: 지금 위치 → 나갈 도로의 오른쪽 차로 입구. (베지에 곡선의 점들)"""
    center = np.array(NODES[node])
    exit_point = center + d_out * (ZONE[node] + 0.04) + right_of(d_out) * LANE
    if abs(float(np.dot(d_in, d_out))) > 0.9:                         # 직진
        corner = center + right_of(d_in) * LANE
    else:                                                             # 우회전 · 좌회전: 들어온 차로와 나갈 차로가 만나는 점
        corner = center + right_of(d_in) * LANE + right_of(d_out) * LANE
    t = np.linspace(0.0, 1.0, 25)[:, None]
    return (1 - t) ** 2 * np.array(position) + 2 * (1 - t) * t * corner + t ** 2 * exit_point


def turn_name(d_in, d_out):
    cross = d_in[0] * d_out[1] - d_in[1] * d_out[0]
    return "직진" if abs(cross) < 0.3 else "좌회전" if cross > 0 else "우회전"


def map_to_city(x, y, yaw):
    """지도 좌표(출발점이 원점, 처음 보던 방향이 +x) → 도시 좌표."""
    a = math.radians(START[2])
    return START[0] + x * math.cos(a) - y * math.sin(a), START[1] + x * math.sin(a) + y * math.cos(a), yaw + a


class CityDriver(Node):
    def __init__(self, places):
        super().__init__("city_driver", parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        self.speed = self.declare_parameter("speed", 0.15).value
        self.gain = self.declare_parameter("gain", 1.6).value
        self.net = cv2.dnn.readNetFromONNX(self.declare_parameter("model", MODEL).value)
        self.todo = list(places)
        self.route, self.state, self.path, self.error, self.wait_until = None, "준비", None, 0.0, None
        self.pose = None
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.command = self.create_publisher(Twist, "/cmd_vel", 10)
        self.debug = self.create_publisher(Image, "/lane/debug", 1)
        self.create_subscription(Image, "/camera/image_raw", self.on_image, qos_profile_sensor_data)
        self.create_timer(3.0, self.report)
        self.get_logger().info("갈 곳: " + " → ".join(self.todo) + ". 위치(map → base_footprint)와 카메라 영상을 기다립니다.")

    # ------------------------------------------------------------ 위치
    def read_pose(self):
        try:
            t = self.tf.lookup_transform("map", "base_footprint", Time()).transform
        except Exception:
            return None
        q = t.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return map_to_city(t.translation.x, t.translation.y, yaw)

    # ------------------------------------------------------------ 영상 한 장마다
    def on_image(self, message):
        pose = self.read_pose()
        if pose is None or message.encoding not in ("rgb8", "bgr8"):
            return
        self.pose = pose
        position, heading = np.array(pose[:2]), np.array([math.cos(pose[2]), math.sin(pose[2])])
        image = np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, message.step // 3, 3)[:, :message.width]
        rgb = image if message.encoding == "rgb8" else image[:, :, ::-1]
        mask = segment(self.net, rgb)
        target = lane_target(mask)
        command = Twist()

        if self.state == "준비":
            self.start_next(position, heading)
        elif self.state == "도착":
            if self.get_clock().now().nanoseconds * 1e-9 >= self.wait_until:
                self.start_next(position, heading)
        elif self.state == "차선 추종":
            edge, direction = self.route[0]
            node = end_node(edge, direction)
            if len(self.route) == 1 and self.arrived(position, edge, direction):
                self.get_logger().info(f"도착: {self.goal}  (도시 좌표 {position[0]:+.2f}, {position[1]:+.2f})")
                self.state, self.wait_until = "도착", self.get_clock().now().nanoseconds * 1e-9 + 2.0
            elif len(self.route) > 1 and np.linalg.norm(position - np.array(NODES[node])) < ZONE[node]:
                d_in = tangent(edge, LENGTH[edge] if direction > 0 else 0.0, direction)
                next_edge, next_dir = self.route[1]
                d_out = tangent(next_edge, 0.0 if next_dir > 0 else LENGTH[next_edge], next_dir)
                self.path, self.index = crossing(node, position, d_in, d_out), 0
                self.state = "교차로 통과"
                self.get_logger().info(f"교차로 '{node}': {turn_name(d_in, d_out)} → {next_edge}")
            else:                                                     # 강의 21 의 차선 추종
                if target is not None:
                    self.error = (target[0] - WIDTH / 2) / (WIDTH / 2)
                command.angular.z = float(np.clip(-self.gain * self.error, -1.5, 1.5))
                command.linear.x = float(self.speed * max(0.35, 1.0 - abs(self.error)))
        if self.state == "교차로 통과":                                 # 지도상 위치를 보고 정해 둔 곡선을 따라간다
            while self.index < len(self.path) - 1 and np.linalg.norm(self.path[self.index] - position) < 0.10:
                self.index += 1                                       # 0.1 m 앞의 점을 겨눈다
            aim = self.path[self.index] - position
            turn = (math.atan2(aim[1], aim[0]) - pose[2] + math.pi) % (2 * math.pi) - math.pi
            command.angular.z = float(np.clip(2.5 * turn, -1.6, 1.6))
            command.linear.x = float(self.speed * max(0.25, 1.0 - abs(turn) / 0.8))
            if self.index >= len(self.path) - 1 and np.linalg.norm(aim) < 0.06:
                self.route.pop(0)
                self.state, self.error = "차선 추종", 0.0
        self.command.publish(command)
        if self.debug.get_subscription_count() > 0:
            self.publish_debug(message, rgb, mask, target)

    def start_next(self, position, heading):
        """다음 장소까지의 길을 찾는다."""
        if not self.todo:
            self.get_logger().info("모든 장소에 들렀습니다. 끝냅니다.")
            self.state = "끝"
            return
        self.goal = self.todo.pop(0)
        edge, along, off = locate(position)
        direction = 1 if float(np.dot(tangent(edge, along, 1), heading)) > 0 else -1
        self.route = plan(edge, along, direction, self.goal)
        nodes = [end_node(e, d) for e, d in self.route[:-1]]
        self.get_logger().info(f"출발 → {self.goal}: 지금 '{edge}' 도로, 지나갈 교차로 " + (" → ".join(nodes) if nodes else "없음"))
        self.state = "차선 추종"

    def arrived(self, position, edge, direction):
        """장소 앞을 지나는 순간인가: 장소까지 진행 방향으로 남은 거리가 0 이하."""
        place = np.array(PLACES[self.goal])
        along = locate(place, edge)[1]
        return float(np.dot(place - position, tangent(edge, along, direction))) <= 0.0 and np.linalg.norm(place - position) < 0.3

    def publish_debug(self, message, rgb, mask, target):
        small = cv2.resize(rgb, (WIDTH, HEIGHT), interpolation=cv2.INTER_AREA)
        view = cv2.resize(cv2.addWeighted(small, 0.45, PALETTE[mask], 0.55, 0.0), (WIDTH * 2, HEIGHT * 2), interpolation=cv2.INTER_NEAREST)
        if self.state == "차선 추종" and target is not None:
            y = (ROWS[0] + ROWS[1])
            cv2.line(view, (WIDTH, HEIGHT * 2 - 1), (int(target[0] * 2), y), (214, 72, 140), 4)
            cv2.circle(view, (int(target[0] * 2), y), 9, (214, 72, 140), -1)
        color = {"차선 추종": (60, 170, 90), "교차로 통과": (60, 110, 220)}.get(self.state, (120, 120, 130))
        cv2.rectangle(view, (0, 0), (WIDTH * 2 - 1, 34), color, -1)
        label = {"차선 추종": "LANE FOLLOW (camera)", "교차로 통과": "INTERSECTION (map position)", "도착": "ARRIVED"}.get(self.state, "...")
        cv2.putText(view, label, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        out = Image()
        out.header = message.header
        out.height, out.width, out.encoding, out.step = view.shape[0], view.shape[1], "rgb8", view.shape[1] * 3
        out.data = view.tobytes()
        self.debug.publish(out)

    def report(self):
        if self.pose is not None and self.state in ("차선 추종", "교차로 통과"):
            self.get_logger().info(f"{self.state}: 위치 ({self.pose[0]:+.2f}, {self.pose[1]:+.2f}), 목적지 {self.goal}")

    def stop(self):
        try:
            for _ in range(5):
                self.command.publish(Twist())
        except Exception:
            pass


def main():
    names = [a for a in sys.argv[1:] if not a.startswith("-")]
    if "--list" in sys.argv or not names:
        print("장소:", ", ".join(f"{k} {v}" for k, v in PLACES.items()))
        print("사용법: python3 city_driver.py 핑크랩 공원")
        return
    unknown = [n for n in names if n not in PLACES]
    if unknown:
        raise SystemExit(f"모르는 장소: {', '.join(unknown)}. 아는 장소: {', '.join(PLACES)}")
    rclpy.init(args=[sys.argv[0]] + [a for a in sys.argv[1:] if a.startswith("-") or a not in names])
    node = CityDriver(names)
    try:
        while rclpy.ok() and node.state != "끝":
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    except Exception:
        if rclpy.ok():
            raise
    node.stop()
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
