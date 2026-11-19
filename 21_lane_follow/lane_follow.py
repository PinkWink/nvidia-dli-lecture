"""강의 21: 카메라 영상에서 도로를 알아보고(세그먼테이션) 차선을 따라 달리는 ROS 2 노드.

    source /opt/ros/jazzy/setup.bash
    python3 ~/nvidia-dli-lecture/21_lane_follow/lane_follow.py
    python3 ~/nvidia-dli-lecture/21_lane_follow/lane_follow.py --ros-args -p speed:=0.2 -p gain:=1.8

받기     /camera/image_raw   sensor_msgs/Image   로봇 카메라 (강의 15)
내보내기  /cmd_vel            geometry_msgs/Twist  바퀴 속도 명령 (강의 14)
        /lane/debug         sensor_msgs/Image    알아본 결과를 색으로 겹치고, 목표 지점을 그린 그림 (RViz2 로 본다)

하는 일 (영상 한 장마다)
    1. 영상을 320 × 180 으로 줄여 강의 20 의 모델(road_seg.onnx)에 넣는다 → 픽셀마다 종류 번호
    2. 로봇 앞 약 0.2 ~ 0.3 m 를 보는 띠(행 105 ~ 135)에서 중앙선과 도로의 오른쪽 끝을 찾는다
    3. 그 둘의 가운데(= 오른쪽 차로의 가운데)가 화면 가운데에 오도록 방향을 튼다

필요한 것: python3-opencv, python3-numpy (ROS 2 를 설치했다면 이미 있습니다). PyTorch 는 쓰지 않습니다.
"""
import os
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Point, Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

HERE = os.path.dirname(os.path.abspath(__file__))
MODEL = os.path.abspath(os.path.join(HERE, "..", "work", "road_model", "road_seg.onnx"))
WIDTH, HEIGHT, MEAN, STD = 320, 180, 0.5, 0.25            # 강의 20 의 train.py 와 같은 값
ROAD, LANE = 2, 3                                         # 종류 번호: 2 도로, 3 중앙선, 4 횡단보도 (2 이상이면 차가 다니는 곳)
ROWS = (105, 135)                                         # 목표를 찾는 띠 (모델 출력의 행). 로봇 앞 약 0.2 ~ 0.3 m
LANE_WIDTH = 190.0                                        # 그 거리에서 차로 하나(0.2 m)가 차지하는 폭 [픽셀]
PALETTE = np.array([(40, 40, 48), (232, 150, 190), (90, 96, 110), (250, 204, 60), (255, 255, 255)], dtype=np.uint8)   # RGB


def segment(net, rgb):
    """RGB 영상 한 장 → 종류 번호 그림 (180 × 320)."""
    small = cv2.resize(rgb, (WIDTH, HEIGHT), interpolation=cv2.INTER_AREA)
    x = ((small.astype(np.float32) / 255.0 - MEAN) / STD).transpose(2, 0, 1)[None]
    net.setInput(x)
    return net.forward()[0].argmax(0).astype(np.uint8)


def lane_target(mask):
    """오른쪽 차로의 가운데가 화면의 몇 번째 열인지 찾는다 → (목표 열, 중앙선 열 또는 None, 도로 오른쪽 끝 열). 못 찾으면 None."""
    band = mask[ROWS[0]:ROWS[1]]
    drivable = band >= ROAD
    if drivable.sum() < 200:
        return None                                       # 도로가 거의 안 보인다
    columns = np.arange(WIDTH)
    road_columns = drivable.sum(0) > (ROWS[1] - ROWS[0]) * 0.4       # 띠의 40 % 이상이 도로인 열
    if not road_columns.any():
        return None
    yellow = band == LANE
    if yellow.sum() >= 12:                                # 중앙선이 보인다: 중앙선과 도로 오른쪽 끝의 가운데로
        lane_x = float((yellow.sum(0) * columns).sum() / yellow.sum())
        right = columns[road_columns & (columns > lane_x)]
        right_x = min(float(right.max()) if len(right) else lane_x + LANE_WIDTH, lane_x + LANE_WIDTH + 10.0)
        return (lane_x + right_x) / 2.0, lane_x, right_x
    left_x, right_x = float(columns[road_columns].min()), float(columns[road_columns].max())
    if right_x - left_x < 30:
        return None
    if right_x < WIDTH - 2:                               # 중앙선은 안 보이지만 도로 오른쪽 끝은 보인다: 끝에서 차로 반 폭 안쪽
        return right_x - LANE_WIDTH / 2.0, None, right_x
    return left_x + LANE_WIDTH * 1.5, None, right_x       # 왼쪽 끝만 보인다: 왼쪽 끝에서 차로 하나 반


class LaneFollow(Node):
    def __init__(self):
        super().__init__("lane_follow")
        self.speed = self.declare_parameter("speed", 0.15).value        # 직진 속도 [m/s]
        self.gain = self.declare_parameter("gain", 1.6).value           # 목표가 화면 끝에 있을 때의 회전 속도 [rad/s]
        model = self.declare_parameter("model", MODEL).value
        # 강의 28: 앞의 Pinky Pro 가 이만큼 크게(상자 높이 / 화면 높이) 보이면 선다. 0 이면 쓰지 않는다
        self.stop_for_pinky = self.declare_parameter("stop_for_pinky", 0.0).value
        self.pinky = None                                 # 마지막으로 받은 /pinky/nearest 와 받은 시각
        if not os.path.exists(model):
            raise SystemExit(f"모델이 없습니다: {model}\n먼저 실행:  ~/isaacsim/python.sh 20_road_segmentation/train.py")
        self.net = cv2.dnn.readNetFromONNX(model)
        self.error = 0.0                                  # 마지막으로 본 목표의 치우침 (−1 왼쪽 끝 ~ +1 오른쪽 끝)
        self.count = 0
        self.lost = 0
        self.command = self.create_publisher(Twist, "/cmd_vel", 10)
        self.debug = self.create_publisher(Image, "/lane/debug", 1)
        self.create_subscription(Image, "/camera/image_raw", self.on_image, qos_profile_sensor_data)
        if self.stop_for_pinky > 0.0:
            self.create_subscription(Point, "/pinky/nearest", self.on_pinky, 2)
        self.create_timer(2.0, self.report)
        self.get_logger().info(f"모델 {os.path.basename(model)}, 속도 {self.speed} m/s, 게인 {self.gain}. /camera/image_raw 를 기다립니다.")

    def on_pinky(self, message):
        self.pinky = (message, time.monotonic())

    def blocked(self):
        """앞쪽(화면 가운데 근처)에 Pinky Pro 가 가까이 있는가. 0.5초 넘게 소식이 없으면 없는 것으로 본다."""
        if self.pinky is None or time.monotonic() - self.pinky[1] > 0.5:
            return False
        nearest = self.pinky[0]
        return nearest.z > 0.0 and abs(nearest.x) < 0.6 and nearest.y >= self.stop_for_pinky

    def on_image(self, message):
        if message.encoding not in ("rgb8", "bgr8"):
            self.get_logger().warn(f"지원하지 않는 영상 형식: {message.encoding}", once=True)
            return
        image = np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, message.step // 3, 3)[:, :message.width]
        rgb = image if message.encoding == "rgb8" else image[:, :, ::-1]
        mask = segment(self.net, rgb)
        target = lane_target(mask)
        if target is not None:
            self.error = (target[0] - WIDTH / 2) / (WIDTH / 2)
            self.lost = 0
        else:
            self.lost += 1
        command = Twist()
        if self.blocked():                                # 앞에 다른 로봇이 있으면 선다 (강의 28)
            pass
        elif self.lost < 30:                              # 잠깐 놓친 동안은 마지막 방향대로 간다. 오래 놓치면 선다
            command.angular.z = float(np.clip(-self.gain * self.error, -1.5, 1.5))     # 목표가 오른쪽(+)이면 오른쪽(−)으로 돈다
            command.linear.x = float(self.speed * max(0.35, 1.0 - abs(self.error)))   # 많이 틀어야 할수록 천천히
        self.command.publish(command)
        self.count += 1
        if self.debug.get_subscription_count() > 0:
            self.publish_debug(message, rgb, mask, target)

    def publish_debug(self, message, rgb, mask, target):
        """영상 위에 알아본 결과를 색으로 겹치고, 띠 · 중앙선 · 도로 끝 · 목표 지점을 그린다."""
        small = cv2.resize(rgb, (WIDTH, HEIGHT), interpolation=cv2.INTER_AREA)
        view = cv2.addWeighted(small, 0.45, PALETTE[mask], 0.55, 0.0)
        view = cv2.resize(view, (WIDTH * 2, HEIGHT * 2), interpolation=cv2.INTER_NEAREST)
        y0, y1 = ROWS[0] * 2, ROWS[1] * 2
        cv2.rectangle(view, (0, y0), (WIDTH * 2 - 1, y1), (255, 255, 255), 1)
        if target is not None:
            x, lane_x, right_x = target
            if lane_x is not None:
                cv2.line(view, (int(lane_x * 2), y0 - 8), (int(lane_x * 2), y1 + 8), (250, 204, 60), 3)
            cv2.line(view, (int(right_x * 2), y0 - 8), (int(right_x * 2), y1 + 8), (255, 255, 255), 3)
            cv2.line(view, (WIDTH, HEIGHT * 2 - 1), (int(x * 2), (y0 + y1) // 2), (214, 72, 140), 4)
            cv2.circle(view, (int(x * 2), (y0 + y1) // 2), 9, (214, 72, 140), -1)
            cv2.circle(view, (int(x * 2), (y0 + y1) // 2), 9, (255, 255, 255), 2)
        out = Image()
        out.header = message.header
        out.height, out.width, out.encoding, out.step = view.shape[0], view.shape[1], "rgb8", view.shape[1] * 3
        out.data = view.tobytes()
        self.debug.publish(out)

    def report(self):
        if self.count:
            state = "앞에 Pinky Pro 가 있어 정지" if self.blocked() else "차선 추종" if self.lost < 30 else "도로를 놓쳐 정지"
            self.get_logger().info(f"{state}: 영상 {self.count / 2.0:.0f} 장/초, 치우침 {self.error:+.2f}")
        self.count = 0

    def stop(self):
        try:                                              # Ctrl+C 뒤에는 이미 닫혀 있을 수 있다
            for _ in range(5):
                self.command.publish(Twist())
        except Exception:
            pass


def main():
    rclpy.init()
    node = LaneFollow()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:                                     # Ctrl+C 와 겹쳐 난 오류는 무시하고, 그 밖의 오류는 그대로 알린다
        if rclpy.ok():
            raise
    node.stop()
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
