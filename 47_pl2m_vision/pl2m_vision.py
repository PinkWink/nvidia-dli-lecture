"""강의 47: PL-2M 의 머리 카메라(색 + 깊이)를 ROS 2 로 냅니다. 컵을 찾는 쪽(cup_detector.py)이 이 영상을 받습니다.

    source /opt/ros/jazzy/setup.bash                                              # Isaac Sim 보다 먼저
    ~/isaacsim/python.sh 47_pl2m_vision/pl2m_vision.py --start drinks             # 음료대 앞(Nav2 의 목적지)에서 시작
    ~/isaacsim/python.sh 47_pl2m_vision/pl2m_vision.py --start drinks --docked    # 음료대에 도킹한 자리에서 시작
    ~/isaacsim/python.sh 47_pl2m_vision/pl2m_vision.py --start drinks --docked --shuffle 3
                                                                                  # 컵을 음료대 위 아무 데나 놓는다 (3 = 무작위 씨앗)
    ~/isaacsim/python.sh 47_pl2m_vision/pl2m_vision.py --start drinks --docked --cups cup_pink=7.60,-1.60 cup_blue=7.80,-1.70
                                                                                  # 컵의 자리를 직접 정한다 (부스 좌표 x,y)
    ~/isaacsim/python.sh 47_pl2m_vision/pl2m_vision.py --gui                      # 창으로 보기

강의 46 의 장면(부스 · PL-2M · 라이다 · 마커 · 앞 카메라 · 쥐기 설정)에 더하는 것
    머리 카메라    head_camera_link 자리 (바닥에서 1.10 m, 아래로 14.3° 숙임). 640 × 480, 세로 화각 60°, 1초에 10장
    내는 토픽      /head_camera/image_raw (색)   /head_camera/depth (깊이 [m])   /head_camera/camera_info
                   /cups/truth (컵의 참값. 채점할 때만 쓴다)
"""
import argparse
import math
import os
import random
import signal
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, ".."))
HEAD_GRAPH = "/Graphs/ros2_head_camera"
HEAD_LINK = "/World/pl2m/openarm_body_link0/head_camera_link"     # URDF 의 카메라 자리: 몸통 위, +X 가 앞
HEAD_CAMERA = HEAD_LINK + "/Camera"
HEAD_SIZE, HEAD_FOVY, HEAD_EVERY = (640, 480), 60.0, 6           # 크기 [픽셀], 세로 화각 [°] (원본 cameras.yaml), 화면 6번에 한 장
STAND = (7.65, -1.60, 0.62, 0.42)                                 # 음료대 상판: 가운데 (x, y), 크기 (x, y) [m]
STAND_TOP = 0.75                                                  # 음료대 상판의 높이 [m]

if __name__ == "__main__":
    sys.path.insert(0, os.path.join(REPO, "44_pl2m_nav"))
    from booth import STATIONS
    parser = argparse.ArgumentParser()
    parser.add_argument("--start", choices=list(STATIONS), default="home", help="시작하는 자리")
    parser.add_argument("--docked", action="store_true", help="--start 의 스테이션에 도킹을 마친 자리에서 시작한다")
    parser.add_argument("--offset", type=float, nargs=2, metavar=("SIDE", "YAW"), default=(0.0, 0.0), help="시작 자리를 왼쪽으로 SIDE [m], 왼쪽으로 YAW [°] 튼다")
    parser.add_argument("--shuffle", type=int, metavar="SEED", help="컵 세 개를 음료대 위 아무 데나 놓는다")
    parser.add_argument("--cups", nargs="+", metavar="NAME=X,Y", default=[], help="컵의 자리를 정한다 (부스 좌표). 예: cup_pink=7.60,-1.60")
    parser.add_argument("--gui", action="store_true", help="창을 띄운다")
    parser.add_argument("--seconds", type=float, help="이 시간만큼만 돌리고 끝낸다")
    args = parser.parse_args()

    from isaacsim import SimulationApp
    app = SimulationApp({"headless": not args.gui, "extra_args": [
        "--/app/runLoops/main/rateLimitEnabled=true", "--/app/runLoops/main/rateLimitFrequency=60"]})

import omni.graph.core as og
import usdrt.Sdf
from pxr import Gf, UsdGeom

sys.path.insert(0, HERE)
sys.path.insert(0, REPO)
for folder in ("38_pl2m_booth", "39_pl2m_drive", "40_pl2m_gravity", "41_pl2m_ros2", "43_pl2m_pick", "44_pl2m_nav", "45_pl2m_dock", "46_pl2m_mission"):
    sys.path.insert(0, os.path.join(REPO, folder))
import make_booth
import pl2m_pick as grasp
from common.sim import Sim, new_scene, set_pose, set_view
from markers import docked_pose
from pl2m_dock import VIEWS, build_dock_scene, start_pose
from pl2m_drive import ARM_JOINTS
from pl2m_mission import CUP_NAMES, cup_states, prepare_grasp
from pl2m_nav import BasePose
from pl2m_ros2 import JointCommandInterpolator, enable_ros2


def add_head_camera(stage):
    """머리 카메라: head_camera_link 의 자식. 링크의 +X 가 카메라가 보는 방향이 되게 돌린다 (강의 45 의 앞 카메라와 같은 회전)."""
    camera = UsdGeom.Camera.Define(stage, HEAD_CAMERA)
    aperture = 20.955                                                       # 가로 센서 크기 [mm] (USD 기본값)
    vertical = aperture * HEAD_SIZE[1] / HEAD_SIZE[0]
    camera.CreateHorizontalApertureAttr(aperture)
    camera.CreateVerticalApertureAttr(vertical)
    camera.CreateFocalLengthAttr(vertical / 2.0 / math.tan(math.radians(HEAD_FOVY / 2.0)))     # 세로 화각 60° → 13.6 mm
    camera.CreateClippingRangeAttr(Gf.Vec2f(0.01, 50.0))
    UsdGeom.Xformable(camera.GetPrim()).AddOrientOp().Set(Gf.Quatf(0.5, Gf.Vec3f(0.5, -0.5, -0.5)))
    return camera.GetPrim()


def add_head_publishers():
    """/head_camera/image_raw (색) · /head_camera/depth (깊이) · /head_camera/camera_info (렌즈 값) 를 낸다.

    세 토픽은 같은 화면에서 나와 시각(header.stamp)이 같다. 받는 쪽은 시각이 같은 색과 깊이를 짝지어 쓴다.
    """
    keys = og.Controller.Keys
    helpers = {"ColorHelper": ("isaacsim.ros2.bridge.ROS2CameraHelper", "head_camera/image_raw"),
               "DepthHelper": ("isaacsim.ros2.bridge.ROS2CameraHelper", "head_camera/depth"),
               "InfoHelper": ("isaacsim.ros2.bridge.ROS2CameraInfoHelper", "head_camera/camera_info")}
    og.Controller.edit({"graph_path": HEAD_GRAPH, "evaluator_name": "execution"}, {
        keys.CREATE_NODES: [("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                            ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                            ("RenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct")]
        + [(node, kind) for node, (kind, _) in helpers.items()],
        keys.SET_VALUES: [("RenderProduct.inputs:cameraPrim", HEAD_CAMERA),
                          ("RenderProduct.inputs:width", HEAD_SIZE[0]), ("RenderProduct.inputs:height", HEAD_SIZE[1]),
                          ("ColorHelper.inputs:type", "rgb"), ("DepthHelper.inputs:type", "depth")]      # depth: 카메라 앞쪽으로 잰 거리 [m], 32비트 실수
        + [(f"{node}.inputs:topicName", topic) for node, (_, topic) in helpers.items()]
        + [(f"{node}.inputs:frameId", "head_camera_optical_frame") for node in helpers]
        + [(f"{node}.inputs:frameSkipCount", HEAD_EVERY - 1) for node in helpers],
        keys.CONNECT: [("OnPlaybackTick.outputs:tick", "RenderProduct.inputs:execIn")]
        + [("RenderProduct.outputs:execOut", f"{node}.inputs:execIn") for node in helpers]
        + [("RenderProduct.outputs:renderProductPath", f"{node}.inputs:renderProductPath") for node in helpers]
        + [("Context.outputs:context", f"{node}.inputs:context") for node in helpers]})


def add_truth_publisher():
    """/cups/truth: 컵 세 개의 참값 (부스 좌표). /tf 와 같은 형식(TFMessage)이고 이름은 cup_pink · cup_blue · cup_yellow 다. 채점에만 쓴다."""
    keys = og.Controller.Keys
    og.Controller.edit({"graph_path": "/Graphs/ros2_cup_truth", "evaluator_name": "execution"}, {
        keys.CREATE_NODES: [("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                            ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                            ("SimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                            ("Publish", "isaacsim.ros2.bridge.ROS2PublishTransformTree")],
        keys.SET_VALUES: [("Publish.inputs:topicName", "cups/truth"),
                          ("Publish.inputs:targetPrims", [usdrt.Sdf.Path(f"/World/Cups/{name}") for name in CUP_NAMES])],
        keys.CONNECT: [("OnPlaybackTick.outputs:tick", "Publish.inputs:execIn"),
                       ("Context.outputs:context", "Publish.inputs:context"),
                       ("SimTime.outputs:simulationTime", "Publish.inputs:timeStamp")]})


def shuffled_cups(seed, gap=0.09, margin=0.05):
    """음료대 위에서 컵 세 개의 자리를 무작위로 고른다. 서로 gap, 가장자리에서 margin 넘게 떨어지게."""
    rng, spots = random.Random(seed), []
    cx, cy, sx, sy = STAND
    while len(spots) < len(CUP_NAMES):
        spot = (rng.uniform(cx - sx / 2 + margin, cx + sx / 2 - margin), rng.uniform(cy - sy / 2 + margin, cy + sy / 2 - margin))
        if all(math.hypot(spot[0] - x, spot[1] - y) >= gap for x, y in spots):
            spots.append(spot)
    return dict(zip(CUP_NAMES, spots))


def move_cups(stage, spots):
    """컵을 옮긴다. spots = {이름: (x, y)}, 높이는 음료대 상판 위."""
    for name, (x, y) in spots.items():
        set_pose(stage.GetPrimAtPath(f"/World/Cups/{name}"), translate=(x, y, STAND_TOP + make_booth.CUP_SIZE[1] / 2.0))


def build_vision_scene(app, stage, start):
    """강의 46 의 장면 + 머리 카메라. 뒤 강의에서도 이 함수를 불러 같은 장면을 만든다."""
    build_dock_scene(app, stage, start)
    prepare_grasp(stage)
    add_head_camera(stage)
    add_head_publishers()
    add_truth_publisher()


if __name__ == "__main__":
    enable_ros2(app)
    stage = new_scene()
    if args.docked:
        x, y, yaw = docked_pose(args.start)
        start = (x - args.offset[0] * math.sin(yaw), y + args.offset[0] * math.cos(yaw), yaw + math.radians(args.offset[1]))
    else:
        start = start_pose(args.start, *args.offset)
    build_vision_scene(app, stage, start)
    spots = shuffled_cups(args.shuffle) if args.shuffle is not None else {}
    for item in args.cups:
        name, value = item.split("=")
        spots[name] = tuple(float(v) for v in value.split(","))
    move_cups(stage, spots)
    if args.gui:
        set_view(*VIEWS["drinks"])
    sim = Sim(app)
    sim.play()
    sim.run(1.0)
    compensation = grasp.PayloadCompensation(stage, joints=ARM_JOINTS)
    interpolator = JointCommandInterpolator(sim)
    pose = BasePose(sim, start)
    print(f"준비됨: 시작 자리 {args.start}{' (도킹한 자리)' if args.docked else ''} ({start[0]:.3f}, {start[1]:.3f}, {math.degrees(start[2]):.1f}°), "
          f"머리 카메라 {HEAD_SIZE[0]} × {HEAD_SIZE[1]}. 끝내려면 Ctrl+C", flush=True)
    print("컵: " + ",  ".join(f"{name} ({x:.3f}, {y:.3f})" for name, (x, y, _, _) in cup_states(stage).items()), flush=True)
    stop, clock = [], sim.time
    signal.signal(signal.SIGINT, lambda *_: stop.append(True))
    while app.is_running() and not stop and (not args.seconds or sim.time - clock < args.seconds):
        app.update()
    pose.stop()
    interpolator.stop()
    compensation.stop()
    sim.stop()
    app.close()
