"""강의 45: 부스에 도킹용 ArUco 마커를 붙이고, PL-2M 의 앞 카메라 영상을 ROS 2 로 냅니다.

    source /opt/ros/jazzy/setup.bash                                        # Isaac Sim 보다 먼저
    ~/isaacsim/python.sh 45_pl2m_dock/pl2m_dock.py                          # 처음 자리에서 시작 (Nav2 로 가서 도킹할 때)
    ~/isaacsim/python.sh 45_pl2m_dock/pl2m_dock.py --start drinks           # 음료대 앞(Nav2 의 목적지)에서 시작: 도킹만 볼 때
    ~/isaacsim/python.sh 45_pl2m_dock/pl2m_dock.py --start drinks --offset 0.10 -6
                                                                            # 그 자리에서 왼쪽으로 10 cm, 오른쪽으로 6° 틀어진 채 시작
    ~/isaacsim/python.sh 45_pl2m_dock/pl2m_dock.py --gui                    # 창으로 보기
    ~/isaacsim/python.sh 45_pl2m_dock/pl2m_dock.py --gui --view drinks      # 음료대 가까이에서 보기 (--view booth: 음료대와 상담 테이블 1 이 함께)

강의 44 의 장면(부스 · PL-2M · 라이다 · 바퀴 오도메트리)에 더하는 것
    마커 6장     스테이션마다 2장. 테이블 앞면, 높이 0.3 m. 그림만 있고 충돌체가 없어 라이다에는 보이지 않는다
    앞 카메라    base_camera_link 자리. 640 × 480, 세로 화각 65°, 1초에 10장
    내는 토픽    /base_camera/image_raw   /base_camera/camera_info
"""
import argparse
import math
import os
import signal
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, ".."))
CAMERA_GRAPH = "/Graphs/ros2_camera"
CAMERA_LINK = "/World/pl2m/base_link/base_camera_link"          # URDF 의 카메라 자리: 베이스 앞면, +X 가 앞
CAMERA = CAMERA_LINK + "/Camera"
CAMERA_SIZE, CAMERA_FOVY, CAMERA_EVERY = (640, 480), 65.0, 6    # 크기 [픽셀], 세로 화각 [°] (원본 cameras.yaml), 화면 6번에 한 장

if __name__ == "__main__":
    sys.path.insert(0, os.path.join(REPO, "44_pl2m_nav"))
    from booth import STATIONS
    parser = argparse.ArgumentParser()
    parser.add_argument("--start", choices=list(STATIONS), default="home", help="시작하는 자리 (스테이션이면 Nav2 의 목적지 자리)")
    parser.add_argument("--offset", type=float, nargs=2, metavar=("SIDE", "YAW"), default=(0.0, 0.0),
                        help="시작 자리를 왼쪽으로 SIDE [m] 옮기고 왼쪽으로 YAW [°] 돌린다 (Nav2 의 도착 오차를 흉내 낸다)")
    parser.add_argument("--true-odom", action="store_true", help="/odom 을 참값으로 낸다 (비교용)")
    parser.add_argument("--no-markers", action="store_true", help="마커를 붙이지 않는다 (마커를 못 찾을 때의 동작을 보는 비교용)")
    parser.add_argument("--gui", action="store_true", help="창을 띄운다")
    parser.add_argument("--view", choices=["start", "drinks", "booth"], default="start",
                        help="창의 시점: start 시작 자리의 뒤, drinks 음료대 가까이, booth 음료대와 상담 테이블 1 이 함께")
    parser.add_argument("--seconds", type=float, help="이 시간만큼만 돌리고 끝낸다")
    parser.add_argument("--log", metavar="JSON", help="참값과 오도메트리를 0.1초마다 적어 이 파일에 저장한다")
    args = parser.parse_args()

    from isaacsim import SimulationApp
    app = SimulationApp({"headless": not args.gui, "extra_args": [
        "--/app/runLoops/main/rateLimitEnabled=true", "--/app/runLoops/main/rateLimitFrequency=60"]})

import omni.graph.core as og
from pxr import Gf, UsdGeom, UsdShade

sys.path.insert(0, HERE)
sys.path.insert(0, REPO)
for folder in ("38_pl2m_booth", "39_pl2m_drive", "40_pl2m_gravity", "41_pl2m_ros2", "44_pl2m_nav"):
    sys.path.insert(0, os.path.join(REPO, folder))
from booth import STATIONS
from common.sim import Sim, new_scene, set_view
from markers import MARKER_HEIGHT, MARKERS, PLATE_SIZE, marker_image
from pl2m_drive import ARM_JOINTS
from pl2m_gravity import GravityCompensation
from pl2m_nav import BasePose, build_scene
from pl2m_ros2 import JointCommandInterpolator, enable_ros2


VIEWS = {"drinks": ((5.9, -4.6, 2.2), (7.6, -2.2, 0.6)),          # 음료대와 그 앞면의 마커가 보이는 자리
         "booth": ((6.6, -5.6, 3.4), (6.9, -2.2, 0.5))}          # 음료대와 상담 테이블 1 이 함께 보이는 자리


def add_markers(stage, folder=os.path.join(REPO, "work", "booth", "markers")):
    """마커 그림을 PNG 로 만들고, 그 그림을 입힌 네모 판을 테이블 앞면에 세운다."""
    import cv2
    import make_booth
    os.makedirs(folder, exist_ok=True)
    UsdGeom.Scope.Define(stage, "/World/Markers")
    for marker, (station, (x, y), facing, _) in MARKERS.items():
        png = os.path.join(folder, f"aruco_{marker}.png")
        cv2.imwrite(png, marker_image(marker))
        material = make_booth.make_material(stage, f"aruco_{marker}", texture=png, roughness=0.9)
        plate = make_booth.add_quad(stage, f"/World/Markers/aruco_{marker}", PLATE_SIZE / 2, PLATE_SIZE / 2)
        # 판의 +Z 가 바라보는 쪽(facing), 판의 +X 가 보는 사람의 오른쪽, 판의 +Y 가 위
        make_booth.place(plate, (x, y, MARKER_HEIGHT), make_booth.rotation_from_axes([-math.sin(facing), math.cos(facing), 0.0, 0.0, 0.0, 1.0]))
        UsdShade.MaterialBindingAPI.Apply(plate).Bind(material)
    return len(MARKERS)


def add_camera(stage):
    """앞 카메라: base_camera_link 의 자식. 링크의 +X 가 카메라가 보는 방향이 되게 돌린다 (강의 12 와 같은 회전)."""
    camera = UsdGeom.Camera.Define(stage, CAMERA)
    aperture = 20.955                                                       # 가로 센서 크기 [mm] (USD 기본값)
    vertical = aperture * CAMERA_SIZE[1] / CAMERA_SIZE[0]
    camera.CreateHorizontalApertureAttr(aperture)
    camera.CreateVerticalApertureAttr(vertical)
    camera.CreateFocalLengthAttr(vertical / 2.0 / math.tan(math.radians(CAMERA_FOVY / 2.0)))     # 세로 화각 65° → 12.3 mm
    camera.CreateClippingRangeAttr(Gf.Vec2f(0.01, 50.0))
    UsdGeom.Xformable(camera.GetPrim()).AddOrientOp().Set(Gf.Quatf(0.5, Gf.Vec3f(0.5, -0.5, -0.5)))
    return camera.GetPrim()


def add_camera_publishers():
    """/base_camera/image_raw (영상) · /base_camera/camera_info (렌즈 값) 를 낸다."""
    keys = og.Controller.Keys
    helpers = ["ImageHelper", "InfoHelper"]
    og.Controller.edit({"graph_path": CAMERA_GRAPH, "evaluator_name": "execution"}, {
        keys.CREATE_NODES: [("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                            ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                            ("RenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                            ("ImageHelper", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                            ("InfoHelper", "isaacsim.ros2.bridge.ROS2CameraInfoHelper")],
        keys.SET_VALUES: [("RenderProduct.inputs:cameraPrim", CAMERA),
                          ("RenderProduct.inputs:width", CAMERA_SIZE[0]), ("RenderProduct.inputs:height", CAMERA_SIZE[1]),
                          ("ImageHelper.inputs:type", "rgb"), ("ImageHelper.inputs:topicName", "base_camera/image_raw"),
                          ("InfoHelper.inputs:topicName", "base_camera/camera_info")]
        + [(f"{node}.inputs:frameId", "base_camera_optical_frame") for node in helpers]
        + [(f"{node}.inputs:frameSkipCount", CAMERA_EVERY - 1) for node in helpers],
        keys.CONNECT: [("OnPlaybackTick.outputs:tick", "RenderProduct.inputs:execIn")]
        + [("RenderProduct.outputs:execOut", f"{node}.inputs:execIn") for node in helpers]
        + [("RenderProduct.outputs:renderProductPath", f"{node}.inputs:renderProductPath") for node in helpers]
        + [("Context.outputs:context", f"{node}.inputs:context") for node in helpers]})


def start_pose(station, side=0.0, yaw_degrees=0.0):
    """시작 자리 (x, y, 방향): 스테이션의 Nav2 목적지에서 왼쪽으로 side, 왼쪽으로 yaw 만큼 틀어진 자리."""
    x, y, yaw = STATIONS[station]["goal"]
    return x - side * math.sin(yaw), y + side * math.cos(yaw), yaw + math.radians(yaw_degrees)


def build_dock_scene(app, stage, start, markers=True):
    """강의 44 의 장면 + 마커 + 앞 카메라. 뒤 강의에서도 이 함수를 불러 같은 장면을 만든다."""
    build_scene(app, stage, start=start)
    if markers:
        add_markers(stage)
    add_camera(stage)
    add_camera_publishers()


if __name__ == "__main__":
    enable_ros2(app)
    stage = new_scene()
    start = start_pose(args.start, *args.offset)
    build_dock_scene(app, stage, start, markers=not args.no_markers)
    if args.gui and args.view == "start":
        back = (math.cos(start[2]), math.sin(start[2]))
        set_view(eye=(start[0] - 2.6 * back[0] + 1.6 * back[1], start[1] - 2.6 * back[1] - 1.6 * back[0], 2.0),
                 target=(start[0] + 0.8 * back[0], start[1] + 0.8 * back[1], 0.5))
    elif args.gui:
        set_view(*VIEWS[args.view])
    sim = Sim(app)
    sim.play()
    sim.run(1.0)
    compensation = GravityCompensation(joints=ARM_JOINTS)
    interpolator = JointCommandInterpolator(sim)
    pose = BasePose(sim, start, args.true_odom)
    print(f"준비됨: 시작 자리 {args.start} ({start[0]:.3f}, {start[1]:.3f}, {math.degrees(start[2]):.1f}°), 마커 {0 if args.no_markers else len(MARKERS)}장, "
          f"카메라 {CAMERA_SIZE[0]} × {CAMERA_SIZE[1]}. 끝내려면 Ctrl+C", flush=True)
    stop, clock = [], sim.time
    signal.signal(signal.SIGINT, lambda *_: stop.append(True))
    while app.is_running() and not stop and (not args.seconds or sim.time - clock < args.seconds):
        app.update()
    truth = pose.truth()
    print(f"끝날 때 참값 ({truth[0]:.3f}, {truth[1]:.3f}, {math.degrees(truth[2]):.1f}°)", flush=True)
    if args.log:
        import json
        with open(args.log, "w") as file:
            json.dump({"columns": ["time", "true_x", "true_y", "true_yaw", "odom_x", "odom_y", "odom_yaw"], "start": list(start), "rows": pose.log}, file)
        print("저장:", args.log, flush=True)
    pose.stop()
    interpolator.stop()
    compensation.stop()
    sim.stop()
    app.close()
