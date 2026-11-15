"""강의 6: SimpleRobot 에 RGB 카메라와 2D 라이다를 달고, 장애물까지의 거리를 읽습니다.

    ~/isaacsim/python.sh 06_sensors/add_sensors.py                         # 라이다 거리 출력 + 카메라 사진 저장
    ~/isaacsim/python.sh 06_sensors/add_sensors.py --no-obstacle-physics   # 장애물에 충돌체 없음: 라이다가 통과
    ~/isaacsim/python.sh 06_sensors/add_sensors.py --gui                   # 창으로 보기
    ~/isaacsim/python.sh 06_sensors/add_sensors.py --keyboard              # 창을 띄우고 W A S D 로 조종
    ~/isaacsim/python.sh 06_sensors/add_sensors.py --save work/simple_robot.usd

GUI 의 동작과 코드
    Create > Xform (Front_Sensor) + Rigid Body          → UsdGeom.Xform + UsdPhysics.RigidBodyAPI
    Create > Physics > Joint > Fixed Joint              → UsdPhysics.FixedJoint
    Create > Camera                                     → UsdGeom.Camera
    Create > Sensors > PhysX Lidar > Rotating           → RangeSensorCreateLidar 명령
"""
import argparse
import math
import os
import sys

from isaacsim import SimulationApp

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--no-obstacle-physics", action="store_true", help="장애물에 충돌체를 주지 않는다")
    parser.add_argument("--image", default="work/robot_camera.png", help="카메라 사진을 저장할 경로")
    parser.add_argument("--keyboard", action="store_true", help="창을 띄우고 W A S D 로 조종한다")
    parser.add_argument("--gui", action="store_true", help="창을 띄운다 (끝나도 닫을 때까지 유지)")
    parser.add_argument("--save", metavar="USD", help="Play 전 장면을 이 경로에 저장한다")
    args = parser.parse_args()
    app = SimulationApp({"headless": not (args.gui or args.keyboard)})

import numpy as np
import omni.kit.commands
import omni.usd
from isaacsim.core.utils.extensions import enable_extension
from pxr import Gf, UsdGeom, UsdPhysics

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
sys.path.insert(0, os.path.join(HERE, "..", "04_build_robot"))
sys.path.insert(0, os.path.join(HERE, "..", "05_control"))
from build_robot import BODY, BODY_SCALE, ROBOT, build_robot
from common.sim import Sim, new_scene, pose_of, save_stage, set_pose, set_view
from drive_robot import add_articulation_root, add_differential_controller, add_keyboard_control

FRONT_SENSOR = ROBOT + "/Front_Sensor"
MOUNT = FRONT_SENSOR + "/Sensor_Mount"
CAMERA = MOUNT + "/Robot_Camera"
LIDAR = FRONT_SENSOR + "/Lidar"
SENSOR_POSITION = (0.9, 0.0, 0.3)        # 몸체 중심에서 본 센서 자리: 몸체 위 앞쪽 끝
LIDAR_MAX_RANGE = 100.0
OBSTACLES = [                            # (모양, 위치) — 로봇 앞(+X)에 놓는다
    ("Cube", (4.0, 0.5, 0.5)),
    ("Cone", (5.0, -1.5, 0.5)),
    ("Cylinder", (3.5, -3.0, 0.5)),
    ("Cube", (6.0, 2.0, 0.5)),
]


def add_sensor_mount(stage):
    """센서 자리: 강체 Xform 을 Fixed Joint 로 몸체에 붙이고, 위치 표시용 10 cm 큐브를 단다."""
    front = UsdGeom.Xform.Define(stage, FRONT_SENSOR)
    set_pose(front.GetPrim(), translate=SENSOR_POSITION)
    UsdPhysics.RigidBodyAPI.Apply(front.GetPrim())

    joint = UsdPhysics.FixedJoint.Define(stage, FRONT_SENSOR + "/FixedJoint")   # 움직임이 없는 관절
    joint.CreateBody0Rel().SetTargets([BODY])
    joint.CreateBody1Rel().SetTargets([FRONT_SENSOR])
    joint.CreateLocalPos0Attr(Gf.Vec3f(*[p / s for p, s in zip(SENSOR_POSITION, BODY_SCALE)]))
    joint.CreateLocalRot0Attr(Gf.Quatf(1.0))
    joint.CreateLocalPos1Attr(Gf.Vec3f(0.0))
    joint.CreateLocalRot1Attr(Gf.Quatf(1.0))

    mount = UsdGeom.Cube.Define(stage, MOUNT)
    mount.CreateSizeAttr(1.0)
    mount.CreateDisplayColorAttr([Gf.Vec3f(0.2, 0.5, 0.9)])
    set_pose(mount.GetPrim(), scale=(0.1, 0.1, 0.1))
    # 충돌체만 준다. 강체(Front_Sensor) 아래에 또 강체를 두면 5.1 은 오류를 낸다.
    UsdPhysics.CollisionAPI.Apply(mount.GetPrim())


def add_camera(stage):
    """RGB 카메라: 앞(+X)을 보고 위가 +Z 가 되게 돌린다."""
    camera = UsdGeom.Camera.Define(stage, CAMERA)
    camera.CreateFocalLengthAttr(18.15)
    camera.CreateClippingRangeAttr(Gf.Vec2f(0.01, 1000.0))
    xform = UsdGeom.Xformable(camera.GetPrim())
    xform.AddTranslateOp().Set(Gf.Vec3d(0.5, 0.0, 0.0))    # 마운트가 0.1 배라 실제 5 cm = 큐브 앞면
    # USD 카메라는 자기 −Z 쪽을 보고 +Y 가 위입니다. GUI 의 Orient (90, −90, 0) 과 같은 회전.
    xform.AddOrientOp().Set(Gf.Quatf(0.5, Gf.Vec3f(0.5, -0.5, -0.5)))
    return camera.GetPrim()


def add_lidar(stage, draw=False):
    """PhysX 회전 라이다: 수평 360° 를 0.4° 간격으로, 1초에 20바퀴."""
    enable_extension("isaacsim.sensors.physx")
    omni.kit.commands.execute(
        "RangeSensorCreateLidar", path="/Lidar", parent=FRONT_SENSOR,
        min_range=0.4, max_range=LIDAR_MAX_RANGE, horizontal_fov=360.0, horizontal_resolution=0.4,
        vertical_fov=30.0, vertical_resolution=4.0, rotation_rate=20.0, high_lod=False, yaw_offset=0.0,
        draw_points=draw, draw_lines=draw, enable_semantics=False)
    lidar = stage.GetPrimAtPath(LIDAR)
    lidar.GetAttribute("xformOp:translate").Set(Gf.Vec3d(0.0, 0.0, 0.1))   # 마운트 위. 너무 낮으면 바퀴를 맞힌다
    return lidar


def add_obstacles(stage, physics=True):
    """장애물 네 개. PhysX 라이다는 충돌 모양만 보므로 충돌체가 있어야 잡힌다."""
    for i, (shape, position) in enumerate(OBSTACLES):
        obstacle = getattr(UsdGeom, shape).Define(stage, f"/World/Obstacle_{i}")
        if shape == "Cube":
            obstacle.CreateSizeAttr(1.0)
        else:
            obstacle.CreateRadiusAttr(0.5)
            obstacle.CreateHeightAttr(1.0)
            obstacle.CreateAxisAttr(UsdGeom.Tokens.z)
        obstacle.CreateDisplayColorAttr([Gf.Vec3f(0.8, 0.3, 0.25)])
        set_pose(obstacle.GetPrim(), translate=position)
        if physics:
            UsdPhysics.CollisionAPI.Apply(obstacle.GetPrim())


def camera_direction(stage):
    """카메라가 보는 방향 (월드 좌표의 단위 벡터)."""
    m = omni.usd.get_world_transform_matrix(stage.GetPrimAtPath(CAMERA))
    forward = Gf.Vec3d(-m[2][0], -m[2][1], -m[2][2])
    return forward.GetNormalized()


def read_full_scan(app):
    """라이다 한 바퀴. 한 프레임에 120° 씩 나오므로 360° 가 찰 때까지 모은다. → (방위각[°], 거리[m])"""
    from isaacsim.sensors.physx import _range_sensor
    interface = _range_sensor.acquire_lidar_sensor_interface()
    scan = {}
    for _ in range(30):
        app.update()
        azimuth = np.degrees(interface.get_azimuth_data(LIDAR))
        depth = interface.get_linear_depth_data(LIDAR)[:, 0]
        for a, d in zip(azimuth, depth):
            scan[round(float(a), 1)] = float(d)
        if len(scan) >= 900:
            break
    angles = np.array(sorted(scan))
    return angles, np.array([scan[a] for a in angles])


def save_camera_image(app, path, size=(640, 480)):
    """Robot_Camera 가 보는 화면을 PNG 로 저장한다."""
    import omni.replicator.core as rep
    from PIL import Image
    render_product = rep.create.render_product(CAMERA, size)
    annotator = rep.AnnotatorRegistry.get_annotator("rgb")
    annotator.attach([render_product])
    for _ in range(60):                      # 렌더링이 안정될 때까지
        app.update()
    data = annotator.get_data()
    path = os.path.abspath(os.path.expanduser(path))
    os.makedirs(os.path.dirname(path), exist_ok=True)
    Image.fromarray(np.asarray(data)[:, :, :3]).save(path)
    annotator.detach()
    return path


def report_lidar(stage, angles, ranges):
    lx, ly, lz, _ = pose_of(stage.GetPrimAtPath(LIDAR))
    hits = ranges < LIDAR_MAX_RANGE - 1e-3
    print(f"라이다 위치: ({lx:.2f}, {ly:.2f}, {lz:.2f}) m, 빔 {len(angles)}개 (0.4° 간격) 중 {int(hits.sum())}개가 물체에 닿음", flush=True)
    print(f"{'장애물':>10} {'방위각[°]':>9} {'잰 거리[m]':>10}", flush=True)
    for i, (shape, (ox, oy, _)) in enumerate(OBSTACLES):
        bearing = math.degrees(math.atan2(oy - ly, ox - lx))        # 라이다에서 장애물 중심을 보는 방향 (+ 왼쪽)
        measured = ranges[int(np.argmin(np.abs(angles - bearing)))]
        text = f"{measured:10.2f}" if measured < LIDAR_MAX_RANGE - 1e-3 else f"{'안 잡힘':>10}"
        print(f"{f'{i} {shape}':>10} {bearing:9.1f} {text}", flush=True)
    return int(hits.sum())


if __name__ == "__main__":
    stage = new_scene()
    build_robot(stage)
    add_articulation_root(stage)
    add_differential_controller()
    if args.keyboard:
        add_keyboard_control()
    add_sensor_mount(stage)
    add_camera(stage)
    add_lidar(stage, draw=args.gui or args.keyboard)
    add_obstacles(stage, physics=not args.no_obstacle_physics)
    if args.save:
        save_stage(stage, args.save)

    if args.gui or args.keyboard:
        set_view(eye=(-5.0, -6.0, 5.0), target=(3.0, 0.0, 0.3))

    sim = Sim(app)
    sim.play()
    sim.run(1.0)                             # 로봇이 바닥에 자리 잡을 때까지
    if args.keyboard:
        print("뷰포트를 한 번 클릭한 뒤 W(앞) S(뒤) A(왼쪽) D(오른쪽) 로 조종합니다. 창을 닫으면 끝납니다.", flush=True)
        sim.wait_until_closed()
    else:
        dx, dy, dz = (round(v, 2) + 0.0 for v in camera_direction(stage))     # + 0.0: -0.00 을 0.00 으로
        print(f"카메라가 보는 방향: ({dx:.2f}, {dy:.2f}, {dz:.2f})  ← (1, 0, 0) 이면 로봇의 앞", flush=True)
        angles, ranges = read_full_scan(app)
        hit_count = report_lidar(stage, angles, ranges)
        print("카메라 사진 저장:", save_camera_image(app, args.image), flush=True)
        if hit_count:
            print("결과: 라이다가 장애물을 봅니다.", flush=True)
        else:
            print("결과: 라이다가 아무것도 보지 못합니다. 장애물에 충돌체가 없으면 빔이 그냥 통과합니다.", flush=True)
        if args.gui:
            sim.wait_until_closed()
    sim.stop()
    app.close()
