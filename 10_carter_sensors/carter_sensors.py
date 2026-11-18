"""강의 10: URDF 로 가져온 Carter 에 RGB 카메라와 2D 라이다를 달고, 장애물까지의 거리를 읽습니다.

    ~/isaacsim/python.sh 10_carter_sensors/carter_sensors.py                         # 라이다 거리 출력 + 카메라 사진 저장
    ~/isaacsim/python.sh 10_carter_sensors/carter_sensors.py --no-obstacle-physics   # 장애물에 충돌체 없음: 라이다가 통과
    ~/isaacsim/python.sh 10_carter_sensors/carter_sensors.py --gui                   # 창으로 보기
    ~/isaacsim/python.sh 10_carter_sensors/carter_sensors.py --keyboard              # 창을 띄우고 W A S D 로 조종
    ~/isaacsim/python.sh 10_carter_sensors/carter_sensors.py --save work/carter_sensors.usd

GUI 의 동작과 코드
    chassis_link 우클릭 > Create > Camera                → UsdGeom.Camera (chassis_link 의 자식)
    Create > Sensors > PhysX Lidar > Rotating            → RangeSensorCreateLidar 명령
    Create > Mesh > Cube · Cylinder · Cone               → UsdGeom.Cube · Cylinder · Cone
    Add > Physics > Rigid Body with Colliders Preset     → UsdPhysics.RigidBodyAPI + CollisionAPI
"""
import argparse
import math
import os
import sys

from isaacsim import SimulationApp

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--no-obstacle-physics", action="store_true", help="장애물에 충돌체를 주지 않는다")
    parser.add_argument("--image", default="work/carter_camera.png", help="카메라 사진을 저장할 경로")
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
sys.path.insert(0, os.path.join(HERE, "..", "08_urdf_import"))
sys.path.insert(0, os.path.join(HERE, "..", "09_carter_drive"))
from common.sim import Sim, new_scene, pose_of, save_stage, set_pose, set_view
from drive_carter import add_differential_controller, add_keyboard_control
from import_carter import BODY, DEFAULT_URDF, find_urdf, import_carter

CAMERA = BODY + "/RGB_Sensor"
LIDAR = BODY + "/Lidar"
CAMERA_POSITION = (0.1, 0.0, 0.33)       # 몸체 앞의 둥근 사각형 (스테레오 카메라 자리)
LIDAR_POSITION = (-0.05, 0.0, 0.42)      # 몸체 위의 원기둥 (라이다 자리)
LIDAR_MAX_RANGE = 100.0
OBSTACLES = [                            # (모양, 위치)
    ("Cube", (2.0, 0.6, 0.5)),
    ("Cylinder", (2.5, -1.0, 0.5)),
    ("Cone", (-1.5, 1.5, 0.5)),
    ("Cube", (0.5, -2.2, 0.5)),
]


def add_camera(stage):
    """RGB 카메라를 chassis_link 의 자식으로: 몸체가 움직이면 함께 움직인다. 앞(+X)을 보고 위가 +Z."""
    camera = UsdGeom.Camera.Define(stage, CAMERA)
    camera.CreateFocalLengthAttr(18.15)
    camera.CreateClippingRangeAttr(Gf.Vec2f(0.01, 1000.0))
    xform = UsdGeom.Xformable(camera.GetPrim())
    xform.AddTranslateOp().Set(Gf.Vec3d(*CAMERA_POSITION))
    # USD 카메라는 자기 −Z 쪽을 보고 +Y 가 위입니다. GUI 의 Orient (90, −90, 0) 과 같은 회전.
    xform.AddOrientOp().Set(Gf.Quatf(0.5, Gf.Vec3f(0.5, -0.5, -0.5)))
    return camera.GetPrim()


def add_lidar(stage, draw=False):
    """PhysX 회전 라이다: 수평 360° 를 0.4° 간격으로, 1초에 20바퀴."""
    enable_extension("isaacsim.sensors.physx")
    omni.kit.commands.execute(
        "RangeSensorCreateLidar", path="/Lidar", parent=BODY,
        min_range=0.4, max_range=LIDAR_MAX_RANGE, horizontal_fov=360.0, horizontal_resolution=0.4,
        vertical_fov=30.0, vertical_resolution=4.0, rotation_rate=20.0, high_lod=False, yaw_offset=0.0,
        draw_points=draw, draw_lines=draw, enable_semantics=False)
    lidar = stage.GetPrimAtPath(LIDAR)
    lidar.GetAttribute("xformOp:translate").Set(Gf.Vec3d(*LIDAR_POSITION))
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
        if physics:                                   # Rigid Body with Colliders Preset 와 같다
            UsdPhysics.RigidBodyAPI.Apply(obstacle.GetPrim())
            UsdPhysics.CollisionAPI.Apply(obstacle.GetPrim())


def camera_direction(stage):
    """카메라가 보는 방향 (월드 좌표의 단위 벡터)."""
    m = omni.usd.get_world_transform_matrix(stage.GetPrimAtPath(CAMERA))
    return Gf.Vec3d(-m[2][0], -m[2][1], -m[2][2]).GetNormalized()


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
    """RGB_Sensor 가 보는 화면을 PNG 로 저장한다."""
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
    lx, ly, lz, _ = (round(v, 2) + 0.0 for v in pose_of(stage.GetPrimAtPath(LIDAR)))     # + 0.0: -0.00 을 0.00 으로
    hits = ranges < LIDAR_MAX_RANGE - 1e-3
    print(f"라이다 위치: ({lx:.2f}, {ly:.2f}, {lz:.2f}) m, 빔 {len(angles)}개 (0.4° 간격) 중 {int(hits.sum())}개가 물체에 닿음", flush=True)
    print(f"{'장애물':>12} {'방위각[°]':>9} {'중심까지[m]':>10} {'잰 거리[m]':>10}", flush=True)
    for i, (shape, _) in enumerate(OBSTACLES):
        ox, oy, _, _ = pose_of(stage.GetPrimAtPath(f"/World/Obstacle_{i}"))
        bearing = math.degrees(math.atan2(oy - ly, ox - lx))        # 라이다에서 장애물 중심을 보는 방향 (+ 왼쪽)
        center = math.hypot(ox - lx, oy - ly)
        measured = ranges[int(np.argmin(np.abs(angles - bearing)))]
        text = f"{measured:10.2f}" if measured < LIDAR_MAX_RANGE - 1e-3 else f"{'안 잡힘':>10}"
        print(f"{f'{i} {shape}':>12} {bearing:9.1f} {center:10.2f} {text}", flush=True)
    return int(hits.sum())


if __name__ == "__main__":
    stage = new_scene()
    import_carter(stage, find_urdf(DEFAULT_URDF))
    add_differential_controller(max_linear=0.5, max_angular=0.5)
    if args.keyboard:
        add_keyboard_control()
    add_camera(stage)
    add_lidar(stage, draw=args.gui or args.keyboard)
    add_obstacles(stage, physics=not args.no_obstacle_physics)
    if args.save:
        save_stage(stage, args.save)
    if args.gui or args.keyboard:
        set_view(eye=(-3.5, -4.5, 4.0), target=(0.8, 0.0, 0.3))

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
