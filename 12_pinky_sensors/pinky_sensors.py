"""강의 12: Pinky Pro 에 라이다 · 카메라 · 초음파 · IMU 센서를 달고 값을 읽습니다.

    ~/isaacsim/python.sh 12_pinky_sensors/pinky_sensors.py                         # 센서 값 출력 + 카메라 사진 저장
    ~/isaacsim/python.sh 12_pinky_sensors/pinky_sensors.py --wall 0.5              # 앞 벽을 0.5 m 로 옮겨 초음파 값 확인
    ~/isaacsim/python.sh 12_pinky_sensors/pinky_sensors.py --no-obstacle-physics   # 장애물에 충돌체 없음: 라이다 · 초음파가 통과
    ~/isaacsim/python.sh 12_pinky_sensors/pinky_sensors.py --gui                   # 창으로 보기
    ~/isaacsim/python.sh 12_pinky_sensors/pinky_sensors.py --keyboard              # 창을 띄우고 W A S D 로 조종

URDF 에는 센서 자리(빈 링크)만 있습니다. 그 자리의 자식으로 Isaac Sim 센서를 만듭니다.
    rplidar_link       → Create > Sensors > PhysX Lidar > Rotating    (RPLidar C1: 360°, 0.05 ~ 12 m)
    front_camera_link  → Create > Camera                               (가로 화각 66°)
    ultrasonic_link    → PhysX Lidar 를 좁은 부채꼴로 바꿔 초음파처럼 씀 (15°, 0.02 ~ 4 m)
    imu_link           → Create > Sensors > Imu Sensor                 (가속도 · 각속도 · 자세)
"""
import argparse
import math
import os
import sys

from isaacsim import SimulationApp

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--wall", type=float, default=1.0, help="로봇 앞 벽까지의 거리 [m]")
    parser.add_argument("--no-obstacle-physics", action="store_true", help="장애물에 충돌체를 주지 않는다")
    parser.add_argument("--image", default="work/pinky_camera.png", help="카메라 사진을 저장할 경로")
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
sys.path.insert(0, os.path.join(HERE, "..", "11_pinky_import"))
from common.sim import Sim, new_scene, pose_of, save_stage, set_pose, set_view
from import_pinky import (BODY, DEFAULT_URDF, ROBOT, add_differential_controller, add_keyboard_control, find_urdf, import_pinky,
                          set_velocity)

LIDAR_LINK = ROBOT + "/rplidar_mount/rplidar_link"            # URDF 의 빈 링크 = 센서 자리
CAMERA_LINK = ROBOT + "/front_camera_mount/front_camera_link"
ULTRASONIC_LINK = BODY + "/ultrasonic_link"
IMU_LINK = BODY + "/imu_link"
IMU = IMU_LINK + "/Imu_Sensor"
LIDAR = LIDAR_LINK + "/Lidar"
CAMERA = CAMERA_LINK + "/Camera"
ULTRASONIC = ULTRASONIC_LINK + "/Ultrasonic"
LIDAR_RANGE = (0.15, 12.0)        # RPLidar C1 은 0.05 m 부터지만, 로봇 자기 몸(LED 바 · 화면)을 보지 않게 0.15 m 부터
ULTRASONIC_RANGE = (0.02, 4.0)
ULTRASONIC_FOV = 15.0             # 초음파가 퍼지는 각도 [°]
CAMERA_HFOV = 66.0                # 공식 Gazebo 설정의 horizontal_fov 1.1519 rad
OBSTACLES = [                     # (모양, 위치, 크기) — 작은 로봇에 맞춘 장애물. 라이다 높이(0.125 m)보다 높아야 잡힌다
    ("Cube", (0.5, 0.3, 0.1), (0.1, 0.1, 0.2)),
    ("Cylinder", (0.4, -0.3, 0.1), (0.1, 0.1, 0.2)),
    ("Cone", (-0.4, 0.3, 0.1), (0.15, 0.15, 0.2)),
    ("Cube", (0.0, -0.5, 0.1), (0.1, 0.1, 0.2)),
]
WALL = "/World/Wall"


def of_robot(path, robot):
    """기본 로봇(/World/pinky)의 Prim 경로를 다른 로봇의 같은 자리 경로로 바꾼다 (강의 35: 로봇 여러 대)."""
    return robot + path[len(ROBOT):]


def add_lidar(stage, draw=False, robot=ROBOT):
    """2D 라이다: rplidar_link 의 자식. 수평 360° 를 0.5° 간격으로, 1초에 10바퀴."""
    enable_extension("isaacsim.sensors.physx")
    omni.kit.commands.execute(
        "RangeSensorCreateLidar", path="/Lidar", parent=of_robot(LIDAR_LINK, robot),
        min_range=LIDAR_RANGE[0], max_range=LIDAR_RANGE[1], horizontal_fov=360.0, horizontal_resolution=0.5,
        vertical_fov=1.0, vertical_resolution=1.0, rotation_rate=10.0, high_lod=False, yaw_offset=0.0,
        draw_points=False, draw_lines=draw, enable_semantics=False)
    return stage.GetPrimAtPath(of_robot(LIDAR, robot))


def add_ultrasonic(stage, draw=False):
    """초음파 센서: 전용 센서가 없어 PhysX Lidar 를 좁은 부채꼴(15°)로 만들고, 가장 가까운 값을 거리로 쓴다."""
    enable_extension("isaacsim.sensors.physx")
    omni.kit.commands.execute(
        "RangeSensorCreateLidar", path="/Ultrasonic", parent=ULTRASONIC_LINK,
        min_range=ULTRASONIC_RANGE[0], max_range=ULTRASONIC_RANGE[1], horizontal_fov=ULTRASONIC_FOV, horizontal_resolution=1.0,
        vertical_fov=1.0, vertical_resolution=1.0, rotation_rate=0.0, high_lod=False, yaw_offset=0.0,
        draw_points=False, draw_lines=draw, enable_semantics=False)
    return stage.GetPrimAtPath(ULTRASONIC)


def add_imu(stage, robot=ROBOT):
    """IMU: imu_link 의 자식. 이 자리가 겪는 가속도 · 각속도 · 자세를 잰다."""
    enable_extension("isaacsim.sensors.physics")
    omni.kit.commands.execute("IsaacSensorCreateImuSensor", path="/Imu_Sensor", parent=of_robot(IMU_LINK, robot))
    return stage.GetPrimAtPath(of_robot(IMU, robot))


def read_imu():
    """IMU 값 한 번: (가속도 [m/s²], 각속도 [rad/s]). 센서 자기 좌표계 기준."""
    from isaacsim.sensors.physics import _sensor
    reading = _sensor.acquire_imu_sensor_interface().get_sensor_reading(IMU)
    acc = (reading.lin_acc_x, reading.lin_acc_y, reading.lin_acc_z)
    gyro = (reading.ang_vel_x, reading.ang_vel_y, reading.ang_vel_z)
    return [round(float(v), 2) + 0.0 for v in acc], [round(float(v), 2) + 0.0 for v in gyro]


def add_camera(stage, robot=ROBOT):
    """카메라: front_camera_link 의 자식. 링크의 +X 가 카메라가 보는 방향이 되게 돌린다."""
    camera = UsdGeom.Camera.Define(stage, of_robot(CAMERA, robot))
    aperture = 20.955                                                   # 가로 센서 크기 [mm] (USD 기본값)
    camera.CreateHorizontalApertureAttr(aperture)
    camera.CreateVerticalApertureAttr(aperture * 9.0 / 16.0)             # 16 : 9 (1280 × 720)
    camera.CreateFocalLengthAttr(aperture / 2.0 / math.tan(math.radians(CAMERA_HFOV / 2.0)))   # 화각 66° → 16.1 mm
    camera.CreateClippingRangeAttr(Gf.Vec2f(0.01, 100.0))
    # USD 카메라는 자기 −Z 쪽을 보고 +Y 가 위입니다. GUI 의 Orient (90, −90, 0) 과 같은 회전.
    UsdGeom.Xformable(camera.GetPrim()).AddOrientOp().Set(Gf.Quatf(0.5, Gf.Vec3f(0.5, -0.5, -0.5)))
    return camera.GetPrim()


def add_obstacles(stage, wall_distance=1.0, physics=True):
    """작은 장애물 네 개와 로봇 앞의 벽. PhysX 센서는 충돌체가 있는 물체만 봅니다."""
    prims = []
    for i, (shape, position, size) in enumerate(OBSTACLES):
        obstacle = getattr(UsdGeom, shape).Define(stage, f"/World/Obstacle_{i}")
        if shape == "Cube":
            obstacle.CreateSizeAttr(1.0)
        else:
            obstacle.CreateRadiusAttr(0.5)
            obstacle.CreateHeightAttr(1.0)
            obstacle.CreateAxisAttr(UsdGeom.Tokens.z)
        set_pose(obstacle.GetPrim(), translate=position, scale=size)
        obstacle.CreateDisplayColorAttr([Gf.Vec3f(0.8, 0.3, 0.25)])
        prims.append(obstacle.GetPrim())
    wall = UsdGeom.Cube.Define(stage, WALL)
    wall.CreateSizeAttr(1.0)
    wall.CreateDisplayColorAttr([Gf.Vec3f(0.3, 0.45, 0.8)])
    set_pose(wall.GetPrim(), translate=(wall_distance + 0.01, 0.0, 0.1), scale=(0.02, 0.3, 0.2))   # 앞면이 wall_distance 에 오게
    prims.append(wall.GetPrim())
    if physics:
        for prim in prims:
            UsdPhysics.CollisionAPI.Apply(prim)


def read_scan(app, path, full_circle):
    """PhysX Lidar 의 값. 회전형은 한 프레임에 일부만 나오므로 한 바퀴가 찰 때까지 모은다. → (각도[°], 거리[m])"""
    from isaacsim.sensors.physx import _range_sensor
    interface = _range_sensor.acquire_lidar_sensor_interface()
    scan = {}
    for _ in range(60):
        app.update()
        azimuth = np.degrees(interface.get_azimuth_data(path))
        depth = interface.get_linear_depth_data(path)
        if depth.size == 0:
            continue
        for a, d in zip(azimuth, depth[:, 0]):
            scan[round(float(a), 1)] = float(d)
        if not full_circle or len(scan) >= 720:
            break
    angles = np.array(sorted(scan))
    return angles, np.array([scan[a] for a in angles])


def save_camera_image(app, path, size=(1280, 720)):
    import omni.replicator.core as rep
    from PIL import Image
    render_product = rep.create.render_product(CAMERA, size)
    annotator = rep.AnnotatorRegistry.get_annotator("rgb")
    annotator.attach([render_product])
    for _ in range(60):
        app.update()
    data = annotator.get_data()
    path = os.path.abspath(os.path.expanduser(path))
    os.makedirs(os.path.dirname(path), exist_ok=True)
    Image.fromarray(np.asarray(data)[:, :, :3]).save(path)
    annotator.detach()
    return path


def report_lidar(stage, angles, ranges):
    """라이다가 잰 거리를 장애물마다 출력한다. 방위각은 로봇 앞(+X)이 0°, 왼쪽이 + 입니다."""
    lidar = stage.GetPrimAtPath(LIDAR)
    to_lidar = omni.usd.get_world_transform_matrix(lidar).GetInverse()
    lx, ly, lz, _ = pose_of(lidar)
    hits = ranges < LIDAR_RANGE[1] - 1e-3
    print(f"라이다 위치: ({round(lx, 3) + 0.0:.3f}, {round(ly, 3) + 0.0:.3f}, {lz:.3f}) m, "
          f"빔 {len(angles)}개 (0.5° 간격) 중 {int(hits.sum())}개가 물체에 닿음", flush=True)
    print(f"{'물체':>12} {'방위각[°]':>9} {'중심까지[m]':>10} {'잰 거리[m]':>10}", flush=True)
    names = [f"{i} {shape}" for i, (shape, _, _) in enumerate(OBSTACLES)] + ["Wall"]
    paths = [f"/World/Obstacle_{i}" for i in range(len(OBSTACLES))] + [WALL]
    for name, path in zip(names, paths):
        ox, oy, _, _ = pose_of(stage.GetPrimAtPath(path))
        bearing = math.degrees(math.atan2(oy - ly, ox - lx))
        local = to_lidar.Transform(Gf.Vec3d(ox, oy, lz))            # 라이다 자기 좌표계에서 본 방향 (rplidar_link 는 180° 돌아 있음)
        azimuth = math.degrees(math.atan2(local[1], local[0]))
        diff = np.abs((angles - azimuth + 180.0) % 360.0 - 180.0)
        measured = ranges[int(np.argmin(diff))]
        text = f"{measured:10.2f}" if measured < LIDAR_RANGE[1] - 1e-3 else f"{'안 잡힘':>10}"
        print(f"{name:>12} {bearing:9.1f} {math.hypot(ox - lx, oy - ly):10.2f} {text}", flush=True)
    return int(hits.sum())


if __name__ == "__main__":
    stage = new_scene()
    import_pinky(stage, find_urdf(DEFAULT_URDF))
    add_differential_controller()
    if args.keyboard:
        add_keyboard_control()
    show = args.gui or args.keyboard
    add_lidar(stage, draw=show)
    add_ultrasonic(stage, draw=show)
    add_camera(stage)
    add_imu(stage)
    add_obstacles(stage, wall_distance=args.wall, physics=not args.no_obstacle_physics)
    if args.save:
        save_stage(stage, args.save)
    if show:
        set_view(eye=(-0.7, -0.9, 0.8), target=(0.3, 0.0, 0.05))

    sim = Sim(app)
    sim.play()
    sim.run(1.0)
    if args.keyboard:
        print("뷰포트를 한 번 클릭한 뒤 W(앞) S(뒤) A(왼쪽) D(오른쪽) 로 조종합니다. 창을 닫으면 끝납니다.", flush=True)
        sim.wait_until_closed()
    else:
        angles, ranges = read_scan(app, LIDAR, full_circle=True)
        hit_count = report_lidar(stage, angles, ranges)
        _, sonar = read_scan(app, ULTRASONIC, full_circle=False)
        nearest = float(sonar.min()) if sonar.size else ULTRASONIC_RANGE[1]
        ux = pose_of(stage.GetPrimAtPath(ULTRASONIC))[0]
        if nearest < ULTRASONIC_RANGE[1] - 1e-3:
            print(f"초음파: 빔 {sonar.size}개 중 가장 가까운 값 {nearest:.3f} m (센서에서 벽까지 실제 {args.wall - ux:.3f} m)", flush=True)
        else:
            print(f"초음파: 빔 {sonar.size}개 모두 안 잡힘 (최대 {ULTRASONIC_RANGE[1]:g} m)", flush=True)
        print("카메라 사진 저장:", save_camera_image(app, args.image), flush=True)
        acc, gyro = read_imu()
        print("IMU (정지)       : 가속도 ({:.2f}, {:.2f}, {:.2f}) m/s², 각속도 ({:.2f}, {:.2f}, {:.2f}) rad/s".format(*acc, *gyro),
              flush=True)
        set_velocity(0.0, 1.0)                   # 제자리에서 왼쪽으로 1 rad/s
        sim.run(1.0)
        acc, gyro = read_imu()
        print("IMU (1 rad/s 회전): 가속도 ({:.2f}, {:.2f}, {:.2f}) m/s², 각속도 ({:.2f}, {:.2f}, {:.2f}) rad/s".format(*acc, *gyro),
              flush=True)
        set_velocity(0.0, 0.0)
        if hit_count:
            print("결과: 라이다와 초음파가 장애물을 봅니다.", flush=True)
        else:
            print("결과: 라이다와 초음파가 아무것도 보지 못합니다. 충돌체가 없으면 빔이 그냥 통과합니다.", flush=True)
        if args.gui:
            sim.wait_until_closed()
    sim.stop()
    app.close()
