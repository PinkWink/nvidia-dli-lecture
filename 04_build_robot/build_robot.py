"""강의 4 에서 마우스로 만든 SimpleRobot 을 코드로 만듭니다: 몸체 · 바퀴 4개 · 회전 관절 · 뒷바퀴 구동.

    ~/isaacsim/python.sh 04_build_robot/build_robot.py                         # 뒷바퀴 50 °/s 로 4초 전진
    ~/isaacsim/python.sh 04_build_robot/build_robot.py --left 50 --right -50   # 제자리에서 돌기
    ~/isaacsim/python.sh 04_build_robot/build_robot.py --no-joints --seconds 1.5   # 관절 없음: 따로 떨어짐
    ~/isaacsim/python.sh 04_build_robot/build_robot.py --left 50 --right -50 --round-wheels   # 완전히 둥근 바퀴와 비교
    ~/isaacsim/python.sh 04_build_robot/build_robot.py --gui                   # 창으로 보기
    ~/isaacsim/python.sh 04_build_robot/build_robot.py --save work/simple_robot.usd

GUI 의 동작과 코드
    Create > Xform, 끌어서 자식으로 넣기          → UsdGeom.Xform / 경로를 /World/SimpleRobot/... 로
    Create > Mesh > Cube · Cylinder              → CreateMeshPrimWithDefaultXform 명령 (메뉴가 부르는 바로 그 명령)
    Rigid Body with Colliders Preset             → UsdPhysics.RigidBodyAPI + CollisionAPI (Mesh 는 convexHull 근사)
    Create > Physics > Joint > Revolute Joint    → UsdPhysics.RevoluteJoint (Body 0 · Body 1 · Axis · Local Rotation)
    Add > Physics > Angular Drive                → UsdPhysics.DriveAPI (Damping · Target Velocity)
"""
import argparse
import os
import sys

from isaacsim import SimulationApp

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--left", type=float, default=50.0, help="왼쪽 뒷바퀴 Target Velocity [°/s]")
    parser.add_argument("--right", type=float, default=50.0, help="오른쪽 뒷바퀴 Target Velocity [°/s]")
    parser.add_argument("--no-joints", action="store_true", help="관절을 만들지 않는다")
    parser.add_argument("--round-wheels", action="store_true", help="바퀴를 Mesh 대신 완전히 둥근 원기둥(UsdGeom.Cylinder)으로 만든다")
    parser.add_argument("--seconds", type=float, default=4.0, help="시뮬레이션할 시간 [s]")
    parser.add_argument("--gui", action="store_true", help="창을 띄운다 (끝나도 닫을 때까지 유지)")
    parser.add_argument("--save", metavar="USD", help="Play 전 장면을 이 경로에 저장한다")
    args = parser.parse_args()
    app = SimulationApp({"headless": not args.gui})

# 아래 import 는 SimulationApp 을 만든 뒤에만 할 수 있습니다.
import omni.kit.commands
from pxr import Gf, UsdGeom, UsdPhysics

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from common.sim import Sim, new_scene, pose_of, save_stage, set_pose, set_view

ROBOT = "/World/SimpleRobot"
BODY = ROBOT + "/Cube"
BODY_SCALE = (2.0, 1.0, 0.5)             # 몸체: 길이 2 m · 폭 1 m · 높이 0.5 m
WHEEL_SCALE = (0.75, 0.75, 0.25)         # 바퀴: 지름 0.75 m · 폭 0.25 m
WHEEL_RADIUS = 0.375
WHEEL_DISTANCE = 1.5                     # 좌우 바퀴 사이 거리
WHEELS = {                               # 이름: 몸체 중심에서 본 위치 (x 앞, y 왼쪽)
    "Front_Left": (0.5, 0.75, 0.0),
    "Front_Right": (0.5, -0.75, 0.0),
    "Rear_Left": (-0.5, 0.75, 0.0),
    "Rear_Right": (-0.5, -0.75, 0.0),
}
DRIVEN = ("Rear_Left", "Rear_Right")     # 구동하는 바퀴


def quat_x(degrees):
    """X 축으로 degrees 만큼 도는 회전 (관절의 Local Rotation 에 넣는 값)."""
    q = Gf.Rotation(Gf.Vec3d(1, 0, 0), degrees).GetQuat()
    return Gf.Quatf(q.GetReal(), Gf.Vec3f(*q.GetImaginary()))


def add_mesh(stage, path, shape):
    """Create > Mesh > Cube · Cylinder. 메뉴와 같은 명령이라 GUI 로 만든 것과 점 하나까지 같은 Mesh 가 생긴다.

    Cylinder 는 완전한 원이 아니라 32각 기둥이다 (지름 1 m · 높이 1 m, 축 Z).
    """
    omni.kit.commands.execute("CreateMeshPrimWithDefaultXform", prim_type=shape, prim_path=path,
                              select_new_prim=False, prepend_default_prim=False)
    return stage.GetPrimAtPath(path)


def make_rigid(prim):
    """Rigid Body with Colliders Preset."""
    UsdPhysics.RigidBodyAPI.Apply(prim)
    UsdPhysics.CollisionAPI.Apply(prim)
    if prim.IsA(UsdGeom.Mesh):      # 움직이는 Mesh 의 충돌 모양: 점들을 감싸는 볼록 껍질
        UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr("convexHull")


def build_robot(stage, joints=True, round_wheels=False):
    """SimpleRobot 을 만든다. 돌려주는 값: {바퀴 이름: 관절 Prim}.

    round_wheels=True 이면 바퀴를 32각 Mesh 대신 완전히 둥근 UsdGeom.Cylinder 로 만든다.
    직진은 같지만 제자리 회전이 더 많이 돈다 (README 의 "바퀴 모양과 회전" 참고).
    """
    # ── 로봇 전체를 묶는 Xform. 바퀴 반지름 0.375 m 보다 조금 위에 둔다 ──
    robot = UsdGeom.Xform.Define(stage, ROBOT)
    set_pose(robot.GetPrim(), translate=(0.0, 0.0, 0.4))

    # ── 몸체 ──────────────────────────────────────────────────────────
    body = add_mesh(stage, BODY, "Cube")
    UsdGeom.Gprim(body).CreateDisplayColorAttr([Gf.Vec3f(0.9, 0.7, 0.2)])
    set_pose(body, scale=BODY_SCALE)
    make_rigid(body)

    joint_prims = {}
    for name, position in WHEELS.items():
        # ── 바퀴: 원기둥을 X 로 90° 눕힌다 ───────────────────────────
        path = f"{ROBOT}/{name}_Wheel"
        if round_wheels:
            cylinder = UsdGeom.Cylinder.Define(stage, path)
            cylinder.CreateRadiusAttr(0.5)
            cylinder.CreateHeightAttr(1.0)
            cylinder.CreateAxisAttr(UsdGeom.Tokens.z)
            wheel = cylinder.GetPrim()
        else:
            wheel = add_mesh(stage, path, "Cylinder")
        UsdGeom.Gprim(wheel).CreateDisplayColorAttr([Gf.Vec3f(0.15, 0.15, 0.17)])
        set_pose(wheel, translate=position, rotate_xyz=(90.0, 0.0, 0.0), scale=WHEEL_SCALE)
        make_rigid(wheel)
        if not joints:
            continue

        # ── 회전 관절: Body 0 = 몸체, Body 1 = 바퀴 ─────────────────
        joint = UsdPhysics.RevoluteJoint.Define(stage, f"{ROBOT}/{name}_Wheel/{name}_Joint")
        joint.CreateBody0Rel().SetTargets([BODY])
        joint.CreateBody1Rel().SetTargets([wheel.GetPath()])
        joint.CreateAxisAttr("Y")                               # 바퀴 축 = 로봇의 왼쪽 방향
        # 관절 위치는 각 몸체의 좌표로 적는다. 몸체는 Scale 이 걸려 있으므로 나눠 준다.
        local = [p / s for p, s in zip(position, BODY_SCALE)]   # (0.5, 0.75, 0) → (0.25, 0.75, 0)
        joint.CreateLocalPos0Attr(Gf.Vec3f(*local))
        joint.CreateLocalRot0Attr(quat_x(0.0))                  # Local Rotation 0 = (0, 0, 0)
        joint.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
        joint.CreateLocalRot1Attr(quat_x(-90.0))                # Local Rotation 1 = (−90, 0, 0): 눕힌 만큼 되돌림
        joint_prims[name] = joint.GetPrim()

        # ── 뒷바퀴 구동: Damping 을 크게 주면 속도 제어가 된다 ────────
        if name in DRIVEN:
            drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular")
            drive.CreateTypeAttr("force")
            drive.CreateStiffnessAttr(0.0)
            drive.CreateDampingAttr(10000.0)
            drive.CreateTargetVelocityAttr(0.0)
    return joint_prims


def set_wheel_speed(joint_prims, left, right):
    """뒷바퀴 Target Velocity [°/s]."""
    UsdPhysics.DriveAPI(joint_prims["Rear_Left"], "angular").GetTargetVelocityAttr().Set(left)
    UsdPhysics.DriveAPI(joint_prims["Rear_Right"], "angular").GetTargetVelocityAttr().Set(right)


def print_header():
    print(f"{'시간[s]':>8} {'x[m]':>8} {'y[m]':>8} {'z[m]':>8} {'방향[°]':>8}", flush=True)


def print_pose(time, prim):
    x, y, z, yaw = pose_of(prim)
    print(f"{time:8.2f} {x:8.3f} {y:8.3f} {z:8.3f} {yaw:8.1f}", flush=True)


if __name__ == "__main__":
    stage = new_scene()
    joint_prims = build_robot(stage, joints=not args.no_joints, round_wheels=args.round_wheels)
    if joint_prims:
        set_wheel_speed(joint_prims, args.left, args.right)
    if args.save:
        save_stage(stage, args.save)

    if args.gui:
        set_view(eye=(4.0, -6.0, 4.0), target=(1.0, 0.0, 0.3))

    body = stage.GetPrimAtPath(BODY)
    sim = Sim(app)
    sim.play()
    print_header()
    print_pose(sim.time, body)
    sim.run(args.seconds, every=0.5, report=lambda t: print_pose(t, body))

    x, y, z, yaw = pose_of(body)
    if args.no_joints:
        wheel_z = pose_of(stage.GetPrimAtPath(ROBOT + "/Front_Left_Wheel"))[2]
        print(f"결과: 관절이 없어 따로 떨어졌습니다 (몸체 z = {z:.2f} m, 바퀴 z = {wheel_z:.2f} m).", flush=True)
    else:
        print(f"결과: {args.seconds:g}초 동안 앞으로 {x:.2f} m, 방향 {yaw:+.1f}°.", flush=True)

    if args.gui:
        if joint_prims:
            set_wheel_speed(joint_prims, 0.0, 0.0)      # 화면 밖으로 나가지 않게 멈춘다
        sim.wait_until_closed()
    sim.stop()
    app.close()
