"""강의 11: 핑크랩의 Pinky Pro 를 URDF 로 가져와 바닥에 세우고, 모터(차동 제어기)로 움직입니다.

    ~/isaacsim/python.sh 11_pinky_import/import_pinky.py                           # URDF 읽기 → 가져오기 → 0.1 m/s 직진 3초
    ~/isaacsim/python.sh 11_pinky_import/import_pinky.py --read-only               # URDF 만 읽고 끝 (Isaac Sim 을 켜지 않음)
    ~/isaacsim/python.sh 11_pinky_import/import_pinky.py --wheels 360 360          # 두 바퀴에 360 °/s 직접 주기
    ~/isaacsim/python.sh 11_pinky_import/import_pinky.py --linear 0 --angular 1    # 제자리에서 왼쪽으로 돌기
    ~/isaacsim/python.sh 11_pinky_import/import_pinky.py --keyboard                # 창을 띄우고 W A S D 로 조종
    ~/isaacsim/python.sh 11_pinky_import/import_pinky.py --save work/pinky_scene.usd

Pinky Pro 의 URDF 는 공식 저장소 https://github.com/pinklab-art/pinky_pro 의 pinky_description 에서 만듭니다.
만드는 방법은 11_pinky_import/README.md 의 1절 (또는 bash 11_pinky_import/prepare_pinky.sh).

GUI 의 동작과 코드
    File > Import > pinky.urdf · Moveable Base · Target      → URDFParseFile · URDFImportRobot 명령 (강의 8 과 같음)
    Tools > Robotics > OmniGraph Controllers > Differential Controller → og.Controller.edit(...) (강의 9 와 같음)
    physicsScene 의 Solver Type = PGS                               → PhysxSceneAPI 의 solverType
"""
import argparse
import os
import sys
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_URDF = os.path.join(HERE, "..", "work", "pinky_description", "urdf", "pinky.urdf")
ROBOT = "/World/pinky"
BODY = ROBOT + "/base_link"
JOINTS = ROBOT + "/joints"
# Differential Controller 는 바퀴 속도를 [왼쪽, 오른쪽] 순서로 냅니다. 관절 이름도 같은 순서로 적습니다.
WHEEL_JOINTS = ["l_wheel_joint", "r_wheel_joint"]
WHEEL_RADIUS = 0.028          # pinky.urdf 의 바퀴 충돌 모양: <sphere radius="0.028">
WHEEL_DISTANCE = 0.0961       # 좌우 바퀴가 바닥에 닿는 점 사이 거리 (공식 Gazebo 설정의 wheel_separation)
MAX_LINEAR = 0.2              # 이 강의에서 쓰는 속도 제한 [m/s]
MAX_ANGULAR = 2.0             # [rad/s]
START_HEIGHT = 0.05           # 가져온 로봇을 바닥에 묻히지 않게 올려 두는 높이 [m]
GRAPH = "/Graphs/differential_controller"


def find_urdf(path):
    path = os.path.abspath(os.path.expanduser(path))
    if not os.path.exists(path):
        print(f"URDF 파일이 없습니다: {path}")
        print("먼저 Pinky Pro 의 URDF 를 만드세요 (ROS 2 Jazzy 필요):")
        print("  bash ~/nvidia-dli-lecture/11_pinky_import/prepare_pinky.sh")
        sys.exit(1)
    return path


def read_urdf(path):
    """<link> 와 <joint> 를 읽어 표로 출력한다. 모양이 없는 링크는 센서 자리 표시용 빈 링크입니다."""
    robot = ET.parse(path).getroot()
    links = robot.findall("link")
    joints = robot.findall("joint")
    print(f"URDF: {path}", flush=True)
    print(f"로봇 이름: {robot.get('name')}, 링크 {len(links)}개, 관절 {len(joints)}개", flush=True)

    print(f"{'링크':<20} {'질량[kg]':>8}  모양", flush=True)
    for link in links:
        mass = link.find("inertial/mass")
        visual = link.find("visual/geometry/mesh")
        shape = link.find("collision/geometry/*")
        if visual is None and shape is None:
            text = "없음 (자리 표시용 빈 링크)"
        else:
            text = os.path.basename(visual.get("filename")) if visual is not None else "-"
            if shape is not None and shape.tag == "sphere":
                text += f", 충돌 = 공 (반지름 {shape.get('radius')} m)"
        print(f"{link.get('name'):<20} {(mass.get('value') if mass is not None else '-'):>8}  {text}", flush=True)

    print(f"{'관절':<32} {'종류':<11} {'부모 → 자식':<40} 축", flush=True)
    for joint in joints:
        axis = joint.find("axis")
        pair = f"{joint.find('parent').get('link')} → {joint.find('child').get('link')}"
        print(f"{joint.get('name'):<32} {joint.get('type'):<11} {pair:<40} {axis.get('xyz') if axis is not None else '-'}",
              flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--urdf", default=DEFAULT_URDF, help="가져올 URDF 파일")
    parser.add_argument("--read-only", action="store_true", help="URDF 만 읽고 끝낸다 (Isaac Sim 을 켜지 않음)")
    parser.add_argument("--wheels", type=float, nargs=2, metavar=("LEFT", "RIGHT"),
                        help="제어기 없이 왼쪽 · 오른쪽 바퀴의 Target Velocity 를 직접 준다 [°/s]")
    parser.add_argument("--linear", type=float, default=0.1, help="직진 속도 v [m/s] (+ 앞)")
    parser.add_argument("--angular", type=float, default=0.0, help="회전 속도 ω [rad/s] (+ 왼쪽으로 돌기)")
    parser.add_argument("--seconds", type=float, default=3.0, help="시뮬레이션할 시간 [s]")
    parser.add_argument("--keyboard", action="store_true", help="창을 띄우고 W A S D 로 조종한다")
    parser.add_argument("--gui", action="store_true", help="창을 띄운다 (끝나도 닫을 때까지 유지)")
    parser.add_argument("--save", metavar="USD", help="Play 전 장면을 이 경로에 저장한다")
    args = parser.parse_args()
    urdf_path = find_urdf(args.urdf)
    read_urdf(urdf_path)
    if args.read_only:
        sys.exit(0)

    from isaacsim import SimulationApp
    app = SimulationApp({"headless": not (args.gui or args.keyboard)})

import omni.graph.core as og
import omni.kit.commands
import omni.usd
from isaacsim.core.utils.extensions import enable_extension
from pxr import PhysxSchema, UsdPhysics

sys.path.insert(0, os.path.join(HERE, ".."))
from common.sim import Sim, new_scene, pose_of, save_stage, set_pose, set_view


def use_pgs_solver(stage, scene_path="/World/PhysicsScene"):
    """물리 솔버를 PGS 로 바꾼다 (GUI: physicsScene 의 Solver Type).

    기본 솔버(TGS)에서는 이 로봇이 깨어 있는 동안 뒤 캐스터가 저절로 조금씩 돌아, 서 있어도 1초에 0.26° 씩 방향이 틀어진다.
    PGS 에서는 0 이다 (강의 PC 측정).
    """
    PhysxSchema.PhysxSceneAPI.Apply(stage.GetPrimAtPath(scene_path)).CreateSolverTypeAttr("PGS")


def import_pinky(stage, urdf_path, height=START_HEIGHT):
    """URDF Importer 창에서 하는 일을 명령으로 한다 (강의 8 의 Carter 와 같은 순서)."""
    use_pgs_solver(stage)
    enable_extension("isaacsim.asset.importer.urdf")
    from isaacsim.asset.importer.urdf import _urdf

    _, config = omni.kit.commands.execute("URDFCreateImportConfig")
    config.fix_base = False                       # Links: Moveable Base
    config.merge_fixed_joints = True              # 창의 기본 동작: 질량 없는 빈 링크는 부모 링크 아래 Xform 으로 남긴다
    config.make_default_prim = False
    config.create_physics_scene = False

    _, robot = omni.kit.commands.execute("URDFParseFile", urdf_path=urdf_path, import_config=config)
    interface = _urdf.acquire_urdf_interface()
    for name in robot.joints:
        drive = robot.joints[name].drive
        if name in WHEEL_JOINTS:
            drive.set_target_type(_urdf.UrdfJointTargetType.JOINT_DRIVE_VELOCITY)    # Target: Velocity
            drive.set_drive_type(_urdf.UrdfJointDriveType.JOINT_DRIVE_FORCE)         # Drive Type: Force (창의 기본값)
            drive.strength = interface.compute_natural_stiffness(robot, name, drive.natural_frequency)
        else:
            drive.set_target_type(_urdf.UrdfJointTargetType.JOINT_DRIVE_NONE)        # Target: None (캐스터)

    _, path = omni.kit.commands.execute("URDFImportRobot", urdf_path=urdf_path, urdf_robot=robot, import_config=config)
    if path != ROBOT:
        omni.kit.commands.execute("MovePrim", path_from=path, path_to=ROBOT)
    prim = stage.GetPrimAtPath(ROBOT)
    set_pose(prim, translate=(0.0, 0.0, height))
    return prim


def report_import(stage):
    """가져온 결과: 강체 링크 · 센서 자리(Xform) · 회전 관절."""
    robot = stage.GetPrimAtPath(ROBOT)
    bodies = [p for p in robot.GetChildren() if p.HasAPI(UsdPhysics.RigidBodyAPI)]
    print(f"가져온 로봇: {ROBOT}, 강체 링크 {len(bodies)}개", flush=True)
    print(f"{'강체 링크':<20} 그 아래 센서 자리 (Xform)", flush=True)
    for body in bodies:
        frames = [c.GetName() for c in body.GetChildren() if c.GetName() not in ("visuals", "collisions")]
        print(f"{body.GetName():<20} {', '.join(frames) if frames else '-'}", flush=True)
    print(f"{'회전 관절':<22} {'축':<3} {'Stiffness':>9} {'Damping':>9}", flush=True)
    for joint in stage.GetPrimAtPath(JOINTS).GetChildren():
        if joint.IsA(UsdPhysics.RevoluteJoint):
            drive = UsdPhysics.DriveAPI(joint, "angular")
            print(f"{joint.GetName():<22} {joint.GetAttribute('physics:axis').Get():<3} "
                  f"{drive.GetStiffnessAttr().Get():9.2f} {drive.GetDampingAttr().Get():9.3f}", flush=True)


def set_wheel_velocity(stage, left, right):
    """관절의 Drive > Angular > Target Velocity [°/s]."""
    for name, value in zip(WHEEL_JOINTS, (left, right)):
        UsdPhysics.DriveAPI(stage.GetPrimAtPath(f"{JOINTS}/{name}"), "angular").GetTargetVelocityAttr().Set(value)


def add_differential_controller(linear=0.0, angular=0.0, max_linear=MAX_LINEAR, max_angular=MAX_ANGULAR):
    """(v, ω) → 왼 · 오 바퀴 속도 → 관절. 강의 9 의 그래프와 같고 값만 Pinky Pro 에 맞췄다."""
    enable_extension("isaacsim.robot.wheeled_robots")
    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": GRAPH, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("DifferentialController", "isaacsim.robot.wheeled_robots.DifferentialController"),
                ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
            ],
            keys.SET_VALUES: [
                ("DifferentialController.inputs:wheelRadius", WHEEL_RADIUS),
                ("DifferentialController.inputs:wheelDistance", WHEEL_DISTANCE),
                ("DifferentialController.inputs:linearVelocity", linear),
                ("DifferentialController.inputs:angularVelocity", angular),
                ("DifferentialController.inputs:maxLinearSpeed", max_linear),
                ("DifferentialController.inputs:maxAngularSpeed", max_angular),
                ("ArticulationController.inputs:targetPrim", ROBOT),
                ("ArticulationController.inputs:jointNames", WHEEL_JOINTS),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "DifferentialController.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
                ("DifferentialController.outputs:velocityCommand", "ArticulationController.inputs:velocityCommand"),
            ],
        },
    )


def add_keyboard_control(key_linear=MAX_LINEAR, key_angular=MAX_ANGULAR):
    """W − S → 직진 속도, A − D → 회전 속도. 키를 누르고 있는 동안만 값이 나간다."""
    keys = og.Controller.Keys
    nodes, values, connects = [], [], []
    for key in "WASD":
        nodes += [(key, "omni.graph.nodes.ReadKeyboardState"), (f"ToDouble{key}", "omni.graph.nodes.ToDouble")]
        values += [(f"{key}.inputs:key", key)]
        connects += [(f"{key}.outputs:isPressed", f"ToDouble{key}.inputs:value")]
    for name, plus, minus, scale, target in (("Linear", "W", "S", key_linear, "linearVelocity"),
                                             ("Angular", "A", "D", key_angular, "angularVelocity")):
        nodes += [(f"Sub{name}", "omni.graph.nodes.Subtract"), (f"Scale{name}", "omni.graph.nodes.ConstantDouble"),
                  (f"Speed{name}", "omni.graph.nodes.Multiply")]
        values += [(f"Scale{name}.inputs:value", scale)]
        connects += [(f"ToDouble{plus}.outputs:converted", f"Sub{name}.inputs:a"),
                     (f"ToDouble{minus}.outputs:converted", f"Sub{name}.inputs:b"),
                     (f"Sub{name}.outputs:difference", f"Speed{name}.inputs:a"),
                     (f"Scale{name}.inputs:value", f"Speed{name}.inputs:b"),
                     (f"Speed{name}.outputs:product", f"{GRAPH}/DifferentialController.inputs:{target}")]
    og.Controller.edit(GRAPH, {keys.CREATE_NODES: nodes, keys.SET_VALUES: values, keys.CONNECT: connects})


def set_velocity(linear, angular):
    og.Controller.attribute(GRAPH + "/DifferentialController.inputs:linearVelocity").set(linear)
    og.Controller.attribute(GRAPH + "/DifferentialController.inputs:angularVelocity").set(angular)


def print_header():
    print(f"{'시간[s]':>8} {'x[m]':>8} {'y[m]':>8} {'z[m]':>8} {'방향[°]':>8}", flush=True)


def print_pose(time, prim):
    x, y, z, yaw = pose_of(prim)
    print(f"{time:8.1f} {round(x, 3) + 0.0:8.3f} {round(y, 3) + 0.0:8.3f} {round(z, 3) + 0.0:8.3f} {round(yaw, 1) + 0.0:8.1f}",
          flush=True)


if __name__ == "__main__":
    stage = new_scene()
    import_pinky(stage, urdf_path)
    report_import(stage)
    use_controller = args.wheels is None
    if args.keyboard:
        add_differential_controller()
        add_keyboard_control()
    elif use_controller:
        add_differential_controller()             # 처음에는 멈춰 있다가, 바닥에 자리 잡은 뒤 속도를 준다
    if args.save:
        save_stage(stage, args.save)
    if args.gui or args.keyboard:
        set_view(eye=(-0.35, -0.45, 0.35), target=(0.1, 0.0, 0.04))

    body = stage.GetPrimAtPath(BODY)
    sim = Sim(app)
    sim.play()
    sim.run(1.0)                              # 바닥에 자리 잡을 때까지
    if args.keyboard:
        print("뷰포트를 한 번 클릭한 뒤 W(앞) S(뒤) A(왼쪽) D(오른쪽) 로 조종합니다. 창을 닫으면 끝납니다.", flush=True)
        sim.wait_until_closed()
    else:
        if use_controller:
            set_velocity(args.linear, args.angular)
        else:
            set_wheel_velocity(stage, *args.wheels)
        start = sim.time
        x0, y0, _, yaw0 = pose_of(body)
        print_header()
        print_pose(0.0, body)
        sim.run(args.seconds, every=0.5, report=lambda t: print_pose(t - start, body))
        x, y, z, yaw = pose_of(body)
        turn = (yaw - yaw0 + 180.0) % 360.0 - 180.0          # −180° ~ +180° 로
        if use_controller:
            print(f"결과: v = {args.linear:g} m/s, ω = {args.angular:g} rad/s 로 {args.seconds:g}초 → "
                  f"이동 ({x - x0:.3f}, {y - y0:.3f}) m, 회전 {turn:+.1f}°.", flush=True)
        else:
            print(f"결과: 왼쪽 {args.wheels[0]:g} °/s, 오른쪽 {args.wheels[1]:g} °/s 로 {args.seconds:g}초 → "
                  f"이동 ({x - x0:.3f}, {y - y0:.3f}) m, 회전 {turn:+.1f}°.", flush=True)
        if args.gui:
            if use_controller:
                set_velocity(0.0, 0.0)
            else:
                set_wheel_velocity(stage, 0.0, 0.0)
            sim.wait_until_closed()
    sim.stop()
    app.close()
