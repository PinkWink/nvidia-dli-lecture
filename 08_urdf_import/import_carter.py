"""강의 8: carter.urdf 를 읽고, URDF Importer 로 Isaac Sim 장면에 가져옵니다.

    ~/isaacsim/python.sh 08_urdf_import/import_carter.py                 # URDF 읽기 → 가져오기 → 바닥에 내려놓기
    ~/isaacsim/python.sh 08_urdf_import/import_carter.py --read-only     # URDF 만 읽고 끝 (Isaac Sim 을 켜지 않음)
    ~/isaacsim/python.sh 08_urdf_import/import_carter.py --fixed-base    # Static Base: 몸체를 공중에 고정
    ~/isaacsim/python.sh 08_urdf_import/import_carter.py --gui           # 창으로 보기
    ~/isaacsim/python.sh 08_urdf_import/import_carter.py --save work/carter_scene.usd

GUI 의 동작과 코드
    File > Import > carter.urdf                         → URDFParseFile 명령 (URDF 를 읽어 로봇 정보로)
    Links: Moveable Base                                → import_config.fix_base = False
    관절 표의 Target (Velocity / None)                  → robot.joints[이름].drive.set_target_type(...)
    Joint Configuration: Natural Frequency · Force      → drive.set_drive_type(...) · compute_natural_stiffness(...)
    Import 버튼                                         → URDFImportRobot 명령
"""
import argparse
import os
import sys
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_URDF = os.path.join(HERE, "..", "work", "carter", "urdf", "carter.urdf")
ROBOT = "/World/carter"
BODY = ROBOT + "/chassis_link"
JOINTS = ROBOT + "/joints"
WHEEL_JOINTS = ("left_wheel", "right_wheel")      # 모터로 돌리는 바퀴: Target = Velocity
START_HEIGHT = 0.3                                # 가져온 로봇을 바닥에 묻히지 않게 올려 두는 높이 [m]


def find_urdf(path):
    path = os.path.abspath(os.path.expanduser(path))
    if not os.path.exists(path):
        print(f"URDF 파일이 없습니다: {path}")
        print("먼저 Isaac Sim 에 들어 있는 Carter 를 복사하세요:")
        print("  mkdir -p ~/nvidia-dli-lecture/work")
        print("  cp -r ~/isaacsim/exts/isaacsim.asset.importer.urdf/data/urdf/robots/carter ~/nvidia-dli-lecture/work/")
        sys.exit(1)
    return path


def read_urdf(path):
    """URDF 는 XML 입니다. <link> 와 <joint> 를 읽어 표로 출력하고 (링크 이름들, 관절 정보들) 을 돌려준다."""
    robot = ET.parse(path).getroot()
    links = robot.findall("link")
    joints = robot.findall("joint")
    print(f"URDF: {path}", flush=True)
    print(f"로봇 이름: {robot.get('name')}, 링크 {len(links)}개, 관절 {len(joints)}개", flush=True)

    print(f"{'링크':<18} {'질량[kg]':>8}  충돌 모양", flush=True)
    for link in links:
        mass = link.find("inertial/mass")
        shape = link.find("collision/geometry/*")
        if shape is None:
            text = "없음"
        elif shape.tag == "cylinder":
            text = f"원기둥 (반지름 {shape.get('radius')} m, 폭 {shape.get('length')} m)"
        elif shape.tag == "box":
            text = f"상자 ({shape.get('size')} m)"
        else:
            text = shape.tag
        print(f"{link.get('name'):<18} {(mass.get('value') if mass is not None else '-'):>8}  {text}", flush=True)

    print(f"{'관절':<12} {'종류':<11} {'부모 → 자식':<36} {'축':<7} 위치 (x y z)", flush=True)
    info = []
    for joint in joints:
        axis = joint.find("axis")
        origin = joint.find("origin")
        row = {
            "name": joint.get("name"),
            "type": joint.get("type"),
            "parent": joint.find("parent").get("link"),
            "child": joint.find("child").get("link"),
            "axis": axis.get("xyz") if axis is not None else "-",
            "xyz": origin.get("xyz") if origin is not None else "0 0 0",
        }
        info.append(row)
        print(f"{row['name']:<12} {row['type']:<11} {row['parent'] + ' → ' + row['child']:<36} {row['axis']:<7} {row['xyz']}",
              flush=True)
    return [link.get("name") for link in links], info


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--urdf", default=DEFAULT_URDF, help="가져올 URDF 파일")
    parser.add_argument("--read-only", action="store_true", help="URDF 만 읽고 끝낸다 (Isaac Sim 을 켜지 않음)")
    parser.add_argument("--fixed-base", action="store_true", help="Static Base 로 가져온다 (몸체를 고정)")
    parser.add_argument("--seconds", type=float, default=2.0, help="시뮬레이션할 시간 [s]")
    parser.add_argument("--gui", action="store_true", help="창을 띄운다 (끝나도 닫을 때까지 유지)")
    parser.add_argument("--save", metavar="USD", help="Play 전 장면을 이 경로에 저장한다")
    args = parser.parse_args()
    urdf_path = find_urdf(args.urdf)
    read_urdf(urdf_path)
    if args.read_only:
        sys.exit(0)

    from isaacsim import SimulationApp
    app = SimulationApp({"headless": not args.gui})

import omni.kit.commands
import omni.usd
from isaacsim.core.utils.extensions import enable_extension
from pxr import UsdPhysics

sys.path.insert(0, os.path.join(HERE, ".."))
from common.sim import Sim, new_scene, pose_of, save_stage, set_pose, set_view


def import_carter(stage, urdf_path, fixed_base=False, height=START_HEIGHT):
    """URDF Importer 창에서 하는 일을 명령으로 한다. 가져온 로봇의 Prim 경로를 돌려준다."""
    enable_extension("isaacsim.asset.importer.urdf")
    from isaacsim.asset.importer.urdf import _urdf

    _, config = omni.kit.commands.execute("URDFCreateImportConfig")
    config.fix_base = fixed_base                  # Links: Moveable Base (False) / Static Base (True)
    config.merge_fixed_joints = False             # 고정 관절로 붙은 링크(com_offset · imu)도 따로 남긴다
    config.make_default_prim = False              # 장면의 기본 Prim(/World)을 바꾸지 않는다
    config.create_physics_scene = False           # 물리 장면은 new_scene() 이 이미 만들었다

    # URDF 를 읽어 로봇 정보로 만든 다음, 관절마다 구동 방식을 정한다 (Importer 창의 관절 표)
    _, robot = omni.kit.commands.execute("URDFParseFile", urdf_path=urdf_path, import_config=config)
    interface = _urdf.acquire_urdf_interface()
    for name in robot.joints:
        drive = robot.joints[name].drive
        if name in WHEEL_JOINTS:
            drive.set_target_type(_urdf.UrdfJointTargetType.JOINT_DRIVE_VELOCITY)    # Target: Velocity (속도로 구동)
            # 창의 기본값과 같게: Drive Type = Force, 세기는 Natural Frequency(25) 로 계산한다.
            # Force 방식은 (진동수² = 625) × (관절이 돌리는 관성 0.493) = 약 308 이 된다.
            drive.set_drive_type(_urdf.UrdfJointDriveType.JOINT_DRIVE_FORCE)
            drive.strength = interface.compute_natural_stiffness(robot, name, drive.natural_frequency)
        else:
            drive.set_target_type(_urdf.UrdfJointTargetType.JOINT_DRIVE_NONE)        # Target: None (모터 없음, 캐스터)

    _, path = omni.kit.commands.execute("URDFImportRobot", urdf_path=urdf_path, urdf_robot=robot, import_config=config)
    prim = stage.GetPrimAtPath(path)
    if str(prim.GetPath()) != ROBOT:              # /carter 로 생기면 /World 아래로 옮긴다
        omni.kit.commands.execute("MovePrim", path_from=path, path_to=ROBOT)
        prim = stage.GetPrimAtPath(ROBOT)
    set_pose(prim, translate=(0.0, 0.0, height))
    return prim


def report_import(stage):
    """가져온 결과: 링크(Xform) · 관절(joints 폴더) · Articulation Root 위치."""
    robot = stage.GetPrimAtPath(ROBOT)
    links = [p.GetName() for p in robot.GetChildren() if p.HasAPI(UsdPhysics.RigidBodyAPI)]
    print(f"가져온 로봇: {ROBOT}, 링크(강체) {len(links)}개: {', '.join(links)}", flush=True)
    roots = [str(p.GetPath()) for p in stage.Traverse() if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
    print(f"Articulation Root: {', '.join(roots) if roots else '없음'}", flush=True)
    print(f"{'관절':<12} {'USD 종류':<22} {'축':<3} {'Body 0 → Body 1':<36} {'Stiffness':>9} {'Damping':>9}", flush=True)
    for joint in stage.GetPrimAtPath(JOINTS).GetChildren():
        body0 = (UsdPhysics.Joint(joint).GetBody0Rel().GetTargets() or [None])[0]
        body1 = (UsdPhysics.Joint(joint).GetBody1Rel().GetTargets() or [None])[0]
        bodies = f"{body0.name if body0 else '-'} → {body1.name if body1 else '-'}"
        axis = joint.GetAttribute("physics:axis").Get() if joint.HasAttribute("physics:axis") else "-"
        drive = UsdPhysics.DriveAPI(joint, "angular")
        if joint.HasAPI(UsdPhysics.DriveAPI, "angular"):
            text = f"{drive.GetStiffnessAttr().Get():9.1f} {drive.GetDampingAttr().Get():9.1f}"
        else:
            text = f"{'-':>9} {'-':>9}"
        print(f"{joint.GetName():<12} {joint.GetTypeName():<22} {axis:<3} {bodies:<36} {text}", flush=True)


if __name__ == "__main__":
    stage = new_scene()
    import_carter(stage, urdf_path, fixed_base=args.fixed_base)
    report_import(stage)
    if args.save:
        save_stage(stage, args.save)
    if args.gui:
        set_view(eye=(2.5, -2.5, 1.8), target=(0.0, 0.0, 0.3))

    body = stage.GetPrimAtPath(BODY)
    sim = Sim(app)
    sim.play()
    print(f"{'시간[s]':>8} {'몸체 높이 z[m]':>14}", flush=True)
    print(f"{sim.time:8.1f} {pose_of(body)[2]:14.3f}", flush=True)
    sim.run(args.seconds, every=0.5, report=lambda t: print(f"{t:8.1f} {pose_of(body)[2]:14.3f}", flush=True))
    z = round(pose_of(body)[2], 2) + 0.0          # + 0.0: -0.00 을 0.00 으로
    if args.fixed_base:
        print(f"결과: 몸체가 고정되어 높이 {z:.2f} m 에 그대로 있습니다 (Static Base).", flush=True)
    else:
        print(f"결과: 바퀴로 바닥에 섰습니다. 몸체 높이 {z:.2f} m = 바퀴 반지름.", flush=True)
    if args.gui:
        sim.wait_until_closed()
    sim.stop()
    app.close()
