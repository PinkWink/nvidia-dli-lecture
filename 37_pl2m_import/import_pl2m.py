"""강의 37: 핑크랩의 이동형 양팔 로봇 PL-2M 을 URDF 로 가져와 바닥에 세웁니다.

    ~/isaacsim/python.sh 37_pl2m_import/import_pl2m.py --read-only         # URDF 만 읽고 끝 (Isaac Sim 을 켜지 않음)
    ~/isaacsim/python.sh 37_pl2m_import/import_pl2m.py                     # 가져오기 → 5초 서 있기
    ~/isaacsim/python.sh 37_pl2m_import/import_pl2m.py --gui               # 창으로 보기
    ~/isaacsim/python.sh 37_pl2m_import/import_pl2m.py --elbow 90          # 왼팔 팔꿈치를 90° 로 굽혀 보기
    ~/isaacsim/python.sh 37_pl2m_import/import_pl2m.py --save work/pl2m/pl2m_scene.usd

PL-2M 의 URDF 는 저장소 https://github.com/PinkWink/pinklab_robotworld2026 의 설계 패키지에서 만듭니다.
만드는 방법은 37_pl2m_import/README.md 의 2절 (또는 bash 37_pl2m_import/prepare_pl2m.sh).

GUI 의 동작과 코드
    File > Import > pl2m.urdf · Moveable Base · 바퀴 Target = Velocity   → URDFParseFile · URDFImportRobot 명령 (강의 8 · 11 과 같음)
"""
import argparse
import math
import os
import sys
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_URDF = os.path.join(HERE, "..", "work", "pl2m", "pl2m.urdf")
ROBOT = "/World/pl2m"
BODY = ROBOT + "/base_link"
JOINTS = ROBOT + "/joints"
WHEEL_JOINTS = ["left_wheel_joint", "right_wheel_joint"]
ELBOW = "openarm_left_joint4"   # 왼팔 팔꿈치
START_HEIGHT = 0.05             # 가져온 로봇을 바닥에 묻히지 않게 올려 두는 높이 [m]


def find_urdf(path):
    path = os.path.abspath(os.path.expanduser(path))
    if not os.path.exists(path):
        print(f"URDF 파일이 없습니다: {path}")
        print("먼저 PL-2M 의 URDF 를 만드세요 (ROS 2 Jazzy 필요):")
        print("  bash ~/nvidia-dli-lecture/37_pl2m_import/prepare_pl2m.sh")
        sys.exit(1)
    return path


def part_of(name):
    """링크 · 관절 이름으로 어느 부분인지 가른다."""
    if "finger" in name or "hand" in name:
        return "그리퍼"
    if name.startswith("openarm"):
        return "양팔"
    if "camera" in name:
        return "카메라"
    return "베이스"


def read_urdf(path):
    """<link> 와 <joint> 를 읽어 부분별 개수와, 움직이는 관절의 표를 출력한다."""
    robot = ET.parse(path).getroot()
    links = robot.findall("link")
    joints = robot.findall("joint")
    moving = [j for j in joints if j.get("type") != "fixed"]
    mass = sum(float(link.find("inertial/mass").get("value")) for link in links if link.find("inertial/mass") is not None)
    print(f"URDF: {path}", flush=True)
    print(f"로봇 이름: {robot.get('name')}, 링크 {len(links)}개, 관절 {len(joints)}개 (움직이는 관절 {len(moving)}개), "
          f"질량 합 {mass:.1f} kg", flush=True)

    print(f"{'부분':<6} {'링크':>4} {'질량[kg]':>8} {'움직이는 관절':>8}", flush=True)
    for part in ("베이스", "양팔", "그리퍼", "카메라"):
        mine = [link for link in links if part_of(link.get("name")) == part]
        kg = sum(float(link.find("inertial/mass").get("value")) for link in mine if link.find("inertial/mass") is not None)
        count = sum(1 for joint in moving if part_of(joint.get("name")) == part)
        print(f"{part:<6} {len(mine):>4} {kg:>8.1f} {count:>8}", flush=True)

    print(f"{'움직이는 관절':<28} {'종류':<11} {'축':<8} {'범위':<16} {'힘 한계':>7}  비고", flush=True)
    for joint in moving:
        axis, limit, mimic = joint.find("axis"), joint.find("limit"), joint.find("mimic")
        if limit is None:
            span = "제한 없음"
        elif joint.get("type") == "prismatic":
            span = f"{float(limit.get('lower')) * 1000:.0f} ~ {float(limit.get('upper')) * 1000:.0f} mm"
        else:
            span = f"{math.degrees(float(limit.get('lower'))):.0f} ~ {math.degrees(float(limit.get('upper'))):.0f}°"
        note = f"{mimic.get('joint')} 을 따라 움직임" if mimic is not None else ""
        print(f"{joint.get('name'):<28} {joint.get('type'):<11} {axis.get('xyz'):<8} {span:<16} "
              f"{(limit.get('effort') if limit is not None else '-'):>7}  {note}", flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--urdf", default=DEFAULT_URDF, help="가져올 URDF 파일")
    parser.add_argument("--read-only", action="store_true", help="URDF 만 읽고 끝낸다 (Isaac Sim 을 켜지 않음)")
    parser.add_argument("--elbow", type=float, metavar="DEG", help="왼팔 팔꿈치(openarm_left_joint4)의 목표 각도 [°]")
    parser.add_argument("--seconds", type=float, default=5.0, help="시뮬레이션할 시간 [s]")
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
from pxr import Usd, UsdPhysics

sys.path.insert(0, os.path.join(HERE, ".."))
from common.sim import Sim, new_scene, pose_of, save_stage, set_pose, set_view


def import_pl2m(stage, urdf_path, height=START_HEIGHT, path=ROBOT):
    """URDF Importer 창에서 하는 일을 명령으로 한다 (강의 11 의 Pinky Pro 와 같은 순서). 물리 솔버는 기본값(TGS) 그대로."""
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
        joint = robot.joints[name]
        if joint.type == _urdf.UrdfJointType.JOINT_FIXED:
            continue
        drive = joint.drive
        drive.set_drive_type(_urdf.UrdfJointDriveType.JOINT_DRIVE_FORCE)             # Drive Type: Force (창의 기본값)
        # 창의 Natural Frequency 방식: Stiffness = 관절이 돌리는 관성 × 고유 진동수², Damping = 2 × 관성 × 고유 진동수 × 감쇠비
        drive.strength = interface.compute_natural_stiffness(robot, name, drive.natural_frequency)
        if name in WHEEL_JOINTS:
            drive.set_target_type(_urdf.UrdfJointTargetType.JOINT_DRIVE_VELOCITY)    # 바퀴: Target = Velocity
        else:
            drive.damping = 2 * joint.inertia * drive.natural_frequency * drive.damping_ratio

    _, imported = omni.kit.commands.execute("URDFImportRobot", urdf_path=urdf_path, urdf_robot=robot, import_config=config)
    if imported != path:
        omni.kit.commands.execute("MovePrim", path_from=imported, path_to=path)
    prim = stage.GetPrimAtPath(path)
    set_pose(prim, translate=(0.0, 0.0, height))
    return prim


def drive_of(joint):
    """관절 Prim 의 구동(Drive) 설정. 회전 관절은 angular, 직선 관절은 linear."""
    return UsdPhysics.DriveAPI(joint, "linear" if joint.IsA(UsdPhysics.PrismaticJoint) else "angular")


def report_import(stage):
    """가져온 결과: 강체 수 · 질량 · 관절 수와 움직이는 관절의 구동 값."""
    robot = stage.GetPrimAtPath(ROBOT)
    prims = list(Usd.PrimRange(robot))
    bodies = [p for p in prims if p.HasAPI(UsdPhysics.RigidBodyAPI)]
    mass = sum(UsdPhysics.MassAPI(b).GetMassAttr().Get() or 0.0 for b in bodies)
    frames = [c.GetName() for b in bodies for c in b.GetChildren()
              if c.GetTypeName() == "Xform" and c.GetName() not in ("visuals", "collisions")]
    roots = [p.GetName() for p in prims if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
    joints = [p for p in prims if p.IsA(UsdPhysics.Joint)]
    moving = [j for j in joints if not j.IsA(UsdPhysics.FixedJoint)]
    print(f"가져온 로봇: {ROBOT}", flush=True)
    print(f"강체 링크 {len(bodies)}개 (질량 합 {mass:.1f} kg), 강체가 되지 않은 빈 링크 {len(frames)}개", flush=True)
    print(f"관절 {len(joints)}개 = 움직이는 관절 {len(moving)}개 + 고정 관절 {len(joints) - len(moving)}개, "
          f"Articulation Root = {', '.join(roots)}", flush=True)
    print(f"{'움직이는 관절':<28} {'축':<3} {'Stiffness':>10} {'Damping':>9} {'Max Force':>10}  비고", flush=True)
    for joint in moving:
        axis = joint.GetAttribute("physics:axis").Get()
        mimic = [name for name in joint.GetAppliedSchemas() if "Mimic" in name]
        if mimic:                                  # 다른 관절을 따라 움직이는 관절: 구동(Drive)이 없다
            target = joint.GetRelationship(f"physxMimicJoint:rot{axis}:referenceJoint").GetTargets()
            print(f"{joint.GetName():<28} {axis:<3} {'-':>10} {'-':>9} {'-':>10}  "
                  f"{target[0].name if target else '?'} 을 따라 움직임 ({mimic[0]})", flush=True)
            continue
        drive = drive_of(joint)
        force = drive.GetMaxForceAttr().Get()
        print(f"{joint.GetName():<28} {axis:<3} {drive.GetStiffnessAttr().Get():10.2f} {drive.GetDampingAttr().Get():9.3f} "
              f"{(f'{force:.1f}' if force < 1e30 else '제한 없음'):>10}", flush=True)


def joint_prim(stage, name):
    return stage.GetPrimAtPath(f"{JOINTS}/{name}")


def joint_angle(stage, name):
    """관절의 지금 각도 [°] (Play 중에 읽는다)."""
    return joint_prim(stage, name).GetAttribute("state:angular:physics:position").Get()


if __name__ == "__main__":
    stage = new_scene()
    import_pl2m(stage, urdf_path)
    report_import(stage)
    if args.save:
        save_stage(stage, args.save)
    if args.gui:
        set_view(eye=(1.9, -1.9, 1.3), target=(0.0, 0.0, 0.55))

    body = stage.GetPrimAtPath(BODY)
    sim = Sim(app)
    sim.play()
    sim.run(1.0)                              # 바닥에 자리 잡을 때까지
    if args.elbow is not None:
        drive_of(joint_prim(stage, ELBOW)).GetTargetPositionAttr().Set(args.elbow)
    start = sim.time
    x0, y0, z0, yaw0 = pose_of(body)
    print(f"{'시간[s]':>8} {'x[m]':>8} {'y[m]':>8} {'z[m]':>8} {'방향[°]':>8} {'왼팔 팔꿈치[°]':>12}", flush=True)

    def report(time):
        x, y, z, yaw = pose_of(body)
        print(f"{time - start:8.1f} {round(x, 3) + 0.0:8.3f} {round(y, 3) + 0.0:8.3f} {round(z, 3) + 0.0:8.3f} "
              f"{round(yaw, 1) + 0.0:8.1f} {round(joint_angle(stage, ELBOW), 1) + 0.0:12.1f}", flush=True)

    report(start)
    sim.run(args.seconds, every=1.0, report=report)
    x, y, z, yaw = pose_of(body)
    print(f"결과: {args.seconds:g}초 동안 이동 {math.hypot(x - x0, y - y0) * 1000:.1f} mm, 회전 {yaw - yaw0:+.2f}°, "
          f"왼팔 팔꿈치 {round(joint_angle(stage, ELBOW), 1) + 0.0:.1f}°.", flush=True)
    if args.gui:
        sim.wait_until_closed()
    sim.stop()
    app.close()
