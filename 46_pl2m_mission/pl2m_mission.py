"""강의 46: 부스에서 컵을 집어 나르는 임무의 시뮬레이터 쪽. 강의 43 · 44 · 45 의 장면을 하나로 합칩니다.

    source /opt/ros/jazzy/setup.bash                                              # Isaac Sim 보다 먼저
    ~/isaacsim/python.sh 46_pl2m_mission/pl2m_mission.py                          # 처음 자리에서 시작
    ~/isaacsim/python.sh 46_pl2m_mission/pl2m_mission.py --gui                    # 창으로 보기
    ~/isaacsim/python.sh 46_pl2m_mission/pl2m_mission.py --start drinks           # 음료대 앞(Nav2 의 목적지)에서 시작
    ~/isaacsim/python.sh 46_pl2m_mission/pl2m_mission.py --start drinks --docked  # 음료대에 도킹한 자리에서 시작 (팔 동작만 볼 때)
    ~/isaacsim/python.sh 46_pl2m_mission/pl2m_mission.py --contacts               # 팔이 부스의 물체나 사람 모형에 닿으면 알린다
    ~/isaacsim/python.sh 46_pl2m_mission/pl2m_mission.py --log run.json           # 참값 · 오도메트리 · 컵의 자리와 기울기 · 손의 기울기를 적어 둔다

합친 것
    강의 38   부스 (바닥판은 아래로 두껍게, 사람 모형은 충돌체)
    강의 44   라이다 · 바퀴 오도메트리
    강의 45   마커 6장 · 앞 카메라
    강의 43   손가락의 묶음을 단단하게 · 손가락마다 자기 충돌 모양 · 손에 든 무게의 보상 (/payload)
"""
import argparse
import math
import os
import signal
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, ".."))

if __name__ == "__main__":
    for folder in ("44_pl2m_nav", "45_pl2m_dock"):
        sys.path.insert(0, os.path.join(REPO, folder))
    from booth import STATIONS
    parser = argparse.ArgumentParser()
    parser.add_argument("--start", choices=list(STATIONS), default="home", help="시작하는 자리")
    parser.add_argument("--docked", action="store_true", help="--start 의 스테이션에 도킹을 마친 자리에서 시작한다")
    parser.add_argument("--offset", type=float, nargs=2, metavar=("SIDE", "YAW"), default=(0.0, 0.0), help="시작 자리를 왼쪽으로 SIDE [m], 왼쪽으로 YAW [°] 튼다")
    parser.add_argument("--no-payload", action="store_true", help="손에 든 무게를 보상하지 않는다 (비교용)")
    parser.add_argument("--contacts", action="store_true", help="팔이 부스의 물체나 사람 모형에 닿으면 0.5초마다 알린다 (원인을 찾을 때)")
    parser.add_argument("--gui", action="store_true", help="창을 띄운다")
    parser.add_argument("--seconds", type=float, help="이 시간만큼만 돌리고 끝낸다")
    parser.add_argument("--log", metavar="JSON", help="참값 · 오도메트리 · 컵의 자리를 0.1초마다 적어 이 파일에 저장한다")
    args = parser.parse_args()

    from isaacsim import SimulationApp
    app = SimulationApp({"headless": not args.gui, "extra_args": [
        "--/app/runLoops/main/rateLimitEnabled=true", "--/app/runLoops/main/rateLimitFrequency=60"]})

from pxr import Gf, UsdGeom, UsdShade

sys.path.insert(0, HERE)
sys.path.insert(0, REPO)
for folder in ("38_pl2m_booth", "39_pl2m_drive", "40_pl2m_gravity", "41_pl2m_ros2", "43_pl2m_pick", "44_pl2m_nav", "45_pl2m_dock"):
    sys.path.insert(0, os.path.join(REPO, folder))
import make_booth
import pl2m_pick as grasp                     # 강의 43: 쥐는 데 필요한 것들
from booth import STATIONS
from common.sim import Sim, new_scene, set_view
from markers import docked_pose
from pl2m_dock import VIEWS, build_dock_scene, start_pose
from pl2m_drive import ARM_JOINTS
from pl2m_nav import BasePose
from pl2m_ros2 import JointCommandInterpolator, enable_ros2
from scene import CUP_FRICTION

CUP_NAMES = [name for name, _, _ in make_booth.CUPS]


def prepare_grasp(stage):
    """강의 43 에서 찾은, 쥐기 위한 설정 넷."""
    grasp.set_finger_friction(stage, CUP_FRICTION)          # 손가락마다 자기 충돌 모양 + 마찰 1.2
    grasp.stiffen_mimic(stage)                              # 둘째 손가락을 단단히 묶음
    grasp.add_payload_subscriber()                          # /payload: 손에 든 무게
    material = grasp.friction_material(stage, "cup", CUP_FRICTION)
    for name in CUP_NAMES:
        cup = stage.GetPrimAtPath(f"/World/Cups/{name}")
        UsdShade.MaterialBindingAPI.Apply(cup).Bind(material, UsdShade.Tokens.strongerThanDescendants, "physics")


def watch_contacts(stage):
    """팔 링크에 접촉 보고를 켠다. 돌려주는 사전에 {(팔 링크, 닿은 물체): 그동안의 충격량 [N·s]} 이 쌓인다.

    로봇의 다른 링크나 컵과 닿는 것은 빼고, 부스에 있는 것(가구 · 사람 모형)과 닿는 것만 적는다.
    """
    import omni.physx
    from pxr import PhysicsSchemaTools, PhysxSchema, Usd, UsdPhysics
    for prim in Usd.PrimRange(stage.GetPrimAtPath("/World/pl2m")):
        if prim.HasAPI(UsdPhysics.RigidBodyAPI) and "openarm" in prim.GetName():
            PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.0)
    touched = {}

    def on_contact(headers, data):
        for header in headers:
            bodies = [str(PhysicsSchemaTools.intToSdfPath(actor)) for actor in (header.actor0, header.actor1)]
            arm = [body for body in bodies if body.startswith("/World/pl2m/")]
            other = [body for body in bodies if body.startswith(make_booth.BOOTH)]
            if arm and other:
                impulse = sum(math.sqrt(sum(v * v for v in data[i].impulse)) for i in range(header.contact_data_offset, header.contact_data_offset + header.num_contact_data))
                key = (arm[0].rsplit("/", 1)[-1], other[0].replace(make_booth.BOOTH + "/", ""))
                touched[key] = touched.get(key, 0.0) + impulse

    touched["_subscription"] = omni.physx.get_physx_simulation_interface().subscribe_contact_report_events(on_contact)     # 붙들고 있어야 계속 불린다
    return touched


def cup_states(stage):
    """컵마다 (x, y, z, 기운 각도 [°])."""
    states = {}
    for name in CUP_NAMES:
        matrix = UsdGeom.Xformable(stage.GetPrimAtPath(f"/World/Cups/{name}")).ComputeLocalToWorldTransform(0)
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, matrix.TransformDir(Gf.Vec3d(0, 0, 1)).GetNormalized()[2]))))
        x, y, z = matrix.ExtractTranslation()
        states[name] = (x, y, z, tilt)
    return states


def hand_tilts(stage):
    """두 손이 쥐는 방향(앞을 보고 옆에서 쥠)에서 기운 각도 [°]: (왼손, 오른손). 손의 x 축이 아래를 보면 0 이다."""
    tilts = []
    for side in ("left", "right"):
        matrix = UsdGeom.Xformable(stage.GetPrimAtPath(f"/World/pl2m/openarm_{side}_hand")).ComputeLocalToWorldTransform(0)
        tilts.append(math.degrees(math.acos(max(-1.0, min(1.0, -matrix.TransformDir(Gf.Vec3d(1, 0, 0)).GetNormalized()[2])))))
    return tilts


if __name__ == "__main__":
    enable_ros2(app)
    stage = new_scene()
    if args.docked:
        x, y, yaw = docked_pose(args.start)
        start = (x - args.offset[0] * math.sin(yaw), y + args.offset[0] * math.cos(yaw), yaw + math.radians(args.offset[1]))
    else:
        start = start_pose(args.start, *args.offset)
    build_dock_scene(app, stage, start)
    prepare_grasp(stage)
    if args.gui:
        set_view(*VIEWS["booth"])                                     # 음료대와 상담 테이블 1 이 함께 보이는 자리
    touched = watch_contacts(stage) if args.contacts else None
    sim = Sim(app)
    sim.play()
    sim.run(1.0)
    compensation = grasp.PayloadCompensation(stage, joints=ARM_JOINTS)
    if args.no_payload:
        compensation.add_extra = lambda efforts: None
    interpolator = JointCommandInterpolator(sim)
    pose = BasePose(sim, start)
    print(f"준비됨: 시작 자리 {args.start}{' (도킹한 자리)' if args.docked else ''} ({start[0]:.3f}, {start[1]:.3f}, {math.degrees(start[2]):.1f}°), "
          f"컵 {make_booth.CUP_MASS:g} kg. 끝내려면 Ctrl+C", flush=True)
    stop, clock, told, cups, tilts, hands, logged, reported = [], sim.time, cup_states(stage), [], [], [], -1.0, -1.0
    signal.signal(signal.SIGINT, lambda *_: stop.append(True))
    while app.is_running() and not stop and (not args.seconds or sim.time - clock < args.seconds):
        app.update()
        states = cup_states(stage)
        if sim.time - logged >= 0.1:
            logged = sim.time
            cups.append([round(sim.time, 2)] + [round(v, 4) for name in CUP_NAMES for v in states[name][:3]])
            tilts.append([round(sim.time, 2)] + [round(states[name][3], 2) for name in CUP_NAMES])
            hands.append([round(sim.time, 2)] + [round(value, 2) for value in hand_tilts(stage)])
        if touched is not None and sim.time - reported >= 0.5:
            reported = sim.time
            hits = {key: value for key, value in touched.items() if key != "_subscription" and value > 0.5}      # 스치기만 한 것(0.5 N·s 아래)은 뺀다
            if hits:
                print(f"{sim.time:7.1f}초  팔이 닿음: " + ",  ".join(f"{arm} ↔ {other} ({value:.1f} N·s)" for (arm, other), value in sorted(hits.items(), key=lambda item: -item[1])[:3])
                      + f"   로봇의 방향 {math.degrees(pose.truth()[2]):.1f}°", flush=True)
            for key in [key for key in touched if key != "_subscription"]:
                del touched[key]
        for name, (x, y, z, tilt) in states.items():                   # 컵이 위아래로 2 cm, 옆으로 10 cm 넘게 움직일 때마다 알린다
            before = told[name]
            if abs(z - before[2]) > 0.02 or math.hypot(x - before[0], y - before[1]) > 0.10:
                told[name] = (x, y, z, tilt)
                print(f"{sim.time:7.1f}초  {name}: ({x:.3f}, {y:.3f}), 높이 {z:.3f} m, 기울기 {tilt:.0f}°", flush=True)
    truth = pose.truth()
    print(f"끝날 때 로봇의 참값 ({truth[0]:.3f}, {truth[1]:.3f}, {math.degrees(truth[2]):.1f}°)", flush=True)
    print("끝날 때 컵: " + ",  ".join(f"{name} ({x:.3f}, {y:.3f}) 높이 {z:.3f} m · 기울기 {tilt:.0f}°" for name, (x, y, z, tilt) in cup_states(stage).items()), flush=True)
    if args.log:
        import json
        with open(args.log, "w") as file:
            json.dump({"columns": ["time", "true_x", "true_y", "true_yaw", "odom_x", "odom_y", "odom_yaw"], "start": list(start), "rows": pose.log,
                       "cup_names": CUP_NAMES, "cups": cups, "tilts": tilts, "hands": hands}, file)
        print("저장:", args.log, flush=True)
    pose.stop()
    interpolator.stop()
    compensation.stop()
    sim.stop()
    app.close()
