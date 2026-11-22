"""강의 46: 부스에서 컵을 집어 나르는 임무의 시뮬레이터 쪽. 강의 43 · 44 · 45 의 장면을 하나로 합칩니다.

    source /opt/ros/jazzy/setup.bash                                              # Isaac Sim 보다 먼저
    ~/isaacsim/python.sh 46_pl2m_mission/pl2m_mission.py                          # 처음 자리에서 시작
    ~/isaacsim/python.sh 46_pl2m_mission/pl2m_mission.py --gui                    # 창으로 보기
    ~/isaacsim/python.sh 46_pl2m_mission/pl2m_mission.py --start drinks           # 음료대 앞(Nav2 의 목적지)에서 시작
    ~/isaacsim/python.sh 46_pl2m_mission/pl2m_mission.py --start drinks --docked  # 음료대에 도킹한 자리에서 시작 (팔 동작만 볼 때)
    ~/isaacsim/python.sh 46_pl2m_mission/pl2m_mission.py --contacts               # 팔이 부스의 물체나 사람 모형에 닿으면 알린다
    ~/isaacsim/python.sh 46_pl2m_mission/pl2m_mission.py --grip-points            # 손가락이 컵의 어디를 누르는지 알린다
    ~/isaacsim/python.sh 46_pl2m_mission/pl2m_mission.py --mesh-fingers           # 비교용: 손가락에 패드를 덧대지 않는다
    ~/isaacsim/python.sh 46_pl2m_mission/pl2m_mission.py --log run.json           # 참값 · 오도메트리 · 컵의 자리와 기울기 · 손의 기울기를 적어 둔다

합친 것
    강의 38   부스 (사람 모형은 충돌체)
    강의 44   라이다 · 바퀴 오도메트리
    강의 45   마커 6장 · 앞 카메라
    강의 43   손가락의 묶음을 단단하게 · 손가락마다 자기 충돌 모양 · 손에 든 무게의 보상 (/payload)
더한 것
    손가락 안쪽에 평평한 패드를 덧댄다 (add_finger_pads). 가져온 충돌 모양만으로는 든 컵이 손 안에서 기울었다.
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
    parser.add_argument("--grip-points", action="store_true", help="손가락이 컵을 누르는 점(컵의 가운데 기준)과 힘을 0.5초마다 알린다")
    parser.add_argument("--mesh-fingers", action="store_true", help="비교용: 손가락에 패드를 덧대지 않는다")
    parser.add_argument("--gui", action="store_true", help="창을 띄운다")
    parser.add_argument("--seconds", type=float, help="이 시간만큼만 돌리고 끝낸다")
    parser.add_argument("--log", metavar="JSON", help="참값 · 오도메트리 · 컵의 자리를 0.1초마다 적어 이 파일에 저장한다")
    args = parser.parse_args()

    from isaacsim import SimulationApp
    app = SimulationApp({"headless": not args.gui, "extra_args": [
        "--/app/runLoops/main/rateLimitEnabled=true", "--/app/runLoops/main/rateLimitFrequency=60"]})

from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics, UsdShade

sys.path.insert(0, HERE)
sys.path.insert(0, REPO)
for folder in ("38_pl2m_booth", "39_pl2m_drive", "40_pl2m_gravity", "41_pl2m_ros2", "43_pl2m_pick", "44_pl2m_nav", "45_pl2m_dock"):
    sys.path.insert(0, os.path.join(REPO, folder))
import make_booth
import pl2m_pick as grasp                     # 강의 43: 쥐는 데 필요한 것들
from booth import STATIONS
from common.sim import Sim, new_scene, set_pose, set_view
from markers import docked_pose
from pl2m_dock import VIEWS, build_dock_scene, start_pose
from pl2m_drive import ARM_JOINTS
from pl2m_nav import BasePose
from pl2m_ros2 import JointCommandInterpolator, enable_ros2
from scene import CUP_FRICTION

CUP_NAMES = [name for name, _, _ in make_booth.CUPS]
# 손가락 안쪽에 덧대는 평평한 패드 (손가락 링크 기준, 왼쪽 손가락. 오른쪽 손가락은 y 의 부호가 반대)
# x = 손가락의 폭 (쥐었을 때 컵의 세로 방향), y = 쥐는 방향, z = 손가락의 길이
# 가져온 충돌 모양의 안쪽 면(y = -0.0078)보다 0.3 mm 앞으로 나오게 놓는다. 폭과 길이는 그 모양보다 3 mm 씩 작다
FINGER_PAD = {"center": (0.0, -0.0061, 0.051), "size": (0.055, 0.004, 0.078)}


def add_finger_pads(stage):
    """손가락 안쪽에 평평한 패드(얇은 상자)를 덧댄다. 컵의 옆면에 세로로 길게 닿는다.

    가져온 충돌 모양(메시)은 컵에 한 점으로 닿았다. 두 손가락의 점이 같은 높이에 오면 컵이 그 두 점을 잇는 선을 축으로 돌았다 (강의 46 의 9절).
    가져온 모양은 그대로 둔다. 패드는 그보다 0.3 mm 만 앞으로 나와 있어, 컵과는 패드가 먼저 닿고 다른 물체와의 충돌은 전과 같다.
    """
    material = UsdShade.Material(stage.GetPrimAtPath("/World/PhysicsMaterials/finger"))
    for link in grasp.FINGERS:
        pad = UsdGeom.Cube.Define(stage, f"{grasp.ROBOT}/{link}/pad")
        pad.CreateSizeAttr(1.0)
        x, y, z = FINGER_PAD["center"]
        set_pose(pad.GetPrim(), translate=(x, y if link.endswith("left_finger") else -y, z), scale=FINGER_PAD["size"])
        UsdPhysics.CollisionAPI.Apply(pad.GetPrim())
        UsdGeom.Imageable(pad.GetPrim()).MakeInvisible()                                  # 충돌에만 쓰고 그리지는 않는다
        UsdShade.MaterialBindingAPI.Apply(pad.GetPrim()).Bind(material, UsdShade.Tokens.strongerThanDescendants, "physics")


class GripPoints:
    """손가락이 컵을 누르는 점을 읽는다. Play 뒤에 만든다."""

    def __init__(self, stage):
        import omni.physics.tensors
        self.stage = stage
        self.physics = omni.physics.tensors.create_simulation_view("numpy")
        self.views = {name: self.physics.create_rigid_contact_view(f"/World/Cups/{name}", filter_patterns=[f"{grasp.ROBOT}/{link}" for link in grasp.FINGERS],
                                                                   max_contact_data_count=64) for name in CUP_NAMES}

    def read(self, name, dt=1.0 / 60.0):
        """[(손가락 이름, 컵의 가운데에서 컵의 축을 따라 위로 [mm], 손이 다가가는 쪽으로 [mm], 누르는 힘 [N])]. 0.5 N 넘는 점만."""
        forces, points, _, _, counts, starts = self.views[name].get_contact_data(dt)
        cup = UsdGeom.Xformable(self.stage.GetPrimAtPath(f"/World/Cups/{name}")).ComputeLocalToWorldTransform(0)
        center, axis = cup.ExtractTranslation(), cup.TransformDir(Gf.Vec3d(0, 0, 1)).GetNormalized()
        found = []
        for k, link in enumerate(grasp.FINGERS):
            hand = UsdGeom.Xformable(self.stage.GetPrimAtPath(f"{grasp.ROBOT}/openarm_{link.split('_')[1]}_hand")).ComputeLocalToWorldTransform(0)
            ahead = hand.TransformDir(Gf.Vec3d(0, 0, 1)).GetNormalized()                 # 손이 다가가는 방향
            for i in range(int(starts[0][k]), int(starts[0][k]) + int(counts[0][k])):
                force = abs(float(forces[i][0]))
                if force > 0.5:
                    offset = Gf.Vec3d(*[float(v) for v in points[i]]) - center
                    found.append((link.replace("openarm_", "").replace("_finger", ""), offset * axis * 1000.0, offset * ahead * 1000.0, force))
        return found


def prepare_grasp(stage, mesh_fingers=False):
    """강의 43 에서 찾은, 쥐기 위한 설정 넷 + 손가락 안쪽의 패드."""
    grasp.set_finger_friction(stage, CUP_FRICTION)          # 손가락마다 자기 충돌 모양 + 마찰 1.2
    grasp.stiffen_mimic(stage)                              # 둘째 손가락을 단단히 묶음
    grasp.add_payload_subscriber()                          # /payload: 손에 든 무게
    if not mesh_fingers:
        add_finger_pads(stage)                              # 안쪽 면이 평평한 패드
    material = grasp.friction_material(stage, "cup", CUP_FRICTION)
    for name in CUP_NAMES:
        cup = stage.GetPrimAtPath(f"/World/Cups/{name}")
        UsdShade.MaterialBindingAPI.Apply(cup).Bind(material, UsdShade.Tokens.strongerThanDescendants, "physics")
        PhysxSchema.PhysxContactReportAPI.Apply(cup).CreateThresholdAttr(0.0)             # --grip-points 가 접촉점을 읽으려고


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
    prepare_grasp(stage, mesh_fingers=args.mesh_fingers)
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
    grip, gripped = GripPoints(stage) if args.grip_points else None, -1.0
    signal.signal(signal.SIGINT, lambda *_: stop.append(True))
    while app.is_running() and not stop and (not args.seconds or sim.time - clock < args.seconds):
        app.update()
        states = cup_states(stage)
        if sim.time - logged >= 0.1:
            logged = sim.time
            cups.append([round(sim.time, 2)] + [round(v, 4) for name in CUP_NAMES for v in states[name][:3]])
            tilts.append([round(sim.time, 2)] + [round(states[name][3], 2) for name in CUP_NAMES])
            hands.append([round(sim.time, 2)] + [round(value, 2) for value in hand_tilts(stage)])
        if grip and sim.time - gripped >= 0.5:
            gripped = sim.time
            for name in CUP_NAMES:
                found = grip.read(name)
                if found:
                    print(f"{sim.time:7.1f}초  {name} 기울기 {states[name][3]:4.1f}°  누르는 점: "
                          + ",  ".join(f"{finger} 위로 {up:+.0f} mm · 앞으로 {ahead:+.1f} mm · {force:.0f} N" for finger, up, ahead, force in found), flush=True)
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
