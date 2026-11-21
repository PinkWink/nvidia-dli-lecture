"""강의 43: 탁자 위의 컵을 집을 PL-2M 을 띄웁니다. 강의 41 의 ROS 2 연결 · 보간기와 강의 40 의 중력 보상을 그대로 씁니다.

    source /opt/ros/jazzy/setup.bash                                   # Isaac Sim 보다 먼저
    ~/isaacsim/python.sh 43_pl2m_pick/pl2m_pick.py                     # 탁자 + 컵 둘 (0.1 kg)
    ~/isaacsim/python.sh 43_pl2m_pick/pl2m_pick.py --cup-mass 1.0      # 무거운 컵
    ~/isaacsim/python.sh 43_pl2m_pick/pl2m_pick.py --finger-friction 0.2   # 손가락이 미끄러우면
    ~/isaacsim/python.sh 43_pl2m_pick/pl2m_pick.py --soft-mimic        # 비교용: 가져온 그대로의 손가락 묶음
    ~/isaacsim/python.sh 43_pl2m_pick/pl2m_pick.py --shared-shape      # 비교용: 가져온 그대로의 손가락 충돌 모양
    ~/isaacsim/python.sh 43_pl2m_pick/pl2m_pick.py --gui               # 창으로

받는 토픽이 하나 더 있습니다.
    /payload (sensor_msgs/JointState 를 빌려 씀)   name = 팔(left · right), position = 손에 든 물체의 무게 [kg]
    무게를 받으면, 그 무게가 팔 관절에 만드는 토크를 중력 보상에 더합니다 (실제 로봇팔의 "부하 설정" 에 해당).
"""
import argparse
import math
import os
import signal
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, ".."))
PAYLOAD_GRAPH = "/Graphs/ros2_payload"
TIP_OFFSET = 0.0835                              # 손(hand) 링크에서 손끝(tcp)까지, 손의 z 방향 [m] (URDF 의 값)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--cup-mass", type=float, default=None, help="컵의 무게 [kg] (기본은 원본의 0.1)")
    parser.add_argument("--finger-friction", type=float, default=None, help="손가락의 마찰 (기본은 컵과 같은 1.2)")
    parser.add_argument("--soft-mimic", action="store_true", help="둘째 손가락의 묶음을 가져온 그대로(무른 용수철) 둔다 (비교용)")
    parser.add_argument("--shared-shape", action="store_true", help="손가락의 충돌 모양을 가져온 그대로(함께 씀) 둔다 (비교용)")
    parser.add_argument("--iterations", type=int, nargs=2, metavar=("POSITION", "VELOCITY"), help="로봇과 컵의 솔버 반복 횟수 (가져온 로봇의 값은 32 · 1)")
    parser.add_argument("--compensate", choices=["both", "left", "none"], default="both", help="중력 보상을 넣을 팔")
    parser.add_argument("--gui", action="store_true", help="창을 띄운다")
    parser.add_argument("--seconds", type=float, help="이 시간만큼만 돌리고 끝낸다 (없으면 Ctrl+C 나 창을 닫을 때까지)")
    args = parser.parse_args()

    from isaacsim import SimulationApp
    app = SimulationApp({"headless": not args.gui, "extra_args": [
        "--/app/runLoops/main/rateLimitEnabled=true", "--/app/runLoops/main/rateLimitFrequency=60"]})

import numpy as np
import omni.graph.core as og
import omni.physics.tensors
from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics, UsdShade

sys.path.insert(0, HERE)
sys.path.insert(0, REPO)
for folder in ("37_pl2m_import", "39_pl2m_drive", "40_pl2m_gravity", "41_pl2m_ros2"):
    sys.path.insert(0, os.path.join(REPO, folder))
from common.sim import Sim, new_scene, set_pose, set_view
from import_pl2m import DEFAULT_URDF, find_urdf, import_pl2m
from pl2m_drive import ARM_JOINTS, JOINTS, ROBOT
from pl2m_gravity import GRAVITY, LEFT_ARM, RIGHT_ARM, ROOT, GravityCompensation, prepare_pl2m
from pl2m_ros2 import JointCommandInterpolator, add_ros2, enable_ros2
from scene import CUP_DIAMETER, CUP_FRICTION, CUP_HEIGHT, CUP_MASS, CUPS, TABLE_LEG, TABLE_TOP

FINGERS = [f"openarm_{side}_{finger}_finger" for side in ("left", "right") for finger in ("left", "right")]


def add_box(stage, path, center, size, color):
    box = UsdGeom.Cube.Define(stage, path)
    box.CreateSizeAttr(1.0)
    box.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    set_pose(box.GetPrim(), translate=center, scale=size)
    UsdPhysics.CollisionAPI.Apply(box.GetPrim())


def friction_material(stage, name, value):
    """마찰 재질. 닿은 두 물체 가운데 작은 쪽의 마찰을 쓰게 한다 (Combine Mode = min)."""
    material = UsdShade.Material.Define(stage, f"/World/PhysicsMaterials/{name}")
    friction = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    friction.CreateStaticFrictionAttr(value)
    friction.CreateDynamicFrictionAttr(value)
    PhysxSchema.PhysxMaterialAPI.Apply(material.GetPrim()).CreateFrictionCombineModeAttr("min")
    return material


def set_finger_friction(stage, value, own_shape=True):
    """손가락의 충돌 모양에 마찰 재질을 묶는다. own_shape: 함께 쓰는 모양(인스턴스)을 풀어 손가락마다 따로 갖게 한다.

    가져온 그대로는 손가락 넷이 충돌 모양을 함께 쓴다. 그 상태에서는 오른손의 한쪽 손가락이 컵과 충돌하지 않았다.
    """
    material = friction_material(stage, "finger", value)
    for link in FINGERS:
        collisions = stage.GetPrimAtPath(f"{ROBOT}/{link}/collisions")
        if own_shape:
            collisions.SetInstanceable(False)
        UsdShade.MaterialBindingAPI.Apply(collisions).Bind(material, UsdShade.Tokens.strongerThanDescendants, "physics")


def stiffen_mimic(stage):
    """둘째 손가락을 첫째 손가락에 단단히 묶는다.

    가져온 그대로는 묶음이 용수철(고유 진동수 25, 감쇠비 0.005)이라, 컵을 쥐면 둘째 손가락이 밀려 벌어진다.
    고유 진동수를 0 으로 두면 용수철이 아니라 단단한 구속이 된다.
    """
    for side in ("left", "right"):
        joint = stage.GetPrimAtPath(f"{JOINTS}/openarm_{side}_finger_joint2")
        for name in ("naturalFrequency", "dampingRatio"):
            joint.GetAttribute(f"physxMimicJoint:rotY:{name}").Set(0.0)


def add_table_and_cups(stage, cup_mass=CUP_MASS):
    """작은 탁자(고정)와 그 위의 컵 둘(움직이는 물체). 컵에는 마찰 재질을 묶는다."""
    add_box(stage, "/World/Table/Top", TABLE_TOP["center"], TABLE_TOP["size"], (0.92, 0.92, 0.90))
    add_box(stage, "/World/Table/Leg", TABLE_LEG["center"], TABLE_LEG["size"], (0.45, 0.45, 0.47))
    material = friction_material(stage, "cup", CUP_FRICTION)
    for name, center, color in CUPS.values():
        cup = UsdGeom.Cylinder.Define(stage, f"/World/Cups/{name}")
        cup.CreateRadiusAttr(CUP_DIAMETER / 2)
        cup.CreateHeightAttr(CUP_HEIGHT)
        cup.CreateAxisAttr("Z")
        cup.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        set_pose(cup.GetPrim(), translate=center)
        UsdPhysics.CollisionAPI.Apply(cup.GetPrim())
        UsdPhysics.RigidBodyAPI.Apply(cup.GetPrim())
        UsdPhysics.MassAPI.Apply(cup.GetPrim()).CreateMassAttr(cup_mass)
        PhysxSchema.PhysxContactReportAPI.Apply(cup.GetPrim()).CreateThresholdAttr(0.0)     # 손가락에서 받는 힘을 재려고
        UsdShade.MaterialBindingAPI.Apply(cup.GetPrim()).Bind(material, UsdShade.Tokens.strongerThanDescendants, "physics")


def add_payload_subscriber():
    """/payload 를 받는 그래프. JointState 를 빌려 쓴다: name = 팔, position = 무게 [kg]."""
    keys = og.Controller.Keys
    og.Controller.edit({"graph_path": PAYLOAD_GRAPH, "evaluator_name": "execution"}, {
        keys.CREATE_NODES: [("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                            ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                            ("Subscribe", "isaacsim.ros2.bridge.ROS2SubscribeJointState")],
        keys.SET_VALUES: [("Subscribe.inputs:topicName", "payload"), ("Subscribe.inputs:queueSize", 1)],
        keys.CONNECT: [("OnPlaybackTick.outputs:tick", "Subscribe.inputs:execIn"),
                       ("Context.outputs:context", "Subscribe.inputs:context")]})


class PayloadCompensation(GravityCompensation):
    """강의 40 의 중력 보상 + 손에 든 물체의 무게. 물체를 손끝에 있는 점 질량으로 보고, 강의 40 의 수식을 그대로 쓴다.

        τ_j = m · g · ( a_j · ( (c − p_j) × ẑ ) )      m 물체의 질량, c 손끝의 위치, a_j · p_j 관절의 축과 축 위의 한 점
    """

    def __init__(self, stage, joints=ARM_JOINTS):
        super().__init__(joints=joints)
        self.stage, self.payload = stage, {"left": 0.0, "right": 0.0}
        self._names = og.Controller.attribute("outputs:jointNames", PAYLOAD_GRAPH + "/Subscribe")
        self._masses = og.Controller.attribute("outputs:positionCommand", PAYLOAD_GRAPH + "/Subscribe")
        self.chain = {}
        for side, arm in (("left", LEFT_ARM), ("right", RIGHT_ARM)):          # 관절마다: 뒤쪽 링크, 그 링크에서 본 축과 축 위의 점
            self.chain[side] = []
            for name in arm:
                if name not in joints:
                    continue
                joint = UsdPhysics.RevoluteJoint(stage.GetPrimAtPath(f"{JOINTS}/{name}"))
                axis = Gf.Rotation(Gf.Quatd(joint.GetLocalRot1Attr().Get())).TransformDir(Gf.Vec3d(*[float(joint.GetAxisAttr().Get() == a) for a in "XYZ"]))
                self.chain[side].append((self.names.index(name), joint.GetBody1Rel().GetTargets()[0].name, Gf.Vec3d(*joint.GetLocalPos1Attr().Get()), axis))

    def world(self, link):
        return UsdGeom.Xformable(self.stage.GetPrimAtPath(f"{ROBOT}/{link}")).ComputeLocalToWorldTransform(0)

    def add_extra(self, efforts):
        names, masses = self._names.get(), self._masses.get()
        if masses is not None and len(masses):                                  # 새로 받은 무게
            for name, mass in zip(names, masses):
                if str(name) in self.payload and self.payload[str(name)] != float(mass):
                    self.payload[str(name)] = float(mass)
                    print(f"부하 설정: {'왼팔' if str(name) == 'left' else '오른팔'} {float(mass):.2f} kg", flush=True)
        up = Gf.Vec3d(0, 0, 1)
        for side, mass in self.payload.items():
            if mass <= 0.0:
                continue
            tip = self.world(f"openarm_{side}_hand").Transform(Gf.Vec3d(0, 0, TIP_OFFSET))
            for dof, link, point, axis in self.chain[side]:
                frame = self.world(link)
                arm = Gf.Dot(frame.TransformDir(axis).GetNormalized(), Gf.Cross(tip - frame.Transform(point), up))
                efforts[0, dof] += mass * GRAVITY * arm


class GripScale:
    """컵이 두 손가락에서 받는 힘을 읽는다. Play 뒤에 만든다."""

    def __init__(self):
        self.physics = omni.physics.tensors.create_simulation_view("numpy")
        self.views = {}
        for side, (name, _, _) in CUPS.items():
            fingers = [f"{ROBOT}/openarm_{side}_{finger}_finger" for finger in ("right", "left")]
            self.views[name] = self.physics.create_rigid_contact_view(f"/World/Cups/{name}", filter_patterns=fingers, max_contact_data_count=64)

    def read(self, name, dt=1.0 / 60.0):
        """(손가락마다 컵을 누르는 힘 [N], 손가락마다 마찰로 컵을 위로 받치는 힘 [N])."""
        forces = self.views[name].get_contact_force_matrix(dt)[0]              # 손가락 2개 × (x, y, z): 면에 수직인 힘
        friction, _, counts, starts = self.views[name].get_friction_data(dt)   # 마찰력은 따로 온다: 접촉점마다 (x, y, z)
        lift = [float(friction[int(starts[0][k]):int(starts[0][k]) + int(counts[0][k]), 2].sum()) for k in range(len(forces))]
        return [abs(float(force[1])) for force in forces], lift


def cup_states(stage):
    """컵마다 (가운데의 높이 [m], 기운 각도 [°], 처음 자리에서 옆으로 움직인 거리 [m])."""
    states = {}
    for side, (name, center, _) in CUPS.items():
        matrix = UsdGeom.Xformable(stage.GetPrimAtPath(f"/World/Cups/{name}")).ComputeLocalToWorldTransform(0)
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, matrix.TransformDir(Gf.Vec3d(0, 0, 1)).GetNormalized()[2]))))
        now = matrix.ExtractTranslation()
        states[name] = (now[2], tilt, math.hypot(now[0] - center[0], now[1] - center[1]))
    return states


def hand_heights(stage):
    """두 손끝의 높이 [m]."""
    return {side: UsdGeom.Xformable(stage.GetPrimAtPath(f"{ROBOT}/openarm_{side}_hand")).ComputeLocalToWorldTransform(0)
            .Transform(Gf.Vec3d(0, 0, TIP_OFFSET))[2] for side in ("left", "right")}


if __name__ == "__main__":
    enable_ros2(app)
    stage = new_scene()
    robot = import_pl2m(stage, find_urdf(DEFAULT_URDF))
    set_pose(robot, translate=(0.0, 0.0, 0.002))
    prepare_pl2m(stage)
    set_finger_friction(stage, args.finger_friction if args.finger_friction is not None else CUP_FRICTION, own_shape=not args.shared_shape)
    if not args.soft_mimic:
        stiffen_mimic(stage)
    add_ros2()
    add_payload_subscriber()
    add_table_and_cups(stage, args.cup_mass if args.cup_mass is not None else CUP_MASS)
    if args.iterations:                                   # 로봇(관절체)과 컵 둘 다에 준다
        robot_solver = PhysxSchema.PhysxArticulationAPI.Apply(stage.GetPrimAtPath(ROOT))
        robot_solver.CreateSolverPositionIterationCountAttr(args.iterations[0])
        robot_solver.CreateSolverVelocityIterationCountAttr(args.iterations[1])
        for name, _, _ in CUPS.values():
            cup_solver = PhysxSchema.PhysxRigidBodyAPI.Apply(stage.GetPrimAtPath(f"/World/Cups/{name}"))
            cup_solver.CreateSolverPositionIterationCountAttr(args.iterations[0])
            cup_solver.CreateSolverVelocityIterationCountAttr(args.iterations[1])
    if args.gui:
        set_view(eye=(1.75, 1.55, 1.35), target=(0.35, 0.05, 0.75))
    sim = Sim(app)
    sim.play()
    sim.run(1.0)
    compensation = PayloadCompensation(stage, joints={"both": ARM_JOINTS, "left": LEFT_ARM, "none": []}[args.compensate])
    interpolator = JointCommandInterpolator(sim)
    grip = GripScale()
    print(f"준비됨: 중력 보상 = {args.compensate}, 컵 {args.cup_mass if args.cup_mass is not None else CUP_MASS:g} kg. 끝내려면 Ctrl+C", flush=True)
    stop, clock, told = [], sim.time, {name: (state[0], state[2]) for name, state in cup_states(stage).items()}
    fallen, pressed = set(), {}
    signal.signal(signal.SIGINT, lambda *_: stop.append(True))
    while app.is_running() and not stop and (not args.seconds or sim.time - clock < args.seconds):
        app.update()
        for name, (height, tilt, moved) in cup_states(stage).items():         # 컵이 위아래로 2 cm, 옆으로 5 cm 넘게 움직이면 알린다
            if name in fallen:
                continue
            if height < 0.70:                                                 # 탁자보다 아래: 떨어졌다. 더는 알리지 않는다
                fallen.add(name)
                print(f"{sim.time - clock:6.1f}초  {name}: 떨어졌음", flush=True)
            elif abs(height - told[name][0]) > 0.02 or abs(moved - told[name][1]) > 0.05:
                told[name] = (height, moved)
                press, lift = grip.read(name)
                pressed[name] = (press, lift)
                print(f"{sim.time - clock:6.1f}초  {name}: 높이 {height:.3f} m, 기울기 {tilt:.0f}°, 옆으로 {moved * 100:.1f} cm"
                      f"   손가락이 누르는 힘 {press[0]:.1f} · {press[1]:.1f} N, 마찰로 받치는 힘 {lift[0]:.1f} · {lift[1]:.1f} N", flush=True)
    print("끝날 때: " + ",  ".join(f"{name} 높이 {height:.3f} m · 기울기 {tilt:.0f}° · 옆으로 {moved * 100:.1f} cm"
                                 for name, (height, tilt, moved) in cup_states(stage).items()), flush=True)
    print("        손끝의 높이 " + ",  ".join(f"{'왼손' if side == 'left' else '오른손'} {height:.4f} m" for side, height in hand_heights(stage).items()), flush=True)
    interpolator.stop()
    compensation.stop()
    sim.stop()
    app.close()
