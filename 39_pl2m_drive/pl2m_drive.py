"""강의 39: PL-2M 을 움직이게 만듭니다. 바퀴 · 팔 · 그리퍼의 구동 값과 접촉 설정을 정하고, 하나씩 재 봅니다.

    ~/isaacsim/python.sh 39_pl2m_drive/pl2m_drive.py --test wheels      # 바퀴: 설정을 하나씩 더하며 직진 · 회전을 잼
    ~/isaacsim/python.sh 39_pl2m_drive/pl2m_drive.py --test arms        # 팔: 구동 값 세 가지로 자세를 바꿔 봄
    ~/isaacsim/python.sh 39_pl2m_drive/pl2m_drive.py --test gripper     # 그리퍼: 열고 닫기
    ~/isaacsim/python.sh 39_pl2m_drive/pl2m_drive.py --test hold        # 팔을 움직일 때 로봇이 제자리를 지키는지 (한 번에 · 1초에 걸쳐, 모터 게인 10 · 100)
    ~/isaacsim/python.sh 39_pl2m_drive/pl2m_drive.py --test slab        # 참고: 바닥이 얇은 상자일 때 직진해 보기
    ~/isaacsim/python.sh 39_pl2m_drive/pl2m_drive.py                    # 다 맞춘 로봇으로 직진 → 회전 → 팔 자세
    ~/isaacsim/python.sh 39_pl2m_drive/pl2m_drive.py --booth --gui --keyboard     # 부스 안에서 W A S D 로 운전

값의 출처: 저장소 pinklab_robotworld2026 의 MuJoCo 설정 (src/mobile_openarm_mujoco/config/mujoco.yaml · model.py)
    바퀴 모터  velocity_gain 10 N·m·s/rad, max_torque 18 N·m          팔      kp 500 N·m/rad, kv 35 N·m·s/rad, 관절 토크로 중력 보상
      (속도 게인만 100 으로 올려 씀: 10 은 제자리 회전이 5 % 느렸음)
      (중력 보상은 옮기지 않았음: 팔 링크의 중력을 꺼서 처짐만 없앰. 팔 12 kg 의 무게가 베이스에 실리지 않는 점이 원본과 다름)
    마찰       캐스터 0.002, 바퀴 1.2                                  그리퍼  kp 1800 N/m, kv 15 N·s/m, 최대 30 N
USD 의 회전 관절 구동 값은 "도" 기준이라, 라디안 기준 값에 π/180 을 곱해 넣습니다.
"""
import argparse
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, ".."))
ROBOT = "/World/pl2m"
BODY = ROBOT + "/base_link"
JOINTS = ROBOT + "/joints"
GRAPH = "/Graphs/pl2m_drive"
WHEEL_JOINTS = ["left_wheel_joint", "right_wheel_joint"]
WHEEL_LINKS = ["left_wheel", "right_wheel"]
CASTER_LINKS = ["front_left_caster", "front_right_caster", "rear_left_caster", "rear_right_caster"]
ARM_JOINTS = [f"openarm_{side}_joint{i}" for side in ("left", "right") for i in range(1, 8)]
FINGER_JOINTS = ["openarm_left_finger_joint1", "openarm_right_finger_joint1"]        # 둘째 손가락은 이것을 따라 움직인다
WHEEL_RADIUS, WHEEL_DISTANCE = 0.0825, 0.4288       # 바퀴 반지름 · 좌우 바퀴 사이 거리 [m] (vicpinky_description 의 값)
MAX_LINEAR, MAX_ANGULAR = 0.35, 0.7                 # 원본 설정의 속도 제한 [m/s], [rad/s]
MOTOR_GAIN, MOTOR_TORQUE = 100.0, 18.0              # 바퀴 모터: 속도 게인 [N·m·s/rad] (원본은 10, --test wheels · hold 참고), 최대 토크 [N·m]
WHEEL_ARMATURE, WHEEL_WIDTH = 0.05, 0.01            # 바퀴 관절에 더하는 회전 관성 [kg·m²], 바퀴 충돌 모양의 폭 [m]
CASTER_FRICTION, WHEEL_FRICTION = 0.002, 1.2
ARM_KP, ARM_KV = 500.0, 35.0                        # 팔: [N·m/rad], [N·m·s/rad]
GRIP_KP, GRIP_KV, GRIP_FORCE = 1800.0, 15.0, 30.0   # 그리퍼: [N/m], [N·s/m], [N]
GRIP_OPEN = 0.044                                   # 손가락 하나가 벌어지는 거리 [m]
SLAB_TOP = 0.02                                     # --test slab 의 상자: 윗면의 높이 [m]. 두께를 바꿀 때는 아래로 늘인다
PER_DEGREE = math.pi / 180.0                        # 라디안 기준 값 → 도 기준 값
POSES = {  # 팔 자세 [rad] (원본 config/drive_poses.yaml). 왼팔 7개, 오른팔은 4번(팔꿈치)만 같고 나머지는 부호가 반대
    "down": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "transport": [0.0, 0.0, 0.0, 1.2, 0.0, 0.0, 0.0],
    "ready": [0.0, 0.0, 0.0, 1.65, 0.0, 0.0, 0.0],
    "carry": [0.93, -2.3726, -0.4355, 2.3729, 0.387, -0.7903, 1.3904],
}
MIRROR = [-1, -1, -1, 1, -1, -1, -1]                # 오른팔 = 왼팔 × 이 부호 (원본의 carry 자세와 대조함)


def pose_degrees(name):
    """자세 이름 → 팔 관절 14개의 목표 각도 [°] (ARM_JOINTS 순서)."""
    left = POSES[name]
    return [math.degrees(v) for v in left] + [math.degrees(v * s) for v, s in zip(left, MIRROR)]


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--test", choices=["wheels", "arms", "gripper", "hold", "slab"], help="한 가지를 설정별로 재 본다")
    parser.add_argument("--booth", action="store_true", help="강의 38 의 부스 안에서 움직인다")
    parser.add_argument("--keyboard", action="store_true", help="창을 띄우고 W A S D 로 운전한다")
    parser.add_argument("--gui", action="store_true", help="창을 띄운다")
    parser.add_argument("--out", metavar="JSON", help="--test arms 의 시간별 오차를 이 파일에 저장한다")
    args = parser.parse_args()

    from isaacsim import SimulationApp
    app = SimulationApp({"headless": not (args.gui or args.keyboard)})

import omni.graph.core as og
import omni.usd
from isaacsim.core.utils.extensions import enable_extension
from pxr import PhysxSchema, Usd, UsdGeom, UsdPhysics, UsdShade

sys.path.insert(0, REPO)
sys.path.insert(0, os.path.join(REPO, "37_pl2m_import"))
from common.sim import Sim, new_scene, pose_of, set_pose, set_view
from import_pl2m import DEFAULT_URDF, find_urdf, import_pl2m


def joint(stage, name):
    return stage.GetPrimAtPath(f"{JOINTS}/{name}")


def drive(stage, name):
    """관절의 구동(Drive). 손가락은 직선(linear), 나머지는 회전(angular)."""
    return UsdPhysics.DriveAPI(joint(stage, name), "linear" if "finger" in name else "angular")


def joint_position(stage, name):
    """관절의 지금 위치: 회전 관절은 [°], 손가락은 [m] (Play 중에 읽는다)."""
    kind = "linear" if "finger" in name else "angular"
    return joint(stage, name).GetAttribute(f"state:{kind}:physics:position").Get()


def set_friction(stage, links, value):
    """링크의 충돌 모양에 마찰 재질을 묶는다 (Property 의 Physics > Physics materials on selected models)."""
    material = UsdShade.Material.Define(stage, f"/World/PhysicsMaterials/friction_{str(value).replace('.', '_')}")
    api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    api.CreateStaticFrictionAttr(value)
    api.CreateDynamicFrictionAttr(value)
    # 두 물체가 닿을 때 어느 쪽 마찰을 쓸지: 미끄러운 캐스터는 작은 값, 바퀴는 큰 값이 이기게
    PhysxSchema.PhysxMaterialAPI.Apply(material.GetPrim()).CreateFrictionCombineModeAttr("min" if value < 0.1 else "max")
    for link in links:
        collisions = stage.GetPrimAtPath(f"{ROBOT}/{link}/collisions")
        UsdShade.MaterialBindingAPI.Apply(collisions).Bind(material, UsdShade.Tokens.strongerThanDescendants, "physics")


def tune_wheels(stage, friction=True, armature=True, thin=True, motor=True, gain=MOTOR_GAIN):
    """바퀴로 다닐 수 있게 하는 네 가지. 시험할 때는 하나씩 켠다."""
    if friction:
        set_friction(stage, CASTER_LINKS, CASTER_FRICTION)         # 1. 고정된 공 모양 캐스터가 바닥에서 미끄러지게
    if armature:
        for name in WHEEL_JOINTS:                                   # 2. 0.05 kg 뿐인 바퀴에 모터 회전자의 관성을 더함
            PhysxSchema.PhysxJointAPI.Apply(joint(stage, name)).CreateArmatureAttr(WHEEL_ARMATURE)
    if thin:
        for link in WHEEL_LINKS:                                    # 3. 폭 5 cm 원기둥 → 1 cm: 제자리에서 돌 때 바닥을 덜 긁음
            collisions = stage.GetPrimAtPath(f"{ROBOT}/{link}/collisions")
            collisions.SetInstanceable(False)                       # 함께 쓰는 모양(인스턴스)을 풀어야 고칠 수 있다
            cylinder = [prim for prim in Usd.PrimRange(collisions) if prim.GetTypeName() == "Cylinder"][0]
            UsdGeom.Cylinder(cylinder).GetHeightAttr().Set(WHEEL_WIDTH)
    if motor:
        set_friction(stage, WHEEL_LINKS, WHEEL_FRICTION)
        for name in WHEEL_JOINTS:                                   # 4. 모터의 속도 게인과 최대 토크
            drive(stage, name).GetDampingAttr().Set(gain * PER_DEGREE)
            drive(stage, name).GetMaxForceAttr().Set(MOTOR_TORQUE)


def tune_arms(stage, gains=True, gravity_off=True):
    """팔 관절의 구동 값을 원본 값으로 바꾸고, 팔 링크의 중력을 끈다.

    중력 보상(관절이 팔 무게만큼의 토크를 더 냄)이 아니라 중력을 없애는 것이다. 팔이 처지지 않는 것은 같지만,
    팔 무게가 베이스에 실리지 않는 점이 다르다.
    """
    if gains:
        for name in ARM_JOINTS:
            drive(stage, name).GetStiffnessAttr().Set(ARM_KP * PER_DEGREE)
            drive(stage, name).GetDampingAttr().Set(ARM_KV * PER_DEGREE)
    if gravity_off:
        for link in stage.GetPrimAtPath(ROBOT).GetChildren():
            if link.GetName().startswith(("openarm_left", "openarm_right")) and link.HasAPI(UsdPhysics.RigidBodyAPI):
                PhysxSchema.PhysxRigidBodyAPI.Apply(link).CreateDisableGravityAttr(True)


def tune_gripper(stage):
    for name in FINGER_JOINTS:
        drive(stage, name).GetStiffnessAttr().Set(GRIP_KP)          # 직선 관절은 미터 기준이라 그대로 넣는다
        drive(stage, name).GetDampingAttr().Set(GRIP_KV)
        drive(stage, name).GetMaxForceAttr().Set(GRIP_FORCE)


def tune_pl2m(stage):
    """이 강의의 결론: 뒤 강의에서는 가져온 뒤 이 함수 하나를 부른다."""
    tune_wheels(stage)
    tune_arms(stage)
    tune_gripper(stage)


def set_wheel_velocity(stage, linear, angular):
    """직진 속도 v [m/s] 와 회전 속도 ω [rad/s] 를 두 바퀴의 Target Velocity [°/s] 로."""
    left = (linear - angular * WHEEL_DISTANCE / 2) / WHEEL_RADIUS
    right = (linear + angular * WHEEL_DISTANCE / 2) / WHEEL_RADIUS
    for name, value in zip(WHEEL_JOINTS, (left, right)):
        drive(stage, name).GetTargetVelocityAttr().Set(math.degrees(value))


def set_arm_targets(stage, targets):
    for joint_name, target in zip(ARM_JOINTS, targets):
        drive(stage, joint_name).GetTargetPositionAttr().Set(target)


def set_arm_pose(stage, name):
    """팔의 목표를 한 번에 바꾼다 (계단 입력). 팔이 관절의 속도 한계로 움직여 베이스를 찬다. 시험에서만 쓴다."""
    set_arm_targets(stage, pose_degrees(name))


def blend(begin, end, ratio):
    """두 자세 사이의 목표. ratio 0 → 1 을 처음과 끝이 느린 곡선으로 바꿔 섞는다."""
    s = min(max(ratio, 0.0), 1.0)
    s = s * s * (3.0 - 2.0 * s)
    return [a + (b - a) * s for a, b in zip(begin, end)]


def move_arms(stage, sim, name, seconds):
    """팔의 목표를 seconds 에 걸쳐 조금씩 옮긴다 (Play 중에 부른다). 뒤 강의에서는 MoveIt 의 궤적이 이 일을 한다."""
    begin = [drive(stage, joint_name).GetTargetPositionAttr().Get() or 0.0 for joint_name in ARM_JOINTS]
    end, start = pose_degrees(name), sim.time
    while sim.time - start < seconds:
        sim.app.update()
        set_arm_targets(stage, blend(begin, end, (sim.time - start) / seconds))


def set_gripper(stage, opening):
    """두 그리퍼의 손가락을 opening [m] 만큼 벌린다 (0 = 닫힘, GRIP_OPEN = 다 열림)."""
    for name in FINGER_JOINTS:
        drive(stage, name).GetTargetPositionAttr().Set(opening)


def add_keyboard_drive(max_linear=MAX_LINEAR, max_angular=MAX_ANGULAR):
    """강의 11 과 같은 그래프: W − S → 직진, A − D → 회전 → Differential Controller → 바퀴 관절."""
    enable_extension("isaacsim.robot.wheeled_robots")
    keys = og.Controller.Keys
    nodes = [("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
             ("DifferentialController", "isaacsim.robot.wheeled_robots.DifferentialController"),
             ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController")]
    values = [("DifferentialController.inputs:wheelRadius", WHEEL_RADIUS), ("DifferentialController.inputs:wheelDistance", WHEEL_DISTANCE),
              ("DifferentialController.inputs:maxLinearSpeed", max_linear), ("DifferentialController.inputs:maxAngularSpeed", max_angular),
              ("ArticulationController.inputs:targetPrim", ROBOT), ("ArticulationController.inputs:jointNames", WHEEL_JOINTS)]
    connects = [("OnPlaybackTick.outputs:tick", "DifferentialController.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
                ("DifferentialController.outputs:velocityCommand", "ArticulationController.inputs:velocityCommand")]
    for key in "WASD":
        nodes += [(key, "omni.graph.nodes.ReadKeyboardState"), (f"ToDouble{key}", "omni.graph.nodes.ToDouble")]
        values += [(f"{key}.inputs:key", key)]
        connects += [(f"{key}.outputs:isPressed", f"ToDouble{key}.inputs:value")]
    for name, plus, minus, scale, target in (("Linear", "W", "S", max_linear, "linearVelocity"), ("Angular", "A", "D", max_angular, "angularVelocity")):
        nodes += [(f"Sub{name}", "omni.graph.nodes.Subtract"), (f"Scale{name}", "omni.graph.nodes.ConstantDouble"),
                  (f"Speed{name}", "omni.graph.nodes.Multiply")]
        values += [(f"Scale{name}.inputs:value", scale)]
        connects += [(f"ToDouble{plus}.outputs:converted", f"Sub{name}.inputs:a"), (f"ToDouble{minus}.outputs:converted", f"Sub{name}.inputs:b"),
                     (f"Sub{name}.outputs:difference", f"Speed{name}.inputs:a"), (f"Scale{name}.inputs:value", f"Speed{name}.inputs:b"),
                     (f"Speed{name}.outputs:product", f"DifferentialController.inputs:{target}")]
    og.Controller.edit({"graph_path": GRAPH, "evaluator_name": "execution"},
                       {keys.CREATE_NODES: nodes, keys.SET_VALUES: values, keys.CONNECT: connects})


def add_slab(stage, center, size, thickness):
    """윗면이 2 cm 높이인 상자를 바닥으로 깐다 (--test slab). 두께만큼 아래로 내려간다."""
    slab = UsdGeom.Cube.Define(stage, "/World/Slab")
    slab.CreateSizeAttr(1.0)
    set_pose(slab.GetPrim(), translate=(center[0], center[1], SLAB_TOP - thickness / 2.0), scale=(size[0], size[1], thickness))
    UsdPhysics.CollisionAPI.Apply(slab.GetPrim())
    return slab.GetPrim()


def fresh_robot(height=0.05):
    """새 장면에 PL-2M 을 가져온다."""
    stage = new_scene()
    import_pl2m(stage, find_urdf(DEFAULT_URDF), height=height)
    return stage


def measure_motion(stage, sim, linear, angular, seconds):
    """seconds 동안 달려 (앞으로 간 거리 [m], 돈 각도 [°], 1초 뒤의 속도 [m/s] 또는 [°/s]) 를 잰다."""
    body = stage.GetPrimAtPath(BODY)
    set_wheel_velocity(stage, linear, angular)
    x0, y0, _, yaw_before = pose_of(body)
    start, turned, early = sim.time, 0.0, None
    while sim.time - start < seconds:
        sim.app.update()
        x, y, _, yaw = pose_of(body)
        turned += (yaw - yaw_before + 180.0) % 360.0 - 180.0
        yaw_before = yaw
        if early is None and sim.time - start >= 1.0:
            early = (math.hypot(x - x0, y - y0), turned, sim.time)
    distance, span = math.hypot(x - x0, y - y0), sim.time - early[2]
    set_wheel_velocity(stage, 0.0, 0.0)
    return distance, turned, ((distance - early[0]) / span, (turned - early[1]) / span)


def test_wheels():
    """설정을 하나씩 더하며 직진 (0.3 m/s) 과 제자리 회전 (0.7 rad/s) 을 3초씩 잰다."""
    cases = [("가져온 그대로", dict(friction=False, armature=False, thin=False, motor=False)),
             ("1. 캐스터 마찰 0.002", dict(armature=False, thin=False, motor=False)),
             ("2. + 바퀴 관절 관성 0.05", dict(thin=False, motor=False)),
             ("3. + 바퀴 충돌 폭 1 cm", dict(motor=False)),
             ("4. + 모터 값 · 바퀴 마찰 1.2", dict()),             # 모터 값: 속도 게인 100, 최대 토크 18
             ("(4 에서 속도 게인만 원본 10)", dict(gain=10.0))]
    print(f"{'설정':<26} {'직진 3초[m]':>10} {'1초 뒤 속도':>10} {'회전 3초[°]':>10} {'1초 뒤 속도':>10}", flush=True)
    print(f"{'(기대)':<26} {0.3 * 3:>10.2f} {0.30:>10.2f} {math.degrees(0.7) * 3:>10.1f} {math.degrees(0.7):>10.1f}", flush=True)
    for name, options in cases:
        row = []
        for linear, angular in ((0.3, 0.0), (0.0, 0.7)):
            stage = fresh_robot()
            tune_wheels(stage, **options)
            sim = Sim(app)
            sim.play()
            sim.run(1.0)
            distance, turned, speed = measure_motion(stage, sim, linear, angular, 3.0)
            row += [distance, speed[0]] if linear else [turned, speed[1]]
            sim.stop()
        print(f"{name:<26} {row[0]:>10.2f} {row[1]:>10.2f} {row[2]:>10.1f} {row[3]:>10.1f}", flush=True)


def test_arms(out=None):
    """내린 자세에서 carry 자세로 한 번에 바꾸고 2초 동안 본다 (계단 입력)."""
    cases = [("Importer 기본값", dict(gains=False, gravity_off=False)),
             ("원본 값 (kp 500 · kv 35)", dict(gravity_off=False)),
             ("원본 값 + 팔 중력 끔", dict())]
    targets, series = pose_degrees("carry"), {}
    print(f"{'설정':<22} {'0.25초':>7} {'0.5초':>7} {'1초':>7} {'2초':>7}   (팔 관절 14개 중 가장 큰 오차 [°])  베이스가 밀린 거리", flush=True)
    for name, options in cases:
        stage = fresh_robot()
        tune_wheels(stage)
        tune_arms(stage, **options)
        body = stage.GetPrimAtPath(BODY)
        sim = Sim(app)
        sim.play()
        sim.run(1.0)
        x0, y0, _, _ = pose_of(body)
        set_arm_pose(stage, "carry")
        start, rows = sim.time, []
        while sim.time - start < 2.0:
            sim.app.update()
            errors = [abs(joint_position(stage, joint_name) - target) for joint_name, target in zip(ARM_JOINTS, targets)]
            rows.append((round(sim.time - start, 3), round(max(errors), 2)))
        x, y, _, _ = pose_of(body)
        at = lambda t: min(rows, key=lambda row: abs(row[0] - t))[1]
        print(f"{name:<22} {at(0.25):>7.1f} {at(0.5):>7.1f} {at(1.0):>7.1f} {at(2.0):>7.1f}   {math.hypot(x - x0, y - y0) * 1000:27.1f} mm", flush=True)
        series[name] = rows
        sim.stop()
    if out:
        with open(out, "w") as file:
            json.dump(series, file, ensure_ascii=False)
        print("저장:", out, flush=True)


def test_gripper():
    """왼손 그리퍼를 열고(44 mm) 닫고(0) 반쯤 열며(20 mm), 두 손가락의 위치를 0.25초마다 본다."""
    names = ("openarm_left_finger_joint1", "openarm_left_finger_joint2")
    for case, tuned in (("Importer 기본값", False), ("원본 값 (kp 1800 · kv 15 · 30 N)", True)):
        stage = fresh_robot()
        if tuned:
            tune_gripper(stage)
        sim = Sim(app)
        sim.play()
        sim.run(1.0)
        print(f"{case}", flush=True)
        print(f"  {'목표[mm]':>8}   0.25초마다 손가락 1 / 손가락 2 의 위치 [mm]", flush=True)
        for target in (GRIP_OPEN, 0.0, 0.02):
            drive(stage, names[0]).GetTargetPositionAttr().Set(target)
            cells = []
            for _ in range(4):
                sim.run(0.25)
                cells.append(" / ".join(f"{joint_position(stage, name) * 1000:5.1f}" for name in names))
            print(f"  {target * 1000:>8.0f}   " + "   ".join(cells), flush=True)
        sim.stop()


def test_hold():
    """1 m 직진 → 90° 회전 → 멈추고 팔을 운반 자세로.

    팔을 한 번에 움직일 때와 1초에 걸쳐 움직일 때, 모터 게인 10(원본)과 100 일 때 로봇이 밀린 거리를 잰다.
    """
    print(f"{'모터 게인':<8} {'팔 움직임':<8} {'직진[m]':>7} {'회전[°]':>7} {'어깨 최고 속도[°/s]':>16} {'0~1.5초에 밀림[mm]':>16} {'그 뒤 4초[mm]':>12}", flush=True)
    for gain, seconds in ((10.0, 0.0), (MOTOR_GAIN, 0.0), (10.0, 1.0), (MOTOR_GAIN, 1.0)):
        stage = fresh_robot()
        tune_pl2m(stage)
        for name in WHEEL_JOINTS:
            drive(stage, name).GetDampingAttr().Set(gain * PER_DEGREE)
        body = stage.GetPrimAtPath(BODY)
        sim = Sim(app)
        sim.play()
        sim.run(1.0)
        move_arms(stage, sim, "transport", 1.5)
        sim.run(0.5)
        x0, y0, _, yaw0 = pose_of(body)
        set_wheel_velocity(stage, 0.25, 0.0)
        sim.run(4.0)
        x1, y1, _, _ = pose_of(body)
        set_wheel_velocity(stage, 0.0, 0.5)
        sim.run(math.pi)
        yaw1 = pose_of(body)[3]
        set_wheel_velocity(stage, 0.0, 0.0)
        begin, end = pose_degrees("transport"), pose_degrees("carry")
        shoulder, fastest, during = "openarm_left_joint2", 0.0, None      # 가장 많이 도는 관절 (136°)
        xa, ya, _, _ = pose_of(body)
        start, angle, clock = sim.time, joint_position(stage, shoulder), sim.time
        while sim.time - start < 5.5:
            set_arm_targets(stage, blend(begin, end, (sim.time - start) / seconds if seconds else 1.0))
            sim.app.update()
            now = joint_position(stage, shoulder)
            if sim.time > clock:
                fastest = max(fastest, abs(now - angle) / (sim.time - clock))
            angle, clock = now, sim.time
            if during is None and sim.time - start >= 1.5:
                during = pose_of(body)
        xb, yb, _, _ = pose_of(body)
        print(f"{gain:<10g} {'한 번에' if not seconds else f'{seconds:g}초에 걸쳐':<10} {math.hypot(x1 - x0, y1 - y0):>7.3f} {yaw1 - yaw0:>7.1f} {fastest:>16.0f} "
              f"{math.hypot(during[0] - xa, during[1] - ya) * 1000:>16.1f} {math.hypot(xb - during[0], yb - during[1]) * 1000:>12.1f}", flush=True)
        sim.stop()


def test_slab():
    """참고: 바닥이 평면일 때와 얇은 상자일 때 0.1 m/s 로 6초 직진한다 (기대 0.6 m).

    상자는 9 × 6 m 이고 윗면의 높이는 2 cm 다. 두께만 바꾼다. 서는 자리 둘은 상자의 가운데 가까이와 가장자리 쪽이다.
    """
    spots = {"가운데 쪽": (5.3, -3.2, 0.0), "가장자리 쪽": (7.65, -2.82, 90.0)}
    print(f"{'바닥':<14} {'선 자리':<10} {'간 거리[m]':>9}", flush=True)
    for thickness in (None, 0.02, 0.05, 0.10, 1.0):
        for spot, (x, y, yaw) in spots.items():
            top = SLAB_TOP if thickness else 0.0
            stage = fresh_robot(height=top + 0.05)
            if thickness:
                add_slab(stage, (4.5, -3.0), (9.0, 6.0), thickness)
            set_pose(stage.GetPrimAtPath(ROBOT), translate=(x, y, top + 0.05), rotate_xyz=(0.0, 0.0, yaw))
            tune_pl2m(stage)
            body = stage.GetPrimAtPath(BODY)
            sim = Sim(app)
            sim.play()
            sim.run(1.0)
            move_arms(stage, sim, "transport", 1.5)
            sim.run(0.5)
            x0, y0, _, _ = pose_of(body)
            set_wheel_velocity(stage, 0.1, 0.0)
            sim.run(6.0)
            x1, y1, _, _ = pose_of(body)
            print(f"{'평면' if not thickness else f'상자 {thickness * 100:g} cm':<14} {spot:<10} {math.hypot(x1 - x0, y1 - y0):>9.3f}", flush=True)
            sim.stop()


def demo(stage, sim):
    """다 맞춘 로봇: 팔을 주행 자세로 → 1 m 직진 → 왼쪽으로 90° → 팔을 운반 자세로 → 그리퍼 열고 닫기."""
    body = stage.GetPrimAtPath(BODY)
    steps = [("팔: 주행 자세", lambda: move_arms(stage, sim, "transport", 1.5), 0.5),
             ("직진 0.25 m/s", lambda: set_wheel_velocity(stage, 0.25, 0.0), 4.0),
             ("왼쪽으로 회전 0.5 rad/s", lambda: set_wheel_velocity(stage, 0.0, 0.5), math.pi),
             ("멈추고 팔: 운반 자세", lambda: (set_wheel_velocity(stage, 0.0, 0.0), move_arms(stage, sim, "carry", 2.0)), 0.5),
             ("그리퍼 열기", lambda: set_gripper(stage, GRIP_OPEN), 1.0),
             ("그리퍼 닫기", lambda: set_gripper(stage, 0.0), 1.0)]
    x0, y0, _, yaw0 = pose_of(body)
    print(f"{'동작':<22} {'x[m]':>7} {'y[m]':>7} {'방향[°]':>7} {'왼팔 팔꿈치[°]':>12} {'왼손 손가락[mm]':>12}", flush=True)
    for name, action, seconds in steps:
        action()
        sim.run(seconds)
        x, y, _, yaw = pose_of(body)
        print(f"{name:<22} {x - x0:>7.3f} {y - y0:>7.3f} {round(yaw - yaw0, 1) + 0.0:>7.1f} {joint_position(stage, 'openarm_left_joint4'):>12.1f} "
              f"{joint_position(stage, 'openarm_left_finger_joint1') * 1000:>12.1f}", flush=True)


if __name__ == "__main__":
    if args.test:
        {"wheels": test_wheels, "gripper": test_gripper, "hold": test_hold, "slab": test_slab}.get(args.test, lambda: test_arms(args.out))()
        app.close()
        sys.exit(0)

    stage = new_scene()
    start = (0.0, 0.0, 0.05)
    if args.booth:                                # 강의 38 의 부스 안, 상담 테이블 사이에서 시작
        sys.path.insert(0, os.path.join(REPO, "38_pl2m_booth"))
        import make_booth
        mjcf = make_booth.read_mjcf(make_booth.build_mjcf())
        make_booth.add_booth(stage, mjcf)
        make_booth.add_lights(stage, mjcf)
        make_booth.add_cups(stage)
        start = (make_booth.SPAWN[0], make_booth.SPAWN[1], 0.05)
    robot = import_pl2m(stage, find_urdf(DEFAULT_URDF))
    set_pose(robot, translate=start)
    tune_pl2m(stage)
    if args.keyboard:
        add_keyboard_drive()
    if args.gui or args.keyboard:
        set_view(eye=(start[0] - 2.4, start[1] - 2.6, 2.2), target=(start[0] + 0.4, start[1], 0.6))
    sim = Sim(app)
    sim.play()
    sim.run(1.0)
    if args.keyboard:
        move_arms(stage, sim, "transport", 1.5)
        print("뷰포트를 한 번 클릭한 뒤 W(앞) S(뒤) A(왼쪽) D(오른쪽) 로 운전합니다. 창을 닫으면 끝납니다.", flush=True)
        sim.wait_until_closed()
    else:
        demo(stage, sim)
        if args.gui:
            sim.wait_until_closed()
    sim.stop()
    app.close()
