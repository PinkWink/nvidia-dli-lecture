"""강의 40: 팔의 중력. 중력이 팔과 베이스에 주는 영향을 재고, 관절 토크로 중력을 보상합니다.

    ~/isaacsim/python.sh 40_pl2m_gravity/pl2m_gravity.py --test read       # 자세마다 "중력을 버티는 데 드는 힘" 읽기
    ~/isaacsim/python.sh 40_pl2m_gravity/pl2m_gravity.py --test formula    # 같은 값을 수식으로 계산해 맞춰 보기
    ~/isaacsim/python.sh 40_pl2m_gravity/pl2m_gravity.py --test compare    # 왼팔에만 보상을 넣고 오른팔과 비교 (위치 · 토크)
    ~/isaacsim/python.sh 40_pl2m_gravity/pl2m_gravity.py --test sag        # 중력 처리 네 가지로 자세별 처짐
    ~/isaacsim/python.sh 40_pl2m_gravity/pl2m_gravity.py --test load       # 바퀴와 캐스터가 바닥에서 받는 힘
    ~/isaacsim/python.sh 40_pl2m_gravity/pl2m_gravity.py --test wrong      # 흔한 실수 둘: 6칸 밀림, 중력을 끈 채 보상
    ~/isaacsim/python.sh 40_pl2m_gravity/pl2m_gravity.py --test solver     # 구동만일 때의 처짐과 솔버 반복 횟수
    ~/isaacsim/python.sh 40_pl2m_gravity/pl2m_gravity.py --test motion     # 팔이 움직이는 동안의 오차
    ~/isaacsim/python.sh 40_pl2m_gravity/pl2m_gravity.py                   # 약한 팔(kp 50)로, 왼팔에만 보상을 넣은 시연
    ~/isaacsim/python.sh 40_pl2m_gravity/pl2m_gravity.py --gui             # 같은 시연을 창으로

강의 39 는 팔의 동작만 보려고 팔 링크의 중력을 꺼 두었습니다. 여기서는 중력을 다시 켜고,
원본(MuJoCo 설정의 bias_compensation)처럼 관절이 팔 무게만큼의 토크를 더 내게 합니다.
뒤 강의에서는 가져온 뒤 prepare_pl2m(stage) 를 부르고, Play 뒤에 GravityCompensation() 을 만듭니다.
"""
import argparse
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, ".."))
ROOT = "/World/pl2m/base_footprint"                  # 관절체(Articulation)의 뿌리
HANDS = {"left": "/World/pl2m/openarm_left_hand", "right": "/World/pl2m/openarm_right_hand"}
GRAVITY = 9.81                                       # 중력 가속도 [m/s²] (장면의 값)
GROUND_LINKS = ["left_wheel", "right_wheel", "front_left_caster", "front_right_caster", "rear_left_caster", "rear_right_caster"]
REACH = {  # 중력의 영향이 큰 자세 [rad] (왼팔. 오른팔은 강의 39 의 MIRROR 로 뒤집는다)
    "forward": [-1.5708, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],        # 두 팔을 앞으로 곧게
    "backward": [1.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],           # 뒤로
    "side": [0.0, -1.5708, 0.0, 0.0, 0.0, 0.0, 0.0],           # 옆으로
}
POSE_LABELS = {"down": "내림", "transport": "주행", "carry": "운반", "forward": "앞으로 뻗기", "backward": "뒤로 뻗기", "side": "옆으로 뻗기"}
STIFF_KP, STIFF_KV = 2000.0, 70.0                    # "게인만 올리기" 에 쓰는 값: kp 4배, kv 2배
SOFT_KP, SOFT_KV = 50.0, 11.0                        # 시연에 쓰는 약한 팔: kp 1/10


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--test", choices=["read", "formula", "compare", "sag", "load", "wrong", "solver", "motion"], help="한 가지를 재 본다")
    parser.add_argument("--gui", action="store_true", help="창을 띄운다")
    parser.add_argument("--out", metavar="JSON", help="--test compare · sag · load 의 결과를 이 파일에 저장한다")
    args = parser.parse_args()

    from isaacsim import SimulationApp
    app = SimulationApp({"headless": not args.gui})

import numpy as np
import omni.physics.tensors
import omni.physx
from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics

sys.path.insert(0, REPO)
sys.path.insert(0, os.path.join(REPO, "39_pl2m_drive"))
from common.sim import Sim, set_view
from pl2m_drive import (ARM_JOINTS, ARM_KP, BODY, JOINTS, MIRROR, PER_DEGREE, POSES, ROBOT, blend, drive, fresh_robot, joint_position, move_arms,
                        pose_degrees, set_arm_targets, tune_arms, tune_gripper, tune_wheels)

POSES.update(REACH)
LEFT_ARM, RIGHT_ARM = ARM_JOINTS[:7], ARM_JOINTS[7:]


def prepare_pl2m(stage, arm_gravity=True):
    """강의 39 의 tune_pl2m 과 같되 팔 링크의 중력을 켜 둔다. 뒤 강의에서는 가져온 뒤 이 함수를 부른다."""
    tune_wheels(stage)
    tune_arms(stage, gravity_off=not arm_gravity)
    tune_gripper(stage)


def set_arm_gains(stage, kp, kv):
    for name in ARM_JOINTS:
        drive(stage, name).GetStiffnessAttr().Set(kp * PER_DEGREE)
        drive(stage, name).GetDampingAttr().Set(kv * PER_DEGREE)


class GravityCompensation:
    """물리 스텝마다, 지금 자세에서 중력을 버티는 데 드는 힘을 팔 관절에 더한다. Play 뒤에 만든다.

    위치 구동(kp · kv)은 그대로 일하고, 이 힘은 거기에 더해진다. enabled 를 False 로 하면 0 을 넣는다.
    """

    def __init__(self, joints=ARM_JOINTS, coriolis=False, shift=None):
        self.physics = omni.physics.tensors.create_simulation_view("numpy")
        self.robot = self.physics.create_articulation_view(ROOT)
        self.names = list(self.robot.shared_metatype.dof_names)
        self.dofs = np.array([self.names.index(name) for name in joints], dtype=int)
        # 바닥에 고정되지 않은 로봇은 앞 6칸이 뿌리(힘 3 + 모멘트 3)이고, 관절은 그 뒤에 온다
        self.shift = (0 if self.robot.shared_metatype.fixed_base else 6) if shift is None else shift
        self.enabled, self.coriolis = True, coriolis
        self._every = np.arange(self.robot.count)
        self._sub = omni.physx.get_physx_interface().subscribe_physics_step_events(self._on_step)

    def raw(self):
        """PhysX 가 계산한 값 그대로: 뿌리 6칸 + 관절 (바닥에 고정된 로봇은 관절만)."""
        return self.robot.get_gravity_compensation_forces()[0].copy()

    def forces(self):
        """팔 관절마다 중력을 버티는 데 드는 힘 [N·m] (joints 순서)."""
        return self.raw()[self.shift + self.dofs]

    def state(self, names):
        """관절들의 (목표 [°], 각도 [°], 관절이 받치고 있는 전체 토크 [N·m], 그 가운데 보상으로 넣은 토크 [N·m])."""
        index = [self.names.index(name) for name in names]
        return (np.degrees(self.robot.get_dof_position_targets()[0][index]), np.degrees(self.robot.get_dof_positions()[0][index]),
                self.robot.get_dof_projected_joint_forces()[0][index].copy(), self.robot.get_dof_actuation_forces()[0][index].copy())

    def _on_step(self, dt):
        efforts = np.zeros((self.robot.count, self.robot.max_dofs), dtype=np.float32)
        if self.enabled:
            needed = self.robot.get_gravity_compensation_forces()
            if self.coriolis:                                     # 움직일 때 생기는 힘(코리올리 · 원심력)까지
                needed = needed + self.robot.get_coriolis_and_centrifugal_compensation_forces()
            efforts[:, self.dofs] = needed[:, self.shift + self.dofs]
            self.add_extra(efforts)
        self.robot.set_dof_actuation_forces(efforts, self._every)

    def add_extra(self, efforts):
        """보상에 더 넣을 힘이 있으면 efforts 에 더한다 (강의 43: 손에 든 물체의 무게)."""

    def stop(self):
        self._sub = None
        self.robot.set_dof_actuation_forces(np.zeros((self.robot.count, self.robot.max_dofs), dtype=np.float32), self._every)


class GroundScale:
    """바퀴 2개와 캐스터 4개가 바닥에서 받는 힘을 읽는 저울. Play 전에 prepare, Play 뒤에 만든다."""

    @staticmethod
    def prepare(stage):
        for link in GROUND_LINKS:                                 # 접촉을 보고하게 한다
            PhysxSchema.PhysxContactReportAPI.Apply(stage.GetPrimAtPath(f"{ROBOT}/{link}")).CreateThresholdAttr(0.0)
        # 가만히 서 있는 로봇은 곧 잠들고(sleep), 잠든 동안에는 접촉 힘이 0 으로 읽힌다. 잠들지 않게 한다
        PhysxSchema.PhysxArticulationAPI.Apply(stage.GetPrimAtPath(ROOT)).CreateSleepThresholdAttr(0.0)

    def __init__(self):
        self.physics = omni.physics.tensors.create_simulation_view("numpy")
        self.contacts = self.physics.create_rigid_contact_view([f"{ROBOT}/{link}" for link in GROUND_LINKS], filter_patterns=[[] for _ in GROUND_LINKS])

    def read(self, sim, seconds=0.5):
        """seconds 동안의 평균: 링크마다 위쪽 힘 [N] (GROUND_LINKS 순서)."""
        samples = []
        sub = omni.physx.get_physx_interface().subscribe_physics_step_events(
            lambda dt: samples.append(self.contacts.get_net_contact_forces(dt)[:, 2].copy()))
        sim.run(seconds)
        sub = None
        return np.mean(samples, axis=0)


def arm_errors(stage, pose):
    """팔 관절 14개의 (지금 각도 − 목표) [°]."""
    return np.array([joint_position(stage, name) - target for name, target in zip(ARM_JOINTS, pose_degrees(pose))])


def hand_position(stage, side="left"):
    """손의 위치를 베이스 기준으로 [m] (x 앞, y 왼쪽, z 위)."""
    base = UsdGeom.Xformable(stage.GetPrimAtPath(BODY)).ComputeLocalToWorldTransform(0)
    hand = UsdGeom.Xformable(stage.GetPrimAtPath(HANDS[side])).ComputeLocalToWorldTransform(0)
    return np.array(base.GetInverse().Transform(hand.ExtractTranslation()))


def world(stage, link):
    return UsdGeom.Xformable(stage.GetPrimAtPath(f"{ROBOT}/{link}")).ComputeLocalToWorldTransform(0)


def gravity_torque(stage, compensation, joint_name):
    """수식으로 계산: 관절 뒤에 달린 링크들의 무게가 관절 축 둘레에 만드는 토크를 버티는 데 드는 힘 [N·m].

    τ = Σ mᵢ · g · ( a · ( (cᵢ − p) × ẑ ) )     a 관절 축, p 축 위의 한 점, cᵢ 링크의 무게중심, ẑ 위쪽
    돌려주는 값: (합, [(링크, 질량, 축에서 본 수평 거리, 토크), ...])
    """
    names = list(compensation.robot.shared_metatype.link_names)
    parents = list(compensation.robot.shared_metatype.link_parents)
    masses, centers = compensation.robot.get_masses()[0], compensation.robot.get_coms()[0]
    joint = UsdPhysics.RevoluteJoint(stage.GetPrimAtPath(f"{JOINTS}/{joint_name}"))
    child = joint.GetBody1Rel().GetTargets()[0].name
    frame = world(stage, child)                                   # 관절 축은 뒤쪽 링크(body1)의 좌표로 적혀 있다
    point = frame.Transform(Gf.Vec3d(*joint.GetLocalPos1Attr().Get()))
    local = Gf.Rotation(Gf.Quatd(joint.GetLocalRot1Attr().Get())).TransformDir(Gf.Vec3d(*[float(joint.GetAxisAttr().Get() == a) for a in "XYZ"]))
    axis = frame.TransformDir(local).GetNormalized()
    after = [child]                                               # 이 관절 뒤에 달린 링크 모두
    for name, parent in zip(names, parents):                      # 링크 이름은 뿌리에서 가까운 순서로 온다
        if parent in after and name not in after:
            after.append(name)
    rows, up = [], Gf.Vec3d(0, 0, 1)
    for name in after:
        index = names.index(name)
        center = world(stage, name).Transform(Gf.Vec3d(*[float(v) for v in centers[index][:3]]))
        arm = Gf.Dot(axis, Gf.Cross(center - point, up))          # 축에서 본 수평 거리 (부호 있음)
        rows.append((name, float(masses[index]), arm, float(masses[index]) * GRAVITY * arm))
    return sum(row[3] for row in rows), rows


def base_pitch(stage):
    """베이스가 앞뒤로 기운 각도 [°] (앞이 들리면 +)."""
    matrix = UsdGeom.Xformable(stage.GetPrimAtPath(BODY)).ComputeLocalToWorldTransform(0)
    return math.degrees(math.asin(max(-1.0, min(1.0, matrix[0][2]))))


def start(arm_gravity=True, gains=None, compensate=False, scale=False, iterations=None, **options):
    """새 장면에 로봇을 가져와 Play 한다. (stage, sim, 보상, 저울) 을 돌려준다."""
    stage = fresh_robot()
    prepare_pl2m(stage, arm_gravity=arm_gravity)
    if gains:
        set_arm_gains(stage, *gains)
    if iterations:                                                # 관절체의 솔버 반복 횟수 (가져온 로봇의 값은 32)
        PhysxSchema.PhysxArticulationAPI.Apply(stage.GetPrimAtPath(ROOT)).CreateSolverPositionIterationCountAttr(iterations)
    if scale:
        GroundScale.prepare(stage)
    if args.gui:
        set_view(eye=(2.3, -1.7, 1.25), target=(0.15, 0.0, 0.72))     # 앞 옆에서: 두 팔의 높이가 비교되게
    sim = Sim(app)
    sim.play()
    sim.run(1.0)
    compensation = GravityCompensation(**options) if compensate else None
    return stage, sim, compensation, GroundScale() if scale else None


def finish(sim, compensation):
    if compensation:
        compensation.stop()
    sim.stop()


def test_formula():
    """PhysX 가 계산한 값을, 링크의 질량 · 무게중심 · 관절 축으로 직접 계산한 값과 맞춰 본다."""
    stage, sim, compensation, _ = start(arm_gravity=False, compensate=True)
    compensation.enabled = False
    move_arms(stage, sim, "forward", 1.5)
    sim.run(1.0)
    total, rows = gravity_torque(stage, compensation, LEFT_ARM[0])
    print("팔을 앞으로 뻗은 자세, 왼팔 1번 관절", flush=True)
    print(f"{'링크':<26} {'질량[kg]':>8} {'축에서 수평 거리[m]':>16} {'m·g·거리 [N·m]':>14}", flush=True)
    for name, mass, arm, torque in rows:
        print(f"{name:<26} {mass:>8.3f} {arm:>16.3f} {torque:>14.2f}", flush=True)
    print(f"{'합':<26} {sum(row[1] for row in rows):>8.3f} {'':>16} {total:>14.2f}     PhysX 의 값 {compensation.forces()[0]:.2f}", flush=True)
    for pose in ("forward", "side", "carry"):
        move_arms(stage, sim, pose, 1.5)
        sim.run(1.0)
        physx = compensation.forces()[:7]
        mine = [gravity_torque(stage, compensation, name)[0] for name in LEFT_ARM]
        print(f"\n{POSE_LABELS[pose]}: 왼팔 관절 1 ~ 7 [N·m]", flush=True)
        print("  수식    " + " ".join(f"{value:7.2f}" for value in mine), flush=True)
        print("  PhysX   " + " ".join(f"{value:7.2f}" for value in physx), flush=True)
    finish(sim, compensation)


def test_compare(out=None):
    """왼팔에만 중력 보상을 넣고 두 팔을 같은 자세로 움직인다. 오른팔은 구동만으로 버틴다. 위치와 토크를 스텝마다 적는다."""
    stage, sim, compensation, _ = start(compensate=True, joints=LEFT_ARM)
    mirror, log, clock = np.array(MIRROR, dtype=float), [], sim.time

    def record(dt):                                               # 오른팔의 값은 부호를 왼팔에 맞춘다
        left, right = compensation.state(LEFT_ARM), [value * mirror for value in compensation.state(RIGHT_ARM)]
        log.append((sim.time - clock, left, right))
    sub = omni.physx.get_physx_interface().subscribe_physics_step_events(record)
    print(f"{'':<10} {'':<6} {'왼팔 (구동 + 보상)':<30} {'오른팔 (구동만)':<22} {'오른손이':>8}", flush=True)
    print(f"{'자세':<8} {'관절':>4} {'오차[°]':>8} {'구동[N·m]':>9} {'보상[N·m]':>9} {'오차[°]':>10} {'구동[N·m]':>9} {'더 낮음[mm]':>12}", flush=True)
    marks = []
    for pose, hold in (("forward", 2.5), ("side", 2.5), ("carry", 2.5), ("down", 1.5)):
        marks.append((sim.time - clock, POSE_LABELS[pose]))
        move_arms(stage, sim, pose, 1.5)
        sim.run(hold)
        _, left, right = log[-1]
        k = int(np.argmax(np.abs(right[2])))                      # 이 자세에서 힘이 가장 많이 드는 관절
        lower = (hand_position(stage, "left")[2] - hand_position(stage, "right")[2]) * 1000
        print(f"{POSE_LABELS[pose]:<8} {k + 1:>4} {left[1][k] - left[0][k]:>8.2f} {left[2][k] - left[3][k]:>9.2f} {left[3][k]:>9.2f} "
              f"{right[1][k] - right[0][k]:>10.2f} {right[2][k] - right[3][k]:>9.2f} {lower:>12.1f}", flush=True)
    sub = None
    finish(sim, compensation)
    if out:
        pack = lambda side: {key: [[round(float(v), 4) for v in row[side][i]] for row in log] for i, key in enumerate(("target", "angle", "total", "added"))}
        with open(out, "w") as file:
            json.dump({"time": [round(row[0], 4) for row in log], "marks": marks, "left": pack(1), "right": pack(2)}, file, ensure_ascii=False)
        print("저장:", out, flush=True)


def test_read():
    """자세마다 PhysX 가 계산한 '중력을 버티는 데 드는 힘' 을 읽는다. 팔은 중력을 끈 채로 둔다 (값은 자세만으로 정해진다)."""
    stage, sim, compensation, _ = start(arm_gravity=False, compensate=True)
    compensation.enabled = False
    raw = compensation.raw()
    print(f"값의 개수 {len(raw)} = 뿌리 6 + 관절 {len(compensation.names)}", flush=True)
    print(f"{'자세':<8} {'뿌리 위쪽 힘[N]':>12} {'뿌리 앞뒤 모멘트[N·m]':>18}   왼팔 관절 1 ~ 7 [N·m]", flush=True)
    for pose in POSE_LABELS:
        move_arms(stage, sim, pose, 1.5)
        sim.run(1.0)
        raw, left = compensation.raw(), compensation.forces()[:7]
        print(f"{POSE_LABELS[pose]:<8} {raw[2]:>12.1f} {raw[4]:>18.2f}   " + " ".join(f"{value:7.2f}" for value in left), flush=True)
    finish(sim, compensation)


MODES = [("중력 끔 (강의 39)", dict(arm_gravity=False)),
         ("중력 켬, 구동만", dict()),
         ("중력 켬, kp 4배", dict(gains=(STIFF_KP, STIFF_KV))),
         ("중력 켬 + 보상", dict(compensate=True))]


def test_sag(out=None):
    """중력 처리 네 가지로 자세 여섯 개를 잡고, 관절 오차와 손이 내려간 거리를 잰다."""
    table, reference = {}, {}
    for mode, options in MODES:
        stage, sim, compensation, _ = start(**options)
        for pose in POSE_LABELS:
            move_arms(stage, sim, pose, 1.5)
            sim.run(2.0)
            error, hand = float(np.max(np.abs(arm_errors(stage, pose)))), hand_position(stage)
            reference.setdefault(pose, hand)                      # 첫 줄(중력 끔)의 손 위치가 기준
            table[(mode, pose)] = (error, float(reference[pose][2] - hand[2]) * 1000)
        finish(sim, compensation)
    for title, column, form in (("관절 오차가 가장 큰 것 [°]", 0, "{:10.2f}"), ("손이 내려간 거리 [mm] (중력을 끈 경우와의 차이)", 1, "{:10.1f}")):
        print(f"\n{title}", flush=True)
        print(f"{'':<16}" + "".join(f"{POSE_LABELS[pose]:>9}" for pose in POSE_LABELS), flush=True)
        for mode, _ in MODES:
            print(f"{mode:<16}" + "".join(form.format(table[(mode, pose)][column]) for pose in POSE_LABELS), flush=True)
    if out:
        with open(out, "w") as file:
            json.dump({mode: {pose: table[(mode, pose)] for pose in POSE_LABELS} for mode, _ in MODES}, file, ensure_ascii=False)
        print("저장:", out, flush=True)


def test_load(out=None):
    """팔 중력을 끈 경우와 켜고 보상한 경우에, 자세마다 바닥이 받치는 힘을 잰다."""
    result = {}
    for mode, options in (MODES[0], MODES[3]):
        stage, sim, compensation, scale = start(scale=True, **options)
        print(f"\n{mode}", flush=True)
        print(f"{'자세':<8} {'합[N]':>8} {'→ 무게[kg]':>9} {'바퀴 둘':>8} {'앞 캐스터 둘':>10} {'뒤 캐스터 둘':>10} {'기운 각도[°]':>10}", flush=True)
        for pose in POSE_LABELS:
            move_arms(stage, sim, pose, 1.5)
            sim.run(2.0)
            force = scale.read(sim)
            row = (force.sum(), force[:2].sum(), force[2:4].sum(), force[4:].sum(), base_pitch(stage))
            result.setdefault(mode, {})[pose] = [float(value) for value in row]
            print(f"{POSE_LABELS[pose]:<8} {row[0]:>8.1f} {row[0] / 9.81:>9.2f} {row[1]:>8.1f} {row[2]:>10.1f} {row[3]:>10.1f} {row[4]:>10.3f}", flush=True)
        finish(sim, compensation)
    if out:
        with open(out, "w") as file:
            json.dump(result, file, ensure_ascii=False)
        print("저장:", out, flush=True)


def test_wrong():
    """보상을 넣을 때 하기 쉬운 실수 둘. 팔을 앞으로 뻗은 자세에서 관절 오차를 본다."""
    cases = [("바르게: 중력 켬 + 보상", dict(compensate=True)),
             ("실수 1: 뿌리 6칸을 건너뛰지 않음", dict(compensate=True, shift=0)),
             ("실수 2: 중력을 끈 채 보상", dict(arm_gravity=False, compensate=True))]
    print(f"{'경우':<24} {'가장 큰 관절 오차[°]':>16} {'왼팔 1번 관절 오차[°]':>18} {'손 높이[m]':>10}", flush=True)
    for name, options in cases:
        stage, sim, compensation, _ = start(**options)
        move_arms(stage, sim, "forward", 1.5)
        sim.run(2.0)
        errors = arm_errors(stage, "forward")
        print(f"{name:<24} {np.max(np.abs(errors)):>16.2f} {errors[0]:>18.2f} {hand_position(stage)[2]:>10.3f}", flush=True)
        finish(sim, compensation)


def test_solver():
    """팔을 앞으로 뻗고, 솔버 반복 횟수를 바꿔 가며 왼팔 1번 관절의 처짐을 잰다. 어림값은 (중력을 버티는 힘) / kp."""
    print(f"{'경우':<14} {'반복 횟수':>6} {'왼팔 1번 관절 오차[°]':>18} {'버티는 힘 / kp [°]':>16}", flush=True)
    for name, options in (("중력 켬, 구동만", dict()), ("중력 켬 + 보상", dict(compensate=True))):
        for iterations in (4, 32, 64, 255):
            stage, sim, compensation, _ = start(iterations=iterations, **options)
            reader = compensation or GravityCompensation()
            if not compensation:
                reader.stop()                                     # 값을 읽는 데만 쓴다
            move_arms(stage, sim, "forward", 1.5)
            sim.run(3.0)
            guess = math.degrees(abs(reader.forces()[0]) / ARM_KP) if not compensation else 0.0
            print(f"{name:<14} {iterations:>6} {arm_errors(stage, 'forward')[0]:>18.2f} {guess:>16.2f}", flush=True)
            finish(sim, compensation)


def test_motion():
    """내린 자세에서 앞으로 뻗기까지 1.5초에 옮기는 동안, 목표와 가장 크게 벌어진 각도를 잰다."""
    cases = [("중력 끔 (강의 39)", dict(arm_gravity=False)),
             ("중력 켬, 구동만", dict()),
             ("중력 켬 + 보상", dict(compensate=True)),
             ("중력 켬 + 보상(움직임 포함)", dict(compensate=True, coriolis=True))]
    print(f"{'경우':<20} {'움직이는 동안 최대[°]':>16} {'멈춘 뒤 2초[°]':>14}", flush=True)
    for name, options in cases:
        stage, sim, compensation, _ = start(**options)
        begin, end, worst, clock = pose_degrees("down"), pose_degrees("forward"), 0.0, sim.time
        while sim.time - clock < 1.5:
            sim.app.update()
            targets = blend(begin, end, (sim.time - clock) / 1.5)
            set_arm_targets(stage, targets)
            now = [joint_position(stage, joint_name) for joint_name in ARM_JOINTS]
            worst = max(worst, max(abs(a - b) for a, b in zip(now, targets)))
        sim.run(2.0)
        print(f"{name:<20} {worst:>16.2f} {np.max(np.abs(arm_errors(stage, 'forward'))):>14.2f}", flush=True)
        finish(sim, compensation)


def demo():
    """약한 팔(kp 50)로, 왼팔에만 중력 보상을 넣고 두 팔을 같은 자세로 움직인다. 게인이 약할수록 차이가 잘 보인다."""
    stage, sim, compensation, _ = start(gains=(SOFT_KP, SOFT_KV), compensate=True, joints=LEFT_ARM)
    print(f"{'자세':<8} {'왼팔(보상) 오차[°]':>16} {'오른팔(구동만) 오차[°]':>18} {'왼손 높이[m]':>11} {'오른손 높이[m]':>12}", flush=True)
    for pose in ("forward", "side", "forward", "down"):
        move_arms(stage, sim, pose, 2.0)
        sim.run(2.5)
        errors = np.abs(arm_errors(stage, pose))
        print(f"{POSE_LABELS[pose]:<8} {errors[:7].max():>16.2f} {errors[7:].max():>18.2f} {hand_position(stage, 'left')[2]:>11.3f} {hand_position(stage, 'right')[2]:>12.3f}", flush=True)
    if args.gui:
        sim.wait_until_closed()
    finish(sim, compensation)


if __name__ == "__main__":
    tests = {"read": test_read, "formula": test_formula, "compare": lambda: test_compare(args.out), "sag": lambda: test_sag(args.out), "load": lambda: test_load(args.out), "wrong": test_wrong, "solver": test_solver, "motion": test_motion}
    tests.get(args.test, demo)()
    app.close()
