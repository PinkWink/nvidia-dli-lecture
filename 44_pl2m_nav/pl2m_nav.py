"""강의 44: 부스 안의 PL-2M 에 라이다를 달고, Nav2 가 쓸 토픽을 냅니다.

    source /opt/ros/jazzy/setup.bash                               # Isaac Sim 보다 먼저
    ~/isaacsim/python.sh 44_pl2m_nav/pl2m_nav.py                   # 부스 + PL-2M + 라이다
    ~/isaacsim/python.sh 44_pl2m_nav/pl2m_nav.py --gui             # 창으로 보기
    ~/isaacsim/python.sh 44_pl2m_nav/pl2m_nav.py --true-odom       # 비교용: /odom 을 바퀴로 셈하지 않고 참값으로 낸다
    ~/isaacsim/python.sh 44_pl2m_nav/pl2m_nav.py --log run.json    # 참값과 오도메트리를 0.1초마다 적어 둔다
    ~/isaacsim/python.sh 44_pl2m_nav/pl2m_nav.py --test deck       # 바닥판이 얇을 때와 두꺼울 때 음료대 앞에서 직진해 본다 (ROS 2 없이)

주고받는 토픽
    받기   /cmd_vel   /joint_command
    내기   /scan (라이다, 10 Hz)   /odom · /tf odom → base_footprint (바퀴가 돈 각도로 셈한 위치)   /joint_states   /clock
           /ground_truth (시뮬레이터만 아는 참값. 채점할 때만 쓴다. 로봇의 프로그램은 이것을 보면 안 된다)

강의 41 과 다른 점
    /odom      강의 41 은 시뮬레이터의 참값을 그대로 냈다. 여기서는 실제 로봇처럼 바퀴 엔코더로 셈한다 (WheelOdometry).
    라이다     앞쪽 260° 만 본다. 뒤쪽은 로봇의 몸통에 가려진다.
    바닥판     충돌 모양을 아래로 두껍게 했다 (윗면은 그대로 2 cm). 두께 2 cm 의 얇은 상자일 때는 어떤 자리에서 바퀴가 헛돌았다.
"""
import argparse
import json
import math
import os
import signal
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, ".."))
BASE_GRAPH = "/Graphs/ros2_state"                       # 강의 41 의 STATE_GRAPH 자리에 놓는다: 보간기가 여기의 SimTime 을 읽는다
LASER_LINK = "/World/pl2m/lidar_mount/laser_link"      # URDF 의 빈 링크 = 라이다 자리. 로봇의 뒤쪽을 +X 로 둔 좌표계다 (원본 URDF)
LIDAR = LASER_LINK + "/Lidar"
LIDAR_FOV, LIDAR_RANGE = 260.0, (0.05, 20.0)           # 앞쪽 260° [°], 재는 거리 [m] (원본 설정의 lidar)
SCAN_EVERY = 6                                         # 화면 6번에 한 번 = 10 Hz
DECK_THICKNESS = 1.0                                   # 바닥판의 충돌 모양 두께 [m]. 윗면의 높이(2 cm)는 그대로 두고 아래로 늘인다

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--true-odom", action="store_true", help="/odom 을 참값으로 낸다 (비교용)")
    parser.add_argument("--arms", choices=["transport", "down"], default="transport", help="처음 팔 자세")
    parser.add_argument("--compensate", choices=["both", "none"], default="both", help="중력 보상을 넣을 팔")
    parser.add_argument("--full-lidar", action="store_true", help="라이다를 360° 로 둔다 (자기 몸이 잡히는 것을 보는 비교용)")
    parser.add_argument("--thin-deck", action="store_true", help="바닥판을 원래의 얇은 상자(2 cm)로 둔다 (비교용)")
    parser.add_argument("--test", choices=["deck"], help="deck: 얇은 바닥판과 두꺼운 바닥판에서 직진을 비교한다")
    parser.add_argument("--gui", action="store_true", help="창을 띄운다")
    parser.add_argument("--seconds", type=float, help="이 시간만큼만 돌리고 끝낸다 (없으면 Ctrl+C 나 창을 닫을 때까지)")
    parser.add_argument("--log", metavar="JSON", help="참값과 오도메트리를 0.1초마다 적어 이 파일에 저장한다")
    args = parser.parse_args()

    from isaacsim import SimulationApp
    app = SimulationApp({"headless": not args.gui, "extra_args": [
        "--/app/runLoops/main/rateLimitEnabled=true", "--/app/runLoops/main/rateLimitFrequency=60"]})

import numpy as np
import omni.graph.core as og
import omni.kit.commands
import omni.physics.tensors
import omni.physx
from isaacsim.core.utils.extensions import enable_extension
from pxr import Gf, UsdGeom

sys.path.insert(0, HERE)
sys.path.insert(0, REPO)
for folder in ("37_pl2m_import", "38_pl2m_booth", "39_pl2m_drive", "40_pl2m_gravity", "41_pl2m_ros2"):
    sys.path.insert(0, os.path.join(REPO, folder))
from booth import SPAWN
from common.sim import Sim, new_scene, set_pose, set_view
from import_pl2m import DEFAULT_URDF, find_urdf, import_pl2m
from pl2m_drive import ARM_JOINTS, WHEEL_DISTANCE, WHEEL_JOINTS, WHEEL_RADIUS, set_arm_pose, set_wheel_velocity
from pl2m_gravity import ROOT, GravityCompensation, prepare_pl2m
from pl2m_ros2 import JointCommandInterpolator, add_cmd_vel, add_joint_command, enable_ros2


def thicken_deck(stage, thickness=DECK_THICKNESS):
    """바닥판(9 × 6 m 상자)을 아래로 두껍게 한다. 윗면의 높이는 그대로다.

    원래는 두께 2 cm 의 얇은 상자다. 그대로 두면 어떤 자리에서 바퀴와 바닥판의 접촉이 사라져, 바퀴는 도는데 로봇이 가지 않았다 (--test deck).
    """
    for prim in stage.GetPrimAtPath("/World/Booth/Parts").GetChildren():
        if prim.GetName().startswith("booth_floor"):
            ops = {op.GetOpName(): op for op in UsdGeom.Xformable(prim).GetOrderedXformOps()}
            center, size = ops["xformOp:translate"].Get(), ops["xformOp:scale"].Get()
            top = center[2] + size[2] / 2.0
            ops["xformOp:translate"].Set(Gf.Vec3d(center[0], center[1], top - thickness / 2.0))
            ops["xformOp:scale"].Set(Gf.Vec3f(size[0], size[1], thickness))
            return top
    raise RuntimeError("바닥판(booth_floor)을 찾지 못함")


def add_lidar(stage, full=False, draw=False):
    """2D 라이다: laser_link 의 자식. 1° 간격. 뒤쪽 100° 는 몸통에 가려지므로 앞쪽 260° 만 쏜다."""
    enable_extension("isaacsim.sensors.physx")
    omni.kit.commands.execute(
        "RangeSensorCreateLidar", path="/Lidar", parent=LASER_LINK, min_range=LIDAR_RANGE[0], max_range=LIDAR_RANGE[1],
        horizontal_fov=360.0 if full else LIDAR_FOV, horizontal_resolution=1.0, vertical_fov=1.0, vertical_resolution=1.0,
        rotation_rate=0.0, high_lod=False, yaw_offset=0.0 if full else 180.0,      # laser_link 의 +X 가 뒤쪽이라 180° 돌려 앞을 가운데로
        draw_points=False, draw_lines=draw, enable_semantics=False)
    return stage.GetPrimAtPath(LIDAR)


def add_base_publishers():
    """/clock · /joint_states · /odom · /tf · /ground_truth · /scan 을 낸다. 위치 값은 BasePose 가 스텝마다 넣어 준다."""
    keys = og.Controller.Keys
    stamped = ["PublishClock", "PublishJointState", "PublishOdometry", "PublishOdomTF", "PublishTruth", "PublishScan"]
    fields = ("azimuthRange", "depthRange", "horizontalFov", "horizontalResolution", "intensitiesData", "linearDepthData", "numCols", "numRows", "rotationRate")
    og.Controller.edit({"graph_path": BASE_GRAPH, "evaluator_name": "execution"}, {
        keys.CREATE_NODES: [("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                            ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                            ("SimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                            ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
                            ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
                            ("PublishOdometry", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
                            ("PublishOdomTF", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
                            ("PublishTruth", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
                            ("ScanGate", "isaacsim.core.nodes.IsaacSimulationGate"),
                            ("ReadLidar", "isaacsim.sensors.physx.IsaacReadLidarBeams"),
                            ("PublishScan", "isaacsim.ros2.bridge.ROS2PublishLaserScan")],
        keys.SET_VALUES: [("PublishJointState.inputs:targetPrim", ROOT), ("PublishJointState.inputs:topicName", "joint_states"),
                          ("PublishOdometry.inputs:topicName", "odom"), ("PublishOdometry.inputs:odomFrameId", "odom"),
                          ("PublishOdometry.inputs:chassisFrameId", "base_footprint"), ("PublishOdometry.inputs:publishRawVelocities", True),
                          ("PublishOdomTF.inputs:parentFrameId", "odom"), ("PublishOdomTF.inputs:childFrameId", "base_footprint"),
                          ("PublishTruth.inputs:topicName", "ground_truth"), ("PublishTruth.inputs:odomFrameId", "map"),
                          ("PublishTruth.inputs:chassisFrameId", "base_footprint"), ("PublishTruth.inputs:publishRawVelocities", True),
                          ("ScanGate.inputs:step", SCAN_EVERY),
                          ("ReadLidar.inputs:lidarPrim", LIDAR),
                          ("PublishScan.inputs:topicName", "scan"), ("PublishScan.inputs:frameId", "laser_link")],
        keys.CONNECT: [("OnPlaybackTick.outputs:tick", f"{node}.inputs:execIn") for node in stamped[:-1] + ["ScanGate"]]
        + [("ScanGate.outputs:execOut", "ReadLidar.inputs:execIn"), ("ReadLidar.outputs:execOut", "PublishScan.inputs:execIn")]
        + [(f"ReadLidar.outputs:{name}", f"PublishScan.inputs:{name}") for name in fields]
        + [("Context.outputs:context", f"{node}.inputs:context") for node in stamped]
        + [("SimTime.outputs:simulationTime", f"{node}.inputs:timeStamp") for node in stamped]})


class WheelOdometry:
    """바퀴가 돈 각도로 로봇의 위치를 셈한다 (엔코더 오도메트리). 처음 자리가 (0, 0, 0) 이다. 원본 시뮬레이터와 같은 식."""

    def __init__(self, view):
        names = list(view.shared_metatype.dof_names)
        self.view, self.dofs = view, [names.index(name) for name in WHEEL_JOINTS]       # 왼쪽 · 오른쪽 바퀴
        self.before = view.get_dof_positions()[0][self.dofs].copy()
        self.x = self.y = self.yaw = self.linear = self.angular = 0.0

    def step(self, dt):
        now = self.view.get_dof_positions()[0][self.dofs]
        turned = (now - self.before + math.pi) % (2.0 * math.pi) - math.pi   # 관절 각도는 한 바퀴를 넘으면 되감긴다: 한 스텝의 변화만 쓴다
        left, right = turned * WHEEL_RADIUS                              # 두 바퀴가 굴러간 거리 [m]
        self.before = now.copy()
        forward, turn = (left + right) / 2.0, (right - left) / WHEEL_DISTANCE
        middle = self.yaw + turn / 2.0                                   # 이 스텝 동안의 가운데 방향
        self.x += forward * math.cos(middle)
        self.y += forward * math.sin(middle)
        self.yaw += turn
        self.linear, self.angular = forward / dt, turn / dt


class BasePose:
    """물리 스텝마다 참값과 오도메트리를 읽어 ROS 2 발행 노드의 입력 칸에 넣는다."""

    def __init__(self, sim, start, true_odom=False):
        self.sim, self.start, self.true_odom = sim, start, true_odom
        self.physics = omni.physics.tensors.create_simulation_view("numpy")       # 참조를 잡아 둬야 뷰가 살아 있다 (강의 40)
        self.view = self.physics.create_articulation_view(ROOT)
        self.odometry = WheelOdometry(self.view)
        attribute = lambda name: og.Controller.attribute(f"{BASE_GRAPH}/{name}")
        self.inputs = {key: attribute(name) for key, name in (
            ("odom_position", "PublishOdometry.inputs:position"), ("odom_orientation", "PublishOdometry.inputs:orientation"),
            ("odom_linear", "PublishOdometry.inputs:linearVelocity"), ("odom_angular", "PublishOdometry.inputs:angularVelocity"),
            ("tf_translation", "PublishOdomTF.inputs:translation"), ("tf_rotation", "PublishOdomTF.inputs:rotation"),
            ("truth_position", "PublishTruth.inputs:position"), ("truth_orientation", "PublishTruth.inputs:orientation"),
            ("truth_linear", "PublishTruth.inputs:linearVelocity"), ("truth_angular", "PublishTruth.inputs:angularVelocity"))}
        self.log, self.logged = [], -1.0
        self._sub = omni.physx.get_physx_interface().subscribe_physics_step_events(self._on_step)

    def truth(self):
        """참값: 부스 좌표계에서 base_footprint 의 (x, y, 방향[rad])."""
        x, y, _, qx, qy, qz, qw = self.view.get_root_transforms()[0]
        return float(x), float(y), math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))

    def odom_from_truth(self, truth):
        """참값을 처음 자리 기준으로 옮긴 것 (--true-odom)."""
        dx, dy = truth[0] - self.start[0], truth[1] - self.start[1]
        cos, sin = math.cos(self.start[2]), math.sin(self.start[2])
        return dx * cos + dy * sin, -dx * sin + dy * cos, truth[2] - self.start[2]

    def _on_step(self, dt):
        self.odometry.step(dt)
        truth = self.truth()
        odom = self.odom_from_truth(truth) if self.true_odom else (self.odometry.x, self.odometry.y, self.odometry.yaw)
        quaternion = lambda yaw: [0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)]        # (x, y, z, w)
        velocity = self.view.get_root_velocities()[0]
        forward = float(velocity[0]) * math.cos(truth[2]) + float(velocity[1]) * math.sin(truth[2])
        for key, value in (("odom_position", [odom[0], odom[1], 0.0]), ("odom_orientation", quaternion(odom[2])),
                           ("odom_linear", [self.odometry.linear, 0.0, 0.0]), ("odom_angular", [0.0, 0.0, self.odometry.angular]),
                           ("tf_translation", [odom[0], odom[1], 0.0]), ("tf_rotation", quaternion(odom[2])),
                           ("truth_position", [truth[0], truth[1], 0.0]), ("truth_orientation", quaternion(truth[2])),
                           ("truth_linear", [forward, 0.0, 0.0]), ("truth_angular", [0.0, 0.0, float(velocity[5])])):
            self.inputs[key].set(value)
        if self.sim.time - self.logged >= 0.1 - 1e-6:
            self.logged = self.sim.time
            self.log.append([round(self.sim.time, 3)] + [round(v, 4) for v in truth] + [round(v, 4) for v in (self.odometry.x, self.odometry.y, self.odometry.yaw)])

    def stop(self):
        self._sub = None


def build_scene(app, stage, start=SPAWN, arms="transport", full_lidar=False, draw_lidar=False, thick_deck=True, ros=True):
    """부스 + 컵 + PL-2M + 라이다 + ROS 2 그래프. 뒤 강의에서도 이 함수를 불러 같은 장면을 만든다."""
    import make_booth
    mjcf = make_booth.read_mjcf(make_booth.build_mjcf())
    make_booth.add_booth(stage, mjcf)
    make_booth.add_lights(stage, mjcf)
    make_booth.add_cups(stage)
    if thick_deck:
        thicken_deck(stage)
    robot = import_pl2m(stage, find_urdf(DEFAULT_URDF))
    set_pose(robot, translate=(start[0], start[1], 0.022), rotate_xyz=(0.0, 0.0, math.degrees(start[2])))      # 바닥판(2 cm) 위
    prepare_pl2m(stage)                                  # 팔 링크의 중력을 켜 둔다 (강의 40)
    if arms != "down":
        set_arm_pose(stage, arms)                        # 이동할 때의 팔 자세: 팔꿈치를 접어 몸 가까이 둔다
    add_lidar(stage, full_lidar, draw_lidar)
    if ros:
        add_joint_command()
        add_cmd_vel()
        add_base_publishers()
    return robot


def test_deck(app):
    """음료대 앞(7.65, -2.82)에서 0.1 m/s 로 6초 직진한다. 바닥판이 얇을 때와 두꺼울 때를 비교한다."""
    for thick in (False, True):
        stage = new_scene()
        build_scene(app, stage, start=(7.65, -2.82, math.pi / 2), thick_deck=thick, ros=False)
        sim = Sim(app)
        sim.play()
        sim.run(1.5)
        physics = omni.physics.tensors.create_simulation_view("numpy")
        view = physics.create_articulation_view(ROOT)
        wheels = [list(view.shared_metatype.dof_names).index(name) for name in WHEEL_JOINTS]
        print(f"바닥판의 충돌 모양: 두께 {DECK_THICKNESS if thick else 0.02:g} m", flush=True)
        print(f"{'시간[s]':>8}{'간 거리[m]':>10}{'로봇 속도[m/s]':>13}{'바퀴 둘레 속도[m/s]':>16}", flush=True)
        begin = view.get_root_transforms()[0][:2].copy()
        for second in range(1, 7):
            set_wheel_velocity(stage, 0.1, 0.0)
            sim.run(1.0)
            position, velocity = view.get_root_transforms()[0][:2], view.get_root_velocities()[0]
            rim = float(np.mean(view.get_dof_velocities()[0][wheels])) * WHEEL_RADIUS
            print(f"{second:>8}{float(np.linalg.norm(position - begin)):>12.3f}{float(np.hypot(velocity[0], velocity[1])):>14.3f}{rim:>18.3f}", flush=True)
        sim.stop()


if __name__ == "__main__":
    enable_ros2(app)
    if args.test == "deck":
        test_deck(app)
        app.close()
        sys.exit(0)
    stage = new_scene()
    build_scene(app, stage, arms=args.arms, full_lidar=args.full_lidar, draw_lidar=args.gui, thick_deck=not args.thin_deck)
    if args.gui:
        set_view(eye=(7.2, -7.2, 4.6), target=(6.4, -2.6, 0.4))       # 처음 자리 · 음료대 · 상담 테이블 1 이 함께 보이는 자리
    sim = Sim(app)
    sim.play()
    sim.run(1.0)
    compensation = GravityCompensation(joints=ARM_JOINTS if args.compensate == "both" else [])
    interpolator = JointCommandInterpolator(sim)
    pose = BasePose(sim, SPAWN, args.true_odom)
    print(f"준비됨: 처음 자리 ({SPAWN[0]:.2f}, {SPAWN[1]:.2f}), /odom = {'참값' if args.true_odom else '바퀴로 셈한 값'}, "
          f"라이다 {'360' if args.full_lidar else f'{LIDAR_FOV:g}'}°. 끝내려면 Ctrl+C", flush=True)
    stop, clock = [], sim.time
    signal.signal(signal.SIGINT, lambda *_: stop.append(True))
    while app.is_running() and not stop and (not args.seconds or sim.time - clock < args.seconds):
        app.update()
    truth, odom = pose.truth(), pose.odom_from_truth(pose.truth())
    print(f"끝날 때 참값 ({truth[0]:.3f}, {truth[1]:.3f}, {math.degrees(truth[2]):.1f}°).  처음 자리 기준으로는 ({odom[0]:.3f}, {odom[1]:.3f}, {math.degrees(odom[2]):.1f}°), "
          f"바퀴로 셈한 값은 ({pose.odometry.x:.3f}, {pose.odometry.y:.3f}, {math.degrees(pose.odometry.yaw):.1f}°)", flush=True)
    if args.log:
        with open(args.log, "w") as file:
            json.dump({"columns": ["time", "true_x", "true_y", "true_yaw", "odom_x", "odom_y", "odom_yaw"], "start": list(SPAWN), "rows": pose.log}, file)
        print("저장:", args.log, flush=True)
    pose.stop()
    interpolator.stop()
    compensation.stop()
    sim.stop()
    app.close()
