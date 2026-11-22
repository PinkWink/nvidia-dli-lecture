"""강의 41: PL-2M 을 ROS 2 에 연결합니다. 관절 명령은 ROS 2 에서 받고, 중력 보상은 시뮬레이터 안에서 합니다.

    source /opt/ros/jazzy/setup.bash                                  # Isaac Sim 보다 먼저
    ~/isaacsim/python.sh 41_pl2m_ros2/pl2m_ros2.py                    # 두 팔 모두 보상
    ~/isaacsim/python.sh 41_pl2m_ros2/pl2m_ros2.py --compensate left  # 왼팔만 보상 (오른팔과 비교)
    ~/isaacsim/python.sh 41_pl2m_ros2/pl2m_ros2.py --compensate none  # 보상 없음
    ~/isaacsim/python.sh 41_pl2m_ros2/pl2m_ros2.py --direct           # 비교용: 받은 목표를 보간 없이 그대로 넣는다
    ~/isaacsim/python.sh 41_pl2m_ros2/pl2m_ros2.py --wire-effort      # 틀린 연결: effort 칸까지 이으면 보상이 지워진다
    ~/isaacsim/python.sh 41_pl2m_ros2/pl2m_ros2.py --booth --gui      # 강의 38 의 부스 안에서, 창으로

주고받는 토픽
    받기   /joint_command (sensor_msgs/JointState, 위치만 씀)   /cmd_vel (geometry_msgs/Twist)
    내기   /joint_states   /odom   /tf (odom → base_footprint)   /clock

역할 나누기 (원본 MuJoCo 시뮬레이터와 같음)
    ROS 2 쪽 (MoveIt · 궤적 서버)   어디로 갈지: 관절의 목표 위치를 정해진 간격으로 보냄 (간격은 보내는 쪽 마음대로)
    시뮬레이터 쪽 (로봇)            받은 목표 사이를 자기 주기(물리 스텝)로 보간 (JointCommandInterpolator)
                                    어떻게 버틸지: 위치 구동 + 중력 보상 토크 (강의 40 의 GravityCompensation)
"""
import argparse
import collections
import json
import os
import signal
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, ".."))
JOINT_GRAPH, DRIVE_GRAPH, STATE_GRAPH = "/Graphs/ros2_joints", "/Graphs/ros2_cmd_vel", "/Graphs/ros2_state"

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--compensate", choices=["both", "left", "none"], default="both", help="중력 보상을 넣을 팔")
    parser.add_argument("--direct", action="store_true", help="받은 목표를 보간 없이 Articulation Controller 로 바로 넣는다 (비교용)")
    parser.add_argument("--wire-effort", action="store_true", help="/joint_command 의 effort 칸까지 제어기에 잇는다 (틀린 연결의 시연. --direct 로 동작)")
    parser.add_argument("--booth", action="store_true", help="강의 38 의 부스 안에서")
    parser.add_argument("--gui", action="store_true", help="창을 띄운다")
    parser.add_argument("--seconds", type=float, help="이 시간만큼만 돌리고 끝낸다 (없으면 Ctrl+C 나 창을 닫을 때까지)")
    parser.add_argument("--log", metavar="JSON", help="팔 관절의 목표 · 각도 · 토크를 물리 스텝마다 적어 이 파일에 저장한다")
    args = parser.parse_args()

    from isaacsim import SimulationApp
    app = SimulationApp({"headless": not args.gui, "extra_args": [
        "--/app/runLoops/main/rateLimitEnabled=true", "--/app/runLoops/main/rateLimitFrequency=60"]})

import numpy as np
import omni.graph.core as og
import omni.physics.tensors
import omni.physx
from isaacsim.core.utils.extensions import enable_extension

sys.path.insert(0, REPO)
sys.path.insert(0, os.path.join(REPO, "37_pl2m_import"))
sys.path.insert(0, os.path.join(REPO, "39_pl2m_drive"))
sys.path.insert(0, os.path.join(REPO, "40_pl2m_gravity"))
from common.sim import Sim, new_scene, set_pose, set_view
from import_pl2m import DEFAULT_URDF, find_urdf, import_pl2m
from pl2m_drive import ARM_JOINTS, MAX_ANGULAR, MAX_LINEAR, MIRROR, WHEEL_DISTANCE, WHEEL_JOINTS, WHEEL_RADIUS
from pl2m_gravity import LEFT_ARM, RIGHT_ARM, ROOT, GravityCompensation, prepare_pl2m


def add_joint_command(direct=False, wire_effort=False):
    """/joint_command (JointState) 를 받는 그래프.

    기본: 받기만 한다. 받은 목표는 JointCommandInterpolator 가 물리 스텝마다 보간해서 관절에 넣는다.
    direct: Articulation Controller 에 바로 잇는다. 메시지가 온 프레임에 목표가 그 값으로 한 번에 바뀐다 (비교용).
    """
    keys = og.Controller.Keys
    nodes = [("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
             ("Context", "isaacsim.ros2.bridge.ROS2Context"),
             ("SubscribeJointState", "isaacsim.ros2.bridge.ROS2SubscribeJointState")]
    values = [("SubscribeJointState.inputs:topicName", "joint_command")]
    connects = [("OnPlaybackTick.outputs:tick", "SubscribeJointState.inputs:execIn"),
                ("Context.outputs:context", "SubscribeJointState.inputs:context")]
    if direct or wire_effort:
        nodes += [("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController")]
        values += [("ArticulationController.inputs:targetPrim", ROOT)]
        connects += [("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
                     ("SubscribeJointState.outputs:jointNames", "ArticulationController.inputs:jointNames"),
                     ("SubscribeJointState.outputs:positionCommand", "ArticulationController.inputs:positionCommand")]
    else:
        values += [("SubscribeJointState.inputs:queueSize", 1)]     # 한 프레임에 여러 개가 와도 가장 새 것만 남긴다
    if wire_effort:      # 이렇게 이으면 메시지의 effort 가 관절 힘으로 들어가, 중력 보상으로 넣은 힘을 덮어쓴다
        connects += [("SubscribeJointState.outputs:effortCommand", "ArticulationController.inputs:effortCommand")]
    og.Controller.edit({"graph_path": JOINT_GRAPH, "evaluator_name": "execution"},
                       {keys.CREATE_NODES: nodes, keys.SET_VALUES: values, keys.CONNECT: connects})


class JointCommandInterpolator:
    """받은 목표 사이를 물리 스텝마다 이어서 관절의 목표 위치로 넣는다. Play 뒤에 만든다.

    보내는 쪽은 자기 간격(예: 50 Hz)으로 보내고, 시뮬레이터는 자기 주기(60 Hz)로 돈다. 받은 값을 그대로 넣으면
    목표가 들쭉날쭉 바뀌므로, 실제 제어기처럼 받는 쪽에서 보간한다.
      - 메시지마다 (그 목표의 시각, 값) 을 모아 두고
      - 그보다 조금 늦은 "재생 시각" 을 스텝마다 한 스텝씩 나아가게 해서, 그 시각의 값을 두 점 사이 직선으로 구한다
    늦게 따라가는 정도(delay)는 0.1초, 또는 보내는 간격의 2배 + 2 스텝 가운데 큰 쪽이다.
    메시지가 늦어져 재생할 것이 떨어지면 그 자리에 멈췄다가, 다시 오면 멈춘 자리에서 이어 간다 (건너뛰지 않는다).
    """

    DELAY = 0.1                                # 기본 지연 [s]: 메시지가 이만큼 늦게 와도 고르게 움직인다

    def __init__(self, sim):
        self.sim = sim
        self.physics = omni.physics.tensors.create_simulation_view("numpy")
        self.robot = self.physics.create_articulation_view(ROOT)
        self.names = list(self.robot.shared_metatype.dof_names)
        self.points = {}                       # 관절 이름 → 최근에 받은 (시각, 값) 들
        self.period, self.delay = 0.02, self.DELAY     # 보내는 간격의 어림값 [s], 지금 쓰는 지연 [s]
        self.play = None                       # 재생 시각 [s]
        self.last, self.newest, self.received, self.stalled = None, 0.0, 0, 0
        node = JOINT_GRAPH + "/SubscribeJointState"
        self._names = og.Controller.attribute("outputs:jointNames", node)
        self._positions = og.Controller.attribute("outputs:positionCommand", node)
        self._stamp = og.Controller.attribute("outputs:timeStamp", node)
        self._clock = og.Controller.attribute("outputs:simulationTime", STATE_GRAPH + "/SimTime")
        self._every = np.arange(self.robot.count)
        self._sub = omni.physx.get_physx_interface().subscribe_physics_step_events(self._on_step)

    def _receive(self, now, dt):
        """새 메시지가 왔으면 (시각, 값) 을 모아 둔다."""
        positions, stamp = self._positions.get(), float(self._stamp.get())
        if positions is None or len(positions) == 0:
            return
        key = (stamp, tuple(float(value) for value in positions))
        if key == self.last:
            return
        self.last, self.received = key, self.received + 1
        when = stamp if stamp > 0.0 else now                       # 보내는 쪽이 시각을 적지 않았으면 받은 시각을 쓴다
        gap = when - self.newest
        fresh = self.play is None or gap >= 0.3                    # 처음이거나 한동안 끊겼다 다시 옴
        if 0.0 < gap < 0.3:                                        # 이어서 오는 중: 보내는 간격을 어림한다
            self.period += 0.1 * (gap - self.period)
            self.delay = max(self.DELAY, 2.0 * self.period + 2.0 * dt)
        if fresh:
            self.play = when - self.delay                          # 지금 목표에서 delay 에 걸쳐 첫 목표로 간다
        targets = self.robot.get_dof_position_targets()[0]
        for name, value in zip(self._names.get(), key[1]):
            points = self.points.setdefault(str(name), collections.deque(maxlen=64))
            if fresh or not points:
                points.clear()
                points.append((self.play, float(targets[self.names.index(str(name))])))
            if when > points[-1][0]:
                points.append((when, value))
            else:
                points[-1] = (points[-1][0], value)                # 같은 시각의 목표가 또 왔으면 값만 바꾼다
        self.newest = max(self.newest, when)

    def _on_step(self, dt):
        now = float(self._clock.get())
        self._receive(now, dt)
        if self.play is None:
            return
        behind = (now - self.delay) - self.play                    # 재생이 밀려 있으면 5 % 빠르게 따라잡는다
        step = min(self.play + dt * (1.05 if behind > dt else 1.0), now - self.delay, self.newest)
        if step <= self.play and self.play < self.newest:
            self.stalled += 1
        self.play = max(step, self.play)
        targets = self.robot.get_dof_position_targets().copy()
        for name, points in self.points.items():
            value = points[-1][1]                                  # 받은 것을 다 재생했으면 마지막 값을 지킨다
            for (t0, v0), (t1, v1) in zip(points, list(points)[1:]):
                if self.play < t1:
                    value = v0 if self.play <= t0 else v0 + (v1 - v0) * (self.play - t0) / (t1 - t0)
                    break
            targets[0, self.names.index(name)] = value
        self.robot.set_dof_position_targets(targets, self._every)

    def stop(self):
        self._sub = None


def add_cmd_vel():
    """/cmd_vel (Twist) → Differential Controller → 바퀴 관절의 속도. 강의 14 의 그래프와 같다."""
    keys = og.Controller.Keys
    og.Controller.edit({"graph_path": DRIVE_GRAPH, "evaluator_name": "execution"}, {
        keys.CREATE_NODES: [("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                            ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                            ("SubscribeTwist", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
                            ("BreakLinear", "omni.graph.nodes.BreakVector3"),
                            ("BreakAngular", "omni.graph.nodes.BreakVector3"),
                            ("DifferentialController", "isaacsim.robot.wheeled_robots.DifferentialController"),
                            ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController")],
        keys.SET_VALUES: [("SubscribeTwist.inputs:topicName", "cmd_vel"),
                          ("DifferentialController.inputs:wheelRadius", WHEEL_RADIUS),
                          ("DifferentialController.inputs:wheelDistance", WHEEL_DISTANCE),
                          ("DifferentialController.inputs:maxLinearSpeed", MAX_LINEAR),
                          ("DifferentialController.inputs:maxAngularSpeed", MAX_ANGULAR),
                          ("ArticulationController.inputs:targetPrim", ROOT),
                          ("ArticulationController.inputs:jointNames", WHEEL_JOINTS)],
        keys.CONNECT: [("OnPlaybackTick.outputs:tick", "SubscribeTwist.inputs:execIn"),
                       ("OnPlaybackTick.outputs:tick", "DifferentialController.inputs:execIn"),
                       ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
                       ("Context.outputs:context", "SubscribeTwist.inputs:context"),
                       ("SubscribeTwist.outputs:linearVelocity", "BreakLinear.inputs:tuple"),
                       ("SubscribeTwist.outputs:angularVelocity", "BreakAngular.inputs:tuple"),
                       ("BreakLinear.outputs:x", "DifferentialController.inputs:linearVelocity"),
                       ("BreakAngular.outputs:z", "DifferentialController.inputs:angularVelocity"),
                       ("DifferentialController.outputs:velocityCommand", "ArticulationController.inputs:velocityCommand")]})


def add_state_publishers():
    """/clock · /joint_states · /odom · /tf (odom → base_footprint) 를 낸다."""
    keys = og.Controller.Keys
    stamped = ["PublishClock", "PublishJointState", "PublishOdometry", "PublishOdomTF"]
    og.Controller.edit({"graph_path": STATE_GRAPH, "evaluator_name": "execution"}, {
        keys.CREATE_NODES: [("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                            ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                            ("SimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                            ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
                            ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
                            ("ComputeOdometry", "isaacsim.core.nodes.IsaacComputeOdometry"),
                            ("PublishOdometry", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
                            ("PublishOdomTF", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree")],
        keys.SET_VALUES: [("PublishJointState.inputs:targetPrim", ROOT), ("PublishJointState.inputs:topicName", "joint_states"),
                          ("ComputeOdometry.inputs:chassisPrim", ROOT),
                          ("PublishOdometry.inputs:topicName", "odom"), ("PublishOdometry.inputs:odomFrameId", "odom"),
                          ("PublishOdometry.inputs:chassisFrameId", "base_footprint"),
                          ("PublishOdomTF.inputs:parentFrameId", "odom"), ("PublishOdomTF.inputs:childFrameId", "base_footprint")],
        keys.CONNECT: [("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                       ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
                       ("OnPlaybackTick.outputs:tick", "ComputeOdometry.inputs:execIn"),
                       ("ComputeOdometry.outputs:execOut", "PublishOdometry.inputs:execIn"),
                       ("ComputeOdometry.outputs:execOut", "PublishOdomTF.inputs:execIn"),
                       ("ComputeOdometry.outputs:position", "PublishOdometry.inputs:position"),
                       ("ComputeOdometry.outputs:orientation", "PublishOdometry.inputs:orientation"),
                       ("ComputeOdometry.outputs:linearVelocity", "PublishOdometry.inputs:linearVelocity"),
                       ("ComputeOdometry.outputs:angularVelocity", "PublishOdometry.inputs:angularVelocity"),
                       ("ComputeOdometry.outputs:position", "PublishOdomTF.inputs:translation"),
                       ("ComputeOdometry.outputs:orientation", "PublishOdomTF.inputs:rotation")]
        + [("Context.outputs:context", f"{node}.inputs:context") for node in stamped]
        + [("SimTime.outputs:simulationTime", f"{node}.inputs:timeStamp") for node in stamped]})


def add_ros2(direct=False, wire_effort=False):
    """ROS 2 그래프 셋을 만든다. 장면을 만들기 전에 enable_ros2() 를 먼저 불러야 한다."""
    add_joint_command(direct, wire_effort)
    add_cmd_vel()
    add_state_publishers()


def enable_ros2(app):
    enable_extension("isaacsim.ros2.bridge")
    enable_extension("isaacsim.robot.wheeled_robots")
    app.update()                                         # 익스텐션이 다 올라온 뒤에 그래프를 만들어야 한다 (안 하면 Play 에서 죽었음)


if __name__ == "__main__":
    enable_ros2(app)
    stage = new_scene()
    start = (0.0, 0.0, 0.002)                            # 바닥에 거의 닿은 높이 (원본 설정의 spawn 과 같음). 높이서 떨어뜨리면 /odom 의 z 가 그만큼 어긋난다
    if args.booth:
        sys.path.insert(0, os.path.join(REPO, "38_pl2m_booth"))
        import make_booth
        mjcf = make_booth.read_mjcf(make_booth.build_mjcf())
        make_booth.add_booth(stage, mjcf)
        make_booth.add_lights(stage, mjcf)
        make_booth.add_cups(stage)
        start = (make_booth.SPAWN[0], make_booth.SPAWN[1], 0.002)
    robot = import_pl2m(stage, find_urdf(DEFAULT_URDF))
    set_pose(robot, translate=start)
    prepare_pl2m(stage)                                  # 팔 링크의 중력을 켜 둔다 (강의 40)
    direct = args.direct or args.wire_effort
    add_ros2(direct, args.wire_effort)
    if args.gui and args.booth:                          # 부스에서는 강의 39 와 같은 자리에서 본다 (앞쪽은 사람 모형이 가린다)
        set_view(eye=(start[0] - 2.4, start[1] - 2.6, 2.2), target=(start[0] + 0.4, start[1], 0.6))
    elif args.gui:
        set_view(eye=(start[0] + 2.3, start[1] - 1.7, 1.25), target=(start[0] + 0.15, start[1], 0.72))
    sim = Sim(app)
    sim.play()
    sim.run(1.0)
    joints = {"both": ARM_JOINTS, "left": LEFT_ARM, "none": []}[args.compensate]
    compensation = GravityCompensation(joints=joints)    # 보상은 시뮬레이터 안에서. ROS 2 는 위치만 보낸다
    interpolator = None if direct else JointCommandInterpolator(sim)   # 받은 목표 사이를 물리 스텝마다 보간
    log, mirror, clock = [], np.array(MIRROR, dtype=float), sim.time
    if args.log:
        def record(dt):                                  # 오른팔의 값은 부호를 왼팔에 맞춘다
            log.append((sim.time - clock, compensation.state(LEFT_ARM), [value * mirror for value in compensation.state(RIGHT_ARM)]))
        record_sub = omni.physx.get_physx_interface().subscribe_physics_step_events(record)
    print(f"준비됨: 중력 보상 = {args.compensate}, 목표 = {'그대로 넣음' if direct else '보간'}"
          + (", effort 칸을 이음" if args.wire_effort else "") + ". 끝내려면 Ctrl+C", flush=True)
    stop = []
    signal.signal(signal.SIGINT, lambda *_: stop.append(True))     # Ctrl+C: 바로 죽지 않고, 기록을 저장한 뒤 끝낸다
    while app.is_running() and not stop and (not args.seconds or sim.time - clock < args.seconds):
        app.update()
    if args.log:
        record_sub = None
        pack = lambda side: {key: [[round(float(v), 4) for v in row[side][i]] for row in log] for i, key in enumerate(("target", "angle", "total", "added"))}
        with open(args.log, "w") as file:
            json.dump({"time": [round(row[0], 4) for row in log], "left": pack(1), "right": pack(2)}, file)
        print("저장:", args.log, flush=True)
    if interpolator:
        print(f"받은 메시지 {interpolator.received}개, 보내는 간격의 어림값 {interpolator.period * 1000:.1f} ms, 지연 {interpolator.delay * 1000:.0f} ms, "
              f"재생할 것이 떨어져 멈춘 스텝 {interpolator.stalled}번", flush=True)
        interpolator.stop()
    compensation.stop()
    sim.stop()
    app.close()
