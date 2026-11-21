"""강의 41: PL-2M 을 ROS 2 에 연결합니다. 관절 명령은 ROS 2 에서 받고, 중력 보상은 시뮬레이터 안에서 합니다.

    source /opt/ros/jazzy/setup.bash                                  # Isaac Sim 보다 먼저
    ~/isaacsim/python.sh 41_pl2m_ros2/pl2m_ros2.py                    # 두 팔 모두 보상
    ~/isaacsim/python.sh 41_pl2m_ros2/pl2m_ros2.py --compensate left  # 왼팔만 보상 (오른팔과 비교)
    ~/isaacsim/python.sh 41_pl2m_ros2/pl2m_ros2.py --compensate none  # 보상 없음
    ~/isaacsim/python.sh 41_pl2m_ros2/pl2m_ros2.py --wire-effort      # 틀린 연결: effort 칸까지 이으면 보상이 지워진다
    ~/isaacsim/python.sh 41_pl2m_ros2/pl2m_ros2.py --booth --gui      # 강의 38 의 부스 안에서, 창으로

주고받는 토픽
    받기   /joint_command (sensor_msgs/JointState, 위치만 씀)   /cmd_vel (geometry_msgs/Twist)
    내기   /joint_states   /odom   /tf (odom → base_footprint)   /clock

역할 나누기 (원본 MuJoCo 시뮬레이터와 같음)
    ROS 2 쪽 (MoveIt · 궤적 서버)   어디로 갈지: 관절의 목표 위치를 시간에 따라 보냄
    시뮬레이터 쪽 (로봇)            어떻게 버틸지: 위치 구동 + 중력 보상 토크 (강의 40 의 GravityCompensation)
"""
import argparse
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
    parser.add_argument("--wire-effort", action="store_true", help="/joint_command 의 effort 칸까지 제어기에 잇는다 (틀린 연결의 시연)")
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


def add_joint_command(wire_effort=False):
    """/joint_command (JointState) → Articulation Controller. 메시지에 이름이 적힌 관절의 목표 위치만 바꾼다."""
    keys = og.Controller.Keys
    connects = [("OnPlaybackTick.outputs:tick", "SubscribeJointState.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
                ("Context.outputs:context", "SubscribeJointState.inputs:context"),
                ("SubscribeJointState.outputs:jointNames", "ArticulationController.inputs:jointNames"),
                ("SubscribeJointState.outputs:positionCommand", "ArticulationController.inputs:positionCommand")]
    if wire_effort:      # 이렇게 이으면 메시지의 effort 가 관절 힘으로 들어가, 중력 보상으로 넣은 힘을 덮어쓴다
        connects += [("SubscribeJointState.outputs:effortCommand", "ArticulationController.inputs:effortCommand")]
    og.Controller.edit({"graph_path": JOINT_GRAPH, "evaluator_name": "execution"}, {
        keys.CREATE_NODES: [("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                            ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                            ("SubscribeJointState", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
                            ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController")],
        keys.SET_VALUES: [("SubscribeJointState.inputs:topicName", "joint_command"),
                          ("ArticulationController.inputs:targetPrim", ROOT)],
        keys.CONNECT: connects})


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


def add_ros2(wire_effort=False):
    """ROS 2 그래프 셋을 만든다. 장면을 만들기 전에 enable_ros2() 를 먼저 불러야 한다."""
    add_joint_command(wire_effort)
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
        start = (make_booth.SPAWN[0], make_booth.SPAWN[1], 0.022)     # 부스 바닥판(2 cm) 위
    robot = import_pl2m(stage, find_urdf(DEFAULT_URDF))
    set_pose(robot, translate=start)
    prepare_pl2m(stage)                                  # 팔 링크의 중력을 켜 둔다 (강의 40)
    add_ros2(args.wire_effort)
    if args.gui and args.booth:                          # 부스에서는 강의 39 와 같은 자리에서 본다 (앞쪽은 사람 모형이 가린다)
        set_view(eye=(start[0] - 2.4, start[1] - 2.6, 2.2), target=(start[0] + 0.4, start[1], 0.6))
    elif args.gui:
        set_view(eye=(start[0] + 2.3, start[1] - 1.7, 1.25), target=(start[0] + 0.15, start[1], 0.72))
    sim = Sim(app)
    sim.play()
    sim.run(1.0)
    joints = {"both": ARM_JOINTS, "left": LEFT_ARM, "none": []}[args.compensate]
    compensation = GravityCompensation(joints=joints)    # 보상은 시뮬레이터 안에서. ROS 2 는 위치만 보낸다
    log, mirror, clock = [], np.array(MIRROR, dtype=float), sim.time
    if args.log:
        def record(dt):                                  # 오른팔의 값은 부호를 왼팔에 맞춘다
            log.append((sim.time - clock, compensation.state(LEFT_ARM), [value * mirror for value in compensation.state(RIGHT_ARM)]))
        record_sub = omni.physx.get_physx_interface().subscribe_physics_step_events(record)
    print(f"준비됨: 중력 보상 = {args.compensate}" + (", effort 칸을 이음" if args.wire_effort else "") + ". 끝내려면 Ctrl+C", flush=True)
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
    compensation.stop()
    sim.stop()
    app.close()
