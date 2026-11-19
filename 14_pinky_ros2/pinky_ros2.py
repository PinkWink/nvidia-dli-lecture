"""강의 14: Pinky Pro 를 ROS 2 에 연결합니다 — /cmd_vel 을 받아 움직이고, 센서 · 위치를 토픽으로 내보냅니다.

    source /opt/ros/jazzy/setup.bash                                   # 먼저 ROS 2 환경을 켭니다
    ~/isaacsim/python.sh 14_pinky_ros2/pinky_ros2.py                   # 창 없이 실행 (Ctrl+C 로 끝냄)
    ~/isaacsim/python.sh 14_pinky_ros2/pinky_ros2.py --gui             # 창으로 보기
    ~/isaacsim/python.sh 14_pinky_ros2/pinky_ros2.py --seconds 30      # 30초 뒤 끝냄
    ~/isaacsim/python.sh 14_pinky_ros2/pinky_ros2.py --no-sensors      # /cmd_vel · /odom · /tf 만 (가볍게)

다른 터미널에서
    ros2 topic list
    ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"
    ros2 run teleop_twist_keyboard teleop_twist_keyboard

토픽 (공식 저장소 https://github.com/pinklab-art/pinky_pro 의 이름을 따랐습니다)
    받기    /cmd_vel            geometry_msgs/Twist     → 차동 제어기 → 바퀴
    내보내기 /odom · /tf         nav_msgs/Odometry · tf  ← 로봇의 위치
            /joint_states       sensor_msgs/JointState  ← 바퀴 각도
            /scan               sensor_msgs/LaserScan   ← 라이다
            /camera/image_raw   sensor_msgs/Image       ← 카메라
            /imu                sensor_msgs/Imu         ← IMU
            /clock              rosgraph_msgs/Clock     ← 시뮬레이션 시간
"""
import argparse
import os
import sys

from isaacsim import SimulationApp

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--seconds", type=float, help="이 시간이 지나면 끝냄 (없으면 Ctrl+C 까지)")
    parser.add_argument("--no-sensors", action="store_true", help="센서 토픽(/scan · /camera · /imu)을 만들지 않는다")
    parser.add_argument("--gui", action="store_true", help="창을 띄운다")
    parser.add_argument("--save", metavar="USD", help="Play 전 장면을 이 경로에 저장한다")
    args = parser.parse_args()
    if "ROS_DISTRO" not in os.environ:
        print("ROS 2 환경이 없습니다. 같은 터미널에서 먼저 실행하세요:  source /opt/ros/jazzy/setup.bash")
        sys.exit(1)
    # 창 없이 돌리면 시뮬레이션이 실제 시간보다 빨리 갑니다. 1초에 60프레임으로 맞춰 실제 시간과 같게 합니다.
    app = SimulationApp({"headless": not args.gui, "extra_args": [
        "--/app/runLoops/main/rateLimitEnabled=true", "--/app/runLoops/main/rateLimitFrequency=60"]})

import omni.graph.core as og
from isaacsim.core.utils.extensions import enable_extension
from pxr import PhysxSchema

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
sys.path.insert(0, os.path.join(HERE, "..", "11_pinky_import"))
sys.path.insert(0, os.path.join(HERE, "..", "12_pinky_sensors"))
from common.sim import Sim, new_scene, pose_of, save_stage, set_view
from import_pinky import (BODY, DEFAULT_URDF, MAX_ANGULAR, MAX_LINEAR, ROBOT, WHEEL_DISTANCE, WHEEL_JOINTS, WHEEL_RADIUS, find_urdf,
                          import_pinky)
from pinky_sensors import (CAMERA, CAMERA_LINK, IMU, IMU_LINK, LIDAR, LIDAR_LINK, add_camera, add_imu, add_lidar, add_obstacles)

BASE = ROBOT + "/base_footprint"          # 로봇의 기준 링크 (바닥에 닿는 점). Articulation Root 가 여기 있다
DRIVE_GRAPH = "/Graphs/ros2_cmd_vel"
STATE_GRAPH = "/Graphs/ros2_state"
SENSOR_GRAPH = "/Graphs/ros2_sensors"
PHYSICS_HZ = 240                          # 물리 계산을 1초에 240번 (기본 60). 작은 로봇이 서 있을 때 천천히 도는 오차를 줄인다
LIDAR_FIELDS = ("azimuthRange", "depthRange", "horizontalFov", "horizontalResolution", "intensitiesData",
                "linearDepthData", "numCols", "numRows", "rotationRate")


def add_cmd_vel_subscriber():
    """/cmd_vel (Twist) → 직진 속도 x · 회전 속도 z → 차동 제어기 → 바퀴 관절.

    강의 11 의 키보드 그래프에서 "키보드" 자리를 "ROS 2 구독" 으로 바꾼 것입니다.
    """
    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": DRIVE_GRAPH, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),                    # ROS_DOMAIN_ID 등 ROS 2 설정
                ("SubscribeTwist", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),      # /cmd_vel 받기
                ("BreakLinear", "omni.graph.nodes.BreakVector3"),                   # (x, y, z) 에서 x 만
                ("BreakAngular", "omni.graph.nodes.BreakVector3"),                  # (x, y, z) 에서 z 만
                ("DifferentialController", "isaacsim.robot.wheeled_robots.DifferentialController"),
                ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
            ],
            keys.SET_VALUES: [
                ("SubscribeTwist.inputs:topicName", "cmd_vel"),
                ("DifferentialController.inputs:wheelRadius", WHEEL_RADIUS),
                ("DifferentialController.inputs:wheelDistance", WHEEL_DISTANCE),
                ("DifferentialController.inputs:maxLinearSpeed", MAX_LINEAR),
                ("DifferentialController.inputs:maxAngularSpeed", MAX_ANGULAR),
                ("ArticulationController.inputs:targetPrim", ROBOT),
                ("ArticulationController.inputs:jointNames", WHEEL_JOINTS),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "SubscribeTwist.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
                ("Context.outputs:context", "SubscribeTwist.inputs:context"),
                ("OnPlaybackTick.outputs:tick", "DifferentialController.inputs:execIn"),   # 메시지가 없어도 매 프레임 계산
                ("SubscribeTwist.outputs:linearVelocity", "BreakLinear.inputs:tuple"),
                ("SubscribeTwist.outputs:angularVelocity", "BreakAngular.inputs:tuple"),
                ("BreakLinear.outputs:x", "DifferentialController.inputs:linearVelocity"),
                ("BreakAngular.outputs:z", "DifferentialController.inputs:angularVelocity"),
                ("DifferentialController.outputs:velocityCommand", "ArticulationController.inputs:velocityCommand"),
            ],
        },
    )


def add_state_publishers():
    """로봇의 상태 내보내기: /clock · /odom · /tf (odom → base_footprint → 각 링크) · /joint_states."""
    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": STATE_GRAPH, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("SimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
                ("ComputeOdometry", "isaacsim.core.nodes.IsaacComputeOdometry"),     # 출발점에서 본 위치 · 속도
                ("PublishOdometry", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
                ("PublishOdomTF", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),   # tf: odom → base_footprint
                ("PublishLinkTF", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),      # tf: base_footprint → 링크 · 센서 자리
                ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
            ],
            keys.SET_VALUES: [
                ("ComputeOdometry.inputs:chassisPrim", BASE),
                ("PublishOdometry.inputs:topicName", "odom"),
                ("PublishOdometry.inputs:odomFrameId", "odom"),
                ("PublishOdometry.inputs:chassisFrameId", "base_footprint"),
                ("PublishOdomTF.inputs:parentFrameId", "odom"),
                ("PublishOdomTF.inputs:childFrameId", "base_footprint"),
                ("PublishLinkTF.inputs:parentPrim", BASE),
                ("PublishLinkTF.inputs:targetPrims", [ROBOT, LIDAR_LINK, CAMERA_LINK, IMU_LINK]),
                ("PublishJointState.inputs:targetPrim", BASE),               # Articulation Root 가 있는 Prim
                ("PublishJointState.inputs:topicName", "joint_states"),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "ComputeOdometry.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "PublishLinkTF.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
                ("ComputeOdometry.outputs:execOut", "PublishOdometry.inputs:execIn"),
                ("ComputeOdometry.outputs:execOut", "PublishOdomTF.inputs:execIn"),
                ("ComputeOdometry.outputs:position", "PublishOdometry.inputs:position"),
                ("ComputeOdometry.outputs:orientation", "PublishOdometry.inputs:orientation"),
                ("ComputeOdometry.outputs:linearVelocity", "PublishOdometry.inputs:linearVelocity"),
                ("ComputeOdometry.outputs:angularVelocity", "PublishOdometry.inputs:angularVelocity"),
                ("ComputeOdometry.outputs:position", "PublishOdomTF.inputs:translation"),
                ("ComputeOdometry.outputs:orientation", "PublishOdomTF.inputs:rotation"),
            ] + [("Context.outputs:context", f"{node}.inputs:context")
                 for node in ("PublishClock", "PublishOdometry", "PublishOdomTF", "PublishLinkTF", "PublishJointState")]
              + [("SimTime.outputs:simulationTime", f"{node}.inputs:timeStamp")
                 for node in ("PublishClock", "PublishOdometry", "PublishOdomTF", "PublishLinkTF", "PublishJointState")],
        },
    )


def add_sensor_publishers(width=640, height=360):
    """센서 내보내기: /scan (라이다) · /camera/image_raw (카메라) · /imu (IMU)."""
    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": SENSOR_GRAPH, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("SimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("ReadLidar", "isaacsim.sensors.physx.IsaacReadLidarBeams"),
                ("PublishScan", "isaacsim.ros2.bridge.ROS2PublishLaserScan"),
                ("RenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("CameraHelper", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("ReadIMU", "isaacsim.sensors.physics.IsaacReadIMU"),
                ("PublishImu", "isaacsim.ros2.bridge.ROS2PublishImu"),
            ],
            keys.SET_VALUES: [
                ("ReadLidar.inputs:lidarPrim", LIDAR),
                ("PublishScan.inputs:topicName", "scan"),
                ("PublishScan.inputs:frameId", "rplidar_link"),
                ("RenderProduct.inputs:cameraPrim", CAMERA),
                ("RenderProduct.inputs:width", width),
                ("RenderProduct.inputs:height", height),
                ("CameraHelper.inputs:type", "rgb"),
                ("CameraHelper.inputs:topicName", "camera/image_raw"),
                ("CameraHelper.inputs:frameId", "front_camera_link"),
                ("ReadIMU.inputs:imuPrim", IMU),
                ("PublishImu.inputs:topicName", "imu"),
                ("PublishImu.inputs:frameId", "imu_link"),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "ReadLidar.inputs:execIn"),
                ("ReadLidar.outputs:execOut", "PublishScan.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "RenderProduct.inputs:execIn"),
                ("RenderProduct.outputs:execOut", "CameraHelper.inputs:execIn"),
                ("RenderProduct.outputs:renderProductPath", "CameraHelper.inputs:renderProductPath"),
                ("OnPlaybackTick.outputs:tick", "ReadIMU.inputs:execIn"),
                ("ReadIMU.outputs:execOut", "PublishImu.inputs:execIn"),
                ("ReadIMU.outputs:angVel", "PublishImu.inputs:angularVelocity"),
                ("ReadIMU.outputs:linAcc", "PublishImu.inputs:linearAcceleration"),
                ("ReadIMU.outputs:orientation", "PublishImu.inputs:orientation"),
                ("SimTime.outputs:simulationTime", "PublishScan.inputs:timeStamp"),
                ("SimTime.outputs:simulationTime", "PublishImu.inputs:timeStamp"),
            ] + [(f"ReadLidar.outputs:{name}", f"PublishScan.inputs:{name}") for name in LIDAR_FIELDS]
              + [("Context.outputs:context", f"{node}.inputs:context") for node in ("PublishScan", "CameraHelper", "PublishImu")],
        },
    )


if __name__ == "__main__":
    enable_extension("isaacsim.ros2.bridge")              # ROS 2 노드들이 들어 있는 익스텐션
    enable_extension("isaacsim.robot.wheeled_robots")
    app.update()

    stage = new_scene()
    PhysxSchema.PhysxSceneAPI.Apply(stage.GetPrimAtPath("/World/PhysicsScene")).CreateTimeStepsPerSecondAttr(PHYSICS_HZ)
    import_pinky(stage, find_urdf(DEFAULT_URDF))
    add_lidar(stage, draw=args.gui)
    add_camera(stage)
    add_imu(stage)
    add_obstacles(stage)
    add_cmd_vel_subscriber()
    add_state_publishers()
    if not args.no_sensors:
        add_sensor_publishers()
    if args.save:
        save_stage(stage, args.save)
    if args.gui:
        set_view(eye=(-0.7, -0.9, 0.8), target=(0.3, 0.0, 0.05))

    topics = "/cmd_vel 받기, /odom · /tf · /joint_states · /clock" + ("" if args.no_sensors else " · /scan · /camera/image_raw · /imu") + " 내보내기"
    print(f"ROS 2 연결: {topics}", flush=True)
    print(f"ROS_DOMAIN_ID = {os.environ.get('ROS_DOMAIN_ID', '0')}. 끝내려면 Ctrl+C 를 누릅니다.", flush=True)
    print(f"{'시간[s]':>8} {'x[m]':>8} {'y[m]':>8} {'방향[°]':>8}", flush=True)

    body = stage.GetPrimAtPath(BODY)
    sim = Sim(app)
    sim.play()

    def report(t):
        x, y, _, yaw = pose_of(body)
        print(f"{t:8.0f} {round(x, 3) + 0.0:8.3f} {round(y, 3) + 0.0:8.3f} {round(yaw, 1) + 0.0:8.1f}", flush=True)

    try:
        if args.seconds:
            sim.run(args.seconds, every=1.0, report=report)
        else:
            next_report = 1.0
            while app.is_running():
                app.update()
                if sim.time >= next_report:
                    report(sim.time)
                    next_report += 1.0
    except KeyboardInterrupt:
        pass
    sim.stop()
    app.close()
