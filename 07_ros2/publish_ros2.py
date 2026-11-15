"""강의 7: SimpleRobot 의 라이다와 카메라를 ROS 2 토픽으로 내보냅니다.

    source /opt/ros/jazzy/setup.bash                               # Isaac Sim 보다 먼저
    ~/isaacsim/python.sh 07_ros2/publish_ros2.py                   # 창 없이 계속 내보냄 (Ctrl+C 로 끝)
    ~/isaacsim/python.sh 07_ros2/publish_ros2.py --keyboard        # 창을 띄우고 W A S D 로 조종
    ~/isaacsim/python.sh 07_ros2/publish_ros2.py --seconds 30      # 30초 뒤 스스로 끝
    ~/isaacsim/python.sh 07_ros2/publish_ros2.py --save work/simple_robot.usd

    나가는 토픽   /scan_lidar  sensor_msgs/msg/LaserScan  frame_id = sim_lidar
                  /rgb         sensor_msgs/msg/Image      frame_id = sim_camera  (640 × 480)

GUI 에서는 Action Graph 창에 노드를 끌어다 놓고 선으로 잇습니다. 코드로는 og.Controller.edit 에 노드 · 값 · 연결을 적습니다.
"""
import argparse
import os
import sys

from isaacsim import SimulationApp

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--seconds", type=float, help="이 시간이 지나면 끝낸다 (없으면 계속)")
    parser.add_argument("--keyboard", action="store_true", help="창을 띄우고 W A S D 로 조종한다")
    parser.add_argument("--gui", action="store_true", help="창을 띄운다")
    parser.add_argument("--save", metavar="USD", help="Play 전 장면을 이 경로에 저장한다")
    args = parser.parse_args()
    if "ROS_DISTRO" not in os.environ:
        sys.exit("ROS 2 환경이 없습니다. 이 터미널에서 먼저 실행하세요:  source /opt/ros/jazzy/setup.bash")
    app = SimulationApp({"headless": not (args.gui or args.keyboard)})

import omni.graph.core as og
from isaacsim.core.utils.extensions import enable_extension

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
for folder in ("04_build_robot", "05_control", "06_sensors"):
    sys.path.insert(0, os.path.join(HERE, "..", folder))
from add_sensors import CAMERA, LIDAR, add_camera, add_lidar, add_obstacles, add_sensor_mount
from build_robot import build_robot
from common.sim import Sim, new_scene, save_stage, set_view
from drive_robot import add_articulation_root, add_differential_controller, add_keyboard_control

SCAN_TOPIC, SCAN_FRAME = "scan_lidar", "sim_lidar"
IMAGE_TOPIC, IMAGE_FRAME = "rgb", "sim_camera"
# Read Lidar Beams 의 출력을 Publish Laser Scan 의 같은 이름 입력에 하나씩 잇는다.
LIDAR_FIELDS = ("azimuthRange", "depthRange", "horizontalFov", "horizontalResolution", "intensitiesData",
                "linearDepthData", "numCols", "numRows", "rotationRate")


def add_lidar_publisher():
    """라이다 → LaserScan. 그래프: Tick → Read Lidar Beams → Publish Laser Scan (+ 시뮬레이션 시간)."""
    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": "/World/ROS2_LidarStream", "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("ReadLidar", "isaacsim.sensors.physx.IsaacReadLidarBeams"),
                ("PublishScan", "isaacsim.ros2.bridge.ROS2PublishLaserScan"),
                ("SimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
            ],
            keys.SET_VALUES: [
                ("ReadLidar.inputs:lidarPrim", LIDAR),
                ("PublishScan.inputs:topicName", SCAN_TOPIC),
                ("PublishScan.inputs:frameId", SCAN_FRAME),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "ReadLidar.inputs:execIn"),
                ("ReadLidar.outputs:execOut", "PublishScan.inputs:execIn"),
                ("SimTime.outputs:simulationTime", "PublishScan.inputs:timeStamp"),
            ] + [(f"ReadLidar.outputs:{name}", f"PublishScan.inputs:{name}") for name in LIDAR_FIELDS],
        },
    )


def add_camera_publisher(width=640, height=480):
    """카메라 → Image. 그래프: Tick → Create Render Product → ROS2 Camera Helper."""
    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": "/World/ROS2_Camera", "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("RenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),   # 카메라 화면을 그리는 곳
                ("CameraHelper", "isaacsim.ros2.bridge.ROS2CameraHelper"),
            ],
            keys.SET_VALUES: [
                ("RenderProduct.inputs:cameraPrim", CAMERA),
                ("RenderProduct.inputs:width", width),
                ("RenderProduct.inputs:height", height),
                ("CameraHelper.inputs:type", "rgb"),
                ("CameraHelper.inputs:topicName", IMAGE_TOPIC),
                ("CameraHelper.inputs:frameId", IMAGE_FRAME),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "RenderProduct.inputs:execIn"),
                ("RenderProduct.outputs:execOut", "CameraHelper.inputs:execIn"),
                ("RenderProduct.outputs:renderProductPath", "CameraHelper.inputs:renderProductPath"),
            ],
        },
    )


if __name__ == "__main__":
    enable_extension("isaacsim.ros2.bridge")     # ROS 2 노드들이 들어 있는 익스텐션
    app.update()

    stage = new_scene()
    build_robot(stage)
    add_articulation_root(stage)
    add_differential_controller()
    if args.keyboard:
        add_keyboard_control()
    add_sensor_mount(stage)
    add_camera(stage)
    add_lidar(stage, draw=args.gui or args.keyboard)
    add_obstacles(stage)
    add_lidar_publisher()
    add_camera_publisher()
    if args.save:
        save_stage(stage, args.save)

    if args.gui or args.keyboard:
        set_view(eye=(-5.0, -6.0, 5.0), target=(3.0, 0.0, 0.3))

    sim = Sim(app)
    sim.play()
    print(f"토픽을 내보내는 중: /{SCAN_TOPIC} (frame_id {SCAN_FRAME}), /{IMAGE_TOPIC} (frame_id {IMAGE_FRAME})", flush=True)
    if args.keyboard:
        print("뷰포트를 한 번 클릭한 뒤 W(앞) S(뒤) A(왼쪽) D(오른쪽) 로 조종합니다. 창을 닫으면 끝납니다.", flush=True)
    else:
        print("끝내려면 Ctrl+C 를 누릅니다.", flush=True)
    try:
        if args.seconds:
            sim.run(args.seconds)
        else:
            sim.wait_until_closed()
    except KeyboardInterrupt:
        pass
    sim.stop()
    app.close()
