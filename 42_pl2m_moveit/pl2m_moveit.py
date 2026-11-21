"""강의 42: MoveIt 으로 움직일 PL-2M 을 띄웁니다. 강의 41 의 ROS 2 연결 · 보간기 · 중력 보상을 그대로 씁니다.

    source /opt/ros/jazzy/setup.bash                                     # Isaac Sim 보다 먼저
    ~/isaacsim/python.sh 42_pl2m_moveit/pl2m_moveit.py                   # 로봇만
    ~/isaacsim/python.sh 42_pl2m_moveit/pl2m_moveit.py --obstacle        # 왼팔이 지나는 길에 기둥과 컵을 둔다
    ~/isaacsim/python.sh 42_pl2m_moveit/pl2m_moveit.py --compensate left # 왼팔만 중력 보상 (오른팔과 비교)
    ~/isaacsim/python.sh 42_pl2m_moveit/pl2m_moveit.py --obstacle --gui  # 창으로
"""
import argparse
import math
import os
import signal
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, ".."))

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--obstacle", action="store_true", help="왼팔이 지나는 길에 기둥과 컵을 둔다")
    parser.add_argument("--compensate", choices=["both", "left", "none"], default="both", help="중력 보상을 넣을 팔")
    parser.add_argument("--gui", action="store_true", help="창을 띄운다")
    parser.add_argument("--seconds", type=float, help="이 시간만큼만 돌리고 끝낸다 (없으면 Ctrl+C 나 창을 닫을 때까지)")
    args = parser.parse_args()

    from isaacsim import SimulationApp
    app = SimulationApp({"headless": not args.gui, "extra_args": [
        "--/app/runLoops/main/rateLimitEnabled=true", "--/app/runLoops/main/rateLimitFrequency=60"]})

from pxr import Gf, UsdGeom, UsdPhysics

sys.path.insert(0, HERE)
sys.path.insert(0, REPO)
for folder in ("37_pl2m_import", "39_pl2m_drive", "40_pl2m_gravity", "41_pl2m_ros2"):
    sys.path.insert(0, os.path.join(REPO, folder))
from common.sim import Sim, new_scene, set_pose, set_view
from import_pl2m import DEFAULT_URDF, find_urdf, import_pl2m
from obstacle import CUP_CENTER, CUP_HEIGHT, CUP_MASS, CUP_RADIUS, POST_SIZE, POST_XY
from pl2m_drive import ARM_JOINTS
from pl2m_gravity import LEFT_ARM, GravityCompensation, prepare_pl2m
from pl2m_ros2 import JointCommandInterpolator, add_ros2, enable_ros2

CUP = "/World/Cup"


def add_obstacle(stage):
    """바닥에 선 기둥(고정)과 그 위의 컵(움직이는 물체)."""
    post = UsdGeom.Cube.Define(stage, "/World/Post")
    post.CreateSizeAttr(1.0)
    post.CreateDisplayColorAttr([Gf.Vec3f(0.35, 0.35, 0.38)])
    set_pose(post.GetPrim(), translate=(POST_XY[0], POST_XY[1], POST_SIZE[2] / 2), scale=POST_SIZE)
    UsdPhysics.CollisionAPI.Apply(post.GetPrim())
    cup = UsdGeom.Cylinder.Define(stage, CUP)
    cup.CreateRadiusAttr(CUP_RADIUS)
    cup.CreateHeightAttr(CUP_HEIGHT)
    cup.CreateAxisAttr("Z")
    cup.CreateDisplayColorAttr([Gf.Vec3f(0.93, 0.25, 0.55)])
    set_pose(cup.GetPrim(), translate=CUP_CENTER)
    UsdPhysics.CollisionAPI.Apply(cup.GetPrim())
    UsdPhysics.RigidBodyAPI.Apply(cup.GetPrim())
    UsdPhysics.MassAPI.Apply(cup.GetPrim()).CreateMassAttr(CUP_MASS)


def cup_moved(stage):
    """컵이 처음 자리에서 움직인 거리 [m]."""
    now = UsdGeom.Xformable(stage.GetPrimAtPath(CUP)).ComputeLocalToWorldTransform(0).ExtractTranslation()
    return math.dist(now, CUP_CENTER)


if __name__ == "__main__":
    enable_ros2(app)
    stage = new_scene()
    robot = import_pl2m(stage, find_urdf(DEFAULT_URDF))
    set_pose(robot, translate=(0.0, 0.0, 0.002))
    prepare_pl2m(stage)
    add_ros2()
    if args.obstacle:
        add_obstacle(stage)
    if args.gui:
        set_view(eye=(1.9, 1.7, 1.25), target=(0.2, 0.05, 0.6))
    sim = Sim(app)
    sim.play()
    sim.run(1.0)
    compensation = GravityCompensation(joints={"both": ARM_JOINTS, "left": LEFT_ARM, "none": []}[args.compensate])
    interpolator = JointCommandInterpolator(sim)
    print(f"준비됨: 중력 보상 = {args.compensate}" + (", 장애물 있음" if args.obstacle else "") + ". 끝내려면 Ctrl+C", flush=True)
    stop, clock, told = [], sim.time, 0.0
    signal.signal(signal.SIGINT, lambda *_: stop.append(True))
    while app.is_running() and not stop and (not args.seconds or sim.time - clock < args.seconds):
        app.update()
        if args.obstacle and cup_moved(stage) > max(0.005, told * 2):       # 컵이 움직였으면 알린다
            told = cup_moved(stage)
            print(f"컵이 움직였음: {told * 100:.1f} cm", flush=True)
    if args.obstacle:
        print(f"컵이 처음 자리에서 움직인 거리: {cup_moved(stage) * 1000:.1f} mm", flush=True)
    interpolator.stop()
    compensation.stop()
    sim.stop()
    app.close()
