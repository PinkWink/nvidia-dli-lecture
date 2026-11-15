"""강의 5: SimpleRobot 을 속도 명령으로 조종합니다 (Articulation Root + Differential Controller 그래프).

    ~/isaacsim/python.sh 05_control/drive_robot.py                              # 직진 1 m/s 로 3초
    ~/isaacsim/python.sh 05_control/drive_robot.py --linear 0 --angular 1       # 왼쪽으로 돌기 1 rad/s
    ~/isaacsim/python.sh 05_control/drive_robot.py --linear 1 --angular 0.5     # 왼쪽으로 원을 그리며
    ~/isaacsim/python.sh 05_control/drive_robot.py --keyboard                   # 창을 띄우고 W A S D 로 조종
    ~/isaacsim/python.sh 05_control/drive_robot.py --keyboard --save work/simple_robot.usd

강의 4 의 뒷바퀴 Target Velocity 를 사람이 넣는 대신, 그래프가 (직진 속도 v, 회전 속도 ω) 를 바퀴 속도로 바꿔 넣습니다.
    SimpleRobot > Add > Physics > Articulation Root                   → UsdPhysics.ArticulationRootAPI
    Tools > Robotics > OmniGraph Controllers > Differential Controller → og.Controller.edit(...) 로 같은 그래프
"""
import argparse
import os
import sys

from isaacsim import SimulationApp

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--linear", type=float, default=1.0, help="직진 속도 v [m/s] (+ 앞)")
    parser.add_argument("--angular", type=float, default=0.0, help="회전 속도 ω [rad/s] (+ 왼쪽으로 돌기)")
    parser.add_argument("--seconds", type=float, default=3.0, help="시뮬레이션할 시간 [s]")
    parser.add_argument("--keyboard", action="store_true", help="창을 띄우고 W A S D 로 조종한다")
    parser.add_argument("--key-linear", type=float, default=1.5, help="W · S 키가 주는 속도 [m/s]")
    parser.add_argument("--key-angular", type=float, default=6.0, help="A · D 키가 주는 속도 [rad/s]")
    parser.add_argument("--gui", action="store_true", help="창을 띄운다 (끝나도 닫을 때까지 유지)")
    parser.add_argument("--save", metavar="USD", help="Play 전 장면을 이 경로에 저장한다")
    args = parser.parse_args()
    app = SimulationApp({"headless": not (args.gui or args.keyboard)})

import omni.graph.core as og
from isaacsim.core.utils.extensions import enable_extension
from pxr import UsdPhysics

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
sys.path.insert(0, os.path.join(HERE, "..", "04_build_robot"))
from build_robot import BODY, ROBOT, WHEEL_DISTANCE, WHEEL_RADIUS, build_robot, print_header, print_pose
from common.sim import Sim, new_scene, pose_of, save_stage, set_view

GRAPH = "/Graphs/differential_controller"
# Differential Controller 는 바퀴 속도를 [왼쪽, 오른쪽] 순서로 냅니다. 관절 이름도 같은 순서로 적습니다.
JOINT_NAMES = ["Rear_Left_Joint", "Rear_Right_Joint"]


def add_articulation_root(stage):
    """관절로 이어진 강체들을 하나의 로봇으로 묶는다. 제어기가 관절을 이름으로 찾을 수 있게 된다."""
    UsdPhysics.ArticulationRootAPI.Apply(stage.GetPrimAtPath(ROBOT))


def add_differential_controller(linear=0.0, angular=0.0):
    """(v, ω) → 왼 · 오 바퀴 속도 → 관절. 메뉴가 만드는 그래프와 같은 구성."""
    enable_extension("isaacsim.robot.wheeled_robots")       # Differential Controller 노드가 들어 있는 익스텐션
    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": GRAPH, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),                         # 매 프레임 실행 신호
                ("DifferentialController", "isaacsim.robot.wheeled_robots.DifferentialController"),
                ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),  # 속도를 관절에 줌
            ],
            keys.SET_VALUES: [
                ("DifferentialController.inputs:wheelRadius", WHEEL_RADIUS),
                ("DifferentialController.inputs:wheelDistance", WHEEL_DISTANCE),
                ("DifferentialController.inputs:linearVelocity", linear),
                ("DifferentialController.inputs:angularVelocity", angular),
                ("ArticulationController.inputs:targetPrim", ROBOT),
                ("ArticulationController.inputs:jointNames", JOINT_NAMES),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "DifferentialController.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
                ("DifferentialController.outputs:velocityCommand", "ArticulationController.inputs:velocityCommand"),
            ],
        },
    )


def add_keyboard_control(key_linear=1.5, key_angular=6.0):
    """W − S → 직진 속도, A − D → 회전 속도. 키를 누르고 있는 동안만 값이 나간다."""
    keys = og.Controller.Keys
    nodes, values, connects = [], [], []
    for key in "WASD":
        nodes += [(key, "omni.graph.nodes.ReadKeyboardState"), (f"ToDouble{key}", "omni.graph.nodes.ToDouble")]
        values += [(f"{key}.inputs:key", key)]
        connects += [(f"{key}.outputs:isPressed", f"ToDouble{key}.inputs:value")]
    for name, plus, minus, scale, target in (("Linear", "W", "S", key_linear, "linearVelocity"),
                                             ("Angular", "A", "D", key_angular, "angularVelocity")):
        nodes += [(f"Sub{name}", "omni.graph.nodes.Subtract"), (f"Scale{name}", "omni.graph.nodes.ConstantDouble"),
                  (f"Speed{name}", "omni.graph.nodes.Multiply")]
        values += [(f"Scale{name}.inputs:value", scale)]
        connects += [(f"ToDouble{plus}.outputs:converted", f"Sub{name}.inputs:a"),
                     (f"ToDouble{minus}.outputs:converted", f"Sub{name}.inputs:b"),
                     (f"Sub{name}.outputs:difference", f"Speed{name}.inputs:a"),
                     (f"Scale{name}.inputs:value", f"Speed{name}.inputs:b"),
                     (f"Speed{name}.outputs:product", f"{GRAPH}/DifferentialController.inputs:{target}")]
    og.Controller.edit(GRAPH, {keys.CREATE_NODES: nodes, keys.SET_VALUES: values, keys.CONNECT: connects})


def set_velocity(linear, angular):
    """그래프의 Desired Linear · Angular Velocity 를 바꾼다."""
    og.Controller.attribute(GRAPH + "/DifferentialController.inputs:linearVelocity").set(linear)
    og.Controller.attribute(GRAPH + "/DifferentialController.inputs:angularVelocity").set(angular)


if __name__ == "__main__":
    stage = new_scene()
    build_robot(stage)
    add_articulation_root(stage)
    if args.keyboard:
        add_differential_controller()
        add_keyboard_control(args.key_linear, args.key_angular)
    else:
        add_differential_controller(args.linear, args.angular)
    if args.save:
        save_stage(stage, args.save)

    if args.gui or args.keyboard:
        set_view(eye=(4.0, -8.0, 6.0), target=(2.0, 0.0, 0.3))

    body = stage.GetPrimAtPath(BODY)
    sim = Sim(app)
    sim.play()
    if args.keyboard:
        print("뷰포트를 한 번 클릭한 뒤 W(앞) S(뒤) A(왼쪽) D(오른쪽) 로 조종합니다. 창을 닫으면 끝납니다.", flush=True)
        sim.wait_until_closed()
    else:
        print_header()
        print_pose(sim.time, body)
        sim.run(args.seconds, every=0.5, report=lambda t: print_pose(t, body))
        x, y, z, yaw = pose_of(body)
        print(f"결과: v = {args.linear:g} m/s, ω = {args.angular:g} rad/s 로 {args.seconds:g}초 → "
              f"위치 ({x:.2f}, {y:.2f}) m, 방향 {yaw:+.1f}°.", flush=True)
        if args.gui:
            set_velocity(0.0, 0.0)                      # 화면 밖으로 나가지 않게 멈춘다
            sim.wait_until_closed()
    sim.stop()
    app.close()
