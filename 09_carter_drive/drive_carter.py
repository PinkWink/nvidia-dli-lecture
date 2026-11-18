"""강의 9: URDF 로 가져온 Carter 를 움직입니다 (바퀴 속도 직접 주기 → 차동 제어기 → 속도 제한).

    ~/isaacsim/python.sh 09_carter_drive/drive_carter.py                             # 직진 0.5 m/s 로 3초
    ~/isaacsim/python.sh 09_carter_drive/drive_carter.py --wheels 20 20 --seconds 4  # 두 바퀴에 20 °/s 직접 주기
    ~/isaacsim/python.sh 09_carter_drive/drive_carter.py --wheels 0 20 --seconds 4   # 왼쪽 0, 오른쪽 20 → 왼쪽으로 돎
    ~/isaacsim/python.sh 09_carter_drive/drive_carter.py --linear 0 --angular 0.5    # 제자리에서 왼쪽으로 돌기
    ~/isaacsim/python.sh 09_carter_drive/drive_carter.py --linear 5 --no-limit       # 제한 없이 5 m/s: 너무 빠름
    ~/isaacsim/python.sh 09_carter_drive/drive_carter.py --linear 5                  # 같은 명령, 0.5 m/s 로 제한됨
    ~/isaacsim/python.sh 09_carter_drive/drive_carter.py --keyboard                  # 창을 띄우고 W A S D 로 조종

GUI 의 동작과 코드
    관절의 Drive > Target Velocity                                      → UsdPhysics.DriveAPI 의 targetVelocity
    Tools > Robotics > OmniGraph Controllers > Differential Controller  → og.Controller.edit(...) 로 같은 그래프
    DifferentialController 노드의 maxLinearSpeed · maxAngularSpeed      → 같은 이름의 입력값
"""
import argparse
import os
import sys

from isaacsim import SimulationApp

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--wheels", type=float, nargs=2, metavar=("LEFT", "RIGHT"),
                        help="제어기 없이 왼쪽 · 오른쪽 바퀴의 Target Velocity 를 직접 준다 [°/s]")
    parser.add_argument("--linear", type=float, default=0.5, help="직진 속도 v [m/s] (+ 앞)")
    parser.add_argument("--angular", type=float, default=0.0, help="회전 속도 ω [rad/s] (+ 왼쪽으로 돌기)")
    parser.add_argument("--max-linear", type=float, default=0.5, help="최대 직진 속도 [m/s]")
    parser.add_argument("--max-angular", type=float, default=0.5, help="최대 회전 속도 [rad/s]")
    parser.add_argument("--no-limit", action="store_true", help="속도 제한을 두지 않는다")
    parser.add_argument("--seconds", type=float, default=3.0, help="시뮬레이션할 시간 [s]")
    parser.add_argument("--keyboard", action="store_true", help="창을 띄우고 W A S D 로 조종한다")
    parser.add_argument("--gui", action="store_true", help="창을 띄운다 (끝나도 닫을 때까지 유지)")
    parser.add_argument("--save", metavar="USD", help="Play 전 장면을 이 경로에 저장한다")
    args = parser.parse_args()
    app = SimulationApp({"headless": not (args.gui or args.keyboard)})

import omni.graph.core as og
from isaacsim.core.utils.extensions import enable_extension
from pxr import UsdPhysics

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
sys.path.insert(0, os.path.join(HERE, "..", "08_urdf_import"))
from common.sim import Sim, new_scene, pose_of, save_stage, set_view
from import_carter import BODY, DEFAULT_URDF, JOINTS, ROBOT, find_urdf, import_carter

GRAPH = "/Graphs/differential_controller"
WHEEL_RADIUS = 0.24          # carter.urdf 의 바퀴 충돌 모양: <cylinder radius="0.240">
WHEEL_DISTANCE = 0.62        # 좌우 바퀴 y = ±0.314 → 0.628. 공식 과정 값 0.62 를 쓴다
# Differential Controller 는 바퀴 속도를 [왼쪽, 오른쪽] 순서로 냅니다. 관절 이름도 같은 순서로 적습니다.
JOINT_NAMES = ["left_wheel", "right_wheel"]
KEY_LINEAR = 5.0             # 메뉴가 만드는 키보드 그래프의 기본값: W · S = 5 m/s
KEY_ANGULAR = 3.0            # A · D = 3 rad/s


def set_wheel_velocity(stage, left, right):
    """관절의 Drive > Angular > Target Velocity [°/s]. Property 창에 숫자를 넣는 것과 같다."""
    for name, value in zip(JOINT_NAMES, (left, right)):
        UsdPhysics.DriveAPI(stage.GetPrimAtPath(f"{JOINTS}/{name}"), "angular").GetTargetVelocityAttr().Set(value)


def add_differential_controller(linear=0.0, angular=0.0, max_linear=None, max_angular=None):
    """(v, ω) → 왼 · 오 바퀴 속도 → 관절. max_* 가 None 이면 제한을 두지 않는다 (노드 기본값 0 = 제한 없음)."""
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
                ("DifferentialController.inputs:maxLinearSpeed", max_linear or 0.0),
                ("DifferentialController.inputs:maxAngularSpeed", max_angular or 0.0),
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


def add_keyboard_control(key_linear=KEY_LINEAR, key_angular=KEY_ANGULAR):
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


def wheel_command():
    """제어기가 바퀴에 내보내는 속도 [rad/s] (왼쪽, 오른쪽)."""
    command = og.Controller.attribute(GRAPH + "/DifferentialController.outputs:velocityCommand").get()
    return float(command[0]), float(command[1])


def print_header():
    print(f"{'시간[s]':>8} {'x[m]':>8} {'y[m]':>8} {'z[m]':>8} {'방향[°]':>8}", flush=True)


def print_pose(time, prim):
    x, y, z, yaw = (round(v, 2) + 0.0 for v in pose_of(prim))      # + 0.0: -0.00 을 0.00 으로
    print(f"{time:8.1f} {x:8.2f} {y:8.2f} {z:8.2f} {yaw:8.1f}", flush=True)


if __name__ == "__main__":
    stage = new_scene()
    import_carter(stage, find_urdf(DEFAULT_URDF))
    max_linear = None if args.no_limit else args.max_linear
    max_angular = None if args.no_limit else args.max_angular
    use_controller = args.wheels is None
    if args.keyboard:
        add_differential_controller(max_linear=max_linear, max_angular=max_angular)
        add_keyboard_control()
    elif use_controller:
        add_differential_controller(args.linear, args.angular, max_linear, max_angular)
    else:
        set_wheel_velocity(stage, *args.wheels)
    if args.save:
        save_stage(stage, args.save)
    if args.gui or args.keyboard:
        set_view(eye=(-2.5, -4.0, 3.0), target=(1.0, 0.0, 0.2))

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
        if use_controller:
            left, right = wheel_command()
            limit = "제한 없음" if args.no_limit else f"최대 {args.max_linear:g} m/s · {args.max_angular:g} rad/s"
            print(f"바퀴 명령: 왼쪽 {left:.2f} rad/s, 오른쪽 {right:.2f} rad/s ({limit})", flush=True)
            print(f"결과: v = {args.linear:g} m/s, ω = {args.angular:g} rad/s 로 {args.seconds:g}초 → "
                  f"위치 ({x:.2f}, {y:.2f}) m, 방향 {yaw:+.1f}°.", flush=True)
        else:
            print(f"결과: 왼쪽 {args.wheels[0]:g} °/s, 오른쪽 {args.wheels[1]:g} °/s 로 {args.seconds:g}초 → "
                  f"위치 ({x:.2f}, {y:.2f}) m, 방향 {yaw:+.1f}°.", flush=True)
        if args.gui:
            if use_controller:
                set_velocity(0.0, 0.0)                  # 화면 밖으로 나가지 않게 멈춘다
            else:
                set_wheel_velocity(stage, 0.0, 0.0)
            sim.wait_until_closed()
    sim.stop()
    app.close()
