"""강의 48: 도킹한 자리에서 팔이 닿는 범위를 잽니다. 팔은 움직이지 않고 MoveIt 에 계획만 물어봅니다.

    source /opt/ros/jazzy/setup.bash
    source ~/pinklab_robotworld2026/install/setup.bash
    python3 48_pl2m_find_pick/reach_map.py                      # 음료대에 도킹한 자리에서 (시뮬레이터를 --start drinks --docked 로)
    python3 48_pl2m_find_pick/reach_map.py --step 0.01 0.02     # 앞뒤 1 cm · 좌우 2 cm 간격으로 촘촘하게
    python3 48_pl2m_find_pick/reach_map.py --json reach.json    # 결과를 저장

컵이 로봇 기준 (앞, 왼쪽) 에 있다고 하고, 강의 46 의 "집기 전 확인" 과 같은 것을 물어본다:
운반 자세 → 뒤쪽 위 → 컵 위 → 컵 → 접근 시작 자리가 직선으로 끝까지 이어지는가. 이어지면 O, 아니면 . 을 찍는다.
"""
import argparse
import json
import os
import sys

import rclpy

HERE = os.path.dirname(os.path.abspath(__file__))
for folder in ("42_pl2m_moveit", "43_pl2m_pick", "44_pl2m_nav", "45_pl2m_dock", "46_pl2m_mission"):
    sys.path.insert(0, os.path.join(HERE, "..", folder))
from booth import STATIONS
from layout import CARRY_TIP, CUP_DIAMETER, CUP_HEIGHT, GRASP
from mission import Mission, shape


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--station", default="drinks", choices=["drinks", "consult_1", "consult_2"])
    parser.add_argument("--ahead", type=float, nargs=2, default=(0.46, 0.59), metavar=("FROM", "TO"), help="로봇의 앞쪽으로 재는 범위 [m]")
    parser.add_argument("--left", type=float, nargs=2, default=(-0.30, 0.30), metavar=("FROM", "TO"), help="왼쪽으로 재는 범위 [m]")
    parser.add_argument("--step", type=float, nargs=2, default=(0.03, 0.03), metavar=("AHEAD", "LEFT"), help="간격 [m]")
    parser.add_argument("--json", metavar="FILE", help="결과를 이 파일에 저장한다")
    args = parser.parse_args()

    rclpy.init()
    mission = Mission()
    for side in ("left", "right"):
        mission.arm.grip(side, GRASP["open"])
    z = STATIONS[args.station]["top"] + CUP_HEIGHT / 2.0
    tip_z, lift = z + GRASP["z_offset"], z + GRASP["z_offset"] + GRASP["lift_z"]
    count = lambda low, high, step: int(round((high - low) / step)) + 1
    aheads = [round(args.ahead[0] + args.step[0] * i, 3) for i in range(count(*args.ahead, args.step[0]))]
    lefts = [round(args.left[1] - args.step[1] * i, 3) for i in range(count(*args.left, args.step[1]))]        # 왼쪽부터 찍는다
    furniture = mission.add_furniture(args.station, (0.0, 0.0))
    rows = []
    print(f"가로: 왼쪽 {lefts[0]:+.2f} m → {lefts[-1]:+.2f} m ({args.step[1] * 100:.0f} cm 간격)", flush=True)
    for x in aheads:
        marks = {"left": "", "right": ""}
        for y in lefts:
            mission.arm.apply(world=[shape("cup_probe", "cylinder", (x, y, z), (CUP_DIAMETER / 2, CUP_HEIGHT))])
            back = max(x + GRASP["retreat_x"], CARRY_TIP[0] + 0.03)
            way_back = [(back, y, lift), (x, y, lift), (x, y, tip_z), (x + GRASP["pre_offset_x"], y, tip_z)]
            row = {"x": x, "y": y}
            for side in ("left", "right"):
                row[side], _, _ = mission.plan_line(side, way_back, start=mission.joints(side, "carry"))
                marks[side] += "O" if row[side] >= 0.999 else "."
            rows.append(row)
        print(f"앞 {x:.2f} m   왼팔 {marks['left']}   오른팔 {marks['right']}", flush=True)
    mission.remove(furniture + ["cup_probe"])
    if args.json:
        with open(args.json, "w") as file:
            json.dump(rows, file)
        print("저장:", args.json)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
