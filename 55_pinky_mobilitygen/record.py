"""강의 55: 핑크시티에서 Pinky Pro 가 스스로 돌아다니는 것을 기록합니다. MobilityGen 의 창에서 Build → Start Recording 을 누르는 일을 코드로 합니다.

    ~/isaacsim/python.sh 55_pinky_mobilitygen/record.py                      # 무작위 목표까지 길을 찾아 따라가기, 60초
    ~/isaacsim/python.sh 55_pinky_mobilitygen/record.py --seconds 120
    ~/isaacsim/python.sh 55_pinky_mobilitygen/record.py --scenario RandomAccelerationScenario     # 속도를 무작위로 바꾸며 다니기
    ~/isaacsim/python.sh 55_pinky_mobilitygen/record.py --gui                # 창으로 보기

기록할 때는 물리만 돌리고 로봇의 상태만 저장한다 (자리 · 관절 · 속도 명령). 카메라 영상은 replay.py 가 나중에 그린다.
로봇이 목표에 닿거나 장애물에 부딪히면 새 자리에 다시 놓이고, 그때마다 새 기록이 시작된다.

결과: work/mobilitygen/recordings/<날짜_시각>/  (state/common/*.npy · occupancy_map/ · stage.usd · config.json)
"""
import argparse
import datetime
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
WORK = os.path.abspath(os.path.join(HERE, "..", "work", "mobilitygen"))

parser = argparse.ArgumentParser()
parser.add_argument("--seconds", type=float, default=60.0, help="기록할 시간 (시뮬레이터의 시간) [s]")
parser.add_argument("--scenario", default="RandomPathFollowingScenario", choices=["RandomPathFollowingScenario", "RandomAccelerationScenario"])
parser.add_argument("--out", default=os.path.join(WORK, "recordings"), help="기록을 둘 폴더")
parser.add_argument("--margin", type=float, help="길을 찾을 때 장애물을 부풀리는 폭 [m] (기본 0.14)")
parser.add_argument("--seed", type=int, default=0, help="무작위 수의 씨앗 (같으면 같은 자리 · 같은 목표가 나온다)")
parser.add_argument("--gui", action="store_true", help="창을 띄운다")
parser.add_argument("--relative-textures", action="store_true", help="비교용: 장면의 사본을 Isaac Sim 의 save_stage 로 만든다 (재생하면 바닥 그림이 사라진다, 강의 55 의 5절)")
args = parser.parse_args()

from isaacsim import SimulationApp
app = SimulationApp({"headless": not args.gui})

from isaacsim.core.utils.extensions import enable_extension
enable_extension("isaacsim.replicator.mobility_gen.examples")     # MobilityGen 은 Isaac Sim 에 든 익스텐션이다. 스크립트에서는 켜 줘야 불러올 수 있다
app.update()

import random

import numpy as np
import omni.usd
from isaacsim.core.api import objects
from isaacsim.core.utils.stage import open_stage, save_stage
from isaacsim.replicator.mobility_gen.examples.robots import ROBOTS
from isaacsim.replicator.mobility_gen.examples.scenarios import SCENARIOS
from isaacsim.replicator.mobility_gen.impl.config import Config
from isaacsim.replicator.mobility_gen.impl.occupancy_map import OccupancyMap
from isaacsim.replicator.mobility_gen.impl.utils.global_utils import new_world
from isaacsim.replicator.mobility_gen.impl.writer import MobilityGenWriter

sys.path.insert(0, HERE)
import pinky_robot  # noqa: F401  (불러오면 로봇 목록에 PinkyRobot 이 등록된다)

random.seed(args.seed)
np.random.seed(args.seed)
if args.margin is not None:
    pinky_robot.PinkyRobot.occupancy_map_radius = args.margin
city, robot_type, scenario_type = os.path.join(WORK, "pink_city.usd"), ROBOTS.get("PinkyRobot"), SCENARIOS.get(args.scenario)
config = Config(scenario_type=args.scenario, robot_type="PinkyRobot", scene_usd=city)
occupancy_map = OccupancyMap.from_ros_yaml(os.path.join(WORK, "map", "map.yaml"))

# 창의 Build 버튼이 하는 일: 장면 열기 → 물리 준비 → 로봇 놓기 → 시나리오 만들기
open_stage(city)
cached_stage = os.path.join(WORK, "stage_cache.usd")
if args.relative_textures:                                        # 비교용: 창의 Build 버튼이 쓰는 저장 함수. 그림 파일의 경로가 상대 경로로 바뀐다
    save_stage(cached_stage, save_and_reload_in_place=False)
else:                                                             # 기록마다 장면의 사본을 함께 넣는다 (재생할 때 쓴다). 경로를 적힌 그대로 둔다
    omni.usd.get_context().get_stage().GetRootLayer().Export(cached_stage)
world = new_world(physics_dt=robot_type.physics_dt)
world.initialize_physics()
objects.GroundPlane("/World/ground_plane", visible=False)
robot = robot_type.build("/World/robot")
scenario = scenario_type.from_robot_occupancy_map(robot, occupancy_map)
world.reset()

recordings = []


def new_recording():
    """창의 Start Recording 이 하는 일: 새 폴더를 만들고 설정 · 지도 · 장면을 적어 둔다."""
    path = os.path.join(args.out, datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S-%f"))
    writer = MobilityGenWriter(path)
    writer.write_config(config)
    writer.write_occupancy_map(scenario.occupancy_map)
    writer.copy_stage(cached_stage)
    recordings.append([path, 0, 0.0])
    return writer


scenario.reset()
writer, step, started = new_recording(), 0, time.time()
steps = int(args.seconds / robot_type.physics_dt)
for count in range(steps):
    world.step(render=args.gui and count % 8 == 0)
    alive = scenario.step(robot_type.physics_dt)                   # 속도 명령을 정하고 바퀴에 넣는다. 끝났으면(도착 · 충돌) False
    if not alive:
        scenario.reset()                                          # 새 자리에 다시 놓는다 → 새 기록
        writer, step = new_recording(), 0
    writer.write_state_dict_common(scenario.state_dict_common(), step=step)
    step += 1
    recordings[-1][1], recordings[-1][2] = step, step * robot_type.physics_dt

print(f"기록 {len(recordings)}개, 시뮬레이터의 시간 {args.seconds:g}초를 {time.time() - started:.0f}초에 돌림  → {args.out}", flush=True)
for path, count, seconds in recordings:
    print(f"  {os.path.basename(path)}   {count:>6} 스텝 · {seconds:5.1f}초", flush=True)
app.close()
