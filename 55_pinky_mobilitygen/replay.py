"""강의 55: 기록을 다시 틀면서 Pinky Pro 의 카메라 영상을 그립니다. 물리는 돌리지 않고, 기록된 자리에 로봇을 놓고 찍기만 합니다.

    ~/isaacsim/python.sh 55_pinky_mobilitygen/replay.py                      # 기록 전부, 24 스텝마다 한 장 (초당 10장)
    ~/isaacsim/python.sh 55_pinky_mobilitygen/replay.py --every 8            # 더 촘촘히 (초당 30장)
    ~/isaacsim/python.sh 55_pinky_mobilitygen/replay.py --size 1280 720 --out work/mobilitygen/replays_hd      # 같은 주행을 다른 크기로
    ~/isaacsim/python.sh 55_pinky_mobilitygen/replay.py --no-depth           # 색 영상만

결과: work/mobilitygen/replays/<기록 이름>/state/  common/*.npy · rgb/<카메라>/*.jpg · depth/<카메라>/*.png
"""
import argparse
import glob
import os
import shutil
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
WORK = os.path.abspath(os.path.join(HERE, "..", "work", "mobilitygen"))

parser = argparse.ArgumentParser()
parser.add_argument("--input", default=os.path.join(WORK, "recordings"), help="기록들이 든 폴더")
parser.add_argument("--out", default=os.path.join(WORK, "replays"), help="영상을 붙인 기록을 둘 폴더")
parser.add_argument("--every", type=int, default=24, help="몇 스텝마다 한 장 그릴지 (물리는 초당 240 스텝)")
parser.add_argument("--size", type=int, nargs=2, metavar=("W", "H"), help="영상의 크기 (기본 640 360)")
parser.add_argument("--no-depth", action="store_true", help="깊이 영상을 그리지 않는다")
parser.add_argument("--warmup", type=int, default=20, help="기록마다 처음에 그리고 버리는 장 수")
parser.add_argument("--limit", type=int, help="앞에서부터 이 개수의 기록만")
args = parser.parse_args()

from isaacsim import SimulationApp
app = SimulationApp({"headless": True})

from isaacsim.core.utils.extensions import enable_extension
enable_extension("isaacsim.replicator.mobility_gen.examples")
app.update()

import omni.replicator.core as rep
from isaacsim.replicator.mobility_gen.impl.build import load_scenario
from isaacsim.replicator.mobility_gen.impl.reader import MobilityGenReader
from isaacsim.replicator.mobility_gen.impl.utils.global_utils import get_world
from isaacsim.replicator.mobility_gen.impl.writer import MobilityGenWriter

sys.path.insert(0, HERE)
import pinky_robot                                                # 기록의 config.json 에 적힌 "PinkyRobot" 을 찾으려면 등록돼 있어야 한다

if args.size:
    pinky_robot.PinkyCamera.resolution = tuple(args.size)         # 기록은 그대로 두고 센서만 바꾼다

paths = sorted(glob.glob(os.path.join(args.input, "*")))[:args.limit]
total, started = 0, time.time()
for number, path in enumerate(paths, 1):
    out = os.path.join(args.out, os.path.basename(path))
    scenario = load_scenario(path)                                # 기록에 든 장면을 열고, 같은 로봇 · 같은 시나리오를 만든다
    get_world().reset()
    scenario.enable_rgb_rendering()
    if not args.no_depth:
        scenario.enable_depth_rendering()
    reader = MobilityGenReader(path)
    scenario.load_state_dict(reader.read_state_dict(index=0))
    scenario.write_replay_data()
    for _ in range(args.warmup):                                  # 첫 자리에서 몇 장을 그리고 버린다: 처음에는 그림(텍스처)을 읽는 중이고, 앞 장면의 잔상도 남는다
        app.update()
        rep.orchestrator.step(rt_subframes=1, delta_time=0.0, pause_timeline=False)
    shutil.rmtree(out, ignore_errors=True)
    writer = MobilityGenWriter(out)
    writer.copy_init(path)                                        # 설정 · 지도 · 장면은 그대로 옮긴다
    count = 0
    for step in range(0, len(reader), args.every):
        scenario.load_state_dict(reader.read_state_dict(index=step))      # 기록된 상태를 읽어
        scenario.write_replay_data()                                      # 로봇을 그 자리 · 그 관절 각도로 놓고
        app.update()
        rep.orchestrator.step(rt_subframes=1, delta_time=0.0, pause_timeline=False)      # 한 장 그린다
        scenario.update_state()                                           # 그린 영상을 읽어 온다
        writer.write_state_dict_common(scenario.state_dict_common(), step)
        writer.write_state_dict_rgb(scenario.state_dict_rgb(), step)
        if not args.no_depth:
            writer.write_state_dict_depth(scenario.state_dict_depth(), step)
        count += 1
    total += count
    print(f"[{number}/{len(paths)}] {os.path.basename(path)}  {len(reader)} 스텝 → {count}장", flush=True)

seconds = time.time() - started
size = sum(os.path.getsize(os.path.join(folder, name)) for folder, _, names in os.walk(args.out) for name in names)
print(f"모두 {total}장을 {seconds:.0f}초에 그림 (초당 {total / max(seconds, 1e-6):.1f}장), {size / 1e6:.0f} MB  → {args.out}", flush=True)
app.close()
