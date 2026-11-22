"""강의 55: MobilityGen 창을 Pinky Pro · 핑크시티가 채워진 채로 엽니다. Build 를 누르고 W A S D 로 몰면서 기록합니다.

    ~/isaacsim/python.sh 55_pinky_mobilitygen/open_ui.py
    ~/isaacsim/python.sh 55_pinky_mobilitygen/open_ui.py --test        # 버튼을 코드로 눌러 6초 기록하고 끝낸다 (점검용)

창에서 하는 일
    1. MobilityGen 창의 Stage · Occupancy Map · Robot Type(PinkyRobot) · Scenario Type(KeyboardTeleoperationScenario) 이 채워져 있다
    2. Build → 로봇이 놓인다.  뷰포트를 한 번 누르고 W A S D 로 몬다
    3. Start Recording → 몰기 → Stop Recording.  Reset 을 누르면 새 자리에 놓이고 새 기록이 시작된다
기록은 record.py 와 같은 곳(work/mobilitygen/recordings)에 쌓이고, replay.py · read_data.py 로 똑같이 다룬다.
"""
import argparse
import gc
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
WORK = os.path.abspath(os.path.join(HERE, "..", "work", "mobilitygen"))
os.environ["MOBILITY_GEN_DATA"] = WORK                             # 창이 기록을 두는 곳: 이 폴더의 recordings/

parser = argparse.ArgumentParser()
parser.add_argument("--test", action="store_true", help="버튼을 코드로 눌러 6초 기록하고 끝낸다")
parser.add_argument("--keep", action="store_true", help="--test 가 끝나도 창을 닫지 않는다")
parser.add_argument("--scenario", default="KeyboardTeleoperationScenario")
args = parser.parse_args()

from isaacsim import SimulationApp
app = SimulationApp({"headless": False, "width": 1600, "height": 900})

import omni.usd
from isaacsim.core.utils.extensions import enable_extension
enable_extension("isaacsim.replicator.mobility_gen.examples")
app.update()
sys.path.insert(0, HERE)
import pinky_robot  # noqa: F401  창이 만들어지기 전에 등록해야 Robot Type 목록에 들어간다

enable_extension("isaacsim.replicator.mobility_gen.ui")
app.update()
import isaacsim.replicator.mobility_gen.ui.extension as window_code
from isaacsim.replicator.mobility_gen.examples.robots import ROBOTS
from isaacsim.replicator.mobility_gen.examples.scenarios import SCENARIOS

# 창의 Build 는 장면의 사본을 Isaac Sim 의 save_stage 로 만든다. 그러면 그림 파일의 경로가 상대 경로로 바뀌어,
# 사본이 기록 폴더로 옮겨진 뒤에는 그림을 못 찾는다 (강의 55 의 5절). 경로를 그대로 두는 저장으로 갈아 끼운다.
window_code.save_stage = lambda path, **_: omni.usd.get_context().get_stage().GetRootLayer().Export(path)

window = next(item for item in gc.get_objects() if type(item).__name__ == "MobilityGenExtension")      # 떠 있는 창의 객체
window._teleop_window.width, window._teleop_window.height = 640, 340                 # 처음 크기(300)로는 경로가 잘려 보인다
window.scene_usd_field_string_model.set_value(os.path.join(WORK, "pink_city.usd"))
window.omap_field_string_model.set_value(os.path.join(WORK, "map", "map.yaml"))
window.robot_combo_box.model.get_item_value_model().set_value(list(ROBOTS.names()).index("PinkyRobot"))
window.scenario_combo_box.model.get_item_value_model().set_value(list(SCENARIOS.names()).index(args.scenario))
print("로봇 목록:", ", ".join(ROBOTS.names()), flush=True)
print("시나리오 목록:", ", ".join(SCENARIOS.names()), flush=True)

if args.test:
    window.build_scenario()                                        # Build 버튼
    for _ in range(2000):
        app.update()
        if window.scenario is not None and window.step > 0:
            break
    for _ in range(120):
        app.update()
    window.enable_recording()                                      # Start Recording 버튼
    start = window.recording_time
    keys = window.keyboard.buttons                                 # W A S D 네 키의 상태. 키보드를 누르는 대신 값을 직접 넣는다
    while window.recording_time - start < 6.0:
        seconds = window.recording_time - start
        keys[0]._value = True                                      # W 를 누르고 있다
        keys[1]._value = 2.0 < seconds < 4.0                       # 2 ~ 4초에는 A 도 함께
        app.update()
    for key in keys:
        key._value = False
    name, steps = window.writer.path, window.step
    window.disable_recording()                                     # Stop Recording 버튼
    print(f"기록: {name}  {steps} 스텝 · {steps * pinky_robot.PinkyRobot.physics_dt:.1f}초", flush=True)
if not args.test or args.keep:
    while app.is_running():
        app.update()
app.close()
