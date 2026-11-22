"""강의 51: 강의 50 의 원래 배치(layout.usda)를 창으로 열고, 이 폴더의 익스텐션(pinklab.cup_tools)을 켭니다.

    ~/isaacsim/python.sh 51_extension/open_with_extension.py                 # 창으로 열기. Window 메뉴에 "PinkLAB Cup Tools" 가 생긴다
    ~/isaacsim/python.sh 51_extension/open_with_extension.py --test          # 창 없이: 버튼이 하는 일을 차례로 불러 보고 결과를 출력

Isaac Sim 을 평소처럼 띄울 때 켜려면:
    ~/isaacsim/isaac-sim.sh --ext-folder ~/nvidia-dli-lecture/51_extension/exts --enable pinklab.cup_tools
"""
import argparse
import os

HERE = os.path.dirname(os.path.abspath(__file__))
SCENE = os.path.abspath(os.path.join(HERE, "..", "work", "booth_layers", "layout.usda"))      # 강의 50 이 만든 원래 배치 (덮어쓰는 파일을 쌓지 않은 것)
NAME = "pinklab.cup_tools"

parser = argparse.ArgumentParser()
parser.add_argument("--test", action="store_true", help="창 없이 버튼이 하는 일을 차례로 불러 본다")
parser.add_argument("--file", default=SCENE, help="열 장면")
args = parser.parse_args()

from isaacsim import SimulationApp
app = SimulationApp({"headless": args.test})

import omni.kit.app
import omni.kit.undo
import omni.usd
from isaacsim.core.utils.viewports import set_camera_view

context = omni.usd.get_context()
context.open_stage(args.file)
while context.get_stage_loading_status()[2] > 0:
    app.update()

manager = omni.kit.app.get_app().get_extension_manager()
manager.add_path(os.path.join(HERE, "exts"))                       # 익스텐션을 찾을 폴더를 하나 더한다
manager.set_extension_enabled_immediate(NAME, True)                # 켠다 → on_startup 이 불린다
print(f"익스텐션 {NAME}: {'켜짐' if manager.is_extension_enabled(NAME) else '켜지지 않음'}", flush=True)

def menu_count():
    """Window 메뉴에 이 익스텐션의 항목이 몇 개 있나."""
    import omni.kit.menu.utils
    return sum(item.name == "PinkLAB Cup Tools" for item in omni.kit.menu.utils.get_merged_menus()["Window"]["items"])


if args.test:
    import sys
    print(f"Window 메뉴의 항목: {menu_count()}개", flush=True)
    tools = sys.modules[NAME].extension.INSTANCE                   # 켜진 익스텐션. 버튼이 부르는 함수를 그대로 불러 본다
    for label, press in (("Shuffle", tools.shuffle), ("blue", lambda: tools.paint("blue")), ("Person on / off", tools.toggle_person), ("Home", tools.go_home)):
        press()
        print(f"[{label}] " + tools.status.replace("\n", "  →  "), flush=True)
    for _ in range(3):
        omni.kit.undo.undo()                                       # Ctrl+Z 세 번: "원래 자리" 가 컵 셋을 옮긴 것을 되돌린다
    tools.report("undo x 3")
    print("[Ctrl+Z × 3] " + tools.status.replace("\n", "  →  "), flush=True)
else:
    set_camera_view(eye=[7.95, -4.05, 2.05], target=[7.80, -1.70, 0.80])
    while app.is_running():
        app.update()
manager.set_extension_enabled_immediate(NAME, False)               # 끈다 → on_shutdown 이 불린다
if args.test:
    for _ in range(10):
        app.update()
    print(f"끈 뒤 Window 메뉴의 항목: {menu_count()}개", flush=True)
app.close()
