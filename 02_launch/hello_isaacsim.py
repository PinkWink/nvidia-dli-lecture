"""Isaac Sim 을 Python 스크립트로 켜고, 버전을 출력한 뒤 끕니다.

    ~/isaacsim/python.sh 02_launch/hello_isaacsim.py             # 창 없이 (headless)
    ~/isaacsim/python.sh 02_launch/hello_isaacsim.py --gui       # 창을 띄우고 닫을 때까지 유지

isaac-sim.sh 는 화면(GUI)을 띄우는 실행 파일이고, python.sh 는 Isaac Sim 에 들어 있는 Python 입니다.
python.sh 로 실행한 스크립트는 SimulationApp 을 만들어 Isaac Sim 을 직접 켭니다.
"""
import argparse

from isaacsim import SimulationApp

parser = argparse.ArgumentParser()
parser.add_argument("--gui", action="store_true", help="창을 띄운다")
args = parser.parse_args()

# SimulationApp 을 만든 뒤에야 omni.* · isaacsim.* · pxr 모듈을 import 할 수 있습니다.
app = SimulationApp({"headless": not args.gui})

import omni.kit.app
import omni.usd
from pxr import UsdGeom

stage = omni.usd.get_context().get_stage()
print("Isaac Sim 버전 :", omni.kit.app.get_app().get_app_version(), flush=True)
print("Kit 버전       :", omni.kit.app.get_app().get_kit_version(), flush=True)
print("위쪽 축        :", UsdGeom.GetStageUpAxis(stage), flush=True)
print("길이 단위      :", UsdGeom.GetStageMetersPerUnit(stage), "m", flush=True)

while args.gui and app.is_running():
    app.update()
app.close()
