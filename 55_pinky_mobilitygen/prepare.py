"""강의 55: MobilityGen 에 넣을 재료 셋을 만듭니다. 도시 파일은 강의 16 의 스크립트로 먼저 만들어 둡니다.

    ~/isaacsim/python.sh 16_pink_city/pink_city.py --save work/mobilitygen/pink_city.usd      # 먼저: 핑크시티를 USD 로
    ~/isaacsim/python.sh 55_pinky_mobilitygen/prepare.py

만드는 것 (work/mobilitygen/)
    pinky.usda              로봇만 든 파일. 강의 11 의 가져오기 + 강의 12 의 카메라
    map/map.png · map.yaml  핑크시티의 점유 지도 (ROS 형식). 로봇의 키(2 ~ 15 cm) 높이에 있는 충돌체를 위에서 본 것
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, ".."))
WORK = os.path.join(REPO, "work", "mobilitygen")
CITY = os.path.join(WORK, "pink_city.usd")
CELL, EDGE = 0.02, 1.6                                           # 지도의 한 칸 [m], 도시의 가운데에서 지도의 끝까지 [m]
Z_RANGE = (0.02, 0.15)                                           # 이 높이에 걸친 충돌체가 장애물이다 [m]

from isaacsim import SimulationApp
app = SimulationApp({"headless": True})

import numpy as np
import omni.usd
from PIL import Image
from pxr import Usd, UsdGeom, UsdPhysics

sys.path.insert(0, REPO)
for folder in ("11_pinky_import", "12_pinky_sensors"):
    sys.path.insert(0, os.path.join(REPO, folder))
from common.sim import new_scene, save_stage, set_pose
from import_pinky import DEFAULT_URDF, ROBOT, find_urdf, import_pinky
from pinky_sensors import add_camera

if not os.path.exists(CITY):
    raise SystemExit(f"{CITY} 이 없음: 먼저 ~/isaacsim/python.sh 16_pink_city/pink_city.py --save work/mobilitygen/pink_city.usd 를 실행할 것")
os.makedirs(os.path.join(WORK, "map"), exist_ok=True)

# 1. 로봇만 든 파일
stage = new_scene()
robot = import_pinky(stage, find_urdf(DEFAULT_URDF))
add_camera(stage)
set_pose(robot, translate=(0.0, 0.0, 0.0))
roots = [str(prim.GetPath())[len(ROBOT) + 1:] for prim in Usd.PrimRange(robot) if prim.HasAPI(UsdPhysics.ArticulationRootAPI)]
save_stage(stage, os.path.join(WORK, "pinky_stage.usd"))
with open(os.path.join(WORK, "pinky.usda"), "w") as file:        # 장면 파일에서 로봇(/World/pinky)만 참조하는 작은 파일 (강의 50)
    file.write('#usda 1.0\n(\n    defaultPrim = "pinky"\n    metersPerUnit = 1\n    upAxis = "Z"\n)\n\n'
               f'def Xform "pinky" (\n    prepend references = @./pinky_stage.usd@<{ROBOT}>\n)\n{{\n}}\n')
print(f"로봇: {os.path.join(WORK, 'pinky.usda')}  (관절 로봇의 뿌리: {roots})", flush=True)

# 2. 점유 지도: 도시의 충돌체를 위에서 본 것
context = omni.usd.get_context()
context.open_stage(CITY)
while context.get_stage_loading_status()[2] > 0:
    app.update()
stage = context.get_stage()
cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.guide])
cells = int(round(2 * EDGE / CELL))
grid = np.full((cells, cells), 255, np.uint8)                    # 255 = 빈 곳 (흰색), 0 = 장애물 (검은색)
count = 0
for prim in stage.Traverse():
    if not prim.HasAPI(UsdPhysics.CollisionAPI) or prim.GetName() == "GroundPlane":
        continue
    box = cache.ComputeWorldBound(prim).ComputeAlignedRange()
    low, high = box.GetMin(), box.GetMax()
    if box.IsEmpty() or low[2] > Z_RANGE[1] or high[2] < Z_RANGE[0]:
        continue
    c0, c1 = int(np.floor((low[0] + EDGE) / CELL)), int(np.ceil((high[0] + EDGE) / CELL))
    r0, r1 = int(np.floor((EDGE - high[1]) / CELL)), int(np.ceil((EDGE - low[1]) / CELL))       # 그림의 첫 줄이 y 가 가장 큰 쪽이다
    grid[max(r0, 0):min(r1, cells), max(c0, 0):min(c1, cells)] = 0
    count += 1
Image.fromarray(grid).save(os.path.join(WORK, "map", "map.png"))
with open(os.path.join(WORK, "map", "map.yaml"), "w") as file:
    file.write(f"image: map.png\nresolution: {CELL}\norigin: [{-EDGE}, {-EDGE}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n")
print(f"지도: {os.path.join(WORK, 'map', 'map.yaml')}  ({cells} × {cells} 칸, 한 칸 {CELL * 100:.0f} cm, 장애물로 친 충돌체 {count}개, 막힌 칸 {(grid == 0).mean() * 100:.0f} %)", flush=True)
app.close()
