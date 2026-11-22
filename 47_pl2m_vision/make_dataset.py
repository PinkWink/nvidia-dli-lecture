"""강의 47: 부스의 테이블 위에 놓인 컵을 찾는 모델의 학습 데이터를 만듭니다. 결과는 바로 YOLO 학습에 쓰는 형식입니다.

    ~/isaacsim/python.sh 47_pl2m_vision/make_dataset.py                    # 400장 → work/cup_yolo
    ~/isaacsim/python.sh 47_pl2m_vision/make_dataset.py --count 12 --out work/cup_yolo_test
    ~/isaacsim/python.sh 47_pl2m_vision/make_dataset.py --count 12 --gui   # 창으로 보기 (느려짐)

결과
    <out>/images/{train,val}/N.png     영상 (640 × 480, 머리 카메라와 같은 렌즈)
    <out>/labels/{train,val}/N.txt     상자 정답 (YOLO 형식: 번호 x중심 y중심 너비 높이)
    <out>/data.yaml                    학습 설정 (0 cup_pink · 1 cup_blue · 2 cup_yellow)

강의 27 과 다른 점
    강의 27   Replicator 의 그래프(rep.trigger.on_frame)에 무작위 규칙을 적고, 저장은 KittiWriter 에 맡김
    이 강의   파이썬 반복문에서 장면을 직접 바꾸고 한 장씩 찍음 (rep.orchestrator.step). 정답은 Annotator 에서 읽어 직접 적음
"""
import argparse
import math
import os
import random
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, ".."))
CLASSES = ["cup_pink", "cup_blue", "cup_yellow"]
SIZE, FOVY = (640, 480), 60.0                     # 머리 카메라와 같은 크기 · 세로 화각
# 컵을 놓는 상판: (가운데 x, y, 높이, 모양, 크기). 상자는 (x 길이, y 길이), 원은 (반지름,)
TOPS = {"drinks": (7.65, -1.60, 0.75, "box", (0.62, 0.42)),
        "consult_1": (6.28, -1.90, 0.72, "disc", (0.30,)),
        "consult_2": (6.38, -4.43, 0.72, "disc", (0.30,))}

parser = argparse.ArgumentParser()
parser.add_argument("--count", type=int, default=400, help="만들 영상 수")
parser.add_argument("--out", default="work/cup_yolo", help="저장할 폴더")
parser.add_argument("--seed", type=int, default=0, help="무작위 씨앗. 같은 값이면 같은 데이터")
parser.add_argument("--gui", action="store_true", help="창을 띄운다")
args = parser.parse_args()
OUT = os.path.abspath(os.path.expanduser(args.out))

from isaacsim import SimulationApp
app = SimulationApp({"headless": not args.gui, "renderer": "RayTracedLighting"})

import numpy as np
import omni.replicator.core as rep
from isaacsim.core.utils.semantics import add_labels
from PIL import Image
from pxr import Gf, UsdGeom, UsdLux

sys.path.insert(0, REPO)
for folder in ("38_pl2m_booth", "44_pl2m_nav"):
    sys.path.insert(0, os.path.join(REPO, folder))
import make_booth
from booth import STATIONS
from common.sim import new_scene, set_pose

started = time.time()
rng = random.Random(args.seed)
rep.settings.carb_settings("/omni/replicator/RTSubframes", 4)          # 물체를 옮긴 뒤 남는 잔상을 줄인다

# 1. 장면: 부스 + 조명 + 컵 (강의 38). 로봇은 넣지 않는다
stage = new_scene()
mjcf = make_booth.read_mjcf(make_booth.build_mjcf())
make_booth.add_booth(stage, mjcf)
make_booth.add_lights(stage, mjcf)
make_booth.add_cups(stage)
cups = {name: stage.GetPrimAtPath(f"/World/Cups/{name}") for name in CLASSES}
colors = {name: color for name, _, color in make_booth.CUPS}
for name, prim in cups.items():
    add_labels(prim, labels=[name], instance_name="class")            # 정답에 적힐 이름
lights = [(UsdLux.LightAPI(prim).GetIntensityAttr(), UsdLux.LightAPI(prim).GetIntensityAttr().Get())
          for prim in stage.Traverse() if prim.HasAPI(UsdLux.LightAPI) and "/Booth/Lights" in str(prim.GetPath())]

# 2. 카메라: 머리 카메라와 같은 렌즈. 자리는 장마다 바꾼다
camera = UsdGeom.Camera.Define(stage, "/World/DatasetCamera")
aperture = 20.955
camera.CreateHorizontalApertureAttr(aperture)
camera.CreateVerticalApertureAttr(aperture * SIZE[1] / SIZE[0])
camera.CreateFocalLengthAttr(aperture * SIZE[1] / SIZE[0] / 2.0 / math.tan(math.radians(FOVY / 2.0)))
camera.CreateClippingRangeAttr(Gf.Vec2f(0.01, 50.0))
view = UsdGeom.Xformable(camera.GetPrim()).AddTransformOp()
product = rep.create.render_product("/World/DatasetCamera", SIZE)
color_out = rep.AnnotatorRegistry.get_annotator("rgb")
boxes_out = rep.AnnotatorRegistry.get_annotator("bounding_box_2d_tight")     # 보이는 부분만 감싼 상자
color_out.attach(product)
boxes_out.attach(product)


def look_from(eye, target):
    """카메라를 eye 에 놓고 target 을 보게 한다. USD 의 카메라는 -Z 를 보고 +Y 가 위다."""
    view.Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*target), Gf.Vec3d(0, 0, 1)).GetInverse())


def spot_on(top, taken, gap=0.07, margin=0.04):
    """상판 위의 빈 자리 하나. 다른 컵과 gap, 가장자리에서 margin 넘게 떨어지게."""
    cx, cy, _, shape, size = TOPS[top]
    while True:
        if shape == "box":
            x, y = rng.uniform(-size[0] / 2 + margin, size[0] / 2 - margin), rng.uniform(-size[1] / 2 + margin, size[1] / 2 - margin)
        else:
            radius, angle = (size[0] - margin) * math.sqrt(rng.random()), rng.uniform(0.0, 2.0 * math.pi)
            x, y = radius * math.cos(angle), radius * math.sin(angle)
        if all(math.hypot(cx + x - px, cy + y - py) >= gap for px, py in taken):
            return cx + x, cy + y


for split in ("train", "val"):
    for kind in ("images", "labels"):
        os.makedirs(os.path.join(OUT, kind, split), exist_ok=True)
print(f"부스에서 {args.count}장을 만듭니다 → {OUT}", flush=True)
counts, empty = dict.fromkeys(CLASSES, 0), 0
for index in range(args.count):
    # 3. 장마다 바꾸는 것: 보는 테이블, 컵의 자리와 색, 카메라의 자리, 조명의 밝기
    station = rng.choice(["drinks", "drinks", "consult_1", "consult_2"])
    taken = []
    for name, prim in cups.items():
        where = rng.choice([station, station, station, rng.choice(list(TOPS)), None])       # None = 치워 둔다 (보이지 않음)
        if where is None:
            set_pose(prim, translate=(0.0, 0.0, -5.0))
            continue
        x, y = spot_on(where, taken)
        taken.append((x, y))
        set_pose(prim, translate=(x, y, TOPS[where][2] + make_booth.CUP_SIZE[1] / 2.0))
        UsdGeom.Gprim(prim).GetDisplayColorAttr().Set([Gf.Vec3f(*[min(1.0, max(0.0, c + rng.uniform(-0.06, 0.06))) for c in colors[name]])])
    (gx, gy, yaw), (ex, ey) = STATIONS[station]["goal"], STATIONS[station]["edge"]
    back, side = rng.uniform(0.30, 1.25), rng.uniform(-0.25, 0.25)                         # 테이블 가장자리에서 뒤로 · 옆으로 [m]
    eye = (ex - back * math.cos(yaw) - side * math.sin(yaw), ey - back * math.sin(yaw) + side * math.cos(yaw), rng.uniform(1.00, 1.20))
    ahead = rng.uniform(0.00, 0.30)                                                        # 가장자리에서 이만큼 안쪽을 본다
    look_from(eye, (ex + ahead * math.cos(yaw) + rng.uniform(-0.15, 0.15), ey + ahead * math.sin(yaw) + rng.uniform(-0.15, 0.15), TOPS[station][2] + rng.uniform(-0.10, 0.15)))
    scale = rng.uniform(0.5, 1.6)
    for attribute, base in lights:
        attribute.Set(base * scale)

    # 4. 한 장 찍고, 정답을 YOLO 형식으로 적는다
    rep.orchestrator.step(rt_subframes=4)
    found = boxes_out.get_data()
    names = {int(key): value["class"] for key, value in found["info"]["idToLabels"].items()}
    lines = []
    for box in found["data"]:
        name = names.get(int(box["semanticId"]))
        width, height = float(box["x_max"] - box["x_min"]), float(box["y_max"] - box["y_min"])
        if name not in CLASSES or width < 6 or height < 6:                                 # 너무 작게 보이는 것은 버린다
            continue
        lines.append(f"{CLASSES.index(name)} {(box['x_min'] + box['x_max']) / 2 / SIZE[0]:.6f} {(box['y_min'] + box['y_max']) / 2 / SIZE[1]:.6f} "
                     f"{width / SIZE[0]:.6f} {height / SIZE[1]:.6f}")
        counts[name] += 1
    empty += not lines
    split = "val" if index % 10 == 9 else "train"                                          # 열 장에 한 장은 검증용
    Image.fromarray(color_out.get_data()[..., :3]).save(os.path.join(OUT, "images", split, f"{index:04d}.png"))
    with open(os.path.join(OUT, "labels", split, f"{index:04d}.txt"), "w") as file:
        file.write("\n".join(lines) + ("\n" if lines else ""))

with open(os.path.join(OUT, "data.yaml"), "w") as file:
    file.write(f"path: {OUT}\ntrain: images/train\nval: images/val\nnames:\n" + "".join(f"  {k}: {name}\n" for k, name in enumerate(CLASSES)))
print(f"저장: 영상 {args.count}장 (학습 {args.count - args.count // 10}, 검증 {args.count // 10}), {time.time() - started:.0f}초", flush=True)
print("상자: " + ",  ".join(f"{name} {count}개" for name, count in counts.items()) + f",  컵이 없는 영상 {empty}장", flush=True)
app.close()
