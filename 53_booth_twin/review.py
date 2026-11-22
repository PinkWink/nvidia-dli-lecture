"""강의 53: 부스에 상자 더미를 새로 놓아도 되는지 디지털트윈에서 검토합니다. 로봇이 다닌 길을 막는가, 쌓은 것이 무너지는가.

    ~/isaacsim/python.sh 50_usd_layers/make_layers.py                                    # 먼저: 강의 50 의 장면
    ~/isaacsim/python.sh 53_booth_twin/review.py                                         # 지금의 부스: 로봇의 길과 가구 사이의 거리
    ~/isaacsim/python.sh 53_booth_twin/review.py --at 6.9 -2.9 --layers 3                # 그 자리에 상자 3단을 놓으면 길을 막는가
    ~/isaacsim/python.sh 53_booth_twin/review.py --at 8.5 -3.4 --layers 5 --drop 3       # 3초 동안 물리를 돌려 무너지는지 본다
    ~/isaacsim/python.sh 53_booth_twin/review.py --at 8.5 -3.4 --layers 5 --jitter 0.10 --drop 3     # 층마다 10 cm 까지 어긋나게 쌓으면
    ~/isaacsim/python.sh 53_booth_twin/review.py --at 8.5 -3.4 --layers 3 --save goods   # 이 안을 work/booth_layers/goods.usda 로 남긴다
    옵션: --photo 파일 (위에서 본 사진)  --gui (창으로 보기)  --clear 0.40 (길에서 이만큼은 비워 둔다 [m])

검토 안은 원래 장면(layout.usda)에 적지 않고 따로 둔 레이어에 적는다 (강의 50). --save 를 주지 않으면 어디에도 남지 않는다.
"""
import argparse
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, ".."))
LAYERS = os.path.join(REPO, "work", "booth_layers")
BOOTH = "/World/Site/Booth"                                        # 강의 50 의 장면에서 부스가 놓인 곳

parser = argparse.ArgumentParser()
parser.add_argument("--at", type=float, nargs=2, metavar=("X", "Y"), help="상자 더미를 놓을 자리 (부스 좌표 [m])")
parser.add_argument("--layers", type=int, default=3, help="몇 단 쌓나")
parser.add_argument("--jitter", type=float, default=0.0, help="층마다 옆으로 어긋나는 정도 [m] (±15° 까지 돌기도 한다)")
parser.add_argument("--seed", type=int, default=0, help="어긋남의 무작위 씨앗")
parser.add_argument("--clear", type=float, default=0.40, help="로봇의 길에서 이만큼은 비워 둔다 [m]: 로봇 폭의 절반 0.32 + 여유 0.08")
parser.add_argument("--drop", type=float, metavar="SECONDS", help="이 시간만큼 물리를 돌려 상자가 제자리에 있는지 본다")
parser.add_argument("--save", metavar="NAME", help="검토 안을 work/booth_layers/NAME.usda 로 저장한다")
parser.add_argument("--photo", metavar="PNG", help="위에서 본 사진을 저장한다")
parser.add_argument("--gui", action="store_true", help="창을 띄운다")
args = parser.parse_args()

from isaacsim import SimulationApp
app = SimulationApp({"headless": not args.gui})

import numpy as np
import omni.timeline
import omni.usd
from pxr import Sdf, Usd, UsdGeom

sys.path.insert(0, HERE)
import twin

context = omni.usd.get_context()
context.open_stage(os.path.join(LAYERS, "layout.usda"))
while context.get_stage_loading_status()[2] > 0:
    app.update()
stage = context.get_stage()
stage.Unload("/World/pl2m")                                        # 로봇은 불러오지 않는다 (payload). 여기서 보는 것은 로봇이 다닌 길이다

# 1. 로봇이 다닌 길
paths = twin.load_paths()
twin.draw_paths(stage, paths)
print("로봇이 다닌 길: " + ",  ".join(f"{name} {float(np.hypot(*np.diff(points, axis=0).T).sum()):.1f} m" for name, points in paths.items()), flush=True)

# 2. 지금의 부스: 길에서 가장 가까운 가구
shapes = twin.footprints(stage, BOOTH)
groups = {}
for name, (distance, path) in twin.clearance(paths, shapes).items():
    group = "/".join(name.split("/")[:2])                          # 예: People/staff_5, Parts/drink_stand
    if group not in groups or distance < groups[group][0]:
        groups[group] = (distance, path)
print(f"\n지금의 부스: 충돌체 {len(shapes)}개. 길의 가운데 선에서 가장 가까운 것 다섯", flush=True)
for group, (distance, path) in sorted(groups.items(), key=lambda item: item[1][0])[:5]:
    print(f"  {group:<28}{distance:5.2f} m   ({path})", flush=True)

# 3. 검토 안: 상자 더미. 따로 둔 레이어에 적는다
if args.at:
    proposal = Sdf.Layer.CreateAnonymous("proposal.usda")
    stage.GetRootLayer().subLayerPaths.insert(0, proposal.identifier)                 # 원래 장면 위에 얹는다. 열어 둔 동안만이고 layout.usda 는 저장하지 않는다
                                                                                      # (세션 레이어에 얹으면 그 안의 강체는 물리가 돌지 않았다)
    with Usd.EditContext(stage, proposal):
        twin.add_stack(stage, args.at[0], args.at[1], args.layers, args.jitter, args.seed)
    distance, path = min(twin.clearance(paths, twin.footprints(stage, twin.STACK.rsplit("/", 1)[0])).values())
    verdict = "막힘" if distance < args.clear else "괜찮음"
    print(f"\n검토 안: ({args.at[0]:.2f}, {args.at[1]:.2f}) 에 상자 {args.layers}단 (높이 {twin.BOX[2] * args.layers:.2f} m)" + (f", 층마다 {args.jitter * 100:.0f} cm 까지 어긋남" if args.jitter else ""), flush=True)
    print(f"  길에서 가장 가까운 거리 {distance:.2f} m ({path})  →  {verdict} (비워 둘 거리 {args.clear:.2f} m)", flush=True)
    gap, neighbor = twin.nearest_shape(twin.footprints(stage, twin.STACK.rsplit("/", 1)[0]), shapes)
    print(f"  가장 가까운 가구까지 {gap:.2f} m ({'/'.join(neighbor.split('/')[:2])})" + ("  →  겹침" if gap <= 0.0 else ""), flush=True)
    if args.save:
        file = os.path.join(LAYERS, f"{args.save}.usda")
        proposal.Export(file)
        print(f"  저장: {file}", flush=True)

timeline = omni.timeline.get_timeline_interface()
if args.drop or args.photo:
    timeline.play()
if args.at and args.drop:
    for _ in range(5):
        app.update()
    before = twin.box_states(stage)
    for _ in range(int(args.drop * 60)):
        app.update()
    moved = twin.fallen(before, twin.box_states(stage))
    print(f"  {args.drop:g}초 뒤: " + ("모두 제자리" if not moved else f"{len(moved)}개가 벗어남: " + ",  ".join(
        f"{name} 높이 {z0:.2f} → {z1:.2f} m · 옆으로 {side * 100:.0f} cm" for name, z0, z1, side in moved)), flush=True)
if args.photo:
    import omni.replicator.core as rep
    from isaacsim.core.utils.viewports import set_camera_view
    from PIL import Image
    camera = UsdGeom.Camera.Define(stage, "/PhotoCamera")
    camera.CreateFocalLengthAttr(18.0)
    center = args.at or (6.4, -3.0)
    set_camera_view(eye=[center[0] - 0.6, center[1] - 3.4, 6.0], target=[center[0], center[1], 0.3], camera_prim_path="/PhotoCamera")
    product = rep.create.render_product("/PhotoCamera", (1280, 720))
    picture = rep.AnnotatorRegistry.get_annotator("rgb")
    picture.attach([product])
    data = np.zeros(0)
    for count in range(400):
        app.update()
        data = np.asarray(picture.get_data())
        if data.ndim == 3 and count > 120:
            break
    Image.fromarray(data[:, :, :3]).save(args.photo)
    print("사진 저장:", args.photo, flush=True)
timeline.stop()
if args.gui:
    from isaacsim.core.utils.viewports import set_camera_view
    center = args.at or (6.4, -3.0)
    set_camera_view(eye=[center[0] - 0.6, center[1] - 3.4, 6.0], target=[center[0], center[1], 0.3])
    while app.is_running():
        app.update()
app.close()
