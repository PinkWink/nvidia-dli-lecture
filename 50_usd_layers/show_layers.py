"""강의 50: 나눠 만든 장면(scene.usda)을 Isaac Sim 에서 열고, 값이 어느 파일에서 왔는지 봅니다. 레이어를 끄거나 Variant 를 바꿔 볼 수 있습니다.

    ~/isaacsim/python.sh 50_usd_layers/show_layers.py                          # 열어서 컵 셋의 색 · 자리와 그 값을 준 파일을 출력
    ~/isaacsim/python.sh 50_usd_layers/show_layers.py --mute rearrange.usda    # 바꿔 보는 안을 끈다 → 원래 배치
    ~/isaacsim/python.sh 50_usd_layers/show_layers.py --color cup_blue=pink    # 열어 둔 채로 Variant 를 바꾼다 (파일에는 적지 않는다)
    ~/isaacsim/python.sh 50_usd_layers/show_layers.py --no-payload             # payload(부스 · 로봇)를 불러오지 않고 연다 (컵만 있는 장면)
    ~/isaacsim/python.sh 50_usd_layers/show_layers.py --photo scene.png        # 음료대를 보는 사진 한 장을 저장
    ~/isaacsim/python.sh 50_usd_layers/show_layers.py --play 3                 # 3초 동안 물리를 돌려 컵이 서 있는지 본다
    ~/isaacsim/python.sh 50_usd_layers/show_layers.py --gui                    # 창으로 보기 (Layer 창 · Property 의 Variants 에서 직접 바꿔 볼 수 있다)
"""
import argparse
import os
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, ".."))
OUT = os.path.join(REPO, "work", "booth_layers")
VIEWS = {"stand": ((7.95, -4.05, 2.05), (7.80, -1.70, 0.80)),      # 음료대와 그 옆에 선 사람이 함께 보이는 자리: (카메라, 바라보는 점)
         "booth": ((3.6, -6.2, 3.0), (6.4, -2.6, 0.6))}            # 로봇과 음료대가 함께 보이는 자리

parser = argparse.ArgumentParser()
parser.add_argument("--file", default="scene.usda", help="열 파일 (work/booth_layers 안)")
parser.add_argument("--mute", nargs="+", metavar="LAYER", default=[], help="끌 레이어의 파일 이름")
parser.add_argument("--color", nargs="+", metavar="CUP=COLOR", default=[], help="컵의 색 Variant 를 바꾼다. 예: cup_blue=pink")
parser.add_argument("--no-payload", action="store_true", help="payload(부스 · 로봇)를 불러오지 않는다")
parser.add_argument("--photo", metavar="PNG", help="음료대를 보는 사진 한 장을 저장한다")
parser.add_argument("--view", choices=list(VIEWS), default="stand", help="사진 · 창의 시점: stand 음료대 가까이, booth 로봇과 음료대가 함께")
parser.add_argument("--play", type=float, metavar="SECONDS", help="이 시간만큼 물리를 돌린다")
parser.add_argument("--gui", action="store_true", help="창을 띄운다")
args = parser.parse_args()

from isaacsim import SimulationApp
app = SimulationApp({"headless": not args.gui})

import numpy as np
import omni.timeline
import omni.usd
from pxr import Usd, UsdGeom


def short(layer):
    return os.path.basename(layer.identifier)


def describe(stage):
    """컵마다: 고른 색, 자리, 그리고 그 값들을 준 파일 (센 것부터)."""
    cups = stage.GetPrimAtPath("/World/Cups")
    for cup in cups.GetChildren() if cups else []:
        color = cup.GetVariantSets().GetVariantSet("color").GetVariantSelection()
        place = cup.GetAttribute("xformOp:translate")
        x, y, z = place.Get()
        opinions = " > ".join(short(spec.layer) for spec in place.GetPropertyStack(Usd.TimeCode.Default()))       # 이 값을 적은 파일들. 맨 앞 것이 이긴다
        print(f"  {cup.GetName():<11} 색 {color:<7} 자리 ({x:.2f}, {y:.2f}, {z:.3f})   자리를 적은 파일: {opinions}", flush=True)
    person = stage.GetPrimAtPath("/World/Site/Booth/People/staff_5")
    if person:
        print(f"  음료대 옆 사람(staff_5): {'있음' if person.IsActive() else '꺼져 있음 (active = false)'}", flush=True)


started = time.time()
path = os.path.join(OUT, args.file)
context = omni.usd.get_context()
context.open_stage(path, load_set=omni.usd.UsdContextInitialLoadSet.LOAD_NONE if args.no_payload else omni.usd.UsdContextInitialLoadSet.LOAD_ALL)
while context.get_stage_loading_status()[2] > 0:                    # 불러오는 중인 파일이 없을 때까지
    app.update()
stage = context.get_stage()
print(f"연 파일: {args.file}  ({time.time() - started:.2f}초)", flush=True)
print("레이어 (센 것부터): " + " > ".join(short(layer) for layer in stage.GetLayerStack()[1:]), flush=True)       # 맨 앞의 세션 레이어는 뺀다
print(f"Prim {len(list(stage.Traverse()))}개" + (" (payload 를 불러오지 않음)" if args.no_payload else ""), flush=True)
describe(stage)

for name in args.mute:
    stage.MuteLayer(next(layer.identifier for layer in stage.GetLayerStack() if short(layer) == name))     # 끈다: 파일은 그대로 두고, 이 장면에서만 없는 것으로 친다
if args.mute:
    print(f"\n레이어를 끔: {', '.join(args.mute)}", flush=True)
    describe(stage)
for item in args.color:
    name, color = item.split("=")
    stage.GetPrimAtPath(f"/World/Cups/{name}").GetVariantSets().GetVariantSet("color").SetVariantSelection(color)
if args.color:
    print(f"\nVariant 를 바꿈: {', '.join(args.color)}  (열어 둔 장면에만. 저장하지 않으면 파일은 그대로)", flush=True)
    describe(stage)

timeline = omni.timeline.get_timeline_interface()
if args.play or args.photo:
    timeline.play()                                                 # 물리도 사진도 Play 중에 된다 (강의 38). 멈췄다 다시 켜지 않고 한 번만 켠다
if args.play:
    for _ in range(int(args.play * 60)):
        app.update()
    print(f"\n{args.play:g}초 뒤 컵의 높이: " + ",  ".join(
        f"{cup.GetName()} {UsdGeom.Xformable(cup).ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()[2]:.3f} m"
        for cup in stage.GetPrimAtPath("/World/Cups").GetChildren()), flush=True)
if args.photo:
    import omni.replicator.core as rep
    from PIL import Image
    camera = UsdGeom.Camera.Define(stage, "/PhotoCamera")           # 세션 레이어에 생긴다: 파일에는 적히지 않는다
    camera.CreateFocalLengthAttr(18.0 if args.view == "booth" else 50.0)          # 넓게 볼 때는 초점 거리를 짧게
    from isaacsim.core.utils.viewports import set_camera_view
    set_camera_view(eye=list(VIEWS[args.view][0]), target=list(VIEWS[args.view][1]), camera_prim_path="/PhotoCamera")
    product = rep.create.render_product("/PhotoCamera", (1280, 720))
    picture = rep.AnnotatorRegistry.get_annotator("rgb")
    picture.attach([product])
    data = np.zeros(0)
    for count in range(400):                                        # 처음에는 그림(텍스처)을 읽느라 화면이 늦게 나온다
        app.update()
        data = np.asarray(picture.get_data())
        if data.ndim == 3 and count > 120:
            break
    Image.fromarray(data[:, :, :3]).save(args.photo)
    print("사진 저장:", args.photo, flush=True)
timeline.stop()
if args.gui:
    from isaacsim.core.utils.viewports import set_camera_view
    import omni.ui
    from isaacsim.core.utils.extensions import enable_extension
    set_camera_view(eye=list(VIEWS[args.view][0]), target=list(VIEWS[args.view][1]))
    enable_extension("omni.kit.widget.layers")                      # Layer 창. 스크립트로 띄운 Isaac Sim 에는 기본으로 켜져 있지 않다
    for _ in range(120):
        app.update()
    layers, tree = omni.ui.Workspace.get_window("Layer"), omni.ui.Workspace.get_window("Stage")
    if layers and tree:
        layers.dock_in(tree, omni.ui.DockPosition.SAME)             # Stage 창 옆의 탭으로 붙인다
    print("\n창을 띄움. Stage 창 옆의 Layer 탭에서 레이어의 눈 모양을 눌러 켜고 끌 수 있음. 창을 닫으면 끝남", flush=True)
    while app.is_running():
        app.update()
app.close()
