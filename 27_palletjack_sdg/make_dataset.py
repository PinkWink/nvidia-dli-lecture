"""강의 27: 창고에서 팔레트잭을 찾는 모델의 학습 데이터를 Replicator 로 만듭니다.

    ~/isaacsim/python.sh 27_palletjack_sdg/make_dataset.py                       # 방해물이 있는 창고 500장
    ~/isaacsim/python.sh 27_palletjack_sdg/make_dataset.py --count 100 --plain --seed 1 --out work/palletjack_dataset/plain
    ~/isaacsim/python.sh 27_palletjack_sdg/make_dataset.py --count 24 --gui --out work/palletjack_dataset/test

결과
    <out>/Camera/rgb/N.png                 영상
    <out>/Camera/object_detection/N.txt    상자 정답 (KITTI 형식)

NVIDIA 의 공개 예제 standalone_palletjack_sdg.py (synthetic_data_generation_training_workflow) 와 같은 구성
(창고 · 팔레트잭 3종 · 방해물 · 프레임마다 무작위 · KITTI) 을 Isaac Sim 5.1 용으로 짧게 다시 쓴 것입니다.
"""
import argparse
import os
import time
from collections import Counter
from pathlib import Path

from isaacsim import SimulationApp

REPO = Path(__file__).resolve().parent.parent

parser = argparse.ArgumentParser()
parser.add_argument("--count", type=int, default=500, help="만들 영상 수")
parser.add_argument("--out", default="work/palletjack_dataset/warehouse", help="저장할 폴더")
parser.add_argument("--plain", action="store_true", help="방해물을 놓지 않는다")
parser.add_argument("--seed", type=int, default=0, help="무작위 씨앗. 같은 값이면 같은 데이터")
parser.add_argument("--width", type=int, default=960)
parser.add_argument("--height", type=int, default=544)
parser.add_argument("--gui", action="store_true", help="창을 띄운다 (느려짐)")
args = parser.parse_args()
OUT = Path(args.out).expanduser()
OUT = OUT if OUT.is_absolute() else Path(os.getcwd()) / OUT

app = SimulationApp({"headless": not args.gui, "renderer": "RayTracedLighting", "width": args.width, "height": args.height})

import omni.replicator.core as rep  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.storage.native import get_assets_root_path  # noqa: E402

ASSETS = get_assets_root_path()
WAREHOUSE = ASSETS + "/Isaac/Environments/Simple_Warehouse/warehouse.usd"
TRUCKS = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/DigitalTwin/Assets/Warehouse/Equipment/Pallet_Trucks"
PALLETJACKS = [f"{TRUCKS}/Scale_A/PalletTruckScale_A01_PR_NVD_01.usd",
               f"{TRUCKS}/Heavy_Duty_A/HeavyDutyPalletTruck_A01_PR_NVD_01.usd",
               f"{TRUCKS}/Low_Profile_A/LowProfilePalletTruck_A01_PR_NVD_01.usd"]
PROPS = ASSETS + "/Isaac/Environments/Simple_Warehouse/Props/"
DISTRACTORS = [PROPS + name for name in (
    "S_TrafficCone.usd", "S_WetFloorSign.usd", "SM_BarelPlastic_A_01.usd", "SM_BarelPlastic_B_01.usd",
    "SM_CardBoxB_01_1262.usd", "SM_CardBoxD_01_1454.usd", "SM_CratePlastic_A_04.usd",
    "SM_PushcartA_02.usd", "SM_RackPile_04.usd", "SM_BucketPlastic_B.usd")]
AREA_LOW, AREA_HIGH = (-6, -6, 0), (6, 12, 0)        # 물체를 놓는 바닥 범위 [m]

started = time.time()
rep.set_global_seed(args.seed)
rep.settings.carb_settings("/omni/replicator/RTSubframes", 4)      # 물체를 옮긴 뒤 남는 잔상을 줄인다

# 1. 장면: 창고
omni.usd.get_context().open_stage(WAREHOUSE)
while omni.usd.get_context().get_stage_loading_status()[2] > 0:
    app.update()
for _ in range(50):
    app.update()

# 2. 물체: 팔레트잭 3종 x 2대 (정답 이름 palletjack), 방해물 10종 x 2개 (정답 없음), 카메라
jacks = rep.create.group([rep.create.from_usd(path, semantics=[("class", "palletjack")], count=2) for path in PALLETJACKS])
props = None if args.plain else rep.create.group([rep.create.from_usd(path, count=2) for path in DISTRACTORS])
camera = rep.create.camera(clipping_range=(0.1, 1000000))

# 3. 프레임마다 무작위로 바꿀 것
with rep.trigger.on_frame(num_frames=args.count):
    with camera:                                        # 카메라 위치. 항상 창고 가운데를 본다
        rep.modify.pose(position=rep.distribution.uniform((-9.0, -11.0, 0.5), (7.0, 15.0, 4.0)), look_at=(0, 0, 0))
    with jacks:                                         # 팔레트잭 위치와 방향. 모델이 cm 단위라 0.01 배
        rep.modify.pose(position=rep.distribution.uniform(AREA_LOW, AREA_HIGH),
                        rotation=rep.distribution.uniform((0, 0, 0), (0, 0, 360)), scale=0.01)
    with rep.get.prims(path_pattern="SteerAxles"):      # 팔레트잭 몸통 색
        rep.randomizer.color(colors=rep.distribution.uniform((0, 0, 0), (1, 1, 1)))
    if props is not None:
        with props:                                     # 방해물 위치 · 방향 · 크기
            rep.modify.pose(position=rep.distribution.uniform(AREA_LOW, AREA_HIGH),
                            rotation=rep.distribution.uniform((0, 0, 0), (0, 0, 360)),
                            scale=rep.distribution.uniform(1.0, 1.5))
    with rep.get.prims(path_pattern="RectLight"):       # 천장 조명의 색 · 밝기. 셋 중 하나는 끈다
        rep.modify.attribute("color", rep.distribution.uniform((0.3, 0.3, 0.3), (1, 1, 1)))
        rep.modify.attribute("intensity", rep.distribution.uniform(3000.0, 25000.0))
        rep.modify.visibility(rep.distribution.choice([True, True, False]))

# 4. 저장: 영상 + 상자 정답 (KITTI 형식)
writer = rep.WriterRegistry.get("KittiWriter")
writer.initialize(output_dir=str(OUT), omit_semantic_type=True)
writer.attach(rep.create.render_product(camera, (args.width, args.height)))
print(f"창고에서 {args.count}장을 만듭니다 → {OUT}", flush=True)
rep.orchestrator.run_until_complete(num_frames=args.count)
rep.BackendDispatch.wait_until_done()

# 5. 만든 것 세기
labels = sorted((OUT / "Camera" / "object_detection").glob("*.txt"))
per_image = Counter()
for label in labels:
    per_image[sum(line.startswith("palletjack ") for line in label.read_text().splitlines())] += 1
boxes = sum(count * images for count, images in per_image.items())
print(f"저장: 영상 {len(labels)}장, 팔레트잭 상자 {boxes}개, {time.time() - started:.0f}초", flush=True)
for count in sorted(per_image):
    print(f"  팔레트잭이 {count}대 보이는 영상: {per_image[count]}장", flush=True)
app.close()
