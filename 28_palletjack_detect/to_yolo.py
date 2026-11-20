"""강의 28: 강의 27 의 데이터(KITTI 형식)를 YOLO 학습 형식으로 바꿉니다.

    ~/yolo_venv/bin/python 28_palletjack_detect/to_yolo.py work/palletjack_dataset/warehouse work/palletjack_dataset/plain
    ~/yolo_venv/bin/python 28_palletjack_detect/to_yolo.py work/palletjack_dataset/plain --out work/palletjack_yolo_plain

KITTI 한 줄:  이름 잘림 가림 각도  x1 y1 x2 y2 ...   (픽셀. 왼쪽 위 · 오른쪽 아래)
YOLO 한 줄:   번호 x중심 y중심 너비 높이             (영상 크기로 나눈 0 ~ 1)

결과: <out>/images/{train,val}, <out>/labels/{train,val}, <out>/data.yaml. 영상은 복사하지 않고 링크합니다.
"""
import argparse
import random
from pathlib import Path

from PIL import Image

parser = argparse.ArgumentParser()
parser.add_argument("datasets", nargs="+", help="make_dataset.py 가 만든 폴더 (여러 개 가능)")
parser.add_argument("--out", default="work/palletjack_yolo", help="저장할 폴더")
parser.add_argument("--val", type=float, default=0.1, help="검증용으로 떼어 둘 비율")
parser.add_argument("--min-pixels", type=int, default=8, help="가로나 세로가 이보다 작은 상자는 버린다")
args = parser.parse_args()

out = Path(args.out).expanduser().resolve()        # YOLO 는 상대 경로를 자기 폴더 기준으로 읽으므로 절대 경로로
for split in ("train", "val"):
    (out / "images" / split).mkdir(parents=True, exist_ok=True)
    (out / "labels" / split).mkdir(parents=True, exist_ok=True)

items = []
for dataset in args.datasets:
    dataset = Path(dataset).expanduser().resolve()
    for image in sorted((dataset / "Camera" / "rgb").glob("*.png")):
        items.append((dataset.name, image, dataset / "Camera" / "object_detection" / f"{image.stem}.txt"))
random.Random(0).shuffle(items)
val_count = int(len(items) * args.val)

boxes = empty = 0
for index, (tag, image, label) in enumerate(items):
    split = "val" if index < val_count else "train"
    width, height = Image.open(image).size
    lines = []
    for row in label.read_text().splitlines() if label.exists() else []:
        field = row.split()
        if field[0] != "palletjack":               # 창고에 원래 붙어 있던 이름(floor, wall ...)은 버린다
            continue
        x1, y1, x2, y2 = map(float, field[4:8])
        if x2 - x1 < args.min_pixels or y2 - y1 < args.min_pixels:
            continue
        lines.append(f"0 {(x1 + x2) / 2 / width:.6f} {(y1 + y2) / 2 / height:.6f} "
                     f"{(x2 - x1) / width:.6f} {(y2 - y1) / height:.6f}")
    boxes += len(lines)
    empty += not lines
    name = f"{tag}_{image.stem}"
    (out / "labels" / split / f"{name}.txt").write_text("\n".join(lines) + ("\n" if lines else ""))
    link = out / "images" / split / f"{name}.png"
    if not link.exists():
        link.symlink_to(image)

(out / "data.yaml").write_text(f"path: {out}\ntrain: images/train\nval: images/val\nnames:\n  0: palletjack\n")
print(f"영상 {len(items)}장 (학습 {len(items) - val_count}, 검증 {val_count}), 팔레트잭 상자 {boxes}개, 팔레트잭이 없는 영상 {empty}장")
print(f"저장: {out / 'data.yaml'}")
