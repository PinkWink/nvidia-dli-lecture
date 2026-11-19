#!/usr/bin/env python3
"""강의 28: 강의 27 의 데이터(KITTI 형식)를 YOLO 학습 형식으로 바꿉니다.

    python3 28_pinky_detect/to_yolo.py                     # work/pinky_dataset → work/pinky_yolo
    python3 28_pinky_detect/to_yolo.py --data work/pinky_dataset_test --out work/pinky_yolo_test

KITTI 한 줄   pinky 0.00 0 0.00  왼쪽 위 오른쪽 아래  …        (픽셀)
YOLO 한 줄    0  가운데x 가운데y 너비 높이                      (영상 크기로 나눈 0 ~ 1 값)

만드는 것 (work/pinky_yolo)
    images/train · images/val     영상 (복사하지 않고 원본을 가리키는 링크)
    labels/train · labels/val     YOLO 정답
    data.yaml                     학습 명령에 넘기는 설정 파일
"""
import argparse
import os
import random

from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, ".."))
CLASS = "pinky"


def convert(label_path, width, height, min_size):
    """KITTI 정답 파일 → YOLO 줄들. 너무 작은 상자는 뺀다."""
    lines = []
    if os.path.exists(label_path):
        for line in open(label_path):
            v = line.split()
            if len(v) < 8 or v[0] != CLASS:
                continue
            left, top, right, bottom = (float(t) for t in v[4:8])
            w, h = right - left, bottom - top
            if w < min_size or h < min_size:
                continue
            lines.append(f"0 {(left + right) / 2 / width:.6f} {(top + bottom) / 2 / height:.6f} {w / width:.6f} {h / height:.6f}")
    return lines


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--data", default=os.path.join(REPO, "work", "pinky_dataset"), help="강의 27 이 만든 폴더")
    parser.add_argument("--out", default=os.path.join(REPO, "work", "pinky_yolo"), help="YOLO 형식으로 저장할 폴더")
    parser.add_argument("--val", type=float, default=0.1, help="검증용으로 떼어 둘 비율")
    parser.add_argument("--min-size", type=float, default=6.0, help="이보다 작은 상자 [픽셀] 는 뺀다")
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args()

    data, out = os.path.abspath(os.path.expanduser(args.data)), os.path.abspath(os.path.expanduser(args.out))
    rgb, labels = os.path.join(data, "RobotCamera", "rgb"), os.path.join(data, "RobotCamera", "object_detection")
    names = sorted((f[:-4] for f in os.listdir(rgb) if f.endswith(".png")), key=int)
    random.Random(args.seed).shuffle(names)
    n_val = max(1, int(len(names) * args.val))
    boxes = {"train": 0, "val": 0}
    for split, chosen in (("val", names[:n_val]), ("train", names[n_val:])):
        for folder in ("images", "labels"):
            os.makedirs(os.path.join(out, folder, split), exist_ok=True)
        for name in chosen:
            image = os.path.join(rgb, name + ".png")
            width, height = Image.open(image).size
            lines = convert(os.path.join(labels, name + ".txt"), width, height, args.min_size)
            link = os.path.join(out, "images", split, name + ".png")
            if os.path.lexists(link):
                os.remove(link)
            os.symlink(image, link)
            with open(os.path.join(out, "labels", split, name + ".txt"), "w") as f:      # 로봇이 없는 영상은 빈 파일
                f.write("\n".join(lines) + ("\n" if lines else ""))
            boxes[split] += len(lines)
    with open(os.path.join(out, "data.yaml"), "w") as f:
        f.write(f"path: {out}\ntrain: images/train\nval: images/val\nnames:\n  0: {CLASS}\n")
    print(f"학습용 {len(names) - n_val}장 (상자 {boxes['train']}개), 검증용 {n_val}장 (상자 {boxes['val']}개)")
    print(f"설정 파일: {os.path.join(out, 'data.yaml')}")


if __name__ == "__main__":
    main()
