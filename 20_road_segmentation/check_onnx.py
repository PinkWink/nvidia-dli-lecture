"""강의 20: 내보낸 ONNX 모델을 Isaac Sim 밖(시스템 파이썬 + OpenCV)에서 돌려 봅니다.

    python3 20_road_segmentation/check_onnx.py

다음 강의의 ROS 2 노드는 시스템 파이썬에서 돕니다. 거기에는 PyTorch 가 없으므로, ONNX 파일을 OpenCV 로 읽어 씁니다.
필요한 것: python3-opencv, python3-numpy (ROS 2 를 설치했다면 이미 있습니다).
"""
import argparse
import os
import time

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, ".."))
NAMES = ["그 밖", "보도 · 공원", "도로", "중앙선", "횡단보도 · 정지선"]
PALETTE = np.array([(40, 40, 48), (232, 150, 190), (90, 96, 110), (250, 204, 60), (255, 255, 255)], dtype=np.uint8)   # RGB
WIDTH, HEIGHT, MEAN, STD = 320, 180, 0.5, 0.25           # train.py 와 같은 값이어야 한다


def segment(net, bgr):
    """OpenCV 영상(BGR) 한 장 → 종류 번호 그림 (180 × 320)."""
    rgb = cv2.cvtColor(cv2.resize(bgr, (WIDTH, HEIGHT), interpolation=cv2.INTER_AREA), cv2.COLOR_BGR2RGB)
    x = ((rgb.astype(np.float32) / 255.0 - MEAN) / STD).transpose(2, 0, 1)[None]      # (1, 3, H, W)
    net.setInput(x)
    return net.forward()[0].argmax(0).astype(np.uint8)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default=os.path.join(REPO, "work", "road_model", "road_seg.onnx"))
    parser.add_argument("--data", default=os.path.join(REPO, "work", "road_dataset"))
    parser.add_argument("--image", default=os.path.join(REPO, "work", "road_model", "onnx_check.png"), help="비교 그림을 저장할 곳")
    args = parser.parse_args()
    net = cv2.dnn.readNetFromONNX(args.model)
    with open(os.path.join(args.data, "val.txt")) as f:
        ids = f.read().split()
    inter, union, seconds, rows = np.zeros(len(NAMES)), np.zeros(len(NAMES)), 0.0, []
    for k, name in enumerate(ids):
        bgr = cv2.imread(os.path.join(args.data, "images", name + ".png"))
        truth = cv2.imread(os.path.join(args.data, "masks", name + ".png"), cv2.IMREAD_GRAYSCALE)[1::2, 1::2]     # 절반 크기로 (train.py 와 같은 픽셀을 고른다)
        start = time.time()
        answer = segment(net, bgr)
        seconds += time.time() - start
        for c in range(len(NAMES)):
            inter[c] += ((answer == c) & (truth == c)).sum()
            union[c] += ((answer == c) | (truth == c)).sum()
        if k < 4:
            small = cv2.resize(bgr, (WIDTH, HEIGHT))
            rows.append(np.concatenate([small, PALETTE[truth][:, :, ::-1], PALETTE[answer][:, :, ::-1]], axis=1))
    cv2.imwrite(args.image, np.concatenate(rows, axis=0))
    iou = inter / np.maximum(union, 1)
    print(f"OpenCV {cv2.__version__} 로 {len(ids)}장 추론: 한 장에 {seconds / len(ids) * 1000:.0f} ms (CPU)")
    for name, v in zip(NAMES, iou):
        print(f"  {name:<12} IoU {v:.3f}")
    print(f"평균 IoU {iou.mean():.3f}")
    print("비교 그림 저장:", args.image)
