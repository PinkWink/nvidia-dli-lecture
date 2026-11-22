"""강의 49: 컵 찾는 모델(강의 47)을 ONNX · TensorRT 엔진으로 바꾸고, 같은 시험 영상으로 정확도 · 속도 · 파일 크기를 견줍니다.

    ~/yolo_venv/bin/python 49_tensorrt/export_compare.py                  # PyTorch · ONNX · TensorRT FP32 · FP16 (엔진을 만드는 데 몇 분)
    ~/yolo_venv/bin/python 49_tensorrt/export_compare.py --only fp16      # 하나만
    ~/yolo_venv/bin/python 49_tensorrt/export_compare.py --model cups_m   # 다른 모델: work/cup_train/cups_m (큰 모델 yolo11m 으로 학습한 것)
    ~/yolo_venv/bin/python 49_tensorrt/export_compare.py --json compare.json

시험 영상은 학습에 쓰지 않은 것으로 따로 만든다:
    ~/isaacsim/python.sh 47_pl2m_vision/make_dataset.py --count 200 --seed 1 --out work/cup_yolo_test

결과: work/cup_engines/<모델 이름>/ 에 best.pt · best.onnx · best_fp32.engine · best_fp16.engine
"""
import argparse
import json
import os
import shutil
import time

from ultralytics import YOLO

HERE = os.path.dirname(os.path.abspath(__file__))
WORK = os.path.abspath(os.path.join(HERE, "..", "work"))
TEST = os.path.join(WORK, "cup_yolo_test")                                    # 시험 영상 200장 (make_dataset.py --seed 1)
SOURCE = OUT = None                                                            # 학습한 모델 파일, 결과를 둘 폴더 (main 에서 정한다)
SIZE = 640                                                                     # 모델에 넣는 영상의 한 변 [픽셀]
NAMES = {"pt": "PyTorch", "onnx": "ONNX Runtime", "fp32": "TensorRT FP32", "fp16": "TensorRT FP16"}


def export(kind):
    """모델을 그 형식으로 바꿔 OUT 에 둔다. (파일, 걸린 시간 [s])"""
    source = shutil.copy(SOURCE, os.path.join(OUT, "best.pt"))                 # 내보낸 파일은 .pt 옆에 생긴다
    if kind == "pt":
        return source, 0.0
    started = time.time()
    if kind == "onnx":
        return YOLO(source).export(format="onnx", imgsz=SIZE, simplify=True), time.time() - started
    made = YOLO(source).export(format="engine", imgsz=SIZE, device=0, half=(kind == "fp16"))     # ONNX 를 거쳐 TensorRT 엔진으로
    return shutil.move(made, os.path.join(OUT, f"best_{kind}.engine")), time.time() - started


def test_data():
    """시험 영상 200장을 모두 쓰는 설정 파일을 만든다. make_dataset.py 는 train · val 두 폴더로 나눠 두므로 둘을 묶는다."""
    names = open(os.path.join(TEST, "data.yaml")).read().split("names:")[1]
    path = os.path.join(OUT, "test.yaml")
    with open(path, "w") as file:
        file.write(f"path: {TEST}\ntrain: images/train\nval: [images/train, images/val]\nnames:{names}")
    return path


def measure(file):
    """시험 영상으로 정확도와 속도를 잰다. 영상은 한 장씩 넣는다 (로봇에서 쓰는 방식).

    rect=False: 모든 형식에 같은 크기(640 × 640, 남는 곳은 회색)로 넣는다. ultralytics 는 .pt 일 때만 영상의 비율에 맞춘 입력을 쓰므로 맞춰 둔다.
    """
    result = YOLO(file, task="detect").val(data=test_data(), imgsz=SIZE, batch=1, rect=False, device=0, verbose=False, plots=False,
                                           project=os.path.join(OUT, "val"), name=os.path.basename(file), exist_ok=True)
    speed = result.speed                                                       # 한 장에 걸린 시간 [ms]: 전처리 · 추론 · 후처리
    return {"mAP50": result.box.map50, "mAP50_95": result.box.map, "inference_ms": speed["inference"],
            "total_ms": speed["preprocess"] + speed["inference"] + speed["postprocess"]}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--only", choices=list(NAMES), help="하나만 한다")
    parser.add_argument("--json", metavar="FILE", help="결과를 이 파일에 저장한다")
    parser.add_argument("--model", default="cups", help="work/cup_train 아래의 학습 결과 이름 (기본: 강의 47 의 cups)")
    args = parser.parse_args()
    global SOURCE, OUT
    SOURCE, OUT = os.path.join(WORK, "cup_train", args.model, "weights", "best.pt"), os.path.join(WORK, "cup_engines", args.model)
    os.makedirs(OUT, exist_ok=True)
    rows = []
    for kind in [args.only] if args.only else list(NAMES):
        file, seconds = export(kind)
        row = {"kind": kind, "file": os.path.basename(file), "size_MB": os.path.getsize(file) / 1e6, "export_s": seconds}
        row.update(measure(file))
        rows.append(row)
    print(f"\n{'형식':<16}{'파일':<20}{'크기 [MB]':>9}{'만드는 데 [s]':>12}{'mAP50':>8}{'mAP50-95':>10}{'추론 [ms]':>10}{'전체 [ms]':>10}")
    for row in rows:
        print(f"{NAMES[row['kind']]:<18}{row['file']:<22}{row['size_MB']:>9.1f}{row['export_s']:>14.0f}{row['mAP50']:>9.3f}{row['mAP50_95']:>10.3f}"
              f"{row['inference_ms']:>11.2f}{row['total_ms']:>11.2f}")
    if args.json:
        with open(args.json, "w") as file:
            json.dump(rows, file, indent=1)
        print("저장:", args.json)


if __name__ == "__main__":
    main()
