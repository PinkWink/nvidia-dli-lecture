# 20. 도로를 알아보는 모델 학습: 세그먼테이션 — 명령 모음

- 강의 19 에서 모은 영상 400장으로, 카메라 영상의 픽셀마다 "도로 · 중앙선 · 보도 …" 를 맞히는 모델을 학습함. PyTorch 는 Isaac Sim 에 들어 있는 것을 쓰므로 따로 설치하지 않음. [19. 도로 인식 데이터 모으기](../19_road_dataset/README.md) 다음 내용임.

## 1. 준비

- `work/road_dataset` 이 있어야 함 (강의 19).

```bash
cd ~/nvidia-dli-lecture
git pull
ls work/road_dataset
```

```text
images  masks  poses.csv  preview  train.txt  val.txt
```

## 2. Isaac Sim 의 PyTorch 확인

```bash
~/isaacsim/python.sh -c "import torch; print(torch.__version__, torch.cuda.is_available())"
```

```text
2.7.0+cu128 True
```

- `True` = GPU 로 학습할 수 있음. `False` 여도 CPU 로 학습은 되지만 훨씬 오래 걸림.

## 3. 학습

```bash
~/isaacsim/python.sh 20_road_segmentation/train.py
```

```text
장치 cuda (NVIDIA GeForce RTX 5060 Ti), PyTorch 2.7.0+cu128
학습 360장 · 검증 40장, 입력 320 × 180, 모델 파라미터 488,069개
종류별 픽셀 비율 → 가중치: 그 밖 41.0% → 0.31, 보도 · 공원 14.0% → 0.53, 도로 42.9% → 0.30, 중앙선 0.9% → 2.12, 횡단보도 · 정지선 1.3% → 1.74
  에포크     학습 손실     검증 손실   평균 IoU   그 밖  보도 · 공원  도로  중앙선  횡단보도 · 정지선
    1    1.2461    1.3885    0.131   0.173  0.000  0.483  0.000  0.000 ← 저장
    5    0.7438    0.5894    0.665   0.835  0.612  0.934  0.740  0.203 ← 저장
   10    0.3319    0.2503    0.786   0.887  0.674  0.979  0.794  0.596 ← 저장
   20    0.1392    0.1011    0.867   0.933  0.798  0.987  0.871  0.743
   30    0.0728    0.0613    0.904   0.965  0.878  0.991  0.880  0.808
   36    0.0597    0.0517    0.927   0.975  0.913  0.990  0.875  0.881 ← 저장
   40    0.0610    0.0507    0.924   0.976  0.913  0.991  0.876  0.864
학습 시간 56초. 가장 좋은 평균 IoU 0.927
저장한 모델의 종류별 IoU (검증):
  그 밖          0.975
  보도 · 공원      0.913
  도로           0.990
  중앙선          0.875
  횡단보도 · 정지선   0.881
저장: .../work/road_model/road_seg.pt, road_seg.onnx (2.0 MB), training.png, predictions.png
```

- 강의 PC(RTX 5060 Ti)에서 약 1분. 시뮬레이터 창은 뜨지 않음 (`python.sh` 를 파이썬 실행기로만 씀). 숫자는 PC 마다 조금씩 다를 수 있음.

## 4. ONNX 모델을 Isaac Sim 밖에서 돌려 보기

```bash
python3 20_road_segmentation/check_onnx.py
```

```text
OpenCV 4.6.0 로 40장 추론: 한 장에 12 ms (CPU)
  그 밖          IoU 0.976
  보도 · 공원      IoU 0.915
  도로           IoU 0.991
  중앙선          IoU 0.875
  횡단보도 · 정지선   IoU 0.882
평균 IoU 0.927
비교 그림 저장: .../work/road_model/onnx_check.png
```

- 시스템 파이썬(ROS 2 가 쓰는 파이썬)과 OpenCV 만으로 같은 정확도가 나오면, 다음 강의의 ROS 2 노드에서 쓸 준비가 된 것임.

## 만들어지는 것 (`work/road_model`)

| 파일 | 내용 |
|---|---|
| `road_seg.pt` | 가장 잘 맞힌 시점의 모델 (PyTorch) |
| `road_seg.onnx` | 같은 모델을 다른 프로그램에서 쓰는 형식 (2.0 MB). 다음 강의의 ROS 2 노드가 읽음 |
| `history.csv` · `training.png` | 에포크마다의 손실 · IoU |
| `predictions.png` | 검증 영상 6장: 영상, 정답, 모델의 답 |
| `onnx_check.png` | `check_onnx.py` 가 만드는 비교 그림 |

## 옵션 (`train.py`)

| 옵션 | 뜻 |
|---|---|
| `--epochs N` | 데이터 전체를 몇 번 볼지 (기본 40) |
| `--batch N` | 한 번에 넣는 장수 (기본 16) |
| `--lr X` | 학습률 (기본 0.002) |
| `--data 폴더` · `--out 폴더` | 데이터 · 저장 폴더 |

## 막힐 때

| 증상 | 해결 |
|---|---|
| "데이터가 없음" | 강의 19 의 `collect_dataset.py` 를 먼저 실행 |
| `CUDA out of memory` | `--batch 8` |
| 중앙선 IoU 가 0.5 아래 | `--epochs 60`, 또는 데이터를 1000장으로 (`collect_dataset.py --count 1000`) |
| `check_onnx.py` 에서 `No module named 'cv2'` | `sudo apt install python3-opencv` |
