# 49. TensorRT 로 모델 빠르게 돌리기: 빨라진 만큼이 로봇에서도 빨라지나 — 명령 모음

- 영상에서 입력하는 터미널 명령과 기대 출력을 순서대로 모았음. [48. 찾은 컵 집기](../48_pl2m_find_pick/README.md) 다음 내용임.

| 파일 |
|---|
| [export_compare.py](export_compare.py) |

## 1. 같은 모델을 더 빠르게

### 이번 강의의 결과

- 강의 47 에서 학습한 컵 찾는 모델을 TensorRT 엔진으로 바꿨음. 영상 한 장의 추론이 2.45 ms 에서 0.54 ms 로 줄었음. 정확도(mAP50)는 같음.
- 그런데 로봇의 컵 찾는 노드에서는 한 장에 19 ms 걸리던 것이 18 ms 가 됐음. 이 강의는 그 차이가 어디서 오는지를 재 봄.

### TensorRT 가 하는 일

| | PyTorch 모델 (`.pt`) | TensorRT 엔진 (`.engine`) |
|---|---|---|
| 담긴 것 | 층의 구조와 가중치 | 이 GPU 에 맞춰 미리 짠 계산 순서 |
| 실행 | 층을 하나씩 부름 | 층을 합치고, GPU 에 맞는 구현을 골라 둠 |
| 숫자 | 32비트 실수 | 32비트(FP32) 또는 16비트(FP16) |
| 옮기기 | 어느 PC 로나 | 만든 GPU 와 TensorRT 버전에서만 |

- 학습은 PyTorch 로 하고, 돌릴 때만 엔진으로 바꿈. 가는 길에 ONNX 라는 중간 형식을 거침.
- 엔진은 다른 PC 로 옮겨 쓰지 않음. 옮길 것은 ONNX 파일이고, 엔진은 쓸 PC 에서 만듦.

### NVIDIA 과정에서는

| | NVIDIA 과정 | 이 강의 |
|---|---|---|
| 모델 | 팔레트잭을 찾는 YOLO | 컵을 찾는 YOLO (강의 47) |
| 엔진을 돌리는 곳 | Isaac ROS 의 TensorRT 노드 (컨테이너) | 강의 47 의 `cup_detector.py` |
| 재는 것 | 형식별 정확도 · 속도 | 그것 + 로봇의 노드에서 단계별 시간 |

- Isaac ROS 의 노드로 옮기는 것은 하지 않았음. 이미 도는 파이썬 노드에 엔진 파일만 바꿔 끼움.

### 만드는 것

| 파일 | 하는 일 |
|---|---|
| `export_compare.py` | 모델을 ONNX · FP32 · FP16 으로 바꾸고 같은 시험 영상으로 견줌 |

- 로봇에서 재는 데는 강의 47 의 `cup_detector.py` 에 `--timing` 을 더했음.

## 2. 준비

### 터미널: 시험 영상 200장

```bash
cd ~/nvidia-dli-lecture
git pull
~/isaacsim/python.sh 47_pl2m_vision/make_dataset.py --count 200 --seed 1 --out work/cup_yolo_test
```

```text
저장: 영상 200장 (학습 180, 검증 20), 27초
상자: cup_pink 131개,  cup_blue 138개,  cup_yellow 141개,  컵이 없는 영상 6장
```

- 강의 47 의 데이터와 씨앗이 달라 학습에 쓰지 않은 영상임. 정확도는 이 200장으로 잼.
- 출력의 "학습 180, 검증 20" 은 폴더를 나눈 것일 뿐임. 이 강의에서는 200장을 모두 시험에 씀.

### TensorRT 패키지

```bash
~/yolo_venv/bin/python -c "import tensorrt; print(tensorrt.__version__)"
```

```text
10.16.1.11
```

- 강의 PC 의 가상환경에 설치돼 있는 버전임. 없다면 `~/yolo_venv/bin/pip install tensorrt` 로 설치함. 새 환경에서 설치해 보지는 않았음.

## 3. 바꾸고 견주기

### 터미널: 네 가지 형식

```bash
~/yolo_venv/bin/python 49_tensorrt/export_compare.py
```

```text
형식              파일                    크기 [MB]   만드는 데 [s]   mAP50  mAP50-95   추론 [ms]   전체 [ms]
PyTorch           best.pt                     5.5             0    0.994     0.897       2.45       3.14
ONNX Runtime      best.onnx                  10.7             1    0.994     0.897       2.31       3.00
TensorRT FP32     best_fp32.engine           11.7            11    0.994     0.897       1.10       1.76
TensorRT FP16     best_fp16.engine            7.4            11    0.994     0.892       0.54       1.25
```

- 결과는 `work/cup_engines/cups/` 에 생김. 엔진 하나를 만드는 데 11초 걸렸음.
- 추론은 FP32 엔진이 2.2배, FP16 엔진이 4.5배 빠름. mAP50 은 넷 다 0.994 임.
- "전체" 는 전처리와 후처리를 더한 것임. 그 둘은 형식을 바꿔도 그대로라 전체로는 2.5배임.

### 코드: 내보내기

```python
if kind == "onnx":
    return YOLO(source).export(format="onnx", imgsz=SIZE, simplify=True), time.time() - started
made = YOLO(source).export(format="engine", imgsz=SIZE, device=0, half=(kind == "fp16"))     # ONNX 를 거쳐 TensorRT 엔진으로
```

- `format="engine"` 이 TensorRT 엔진임. `half=True` 면 FP16 으로 만듦.
- 입력의 크기(640 × 640)는 만들 때 정해짐. 다른 크기로 쓰려면 다시 만들어야 함.

### 코드: 같은 조건으로 재기

```python
result = YOLO(file, task="detect").val(data=test_data(), imgsz=SIZE, batch=1, rect=False, device=0, verbose=False, plots=False, ...)
speed = result.speed                                                       # 한 장에 걸린 시간 [ms]: 전처리 · 추론 · 후처리
```

- 영상은 한 장씩 넣음 (`batch=1`). 로봇의 카메라가 한 장씩 내기 때문임.
- `rect=False` 는 모든 형식에 같은 크기로 넣으라는 뜻임. ultralytics 는 `.pt` 일 때만 영상의 비율에 맞춘 입력을 쓰므로 맞춰 둠.

### FP16 이 잃는 것

| | FP32 | FP16 |
|---|---|---|
| 숫자 하나 | 32비트 | 16비트 |
| 엔진의 크기 | 11.7 MB | 7.4 MB |
| mAP50 | 0.994 | 0.994 |
| mAP50-95 | 0.897 | 0.892 |

- 숫자를 절반의 비트로 줄였는데 mAP50 은 그대로임. 상자를 더 엄격하게 채점하는 mAP50-95 만 0.005 내려감.
- 로봇에서의 컵의 자리는 4절에서 다시 견줌.

## 4. 로봇에서 재 보기

### 터미널 1 · 2: 시뮬레이터와 카메라의 자리

```bash
# 터미널 1
source /opt/ros/jazzy/setup.bash
~/isaacsim/python.sh 47_pl2m_vision/pl2m_vision.py --start drinks --docked --shuffle 3
# 터미널 2
source /opt/ros/jazzy/setup.bash
source ~/pinklab_robotworld2026/install/setup.bash
ros2 launch 45_pl2m_dock/dock.launch.py
```

- 강의 47 의 5절과 같음.

### 터미널 3: 엔진으로 컵 찾기

```bash
source /opt/ros/jazzy/setup.bash
source ~/pinklab_robotworld2026/install/setup.bash
~/yolo_venv/bin/python 47_pl2m_vision/cup_detector.py --timing --weights work/cup_engines/cups/best_fp16.engine
```

```text
[    5.8초] 영상 10장, 한 장에 18 ms:  cup_blue 0.91 앞 0.492 · 왼쪽 -0.065 · 높이 0.796 m,  cup_pink 0.87 앞 0.647 · 왼쪽 +0.138 · 높이 0.795 m, ...
            영상 변환 0.9 ms · YOLO 13.5 ms (전처리 1.6 · 추론 3.9 · 후처리 7.7 · 그 밖 0.3) · 자리 구하기와 내보내기 3.1 ms
```

- `--weights` 에 엔진 파일을 주면 됨. 노드의 코드는 바꾸지 않음. ultralytics 가 파일의 형식을 보고 알아서 돌림.
- `--timing` 은 한 장을 처리하는 시간을 단계로 나눠 1초마다 냄.

### 같은 것을 PyTorch 모델로

```bash
~/yolo_venv/bin/python 47_pl2m_vision/cup_detector.py --timing --weights work/cup_engines/cups/best.pt
```

```text
            영상 변환 0.9 ms · YOLO 16.4 ms (전처리 2.1 · 추론 8.5 · 후처리 5.5 · 그 밖 0.4) · 자리 구하기와 내보내기 2.1 ms
```

- 추론은 8.5 ms 에서 3.9 ms 로 줄었음. 한 장 전체는 19 ms 에서 18 ms 임.

### 어디에 시간이 드나

| | 추론 | 추론이 아닌 것 | 한 장 전체 |
|---|---|---|---|
| 작은 모델 · PyTorch | 8.7 ms | 10.8 ms | 19.5 ms |
| 작은 모델 · FP16 | 3.9 ms | 13.8 ms | 17.7 ms |

- 추론을 4.8 ms 줄였지만 전체는 1.8 ms 만 줄었음. 엔진으로 바꾸자 후처리가 5.7 ms 에서 8.0 ms 로 늘었음. 그 까닭은 확인하지 않았음.
- 노드가 쓰는 시간의 절반 넘게가 추론이 아님. 모델만 빠르게 해서는 그만큼 빨라지지 않음.
- 표는 1초씩 네 번(영상 40장쯤)의 평균임.

### 혼자 잴 때와 다른 까닭

| | 3절 (모델만) | 4절 (로봇의 노드) |
|---|---|---|
| GPU 를 쓰는 것 | 이 모델 하나 | 이 모델 + Isaac Sim |
| PyTorch 추론 | 2.45 ms | 8.7 ms |
| FP16 추론 | 0.54 ms | 3.9 ms |

- 3절은 GPU 를 혼자 쓰며 잰 것이고, 4절은 Isaac Sim 이 같은 GPU 로 장면을 그리는 중에 잰 것임.
- 실제 로봇에는 시뮬레이터가 없으므로 3절에 가까울 것임. 로봇의 컴퓨터에서 재 보지는 않았음.

### 컵의 자리는 같은가

| | 파란 컵 | 분홍 컵 | 노란 컵 |
|---|---|---|---|
| PyTorch | 3.3 mm | 2.2 mm | 2.4 mm |
| TensorRT FP16 | 3.4 mm | 2.4 mm | 2.4 mm |

- 강의 47 의 `check_cups.py` 로 참값과의 거리를 잰 것임 (영상 50장의 평균). 0.2 mm 안에서 같음.
- 엔진으로 바꿔도 강의 48 의 임무에 쓸 수 있는 정확도임.

## 5. 모델이 크면

### 터미널: 큰 모델로 학습

```bash
~/yolo_venv/bin/yolo detect train data=work/cup_yolo/data.yaml model=yolo11m.pt epochs=40 imgsz=640 project=$PWD/work/cup_train name=cups_m
```

```text
40 epochs completed in 0.053 hours.
YOLO11m summary (fused): 125 layers, 20,032,345 parameters, 0 gradients, 67.8 GFLOPs
                   all         40         85      0.974      0.988      0.984      0.926
```

- 강의 47 과 같은 데이터로 `yolo11n` 대신 `yolo11m` 을 학습했음. 가중치가 260만 개에서 2,000만 개로 늘어남. 3분 11초 걸렸음.

### 터미널: 바꾸고 견주기

```bash
~/yolo_venv/bin/python 49_tensorrt/export_compare.py --model cups_m
```

```text
형식              파일                    크기 [MB]   만드는 데 [s]   mAP50  mAP50-95   추론 [ms]   전체 [ms]
PyTorch           best.pt                    40.5             0    0.995     0.924       9.23       9.91
ONNX Runtime      best.onnx                  80.4             1    0.995     0.924       7.71       8.38
TensorRT FP32     best_fp32.engine           82.6            15    0.995     0.924       5.03       5.66
TensorRT FP16     best_fp16.engine           42.4            13    0.995     0.920       1.73       2.38
```

- 추론이 9.23 ms 에서 1.73 ms 로 5.3배 빨라짐. FP16 으로 돌린 큰 모델이 PyTorch 로 돌린 작은 모델(2.45 ms)보다 빠름.

### 로봇에서

| | 추론 | 한 장 전체 |
|---|---|---|
| 큰 모델 · PyTorch | 19.5 ms | 28.5 ms |
| 큰 모델 · FP16 | 5.1 ms | 18.8 ms |
| (작은 모델 · FP16) | 3.9 ms | 17.7 ms |

- 큰 모델은 추론이 한 장의 대부분이라, 엔진으로 바꾸면 전체가 9.7 ms 줄어듦.
- FP16 엔진으로는 큰 모델과 작은 모델의 차이가 1.1 ms 임. 엔진으로 바꾸면 같은 시간에 더 큰 모델을 쓸 수 있음.
- 이 컵 세 개에는 작은 모델로도 mAP50 이 0.994 라 큰 모델이 필요하지 않음. 물체가 많고 어려운 일에서 쓸 수 있는 여유임.

## 6. 필요한 만큼인가

### 카메라는 1초에 10장

| | 값 |
|---|---|
| 머리 카메라가 영상을 내는 간격 | 100 ms |
| 노드가 한 장을 처리하는 시간 (작은 모델 · PyTorch) | 19.5 ms |
| 임무에서 컵을 보는 시간 (강의 48) | 세 번에 2.6초, 임무 전체의 1.4 % |

- 지금의 노드는 영상이 오는 간격의 5분의 1 만에 한 장을 끝냄. 더 빨라져도 임무가 빨라지지 않음.
- 빠르게 하는 일은 재서 모자란 곳에 함. 이 임무에서 시간이 드는 곳은 옆걸음과 팔의 움직임임 (강의 48 의 시간표).

### 이득이 큰 경우

- 모델이 큰 경우. 5절처럼 추론이 한 장의 대부분일 때임.
- 카메라가 빠르거나 여러 대인 경우. 1초에 30장을 내는 카메라 넉 대면 한 장에 8 ms 안에 끝내야 함.
- GPU 가 작은 경우. 로봇에 싣는 컴퓨터는 강의 PC 보다 느림. 그런 컴퓨터에서 재 보지는 않았음.

## 7. 해 볼 것

### 직접 바꿔 보기

- `cup_detector.py --timing --device cpu` 로 GPU 없이 걸리는 시간 보기.
- `export_compare.py --only fp16` 을 두 번 돌려, 잴 때마다 추론 시간이 얼마나 달라지는지 보기.
- 시뮬레이터를 `--gui` 로 띄우고 `--timing` 의 추론 시간이 어떻게 바뀌는지 보기.
- 강의 28 의 팔레트잭 모델을 같은 방법으로 엔진으로 바꿔 보기.

## 8. 정리

### 기억할 것

- TensorRT 엔진은 그 GPU 에 맞춰 미리 짜 둔 계산임. 학습한 모델을 ONNX 를 거쳐 엔진으로 바꿈.
- FP16 엔진은 추론이 4.5 ~ 5.3배 빠르고 정확도는 거의 같았음.
- 로봇의 노드에서는 추론이 전체의 일부임. 단계별로 재야 어디를 줄일지 보임.
- 빨라진 것이 쓸모 있는지는 카메라의 속도와 임무의 시간에 견줘 봄.

### 한계

- INT8 은 다루지 않았음. ultralytics 의 `int8=True` 로 내보내면 강의 PC (RTX 5060 Ti, TensorRT 10.16.1) 에서 엔진을 만들다 `Error Code 10: Internal Error (Could not find any implementation for node ...)` 로 실패했음.
- 엔진으로 바꾸면 후처리 시간이 늘어난 까닭을 확인하지 않았음.
- 로봇에 싣는 컴퓨터(Jetson 등)에서는 재지 않았음. 속도의 숫자는 강의 PC 한 대의 값임.
- Isaac ROS 의 TensorRT 노드는 쓰지 않았음.
