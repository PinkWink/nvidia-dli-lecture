# 28. 합성 데이터로 YOLO 학습: 다른 Pinky Pro 를 보고 멈추기 — 명령 모음

영상에서 입력하는 터미널 명령과 기대 출력을 순서대로 모았습니다. [27. 합성 데이터 만들기](../27_pinky_sdg/README.md) 다음 내용입니다.

| 파일 | 내용 |
|---|---|
| [to_yolo.py](to_yolo.py) | 강의 27 의 데이터(KITTI 형식)를 YOLO 학습 형식으로 바꾼다 |
| [pinky_detect.py](pinky_detect.py) | 로봇 카메라에서 다른 Pinky Pro 를 찾는 ROS 2 노드 (`/pinky/debug`, `/pinky/nearest`) |

이 강의에서 다른 강의의 코드에 더한 것: `16_pink_city/pink_city.py` 의 `--parked X Y 방향` (서 있는 Pinky Pro 놓기), `21_lane_follow/lane_follow.py` 의 `stop_for_pinky` (앞 로봇을 보고 멈추기).

## 1. 준비: YOLO 를 설치할 가상환경

### 가상환경 만들기 (한 번)

```bash
python3 -m venv ~/yolo_venv
~/yolo_venv/bin/pip install ultralytics
~/yolo_venv/bin/python -c "import torch, ultralytics; print(torch.__version__, torch.cuda.is_available(), ultralytics.__version__)"
```

```text
2.15.0+cu132 True 8.4.192
```

- **가상환경**은 파이썬 패키지를 따로 담아 두는 폴더입니다. YOLO(ultralytics)는 자기 버전의 PyTorch 를 설치하므로, Isaac Sim 의 파이썬과 섞이지 않게 따로 둡니다.
- 가운데의 `True` 가 GPU 를 쓸 수 있다는 뜻입니다.
- 내려받는 양이 수 GB 입니다. 강의 PC 에서는 2026년 10월에 설치했고(그때 폴더 크기 5.9 GB), 위 출력은 그 환경의 버전입니다. 지금 설치하면 더 새 버전이 깔립니다.

## 2. 데이터를 YOLO 형식으로

```bash
cd ~/nvidia-dli-lecture
git pull
python3 28_pinky_detect/to_yolo.py
```

```text
학습용 540장 (상자 985개), 검증용 60장 (상자 101개)
설정 파일: /home/내이름/nvidia-dli-lecture/work/pinky_yolo/data.yaml
```

### 무엇이 바뀌나

| | KITTI (강의 27) | YOLO |
|---|---|---|
| 한 줄 | `pinky 0.00 0 0.00 39 76 104 139 …` | `0 0.111719 0.298611 0.101562 0.175000` |
| 뜻 | 이름, 상자의 왼쪽 · 위 · 오른쪽 · 아래 (픽셀) | 종류 번호, 상자의 가운데 x · y, 너비, 높이 (영상 크기로 나눈 0 ~ 1) |

- 600장 중 60장(10 %)을 검증용으로 떼어 둡니다. 학습에 쓰지 않은 영상으로 점수를 매기기 위해서입니다.
- 영상은 복사하지 않고 원본을 가리키는 링크로 둡니다.

### data.yaml

```bash
cat work/pinky_yolo/data.yaml
```

```text
path: /home/내이름/nvidia-dli-lecture/work/pinky_yolo
train: images/train
val: images/val
names:
  0: pinky
```

## 3. 학습

```bash
cd ~/nvidia-dli-lecture/work/pinky_yolo
~/yolo_venv/bin/yolo detect train model=yolo11n.pt data=$PWD/data.yaml epochs=40 imgsz=640 batch=16 project=$PWD/runs name=pinky
```

```text
YOLO11n summary (fused): 101 layers, 2,597,051 parameters, 0 gradients, 6.4 GFLOPs
                 Class     Images  Instances      Box(P          R      mAP50  mAP50-95)
                   all         60        101      0.967       0.97      0.984      0.935
Speed: 0.3ms preprocess, 2.7ms inference, 0.0ms loss, 0.4ms postprocess per image
Results saved to /home/내이름/nvidia-dli-lecture/work/pinky_yolo/runs/pinky
```

- `yolo11n.pt` 는 일상 물체 80종으로 미리 학습된 가장 작은 YOLO 입니다. 처음 실행할 때 저절로 받습니다. 이것을 Pinky Pro 한 종류로 다시 학습시킵니다(미세 조정).
- `epochs=40` 은 540장을 40번 되풀이해 본다는 뜻입니다.
- 강의 PC: 1분 26초. 결과 모델은 `runs/pinky/weights/best.pt` (5.5 MB).

### 점수 읽기

| 점수 | 값 | 뜻 |
|---|---|---|
| P (정밀도) | 0.967 | 로봇이라고 답한 것 중 진짜 로봇인 비율 |
| R (재현율) | 0.970 | 진짜 로봇 중 찾아낸 비율 |
| mAP50 | 0.984 | 상자가 정답과 절반 이상 겹치면 맞은 것으로 칠 때의 종합 점수 |
| mAP50-95 | 0.935 | 더 정확히 겹쳐야 맞은 것으로 칠 때의 종합 점수 |

- 검증용 60장(로봇 101대)으로 매긴 점수입니다. 이 60장도 핑크시티의 합성 영상입니다.

### 학습 곡선

- `runs/pinky/results.png` 입니다. 위 줄이 학습용, 아래 줄이 검증용의 오차이고, 오른쪽 네 칸이 점수입니다. 오차는 내려가고 점수는 올라가 평평해졌습니다.

### 검증 영상에 그린 예측

- `runs/pinky/val_batch0_pred.jpg` 입니다. 상자 옆의 숫자는 믿는 정도(0 ~ 1)입니다.

## 4. 달리는 로봇의 카메라에서 찾기

터미널 세 개를 씁니다.

### 터미널 1: 핑크시티, 서 있는 Pinky Pro 두 대

```bash
source /opt/ros/jazzy/setup.bash
~/isaacsim/python.sh ~/nvidia-dli-lecture/16_pink_city/pink_city.py --ros2 --gui --hide-lidar \
    --parked -1.15 0.2 90 --parked -0.9 0.8 -60
```

```text
서 있는 Pinky Pro 0: (-1.15, 0.2) 방향 90.0°
서 있는 Pinky Pro 1: (-0.9, 0.8) 방향 -60.0°
ROS 2 연결: ROS_DOMAIN_ID = 0, 시작 위치 (-1.05, -0.5) 방향 90.0°
```

- `--parked X Y 방향` 은 이번 강의에서 더한 옵션입니다. 그 자리에 움직이지 않는 Pinky Pro 를 한 대 놓습니다. 여러 번 쓸 수 있습니다.
- 서 있는 로봇은 관절과 강체 설정을 지운 사본입니다. 충돌체는 남아 있어 부딪히고 라이다에도 보입니다.

### 터미널 2: 탐지 노드

```bash
source /opt/ros/jazzy/setup.bash
~/yolo_venv/bin/python ~/nvidia-dli-lecture/28_pinky_detect/pinky_detect.py
```

```text
[INFO] [pinky_detect]: Pinky Pro 2대  (가장 가까운 것: 가로 -0.24, 높이 0.31, 믿는 정도 0.95)  1초 동안 60장, 한 장 7 ms
[INFO] [pinky_detect]: Pinky Pro 2대  (가장 가까운 것: 가로 -0.24, 높이 0.31, 믿는 정도 0.95)  1초 동안 60장, 한 장 8 ms
```

- 가상환경의 파이썬으로 실행합니다. ROS 2 를 먼저 `source` 하면 그 파이썬에서도 `rclpy` 를 쓸 수 있습니다.
- 강의 PC: 한 장에 7 ~ 9 ms, 카메라가 보내는 60장을 모두 처리합니다 (GPU 사용).

### 터미널 3: 결과 보기

```bash
source /opt/ros/jazzy/setup.bash
ros2 run rqt_image_view rqt_image_view /pinky/debug
```

- 0.7 m 앞에 뒤를 보이고 선 로봇(0.95)과 1.3 m 앞에 비스듬히 선 로봇(0.92)을 모두 찾았습니다.
- 이 장면은 학습 데이터에 없던 배치입니다.

### 노드가 내보내는 것

| 토픽 | 형식 | 내용 |
|---|---|---|
| `/pinky/debug` | `sensor_msgs/Image` | 상자를 그린 영상 |
| `/pinky/nearest` | `geometry_msgs/Point` | 가장 크게 보이는 로봇: `x` 화면 가로 위치(−1 ~ +1), `y` 상자 높이 ÷ 화면 높이, `z` 믿는 정도. 없으면 `z` = 0 |

```bash
ros2 topic echo --once /pinky/nearest
```

```text
x: -0.2388627976179123
y: 0.30709317326545715
z: 0.9538930654525757
```

- 상자 높이(`y`)는 거리를 대신합니다. 가까울수록 크게 보입니다. 0.7 m 에서 0.31 이었습니다.

## 5. 앞에 로봇이 있으면 멈추기

### 터미널 1: 같은 차로 앞에 로봇 한 대

터미널 1 을 끄고 다시 실행합니다.

```bash
~/isaacsim/python.sh ~/nvidia-dli-lecture/16_pink_city/pink_city.py --ros2 --gui --hide-lidar \
    --start -1.15 0.5 -90 --parked -1.15 -0.7 -90
```

- 강의 21 의 시작 위치(바깥 차로)에서 출발하고, 같은 차로 1.2 m 앞에 로봇을 세워 둡니다.

### 터미널 2: 탐지 노드 (그대로)

```bash
~/yolo_venv/bin/python ~/nvidia-dli-lecture/28_pinky_detect/pinky_detect.py
```

### 터미널 3: 차선 추종 + 멈추기

```bash
source /opt/ros/jazzy/setup.bash
python3 ~/nvidia-dli-lecture/21_lane_follow/lane_follow.py --ros-args -p stop_for_pinky:=0.5
```

```text
[INFO] [lane_follow]: 앞에 Pinky Pro 가 있어 정지: 영상 32 장/초, 치우침 +0.11
[INFO] [lane_follow]: 앞에 Pinky Pro 가 있어 정지: 영상 31 장/초, 치우침 +0.11
```

- `stop_for_pinky:=0.5` 는 "앞 로봇의 상자 높이가 화면 높이의 절반을 넘으면 선다"는 뜻입니다. 이 값을 주지 않으면 강의 21 과 똑같이 동작합니다.

### 멈춘 순간: 탐지 화면과 차선 화면

- 왼쪽이 `/pinky/debug`, 오른쪽이 `/lane/debug` 입니다. 차선 추종의 목표(분홍 점)는 여전히 앞을 가리키지만, 탐지 결과 때문에 속도 명령이 0 입니다.

### 측정 (강의 PC)

| 시간 | 달린 거리 | 속도 |
|---|---|---|
| 2.5초 | 0.23 m | 0.14 m/s |
| 4.6초 | 0.47 m | 0.15 m/s |
| 6.6초 | 0.70 m | 0.14 m/s |
| 8.7초 | 0.76 m | 0 |
| 39초 | 0.76 m | 0 |

- 0.76 m 를 달린 뒤 멈췄습니다. 앞 로봇까지 중심 사이 0.44 m 가 남았고, 그때 상자 높이는 0.54 였습니다.
- 그 뒤 30초 동안 그대로 서 있었습니다.

### lane_follow.py 에 더한 부분

```python
def blocked(self):
    if self.pinky is None or time.monotonic() - self.pinky[1] > 0.5:
        return False
    nearest = self.pinky[0]
    return nearest.z > 0.0 and abs(nearest.x) < 0.6 and nearest.y >= self.stop_for_pinky
```

- 화면 가운데 근처(`|x| < 0.6`)에 있고 충분히 크게 보일 때만 섭니다. 옆 차로나 멀리 있는 로봇에는 서지 않습니다.
- 탐지 노드가 0.5초 넘게 소식이 없으면 "없다"로 봅니다.
