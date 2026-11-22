# 47. 머리 카메라로 컵 찾기: YOLO 와 깊이 영상으로 컵의 자리 구하기 — 명령 모음

- 영상에서 입력하는 터미널 명령과 기대 출력을 순서대로 모았음. [46. 컵을 집어 나르기](../46_pl2m_mission/README.md) 다음 내용임.

| 파일 |
|---|
| [check_cups.py](check_cups.py) |
| [cup_detector.py](cup_detector.py) |
| [locate.py](locate.py) |
| [make_dataset.py](make_dataset.py) |
| [pl2m_vision.py](pl2m_vision.py) |

## 1. 컵이 어디 있는지 로봇이 직접 봄

### 이번 강의의 결과

- 컵 세 개를 음료대 위 아무 데나 놓았음. 로봇이 다가가는 동안 머리 카메라의 영상에서 컵을 찾고, 로봇 기준으로 어디 있는지를 숫자로 냄.
- 오른쪽 화면의 상자 위 글자가 컵의 이름과 점수, 아래 글자가 로봇 기준의 자리(앞, 왼쪽)임.

### 강의 46 에서 남은 것

```python
CUPS = {                                      # 음료대 위의 컵: 가운데 (x, y, z)
    "cup_pink":   (7.50, -1.72, 0.795),
    "cup_blue":   (7.65, -1.72, 0.795),
    "cup_yellow": (7.80, -1.72, 0.795),
}
```

- 강의 46 의 임무는 컵의 자리를 이 표에서 읽었음. 누가 컵을 옆으로 5 cm 만 옮겨도 빈손으로 돌아옴.
- 이 강의는 표를 카메라로 바꿀 준비를 함. 컵을 찾아 자리를 내는 데까지 하고, 그 자리로 집는 것은 다음 강의에서 함.

### NVIDIA 과정에서는

| 단계 | NVIDIA 과정 (Isaac for Manipulation) | 이 강의 |
|---|---|---|
| 물체 찾기 | RT-DETR (SyntheticaDETR) | YOLO11n |
| 물체의 자리 | FoundationPose: 위치와 방향 | 상자 + 깊이 영상: 위치만 |
| 잡는 자세 | Grasp Editor 로 만든 파일 | 강의 43 의 값 (옆에서 쥠) |
| 팔의 경로 | cuMotion | MoveIt (강의 42) |

- 둘 다 합성 데이터로 학습한 모델로 물체를 찾음. NVIDIA 쪽은 단계마다 GPU 를 쓰는 큰 모델이라, 그 과정의 설정 문서는 시뮬레이터 쪽에만 GPU 메모리 25 GB 이상을 요구함.
- 컵은 세워 둔 원기둥이라 방향을 알 필요가 없음. 위치만 구하면 되므로 상자와 깊이 영상으로 충분함.
- 이 방법은 PL-2M 의 원본 저장소가 쓰는 방법임 (`warehouse_lecture/vision/yolo_detector.py`).

### 만드는 것

| 파일 | 어디서 도나 | 하는 일 |
|---|---|---|
| `pl2m_vision.py` | Isaac Sim | 강의 46 의 장면 + 머리 카메라 (색 · 깊이) |
| `make_dataset.py` | Isaac Sim | 학습 데이터 400장 |
| `locate.py` | ROS 2 | 상자 + 깊이 → 로봇 기준의 자리 (계산만) |
| `cup_detector.py` | ROS 2 | 영상에서 컵을 찾아 자리를 냄 |
| `check_cups.py` | ROS 2 | 구한 자리를 참값과 견줌 |

- 순서는 카메라 달기 → 데이터 만들기 → 학습 → 찾기 → 자리 구하기 → 채점임.

## 2. 머리 카메라

### 터미널 1: Isaac Sim

```bash
source /opt/ros/jazzy/setup.bash
cd ~/nvidia-dli-lecture
git pull
~/isaacsim/python.sh 47_pl2m_vision/pl2m_vision.py --start drinks --docked
```

```text
준비됨: 시작 자리 drinks (도킹한 자리) (7.650, -2.235, 90.0°), 머리 카메라 640 × 480. 끝내려면 Ctrl+C
컵: cup_pink (7.500, -1.720),  cup_blue (7.650, -1.720),  cup_yellow (7.800, -1.720)
```

- 강의 46 의 장면에 머리 카메라를 더한 것임. `--docked` 는 음료대에 도킹을 마친 자리에서 시작함.
- 창으로 보려면 `--gui` 를 붙임.

### 카메라가 달린 자리

| | 앞 카메라 (강의 45) | 머리 카메라 |
|---|---|---|
| 달린 곳 | 베이스 앞면, 바닥에서 0.21 m | 몸통 위, 바닥에서 1.10 m |
| 보는 방향 | 정면 | 아래로 14.3° 숙임 |
| 보는 것 | 테이블 앞면의 마커 | 테이블 위 |
| 내는 것 | 색 | 색 + 깊이 |

- 둘 다 URDF 에 자리가 있음 (`base_camera_link`, `head_camera_link`). 원본의 `cameras.yaml` 값 그대로임.
- 앞 카메라는 낮아서 상판 위가 보이지 않음. 컵은 머리 카메라로 봄.

### 코드: 색과 깊이를 함께 냄

```python
helpers = {"ColorHelper": ("isaacsim.ros2.bridge.ROS2CameraHelper", "head_camera/image_raw"),
           "DepthHelper": ("isaacsim.ros2.bridge.ROS2CameraHelper", "head_camera/depth"),
           "InfoHelper": ("isaacsim.ros2.bridge.ROS2CameraInfoHelper", "head_camera/camera_info")}
...
("ColorHelper.inputs:type", "rgb"), ("DepthHelper.inputs:type", "depth")
```

- 강의 45 의 카메라 그래프에 `ROS2CameraHelper` 를 하나 더 두고 `type` 을 `depth` 로 했음.
- 세 토픽은 같은 화면에서 나와 시각(`header.stamp`)이 같음.

### 터미널 2: 토픽 확인

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic list | grep -E 'head|cups'
ros2 topic echo --once /head_camera/depth --no-arr | grep -E 'height|width|encoding'
```

```text
/cups/truth
/head_camera/camera_info
/head_camera/depth
/head_camera/image_raw
height: 480
width: 640
encoding: 32FC1
```

- 깊이 영상은 픽셀마다 32비트 실수 하나임 (`32FC1`). 값은 카메라 앞쪽으로 잰 거리 [m] 임.
- `/cups/truth` 는 컵의 참값임. 채점할 때만 씀.

### 색 영상과 깊이 영상

- 왼쪽이 색, 오른쪽이 깊이임. 깊이는 가까울수록 노랗게 그렸음.
- 흰 상자 안의 가운데 부분에서 읽은 거리가 0.49 m 임. 컵의 보이는 면까지의 거리임.
- 분홍 컵은 분홍 상판 위에 있음. 색만으로 컵을 가려내기 어려운 장면임.

## 3. 학습 데이터 만들기

### 터미널: 400장

```bash
~/isaacsim/python.sh 47_pl2m_vision/make_dataset.py
```

```text
부스에서 400장을 만듭니다 → /home/내이름/nvidia-dli-lecture/work/cup_yolo
저장: 영상 400장 (학습 360, 검증 40), 51초
상자: cup_pink 279개,  cup_blue 278개,  cup_yellow 288개,  컵이 없는 영상 12장
```

- 터미널 1 의 Isaac Sim 은 끄고 실행함. 부스에서 컵과 카메라의 자리를 바꿔 가며 400장을 찍음.
- 결과는 YOLO 가 바로 읽는 형식임. 강의 28 의 `to_yolo.py` 같은 변환이 필요 없음.

### 만든 데이터

- 컵은 음료대와 상담 테이블 두 곳에 놓임. 다음 강의에서 상담 테이블에 놓은 컵도 보려고 넣었음.
- 카메라는 머리 카메라와 같은 렌즈임. 높이 1.0 ~ 1.2 m, 테이블 가장자리에서 0.3 ~ 1.25 m 뒤에 둠.
- 조명의 밝기는 장마다 0.5 ~ 1.6 배로 바꿈.

### 강의 27 과 다른 방법

| | 강의 27 | 이 강의 |
|---|---|---|
| 무작위 규칙 | Replicator 의 그래프 (`rep.trigger.on_frame`) | 파이썬 반복문에서 직접 |
| 한 장 찍기 | `rep.orchestrator.run_until_complete` | `rep.orchestrator.step` |
| 정답 저장 | `KittiWriter` | Annotator 에서 읽어 직접 적음 |

- 그래프 방식은 짧지만 "상판 위에서 서로 겹치지 않게" 같은 조건을 적기 어려움.
- 반복문 방식은 조건을 파이썬으로 그대로 적음. 장면을 바꾸고 `step` 을 부르면 한 장이 찍힘.

### 코드: 한 장을 찍고 정답을 적음

```python
rep.orchestrator.step(rt_subframes=4)
found = boxes_out.get_data()
names = {int(key): value["class"] for key, value in found["info"]["idToLabels"].items()}
for box in found["data"]:
    name = names.get(int(box["semanticId"]))
    width, height = float(box["x_max"] - box["x_min"]), float(box["y_max"] - box["y_min"])
    lines.append(f"{CLASSES.index(name)} {(box['x_min'] + box['x_max']) / 2 / SIZE[0]:.6f} ...")
```

- `boxes_out` 은 `bounding_box_2d_tight` Annotator 임. 보이는 부분만 감싼 상자를 줌.
- 컵에 붙여 둔 이름(`add_labels`)이 `idToLabels` 로 돌아옴. 그 이름의 번호와 상자를 YOLO 형식 한 줄로 적음.

## 4. 학습

### 터미널: YOLO11n 40 에포크

```bash
~/yolo_venv/bin/yolo detect train data=work/cup_yolo/data.yaml model=yolo11n.pt epochs=40 imgsz=640 project=$PWD/work/cup_train name=cups
```

```text
40 epochs completed in 0.015 hours.
                 Class     Images  Instances      Box(P          R      mAP50  mAP50-95)
                   all         40         85      0.981      0.988      0.985      0.922
              cup_pink         29         29      0.989          1      0.995      0.928
              cup_blue         27         27      0.959      0.963      0.965      0.886
            cup_yellow         29         29      0.995          1      0.995      0.952
```

- 강의 28 에서 만든 가상환경을 그대로 씀. 강의 PC(RTX 5060 Ti)에서 54초 걸렸음.
- 검증 40장에서 mAP50 이 0.985 임. 모델은 `work/cup_train/cups/weights/best.pt` 에 저장됨.
- 이 점수는 같은 방법으로 만든 영상에서 잰 것임. 로봇의 카메라에서 잘 되는지는 5절에서 봄.

## 5. 컵 찾기

### 터미널 1 · 2: 시뮬레이터와 카메라의 자리

```bash
# 터미널 1
~/isaacsim/python.sh 47_pl2m_vision/pl2m_vision.py --start drinks --docked
# 터미널 2
source /opt/ros/jazzy/setup.bash
source ~/pinklab_robotworld2026/install/setup.bash
cd ~/nvidia-dli-lecture
ros2 launch 45_pl2m_dock/dock.launch.py
```

- 터미널 2 는 강의 45 의 런치임. `robot_state_publisher` 가 URDF 를 읽어 카메라가 달린 자리를 `/tf` 로 냄.

### 터미널 3: 컵 찾는 노드

```bash
source /opt/ros/jazzy/setup.bash
source ~/pinklab_robotworld2026/install/setup.bash
cd ~/nvidia-dli-lecture
~/yolo_venv/bin/python 47_pl2m_vision/cup_detector.py --print
```

```text
[    3.8초] 영상 10장, 한 장에 19 ms:  cup_blue 0.91 앞 0.512 · 왼쪽 +0.001 · 높이 0.796 m,  cup_pink 0.89 앞 0.512 · 왼쪽 +0.151 · 높이 0.796 m,  cup_yellow 0.90 앞 0.512 · 왼쪽 -0.150 · 높이 0.796 m
```

- YOLO 가 든 가상환경의 파이썬으로 실행함. ROS 2 는 그 터미널에서 `source` 해 두면 같이 쓸 수 있음.
- 컵 셋을 점수 0.89 ~ 0.91 로 찾았음. 한 장에 19 ms 걸림.
- 자리는 로봇 기준임. 파란 컵이 정면 0.51 m, 분홍 컵이 왼쪽 15 cm, 노란 컵이 오른쪽 15 cm 에 있다고 나옴.

### 찾은 컵

- `cup_detector.py --save cups.png` 로 저장한 영상임. 창으로 계속 보려면 `--window` 를 붙임.
- 같은 영상이 `/vision/cups_image` 토픽으로도 나옴. `rqt_image_view` 로 볼 수 있음.

### 코드: 색과 깊이를 짝지음

```python
def pair(self, stamp, **part):
    self.waiting.setdefault(stamp, {}).update(part)
    for old in sorted(self.waiting)[:-5]:
        del self.waiting[old]
    if len(self.waiting[stamp]) == 2:
        both = self.waiting.pop(stamp)
        self.detect(both["color"], both["depth"])
```

- 색과 깊이는 다른 토픽으로 따로 옴. 시각이 같은 둘이 모였을 때 처리함.
- 어느 쪽이 먼저 올지는 정해져 있지 않음. 처음에는 깊이가 먼저 온다고 보고 짰는데, 35초 동안 3장만 처리됐음.

### 코드: 찾은 것을 냄

```python
result = self.model.predict(picture, conf=self.confidence, device=self.device, verbose=False)[0]
for box, number, score in zip(result.boxes.xyxy.cpu().numpy(), result.boxes.cls.int().tolist(), result.boxes.conf.tolist()):
    item = Detection3D(label=self.model.names[number], score=float(score), track_id=-1, camera="head_camera")
    item.bbox_xyxy = [float(v) for v in box]
    center = cup_center(box, depth, self.matrix, self.base_from_camera, self.push)
```

- YOLO 가 주는 것은 상자(픽셀)와 이름, 점수임. 여기에 `cup_center` 가 구한 자리를 붙여 `/vision/detections` 로 냄.
- 메시지 형식은 원본의 `warehouse_interfaces/Detection3DArray` 임. 원본의 다른 프로그램이 같은 토픽을 받아 쓸 수 있음.

## 6. 상자와 깊이로 자리 구하기

### 픽셀에서 점으로

- 첫 줄은 픽셀 하나와 그 깊이로 카메라 기준의 점을 구하는 식임. fx · fy · cx · cy 는 `camera_info` 에 있는 렌즈 값임 (이 카메라는 fx = fy = 415.7, cx = 320, cy = 240).
- 셋째 줄은 그 점을 로봇 기준으로 옮김. 카메라가 달린 자리는 `/tf` 에서 한 번만 읽음. 카메라가 몸통에 고정돼 있어 바뀌지 않음.

### 깊이는 보이는 면까지임

- 파란 점은 상자 안의 픽셀을 하나씩 점으로 바꾼 것임. 컵의 앞면과 윗면, 뒤의 상판이 보임.
- 상자의 가운데 픽셀로 나가는 선은 컵의 앞면에서 멈춤 (주황 점). 컵의 가운데는 거기서 더 들어간 곳임.
- 같은 선 위에서 컵의 반지름(2.5 cm)만큼 더 보낸 점이 초록 점임. 참값(+)과 3.4 mm 떨어져 있음.

### 코드

```python
x1, y1, x2, y2 = box
w, h = x2 - x1, y2 - y1
patch = depth[int(y1 + h / 4):int(y2 - h / 4) + 1, int(x1 + w / 4):int(x2 - w / 4) + 1]
patch = patch[np.isfinite(patch) & (patch > 0.05)]
point = back_project(np.array((x1 + x2) / 2.0), np.array((y1 + y2) / 2.0), np.array(float(np.median(patch))), matrix)
point = point * (1.0 + push / np.linalg.norm(point))
return to_base(point, base_from_camera)
```

- 깊이는 상자의 가운데 부분에서 중앙값을 씀. 상자의 가장자리에는 컵 뒤의 배경이 섞임.
- `push` 가 컵의 반지름임. 원본 저장소는 물체마다 이 값을 표로 둠 (`HALF_DEPTH`).

## 7. 얼마나 정확한가

### 터미널 4: 참값과 견줌

```bash
source /opt/ros/jazzy/setup.bash
source ~/pinklab_robotworld2026/install/setup.bash
cd ~/nvidia-dli-lecture
python3 47_pl2m_vision/check_cups.py
```

```text
영상 20장.  오차 = 카메라로 구한 자리 - 참값 [mm] (로봇 기준: 앞 · 왼쪽 · 높이)
컵              찾은 횟수     점수     참값: 앞      왼쪽     오차: 앞     왼쪽     높이    수평 거리
cup_blue            20     0.91     0.515   +0.000       -3.2    +0.6    +1.3       3.3
cup_pink            20     0.89     0.515   +0.150       -2.6    +0.5    +1.0       2.6
cup_yellow          20     0.90     0.515   -0.150       -2.6    +0.2    +1.4       2.6
```

- 도킹한 자리에서 영상 20장을 모아 평균을 냈음. 셋 다 참값에서 3.3 mm 안임.
- 참값은 시뮬레이터가 내는 `/cups/truth` 와 `/ground_truth` 에서 읽음. 컵 찾는 노드는 이 둘을 쓰지 않음.

### 컵을 아무 데나 놓고

```bash
~/isaacsim/python.sh 47_pl2m_vision/pl2m_vision.py --start drinks --docked --shuffle 3
```

```text
컵: cup_pink (7.514, -1.586),  cup_blue (7.715, -1.739),  cup_yellow (7.397, -1.492)
```

```text
cup_blue            20     0.91     0.496   -0.065       -3.3    -0.1    +0.5       3.3
cup_pink            20     0.88     0.649   +0.136       -1.7    +1.5    +0.6       2.2
cup_yellow          20     0.91     0.743   +0.253       -1.3    +2.1    +1.4       2.5
```

- 터미널 1 을 `--shuffle 3` 으로 다시 띄운 결과임. 숫자는 무작위 씨앗이고, 같은 숫자면 같은 자리에 놓임.
- `--cups cup_pink=7.60,-1.60` 처럼 자리를 직접 줄 수도 있음.

### 열두 가지 경우

| 로봇이 선 자리 | 컵의 자리 | 컵까지의 거리 | 오차 (수평 거리) |
|---|---|---|---|
| 도킹한 자리 | 원래 자리 + 무작위 여섯 가지 | 0.50 ~ 0.81 m | 0.6 ~ 3.3 mm |
| Nav2 의 목적지 | 원래 자리 + 무작위 세 가지 | 1.08 ~ 1.38 m | 0.4 ~ 3.0 mm |
| 목적지에서 옆으로 10 cm · 8° 틀어진 자리 | 무작위 한 가지 | 1.09 ~ 1.40 m | 0.7 ~ 2.1 mm |

- 컵 36개를 모두 찾았음. 평균 1.9 mm, 가장 큰 것이 3.3 mm 임.
- 1 m 넘게 떨어진 목적지에서도 오차가 커지지 않았음. 시뮬레이터의 깊이 영상에는 잡음이 없기 때문임.

### 반지름만큼 더 들어가지 않으면

- `cup_detector.py --surface` 는 보이는 면의 점을 그대로 냄. 같은 열두 가지 경우에서 평균 23.7 mm 벗어남.
- 손가락 사이의 여유는 컵 양옆으로 1.9 cm 씩임 (강의 45). 2.4 cm 는 그 밖임.
- 벗어나는 방향은 늘 카메라 쪽임. 그래서 컵의 크기를 알면 고칠 수 있음.

### 컵이 다른 컵에 가려지면

```bash
~/isaacsim/python.sh 47_pl2m_vision/pl2m_vision.py --start drinks --docked --cups cup_blue=7.65,-1.72 cup_pink=7.65,-1.66
```

| 컵 | 점수 | 수평 거리 | 높이 |
|---|---|---|---|
| 파란 컵 (앞) | 0.90 | 3.3 mm | +1.3 mm |
| 분홍 컵 (6 cm 뒤) | 0.59 | 13.4 mm | +34.0 mm |

- 뒤의 컵은 윗부분만 보임. 상자가 그 부분만 감싸므로 상자의 가운데가 컵의 가운데보다 3.4 cm 높게 나옴.
- 점수도 0.59 로 내려감. 점수가 낮은 컵은 가려졌을 수 있다고 보고, 앞의 컵을 먼저 치우거나 다른 쪽에서 봐야 함.

## 8. 해 볼 것

### 직접 바꿔 보기

- `--shuffle` 의 숫자를 바꿔 가며 `check_cups.py` 로 오차 보기.
- `cup_detector.py --confidence 0.8` 로 올리고, Nav2 의 목적지(`--start drinks`)에서 어느 컵이 빠지는지 보기.
- `make_dataset.py --count 100` 으로 만든 데이터로 학습해 점수와 오차 견주기.
- `cup_detector.py --device cpu` 로 한 장에 걸리는 시간 보기.

## 9. 정리

### 기억할 것

- 머리 카메라는 색과 깊이를 같은 시각으로 냄. 받는 쪽은 시각이 같은 둘을 짝지음.
- YOLO 는 영상에서 상자를 줌. 상자의 가운데 픽셀과 깊이로 카메라 기준의 점이 나오고, `/tf` 로 로봇 기준이 됨.
- 깊이는 보이는 면까지의 거리임. 물체의 가운데는 그 크기의 절반만큼 더 들어간 곳임.
- 구한 자리는 참값과 견줘 숫자로 확인함. 이 강의의 설정에서는 평균 1.9 mm 였음.

### 한계

- 위치만 구함. 누운 컵처럼 방향이 필요한 물체는 이 방법으로 집을 수 없음.
- 다른 컵에 가려진 컵은 높이가 3.4 cm 틀리게 나옴.
- 시뮬레이터의 깊이 영상에는 잡음이 없음. 실제 깊이 카메라에서는 오차가 이보다 큼. 얼마나 큰지는 재지 않았음.
- 모델은 이 부스의 컵 세 개만 앎. 모양이나 색이 다른 컵은 데이터를 다시 만들어 학습해야 함.
- 실제 사진으로는 시험하지 않았음.
