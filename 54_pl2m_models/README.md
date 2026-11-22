# 54. 로봇의 눈에 AI 모델 붙이기: 이름으로 찾고, 말로 묻고, 사진 한 장으로 거리 재기 — 명령 모음

- 영상에서 입력하는 터미널 명령과 기대 출력을 순서대로 모았음. [53. 부스의 디지털트윈](../53_booth_twin/README.md) 다음 내용임.

| 파일 |
|---|
| [find_go.py](find_go.py) |
| [models.py](models.py) |
| [try_models.py](try_models.py) |

## 1. 학습하지 않고 쓰는 모델

### 이번 강의의 결과

- PL-2M 의 머리 카메라 영상 한 장에 미리 학습된 모델 넷을 돌린 것임. 왼쪽부터 이름으로 찾은 컵, 픽셀마다 무엇인지, 깊이 센서가 잰 거리, 색 영상만으로 어림한 거리임.
- 이 강의에서 학습한 것은 하나도 없음. 받아서 바로 쓴 모델들임.

### 강의 47 과 견주면

| | 강의 47 | 이 강의 |
|---|---|---|
| 컵을 찾는 모델 | 직접 학습한 YOLO | 받아 쓰는 OWLv2 |
| 준비 | 데이터 400장 + 학습 1분 | 없음. 이름을 글로 적음 |
| 찾을 수 있는 것 | 학습한 세 가지 | 글로 적을 수 있는 것 |
| 한 장에 | 19 ms | 538 ms |

- 받아 쓰는 모델은 준비가 필요 없고 무엇이든 찾음. 대신 느리고, 엉뚱한 것을 찾기도 함. 얼마나 그런지를 재 봄.

### NVIDIA 과정에서는

| | NVIDIA 과정 (GRID) | 이 강의 |
|---|---|---|
| 돌리는 곳 | General Robotics 의 클라우드 | 내 PC |
| 로봇 | Unitree Go2, 창고 | PL-2M, 부스 |
| 모델 | MoonDream · OneFormer · Metric3D · OWLv2 | Qwen3-VL · OneFormer · Depth Anything V2 · OWLv2 |
| 기능 | 지게차를 찾아가기 | 테이블을 찾아가기 |

- 그 과정이 쓰던 무료 클라우드는 종료됐음. 같은 일을 하는 공개 모델로 내 PC 에서 함.
- 모델 둘은 같은 일을 하는 다른 것으로 바꿨음. 따로 받은 코드를 실행하지 않고 `transformers` 만으로 돌아가는 것을 골랐음.

### 만드는 것

| 파일 | 하는 일 |
|---|---|
| `models.py` | 모델 넷을 같은 모양으로 부름 |
| `try_models.py` | 머리 카메라 영상 한 장에 넷을 돌리고 참값과 견줌 |
| `find_go.py` | 이름을 말하면 찾아서 다가감 |

## 2. 준비

### 터미널: 패키지

```bash
~/yolo_venv/bin/pip install transformers accelerate
~/yolo_venv/bin/python -c "import transformers; print(transformers.__version__)"
```

```text
5.22.0
```

- 강의 28 의 가상환경에 더함. 위 버전은 강의 PC 의 것임.
- 모델은 처음 부를 때 받아짐. 넷을 합쳐 5 GB 쯤임 (말로 묻는 모델이 4 GB).

### 터미널 1 · 2: 시뮬레이터

```bash
# 터미널 1
source /opt/ros/jazzy/setup.bash
cd ~/nvidia-dli-lecture
git pull
~/isaacsim/python.sh 47_pl2m_vision/pl2m_vision.py --start drinks --docked
# 터미널 2
source /opt/ros/jazzy/setup.bash
source ~/pinklab_robotworld2026/install/setup.bash
ros2 launch 45_pl2m_dock/dock.launch.py
```

- 강의 47 과 같음. 머리 카메라가 색과 깊이를 냄.

## 3. 영상 한 장에 넷

### 터미널 3

```bash
source /opt/ros/jazzy/setup.bash
source ~/pinklab_robotworld2026/install/setup.bash
cd ~/nvidia-dli-lecture
~/yolo_venv/bin/python 54_pl2m_models/try_models.py --save models.png
```

- 영상을 한 장 받아 모델 넷을 차례로 돌리고 끝남. 아래 넷이 그 출력임.

### 이름으로 찾기 (OWLv2)

```text
[이름으로 찾기] 올리는 데 2.8초, 한 장에 539 ms.  찾은 것 5개
  a pink cup     점수 0.39  상자 (175, 344) ~ (221, 431)  →  로봇 기준 앞 0.512 · 왼쪽 +0.151 m  (참값과 3.0 mm)
  a blue cup     점수 0.45  상자 (299, 343) ~ (340, 430)  →  로봇 기준 앞 0.512 · 왼쪽 +0.000 m  (참값과 3.4 mm)
  a yellow cup   점수 0.46  상자 (419, 343) ~ (465, 430)  →  로봇 기준 앞 0.512 · 왼쪽 -0.151 m  (참값과 2.8 mm)
```

- `"a pink cup"` 같은 말을 적어 주면 그것의 상자를 줌. 컵 셋을 다 찾았음.
- 상자에서 자리를 구하는 것은 강의 47 의 `cup_center` 그대로임. 참값과 3 mm 안으로, 직접 학습한 YOLO 와 같은 수준임.
- 찾은 것이 5개인 것은 옆 부스의 작은 물건 둘을 컵이라고 했기 때문임.

### 코드: 이름으로 찾기

```python
inputs = self.processor(text=[list(names)], images=Image.fromarray(image), return_tensors="pt").to(DEVICE)
side = max(height, width)
found = self.processor.image_processor.post_process_object_detection(self.model(**inputs), threshold=threshold, target_sizes=torch.tensor([[side, side]]))[0]
```

- 영상과 이름들을 함께 넣음. 이름마다 점수가 기준(0.15)을 넘는 상자가 나옴.
- 이 모델은 영상을 정사각형으로 채워서 봄. 상자의 좌표도 그 크기로 돌려받아야 맞음.

### 사진 한 장으로 거리 어림하기 (Depth Anything V2)

```text
[영상 한 장으로 거리 어림하기] 올리는 데 0.6초, 한 장에 101 ms
  깊이 영상과 견줌: 오차의 중앙값 9 %, 10 % 안에 든 픽셀 52 %
  a pink cup     까지: 깊이 영상 0.488 m,  어림 0.517 m  (+2.9 cm)
  a blue cup     까지: 깊이 영상 0.488 m,  어림 0.506 m  (+1.8 cm)
  a yellow cup   까지: 깊이 영상 0.488 m,  어림 0.499 m  (+1.1 cm)
```

- 깊이 센서 없이 색 영상만 보고 픽셀마다 거리를 냄. 모양은 그럴듯하고 값은 9 % 쯤 어긋남.
- 컵까지의 거리는 1 ~ 3 cm 틀림. 컵을 집을 수 있는 여유(1.9 cm)를 넘는 것이 있음.

### 픽셀마다 무엇인지 (OneFormer)

```text
[픽셀마다 무엇인지] 올리는 데 2.1초, 한 장에 210 ms
  chair 29 %,  table 24 %,  wall 21 %,  floor 17 %,  bench 6 %,  step, stair 2 %,  glass, drinking glass 1 %
```

- 150가지 이름 가운데 하나를 픽셀마다 붙임. 음료대를 `table`, 컵을 `glass` 라고 했음.
- 화면의 29 % 를 `chair` 라고 했는데, 그림을 보면 사람 모형이 거기에 들어 있음. 캡슐로 만든 사람 모형을 사람으로 보지 않았음.

### 말로 묻기 (Qwen3-VL)

```text
[말로 묻기] 올리는 데 2.1초
  Q: How many cups are on the pink table? Answer with a number.
  A: 0   (0.4초)
  Q: What colors are the cups on the table, from left to right?
  A: Based on the image provided, the cups on the table are colored as follows, from left to right: - **Pink** - **Blue** - **Yellow**   (0.9초)
  Q: Is there a person in the image? Answer yes or no.
  A: yes   (0.2초)
```

- 영상에 대해 글로 묻고 글로 답을 받음. 컵의 색과 순서는 맞혔음.
- 그런데 컵이 몇 개냐고 물으니 0 이라고 함. 바로 다음 질문에서는 컵 셋의 색을 말했음.
- 답이 그럴듯한 글로 나오므로 틀려도 알아채기 어려움. 로봇의 판단에 쓰려면 다른 것으로 확인해야 함.

## 4. 얼마나 믿을 수 있나

### 컵 찾기: 네 자리에서

| 로봇이 선 자리 | 컵 | 컵 셋의 오차 (참값과의 수평 거리) |
|---|---|---|
| 도킹한 자리 | 원래 자리 | 3.0 · 3.4 · 2.8 mm |
| 도킹한 자리 | 무작위 (`--shuffle 3`) | 1.6 · 3.6 · 1.3 mm |
| Nav2 의 목적지 (1.1 m) | 원래 자리 | 0.9 · 0.2 · 0.8 mm |
| Nav2 의 목적지 | 무작위 (`--shuffle 2`) | 1.7 · 0.5 mm · **1.32 m** |

- 열두 개 가운데 열하나는 3.6 mm 안임. 학습한 YOLO(평균 1.9 mm)와 같은 수준임.
- 하나는 1.32 m 틀림. 멀리 있는 노란 컵 대신 옆 부스의 노란 물건을 찾았음 (점수 0.24).
- 학습한 YOLO 는 이 부스의 컵만 배웠으므로 그런 일이 없었음. 받아 쓰는 모델은 "노란 컵처럼 보이는 것" 을 다 찾음.

### 거리 어림: 멀어지면

| 로봇이 선 자리 | 오차의 중앙값 | 컵까지의 거리에서 틀린 양 |
|---|---|---|
| 도킹한 자리 (컵까지 0.5 m) | 8 ~ 9 % | -7.2 ~ +2.9 cm |
| Nav2 의 목적지 (컵까지 1.1 m) | 20 ~ 23 % | -14 ~ -32 cm |

- 1 m 넘게 떨어지면 20 % 넘게 틀림. 실제보다 가깝다고 봄.
- 컵을 집는 데는 쓸 수 없음. 다가가는 데는 쓸 수 있는지를 5절에서 봄.

### 걸리는 시간

| 모델 | 올리는 데 | 한 장에 |
|---|---|---|
| 이름으로 찾기 | 2.8초 | 538 ms |
| 거리 어림 | 0.6초 | 101 ms |
| 픽셀마다 무엇인지 | 2.1초 | 210 ms |
| 말로 묻기 | 2.1초 | 0.2 ~ 0.9초 (답의 길이에 따라) |

- 시뮬레이터가 같은 GPU 를 쓰는 중에 잰 것임. 모델을 이미 받아 둔 뒤의 값임.
- 1초에 두 장쯤임. 움직이면서 보기에는 느려서, 서서 보고 움직이기를 되풀이하는 식으로 씀.

## 5. 이름으로 찾아가기

### 터미널 1: 처음 자리에서

```bash
~/isaacsim/python.sh 47_pl2m_vision/pl2m_vision.py
```

- 시뮬레이터를 처음 자리에서 다시 띄움. 로봇은 부스의 가운데에서 +x 쪽을 보고 서 있음.

### 터미널 3: 둥근 흰 테이블

```bash
~/yolo_venv/bin/python 54_pl2m_models/find_go.py "a round white table"
```

```text
찾을 것: 'a round white table'.  거리는 깊이 센서로 잼
[  7.5초] SEARCH    보이지 않음: 왼쪽으로 25° 돎
[  9.6초] 보임      점수 0.45 · 왼쪽으로 +28.7° · 거리 1.43 m
[  9.6초] AIM       +28.7° 돎
[ 11.8초] 보임      점수 0.55 · 왼쪽으로 -0.3° · 거리 1.64 m
[ 11.8초] APPROACH  0.32 m 나아감
[ 14.7초] 보임      점수 0.55 · 왼쪽으로 -0.2° · 거리 1.32 m
[ 14.7초] APPROACH  0.16 m 나아감
[ 16.5초] 보임      점수 0.50 · 왼쪽으로 -0.4° · 거리 0.92 m
[ 16.5초] 도착      0.92 m 앞에 섬
채점 (참값): 로봇 (5.59, -2.82).  보고 있는 쪽의 상담 테이블 1: 가장자리까지 0.85 m,  그쪽에서 0.5° 틀어져 있음
```

- 지도도 Nav2 도 쓰지 않았음. 처음에는 보이지 않아 25° 돌았고, 보이자 그쪽으로 돌아서 두 번에 나눠 다가갔음.
- 1 m 앞에 서라고 했는데 테이블의 가장자리에서 0.85 m 에 섰음. 방향은 0.5° 안임.

### 코드: 보고, 움직이고, 다시 봄

```python
seen = look()
if seen is None:
    robot.turn(SEARCH_TURN)                      # 없으면 25° 돌고 다시 본다
    continue
score, bearing, distance = seen
if abs(bearing) > AIM_TOLERANCE:
    robot.turn(bearing)                          # 화면 가운데에 오게 돈다
    continue
if distance <= args.stop + 0.03:
    break
robot.drive(min((distance - args.stop) * 0.5, STEP_LIMIT))       # 남은 거리의 절반만 간다
```

- 움직이는 동안에는 보지 않음. 한 장에 0.5초가 걸리므로, 서서 보고 조금 움직이고 다시 봄.
- 남은 거리의 절반만 감. 거리가 틀려도 한 번에 지나치지 않게 하려는 것임.
- 방향은 상자의 가운데가 화면 가운데에서 벗어난 만큼임 (`atan2(cx - u, fx)`).

### 고치는 동안 본 것: 테이블까지의 거리

| 상자 안의 거리에서 고르는 값 | 섰을 때 테이블의 가장자리까지 |
|---|---|
| 가운데 부분의 중앙값 (강의 47 의 컵처럼) | 0.49 m |
| 가까운 쪽 20 % 자리의 값 | 0.84 ~ 0.85 m |

- 처음에는 강의 47 의 컵처럼 중앙값을 썼음. 읽은 거리가 2.37 → 1.67 → 0.65 m 로 들쭉날쭉했고, 1 m 가 아니라 0.49 m 까지 다가갔음.
- 테이블은 다리 사이가 비어 있어 상자 안에 뒤의 배경이 많이 들어옴. 컵처럼 속이 찬 물체와 달리 중앙값이 배경의 거리가 됨.
- 가까운 쪽의 값을 쓰자 읽은 거리가 고르게 줄었음. `--median` 으로 예전 방식을 볼 수 있음.

### 깊이 센서 없이

```bash
~/yolo_venv/bin/python 54_pl2m_models/find_go.py "a round white table" --no-depth
```

| 거리를 재는 것 | 세 번 해서, 테이블의 가장자리까지 | 방향 | 걸린 시간 |
|---|---|---|---|
| 깊이 센서 | 0.84 · 0.85 · 0.85 m | 0.1 ~ 1.5° | 16초 |
| 색 영상에서 어림 (`--no-depth`) | 0.83 · 0.83 · 0.84 m | 0.6 ~ 1.6° | 17초 |

- 거리를 색 영상에서 어림해도 같은 자리에 섰음. 4절에서 본 20 % 의 오차는 멀리 있을 때의 것이고, 다가갈수록 다시 재므로 마지막에는 차이가 없어짐.
- 다가가는 일은 거리가 대강만 맞아도 됨. 집는 일과 다름.

### 엉뚱한 것을 찾아감

```bash
~/yolo_venv/bin/python 54_pl2m_models/find_go.py "a pink table"
```

```text
[ 16.3초] 도착      0.92 m 앞에 섬
채점 (참값): 로봇 (5.58, -2.82).  보고 있는 쪽의 음료대: 가장자리까지 2.19 m,  그쪽에서 23.0° 틀어져 있음
```

- 분홍 상판의 음료대를 찾으라고 했는데 위와 같은 자리, 곧 상담 테이블 앞에 가서 섰음. 음료대는 2.19 m 떨어져 있음.
- 그림을 보면 까닭이 보임. 음료대는 분홍 옷을 입은 사람 뒤에 거의 가려져 있고, `a pink table` 의 가장 높은 점수(0.26)는 화면 아래쪽 전체를 덮은 상자임.
- 프로그램은 "도착" 이라고 했음. 찾은 것이 맞는지는 이 프로그램이 알 수 없음.

## 6. 해 볼 것

### 직접 바꿔 보기

- `try_models.py --only find --find "a person" "a chair"` 로 다른 이름 찾아 보기.
- `try_models.py --ask "질문"` 으로 영어 질문을 바꿔 가며 틀리는 것 찾기.
- `find_go.py "a round white table" --stop 1.5` 로 멀리 서게 하기.
- `find_go.py` 의 `--score` 를 0.4 로 올리고 `"a pink table"` 을 다시 해 보기.

## 7. 정리

### 기억할 것

- 받아 쓰는 모델은 학습 없이 바로 씀. 이름으로 찾기, 말로 묻기, 거리 어림, 픽셀마다 무엇인지.
- 이름으로 찾은 컵의 자리는 학습한 모델만큼 정확했음. 대신 한 장에 0.5초가 걸리고, 비슷하게 생긴 다른 것을 찾기도 함.
- 말로 묻는 모델의 답은 그럴듯하게 틀림. 컵이 셋 있는데 0 개라고 했음.
- 거리 어림은 다가가는 데는 충분하고 집는 데는 모자람.
- 느린 모델은 서서 보고, 움직이고, 다시 보는 식으로 씀.

### 한계

- 찾아가는 동안 장애물을 피하지 않음. 가는 길에 사람이나 가구가 있으면 부딪힘. Nav2 와 잇는 것은 하지 않았음.
- 찾은 것이 맞는지 확인하는 단계가 없음. "분홍 테이블" 에서 본 그대로임.
- 모델마다 영상 한 장, 질문 몇 개로 본 것임. 정확도를 많은 영상으로 재지 않았음.
- 사람 모형이 캡슐이라 모델이 사람으로 보지 않는 일이 있었음. 실제 영상에서는 다를 수 있음. 해 보지 않았음.
- 공식 과정의 Rerun 화면은 쓰지 않았음.
