# 48. 찾은 컵 집기: 좌표표 없이 카메라로 본 자리로 집어 나르기 — 명령 모음

- 영상에서 입력하는 터미널 명령과 기대 출력을 순서대로 모았음. [47. 머리 카메라로 컵 찾기](../47_pl2m_vision/README.md) 다음 내용임.

| 파일 |
|---|
| [find_pick.py](find_pick.py) |
| [reach_map.py](reach_map.py) |

## 1. 보고 나서 움직임

### 이번 강의의 결과

- 컵 세 개를 음료대의 앞줄 아무 데나 놓았음. 로봇은 컵이 어디 있는지 모르는 채로 출발함.
- 음료대 앞에 서서 머리 카메라로 컵을 찾고, 그 자리에 맞춰 설 곳과 팔을 정함. 도킹한 뒤 다시 보고 집음.

### 강의 46 에서 바뀌는 것

| | 강의 46 | 이 강의 |
|---|---|---|
| 컵의 자리 | 좌표표 + 도킹이 알려 준 값으로 셈 | 도킹한 뒤 카메라로 봄 |
| 집을 팔 · 비켜 설 거리 | 좌표표로 정함 | Nav2 로 도착한 자리에서 본 컵의 자리로 정함 |
| 다른 컵 | MoveIt 에 알리지 않음 | 본 것을 피할 물체로 알림 |
| 놓을 자리 | 비었다고 봄 | 내려놓기 전에 봄 |
| 닿지 않는 컵 | 다루지 않음 | 팔을 움직이기 전에 그만둠 |

- 베이스를 몰고 팔을 움직이는 코드는 강의 46 그대로임. 컵의 자리를 정하는 곳만 바꿈.
- 컵 찾는 노드는 강의 47 의 것을 그대로 씀.

### 모드 하나를 더함

| LOOK | 언제 | 무엇을 정하나 |
|---|---|---|
| 첫 번째 | Nav2 로 음료대 앞에 도착한 뒤 | 집을 팔, 비켜 설 거리. 닿지 않으면 그만둠 |
| 두 번째 | 음료대에 도킹한 뒤 | 컵의 자리, 피할 다른 컵 |
| 세 번째 | 상담 테이블에 도킹한 뒤 | 놓을 자리가 비었는지 |

- LOOK 은 베이스도 팔도 움직이지 않는 모드임. 멈춘 뒤에 찍은 영상 다섯 장을 모아 평균을 씀.
- 세 번을 더해 2.6초임. 임무 전체는 180초임.

### 만드는 것

| 파일 | 어디서 도나 | 하는 일 |
|---|---|---|
| `find_pick.py` | ROS 2 | 강의 46 의 임무에 LOOK 을 더한 것 |
| `reach_map.py` | ROS 2 | 팔이 닿는 범위를 잼 |

- 시뮬레이터는 강의 47 의 `pl2m_vision.py` 를 씀. 강의 46 의 `mission.py` 에는 바꿔 끼울 자리 둘을 냈음 (7절).

## 2. 띄우기

### 터미널 1: Isaac Sim

```bash
source /opt/ros/jazzy/setup.bash
cd ~/nvidia-dli-lecture
git pull
~/isaacsim/python.sh 47_pl2m_vision/pl2m_vision.py --shuffle 4 --front
```

```text
준비됨: 시작 자리 home (5.300, -3.200, 0.0°), 머리 카메라 640 × 480. 끝내려면 Ctrl+C
컵: cup_pink (7.513, -1.745),  cup_blue (7.867, -1.710),  cup_yellow (7.669, -1.736)
```

- `--shuffle 4` 는 컵을 무작위로 놓음. `--front` 를 붙이면 음료대의 앞줄(앞면에서 6 ~ 11 cm)에만 놓음. 팔이 닿는 깊이임 (3절).
- 창으로 보려면 `--gui --view booth` 를 붙임.

### 터미널 2 ~ 5: 팔과 베이스

```bash
source /opt/ros/jazzy/setup.bash
source ~/pinklab_robotworld2026/install/setup.bash
cd ~/nvidia-dli-lecture
python3 41_pl2m_ros2/trajectory_server.py &
ros2 launch 42_pl2m_moveit/moveit.launch.py rviz:=false &
ros2 launch 44_pl2m_nav/nav2.launch.py rsp:=false rviz:=false &
ros2 launch 45_pl2m_dock/dock.launch.py rsp:=false
```

- 강의 46 과 같음. 궤적 서버, MoveIt, Nav2, 마커 찾는 노드임.
- `You can start planning now!` 와 `Managed nodes are active` 가 나오면 준비된 것임.

### 터미널 6: 컵 찾는 노드

```bash
source /opt/ros/jazzy/setup.bash
source ~/pinklab_robotworld2026/install/setup.bash
cd ~/nvidia-dli-lecture
~/yolo_venv/bin/python 47_pl2m_vision/cup_detector.py
```

- 강의 47 의 노드임. 강의 47 에서 학습한 모델(`work/cup_train/cups/weights/best.pt`)이 있어야 함.

### 터미널 7: 임무

```bash
source /opt/ros/jazzy/setup.bash
source ~/pinklab_robotworld2026/install/setup.bash
cd ~/nvidia-dli-lecture
python3 48_pl2m_find_pick/find_pick.py --cup cup_yellow
```

```text
[   13.1초] ARM    → NAV2   (Nav2 이동)  음료대 앞 (7.65, -2.82, 90°) 으로
            Nav2: 도착 (12.1초). 도착 오차 13.7 cm · -9.2° (참값으로 채점)
[   25.5초] NAV2   → LOOK   (컵 찾기)  서서 마커와 cup_yellow 을(를) 봄
            cup_yellow: 점수 0.91, 로봇 기준 앞 1.133 · 왼쪽 +0.033 m.  내가 선 자리: 면까지 1.065 m · 축이 왼쪽으로 -0.142 m · 틀어진 각 +9.8°
            → 컵은 축에서 왼쪽으로 -1.8 cm, 테이블 앞면에서 안쪽으로 5.7 cm
            → 오른팔로 집음. 축이 왼쪽으로 -6 cm 인 자리에 비켜 서서 도킹함
[   26.7초] LOOK   → SETTLE (멈춤 확인)  Nav2 가 끝났는지, 베이스가 멈췄는지 봄
[   27.8초] SEARCH → ALIGN  (옆걸음)  설 자리(축이 왼쪽으로 -6 cm 인 곳)에서 8.0 cm 벗어남 (한계 2 cm): 옆걸음
[   54.2초] BLIND  → DOCKED (도킹됨)  면까지 0.43 m 로 봄, 축이 왼쪽으로 -0.061 m, 축과 나란해지려면 -1.2°
[   54.4초] DOCKED → LOOK   (컵 찾기)  도킹한 자리에서 다시 봄
            cup_yellow: 점수 0.89, 로봇 기준 앞 0.493 · 왼쪽 -0.091 · 높이 0.796 m.  다른 컵 2개를 피할 물체로 알림
[   55.0초] LOOK   → ARM    (팔 작업)  베이스는 서 있음. cup_yellow 을(를) 집음
            집기 전 확인: 운반 자세 ↔ 컵 ↔ 접근 시작 자리가 직선으로 이어짐. 그 끝의 팔 모양으로 감
            쥐었음: 손가락이 27.4 mm 에서 멈춤. 로봇에 0.2 kg 을 알림
```

- 컵의 이름만 줌. 자리는 주지 않음.
- 첫 LOOK 에서 노란 컵이 축에서 오른쪽으로 1.8 cm 에 있다고 봤음. 가운데 가까이라 어느 팔도 닿지 않는 자리임. 그래서 왼쪽으로 6 cm 비켜 서서 오른팔로 집기로 함. 도킹한 뒤의 LOOK 에서 컵이 로봇의 오른쪽 9.1 cm 에 보였고, 그 자리로 집었음.

### 놓고 돌아옴

```text
[  149.5초] BLIND  → DOCKED (도킹됨)  면까지 0.43 m 로 봄, 축이 왼쪽으로 -0.004 m, 축과 나란해지려면 +0.4°
[  149.6초] DOCKED → LOOK   (컵 찾기)  놓을 자리가 비었는지 봄
            놓을 자리가 비어 있음
[  150.3초] LOOK   → ARM    (팔 작업)  베이스는 서 있음. cup_yellow 을(를) 오른팔의 정면에 놓음
            놓을 자리: 로봇 기준 앞 0.516 · 왼쪽 -0.150 · 컵 가운데의 높이 0.765 m
            놓았음: 그리퍼를 열고, 로봇에 알린 무게를 0 kg 으로 되돌림
[  171.9초] ARM    → UNDOCK (물러나기)  오도메트리로 0.75 m 뒤로
[  182.7초] UNDOCK → NAV2   (Nav2 이동)  처음 자리 (5.30, -3.20, 0°) 으로
[  192.4초] NAV2   → IDLE   (대기)  임무 끝
```

```text
끝날 때 컵: cup_pink (7.513, -1.745) 높이 0.795 m · 기울기 0°,  cup_blue (7.867, -1.710) 높이 0.795 m · 기울기 0°,  cup_yellow (6.076, -2.050) 높이 0.765 m · 기울기 0°
```

- 위는 임무의 뒷부분, 아래는 시뮬레이터가 끝날 때 낸 컵의 참값임. 노란 컵이 상담 테이블 1 의 오른팔 자리 (6.07, -2.05) 에서 6 mm 떨어진 곳에 서 있음.

## 3. 팔이 닿는 범위

### 터미널: 재기

```bash
~/isaacsim/python.sh 47_pl2m_vision/pl2m_vision.py --start drinks --docked     # 터미널 1 을 이렇게 다시 띄움
python3 48_pl2m_find_pick/reach_map.py                                         # 터미널 7
```

```text
가로: 왼쪽 +0.30 m → -0.30 m (3 cm 간격)
앞 0.46 m   왼팔 OOOOOO...............   오른팔 ...............OOOO..
앞 0.49 m   왼팔 OOOOOOOOOO...........   오른팔 ...........OOOOOOOO..
앞 0.52 m   왼팔 OOOOOOOOOO...........   오른팔 ...........OOOOOOOO..
앞 0.55 m   왼팔 .OOOOOOOO............   오른팔 ............OOOOOOO..
앞 0.58 m   왼팔 .....................   오른팔 .....................
```

- 컵이 로봇 기준 어느 자리에 있다고 하고, 강의 46 의 "집기 전 확인" 을 MoveIt 에 물어봄. 팔은 움직이지 않음.
- 운반 자세에서 컵을 거쳐 접근 시작 자리까지 직선으로 이어지면 `O` 임.
- 컵을 아무 데나 놓으려면 먼저 어디까지 집을 수 있는지 알아야 함.

### 촘촘하게 잰 것

- 앞뒤 1 cm, 좌우 2 cm 간격으로 잰 것임 (`--step 0.01 0.02`).
- 앞뒤로는 로봇의 앞 0.48 ~ 0.55 m 임. 테이블 앞면에서 5 ~ 12 cm 안쪽에 해당함. 음료대의 깊이 42 cm 가운데 앞쪽 일부만 닿음.
- 옆으로는 로봇의 가운데에서 6 ~ 26 cm 임. 가운데 가까이는 어느 팔도 닿지 않음.

### 코드에 적은 값

```python
REACH_DEPTH = (0.05, 0.12)         # 컵의 가운데가 테이블 앞면(마커 면)에서 안쪽으로 이만큼 [m]
REACH_SIDE = (0.08, 0.24)          # 컵이 로봇의 가운데에서 집는 팔 쪽으로 이만큼 [m]
MAX_STEP = 0.12                    # 옆으로 비켜 서는 거리의 한계 [m]
```

- 옆으로는 잰 범위에서 양쪽 2 cm 씩을 뺐음. 도킹이 옆으로 2 cm 까지 어긋날 수 있기 때문임 (강의 46).
- 그림의 점선 네모가 이 값임.

## 4. 첫 번째 LOOK: 설 자리를 정함

### 두 가지를 봄

| 보는 것 | 카메라 | 알게 되는 것 |
|---|---|---|
| 마커 두 장 | 앞 카메라 | 내가 스테이션의 어디에 서 있나 (강의 45 의 along · side · turn) |
| 컵 | 머리 카메라 | 컵이 나의 어디에 있나 (강의 47) |

- 둘을 합치면 컵이 스테이션의 어디에 있는지가 나옴. 팔과 설 자리는 이것으로 정함.
- Nav2 로 도착한 자리는 10 cm · 10° 쯤 어긋나 있음. 컵을 로봇 기준으로만 알아서는 도킹한 뒤 어디에 올지 알 수 없음.

### 수식

- 첫 줄은 로봇이 본 자리를 스테이션의 축과 나란한 방향으로 돌림. 둘째 줄은 마커 면까지의 거리와 옆으로 벗어난 거리를 뺌.
- 셋째 줄이 도킹한 뒤의 자리임. 축에서 aim 만큼 비켜 서면 컵은 그만큼 옆으로 옮겨 보임.

### 코드

```python
along, side, turn = station_geometry(base.find_markers(offsets), offsets)
(x, y, _), score = self.checked(self.look(), cup, STATIONS[station]["top"])
ahead, left = x * math.cos(turn) + y * math.sin(turn), -x * math.sin(turn) + y * math.cos(turn)
left, depth = left - side, ahead - along
```

- `station_geometry` 는 강의 45 의 함수임. 도킹이 쓰는 것과 같은 값을 여기서 미리 한 번 구함.
- `self.look()` 은 멈춘 뒤에 찍은 영상 다섯 장에서 컵의 자리를 모아 평균을 냄.

### 멀리서 본 값은 얼마나 맞나

| 임무 | 축에서 왼쪽으로: 본 값 | 참값 | 앞면에서 안쪽으로: 본 값 | 참값 |
|---|---|---|---|---|
| 2절의 임무 (무작위 4 · 노랑) | -1.8 cm | -1.9 cm | 5.7 cm | 6.9 cm |
| 무작위 2 · 파랑 | +23.4 cm | +23.1 cm | 4.9 cm | 5.9 cm |
| 무작위 3 · 노랑 | +25.5 cm | +25.3 cm | 9.6 cm | 10.4 cm |
| 무작위 4 · 분홍 | +13.8 cm | +13.7 cm | 5.3 cm | 6.0 cm |
| 무작위 5 · 파랑 | -15.2 cm | -15.4 cm | 9.3 cm | 10.2 cm |
| 무작위 6 · 노랑 | +26.2 cm | +26.0 cm | 8.0 cm | 8.8 cm |
| 축 위 · 파랑 | +0.1 cm | 0.0 cm | 7.6 cm | 8.5 cm |

- 옆으로는 3 mm 안에서 맞음. 깊이는 0.7 ~ 1.2 cm 얕게 나옴.
- 깊이가 어긋나는 것은 마커 쪽임. 1.05 m 떨어진 마커 면을 1.2 ~ 1.4 cm 멀게 쟀음. 컵의 자리는 강의 47 에서 본 대로 3 mm 안임.
- 그래서 멀리서는 깊이를 2 cm 넉넉하게 봄 (`FAR_SLACK`). 확실히 벗어난 컵만 여기서 거름.

## 5. 팔과 비켜 설 거리

### 규칙

- 가로축이 컵의 자리, 세로축이 비켜 설 거리임. 선의 색이 집을 팔임.
- 컵이 그 팔의 범위(8 ~ 24 cm) 안에 있으면 비키지 않음 (평평한 부분). 벗어나 있으면 범위의 끝에 오도록 비킴 (기운 부분).
- 두 팔이 다 되면 덜 움직이는 쪽을 고름.

### 코드

```python
for arm, sign in (("left", 1.0), ("right", -1.0)):
    toward = sign * left                                                          # 축 위에 섰을 때 컵이 그 팔 쪽으로 떨어진 거리
    aim = sign * min(max(toward, REACH_SIDE[0]), REACH_SIDE[1]) - left
    if low - 1e-6 <= aim <= high + 1e-6:
        choices.append((abs(aim), arm, aim))
_, arm, aim = min(choices)
```

- `left` 는 4절에서 구한 값임. `low` · `high` 는 비켜 설 수 있는 범위임.
- 고를 것이 하나도 없으면 "어느 팔도 닿지 않음" 으로 그만둠.

### 옆에 선 사람

```python
for _, kind, center, size in people_near(station):
    ahead, left, _ = to_base(station, center)
    if kind == "cylinder" and -0.4 < ahead < 0.45:                                # 도킹한 베이스의 옆에 있는 것만
        side = "left" if left > 0 else "right"
        room[side] = min(room[side], max(0.0, abs(left) - size[0] - BODY_HALF - PERSON_GAP))
```

- 음료대의 오른쪽에는 사람 모형이 서 있음. 옆걸음은 오도메트리만 보고 움직이므로 사람을 피하지 못함.
- 그래서 사람이 선 쪽으로는 틈이 남는 만큼만 비킴. 음료대에서는 왼쪽으로 12 cm, 오른쪽으로 4 cm 임.
- 강의 46 의 "사람이 없는 쪽으로 비켜 섬" 을 거리로 바꾼 것임.

### 가운데에 놓인 컵

```bash
~/isaacsim/python.sh 47_pl2m_vision/pl2m_vision.py --cups cup_blue=7.65,-1.72 cup_pink=7.40,-1.72 cup_yellow=7.88,-1.72
python3 48_pl2m_find_pick/find_pick.py --cup cup_blue
```

```text
            → 컵은 축에서 왼쪽으로 +0.1 cm, 테이블 앞면에서 안쪽으로 7.6 cm
            → 오른팔로 집음. 축이 왼쪽으로 -8 cm 인 자리에 비켜 서서 도킹함
            cup_blue: 점수 0.91, 로봇 기준 앞 0.512 · 왼쪽 -0.078 · 높이 0.797 m.  다른 컵 2개를 피할 물체로 알림
```

- 파란 컵을 축 위에 놓았음. 로봇은 왼쪽으로 8 cm 비켜 서서 오른팔로 집었음.
- 도킹한 뒤 컵은 로봇의 오른쪽 7.8 cm 에 보였음. 셈한 값(8 cm)과 2 mm 차이임.
- 같은 컵을 왼쪽으로 6 cm 옮기면 오른쪽으로 2 cm 비켜 서서 왼팔로 집음.

## 6. 두 번째 LOOK: 컵의 자리

### 도킹한 뒤에 다시 보는 까닭

| | 멀리서 본 값으로 셈 | 도킹한 뒤 다시 봄 |
|---|---|---|
| 들어가는 오차 | 멀리서 본 오차 + 도킹의 오차 | 가까이서 본 오차 |
| 크기 | 깊이 1 cm + 옆으로 2 cm 까지 | 2.0 ~ 3.3 mm |

- 도킹한 뒤에는 컵이 0.5 m 앞에 있음. 여기서 본 자리가 그대로 로봇 기준의 자리임.
- 도킹이 어긋난 만큼은 컵의 자리에 들어가지 않음. 로봇이 서 있는 그 자리에서 봤기 때문임.
- 8절의 둘째 임무에서는 도킹이 옆으로 3.1 cm 어긋났음. 컵이 그만큼 옆에 보였고(셈한 14.6 cm 대신 18.2 cm), 그 자리로 집었음.

### 일곱 번의 값

| 임무 | 카메라로 본 자리 (앞, 왼쪽) | 참값 | 차이 |
|---|---|---|---|
| 2절의 임무 (무작위 4 · 노랑) | (0.493, -0.091) | (0.496, -0.090) | 3.2 mm |
| 무작위 2 · 파랑 | (0.482, +0.238) | (0.485, +0.237) | 2.7 mm |
| 무작위 3 · 노랑 | (0.535, +0.228) | (0.537, +0.227) | 2.3 mm |
| 무작위 4 · 분홍 | (0.483, +0.158) | (0.486, +0.157) | 3.3 mm |
| 무작위 5 · 파랑 | (0.525, -0.159) | (0.527, -0.160) | 2.2 mm |
| 무작위 6 · 노랑 | (0.516, +0.243) | (0.518, +0.243) | 2.0 mm |
| 축 위 · 파랑 | (0.512, -0.078) | (0.515, -0.077) | 2.7 mm |

- 참값은 시뮬레이터가 낸 컵과 로봇의 자리로 셈한 것임. 채점에만 씀.

### 코드: 믿을 수 없는 것은 집지 않음

```python
position, score = seen[cup]
off = position[2] - (top + CUP_HEIGHT / 2.0)
if score < MIN_SCORE or abs(off) > HEIGHT_TOLERANCE:
    raise ArmError(f"{cup} 의 자리를 믿을 수 없음: 점수 {score:.2f} ...")
```

- 강의 47 에서 다른 컵에 가려진 컵은 점수가 0.59 로 내려가고 높이가 3.4 cm 높게 나왔음.
- 컵은 상판 위에 서 있으므로 높이를 미리 앎. 점수가 0.7 아래이거나 높이가 1.5 cm 넘게 다르면 그만둠.

### 코드: 다른 컵을 피할 물체로

```python
self.others = [shape(name, "cylinder", tuple(float(v) for v in position), (CUP_DIAMETER / 2, CUP_HEIGHT))
               for name, (position, _) in seen.items() if name != cup and name in CUPS]
```

- 강의 46 에서는 컵들이 15 cm 씩 떨어진 정해진 자리에 있어 알리지 않아도 됐음. 아무 데나 놓이면 팔이 가는 길에 다른 컵이 있을 수 있음.
- 같은 영상에서 본 다른 컵을 원기둥으로 MoveIt 에 넣음. 집기 전 확인이 이 컵들과의 충돌도 봄.

## 7. 강의 46 의 코드에 낸 자리

### 바꿔 끼울 두 곳

```python
def choose_arm(self, station, cup):
    return arm_and_aim(station, cup)

def cup_position(self, station, cup, where):
    return to_base(station, CUPS[cup], *where)
```

- 강의 46 의 `mission.py` 에서 컵의 자리를 정하는 두 줄을 메서드로 뺐음. 하는 일은 전과 같음.

### 이 강의에서 바꾼 것

```python
class FindPick(Mission):
    def choose_arm(self, station, cup):
        return self.plan                                   # 첫 번째 LOOK 이 정한 (팔, aim)

    def cup_position(self, station, cup, where):
        return tuple(float(v) for v in self.target)        # 두 번째 LOOK 이 본 자리
```

- `Mission` 을 물려받아 그 둘만 바꿈. 집는 순서, 집기 전 확인, 놓는 동작은 강의 46 의 코드가 그대로 돎.
- 도킹이 알려 준 값(`where`)은 가구의 자리를 셈하는 데만 씀.

## 8. 세 번째 LOOK 과 실패하는 경우

### 놓을 자리에 컵이 있으면

```bash
~/isaacsim/python.sh 47_pl2m_vision/pl2m_vision.py --cups cup_pink=7.50,-1.72 cup_yellow=7.40,-1.72 cup_blue=7.80,-1.72
python3 48_pl2m_find_pick/find_pick.py --cup cup_pink
python3 48_pl2m_find_pick/find_pick.py --cup cup_yellow
```

```text
[  309.5초] DOCKED → LOOK   (컵 찾기)  놓을 자리가 비었는지 봄
[  309.9초] LOOK   → FAILED (실패)  놓을 자리에 cup_pink 이(가) 있음 (놓을 자리에서 0.4 cm): 내려놓지 않음
```

- 두 컵 모두 왼팔로 집어 상담 테이블 1 의 같은 자리에 놓게 됨. 두 번째 임무는 놓기 전에 그만둠.
- 로봇은 컵을 든 채 도킹한 자리에 서 있음. 다른 자리를 고르는 것은 넣지 않았음.

### 닿지 않는 컵

```bash
~/isaacsim/python.sh 47_pl2m_vision/pl2m_vision.py --cups cup_pink=7.55,-1.58
python3 48_pl2m_find_pick/find_pick.py --cup cup_pink
```

```text
            → 컵은 축에서 왼쪽으로 +10.1 cm, 테이블 앞면에서 안쪽으로 21.6 cm
[   22.1초] LOOK   → FAILED (실패)  컵이 테이블 앞면에서 21.6 cm 안쪽에 있음: 팔이 닿는 것은 5 ~ 12 cm
```

- 첫 번째 LOOK 에서 그만둠. 도킹도 하지 않고 팔도 움직이지 않음.

### 가려진 컵

```bash
~/isaacsim/python.sh 47_pl2m_vision/pl2m_vision.py --start drinks --docked --cups cup_blue=7.50,-1.752 cup_pink=7.50,-1.69
python3 48_pl2m_find_pick/find_pick.py --only pick --cup cup_pink
```

```text
[    7.3초] IDLE   → LOOK   (컵 찾기)  도킹한 자리에서 봄
[    8.6초] LOOK   → FAILED (실패)  cup_pink 의 자리를 믿을 수 없음: 점수 0.78 (한계 0.7), 높이가 상판 위에 선 컵보다 +24 mm (한계 ±15 mm). 가려졌을 수 있음
```

- 분홍 컵을 파란 컵의 6 cm 뒤에 놓고, 도킹한 자리에서 집기만 시켜 본 것임 (`--only pick`).
- 점수는 0.78 로 한계를 넘었지만 높이가 2.4 cm 높게 나와 걸렸음. 점수만 봤다면 틀린 자리로 손을 보냈을 것임.

### 앞의 컵을 먼저 치우면

```bash
~/isaacsim/python.sh 47_pl2m_vision/pl2m_vision.py --cups cup_blue=7.50,-1.752 cup_pink=7.50,-1.69
python3 48_pl2m_find_pick/find_pick.py --cup cup_blue --to consult_2
python3 48_pl2m_find_pick/find_pick.py --cup cup_pink
```

```text
            cup_pink 의 자리가 cup_blue 와 겹쳐 보임 (가운데끼리 3.4 cm, 점수 0.71): 뒤에 가려진 것으로 보고 2.4 cm 뒤로 옮겨 알림
            cup_blue: 점수 0.90, 로봇 기준 앞 0.477 · 왼쪽 +0.152 · 높이 0.798 m.  다른 컵 2개를 피할 물체로 알림
            쥐었음: 손가락이 27.4 mm 에서 멈춤. 로봇에 0.2 kg 을 알림
...
            cup_pink: 점수 0.89, 로봇 기준 앞 0.541 · 왼쪽 +0.182 · 높이 0.795 m.  다른 컵 1개를 피할 물체로 알림
            쥐었음: 손가락이 27.4 mm 에서 멈춤. 로봇에 0.2 kg 을 알림
```

- 위 세 줄이 첫 임무(파란 컵), 아래 두 줄이 둘째 임무(분홍 컵)임. 파란 컵을 치운 뒤에는 분홍 컵이 점수 0.89 로 보였고 그대로 집었음.

### 고치는 동안 본 실패: 겹쳐 보이는 컵

```text
            쥐었음: 손가락이 27.4 mm 에서 멈춤. 로봇에 0.2 kg 을 알림
[   67.0초] ARM    → FAILED (실패)  들기: 직선의 0 % 만 갈 수 있음 (부딪힘: cup_blue × cup_pink)
```

- 처음에는 위의 첫 임무가 파란 컵을 쥔 뒤 여기서 멈췄음. 뒤에 가려진 분홍 컵의 자리가 3.3 cm 앞으로 잘못 나와, 피할 물체로 넣은 원기둥이 파란 컵과 겹쳤음.
- 컵은 서로 겹칠 수 없음. 겹쳐 보이면 가려진 쪽이 틀린 것임.

### 코드: 겹치지 않는 데까지 뒤로

```python
ux, uy = x - HEAD_CAMERA[0], y - HEAD_CAMERA[1]                               # 카메라에서 그 컵을 보는 방향
...
reach = -along + math.sqrt(max(0.0, along * along - (bx * bx + by * by) + (CUP_DIAMETER + 0.005) ** 2))
x, y, z = HEAD_CAMERA[0] + reach * ux, HEAD_CAMERA[1] + reach * uy, top + CUP_HEIGHT / 2.0
```

- 집을 컵과 겹쳐 보이는 컵은 카메라에서 본 방향을 따라, 집을 컵과 5.5 cm 떨어지는 데까지 뒤로 옮겨서 알림 (`untangle`). 높이는 상판 위로 둠.
- 위의 경우 2.4 cm 를 옮겼음. 옮긴 자리는 참값과 1.1 cm 차이임. 옮기기 전에는 3.5 cm 였음.

## 9. 되풀이해 보기

### 여덟 번

| 컵의 자리 | 집을 컵 | 팔 · 비켜 선 거리 | 결과 | 놓인 자리와 목표의 거리 |
|---|---|---|---|---|
| 무작위 2 | 파랑 | 왼팔 · 0 | 놓음 | 9 mm |
| 무작위 3 | 노랑 | 왼팔 · 왼쪽으로 2 cm | 놓음 | 3 mm |
| 무작위 4 | 분홍 | 왼팔 · 0 | 놓음 | 7 mm |
| 무작위 4 | 노랑 | 오른팔 · 왼쪽으로 6 cm | 놓음 | 6 mm |
| 무작위 5 | 파랑 | 오른팔 · 0 | 놓음 | 4 mm |
| 무작위 6 | 노랑 | 왼팔 · 왼쪽으로 2 cm | 놓음 | 8 mm |
| 축 위 | 파랑 | 오른팔 · 왼쪽으로 8 cm | 놓음 | 8 mm |
| 축에서 왼쪽 6 cm | 파랑 | 왼팔 · 오른쪽으로 2 cm | 놓음 | 7 mm |

- 여덟 번 모두 집어서 놓았음. 놓인 자리는 목표에서 3 ~ 9 mm 임. 한 번에 149 ~ 180초 걸렸음.
- 좌표표로 집은 강의 46 (여섯 번, 0 ~ 4 mm, 145 ~ 165초) 과 견주면 놓인 자리가 몇 mm 더 벗어났음. 원인은 가려 보지 않았음.

## 10. 해 볼 것

### 직접 바꿔 보기

- `--shuffle` 의 숫자를 바꿔 가며 세 컵을 차례로 나르기.
- `--front` 를 빼고 띄워, 어느 컵에서 "닿지 않음" 으로 그만두는지 보기.
- `find_pick.py` 의 `REACH_SIDE` 를 (0.02, 0.24) 로 넓혀, 가운데 가까운 컵에서 집기 전 확인이 실패하는 것 보기.
- `reach_map.py --station consult_1` 로 상담 테이블에서의 범위 재기.

## 11. 정리

### 기억할 것

- 움직이기 전에 봄. 멈춘 뒤에 찍은 영상만 쓰고, 여러 장의 평균을 씀.
- 멀리서 본 값으로는 설 자리를 정하고, 가까이서 본 값으로 집음. 가까이서 본 값에는 도킹의 오차가 들어가지 않음.
- 팔이 닿는 범위는 재서 숫자로 적어 둠. 그 밖의 컵은 팔을 움직이기 전에 그만둠.
- 본 것은 모두 MoveIt 에 알림. 집을 컵만이 아니라 옆의 컵도 피할 물체임.
- 점수가 낮거나 높이가 맞지 않는 검출은 쓰지 않음.

### 한계

- 음료대의 깊이 42 cm 가운데 앞쪽 7 cm 폭에 놓인 컵만 집음. 더 안쪽의 컵은 다른 쪽에서 다가가거나 팔의 자세를 바꿔야 함. 해 보지 않았음.
- 놓을 자리가 차 있으면 그만둠. 빈 자리를 찾아 놓지는 않음.
- 가려진 컵은 앞의 컵을 먼저 치우라고 사람이 명령해야 함. 로봇이 순서를 정하지는 않음.
- 가구와 사람의 자리는 여전히 표(`layout.py`)에서 읽음. 카메라로 보는 것은 컵뿐임.
- 시뮬레이터의 깊이 영상에는 잡음이 없음 (강의 47).
