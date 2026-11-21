# 45. ArUco 마커로 도킹하기: Nav2 모드와 도킹 모드를 오가는 법 — 명령 모음

- 영상에서 입력하는 터미널 명령과 기대 출력을 순서대로 모았음. [44. 부스에서 Nav2 로 이동하기](../44_pl2m_nav/README.md) 다음 내용임.

| 파일 |
|---|
| [aruco_node.py](aruco_node.py) |
| [dock.launch.py](dock.launch.py) |
| [dock.py](dock.py) |
| [markers.py](markers.py) |
| [pl2m_dock.py](pl2m_dock.py) |

## 1. 마지막 1 m

### 이번 강의의 결과

- 로봇이 Nav2 로 음료대 앞까지 가고, 거기서부터는 테이블에 붙은 마커를 보며 정해진 자리에 섬. 일을 마치면 뒤로 물러나 다시 Nav2 로 다님.
- 화면 위의 글자가 지금의 모드임. 이 강의는 모드가 언제, 무엇을 확인하고 바뀌는지를 다룸.

### 강의 44 에서 남은 문제

| | Nav2 로 도착했을 때 | 컵을 집으려면 |
|---|---|---|
| 자리 | 평균 10.6 cm · 최대 19.0 cm 벗어남 | 옆으로 ±1.9 cm 안 |
| 방향 | 6 ~ 10° 틀어짐 | 2.2° 안 (0.5 m 앞의 1.9 cm) |

- Nav2 는 지도와 라이다로 위치를 추정함. 방 안을 다니기에는 충분하지만 컵을 집기에는 모자람.
- 테이블 앞에서는 지도 대신 테이블 자체를 보고 서면 됨. 테이블에 마커를 붙여 두고 카메라로 봄.

### 모드

| 모드 | 하는 일 | 무엇을 보고 움직이나 | `/cmd_vel` 을 내는 쪽 |
|---|---|---|---|
| NAV2 | 스테이션 앞 1 m 까지 | 지도 + 라이다 | Nav2 |
| SETTLE | Nav2 가 끝났는지, 멈췄는지 확인 | Nav2 의 목표 상태, 오도메트리의 속도 | 이 프로그램 (0 만) |
| SEARCH | 마커 두 장을 찾음 | 카메라 | 이 프로그램 |
| ALIGN | 옆으로 많이 벗어났으면 옆걸음 | 오도메트리 | 이 프로그램 |
| SERVO | 마커를 보며 다가감 | 카메라 | 이 프로그램 |
| BLIND | 마커가 화면을 벗어난 뒤의 직진 | 오도메트리 | 이 프로그램 |
| DOCKED | 서 있음 (팔이 일할 차례) | | 아무도 |
| UNDOCK | 뒤로 물러남 | 오도메트리 | 이 프로그램 |

- 모드는 한 번에 하나만 켜짐. 모드마다 자리를 재는 기준이 다름.
- 크게는 둘임. Nav2 가 로봇을 모는 NAV2 모드와, 이 강의의 프로그램이 모는 도킹 모드(SETTLE ~ UNDOCK).

### 모드가 바뀌는 조건

- 화살표 위의 글이 다음 모드로 넘어가는 조건임. 조건이 채워지지 않으면 넘어가지 않음.
- 절차와 값은 원본의 도킹 코드를 따랐음. 여기서는 그것을 모드 하나씩으로 풀어 씀.

### 만드는 것

| 파일 | 어디서 도나 | 하는 일 |
|---|---|---|
| `markers.py` | 양쪽 | 마커가 붙은 자리, 도킹의 값 |
| `pl2m_dock.py` | Isaac Sim | 강의 44 의 장면 + 마커 6장 + 앞 카메라 |
| `aruco_node.py` | ROS 2 | 영상에서 마커를 찾아 로봇 기준의 자리를 냄 |
| `dock.launch.py` | ROS 2 | 위 노드를 띄움 |
| `dock.py` | ROS 2 | 모드를 바꿔 가며 도킹함 |

- 터미널은 넷을 씀. Isaac Sim, Nav2, 마커 찾는 노드, 명령을 치는 곳임.

## 2. 마커와 카메라

### 터미널 1: Isaac Sim

```bash
source /opt/ros/jazzy/setup.bash
cd ~/nvidia-dli-lecture
git pull
~/isaacsim/python.sh 45_pl2m_dock/pl2m_dock.py
```

```text
준비됨: 시작 자리 home (5.300, -3.200, 0.0°), 마커 6장, 카메라 640 × 480. 끝내려면 Ctrl+C
```

- 강의 44 의 장면에 마커 6장과 앞 카메라를 더한 것임. 창으로 보려면 `--gui` 를 붙임.

### 터미널 2: Nav2

```bash
source /opt/ros/jazzy/setup.bash
cd ~/nvidia-dli-lecture
ros2 launch 44_pl2m_nav/nav2.launch.py
```

- 강의 44 와 같음. `Managed nodes are active` 가 나올 때까지 기다림.

### 터미널 3: 마커 찾는 노드

```bash
source /opt/ros/jazzy/setup.bash
source ~/pinklab_robotworld2026/install/setup.bash
cd ~/nvidia-dli-lecture
ros2 launch 45_pl2m_dock/dock.launch.py rsp:=false
```

- 이 노드는 원본의 메시지 형식을 씀. 그래서 강의 42 에서 빌드한 원본의 설치 폴더를 불러옴.
- `rsp:=false` 는 `robot_state_publisher` 를 Nav2 쪽이 이미 띄웠기 때문임.

### 코드: 마커가 붙은 자리

```python
MARKERS = {
    0: ("drinks",    (7.82, -1.805),  -math.pi / 2, -0.17),
    1: ("drinks",    (7.48, -1.805),  -math.pi / 2, +0.17),
    2: ("consult_1", (5.985, -2.05),  math.pi,      -0.15),
    3: ("consult_1", (5.985, -1.75),  math.pi,      +0.15),
    4: ("consult_2", (6.085, -4.58),  math.pi,      -0.15),
    5: ("consult_2", (6.085, -4.28),  math.pi,      +0.15),
}
```

- 스테이션마다 두 장임. 테이블 앞면, 바닥에서 0.3 m 높이에 좌우로 벌려 붙였음. 검은 네모의 한 변이 15 cm 임.
- 넷째 값이 스테이션의 축에서 옆으로 벗어난 거리임. 도킹한 로봇이 볼 때 왼쪽이 + 임.

### 코드: 마커 판과 카메라

```python
png = os.path.join(folder, f"aruco_{marker}.png")
cv2.imwrite(png, marker_image(marker))
material = make_booth.make_material(stage, f"aruco_{marker}", texture=png, roughness=0.9)
plate = make_booth.add_quad(stage, f"/World/Markers/aruco_{marker}", PLATE_SIZE / 2, PLATE_SIZE / 2)
make_booth.place(plate, (x, y, MARKER_HEIGHT), make_booth.rotation_from_axes([-math.sin(facing), math.cos(facing), 0.0, 0.0, 0.0, 1.0]))
UsdShade.MaterialBindingAPI.Apply(plate).Bind(material)
```

- 마커 그림을 OpenCV 로 만들어 PNG 로 저장하고, 그 그림을 입힌 네모 판을 세움. 강의 38 에서 벽 그림을 붙인 방법과 같음.
- 판에는 충돌체가 없음. 라이다에는 보이지 않고 카메라에만 보임.
- 카메라는 URDF 의 `base_camera_link` 자리에 달았음. 640 × 480, 세로 화각 65°, 1초에 10장임.

### 카메라에 보이는 마커

- 음료대 앞에 섰을 때 앞 카메라에 보이는 것임. 찾은 마커에 테두리와 번호를 그렸음. 왼쪽이 1번, 오른쪽이 0번임.
- `aruco_node.py --save markers.png` 로 이런 영상을 한 장 저장할 수 있음.

### 터미널: 찾은 마커를 출력

```bash
python3 45_pl2m_dock/aruco_node.py --print
```

```text
[    6.5초] 영상 10장, 늦음 17 ms:  0번 앞 1.023 · 왼쪽 -0.171 · 높이 0.283 m, 법선 -180.0°,  1번 앞 1.022 · 왼쪽 +0.173 · 높이 0.283 m, 법선 +178.4°
```

- 음료대 앞에 서 있을 때의 출력임. 두 장이 로봇의 앞 1.02 m, 좌우 ±0.17 m 에 있다고 나옴.
- 영상을 찍고 결과가 나오기까지 17 ms 임.

### 코드: 영상의 네 꼭짓점 → 로봇 기준의 자리

```python
half = MARKER_SIZE / 2.0
square = np.array([[-half, half, 0], [half, half, 0], [half, -half, 0], [-half, -half, 0]], np.float32)   # 왼쪽 위부터 시계 방향
ok, rotation, position = cv2.solvePnP(square, corners, matrix, None, flags=cv2.SOLVEPNP_IPPE_SQUARE)
```

- 마커의 한 변이 15 cm 라는 것을 알므로, 네 꼭짓점이 영상에서 어디에 찍혔는지로 카메라에서 본 위치와 방향이 나옴.
- 카메라 기준의 값은 `/tf` 로 로봇 기준(`base_footprint`)으로 바꿈. 카메라가 로봇의 어디에 달렸는지는 URDF 에 있음.
- 쓰는 것은 영상, 렌즈 값, 카메라가 달린 자리뿐임. 시뮬레이터의 참값을 쓰지 않으므로 실제 로봇에서도 같은 코드가 돎.

## 3. 마커 두 장으로 재는 세 값

### 그림

- 도킹에 필요한 것은 셋임. 마커 면까지의 거리, 스테이션의 축에서 옆으로 벗어난 거리, 축과 틀어진 각.
- 축은 두 마커의 가운데를 지나 테이블에 수직인 선임. 로봇이 서야 할 자리는 이 축 위, 면에서 0.43 m 임.

### 수식

- 방향을 두 마커를 잇는 선에서 얻는 것이 핵심임. 마커 한 장의 기울기에서 얻지 않음.

### 코드

```python
a, b = seen[0], seen[-1]
ux, uy = fixes[b][0] - fixes[a][0], fixes[b][1] - fixes[a][1]              # 오른쪽 마커 → 왼쪽 마커
length = math.hypot(ux, uy)
ux, uy = ux / length, uy / length
scale = length / (offsets[b] - offsets[a])                                 # 잰 간격 ÷ 실제 간격 (1 에 가까워야 한다)
ox, oy = fixes[a][0] - ux * offsets[a] * scale, fixes[a][1] - uy * offsets[a] * scale   # 축이 마커 면과 만나는 점
nx, ny = -uy, ux                                                               # 마커 면의 법선 (로봇 쪽을 향함)
return -(ox * nx + oy * ny), ox * ux + oy * uy, -math.atan2(ny, -nx)
```

- 수식을 그대로 옮긴 것임. 돌려주는 값이 along · side · turn 임.

### 두 장을 쓰는 까닭

```bash
python3 45_pl2m_dock/dock.py drinks --measure 100
```

```text
영상 100장 (서 있는 채로).                  면까지 [mm]            축 [mm]           각 [°]
  참값                                    1015.2              -0.0               0.0
  마커 두 장: 평균 ± 흩어짐       1023.8 ± 0.6        -1.3 ± 4.2         0.1 ± 0.2  
  마커 한 장: 평균 ± 흩어짐       1024.7 ± 2.6         2.6 ± 10.1       -0.2 ± 0.6  
```

- 음료대 앞 1 m 에 서서 영상 100장을 잰 것임. 한 장만 쓰면 옆 거리가 ±10 mm, 각이 ±0.6° 흩어짐. 두 장이면 ±4 mm, ±0.2° 임.
- 마커 한 장의 기울기는 네 꼭짓점의 미세한 차이에서 나옴. 영상에서 마커가 80 픽셀쯤이라 한두 픽셀에 크게 흔들림.
- 거리는 두 경우 모두 9 mm 쯤 멀게 나옴. 원인은 가려내지 못했음. 6절의 결과에 이 치우침이 들어 있음.

## 4. 모드를 하나씩

### 터미널 4: 가서, 도킹하고, 물러나기

```bash
source /opt/ros/jazzy/setup.bash
source ~/pinklab_robotworld2026/install/setup.bash
cd ~/nvidia-dli-lecture
python3 45_pl2m_dock/dock.py drinks --nav --undock
```

```text
[   12.7초] IDLE   → NAV2   (Nav2 이동)  음료대 앞 (7.65, -2.82, 90°) 으로
            Nav2: 도착 (12.4초). 도착 오차 15.2 cm · -9.8° (참값으로 채점)
[   25.4초] NAV2   → SETTLE (멈춤 확인)  Nav2 가 끝났는지, 베이스가 멈췄는지 봄
            멈춤: 0.5초 걸림
[   26.3초] SETTLE → SEARCH (마커 찾기)  마커 [0, 1] 를 찾음
            마커 [0, 1]: 면까지 1.094 m, 축이 왼쪽으로 -0.157 m, 돌아야 할 각 +10.8°
[   26.4초] SEARCH → ALIGN  (옆걸음)  축에서 15.7 cm 벗어남 (한계 5 cm): 옆걸음
[   43.8초] ALIGN  → SEARCH (마커 찾기)  옆걸음 뒤 다시 잼
            마커 [0, 1]: 면까지 1.093 m, 축이 왼쪽으로 +0.025 m, 돌아야 할 각 -1.3°
[   43.8초] SEARCH → SERVO  (마커 보며 접근)  마커를 보며 면까지 0.80 m 가 될 때까지
[   47.0초] SERVO  → BLIND  (오도메트리 직진)  면까지 0.797 m. 마지막으로 잰 값: 면까지 0.797 m · 축 +0.008 m · 각 -2.2°. 그 뒤 1.2 cm 를 이미 갔으므로 35.4 cm 를 오도메트리로 감
[   54.6초] BLIND  → DOCKED (도킹됨)  면까지 0.43 m 로 봄, 축이 왼쪽으로 +0.007 m, 축과 나란해지려면 +0.0°
채점 (참값 - 도킹 자리): 앞뒤 +1.9 mm, 좌우 -10.1 mm, 방향 -0.24°
     로봇이 아는 값:      좌우 -7.4 mm, 방향 -0.00°   (참값과의 차이 +2.7 mm, +0.24°)
[   54.8초] DOCKED → UNDOCK (물러나기)  오도메트리로 0.75 m 뒤로
[   65.7초] UNDOCK → IDLE   (대기)  물러남. 이제 Nav2 에 다음 목적지를 줄 수 있음
```

- 모드가 바뀔 때마다 한 줄씩 나옴. 시각, 앞 모드 → 새 모드, 바뀐 까닭임. 맨 위의 영상이 이 실행임.
- Nav2 로는 15.2 cm · 9.8° 벗어나 도착했음. 도킹을 마친 자리는 앞뒤 1.9 mm · 좌우 10.1 mm · 방향 0.24° 임.

### NAV2 → SETTLE: 멈춘 것을 확인함

```python
start = since = self.now()
while self.now() - start < timeout:
    self.command(0.0, 0.0)
    self.spin(0.05)
    linear, angular = self.speed()
    if abs(linear) > STILL_LINEAR or abs(angular) > STILL_ANGULAR:
        since = self.now()
    elif self.now() - since >= STILL_TIME:
        return self.now() - start
raise DockError("로봇이 멈추지 않음")
```

- Nav2 가 도착이라고 한 직후에도 로봇은 아직 굴러가고 있을 수 있음. 움직이는 중에 찍은 영상으로 자리를 재면 안 됨.
- 정해진 시간만큼 기다리지 않고, 오도메트리의 속도가 0 에 가까운 상태가 0.5초 이어지는 것을 확인함.

### NAV2 를 끝내는 조건 하나 더

```python
NEAR = (0.25, 5.0)                                             # Nav2 가 목표의 25 cm 안에서 5초 동안 나아가지 않으면 도착으로 본다: 도킹이 받아 줄 수 있는 범위
```

- Nav2 는 목표의 8 cm 안에 들어야 도착이라고 함. 그런데 목표를 10 cm 쯤 지나쳐 선 뒤, 도착도 아니고 나아가지도 않는 채로 있을 때가 있었음. 20초 뒤에야 복구 동작을 거쳐 도착함. 보통 11 ~ 19초인 구간이 39 ~ 106초가 됨.
- 도킹은 그보다 넓은 범위에서 시작할 수 있음. 옆걸음이 있고, 마커 두 장만 보이면 됨. 그래서 추정한 위치가 목표의 25 cm 안에 든 채 5초 동안 2 cm 도 나아가지 않으면, Nav2 의 목표를 거두고 다음 모드로 넘어감. 방향이 10° 넘게 틀어져 있으면 오도메트리로 돌려 맞춤.
- 모드를 끝내는 조건을 그 모드의 기준(8 cm)이 아니라 다음 모드가 받아 줄 수 있는 범위로 정한 것임.

### 그 조건을 확인함

```text
[   22.6초] ALIGN  → NAV2   (Nav2 이동)  음료대 앞 (7.65, -2.82, 90°) 으로
            Nav2 가 목표에서 6 cm 떨어진 곳(추정)에서 5초째 나아가지 않음: 목표를 거둠
            Nav2: 가까이서 멈춤 (7.4초). 도착 오차 11.9 cm · -2.6° (참값으로 채점)
```

- 목표에서 12 cm 떨어진 자리에 서 있는 로봇에게 같은 목표를 다시 준 것임. Nav2 는 5초 동안 나아가지 않았고, 이 조건으로 목표를 거뒀음. 거둔 뒤 Nav2 에 끝나지 않은 목표가 없는 것도 확인했음.
- 이 조건을 넣은 뒤의 주행 여섯 번(6절)에서는 Nav2 가 모두 11.6 ~ 18.1초에 도착해, 이 조건이 쓰이지 않았음.
- 저절로 쓰인 것은 강의 46 의 임무를 녹화할 때 한 번 보았음. Nav2 가 상담 테이블 앞 18 cm(추정)에 서 버렸고, 19.1초 만에 도킹으로 넘어가 임무를 마쳤음.

### SETTLE → SEARCH: 멈춘 뒤에 찍은 영상만 씀

```python
stamp, found, _ = self.markers
if self.now() - stamp > max_age or (since is not None and stamp < since):
    return {}
```

- 마커의 메시지에는 영상을 찍은 시각이 들어 있음. 멈춘 때(`since`)보다 먼저 찍은 영상과, 찍은 지 0.8초가 넘은 영상은 버림.
- 마커가 안 보이면 제자리에서 왼쪽 36°, 오른쪽 72° 로 돌며 찾음. 그래도 없으면 실패임.

### SEARCH → ALIGN: 옆으로 많이 벗어났으면 옆걸음

```python
def sidestep(self, lateral):
    """옆으로 lateral [m] 만큼 옮긴다 (+ 왼쪽): 90° 돌고, 직진하고, 다시 90° 돌아온다."""
    sign = 1.0 if lateral >= 0 else -1.0
    self.turn(sign * math.pi / 2)
    self.drive(abs(lateral))
    self.turn(-sign * math.pi / 2)
```

- 축에서 5 cm 넘게 벗어나 있으면 먼저 옆으로 옮김. 두 바퀴 로봇은 옆으로 못 가므로 돌고 · 가고 · 돌아옴.
- 이 동안은 마커를 보지 않고 오도메트리로 움직임. 끝나면 SEARCH 로 돌아가 다시 잼.
- 남은 거리가 1 m 뿐이라, 다가가면서 옆으로 옮길 수 있는 양이 작음. 그래서 먼저 맞춤.

### SERVO: 마커를 보며 다가감

- 위 둘이 다가갈 때의 속도 명령임. 남은 거리에 비례해 전진하고, 축 쪽으로 틀면서 축과 나란해지게 돎.
- 셋째 줄은 BLIND 모드에서 오도메트리로 갈 거리임.

### SERVO: 코드

```python
linear = min(DOCK["max_speed"], max(0.03, DOCK["gain"] * (along - DOCK["target_dx"])))
angular = DOCK["lateral_gain"] * side / max(along, 0.3) + DOCK["heading_gain"] * turn
angular = max(-0.3, min(0.3, angular))
self.command(linear, angular)
```

- 영상이 올 때마다 세 값을 다시 재서 속도를 정함. 1초에 10번임.
- 마커 두 장이 다 보이지 않는 영상이 오면 서서 다음 영상을 기다림. 5초를 기다려도 안 보이면 실패로 끝냄.

### SERVO → BLIND: 0.80 m 에서 눈을 감음

```bash
python3 45_pl2m_dock/dock.py drinks --visual-end 0.45
```

```text
[    5.6초] SEARCH → SERVO  (마커 보며 접근)  마커를 보며 면까지 0.45 m 가 될 때까지
            면까지 0.725 m · 축 +0.002 m · 각  -0.2°  →  전진 0.10 m/s · 회전 +0.00 rad/s
            면까지 0.674 m · 축 -0.003 m · 각  +0.1°  →  전진 0.10 m/s · 회전 -0.00 rad/s
            마커가 다 보이지 않게 됨: 마지막으로 잰 면까지의 거리 0.634 m
[   14.9초] SERVO  → FAILED (실패)  마커가 5초째 다 보이지 않음 (마지막으로 잰 면까지의 거리 0.634 m)
```

- 마커를 보며 0.45 m 까지 가 보라고 한 것임. 0.63 m 에서 마커 두 장이 다 보이지 않게 됐고, 로봇은 서서 5초를 기다린 뒤 그만둠.
- 가까이 가면 좌우로 벌려 붙인 마커가 화면 밖으로 나감. 원본의 0.80 m 는 그보다 여유를 둔 값임.

### BLIND: 남은 거리는 오도메트리로

```python
travelled = math.hypot(x - last_pose[1], y - last_pose[2]) if last_pose else 0.0     # 그 영상을 찍은 뒤 이미 간 거리
remaining = along - travelled - DOCK["target_dx"]
if abs(turn) > 0.02:                                   # 1.1° 넘게 틀어져 있으면 제자리에서 돌아 축과 나란히 선다
    self.turn(turn)
    side, turn = side + travelled * math.sin(turn), 0.0
else:                                                  # 작으면 그대로 간다. 비스듬히 가는 만큼 옆으로 옮겨지는 것을 셈해 둔다
    side = side + (along - DOCK["target_dx"]) * math.sin(turn)
self.drive(remaining, DOCK["blind_speed"])
```

- 마지막으로 마커를 본 영상의 값에서 출발함. 그 영상을 찍은 뒤 이미 간 거리는 빼야 함. 영상을 찍은 순간의 오도메트리를 찾아 둔 까닭임.
- 조금 틀어진 채로 직진하면 옆으로도 옮겨짐. 0.36 m 를 1° 틀어져 가면 6 mm 임. 이것을 셈해서 side 를 고쳐 둠.

### DOCKED: 다음 모드에 건네는 값

| 값 | 뜻 | 위의 실행에서 |
|---|---|---|
| 0.43 m | 마커 면까지의 거리 | 참값 0.428 m |
| side | 축이 로봇의 왼쪽으로 벗어난 거리 | 로봇이 아는 값 +7.4 mm, 참값 +10.1 mm |
| turn | 축과 나란해지려면 돌아야 하는 각 | 로봇이 아는 값 0.0°, 참값 +0.24° |

- 도킹은 옆으로 벗어난 것을 다 없애지 않음. 대신 얼마나 벗어나 섰는지를 알고 있음.
- 이 세 값이 도킹 모드가 팔에게 건네는 것임. 팔은 이것으로 컵의 자리를 셈함 (강의 46).

### 한 번의 기록

- 축에서 오른쪽으로 12 cm, 왼쪽으로 7° 틀어진 자리에서 시작한 도킹임. 색 띠가 모드, 검은 선이 참값, 주황 점이 로봇이 마커로 잰 값임.
- ALIGN 에서 90° 돌아 옆 거리를 0 으로 만들고 되돌아옴. SERVO 에서 면까지 0.80 m, BLIND 에서 0.43 m 가 되고, UNDOCK 에서 다시 멀어짐.
- 주황 점은 SEARCH 와 SERVO 에만 있음. 나머지 모드에서는 로봇이 마커를 보지 않음.

## 5. `/cmd_vel` 은 한 번에 한쪽만

### 코드: Nav2 모드에서는 내지 않음

```python
def command(self, linear, angular):
    """이 프로그램이 /cmd_vel 을 낸다. Nav2 모드에서는 부르지 않는다."""
    if self.mode == "NAV2":
        raise DockError("Nav2 모드에서는 이 프로그램이 /cmd_vel 을 내지 않음")
    self.publish(linear, angular)
    if (float(linear), float(angular)) != (0.0, 0.0):
        self.pending.append((float(linear), float(angular)))
```

- Nav2 와 이 프로그램은 같은 토픽 `/cmd_vel` 로 로봇을 몲. 둘이 함께 내면 로봇은 두 명령을 번갈아 받음.
- 그래서 모드로 나눔. NAV2 모드에서는 이 프로그램이 내지 않고, 도킹 모드에서는 Nav2 에 목적지를 주지 않음.

### 코드: 남이 낸 명령을 알아챔

```python
value = (msg.linear.x, msg.angular.z)
if value == (0.0, 0.0):
    return
if value in self.pending:
    self.pending.remove(value)
else:
    self.foreign += 1
```

- 이 프로그램도 `/cmd_vel` 을 듣고 있음. 0 이 아닌 명령인데 자기가 낸 것이 아니면 다른 노드가 로봇을 움직이는 중임.
- 도킹을 시작할 때 0.5초 동안 듣기만 함. 그런 명령이 들리면 아무것도 내지 않고 그만둠. 도킹 중에 들려도 그만둠.

### 코드: Nav2 가 끝났는지 물어봄

```python
def nav_busy(self, wait=2.0):
    """Nav2 에 아직 끝나지 않은 목표가 있나. 방금 끝난 목표의 상태가 늦게 올 수 있어 wait 초까지 기다려 본다."""
    busy = (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING, GoalStatus.STATUS_CANCELING)
    end = time.monotonic() + wait
    while True:
        self.spin(0.05)
        if not any(status in busy for status in self.nav_goals):
            return False
        if time.monotonic() > end:
            return True
```

- Nav2 의 액션 서버는 받은 목표들의 상태를 `/navigate_to_pose/_action/status` 로 알림. 끝나지 않은 목표가 있으면 도킹을 시작하지 않음.
- `/cmd_vel` 을 듣는 것만으로는 모자랐음. Nav2 가 목표를 받고 아직 움직이기 전이면 조용하기 때문임.

### 해 보기: Nav2 가 움직이는 중에 도킹을 시작함

```text
[   12.6초] IDLE   → SETTLE (멈춤 확인)  Nav2 가 끝났는지, 베이스가 멈췄는지 봄
[   14.2초] SETTLE → FAILED (실패)  Nav2 에 끝나지 않은 목표가 있음: Nav2 가 끝난 뒤에 도킹할 것
```

- 다른 터미널에서 `go_to.py home` 을 준 직후에 `dock.py drinks` 를 실행한 것임. 도킹 프로그램은 아무 명령도 내지 않고 1.6초 만에 그만둠. Nav2 는 17.0초에 처음 자리에 도착했음.
- 이 확인을 넣기 전에는 달랐음. 도킹 프로그램이 멈추라고 0 을 내고 Nav2 는 가라고 내서, 10초 뒤 `로봇이 멈추지 않음` 으로 끝났음.
- 그때 로봇은 2.4 m 가운데 1.0 ~ 1.2 m 를 간 자리에 있었음 (두 번 해 봄). 두 프로그램이 한 토픽에 함께 내면 어느 쪽도 뜻대로 되지 않음.

## 6. 얼마나 정확한가

### 시작 자리를 바꿔 가며

| 시험 | 시작 (좌우 · 방향) | 옆걸음 | 도킹 뒤 앞뒤 | 좌우 | 방향 |
|---|---|---|---|---|---|
| 1 | 0 cm · 0° | 안 함 | +3.9 mm | +0.0 mm | +0.01° |
| 2 | +3 cm · +4° | 안 함 | +1.9 mm | +33.6 mm | +0.31° |
| 3 | -4 cm · -6° | 안 함 | +5.0 mm | -43.5 mm | -0.14° |
| 4 | +8 cm · -8° | 함 | +3.5 mm | -10.7 mm | -0.09° |
| 5 | +10 cm · 0° | 함 | +2.6 mm | -2.0 mm | +1.16° |
| 6 | -12 cm · +7° | 함 | +3.4 mm | -0.5 mm | -0.04° |

- 음료대 앞에서 시작 자리만 바꿔 도킹한 결과임 (`pl2m_dock.py --start drinks --offset 0.10 0`). 값은 참값에서 도킹 자리를 뺀 것임.
- 앞뒤는 모두 5 mm 안, 방향은 1.2° 안임. 좌우는 시작이 5 cm 안이면 그대로 남고, 넘으면 옆걸음으로 1 cm 안이 됨.

### 그림으로

- 파란 점이 시작, 주황 점이 도킹한 뒤임. 회색 띠 안에서 시작한 시험은 좌우가 거의 그대로임.
- 옆걸음을 끄면(`--no-align`) 10 cm 벗어난 자리에서 시작해 7.2 cm 가 남았음.

### Nav2 → 도킹을 여섯 번

```bash
python3 45_pl2m_dock/dock.py --tour 2
```

```text
  1바퀴 drinks     Nav2 도착 오차  16.1 cm ·  -7.4°   →   도킹 뒤 앞뒤   +1.9 mm · 좌우  -12.7 mm (로봇이 아는 값  -11.6) · 방향 -0.70° (아는 값 -0.00)
  1바퀴 consult_1  Nav2 도착 오차  10.7 cm ·  +7.3°   →   도킹 뒤 앞뒤   +3.4 mm · 좌우   +0.6 mm (로봇이 아는 값   +0.6) · 방향 +0.23° (아는 값 +0.10)
  1바퀴 consult_2  Nav2 도착 오차  10.0 cm ·  -6.0°   →   도킹 뒤 앞뒤   +2.7 mm · 좌우   +0.6 mm (로봇이 아는 값   -1.3) · 방향 +0.76° (아는 값 +0.98)
  2바퀴 drinks     Nav2 도착 오차  13.9 cm ·  -7.9°   →   도킹 뒤 앞뒤   +4.5 mm · 좌우   +1.0 mm (로봇이 아는 값   +1.8) · 방향 -0.08° (아는 값 -0.14)
  2바퀴 consult_1  Nav2 도착 오차  10.7 cm ·  +7.3°   →   도킹 뒤 앞뒤   +4.1 mm · 좌우  +17.9 mm (로봇이 아는 값  +17.7) · 방향 +0.81° (아는 값 -0.00)
  2바퀴 consult_2  Nav2 도착 오차  10.6 cm ·  -7.4°   →   도킹 뒤 앞뒤   +3.5 mm · 좌우   +3.1 mm (로봇이 아는 값   +0.3) · 방향 -0.00° (아는 값 +0.43)
```

- 스테이션 셋을 두 바퀴 돌며 Nav2 → 도킹 → 물러나기를 되풀이했음. 여섯 번 모두 도킹했음.
- 괄호 안이 로봇이 아는 값(side · turn)임. 참값과 좌우 3 mm, 방향 0.8° 안에서 맞음. 다음 강의에서 팔이 이 값을 씀.

### 같은 주행에서의 오차

- 네모가 Nav2 로 도착했을 때, 동그라미가 도킹한 뒤임. 자리의 오차가 10 ~ 16 cm 에서 0.3 ~ 1.8 cm 로, 방향이 6 ~ 8° 에서 0.8° 안으로 줄었음.
- 도킹 뒤에 남은 것은 대부분 좌우임. 5 cm 안이면 옆걸음을 하지 않기 때문임. 그 값은 로봇이 알고 있음.

## 7. 다시 Nav2 로

### 물러나지 않고 Nav2 를 부르면

| | 도킹한 자리에서 바로 | 0.75 m 물러난 뒤 |
|---|---|---|
| 상담 테이블 1 앞까지 걸린 시간 | 108.2초 | 17.8초 |
| Nav2 의 출력 | `No valid trajectories out of 464` · `Failed to make progress` 2번 | 없음 |

- 도킹한 자리는 테이블에서 13 cm 임. Nav2 의 비용 지도에서는 장애물 둘레를 부풀린 곳 안이라, 갈 수 있는 궤적을 찾지 못함.
- 막혔을 때의 동작을 거듭한 끝에 빠져나왔음. 걸린 시간이 6배임.
- 그래서 도킹 모드는 스스로 뒤로 물러난 뒤에 끝남. 물러나는 것은 지도를 쓰지 않고 오도메트리로 함.

### 마커를 찾지 못하면

```text
[   10.0초] SETTLE → SEARCH (마커 찾기)  마커 [0, 1] 를 찾음
            마커 [0, 1] 가 안 보임: 제자리에서 돌며 찾음
[   26.4초] SEARCH → FAILED (실패)  마커 [0, 1] 를 찾지 못함 (1/2번째)
[   31.1초] FAILED → NAV2   (Nav2 이동)  음료대 앞 (7.65, -2.82, 90°) 으로
            Nav2: 도착 (9.1초). 도착 오차 17.4 cm · -2.2° (참값으로 채점)
[   41.6초] SETTLE → SEARCH (마커 찾기)  마커 [0, 1] 를 찾음
            마커 [0, 1] 가 안 보임: 제자리에서 돌며 찾음
[   58.1초] SEARCH → FAILED (실패)  마커 [0, 1] 를 찾지 못함 (2/2번째)
```

- 마커를 붙이지 않고(`pl2m_dock.py --no-markers`) 해 본 것임. 제자리에서 좌우로 돌며 찾다가 실패로 끝냄.
- 실패하면 30 cm 물러나 NAV2 모드로 돌아가, 스테이션 앞에 다시 선 뒤 한 번 더 함. 두 번째도 실패하면 그만둠.
- 실패했을 때 돌아갈 곳이 있다는 것이 모드를 나눈 덕임. 어디서 실패하든 물러나서 Nav2 로 다시 시작함.

## 8. Nav2 의 도킹 서버

### 같은 순서

| 이 강의의 모드 | Nav2 도킹 서버의 상태 (`DockRobot` 액션의 feedback) |
|---|---|
| NAV2 | `NAV_TO_STAGING_POSE` |
| SEARCH | `INITIAL_PERCEPTION` |
| SERVO · BLIND | `CONTROLLING` |
| DOCKED | `WAIT_FOR_CHARGE` |
| 실패했을 때 다시 | `RETRY` |
| UNDOCK | `UndockRobot` 액션 |

- ROS 2 Jazzy 의 Nav2 에는 도킹 서버(`opennav_docking`)가 들어 있음. 액션 정의에 적힌 상태의 이름이 이 강의의 모드와 같은 순서임.
- 오른쪽 이름은 설치된 파일 `/opt/ros/jazzy/share/nav2_msgs/action/DockRobot.action` 에서 옮긴 것임.
- 이 강의는 그 서버를 쓰지 않았음. 원본의 절차를 직접 짜서 모드가 바뀌는 조건을 하나씩 보았음. 그 서버를 PL-2M 에 써 보지는 않았음.

## 9. 해 볼 것

### 직접 바꿔 보기

- `pl2m_dock.py --start drinks --offset 0.04 5` 로 띄우고 `dock.py drinks` 만 실행해, 6절의 표에 한 줄을 더해 보기.
- `dock.py drinks --single` 로 마커 한 장만 써서 도킹하고, 방향의 오차를 두 장일 때와 견줘 보기.
- `ros2 launch 45_pl2m_dock/dock.launch.py rsp:=false window:=true` 로 카메라 영상을 창으로 보며 도킹해 보기.
- `markers.py` 의 `max_lateral_error` 를 0.02 로 줄이고, 옆걸음이 얼마나 자주 일어나는지 보기.

## 10. 정리

### 기억할 것

- Nav2 는 스테이션 앞 1 m 까지, 마지막 구간은 마커를 보고 감. 모드마다 자리를 재는 기준이 다름: 지도, 카메라, 오도메트리.
- 모드를 바꾸기 전에 확인함: 멈췄는가, 멈춘 뒤에 찍은 영상인가, 마커 두 장이 보이는가, 다른 노드가 로봇을 움직이고 있지 않은가.
- 방향은 마커 두 장을 잇는 선에서 얻음. 마커가 화면을 벗어나는 마지막 구간은 오도메트리로 감.
- 도킹은 선 자리를 세 값(거리 · side · turn)으로 다음 모드에 건넴.
- 일을 마치면 물러난 뒤에 Nav2 로 돌아감. 물러나지 않으면 Nav2 가 길을 못 찾고 헤맸음.

### 한계

- 이 시뮬레이터의 바퀴는 거의 미끄러지지 않았음. 오도메트리로 가는 구간(옆걸음 · 마지막 직진 · 물러나기)이 실제 로봇에서는 더 틀릴 것임.
- 영상이 17 ms 만에 처리됐음. 원본은 영상이 몇 초 늦게 오는 경우까지 대비해 찍은 시각을 따짐. 여기서는 그런 지연을 만들어 보지 않았음.
- 마커는 조명이 고른 자리에 붙어 있고 가려지지 않았음. 사람이 가리거나 조명이 번지는 경우는 보지 않았음.
- 거리가 9 mm 쯤 멀게 재어지는 까닭은 찾지 못했음.
- 도킹은 시작 자리 여섯 가지와 주행 여섯 번을 해 본 것임. 실패가 얼마나 잦은지는 세지 않았음.
- 강의 PC 한 대(Isaac Sim 5.1, ROS 2 Jazzy)에서 확인한 결과임.
