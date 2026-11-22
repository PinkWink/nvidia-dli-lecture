# 44. 부스에서 Nav2 로 이동하기: 라이다 · 바퀴 오도메트리 · 도착 오차 — 명령 모음

- 영상에서 입력하는 터미널 명령과 기대 출력을 순서대로 모았음. [43. 컵 집기](../43_pl2m_pick/README.md) 다음 내용임.

| 파일 |
|---|
| [booth.py](booth.py) |
| [go_to.py](go_to.py) |
| [make_map.py](make_map.py) |
| [nav2.launch.py](nav2.launch.py) |
| [pl2m_nav.py](pl2m_nav.py) |

## 1. 팔이 일하려면 먼저 가야 함

### 이번 강의의 결과

- PL-2M 이 부스 안에서 목적지를 받아 스스로 감. 음료대 앞, 상담 테이블 앞, 처음 자리를 돎.
- 강의 43 까지는 로봇이 제자리에 서 있었음. 컵을 집어 나르려면 먼저 음료대까지 가야 함.

### 세 강의의 계획

| 강의 | 하는 일 | 로봇이 무엇을 보고 움직이나 |
|---|---|---|
| 44 (이번) | Nav2 로 스테이션 앞까지 이동 | 지도와 라이다 |
| 45 | ArUco 마커를 보고 테이블에 정확히 서기 (도킹) | 카메라에 보이는 마커 |
| 46 | 컵을 집어 다른 테이블로 나르기 | 위의 둘 + 팔 |

- 로봇은 일의 단계마다 다른 방법으로 움직임. 이 강의들에서는 그 단계를 모드라고 부름.
- 이번 강의의 질문: Nav2 만으로 컵을 집을 만큼 정확히 설 수 있는가.

### 만드는 것

| 파일 | 어디서 도나 | 하는 일 |
|---|---|---|
| `booth.py` | 양쪽 | 스테이션의 자리, 처음 자리, 부스의 범위 |
| `make_map.py` | 터미널 | Nav2 가 쓸 지도를 만듦 |
| `pl2m_nav.py` | Isaac Sim | 부스 + PL-2M + 라이다 + 바퀴 오도메트리 |
| `nav2.launch.py` · `nav2_params.yaml` | ROS 2 | Nav2 를 띄움 |
| `go_to.py` | ROS 2 | 목적지를 주고 도착 오차를 잼 |

- 터미널은 셋을 씀. Isaac Sim, Nav2, 명령을 치는 곳임.

## 2. 지도

### 터미널: 지도 만들기 (한 번만)

```bash
cd ~/nvidia-dli-lecture
git pull
python3 44_pl2m_nav/make_map.py
```

```text
원본 지도: 240 × 180 칸, 한 칸 5 cm, 막힌 칸 5428개
부스의 테두리를 그려 넣음: 막힌 칸 5914개 (+486)
저장: /home/user/nvidia-dli-lecture/work/booth/booth_map.yaml
```

- 원본 저장소에 부스의 지도가 들어 있음. 원본이 자기 MuJoCo 장면에서 바로 만든 것임. 강의 17 처럼 SLAM 으로 만들지 않고 그것을 가져옴.
- 거기에 부스의 테두리를 벽으로 그려 넣음.

### 지도와 스테이션

- 회색이 막힌 곳임. 벽 · 가구 · 기둥이 들어 있고 사람은 없음.
- 주황 점은 로봇이 처음 자리에서 라이다로 본 점임. 가구의 앞면에 겹쳐짐. 지도에 없는 점은 사람 모형임.
- 테이블은 지도에 상판 전체가 막힌 곳으로 들어 있음. 라이다는 그 아래의 다리와 마커 판만 봄.

### 코드: 부스의 테두리를 막음

```python
x0, y0, x1, y1 = AREA
top, left = cell(x0, y1 + resolution)          # 테두리 바로 바깥 칸에 그린다: 부스 안은 지우지 않는다
bottom, right = cell(x1 + resolution, y0 - resolution)
grid[top, left:right + 1] = 0
grid[bottom, left:right + 1] = 0
grid[top:bottom + 1, left] = 0
grid[top:bottom + 1, right] = 0
```

- 부스에서 벽이 없는 쪽은 통로와 그대로 이어짐. 라이다에는 그 경계가 보이지 않음.
- 로봇은 부스 안에서만 일함. 그래서 테두리를 지도에 벽으로 그려 넣어 Nav2 가 통로 쪽으로 길을 내지 않게 함.

### 코드: 스테이션의 자리

```python
STATIONS = {
    "drinks":    {"goal": (7.65, -2.82, math.pi / 2), "edge": (7.65, -1.80), "top": 0.75, "label": "음료대"},
    "consult_1": {"goal": (4.96, -1.90, 0.0),         "edge": (5.98, -1.90), "top": 0.72, "label": "상담 테이블 1"},
    "consult_2": {"goal": (5.06, -4.43, 0.0),         "edge": (6.08, -4.43), "top": 0.72, "label": "상담 테이블 2"},
    "home":      {"goal": SPAWN,                      "edge": None,          "top": None, "label": "처음 자리"},
}
```

- `goal` 이 Nav2 로 가는 자리와 방향임. 테이블 가장자리에서 1 m 떨어져 테이블을 바라봄. 값은 원본의 것임.
- 테이블에 바로 붙는 자리가 아님. 붙는 일은 다음 강의의 도킹이 함.

## 3. 라이다

### 터미널 1: Isaac Sim

```bash
source /opt/ros/jazzy/setup.bash
~/isaacsim/python.sh 44_pl2m_nav/pl2m_nav.py
```

```text
준비됨: 처음 자리 (5.30, -3.20), /odom = 바퀴로 셈한 값, 라이다 260°. 끝내려면 Ctrl+C
```

- 강의 38 의 부스에 PL-2M 을 올리고 라이다를 달았음. 팔은 이동할 때의 자세(transport)로 시작함.
- 창으로 보려면 `--gui` 를 붙임.

### 코드: 라이다 달기

```python
omni.kit.commands.execute(
    "RangeSensorCreateLidar", path="/Lidar", parent=LASER_LINK, min_range=LIDAR_RANGE[0], max_range=LIDAR_RANGE[1],
    horizontal_fov=360.0 if full else LIDAR_FOV, horizontal_resolution=1.0, vertical_fov=1.0, vertical_resolution=1.0,
    rotation_rate=0.0, high_lod=False, yaw_offset=0.0 if full else 180.0,      # laser_link 의 +X 가 뒤쪽이라 180° 돌려 앞을 가운데로
    draw_points=False, draw_lines=draw, enable_semantics=False)
```

- 강의 12 의 Pinky Pro 와 같은 PhysX 라이다임. URDF 의 `laser_link` 자리에 붙임.
- 1° 간격으로 앞쪽 260° 를 쏨. `/scan` 은 1초에 10번 냄.

### 뒤쪽은 자기 몸통

- 360° 로 쏘면 뒤쪽 90° 의 빛이 9 ~ 13 cm 에서 막힘. 라이다 바로 뒤에 팔이 달린 몸통이 있음.
- 그대로 Nav2 에 주면 로봇 바로 뒤에 늘 장애물이 있는 것이 됨. 그래서 앞쪽 260° 만 쏨.
- `--full-lidar` 로 띄우면 360° 로 쏘는 것을 볼 수 있음.

## 4. 바퀴 오도메트리

### 강의 41 과 다른 점

| | 강의 41 | 이번 |
|---|---|---|
| `/odom` | 시뮬레이터가 아는 참값 | 바퀴가 돈 각도로 셈한 값 |
| 실제 로봇과 | 다름 | 같음 (엔코더) |
| 참값 | `/odom` 이 곧 참값 | `/ground_truth` 로 따로 냄 |

- 실제 로봇은 자기 위치의 참값을 모름. 바퀴가 얼마나 돌았는지로 셈할 뿐임. 원본 시뮬레이터도 그렇게 함.
- `/ground_truth` 는 채점에만 씀. 로봇을 움직이는 프로그램은 이것을 보지 않음.

### 수식

- 두 바퀴가 굴러간 거리의 평균이 전진, 차이가 회전임.
- 바퀴가 미끄러지면 그만큼 틀림. 틀린 것은 쌓임.

### 코드

```python
def step(self, dt):
    now = self.view.get_dof_positions()[0][self.dofs]
    turned = (now - self.before + math.pi) % (2.0 * math.pi) - math.pi   # 관절 각도는 한 바퀴를 넘으면 되감긴다: 한 스텝의 변화만 쓴다
    left, right = turned * WHEEL_RADIUS                              # 두 바퀴가 굴러간 거리 [m]
    self.before = now.copy()
    forward, turn = (left + right) / 2.0, (right - left) / WHEEL_DISTANCE
    middle = self.yaw + turn / 2.0                                   # 이 스텝 동안의 가운데 방향
    self.x += forward * math.cos(middle)
    self.y += forward * math.sin(middle)
    self.yaw += turn
```

- 물리 스텝마다 바퀴 관절의 각도를 읽어 위의 수식을 그대로 셈함.
- 처음에는 둘째 줄이 없었음. 0.8 m 를 갔는데 오도메트리는 -0.25 m 라고 했음. 바퀴 관절의 각도가 한 바퀴를 넘으면 되감겨서 생긴 일임.

### 확인: 참값과 견줌

| 움직임 | 참값 | 바퀴로 셈한 값 |
|---|---|---|
| 0.2 m/s 로 4초 직진 | 0.796 m | 0.797 m |
| 0.5 rad/s 로 4초 제자리 회전 | 112.9° | 113.1° |
| 0.2 m/s · -0.3 rad/s 로 4초 곡선 | -67.5° | -67.3° |

- 처음 자리에서 `/cmd_vel` 을 직접 주어 잰 것임. 세 경우 모두 1 mm, 0.2° 안에서 맞음.
- 짧은 움직임 하나에서는 잘 맞음. 길게 달리면 이 작은 차이가 쌓임 (6절).

## 5. Nav2 띄우기

### 터미널 2: Nav2 와 RViz

```bash
source /opt/ros/jazzy/setup.bash
cd ~/nvidia-dli-lecture
ros2 launch 44_pl2m_nav/nav2.launch.py
```

```text
[lifecycle_manager-9] [INFO] [lifecycle_manager]: Managed nodes are active
```

- `Managed nodes are active` 가 나오면 준비된 것임. RViz 창이 함께 뜸.
- 띄우는 노드와 설정은 원본의 내비게이션 패키지를 따랐음.

### 띄우는 노드

| 노드 | 하는 일 |
|---|---|
| `robot_state_publisher` | 링크 사이의 `/tf` (라이다의 자리가 여기서 나옴) |
| `map_server` · `amcl` | 지도를 내고, 라이다로 지도 위의 위치를 추정함 |
| `planner_server` | 목적지까지의 길을 찾음 |
| `controller_server` | 길을 따라가는 `/cmd_vel` 을 냄 |
| `behavior_server` | 막혔을 때의 동작 (제자리 돌기 · 뒤로 가기) |
| `bt_navigator` | `/navigate_to_pose` 액션을 받아 위의 것들을 순서대로 부림 |

- 강의 18 에서 Pinky Pro 에 쓴 것과 같은 Nav2 임. 로봇의 크기와 속도에 맞춘 값이 다름.

### 설정에서 볼 값

```yaml
goal_checker:
  plugin: nav2_controller::SimpleGoalChecker
  xy_goal_tolerance: 0.08
  yaw_goal_tolerance: 0.15
  stateful: true
```

- Nav2 는 목표에서 8 cm, 0.15 rad(8.6°) 안에 들어오면 도착이라고 함. 이 값이 6절의 결과를 정함.
- 로봇의 발자국은 앞뒤 ±0.38 m, 좌우 ±0.32 m 로 주었음. 장애물 둘레는 0.55 m 를 부풀림.

### RViz 화면

- 짙은 보라가 지도의 막힌 곳임. 그 둘레의 하늘색과 분홍이 장애물을 부풀린 비용 지도임. 로봇은 흰 곳으로만 다님.
- 가운데 초록 네모가 로봇의 발자국, 거기서 뻗은 가는 선이 계획한 길임. 장애물 위의 작은 자주색 점들은 라이다가 본 것임.

## 6. 목적지 주기

### 터미널 3

```bash
source /opt/ros/jazzy/setup.bash
cd ~/nvidia-dli-lecture
python3 44_pl2m_nav/go_to.py drinks
```

```text
Nav2 에 목적지를 줌: 음료대 (7.65, -2.82, 90°)
  →  도착 (11.6초)
                    x[m]    y[m]  방향[°]
    목표             7.650  -2.820    90.0
    추정 (AMCL)      7.579  -2.843    83.6
    참값             7.522  -2.856    81.7
    도착 오차 (참값 - 목표): 거리 13.3 cm, 방향 -8.3°
    추정 오차 (추정 - 참값): 거리 5.9 cm, 방향 +1.9°
```

- 음료대 앞으로 가라고 했음. 11.6초 뒤 도착했고, 목표 · 추정 · 참값을 나란히 보여 줌.
- 추정은 AMCL 이 생각하는 자리임. 참값은 시뮬레이터만 아는 자리임. 로봇은 목표에서 13.3 cm 떨어져 서 있음.

### 코드: Nav2 에 목적지를 줌

```python
goal = NavigateToPose.Goal()
goal.pose.header.frame_id = "map"
goal.pose.header.stamp = self.get_clock().now().to_msg()
goal.pose.pose.position.x, goal.pose.pose.position.y = float(x), float(y)
goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
sent = self.nav.send_goal_async(goal)
```

- `/navigate_to_pose` 액션에 지도 좌표의 자리와 방향을 보냄. 결과가 올 때까지 기다림.
- Nav2 가 중단하면 한 번 더 보냄. 같은 자리라 다시 보내도 됨. 원본도 그렇게 함.

### 세 바퀴 돌며 재기

```bash
python3 44_pl2m_nav/go_to.py --tour 3
```

```text
  바퀴 목적지       결과      시간[s]   도착 오차[cm]   방향[°]   추정 오차[cm]   방향[°]
   1 drinks       도착        11.3         12.0      -7.7           5.8      +2.5
   1 consult_1    도착        45.1          2.5      +6.8           6.9      +0.2
   1 consult_2    도착        15.3         11.8      -6.8           5.5      -0.3
   1 home         도착         9.5          4.3      +6.8           1.5      +0.2
   2 drinks       도착        11.6         15.5      -6.2           8.2      +1.4
   2 consult_1    도착        18.0          9.0      +7.6           4.8      -0.6
   2 consult_2    도착        15.4         12.1      -5.1           3.1      -0.3
   2 home         도착         9.9          6.2      +5.3           1.4      +0.7
   3 drinks       도착        11.3         19.7      -8.7          11.7      +0.8
   3 consult_1    도착        18.4          7.2      +6.9           3.2      -0.7
   3 consult_2    도착        14.4         11.3      -5.9           4.3      -0.4
   3 home         도착        38.6          2.4      +6.6           3.4      -0.1
도착 12 / 12번.  도착 오차: 거리 평균 9.5 cm · 최대 19.7 cm, 방향 최대 8.7°
```

- 음료대 → 상담 1 → 상담 2 → 처음 자리를 세 바퀴 돎. 12번 모두 도착했음. 열 번은 한 구간에 10 ~ 18초, 두 번은 39초 · 45초가 걸렸음.
- 도착한 자리는 목표에서 평균 9.5 cm, 가장 클 때 19.7 cm 벗어나 있었음.

### 가끔 목표 가까이에서 맴돎

| 세 바퀴 돌기 | 제때 도착 (20초 안) | 오래 걸림 | 180초 안에 못 끝냄 |
|---|---|---|---|
| 첫째 (위의 표) | 10 | 2 (39초 · 45초) | 0 |
| 둘째 | 11 | 0 | 1 |
| 셋째 (둘째 바퀴 중간까지) | 3 | 2 (41초 · 135초) | 1 |

- 같은 세 바퀴를 세 번 돌린 결과임. 셋째는 정해 둔 시간(420초)이 끝나 여섯 구간만 했음.
- 오래 걸린 구간에서 로봇은 목표를 9 cm 쯤 지나쳐 서 있었음. 허용 범위 8 cm 밖이라 도착이 아니고, 되돌아가려고 제자리에서 돌기를 되풀이했음. Nav2 는 `Failed to make progress` 를 냈음.
- 이 강의에서는 고치지 않았음. 강의 45 의 도킹은 목표 가까이에서 Nav2 가 나아가지 않으면 목표를 거두고 마커를 보는 모드로 넘어감.

### 도착 오차

- 왼쪽: 목표를 가운데에 두고 로봇이 실제로 선 자리를 찍었음. 스테이션마다 늘 같은 쪽으로 벗어남. 음료대 앞에서는 12 ~ 20 cm 벗어남.
- 오른쪽: 방향의 오차임. 스테이션마다 늘 같은 쪽으로 5 ~ 9° 틀어져 있음.

### 왜 8 cm 를 넘나

- Nav2 는 자기가 추정한 위치로 도착을 판정함. 추정이 참값과 다르면 그만큼 더 벗어남. 위 표의 추정 오차가 1 ~ 12 cm 임.
- 추정이 틀리는 까닭 하나: 지도에는 테이블의 상판이 막힌 곳으로 들어 있는데 라이다는 그 아래의 다리만 봄. 사람 모형도 지도에 없음.
- 방향은 허용 범위 8.6° 안에 들어서자마자 멈춤. 그래서 늘 한쪽으로 5 ~ 8° 가 남음. 추정이 0.8° 틀렸을 때는 8.7° 가 됐음.

### 오도메트리는 얼마나 어긋났나

- 세 바퀴 26 m 를 달린 뒤 오도메트리와 참값의 차이가 23 cm, 5° 였음. 달린 거리의 1 % 쯤임.
- 4절의 짧은 시험에서는 1 mm · 0.2° 안에서 맞았음. 돌고 서기를 되풀이하는 동안 작은 차이가 쌓인 것임. 이 주행에서 로봇은 모두 3000° 넘게 돌았음.
- AMCL 은 오도메트리가 조금씩 어긋나는 것을 라이다와 지도로 바로잡음 (`/tf` 의 map → odom).

## 7. 이 정도로 컵을 집을 수 있나

### 집는 데 필요한 정확도

| | 값 |
|---|---|
| 그리퍼가 벌어지는 폭 | 8.8 cm (손가락마다 4.4 cm) |
| 컵의 지름 | 5 cm |
| 옆으로 틀려도 되는 거리 | ±1.9 cm |
| Nav2 의 도착 오차 | 평균 9.5 cm · 최대 19.7 cm, 방향 최대 8.7° |

- 손끝이 컵에서 1.9 cm 넘게 옆으로 벗어나면 손가락이 컵을 밀어 버림.
- Nav2 의 도착 오차는 그 5 ~ 10배임. 방향이 7° 틀리면 0.5 m 앞의 손끝은 6 cm 옆으로 감.
- 그래서 Nav2 는 테이블 앞 1 m 까지만 데려다주고, 마지막 구간은 다른 방법으로 감. 다음 강의의 도킹임.

## 8. 해 볼 것

### 직접 바꿔 보기

- `go_to.py 6.0 -3.0 90` 처럼 지도 위의 자리를 직접 줘 보기.
- `--true-odom` 으로 띄워 `/odom` 을 참값으로 내고, 도착 오차가 달라지는지 보기.
- `--full-lidar` 로 띄우고 RViz 에서 로봇 뒤의 비용 지도가 어떻게 되는지 보기.
- `nav2_params.yaml` 의 `yaw_goal_tolerance` 를 0.05 로 줄이고 방향의 오차를 다시 재 보기.

## 9. 정리

### 기억할 것

- Nav2 로 부스의 스테이션 앞까지 갔음. 지도는 원본의 것을 쓰고, 라이다에 보이지 않는 부스의 테두리는 지도에 그려 넣었음.
- 라이다는 자기 몸통을 보지 않게 앞쪽 260° 만 썼음.
- `/odom` 을 바퀴로 셈하게 바꿨음. 26 m 를 달리는 동안 23 cm · 5° 어긋났고, AMCL 이 그것을 바로잡음.
- Nav2 의 도착 오차는 평균 9.5 cm, 방향 5 ~ 9° 였음. 허용 범위(8 cm · 8.6°)와 추정 오차가 합쳐진 값임. 컵을 집기에는 모자람.

### 한계

- 평평한 부스 안에서, 움직이지 않는 사람 모형 사이를 다닌 결과임. 움직이는 사람은 없었음.
- Nav2 가 목표를 조금 지나쳐 선 뒤 제자리에서 맴도는 일이 30구간 가운데 6번 있었고, 그중 2번은 180초 안에 끝나지 않았음. 설정을 바꿔 고쳐 보지 않았음.
- 지도는 원본이 만든 것을 그대로 썼음. 이 장면에서 SLAM 으로 직접 만들어 보지 않았음.
- 부스의 테두리를 지도에 벽으로 그렸음. Nav2 에는 들어가면 안 되는 곳을 따로 주는 방법(keepout)도 있는데 써 보지 않았음.
- 강의 PC 한 대(Isaac Sim 5.1, ROS 2 Jazzy)에서 확인한 결과임.
