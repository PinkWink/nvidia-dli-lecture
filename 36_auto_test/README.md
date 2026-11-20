# 36. ROS 2 로 시뮬레이터 조종하기: 자율주행 자동 시험 — 명령 모음

- 영상에서 입력하는 터미널 명령과 기대 출력을 순서대로 모았음. [35. 핑크시티에 Pinky Pro 두 대](../35_two_pinkys/README.md) 다음 내용임.

| 파일 |
|---|
| [nav_test.py](nav_test.py) |

## 1. 손으로 하던 시험을 자동으로

### 이번 강의에서 할 일

- 강의 18 · 35 에서는 목적지를 손으로 주고, 도착했는지 눈으로 확인했음.
- 이번에는 **스크립트 하나**가 로봇을 출발점에 놓고, 목적지를 주고, 도착을 판정함. 이것을 여러 번 되풀이해 **통과율**을 냄.
- 그러려면 시뮬레이터를 ROS 2 로 조종할 수 있어야 함. Isaac Sim 의 **Simulation Interfaces** 서비스를 씀.

### 시험 한 번의 순서

- 1 · 4 는 시뮬레이터에, 2 · 3 은 Nav2 에 하는 일임.
- 판정은 Nav2 의 말이 아니라 시뮬레이터가 아는 **실제 위치**로 함.

### 준비물

- 강의 11 의 Pinky Pro 준비(`prepare_pinky.sh`)와 강의 17 의 지도(`work/pink_city/pink_city_map.yaml`)
- 강의 18 에서 빌드한 `pinky_navigation` 패키지
- ROS 2 패키지 `simulation_interfaces` (아래에서 확인)

### simulation_interfaces 패키지 확인

```bash
source /opt/ros/jazzy/setup.bash
ros2 interface list | grep -c simulation_interfaces
```

```text
38
```

- 시뮬레이터 종류와 상관없이 같은 메시지로 조종하자는 ROS 2 의 표준 묶음임. Isaac Sim 이 이 표준을 구현함.
- `0` 이 나오면 `sudo apt install ros-jazzy-simulation-interfaces` 로 설치함. 강의 PC 에는 이미 있었음(1.5.1).

## 2. 시뮬레이터 켜기: 이번에는 한 줄이 더 있음

### 터미널 1: 핑크시티 + Pinky Pro

```bash
source /opt/ros/jazzy/setup.bash
cd ~/nvidia-dli-lecture
git pull
source common/isaac_ros_env.sh
~/isaacsim/python.sh 16_pink_city/pink_city.py --ros2 --sim-control --gui
```

- 강의 18 과 다른 곳은 두 군데임: `source common/isaac_ros_env.sh` 한 줄과 `--sim-control` 옵션.
- `--sim-control` = 시뮬레이터 조종 서비스를 켬.

### 출력

```text
Isaac Sim 내장 ROS 2 (jazzy) 를 씁니다. ROS_DOMAIN_ID = 0
...
ROS 2 연결: ROS_DOMAIN_ID = 0, 시작 위치 (-1.05, -0.5) 방향 90.0°
핑크시티: 3.0 × 3.0 m, 도로 폭 0.4 m, 블록 0.65 m
```

- 첫 줄은 `isaac_ros_env.sh` 가 낸 것임.
- `ROS_DOMAIN_ID` 를 정해 두었다면 그 번호가 나옴.

### 왜 한 줄이 더 필요한가

| | Python | 시뮬레이터 조종 서비스 |
|---|---|---|
| 시스템 ROS 2 (`/opt/ros/jazzy`) | 3.12 | Isaac Sim 이 불러오지 못함 |
| Isaac Sim 안의 ROS 2 | 3.11 | 동작함 |

- 지금까지의 토픽(`/scan`, `/odom` 등)은 Python 을 거치지 않아 시스템 ROS 2 로도 나왔음.
- 조종 서비스는 Python(`rclpy`)으로 만들어져 있어, Isaac Sim 의 Python 3.11 에 맞는 것이 필요함.

### isaac_ros_env.sh 가 하는 일

- **이 터미널에서만** 시스템 ROS 2 의 경로를 빼고, Isaac Sim 안에 들어 있는 ROS 2 Jazzy 를 쓰게 함.
- `ROS_DOMAIN_ID` 는 그대로 둠. 다른 터미널의 `ros2` 명령 · Nav2 · RViz2 와 그대로 통신함.
- 이 터미널에서는 `ros2` 명령을 쓸 수 없게 됨. 시뮬레이터만 띄우는 터미널이라 괜찮음.

### 빠뜨리고 실행하면

```text
--sim-control 은 Isaac Sim 안의 ROS 2 를 씁니다. 같은 터미널에서 먼저 실행하세요:  source common/isaac_ros_env.sh
There was an error running python
```

- 스크립트가 알려 주고 끝남. 안내대로 한 줄을 실행하고 다시 켬.
- 강의 PC 에서 이 확인 없이 섞어 켜 보니 서비스가 뜨지 않거나 Isaac Sim 이 죽었음.

## 3. ROS 2 서비스로 시뮬레이터 조종

### 터미널 2: 서비스 목록

```bash
source /opt/ros/jazzy/setup.bash
ros2 service list | grep -E "entit|simulat|world"
```

```text
/delete_entity
/get_available_worlds
/get_current_world
/get_entities
/get_entities_states
/get_entity_info
/get_entity_state
/get_simulation_state
/get_simulator_features
/load_world
/reset_simulation
/set_entity_state
/set_simulation_state
/spawn_entity
/step_simulation
/unload_world
```

### 이 강의에서 쓰는 서비스

| 서비스 | 하는 일 |
|---|---|
| `/get_simulation_state` · `/set_simulation_state` | 재생 · 일시정지 상태 읽기 · 바꾸기 |
| `/get_entity_state` | 물체의 위치 · 속도 읽기 |
| `/set_entity_state` | 물체를 원하는 자리로 옮기기 |
| `/reset_simulation` | 처음 상태로 되돌리기 |

- entity(엔티티) = 장면 안의 물체 하나. Stage 창에 보이는 Prim 경로가 이름임.

### 일시정지와 재생

```bash
ros2 service call /set_simulation_state simulation_interfaces/srv/SetSimulationState "{state: {state: 2}}"
ros2 service call /set_simulation_state simulation_interfaces/srv/SetSimulationState "{state: {state: 1}}"
```

```text
simulation_interfaces.srv.SetSimulationState_Response(result=simulation_interfaces.msg.Result(result=1, error_message=''))
```

- 1 = 재생, 2 = 일시정지. 응답의 `result=1` 은 성공임.
- 일시정지 중에는 `/clock` 이 멈추고, 재생하면 다시 1초에 60번 나옴 (강의 PC: 59.7).

### 로봇 위치 읽기

```bash
ros2 service call /get_entity_state simulation_interfaces/srv/GetEntityState "{entity: '/World/pinky/base_footprint'}"
```

```text
... result=1 ... position=geometry_msgs.msg.Point(x=-1.0500000715255737, y=-0.5000000596046448, z=1.1622901041619116e-07) ...
```

- 시뮬레이터가 아는 **실제 위치**임 (도시 좌표). 센서 잡음도 추정 오차도 없음.
- 응답이 길어 위치 부분만 보였음.

### 읽을 때는 몸체(base_footprint)로

| 읽은 이름 | 처음 (x, y) | 4초 직진 뒤 (x, y) |
|---|---|---|
| `/World/pinky` | (−1.05, −0.50) | (−1.05, −0.50) |
| `/World/pinky/base_footprint` | (−1.05, −0.50) | (−1.05, −0.09) |

- `/World/pinky` 는 로봇을 담은 맨 위 Prim 이라 주행해도 값이 그대로임.
- 실제로 움직이는 것은 그 아래의 몸체 `base_footprint` 임.

### 로봇 옮기기

```bash
ros2 service call /set_entity_state simulation_interfaces/srv/SetEntityState "{entity: '/World/pinky/base_footprint', state: {pose: {position: {x: 1.05, y: 0.5}, orientation: {z: -0.7071, w: 0.7071}}}}"
```

```text
... result=1, error_message='Successfully set state for entity: /World/pinky/base_footprint. Position, orientation, and velocities set successfully.'
```

- 로봇이 도시 좌표 (1.05, 0.5) 로 순간 이동함. `orientation` 은 −90° 를 보는 방향임.
- 속도를 적지 않으면 0 으로 들어가 멈춘 채로 옮겨짐.

### 옮길 때도 몸체로

| 옮긴 이름 | 옮긴 뒤 몸체의 실제 위치 | 목표 (1.05, 0.5) 와의 차이 |
|---|---|---|
| `/World/pinky` | (1.048, 0.083) | 42 cm |
| `/World/pinky/base_footprint` | (1.049, 0.495) | 0.5 cm |

- 0.41 m 주행한 로봇으로 잰 값임. `/World/pinky` 를 옮기면 그동안 주행한 만큼 어긋난 자리에 놓임.
- 주행한 적 없는 로봇은 둘 다 같은 자리에 놓여, 한 번만 해 보면 이 차이를 놓치기 쉬움.

### 처음으로 되돌리기: /reset_simulation

```bash
ros2 service call /reset_simulation simulation_interfaces/srv/ResetSimulation
```

- 로봇이 `/World/pinky` 의 자리(처음 자리)로 돌아가고 `/odom` 도 0 에서 다시 시작함.
- 강의 PC 에서 `/clock` 은 0 으로 돌아가지 않고 이어졌음 (38초 → 42초).
- 이 강의의 시험은 시작 자리를 바꿔 가며 하므로 초기화 대신 **옮기기**를 씀.

## 4. Nav2 와 함께 쓸 때

### 터미널 3: robot_state_publisher

```bash
source /opt/ros/jazzy/setup.bash
ros2 run robot_state_publisher robot_state_publisher --ros-args -p use_sim_time:=true -p robot_description:="$(cat ~/nvidia-dli-lecture/work/pinky_description/urdf/pinky.urdf)"
```

- 강의 18 의 터미널 2 와 같은 명령임.

### 터미널 4: Nav2

```bash
source /opt/ros/jazzy/setup.bash
source ~/pinky_pro/install/setup.bash
ros2 launch pinky_navigation bringup_launch.xml map:=$HOME/nvidia-dli-lecture/work/pink_city/pink_city_map.yaml params_file:=$HOME/nvidia-dli-lecture/18_pinky_nav2/nav2_params.yaml use_sim_time:=True
```

- 강의 18 의 터미널 3 과 같은 명령임. `Managed nodes are active` 가 두 번 나오면 준비된 것임.

### 터미널 5: RViz2

```bash
source /opt/ros/jazzy/setup.bash
source ~/pinky_pro/install/setup.bash
rviz2 -d ~/nvidia-dli-lecture/18_pinky_nav2/rviz/nav2.rviz --ros-args -p use_sim_time:=true
```

- 시험에는 없어도 됨. 무슨 일이 일어나는지 보려고 켬.

### 로봇만 옮기면: Nav2 는 모름

- 왼쪽 Isaac Sim 에서는 로봇이 본사 건너편(오른쪽 위 도로)에 있음.
- 오른쪽 RViz2 의 Nav2 는 옮겨진 것을 모름. 라이다로 만든 비용 지도(굵은 선)가 지도의 벽과 어긋나 비스듬히 얹힘.

### 새 위치를 알려 주면

- RViz2 의 `2D Pose Estimate` 와 같은 일을 토픽 `/initialpose` 로 함.
- 로봇 모델이 오른쪽 위로 옮겨지고, 비용 지도가 벽과 다시 겹침.

### 좌표 바꾸기: 지도 ↔ 도시

```python
def map_to_city(x, y, yaw_deg=0.0):
    """지도 좌표 → 시뮬레이터(도시) 좌표. 강의 18: 지도 (x, y) = 도시 (Y + 0.5, −X − 1.05)."""
    return -y - 1.05, x - 0.5, yaw_deg + 90.0


def city_to_map(x, y):
    return y + 0.5, -x - 1.05
```

- Nav2 는 지도 좌표로, 시뮬레이터는 도시 좌표로 말함. 시험 스크립트는 지도 좌표로 적고 필요할 때 바꿈.
- 지도 좌표의 (0, 0) 은 강의 17 에서 지도를 만들기 시작한 자리임.

## 5. 시험 스크립트 읽기

### 시험할 길

```python
BODY = "/World/pinky/base_footprint"
# 시험할 길: 이름, 출발 (x, y, 방향[도]), 목적지 (x, y). 모두 지도 좌표 [m]
ROUTES = [("출발점 → 본사 건너편", (0.0, 0.0, 0.0), (1.0, -2.1)),
          ("본사 건너편 → 출발점", (1.0, -2.1, 180.0), (0.0, 0.0)),
          ("출발점 → 사거리", (0.0, 0.0, 0.0), (0.5, -1.05))]
```

- `36_auto_test/nav_test.py` 의 앞부분임. 길 세 개를 차례로 돎.
- 길을 더하려면 이 표에 한 줄을 더하면 됨.

### 1. 옮기기

```python
request = SetEntityState.Request(entity=BODY)
request.state.pose.position.x, request.state.pose.position.y = city_x, city_y
request.state.pose.orientation.z = math.sin(math.radians(city_yaw) / 2)
request.state.pose.orientation.w = math.cos(math.radians(city_yaw) / 2)
answer = self.call(self.set_state, request)
if answer.result.result != 1:
    raise RuntimeError(f"순간 이동 실패: {answer.result.error_message}")
self.wait(1.0)
if math.dist(self.true_position(), (x, y)) > 0.05:
    raise RuntimeError("순간 이동을 했는데 로봇이 그 자리에 없습니다.")
```

- 3절에서 터미널로 부른 `/set_entity_state` 를 Python 으로 부름.
- 응답의 `result` 를 확인하고, 옮긴 뒤 위치를 다시 읽어 정말 그 자리인지 확인함.

### 2. 알리기

```python
self.estimate, end = None, self.now() + 10.0
while self.now() < end:
    if self.estimate is not None and math.dist(self.estimate, (x, y)) < 0.10:
        break
    self.initial.publish(message)
    self.wait(0.5)
else:
    return False
for client in self.clear:
    self.call(client, ClearEntireCostmap.Request())
```

- `/initialpose` 를 보내고, Nav2 의 추정 위치(`/amcl_pose`)가 새 자리로 옮겨질 때까지 확인함. 정해진 시간을 기다리지 않음.
- 그 뒤 비용 지도를 비움. 옮기기 전 자리에서 본 장애물 흔적이 남아 있기 때문임.

### 3. 달리기

```python
sent = self.nav.send_goal_async(goal)
rclpy.spin_until_future_complete(self, sent, timeout_sec=10.0)
handle = sent.result()
start, result = self.now(), handle.get_result_async()
while not result.done() and self.now() - start < limit:
    rclpy.spin_once(self, timeout_sec=0.05)
if not result.done():
    handle.cancel_goal_async()
    return "시간 초과", self.now() - start
return STATUS.get(result.result().status, str(result.result().status)), self.now() - start
```

- 강의 18 에서 `ros2 action send_goal` 로 한 일임 (일부만 보였음).
- 끝나면 상태(도착 · 중단)와 걸린 시간을 돌려줌. `--limit` 초를 넘기면 취소하고 "시간 초과" 로 적음.

### 4. 판정

```python
error = math.dist(self.true_position(), (goal_x, goal_y))
estimate_error = math.dist(self.true_position(), self.estimate) if self.estimate else float("nan")
passed = status == "도착" and error <= tolerance
```

- `true_position()` = `/get_entity_state` 로 읽은 실제 위치를 지도 좌표로 바꾼 것.
- 통과 = Nav2 가 "도착" 이라 했고, **실제 위치**가 목적지에서 `--tolerance`(기본 10 cm) 안.
- 추정 오차 = Nav2 가 생각하는 위치와 실제 위치의 거리. 판정에는 쓰지 않고 적어만 둠.

## 6. 돌려 보기

### 터미널 6: 12번

```bash
source /opt/ros/jazzy/setup.bash
cd ~/nvidia-dli-lecture
python3 36_auto_test/nav_test.py --trials 12 --out work/nav_test_result.json
```

- 길 세 개를 네 바퀴 돎.

### 출력

```text
시험 12번, 통과 기준: 도착 + 목적지에서 10 cm 안
  1  출발점 → 본사 건너편   도착        15.3초  목적지까지   8.2 cm  추정 오차  5.5 cm  →  통과
  2  본사 건너편 → 출발점   도착        17.1초  목적지까지   6.5 cm  추정 오차  5.5 cm  →  통과
  3  출발점 → 사거리      도착         9.8초  목적지까지   7.7 cm  추정 오차  1.6 cm  →  통과
...
  9  출발점 → 사거리      도착         9.1초  목적지까지  11.0 cm  추정 오차  1.9 cm  →  실패
...
통과 11/12 (92%)
  출발점 → 본사 건너편   통과 4/4  도착 4/4  평균 14.8초  목적지까지 평균 7.4 cm
  본사 건너편 → 출발점   통과 4/4  도착 4/4  평균 20.8초  목적지까지 평균 6.5 cm
  출발점 → 사거리      통과 3/4  도착 4/4  평균 9.6초  목적지까지 평균 8.7 cm
결과 저장: work/nav_test_result.json
```

### 움직이는 모습

- 길이 바뀔 때 로봇이 순간 이동하는 것이 보임. 사람은 아무것도 누르지 않음.

### 읽기

- 12번 모두 Nav2 는 "도착" 이라 했음. 그중 9번째는 실제로 11.0 cm 떨어져 서서 실패로 판정됨.
- Nav2 의 도착 허용 거리는 8 cm 임. Nav2 는 **자기가 추정한 위치**로 도착을 판단하므로, 실제 위치는 추정 오차만큼 더 벗어날 수 있음.
- 손으로 한두 번 해 봐서는 보이지 않는 실패임.

### 강의 PC 의 전체 결과 (설정은 강의 18 그대로)

| 조건 | 시험 수 | Nav2 도착 | 통과 (10 cm 안) | 목적지까지 실제 거리 |
|---|---|---|---|---|
| 출발 위치 고정 | 33 | 33 | 32 | 3.2 ~ 11.0 cm |
| 출발 위치를 5 cm 까지 흔듦 | 18 | 18 | 18 | 2.9 ~ 8.9 cm |

- 고정 33번 = 12번씩 두 차례 + 3번씩 세 차례 (그중 한 차례는 창을 띄운 채).
- 중단되거나 시간 초과가 난 시험은 없었음.

### 출발 위치 흔들기

```bash
python3 36_auto_test/nav_test.py --trials 6 --jitter 0.05 --seed 1
```

- 출발 위치를 매번 최대 5 cm 흔듦. 매번 똑같은 조건만 시험하지 않으려는 것임.
- `--seed` 가 같으면 같은 순서로 흔들림. 실패한 시험을 다시 해 볼 수 있음.

## 7. 설정값 하나를 바꿔 보기

### 바꿀 값: 도착 허용 거리

```bash
cd ~/nvidia-dli-lecture
sed 's/xy_goal_tolerance: 0.08/xy_goal_tolerance: 0.14/' 18_pinky_nav2/nav2_params.yaml > work/nav2_params_test.yaml
```

- `xy_goal_tolerance` = Nav2 가 "도착" 으로 인정하는 거리. 강의 18 에서 공식 값 0.25 를 0.08 로 줄였음.
- 터미널 4(Nav2)를 Ctrl+C 로 끄고, `params_file:=$HOME/nvidia-dli-lecture/work/nav2_params_test.yaml` 로 바꿔 다시 켬. 시뮬레이터는 그대로 둠.

### 0.14 로 바꾸면

```text
통과 0/12 (0%)
  출발점 → 본사 건너편   통과 0/4  도착 4/4  평균 14.4초  목적지까지 평균 11.6 cm
  본사 건너편 → 출발점   통과 0/4  도착 4/4  평균 20.3초  목적지까지 평균 11.2 cm
  출발점 → 사거리      통과 0/4  도착 4/4  평균 8.6초  목적지까지 평균 14.4 cm
```

- Nav2 는 12번 모두 "도착" 이라 했지만, 실제로는 10.1 ~ 15.6 cm 떨어져 섰음.
- 6번을 따로 돌린 때에는 2번이 10 cm 안(9.0 · 9.7 cm)이었음. 합치면 18번 중 2번 통과임.

### 0.25 (공식 값)로 바꾸면: 출발하지 못함

```text
  2  본사 건너편 → 출발점   시간 초과     91.0초  목적지까지 229.1 cm  추정 오차  6.9 cm  →  실패
  3  출발점 → 사거리      시간 초과     91.0초  목적지까지 119.1 cm  추정 오차  8.3 cm  →  실패
```

- 6번 모두 로봇이 출발점 근처에서 벗어나지 못했음. Nav2 로그에는 `Failed to make progress` 가 되풀이됨.
- 위치 추정은 맞았음(추정 오차 4 ~ 12 cm). 길을 못 찾은 것이 아니라 **가지 않은** 것임.

### 원인 좁히기

| 도착 허용 거리 | 최소 주시 거리 | 결과 |
|---|---|---|
| 0.14 | 0.15 | 출발하고 도착함 (18번) |
| 0.16 | 0.15 | 출발하지 못함 (6번) |
| 0.25 | 0.15 | 출발하지 못함 (6번) |
| 0.25 | 0.30 | 출발하고 도착함 (6번, 17 ~ 24 cm 에 섬) |

- 최소 주시 거리 `min_lookahead_dist` = 로봇이 경로에서 따라가려고 보는 점까지의 가장 짧은 거리. 강의 18 에서 0.3 을 0.15 로 줄였음.
- 도착 허용 거리가 이 값보다 크면 출발하지 못했음. 따라갈 점이 이미 "도착 범위" 안에 있는 셈임.
- 마지막 줄은 `lookahead_dist` 도 0.25 에서 0.3 으로 함께 올렸음.

### 한 장으로

- 점 하나가 시험 한 번임. 모든 점은 Nav2 가 "도착" 이라고 한 시험임.
- 허용 거리를 키우면 그만큼 멀리 섬. 0.08 일 때만 대부분 10 cm 안임.

### 읽기

- 공식 설정은 값들이 서로 맞물려 있음 (허용 거리 0.25 < 최소 주시 거리 0.3). 강의 18 에서 한쪽만 줄였다면 로봇이 출발하지 못했을 것임.
- 설정을 바꿀 때마다 같은 시험을 돌리면, 이런 일을 손으로 찾기 전에 숫자로 알게 됨.

## 8. 해 볼 것

### 직접 바꿔 보기

- `--tolerance 0.08` 로 통과 기준을 좁혀 돌려 보기. 어느 길이 먼저 실패하는지.
- `ROUTES` 에 길을 하나 더해 보기. 예: `("사거리 → 출발점", (0.5, -1.05, 90.0), (0.0, 0.0))`
- `desired_linear_vel` 을 0.2 에서 0.3 으로 올리고 12번 돌려 보기. 걸린 시간과 도착 거리가 어떻게 달라지는지?
- 결과 파일(`work/nav_test_result.json`)을 설정마다 따로 저장해 비교해 보기.

## 9. 정리

### 기억할 것

- Isaac Sim 은 ROS 2 서비스(Simulation Interfaces)로 조종할 수 있음: 재생 · 일시정지, 위치 읽기, 옮기기, 초기화.
- 이 기능을 쓸 때는 시뮬레이터 터미널에서 `source common/isaac_ros_env.sh` 를 한 줄 더 함.
- 로봇의 위치는 맨 위 Prim 이 아니라 몸체(`base_footprint`)로 읽고 옮김.
- 로봇을 옮겼으면 Nav2 에게 새 위치를 알리고 비용 지도를 비워야 함.
- 판정은 시험 대상(Nav2)의 말이 아니라 시뮬레이터의 실제 위치로 함.

### 한계

- 강의 PC 한 대, 길 세 개, 로봇 한 대로 잰 결과임. 횟수는 조건마다 6 ~ 33번임.
- "허용 거리가 최소 주시 거리보다 크면 출발하지 못함" 은 위 표의 네 조건으로 확인한 것임. Nav2 의 소스 코드와 대조하지는 않았음.
- `/reset_simulation` 뒤 시계가 이어지는 것은 두 번 확인했음. 초기화를 섞은 시험은 3번(모두 통과)만 해 봤음.
- `/spawn_entity`, `/step_simulation`, `/load_world` 는 다루지 않았음.
- 시스템 ROS 2 를 `source` 한 채 켜면 조종 서비스가 안 뜨는 원인은 Python 버전이 다른 것까지만 확인했음.
