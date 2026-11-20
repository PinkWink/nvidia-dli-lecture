# 35. 핑크시티에 Pinky Pro 두 대: 네임스페이스와 동시 자율주행 — 명령 모음

영상에서 입력하는 터미널 명령과 기대 출력을 순서대로 모았습니다. [34. 사족 로봇을 걷게 하기](../34_quadruped/README.md) 다음 내용입니다.

| 파일 |
|---|
| [two_pinky_nav2.launch.py](two_pinky_nav2.launch.py) |
| [two_pinkys.py](two_pinkys.py) |

## 1. 로봇이 두 대가 되면

### 이번 강의에서 할 일

- 강의 16 ~ 18 의 핑크시티에 Pinky Pro 를 **두 대** 올립니다.
- 두 대가 각자의 Nav2 로 **동시에** 목적지까지 갑니다.
- 그러려면 두 로봇의 토픽이 섞이지 않게 나눠야 합니다. 그 방법이 **네임스페이스**입니다.

### 준비물

- 강의 11 의 Pinky Pro 준비(`prepare_pinky.sh`)와 강의 17 의 지도(`work/pink_city/pink_city_map.yaml`)
- 강의 18 에서 빌드한 `pinky_navigation` 패키지
- 모든 터미널에서 먼저 `source /opt/ros/jazzy/setup.bash`

### 먼저, 아무것도 나누지 않고 두 대를 올려 보기

```bash
source /opt/ros/jazzy/setup.bash
cd ~/nvidia-dli-lecture
git pull
~/isaacsim/python.sh 35_two_pinkys/two_pinkys.py --gui --no-namespace
```

- `--no-namespace`: 두 로봇이 강의 14 와 똑같은 토픽 이름을 그대로 씁니다. 일부러 겹치게 해 보는 것입니다.

### 결과: 토픽 하나를 두 로봇이 함께 쓴다

```text
$ ros2 topic info /scan
Publisher count: 2
$ ros2 topic info /cmd_vel
Subscription count: 2
```

- `/scan` 을 내보내는 쪽이 둘입니다. 받는 쪽은 어느 로봇의 라이다인지 알 수 없습니다. 주기도 10 Hz 가 아니라 약 20 Hz 로 보입니다.
- `/cmd_vel` 을 듣는 쪽도 둘입니다.

### /cmd_vel 한 번에 두 대가 함께 움직인다

| | 명령 전 (x, y) | `/cmd_vel` 로 0.1 m/s 를 5초 보낸 뒤 |
|---|---|---|
| pinky1 | (−1.05, −0.50) | (−1.05, 0.01) |
| pinky2 | (1.05, 0.50) | (1.05, −0.01) |

- 두 대가 각자 앞으로 0.5 m 씩 갔습니다. 한 대만 움직일 방법이 없습니다.
- `/odom` 과 `/tf` 도 내보내는 쪽이 둘입니다. 어느 로봇의 위치인지 가릴 수 없어, 이 상태로는 Nav2 를 쓸 수 없습니다.

## 2. 네임스페이스

### 토픽 이름 앞에 로봇 이름을 붙인다

- `/cmd_vel` → `/pinky1/cmd_vel`, `/pinky2/cmd_vel`. 이름이 다르니 섞이지 않습니다.
- `/tf` 도 로봇마다 따로 둡니다. 그래서 두 로봇이 `odom`, `base_footprint` 같은 프레임 이름을 똑같이 써도 됩니다.
- `/clock` 만은 공통입니다. 시뮬레이션의 시계는 하나이기 때문입니다.

### Isaac Sim 쪽: Node Namespace 칸

```python
def _namespaced(nodes, namespace):
    return [(f"{node}.inputs:nodeNamespace", namespace) for node in nodes] if namespace else []
```

- Isaac Sim 의 ROS 2 노드(Subscribe Twist, Publish Odometry …)에는 모두 **Node Namespace** 칸이 있습니다.
- 여기에 `pinky1` 을 넣으면 그 노드의 토픽 `cmd_vel` 이 `/pinky1/cmd_vel` 이 됩니다.
- 강의 14 의 그래프 함수에 이 한 줄을 더해, 이름을 인자로 받게 했습니다(`14_pinky_ros2/pinky_ros2.py`).

### 로봇 한 대 = 반복문 한 바퀴

```python
ROBOTS = [("pinky1", -1.05, -0.5, 90.0),
          ("pinky2", 1.05, 0.5, -90.0)]

for k, (name, x, y, yaw) in enumerate(ROBOTS):
    robot = f"/World/{name}"                       # 로봇마다 다른 Prim 경로
    import_pinky(stage, urdf, height=0.0, path=robot)
    add_lidar(stage, robot=robot)
    add_cmd_vel_subscriber(robot=robot, namespace=name, graph=f"/Graphs/{name}_cmd_vel")
    add_state_publishers(robot=robot, namespace=name, graph=f"/Graphs/{name}_state", clock=(k == 0))
    add_sensor_publishers(robot=robot, namespace=name, graph=f"/Graphs/{name}_sensors")
```

- `35_two_pinkys/two_pinkys.py` 의 핵심입니다(일부만 보였습니다). 강의 11 · 12 · 14 의 함수를 그대로 부릅니다.
- 로봇마다 다른 것은 세 가지뿐입니다: **Prim 경로**, **네임스페이스**, **그래프 경로**.
- `clock=(k == 0)`: `/clock` 은 첫 로봇의 그래프만 내보냅니다.

### 시작 위치

| 로봇 | 도시 좌표 (x, y) | 방향 | 자리 |
|---|---|---|---|
| pinky1 | (−1.05, −0.5) | 90° (+Y) | 왼쪽 도로. 강의 16 ~ 18 과 같은 자리 |
| pinky2 | (1.05, 0.5) | −90° (−Y) | 오른쪽 도로. 맞은편 |

- 표의 줄을 늘리면 세 대, 네 대도 됩니다. 이 강의에서는 두 대만 확인했습니다.

## 3. 두 대 올리기

### 터미널 1: 핑크시티 + Pinky Pro 두 대

```bash
source /opt/ros/jazzy/setup.bash
cd ~/nvidia-dli-lecture
~/isaacsim/python.sh 35_two_pinkys/two_pinkys.py --gui
```

- 앞에서 `--no-namespace` 로 띄운 것은 Ctrl+C 로 끄고 다시 실행합니다.

### 실행 결과

```text
pinky1: Prim /World/pinky1, 네임스페이스 'pinky1', 시작 (-1.05, -0.5) 방향 90.0°
pinky2: Prim /World/pinky2, 네임스페이스 'pinky2', 시작 (1.05, 0.5) 방향 -90.0°
   시간[s]  pinky1 x       y      방향  pinky2 x       y      방향
       5     -1.05   -0.50      90      1.05    0.50     -90
```

- 5초마다 두 로봇의 실제 위치(도시 좌표)가 찍힙니다.

### Isaac Sim 화면

- 왼쪽 도로에 pinky1, 오른쪽 도로에 pinky2 가 서 있습니다.
- Stage 창에는 `/World/pinky1`, `/World/pinky2` 가 따로 보입니다.

### 터미널 2: 토픽 목록

```text
$ ros2 topic list
/clock
/pinky1/camera/image_raw
/pinky1/cmd_vel
/pinky1/imu
/pinky1/joint_states
/pinky1/odom
/pinky1/scan
/pinky1/tf
/pinky2/camera/image_raw
/pinky2/cmd_vel
...
```

- 로봇마다 같은 토픽 한 벌이 생겼습니다. `/clock` 만 하나입니다.

### 한 대만 움직이기

```bash
ros2 topic pub -r 10 /pinky2/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"
```

| | 명령 전 (x, y) | 5초 뒤 |
|---|---|---|
| pinky1 | (−1.05, −0.50) | (−1.05, −0.50) |
| pinky2 | (1.05, 0.50) | (1.05, −0.01) |

- pinky2 만 움직였습니다. 확인했으면 Ctrl+C 로 멈추고, 시뮬레이터를 껐다 켜서 처음 자리로 되돌립니다.

## 4. Nav2 를 로봇마다 한 벌

### 무엇을 두 번 띄우나

| 로봇마다 띄우는 것 | 하는 일 | 강의 |
|---|---|---|
| robot_state_publisher | `/pinky1/joint_states` → 로봇 몸의 tf | 15 |
| map_server · amcl | 지도 내보내기, 위치 추정 | 18 |
| planner · controller · bt_navigator … | 경로 계획, 주행 | 18 |
| RViz2 | 로봇마다 창 하나 | 18 |

- 전부 같은 네임스페이스(`pinky1`, `pinky2`)로 띄웁니다. 지도 파일은 둘이 같은 것을 씁니다.
- 실행 파일 하나(`35_two_pinkys/two_pinky_nav2.launch.py`)가 이 표를 두 번 반복합니다.

### 설정은 강의 18 것을 그대로, 세 가지만 바꾼다

```python
text = open(PARAMS).read()                                    # 18_pinky_nav2/nav2_params.yaml
text = text.replace("topic: /scan", f"topic: /{name}/scan")
text = text.replace("initial_pose: [0, 0, 0]",
                    f"initial_pose: {{x: {mx:.3f}, y: {my:.3f}, z: 0.0, yaw: {myaw:.4f}}}")
text = f"{name}:\n" + "".join("  " + line for line in text.splitlines(keepends=True))
```

- **라이다 토픽**: 비용 지도가 `/pinky1/scan` 을 읽게 합니다.
- **처음 위치**: AMCL 에게 로봇이 지도의 어디에서 시작하는지 알려 줍니다.
- **맨 위에 로봇 이름 한 겹**: `amcl:` 은 `/amcl` 의 설정입니다. `/pinky1/amcl` 은 `pinky1:` 아래의 `amcl:` 을 읽습니다.

### 처음 위치: 도시 좌표 → 지도 좌표

| 로봇 | 도시 좌표 (x, y, 방향) | 지도 좌표 (x, y, 방향) |
|---|---|---|
| pinky1 | (−1.05, −0.5, 90°) | (0, 0, 0°) |
| pinky2 | (1.05, 0.5, −90°) | (1.0, −2.1, 180°) |

- 지도는 강의 17 에서 pinky1 의 자리를 원점으로 만들었습니다. 그래서 pinky1 은 (0, 0) 입니다.
- 바꾸는 식: 지도 (x, y) = (도시 Y + 0.5, −도시 X − 1.05), 방향은 90° 를 뺍니다.

### 로봇 한 대씩, 준비 시작은 조금 늦게

```python
STAGGER = 15.0        # 로봇 한 대의 Nav2 가 다 뜰 때까지 기다리는 시간 [s]
START_DELAY = 8.0     # 노드를 띄운 뒤 준비 시작을 보낼 때까지 기다리는 시간 [s]
RVIZ_DELAY = 10.0     # 준비 시작을 보낸 뒤 RViz2 를 띄울 때까지 [s]
```

- pinky1 의 노드를 띄우고 → 8초 뒤 "준비 시작" → 15초 뒤 pinky2 를 같은 순서로 띄웁니다.
- Nav2 는 보통 노드가 뜨자마자 준비를 시작합니다(autostart). 이 강의에서는 그것을 끄고 따로 보냅니다. 이유는 다음 장입니다.

### 왜 늦게 보내나: 강의 PC 에서 잰 결과

| 준비 시작을 보내는 때 | 시험 횟수 | 두 대 모두 준비됨 |
|---|---|---|
| 노드가 뜨자마자 (기본) | 4 | 0 |
| 8초 뒤 | 10 | 10 |

- 기본 방식에서는 `Failed to change state for node: …` 가 나오며 Nav2 가 준비를 멈췄습니다.
- 로그를 보면 노드들이 서로를 찾기 전에 준비 요청이 오가면서 응답이 유실되거나 거절됩니다. 정확히 왜 그런지까지는 확인하지 못했습니다.
- 한 대일 때(강의 18, Nav2 를 한 프로세스에 묶어 띄움)는 보지 못한 문제입니다. 이번에는 로봇당 프로세스 12개를 따로 띄웁니다.

## 5. Nav2 실행

### 터미널 2: Nav2 두 벌 + RViz2 두 창

```bash
source /opt/ros/jazzy/setup.bash
source ~/pinky_pro/install/setup.bash
ros2 launch ~/nvidia-dli-lecture/35_two_pinkys/two_pinky_nav2.launch.py
```

- 터미널 1 의 시뮬레이터가 켜진 상태에서 실행합니다.

### 준비 확인: Managed nodes are active 네 번

```text
[pinky1.lifecycle_manager_localization]: Managed nodes are active
[pinky1.lifecycle_manager_navigation]: Managed nodes are active
[pinky2.lifecycle_manager_localization]: Managed nodes are active
[pinky2.lifecycle_manager_navigation]: Managed nodes are active
```

- 로봇 두 대 × (위치 추정 · 주행) = 네 번입니다. 강의 PC 에서는 실행 후 31 ~ 34초에 네 번째가 나왔습니다.
- 네 번이 다 나오지 않으면 Ctrl+C 로 끄고 다시 실행합니다.

### RViz2 두 창

- 창마다 로봇 한 대의 세계입니다. 왼쪽은 pinky1, 오른쪽은 pinky2 가 자기 위치를 지도의 다른 곳에 그리고 있습니다.
- 두 창 모두 Navigation · Localization 이 `active` 여야 합니다.
- 창을 나눈 이유: RViz2 한 개는 `/tf` 를 하나만 읽습니다. tf 를 로봇마다 나눴으니 창도 로봇마다 하나입니다.

## 6. 동시에 목적지 주기

### 터미널 3: 서로의 자리로 보내기

```bash
source /opt/ros/jazzy/setup.bash
ros2 action send_goal /pinky1/navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: -2.1}, orientation: {w: 1.0}}}}" &
ros2 action send_goal /pinky2/navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: 0.0, y: 0.0}, orientation: {w: 1.0}}}}"
```

- 액션 이름에도 네임스페이스가 붙습니다: `/pinky1/navigate_to_pose`.
- 첫 줄 끝의 `&` 가 두 명령을 동시에 보냅니다. pinky1 은 pinky2 의 자리로, pinky2 는 pinky1 의 자리로 갑니다.
- RViz2 에서 하려면 각 창의 `Nav2 Goal` 로 찍습니다. 그 창의 로봇만 움직입니다.

### 결과

```text
Goal finished with status: SUCCEEDED
Goal finished with status: SUCCEEDED
```

| | 걸린 시간 (세 번) | 도착 위치와 목표의 거리 |
|---|---|---|
| pinky1 | 21 · 22 · 24초 | 7 cm |
| pinky2 | 25 · 27 · 30초 | 4 cm |

- 세 번 모두 두 대가 도착했습니다. 거리는 마지막 회의 값이고, 도착 허용 오차는 8 cm 입니다.

### 움직이는 모습

### 사거리에서 스쳐 지나가는 순간

- 두 로봇 모두 가장 짧은 길인 가운데 도로를 골랐습니다. 5초 간격 기록에서 가장 가까웠을 때 둘 사이는 약 24 cm 였습니다.
- 부딪히지 않은 것은 각자의 라이다에 상대가 잡혔기 때문입니다. 서로 길을 조율한 것이 아닙니다.

### 서로를 장애물로 본다

- 가운데 그림(pinky1 의 RViz2)에서 오른쪽 위 경로가 동그란 덩어리를 돌아갑니다. 출발할 때 그 자리에 서 있던 pinky2 입니다.
- 지도에는 없는 물체이므로, pinky1 의 비용 지도는 라이다로 본 pinky2 를 "지금 생긴 장애물" 로 그립니다.
- 그림에서 확인한 것은 pinky1 의 창입니다. pinky2 쪽은 따로 확대해 보지 않았습니다.

### 도착

## 7. 공식 실행 파일을 그대로 못 쓴 곳

### 네임스페이스를 주면 달라지는 세 가지

| 한 대일 때 (강의 18) | 두 대일 때 생긴 일 | 이 강의의 처리 |
|---|---|---|
| `bringup_launch.xml` 한 줄 | `namespace` 를 주면 노드를 `/nav2_container` 에 실으려다 멈춤 | `localization_launch.xml` · `navigation_launch.xml` 을 직접 부르고, 노드를 따로따로 띄움 |
| 설정 파일을 그대로 줌 | `/pinky1/amcl` 이 설정을 못 읽음 | 설정 맨 위에 로봇 이름 한 겹 |
| `initial_pose: [0, 0, 0]` | 이 형식은 무시되고 늘 (0, 0) 이 됨. pinky2 가 자기 위치를 틀리게 앎 | `{x, y, z, yaw}` 형식으로 |

- 한 대일 때는 처음 위치가 원래 (0, 0) 이라 드러나지 않던 문제입니다.
- 실행 파일과 설정 값 자체는 Pinky Pro [공식 저장소](https://github.com/pinklab-art/pinky_pro)의 것을 씁니다.

### 실행 파일을 두 번 부를 때의 주의

```python
actions.append(GroupAction([IncludeLaunchDescription(
    AnyLaunchDescriptionSource(os.path.join(NAVIGATION, "localization_launch.xml")),
    launch_arguments={**common, "map": LaunchConfiguration("map"), "lifecycle_nodes": LOCAL_NODES}.items())]))
```

- `GroupAction` 으로 감싸지 않으면 앞에서 준 인자가 다음 것에 남습니다.
- 처음에는 주행용 노드 목록이 위치 추정 쪽 관리자에게 넘어가, 엉뚱한 노드를 켜려다 실패했습니다.

## 8. 해 볼 것

### 직접 바꿔 보기

- RViz2 의 `Nav2 Goal` 로 두 로봇에게 서로 다른 목적지를 차례로 찍어 보기.
- 한 로봇을 좁은 길 가운데 세워 두고, 다른 로봇을 그 너머로 보내 보기. 돌아가는지, 멈추는지.
- `ROBOTS` 표(두 파일 모두)에 `("pinky3", 0.0, -1.05, 0.0)` 을 더해 세 대로 늘려 보기.
- `rqt_image_view` 로 `/pinky1/camera/image_raw` 와 `/pinky2/camera/image_raw` 를 나란히 보기.

## 9. 정리

### 기억할 것

- 같은 로봇을 여러 대 쓰려면 토픽 · tf · 액션 이름을 로봇마다 나눠야 합니다. 그 방법이 네임스페이스입니다.
- Isaac Sim 쪽은 ROS 2 노드의 **Node Namespace** 칸, ROS 2 쪽은 노드를 띄울 때의 `namespace` 입니다.
- Nav2 는 로봇마다 한 벌씩 띄웁니다. 설정에서 로봇마다 다른 것은 라이다 토픽과 처음 위치입니다.
- 두 로봇은 서로를 라이다로 보고 피할 뿐, 길을 조율하지는 않습니다.

### 한계

- 두 대, 한 가지 경로(자리 맞바꾸기)만 확인했습니다. 좁은 길에서 정면으로 마주치는 경우는 시험하지 않았습니다.
- Nav2 준비 실패의 원인은 "뜨자마자 준비 요청을 보내는 것" 까지만 좁혔습니다. 8초 · 15초는 강의 PC 에서 통한 값이고, 더 느린 PC 에서는 늘려야 할 수 있습니다.
- 세 대 이상은 해 보지 않았습니다.
