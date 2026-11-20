# 35. 핑크시티에 Pinky Pro 두 대: 네임스페이스와 동시 자율주행 — 명령 모음

- 영상에서 입력하는 터미널 명령과 기대 출력을 순서대로 모았음. [34. 사족 로봇을 걷게 하기](../34_quadruped/README.md) 다음 내용임.

| 파일 |
|---|
| [start_nav2.py](start_nav2.py) |
| [two_pinky_nav2.launch.py](two_pinky_nav2.launch.py) |
| [two_pinkys.py](two_pinkys.py) |

## 1. 로봇이 두 대가 되면

### 이번 강의에서 할 일

- 강의 16 ~ 18 의 핑크시티에 Pinky Pro 를 **두 대** 올림.
- 두 대가 각자의 Nav2 로 **동시에** 목적지까지 감.
- 그러려면 두 로봇의 토픽이 섞이지 않게 나눠야 함. 그 방법이 **네임스페이스**임.

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

- `--no-namespace`: 두 로봇이 강의 14 와 똑같은 토픽 이름을 그대로 씀. 일부러 겹치게 해 보는 것임.

### 결과: 토픽 하나를 두 로봇이 함께 씀

```text
$ ros2 topic info /scan
Publisher count: 2
$ ros2 topic info /cmd_vel
Subscription count: 2
```

- `/scan` 을 내보내는 쪽이 둘임. 받는 쪽은 어느 로봇의 라이다인지 알 수 없음. 주기도 10 Hz 가 아니라 약 20 Hz 로 보임.
- `/cmd_vel` 을 듣는 쪽도 둘임.

### /cmd_vel 한 번에 두 대가 함께 움직임

| | 명령 전 (x, y) | `/cmd_vel` 로 0.1 m/s 를 5초 보낸 뒤 |
|---|---|---|
| pinky1 | (−1.05, −0.50) | (−1.05, 0.01) |
| pinky2 | (1.05, 0.50) | (1.05, −0.01) |

- 두 대가 각자 앞으로 0.5 m 씩 갔음. 한 대만 움직일 방법이 없음.
- `/odom` 과 `/tf` 도 내보내는 쪽이 둘임. 어느 로봇의 위치인지 가릴 수 없어, 이 상태로는 Nav2 를 쓸 수 없음.

## 2. 네임스페이스

### 토픽 이름 앞에 로봇 이름을 붙임

- `/cmd_vel` → `/pinky1/cmd_vel`, `/pinky2/cmd_vel`. 이름이 다르니 섞이지 않음.
- `/tf` 도 로봇마다 따로 둠. 그래서 두 로봇이 `odom`, `base_footprint` 같은 프레임 이름을 똑같이 써도 됨.
- `/clock` 만은 공통임. 시뮬레이션의 시계는 하나이기 때문임.

### Isaac Sim 쪽: Node Namespace 칸

```python
def _namespaced(nodes, namespace):
    return [(f"{node}.inputs:nodeNamespace", namespace) for node in nodes] if namespace else []
```

- Isaac Sim 의 ROS 2 노드(Subscribe Twist, Publish Odometry …)에는 모두 **Node Namespace** 칸이 있음.
- 여기에 `pinky1` 을 넣으면 그 노드의 토픽 `cmd_vel` 이 `/pinky1/cmd_vel` 이 됨.
- 강의 14 의 그래프 함수에 이 한 줄을 더해, 이름을 인자로 받게 했음(`14_pinky_ros2/pinky_ros2.py`).

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

- `35_two_pinkys/two_pinkys.py` 의 핵심임(일부만 보였음). 강의 11 · 12 · 14 의 함수를 그대로 부름.
- 로봇마다 다른 것은 세 가지뿐임: **Prim 경로**, **네임스페이스**, **그래프 경로**.
- `clock=(k == 0)`: `/clock` 은 첫 로봇의 그래프만 내보냄.

### 시작 위치

| 로봇 | 도시 좌표 (x, y) | 방향 | 자리 |
|---|---|---|---|
| pinky1 | (−1.05, −0.5) | 90° (+Y) | 왼쪽 도로. 강의 16 ~ 18 과 같은 자리 |
| pinky2 | (1.05, 0.5) | −90° (−Y) | 오른쪽 도로. 맞은편 |

- 표의 줄을 늘리면 세 대, 네 대도 됨. 이 강의에서는 두 대만 확인했음.

## 3. 두 대 올리기

### 터미널 1: 핑크시티 + Pinky Pro 두 대

```bash
source /opt/ros/jazzy/setup.bash
cd ~/nvidia-dli-lecture
~/isaacsim/python.sh 35_two_pinkys/two_pinkys.py --gui
```

- 앞에서 `--no-namespace` 로 띄운 것은 Ctrl+C 로 끄고 다시 실행함.

### 실행 결과

```text
pinky1: Prim /World/pinky1, 네임스페이스 'pinky1', 시작 (-1.05, -0.5) 방향 90.0°
pinky2: Prim /World/pinky2, 네임스페이스 'pinky2', 시작 (1.05, 0.5) 방향 -90.0°
   시간[s]  pinky1 x       y      방향  pinky2 x       y      방향
       5     -1.05   -0.50      90      1.05    0.50     -90
```

- 5초마다 두 로봇의 실제 위치(도시 좌표)가 찍힘.

### Isaac Sim 화면

- 왼쪽 도로에 pinky1, 오른쪽 도로에 pinky2 가 서 있음.
- Stage 창에는 `/World/pinky1`, `/World/pinky2` 가 따로 보임.

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

- 로봇마다 같은 토픽 한 벌이 생겼음. `/clock` 만 하나임.

### 한 대만 움직이기

```bash
ros2 topic pub -r 10 /pinky2/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"
```

| | 명령 전 (x, y) | 5초 뒤 |
|---|---|---|
| pinky1 | (−1.05, −0.50) | (−1.05, −0.50) |
| pinky2 | (1.05, 0.50) | (1.05, −0.01) |

- pinky2 만 움직였음. 확인했으면 Ctrl+C 로 멈추고, 시뮬레이터를 껐다 켜서 처음 자리로 되돌림.

## 4. Nav2 를 로봇마다 한 벌

### 무엇을 두 번 띄우나

| 로봇마다 띄우는 것 | 하는 일 | 강의 |
|---|---|---|
| robot_state_publisher | `/pinky1/joint_states` → 로봇 몸의 tf | 15 |
| map_server · amcl | 지도 내보내기, 위치 추정 | 18 |
| planner · controller · bt_navigator … | 경로 계획, 주행 | 18 |
| RViz2 | 로봇마다 창 하나 | 18 |

- 전부 같은 네임스페이스(`pinky1`, `pinky2`)로 띄움. 지도 파일은 둘이 같은 것을 씀.
- 실행 파일 하나(`35_two_pinkys/two_pinky_nav2.launch.py`)가 이 표를 두 번 반복하고, 끝으로 `start_nav2.py` 를 실행함.

### 설정은 강의 18 것을 그대로, 세 가지만 바꿈

```python
text = open(PARAMS).read()                                    # 18_pinky_nav2/nav2_params.yaml
text = text.replace("topic: /scan", f"topic: /{name}/scan")
text = text.replace("initial_pose: [0, 0, 0]",
                    f"initial_pose: {{x: {mx:.3f}, y: {my:.3f}, z: 0.0, yaw: {myaw:.4f}}}")
text = f"{name}:\n" + "".join("  " + line for line in text.splitlines(keepends=True))
```

- **라이다 토픽**: 비용 지도가 `/pinky1/scan` 을 읽게 함.
- **처음 위치**: AMCL 에게 로봇이 지도의 어디에서 시작하는지 알려 줌.
- **맨 위에 로봇 이름 한 겹**: `amcl:` 은 `/amcl` 의 설정임. `/pinky1/amcl` 은 `pinky1:` 아래의 `amcl:` 을 읽음.

### 처음 위치: 도시 좌표 → 지도 좌표

| 로봇 | 도시 좌표 (x, y, 방향) | 지도 좌표 (x, y, 방향) |
|---|---|---|
| pinky1 | (−1.05, −0.5, 90°) | (0, 0, 0°) |
| pinky2 | (1.05, 0.5, −90°) | (1.0, −2.1, 180°) |

- 지도는 강의 17 에서 pinky1 의 자리를 원점으로 만들었음. 그래서 pinky1 은 (0, 0) 임.
- 바꾸는 식: 지도 (x, y) = (도시 Y + 0.5, −도시 X − 1.05), 방향은 90° 를 뺌.

### Nav2 는 "준비 시작" 을 받아야 켜짐

- Nav2 의 노드들은 뜨기만 해서는 일하지 않음. 관리자(lifecycle_manager)가 노드마다 "설정해라 → 켜라" 를 차례로 보내야 함.
- 보통은 관리자가 뜨자마자 스스로 이것을 시작함(autostart).
- 두 대를 띄운 강의 PC 에서는 이 방식이 네 번 모두 실패했음. `Failed to change state for node: …` 가 나오며 멈춤.

### 로그에서 본 것: 너무 일찍 보냄

```text
[map_server]: failed to send response to /pinky2/map_server/change_state (timeout): client will not receive response
```

- 관리자는 뜬 지 0.1 ~ 1초 만에 첫 요청을 보냄. 그때는 노드들이 아직 서로를 다 찾지 못했음.
- 위 로그는 지도 서버가 "응답을 받을 상대를 못 찾아 응답을 버렸다" 는 뜻임. 관리자는 그 응답을 계속 기다림.
- 왜 이렇게 되는지 그 아래 원인까지는 확인하지 못했음.

### 해결: 받을 준비가 됐는지 확인하고 보냄

```python
def connected(node, robot, group):
    if node.count_services(f"/{robot}/lifecycle_manager_{group}/manage_nodes") < 1:
        return False
    for name in GROUPS[group]:
        for service in ("change_state", "get_state"):
            path = f"/{robot}/{name}/{service}"
            if node.count_services(path) < 1 or node.count_clients(path) < 1:
                return False
    return True
```

- `35_two_pinkys/start_nav2.py` 의 일부임. autostart 를 끄고, 이 스크립트가 대신 시작을 보냄.
- 서비스마다 **서버 1개(노드)와 클라이언트 1개(관리자)** 가 보이는지 셈. 둘 다 보이면 서로 연결된 것임.
- 정해 둔 시간을 기다리는 것이 아니라, 연결이 실제로 보일 때까지만 기다림.

### 보낸 뒤에도 확인함

```python
call(node, manage, ManageLifecycleNodes.Request(command=STARTUP))
answer = call(node, is_active, Trigger.Request())
if answer is not None and answer.success:
    return True
call(node, manage, ManageLifecycleNodes.Request(command=RESET))    # 아니면 초기화하고 다시 (최대 3번)
```

- 관리자의 `is_active` 서비스는 "맡은 노드가 모두 켜졌는가" 에 답함.
- 켜지지 않았으면 초기화하고 다시 시작함. 실행 파일은 이 스크립트가 끝난 뒤에 RViz2 를 띄움.

### 강의 PC 에서 잰 결과

| 준비 시작을 보내는 방법 | 시험 횟수 | 두 대 모두 준비됨 | 준비까지 |
|---|---|---|---|
| 뜨자마자 (기본 autostart) | 4 | 0 | — |
| 정해 둔 시간(8초 · 15초) 뒤 | 10 | 10 | 31 ~ 34초 |
| **연결을 확인한 뒤** (이 강의) | 7 | 7 | 12초 |

- 연결 확인 방식은 7번 모두 첫 시도에 켜졌음. 다시 시도하는 경로는 한 번도 쓰이지 않았음.
- 연결을 기다린 시간은 0.4 ~ 1.8초였음. 8초를 기다릴 필요가 없었던 셈임.

## 5. Nav2 실행

### 터미널 2: Nav2 두 벌 + RViz2 두 창

```bash
source /opt/ros/jazzy/setup.bash
source ~/pinky_pro/install/setup.bash
ros2 launch ~/nvidia-dli-lecture/35_two_pinkys/two_pinky_nav2.launch.py
```

- 터미널 1 의 시뮬레이터가 켜진 상태에서 실행함.

### 준비 확인: Managed nodes are active 네 번

```text
[pinky1.lifecycle_manager_localization]: Managed nodes are active
[pinky1.lifecycle_manager_navigation]: Managed nodes are active
[pinky2.lifecycle_manager_localization]: Managed nodes are active
[pinky2.lifecycle_manager_navigation]: Managed nodes are active
```

- 로봇 두 대 × (위치 추정 · 주행) = 네 번임. 강의 PC 에서는 실행 후 약 12초에 네 번째가 나왔음.
- 이어서 `Nav2 준비 완료` 가 나오고 RViz2 창 두 개가 뜸. `Nav2 준비 실패` 가 나오면 Ctrl+C 로 끄고 다시 실행함.

### RViz2 두 창

- 창마다 로봇 한 대의 세계임. 이 그림에서는 왼쪽이 pinky1, 오른쪽이 pinky2 임. 창이 뜨는 순서는 바뀔 수 있으니, 지도 위 로봇의 위치로 구분함.
- 두 창 모두 Navigation · Localization 이 `active` 여야 함.
- 창을 나눈 이유: RViz2 한 개는 `/tf` 를 하나만 읽음. tf 를 로봇마다 나눴으니 창도 로봇마다 하나임.

## 6. 동시에 목적지 주기

### 터미널 3: 서로의 자리로 보내기

```bash
source /opt/ros/jazzy/setup.bash
ros2 action send_goal /pinky1/navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: -2.1}, orientation: {w: 1.0}}}}" &
ros2 action send_goal /pinky2/navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: 0.0, y: 0.0}, orientation: {w: 1.0}}}}"
```

- 액션 이름에도 네임스페이스가 붙음: `/pinky1/navigate_to_pose`.
- 첫 줄 끝의 `&` 가 두 명령을 동시에 보냄. pinky1 은 pinky2 의 자리로, pinky2 는 pinky1 의 자리로 감.
- RViz2 에서 하려면 각 창의 `Nav2 Goal` 로 찍음. 그 창의 로봇만 움직임.

### 결과

```text
Goal finished with status: SUCCEEDED
Goal finished with status: SUCCEEDED
```

| | 걸린 시간 (네 번) | 도착 위치와 목표의 거리 |
|---|---|---|
| pinky1 | 21 · 22 · 23 · 24초 | 7 cm |
| pinky2 | 25 · 27 · 30 · 33초 | 4 cm |

- 네 번 모두 두 대가 도착했음. 거리는 그중 한 회의 값이고, 도착 허용 오차는 8 cm 임.

### 움직이는 모습

### 사거리에서 스쳐 지나가는 순간

- 두 로봇 모두 가장 짧은 길인 가운데 도로를 골랐음. 5초 간격 기록에서 가장 가까웠을 때 둘 사이는 약 24 cm 였음.
- 부딪히지 않은 것은 각자의 라이다에 상대가 잡혔기 때문임. 서로 길을 조율한 것이 아님.

### 서로를 장애물로 봄

- 가운데 그림(pinky1 의 RViz2)에서 오른쪽 위 경로가 동그란 덩어리를 돌아감. 출발할 때 그 자리에 서 있던 pinky2 임.
- 지도에는 없는 물체이므로, pinky1 의 비용 지도는 라이다로 본 pinky2 를 "지금 생긴 장애물" 로 그림.
- 그림에서 확인한 것은 pinky1 의 창임. pinky2 쪽은 따로 확대해 보지 않았음.

### 도착

## 7. 공식 실행 파일을 그대로 못 쓴 곳

### 네임스페이스를 주면 달라지는 세 가지

| 한 대일 때 (강의 18) | 두 대일 때 생긴 일 | 이 강의의 처리 |
|---|---|---|
| `bringup_launch.xml` 한 줄 | `namespace` 를 주면 노드를 `/nav2_container` 에 실으려다 멈춤 | `localization_launch.xml` · `navigation_launch.xml` 을 직접 부르고, 노드를 따로따로 띄움 |
| 설정 파일을 그대로 줌 | `/pinky1/amcl` 이 설정을 못 읽음 | 설정 맨 위에 로봇 이름 한 겹 |
| `initial_pose: [0, 0, 0]` | 이 형식은 무시되고 늘 (0, 0) 이 됨. pinky2 가 자기 위치를 틀리게 앎 | `{x, y, z, yaw}` 형식으로 |

- 한 대일 때는 처음 위치가 원래 (0, 0) 이라 드러나지 않던 문제임.
- 실행 파일과 설정 값 자체는 Pinky Pro [공식 저장소](https://github.com/pinklab-art/pinky_pro)의 것을 씀.

### 실행 파일을 두 번 부를 때의 주의

```python
ld.add_action(GroupAction([IncludeLaunchDescription(
    AnyLaunchDescriptionSource(os.path.join(NAVIGATION, "localization_launch.xml")),
    launch_arguments={**common, "map": LaunchConfiguration("map"), "lifecycle_nodes": LOCAL_NODES}.items())]))
```

- `GroupAction` 으로 감싸지 않으면 앞에서 준 인자가 다음 것에 남음.
- 처음에는 주행용 노드 목록이 위치 추정 쪽 관리자에게 넘어가, 엉뚱한 노드를 켜려다 실패했음.

## 8. 해 볼 것

### 직접 바꿔 보기

- RViz2 의 `Nav2 Goal` 로 두 로봇에게 서로 다른 목적지를 차례로 찍어 보기.
- 한 로봇을 좁은 길 가운데 세워 두고, 다른 로봇을 그 너머로 보내 보기. 돌아가는지, 멈추는지.
- `ROBOTS` 표(두 파일 모두)에 `("pinky3", 0.0, -1.05, 0.0)` 을 더해 세 대로 늘려 보기.
- `rqt_image_view` 로 `/pinky1/camera/image_raw` 와 `/pinky2/camera/image_raw` 를 나란히 보기.

## 9. 정리

### 기억할 것

- 같은 로봇을 여러 대 쓰려면 토픽 · tf · 액션 이름을 로봇마다 나눠야 함. 그 방법이 네임스페이스임.
- Isaac Sim 쪽은 ROS 2 노드의 **Node Namespace** 칸, ROS 2 쪽은 노드를 띄울 때의 `namespace` 임.
- Nav2 는 로봇마다 한 벌씩 띄움. 설정에서 로봇마다 다른 것은 라이다 토픽과 처음 위치임.
- 노드가 많을 때는 "얼마 기다렸다 보내기" 보다 "받을 준비가 됐는지 확인하고 보내기" 가 빠르고 확실함.
- 두 로봇은 서로를 라이다로 보고 피할 뿐, 길을 조율하지는 않음.

### 한계

- 두 대, 한 가지 경로(자리 맞바꾸기)만 확인했음. 좁은 길에서 정면으로 마주치는 경우는 시험하지 않았음.
- Nav2 준비 실패의 원인은 "뜨자마자 준비 요청을 보내는 것" 까지만 좁혔음. 연결 확인 방식은 강의 PC 한 대에서 7번 시험한 결과임.
- 세 대 이상은 해 보지 않았음.
