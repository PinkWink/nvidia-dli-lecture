# 14. Pinky Pro 를 ROS 2 로 움직이기: /cmd_vel 구독 · teleop — 명령 모음

영상에서 입력하는 터미널 명령과 화면에 넣는 값을 순서대로 모았습니다. [13. Pinky Pro 화면과 LED](../13_pinky_lcd_led/README.md) 다음 내용입니다.

강의 7 에서는 Isaac Sim 이 ROS 2 로 **내보내기만** 했습니다. 이번에는 ROS 2 의 속도 명령 `/cmd_vel` 을 **받아서** 로봇을 움직입니다. 실물 Pinky Pro 를 움직이는 것과 같은 토픽입니다 ([공식 저장소](https://github.com/pinklab-art/pinky_pro)).

## 1. 준비

키보드 조종 프로그램을 설치합니다 (한 번만).

```bash
sudo apt install -y ros-jazzy-teleop-twist-keyboard
```

`source` 를 Isaac Sim 보다 먼저 합니다.

```bash
source /opt/ros/jazzy/setup.bash
cd ~/isaacsim
./isaac-sim.sh
```

- `File > Open` → `~/nvidia-dli-lecture/work/pinky_gui.usd` (강의 12 · 13 에서 저장한 장면)

## 2. 화면에서 누르는 순서

### 준비: 키보드 그래프 지우기 · 물리 주기

| 순서 | 동작 | 값 |
|---|---|---|
| 1 | Stage 의 `Graphs > differential_controller` 우클릭 → `Delete` | 강의 11 의 키보드 그래프를 지움 |
| 2 | `physicsScene` 선택 → Property 의 Time Steps Per Second | `240` |

### 그래프 만들기

| 순서 | 동작 | 값 |
|---|---|---|
| 3 | `pinky` 우클릭 → `Create > Visual Scripting > Action Graph` | 아래에 Action Graph 창이 열림 (창을 크게) |
| 4 | 왼쪽 검색칸에 이름을 넣고 노드를 캔버스로 끌어 놓기 | 아래 노드 10개 |
| 5 | `Make Array` 노드 선택 → Property 의 `+` (Add New Input) | input1 이 생김 |
| 6 | 노드 사이를 선으로 잇기 | 아래 연결 표 |

노드 10개 (검색어 → 노드 이름):

| 검색어 | 노드 | 개수 |
|---|---|---|
| `Playback Tick` | On Playback Tick | 1 |
| `ROS2 Context` | ROS2 Context | 1 |
| `Subscribe Twist` | ROS2 Subscribe Twist | 1 |
| `Break 3-Vector` | Break 3-Vector | 2 |
| `Differential Controller` | Differential Controller | 1 |
| `Constant Token` | Constant Token | 2 |
| `Make Array` | Make Array | 1 |
| `Articulation Controller` | Articulation Controller | 1 |

연결 (나가는 점을 끌어 들어가는 점에 놓습니다):

| 나가는 곳 (왼쪽 노드의 오른쪽 점) | 들어가는 곳 (오른쪽 노드의 왼쪽 점) |
|---|---|
| On Playback Tick › Tick | ROS2 Subscribe Twist › Exec In |
| On Playback Tick › Tick | Differential Controller › Exec In |
| On Playback Tick › Tick | Articulation Controller › Exec In |
| ROS2 Context › Context | ROS2 Subscribe Twist › Context |
| ROS2 Subscribe Twist › Linear Velocity | Break 3-Vector (위) › Vector |
| ROS2 Subscribe Twist › Angular Velocity | Break 3-Vector (아래) › Vector |
| Break 3-Vector (위) › X | Differential Controller › Linear Velocity |
| Break 3-Vector (아래) › Z | Differential Controller › Angular Velocity |
| Differential Controller › Velocity Command | Articulation Controller › Velocity Command |
| Constant Token (위) › Value | Make Array › Input0 |
| Constant Token (아래) › Value | Make Array › Input1 |
| Make Array › Array | Articulation Controller › Joint Names |

### 노드 값

| 순서 | 노드 | 속성 | 값 |
|---|---|---|---|
| 7 | Differential Controller | wheelRadius | `0.028` |
| 8 | Differential Controller | wheelDistance | `0.0961` |
| 9 | Differential Controller | maxLinearSpeed | `0.2` |
| 10 | Differential Controller | maxAngularSpeed | `2.0` |
| 11 | Articulation Controller | targetPrim → `Add Target...` → `pinky` → Select | `/World/pinky` |
| 12 | Constant Token (위) | Value | `l_wheel_joint` |
| 13 | Constant Token (아래) | Value | `r_wheel_joint` |
| 14 | ROS2 Subscribe Twist | topicName | `cmd_vel` (기본값 그대로) |

`Ctrl` + `S` 로 저장한 뒤 ▶ Play.

## 3. ROS 2 로 움직이기

Isaac Sim 이 ▶ Play 중일 때, 새 터미널에서 합니다.

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic list
```

`/cmd_vel` 이 보이면 Isaac Sim 이 듣고 있는 것입니다.

앞으로 (0.1 m/s). `Ctrl` + `C` 로 보내기를 멈춥니다.

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"
```

정지 (0 을 보내야 멈춥니다).

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"
```

제자리에서 왼쪽으로 (1 rad/s).

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 1.0}}"
```

키보드로 조종.

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

| 키 | 동작 |
|---|---|
| `i` | 앞으로 |
| `,` | 뒤로 |
| `j` · `l` | 왼쪽 · 오른쪽으로 돌기 |
| `k` | 정지 |

키는 teleop 을 실행한 터미널을 클릭한 상태에서 누릅니다. 기본 속도 0.5 m/s 는 0.2 m/s 로 잘립니다.

## 4. 같은 일을 코드로

Isaac Sim 창을 닫은 뒤, 터미널 1:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/nvidia-dli-lecture
git pull
~/isaacsim/python.sh 14_pinky_ros2/pinky_ros2.py
```

```text
ROS 2 연결: /cmd_vel 받기, /odom · /tf · /joint_states · /clock · /scan · /camera/image_raw · /imu 내보내기
ROS_DOMAIN_ID = 0. 끝내려면 Ctrl+C 를 누릅니다.
   시간[s]     x[m]     y[m]    방향[°]
       1    0.000    0.000      0.1
       2    0.000    0.000      0.1
       3    0.000    0.000      0.2
```

1초마다 로봇의 위치가 한 줄씩 나옵니다. 터미널 2 에서 3절의 명령을 보내면 숫자가 바뀝니다. 끝낼 때는 `Ctrl` + `C`.

터미널 2:

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic list -t
```

```text
/camera/image_raw [sensor_msgs/msg/Image]
/clock [rosgraph_msgs/msg/Clock]
/cmd_vel [geometry_msgs/msg/Twist]
/imu [sensor_msgs/msg/Imu]
/joint_states [sensor_msgs/msg/JointState]
/odom [nav_msgs/msg/Odometry]
/scan [sensor_msgs/msg/LaserScan]
/tf [tf2_msgs/msg/TFMessage]
```

강의 PC 측정: `x: 0.1` 을 5초 보내면 약 0.4 m 전진, `z: 1.0` 을 3초 보내면 약 140° 회전 (명령이 도착하기까지의 지연 포함).

## 5. 바꿔 보기

```bash
~/isaacsim/python.sh 14_pinky_ros2/pinky_ros2.py --gui
~/isaacsim/python.sh 14_pinky_ros2/pinky_ros2.py --no-sensors
~/isaacsim/python.sh 14_pinky_ros2/pinky_ros2.py --seconds 30
```

| 옵션 | 결과 |
|---|---|
| `--gui` | 창으로 보기 |
| `--no-sensors` | `/scan` · `/camera/image_raw` · `/imu` 를 만들지 않음 (가볍게) |
| `--seconds S` | S 초 뒤 끝냄 |

## 막힐 때

| 증상 | 해결 |
|---|---|
| `ROS 2 환경이 없습니다` | 같은 터미널에서 `source /opt/ros/jazzy/setup.bash` 를 먼저 |
| `ros2 topic list` 에 `/cmd_vel` 이 없음 | Isaac Sim 이 ▶ Play 중인지, 두 터미널의 `ROS_DOMAIN_ID` 가 같은지 확인 |
| 명령을 멈췄는데 로봇이 계속 감 | 마지막 명령이 남아 있음 → 정지 명령(`"{}"`)을 보내거나 teleop 의 `k` |
| 앞으로 보냈는데 제자리에서 돎 | Constant Token 두 개의 값(위 `l_wheel_joint`, 아래 `r_wheel_joint`)과 Make Array 연결 확인 |
| 그래프에 `ROS2 ...` 노드가 없음 | `source` 를 하지 않고 Isaac Sim 을 켰음 → 닫고 1절부터 |
| teleop 키가 안 먹음 | teleop 을 실행한 터미널을 클릭한 뒤 누름 |

다음: Pinky Pro 의 센서와 위치를 ROS 2 로 내보내기 (`/scan` · `/camera/image_raw` · `/imu` · `/odom`) 와 RViz2.
