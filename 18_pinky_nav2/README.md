# 18. 핑크시티 자율주행: AMCL · Nav2 — 명령 모음

강의 17 에서 저장한 지도 위에서 Pinky Pro 의 위치를 추정하고(AMCL), 목적지를 주면 스스로 가게 합니다(Nav2). [17. 핑크시티 지도 만들기](../17_pinky_slam/README.md) 다음 내용입니다.

이 강의의 로봇은 도로를 모릅니다. 지도에는 벽 · 건물 · 나무만 있어서, 보도를 가로질러 가장 짧은 길로 갑니다.

## 1. 준비 (한 번만)

강의 17 의 지도(`work/pink_city/pink_city_map.yaml` · `.pgm`)가 있어야 합니다. Pinky Pro [공식 저장소](https://github.com/pinklab-art/pinky_pro)의 내비게이션 패키지를 빌드합니다 (저장소는 강의 11 에서 `~/pinky_pro` 에 받았습니다).

```bash
cd ~/pinky_pro
source /opt/ros/jazzy/setup.bash
colcon build --packages-select pinky_navigation
```

```text
Summary: 1 package finished
```

모든 터미널에서 먼저 `source /opt/ros/jazzy/setup.bash`. 다른 ROS 2 장비와 같은 네트워크라면 이어서 `export ROS_DOMAIN_ID=83` 처럼 같은 번호를 정합니다.

## 2. 터미널 1: 핑크시티 + Pinky Pro

```bash
source /opt/ros/jazzy/setup.bash
cd ~/nvidia-dli-lecture
git pull
~/isaacsim/python.sh 16_pink_city/pink_city.py --ros2 --gui
```

## 3. 터미널 2: robot_state_publisher

```bash
source /opt/ros/jazzy/setup.bash
ros2 run robot_state_publisher robot_state_publisher --ros-args -p use_sim_time:=true -p robot_description:="$(cat ~/nvidia-dli-lecture/work/pinky_description/urdf/pinky.urdf)"
```

## 4. 터미널 3: Nav2 (지도 서버 · AMCL · 경로 계획 · 제어)

```bash
source /opt/ros/jazzy/setup.bash
source ~/pinky_pro/install/setup.bash
ros2 launch pinky_navigation bringup_launch.xml map:=$HOME/nvidia-dli-lecture/work/pink_city/pink_city_map.yaml params_file:=$HOME/nvidia-dli-lecture/18_pinky_nav2/nav2_params.yaml use_sim_time:=True
```

`Managed nodes are active` 가 두 번(localization · navigation) 나오면 준비된 것입니다.

## 5. 터미널 4: RViz2

```bash
source /opt/ros/jazzy/setup.bash
source ~/pinky_pro/install/setup.bash
rviz2 -d ~/nvidia-dli-lecture/18_pinky_nav2/rviz/nav2.rviz --ros-args -p use_sim_time:=true
```

왼쪽 Navigation 2 패널에 Navigation: active, Localization: active 가 보여야 합니다.

## 6. 목적지 주기

RViz2 위쪽의 `Nav2 Goal` 을 누르고, 지도에서 가고 싶은 곳을 누른 채 바라볼 방향으로 끌었다 놓습니다.

명령으로 줄 수도 있습니다 (터미널 5):

```bash
source /opt/ros/jazzy/setup.bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: -2.1}, orientation: {w: 1.0}}}}"
```

```text
Goal finished with status: SUCCEEDED
```

출발점으로 돌아오기:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: 0.0, y: 0.0}, orientation: {w: 1.0}}}}"
```

지도 좌표 = 출발점이 (0, 0), 로봇이 처음 보던 방향이 +x 입니다. 도시 좌표 (X, Y) 는 지도 좌표 (Y + 0.5, −X − 1.05) 가 됩니다.

## 7. 위치 추정이 틀어졌을 때

RViz2 위쪽의 `2D Pose Estimate` 를 누르고, 지도에서 로봇이 실제로 있는 곳을 누른 채 로봇이 보는 방향으로 끌었다 놓습니다.

이 강의에서는 누르지 않아도 됩니다. 시뮬레이터를 새로 켜면 로봇이 지도를 만들 때와 같은 자리에서 시작하고, 설정의 `initial_pose: [0, 0, 0]` 이 그 자리이기 때문입니다.

## 공식 설정에서 바꾼 값

`18_pinky_nav2/nav2_params.yaml` 은 공식 저장소의 `pinky_navigation/params/nav2_params.yaml` 에서 아래 값만 바꿨습니다. 실행 파일(`bringup_launch.xml`)과 RViz2 설정(시점만 확대)은 공식 것을 씁니다.

| 값 | 공식 설정 | 이 강의 | 뜻 |
|---|---|---|---|
| costmap `resolution` | 0.05 | 0.02 | 비용 지도 한 칸의 크기 [m]. 지도와 같게 |
| local_costmap `width` · `height` | 3 | 2 | 로봇 주변만 보는 비용 지도의 크기 [m] |
| `lookahead_dist` | 0.6 | 0.25 | 경로에서 얼마나 앞의 점을 보고 따라갈지 [m] |
| `min` · `max_lookahead_dist` | 0.3 · 0.9 | 0.15 · 0.4 | 위 값의 범위 |
| `curvature_lookahead_dist` | 0.6 | 0.25 | 굽은 정도를 볼 때 내다보는 거리 [m] |
| `approach_velocity_scaling_dist` | 0.6 | 0.3 | 목표 앞에서 감속을 시작하는 거리 [m] |
| `xy_goal_tolerance` | 0.25 | 0.08 | 도착으로 인정하는 거리 [m] |
| `required_movement_radius` | 0.5 | 0.2 | 10초 동안 이만큼 못 움직이면 끼인 것으로 봄 [m] |
| planner `tolerance` | 0.5 | 0.1 | 목표 근처에서 경로를 못 찾을 때 허용하는 거리 [m] |
| `use_sim_time` | false | true | 시뮬레이션 시간 사용 |

## 강의 PC 의 결과

| 목표 (지도 좌표) | 결과 | 걸린 시간 | 도착 위치와 목표의 거리 | 위치 추정 오차 |
|---|---|---|---|---|
| (1.0, −2.1) 본사 건너편 | 도착 | 15초 | 3 cm | 6 cm |
| (0.5, −1.05) 사거리 가운데 | 도착 | 10초 | 7 cm | 1 cm |
| (0.0, 0.0) 출발점 | 도착 | 13초 | 5 cm | 6 cm |

## 막힐 때

| 증상 | 해결 |
|---|---|
| `Package 'pinky_navigation' not found` | 1절의 빌드를 하고, 그 터미널에서 `source ~/pinky_pro/install/setup.bash` |
| 지도가 안 나옴 · `map_server` 오류 | `map:=` 경로의 파일이 있는지 확인 (강의 17 의 지도 저장) |
| Navigation 패널이 inactive 인 채로 있음 | robot_state_publisher(터미널 2)와 시뮬레이터가 켜져 있는지, `use_sim_time:=True` 를 줬는지 확인 |
| 목표를 줘도 안 움직임 · "Failed to make progress" | 목표가 건물 안이나 벽에 너무 가까움. 빈 곳을 찍기 |
| 라이다 점과 지도의 벽이 어긋나 있음 | 위치 추정이 틀어짐 → `2D Pose Estimate` 로 다시 알려 주기 |
| 다시 처음부터 하고 싶음 | 터미널 3(Nav2)과 터미널 1(시뮬레이터)을 모두 껐다 켬 |
