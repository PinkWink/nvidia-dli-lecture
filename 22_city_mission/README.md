# 22. 장소 이름으로 명령하기: 위치 추정 + 차선 추종 — 명령 모음

"핑크랩에서 공원까지" 처럼 장소 이름을 주면, Pinky Pro 가 차선을 지키며 교차로에서 방향을 골라 찾아갑니다. 강의 18 의 위치 추정(AMCL)과 강의 21 의 차선 추종을 합칩니다. [21. 카메라로 차선을 따라 달리기](../21_lane_follow/README.md) 다음 내용입니다.

## 1. 준비

필요한 것: 강의 17 의 지도(`work/pink_city/pink_city_map.yaml`), 강의 20 의 모델(`work/road_model/road_seg.onnx`).

```bash
cd ~/nvidia-dli-lecture
git pull
ls work/pink_city/pink_city_map.yaml work/road_model/road_seg.onnx
```

모든 터미널에서 먼저 `source /opt/ros/jazzy/setup.bash`. 다른 ROS 2 장비와 같은 네트워크라면 이어서 `export ROS_DOMAIN_ID=83` 처럼 같은 번호를 정합니다.

## 2. 터미널 1: 핑크시티 + Pinky Pro

```bash
source /opt/ros/jazzy/setup.bash
cd ~/nvidia-dli-lecture
~/isaacsim/python.sh 16_pink_city/pink_city.py --ros2 --gui --hide-lidar
```

이번에는 `--start` 를 주지 않습니다. 지도를 만들 때와 같은 자리에서 시작해야 위치 추정이 바로 맞습니다.

## 3. 터미널 2: robot_state_publisher

```bash
source /opt/ros/jazzy/setup.bash
ros2 run robot_state_publisher robot_state_publisher --ros-args -p use_sim_time:=true -p robot_description:="$(cat ~/nvidia-dli-lecture/work/pinky_description/urdf/pinky.urdf)"
```

## 4. 터미널 3: 위치 추정 (지도 서버 + AMCL)

```bash
source /opt/ros/jazzy/setup.bash
ros2 launch nav2_bringup localization_launch.py map:=$HOME/nvidia-dli-lecture/work/pink_city/pink_city_map.yaml params_file:=$HOME/nvidia-dli-lecture/18_pinky_nav2/nav2_params.yaml use_sim_time:=True
```

Nav2 중에서 위치 추정 부분만 켭니다. 경로 계획 · 제어는 켜지 않습니다 (운전은 우리 노드가 합니다).

## 5. 터미널 4 · 5: 지켜볼 화면

```bash
source /opt/ros/jazzy/setup.bash
source ~/pinky_pro/install/setup.bash
rviz2 -d ~/nvidia-dli-lecture/22_city_mission/rviz/city.rviz --ros-args -p use_sim_time:=true
```

```bash
source /opt/ros/jazzy/setup.bash
ros2 run rqt_image_view rqt_image_view /lane/debug
```

## 6. 터미널 6: 명령

```bash
source /opt/ros/jazzy/setup.bash
python3 ~/nvidia-dli-lecture/22_city_mission/city_driver.py 핑크랩 공원
```

```text
[INFO] [city_driver]: 갈 곳: 핑크랩 → 공원. 위치(map → base_footprint)와 카메라 영상을 기다립니다.
[INFO] [city_driver]: 출발 → 핑크랩: 지금 '남-서' 도로, 지나갈 교차로 서 → 가운데
[INFO] [city_driver]: 교차로 '서': 우회전 → 가운데-서
[INFO] [city_driver]: 교차로 '가운데': 직진 → 가운데-동
[INFO] [city_driver]: 도착: 핑크랩  (도시 좌표 +0.53, -0.10)
[INFO] [city_driver]: 출발 → 공원: 지금 '가운데-동' 도로, 지나갈 교차로 동 → 남
[INFO] [city_driver]: 교차로 '동': 우회전 → 동-남
[INFO] [city_driver]: 교차로 '남': 우회전 → 가운데-남
[INFO] [city_driver]: 도착: 공원  (도시 좌표 +0.10, -0.52)
[INFO] [city_driver]: 모든 장소에 들렀습니다. 끝냅니다.
```

장소를 여러 개 적으면 순서대로 들릅니다.

```bash
python3 ~/nvidia-dli-lecture/22_city_mission/city_driver.py 아파트 아카데미 핑크랩 공원
python3 ~/nvidia-dli-lecture/22_city_mission/city_driver.py --list
```

다시 하려면 터미널 1 · 3 을 껐다 켭니다 (로봇과 위치 추정을 처음 자리로).

## 장소

| 장소 이름 | 도로 위의 점 (도시 좌표) | 어느 도로 |
|---|---|---|
| `핑크랩` | (0.525, 0) | 가운데 ~ 동 (본사 앞) |
| `아카데미` | (−0.655, 0) | 가운데 ~ 서 (핑크 아카데미 앞) |
| `공원` | (0, −0.525) | 가운데 ~ 남 (공원 옆) |
| `아파트` | (1.05, −0.55) | 동 ~ 남 순환 도로 (아파트 옆) |

장소를 더하려면 `city_driver.py` 의 `PLACES` 에 이름과 도로 중심선 위의 점을 한 줄 넣습니다.

## 강의 PC 의 결과

| 명령 | 결과 | 걸린 시간 | 달린 거리 | 교차로 |
|---|---|---|---|---|
| `핑크랩 공원` | 두 곳 모두 도착 | 39초 | 4.0 m | 4번 (우회전 3, 직진 1) |
| `아파트 아카데미 핑크랩 공원` | 네 곳 모두 도착 | 106초 | 10.5 m | 10번 (우회전 6, 좌회전 2, 직진 2) |

## 막힐 때

| 증상 | 해결 |
|---|---|
| "위치와 카메라 영상을 기다립니다" 에서 멈춤 | 터미널 3(위치 추정)이 `Managed nodes are active` 까지 갔는지, 터미널 2 가 켜져 있는지 확인 |
| 교차로에서 엉뚱한 쪽으로 돎 | 위치 추정이 틀어짐. RViz2 에서 빨간 라이다 점과 지도의 벽이 겹치는지 확인. 터미널 1 · 3 을 다시 켬 |
| "모르는 장소" | `--list` 로 이름 확인 (`핑크랩` · `아카데미` · `공원` · `아파트`) |
| 터미널 1 을 `--start` 로 켰음 | 지도의 원점과 로봇의 자리가 달라 위치가 틀립니다. `--start` 없이 켜기 |
| 한글 입력이 어려움 | 이름을 영문으로 바꾸려면 `PLACES` 의 키를 고치면 됩니다 |
