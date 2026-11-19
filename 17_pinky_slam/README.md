# 17. 핑크시티 지도 만들기: slam_toolbox — 명령 모음

Pinky Pro 로 핑크시티를 돌아다니며 라이다로 지도를 만들고 저장합니다. [16. 핑크시티 만들기](../16_pink_city/README.md) 다음 내용입니다.

도로는 바닥 그림이라 라이다에 잡히지 않습니다. 이 강의의 지도에는 벽 · 건물 · 나무만 나옵니다.

## 1. 준비 (한 번만)

```bash
sudo apt install ros-jazzy-slam-toolbox ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-teleop-twist-keyboard
```

터미널이 5개 필요합니다. 모든 터미널에서 먼저 `source /opt/ros/jazzy/setup.bash` 를 합니다. 같은 네트워크에 다른 ROS 2 장비가 있으면 이어서 `export ROS_DOMAIN_ID=83` 처럼 같은 번호를 정해 둡니다.

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

## 4. 터미널 3: slam_toolbox

```bash
source /opt/ros/jazzy/setup.bash
ros2 launch slam_toolbox online_async_launch.py slam_params_file:=$HOME/nvidia-dli-lecture/17_pinky_slam/mapper_params.yaml use_sim_time:=true
```

## 5. 터미널 4: RViz2

```bash
source /opt/ros/jazzy/setup.bash
source ~/pinky_pro/install/setup.bash
rviz2 -d ~/nvidia-dli-lecture/17_pinky_slam/rviz/slam.rviz --ros-args -p use_sim_time:=true
```

## 6. 터미널 5: 로봇 몰기

손으로:

```bash
source /opt/ros/jazzy/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

또는 자동으로 한 바퀴 (약 100초):

```bash
source /opt/ros/jazzy/setup.bash
python3 ~/nvidia-dli-lecture/17_pinky_slam/drive_city.py
```

```text
점 1 / 21 통과
...
점 21 / 21 통과
도착. 로봇을 세웠습니다.
```

## 7. 지도 저장

터미널 5 에서 (slam_toolbox 는 켜 둔 채로):

```bash
ros2 run nav2_map_server map_saver_cli -f ~/nvidia-dli-lecture/work/pink_city/pink_city_map --ros-args -p use_sim_time:=true
```

```text
[INFO] [map_saver]: Map saved successfully
```

```bash
cat ~/nvidia-dli-lecture/work/pink_city/pink_city_map.yaml
```

```text
image: pink_city_map.pgm
mode: trinary
resolution: 0.020
origin: [-1.100, -2.599, 0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

`pink_city_map.pgm`(그림) 과 `pink_city_map.yaml`(설명) 두 파일이 생깁니다. `origin` 의 숫자는 주행에 따라 조금씩 다릅니다.

## 공식 설정에서 바꾼 값

`17_pinky_slam/mapper_params.yaml` 은 [공식 저장소](https://github.com/pinklab-art/pinky_pro)의 `pinky_navigation/params/mapper_params.yaml` 에서 출발해 작은 도시에 맞게 고쳤습니다.

| 값 | 공식 설정 | 이 강의 | 이유 |
|---|---|---|---|
| `resolution` | 0.05 | 0.02 | 지도 한 칸의 크기 [m]. 도로 폭 0.4 m 가 8칸 → 20칸 |
| `minimum_travel_distance` | 0.5 | 0.1 | 이만큼 움직일 때마다 스캔을 지도에 더함 [m] |
| `minimum_travel_heading` | 0.5 | 0.1 | 이만큼 돌 때마다 더함 [rad] |
| `map_update_interval` | 5.0 | 1.0 | 지도를 내보내는 간격 [s] |
| `max_laser_range` | 10.0 | 6.0 | 지도에 쓰는 라이다 최대 거리 [m] |

## 막힐 때

| 증상 | 해결 |
|---|---|
| RViz2 에 지도가 안 나옴 | slam_toolbox 터미널에 오류가 없는지, `use_sim_time:=true` 를 줬는지 확인. `ros2 topic hz /scan` 으로 라이다가 나오는지 확인 |
| slam_toolbox 에 "Message Filter dropping message" 가 계속 나옴 | robot_state_publisher(터미널 2)가 꺼져 있거나 `use_sim_time` 이 빠짐 |
| 지도가 겹쳐 그려지거나 벽이 두 겹 | 너무 빨리 돌았을 때. 천천히 몰고, 다시 하려면 slam_toolbox 를 껐다 켬 |
| 다른 장비의 지도 · 좌표가 섞임 | 모든 터미널에서 같은 새 `ROS_DOMAIN_ID` |
| `drive_city.py` 가 엉뚱한 곳으로 감 | 시뮬레이터를 새로 켠 직후(로봇이 시작 위치에 있을 때) 실행해야 함 |
