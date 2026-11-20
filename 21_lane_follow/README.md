# 21. 카메라로 차선을 따라 달리기: 세그먼테이션 + 제어기 — 명령 모음

- 강의 20 의 모델을 ROS 2 노드로 만들어, Pinky Pro 가 카메라 영상만 보고 핑크시티의 오른쪽 차로를 따라 달리게 함. [20. 도로를 알아보는 모델 학습](../20_road_segmentation/README.md) 다음 내용임.

## 1. 준비

- `work/road_model/road_seg.onnx` 가 있어야 함 (강의 20).

```bash
sudo apt install ros-jazzy-rqt-image-view      # 한 번만 (이미 있으면 넘어감)
cd ~/nvidia-dli-lecture
git pull
ls work/road_model/road_seg.onnx
```

- 모든 터미널에서 먼저 `source /opt/ros/jazzy/setup.bash`. 다른 ROS 2 장비와 같은 네트워크라면 이어서 `export ROS_DOMAIN_ID=83` 처럼 같은 번호를 정함.

## 2. 터미널 1: 핑크시티 + Pinky Pro

```bash
source /opt/ros/jazzy/setup.bash
cd ~/nvidia-dli-lecture
~/isaacsim/python.sh 16_pink_city/pink_city.py --ros2 --gui --hide-lidar --start -1.15 0.5 -90
```

- `--start -1.15 0.5 -90` = 순환 도로 왼쪽 변의 바깥 차로에서, 아래(−Y)를 보고 시작
- `--hide-lidar` = 라이다 빔을 그리지 않음 (빔이 로봇 카메라 영상에 찍히면 인식이 흐트러짐)

## 3. 터미널 2: 인식 결과 보기

```bash
source /opt/ros/jazzy/setup.bash
ros2 run rqt_image_view rqt_image_view /lane/debug
```

- 노드를 켜기 전에는 빈 화면임.

## 4. 터미널 3: 차선 추종 노드

```bash
source /opt/ros/jazzy/setup.bash
python3 ~/nvidia-dli-lecture/21_lane_follow/lane_follow.py
```

```text
[INFO] [lane_follow]: 모델 road_seg.onnx, 속도 0.15 m/s, 게인 1.6. /camera/image_raw 를 기다립니다.
[INFO] [lane_follow]: 차선 추종: 영상 42 장/초, 치우침 -0.04
[INFO] [lane_follow]: 차선 추종: 영상 42 장/초, 치우침 +0.04
[INFO] [lane_follow]: 차선 추종: 영상 41 장/초, 치우침 -0.07
```

- `Ctrl` + `C` 로 끝내면 로봇이 섬.

## 5. 확인

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic echo /cmd_vel --once
```

```text
linear:
  x: 0.1406158312958435
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.10009779951100257
```

## 6. 값을 바꿔 보기

```bash
python3 ~/nvidia-dli-lecture/21_lane_follow/lane_follow.py --ros-args -p speed:=0.2 -p gain:=1.8
```

| 파라미터 | 기본값 | 뜻 |
|---|---|---|
| `speed` | 0.15 | 직진 속도 [m/s]. 많이 틀 때는 자동으로 줄어듦 (최소 35 %) |
| `gain` | 1.6 | 목표가 화면 끝에 있을 때의 회전 속도 [rad/s] |
| `model` | `work/road_model/road_seg.onnx` | 강의 20 의 모델 |

## 7. 한계 보기: 안쪽 차로로 돌면

- 터미널 1 을 끄고 `--start` 없이 다시 켠 뒤, 터미널 3 의 노드를 다시 실행함.

```bash
~/isaacsim/python.sh 16_pink_city/pink_city.py --ros2 --gui --hide-lidar
```

- 이 방향으로는 오른쪽 차로가 안쪽이라 삼거리 입구가 오른쪽에 열려 있음. 강의 PC 에서는 삼거리에서 옆길로 들어간 뒤 사거리 앞에서 목표를 잃고, 28초 만에(0.84 m) 공원 모퉁이에서 멈췄음. 이 문제는 다음 강의에서 지도상 위치를 더해 풂.

## 강의 PC 의 결과

| 항목 | 값 |
|---|---|
| 한 바퀴 (약 8 m) | 59초 |
| 연속 주행 | 129초 동안 17.7 m (두 바퀴 넘게), 평균 0.137 m/s |
| 직선 구간에서 차로 가운데와의 차이 | 평균 1 cm, 최대 5 cm (모두 안쪽으로) |
| 영상 처리 | 1초에 41 ~ 48장 (CPU) |

## 막힐 때

| 증상 | 해결 |
|---|---|
| "모델이 없음" | 강의 20 의 `train.py` 를 먼저 실행 |
| 노드는 켜졌는데 로봇이 안 움직임 | 터미널 1 과 같은 `ROS_DOMAIN_ID` 인지, `ros2 topic hz /camera/image_raw` 로 영상이 나오는지 확인 |
| "도로를 놓쳐 정지" | 카메라에 도로가 안 보임 (벽을 보고 있거나 보도 위). 시뮬레이터를 다시 켜서 시작 위치로 |
| 옆길로 빠지거나 사거리에서 멈춤 | `--start` 없이 켜면 안쪽 차로에서 시작해 갈림길을 만남. 이 강의는 바깥 차로(`--start -1.15 0.5 -90`)에서 함 |
| 좌우로 흔들림 | `gain` 을 낮추기 (예: 1.2) |
| 곡선에서 바깥으로 밀림 | `speed` 를 낮추거나 `gain` 을 높이기 |
