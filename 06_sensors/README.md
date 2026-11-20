# 6. 센서 달기: 카메라 · 2D 라이다 — 명령 모음

- 영상에서 입력하는 터미널 명령과 화면에 넣는 값을 순서대로 모았음. [5. 로봇 조종](../05_control/README.md) 다음 내용임.

## 1. 준비

```bash
cd ~/isaacsim
./isaac-sim.sh
```

- `File > Open` → `~/nvidia-dli-lecture/work/simple_robot.usd`

## 2. 화면에서 누르는 순서

- 모두 ■ Stop 상태에서 함.

### 센서 자리

| 순서 | 동작 | 값 |
|---|---|---|
| 1 | SimpleRobot 우클릭 → `Create > Xform` → 이름 바꾸기 | `Front_Sensor` |
| 2 | Front_Sensor → Translate | `0.9` `0` `0.3` |
| 3 | Front_Sensor 우클릭 → `Add > Physics > Rigid Body` | |
| 4 | Cube 클릭 → Ctrl 누른 채 Front_Sensor 클릭 → 우클릭 → `Create > Physics > Joint > Fixed Joint` | |
| 5 | Front_Sensor 우클릭 → `Create > Mesh > Cube` → 이름 바꾸기 | `Sensor_Mount` |
| 6 | Sensor_Mount → Scale | `0.1` `0.1` `0.1` |
| 7 | Sensor_Mount 우클릭 → `Add > Physics > Colliders Preset` | (Rigid Body 는 주지 않음) |

### 카메라

| 순서 | 동작 | 값 |
|---|---|---|
| 8 | Sensor_Mount 우클릭 → `Create > Camera` → 이름 바꾸기 | `Robot_Camera` |
| 9 | Robot_Camera → Translate | `0.5` `0` `0` |
| 10 | Robot_Camera → Orient | `90` `-90` `0` |
| 11 | 선택을 풀고 `Create > Mesh > Cube` (Cone · Cylinder 도) 로 장애물 4개 | 로봇 앞(+X) 1~5 m |
| 12 | 뷰포트 위 `Perspective > Cameras > Robot_Camera` | 로봇 카메라 시점 |
| 13 | `Window > Viewports > Viewport 2` → Viewport 2 를 Robot_Camera 로 | 두 화면 보기 |

- 영상에서 쓴 장애물 위치:

| 이름 | 모양 | Translate |
|---|---|---|
| `Obstacle_0` | Cube | `4` `0.5` `0.5` |
| `Obstacle_1` | Cone | `5` `-1.5` `0.5` |
| `Obstacle_2` | Cylinder | `3.5` `-3` `0.5` |
| `Obstacle_3` | Cube | `6` `2` `0.5` |

### 2D 라이다

| 순서 | 동작 | 값 |
|---|---|---|
| 14 | Front_Sensor 선택 → `Create > Sensors > PhysX Lidar > Rotating` | |
| 15 | Lidar → Translate Z | `0.1` |
| 16 | Property 의 Raw USD Properties → 검색칸에 `draw` → drawLines · drawPoints 체크 | |
| 17 | ▶ Play → ■ Stop | 선이 장애물을 통과 |
| 18 | Ctrl 누른 채 장애물 4개 선택 → 우클릭 → `Add > Physics > Rigid Body with Colliders Preset` | |
| 19 | ▶ Play → ■ Stop | 선이 장애물에서 멈춤 |
| 20 | `Ctrl` + `S` 후 Isaac Sim 닫기 | |

## 3. 같은 센서를 코드로

- Isaac Sim 창을 닫은 뒤 실행함.

```bash
cd ~/nvidia-dli-lecture
git pull
~/isaacsim/python.sh 06_sensors/add_sensors.py
```

```text
카메라가 보는 방향: (1.00, 0.00, 0.00)  ← (1, 0, 0) 이면 로봇의 앞
라이다 위치: (0.90, 0.00, 0.77) m, 빔 900개 (0.4° 간격) 중 115개가 물체에 닿음
       장애물    방위각[°]    잰 거리[m]
    0 Cube       9.2       2.63
    1 Cone     -20.1       4.25
2 Cylinder     -49.1       3.47
    3 Cube      21.4       4.95
카메라 사진 저장: /home/pw/nvidia-dli-lecture/work/robot_camera.png
결과: 라이다가 장애물을 봅니다.
```

- 카메라 사진은 `work/robot_camera.png` 에 저장됨. 파일 관리자에서 열어 봄.

## 4. 충돌체를 빼 보기

```bash
~/isaacsim/python.sh 06_sensors/add_sensors.py --no-obstacle-physics
```

```text
결과: 라이다가 아무것도 보지 못합니다. 장애물에 충돌체가 없으면 빔이 그냥 통과합니다.
```

## 5. 창으로 보기 · 키보드로 몰기

```bash
~/isaacsim/python.sh 06_sensors/add_sensors.py --gui
~/isaacsim/python.sh 06_sensors/add_sensors.py --keyboard
```

## 옵션 전체

| 옵션 | 뜻 | 기본값 |
|---|---|---|
| `--no-obstacle-physics` | 장애물에 충돌체를 주지 않음 | |
| `--image PNG` | 카메라 사진을 저장할 경로 | `work/robot_camera.png` |
| `--keyboard` | 창을 띄우고 W A S D 로 조종 | |
| `--gui` | 창을 띄움 | |
| `--save USD` | Play 전 장면을 파일로 저장 | |

- 다음: [7. ROS 2 로 데이터 보내기](../07_ros2/README.md)
