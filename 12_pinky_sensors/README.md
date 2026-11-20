# 12. Pinky Pro 센서: 라이다 · 카메라 · 초음파 · IMU — 명령 모음

- 영상에서 입력하는 터미널 명령과 화면에 넣는 값을 순서대로 모았음. [11. 내 로봇 가져오기: Pinky Pro](../11_pinky_import/README.md) 다음 내용임.

## 1. 준비

```bash
cd ~/isaacsim
./isaac-sim.sh
```

- `File > Open` → `~/nvidia-dli-lecture/work/pinky_gui.usd` (강의 11 에서 저장한 장면)

## 2. 화면에서 누르는 순서

### 라이다 (RPLidar 자리)

| 순서 | 동작 | 값 |
|---|---|---|
| 1 | `pinky > rplidar_mount > rplidar_link` 클릭 | |
| 2 | `Create > Sensors > PhysX Lidar > Rotating` | `rplidar_link/Lidar` 가 생김 |
| 3 | `Lidar` 선택 → Property 검색칸에 `range` → minRange | `0.15` |
| 4 | maxRange | `12` |
| 5 | 검색칸에 `draw` → drawLines 체크 → 검색칸 비우기 | |

### 장애물 · 벽

| 순서 | 동작 | 값 |
|---|---|---|
| 6 | Stage 빈 곳 클릭 → `Create > Mesh > Cube` → Translate / Scale | (`0.5`, `0.3`, `0.1`) / (`0.1`, `0.1`, `0.2`) |
| 7 | 같은 방법으로 Cylinder · Cone · Cube | 아래 표 |
| 8 | 벽: Cube → 이름 `Wall` → Translate / Scale | (`1.01`, `0`, `0.1`) / (`0.02`, `0.3`, `0.2`) |
| 9 | ▶ Play → ■ Stop | 선이 모두 옅은 회색 (못 봄) |
| 10 | `Ctrl` 로 장애물 4개와 Wall 선택 → 우클릭 → `Add > Physics > Colliders Preset` | |
| 11 | ▶ Play → ■ Stop | 장애물에 닿은 선에 색이 생김 |

| 모양 | Translate | Scale |
|---|---|---|
| Cube | (0.5, 0.3, 0.1) | (0.1, 0.1, 0.2) |
| Cylinder | (0.4, −0.3, 0.1) | (0.1, 0.1, 0.2) |
| Cone | (−0.4, 0.3, 0.1) | (0.15, 0.15, 0.2) |
| Cube | (0, −0.5, 0.1) | (0.1, 0.1, 0.2) |

### 카메라

| 순서 | 동작 | 값 |
|---|---|---|
| 12 | `pinky > front_camera_mount > front_camera_link` 우클릭 → `Create > Camera` | |
| 13 | `Camera` 선택 → Orient | `90`, `-90`, `0` |
| 14 | Focal Length | `16.13` |
| 15 | 뷰포트 위 `Perspective > Cameras > Camera` → 확인 후 다시 `Perspective` | |
| 16 | `Window > Viewports > Viewport 2` → Viewport 2 의 `Perspective > Cameras > Camera` | |

### 초음파 (PhysX Lidar 를 좁게)

| 순서 | 동작 | 값 |
|---|---|---|
| 17 | `pinky > base_link > ultrasonic_link` 클릭 → `Create > Sensors > PhysX Lidar > Rotating` | |
| 18 | 새 `Lidar` 더블클릭 → 이름 | `Ultrasonic` |
| 19 | 검색 `fov` → horizontalFov / verticalFov | `15` / `1` |
| 20 | 검색 `resolution` → horizontalResolution / verticalResolution | `1` / `1` |
| 21 | 검색 `rotation` → rotationRate | `0` |
| 22 | 검색 `range` → minRange / maxRange | `0.02` / `4` |
| 23 | 검색 `draw` → drawLines 체크 → 검색칸 비우기 | |
| 24 | ▶ Play → ■ Stop | 앞 벽까지 부채꼴 선 |

### IMU

| 순서 | 동작 | 값 |
|---|---|---|
| 25 | `pinky > base_link > imu_link` 클릭 → `Create > Sensors > Imu Sensor` | `imu_link/Imu_Sensor` 가 생김 |
| 26 | `Ctrl` + `S` | 값은 3절의 코드로 확인 |

## 3. 같은 센서를 코드로

- Isaac Sim 창을 닫은 뒤 실행함.

```bash
cd ~/nvidia-dli-lecture
git pull
~/isaacsim/python.sh 12_pinky_sensors/pinky_sensors.py
```

```text
라이다 위치: (-0.017, 0.000, 0.125) m, 빔 720개 (0.5° 간격) 중 119개가 물체에 닿음
          물체    방위각[°]    중심까지[m]    잰 거리[m]
      0 Cube      30.1       0.60       0.54
  1 Cylinder     -35.7       0.51       0.46
      2 Cone     141.9       0.49       0.46
      3 Cube     -88.1       0.50       0.45
        Wall       0.0       1.03       1.02
초음파: 빔 15개 중 가장 가까운 값 0.973 m (센서에서 벽까지 실제 0.973 m)
카메라 사진 저장: /home/pw/nvidia-dli-lecture/work/pinky_camera.png
IMU (정지)       : 가속도 (0.00, 0.00, 9.81) m/s², 각속도 (0.00, 0.00, 0.00) rad/s
IMU (1 rad/s 회전): 가속도 (0.00, 0.00, 9.81) m/s², 각속도 (0.00, 0.00, 0.99) rad/s
결과: 라이다와 초음파가 장애물을 봅니다.
```

- 초음파 값은 부채꼴 빔 15개 중 가장 가까운 값임. 센서가 로봇 중심에서 0.027 m 앞에 있어 벽(1.0 m)까지 0.973 m 임.
- 카메라 사진은 `work/pinky_camera.png` (1280 × 720) 로 저장됨.
- IMU 는 서 있을 때와 1 rad/s 로 도는 중에 한 번씩 읽음. 가속도 z 9.81 은 중력임.

## 4. 바꿔 보기

```bash
~/isaacsim/python.sh 12_pinky_sensors/pinky_sensors.py --wall 0.5
~/isaacsim/python.sh 12_pinky_sensors/pinky_sensors.py --no-obstacle-physics
~/isaacsim/python.sh 12_pinky_sensors/pinky_sensors.py --gui
~/isaacsim/python.sh 12_pinky_sensors/pinky_sensors.py --keyboard
```

| 옵션 | 결과 |
|---|---|
| (없음) 벽 1.0 m | 초음파 0.973 m |
| `--wall 0.5` | 초음파 0.473 m |
| `--no-obstacle-physics` | 라이다 0개, 초음파 모두 안 잡힘 — 충돌체가 없으면 빔이 통과 |
| `--gui` · `--keyboard` | 창으로 보기 · W A S D 로 조종 |

## 막힐 때

| 증상 | 해결 |
|---|---|
| 라이다 선이 로봇 바로 옆에서 끊김 | 자기 몸(LED 바 · 화면)을 본 것 → minRange 를 `0.15` 로 |
| 장애물이 안 잡힘 | 장애물 높이가 라이다 높이(0.125 m)보다 높은지, `Colliders Preset` 을 넣었는지 확인 |
| Property 에 minRange 가 안 보임 | 맨 아래 `Raw USD Properties` 를 펼치고 검색칸에 `range` |
| 카메라 화면이 옆으로 누움 | Orient 가 (`90`, `-90`, `0`) 인지 확인 |
| `URDF 파일이 없습니다` (코드) | 강의 11 의 1절을 먼저 실행 |

## 옵션 전체

| 옵션 | 뜻 |
|---|---|
| `--wall D` | 로봇 앞 벽까지의 거리 [m] (기본 1.0) |
| `--no-obstacle-physics` | 장애물에 충돌체를 주지 않음 |
| `--image 경로` | 카메라 사진 저장 경로 (기본 `work/pinky_camera.png`) |
| `--keyboard` | 창을 띄우고 W A S D 로 조종 |
| `--gui` | 창을 띄움 |
| `--save USD` | Play 전 장면을 저장 |

- 다음: [13. Pinky Pro 화면과 LED](../13_pinky_lcd_led/README.md)
