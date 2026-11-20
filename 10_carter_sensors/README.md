# 10. 가져온 로봇에 센서 달기 · 준비된 로봇 Nova Carter — 명령 모음

- 영상에서 입력하는 터미널 명령과 화면에 넣는 값을 순서대로 모았음. [9. 가져온 로봇 몰기](../09_carter_drive/README.md) 다음 내용임.

## 1. 준비

```bash
cd ~/isaacsim
./isaac-sim.sh
```

- `File > Open` → `~/nvidia-dli-lecture/work/carter_gui.usd` (강의 9 에서 저장한 장면)

## 2. 화면에서 누르는 순서

### RGB 카메라 · 장애물

| 순서 | 동작 | 값 |
|---|---|---|
| 1 | `carter > chassis_link` 클릭 → 우클릭 → `Create > Camera` | |
| 2 | `Camera` 더블클릭 → 이름 | `RGB_Sensor` |
| 3 | Translate | `0.1`, `0`, `0.33` |
| 4 | Orient | `90`, `-90`, `0` |
| 5 | Stage 빈 곳 클릭 → `Create > Mesh > Cube` → Translate | `2`, `0.6`, `0.5` |
| 6 | 같은 방법으로 Cylinder · Cone · Cube | (`2.5`, `-1`, `0.5`) · (`-1.5`, `1.5`, `0.5`) · (`0.5`, `-2.2`, `0.5`) |
| 7 | 뷰포트 위 `Perspective > Cameras > RGB_Sensor` → 확인 후 다시 `Perspective` | |
| 8 | `Window > Viewports > Viewport 2` → Viewport 2 의 `Perspective > Cameras > RGB_Sensor` | |

### 2D 라이다

| 순서 | 동작 | 값 |
|---|---|---|
| 9 | `chassis_link` 선택 → `Create > Sensors > PhysX Lidar > Rotating` | |
| 10 | `Lidar` 의 Translate | `-0.05`, `0`, `0.42` |
| 11 | Property 검색칸에 `draw` → drawLines 체크 → 검색칸 비우기 | |
| 12 | ▶ Play → ■ Stop | 선이 모두 회색 (장애물을 못 봄) |
| 13 | `Ctrl` 로 장애물 4개 선택 → 우클릭 → `Add > Physics > Rigid Body with Colliders Preset` | |
| 14 | ▶ Play | 장애물에 맞은 선이 빨강 |
| 15 | 뷰포트 클릭 → W · A · S · D 로 몰아 보기 → ■ Stop → `Ctrl` + `S` | |

### Nova Carter

| 순서 | 동작 | 값 |
|---|---|---|
| 16 | `File > New` → `Create > Environments > Flat Grid` | |
| 17 | `Create > Robots > Nova Carter with Sensors` | 처음에는 내려받느라 몇 분 걸림 |
| 18 | Stage 에서 `Nova_Carter > chassis_link > sensors` 펼치기 | `front_hawk` · `front_RPLidar` 등 |

## 3. 같은 센서를 코드로

- Isaac Sim 창을 닫은 뒤 실행함.

```bash
cd ~/nvidia-dli-lecture
git pull
~/isaacsim/python.sh 10_carter_sensors/carter_sensors.py
```

```text
카메라가 보는 방향: (1.00, 0.00, 0.00)  ← (1, 0, 0) 이면 로봇의 앞
라이다 위치: (-0.05, 0.00, 0.66) m, 빔 900개 (0.4° 간격) 중 235개가 물체에 닿음
         장애물    방위각[°]    중심까지[m]    잰 거리[m]
      0 Cube      16.3       2.14       1.61
  1 Cylinder     -21.4       2.74       2.24
      2 Cone     134.0       2.09       1.92
      3 Cube     -76.0       2.27       1.75
카메라 사진 저장: /home/pw/nvidia-dli-lecture/work/carter_camera.png
결과: 라이다가 장애물을 봅니다.
```

- `중심까지` 는 라이다에서 장애물 중심까지의 거리, `잰 거리` 는 라이다가 잰 표면까지의 거리임.
- 카메라 사진은 `work/carter_camera.png` 로 저장됨.

## 4. 바꿔 보기

```bash
~/isaacsim/python.sh 10_carter_sensors/carter_sensors.py --no-obstacle-physics
~/isaacsim/python.sh 10_carter_sensors/carter_sensors.py --gui
~/isaacsim/python.sh 10_carter_sensors/carter_sensors.py --keyboard
```

| 옵션 | 결과 |
|---|---|
| (없음) | 빔 900개 중 235개가 장애물에 닿음 |
| `--no-obstacle-physics` | 0개 — 충돌체가 없으면 빔이 통과 (`안 잡힘`) |
| `--gui` | 창으로 보기. 라이다 선이 그려짐 |
| `--keyboard` | 창을 띄우고 W A S D 로 조종 (최대 0.5 m/s) |

## 5. Nova Carter 살펴보기

```bash
~/isaacsim/python.sh 10_carter_sensors/nova_carter.py
```

```text
에셋: https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.1/Isaac/Robots/NVIDIA/NovaCarter/nova_carter.usd
센서 묶음            종류             개수  Prim 이름
front_hawk       카메라             2  camera_right, camera_left
front_hawk       IMU             1  Imu_Sensor
left_hawk        카메라             2  camera_right, camera_left
left_hawk        IMU             1  Imu_Sensor
right_hawk       카메라             2  camera_right, camera_left
right_hawk       IMU             1  Imu_Sensor
back_hawk        카메라             2  camera_right, camera_left
back_hawk        IMU             1  Imu_Sensor
front_RPLidar    라이다 (RTX)       1  RPLidar_S2E
rear_RPLidar     라이다 (RTX)       1  RPLidar_S2E
XT_32            라이다 (RTX)       1  PandarXT_32_10hz
front_owl        카메라             1  camera
left_owl         카메라             1  camera
right_owl        카메라             1  camera
back_owl         카메라             1  camera
chassis_imu      IMU             1  Imu_Sensor
합계: 카메라 12개, IMU 5개, 라이다 (RTX) 3개
회전 관절                  구동 Damping
joint_caster_base      10
joint_swing_left       1e-05
joint_swing_right      1e-05
joint_caster_left      0
joint_caster_right     0
joint_wheel_left       1e+06
joint_wheel_right      1e+06
결과: Nova Carter 에는 카메라 12개 · IMU 5개 · 라이다 (RTX) 3개 가 달려 있습니다.
```

- 처음 실행할 때는 에셋을 인터넷에서 내려받느라 몇 분 걸릴 수 있음.

```bash
~/isaacsim/python.sh 10_carter_sensors/nova_carter.py --gui
```

## 막힐 때

| 증상 | 해결 |
|---|---|
| 카메라 시점에서 화면이 이상하게 움직임 | 카메라 시점에서 마우스로 움직이면 카메라가 움직임. `Perspective` 로 돌아온 뒤 조작 |
| 라이다 선이 안 보임 | `Lidar` 의 drawLines 체크, ▶ Play 중인지 확인 |
| 선이 모두 회색 | 장애물에 `Rigid Body with Colliders Preset` 을 넣었는지 확인 |
| `에셋 서버에 연결하지 못했습니다` | 인터넷 연결 확인 |
| `URDF 파일이 없습니다` (코드) | 강의 8 의 1절 `cp -r` 명령을 먼저 실행 |

## 옵션 전체 (carter_sensors.py)

| 옵션 | 뜻 |
|---|---|
| `--no-obstacle-physics` | 장애물에 충돌체를 주지 않음 |
| `--image 경로` | 카메라 사진 저장 경로 (기본 `work/carter_camera.png`) |
| `--keyboard` | 창을 띄우고 W A S D 로 조종 |
| `--gui` | 창을 띄움 |
| `--save USD` | Play 전 장면을 저장 |
