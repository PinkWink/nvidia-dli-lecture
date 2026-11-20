# 4. 로봇 만들기: 몸체 · 바퀴 · 회전 관절 · 구동 — 명령 모음

- 영상에서 입력하는 터미널 명령과 화면에 넣는 값을 순서대로 모았음. [3. 첫 장면](../03_first_scene/README.md) 다음 내용임.

## 1. 준비

```bash
cd ~/isaacsim
./isaac-sim.sh
```

- `File > Open` → `~/nvidia-dli-lecture/work/first_scene.usd` (강의 3 에서 저장한 장면)
- `File > Save As` → 같은 폴더에 `simple_robot.usd`

## 2. 화면에서 누르는 순서

- 값을 바꿀 때는 항상 ■ Stop 상태에서 바꿈.

### 몸체

| 순서 | 동작 | 값 |
|---|---|---|
| 1 | Cube 선택 → Translate | `0` `0` `0` |
| 2 | Cube → Scale | `2` `1` `0.5` |
| 3 | World 우클릭 → `Create > Xform` → 이름 바꾸기 | `SimpleRobot` |
| 4 | Cube 를 끌어서 SimpleRobot 위에 놓기 | |
| 5 | SimpleRobot → Translate Z | `1` |

### 바퀴 네 개

| 순서 | 동작 | 값 |
|---|---|---|
| 6 | SimpleRobot 우클릭 → `Create > Mesh > Cylinder` | |
| 7 | Cylinder → Orient | `90` `0` `0` |
| 8 | Cylinder → Scale | `0.75` `0.75` `0.25` |
| 9 | Cylinder → Translate, 이름 바꾸기 | `0.5` `0.75` `0`, `Front_Left_Wheel` |
| 10 | 우클릭 → `Duplicate` 세 번, 이름과 Translate | 아래 표 |
| 11 | Ctrl 누른 채 바퀴 4개 클릭 → 우클릭 → `Add > Physics > Rigid Body with Colliders Preset` | |

| 바퀴 | Translate |
|---|---|
| `Front_Left_Wheel` | `0.5` `0.75` `0` |
| `Front_Right_Wheel` | `0.5` `-0.75` `0` |
| `Rear_Left_Wheel` | `-0.5` `0.75` `0` |
| `Rear_Right_Wheel` | `-0.5` `-0.75` `0` |

### 회전 관절 (바퀴마다 한 번씩, 네 번)

| 순서 | 동작 | 값 |
|---|---|---|
| 12 | Cube 클릭 → Ctrl 누른 채 바퀴 클릭 (순서 중요) | |
| 13 | 바퀴 위에서 우클릭 → `Create > Physics > Joint > Revolute Joint` | |
| 14 | 관절 이름 바꾸기 | `Front_Left_Joint` · `Front_Right_Joint` · `Rear_Left_Joint` · `Rear_Right_Joint` |
| 15 | Axis | `Y` |
| 16 | Local Rotation 0 | `0` `0` `0` |
| 17 | Local Rotation 1 | `-90` `0` `0` |
| 18 | (네 관절을 다 만든 뒤) SimpleRobot → Translate Z | `0.4` |

### 뒷바퀴 구동 (Rear_Left_Joint · Rear_Right_Joint)

| 순서 | 동작 | 값 |
|---|---|---|
| 19 | 관절 우클릭 → `Add > Physics > Angular Drive` | |
| 20 | Drive > Angular > Damping | `10000` |
| 21 | Target Velocity | `50` |
| 22 | ▶ Play 4초 → ■ Stop | 앞으로 약 1.27 m |
| 23 | 도전: Rear_Right_Joint 의 Target Velocity 를 `-50` 으로 → ▶ Play | 오른쪽으로 돎 |
| 24 | 두 Target Velocity 를 `0` 으로 돌려놓고 `Ctrl` + `S` | |

## 3. 같은 로봇을 코드로

- Isaac Sim 창을 닫은 뒤 실행함.

```bash
cd ~/nvidia-dli-lecture
git pull
~/isaacsim/python.sh 04_build_robot/build_robot.py
```

```text
   시간[s]     x[m]     y[m]     z[m]    방향[°]
    0.00    0.000    0.000    0.400      0.0
    0.50    0.136   -0.000    0.375      0.0
    1.00    0.296   -0.000    0.375      0.0
    1.50    0.453   -0.000    0.376     -0.0
    2.00    0.614   -0.000    0.374      0.0
    2.50    0.775   -0.000    0.374     -0.0
    3.00    0.934   -0.000    0.375      0.0
    3.50    1.094   -0.000    0.375     -0.0
    4.00    1.254   -0.000    0.374     -0.0
결과: 4초 동안 앞으로 1.25 m, 방향 -0.0°.
```

## 4. 하나씩 바꿔 보기

```bash
~/isaacsim/python.sh 04_build_robot/build_robot.py --no-joints --seconds 1.5
~/isaacsim/python.sh 04_build_robot/build_robot.py --left 50 --right -50
```

| 옵션 | 결과 |
|---|---|
| (없음) | 4초 동안 앞으로 1.25 m |
| `--no-joints --seconds 1.5` | 관절이 없어 따로 떨어짐 (몸체 z = 0.25 m, 바퀴 z = 0.37 m) |
| `--left 50 --right -50` | 4초 동안 오른쪽으로 18.7° 돎 |

### 바퀴 모양과 회전

```bash
~/isaacsim/python.sh 04_build_robot/build_robot.py --left 50 --right -50 --round-wheels
```

| 바퀴 | 직진 4초 | 제자리 회전 4초 |
|---|---|---|
| Mesh Cylinder (기본. GUI 의 `Create > Mesh > Cylinder` 와 같은 32각 기둥) | 1.25 m | 18.7° |
| `--round-wheels` (완전히 둥근 `UsdGeom.Cylinder`) | 1.28 m | 30.3° |

- 스크립트는 메뉴와 같은 명령(`CreateMeshPrimWithDefaultXform`)으로 바퀴를 만들어 GUI 로 만든 로봇과 같은 값을 냄.
- 4륜 스키드 회전은 바퀴가 옆으로 미끄러져야 하므로 바퀴 충돌 모양에 민감함. 직진은 거의 같음.

## 5. 창으로 보기 · 장면 저장

```bash
~/isaacsim/python.sh 04_build_robot/build_robot.py --gui
~/isaacsim/python.sh 04_build_robot/build_robot.py --save work/simple_robot_code.usd
```

## 옵션 전체

| 옵션 | 뜻 | 기본값 |
|---|---|---|
| `--left V` | 왼쪽 뒷바퀴 Target Velocity [°/s] | 50 |
| `--right V` | 오른쪽 뒷바퀴 Target Velocity [°/s] | 50 |
| `--no-joints` | 관절을 만들지 않음 | |
| `--round-wheels` | 바퀴를 32각 Mesh 대신 완전히 둥근 원기둥으로 만듦 | |
| `--seconds S` | 시뮬레이션할 시간 [s] | 4.0 |
| `--gui` | 창을 띄움 | |
| `--save USD` | Play 전 장면을 파일로 저장 | |

- 다음: [5. 로봇 조종](../05_control/README.md)
