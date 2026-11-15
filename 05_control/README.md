# 5. 로봇 조종: Articulation Root · Differential Controller · 키보드 — 명령 모음

영상에서 입력하는 터미널 명령과 화면에 넣는 값을 순서대로 모았습니다. [4. 로봇 만들기](../04_build_robot/README.md) 다음 내용입니다.

## 1. 준비

```bash
cd ~/isaacsim
./isaac-sim.sh
```

- `File > Open` → `~/nvidia-dli-lecture/work/simple_robot.usd`

## 2. 화면에서 누르는 순서

| 순서 | 동작 | 값 |
|---|---|---|
| 1 | SimpleRobot 우클릭 → `Add > Physics > Articulation Root` | |
| 2 | `Tools > Robotics > OmniGraph Controllers > Differential Controller` | 아래 표 → OK |
| 3 | Stage 의 `Graphs/differential_controller` 우클릭 → `Open Graph` | |
| 4 | DifferentialController 노드 선택 → Desired Linear Velocity | `1` |
| 5 | ▶ Play 3초 → ■ Stop | 앞으로 약 2.8 m |
| 6 | Desired Linear Velocity `0`, Desired Angular Velocity | `1` |
| 7 | ▶ Play 3초 → ■ Stop | 왼쪽으로 돎 |

### Differential Controller 창에 넣을 값

| 칸 | 값 |
|---|---|
| graph path | `/Graphs/differential_controller` (기본값) |
| Robot Prim | `/World/SimpleRobot` |
| wheel radius | `0.375` |
| distance between wheels | `1.5` |
| Right Joint Name | `Rear_Left_Joint` |
| Left Joint Name | `Rear_Right_Joint` |
| Use Keyboard Control (WASD) | 처음엔 끔, 키보드 조종 때 켬 |

**주의 (Isaac Sim 5.1)**: Right 칸에 `Rear_Left_Joint`, Left 칸에 `Rear_Right_Joint` 를 넣습니다. 칸 이름대로 넣으면 회전 방향이 반대가 됩니다.

### 키보드 조종

| 순서 | 동작 | 값 |
|---|---|---|
| 8 | `Graphs/differential_controller` 우클릭 → `Delete` | |
| 9 | 메뉴를 다시 열어 같은 값 + `Use Keyboard Control (WASD)` 체크 → OK | |
| 10 | `Open Graph` → ScaleLinear 노드 선택 → Value | `1.5` |
| 11 | ▶ Play → 뷰포트를 한 번 클릭 → W · A · S · D | |
| 12 | ■ Stop → `Ctrl` + `S` | |

## 3. 같은 조종을 코드로

Isaac Sim 창을 닫은 뒤 실행합니다.

```bash
cd ~/nvidia-dli-lecture
git pull
~/isaacsim/python.sh 05_control/drive_robot.py
```

```text
   시간[s]     x[m]     y[m]     z[m]    방향[°]
    0.00    0.000    0.000    0.400      0.0
    0.50    0.324    0.000    0.375      0.0
    1.00    0.824   -0.000    0.375      0.0
    1.50    1.324   -0.000    0.375      0.0
    2.00    1.824   -0.000    0.375      0.0
    2.50    2.324   -0.001    0.375      0.0
    3.00    2.823   -0.001    0.375      0.0
결과: v = 1 m/s, ω = 0 rad/s 로 3초 → 위치 (2.82, -0.00) m, 방향 +0.0°.
```

## 4. 속도를 바꿔 보기

```bash
~/isaacsim/python.sh 05_control/drive_robot.py --linear 0 --angular 1
~/isaacsim/python.sh 05_control/drive_robot.py --linear -1 --seconds 2
```

| 옵션 | 결과 |
|---|---|
| (없음) v = 1 m/s | 3초에 앞으로 2.82 m |
| `--linear 0 --angular 1` | 3초에 왼쪽으로 52.9° (방향 + = 왼쪽) |
| `--linear -1 --seconds 2` | 2초에 뒤로 1.73 m |

## 5. 키보드로 조종하기

```bash
~/isaacsim/python.sh 05_control/drive_robot.py --keyboard
```

창이 뜨면 뷰포트를 한 번 클릭한 뒤 W(앞) · S(뒤) · A(왼쪽) · D(오른쪽). 창을 닫으면 끝납니다.

```bash
~/isaacsim/python.sh 05_control/drive_robot.py --keyboard --key-linear 1 --key-angular 3
```

## 옵션 전체

| 옵션 | 뜻 | 기본값 |
|---|---|---|
| `--linear V` | 직진 속도 [m/s] (+ 앞) | 1.0 |
| `--angular W` | 회전 속도 [rad/s] (+ 왼쪽으로 돌기) | 0.0 |
| `--seconds S` | 시뮬레이션할 시간 [s] | 3.0 |
| `--keyboard` | 창을 띄우고 W A S D 로 조종 | |
| `--key-linear V` | W · S 키가 주는 속도 [m/s] | 1.5 |
| `--key-angular W` | A · D 키가 주는 속도 [rad/s] | 6.0 |
| `--gui` | 창을 띄움 | |
| `--save USD` | Play 전 장면을 파일로 저장 | |

다음: [6. 센서 달기](../06_sensors/README.md)
