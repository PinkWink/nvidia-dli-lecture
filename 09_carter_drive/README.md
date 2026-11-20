# 9. 가져온 로봇 몰기: 바닥 · 관절 · 차동 제어기 · 속도 제한 — 명령 모음

- 영상에서 입력하는 터미널 명령과 화면에 넣는 값을 순서대로 모았음. [8. URDF 로봇 가져오기](../08_urdf_import/README.md) 다음 내용임.

## 1. 준비

```bash
cd ~/isaacsim
./isaac-sim.sh
```

- `File > Open` → `~/nvidia-dli-lecture/work/carter_gui.usd` (강의 8 에서 저장한 장면)

## 2. 화면에서 누르는 순서

### 바닥에 내려놓기

| 순서 | 동작 | 값 |
|---|---|---|
| 1 | `Create > Environments > Flat Grid` | |
| 2 | Stage 의 `Environment` 눈 아이콘 끄기 | |
| 3 | `carter` 선택 → Translate Z | `0.3` |
| 4 | ▶ Play 2초 → ■ Stop | 몸체 높이 0.24 m |

### 관절 살펴보기 · 바퀴 직접 돌리기

| 순서 | 동작 | 값 |
|---|---|---|
| 5 | `carter > joints > left_wheel` 클릭 | Axis `Y`, Damping 약 308 |
| 6 | `rear_pivot` 클릭 | Axis `Z`, Damping 0 |
| 7 | `left_wheel` 클릭, `Ctrl` + `right_wheel` 클릭 → Drive 의 Target Velocity | `20` |
| 8 | ▶ Play 4초 | 앞으로 약 0.34 m |
| 9 | Play 중에 `left_wheel` 만 Target Velocity | `0` → 왼쪽으로 돎 |
| 10 | ■ Stop → 두 Target Velocity | `0` |

### 차동 제어기 · 속도 제한

| 순서 | 동작 | 값 |
|---|---|---|
| 11 | `Tools > Robotics > OmniGraph Controllers > Differential Controller` | 아래 표 → OK |
| 12 | ▶ Play → 뷰포트 클릭 → `W` 3초 → ■ Stop | 너무 빨라 휘청임 |
| 13 | Stage 의 `Graphs > differential_controller > DifferentialController` 선택 → maxLinearSpeed | `0.5` |
| 14 | maxAngularSpeed | `0.5` |
| 15 | ▶ Play → `W` 3초 | 똑바로 약 1.4 m |
| 16 | ■ Stop → `Ctrl` + `S` | |

### Differential Controller 창에 넣을 값

| 칸 | 값 |
|---|---|
| Robot Prim | `/World/carter` |
| wheel radius | `0.24` |
| distance between wheels | `0.62` |
| Right Joint Name | `left_wheel` |
| Left Joint Name | `right_wheel` |
| Use Keyboard Control (WASD) | 체크 |

- **주의 (Isaac Sim 5.1)**: Right 칸에 `left_wheel`, Left 칸에 `right_wheel` 을 넣음. 칸 이름대로 넣으면 회전 방향이 반대가 됨 (강의 5 와 같은 문제).

## 3. 같은 주행을 코드로

- Isaac Sim 창을 닫은 뒤 실행함.

```bash
cd ~/nvidia-dli-lecture
git pull
~/isaacsim/python.sh 09_carter_drive/drive_carter.py
```

```text
   시간[s]     x[m]     y[m]     z[m]    방향[°]
     0.0     0.00     0.00     0.30      0.0
     0.5     0.18     0.00     0.24      0.1
     1.0     0.43     0.00     0.24      0.1
     1.5     0.68     0.00     0.24      0.2
     2.0     0.93     0.00     0.24      0.2
     2.5     1.18     0.00     0.24      0.2
     3.0     1.43     0.00     0.24      0.2
바퀴 명령: 왼쪽 2.08 rad/s, 오른쪽 2.08 rad/s (최대 0.5 m/s · 0.5 rad/s)
결과: v = 0.5 m/s, ω = 0 rad/s 로 3초 → 위치 (1.43, 0.00) m, 방향 +0.2°.
```

- 바퀴 명령 2.08 rad/s = 0.5 m/s ÷ 바퀴 반지름 0.24 m.

## 4. 바퀴 속도를 직접 주기

```bash
~/isaacsim/python.sh 09_carter_drive/drive_carter.py --wheels 20 20 --seconds 4
~/isaacsim/python.sh 09_carter_drive/drive_carter.py --wheels 0 20 --seconds 4
```

```text
   시간[s]     x[m]     y[m]     z[m]    방향[°]
     0.0     0.00     0.00     0.30      0.0
     0.5     0.03     0.00     0.24      0.0
     1.0     0.08     0.00     0.24      0.0
     1.5     0.12     0.00     0.24      0.0
     2.0     0.16     0.00     0.24      0.0
     2.5     0.20     0.00     0.24      0.0
     3.0     0.25     0.00     0.24      0.0
     3.5     0.29     0.00     0.24      0.0
     4.0     0.33     0.00     0.24      0.0
결과: 왼쪽 20 °/s, 오른쪽 20 °/s 로 4초 → 위치 (0.33, 0.00) m, 방향 +0.0°.
```

| 옵션 | 결과 |
|---|---|
| `--wheels 20 20 --seconds 4` | 4초에 앞으로 0.33 m (계산: 20 °/s = 0.349 rad/s × 0.24 m × 4 s = 0.335 m) |
| `--wheels 0 20 --seconds 4` | 4초에 왼쪽으로 22.4° |

## 5. 속도 제한 비교

```bash
~/isaacsim/python.sh 09_carter_drive/drive_carter.py --linear 5 --no-limit
~/isaacsim/python.sh 09_carter_drive/drive_carter.py --linear 5
~/isaacsim/python.sh 09_carter_drive/drive_carter.py --linear 0 --angular 0.5
```

| 옵션 | 바퀴 명령 | 결과 (3초) |
|---|---|---|
| `--linear 5 --no-limit` | 20.83 rad/s | (3.27, 1.12) m, 방향 −110.6° — 휘청이며 틀어짐 |
| `--linear 5` | 2.08 rad/s (0.5 m/s 로 제한) | (1.43, 0.00) m, 방향 +0.2° — 똑바로 |
| `--linear 0 --angular 0.5` | ∓0.65 rad/s | 제자리에서 왼쪽으로 86.4° |

## 6. 키보드로 조종하기

```bash
~/isaacsim/python.sh 09_carter_drive/drive_carter.py --keyboard
```

- 창이 뜨면 뷰포트를 한 번 클릭한 뒤 W · A · S · D. 창을 닫으면 끝남.

## 막힐 때

| 증상 | 해결 |
|---|---|
| A 를 눌렀는데 오른쪽으로 돎 | 제어기 창의 Right · Left 칸을 서로 바꿔 넣었는지 확인 |
| W 를 눌러도 안 움직임 | ▶ Play 중인지, 뷰포트를 한 번 클릭했는지 확인 |
| 로봇이 바닥에 묻혀 튀어 오름 | `carter` 의 Translate Z 를 `0.3` 으로 |
| `URDF 파일이 없습니다` (코드) | 강의 8 의 1절 `cp -r` 명령을 먼저 실행 |

## 옵션 전체

| 옵션 | 뜻 |
|---|---|
| `--wheels L R` | 제어기 없이 왼 · 오 바퀴 Target Velocity [°/s] |
| `--linear V` | 직진 속도 [m/s] (기본 0.5) |
| `--angular W` | 회전 속도 [rad/s] (+ 왼쪽) |
| `--max-linear` · `--max-angular` | 속도 제한 (기본 0.5 · 0.5) |
| `--no-limit` | 속도 제한 없음 |
| `--seconds S` | 시뮬레이션할 시간 (기본 3) |
| `--keyboard` | 창을 띄우고 W A S D 로 조종 |
| `--gui` | 창을 띄움 |
| `--save USD` | Play 전 장면을 저장 |

- 다음: [10. 가져온 로봇에 센서 달기 · Nova Carter](../10_carter_sensors/README.md)
