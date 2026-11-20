# 11. 내 로봇 가져오기: Pinky Pro (URDF 만들기 · 가져오기 · 모터) — 명령 모음

- 영상에서 입력하는 터미널 명령과 화면에 넣는 값을 순서대로 모았음. 강의 8 ~ 10 에서 Carter 로 연습한 순서를 핑크랩의 교육용 로봇 **Pinky Pro** 로 다시 함.

- Pinky Pro 공식 저장소 (URDF 의 출처): https://github.com/pinklab-art/pinky_pro
- 필요한 것: 강의 7 에서 설치한 ROS 2 Jazzy (xacro 파일을 URDF 로 바꾸는 데 씀)

## 1. Pinky Pro URDF 준비

- 방법 A 와 B 중 하나만 함.

### 방법 A: 스크립트로 한 번에

```bash
sudo apt install -y git python3-colcon-common-extensions ros-jazzy-xacro
cd ~/nvidia-dli-lecture
git pull
bash 11_pinky_import/prepare_pinky.sh
```

- 마지막에 `준비 완료: .../work/pinky_description/urdf/pinky.urdf` 가 나오면 끝임.

### 방법 B: 한 줄씩 직접

- ① 도구 설치

```bash
sudo apt install -y git python3-colcon-common-extensions ros-jazzy-xacro
```

- ② 공식 저장소 받기

```bash
mkdir -p ~/pinky_pro/src
cd ~/pinky_pro/src
git clone https://github.com/pinklab-art/pinky_pro.git
```

- ③ `pinky_description` 패키지만 빌드

```bash
cd ~/pinky_pro
source /opt/ros/jazzy/setup.bash
colcon build --packages-select pinky_description
source install/setup.bash
```

- ④ 작업 폴더로 복사하고 xacro → URDF

```bash
mkdir -p ~/nvidia-dli-lecture/work/pinky_description
cp -r src/pinky_pro/pinky_description/. ~/nvidia-dli-lecture/work/pinky_description/
xacro src/pinky_pro/pinky_description/urdf/robot.urdf.xacro > ~/nvidia-dli-lecture/work/pinky_description/urdf/pinky.urdf
grep -c "<link" ~/nvidia-dli-lecture/work/pinky_description/urdf/pinky.urdf
```

- `18` 이 나오면 맞음 (링크 18개).

- ⑤ 화면에 띄울 표정 그림 (강의 13 에서 사용, GIF 마다 한 장면)

```bash
~/isaacsim/python.sh ~/nvidia-dli-lecture/11_pinky_import/make_faces.py src/pinky_pro/pinky_emotion/emotion ~/nvidia-dli-lecture/work/pinky_faces
```

- 셸이 zsh 라면 `setup.bash` 대신 `setup.zsh` 를 씀.

## 2. URDF 읽기

```bash
cd ~/nvidia-dli-lecture
~/isaacsim/python.sh 11_pinky_import/import_pinky.py --read-only
```

```text
URDF: /home/pw/nvidia-dli-lecture/work/pinky_description/urdf/pinky.urdf
로봇 이름: pinky, 링크 18개, 관절 17개
링크                     질량[kg]  모양
base_footprint              -  없음 (자리 표시용 빈 링크)
base_link                 0.8  base_link.dae
l_wheel                   0.2  main_wheel.dae, 충돌 = 공 (반지름 0.028 m)
r_wheel                   0.2  main_wheel.dae, 충돌 = 공 (반지름 0.028 m)
caster_mount_link         0.1  caster_mount.dae
caster_roate_link        0.05  caster_rotate.dae
caster_wheel             0.05  caster_wheel.dae, 충돌 = 공 (반지름 0.0065 m)
robot_lamp               0.01  robot_lamp.dae
rplidar_mount            0.11  rplidar_c1.dae
rplidar_link                -  없음 (자리 표시용 빈 링크)
front_camera_mount       0.01  camera_assy.dae
front_camera_link           -  없음 (자리 표시용 빈 링크)
screen_mount             0.01  screen_assy.dae
ultrasonic_link             -  없음 (자리 표시용 빈 링크)
ir_mid_link                 -  없음 (자리 표시용 빈 링크)
ir_l_link                   -  없음 (자리 표시용 빈 링크)
ir_r_link                   -  없음 (자리 표시용 빈 링크)
imu_link                    -  없음 (자리 표시용 빈 링크)
관절                               종류          부모 → 자식                                  축
base_link_fixed_joint            fixed       base_footprint → base_link               -
l_wheel_joint                    continuous  base_link → l_wheel                      0 1 0
r_wheel_joint                    continuous  base_link → r_wheel                      0 1 0
caster_mount_fixed_joint         fixed       base_link → caster_mount_link            -
caster_rotate_joint              continuous  caster_mount_link → caster_roate_link    0 0 1
caster_wheel_joint               continuous  caster_roate_link → caster_wheel         0 1 0
robot_lamp_mount_fixed_joint     fixed       base_link → robot_lamp                   -
rplidar_mount_fixed_joint        fixed       base_link → rplidar_mount                -
rplidar_link_fixed_joint         fixed       rplidar_mount → rplidar_link             -
front_camera_mount_fixed_joint   fixed       base_link → front_camera_mount           -
front_camera_fixed_joint         fixed       front_camera_mount → front_camera_link   -
screen_mount_fixed_joint         fixed       base_link → screen_mount                 -
ultrasonic_fixed_joint           fixed       base_link → ultrasonic_link              -
ir_mid_fixed_joint               fixed       base_link → ir_mid_link                  -
ir_l_fixed_joint                 fixed       base_link → ir_l_link                    -
ir_r_fixed_joint                 fixed       base_link → ir_r_link                    -
imu_fixed_joint                  fixed       base_link → imu_link                     -
```

## 3. 화면에서 누르는 순서: 가져오기

```bash
cd ~/isaacsim
./isaac-sim.sh
```

| 순서 | 동작 | 값 |
|---|---|---|
| 1 | `File > New` | 저장할지 물으면 Don't Save |
| 2 | `File > Import` | |
| 3 | 위 경로 칸에 입력 → Enter | `~/nvidia-dli-lecture/work/pinky_description/urdf/` |
| 4 | `pinky.urdf` 클릭 | 오른쪽에 옵션이 나옴 |
| 5 | Links | `Moveable Base` |
| 6 | Joint Configuration | `Natural Frequency` (기본값) |
| 7 | 관절 표의 Target | `l_wheel_joint` · `r_wheel_joint` = `Velocity`, `caster_rotate_joint` · `caster_wheel_joint` = `None` |
| 8 | `Import` → 저장 위치 확인 창 | `Yes` |

## 4. 화면에서 누르는 순서: 바닥 · 모터

| 순서 | 동작 | 값 |
|---|---|---|
| 9 | `Create > Environments > Flat Grid` | |
| 10 | Stage 의 `Environment` 눈 아이콘 끄기 | |
| 10-1 | `physicsScene` 선택 → Property 의 Solver Type | `PGS` |
| 11 | `pinky` 선택 → Translate Z | `0.05` |
| 12 | ▶ Play 2초 → ■ Stop | 수평으로 섬 (`base_link` 높이 0.028 m) |
| 13 | `pinky > joints > l_wheel_joint` 클릭, `Ctrl` + `r_wheel_joint` 클릭 → Drive 의 Target Velocity | `360` |
| 14 | ▶ Play 2초 → ■ Stop → 두 Target Velocity | 앞으로 약 0.35 m → `0` |
| 15 | `Tools > Robotics > OmniGraph Controllers > Differential Controller` | 아래 표 → OK |
| 16 | Stage 의 `Graphs > differential_controller > DifferentialController` 선택 → maxLinearSpeed | `0.2` |
| 17 | maxAngularSpeed | `2.0` |
| 18 | ▶ Play → 뷰포트 클릭 → W · A · S · D → ■ Stop | W 3초에 약 0.6 m |
| 19 | `File > Save As` | `~/nvidia-dli-lecture/work/pinky_gui.usd` |

### Differential Controller 창에 넣을 값

| 칸 | 값 |
|---|---|
| Robot Prim | `/World/pinky` |
| wheel radius | `0.028` |
| distance between wheels | `0.0961` |
| Right Joint Name | `l_wheel_joint` |
| Left Joint Name | `r_wheel_joint` |
| Use Keyboard Control (WASD) | 체크 |

- **Solver Type 을 PGS 로 바꾸는 이유**: 기본 솔버 TGS 에서는 이 로봇이 서 있는 동안에도 1초에 0.26° 씩 방향이 틀어짐 (강의 PC 측정, PGS 에서는 0). 두 솔버의 차이는 강의 글 6절에 있음. 코드(`import_pinky.py`)는 자동으로 PGS 를 씀.

- **주의 (Isaac Sim 5.1)**: Right 칸에 `l_wheel_joint`, Left 칸에 `r_wheel_joint` 를 넣음. 칸 이름대로 넣으면 회전 방향이 반대가 됨 (강의 5 · 9 와 같은 문제).

## 5. 같은 일을 코드로

- Isaac Sim 창을 닫은 뒤 실행함.

```bash
cd ~/nvidia-dli-lecture
~/isaacsim/python.sh 11_pinky_import/import_pinky.py
```

```text
가져온 로봇: /World/pinky, 강체 링크 11개
강체 링크                그 아래 센서 자리 (Xform)
base_footprint       -
base_link            imu_link, ir_l_link, ir_mid_link, ir_r_link, ultrasonic_link
caster_mount_link    -
caster_roate_link    -
caster_wheel         -
front_camera_mount   front_camera_link
l_wheel              -
r_wheel              -
robot_lamp           -
rplidar_mount        rplidar_link
screen_mount         -
회전 관절                  축   Stiffness   Damping
caster_rotate_joint    Z        0.00     0.000
caster_wheel_joint     Y        0.00     0.000
l_wheel_joint          Y        0.00     0.443
r_wheel_joint          Y        0.00     0.443
   시간[s]     x[m]     y[m]     z[m]    방향[°]
     0.0    0.000    0.000    0.028      0.0
     0.5    0.041    0.000    0.028     -0.1
     1.0    0.090    0.000    0.028     -0.1
     1.5    0.140    0.000    0.028     -0.1
     2.0    0.190    0.000    0.028     -0.1
     2.5    0.240    0.000    0.028     -0.1
     3.0    0.290   -0.001    0.028     -0.1
결과: v = 0.1 m/s, ω = 0 rad/s 로 3초 → 이동 (0.290, -0.001) m, 회전 -0.1°.
```

- 앞부분에는 2절의 URDF 표가 먼저 나옴.

## 6. 바꿔 보기

```bash
~/isaacsim/python.sh 11_pinky_import/import_pinky.py --wheels 360 360 --seconds 2
~/isaacsim/python.sh 11_pinky_import/import_pinky.py --linear 0 --angular 1
~/isaacsim/python.sh 11_pinky_import/import_pinky.py --linear 1
~/isaacsim/python.sh 11_pinky_import/import_pinky.py --keyboard
```

| 옵션 | 결과 |
|---|---|
| (없음) v = 0.1 m/s | 3초에 0.290 m, 방향 변화 −0.1° |
| `--wheels 360 360 --seconds 2` | 2초에 0.337 m (계산: 1초에 한 바퀴 = 2π × 0.028 m × 2 = 0.352 m) |
| `--linear 0 --angular 1` | 3초에 왼쪽으로 166.3° (계산 171.9°) |
| `--linear 1` | 0.2 m/s 로 제한되어 3초에 0.580 m |
| `--keyboard` | 창을 띄우고 W A S D 로 조종 |

## 막힐 때

| 증상 | 해결 |
|---|---|
| `URDF 파일이 없습니다` | 1절을 먼저 실행 |
| `ROS 2 Jazzy 가 없습니다` | 강의 7 의 1절(ROS 2 설치) |
| `colcon: command not found` · `xacro: command not found` | 1절 ①의 `sudo apt install` |
| `source` 에서 오류 (zsh) | `setup.bash` 대신 `setup.zsh` |
| 가져온 로봇이 너무 작게 보임 | 실제 크기가 손바닥만 함 (바퀴 지름 5.6 cm). Stage 에서 `pinky` 를 고르고 뷰포트에서 `F` 키 |
| A 를 눌렀는데 오른쪽으로 돎 | 제어기 창의 Right · Left 칸을 서로 바꿔 넣었는지 확인 |
| 서 있는데 방향이 조금씩 틀어짐 (1분에 약 16°) | `physicsScene` 의 Solver Type 이 기본값 TGS → `PGS` 로 (순서 10-1) |

## 옵션 전체

| 옵션 | 뜻 |
|---|---|
| `--urdf 경로` | 가져올 URDF (기본: `work/pinky_description/urdf/pinky.urdf`) |
| `--read-only` | URDF 만 읽고 끝냄 |
| `--wheels L R` | 제어기 없이 왼 · 오 바퀴 Target Velocity [°/s] |
| `--linear V` · `--angular W` | 직진 속도 [m/s] · 회전 속도 [rad/s] |
| `--seconds S` | 시뮬레이션할 시간 (기본 3) |
| `--keyboard` | 창을 띄우고 W A S D 로 조종 |
| `--gui` | 창을 띄움 |
| `--save USD` | Play 전 장면을 저장 |

- 다음: [12. Pinky Pro 센서](../12_pinky_sensors/README.md)
