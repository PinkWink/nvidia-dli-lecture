# 37. PL-2M 가져오기: 이동형 양팔 로봇을 Isaac Sim 으로 — 명령 모음

- 영상에서 입력하는 터미널 명령과 기대 출력을 순서대로 모았음. [36. ROS 2 로 시뮬레이터 조종하기](../36_auto_test/README.md) 다음 내용임.

| 파일 |
|---|
| [import_pl2m.py](import_pl2m.py) |
| [prepare_pl2m.sh](prepare_pl2m.sh) |

## 1. PL-2M

### 핑크랩의 이동형 양팔 로봇

- PL-2M 은 핑크랩(PinkLAB)의 이동형 양팔 로봇임. 바퀴로 다니는 베이스 위에 팔 두 개와 그리퍼가 있음.
- 저장소 (이 강의에서 쓰는 URDF 의 출처): [https://github.com/PinkWink/pinklab_robotworld2026](https://github.com/PinkWink/pinklab_robotworld2026)
- 로보월드 2026 핑크랩 부스를 MuJoCo 로 옮긴 저장소임. 이 강의부터는 같은 로봇과 부스를 Isaac Sim 으로 옮김.

### 로봇의 구성

| 부분 | 설계 패키지 | 내용 |
|---|---|---|
| 베이스 | `vicpinky_description` | 구동 바퀴 2 + 캐스터 4, 라이다 자리 |
| 양팔 | `openarm_description` | 관절 7개짜리 팔 2개 (OpenArm v1.0) |
| 그리퍼 | `openarm_description` | 손가락 두 개가 마주 보고 움직이는 평행 그리퍼 2개 |
| 합친 것 | `mobile_openarm_description` | 위 둘을 잇는 판, 카메라 자리 4개, 로고 |

- 크기는 가로 0.66 m, 세로 0.50 m, 높이 1.11 m 이고 질량은 90.2 kg 임 (URDF 기준).
- `openarm_description` 과 `mobile_openarm_description` 은 Apache-2.0 라이선스임.

### 강의 37 부터 할 일

| 강의 | 내용 |
|---|---|
| 37 (이번) | URDF 를 만들어 Isaac Sim 으로 가져오고 바닥에 세우기 |
| 다음 | 바퀴로 다니기, 팔과 그리퍼를 제대로 움직이게 구동 값 정하기 |
| 그 뒤 | 부스 환경 만들기 → ROS 2 와 MoveIt 으로 팔의 경로 계획 → 물체 집기 |

- 강의 11 의 Pinky Pro 와 같은 순서임. 관절이 20개로 늘고 팔이 생긴 것이 다름.
- 뒤 강의의 순서는 진행하면서 바뀔 수 있음.

## 2. URDF 만들기

### 터미널: 스크립트로 한 번에 준비

```bash
cd ~/nvidia-dli-lecture
git pull
bash 37_pl2m_import/prepare_pl2m.sh
```

- 필요한 것: 강의 7 의 ROS 2 Jazzy, 강의 11 에서 설치한 `git` · `colcon` · `xacro`.
- 저장소 받기 → 설계 패키지 빌드 → URDF 만들기를 한 번에 함.

### 실행 결과

```text
[1/3] 저장소 받기: /home/pw/pinklab_robotworld2026
Cloning into '/home/pw/pinklab_robotworld2026'...
[2/3] 설계 패키지 빌드: vicpinky_description openarm_description mobile_openarm_description
...
Summary: 3 packages finished [1.35s]
[3/3] xacro → URDF
  링크 47개, 관절 46개

준비 완료: /home/pw/nvidia-dli-lecture/work/pl2m/pl2m.urdf
```

- 받은 저장소는 `~/pinklab_robotworld2026` 에 있음. 강의 PC 에서 빌드까지 마친 크기가 564 MB 였음 (모양 파일이 대부분).

### 직접 하기 ①: 저장소 받기

```bash
git clone --depth 1 https://github.com/PinkWink/pinklab_robotworld2026.git ~/pinklab_robotworld2026
```

- `--depth 1` = 지난 기록은 빼고 지금 상태만 받음.

### 직접 하기 ②: 설계 패키지 세 개만 빌드

```bash
cd ~/pinklab_robotworld2026
source /opt/ros/jazzy/setup.bash
colcon build --packages-select vicpinky_description openarm_description mobile_openarm_description
source install/setup.bash
```

- 저장소에는 MuJoCo · Nav2 · MoveIt 패키지도 있지만, URDF 를 만드는 데는 설계 패키지 세 개면 됨.

### 직접 하기 ③: xacro → URDF

```bash
mkdir -p ~/nvidia-dli-lecture/work/pl2m
xacro src/mobile_openarm_description/urdf/mobile_openarm.urdf.xacro ros2_control:=false \
  | sed "s|package://\([A-Za-z0-9_]*\)/|$HOME/pinklab_robotworld2026/src/\1/|g" > ~/nvidia-dli-lecture/work/pl2m/pl2m.urdf
```

- `sed` 는 모양 파일의 위치를 `package://패키지/...` 에서 실제 폴더 경로로 바꿔 적음. 패키지가 세 개라 Isaac Sim 이 위치를 찾지 못할 수 있어서임.
- `ros2_control:=false` = 실물 모터 제어 설정을 빼고 로봇 모양과 관절만 꺼냄.

## 3. URDF 읽기

### 터미널

```bash
cd ~/nvidia-dli-lecture
~/isaacsim/python.sh 37_pl2m_import/import_pl2m.py --read-only
```

### 실행 결과: 부분별 개수

```text
로봇 이름: mobile_openarm, 링크 47개, 관절 46개 (움직이는 관절 20개), 질량 합 90.2 kg
부분       링크   질량[kg]  움직이는 관절
베이스      12     64.2        2
양팔       18     25.5       14
그리퍼       8      0.4        4
카메라       9      0.0        0
```

- 질량의 대부분(64.2 kg)이 베이스에 있음. 무거운 베이스가 팔을 뻗어도 넘어지지 않게 받쳐 줌.
- 카메라 링크 9개는 질량도 모양도 없는 자리 표시임.

### 실행 결과: 움직이는 관절 20개

```text
움직이는 관절                      종류          축        범위                  힘 한계  비고
left_wheel_joint             continuous  0 0 1    제한 없음                  -
right_wheel_joint            continuous  0 0 -1   제한 없음                  -
openarm_left_joint1          revolute    0 0 1    -200 ~ 80°            40
openarm_left_joint2          revolute    -1 0 0   -190 ~ 10°            40
openarm_left_joint3          revolute    0 0 1    -90 ~ 90°             27
openarm_left_joint4          revolute    0 1 0    0 ~ 140°              27
openarm_left_joint5          revolute    0 0 1    -90 ~ 90°              7
openarm_left_joint6          revolute    1 0 0    -45 ~ 45°              7
openarm_left_joint7          revolute    0 -1 0   -90 ~ 90°              7
...
openarm_left_finger_joint1   prismatic   0 -1 0   0 ~ 44 mm            333
openarm_left_finger_joint2   prismatic   0 1 0    0 ~ 44 mm            333  openarm_left_finger_joint1 을 따라 움직임
```

- 오른팔(`openarm_right_...`)도 같은 구성이라 줄였음.
- 팔 관절은 어깨 쪽(40 N·m)이 힘이 세고 손목 쪽(7 N·m)이 약함.
- 손가락은 직선으로 움직이는 관절(prismatic)임. 두 번째 손가락은 첫 번째를 **따라 움직이는** 관절(mimic)임.

## 4. Isaac Sim 으로 가져오기

### 준비

```bash
cd ~/isaacsim
./isaac-sim.sh
```

- 강의 11 의 4절과 같은 순서임. 다른 곳은 파일 이름과 관절 표뿐임.

### File > New 로 새 장면

### File > Import 누르기

### 위 경로 칸에 ~/nvidia-dli-lecture/work/pl2m/ 입력 → Enter

- `~` 는 내 홈 폴더로 바꿔 적음 (예: `/home/내이름/nvidia-dli-lecture/work/pl2m/`).

### pl2m.urdf 클릭 — 오른쪽에 옵션이 나옴

### Links: Moveable Base 선택

- 바퀴로 다니는 로봇이라 바닥에 고정하지 않음.

### 관절 표의 left_wheel_joint: Target 칸을 눌러 Velocity

### left_wheel_joint = Velocity

- 바퀴는 각도가 아니라 속도로 구동함. 팔과 손가락은 Position 그대로 둠.

### 관절 표를 아래로 내리기

- 표는 다섯 줄씩만 보임. `right_wheel_joint` 는 스무 번째 줄에 있음.

### right_wheel_joint = Velocity

### Import 누르기

### 저장 위치 확인 창 → Yes

- 가져온 결과(USD)를 URDF 옆 `pl2m` 폴더에 저장한다는 확인임.

### /World/mobile_openarm 이 생김

- 이름은 URDF 의 로봇 이름(`mobile_openarm`)을 따름.
- 아직 바닥도 조명도 없어 어둡게 보임.

## 5. 바닥에 내려놓기

### Create > Environments > Flat Grid

### Stage 의 Environment 눈 아이콘 끄기

- Flat Grid 에 조명이 들어 있어, 새 장면의 기본 조명은 끔 (강의 11 과 같음).

### mobile_openarm 선택 → Translate Z = 0.05

### Play 3 초 — 그대로 서 있음

- 5 cm 위에서 내려앉아 `base_link` 높이 0.087 m 에 섬.
- 물리 솔버는 기본값(TGS) 그대로임. Pinky Pro 에서 본 "서 있는데 조금씩 도는" 현상은 5초 동안 없었음 (이동 0.0 mm, 회전 0.00°).

## 6. 가져온 결과 읽기

### 터미널: 같은 일을 코드로

```bash
cd ~/nvidia-dli-lecture
~/isaacsim/python.sh 37_pl2m_import/import_pl2m.py
```

- 4 · 5절에서 창으로 한 일을 명령으로 함. 가져온 뒤 결과를 표로 출력하고 5초 동안 서 있는지 봄.
- 코드에서는 로봇 이름을 `/World/pl2m` 으로 바꿈.

### 실행 결과: 강체와 관절

```text
가져온 로봇: /World/pl2m
강체 링크 33개 (질량 합 90.2 kg), 강체가 되지 않은 빈 링크 10개
관절 32개 = 움직이는 관절 20개 + 고정 관절 12개, Articulation Root = base_footprint
```

- URDF 의 링크 47개 중 질량이 없는 빈 링크는 강체가 되지 않고, 부모 링크 아래 Xform 으로 들어옴 (강의 11 과 같음).
- 질량 합 90.2 kg 은 URDF 와 같음.

### 실행 결과: 관절의 구동 값

```text
움직이는 관절                      축    Stiffness   Damping  Max Force  비고
left_wheel_joint             Z         0.00     2.151      제한 없음
openarm_left_joint1          Z       432.07     0.173       40.0
openarm_left_joint2          X       388.68     0.155       40.0
openarm_left_joint3          Z       380.71     0.152       27.0
openarm_left_joint4          Y       250.83     0.100       27.0
openarm_left_joint5          Z        85.22     0.034        7.0
openarm_left_joint6          X        31.11     0.012        7.0
openarm_left_joint7          Y         5.56     0.002        7.0
openarm_left_finger_joint1   Y         0.51     0.000      333.0
openarm_left_finger_joint2   Y            -         -          -  openarm_left_finger_joint1 을 따라 움직임 (PhysxMimicJointAPI:rotY)
...
right_wheel_joint            Z         0.00     2.151      제한 없음
```

- 창으로 가져온 결과와 같은 값임 (강의 PC 에서 관절 20개 모두 대조).
- Max Force 는 URDF 의 힘 한계가 그대로 들어온 것임.

### Stiffness 는 누가 정했나

- URDF 에는 구동의 세기가 없음. Importer 가 **Natural Frequency** 방식으로 정함: Stiffness = 관절이 돌리는 관성 × 25².
- 그래서 무거운 것을 돌리는 어깨(432)는 크고, 가벼운 손목(5.56)과 손가락(0.51)은 작음.
- 로봇이 "움직이기는 하게" 하는 기본값임. 실제 모터에 맞춘 값은 다음 강의에서 정함.

### 따라 움직이는 관절

- 두 번째 손가락 관절에는 구동(Drive)이 없고 `PhysxMimicJointAPI` 가 붙음.
- 첫 번째 손가락 관절만 움직이면 두 번째가 따라 움직여, 두 손가락이 함께 벌어지고 닫힘.
- 이번 강의에서는 들어온 것만 확인했음. 실제로 따라 움직이는지는 다음 강의에서 봄.

## 7. 팔꿈치 굽혀 보기

### joints > openarm_left_joint4 클릭

- Stage 에서 mobile_openarm > joints 를 펼치면 관절 32개가 보임. `openarm_left_joint4` 가 왼팔 팔꿈치임.

### Property: Drive 의 Target Position = 90

- 아래에 Importer 가 정한 Stiffness 250.83 과 Damping 0.100 이 보임.

### Play 3 초 — 왼팔이 앞으로 굽음

- 관절의 실제 각도는 90.0° 였음. 베이스는 제자리였고 방향만 0.1° 달라졌음.

### 코드로

```bash
~/isaacsim/python.sh 37_pl2m_import/import_pl2m.py --elbow 90 --seconds 6
```

```text
   시간[s]     x[m]     y[m]     z[m]    방향[°]    왼팔 팔꿈치[°]
     0.0    0.000    0.000    0.087      0.0          0.0
     1.0    0.000    0.000    0.087      0.0         90.0
...
     6.0    0.000    0.000    0.087      0.0         90.0
결과: 6초 동안 이동 0.1 mm, 회전 -0.00°, 왼팔 팔꿈치 90.0°.
```

## 8. 코드 읽기

### 가져오기

```python
_, robot = omni.kit.commands.execute("URDFParseFile", urdf_path=urdf_path, import_config=config)
for name in robot.joints:
    joint = robot.joints[name]
    if joint.type == _urdf.UrdfJointType.JOINT_FIXED:
        continue
    drive = joint.drive
    drive.set_drive_type(_urdf.UrdfJointDriveType.JOINT_DRIVE_FORCE)             # Drive Type: Force (창의 기본값)
    drive.strength = interface.compute_natural_stiffness(robot, name, drive.natural_frequency)
    if name in WHEEL_JOINTS:
        drive.set_target_type(_urdf.UrdfJointTargetType.JOINT_DRIVE_VELOCITY)    # 바퀴: Target = Velocity
    else:
        drive.damping = 2 * joint.inertia * drive.natural_frequency * drive.damping_ratio
```

- `37_pl2m_import/import_pl2m.py` 의 `import_pl2m` 임 (일부만 보였음). 창의 관절 표가 하는 계산을 그대로 적었음.
- Drive Type 을 Force 로 먼저 정해야 창과 같은 값이 나옴. 빠뜨리면 모든 관절의 Stiffness 가 625 로 들어옴 (강의 PC 에서 확인).

### 다른 실행 방법

```bash
~/isaacsim/python.sh 37_pl2m_import/import_pl2m.py --gui                              # 창으로 보기
~/isaacsim/python.sh 37_pl2m_import/import_pl2m.py --save work/pl2m/pl2m_scene.usd    # 장면 저장
```

- 저장한 장면은 강의 PC 에서 112 MB 였음.

## 9. 정리

### 기억할 것

- PL-2M 은 설계 패키지 세 개(베이스 · 양팔 · 합친 것)에서 URDF 한 개를 만들어 가져옴.
- 가져오는 순서는 Pinky Pro 와 같음: Moveable Base, 바퀴만 Velocity, 나머지는 Position.
- 링크 47개 중 33개가 강체가 되고, 움직이는 관절은 20개임 (바퀴 2 + 팔 14 + 손가락 4).
- 구동 값은 Importer 가 관성에 비례해 정한 기본값임. 손목과 손가락은 아주 약함.
- 기본 솔버(TGS)에서 5초 동안 그대로 서 있었음.

### 한계

- 서 있기와 왼팔 팔꿈치 하나만 확인했음. 바퀴로 다니기, 다른 관절, 그리퍼는 아직 움직여 보지 않았음.
- 캐스터 4개는 URDF 에서 고정된 공 모양임. 바닥과의 마찰이 주행에 어떤 영향을 주는지는 다음 강의에서 확인함.
- 서 있는지는 5 ~ 6초만 보았음. 더 오래 두었을 때는 확인하지 않았음.
- 강의 PC 한 대(Isaac Sim 5.1)에서 확인한 결과임.
