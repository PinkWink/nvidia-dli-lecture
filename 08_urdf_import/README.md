# 8. URDF 로봇 가져오기: carter.urdf 읽기 · URDF Importer — 명령 모음

- 영상에서 입력하는 터미널 명령과 화면에 넣는 값을 순서대로 모았음. 강의 4 ~ 7 에서는 로봇을 손으로 만들었고, 여기서는 설계도 파일(URDF)로 가져옴.

## 1. Carter URDF 준비

- Isaac Sim 안에 들어 있는 예제 로봇 Carter 의 URDF 를 작업 폴더로 복사함.

```bash
cd ~/nvidia-dli-lecture
git pull
mkdir -p work
cp -r ~/isaacsim/exts/isaacsim.asset.importer.urdf/data/urdf/robots/carter work/
ls work/carter work/carter/urdf work/carter/meshes
```

```text
work/carter:
meshes  urdf

work/carter/meshes:
caster_wheel.obj  chassis.obj  pivot.obj  side_wheel.obj

work/carter/urdf:
carter.urdf
```

## 2. URDF 읽기

```bash
grep -n "<link\|<joint" work/carter/urdf/carter.urdf
```

```bash
~/isaacsim/python.sh 08_urdf_import/import_carter.py --read-only
```

- 링크 7개 · 관절 6개의 표가 나옴 (아래 4절 출력의 앞부분과 같음). Isaac Sim 을 켜지 않아 바로 끝남.

## 3. 화면에서 누르는 순서

```bash
cd ~/isaacsim
./isaac-sim.sh
```

| 순서 | 동작 | 값 |
|---|---|---|
| 1 | `File > New` | 저장할지 물으면 Don't Save |
| 2 | `File > Import` | |
| 3 | 위 경로 칸에 입력 → Enter | `~/nvidia-dli-lecture/work/carter/urdf/` |
| 4 | `carter.urdf` 클릭 | 오른쪽에 옵션이 나옴 |
| 5 | Links | `Moveable Base` |
| 6 | Joint Configuration | `Natural Frequency` (기본값) |
| 7 | 관절 표의 Target | `left_wheel` · `right_wheel` = `Velocity`, `rear_axle` · `rear_pivot` = `None` |
| 8 | `Import` → 저장 위치 확인 창 | `Yes` |
| 9 | Stage 에서 `carter` → `joints` 펼치기 | 관절 6개 |
| 10 | `File > Save As` | `~/nvidia-dli-lecture/work/carter_gui.usd` |

- 가져온 결과는 `work/carter/urdf/carter/carter.usd` 로도 저장됨.

## 4. 같은 가져오기를 코드로

- Isaac Sim 창을 닫은 뒤 실행함.

```bash
cd ~/nvidia-dli-lecture
~/isaacsim/python.sh 08_urdf_import/import_carter.py
```

```text
URDF: /home/pw/nvidia-dli-lecture/work/carter/urdf/carter.urdf
로봇 이름: carter, 링크 7개, 관절 6개
링크                   질량[kg]  충돌 모양
chassis_link          45.88  상자 (.55 .40 .54 m)
left_wheel_link    4.195729  원기둥 (반지름 0.240 m, 폭 0.1 m)
right_wheel_link   4.195729  원기둥 (반지름 0.240 m, 폭 0.1 m)
rear_pivot_link    0.579519  상자 (.09 .08 .12 m)
rear_wheel_link        1.25  원기둥 (반지름 0.076 m, 폭 0.04 m)
com_offset             20.0  상자 (.25 .25 .1 m)
imu                      .1  상자 (.1 .1 .1 m)
관절           종류          부모 → 자식                              축       위치 (x y z)
com_joint    fixed       chassis_link → com_offset            0 1 0   -.3 0 0
imu_joint    fixed       chassis_link → imu                   0 1 0   0 0 0
left_wheel   continuous  chassis_link → left_wheel_link       0 1 0   0 0.31420517 0
right_wheel  continuous  chassis_link → right_wheel_link      0 1 0   0 -0.31420517 0
rear_pivot   continuous  chassis_link → rear_pivot_link       0 0 1   -0.31613607 0 -.0758
rear_axle    continuous  rear_pivot_link → rear_wheel_link    0 1 0   -.03809695 0 -.08961022
가져온 로봇: /World/carter, 링크(강체) 7개: chassis_link, com_offset, imu, left_wheel_link, rear_pivot_link, rear_wheel_link, right_wheel_link
Articulation Root: /World/carter/chassis_link
관절           USD 종류                 축   Body 0 → Body 1                      Stiffness   Damping
com_joint    PhysicsFixedJoint      -   chassis_link → com_offset                    -         -
imu_joint    PhysicsFixedJoint      -   chassis_link → imu                           -         -
left_wheel   PhysicsRevoluteJoint   Y   chassis_link → left_wheel_link             0.0     307.9
rear_pivot   PhysicsRevoluteJoint   Z   chassis_link → rear_pivot_link             0.0       0.0
rear_axle    PhysicsRevoluteJoint   Y   rear_pivot_link → rear_wheel_link          0.0       0.0
right_wheel  PhysicsRevoluteJoint   Y   chassis_link → right_wheel_link            0.0     307.9
   시간[s]     몸체 높이 z[m]
     0.0          0.300
     0.5          0.240
     1.0          0.240
     1.5          0.240
     2.0          0.240
결과: 바퀴로 바닥에 섰습니다. 몸체 높이 0.24 m = 바퀴 반지름.
```

- 이 표들 앞뒤로 `[Warning]` 등 로그가 함께 나옴. 정상임.

- 바퀴 Damping 307.9 = Natural Frequency 25 의 제곱(625) × 관절이 돌리는 관성(0.493). 화면의 Property 에 보이는 값과 같음.

## 5. 바꿔 보기

```bash
~/isaacsim/python.sh 08_urdf_import/import_carter.py --fixed-base
~/isaacsim/python.sh 08_urdf_import/import_carter.py --gui
~/isaacsim/python.sh 08_urdf_import/import_carter.py --save work/carter_scene.usd
```

| 옵션 | 결과 |
|---|---|
| (없음) Moveable Base | 0.3 m 에서 떨어져 바퀴로 섬 (몸체 높이 0.24 m) |
| `--fixed-base` Static Base | 몸체가 0.30 m 에 고정됨 |
| `--gui` | 창으로 보기 (창을 닫으면 끝) |
| `--save 경로` | Play 전 장면을 USD 로 저장 |

## 막힐 때

| 증상 | 해결 |
|---|---|
| `URDF 파일이 없습니다` | 1절의 `cp -r` 명령을 먼저 실행 |
| Import 창에 `carter.urdf` 가 안 보임 | 경로 칸 끝에 `/` 까지 넣고 Enter. 오른쪽 아래 파일 종류가 All Supported Files 인지 확인 |
| 가져온 로봇이 바닥 없이 떠 있음 | 정상. 바닥은 강의 9 에서 넣음 |
| 덮어쓸지 묻는 창 (URDF Confirm Overwrite) | 이미 가져온 적이 있음 → `Yes` |

## 옵션 전체

| 옵션 | 뜻 |
|---|---|
| `--urdf 경로` | 가져올 URDF (기본: `work/carter/urdf/carter.urdf`) |
| `--read-only` | URDF 만 읽고 끝냄 |
| `--fixed-base` | Static Base 로 가져옴 |
| `--seconds S` | 시뮬레이션할 시간 (기본 2) |
| `--gui` | 창을 띄움 |
| `--save USD` | Play 전 장면을 저장 |

- 다음: [9. 가져온 로봇 몰기](../09_carter_drive/README.md)
