# 23. SIL · HIL 과 Script Editor: 코드로 장면 다루기 — 명령 모음

영상에서 입력하는 터미널 명령과 Script Editor 에 붙여 넣는 코드를 순서대로 모았습니다. [22. 장소 이름으로 명령하기](../22_city_mission/README.md) 다음 내용입니다.

| 파일 | 내용 |
|---|---|
| [01_usd_api.py](01_usd_api.py) | USD API 로 바닥과 떨어지는 상자 만들기 |
| [02_core_api.py](02_core_api.py) | Core API 로 같은 장면 만들기 |
| [03_drive_pinky.py](03_drive_pinky.py) | 열려 있는 Pinky Pro 장면의 바퀴 속도 바꾸기 |

세 파일은 `python.sh` 로 실행하는 파일이 아니라, **Isaac Sim 의 Script Editor 에 붙여 넣는 코드**입니다. Isaac Sim 5.1 의 Script Editor 는 한글을 `??` 로 표시하므로 주석과 출력 문장을 영어로 썼습니다.

## 1. Script Editor 열기

```bash
cd ~/isaacsim
./isaac-sim.sh
```

| 순서 | 동작 | 값 |
|---|---|---|
| 1 | `Window > Script Editor` | |
| 2 | 아래 칸에 입력하고 `Run` (또는 `Ctrl` + `Enter`) | `robot_name = "Pinky Pro"` |
| 3 | Script Editor 의 `Tab > Add Tab` | |
| 4 | 새 탭에 입력하고 `Run` | `print(robot_name)` → 출력 `Pinky Pro` |

## 2. USD API 로 상자 떨어뜨리기

| 순서 | 동작 |
|---|---|
| 5 | `File > New` (저장할지 물으면 `Don't Save`) |
| 6 | [01_usd_api.py](01_usd_api.py) 의 내용을 붙여 넣고 `Run` |
| 7 | `Play` → 상자가 떨어짐 → `Stop` |

기대 출력:

```text
created: ['/World/PhysicsScene', '/World/Ground', '/World/Cube']
```

강의 PC 측정: 상자 중심 높이 1.0 m → 0.2 m.

## 3. Core API 로 같은 일 하기

| 순서 | 동작 |
|---|---|
| 8 | `File > New` |
| 9 | [02_core_api.py](02_core_api.py) 의 내용을 붙여 넣고 `Run` |
| 10 | `Play` → 상자가 떨어짐 → `Stop` |

기대 출력:

```text
cube position: [0. 0. 1.]
```

## 4. Pinky Pro 의 바퀴 속도를 코드로 바꾸기

새 터미널에서 장면 파일을 만듭니다 ([11. Pinky Pro 가져오기](../11_pinky_import/README.md) 의 준비가 되어 있어야 합니다).

```bash
cd ~/nvidia-dli-lecture
~/isaacsim/python.sh 11_pinky_import/import_pinky.py --wheels 0 0 --seconds 0.5 --save work/pinky_scene.usd
```

```text
장면 저장: /home/내이름/nvidia-dli-lecture/work/pinky_scene.usd
결과: 왼쪽 0 °/s, 오른쪽 0 °/s 로 0.5초 → 이동 (0.000, 0.000) m, 회전 +0.0°.
```

| 순서 | 동작 |
|---|---|
| 11 | Isaac Sim 창에서 `File > Open` → `~/nvidia-dli-lecture/work/pinky_scene.usd` |
| 12 | `Play` (로봇은 서 있음) |
| 13 | [03_drive_pinky.py](03_drive_pinky.py) 의 내용을 붙여 넣고 `Run` → 로봇이 오른쪽으로 원을 그림 |
| 14 | `LEFT, RIGHT` 숫자를 바꿔 다시 `Run` |

기대 출력:

```text
l_wheel_joint target 360.0 deg/s
r_wheel_joint target 180.0 deg/s
robot position: x=0.000 y=-0.000 z=0.028 m
```

강의 PC 측정 (`360.0, 180.0`): 1초에 약 56° 씩 돌며 지름 약 0.28 m 의 원.
