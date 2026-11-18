# nvidia-dli-lecture

NVIDIA Isaac Sim 온라인 강의(YouTube)의 예제 코드입니다. 강의가 진행되는 대로 폴더가 추가됩니다.

## 기준 환경

| 항목 | 버전 |
|---|---|
| 운영체제 | Ubuntu 24.04 |
| Isaac Sim | 5.1.0 (`~/isaacsim` 에 설치) |
| NVIDIA 드라이버 | 580 이상 |
| GPU | RTX 계열, VRAM 16 GB 권장 |
| ROS 2 | Jazzy (강의 7 부터) |

모든 코드는 위 환경에서 실제로 실행해 확인했습니다.

## 받기

```bash
cd ~
git clone https://github.com/PinkWink/nvidia-dli-lecture.git
cd ~/nvidia-dli-lecture
```

새 강의의 코드는 `git pull` 로 받습니다.

## 강의별 명령 모음

영상에서 입력하는 터미널 명령은 강의마다 아래 문서에 순서대로 모아 두었습니다. 영상을 보면서 복사해 붙여 넣으세요.

| 강의 | 명령 모음 | 코드 |
|---|---|---|
| 1. Isaac Sim 5.1 설치 | [01_install/README.md](01_install/README.md) | `check_env.sh` PC 점검 · `install_isaacsim.sh` 내려받기와 설치 · `verify_install.sh` 설치 확인 |
| 2. Isaac Sim 실행과 화면 구성 | [02_launch/README.md](02_launch/README.md) | `hello_isaacsim.py` Python 으로 Isaac Sim 켜기 |
| 3. 첫 장면: 큐브를 떨어뜨리며 배우는 물리 기초 | [03_first_scene/README.md](03_first_scene/README.md) | `falling_cube.py` 강체 · 충돌체 · 바닥을 코드로 |
| 4. 로봇 만들기: 몸체 · 바퀴 · 회전 관절 · 구동 | [04_build_robot/README.md](04_build_robot/README.md) | `build_robot.py` 몸체 · 바퀴 · 관절 · 구동을 코드로 |
| 5. 로봇 조종: Articulation Root · Differential Controller · 키보드 | [05_control/README.md](05_control/README.md) | `drive_robot.py` 속도 명령 · 키보드 조종 그래프 |
| 6. 센서 달기: 카메라 · 2D 라이다 | [06_sensors/README.md](06_sensors/README.md) | `add_sensors.py` 카메라 사진 · 라이다 거리 |
| 7. ROS 2 로 데이터 보내기: LaserScan · 카메라 · RViz2 | [07_ros2/README.md](07_ros2/README.md) | `publish_ros2.py` 토픽 내보내기 · `rviz/simple_robot.rviz` |

강의 4 ~ 7 은 로봇 하나(SimpleRobot)를 단계별로 키웁니다. 뒤 강의의 스크립트는 앞 강의의 스크립트를 불러다 쓰고, 함께 쓰는 도구는 `common/sim.py` 에 있습니다.

## 한눈에 보기

```bash
# 1. 내 PC 점검, Isaac Sim 설치
bash 01_install/check_env.sh
bash 01_install/install_isaacsim.sh

# 2. Python 으로 Isaac Sim 켜기
~/isaacsim/python.sh 02_launch/hello_isaacsim.py

# 3. 떨어지는 큐브
~/isaacsim/python.sh 03_first_scene/falling_cube.py
~/isaacsim/python.sh 03_first_scene/falling_cube.py --no-ground --seconds 1
~/isaacsim/python.sh 03_first_scene/falling_cube.py --gui

# 4 ~ 6. 로봇 만들기 → 조종 → 센서
~/isaacsim/python.sh 04_build_robot/build_robot.py
~/isaacsim/python.sh 05_control/drive_robot.py --keyboard
~/isaacsim/python.sh 06_sensors/add_sensors.py

# 7. ROS 2 (ROS 2 Jazzy 를 먼저 source)
source /opt/ros/jazzy/setup.bash
~/isaacsim/python.sh 07_ros2/publish_ros2.py
```

- Python 스크립트는 시스템의 `python3` 가 아니라 Isaac Sim 에 들어 있는 `~/isaacsim/python.sh` 로 실행합니다.
- Isaac Sim 을 다른 곳에 설치했다면 `~/isaacsim` 자리에 그 경로를 씁니다. 셸 스크립트는 `ISAACSIM_PATH=/경로 bash ...` 로 지정합니다.
- 내가 만든 장면은 `work/` 에 저장합니다 (git 이 무시하는 폴더).
