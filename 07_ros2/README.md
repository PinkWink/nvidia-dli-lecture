# 7. ROS 2 로 데이터 보내기: LaserScan · 카메라 · RViz2 — 명령 모음

영상에서 입력하는 터미널 명령과 화면에 넣는 값을 순서대로 모았습니다. [6. 센서 달기](../06_sensors/README.md) 다음 내용입니다.

## 1. ROS 2 Jazzy 설치

이미 설치되어 있으면 건너뜁니다 (`ls /opt/ros/jazzy` 에 파일이 보이면 설치된 것).

```bash
sudo apt install -y software-properties-common curl
sudo add-apt-repository -y universe
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
```

```bash
sudo apt update
sudo apt install -y ros-jazzy-desktop
```

### 확인

터미널 1:

```bash
source /opt/ros/jazzy/setup.bash
ros2 run demo_nodes_cpp talker
```

터미널 2:

```bash
source /opt/ros/jazzy/setup.bash
ros2 run demo_nodes_py listener
```

터미널 2 에 `I heard: [Hello World: N]` 이 나오면 됩니다. 둘 다 `Ctrl` + `C` 로 멈춥니다.

## 2. ROS 2 와 함께 Isaac Sim 열기

`source` 를 Isaac Sim 보다 먼저 합니다.

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic list
~/isaacsim/isaac-sim.sh
```

- `File > Open Recent` → `simple_robot.usd`
- `Window > Extensions` → 검색칸에 `ros2` → `isaacsim.ros2.bridge` 가 켜져 있는지 확인

같은 네트워크에 다른 ROS 2 장비가 있어 모르는 토픽이 섞이면, 이 강의에서 여는 모든 터미널에서 먼저:

```bash
export ROS_DOMAIN_ID=83
```

## 3. 화면에서 누르는 순서

모두 ■ Stop 상태에서 합니다.

### 라이다 → LaserScan

| 순서 | 동작 | 값 |
|---|---|---|
| 1 | World 우클릭 → `Create > Visual Scripting > Action Graph` → 이름 바꾸기 | `ROS2_LidarStream` |
| 2 | ROS2_LidarStream 우클릭 → `Open Graph` | |
| 3 | 노드 검색 → 캔버스로 끌어 놓기 (4개) | `On Playback Tick` · `Isaac Read Lidar Beams` · `ROS2 Publish Laser Scan` · `Isaac Read Simulation Time` |
| 4 | 선 잇기 | 아래 표 |
| 5 | Read Lidar Beams 선택 → Lidar Prim → `Add Target...` | `/World/SimpleRobot/Front_Sensor/Lidar` |
| 6 | Publish Laser Scan 선택 → Topic Name | `scan_lidar` |
| 7 | Frame Id 확인 | `sim_lidar` (기본값) |

| 출력 (어디서) | 입력 (어디로) |
|---|---|
| On Playback Tick 의 `Tick` | Read Lidar Beams 의 `Exec In` |
| Read Lidar Beams 의 `Exec Out` | Publish Laser Scan 의 `Exec In` |
| Read Lidar Beams 의 `Azimuth Range` · `Depth Range` · `Horizontal Fov` · `Horizontal Resolution` · `Intensities Data` · `Linear Depth Data` · `Num Cols` · `Num Rows` · `Rotation Rate` | Publish Laser Scan 의 같은 이름 입력 |
| Read Simulation Time 의 `Simulation Time` | Publish Laser Scan 의 `Time Stamp` |

### 카메라 → Image

| 순서 | 동작 | 값 |
|---|---|---|
| 8 | World 우클릭 → `Create > Visual Scripting > Action Graph` → 이름 바꾸기 → `Open Graph` | `ROS2_Camera` |
| 9 | 노드 3개 끌어 놓기 | `On Playback Tick` · `Isaac Create Render Product` · `ROS2 Camera Helper` |
| 10 | 선 잇기 | `Tick` → `Exec In`, `Exec Out` → `Exec In`, `Render Product Path` → `Render Product Path` |
| 11 | Create Render Product → Camera Prim → `Add Target...` | `Robot_Camera` |
| 12 | Camera Helper → Type · Topic Name · Frame Id | `rgb` · `rgb` · `sim_camera` |
| 13 | ▶ Play | |

## 4. 토픽 확인 (새 터미널)

Isaac Sim 이 ▶ Play 중이어야 합니다.

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic list | grep -E 'scan_lidar|rgb'
ros2 topic hz /scan_lidar
ros2 topic echo /scan_lidar --once --no-arr
```

`ros2 topic hz` 는 `Ctrl` + `C` 로 멈춥니다.

## 5. RViz2

```bash
source /opt/ros/jazzy/setup.bash
rviz2
```

| 순서 | 동작 | 값 |
|---|---|---|
| 1 | Global Options → Fixed Frame (직접 타이핑) | `sim_lidar` |
| 2 | Add → By topic → `/scan_lidar` → LaserScan → OK, Size (m) | `0.05` |
| 3 | Add → By topic → `/rgb` → Image → OK | |

저장소의 설정 파일로 한 번에 열 수도 있습니다.

```bash
rviz2 -d ~/nvidia-dli-lecture/07_ros2/rviz/simple_robot.rviz
```

## 6. 같은 것을 코드로

Isaac Sim 창을 닫은 뒤 실행합니다.

터미널 1:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/nvidia-dli-lecture
git pull
~/isaacsim/python.sh 07_ros2/publish_ros2.py
```

```text
토픽을 내보내는 중: /scan_lidar (frame_id sim_lidar), /rgb (frame_id sim_camera)
끝내려면 Ctrl+C 를 누릅니다.
```

터미널 2:

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic list
ros2 topic hz /scan_lidar
ros2 topic echo /scan_lidar --once --no-arr
rviz2 -d ~/nvidia-dli-lecture/07_ros2/rviz/simple_robot.rviz
```

```text
header:
  stamp:
    sec: 36
    nanosec: 700001914
  frame_id: sim_lidar
angle_min: -3.1415927410125732
angle_max: 3.134611129760742
angle_increment: 0.006981316953897476
time_increment: 5.555555617320351e-05
scan_time: 0.05000000074505806
range_min: 0.4000000059604645
range_max: 100.0
ranges: '<sequence type: float, length: 900>'
intensities: '<sequence type: float, length: 900>'
```

### 로봇을 몰면서 보기

```bash
source /opt/ros/jazzy/setup.bash
~/isaacsim/python.sh 07_ros2/publish_ros2.py --keyboard
```

Isaac Sim 창의 뷰포트를 클릭한 뒤 W · A · S · D. RViz2 의 점과 영상이 함께 움직입니다.

## 막힐 때

| 증상 | 해결 |
|---|---|
| `ros2: command not found` | 이 터미널에서 `source /opt/ros/jazzy/setup.bash` |
| `ROS 2 환경이 없습니다` (publish_ros2.py) | 같은 터미널에서 `source /opt/ros/jazzy/setup.bash` 를 먼저 실행 |
| `source` 에서 `setup.sh` 를 못 찾는다는 오류 | 셸이 zsh 입니다. `source /opt/ros/jazzy/setup.zsh` 를 씁니다 |
| `ros2 topic list` 에 `/scan_lidar` 가 없음 | Isaac Sim 이 ▶ Play 중인지, 두 터미널의 `ROS_DOMAIN_ID` 가 같은지 확인 |
| RViz2 에 점이 안 보임 | Fixed Frame 이 `sim_lidar` 인지 확인 |
| 점이 로봇 바로 옆에 동그랗게 찍힘 | 라이다가 바퀴를 보고 있음 → Lidar 의 Translate Z 를 더 위로 |

## 옵션 전체

| 옵션 | 뜻 |
|---|---|
| `--seconds S` | 이 시간이 지나면 끝냄 (없으면 계속) |
| `--keyboard` | 창을 띄우고 W A S D 로 조종 |
| `--gui` | 창을 띄움 |
| `--save USD` | Play 전 장면을 파일로 저장 |
