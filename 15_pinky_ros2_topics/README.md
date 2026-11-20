# 15. Pinky Pro 의 센서와 위치를 ROS 2 로 내보내기 · RViz2 — 명령 모음

- 영상에서 입력하는 터미널 명령과 화면에 넣는 값을 순서대로 모았음. [14. Pinky Pro 를 ROS 2 로 움직이기](../14_pinky_ros2/README.md) 다음 내용임.

- 강의 14 는 ROS 2 에서 **받는** 쪽(`/cmd_vel`)이었음. 이번에는 **내보내는** 쪽임: 로봇의 위치(`/odom` · `/tf` · `/joint_states`)와 센서(`/scan` · `/camera/image_raw` · `/imu`)를 내보내고 RViz2 로 봄. 토픽 이름은 [공식 저장소](https://github.com/pinklab-art/pinky_pro)를 따름.

## 1. 준비

```bash
source /opt/ros/jazzy/setup.bash
cd ~/isaacsim
./isaac-sim.sh
```

- `File > Open` → `~/nvidia-dli-lecture/work/pinky_gui.usd` (강의 14 에서 저장한 장면)
- `pinky` 선택 → Translate Z 를 `0` 으로 (바닥에 닿은 채로 시작해야 `/odom` 의 높이가 0 이 됨)

- 다른 ROS 2 장비와 같은 네트워크라면 모든 터미널에서 같은 번호를 정해 두는 편이 안전함 (예: `export ROS_DOMAIN_ID=83`, `source` 다음 · Isaac Sim 실행 전).

## 2. 위치 그래프 (두 번째 그래프)

- `pinky` 우클릭 → `Create > Visual Scripting > Action Graph`. 노드 7개:

| 검색어 | 노드 | 하는 일 |
|---|---|---|
| `Playback Tick` | On Playback Tick | 매 프레임 실행 신호 |
| `Simulation Time` | Isaac Read Simulation Time | 시뮬레이션 시간 (메시지의 시각 도장) |
| `Compute Odometry` | Isaac Compute Odometry Node | 출발점에서 본 로봇의 위치 · 속도 |
| `Publish Odometry` | ROS2 Publish Odometry | `/odom` 내보내기 |
| `Raw Transform` | ROS2 Publish Raw Transform Tree | `/tf` 의 odom → base_footprint |
| `Joint State` | ROS2 Publish Joint State | `/joint_states` (바퀴 각도) |
| `Publish Clock` | ROS2 Publish Clock | `/clock` (시뮬레이션 시간) |

- 연결:

| 나가는 곳 | 들어가는 곳 |
|---|---|
| On Playback Tick › Tick | Isaac Compute Odometry Node › Exec In |
| On Playback Tick › Tick | ROS2 Publish Joint State › Exec In |
| On Playback Tick › Tick | ROS2 Publish Clock › Exec In |
| Isaac Compute Odometry Node › Exec Out | ROS2 Publish Odometry › Exec In |
| Isaac Compute Odometry Node › Exec Out | ROS2 Publish Raw Transform Tree › Exec In |
| Compute Odometry › Position · Orientation · Linear Velocity · Angular Velocity | ROS2 Publish Odometry 의 같은 이름 |
| Compute Odometry › Position | Raw Transform Tree › Translation |
| Compute Odometry › Orientation | Raw Transform Tree › Rotation |
| Isaac Read Simulation Time › Simulation Time | 내보내기 노드 4개의 Timestamp |

- 노드 값:

| 노드 | 속성 | 값 |
|---|---|---|
| Isaac Compute Odometry Node | chassisPrim → `Add Target...` | `/World/pinky/base_footprint` |
| ROS2 Publish Odometry | chassisFrameId | `base_footprint` |
| ROS2 Publish Raw Transform Tree | childFrameId | `base_footprint` |
| ROS2 Publish Joint State | targetPrim → `Add Target...` | `/World/pinky/base_footprint` |

- 나머지(`odom` · `tf` · `joint_states` · `clock`)는 기본값 그대로임.

## 3. 센서 그래프 (세 번째 그래프)

- `pinky` 우클릭 → `Create > Visual Scripting > Action Graph` 를 한 번 더. 노드 8개:

| 검색어 | 노드 | 하는 일 |
|---|---|---|
| `Playback Tick` | On Playback Tick | 매 프레임 실행 신호 |
| `Simulation Time` | Isaac Read Simulation Time | 시각 도장 |
| `Read Lidar` | Isaac Read Lidar Beams Node | 라이다 값 읽기 |
| `Laser Scan` | ROS2 Publish Laser Scan | `/scan` 내보내기 |
| `Render Product` | Isaac Create Render Product | 카메라 화면을 그리는 곳 만들기 |
| `Camera Helper` | ROS2 Camera Helper | `/camera/image_raw` 내보내기 |
| `Isaac Read IMU` | Isaac Read IMU Node | IMU 값 읽기 |
| `Publish Imu` | ROS2 Publish Imu | `/imu` 내보내기 |

- 연결:

| 나가는 곳 | 들어가는 곳 |
|---|---|
| On Playback Tick › Tick | Read Lidar Beams · Create Render Product · Read IMU 의 Exec In |
| Isaac Read Lidar Beams Node › Exec Out | ROS2 Publish Laser Scan › Exec In |
| Read Lidar Beams › Azimuth Range · Depth Range · Horizontal Fov · Horizontal Resolution · Intensities Data · Linear Depth Data · Num Cols · Num Rows · Rotation Rate | ROS2 Publish Laser Scan 의 같은 이름 (9개) |
| Isaac Create Render Product › Exec Out | ROS2 Camera Helper › Exec In |
| Isaac Create Render Product › Render Product Path | ROS2 Camera Helper › Render Product Path |
| Isaac Read IMU Node › Exec Out | ROS2 Publish Imu › Exec In |
| Read IMU › Angular Velocity Vector · Linear Acceleration Vector · Sensor Orientation Quaternion | Publish Imu › Angular Velocity · Linear Acceleration · Orientation |
| Isaac Read Simulation Time › Simulation Time | Publish Laser Scan · Publish Imu 의 Timestamp |

- 노드 값:

| 노드 | 속성 | 값 |
|---|---|---|
| Isaac Read Lidar Beams Node | lidarPrim → `Add Target...` | `…/rplidar_mount/rplidar_link/Lidar` |
| ROS2 Publish Laser Scan | frameId | `rplidar_link` (topicName 은 기본값 `scan`) |
| Isaac Create Render Product | cameraPrim → `Add Target...` | `…/front_camera_mount/front_camera_link/Camera` |
| Isaac Create Render Product | width / height | `640` / `360` |
| ROS2 Camera Helper | topicName | `camera/image_raw` |
| ROS2 Camera Helper | frameId | `front_camera_link` |
| Isaac Read IMU Node | IMU Prim → `Add Target...` | `…/base_link/imu_link/Imu_Sensor` |
| ROS2 Publish Imu | frameId | `imu_link` (topicName 은 기본값 `imu`) |

- `Ctrl` + `S` 로 저장한 뒤 ▶ Play.

## 4. 토픽 확인

- 새 터미널에서:

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic list -t
```

```text
/camera/image_raw [sensor_msgs/msg/Image]
/clock [rosgraph_msgs/msg/Clock]
/cmd_vel [geometry_msgs/msg/Twist]
/imu [sensor_msgs/msg/Imu]
/joint_states [sensor_msgs/msg/JointState]
/odom [nav_msgs/msg/Odometry]
/scan [sensor_msgs/msg/LaserScan]
/tf [tf2_msgs/msg/TFMessage]
```

```bash
ros2 topic hz /scan
ros2 topic echo /odom --once
ros2 topic echo /scan --once
ros2 topic echo /imu --once
ros2 topic echo /camera/image_raw --once --no-arr
```

- `ros2 topic hz` 는 `Ctrl` + `C` 로 멈춤. 강의 PC: `/odom` · `/tf` · `/joint_states` · `/imu` · `/clock` 약 60 Hz, `/scan` 약 19 Hz, `/camera/image_raw` 약 45 Hz.

## 5. robot_state_publisher 와 RViz2

- 터미널 2 (로봇의 링크 좌표계를 계산해 `/tf` 로 내보냄):

```bash
source /opt/ros/jazzy/setup.bash
ros2 run robot_state_publisher robot_state_publisher --ros-args -p use_sim_time:=true -p robot_description:="$(cat ~/nvidia-dli-lecture/work/pinky_description/urdf/pinky.urdf)"
```

- 터미널 3:

```bash
source /opt/ros/jazzy/setup.bash
source ~/pinky_pro/install/setup.bash
rviz2 -d ~/nvidia-dli-lecture/15_pinky_ros2_topics/rviz/pinky.rviz --ros-args -p use_sim_time:=true
```

- 터미널 4 (움직여 보기):

```bash
source /opt/ros/jazzy/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

- 좌표계 확인:

```bash
ros2 run tf2_ros tf2_echo odom rplidar_link --ros-args -p use_sim_time:=true
```

```text
- Translation: [-0.017, -0.000, 0.125]
- Matrix:
 -1.000 -0.000  0.000 -0.017
  0.000 -1.000  0.000 -0.000
  0.000  0.000  1.000  0.125
  0.000  0.000  0.000  1.000
```

- 라이다 높이 0.125 m 가 나오면 `odom → base_footprint → … → rplidar_link` 가 모두 이어진 것임.

## 6. 같은 일을 코드로

- Isaac Sim 창을 닫은 뒤, 터미널 1:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/nvidia-dli-lecture
git pull
~/isaacsim/python.sh 14_pinky_ros2/pinky_ros2.py
```

- 강의 14 의 스크립트가 이 강의의 그래프까지 모두 만듦. 4 · 5절의 명령을 그대로 쓰면 됨. 코드에서는 `/scan` 이 약 10 Hz 임 (라이다를 1초에 10바퀴로 만들기 때문).

## 막힐 때

| 증상 | 해결 |
|---|---|
| RViz2 에 로봇이 흰 덩어리로 보이거나 RobotModel 이 빨간색 | 터미널 3 에서 `source ~/pinky_pro/install/setup.bash` 를 했는지, 터미널 2 의 robot_state_publisher 가 켜져 있는지 확인 |
| LaserScan 이 안 보임 · "queue is full" 메시지 | `/tf` 가 끊긴 것. robot_state_publisher · RViz2 에 `use_sim_time:=true` 를 줬는지 확인 |
| 다른 장비의 토픽이 섞여 보임 · 좌표계가 이상함 | 같은 네트워크의 다른 ROS 2 장비와 섞인 것. 모든 터미널과 Isaac Sim 에서 같은 `ROS_DOMAIN_ID` 를 새 번호로 |
| `/odom` 의 z 가 −0.03 | `pinky` 의 Translate Z 가 0.05 인 채로 시작함 → `0` 으로 |
| `/camera/image_raw` 가 없음 | Isaac Create Render Product 의 cameraPrim, Camera Helper 연결 확인. ▶ Play 를 다시 |
| 그래프에 `ROS2 ...` 노드가 없음 | `source` 를 하지 않고 Isaac Sim 을 켰음 |
