# 41. PL-2M 을 ROS 2 에 연결하기: 궤적은 ROS 2 에서, 중력 보상은 로봇 안에서 — 명령 모음

- 영상에서 입력하는 터미널 명령과 기대 출력을 순서대로 모았음. [40. PL-2M 팔의 중력](../40_pl2m_gravity/README.md) 다음 내용임.

| 파일 |
|---|
| [check_ripple.py](check_ripple.py) |
| [pl2m_ros2.py](pl2m_ros2.py) |
| [send_pose.py](send_pose.py) |
| [trajectory_server.py](trajectory_server.py) |

## 1. 중력 보상은 어디에 두나

### 이번 강의의 결과

- 다른 터미널에서 ROS 2 로 궤적을 보내면, 부스 안의 PL-2M 이 두 팔과 그리퍼를 움직임.
- ROS 2 는 관절의 목표 위치만 보냄. 팔의 무게를 버티는 토크는 시뮬레이터 안에서 넣음.

### 질문

- 다음 강의에서 쓸 MoveIt 은 "어느 관절을 언제 어디로" 를 적은 궤적만 보냄. 토크는 보내지 않음.
- 그러면 강의 40 의 중력 보상은 누가 하나. ROS 2 쪽인가, 시뮬레이터 쪽인가.

### 세 가지 선택

| 방법 | 필요한 것 | 이 강의의 판단 |
|---|---|---|
| 시뮬레이터 안에서 보상 | 강의 40 의 `GravityCompensation` 그대로 | 이것을 씀 |
| ROS 2 쪽에서 토크를 계산해 보냄 | 로봇의 g(q) 를 계산하는 ROS 2 노드, 토크 명령을 받는 그래프 | 만들지 않았음 |
| 팔 링크의 중력을 끔 | 없음 | 강의 40 에서 본 대로 팔의 무게가 없어짐 |

- 원본 MuJoCo 시뮬레이터가 첫째 방법임. 시뮬레이터 노드가 궤적을 받고, 그 안에서 보상함.
- 강의 40 의 배경에서 본 상용 로봇팔과도 같은 구조임. 사용자는 위치를 보내고, 중력은 로봇의 제어기가 처리함.

### 역할 나누기

- ROS 2 쪽은 어디로 갈지를 정함. 궤적을 시간에 맞춰 목표 위치로 바꿔 `/joint_command` 로 보냄.
- Isaac Sim 쪽은 어떻게 버틸지를 맡음. 위치 구동이 목표를 따라가고, `GravityCompensation` 이 무게만큼의 토크를 더함.

### 이 강의에서 만드는 것

| 파일 | 어디서 도나 | 하는 일 |
|---|---|---|
| `pl2m_ros2.py` | Isaac Sim | ROS 2 그래프 셋 + 중력 보상 |
| `trajectory_server.py` | ROS 2 | 궤적 액션을 받아 `/joint_command` 를 냄 |
| `send_pose.py` | ROS 2 | 정해 둔 자세로 가는 궤적을 보냄 (MoveIt 이 할 일을 손으로) |
| `check_ripple.py` | 터미널 | 기록에서 목표와 토크가 고르게 바뀌었는지 읽음 |

- 액션의 이름은 원본과 같게 했음. 다음 강의에서 원본의 MoveIt 설정을 그대로 쓰기 위한 것임.

## 2. Isaac Sim 쪽

### 터미널 1: 시뮬레이터

```bash
cd ~/nvidia-dli-lecture
git pull
source /opt/ros/jazzy/setup.bash
~/isaacsim/python.sh 41_pl2m_ros2/pl2m_ros2.py
```

- `source` 를 Isaac Sim 보다 먼저 함. 강의 14 와 같음.
- `준비됨: 중력 보상 = both. 끝내려면 Ctrl+C` 가 나오면 준비된 것임. 창으로 보려면 `--gui`, 부스 안에서 하려면 `--booth` 를 붙임.

### 주고받는 토픽

| 방향 | 토픽 | 내용 |
|---|---|---|
| 받기 | `/joint_command` | 관절의 목표 위치 (sensor_msgs/JointState) |
| 받기 | `/cmd_vel` | 베이스의 속도 (geometry_msgs/Twist) |
| 내기 | `/joint_states` | 관절의 위치 · 속도 · 힘 |
| 내기 | `/odom`, `/tf` | 출발점에서 본 베이스의 위치 (odom → base_footprint) |
| 내기 | `/clock` | 시뮬레이션 시간 |

- `/cmd_vel` 과 상태를 내는 그래프는 강의 14 의 Pinky Pro 와 같은 구성임. 새로운 것은 `/joint_command` 임.

### 그래프: /joint_command → 관절

```python
connects = [("OnPlaybackTick.outputs:tick", "SubscribeJointState.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
            ("Context.outputs:context", "SubscribeJointState.inputs:context"),
            ("SubscribeJointState.outputs:jointNames", "ArticulationController.inputs:jointNames"),
            ("SubscribeJointState.outputs:positionCommand", "ArticulationController.inputs:positionCommand")]
```

- ROS2 Subscribe JointState 노드가 메시지를 받고, Articulation Controller 가 이름이 적힌 관절의 목표 위치를 바꿈.
- **위치만 이음.** 속도와 힘(effort) 칸은 잇지 않음. 이유는 7절에서 봄.

### 중력 보상은 Play 뒤에

```python
prepare_pl2m(stage)                                  # 팔 링크의 중력을 켜 둔다 (강의 40)
add_ros2(args.wire_effort)
sim = Sim(app)
sim.play()
sim.run(1.0)
joints = {"both": ARM_JOINTS, "left": LEFT_ARM, "none": []}[args.compensate]
compensation = GravityCompensation(joints=joints)    # 보상은 시뮬레이터 안에서. ROS 2 는 위치만 보낸다
```

- 강의 40 의 클래스를 그대로 씀. ROS 2 그래프와는 따로 돌고, 물리 스텝마다 토크를 더함.
- `--compensate left` 는 왼팔만, `none` 은 보상 없이 실행함. 비교에 씀.

### 익스텐션을 켠 뒤 한 번 진행

```python
def enable_ros2(app):
    enable_extension("isaacsim.ros2.bridge")
    enable_extension("isaacsim.robot.wheeled_robots")
    app.update()                                         # 익스텐션이 다 올라온 뒤에 그래프를 만들어야 한다 (안 하면 Play 에서 죽었음)
```

- `app.update()` 를 빼고 그래프를 만들었더니 Play 하는 순간 Isaac Sim 이 죽었음. 강의 14 도 같은 순서임.

## 3. 궤적 서버

### 터미널 2: 궤적 서버

```bash
cd ~/nvidia-dli-lecture
source /opt/ros/jazzy/setup.bash
python3 41_pl2m_ros2/trajectory_server.py
```

- `궤적 서버 준비됨: 팔 2 · 그리퍼 2 (/joint_states 에 맞춰 보냄)` 가 나옴.

### 터미널 3: 토픽과 액션 확인

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic list
ros2 action list
ros2 topic hz /joint_states
```

### 실행 결과

```text
/clock
/cmd_vel
/joint_command
/joint_states
/odom
/tf
/left_gripper_controller/gripper_cmd
/left_joint_trajectory_controller/follow_joint_trajectory
/right_gripper_controller/gripper_cmd
/right_joint_trajectory_controller/follow_joint_trajectory
average rate: 59.779
```

- 토픽 여섯 개는 Isaac Sim 이, 액션 네 개는 궤적 서버가 내는 것임. `/parameter_events` 와 `/rosout` 은 뺐음.
- `/joint_states` 는 초당 60번쯤 나옴. 물리 스텝마다 한 번임.

### 액션 네 개

| 액션 | 종류 | 관절 |
|---|---|---|
| `/left_joint_trajectory_controller/follow_joint_trajectory` | FollowJointTrajectory | 왼팔 관절 7개 |
| `/right_joint_trajectory_controller/follow_joint_trajectory` | FollowJointTrajectory | 오른팔 관절 7개 |
| `/left_gripper_controller/gripper_cmd` | GripperCommand | 왼손 손가락 |
| `/right_gripper_controller/gripper_cmd` | GripperCommand | 오른손 손가락 |

- 원본의 `moveit_controllers.yaml` 에 적힌 이름과 같음. MoveIt 은 이 이름으로 궤적을 보냄.
- 원본은 시뮬레이터 노드가 이 액션을 직접 냄. 여기서는 따로 띄우는 ROS 2 노드가 냄.

### 코드: 궤적의 한 점 구하기

```python
i = int(np.searchsorted(self.times, t, side="right")) - 1
dt = self.times[i + 1] - self.times[i]
u = (t - self.times[i]) / dt
a, b = self.positions[i], self.positions[i + 1]
if self.velocities is None:
    return a + (b - a) * u
```

- 궤적은 "몇 초에 어느 자세" 를 적은 점들임. 점과 점 사이의 시각에는 그 사이를 이어서 목표를 구함.
- 점에 속도가 없으면 직선으로, 있으면 3차 곡선으로 이음. 원본의 `trajectory.py` 와 같은 방법임.

### 코드: 스텝마다 목표 보내기

```python
def on_joint_states(self, msg):
    with self.lock:
        self.measured.update(zip(msg.name, msg.position))
        self.effort.update(zip(msg.name, msg.effort))
        self.stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    if not self.wall:
        self.step()                            # 시뮬레이터가 한 스텝 나아갈 때마다 목표를 한 번 보낸다
```

- `/joint_states` 가 올 때마다 그 시각의 목표를 구해 `/joint_command` 를 한 번 보냄.
- 팔 둘과 그리퍼의 목표를 한 메시지에 모아 보냄. 메시지에는 위치만 넣음.
- 왜 `/joint_states` 에 맞추는지는 6절에서 봄.

### 코드: 끝났는지 보기

```python
if t >= trajectory.times[-1] and not state["done"].is_set():
    error = np.abs(actual - trajectory.positions[-1])
    if all(error[i] <= state["tolerance"].get(name, GOAL_TOLERANCE) for i, name in enumerate(order)):
        state["outcome"] = ("succeed", "")
        state["done"].set()
    elif t > trajectory.times[-1] + state["timeout"]:
        state["outcome"] = ("abort", f"목표에 들어오지 못함: 가장 큰 오차 {math.degrees(error.max()):.2f}°")
        state["done"].set()
```

- 궤적의 시간이 끝나고, 모든 관절이 목표의 0.035 rad(2°) 안에 들어오면 성공임. 3초 안에 못 들어오면 실패임.
- 두 값은 원본 설정의 `goal_tolerance` 와 `goal_time_tolerance` 임.

## 4. 자세 보내기

### 터미널 3

```bash
python3 41_pl2m_ros2/send_pose.py forward
python3 41_pl2m_ros2/send_pose.py side
python3 41_pl2m_ros2/send_pose.py carry
python3 41_pl2m_ros2/send_pose.py down
```

- 두 팔의 지금 자세에서 정해 둔 자세까지, 3초짜리 궤적(점 20개)을 만들어 두 액션에 보냄.
- 끝나면 `/joint_states` 에서 읽은 위치와 목표의 차이를 적음.

### 실행 결과

```text
팔      결과         가장 큰 오차[°]    1번 관절 오차[°]    2번 관절 오차[°]
왼팔     성공               0.11           0.09          -0.06
오른팔    성공               0.10           0.05           0.06
팔      결과         가장 큰 오차[°]    1번 관절 오차[°]    2번 관절 오차[°]
왼팔     성공               0.06          -0.05           0.06
오른팔    성공               0.04          -0.04           0.01
팔      결과         가장 큰 오차[°]    1번 관절 오차[°]    2번 관절 오차[°]
왼팔     성공               0.28           0.09           0.12
오른팔    성공               0.28           0.06          -0.08
팔      결과         가장 큰 오차[°]    1번 관절 오차[°]    2번 관절 오차[°]
왼팔     성공               0.15           0.01          -0.05
오른팔    성공               0.10           0.01           0.03
```

- 네 자세 모두 0.3° 안에서 끝남. 운반 자세의 0.28° 는 강의 39 에서 본 6번 관절의 한계임.
- ROS 2 쪽은 위치만 보냈음. 팔이 처지지 않는 것은 시뮬레이터 안의 보상 덕분임. 다음 절에서 한쪽만 꺼서 확인함.

## 5. 한쪽 팔에만 보상을 넣어 비교

### 터미널 1: 왼팔만 보상

```bash
~/isaacsim/python.sh 41_pl2m_ros2/pl2m_ros2.py --compensate left --log work/pl2m/ros2_left.json
```

- 터미널 1 과 2 를 Ctrl+C 로 끝내고, 터미널 1 은 아래 명령으로, 터미널 2 는 3절의 명령으로 다시 실행함.
- 터미널 3 에서 4절의 네 자세를 다시 보냄. `--log` 는 Ctrl+C 로 끝낼 때 위치와 토크를 파일에 저장함.

### 실행 결과

```text
팔      결과         가장 큰 오차[°]    1번 관절 오차[°]    2번 관절 오차[°]
왼팔     성공               0.11           0.08          -0.06
오른팔    성공               1.66           1.66           0.05
팔      결과         가장 큰 오차[°]    1번 관절 오차[°]    2번 관절 오차[°]
왼팔     성공               0.06          -0.05           0.06
오른팔    성공               1.27          -0.12           1.27
팔      결과         가장 큰 오차[°]    1번 관절 오차[°]    2번 관절 오차[°]
왼팔     성공               0.28           0.08           0.12
오른팔    성공               0.71           0.71           0.12
```

- 같은 궤적을 보냈는데, 보상이 없는 오른팔은 앞으로 뻗기에서 1.66°, 옆으로 뻗기에서 1.27° 가 남음.
- 강의 40 에서 시뮬레이터 안에서 직접 움직였을 때와 같은 값임. ROS 2 를 거쳐도 보상이 그대로 일함.

### 비교 그래프

- 위: 멈춘 뒤 오른팔(구동만)은 1.66°, 왼팔(구동 + 보상)은 0.08° 임.
- 아래: 버티는 토크 11 N·m 를 오른팔은 구동이, 왼팔은 보상이 냄. 왼팔의 구동이 낸 토크는 0 근처임.
- 오른팔은 새 궤적이 시작될 때 토크가 튐. 궤적이 "지금 잰 자세" 에서 출발하는데, 처져 있던 만큼 목표가 한 번에 바뀌기 때문임.

### 읽기

- ROS 2 로 온 것은 위치뿐임. 그래프의 초록 선(보상 토크)은 ROS 2 메시지 어디에도 없음. 시뮬레이터가 자세에서 계산해 넣은 것임.
- 그래서 MoveIt 은 중력을 몰라도 됨. 계획한 자세에 팔이 실제로 가 있게 하는 것은 로봇 쪽의 일임.

## 6. 목표를 보내는 박자

### 처음 만든 서버는 토크가 출렁였음

- 처음에는 궤적 서버가 자기 시계로 50 Hz 씩 목표를 보냈음. 시뮬레이터는 60 Hz 로 돎.
- 물리 스텝 180번 가운데 41번은 목표가 그대로였고, 그 다음 스텝에 두 번 치가 한꺼번에 바뀌었음.
- 구동이 낸 토크가 −9.0 ~ 3.2 N·m 로 출렁였음. `/joint_states` 가 올 때마다 보내게 고친 뒤에는 −0.7 ~ 0.7 N·m 임.

### 주파수를 맞추면 되지 않나

```bash
python3 41_pl2m_ros2/trajectory_server.py --wall 60
```

- `--wall` 뒤에 숫자를 주면, 서버가 자기 시계로 그 주파수만큼 보냄. 50 · 60 · 240 · 1000 Hz 로 바꿔 가며 같은 궤적을 보냈음.
- 시뮬레이터는 `--compensate left --log` 로 실행하고, 기록을 아래 스크립트로 읽음.

### 터미널 3: 기록 읽기

```bash
python3 41_pl2m_ros2/check_ripple.py work/pl2m/ros2_wall50.json work/pl2m/ros2_wall60.json work/pl2m/ros2_wall240.json work/pl2m/ros2_wall1000.json work/pl2m/ros2_sync.json
```

```text
기록                              목표가 안 바뀐 스텝   한 스텝 최대[°]        구동 토크 범위[N·m]    스텝 사이 변화 평균   오차 최대[°]
ros2_wall50.json                     41 / 180         2.60       -9.0 ~ 3.2              1.07       3.41
ros2_wall60.json                     16 / 180         1.79       -3.7 ~ 2.7              0.50       2.96
ros2_wall240.json                    32 / 180         2.61       -7.1 ~ 2.9              0.94       3.79
ros2_wall1000.json                   30 / 180         5.20      -17.4 ~ 4.9              0.94       5.01
ros2_sync.json                        5 / 180         0.75       -0.7 ~ 0.8              0.07       2.83
```

- 시뮬레이터와 같은 60 Hz 가 가장 나음. 그래도 스텝에 맞춘 경우보다 토크의 변화가 7배 큼 (0.50 과 0.07).
- 주파수를 더 올리면 좋아지지 않고 오히려 나빠짐. 1000 Hz 에서는 한 스텝에 목표가 5.2° 바뀌었음.
- 값은 실행할 때마다 달라질 수 있음. 서버를 만드는 동안 50 Hz 에서 "안 바뀐 스텝" 이 31 ~ 41 사이로 나왔음.

### 왜 주파수만으로는 안 되나

- Isaac Sim 의 구독 노드는 프레임마다 메시지를 **하나씩** 꺼내 씀. 서로 다른 목표 5개를 한꺼번에 보냈더니, 다섯 프레임에 걸쳐 차례로 하나씩 적용되었음.
- 프레임당 한 개보다 적게 오면 건너뛰는 프레임이 생기고, 많이 오면 밀려서 늦게 적용됨.
- 서버의 시계와 시뮬레이터의 시계는 따로 돎. 60 Hz 로 맞춰도 프레임당 정확히 한 개가 되지는 않음.

### 읽기

- 위치 구동은 목표가 갑자기 바뀌면 큰 토크를 냄. 목표가 고르게 바뀌어야 토크도 고름.
- 시뮬레이터가 내는 `/joint_states` 를 박자로 쓰면, 시뮬레이터가 한 프레임 나아갈 때마다 목표가 정확히 한 번 감.
- 멈춘 뒤의 오차는 어느 방법이나 같음 (왼팔 0.08°, 오른팔 1.66°). 달라지는 것은 움직이는 동안의 토크임.

## 7. 틀리기 쉬운 것: effort 칸을 이으면 보상이 지워짐

### 터미널 1

```bash
~/isaacsim/python.sh 41_pl2m_ros2/pl2m_ros2.py --compensate left --wire-effort
```

- `--wire-effort` 는 `/joint_command` 의 effort 칸까지 Articulation Controller 에 잇는 틀린 연결임. 터미널 3 에서 `send_pose.py forward` 를 보냄.

### 실행 결과

```text
팔      결과         가장 큰 오차[°]    1번 관절 오차[°]    2번 관절 오차[°]
왼팔     성공               1.69           1.69          -0.06
오른팔    성공               1.65           1.65           0.05
```

- 보상을 넣은 왼팔도 1.69° 처졌음. 보상이 없는 오른팔과 같아졌음.

### 왜 그런가

```python
if wire_effort:      # 이렇게 이으면 메시지의 effort 가 관절 힘으로 들어가, 중력 보상으로 넣은 힘을 덮어쓴다
    connects += [("SubscribeJointState.outputs:effortCommand", "ArticulationController.inputs:effortCommand")]
```

- 보상으로 넣은 토크를 Articulation Controller 가 프레임마다 다시 써 버리는 것으로 보임. 안에서 어떤 순서로 일어나는지는 확인하지 않았음.
- 궤적 서버는 effort 칸을 비워서 보냈는데도 지워졌음. 칸을 잇는 것만으로 일어남.
- 위치 · 속도 · 힘 세 칸을 모두 잇는 예제를 그대로 따라 하면 보상이 사라짐. 이 로봇에서는 위치만 이어야 함.

## 8. 그리퍼와 바퀴

### 터미널 3: 그리퍼

```bash
python3 41_pl2m_ros2/send_pose.py transport --grip 0.03
```

```text
왼손 그리퍼: 위치 30.0 mm, 도착 예
오른손 그리퍼: 위치 30.0 mm, 도착 예
```

- 팔을 주행 자세로 옮긴 뒤 그리퍼를 30 mm 벌림. GripperCommand 액션으로 보냄.
- 목표의 2 mm 안에 들어오면 도착임. 1초 동안 움직이지 않으면 멈춘 것으로 보고 끝냄. 물체를 쥐는 것은 뒤 강의에서 확인함.

### 터미널 3: 바퀴

```bash
timeout 4 ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.25}}"
ros2 topic echo /odom --once --field pose.pose.position
```

```text
x: 1.0275075407305796
y: -4.260609472267809e-05
z: -0.0007081668624647433
```

- 0.25 m/s 를 4초쯤 보내니 `/odom` 이 1.03 m 를 가리킴. 보낸 시간이 정확히 4초는 아니었음.
- 로봇을 바닥에 거의 닿은 높이(2 mm)에서 시작하게 했음. 5 cm 에서 떨어뜨렸을 때는 `/odom` 의 z 가 −0.033 으로 어긋났음.

## 9. 다음 강의

### MoveIt 을 붙일 준비

- 액션의 이름이 원본의 MoveIt 설정과 같음. MoveIt 을 띄우면 이 궤적 서버로 궤적을 보내게 됨.
- 이 강의의 `send_pose.py` 는 속도가 없는 궤적을 보냈음. MoveIt 이 보내는 속도 있는 궤적은 다음 강의에서 확인함.
- MoveIt 이 로봇의 지금 자세를 알려면 `/joint_states` 와 링크 사이의 `/tf` 가 필요함. `/tf` 는 지금 odom → base_footprint 만 내고 있음. 나머지는 다음 강의에서 붙임.

## 10. 해 볼 것

### 직접 바꿔 보기

- `--compensate none` 으로 실행하고 `send_pose.py forward` 를 보내, 두 팔이 모두 1.7° 쯤 처지는지 보기.
- `send_pose.py forward --seconds 1` 처럼 궤적을 짧게 해서, 움직이는 동안의 오차와 끝난 뒤의 오차가 어떻게 달라지는지 보기.
- `trajectory_server.py` 의 `GOAL_TOLERANCE` 를 0.02 로 줄이고 `--compensate none` 으로 실행해, 궤적이 실패로 끝나는지 보기.
- `ros2 topic echo /joint_command --once` 로 메시지에 위치만 들어 있는지 보기.

## 11. 정리

### 기억할 것

- ROS 2 쪽(MoveIt · 궤적 서버)은 위치만 보냄. 중력 보상은 시뮬레이터 안, 로봇 쪽에 둠. 원본과 상용 로봇팔이 같은 구조임.
- `/joint_command` 는 위치 칸만 Articulation Controller 에 이음. effort 칸을 이으면 보상이 지워짐.
- 목표는 `/joint_states` 가 올 때마다 한 번씩 보냄. 주파수를 맞추는 것만으로는 부족했음. 구독 노드가 프레임마다 메시지를 하나씩 쓰기 때문임.
- 한쪽 팔만 보상해 보면 ROS 2 를 거쳐도 강의 40 과 같은 값이 나옴 (왼팔 0.08°, 오른팔 1.66°).
- 익스텐션을 켠 뒤 `app.update()` 를 한 번 하고 그래프를 만듦.

### 한계

- MoveIt 은 아직 붙이지 않았음. 속도가 든 궤적, 궤적 취소, 경로 도중의 오차 검사는 확인하지 않았음.
- ROS 2 쪽에서 토크를 계산해 보내는 방법은 만들어 보지 않았음. 두 방법을 재서 비교한 것은 아님.
- 궤적 서버와 시뮬레이터가 한 PC 에 있을 때의 결과임. 네트워크로 떨어져 있을 때의 지연은 재지 않았음.
- 실물 로봇에 목표를 어떤 박자로 보내야 하는지는 다루지 않았음. 여기서 본 것은 Isaac Sim 의 구독 노드에 맞춘 방법임.
- 손에 든 물체의 무게는 보상에 들어 있지 않음. 물체를 집는 강의에서 봄.
- 강의 PC 한 대(Isaac Sim 5.1, ROS 2 Jazzy)에서 확인한 결과임.
