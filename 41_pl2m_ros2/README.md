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

- ROS 2 쪽은 어디로 갈지를 정함. 궤적을 목표 위치로 바꿔, 자기가 정한 간격(50 Hz)으로 `/joint_command` 에 보냄.
- Isaac Sim 쪽은 로봇의 제어기 노릇을 함. 받은 목표 사이를 자기 주기로 잇고(보간기), 위치 구동이 따라가고, `GravityCompensation` 이 무게만큼의 토크를 더함.

### 이 강의에서 만드는 것

| 파일 | 어디서 도나 | 하는 일 |
|---|---|---|
| `pl2m_ros2.py` | Isaac Sim | ROS 2 그래프 셋 + 보간기 + 중력 보상 |
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
- `준비됨: 중력 보상 = both, 목표 = 보간. 끝내려면 Ctrl+C` 가 나오면 준비된 것임. 창으로 보려면 `--gui`, 부스 안에서 하려면 `--booth` 를 붙임.

### 주고받는 토픽

| 방향 | 토픽 | 내용 |
|---|---|---|
| 받기 | `/joint_command` | 관절의 목표 위치 (sensor_msgs/JointState) |
| 받기 | `/cmd_vel` | 베이스의 속도 (geometry_msgs/Twist) |
| 내기 | `/joint_states` | 관절의 위치 · 속도 · 힘 |
| 내기 | `/odom`, `/tf` | 출발점에서 본 베이스의 위치 (odom → base_footprint) |
| 내기 | `/clock` | 시뮬레이션 시간 |

- `/cmd_vel` 과 상태를 내는 그래프는 강의 14 의 Pinky Pro 와 같은 구성임. 새로운 것은 `/joint_command` 임.

### 그래프: /joint_command 받기

```python
nodes = [("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
         ("Context", "isaacsim.ros2.bridge.ROS2Context"),
         ("SubscribeJointState", "isaacsim.ros2.bridge.ROS2SubscribeJointState")]
values = [("SubscribeJointState.inputs:topicName", "joint_command")]
connects = [("OnPlaybackTick.outputs:tick", "SubscribeJointState.inputs:execIn"),
            ("Context.outputs:context", "SubscribeJointState.inputs:context")]
```

- ROS2 Subscribe JointState 노드가 메시지를 받음. 받은 값을 관절에 넣는 일은 그래프가 아니라 보간기가 함 (6절).
- 메시지의 위치만 씀. 속도와 힘(effort) 칸은 쓰지 않음. 이유는 7절에서 봄.

### 보간기와 중력 보상은 Play 뒤에

```python
sim.play()
sim.run(1.0)
joints = {"both": ARM_JOINTS, "left": LEFT_ARM, "none": []}[args.compensate]
compensation = GravityCompensation(joints=joints)    # 보상은 시뮬레이터 안에서. ROS 2 는 위치만 보낸다
interpolator = None if direct else JointCommandInterpolator(sim)   # 받은 목표 사이를 물리 스텝마다 보간
```

- 둘 다 물리 스텝마다 도는 파이썬 코드임. 하나는 관절의 목표 위치를, 하나는 관절에 더하는 토크를 넣음.
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

- `궤적 서버 준비됨: 팔 2 · 그리퍼 2 (50 Hz 로 보냄)` 가 나옴.

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
average rate: 59.776
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

### 코드: 정해진 간격으로 목표 보내기

```python
def wall_loop(self, period):
    """period 마다 step 을 부른다. 밀리지 않게 다음 시각을 정해 두고 그때까지 잔다."""
    due = time.perf_counter()
    while rclpy.ok():
        self.step()
        due += period
        time.sleep(max(0.0, due - time.perf_counter()))
```

- 0.02초(50 Hz)마다 지금 시각의 목표를 구해 `/joint_command` 를 한 번 보냄. `--rate 20` 처럼 간격을 바꿀 수 있음.
- 시뮬레이터가 몇 Hz 로 도는지는 신경 쓰지 않음. 그 차이는 받는 쪽이 메움 (6절).
- 팔 둘과 그리퍼의 목표를 한 메시지에 모아 보냄. 메시지에는 위치와, 그 목표가 궤적의 어느 시각의 것인지(`header.stamp`)를 넣음.

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
오른팔    성공               0.03          -0.03           0.01
팔      결과         가장 큰 오차[°]    1번 관절 오차[°]    2번 관절 오차[°]
왼팔     성공               0.28           0.08           0.13
오른팔    성공               0.28           0.05          -0.07
팔      결과         가장 큰 오차[°]    1번 관절 오차[°]    2번 관절 오차[°]
왼팔     성공               0.15           0.01          -0.05
오른팔    성공               0.09           0.02           0.01
```

- 네 자세 모두 0.3° 안에서 끝남. 운반 자세의 0.28° 는 강의 39 에서 본 6번 관절의 한계임.
- ROS 2 쪽은 위치만 보냈음. 팔이 처지지 않는 것은 시뮬레이터 안의 보상 덕분임. 다음 절에서 한쪽만 꺼서 확인함.

## 5. 한쪽 팔에만 보상을 넣어 비교

### 터미널 1: 왼팔만 보상

```bash
~/isaacsim/python.sh 41_pl2m_ros2/pl2m_ros2.py --compensate left --log work/pl2m/ros2_left.json
```

- 터미널 1 과 2 를 Ctrl+C 로 끝내고, 터미널 1 은 위 명령으로, 터미널 2 는 3절의 명령으로 다시 실행함.
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
왼팔     성공               0.28           0.07           0.13
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

## 6. 받은 목표를 보간하기

### 보내는 간격과 로봇의 주기는 다름

- 궤적 서버는 50 Hz 로 보내고, 시뮬레이터는 60 Hz 로 돎. 둘을 맞추지 않았음.
- 보내는 쪽은 로봇이 안에서 몇 Hz 로 도는지 몰라도 되어야 함. 실제 로봇에 명령을 보낼 때도 그럼.
- 그 차이는 받는 쪽, 곧 로봇의 제어기가 메워야 함. 받은 목표 사이를 자기 주기로 이어 주는 것이 보간기임.

### 그대로 넣으면 생기는 일

- 왼쪽이 받은 값을 Articulation Controller 로 그대로 넣은 경우임 (`--direct`). 스텝 179번 가운데 39번은 새 목표가 없어 그대로였고, 그 다음 스텝에 몰아서 바뀌었음.
- 위치 구동은 목표가 갑자기 바뀌면 큰 토크를 냄. 구동이 낸 토크가 −6.7 ~ 3.8 N·m 로 출렁였음.
- 오른쪽이 보간한 경우임. 목표가 스텝마다 고르게 바뀌고, 토크는 −0.6 ~ 1.6 N·m 임.

### 보내는 주파수를 바꿔도 해결되지 않았음

- 보간 없이 보내는 주파수만 60 Hz 로 맞췄을 때도 토크의 변화가 보간한 경우의 6배쯤이었음. 두 시계가 따로 돌아서 스텝당 정확히 한 개가 되지 않음.
- 더 빨리 보내면 오히려 나빠졌음. Isaac Sim 의 구독 노드가 프레임마다 메시지를 하나씩 꺼내 쓰기 때문임. 목표 5개를 한꺼번에 보냈더니 다섯 프레임에 걸쳐 차례로 적용되었음.
- 시뮬레이터의 `/joint_states` 가 올 때마다 하나씩 보내면 고르게 됨. 하지만 보내는 쪽이 로봇의 주기에 묶이는 방법이라 쓰지 않았음.

### 실제 제어기는 받는 쪽에서 이어 줌

| 제어기 | 문서에 적힌 것 |
|---|---|
| ros2_control 의 joint_trajectory_controller | 궤적의 점 사이를 시간에 따라 보간하므로 점 사이의 간격은 아무래도 됨. 보간 방법은 `none` 과 `spline` |
| Universal Robots 의 `servoj` | `lookahead_time`(0.03 ~ 0.2초, 기본 0.1초)이 궤적을 부드럽게 함. 목표가 낮은 주파수로 갱신될 때 gain 이 높거나 lookahead 가 짧으면 진동할 수 있다고 적음 |
| Franka 의 FCI | 1 kHz 로 실시간 제어 값을 보내는 인터페이스 |

- 궤적이나 목표 위치를 받는 인터페이스는 받는 쪽이 사이를 이어 줌. 그래서 사용자는 로봇의 주기를 신경 쓰지 않음.
- Franka 의 FCI 처럼 로봇의 주기(1 kHz)에 맞춰 직접 값을 넣는 저수준 인터페이스도 있음. 이쪽은 보내는 쪽이 주기를 맞춤.
- Isaac Sim 의 Subscribe JointState 노드와 Articulation Controller 노드에는 보간하는 설정이 없었음. 그래서 코드로 만들었음.

### 원리: 조금 늦게 따라가며 사이를 이음

- 받은 목표를 (시각, 값) 으로 모아 둠. 점들이 그것임.
- 지금보다 조금 앞선 시각을 "재생 시각" 으로 삼고, 그 시각의 값을 앞뒤 두 점 사이의 직선에서 구함. 파란 선임.
- 늦게 따라가기 때문에 "다음 목표" 를 이미 받아 둔 상태임. 그래서 사이를 이을 수 있음. 계산으로 그린 예시임.

### 지연은 얼마로

- 0.1초를 기본으로 하고, 보내는 간격이 길면 간격의 2배 + 2 스텝으로 늘림. 20 Hz 로 보내면 0.127초가 되었음.
- 지연이 길수록 메시지가 늦게 와도 버팀. 대신 그만큼 늦게 따라감.
- 보내는 간격은 받은 메시지의 시각 차이로 어림함. 사용자가 알려 줄 필요가 없음.

### 코드: 받기

```python
when = stamp if stamp > 0.0 else now                       # 보내는 쪽이 시각을 적지 않았으면 받은 시각을 쓴다
gap = when - self.newest
fresh = self.play is None or gap >= 0.3                    # 처음이거나 한동안 끊겼다 다시 옴
if 0.0 < gap < 0.3:                                        # 이어서 오는 중: 보내는 간격을 어림한다
    self.period += 0.1 * (gap - self.period)
    self.delay = max(self.DELAY, 2.0 * self.period + 2.0 * dt)
if fresh:
    self.play = when - self.delay                          # 지금 목표에서 delay 에 걸쳐 첫 목표로 간다
```

- 물리 스텝마다 구독 노드의 출력(관절 이름 · 위치 · 시각)을 읽고, 새 메시지이면 관절마다 (시각, 값) 을 모아 둠.
- 처음 받았거나 0.3초 넘게 끊겼다 다시 오면, 지금 목표에서 새 목표까지 `delay` 에 걸쳐 가도록 출발점을 넣음. 목표가 한 번에 튀지 않음.

### 코드: 스텝마다 재생

```python
behind = (now - self.delay) - self.play                    # 재생이 밀려 있으면 5 % 빠르게 따라잡는다
step = min(self.play + dt * (1.05 if behind > dt else 1.0), now - self.delay, self.newest)
self.play = max(step, self.play)
for name, points in self.points.items():
    value = points[-1][1]                                  # 받은 것을 다 재생했으면 마지막 값을 지킨다
    for (t0, v0), (t1, v1) in zip(points, list(points)[1:]):
        if self.play < t1:
            value = v0 if self.play <= t0 else v0 + (v1 - v0) * (self.play - t0) / (t1 - t0)
            break
    targets[0, self.names.index(name)] = value
self.robot.set_dof_position_targets(targets, self._every)
```

- 재생 시각을 스텝마다 한 스텝씩 앞으로 보냄. 받은 마지막 목표의 시각은 넘지 않음.
- 재생 시각의 앞뒤 두 점 사이를 직선으로 이어 값을 구하고, 관절의 목표 위치로 넣음.
- 궤적이 끝나면 마지막 목표에 도착해 멈춤. 받은 것보다 더 나아가지 않음.

### 메시지가 늦게 오면

- 재생할 것이 떨어지면 그 자리에 멈춰 기다림. 다시 오면 멈춘 자리에서 이어 감. 밀린 만큼을 한 번에 건너뛰지 않음.
- 밀린 시간은 5 % 빠르게 재생해 천천히 따라잡음.
- 위 그림 오른쪽의 2.5초 근처에 한 스텝 멈춘 자국이 있음. 이런 멈춤은 가끔 있었고, 4절과 5절의 실행에서는 0번이었음.

### 터미널: 간격을 바꿔 가며 재 보기

```bash
python3 41_pl2m_ros2/trajectory_server.py --rate 20
python3 41_pl2m_ros2/check_ripple.py work/pl2m/ros2_direct50.json work/pl2m/ros2_rate20.json work/pl2m/ros2_rate50.json work/pl2m/ros2_rate100.json work/pl2m/ros2_rate250.json work/pl2m/ros2_sync.json
```

- 시뮬레이터는 `--compensate left --log 파일` 로 실행하고, 궤적 서버의 `--rate` 를 20 · 50 · 100 · 250 으로 바꿔 가며 `send_pose.py forward` 를 보냄.
- 첫 줄의 `direct50` 은 시뮬레이터에 `--direct` 를 붙여 보간 없이 실행한 기록임. 마지막 줄은 서버에 `--sync` 를 붙여 `/joint_states` 가 올 때마다 보낸 것임.

### 실행 결과

```text
기록                              목표가 안 바뀐 스텝   한 스텝 최대[°]        구동 토크 범위[N·m]    스텝 사이 변화 평균   오차 최대[°]
ros2_direct50.json                   39 / 179         2.09       -6.7 ~ 3.8              1.20       3.48
ros2_rate20.json                      0 / 181         0.78       -0.7 ~ 0.7              0.10       3.66
ros2_rate50.json                      1 / 180         0.75       -0.6 ~ 1.6              0.08       3.58
ros2_rate100.json                     3 / 180         0.78       -1.2 ~ 1.2              0.09       3.74
ros2_rate250.json                     0 / 180         0.75       -0.5 ~ 0.7              0.06       3.58
ros2_sync.json                        0 / 180         0.75       -0.7 ~ 0.7              0.07       3.58
```

- 보간하면 20 Hz 로 보내든 250 Hz 로 보내든 토크의 변화가 0.06 ~ 0.10 임. 그대로 넣은 경우(1.20)의 1/12 이하임.
- 보내는 쪽은 간격을 마음대로 정해도 됨. 로봇의 주기에 맞춘 경우(`sync`)와 차이가 없음.

### 얻은 것과 치른 값

- 얻은 것: 보내는 쪽이 로봇의 주기를 몰라도 고르게 움직임.
- 치른 값: 정해 둔 지연(0.1초)만큼 늦게 따라감. 얼마나 늦게 끝나는지를 따로 재지는 않았음.
- 멈춘 뒤의 오차는 달라지지 않음 (왼팔 0.08°, 오른팔 1.66°).

## 7. 틀리기 쉬운 것: effort 칸을 이으면 보상이 지워짐

### 터미널 1

```bash
~/isaacsim/python.sh 41_pl2m_ros2/pl2m_ros2.py --compensate left --wire-effort
```

- `--wire-effort` 는 받은 메시지를 Articulation Controller 에 잇되, effort 칸까지 잇는 틀린 연결임. 터미널 3 에서 `send_pose.py forward` 를 보냄.

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
- 위치 · 속도 · 힘 세 칸을 모두 잇는 예제를 그대로 따라 하면 보상이 사라짐. 이 강의의 보간기는 위치만 씀.

## 8. 그리퍼와 바퀴

### 터미널 3: 그리퍼

```bash
python3 41_pl2m_ros2/send_pose.py transport --grip 0.03
```

```text
왼손 그리퍼: 위치 29.7 mm, 도착 예
오른손 그리퍼: 위치 29.7 mm, 도착 예
```

- 팔을 주행 자세로 옮긴 뒤 그리퍼를 30 mm 벌림. GripperCommand 액션으로 보냄.
- 목표의 2 mm 안에 들어오면 도착임. 1초 동안 움직이지 않으면 멈춘 것으로 보고 끝냄. 물체를 쥐는 것은 뒤 강의에서 확인함.

### 터미널 3: 바퀴

```bash
timeout 4 ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.25}}"
ros2 topic echo /odom --once --field pose.pose.position
```

```text
x: 1.0282757470313753
y: -0.0005499790782695639
z: -0.0007087199110743307
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
- 궤적 서버를 `--rate 10` 으로 띄우고 `check_ripple.py` 로 읽어, 보간기의 지연과 토크의 변화가 어떻게 되는지 보기.
- `JointCommandInterpolator.DELAY` 를 0.03 으로 줄이고 `--rate 50` 으로 돌려, 멈춘 스텝이 몇 번 생기는지 보기.
- `ros2 topic echo /joint_command --once` 로 메시지에 위치와 시각만 들어 있는지 보기.

## 11. 정리

### 기억할 것

- ROS 2 쪽(MoveIt · 궤적 서버)은 위치만 보냄. 중력 보상은 시뮬레이터 안, 로봇 쪽에 둠. 원본과 상용 로봇팔이 같은 구조임.
- 보내는 쪽은 자기 간격으로 보내면 됨. 받은 목표 사이를 로봇의 주기로 잇는 것은 받는 쪽의 일이고, Isaac Sim 에는 그 설정이 없어 보간기를 코드로 만들었음.
- 보간의 원리: 받은 목표를 (시각, 값) 으로 모으고, 0.1초 늦은 재생 시각의 값을 두 목표 사이 직선에서 구함.
- 받은 값을 그대로 넣으면 구동 토크가 −6.7 ~ 3.8 N·m 로 출렁였고, 보간하면 어느 간격으로 보내도 고름.
- `/joint_command` 의 effort 칸을 Articulation Controller 에 이으면 보상이 지워짐.

### 한계

- MoveIt 은 아직 붙이지 않았음. 속도가 든 궤적, 궤적 취소, 경로 도중의 오차 검사는 확인하지 않았음.
- 보간은 두 목표 사이를 직선으로 이음. 속도까지 맞춰 잇는 방법은 만들지 않았음.
- 보간기의 지연 0.1초는 정해 본 값임. 더 짧게 해도 되는지, 네트워크로 떨어진 PC 에서 얼마가 필요한지는 재지 않았음.
- 실제 제어기의 문서에서 옮긴 것은 "받는 쪽이 보간한다" 는 사실까지임. 그 안의 계산이 이 강의의 보간기와 같은지는 확인하지 못했음.
- ROS 2 쪽에서 토크를 계산해 보내는 방법은 만들어 보지 않았음.
- 강의 PC 한 대(Isaac Sim 5.1, ROS 2 Jazzy)에서 확인한 결과임.

### 출처

| 내용 | 문서 |
|---|---|
| joint_trajectory_controller 의 보간 | [userdoc](https://control.ros.org/jazzy/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html), [trajectory](https://control.ros.org/jazzy/doc/ros2_controllers/joint_trajectory_controller/doc/trajectory.html) |
| UR `servoj` 의 lookahead_time · gain | [Servoj command](https://www.universal-robots.com/articles/ur/programming/servoj-command/) |
| Franka FCI 의 1 kHz | [Franka Control Interface Overview](https://frankarobotics.github.io/docs/overview.html) |

- 2026년 11월에 열어 본 문서임. 문장을 직접 대조한 것만 적었음.
