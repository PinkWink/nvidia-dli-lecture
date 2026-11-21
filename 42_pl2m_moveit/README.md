# 42. MoveIt 으로 팔의 경로 계획하기: 목표만 주면 부딪히지 않는 길을 찾음 — 명령 모음

- 영상에서 입력하는 터미널 명령과 기대 출력을 순서대로 모았음. [41. PL-2M 을 ROS 2 에 연결하기](../41_pl2m_ros2/README.md) 다음 내용임.

| 파일 |
|---|
| [add_obstacle.py](add_obstacle.py) |
| [move_arm.py](move_arm.py) |
| [moveit.launch.py](moveit.launch.py) |
| [obstacle.py](obstacle.py) |
| [pl2m_moveit.py](pl2m_moveit.py) |
| [prepare_moveit.sh](prepare_moveit.sh) |

## 1. MoveIt 이 하는 일

### 이번 강의의 결과

- 왼팔을 내린 자세에서 앞으로 든 자세로 보냄. 손끝이 지나는 길에 기둥과 컵이 있음.
- 왼쪽: MoveIt 이 장애물을 모름. 팔이 컵을 쳐서 떨어뜨림.
- 오른쪽: 장애물을 알려 줌. 같은 목표인데 돌아가는 길을 찾아 컵을 건드리지 않음.

### 강의 41 과 무엇이 다른가

- 강의 41 에서는 "관절을 몇 도로" 가는 궤적을 손으로 만들어 보냈음 (`send_pose.py`). 지금 자세와 목표 사이를 곧게 이었을 뿐임.
- MoveIt 은 목표만 받음. 로봇의 모양과 주변을 따져 부딪히지 않는 길을 찾고, 관절의 속도 한계에 맞춰 시간을 입혀 궤적으로 만듦.
- 손끝의 위치로 목표를 줘도 됨. 그 자리에 닿는 관절 각도는 MoveIt 이 찾음.

### 주고받는 것

- 받는 것: 목표, 로봇의 설정, 장애물, 로봇의 지금 자세(`/joint_states` · `/tf`).
- 내는 것: 궤적. 강의 41 의 궤적 서버로 보내고, 그 뒤는 강의 41 과 같음.
- MoveIt 은 위치 궤적만 냄. 중력 보상과 목표 보간은 시뮬레이터가 그대로 함.

### 설정은 원본의 것을 그대로 씀

| 파일 | 내용 |
|---|---|
| `mobile_openarm.srdf` | 관절을 묶은 그룹(left_arm · right_arm · both_arms), 이름 붙은 자세, 서로 닿아도 되는 링크 쌍 |
| `kinematics.yaml` | 역기구학 풀이 방법 (KDL) |
| `joint_limits.yaml` | 계획할 때 쓰는 관절의 속도 · 가속도 한계 |
| `ompl_planning.yaml` | 경로를 찾는 계획기(OMPL)의 설정 |
| `moveit_controllers.yaml` | 궤적을 보낼 액션의 이름. 강의 41 의 궤적 서버가 이 이름으로 받음 |

- 원본 저장소의 `mobile_openarm_moveit_config` 패키지에 있음. 이 강의는 설정을 고치지 않고 빌드해서 씀.
- 런치 파일만 새로 만들었음. 원본의 런치는 MuJoCo 시뮬레이터가 내는 토픽으로 장애물을 넣는 노드까지 띄움.

### 터미널 네 개

| 터미널 | 띄우는 것 |
|---|---|
| 1 | Isaac Sim (`pl2m_moveit.py`) |
| 2 | 궤적 서버 (강의 41 의 `trajectory_server.py`) |
| 3 | MoveIt 과 RViz (`moveit.launch.py`) |
| 4 | 목표 보내기 (`move_arm.py`), 장애물 알리기 (`add_obstacle.py`) |

## 2. 준비

### 터미널: MoveIt 설정 빌드

```bash
cd ~/nvidia-dli-lecture
git pull
bash 42_pl2m_moveit/prepare_moveit.sh
```

- 강의 37 에서 받은 저장소 `~/pinklab_robotworld2026` 에서 MoveIt 설정 패키지를 빌드함. 한 번만 하면 됨.
- MoveIt 이 없으면 설치 명령을 알려 주고 멈춤.

### 실행 결과

```text
Summary: 6 packages finished [2.34s]
  설정을 읽었음: 계획기 ompl, 제어기 4개 (left_joint_trajectory_controller, left_gripper_controller, right_joint_trajectory_controller, right_gripper_controller)

준비됨. MoveIt 을 쓰는 터미널에서는 먼저 다음을 실행하세요.
  source /opt/ros/jazzy/setup.bash
  source ~/pinklab_robotworld2026/install/setup.bash
```

- 설정 패키지가 기대는 패키지까지 6개가 빌드됨. 처음 빌드할 때는 10초쯤 걸렸음.
- 제어기 4개의 이름이 강의 41 의 궤적 서버가 내는 액션 4개와 같음.

## 3. 띄우기

### 터미널 1: 시뮬레이터

```bash
source /opt/ros/jazzy/setup.bash
~/isaacsim/python.sh 42_pl2m_moveit/pl2m_moveit.py
```

- 강의 41 의 ROS 2 그래프 · 보간기 · 중력 보상을 그대로 불러 씀. 달라진 것은 장애물을 놓는 `--obstacle` 뿐임.
- `준비됨: 중력 보상 = both. 끝내려면 Ctrl+C` 가 나오면 준비된 것임.

### 터미널 2: 궤적 서버

```bash
source /opt/ros/jazzy/setup.bash
python3 41_pl2m_ros2/trajectory_server.py
```

### 터미널 3: MoveIt

```bash
source /opt/ros/jazzy/setup.bash
source ~/pinklab_robotworld2026/install/setup.bash
ros2 launch 42_pl2m_moveit/moveit.launch.py
```

- RViz 창이 뜨고, 터미널에 `You can start planning now!` 가 나오면 준비된 것임.
- RViz 없이 띄우려면 끝에 `rviz:=false` 를 붙임.

### 런치 파일

```python
def setup(context):
    params = parameters()                                  # URDF · SRDF · 기구학 · 관절 한계 · OMPL · 제어기(액션 이름)
    nodes = [
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": params["robot_description"], "use_sim_time": True}]),
        Node(package="moveit_ros_move_group", executable="move_group", output="screen", parameters=[params]),
    ]
```

- `parameters()` 는 원본 설정 패키지의 함수임. 1절의 설정 파일들을 읽어 한 묶음으로 돌려줌.
- `move_group` 이 MoveIt 의 본체임. 계획하고, 궤적을 액션으로 보냄.
- `robot_state_publisher` 는 `/joint_states` 로 링크 사이의 `/tf` 를 냄. RViz 가 로봇을 그리는 데 씀.

### MoveIt 이 로봇의 지금 자세를 아는 방법

| 무엇 | 어디서 |
|---|---|
| 관절의 각도 | Isaac Sim 이 내는 `/joint_states` |
| 베이스의 위치 | Isaac Sim 이 내는 `/tf` 의 odom → base_footprint |
| 링크의 위치 | `robot_state_publisher` 가 내는 `/tf` |
| 시각 | Isaac Sim 이 내는 `/clock` (`use_sim_time`) |

- 모두 강의 41 에서 내기 시작한 토픽임. MoveIt 을 위해 시뮬레이터 쪽을 고친 것은 없음.
- SRDF 가 로봇을 odom 에 평면으로 떠 있는 것으로 적어 둠. 그래서 odom → base_footprint 가 필요함.

## 4. 관절 목표: 이름 붙은 자세로

### 터미널 4

```bash
source /opt/ros/jazzy/setup.bash
source ~/pinklab_robotworld2026/install/setup.bash
python3 42_pl2m_moveit/move_arm.py both ready
python3 42_pl2m_moveit/move_arm.py left carry --plan-only
python3 42_pl2m_moveit/move_arm.py both carry
python3 42_pl2m_moveit/move_arm.py both home
```

- 첫 인자는 팔(left · right · both), 둘째는 SRDF 에 적힌 자세의 이름임 (home · transport · ready · carry · hands_up).
- `--plan-only` 는 계획만 하고 움직이지 않음.

### 실행 결과

```text
both_arms: ready 자세  →  성공
  궤적의 점 62개, 궤적의 길이 6.00초, 점마다 속도 있음
left_arm: carry 자세  →  성공
  궤적의 점 86개, 궤적의 길이 8.42초, 점마다 속도 있음, 계획에 걸린 시간 0.04초
both_arms: carry 자세  →  성공
  궤적의 점 86개, 궤적의 길이 8.40초, 점마다 속도 있음
both_arms: home 자세  →  성공
  궤적의 점 86개, 궤적의 길이 8.41초, 점마다 속도 있음
```

- 두 팔을 한 번에 계획하고 함께 움직임. MoveIt 이 궤적을 왼팔 · 오른팔의 액션 둘로 나눠 보냄.
- 궤적의 길이는 MoveIt 이 정함. 관절의 속도 · 가속도 한계의 0.3배로 움직이게 했을 때의 값임 (`--speed`).
- 점마다 속도가 들어 있음. 강의 41 의 궤적 서버가 속도 있는 점 사이를 3차 곡선으로 이음.

### 코드: 요청 만들기

```python
request.group_name = group
request.allowed_planning_time = 8.0
request.num_planning_attempts = 5
request.max_velocity_scaling_factor = speed           # 관절 속도 한계의 몇 배로 움직일지
request.max_acceleration_scaling_factor = speed
request.start_state.is_diff = True                    # 지금 자세에서 출발
request.goal_constraints = [constraints]
goal.planning_options.plan_only = plan_only
```

- `/move_action` 액션(moveit_msgs/action/MoveGroup)에 요청을 보냄. 어느 그룹을, 어떤 목표로, 얼마나 빨리 움직일지를 적음.
- 계획과 실행을 MoveIt 이 이어서 함. 결과에는 성공 여부와 계획한 궤적이 담겨 옴.

### 코드: 관절 목표

```python
return [JointConstraint(joint_name=j.get("name"), position=float(j.get("value")), tolerance_above=0.01, tolerance_below=0.01, weight=1.0)
        for j in state.findall("joint")]
```

- 목표는 "제약" 으로 적음. 관절마다 "이 각도의 0.01 rad 안" 이라는 제약을 줌.
- 각도는 SRDF 의 `group_state` 에서 읽음. 강의 39 의 자세 값과 같은 값임.

## 5. 자세 목표: 손끝의 위치로

### 터미널 4

```bash
python3 42_pl2m_moveit/move_arm.py left --xyz 0.50 0.152 0.78
python3 42_pl2m_moveit/move_arm.py left --shift 0.10 0 0.05
```

- `--xyz` 는 손끝(tcp)을 베이스 기준 그 자리로 보냄. x 앞, y 왼쪽, z 위, 단위는 m 임.
- `--shift` 는 지금 자리에서 그만큼 옮김. 손끝의 방향은 둘 다 지금 방향을 지킴.
- 팔을 `ready` 자세로 든 뒤에 해 볼 것. 내린 자세에서는 닿지 못하는 자리가 많음.

### 코드: 자세 목표

```python
where = PositionConstraint(link_name=tip(side), weight=1.0)
where.header.frame_id = BASE
where.constraint_region = BoundingVolume(primitives=[SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.001])], primitive_poses=[center])
facing = OrientationConstraint(link_name=tip(side), orientation=orientation, weight=1.0,
                               absolute_x_axis_tolerance=0.02, absolute_y_axis_tolerance=0.02, absolute_z_axis_tolerance=0.02)
```

- 위치 제약: 손끝 링크가 목표를 중심으로 한 반지름 1 mm 공 안에 있을 것.
- 방향 제약: 손끝의 방향이 정한 방향에서 0.02 rad 안에 있을 것.
- 이 제약을 만족하는 관절 각도는 MoveIt 이 역기구학(KDL)으로 찾음. 팔 관절이 7개라 답이 여럿일 수 있음.

### 한쪽 팔에만 중력 보상을 넣고 같은 자리로

```bash
~/isaacsim/python.sh 42_pl2m_moveit/pl2m_moveit.py --compensate left
```

```bash
python3 42_pl2m_moveit/move_arm.py both ready
python3 42_pl2m_moveit/move_arm.py left --xyz 0.50 0.152 0.78
python3 42_pl2m_moveit/move_arm.py right --xyz 0.50 -0.152 0.78
```

- 터미널 1 을 왼팔만 보상하도록 다시 띄우고, 두 손끝을 좌우로 같은 자리에 보냄.

### 실행 결과

```text
left_arm: 손끝을 (0.500, 0.152, 0.780) 로  →  성공
  궤적의 점 23개, 궤적의 길이 2.13초, 점마다 속도 있음
  손끝이 간 곳 (0.499, 0.153, 0.779),  목표와의 차이 x -1.1 · y +0.7 · z -0.9 mm  (거리 1.6 mm)
right_arm: 손끝을 (0.500, -0.152, 0.780) 로  →  성공
  궤적의 점 24개, 궤적의 길이 2.22초, 점마다 속도 있음
  손끝이 간 곳 (0.497, -0.151, 0.768),  목표와의 차이 x -3.4 · y +0.6 · z -12.4 mm  (거리 12.9 mm)
```

- 보상을 넣은 왼팔은 목표에서 1.6 mm, 보상이 없는 오른팔은 12.9 mm 떨어져 멈춤. 오른손은 12.4 mm 낮음.
- 둘 다 MoveIt 은 "성공" 이라고 함. 관절의 오차가 궤적 서버가 도착으로 인정하는 범위 안이기 때문임.
- "손끝이 간 곳" 은 `/tf` 에서 읽은 값임. 잰 관절 각도로 계산한 손끝의 위치임.

### 읽기

- MoveIt 은 기하만 따짐. 팔의 무게를 모름. 계획한 관절 각도에 팔이 실제로 가 있는지는 로봇 쪽에 달려 있음.
- 컵을 집으려면 손끝이 몇 mm 안에 가야 함. 강의 40 의 중력 보상이 여기서 필요해짐.

## 6. RViz 에서 마커로 움직이기

### 화면

- 터미널 3 에서 띄운 RViz 의 MotionPlanning 패널로, 명령을 치지 않고 팔을 움직임.
- 왼쪽이 패널, 가운데가 RViz 의 3D 화면, 오른쪽이 Isaac Sim 임.

### 순서

| 순서 | 하는 일 | 화면에서 일어나는 일 |
|---|---|---|
| 1 | Planning 탭의 Planning Group 을 `left_arm` 으로 둠 | 왼손 끝에 화살표와 고리가 달린 마커가 나타남 |
| 2 | 화살표를 끎 | 손끝이 그 방향으로 감. 파란 팔이 목표 자세임 |
| 3 | 고리(휠)를 끎 | 손끝의 방향이 그 축 둘레로 돎 |
| 4 | Plan & Execute 를 누름 | 지금 자세에서 목표까지의 경로를 계획함 |
| 5 | 기다림 | 계획한 궤적대로 RViz 와 Isaac Sim 의 팔이 움직임 |

- 마커를 끄는 동안 역기구학이 계속 풀려, 그 손끝 자세에 맞는 팔 모양이 파란 팔로 보임.
- 5절의 자세 목표를 마우스로 주는 것과 같음. 화살표는 위치를, 고리는 방향을 바꿈.

### 알아 둘 것

- 파란 팔이 실제 팔과 다른 곳에 있으면 Goal State 를 `<current>` 로 다시 고름. 목표가 지금 자세로 돌아옴.
- 3D 화면의 빈 곳을 끌면 시점이 돎. 마커가 잘 보이는 쪽으로 돌려 놓고 시작함.
- 패널의 Velocity Scaling 은 0.10 임. `move_arm.py` 의 0.3 보다 느리게 움직임.

## 7. 장애물

### 터미널 1: 기둥과 컵을 놓고 띄우기

```bash
~/isaacsim/python.sh 42_pl2m_moveit/pl2m_moveit.py --obstacle
```

- 왼팔 앞에 높이 0.47 m 의 기둥을 세우고 그 위에 컵(0.2 kg)을 올려 둠. 왼팔이 `home` 에서 `ready` 로 갈 때 손끝이 지나는 자리임.
- `준비됨: 중력 보상 = both, 장애물 있음. 끝내려면 Ctrl+C` 가 나옴. 터미널 2 와 3 도 다시 띄움.

### 장애물을 모를 때

```bash
python3 42_pl2m_moveit/move_arm.py left ready
```

```text
left_arm: ready 자세  →  성공
  궤적의 점 62개, 궤적의 길이 6.02초, 점마다 속도 있음
컵이 움직였음: 0.6 cm
컵이 움직였음: 1.4 cm
컵이 움직였음: 3.0 cm
컵이 움직였음: 6.5 cm
컵이 움직였음: 14.4 cm
컵이 움직였음: 29.3 cm
컵이 처음 자리에서 움직인 거리: 487.0 mm
```

- 위 두 줄은 터미널 4 에, "컵이 움직였음" 은 터미널 1 에 나옴. 마지막 줄은 터미널 1 을 Ctrl+C 로 끝낼 때 나옴.
- MoveIt 은 "성공" 이라고 함. 팔은 목표에 갔기 때문임. 컵이 있는지 몰랐음.

### 터미널 4: 장애물 알리기

```bash
python3 42_pl2m_moveit/add_obstacle.py
python3 42_pl2m_moveit/move_arm.py left ready
```

```text
장애물 post_and_cup 을 넣었음: 가운데 (0.38, 0.15, 0.29), 크기 (0.08, 0.08, 0.58)
left_arm: ready 자세  →  성공
  궤적의 점 86개, 궤적의 길이 8.44초, 점마다 속도 있음
컵이 처음 자리에서 움직인 거리: 0.0 mm
```

- 터미널 1 을 다시 띄워 컵을 제자리에 놓고, 이번에는 장애물을 알린 뒤 같은 목표를 보냄.
- 궤적이 6.02초에서 8.44초로 길어짐. 돌아가는 길임. 컵은 0.0 mm 움직였음.

### 손끝이 지난 길

- 주황: 장애물을 모를 때. 컵이 있는 자리를 그대로 지나감.
- 파랑: 알려 준 뒤. 위에서 본 모습에서 왼쪽으로 8 cm 쯤 비켜 상자를 돌아감.
- 옆에서 보면 두 길이 비슷해 보임. 비켜 간 방향이 화면 안쪽이기 때문임.

### 코드: 장애물 알리기

```python
box = CollisionObject(id="post_and_cup")
box.header.frame_id = "odom"                             # 로봇이 출발한 자리 기준
box.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=list(BOX_SIZE))]
box.primitive_poses = [pose]
box.operation = CollisionObject.ADD
scene = PlanningScene(is_diff=True)
scene.world.collision_objects = [box]
done = client.call_async(ApplyPlanningScene.Request(scene=scene))
```

- 기둥과 컵을 한꺼번에 감싸는 상자 하나를 MoveIt 의 계획 장면(planning scene)에 넣음. 컵보다 옆으로 1 cm, 위로 2 cm 크게 잡았음.
- 자리와 크기는 `obstacle.py` 에 적어 두고 시뮬레이터와 이 스크립트가 같이 읽음.
- `--remove` 를 붙이면 뺌.

### RViz 에 보이는 계획 장면

- 초록 상자가 MoveIt 이 알고 있는 장애물임. MoveIt 이 보는 세상은 이것이 전부임.
- MoveIt 은 시뮬레이터 안을 보지 못함. 무엇이 어디에 있는지는 누군가 알려 줘야 함. 뒤 강의에서는 부스의 테이블과 음료대를 이렇게 넣음.

## 8. 계획이 실패할 때

### 한 번 실패했음

- 장애물을 넣고 녹화하던 중에 계획이 한 번 `INVALID_MOTION_PLAN` 으로 실패했음. 같은 요청을 다시 보내니 성공했음.
- OMPL 은 무작위로 길을 찾음. 같은 요청에도 매번 다른 궤적이 나오고(점 72개, 71개, 77개, 80개, 86개), 가끔 실패함.
- 그래서 `move_arm.py` 는 실패하면 다시 계획함. 기본 세 번까지임 (`--tries`).

### 코드

```python
for attempt in range(1, args.tries + 1):
    result = arm.move(group, constraints, args.plan_only, args.speed)
    code = result.error_code.val
    if code == MoveItErrorCodes.SUCCESS:
        break
    print(f"{group}: {attempt}번째 시도 실패 ({ERRORS.get(code, str(code))})" + (", 다시 계획함" if attempt < args.tries else ""))
```

- 실패의 이유는 오류 코드로 옴. 닿을 수 없는 자리를 주면 다시 해도 실패함.

## 9. 해 볼 것

### 직접 바꿔 보기

- `move_arm.py left ready --speed 0.6` 으로 궤적의 길이가 어떻게 달라지는지 보기.
- `ready` 자세에서 `move_arm.py left --shift 0.5 0 0` 처럼 닿지 못하는 자리를 주고, 어떤 오류 코드가 나오는지 보기.
- `obstacle.py` 의 `BOX_SIZE` 에서 옆 여유를 늘려 상자를 0.20 m 폭으로 만들고, 손끝이 지나는 길이 어떻게 달라지는지 RViz 에서 보기.
- RViz 에서 Planning Group 을 `right_arm` 으로 바꿔 오른팔을 마커로 움직여 보기.

## 10. 정리

### 기억할 것

- MoveIt 에는 목표만 줌. 관절 목표(각도)와 자세 목표(손끝의 위치 · 방향) 두 가지가 있음.
- MoveIt 은 부딪히지 않는 길을 찾고 시간을 입혀 궤적으로 만들어, 강의 41 의 궤적 서버로 보냄.
- MoveIt 은 알려 준 것만 피함. 모르는 컵은 쳤고(487.0 mm), 알려 주니 돌아갔음(0.0 mm).
- MoveIt 은 팔의 무게를 모름. 같은 자리로 보냈을 때 보상한 팔은 1.6 mm, 보상 없는 팔은 12.9 mm 어긋났음.
- 계획은 가끔 실패함. 다시 계획하게 해 둠.

### 한계

- 장애물은 손으로 적어 넣은 상자 하나임. 카메라나 라이다로 장애물을 알아내는 것은 하지 않았음.
- 부스 안에서는 해 보지 않았음. 테이블과 음료대를 계획 장면에 넣는 것은 뒤 강의에서 함.
- 실패는 한 번 보았을 뿐임. 얼마나 자주 실패하는지 세어 보지 않았음.
- 손끝의 오차는 `/tf` 로 계산한 값임. 시뮬레이터 안의 손 위치를 직접 잰 것이 아님.
- 그리퍼를 MoveIt 으로 움직이는 것, 물체를 집는 것은 아직 하지 않았음.
- 강의 PC 한 대(Isaac Sim 5.1, ROS 2 Jazzy, MoveIt 2)에서 확인한 결과임.
