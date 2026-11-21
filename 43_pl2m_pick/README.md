# 43. 컵 집기: 그리퍼로 쥐고, 들고, 손에 든 무게를 알리기 — 명령 모음

- 영상에서 입력하는 터미널 명령과 기대 출력을 순서대로 모았음. [42. MoveIt 으로 팔의 경로 계획하기](../42_pl2m_moveit/README.md) 다음 내용임.

| 파일 |
|---|
| [pick_cup.py](pick_cup.py) |
| [pl2m_pick.py](pl2m_pick.py) |
| [scene.py](scene.py) |

## 1. 집는다는 것

### 이번 강의의 결과

- 탁자 위의 컵 둘을 왼팔과 오른팔이 하나씩 집어 듦. 무게 0.2 kg, 지름 5 cm 의 컵임.
- 강의 42 까지는 팔을 움직이기만 했음. 이번에는 물체를 쥐고, 든 채로 움직임.

### 집는 순서

| 단계 | 하는 일 | 값 (원본의 설정) |
|---|---|---|
| 1 | 탁자와 컵을 MoveIt 에 알림 | |
| 2 | 그리퍼를 다 엶 | 손가락마다 44 mm |
| 3 | 접근 시작 자리로 감 | 컵의 뒤 10 cm, 컵의 가운데보다 2 cm 위 |
| 4 | 손끝을 직선으로 컵까지 보냄 | 10 cm |
| 5 | 그리퍼를 닫음 | 12 mm 까지 닫으라고 함. 25 mm 쯤에서 멈추면 쥔 것 |
| 6 | 든 무게를 로봇에, 붙은 것을 MoveIt 에 알림 | |
| 7 | 들고, 뒤로 뺌 | 4 cm, 14 cm |

- 순서와 값은 원본의 집기 코드(`warehouse_skills` 의 grasp)에서 가져왔음. 이 강의는 그것을 한 단계씩 풀어 씀.

### 새로 생기는 문제 셋

| 문제 | 어디서 다루나 |
|---|---|
| 그리퍼가 컵을 쥐지 못함 (가져온 그대로는 두 가지가 틀려 있었음) | 4절 |
| 얼마나 무거운 것까지 쥘 수 있나 (쥐는 힘과 마찰) | 5절 |
| 든 물체의 무게 때문에 팔이 처짐 (강의 40 의 한계) | 6절 |

### 만드는 것

| 파일 | 어디서 도나 | 하는 일 |
|---|---|---|
| `scene.py` | 양쪽 | 탁자와 컵의 자리, 집는 방법의 값 |
| `pl2m_pick.py` | Isaac Sim | 탁자 · 컵 · 손가락 설정 · 든 무게의 보상 |
| `pick_cup.py` | ROS 2 | 집는 순서 일곱 단계 |

- 터미널은 강의 42 와 같이 넷을 씀. Isaac Sim, 궤적 서버, MoveIt, 명령을 치는 곳임.

## 2. 띄우기

### 터미널 1 · 2 · 3

```bash
cd ~/nvidia-dli-lecture
git pull
source /opt/ros/jazzy/setup.bash
~/isaacsim/python.sh 43_pl2m_pick/pl2m_pick.py
```

```bash
source /opt/ros/jazzy/setup.bash
python3 41_pl2m_ros2/trajectory_server.py
```

```bash
source /opt/ros/jazzy/setup.bash
source ~/pinklab_robotworld2026/install/setup.bash
ros2 launch 42_pl2m_moveit/moveit.launch.py rviz:=false
```

- 터미널 1 에 `준비됨: 중력 보상 = both, 컵 0.2 kg. 끝내려면 Ctrl+C` 가 나옴.
- 궤적 서버와 MoveIt 은 강의 41 · 42 의 것을 그대로 씀. RViz 로 보려면 `rviz:=false` 를 뺌.

### 장면의 값

```python
TABLE_TOP = {"center": (0.58, 0.0, 0.735), "size": (0.30, 0.60, 0.03)}      # 상판: 앞 가장자리 x = 0.43, 윗면 z = 0.75
CUP_DIAMETER, CUP_HEIGHT = 0.05, 0.09                                        # 원본 worlds/booth.yaml 의 컵
CUP_MASS = 0.2                                                               # 내용물이 든 컵으로 보고 200 g 으로 가정 (원본은 0.1)
CUP_FRICTION = 1.2                                                           # 원본 MuJoCo 모델의 물체 마찰
CUPS = {  # 팔 → (이름, 가운데, 색)
    "left": ("cup_pink", (0.485, 0.15, 0.795), (0.87, 0.42, 0.65)),
    "right": ("cup_blue", (0.485, -0.15, 0.795), (0.28, 0.39, 0.68)),
}
```

- 로봇이 원본의 음료대 앞에 섰을 때와 같은 자리임. 탁자 가장자리까지 0.43 m, 거기서 컵까지 0.055 m.
- 컵의 무게는 200 g 으로 가정함. 내용물이 든 컵으로 본 것이고, 원본의 값은 100 g 임.
- 컵의 자리는 미리 알고 있는 것으로 함. 카메라로 컵을 찾는 것은 이 강의에서 하지 않음.

## 3. 컵 집기

### 터미널 4

```bash
source /opt/ros/jazzy/setup.bash
source ~/pinklab_robotworld2026/install/setup.bash
python3 42_pl2m_moveit/move_arm.py both ready
python3 43_pl2m_pick/pick_cup.py left
```

- 팔을 `ready` 자세로 든 뒤, 왼팔로 분홍 컵을 집음. `right` 를 주면 오른팔로 파란 컵을 집음.

### 실행 결과

```text
1. 장면 알리기: 탁자와 컵  →  됨
2. 그리퍼 열기  →  손가락 42.0 mm (둘째 손가락 42.0 mm)
3. 접근 시작 자리 (0.385, 0.150, 0.815)  →  도착
4. 곧게 접근 10 cm  →  직선의 100 % 를 감, 손끝 (0.486, 0.150, 0.814)
5. 그리퍼 닫기  →  손가락이 27.0 mm 에서 멈춤 (둘째 손가락 27.7 mm): 쥐었음
   손가락 관절에 걸린 힘: 첫째 -12.9 N, 둘째 -14.1 N
6. 무게 알리기  →  로봇에 0.2 kg, MoveIt 에는 컵이 손에 붙었다고 알림
7. 들기 4 cm · 빼기 14 cm  →  직선의 100 % · 100 %
   손끝의 높이 0.8541 m (목표 0.8550, 차이 -0.9 mm),  손가락 27.0 mm
```

- 손가락이 12 mm 까지 가지 못하고 27 mm 에서 멈췄음. 사이에 컵이 있다는 뜻임.
- 든 뒤에도 손가락이 27.0 mm 그대로임. 컵을 쥐고 있음.

### 터미널 1 에 나오는 것

```text
부하 설정: 왼팔 0.20 kg
  55.4초  cup_pink: 높이 0.815 m, 기울기 1°, 옆으로 0.7 cm   손가락이 누르는 힘 13.5 · 13.5 N, 마찰로 받치는 힘 -5.2 · 7.1 N
  57.3초  cup_pink: 높이 0.833 m, 기울기 1°, 옆으로 5.7 cm   손가락이 누르는 힘 13.5 · 13.6 N, 마찰로 받치는 힘 6.0 · -4.0 N
  58.2초  cup_pink: 높이 0.834 m, 기울기 1°, 옆으로 10.7 cm   손가락이 누르는 힘 13.5 · 13.5 N, 마찰로 받치는 힘 -5.0 · 6.9 N
```

- 시뮬레이터가 컵의 움직임을 알려 줌. 높이가 0.795 m 에서 0.834 m 로 4 cm 올라가고, 옆으로 14 cm 옮겨짐.
- 손가락이 컵을 누르는 힘과 마찰력도 함께 나옴. 5절에서 봄.

### 코드: 접근 시작 자리와 손의 방향

```python
# 앞으로 뻗어 옆에서 쥐는 손의 방향 (x, y, z, w): 손의 z(나아가는 쪽) → 앞, y(손가락이 닫히는 쪽) → 왼쪽, x → 아래
GRASP_FORWARD = (0.0, 0.7071068, 0.0, 0.7071068)
```

- 강의 42 의 자세 목표로 감. 위치는 컵의 뒤 10 cm, 방향은 이 값임. 원본의 `GRASP_FORWARD` 와 같은 값임을 계산해 확인했음.
- 손끝의 높이는 컵의 가운데보다 2 cm 위임. 손가락이 손끝보다 3 cm 아래까지 내려와서, 탁자와 닿지 않게 띄우는 것임.

### 코드: 곧게 접근

```python
request = GetCartesianPath.Request()
request.header.frame_id = BASE
request.start_state.is_diff = True
request.group_name, request.link_name = f"{side}_arm", tip(side)
request.waypoints = [pose_at(x, y, z)]
request.max_step = 0.01                                   # 1 cm 마다 역기구학을 푼다
request.avoid_collisions = True
request.max_velocity_scaling_factor = request.max_acceleration_scaling_factor = speed
answer = self.call(self.cartesian, request)
```

- 컵에 다가갈 때는 손끝이 직선으로 가야 함. 강의 42 의 계획은 길의 모양을 정해 주지 않음.
- `/compute_cartesian_path` 서비스가 손끝의 직선 길을 1 cm 마다 풀어 궤적으로 만듦. 결과의 `fraction` 이 직선의 몇 % 를 갈 수 있는지임.
- 만든 궤적은 `/execute_trajectory` 액션으로 실행함.

### 코드: 그리퍼 닫기와 판정

```python
position, reached, stalled = arm.grip(side, GRASP["close"])
held = GRASP["close_min"] <= position <= GRASP["close_max"]
```

- 12 mm 까지 닫으라고 함. 컵의 지름이 50 mm 라 손가락마다 25 mm 쯤에서 막힘. 멈춘 자리가 16 ~ 28 mm 이면 쥔 것으로 봄.
- 강의 41 의 궤적 서버는 1초 동안 움직이지 않으면 멈춘 것으로 보고 결과를 돌려줌.
- 이번에 궤적 서버의 그리퍼를 초속 3 cm 로 움직이게 고쳤음 (`GRIP_SPEED`). 목표를 한 번에 바꾸지 않음.

### 코드: MoveIt 에 붙었다고 알리기

```python
item = AttachedCollisionObject(link_name=f"openarm_{side}_hand")
item.object.id = name
item.object.operation = CollisionObject.ADD if attach else CollisionObject.REMOVE
item.touch_links = [f"openarm_{side}_hand", f"openarm_{side}_left_finger", f"openarm_{side}_right_finger", tip(side)]
return self.apply(attached=[item])
```

- 쥔 뒤에는 손가락과 컵이 닿아 있음. 그대로 두면 MoveIt 은 충돌로 보고 계획을 거부함.
- 컵을 손에 붙은 물체로 바꾸면, 손 · 손가락과 닿는 것은 허용하고 컵이 탁자 같은 다른 것에 닿지 않게 계획함.

## 4. 가져온 그대로는 쥐지 못했음

### 첫째: 둘째 손가락이 밀려 벌어짐

```bash
~/isaacsim/python.sh 43_pl2m_pick/pl2m_pick.py --soft-mimic
```

```text
5. 그리퍼 닫기  →  손가락이 13.6 mm 에서 멈춤 (둘째 손가락 41.9 mm): 놓쳤음
끝날 때: cup_pink 높이 0.795 m · 기울기 0° · 옆으로 1.4 cm,  cup_blue 높이 0.795 m · 기울기 0° · 옆으로 0.0 cm
```

- 첫째 손가락은 14 mm 까지 닫혔는데 둘째 손가락은 42 mm 로 벌어져 있음. 컵은 옆으로 1.4 cm 밀렸음.
- 두 손가락 사이는 컵의 지름만큼임. 컵을 사이에 두기는 했지만, 둘째 손가락이 버티지 못하고 밀려난 것임.

### 원인: 손가락의 묶음이 용수철이었음

```python
for side in ("left", "right"):
    joint = stage.GetPrimAtPath(f"{JOINTS}/openarm_{side}_finger_joint2")
    for name in ("naturalFrequency", "dampingRatio"):
        joint.GetAttribute(f"physxMimicJoint:rotY:{name}").Set(0.0)
```

- 그리퍼는 첫째 손가락만 구동하고, 둘째 손가락은 첫째를 따라 움직이게 묶여 있음 (강의 37 의 `PhysxMimicJointAPI`).
- 가져온 값은 고유 진동수 25, 감쇠비 0.005 였음. 묶음이 무른 용수철임. 빈손일 때는 잘 따라왔지만(강의 39), 컵에 닿자 밀렸음.
- 둘을 0 으로 두면 단단한 구속이 됨. `pl2m_pick.py` 는 기본으로 이렇게 함.

### 둘째: 오른손의 한쪽 손가락이 컵을 통과함

```bash
~/isaacsim/python.sh 43_pl2m_pick/pl2m_pick.py --shared-shape
```

```text
5. 그리퍼 닫기  →  손가락이 13.6 mm 에서 멈춤 (둘째 손가락 12.8 mm): 놓쳤음
끝날 때: cup_pink 높이 0.795 m · 기울기 0° · 옆으로 0.0 cm,  cup_blue 높이 0.795 m · 기울기 0° · 옆으로 1.4 cm
```

- 오른팔로 집은 결과임. 두 손가락이 모두 끝까지 닫혔고, 컵은 한쪽 손가락이 있는 자리로 1.4 cm 밀려 들어가 있었음.
- 왼손은 같은 조건에서 쥐었음. 오른손의 `openarm_right_right_finger` 만 컵과 충돌하지 않았음.

### 원인: 충돌 모양을 함께 쓰고 있었음

```python
material = friction_material(stage, "finger", value)
for link in FINGERS:
    collisions = stage.GetPrimAtPath(f"{ROBOT}/{link}/collisions")
    if own_shape:
        collisions.SetInstanceable(False)
```

- 가져온 로봇은 같은 모양의 충돌체를 여러 링크가 함께 씀(인스턴스). 강의 39 에서 바퀴를 고칠 때 풀었던 것과 같은 것임.
- 손가락 넷의 인스턴스를 풀어 각자 충돌 모양을 갖게 하니 오른손도 쥐었음.
- 왜 그 손가락 하나만 충돌하지 않았는지는 가려내지 못했음. 가져올 때 그 링크에만 경고가 나왔다는 것까지 보았음.

### 고친 뒤: 두 팔 모두

```text
5. 그리퍼 닫기  →  손가락이 27.0 mm 에서 멈춤 (둘째 손가락 27.8 mm): 쥐었음
7. 들기 4 cm · 빼기 14 cm  →  직선의 100 % · 100 %
   손끝의 높이 0.8545 m (목표 0.8550, 차이 -0.5 mm),  손가락 27.0 mm
```

- `pick_cup.py right` 의 결과임. 오른팔도 27.0 mm 에서 쥐고, 들고, 뺐음.
- 가져온 로봇이 움직이는 것과 쥐는 것은 다름. 쥐려면 접촉이 맞아야 함.

## 5. 쥐는 힘과 마찰

### 수식

- 손가락이 막혀 멈추면, 구동은 목표와 멈춘 자리의 차이만큼 힘을 냄. 그 힘이 두 손가락에 반씩 나뉨.
- 컵이 흘러내리지 않게 받치는 것은 마찰력임. 마찰력은 누르는 힘에 마찰 계수를 곱한 값을 넘지 못함.

### 누르는 힘은 수식대로였음

| | 수식 | 잰 값 |
|---|---|---|
| 손가락 하나가 컵을 누르는 힘 | 1800 × (0.0271 − 0.012) ÷ 2 = 13.6 N | 13.4 ~ 13.7 N |
| 한쪽 손가락의 마찰력의 한계 (마찰 1.2) | 1.2 × 14.1 = 16.9 N | 16.7 ~ 16.9 N |

- 누르는 힘은 3절의 터미널 1 출력에 나온 값임. 두 손가락이 같은 힘으로 누름.
- 마찰력의 한계는 2 kg 컵을 쥐었을 때 잰 값임. 한쪽 손가락의 마찰력이 16.9 N 에서 더 오르지 않았음.

### 마찰은 한쪽이 거의 다 냈음

```text
  27.1초  cup_pink: 높이 0.815 m, 기울기 1°, 옆으로 0.4 cm   손가락이 누르는 힘 13.5 · 13.6 N, 마찰로 받치는 힘 0.4 · 9.4 N
  28.9초  cup_pink: 높이 0.833 m, 기울기 1°, 옆으로 5.5 cm   손가락이 누르는 힘 13.5 · 13.6 N, 마찰로 받치는 힘 0.4 · 9.4 N
```

- 1 kg 컵을 든 기록임. 무게 9.8 N 을 한쪽 손가락이 9.4 N, 다른 쪽이 0.4 N 받치고 있음.
- 두 손가락이 반씩 나눠 받지 않았음. 그래서 버티는 조건이 2 μ N 이 아니라 μ N 에 가까움.
- 어느 쪽이 받는지는 실행마다, 움직이는 동안에도 바뀌었음.

### 무게와 마찰을 바꿔 가며

```bash
~/isaacsim/python.sh 43_pl2m_pick/pl2m_pick.py --cup-mass 1.0 --finger-friction 0.7
python3 43_pl2m_pick/pick_cup.py left --mass 1.0
```

| 컵 | 손가락 마찰 | 무게 ÷ (마찰 × 누르는 힘) | 결과 |
|---|---|---|---|
| 0.2 kg | 1.2 | 0.12 | 쥠 |
| 1.0 kg | 1.2 | 0.60 | 쥠 |
| 0.5 kg | 0.5 | 0.72 | 3 cm 미끄러진 뒤 쥠 |
| 1.0 kg | 0.9 | 0.80 | 2 cm 미끄러진 뒤 쥠 |
| 1.5 kg | 1.2 | 0.90 | 2 cm 미끄러진 뒤 쥠 |
| 0.5 kg | 0.4 | 0.90 | 떨어짐 |
| 1.0 kg | 0.7 | 1.03 | 떨어짐 |
| 2.0 kg | 1.2 | 1.20 | 든 채로 계속 미끄러짐 |

- 누르는 힘은 13.6 N 으로 놓고 계산했음. 셋째 칸이 0.6 이하일 때만 미끄러지지 않고 쥐었음. 첫 줄이 이 강의의 컵임.
- 0.7 ~ 0.9 에서는 미끄러진 뒤 멈추거나 떨어졌음. 1 을 넘으면 버티지 못함.
- 수식의 한계까지 쓰면 안 됨. 여유를 둬야 함. 이 강의의 0.2 kg 컵은 0.12 라 여유가 큼.

### 코드: 손가락이 컵에 주는 힘 읽기

```python
forces = self.views[name].get_contact_force_matrix(dt)[0]              # 손가락 2개 × (x, y, z): 면에 수직인 힘
friction, _, counts, starts = self.views[name].get_friction_data(dt)   # 마찰력은 따로 온다: 접촉점마다 (x, y, z)
lift = [float(friction[int(starts[0][k]):int(starts[0][k]) + int(counts[0][k]), 2].sum()) for k in range(len(forces))]
return [abs(float(force[1])) for force in forces], lift
```

- 강의 40 의 저울과 같은 접촉 뷰를 씀. 이번에는 컵을 "재는 물체" 로, 두 손가락을 "상대" 로 정해 둘 사이의 힘만 읽음.
- 누르는 힘과 마찰력이 따로 나옴. 누르는 힘만 읽었을 때는 받치는 힘이 0 으로 보였음.

## 6. 손에 든 무게를 알리기

### 강의 40 의 한계가 여기서 드러남

- 강의 40 의 중력 보상은 팔의 무게만 계산함. 손에 든 물체는 모델에 없음.
- 컵을 들면 그 무게만큼 팔이 처짐. 상용 로봇팔에서 집을 때마다 부하를 바꿔 줘야 하는 이유와 같음 (강의 40 의 13절).

### 수식

- 물체를 손끝에 달린 점 질량으로 봄. 강의 40 의 수식에 링크 하나가 더 붙는 것과 같음.
- 관절마다 "물체의 무게 × 그 관절 축에서 손끝까지의 수평 거리" 를 더함.

### 코드

```python
up = Gf.Vec3d(0, 0, 1)
for side, mass in self.payload.items():
    if mass <= 0.0:
        continue
    tip = self.world(f"openarm_{side}_hand").Transform(Gf.Vec3d(0, 0, TIP_OFFSET))
    for dof, link, point, axis in self.chain[side]:
        frame = self.world(link)
        arm = Gf.Dot(frame.TransformDir(axis).GetNormalized(), Gf.Cross(tip - frame.Transform(point), up))
        efforts[0, dof] += mass * GRAVITY * arm
```

- 강의 40 의 `GravityCompensation` 을 이어받은 `PayloadCompensation` 임. 팔의 중력 보상에 이 토크를 더해 넣음.
- 무게는 `/payload` 토픽으로 받음. `pick_cup.py` 가 6단계에서 보냄. 실제 로봇의 "부하 설정" 명령에 해당함.
- 토픽은 JointState 를 빌려 썼음. `name` 에 팔, `position` 에 무게 [kg] 를 넣음.

### 터미널: 든 채로 알린 무게를 바꿔 보기

```bash
~/isaacsim/python.sh 43_pl2m_pick/pl2m_pick.py --cup-mass 1.0
python3 43_pl2m_pick/pick_cup.py left --mass 1.0 --compare
```

```text
   팔 관절 1 ~ 7 [°]: 50.1 -149.4 -17.0 133.7 14.9 -31.6 84.3
      알린 무게[kg]     손끝의 높이[m]    내려간 거리[mm]
           1.00        0.8542           0.0
           0.50        0.8508           3.3
           0.00        0.8474           6.7
           1.00        0.8541           0.0
```

- 1 kg 컵을 든 채, 로봇에 알린 무게만 1.0 → 0.5 → 0 → 1.0 kg 으로 바꿨음. 팔의 자세는 그대로임.
- 맞게 알리면 손끝이 목표보다 0.8 ~ 0.9 mm 낮고, 알리지 않으면 거기서 6.7 mm 더 내려감. 다시 알리면 돌아옴.
- 이 강의의 0.2 kg 컵으로 같은 것을 하면, 알리지 않았을 때 0.9 mm 내려갔음. 한 번 잰 값임.

### 그림으로

- 처짐은 로봇이 모르는 무게에 비례함. 세 번 실행한 것이 모두 직선임.
- 기울기는 실행마다 달랐음. 1 kg 에 4.5 mm, 6.7 mm, 9.7 mm.

### 같은 손끝 자리, 다른 팔 모양

- 손끝의 자리와 방향은 세 번 모두 같았음. 달랐던 것은 팔 모양임. 팔 관절이 7개라 같은 손끝 자세에 답이 여럿임.
- MoveIt 이 접근 시작 자리로 갈 때 고른 답이 실행마다 달랐고, 그 팔 모양에 따라 같은 무게에도 처짐이 2배 달랐음.
- 처음에는 0.5 kg 과 1 kg 의 처짐이 비슷하게 나와 측정이 틀린 줄 알았음. 팔 관절 각도를 같이 적어 보고서야 이유를 알았음.

### 무게를 틀리게 알리면

- 알리지 않고 0.5 kg 컵을 들고 뒤로 빼다가, 궤적이 한 번 실패로 끝났음. 궤적 서버가 "목표에 들어오지 못함: 가장 큰 오차 2.48°" 라고 했음.
- 컵이 빠져 버렸는데 1 kg 이라고 알린 채로 두면, 손끝이 목표보다 8.9 mm 위로 떠 있었음. 없는 무게를 받치고 있는 것임.
- 놓을 때는 무게를 0 으로 되돌려야 함. `pick_cup.py --put-back` 이 그렇게 함.

## 7. 내려놓기

### 터미널 4

```bash
python3 43_pl2m_pick/pick_cup.py left --put-back
```

```text
8. 제자리에 내려놓기  →  손가락 42.2 mm, 무게 0 kg 으로 되돌림
```

```text
부하 설정: 왼팔 0.00 kg
끝날 때: cup_pink 높이 0.795 m · 기울기 0° · 옆으로 0.1 cm,  cup_blue 높이 0.835 m · 기울기 0° · 옆으로 14.0 cm
```

- 들었다가 같은 길을 되짚어 가서, 탁자 위 4 mm 에서 그리퍼를 엶. 분홍 컵이 처음 자리로 돌아왔음 (높이 0.795 m, 옆으로 0.1 cm).
- 놓은 뒤에는 MoveIt 에 컵이 손에서 떨어졌다고 알리고, 로봇에는 무게를 0 kg 으로 알림.
- 파란 컵은 앞서 오른팔이 든 채로 있음 (높이 0.835 m).

## 8. 해 볼 것

### 직접 바꿔 보기

- `--soft-mimic` 과 `--shared-shape` 로 각각 띄워, 4절의 두 실패를 직접 보기.
- `--cup-mass 1.0 --finger-friction 0.8` 로 띄우고 집어, 5절 표의 어느 줄에 가까운지 보기.
- `pick_cup.py left --mass 1.0 --compare` 를 여러 번 돌려, 팔 관절 2번의 각도와 1 kg 당 처짐이 어떻게 짝지어지는지 적어 보기.
- `scene.py` 의 `GRASP["z_offset"]` 을 0.0 으로 바꿔, 손가락이 탁자에 닿는지 보기.

## 9. 정리

### 기억할 것

- 집는 순서: 열기 → 뒤에서 곧게 다가가기 → 닫기 → 붙었다고 알리기 → 들기. 다가가고 드는 길은 직선 경로로 만듦.
- 가져온 로봇은 그대로는 쥐지 못했음. 손가락의 묶음을 단단하게 하고, 손가락의 충돌 모양을 따로 갖게 해야 했음.
- 손가락이 누르는 힘은 kp × (멈춘 자리 − 목표) ÷ 2 로 13.6 N. 무게가 마찰 × 누르는 힘의 0.6배 이하일 때만 미끄러지지 않았음.
- 든 물체의 무게는 로봇에 알려야 함. 알리지 않으면 1 kg 에 4.5 ~ 9.7 mm 처지고, 그 크기는 팔 모양에 따라 달랐음.
- 실패의 원인은 숫자를 읽어야 보였음. 두 손가락의 위치, 접촉 힘, 팔 관절 각도를 찍어 보고 알았음.

### 한계

- 평평한 바닥의 작은 탁자에서, 자리를 미리 아는 컵을 집은 결과임. 부스 안에서, 컵을 찾아서 집는 것은 하지 않았음.
- 집기는 조건마다 한두 번씩 해 본 것임. 얼마나 자주 실패하는지 세어 보지 않았음.
- 마찰을 한쪽 손가락이 거의 다 내는 것은 이 시뮬레이터에서 본 것임. 실제 그리퍼에서 그런지는 확인하지 않았음.
- 오른손의 손가락 하나가 충돌하지 않았던 원인은 찾지 못했음. 인스턴스를 풀면 고쳐진다는 것만 확인했음.
- MoveIt 이 고르는 팔 모양을 정해 주지 않았음. 그래서 처짐의 크기가 실행마다 달랐음.
- 든 물체는 손끝의 점으로 보았음. 크거나 길쭉한 물체에서는 맞지 않을 수 있음.
- 강의 PC 한 대(Isaac Sim 5.1, ROS 2 Jazzy, MoveIt 2)에서 확인한 결과임.
