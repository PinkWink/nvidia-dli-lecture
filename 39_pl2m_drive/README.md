# 39. PL-2M 움직이기: 바퀴 · 팔 · 그리퍼의 구동 값 — 명령 모음

- 영상에서 입력하는 터미널 명령과 기대 출력을 순서대로 모았음. [38. 핑크랩 부스 만들기](../38_pl2m_booth/README.md) 다음 내용임.

| 파일 |
|---|
| [pl2m_drive.py](pl2m_drive.py) |

## 1. 가져온 로봇은 아직 움직이지 못함

### 이번 강의의 결과

- 팔을 주행 자세로 접고, 1 m 직진하고, 왼쪽으로 돌고, 팔을 운반 자세로 들고, 그리퍼를 열고 닫음.
- 강의 37 에서 가져온 그대로는 바퀴로 가지 못하고 그리퍼는 출렁임. 이번 강의에서 하나씩 재 보며 고침.

### 무엇을 정해야 하나

| 부분 | 가져온 그대로의 문제 | 이 강의에서 정하는 것 |
|---|---|---|
| 바퀴 | 바퀴를 돌려도 로봇이 가지 않음 | 캐스터 마찰, 바퀴 관절의 관성, 바퀴 충돌 모양, 모터 값 |
| 팔 | 세기가 관성에 따라 제각각 | 구동 값(세기 · 감쇠), 팔 링크의 중력 |
| 그리퍼 | 너무 약해 출렁임 | 구동 값, 최대 힘 |

- URDF 에는 로봇의 모양과 질량은 있지만 모터의 세기는 없음. 강의 37 의 값은 Importer 가 임시로 정한 것임.

### 값의 출처

| 부분 | 원본(MuJoCo 설정)의 값 |
|---|---|
| 바퀴 모터 | 속도 게인 10 N·m·s/rad, 최대 토크 18 N·m |
| 마찰 | 캐스터 0.002, 바퀴 1.2 |
| 팔 | kp 500 N·m/rad, kv 35 N·m·s/rad, 관절 토크로 중력 보상 |
| 그리퍼 | kp 1800 N/m, kv 15 N·s/m, 최대 30 N |

- 저장소 `pinklab_robotworld2026` 의 `src/mobile_openarm_mujoco/config/mujoco.yaml` 과 `model.py` 에 있음.
- 값을 옮긴다고 끝나지 않음. 물리 엔진이 달라서 그대로는 안 되는 곳이 있고, 그것을 찾는 것이 이 강의의 대부분임.
- 팔의 중력 보상은 이 강의에서 옮기지 않음. 팔의 동작만 보려고 팔 링크의 중력을 꺼 두고, 다음 강의에서 다룸.

## 2. 바퀴

### 터미널

```bash
cd ~/nvidia-dli-lecture
git pull
~/isaacsim/python.sh 39_pl2m_drive/pl2m_drive.py --test wheels
```

- 설정을 하나씩 더해 가며 직진(0.3 m/s)과 제자리 회전(0.7 rad/s)을 3초씩 잼.

### 실행 결과

```text
설정                           직진 3초[m]    1초 뒤 속도   회전 3초[°]    1초 뒤 속도
(기대)                             0.90       0.30      120.3       40.1
가져온 그대로                          0.00       0.00        0.0        0.0
1. 캐스터 마찰 0.002                  0.61       0.25       65.3       26.2
2. + 바퀴 관절 관성 0.05               0.89       0.30      106.5       35.6
3. + 바퀴 충돌 폭 1 cm                0.89       0.30      118.3       39.6
4. + 모터 값 · 바퀴 마찰 1.2            0.89       0.30      119.4       40.0
(4 에서 속도 게인만 원본 10)              0.89       0.30      112.6       38.0
```

- 네 가지를 모두 넣어야 기대한 대로 움직임. 아래에서 하나씩 봄.
- 마지막 줄은 4번에서 모터의 속도 게인만 원본 값 10 으로 되돌린 것임. 3절에서 봄.

### 0단계: 가져온 그대로는 가지 않음

- PL-2M 의 캐스터 4개는 구르는 바퀴가 아니라 **고정된 공** 모양임. 바닥에 끌리면서 가야 함.
- 기본 마찰로는 바퀴를 돌려도 로봇이 제자리였음 (3초 동안 0.00 m).

### 1단계: 캐스터 마찰 0.002

```python
material = UsdShade.Material.Define(stage, f"/World/PhysicsMaterials/friction_{str(value).replace('.', '_')}")
api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
api.CreateStaticFrictionAttr(value)
api.CreateDynamicFrictionAttr(value)
PhysxSchema.PhysxMaterialAPI.Apply(material.GetPrim()).CreateFrictionCombineModeAttr("min" if value < 0.1 else "max")
for link in links:
    collisions = stage.GetPrimAtPath(f"{ROBOT}/{link}/collisions")
    UsdShade.MaterialBindingAPI.Apply(collisions).Bind(material, UsdShade.Tokens.strongerThanDescendants, "physics")
```

- 마찰 재질(Physics Material)을 만들어 캐스터의 충돌 모양에 묶음. 원본도 캐스터를 거의 마찰 없는 공으로 둠.
- Combine Mode 를 `min` 으로 함. 닿은 두 물체의 마찰 중 작은 쪽을 쓰게 하는 설정임. 기본(평균)으로는 재 보지 않았음.

### 2단계: 바퀴 관절의 관성

```python
for name in WHEEL_JOINTS:
    PhysxSchema.PhysxJointAPI.Apply(joint(stage, name)).CreateArmatureAttr(WHEEL_ARMATURE)
```

- 1단계만으로는 속도가 천천히 올라가 3초에 0.61 m 밖에 못 감.
- URDF 의 바퀴 질량은 0.05 kg 임. 90 kg 몸체에 비해 너무 가벼워 계산이 느리게 따라옴.
- Armature = 관절에 더하는 회전 관성(모터 회전자의 관성). 0.05 kg·m² 를 주면 0.5초 안에 0.30 m/s 에 도달함.

### 3단계: 바퀴 충돌 모양을 얇게

```python
collisions = stage.GetPrimAtPath(f"{ROBOT}/{link}/collisions")
collisions.SetInstanceable(False)                       # 함께 쓰는 모양(인스턴스)을 풀어야 고칠 수 있다
cylinder = [prim for prim in Usd.PrimRange(collisions) if prim.GetTypeName() == "Cylinder"][0]
UsdGeom.Cylinder(cylinder).GetHeightAttr().Set(WHEEL_WIDTH)
```

- 2단계까지는 제자리 회전이 기대의 89 % 였음 (35.6 / 40.1 °/s).
- 바퀴의 충돌 모양은 폭 5 cm 의 원기둥임. 제자리에서 돌면 안쪽 모서리와 바깥쪽 모서리가 서로 다른 속도로 굴러야 해서 바닥을 긁음.
- 폭을 1 cm 로 줄이면 39.6 °/s 가 됨. 보이는 모양(visuals)은 그대로임.

### 4단계: 모터 값

```python
set_friction(stage, WHEEL_LINKS, WHEEL_FRICTION)
for name in WHEEL_JOINTS:
    drive(stage, name).GetDampingAttr().Set(MOTOR_GAIN * PER_DEGREE)
    drive(stage, name).GetMaxForceAttr().Set(MOTOR_TORQUE)
```

- 속도로 구동하는 관절에서는 Damping 이 "속도 오차 1 에 내는 토크" 임. 모터의 속도 게인에 해당함.
- USD 의 회전 관절 값은 도(°) 기준임. 라디안 기준 값에 π/180 을 곱해 넣음 (`PER_DEGREE`).
- 최대 토크 18 N·m 는 원본 그대로, 속도 게인은 원본의 10 이 아니라 100 을 썼음. 이유는 다음 절에서 봄.

## 3. 모터 게인을 10 에서 100 으로 올린 이유

### 제자리 회전이 5 % 느림

```text
설정                           직진 3초[m]    1초 뒤 속도   회전 3초[°]    1초 뒤 속도
4. + 모터 값 · 바퀴 마찰 1.2            0.89       0.30      119.4       40.0
(4 에서 속도 게인만 원본 10)              0.89       0.30      112.6       38.0
```

- 속도 게인 10 에서는 제자리 회전 속도가 38.0 °/s 에서 더 오르지 않음. 기대 40.1 °/s 의 95 % 임.
- 100 에서는 40.0 °/s 가 나옴. 직진은 두 값이 같음.

### 터미널: 팔을 움직일 때 제자리를 지키는지

```bash
~/isaacsim/python.sh 39_pl2m_drive/pl2m_drive.py --test hold
```

- 빈 바닥에서 1 m 직진 → 90° 회전 → 멈춘 뒤 팔을 주행 자세에서 운반 자세로 바꿈.
- 팔의 목표를 **한 번에** 바꿀 때와 **1초에 걸쳐** 바꿀 때를, 게인 10 과 100 으로 각각 잼.

### 실행 결과

```text
모터 게인    팔 움직임      직진[m]   회전[°]    어깨 최고 속도[°/s]   0~1.5초에 밀림[mm]   그 뒤 4초[mm]
10         한 번에         0.989    82.6              945              6.4          0.0
100        한 번에         0.994    87.1              945              9.6          0.0
10         1초에 걸쳐       0.989    82.6              200              0.1          0.0
100        1초에 걸쳐       0.994    87.1              200              0.0          0.0
```

- 밀림을 가장 크게 바꾸는 것은 게인이 아니라 **팔을 움직이는 방법**임. 한 번에 바꾸면 6.4 ~ 9.6 mm, 1초에 걸치면 0.1 mm 아래임.
- 한 번에 바꿀 때의 밀림은 게인 100 쪽이 오히려 3 mm 컸음. 회전은 게인 100 일 때 목표 90° 에 더 가까움 (87.1°).

### 읽기

- 목표를 한 번에 바꾸면 어깨 관절이 최고 945 °/s 로 돎. URDF 에 적힌 속도 한계 960 °/s 에 가까운 값임.
- 그 반동으로 베이스가 뒤로 차임. 게인 100 에서도 9.6 mm 밀렸으니, 게인을 올리는 것으로 없어지는 일이 아님.
- 팔을 1초에 걸쳐 움직이면 어깨의 최고 속도가 200 °/s 로 줄고, 밀림은 0.1 mm 아래가 됨.

## 4. 팔

### 터미널

```bash
~/isaacsim/python.sh 39_pl2m_drive/pl2m_drive.py --test arms --out work/pl2m/arms_step.json
```

- 팔을 내린 자세에서 운반 자세(carry)로 **한 번에** 목표를 바꾸고 2초 동안 봄.
- 운반 자세는 원본의 `config/drive_poses.yaml` 에 있는 값임. 어깨와 팔꿈치를 136° 돌리는 큰 동작임.

### 실행 결과

```text
설정                       0.25초    0.5초      1초      2초   (팔 관절 14개 중 가장 큰 오차 [°])  베이스가 밀린 거리
Importer 기본값             115.0    76.1     0.3     0.3                           6.7 mm
원본 값 (kp 500 · kv 35)    110.2    32.7     0.8     0.8                           0.2 mm
원본 값 + 팔 중력 끔            110.6    34.8     0.3     0.3                           3.9 mm
```

### 그림으로

- 세 설정 모두 0.8초쯤이면 목표 근처에 옴.
- 원본 값만 쓰면 0.8° 가 남음. 팔의 무게 때문에 처진 것임.
- 끝에 남는 0.3° 는 6번 관절의 한계 때문임. 목표 −45.3° 가 관절 한계 −45° 를 넘음.

### 원본 값으로 바꾸기

```python
for name in ARM_JOINTS:
    drive(stage, name).GetStiffnessAttr().Set(ARM_KP * PER_DEGREE)
    drive(stage, name).GetDampingAttr().Set(ARM_KV * PER_DEGREE)
```

- Importer 의 기본값은 관절마다 달랐음 (어깨 432, 손목 5.56). 원본은 14개 모두 같은 값임.
- kp 500 N·m/rad 는 USD 값으로 8.73, kv 35 는 0.61 임.

### 팔 링크의 중력 끄기

```python
for link in stage.GetPrimAtPath(ROBOT).GetChildren():
    if link.GetName().startswith(("openarm_left", "openarm_right")) and link.HasAPI(UsdPhysics.RigidBodyAPI):
        PhysxSchema.PhysxRigidBodyAPI.Apply(link).CreateDisableGravityAttr(True)
```

- 팔의 동작만 확인하려고 **팔 링크에만** 중력을 껐음. Property 의 Rigid Body > Disable Gravity 와 같음.
- 베이스와 바퀴, 손에 든 물체의 중력은 그대로임.

### 중력을 끈 것은 중력 보상이 아님

| | 중력 보상 (원본) | 중력 끄기 (이 강의) |
|---|---|---|
| 하는 일 | 관절이 팔 무게만큼의 토크를 더 냄 | 팔 링크에 중력이 작용하지 않음 |
| 팔의 처짐 | 없음 | 없음 |
| 팔의 무게 | 베이스에 실림 | 베이스에 실리지 않음 |

- 원본 MuJoCo 설정은 관절 토크로 보상함 (`bias_compensation: true`). 중력은 그대로 있음.
- 팔 링크의 질량은 URDF 기준 12.05 kg 으로, 로봇 90.15 kg 의 13 % 임. 이 무게가 바퀴와 캐스터에 실리지 않는 점이 원본과 다름.
- 중력의 영향과 중력을 보상하는 방법은 다음 강의에서 다룸.

### 계단 입력은 거칢

```python
def move_arms(stage, sim, name, seconds):
    begin = [drive(stage, joint_name).GetTargetPositionAttr().Get() or 0.0 for joint_name in ARM_JOINTS]
    end, start = pose_degrees(name), sim.time
    while sim.time - start < seconds:
        sim.app.update()
        set_arm_targets(stage, blend(begin, end, (sim.time - start) / seconds))
```

- 목표를 한 번에 바꾸면 팔이 관절의 속도 한계로 움직이고, 그 반동으로 베이스가 밀림 (3절).
- `move_arms` 는 목표를 정한 시간에 걸쳐 조금씩 옮김. 이 강의의 시연과 키보드 운전은 이것을 씀.
- 실제로는 궤적을 만들어 보냄. 뒤 강의에서 MoveIt 이 그 궤적을 만듦.

## 5. 그리퍼

### 터미널

```bash
~/isaacsim/python.sh 39_pl2m_drive/pl2m_drive.py --test gripper
```

- 왼손 그리퍼의 목표를 44 mm(다 열림) → 0(닫힘) → 20 mm 로 바꾸며 두 손가락의 위치를 0.25초마다 봄.

### 실행 결과

```text
Importer 기본값
    목표[mm]   0.25초마다 손가락 1 / 손가락 2 의 위치 [mm]
        44    14.8 /  14.7    34.4 /  34.5    43.8 /  46.0    42.6 /  43.5
         0    31.7 /  33.1     7.6 /   8.8     0.1 /  -0.9     1.4 /   1.7
        20     7.1 /   7.3    18.1 /  18.5    29.9 /  30.6    37.5 /  38.4
원본 값 (kp 1800 · kv 15 · 30 N)
    목표[mm]   0.25초마다 손가락 1 / 손가락 2 의 위치 [mm]
        44    44.0 /  42.6    44.0 /  44.0    44.0 /  44.0    44.0 /  44.0
         0     0.0 /   1.7     0.0 /   0.2     0.0 /   0.2     0.0 /   0.2
        20    20.0 /  19.4    20.0 /  20.1    20.0 /  20.1    20.0 /  20.1
```

### 읽기

- 기본값(Stiffness 0.51)은 1초가 지나도 목표에 서지 못하고 출렁임. 목표 20 mm 에 37.5 mm 까지 지나침.
- 원본 값은 0.25초 안에 목표에 도달함.
- 둘째 손가락은 구동이 없는데도 첫째를 따라 움직임. 강의 37 에서 본 `PhysxMimicJointAPI` 가 일하는 것임. 0.1 ~ 0.2 mm 차이로 따라옴.

### 코드

```python
for name in FINGER_JOINTS:
    drive(stage, name).GetStiffnessAttr().Set(GRIP_KP)          # 직선 관절은 미터 기준이라 그대로 넣는다
    drive(stage, name).GetDampingAttr().Set(GRIP_KV)
    drive(stage, name).GetMaxForceAttr().Set(GRIP_FORCE)
```

- 손가락은 직선으로 움직이는 관절이라 π/180 을 곱하지 않음.
- 최대 힘 30 N 은 물체를 쥐는 힘의 한계가 됨. 컵을 쥐는 것은 뒤 강의에서 확인함.

## 6. 모두 합치기

### 한 번에 부르는 함수

```python
def tune_pl2m(stage):
    """이 강의의 결론: 뒤 강의에서는 가져온 뒤 이 함수 하나를 부른다."""
    tune_wheels(stage)
    tune_arms(stage)
    tune_gripper(stage)
```

- 뒤 강의의 스크립트는 `import_pl2m` 으로 가져온 뒤 `tune_pl2m` 을 부름.

### 터미널: 부스 안에서

```bash
~/isaacsim/python.sh 39_pl2m_drive/pl2m_drive.py --booth --gui
```

- 강의 38 의 부스를 만들고, 상담 테이블 사이에서 시연을 함.
- `--booth` 를 빼면 빈 바닥에서 같은 시연을 함.

### 실행 결과

```text
동작                        x[m]    y[m]   방향[°]    왼팔 팔꿈치[°]   왼손 손가락[mm]
팔: 주행 자세                -0.000   0.000     0.0         68.7          0.0
직진 0.25 m/s              0.994  -0.001    -0.1         68.7          0.0
왼쪽으로 회전 0.5 rad/s        1.001   0.000    87.2         68.7          0.0
멈추고 팔: 운반 자세             1.001   0.000    87.4        135.9          0.0
그리퍼 열기                   1.001   0.000    87.4        135.9         44.0
그리퍼 닫기                   1.001   0.000    87.4        135.9          0.0
```

- 4초 직진의 기대는 1.00 m, 회전의 기대는 90° 임. 0.994 m 와 87.2° 가 나왔음.
- 회전이 2.8° 모자란 것은 속도가 올라가고 내려가는 동안의 차이로 보임.
- 팔은 `move_arms` 로 1.5 ~ 2초에 걸쳐 움직임. 운반 자세로 바꾸는 동안 베이스의 위치는 1 mm 도 움직이지 않았음.

### 운반 자세

- 두 손이 몸 앞에서 같은 높이로 모임. 컵을 세운 채로 나르기 위한 자세임.

### 키보드로 운전

```bash
~/isaacsim/python.sh 39_pl2m_drive/pl2m_drive.py --booth --gui --keyboard
```

- 뷰포트를 한 번 클릭한 뒤 W(앞) S(뒤) A(왼쪽) D(오른쪽). 강의 11 과 같은 Differential Controller 그래프를 씀.
- 속도 제한은 원본 설정의 0.35 m/s, 0.7 rad/s 임.

## 7. 참고: 바닥을 얇은 상자로 만들면

### 터미널

```bash
~/isaacsim/python.sh 39_pl2m_drive/pl2m_drive.py --test slab
```

- 바닥이 평면일 때와 상자일 때, 0.1 m/s 로 6초 직진한 거리를 잼 (기대 0.6 m).
- 상자는 9 × 6 m 이고 윗면의 높이는 2 cm 임. 두께만 바꿈. 서는 자리는 상자의 가운데 쪽과 가장자리 쪽 둘임.

### 실행 결과

```text
바닥             선 자리         간 거리[m]
평면             가운데 쪽          0.599
평면             가장자리 쪽         0.599
상자 2 cm        가운데 쪽          0.598
상자 2 cm        가장자리 쪽         0.336
상자 5 cm        가운데 쪽          0.000
상자 5 cm        가장자리 쪽         0.000
상자 10 cm       가운데 쪽          0.597
상자 10 cm       가장자리 쪽         0.599
상자 100 cm      가운데 쪽          0.598
상자 100 cm      가장자리 쪽         0.595
```

### 읽기

- 두께 2 cm 의 상자에서는 자리에 따라, 5 cm 에서는 두 자리 모두에서 로봇이 가지 못했음. 10 cm 부터는 평면과 같았음.
- 원본의 부스에는 두께 2 cm 의 바닥판이 있음. 그것을 상자 그대로 옮겼을 때 Nav2 주행 중에 바퀴가 헛도는 자리가 있어서 찾은 일임.
- 이 강의의 부스에는 그 바닥판이 없고(강의 38), 로봇은 평면 바닥 위를 다님. 바닥을 상자로 만들 때는 두껍게 만드는 편이 안전함.

## 8. 해 볼 것

### 직접 바꿔 보기

- `test_hold` 에서 팔을 움직이는 시간 1.0 을 0.5 와 3.0 으로 바꿔, 어깨 최고 속도와 밀린 거리가 어떻게 달라지는지 보기.
- `WHEEL_WIDTH` 를 0.05(원래 폭)로 되돌리고 `--test wheels` 의 4번 줄이 어떻게 바뀌는지 보기.
- `tune_arms` 에서 `gravity_off=False` 로 두고 시연을 돌려, 운반 자세의 팔꿈치 각도가 얼마나 달라지는지 보기.
- `--keyboard` 로 부스 안을 돌아다니며 음료대 앞 (7.65, −2.82) 까지 가 보기.

## 9. 정리

### 기억할 것

- 가져온 로봇이 움직이려면 구동 값과 접촉 설정을 정해야 함. URDF 에는 없는 값들임.
- 바퀴: 캐스터 마찰 0.002, 바퀴 관절의 관성 0.05, 바퀴 충돌 폭 1 cm, 모터 게인 100 · 최대 18 N·m.
- 팔: kp 500 · kv 35 + 팔 링크의 중력 끄기(중력 보상이 아님). 그리퍼: kp 1800 · kv 15 · 최대 30 N.
- USD 의 회전 관절 값은 도 기준임. 라디안 기준 값에 π/180 을 곱함.
- 팔의 목표는 한 번에 바꾸지 않고 서서히 옮김. 한 번에 바꾸면 반동으로 베이스가 6 ~ 10 mm 밀렸음.
- 다른 시뮬레이터의 값을 옮길 때는 하나씩 재 봐야 함. 그대로 옮긴 모터 게인 10 은 제자리 회전이 5 % 느렸음.

### 한계

- 평평한 바닥에서, 직진 · 제자리 회전 · 자세 하나 · 여닫기만 잰 결과임. 곡선 주행이나 경사는 재지 않았음.
- 바퀴 관절의 관성 0.05 와 바퀴 폭 1 cm 는 원본에 없는 값이고, 재 보면서 정했음. 실물 값과 비교하지 않았음.
- 팔 링크의 중력을 껐기 때문에 팔 무게 12 kg 이 베이스에 실리지 않음. 원본과 다른 점이고, 다음 강의에서 중력의 영향과 보상 방법을 다룸.
- 얇은 상자 바닥에서 바퀴가 헛도는 까닭은 PhysX 안까지 보지 않았음. 두께를 바꿔 잰 결과만 있음 (7절).
- 그리퍼로 물체를 쥐는 것, 팔을 궤적으로 움직이는 것은 아직 하지 않았음.
- 물리 솔버는 기본값(TGS), 물리 주기는 기본값(60 Hz) 그대로임. 바꿨을 때는 재지 않았음.
- 강의 PC 한 대(Isaac Sim 5.1)에서 확인한 결과임.
