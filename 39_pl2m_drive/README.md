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
| 팔 | 세기가 관성에 따라 제각각 | 구동 값(세기 · 감쇠), 중력 보상 |
| 그리퍼 | 너무 약해 출렁임 | 구동 값, 최대 힘 |

- URDF 에는 로봇의 모양과 질량은 있지만 모터의 세기는 없음. 강의 37 의 값은 Importer 가 임시로 정한 것임.

### 값의 출처

| 부분 | 원본(MuJoCo 설정)의 값 |
|---|---|
| 바퀴 모터 | 속도 게인 10 N·m·s/rad, 최대 토크 18 N·m |
| 마찰 | 캐스터 0.002, 바퀴 1.2 |
| 팔 | kp 500 N·m/rad, kv 35 N·m·s/rad, 중력 보상 |
| 그리퍼 | kp 1800 N/m, kv 15 N·s/m, 최대 30 N |

- 저장소 `pinklab_robotworld2026` 의 `src/mobile_openarm_mujoco/config/mujoco.yaml` 과 `model.py` 에 있음.
- 값을 옮긴다고 끝나지 않음. 물리 엔진이 달라서 그대로는 안 되는 곳이 있고, 그것을 찾는 것이 이 강의의 대부분임.

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
```

- 네 가지를 모두 넣어야 기대한 대로 움직임. 아래에서 하나씩 봄.

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

### 터미널

```bash
~/isaacsim/python.sh 39_pl2m_drive/pl2m_drive.py --test hold
```

- 부스 바닥판과 같은 판 위에서 1 m 직진 → 90° 회전 → 팔을 운반 자세로 바꿈. 그 뒤 로봇이 제자리에 있는지 봄.

### 실행 결과

```text
모터 속도 게인          직진[m]    회전[°]   팔을 움직이고 2.5초 뒤부터 3초 동안 밀린 거리
10                0.986     82.8   26.2 mm
100               0.993     87.3   0.0 mm
```

- 원본 값 10 에서는 팔을 움직인 반동으로 로봇이 26.2 mm 밀렸음. 100 에서는 밀리지 않았음.
- 회전도 100 일 때 목표 90° 에 더 가까움 (87.3°).

### 읽기

- 속도 게인이 작으면 모터가 "멈춰 있으라" 는 명령을 약하게 지킴. 캐스터가 미끄러운 로봇이라 작은 힘에도 굴러감.
- 같은 시험을 다른 자리에서 하면 게인 10 에서도 밀리지 않을 때가 있었음. 항상 일어나는 일은 아님.
- 원본은 물리 계산 주기가 2 ms(500 Hz)이고 Isaac Sim 의 기본은 60 Hz 임. 이 차이가 원인인지는 확인하지 않았음.

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

### 중력 보상: 팔 링크의 중력 끄기

```python
for link in stage.GetPrimAtPath(ROBOT).GetChildren():
    if link.GetName().startswith(("openarm_left", "openarm_right")) and link.HasAPI(UsdPhysics.RigidBodyAPI):
        PhysxSchema.PhysxRigidBodyAPI.Apply(link).CreateDisableGravityAttr(True)
```

- 실물 팔의 제어기는 팔 무게를 버티는 힘을 미리 계산해 더함(중력 보상). 원본 MuJoCo 설정도 그렇게 함.
- Isaac Sim 에서는 팔 링크의 중력을 꺼서 같은 효과를 냄. Property 의 Rigid Body > Disable Gravity 와 같음.
- 손에 든 물체의 중력은 그대로임. 물체 무게만큼의 처짐은 남음.

### 계단 입력은 거칢

- 목표를 한 번에 바꾸면 팔이 급하게 움직이고, 그 반동으로 베이스가 0.2 ~ 6.7 mm 밀렸음.
- 실제로는 목표를 조금씩 바꿔 가며 보냄(궤적). 다음 강의에서 MoveIt 이 그 궤적을 만듦.

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
팔: 주행 자세                 0.000  -0.000     0.0         68.7          0.0
직진 0.25 m/s              0.993  -0.001    -0.1         68.7          0.0
왼쪽으로 회전 0.5 rad/s        1.000  -0.000    87.3         68.6          0.0
멈추고 팔: 운반 자세             1.000  -0.009    88.1        135.9          0.0
그리퍼 열기                   1.000  -0.009    88.1        135.9         44.0
그리퍼 닫기                   1.000  -0.010    88.1        135.9          0.0
```

- 4초 직진의 기대는 1.00 m, 회전의 기대는 90° 임. 0.993 m 와 87.3° 가 나왔음.
- 회전이 2.7° 모자란 것은 속도가 올라가고 내려가는 동안의 차이로 보임.

### 운반 자세

- 두 손이 몸 앞에서 같은 높이로 모임. 컵을 세운 채로 나르기 위한 자세임.

### 키보드로 운전

```bash
~/isaacsim/python.sh 39_pl2m_drive/pl2m_drive.py --booth --gui --keyboard
```

- 뷰포트를 한 번 클릭한 뒤 W(앞) S(뒤) A(왼쪽) D(오른쪽). 강의 11 과 같은 Differential Controller 그래프를 씀.
- 속도 제한은 원본 설정의 0.35 m/s, 0.7 rad/s 임.

## 7. 부스 바닥판의 턱

### 터미널

```bash
~/isaacsim/python.sh 39_pl2m_drive/pl2m_drive.py --test step
```

- 부스 바닥판과 같은 두께 2 cm 의 판 위에서 출발해, 앞으로 가서 턱을 내려갔다가 뒤로 돌아옴.

### 실행 결과

```text
동작             시간[s]    x[m]    z[m]
전진 0.3           1.5   0.442   0.107
전진 0.3           2.0   0.603   0.097
전진 0.3           2.5   0.784   0.088
...
후진 0.3           1.5   0.823   0.087
후진 0.3           2.0   0.780   0.087
...
후진 0.3           4.0   0.780   0.087
결과: 끝난 자리 x = 0.780 m, 높이 z = 0.087 m (판 위에 있으면 0.107, 바닥이면 0.087)
```

### 읽기

- 내려가는 것은 됨 (높이 0.107 → 0.087 m).
- 다시 올라오지는 못함. x = 0.780 m 에서 멈춘 채 2초 동안 그대로였음. 뒤쪽 캐스터가 턱에 걸린 것으로 보임.
- 부스에서는 로봇이 바닥판(9 × 6 m) 밖으로 나가지 않게 해야 함. 뒤 강의에서 Nav2 지도를 만들 때 반영함.

## 8. 해 볼 것

### 직접 바꿔 보기

- `MOTOR_GAIN` 을 10, 30, 100 으로 바꿔 `--test hold` 를 돌려 보기.
- `WHEEL_WIDTH` 를 0.05(원래 폭)로 되돌리고 `--test wheels` 의 4번 줄이 어떻게 바뀌는지 보기.
- `tune_arms` 에서 `gravity_off=False` 로 두고 시연을 돌려, 운반 자세의 팔꿈치 각도가 얼마나 달라지는지 보기.
- `--keyboard` 로 부스 안을 돌아다니며 음료대 앞 (7.65, −2.82) 까지 가 보기.

## 9. 정리

### 기억할 것

- 가져온 로봇이 움직이려면 구동 값과 접촉 설정을 정해야 함. URDF 에는 없는 값들임.
- 바퀴: 캐스터 마찰 0.002, 바퀴 관절의 관성 0.05, 바퀴 충돌 폭 1 cm, 모터 게인 100 · 최대 18 N·m.
- 팔: kp 500 · kv 35 + 팔 링크의 중력 끄기. 그리퍼: kp 1800 · kv 15 · 최대 30 N.
- USD 의 회전 관절 값은 도 기준임. 라디안 기준 값에 π/180 을 곱함.
- 다른 시뮬레이터의 값을 옮길 때는 하나씩 재 봐야 함. 그대로 옮긴 값(모터 게인 10)은 여기서 통하지 않았음.

### 한계

- 평평한 바닥에서, 직진 · 제자리 회전 · 자세 하나 · 여닫기만 잰 결과임. 곡선 주행이나 경사는 재지 않았음.
- 바퀴 관절의 관성 0.05 와 바퀴 폭 1 cm 는 원본에 없는 값이고, 재 보면서 정했음. 실물 값과 비교하지 않았음.
- 모터 게인 10 에서 밀리는 일은 한 자리에서 확인했고, 다른 자리에서는 일어나지 않았음. 원인은 가려내지 못했음.
- 그리퍼로 물체를 쥐는 것, 팔을 궤적으로 움직이는 것은 아직 하지 않았음.
- 물리 솔버는 기본값(TGS), 물리 주기는 기본값(60 Hz) 그대로임. 바꿨을 때는 재지 않았음.
- 강의 PC 한 대(Isaac Sim 5.1)에서 확인한 결과임.
