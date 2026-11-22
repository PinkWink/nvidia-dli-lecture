# 52. 팔이 목표에 손을 뻗게 학습하기: OpenArm Reach 와 PL-2M 의 왼팔 — 명령 모음

- 영상에서 입력하는 터미널 명령과 기대 출력을 순서대로 모았음. [51. 익스텐션 만들기](../51_extension/README.md) 다음 내용임.

| 파일 |
|---|
| [pl2m_reach_env.py](pl2m_reach_env.py) |
| [play.py](play.py) |
| [reach_samples.py](reach_samples.py) |
| [train.py](train.py) |

## 1. 팔을 강화학습으로

### 이번 강의의 결과

- 색 화살표가 4초마다 바뀌는 목표임. 왼팔이 손을 그 자리와 방향으로 가져감.
- 팔을 움직이는 것은 MoveIt 이 아니라 6분 동안 학습한 신경망임. 목표에서 평균 5.4 mm, 방향으로 3.0° 떨어진 곳까지 감.

### 과제: Reach

| | 내용 |
|---|---|
| 목표 | 손이 가야 할 자리와 방향. 4초마다 새로 뽑음 |
| 관측 | 팔 관절의 각도 · 속도, 목표, 직전에 낸 행동 |
| 행동 | 팔 관절 일곱 개의 목표 각도 |
| 보상 | 손이 목표에 가까울수록, 방향이 맞을수록 큼 |

- 강의 30 의 Cartpole, 강의 34 의 사족 보행과 같은 틀임. 로봇팔에서 가장 기본이 되는 과제임.
- NVIDIA 과정은 UR10 으로 이 과제를 만듦. 이 강의는 PL-2M 의 팔인 OpenArm 으로 함.

### 강의 42 의 MoveIt 과 견주면

| | MoveIt (강의 42) | 학습한 정책 (이 강의) |
|---|---|---|
| 팔을 움직이는 것 | 그때그때 경로를 계획함 | 관측을 넣으면 행동이 바로 나옴 |
| 장애물 | 알려 준 것을 피함 | 모름 |
| 손끝의 오차 | 1.6 mm (보상한 팔) | 5.4 mm |
| 준비 | 로봇의 설정 파일 | 과제를 짜고 6분 학습 |

- 정책은 장애물을 모름. 이 과제에는 피할 것이 없었기 때문임.
- 부스에서 컵을 집는 일은 MoveIt 으로 하는 것이 맞음. 이 강의는 같은 팔을 다른 방법으로 움직여 보는 것임.

### 만드는 것

| 파일 | 하는 일 |
|---|---|
| `pl2m_reach_env.py` | Isaac Lab 의 OpenArm Reach 과제에서 로봇을 PL-2M 으로 바꾼 것 |
| `reach_samples.py` | 팔이 닿는 범위를 잼 |
| `train.py` · `play.py` | 학습 · 재생과 평가 |

- 앞의 2 ~ 3절은 파일 없이 Isaac Lab 에 든 과제를 그대로 돌림.

## 2. Isaac Lab 에 든 OpenArm Reach

### 터미널: 학습

```bash
cd ~/IsaacLab
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py --task Isaac-Reach-OpenArm-v0 --headless
```

```text
                      Learning iteration 999/1000
                       Computation: 388431 steps/s (collection: 0.204s, learning 0.049s)
                       Mean reward: 0.58
    Metrics/ee_pose/position_error: 0.0107
 Metrics/ee_pose/orientation_error: 0.2536
Training time: 264.42 seconds
```

- Isaac Lab 2.3.2 에는 OpenArm 한 팔짜리 Reach 과제가 들어 있음. 로봇 4,096대로 1,000번 고치는 데 4분 24초 걸렸음.
- 끝에서 자리의 오차가 1.07 cm, 방향의 오차가 0.25 rad (14.5°) 임. 학습하는 동안 모든 스텝의 평균이라 목표가 막 바뀐 순간도 들어 있음.

### 과제의 설정

```python
ranges=mdp.UniformPoseCommandCfg.Ranges(
    pos_x=(0.25, 0.35), pos_y=(-0.2, 0.2), pos_z=(0.3, 0.4),
    roll=(-math.pi / 6, math.pi / 6), pitch=(math.pi / 2, math.pi / 2), yaw=(-math.pi / 9, math.pi / 9),
)
```

- 목표는 로봇 기준의 상자 안에서 뽑음. 가로 10 cm · 세로 40 cm · 높이 10 cm 임.
- 방향은 손이 앞을 보는 자세(pitch 90°)에서 roll ±30° · yaw ±20° 로 흔듦.

### 보상

| 항 | 가중치 | 뜻 |
|---|---|---|
| `end_effector_position_tracking` | -0.2 | 손과 목표 사이의 거리 |
| `end_effector_position_tracking_fine_grained` | +0.1 | 가까이 왔을 때 더 주는 점수 (10 cm 안에서 커짐) |
| `end_effector_orientation_tracking` | -0.1 | 방향이 틀어진 각 |
| `action_rate` · `joint_vel` | -0.0001 → 더 크게 | 행동이 급하게 바뀌는 것 · 관절이 빠른 것 |

- 벌점 둘은 학습이 진행되면서 커짐 (Curriculum). 처음에는 가는 법을 배우고, 나중에 부드럽게 가는 법을 배움.

## 3. 방향 보상을 빼면

### 터미널

```bash
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py --task Isaac-Reach-OpenArm-v0 --headless env.rewards.end_effector_orientation_tracking.weight=0.0
```

- 명령줄 끝에 `env.` 로 시작하는 값을 적으면 과제의 설정을 그 값으로 바꿔서 돌림. 파일을 고치지 않아도 됨.

### 결과

| | 자리의 오차 | 방향의 오차 |
|---|---|---|
| 기본 | 1.1 cm | 14.7° |
| 방향 보상을 뺌 | 0.3 cm | 63.6° |

- 방향 보상을 빼면 자리는 더 잘 맞고 방향은 아무렇게나 됨. 방향의 오차 64° 는 맞추려 하지 않은 값임.
- 두 가지를 다 맞추려면 자리에서 조금 손해를 봄. 보상에 무엇을 넣느냐가 정책이 무엇을 포기하느냐를 정함.

## 4. PL-2M 으로 바꾸기

### 터미널: 준비

```bash
cd ~/nvidia-dli-lecture
git pull
ls work/pl2m/pl2m.urdf
```

- 강의 37 에서 만든 URDF 를 씀. 없으면 강의 37 의 `prepare_pl2m.sh` 를 먼저 실행함.

### 코드: 로봇의 설정

```python
PL2M_CFG = ArticulationCfg(
    spawn=sim_utils.UrdfFileCfg(
        asset_path=URDF,
        fix_base=True,                                               # 팔만 배우므로 베이스는 바닥에 붙여 둔다
        merge_fixed_joints=True,
        ...
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        joint_pos={f"openarm_left_joint{i + 1}": value for i, value in enumerate(START)},
    ),
    actuators={
        "arms": ImplicitActuatorCfg(joint_names_expr=["openarm_.*_joint[1-7]"], stiffness=500.0, damping=35.0),         # 강의 39 의 값
        ...
    },
)
```

- Isaac Lab 은 URDF 를 직접 받음 (`UrdfFileCfg`). 처음 실행할 때 USD 로 바꿔 두고 다음부터는 그것을 씀.
- 팔의 게인은 강의 39 에서 쓴 값(500 · 35)임. 베이스는 바닥에 고정함.

### 코드: 과제에서 바꾼 곳

```python
class Pl2mReachEnvCfg(ReachEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        self.scene.robot = PL2M_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
        self.scene.table = None
        self.actions.arm_action = mdp.JointPositionActionCfg(asset_name="robot", joint_names=[ARM], scale=0.5, use_default_offset=True)
        self.commands.ee_pose.body_name = HAND
        set_target_box(self, TARGET_BOX)
```

- Isaac Lab 의 `ReachEnvCfg` 를 물려받아 로봇, 움직일 관절, 손, 목표의 범위만 바꿈. 보상과 PPO 의 값은 그대로임.
- 움직이는 것은 왼팔 일곱 관절(`openarm_left_joint.*`)뿐임. 오른팔은 내린 채로 둠.

### 손의 이름이 다름

| | 강의 37 의 가져오기 | Isaac Lab 의 `UrdfFileCfg` |
|---|---|---|
| `openarm_left_hand` | 따로 남음 | `openarm_left_link7` 에 합쳐짐 |
| 강체의 수 | 33개 | 21개 |

- 같은 URDF 인데 바꾸는 도구에 따라 링크가 다르게 합쳐졌음. 손을 `openarm_left_hand` 로 적었더니 그런 이름이 없다는 오류가 났음.
- 그래서 손이 붙은 마지막 링크(`openarm_left_link7`)를 끝점으로 씀. 손끝은 거기서 19 cm 더 앞임.

## 5. 목표를 어디에 낼 것인가

### 터미널: 팔이 닿는 범위

```bash
~/IsaacLab/isaaclab.sh -p 52_openarm_reach/reach_samples.py --headless
```

```text
자세 24576가지.  끝 링크가 간 범위: 앞 -0.44 ~ 0.44 m,  왼쪽 -0.10 ~ 0.59 m,  높이 0.48 ~ 1.35 m
그 가운데 쥐는 방향에서 35° 안인 것 283가지: 앞 -0.12 ~ 0.37 m,  왼쪽 0.07 ~ 0.50 m,  높이 0.73 ~ 1.26 m  (5 ~ 95 %)
TARGET_BOX: 상자 안에 든 자세 551가지 (쥐는 방향 35° 안 12가지),  가장 먼 모서리에서 가장 가까운 표본까지 2.2 cm
WIDE_BOX: 상자 안에 든 자세 817가지 (쥐는 방향 35° 안 16가지),  가장 먼 모서리에서 가장 가까운 표본까지 10.9 cm
```

- 왼팔의 관절을 한계 안에서 무작위로 놓고 끝 링크가 어디에 오는지를 모은 것임. 로봇 4,096대를 여섯 번 다시 놓아 2만 4천 가지를 봄.
- 끝 링크는 어떤 자세로도 앞으로 0.44 m 까지만 감.

### 상자 둘

- 주황 상자는 처음에 어림으로 잡은 것임. 오른쪽 아래가 팔이 닿는 곳 밖으로 나가 있음. 가장 먼 모서리는 가장 가까운 표본에서 10.9 cm 떨어져 있음.
- 파란 상자가 고친 것임. 닿는 곳 안에 들어 있음.

### 시작 자세

| | 운반 자세 (강의 39) | 고른 시작 자세 |
|---|---|---|
| 6번 관절 (한계 ±0.785) | -0.78 | -0.48 |
| 4번 관절 (한계 0 ~ 2.44) | 2.37 | 2.02 |
| 손의 방향 | 쥐는 방향 | 쥐는 방향과 16° |

- 행동은 시작 자세에서 얼마나 움직일지로 나감. 운반 자세는 6번 관절이 이미 한계에 닿아 있어 한쪽으로는 움직일 수 없음.
- 위의 표본 가운데, 쥐는 방향에 가까우면서 관절이 한계에서 가장 먼 자세를 골라 시작 자세로 삼았음.

## 6. 학습과 평가

### 터미널: 학습

```bash
~/IsaacLab/isaaclab.sh -p 52_openarm_reach/train.py --headless
```

```text
                       Computation: 247442 steps/s (collection: 0.338s, learning 0.059s)
    Metrics/ee_pose/position_error: 0.0059
 Metrics/ee_pose/orientation_error: 0.0519
학습 시간: 375초
저장: /home/내이름/nvidia-dli-lecture/work/reach_logs/2026-11-22_17-58-51
```

- 6분 15초 걸렸음. 팔 하나짜리 과제(4분 24초)보다 느림. PL-2M 은 관절이 20개라 계산할 것이 많음.

### 터미널: 평가

```bash
~/IsaacLab/isaaclab.sh -p 52_openarm_reach/play.py --headless --num_envs 256 --seconds 12
```

```text
목표 768개.  자리의 오차: 평균 5.4 mm · 가장 큰 것 20.8 mm · 2 cm 안에 든 비율 100 %
            방향의 오차: 평균 3.0° · 가장 큰 것 11.8°
```

- 목표가 바뀌기 직전에 손이 목표에서 얼마나 떨어져 있는지를 모은 것임. 로봇 256대가 목표 셋씩 768개임.
- 창으로 보려면 `--headless` 와 뒤의 옵션을 뺌. 1절의 그림이 그 화면임.

### 하나씩만 바꿔 보면

| 바꾼 것 | 옵션 | 자리의 오차 | 2 cm 안 | 방향의 오차 |
|---|---|---|---|---|
| (기본) | | 5.4 mm | 100 % | 3.0° |
| 목표의 상자를 넓은 것으로 | `--wide` | 96.1 mm | 2 % | 4.2° |
| 시작 자세를 운반 자세로 | `--carry_start` | 42.7 mm | 12 % | 11.5° |
| 방향을 넓게 흔듦 (roll ±30° · yaw ±20°) | `--full_twist` | 5.8 mm | 99 % | 6.8° |
| 방향 보상을 뺌 | `--no_orientation` | 7.5 mm | 99 % | 50.1° |

- 닿지 않는 곳에 목표를 내면 9.6 cm 가 남음. 아무리 학습해도 줄지 않는 오차임.
- 시작 자세가 관절의 한계에 닿아 있으면 4.3 cm 가 남음. 정책이 낸 목표 각도를 관절이 따라가지 못함.
- 방향의 범위는 넓혀도 자리가 거의 그대로였음. 처음에는 이것이 원인이라 보고 좁혔는데, 재 보니 아니었음.
- 다섯 가지 모두 한 번씩만 학습한 값임.

### 고치는 동안 본 것

| 순서 | 한 일 | 자리의 오차 |
|---|---|---|
| 1 | 어림으로 잡은 상자 + 운반 자세 | 62 mm |
| 2 | 상자만 닿는 곳으로 | 92 mm |
| 3 | 정책이 어떻게 움직이는지 찍어 봄 | 6번 관절이 83 % 의 목표에서 한계에 있음 |
| 4 | 시작 자세를 바꿈 | 5.4 mm |

- 2번에서 오차가 오히려 커져 원인을 다시 찾았음. 손의 자리만 봐서는 알 수 없었고, 관절의 각도를 찍어 보고서야 한계에 걸린 것이 보였음.
- 1 · 2번의 상자는 지금의 것과 조금 달랐음. 위의 "하나씩만 바꿔 보면" 은 지금의 설정에서 다시 잰 것임.

## 7. 해 볼 것

### 직접 바꿔 보기

- `train.py --max_iterations 200` 으로 짧게 학습하고 `play.py` 로 오차 보기.
- `pl2m_reach_env.py` 의 `TARGET_BOX` 를 로봇의 정면(`(0.15, 0.30), (-0.05, 0.15), (0.80, 0.96)`)으로 옮기고 `reach_samples.py` 로 닿는지부터 보기.
- `ARM` 과 `HAND` 를 오른팔(`openarm_right_...`)로 바꾸고, 시작 자세와 상자의 좌우를 뒤집어 학습하기.
- TensorBoard 로 곡선 보기: `~/IsaacLab/isaaclab.sh -p -m tensorboard.main --logdir work/reach_logs`.

## 8. 정리

### 기억할 것

- Reach 는 손을 목표 자세로 보내는 과제임. 목표 · 관측 · 행동 · 보상 넷으로 짜임.
- 이미 있는 과제를 물려받아 로봇과 범위만 바꾸면 내 로봇의 과제가 됨.
- 목표는 팔이 닿는 곳에 내야 함. 닿는 범위는 재서 정함.
- 시작 자세는 관절이 한계에서 먼 곳으로 잡음.
- 오차가 줄지 않으면 손만 보지 말고 관절을 봄. 한계에 걸려 있는지가 먼저임.

### 한계

- 학습한 정책을 부스의 임무(강의 46 · 48)에 쓰지는 않았음. 정책은 가구와 컵을 피하지 않음.
- 끝점은 손끝이 아니라 손이 붙은 링크임. 손끝을 목표로 하려면 19 cm 의 차이를 따로 넣어야 함.
- 베이스를 고정했음. 팔이 움직일 때 베이스가 밀리는 것(강의 39)은 들어 있지 않음.
- 설정마다 한 번씩만 학습했음. 씨앗을 바꿔 되풀이하지 않았음.
- Isaac Lab 에 든 OpenArm 과제의 재생(play)은 돌려 보지 않았음. 학습의 기록만 봤음.
