# 31. Cartpole 들여다보기: 재생, 학습 곡선, 과제 코드 — 명령 모음

- 영상에서 입력하는 터미널 명령과 기대 출력을 순서대로 모았음. [30. 첫 강화학습](../30_cartpole_train/README.md) 다음 내용임.


## 1. 정책 재생

### train 을 play 로

```bash
cd ~/Cartpole
~/IsaacLab/isaaclab.sh -p scripts/skrl/play.py --task=Template-Cartpole-v0 --num_envs=32
```

```text
[INFO] Loading experiment from directory: /home/내이름/Cartpole/logs/skrl/cartpole_direct
```

- 학습 명령에서 `train.py` 를 `play.py` 로 바꿨음. `--num_envs=32` 로 32대만 띄움.
- 가장 최근 학습 폴더의 `best_agent.pt` 를 저절로 고름. 다른 것을 쓰려면 `--checkpoint 파일경로` 를 붙임.
- 재생은 학습이 아님. 정책이 바뀌지 않고 기록도 남지 않음.

### 재생 화면

- Simulation Settings 창을 닫고 휠로 멀어지면 32대가 보임.

- 막대가 기울면 카트가 그쪽으로 움직여 다시 세움.
- 5초마다 무작위 자세로 다시 시작하므로, 세우는 장면이 계속 나옴.

### 끝내기

- 재생은 저절로 끝나지 않음. 터미널에서 `Ctrl` + `C` 를 누름.

## 2. 학습 곡선: TensorBoard

### 켜기

```bash
~/IsaacLab/isaaclab.sh -p -m tensorboard.main --logdir logs/skrl/cartpole_direct
```

```text
TensorBoard 2.21.0 at http://localhost:6006/ (Press CTRL+C to quit)
```

- 웹 브라우저에서 `http://localhost:6006/` 을 엶. TensorBoard 는 Isaac Sim 의 파이썬에 이미 들어 있음.

### 처음 화면

- 왼쪽의 **Run** 하나가 학습 한 번임. 학습을 여러 번 했으면 색이 다른 선으로 겹쳐 그려짐.
- 가로축은 스텝(0 ~ 4,800)임.

### Total 로 걸러 보기

- 위쪽 **Filter tags** 칸에 `Total` 을 넣으면 가장 중요한 두 그래프가 남음.
- **Episode / Total timesteps (mean)**: 에피소드의 평균 길이. 최대 300(5초를 다 버팀).
- **Reward / Total reward (mean)**: 에피소드 보상 합의 평균.

### 곡선 읽기

| 스텝 | 에피소드 길이 | 보상 합 | 행동의 무작위 정도 |
|---|---|---|---|
| 48 | 31.6 | −1.14 | 1.00 |
| 960 | 15.2 | 0.00 | 0.66 |
| 1,920 | 225.9 | 3.53 | 0.33 |
| 2,400 | 291.8 | 4.64 | 0.29 |
| 4,800 | 291.8 | 4.73 | 0.14 |

- 강의 PC 의 2026년 11월 20일 학습에서 읽은 값임.
- **처음에 길이가 오히려 짧아짐**(31.6 → 15.2). 카트를 세게 밀다 레일 밖으로 나가 일찍 끝나는 때임.
- 1,000 ~ 2,400 스텝 사이에 막대 세우는 법을 찾아 길이와 보상이 급히 오름.
- **Policy / Standard deviation** 은 정책이 얼마나 무작위로 움직여 보는지임. 1.0 에서 0.14 로 줄었음. 확신이 생길수록 줄어듦.

### 보상 항목별로 보기

- `Episode_Reward` 로 거르면 보상의 항목마다 그래프가 따로 나옴.
- 학습이 이상할 때 어느 항목이 문제인지 여기서 찾음.

## 3. 과제 코드: 전체 그림

### 파일 열기

```bash
cd ~/Cartpole/source/Cartpole/Cartpole/tasks/manager_based/cartpole
ls
```

| 파일 | 내용 |
|---|---|
| `cartpole_env_cfg.py` | 장면 · 행동 · 관측 · 리셋 · 보상 · 끝 |
| `mdp/rewards.py` | 이 과제만의 보상 함수 하나 |
| `__init__.py` | 과제 이름 `Template-Cartpole-v0` 등록 |
| `agents/skrl_ppo_cfg.yaml` | PPO 의 설정값 |

- `cartpole_env_cfg.py` 를 편집기(VS Code, gedit 등)로 엶.

### 강화학습의 요소와 설정 클래스

- 강의 29 의 한 바퀴 그림의 요소마다 클래스가 하나씩 있음. 이것이 **Manager-Based** 방식임.

### 학습을 시작할 때 터미널에 나오는 표

```text
[INFO] Action Manager:  <ActionManager> contains 1 active terms.
| Index | Name         | Dimension |
|   0   | joint_effort |         1 |

[INFO] Observation Manager: <ObservationManager> contains 1 groups.
| Active Observation Terms in Group: 'policy' (shape: (4,)) |
|     0      | joint_pos_rel              |    (2,)    |
|     1      | joint_vel_rel              |    (2,)    |

[INFO] Termination Manager:  <TerminationManager> contains 2 active terms.
|   0   | time_out           |   True   |
|   1   | cart_out_of_bounds |  False   |

[INFO] Reward Manager:  <RewardManager> contains 5 active terms.
|   0   | alive       |    1.0 |
|   1   | terminating |   -2.0 |
|   2   | pole_pos    |   -1.0 |
|   3   | cart_vel    |  -0.01 |
|   4   | pole_vel    | -0.005 |
```

- 강의 30 의 학습 출력 앞부분임(테두리 줄은 줄였음). 이 표의 이름과 숫자가 아래 코드와 그대로 같음.

## 4. 과제 코드: 하나씩

### 행동

```python
@configclass
class ActionsCfg:
    joint_effort = mdp.JointEffortActionCfg(asset_name="robot", joint_names=["slider_to_cart"], scale=100.0)
```

- 정책이 내는 숫자 a 하나가 카트 관절 `slider_to_cart` 에 **힘 a × 100 N** 으로 들어감.
- 막대 관절 `cart_to_pole` 에는 힘을 주지 않음.

### 관측

```python
@configclass
class ObservationsCfg:
    @configclass
    class PolicyCfg(ObsGroup):
        joint_pos_rel = ObsTerm(func=mdp.joint_pos_rel)
        joint_vel_rel = ObsTerm(func=mdp.joint_vel_rel)

        def __post_init__(self) -> None:
            self.enable_corruption = False
            self.concatenate_terms = True

    policy: PolicyCfg = PolicyCfg()
```

- 관절 두 개의 위치 2개 + 속도 2개 = **숫자 4개**임.
- 적힌 순서대로 이어 붙임. 실물 로봇으로 옮길 때 이 순서가 같아야 함.
- `enable_corruption = False` 는 관측에 잡음을 넣지 않는다는 뜻임.

### 보상

```python
@configclass
class RewardsCfg:
    alive = RewTerm(func=mdp.is_alive, weight=1.0)
    terminating = RewTerm(func=mdp.is_terminated, weight=-2.0)
    pole_pos = RewTerm(
        func=mdp.joint_pos_target_l2,
        weight=-1.0,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=["cart_to_pole"]), "target": 0.0},
    )
    cart_vel = RewTerm(
        func=mdp.joint_vel_l1,
        weight=-0.01,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=["slider_to_cart"])},
    )
    pole_vel = RewTerm(
        func=mdp.joint_vel_l1,
        weight=-0.005,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=["cart_to_pole"])},
    )
```

| 항목 | 가중치 | 뜻 |
|---|---|---|
| `alive` | +1.0 | 살아 있는 동안 매번 더함. 오래 버티는 것이 곧 목표 |
| `terminating` | −2.0 | 카트가 레일 밖으로 나가 끝나면 뺌 |
| `pole_pos` | −1.0 | 막대 각도가 0(똑바로)에서 멀수록 뺌. **핵심 항목** |
| `cart_vel` | −0.01 | 카트가 빠르면 조금 뺌 |
| `pole_vel` | −0.005 | 막대가 빨리 돌면 조금 뺌 |

- 실제로 더해지는 값은 "함수 값 × 가중치 × 1/60초"임. 그래서 5초를 다 버틴 에피소드의 보상 합이 300 이 아니라 5 근처임(앞의 표에서 4.73).

### 보상 함수 하나: mdp/rewards.py

```python
def joint_pos_target_l2(env: ManagerBasedRLEnv, target: float, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    asset: Articulation = env.scene[asset_cfg.name]
    joint_pos = wrap_to_pi(asset.data.joint_pos[:, asset_cfg.joint_ids])
    return torch.sum(torch.square(joint_pos - target), dim=1)
```

- 막대 각도를 −π ~ π 로 맞추고, (각도 − 목표)² 을 돌려줌.
- 이 함수는 **4,096대를 한 번에** 계산함. `joint_pos` 가 4,096줄짜리 텐서이고, 반복문이 없음. GPU 에서 병렬로 계산되는 이유임.
- `is_alive` 같은 나머지 함수는 Isaac Lab 이 공통으로 제공함. 목록은 이렇게 봄.

```bash
grep -n "^def " ~/IsaacLab/source/isaaclab/isaaclab/envs/mdp/rewards.py
```

```text
32:def is_alive(env: ManagerBasedRLEnv) -> torch.Tensor:
37:def is_terminated(env: ManagerBasedRLEnv) -> torch.Tensor:
...
149:def joint_vel_l1(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg) -> torch.Tensor:
```

### 끝

```python
@configclass
class TerminationsCfg:
    time_out = DoneTerm(func=mdp.time_out, time_out=True)
    cart_out_of_bounds = DoneTerm(
        func=mdp.joint_pos_out_of_manual_limit,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=["slider_to_cart"]), "bounds": (-3.0, 3.0)},
    )
```

- 5초가 지나면 끝남. `time_out=True` 는 "실패가 아니라 시간이 다 된 것"이라는 표시임.
- 카트가 ±3 m 밖으로 나가면 끝남. 이것은 실패이고 `terminating` 벌점을 받음.
- 막대가 넘어지는 것만으로는 끝나지 않음.

### 리셋

```python
@configclass
class EventCfg:
    reset_cart_position = EventTerm(
        func=mdp.reset_joints_by_offset,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=["slider_to_cart"]),
            "position_range": (-1.0, 1.0),
            "velocity_range": (-0.5, 0.5),
        },
    )
    reset_pole_position = EventTerm(
        func=mdp.reset_joints_by_offset,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=["cart_to_pole"]),
            "position_range": (-0.25 * math.pi, 0.25 * math.pi),
            "velocity_range": (-0.25 * math.pi, 0.25 * math.pi),
        },
    )
```

- 에피소드가 끝난 환경을 다시 놓을 때 실행됨.
- 카트는 ±1 m, 막대는 ±45° 안에서 무작위로 시작함. 늘 같은 자세에서 시작하면 그 자세만 잘하는 정책이 됨.

### 장면

```python
@configclass
class CartpoleSceneCfg(InteractiveSceneCfg):
    ground = AssetBaseCfg(prim_path="/World/ground", spawn=sim_utils.GroundPlaneCfg(size=(100.0, 100.0)))
    robot: ArticulationCfg = CARTPOLE_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
    dome_light = AssetBaseCfg(
        prim_path="/World/DomeLight",
        spawn=sim_utils.DomeLightCfg(color=(0.9, 0.9, 0.9), intensity=500.0),
    )
```

- 바닥과 조명은 하나만 만들어 모든 환경이 함께 씀.
- `{ENV_REGEX_NS}/Robot` 은 환경마다 하나씩 복제됨: `/World/envs/env_0/Robot`, `env_1/Robot`, …
- `CARTPOLE_CFG` 는 Isaac Lab 에 들어 있는 Cartpole 로봇 설정(USD 파일, 시작 자세, 구동기)임.

### 묶기와 시간

```python
@configclass
class CartpoleEnvCfg(ManagerBasedRLEnvCfg):
    scene: CartpoleSceneCfg = CartpoleSceneCfg(num_envs=4096, env_spacing=4.0)
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    events: EventCfg = EventCfg()
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()

    def __post_init__(self) -> None:
        self.decimation = 2
        self.episode_length_s = 5
        self.viewer.eye = (8.0, 0.0, 5.0)
        self.sim.dt = 1 / 120
        self.sim.render_interval = self.decimation
```

- `num_envs=4096`, `env_spacing=4.0`: 4,096대를 4 m 간격으로. 명령의 `--num_envs` 가 이 값을 덮어씀.
- `sim.dt = 1/120`, `decimation = 2`: 물리는 1초에 120번, 정책은 그 두 번에 한 번 = 1초에 60번.
- `episode_length_s = 5`: 에피소드는 최대 5초 = 300번의 결정.
- `viewer.eye = (8, 0, 5)`: 창의 카메라 자리. 강의 30 에서 처음에 몇 대만 크게 보인 이유임.

### 과제 등록: __init__.py

```python
gym.register(
    id="Template-Cartpole-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.cartpole_env_cfg:CartpoleEnvCfg",
        "skrl_cfg_entry_point": f"{agents.__name__}:skrl_ppo_cfg.yaml",
    },
)
```

- `--task=Template-Cartpole-v0` 이 이 `id` 임.

## 5. 학습 설정: skrl_ppo_cfg.yaml

| 항목 | 값 | 뜻 |
|---|---|---|
| `seed` | 42 | 무작위 씨앗. 같으면 결과도 같음 |
| `models.policy.network.layers` | [32, 32] | 정책 신경망: 관측 4 → 32 → 32 → 행동 1 |
| `agent.rollouts` | 32 | 정책을 고치기 전에 모으는 스텝 수 |
| `agent.learning_epochs`, `mini_batches` | 8, 8 | 모은 경험으로 몇 번, 몇 조각으로 나눠 고치나 |
| `agent.discount_factor` | 0.99 | 먼 미래의 보상을 얼마나 중요하게 보나 |
| `agent.learning_rate` | 5e-4 | 한 번에 얼마나 크게 고치나 |
| `trainer.timesteps` | 4800 | 학습 길이 |

- 32스텝 × 4,096대 = 131,072개의 경험을 모을 때마다 정책을 한 번 고침. 4,800스텝이면 150번 고침.

## 6. 해 볼 것

- `TerminationsCfg` 의 `bounds` 를 `(-2.0, 2.0)` 으로 줄이고 다시 학습해 곡선을 비교함.
- `RewardsCfg` 의 `pole_pos` 가중치를 `0.0` 으로 바꾸면 무엇을 배우는지? 학습한 뒤 재생해 봄.
- 위 두 가지는 이 강의에서 실행해 보지 않았음. 결과를 직접 확인해 볼 것.

## 7. 정리

- `play.py` 로 학습된 정책을 보고, TensorBoard 로 곡선을 읽음.
- Manager-Based 과제는 **행동 · 관측 · 보상 · 끝 · 리셋 · 장면** 을 설정 클래스로 나열함.
- 보상 함수는 모든 환경을 한 번에 계산하는 텐서 연산임.
- 학습이 이상하면 보상 항목별 그래프를 봄.

### 다음 강의

- 같은 흐름을 실물이 있는 로봇에 적용함. 핑크랩이 만든 로터리 역진자임.
