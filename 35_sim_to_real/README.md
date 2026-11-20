# 35. Sim-to-Real: 시뮬레이션과 현실의 차이, 도메인 랜덤화의 득과 실 — 명령 모음

영상에서 입력하는 터미널 명령과 기대 출력을 순서대로 모았습니다. [34. 사족 로봇을 걷게 하기](../34_quadruped/README.md) 다음 내용입니다.

| 파일 |
|---|
| [eval_policy.py](eval_policy.py) |

- 숫자와 그래프, 움직이는 그림은 모두 2026년 11월 20일에 강의 PC 에서 실행한 결과입니다.
- 실물 로봇으로 옮기는 실험은 하지 않았습니다. "달라진 조건"은 시뮬레이션 안에서 막대 질량을 바꿔 만들었습니다.

## 1. Sim-to-Real

### 지금까지 한 일

- 강의 30 ~ 34 에서 Cartpole · 역진자 · ANYmal 의 정책을 학습했습니다.
- 그 정책들은 모두 **시뮬레이션 안에서만** 움직였습니다.
- 시뮬레이션에서 배운 정책을 실물 로봇에 올려 쓰는 일을 **sim-to-real** 이라고 합니다.

### 왜 시뮬레이션에서 배우나

- **양**: 강의 34 의 ANYmal 은 2분 54초 동안 약 3천만 번의 걸음 결정을 겪었습니다. 실물 한 대로는 모을 수 없는 양입니다.
- **안전**: 학습 초반의 행동은 엉망입니다. 실물이면 로봇도 사람도 다칩니다.
- **수고**: 실물은 넘어질 때마다 사람이 일으켜 세워야 합니다.

### 현실과의 차이

- 시뮬레이션과 현실은 같지 않습니다. 이 차이를 **Reality Gap** 이라고 부릅니다.
- 정책은 자기가 겪은 조건에서만 잘합니다. 겪지 않은 조건에서는 무너질 수 있습니다.
- 강의 33 에서 이미 봤습니다. 지연 40 ms 를 넣자 강의 32 의 정책은 256대 중 한 대도 세우지 못했습니다.

### 차이는 어디서 오나

- 세 가지 모두 "시뮬레이션이 틀렸다"기보다 "현실을 다 담지 못했다"는 뜻입니다.

### 원으로 보기

- 원은 로봇이 겪을 수 있는 조건의 모음입니다. 두 원이 겹치지 않는 부분이 차이입니다.
- 로봇을 실제로 쓰는 조건(별)이 겹친 곳에 있어야 안심할 수 있습니다.
- 방법은 둘입니다. 시뮬레이션의 원을 **넓히거나**, 현실 쪽으로 **옮깁니다**.

## 2. 차이를 줄이는 세 갈래

### 한눈에

- NVIDIA 의 공식 과정(Transferring Robot Learning Policies From Simulation to Reality)이 나누는 방식입니다.

### 이미 해 본 것

| 방법 | 어느 강의에서 |
|---|---|
| 도메인 랜덤화 (물리 값) | 33: 지연 · 마찰을 환경마다 다르게 |
| 도메인 랜덤화 (영상) | 27: 조명 · 색 · 카메라를 장마다 다르게 |
| 시작 자세 무작위 | 30: Cartpole 의 리셋, 32: 진자의 리셋 |
| 관측 잡음 | 33: 센서 눈금과 잡음, 34: ANYmal 의 관측 잡음 |
| 정규화 (벌점) | 32: 행동 변화 · 팔 속도, 34: 보상 ③ "부드럽게" |

- 이름을 붙이지 않았을 뿐 여러 번 썼습니다.

### 현실 데이터를 넣는 예: ANYmal 의 모터

```python
ANYDRIVE_3_LSTM_ACTUATOR_CFG = ActuatorNetLSTMCfg(
    joint_names_expr=[".*HAA", ".*HFE", ".*KFE"],
    network_file=f"{ISAACLAB_NUCLEUS_DIR}/ActuatorNets/ANYbotics/anydrive_3_lstm_jit.pt",
    saturation_effort=120.0,
    effort_limit=80.0,
    velocity_limit=7.5,
)
```

- `~/IsaacLab/source/isaaclab_assets/isaaclab_assets/robots/anymal.py` 에 있습니다. 강의 34 의 ANYmal-C 가 이 설정을 씁니다.
- 다리 관절의 토크를 식이 아니라 **신경망 파일**이 계산합니다. 실제 모터의 기록으로 만든 모델이라고 공식 과정은 설명합니다.
- 이것이 "시뮬레이션의 원을 현실 쪽으로 옮기는" 방법입니다.

### 특권 정보

- 시뮬레이션에서는 쉽게 얻지만 실물에서는 잴 수 없는 값을 **특권 정보**라고 합니다. 바닥의 마찰 계수, 물체의 정확한 위치 같은 것입니다.
- PPO 는 신경망 두 개를 학습합니다. 행동을 정하는 Actor 와, 점수를 매기는 Critic 입니다.
- Critic 은 학습 때만 쓰므로 특권 정보를 줘도 됩니다. 실물에 올라가는 Actor 는 잴 수 있는 값만 받습니다.

### 랜덤화에는 대가가 있다

- 원을 넓힐수록 정책은 여러 조건을 고루 하는 쪽이 되고, 한 조건에서의 솜씨는 떨어질 수 있습니다.
- 공식 과정의 퀴즈: "도메인 랜덤화는 항상 실물 성능을 높인다" → **거짓**.
- 아래 실험으로 직접 확인합니다.

## 3. 실험 준비: 막대 질량

### 질문

- Cartpole 을 학습할 때 막대 질량을 환경마다 다르게 하면, 질량이 달라져도 버틸까요?
- 그 대가로 잃는 것은 없을까요?

### 두 정책

| 정책 | 학습할 때의 막대 질량 | 폴더 이름 끝 |
|---|---|---|
| 기준 | 그대로 (1배) | `_base` |
| 랜덤화 | 환경마다 0.5 ~ 5배 중에서 뽑음 | `_dr` |

- 다른 것은 모두 같게 둡니다. 한 번에 하나만 바꿔야 차이의 원인을 말할 수 있습니다.

### 과제 파일 열기

```bash
cd ~/Cartpole
code source/Cartpole/Cartpole/tasks/manager_based/cartpole/cartpole_env_cfg.py
```

- 강의 31 에서 읽은 파일입니다. `EventCfg` 클래스를 찾습니다.

### EventCfg 맨 아래에 한 항 넣기

```python
    # startup: 막대 질량에 곱할 값. (1.0, 1.0) 이면 그대로, (0.5, 5.0) 이면 환경마다 0.5 ~ 5 배 중에서 뽑는다 (강의 35)
    pole_mass = EventTerm(
        func=mdp.randomize_rigid_body_mass,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=["pole"]),
            "mass_distribution_params": (1.0, 1.0),
            "operation": "scale",
        },
    )
```

- `reset_pole_position` 항 바로 아래, `RewardsCfg` 위에 넣습니다. 들여쓰기는 위의 항들과 같게 맞춥니다.
- `mode="startup"`: 시작할 때 한 번 실행합니다. 환경마다 값을 따로 뽑습니다.
- 범위가 `(1.0, 1.0)` 이라 지금은 아무것도 바뀌지 않습니다. 범위는 명령에서 바꿉니다.

### 기준 정책 학습

```bash
~/IsaacLab/isaaclab.sh -p scripts/skrl/train.py --task=Template-Cartpole-v0 --headless \
    --max_iterations 300 agent.agent.experiment.experiment_name=base
```

- `--max_iterations 300`: 강의 30 의 두 배(9,600 스텝)를 학습합니다.
- `agent.agent.experiment.experiment_name=base`: 결과 폴더 이름 끝에 `_base` 를 붙입니다.
- 명령 뒤에 `이름=값` 을 붙이면 설정 파일을 고치지 않고 그 값만 바꿉니다.

### 랜덤화 정책 학습

```bash
~/IsaacLab/isaaclab.sh -p scripts/skrl/train.py --task=Template-Cartpole-v0 --headless \
    --max_iterations 300 agent.agent.experiment.experiment_name=dr \
    "env.events.pole_mass.params.mass_distribution_params=[0.5,5.0]"
```

- 방금 넣은 항의 범위만 `[0.5, 5.0]` 으로 바꿨습니다.
- 4,096대가 각자 다른 질량의 막대를 받습니다. 한 번의 학습에서 가벼운 막대와 무거운 막대를 모두 겪습니다.

### 학습 결과 확인

```bash
ls logs/skrl/cartpole_direct/
```

```text
2026-11-20_14-50-31_ppo_torch_base
2026-11-20_14-51-46_ppo_torch_dr
```

- 강의 PC 에서 각각 75초 걸렸습니다 (창 없이, 9,600 스텝).
- 날짜와 시각은 내 PC 에서 다릅니다. 아래 명령에서는 이 이름을 내 것으로 바꿉니다.

## 4. 평가

### 평가 스크립트 넣기

```bash
cp ~/nvidia-dli-lecture/35_sim_to_real/eval_policy.py scripts/skrl/
```

- `play.py` 를 고친 것입니다. 창 없이 256대를 20초 돌리고 숫자 세 개를 출력합니다.
- 실패한 횟수(카트가 범위 밖으로 나감), 막대가 12° 안에 서 있던 시간의 비율, 평균 각도입니다.

### 질량을 3배로 고정해 기준 정책 평가

```bash
~/IsaacLab/isaaclab.sh -p scripts/skrl/eval_policy.py --task=Template-Cartpole-v0 --headless \
    --checkpoint logs/skrl/cartpole_direct/2026-11-20_14-50-31_ppo_torch_base/checkpoints/best_agent.pt \
    "env.events.pole_mass.params.mass_distribution_params=[3.0,3.0]"
```

- 범위의 최소와 최대를 같게 주면 모든 환경이 그 값으로 고정됩니다.

### 출력

```text
[EVAL] failures        : 53  (cart out of bounds, 0.21 per env)
[EVAL] upright ratio   : 87.7 %  (|pole angle| < 12 deg)
[EVAL] mean |angle|    : 5.6 deg
```

- 256대가 20초 동안 53번 실패했습니다.

### 같은 조건에서 랜덤화 정책 평가

```bash
~/IsaacLab/isaaclab.sh -p scripts/skrl/eval_policy.py --task=Template-Cartpole-v0 --headless \
    --checkpoint logs/skrl/cartpole_direct/2026-11-20_14-51-46_ppo_torch_dr/checkpoints/best_agent.pt \
    "env.events.pole_mass.params.mass_distribution_params=[3.0,3.0]"
```

```text
[EVAL] failures        : 13  (cart out of bounds, 0.05 per env)
[EVAL] upright ratio   : 91.1 %  (|pole angle| < 12 deg)
[EVAL] mean |angle|    : 4.9 deg
```

- 같은 3배 막대에서 실패가 53번에서 13번으로 줄었습니다.
- `[3.0,3.0]` 을 `[0.5,0.5]` · `[1.0,1.0]` · `[5.0,5.0]` · `[8.0,8.0]` 으로 바꿔 같은 방법으로 잽니다.

### 결과: 실패한 횟수

| 평가할 때의 막대 질량 | 기준 | 랜덤화 |
|---|---|---|
| 0.5배 | **1** | 7 |
| 1배 (기준 정책이 학습한 조건) | **3** | 4 |
| 3배 | 53 | **13** |
| 5배 | 331 | **42** |
| 8배 (랜덤화 범위 밖) | 2,349 | **122** |

- 256대 × 20초 동안의 횟수입니다. 적을수록 좋습니다.

### 결과: 그래프

- 왼쪽 8배의 파란 막대는 2,349 라 그래프 위로 잘렸습니다.
- 오른쪽: 기준 정책은 5배에서 서 있는 시간이 65.8 % 로 떨어지고, 랜덤화 정책은 88.1 % 를 지킵니다.

### 움직이는 모습: 5배 무거운 막대

- 왼쪽 = 기준 정책, 오른쪽 = 랜덤화 정책. 둘 다 막대 질량을 5배로 고정하고 `play.py` 로 재생한 것입니다.
- 이 조건에서 왼쪽은 20초에 한 대당 1.29번, 오른쪽은 0.16번 실패합니다. 실패하면 그 카트는 새 자세에서 다시 시작합니다.

### 재생 명령

```bash
~/IsaacLab/isaaclab.sh -p scripts/skrl/play.py --task=Template-Cartpole-v0 --num_envs=16 --real-time \
    --checkpoint logs/skrl/cartpole_direct/2026-11-20_14-50-31_ppo_torch_base/checkpoints/best_agent.pt \
    "env.events.pole_mass.params.mass_distribution_params=[5.0,5.0]"
```

- 위 그림은 여기에 `"env.viewer.eye=[11.0,11.0,8.0]" "env.viewer.lookat=[0.0,0.0,1.0]"` 을 붙여 카메라를 옮기고 찍었습니다.

## 5. 결과 읽기

### 넓힌 만큼 튼튼해진다

- 막대가 무거워지면 랜덤화 정책이 훨씬 잘 버팁니다. 5배에서 실패가 331번 대 42번입니다.
- "무거운 막대"라는 조건이 시뮬레이션의 원 안으로 들어온 것입니다.

### 대가가 있다

- 학습한 조건(1배)과 가벼운 막대(0.5배)에서는 기준 정책이 조금 낫습니다. 실패 3 대 4, 1 대 7 입니다.
- 평균 각도도 기준이 더 작습니다 (1배에서 3.1° 대 4.3°).
- 넓은 범위를 모두 맞추느라 한 조건에서의 솜씨를 조금 내준 것입니다.

### 범위 밖에서는 다시 나빠진다

- 8배는 랜덤화 범위(0.5 ~ 5배) 밖입니다. 랜덤화 정책도 실패가 42번에서 122번으로 늘었습니다.
- 그래도 기준 정책(2,349번)보다는 훨씬 낫습니다.
- 랜덤화는 겪은 범위 근처까지만 지켜 줍니다.

### 강의 33 과 같은 이야기

| | 한 조건만 겪은 정책 | 여러 조건을 겪은 정책 |
|---|---|---|
| 강의 33 진자: 불완전함 없음 | 세우는 데 1.43초 | 2.12초 |
| 강의 33 진자: 지연 40 ms + 잡음 | 0대 | 256대 |
| 이 강의 Cartpole: 1배 | 실패 3번 | 4번 |
| 이 강의 Cartpole: 5배 | 실패 331번 | 42번 |

- 과제가 달라도 모양이 같습니다. 겪은 조건에서는 조금 손해, 달라진 조건에서는 큰 이득입니다.

### 그래서 어떻게 정하나

- 실물이 어느 범위에 있을지 알아야 합니다. 실물 막대가 1배 근처라고 확신하면 기준 정책이 낫습니다.
- 모르면 넓히되, 필요한 만큼만 넓힙니다.
- 랜덤화하지 않은 차이(예: 지연)에는 이 정책도 약할 수 있습니다. 이 강의에서 확인하지는 않았습니다.

## 6. 실물로 옮길 때의 팁

### 공식 과정이 권하는 것

- **시스템을 잘 알기**: 실물을 재고, 실험 데이터를 모두 기록하고, 잡음과 교란의 크기를 데이터로 정합니다.
- **가정하지 않기**: 한 문제에서 통한 방법이 다른 문제에서도 통한다는 보장은 없습니다.
- **바닥부터**: 가장 적은 랜덤화로 시작하고, 근거가 있을 때만 더합니다.

### 첫 배치는 조심해서

- 처음 실물에 올리면 예상하지 못한 행동이 나올 수 있습니다.
- 비상 정지, 넓은 공간, 낮은 속도 제한을 준비합니다.

## 7. 해 볼 것

### 세 가지

- 범위를 `[0.8,1.5]` 로 좁히거나 `[0.1,10.0]` 으로 넓혀 학습하면 1배와 5배에서의 결과가 어떻게 바뀌나요?
- `body_names=["cart"]` 로 카트 질량도 같은 방법으로 바꿔 봅니다.
- `--max_iterations` 를 늘리면 랜덤화 정책이 1배에서의 손해를 메우나요?
- 위 세 가지는 이 강의에서 실행해 보지 않았습니다.

## 8. 정리

### 이 강의에서 본 것

- 시뮬레이션과 현실의 차이는 근사 오차 · 모델 오차 · 넣지 않은 것에서 옵니다.
- 줄이는 길은 셋입니다. 시뮬레이션을 넓히기, 현실 데이터를 넣기, 정책을 튼튼하게 하기.
- 막대 질량을 0.5 ~ 5배로 랜덤화하면 5배 막대에서 실패가 331번에서 42번으로 줄고, 1배에서는 3번에서 4번으로 조금 늘었습니다.

### 기억할 것

- 도메인 랜덤화는 **겪게 한 차이에만** 튼튼하게 해 줍니다.
- 넓힐수록 좋은 것이 아닙니다. 실물이 있을 범위만큼 넓힙니다.
- 설정 한 항과 명령 뒤의 `이름=값` 만으로 조건을 바꿔 가며 비교할 수 있습니다.

### 한계

- 정책마다 학습을 한 번씩만 했습니다. 실패 3번과 4번 같은 작은 차이는 다시 학습하면 뒤집힐 수 있습니다.
- 평가는 256대 × 20초 한 번입니다.
- 실물 Cartpole 로 옮겨 본 것이 아닙니다.
