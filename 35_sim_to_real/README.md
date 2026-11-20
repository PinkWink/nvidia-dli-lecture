# 35. Sim-to-Real: 정책을 꺼내 다른 시뮬레이터에서 돌려 보기 — 명령 모음

영상에서 입력하는 터미널 명령과 기대 출력을 순서대로 모았습니다. [34. 사족 로봇을 걷게 하기](../34_quadruped/README.md) 다음 내용입니다.

| 파일 |
|---|
| [cartpole_numpy.py](cartpole_numpy.py) |
| [eval_policy.py](eval_policy.py) |
| [export_onnx.py](export_onnx.py) |
| [isaac_rollout.py](isaac_rollout.py) |

- 숫자와 그래프, 움직이는 그림은 모두 2026년 11월 20일에 강의 PC 에서 실행한 결과입니다.
- 실물 로봇으로 옮기는 실험은 하지 않았습니다.

## 1. Sim-to-Real

### 지금까지 한 일

- 강의 30 ~ 34 에서 Cartpole · 역진자 · ANYmal 의 정책을 학습했습니다.
- 그 정책들은 모두 **Isaac Sim 안에서만** 움직였습니다.
- 시뮬레이션에서 배운 정책을 실물 로봇에 올려 쓰는 일을 **sim-to-real** 이라고 합니다.

### 현실과의 차이

- 시뮬레이션과 현실은 같지 않습니다. 이 차이를 **Reality Gap** 이라고 부릅니다.
- 정책은 자기가 겪은 조건에서만 잘합니다.
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
| 관측 잡음 | 33: 센서 눈금과 잡음 |
| 정규화 (벌점) | 32: 행동 변화 · 팔 속도, 34: 보상 ③ "부드럽게" |

- 이름을 붙이지 않았을 뿐 여러 번 썼습니다.

### 강의 34 의 ANYmal 과제에 들어 있는 것

| 방법 | 설정 이름 | 값 |
|---|---|---|
| 현실 데이터: 모터 모델 | `ANYDRIVE_3_LSTM_ACTUATOR_CFG` | 다리 토크를 신경망 파일이 계산 |
| 랜덤화: 몸통 질량 | `add_base_mass` | 환경마다 ±5 kg |
| 교란: 밀기 | `push_robot` | 10 ~ 15초마다 ±0.5 m/s |
| 관측 잡음 | `noise=Unoise(...)` | 관절 위치 ±0.01, 관절 속도 ±1.5 |
| 정규화 | `RewardsCfg` | 토크 · 가속도 · 행동 변화에 벌점 |

- 강의 34 에서 코드를 고치지 않고 걷게 한 과제 안에, 세 갈래가 모두 들어 있습니다.
- 모터 모델은 `isaaclab_assets/robots/anymal.py`, 나머지는 `locomotion/velocity/velocity_env_cfg.py` 에 있습니다.

### 밀기: 몇 초마다 몸통을 민다

```python
    push_robot = EventTerm(
        func=mdp.push_by_setting_velocity,
        mode="interval",
        interval_range_s=(10.0, 15.0),
        params={"velocity_range": {"x": (-0.5, 0.5), "y": (-0.5, 0.5)}},
    )
```

- `mode="interval"`: 10 ~ 15초마다 몸통 속도를 갑자기 바꿉니다. 넘어지지 않고 회복하는 법을 겪게 합니다.
- 재생용 과제(`-Play-`)는 이 항과 관측 잡음을 끕니다. 그래서 강의 34 의 재생에서는 밀리지 않았습니다.

### 관절 속도의 잡음이 큰 이유

```python
        joint_pos = ObsTerm(func=mdp.joint_pos_rel, noise=Unoise(n_min=-0.01, n_max=0.01))
        joint_vel = ObsTerm(func=mdp.joint_vel_rel, noise=Unoise(n_min=-1.5, n_max=1.5))
```

- 위치는 ±0.01, 속도는 ±1.5 입니다.
- 실물에서 속도는 위치의 차이로 구하기 때문에 원래 잡음이 큽니다. 강의 33 의 진자에서 속도를 각도 차이로 구한 것과 같은 이유입니다.

### 특권 정보

- 시뮬레이션에서는 쉽게 얻지만 실물에서는 잴 수 없는 값을 **특권 정보**라고 합니다. 바닥의 마찰 계수, 물체의 정확한 위치 같은 것입니다.
- PPO 는 신경망 두 개를 학습합니다. 행동을 정하는 Actor 와, 점수를 매기는 Critic 입니다.
- Critic 은 학습 때만 쓰므로 특권 정보를 줘도 됩니다. 실물에 올라가는 Actor 는 잴 수 있는 값만 받습니다.

## 3. 실물 대신 다른 시뮬레이터로

### sim-to-sim

- 실물 로봇이 없어도 옮기는 일을 겪어 볼 수 있습니다. 정책을 **전혀 다른 시뮬레이터**로 옮기는 것입니다.
- 시뮬레이터마다 계산 방식과 값이 조금씩 다릅니다. 작은 Reality Gap 입니다.
- 이 강의의 "다른 시뮬레이터"는 numpy 로 직접 짠 Cartpole 운동방정식입니다. Isaac Sim 도 PhysX 도 쓰지 않습니다.

### 흐름

| 단계 | 스크립트 | 쓰는 것 |
|---|---|---|
| ① 정책을 파일로 꺼내기 | `export_onnx.py` | PyTorch |
| ② Isaac 에서 기준 궤적 기록 | `isaac_rollout.py` | Isaac Lab (창 없이) |
| ③ numpy Cartpole 에서 실행 · 비교 | `cartpole_numpy.py` | numpy + onnxruntime |

- 세 스크립트는 `~/nvidia-dli-lecture/35_sim_to_real/` 에 있습니다.
- ① 과 ③ 도 `isaaclab.sh -p` 로 실행하지만 Isaac Sim 의 파이썬만 빌려 씁니다. 시뮬레이터는 뜨지 않습니다.

## 4. 준비: 정책 두 개

### 왜 두 개인가

- 하나는 강의 30 처럼 학습한 **기준** 정책입니다.
- 다른 하나는 막대 질량을 환경마다 0.5 ~ 5배로 바꿔 가며 학습한 **랜덤화** 정책입니다.
- 강의 33 과 같은 생각(도메인 랜덤화)이므로 여기서는 짧게 만들고, 8절에서 둘을 옮겨 비교합니다.

### 과제 파일에 한 항 넣기

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

- `~/Cartpole/source/Cartpole/Cartpole/tasks/manager_based/cartpole/cartpole_env_cfg.py` 의 `EventCfg` 맨 아래(`RewardsCfg` 위)에 넣습니다.
- ANYmal 의 `add_base_mass` 와 같은 함수입니다. 범위가 `(1.0, 1.0)` 이라 평소에는 아무것도 바뀌지 않습니다.

### 학습 두 번

```bash
cd ~/Cartpole
~/IsaacLab/isaaclab.sh -p scripts/skrl/train.py --task=Template-Cartpole-v0 --headless \
    --max_iterations 300 agent.agent.experiment.experiment_name=base
~/IsaacLab/isaaclab.sh -p scripts/skrl/train.py --task=Template-Cartpole-v0 --headless \
    --max_iterations 300 agent.agent.experiment.experiment_name=dr \
    "env.events.pole_mass.params.mass_distribution_params=[0.5,5.0]"
```

- 명령 뒤에 `이름=값` 을 붙이면 설정 파일을 고치지 않고 그 값만 바꿉니다. 두 번째 명령은 질량 범위만 바꿨습니다.
- `experiment_name` 은 결과 폴더 이름 끝에 붙습니다. 강의 PC 에서 각각 75초 걸렸습니다.

### 결과 폴더

```text
logs/skrl/cartpole_direct/2026-11-20_14-50-31_ppo_torch_base
logs/skrl/cartpole_direct/2026-11-20_14-51-46_ppo_torch_dr
```

- 날짜와 시각은 내 PC 에서 다릅니다. 아래 명령에서는 이 이름을 내 것으로 바꿉니다.

### Isaac 안에서 먼저 비교: 실패한 횟수

| 평가할 때의 막대 질량 | 기준 | 랜덤화 |
|---|---|---|
| 0.5배 | **1** | 7 |
| 1배 | **3** | 4 |
| 3배 | 53 | **13** |
| 5배 | 331 | **42** |
| 8배 (랜덤화 범위 밖) | 2,349 | **122** |

- 256대를 20초 돌려 카트가 범위 밖으로 나간 횟수입니다. 막대가 무거워지면 랜덤화 정책이 훨씬 잘 버팁니다.
- 강의 33 과 같은 결과입니다. 겪은 조건에서는 조금 손해, 달라진 조건에서는 큰 이득.

### 위 표를 잰 명령

```bash
cp ~/nvidia-dli-lecture/35_sim_to_real/eval_policy.py scripts/skrl/
~/IsaacLab/isaaclab.sh -p scripts/skrl/eval_policy.py --task=Template-Cartpole-v0 --headless \
    --checkpoint logs/skrl/cartpole_direct/2026-11-20_14-50-31_ppo_torch_base/checkpoints/best_agent.pt \
    "env.events.pole_mass.params.mass_distribution_params=[5.0,5.0]"
```

- 범위의 최소와 최대를 같게 주면 모든 환경이 그 값으로 고정됩니다.
- `eval_policy.py` 는 `play.py` 를 고쳐 창 없이 숫자만 출력하게 한 것입니다.

### 움직이는 모습: 5배 무거운 막대

- 왼쪽 = 기준 정책, 오른쪽 = 랜덤화 정책. 막대 질량을 5배로 고정하고 `play.py` 로 재생한 것입니다.
- 이 조건에서 왼쪽은 20초에 한 대당 1.29번, 오른쪽은 0.16번 실패합니다.

## 5. 정책을 파일로 꺼내기

### ONNX

- 신경망을 프레임워크와 상관없이 담는 표준 파일 형식입니다.
- 로봇의 컴퓨터에서는 PyTorch 없이 ONNX Runtime 같은 실행기로 돌립니다.
- 강의 34 의 rsl_rl 은 재생할 때 `policy.onnx` 를 저절로 만들었습니다. skrl 로 만든 Cartpole 에는 그 기능이 없어 스크립트를 씁니다.

### 내보내기

```bash
~/IsaacLab/isaaclab.sh -p ~/nvidia-dli-lecture/35_sim_to_real/export_onnx.py \
    logs/skrl/cartpole_direct/2026-11-20_14-50-31_ppo_torch_base/checkpoints/best_agent.pt
```

- 같은 명령을 `_dr` 폴더의 `best_agent.pt` 로 한 번 더 실행해 랜덤화 정책도 꺼냅니다.

### 출력

```text
obs mean   : [0.0558 0.0593 0.6017 0.3811]
obs std    : [1.2059 0.6723 5.1664 2.9332]
test obs   : [[0.  0.1 0.  0. ]] -> action -0.07211339473724365
saved      : logs/skrl/cartpole_direct/2026-11-20_14-50-31_ppo_torch_base/exported/policy.onnx (6228 bytes)
```

- 정책 전체가 6 KB 짜리 파일 하나입니다.

### 파일에 들어 있는 것

- **관측 정규화**: `(관측 − mean) / std`. 위 출력의 mean · std 는 학습하는 동안 모은 값이고, 정책의 일부입니다.
- **신경망**: 입력 4 → 32 → 32 → 출력 1.
- **평균 행동만**: 학습 때 탐색하려고 더하던 무작위는 뺍니다.

## 6. 두 시뮬레이터 맞춰 보기

### Isaac 에서 기준 궤적 기록

```bash
~/IsaacLab/isaaclab.sh -p ~/nvidia-dli-lecture/35_sim_to_real/isaac_rollout.py --headless --zero --seconds 1.5 --out isaac_zero.csv
~/IsaacLab/isaaclab.sh -p ~/nvidia-dli-lecture/35_sim_to_real/isaac_rollout.py --headless \
    --onnx logs/skrl/cartpole_direct/2026-11-20_14-50-31_ppo_torch_base/exported/policy.onnx
```

- 정해 둔 시작 자세 4개에서 돌려 CSV 로 남깁니다. 첫 줄은 행동 0(그냥 쓰러짐), 둘째 줄은 정책으로 5초입니다.
- 시작 자세 (카트 위치 m, 막대 각도 rad): (0, 0.2) · (0, −0.4) · (1.0, 0.6) · (−0.5, −0.2).

### numpy Cartpole: 물리 한 스텝

```python
def physics_step(s, f_cmd):
    x, th, xd, thd = s
    phi, phid = -th, -thd                       # phi: +x 쪽으로 기운 각도 (교과서 식)
    f = np.clip(f_cmd, -LIMIT, LIMIT) - a.damping * xd
    M, m, l, I = a.cart_mass, a.pole_mass, a.pole_com, a.pole_inertia
    c, sn = np.cos(phi), np.sin(phi)
    A = np.array([[M + m, m * l * c], [m * l * c, I + m * l * l]])
    b = np.array([f + m * l * sn * phid ** 2, m * G * l * sn])
    xdd, phidd = np.linalg.solve(A, b)
    xd, phid = xd + xdd * DT, phid + phidd * DT
    return np.array([x + xd * DT, -(phi + phid * DT), xd, -phid])
```

- 카트와 막대의 운동방정식을 풀어 1/120초 뒤의 상태를 돌려줍니다. 이것이 시뮬레이터의 전부입니다.
- Isaac 의 막대 각도는 부호가 교과서와 반대라서, 들어올 때와 나갈 때 부호를 뒤집습니다.

### 맞춰 넣은 값

| 값 | 어디서 가져왔나 | 값 |
|---|---|---|
| 카트 · 막대 질량 | `cartpole.usd` | 1.0 kg · 1.0 kg |
| 관절에서 막대 무게중심까지 | `cartpole.usd` | 0.47 m |
| 막대의 관성 | USD 에 없음 → 길이 0.94 m 막대로 가정 | 0.0736 |
| 카트 감쇠 · 힘 한도 · 행동 배율 | Isaac Lab 의 Cartpole 설정 | 10 · 400 N · ×100 |
| 시간 | 물리 1/120초, 정책은 2스텝마다 | 60 Hz |

- 관측의 순서(위치 2개, 속도 2개)도 학습 때와 똑같이 맞춥니다. 실물 로봇에 옮길 때 하는 일과 같습니다.
- 값의 출처는 원본 자료(DLI 실습)에 적힌 것을 따랐고, 이 강의에서 USD 를 다시 열어 확인하지는 않았습니다.

### 행동 0: 그냥 쓰러지는 모습

```bash
~/IsaacLab/isaaclab.sh -p ~/nvidia-dli-lecture/35_sim_to_real/cartpole_numpy.py --zero --seconds 1.5 --compare isaac_zero.csv
```

- 정책 없이 물리만 비교합니다. 두 선이 거의 겹칩니다. 가정한 관성 값이 대체로 맞다는 근거입니다.
- 맞지 않으면 값을 바꿔 가며 겹치게 맞춥니다. 이것이 **시스템 식별**의 축소판입니다.

### 정책을 numpy 에서 실행

```bash
~/IsaacLab/isaaclab.sh -p ~/nvidia-dli-lecture/35_sim_to_real/cartpole_numpy.py --compare isaac_rollout.csv \
    --onnx logs/skrl/cartpole_direct/2026-11-20_14-50-31_ppo_torch_base/exported/policy.onnx
```

```text
[NUMPY] start (x=+0.0, theta=+0.2) -> OK    final x=-0.03 m, theta=+0.00 rad
[NUMPY] start (x=+0.0, theta=-0.4) -> OK    final x=+0.14 m, theta=-0.00 rad
[NUMPY] start (x=+1.0, theta=+0.6) -> OK    final x=-0.08 m, theta=+0.00 rad
[NUMPY] start (x=-0.5, theta=-0.2) -> OK    final x=+0.02 m, theta=-0.00 rad
[NUMPY] success 4/4  (pole_mass=1.0, delay=0, obs_noise=0.0, swap=False, flip=False)
```

- Isaac 에서 배운 정책이 한 번도 본 적 없는 시뮬레이터에서 막대를 세웁니다.
- 성공의 기준: 5초 뒤 막대가 0.5 rad 안에 서 있고, 카트가 3 m 를 벗어나지 않음.

### 같은 정책, 두 시뮬레이터

- 두 선이 거의 같고, 조금씩 벌어지는 곳이 있습니다. 이것이 두 시뮬레이터 사이의 작은 차이입니다.

## 7. 차이를 일부러 넣어 보기

### 옵션 하나씩

```bash
P=logs/skrl/cartpole_direct/2026-11-20_14-50-31_ppo_torch_base/exported
N="$HOME/IsaacLab/isaaclab.sh -p $HOME/nvidia-dli-lecture/35_sim_to_real/cartpole_numpy.py"
$N --onnx $P/policy.onnx --pole_mass 3      # 모델 오차: 막대 3 kg
$N --onnx $P/policy.onnx --obs_noise 0.2    # 센서 잡음
$N --onnx $P/policy.onnx --delay 6          # 지연 6스텝 = 100 ms
$N --onnx $P/policy.onnx --swap_obs         # 관측 순서를 틀림
$N --onnx $P/policy.onnx --flip_angle       # 막대 각도의 부호를 틀림
```

- numpy 시뮬레이터는 한 번 실행에 1초도 걸리지 않습니다. 조건을 바꿔 가며 빠르게 볼 수 있습니다.
- zsh 에서는 `$N` 대신 `${=N}` 으로 적습니다.

### 결과

| 넣은 차이 | 종류 | 4개 중 성공 |
|---|---|---|
| 막대 질량 2 · 3 · 5 kg | 모델 오차 | 4 |
| 관측 잡음 σ 0.05 · 0.2 | 센서 잡음 | 4 |
| 지연 2 · 4스텝 (33 · 67 ms) | 지연 | 4 |
| **지연 6스텝 (100 ms)** | 지연 | **1** |
| **관측 순서를 (카트 2, 막대 2) 로** | 배치 실수 | **0** |
| **막대 각도의 부호 반대** | 배치 실수 | **0** |
| **관측 정규화를 빠뜨림** | 배치 실수 | **0** |

- 기준 정책(랜덤화 없음)의 결과입니다.

### 지연 6스텝

- 한 대(파랑)만 버티고, 나머지는 막대가 돌고 카트가 수십 m 밖으로 달아납니다.
- 물리 값의 차이에는 꽤 버티지만 지연은 어느 선을 넘으면 무너집니다. 강의 33 의 진자와 같습니다.

### 정규화를 빠뜨린 정책 만들기

```bash
~/IsaacLab/isaaclab.sh -p ~/nvidia-dli-lecture/35_sim_to_real/export_onnx.py --no_norm \
    logs/skrl/cartpole_direct/2026-11-20_14-50-31_ppo_torch_base/checkpoints/best_agent.pt
$N --onnx $P/policy_no_norm.onnx
```

- `--no_norm` 은 관측 정규화를 뺀 파일(`policy_no_norm.onnx`)을 만듭니다. 신경망은 그대로인데 4개 모두 실패합니다.

### 배치 실수는 정책이 좋아도 전부 실패

- 순서 · 부호 · 정규화 세 가지는 정책의 실력과 상관없이 0/4 입니다.
- 실물에 옮길 때는 관측의 순서 · 부호 · 단위 · 정규화를 학습 코드와 한 줄씩 대조합니다.
- 이런 실수를 실물 앞에서가 아니라 다른 시뮬레이터에서 먼저 잡는 것이 sim-to-sim 의 쓸모입니다.

## 8. 두 정책을 옮기면

### 시작 자세 50개로 성공률

```bash
D=logs/skrl/cartpole_direct/2026-11-20_14-51-46_ppo_torch_dr/exported
$N --onnx $P/policy.onnx --random_starts 50 --pole_mass 8
$N --onnx $D/policy.onnx --random_starts 50 --pole_mass 8
$N --onnx $P/policy.onnx --random_starts 50 --delay 3
$N --onnx $D/policy.onnx --random_starts 50 --delay 3
```

- `--random_starts 50`: 시작 자세 50개를 무작위로 뽑아(카트 ±1 m, 막대 ±0.6 rad) 성공한 개수를 셉니다.

### 결과

- **질량**: 랜덤화 정책은 8 kg 에서도 50개 모두 성공합니다. 기준 정책은 6 kg 에서 42개, 8 kg 에서 2개입니다.
- **지연**: 반대입니다. 랜덤화 정책은 3스텝(50 ms)에서 0개가 되고, 기준 정책은 4스텝(67 ms)까지 50개 모두 성공합니다.

### 읽기

- 질량을 랜덤화한 정책은 **질량에만** 튼튼해졌습니다. 겪지 않은 차이(지연)에는 오히려 더 약합니다.
- "랜덤화를 했으니 실물에서도 괜찮겠지"라고 가정할 수 없습니다. 걱정되는 차이는 학습에 넣고, 넣은 뒤에는 재 봐야 합니다.

### Isaac 의 표와 숫자가 다른 이유

- 4절의 표에서는 기준 정책이 5배 막대에서 331번 실패했는데, numpy 에서는 5 kg 에서 4개 모두 성공했습니다.
- 재는 방법이 다릅니다. 4절은 속도가 있는 무작위 자세에서 시작해 20초 동안 범위를 벗어난 횟수이고, numpy 는 멈춘 자세에서 시작해 5초 뒤에 서 있는지만 봅니다.
- 두 숫자를 직접 비교할 수는 없습니다. 같은 표 안에서 두 정책을 비교하는 데만 씁니다.

## 9. 실물로 옮길 때의 팁

### 공식 과정이 권하는 것

- **시스템을 잘 알기**: 실물을 재고, 실험 데이터를 모두 기록하고, 잡음과 교란의 크기를 데이터로 정합니다.
- **가정하지 않기**: 한 문제에서 통한 방법이 다른 문제에서도 통한다는 보장은 없습니다.
- **바닥부터**: 가장 적은 랜덤화로 시작하고, 근거가 있을 때만 더합니다.

### 첫 배치는 조심해서

- 처음 실물에 올리면 예상하지 못한 행동이 나올 수 있습니다.
- 비상 정지, 넓은 공간, 낮은 속도 제한을 준비합니다.

## 10. 해 볼 것

### 세 가지

- `--pole_inertia 0.05` 와 `0.1` 로 행동 0 비교 그림이 얼마나 벌어지는지 보고, 가장 잘 맞는 값을 찾아봅니다.
- `--obs_noise` 를 0.5, 1.0 까지 올리면 어디서 무너지나요?
- `--delay 3 --pole_mass 3` 처럼 차이 두 개를 함께 넣어 봅니다.
- 위 세 가지는 이 강의에서 실행해 보지 않았습니다.

## 11. 정리

### 이 강의에서 본 것

- 정책은 6 KB 짜리 ONNX 파일 하나로 꺼낼 수 있고, Isaac Sim 없이 돌릴 수 있습니다.
- numpy 로 짠 Cartpole 은 Isaac 의 궤적과 거의 겹쳤고, 정책은 그 안에서도 막대를 세웠습니다.
- 정책은 질량 · 잡음에는 버텼지만, 지연 100 ms 와 배치 실수 세 가지에는 무너졌습니다.

### 기억할 것

- 정책 파일에는 신경망만이 아니라 **관측 정규화**가 함께 들어가야 합니다.
- 관측의 순서 · 부호가 하나라도 다르면 정책이 아무리 좋아도 실패합니다.
- 도메인 랜덤화는 **겪게 한 차이에만** 튼튼하게 해 줍니다.

### 한계

- 정책마다 학습을 한 번씩만 했습니다. 다시 학습하면 무너지는 지점(예: 지연 몇 스텝)은 달라질 수 있습니다.
- numpy Cartpole 은 Isaac 과 맞춘 것이지 실물과 맞춘 것이 아닙니다. 실물로 옮겨 본 것이 아닙니다.
- 7절의 표는 시작 자세 4개, 8절은 50개만 본 결과입니다.
