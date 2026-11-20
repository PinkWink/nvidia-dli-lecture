# 32. 핑크랩 로터리 역진자: 보상을 직접 설계해 세우기 — 명령 모음

- 영상에서 입력하는 터미널 명령과 기대 출력을 순서대로 모았음. [31. Cartpole 들여다보기](../31_cartpole_code/README.md) 다음 내용임.

| 파일 |
|---|
| [pendulum_env.py](pendulum_env.py) |
| [play.py](play.py) |
| [ppo_cfg.py](ppo_cfg.py) |
| [train.py](train.py) |

## 1. 로터리 역진자

### 무엇인가

- 모터가 **팔**을 수평으로 돌림. 팔 끝의 **진자**에는 모터가 없고 자유롭게 돎.
- 팔을 잘 흔들어 매달린 진자를 위로 올리고(swing-up), 세운 채 버팀(balance).
- 핑크랩이 실물로 만들어 전시에서 돌린 장치임. 이 강의는 그 장치의 시뮬레이션 모델을 씀.

### Cartpole 과의 차이

| | Cartpole (강의 30 ~ 31) | 로터리 역진자 |
|---|---|---|
| 움직이는 것 | 카트가 직선으로 | 팔이 원을 그리며 |
| 시작 자세 | 막대가 위쪽 ±45° 안 | 진자가 **아래로 매달림** |
| 할 일 | 버티기 | 흔들어 올리기 + 버티기 |
| 행동 | 카트를 미는 힘 | 팔의 각가속도 |
| 과제 코드 | 생성기가 만들어 준 것 | **직접 적음** |
| 적는 방식 | Manager-Based | Direct |
| 학습 라이브러리 | skrl | rsl_rl |

### 무엇이 어려운가

- 버티기만 하면 되는 Cartpole 과 달리, 먼저 **멀리 돌아가야** 함. 진자를 올리려면 일단 반대로 흔들어야 하기 때문임. 보상 설계가 중요해지는 이유임.

### 실물과 맞춘 것

| 항목 | 실물 | 시뮬레이션 |
|---|---|---|
| 팔 구동 | 스텝모터. 컴퓨터가 각가속도를 보내면 속도로 적분 | 같은 계산. 팔은 주어진 속도를 그대로 따라감 |
| 행동의 크기 | 각가속도 ±150 rad/s², 속도 ±5 rad/s | 같음 |
| 팔 한계 | ±125° 를 넘으면 브레이크 | 같음 |
| 진자 마찰 | 흔들어 놓고 멈추는 모습을 잰 값 | 그 값을 토크로 넣음 |
| 제어 주기 | 58.33 Hz | 물리 700 Hz ÷ 12 |

- 위 숫자는 장치를 만든 팀의 인수인계 자료에 적힌 값임. 이 강의에서 실물로 다시 재지는 않았음.

## 2. 준비

### 파일 확인

```bash
cd ~/nvidia-dli-lecture
git pull
ls 32_pinklab_pendulum 32_pinklab_pendulum/model
```

```text
32_pinklab_pendulum:
model  pendulum_env.py  play.py  ppo_cfg.py  README.md  train.py

32_pinklab_pendulum/model:
configuration  model_v3_nodisc.usd
```

| 파일 | 내용 |
|---|---|
| `model/model_v3_nodisc.usd` | 진자의 시뮬레이션 모델 |
| `pendulum_env.py` | **과제**: 관측 · 행동 · 보상 · 끝 · 리셋 |
| `ppo_cfg.py` | PPO 설정 |
| `train.py`, `play.py` | 학습 · 재생 |

- 강의 29 의 Isaac Lab 설치(rsl_rl 포함)가 되어 있어야 함.

### 모델 열어 보기

- Isaac Sim 에서 `model_v3_nodisc.usd` 를 연 모습임. 분홍색 막대가 진자, 그 위의 회색 덩어리가 팔임.
- Stage 의 `ViewLight` 는 그림을 찍으려고 넣은 조명이고 모델에는 없음.

### 모델의 구조

| 이름 | 하는 일 |
|---|---|
| `base` | 바닥에 고정된 몸체 |
| `arm` | 모터가 돌리는 팔. 관절 `base_to_arm` |
| `pendulum` | 팔 끝의 진자. 관절 `arm_to_pendulum` (제한 없이 돎) |

- 강의 8 의 Carter 처럼 URDF 에서 가져온 모델임.

## 3. 과제를 적는다

- `pendulum_env.py` 에 적은 내용임. 코드는 7절에서 읽음.

### 행동 1개

- 정책이 내는 숫자 a (−1 ~ 1) → 팔의 각가속도 150 · a rad/s².
- 각가속도를 시간으로 쌓아 속도를 만들고(±5 rad/s 에서 자름), 팔은 그 속도로 돎.

### 관측 6개

| 순서 | 값 |
|---|---|
| 1 | 팔 각도 φ |
| 2, 3 | 진자 각도 θ 의 sin, cos (θ = 0 이 똑바로 선 것) |
| 4 | 팔 속도 |
| 5 | 진자 속도 |
| 6 | 직전 행동 |

- 진자는 몇 바퀴든 돌 수 있음. 각도를 그대로 주면 180° 에서 −180° 로 값이 튐. sin 과 cos 로 주면 값이 이어짐.

### 끝

- 에피소드는 8초가 지나야 끝남. 넘어져도, 팔이 한계에 닿아도 끝내지 않음.

### 리셋

- 학습 때는 진자를 **한 바퀴 어디서든** 시작시킴. 선 자세 근처에서 시작한 환경은 버티기를, 매달린 자세에서 시작한 환경은 올리기를 배움.

### 보상 (첫 번째 안)

```text
보상 = 세운 정도 − 0.02 × 팔각도² − 0.001 × 팔속도² − 0.01 × (행동의 변화)²
세운 정도 = (1 + cos θ) / 2        섰으면 1, 매달렸으면 0
```

- Cartpole 의 `pole_pos` 와 같은 생각임. 진자가 위에 가까울수록 점수를 줌.
- 나머지 세 항은 "팔은 가운데 근처에서, 천천히, 부드럽게"임.

## 4. 첫 학습: 실패

### 에너지 항 없이 300번

```bash
~/IsaacLab/isaaclab.sh -p 32_pinklab_pendulum/train.py --headless --energy_weight 0 --max_iterations 300
```

- `--energy_weight 0` 이 위의 첫 번째 안임. 강의 PC 에서 3분 58초 걸림.

### 학습의 마지막 출력

```text
                       Mean reward: 229.82
               Mean episode length: 466.00
              Mean episode upright: 0.5137
학습 시간: 238초
```

- `Mean episode upright` 가 "세운 정도"의 평균임. 0.51 에서 더 오르지 않음.

### 매달린 자세에서 평가

```bash
~/IsaacLab/isaaclab.sh -p 32_pinklab_pendulum/play.py --headless --num_envs 256 --seconds 8
```

- `play.py` 는 가장 최근에 학습한 정책을 불러, 모든 진자를 매달린 자세에서 시작시킴.
- "섰다"의 기준은 진자가 똑바른 자세의 ±10° 안에 1초 동안 머문 것임.

### 평가 결과

```text
256대를 매달린 자세에서 8초 동안 돌렸습니다.
  선 진자: 0대 (0 %)
  끝날 때 서 있는 진자: 1대
  팔이 가장 멀리 간 각도: 평균 50°, 최대 53°
```

- **한 대도 서지 못했음.** 끝나는 순간에 ±10° 안을 지나가던 것이 1대 있었을 뿐임.

### 왜 실패했나

- 매달린 진자를 올리려면 먼저 흔들어야 함. 그런데 조금 흔드는 동안에는 "세운 정도"가 거의 오르지 않고, 팔을 움직인 벌점만 쌓임.
- 정책은 "가만히 있는 편이 낫다"에서 벗어나지 못함. 평균 0.51 은 위쪽에서 시작한 환경만 버틴 결과로 보임. (평가에서 매달린 자세는 하나도 세우지 못한 것과 맞음. 다만 위쪽에서 시작한 환경을 따로 세어 보지는 않았음.)
- Cartpole 에는 없던 문제임. Cartpole 은 처음부터 위쪽에서 시작했음.

## 5. 보상 고치기: 에너지

### 생각

- 매달려 멈춘 진자는 에너지가 가장 낮고, 서서 멈춘 진자는 높음. 흔들림이 커질수록 에너지가 올라감.
- "진자의 에너지가 **서서 멈춘 상태의 에너지**에 가까울수록 좋다"를 보상에 더함. 그러면 조금 흔들기만 해도 점수가 오름. 올라가는 길에 계단을 놓아 주는 셈임.

### 보상 (두 번째 안)

```text
에너지 = (운동 에너지 + 위치 에너지) 를 "서서 멈춤 = 0, 매달려 멈춤 = −1" 이 되게 맞춘 값
보상 = 세운 정도 − 0.5 × |에너지| − (팔 각도 · 팔 속도 · 행동 변화의 벌점)
```

### 600번 학습

```bash
~/IsaacLab/isaaclab.sh -p 32_pinklab_pendulum/train.py --headless
```

```text
              Mean episode upright: 0.9407
학습 시간: 474초
저장: /home/내이름/nvidia-dli-lecture/work/pendulum_logs/2026-11-20_10-48-49
```

- 옵션을 주지 않으면 에너지 항의 가중치 0.5, 600번임. 강의 PC 에서 7분 54초(4,096대, 한 번에 0.79초).

### 학습 곡선

- 빨간 선이 첫 번째 안, 파란 선이 에너지 항을 더한 것임.
- 파란 선은 150 ~ 200번 사이에 급히 오름. 올리는 법을 찾은 때임.
- TensorBoard 로도 볼 수 있음: `~/IsaacLab/isaaclab.sh -p -m tensorboard.main --logdir work/pendulum_logs`

### 매달린 자세에서 평가

```bash
~/IsaacLab/isaaclab.sh -p 32_pinklab_pendulum/play.py --headless --num_envs 256 --seconds 8
```

### 평가 결과: 모두 섰다

```text
256대를 매달린 자세에서 8초 동안 돌렸습니다.
  선 진자: 256대 (100 %)
  서는 데 걸린 시간: 평균 1.44초, 가장 느린 것 2.22초
  끝날 때 서 있는 진자: 256대
  팔이 가장 멀리 간 각도: 평균 73°, 최대 105°
```

### 학습 중간의 정책들

```bash
~/IsaacLab/isaaclab.sh -p 32_pinklab_pendulum/play.py --headless --num_envs 256 --seconds 8 --checkpoint work/pendulum_logs/내폴더/model_100.pt
```

- 학습 중 100번마다 정책이 저장됨. `--checkpoint` 로 골라서 평가함.

### 정책 네 개의 비교

| 정책 | 선 진자 (256대 중) | 서는 데 걸린 시간 (평균) | 팔이 가장 멀리 간 각도 (최대) |
|---|---|---|---|
| `model_100.pt` | 49대 (19 %) | 4.82초 | 73° |
| `model_200.pt` | 153대 (60 %) | 2.69초 | 74° |
| `model_300.pt` | 256대 (100 %) | 2.47초 | 73° |
| `model_599.pt` | 256대 (100 %) | 1.44초 | 105° |

- 300번이면 모두 세움. 600번까지 가면 1초 더 빨리 세움.
- 대신 팔을 더 크게 휘두름(73° → 105°). 한계는 125° 임. "빨리 세운다"만 보고 고르면 안 되는 이유임.

## 6. 재생

### 창에서 보기

```bash
~/IsaacLab/isaaclab.sh -p 32_pinklab_pendulum/play.py
```

- 16대가 매달린 자세에서 시작해 진자를 세움. 8초마다 다시 시작함.
- Simulation Settings 창은 X 로 닫음. 끝낼 때는 터미널에서 `Ctrl` + `C`.

### 한 대의 8초

```bash
~/IsaacLab/isaaclab.sh -p 32_pinklab_pendulum/play.py --headless --num_envs 256 --seconds 8 --trace work/pendulum_trace.csv
```

- `--trace` 는 0번 진자의 각도와 행동을 파일에 적음.

### 진자 각도 · 팔 각도 · 행동

### 그래프 읽기

- **0 ~ 2초**: 팔을 좌우로 크게 흔듦. 행동이 +1 과 −1 을 오감. 진자는 ±180°(매달림)에서 점점 크게 흔들리다 꼭대기를 넘음.
- **2.3초 이후**: 진자가 0° 에 머묾. 행동은 ±0.1 안의 작은 값임.
- 서 있는 동안 **팔은 가운데(0°)로 돌아오지 않고** 33° 근처에서 ±10° 로 출렁임. 팔을 가운데로 모으는 벌점(0.02)이 약하기 때문으로 보임. 이 값을 바꿔 확인해 보지는 않았음.

## 7. 코드 읽기

- `32_pinklab_pendulum/pendulum_env.py` 의 중요한 부분임. Cartpole 은 설정 항목을 나열했고(Manager-Based), 여기서는 함수를 직접 씀(**Direct**).

### 구동기

```python
actuators={
    "motor": ImplicitActuatorCfg(joint_names_expr=["base_to_arm"], stiffness=0.0, damping=16414.0,
                                 effort_limit_sim=0.14, velocity_limit_sim=50.0),
    "pendulum": ImplicitActuatorCfg(joint_names_expr=["arm_to_pendulum"], stiffness=0.0, damping=0.0),
},
```

- 팔: 위치를 붙잡는 힘(`stiffness`)은 0, 속도를 따라가는 힘(`damping`)은 아주 크게. 주어진 속도를 그대로 따라가는 스텝모터를 흉내 냄.
- 진자: 둘 다 0. 모터가 없음.

### 행동: 물리 한 번마다

```python
def _apply_action(self):
    accel = self.action * MAX_ACCEL
    arm_angle = self.robot.data.joint_pos[:, self.arm]
    accel = torch.where(arm_angle >= ARM_RAIL, torch.full_like(accel, -MAX_ACCEL), accel)
    accel = torch.where(arm_angle <= -ARM_RAIL, torch.full_like(accel, MAX_ACCEL), accel)
    self.arm_speed_command = (self.arm_speed_command + accel * self.physics_dt).clamp(-MAX_ARM_SPEED, MAX_ARM_SPEED)
    self.robot.set_joint_velocity_target(self.arm_speed_command.unsqueeze(-1), joint_ids=[self.arm])
    speed = self.robot.data.joint_vel[:, self.pendulum]
    friction = -VISCOUS * speed - COULOMB * torch.tanh(speed / 0.05)
    self.robot.set_joint_effort_target(friction.unsqueeze(-1), joint_ids=[self.pendulum])
```

- 팔이 한계(±125°)를 넘으면 정책의 명령을 버리고 반대로 최대 가속함.
- 각가속도를 쌓아 속도 명령을 만듦.
- 진자에는 마찰을 토크로 넣음. 속도에 비례하는 것과, 방향만 따르는 것 두 가지임.

### 관측

```python
def _get_observations(self) -> dict:
    theta = self.upright_angle()
    observation = torch.stack([
        self.robot.data.joint_pos[:, self.arm],
        torch.sin(theta), torch.cos(theta),
        self.robot.data.joint_vel[:, self.arm],
        self.robot.data.joint_vel[:, self.pendulum],
        self.action,
    ], dim=-1)
    return {"policy": observation}
```

### 보상

```python
def _get_rewards(self) -> torch.Tensor:
    theta = self.upright_angle()
    speed = self.robot.data.joint_vel[:, self.pendulum]
    upright = (1.0 + torch.cos(theta)) / 2.0
    energy = (0.5 * PENDULUM_INERTIA * speed ** 2 + GRAVITY_TORQUE * (torch.cos(theta) - 1.0)) / (2.0 * GRAVITY_TORQUE)
    ...
    return (upright
            - self.cfg.energy_weight * energy.abs().clamp(max=2.0)
            - self.cfg.arm_center_weight * arm_angle ** 2
            - self.cfg.arm_speed_weight * arm_speed ** 2
            - self.cfg.action_rate_weight * (self.action - self.previous_action) ** 2)
```

- `energy` 의 앞부분이 운동 에너지, 뒷부분이 위치 에너지임. 서서 멈추면 0 임.
- Cartpole 과 같이, 모든 줄이 4,096대를 한 번에 계산함.

### 끝

```python
def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
    time_out = self.episode_length_buf >= self.max_episode_length - 1
    return torch.zeros_like(time_out), time_out
```

- 실패로 끝나는 경우가 없음. 벌점이 쌓이는 과제에서 "일찍 끝나기"를 허용하면, 정책이 일부러 일찍 끝내는 법을 배울 수 있음.

## 8. 정리

### 이 강의에서 한 일

- 과제를 직접 적었음: 행동 1개, 관측 6개, 보상, 끝, 리셋.
- "세운 정도"만으로는 매달린 진자를 올리지 못했음(0 %). **에너지 항**을 더하자 256대가 모두 평균 1.44초에 섰음.

### 기억할 것

- 강화학습에서 사람이 하는 일의 큰 부분은 **보상 설계**임. 원하는 곳까지 가는 길에 점수의 계단을 놓아 줌.
- 정책은 여러 개를 저장해 두고 평가해서 고름. 빠른 정책이 팔을 더 크게 휘둘렀음.

### 다음 강의

- 이 정책은 "명령이 바로 닿고 센서가 정확한" 시뮬레이션에서만 배웠음. 실물처럼 지연 · 마찰의 차이 · 센서 잡음을 넣으면 어떻게 되는지 보고, 그 조건으로 다시 학습함.
