# 33. 사족 로봇을 걷게 하기: ANYmal 보행 학습 — 명령 모음

영상에서 입력하는 터미널 명령과 기대 출력을 순서대로 모았습니다. [32. 핑크랩 로터리 역진자](../32_pinklab_pendulum/README.md) 다음 내용입니다.


## 1. 과제

### 속도 명령을 따라 걷기

- 과제 이름: `Isaac-Velocity-Flat-Anymal-C-v0`. 평평한 바닥, Manager-Based 방식입니다.
- 로봇은 "앞뒤 속도, 좌우 속도, 회전 속도" 명령을 받고 그 속도대로 걷도록 배웁니다.
- 에피소드는 20초입니다. 정책은 0.02초마다 행동하므로 최대 1,000스텝입니다. 몸통이 땅에 닿으면 넘어진 것으로 보고 끝냅니다.

### 지금까지와의 비교

| | Cartpole | 핑크랩 진자 | ANYmal |
|---|---|---|---|
| 관측 | 4 | 6 | 48 |
| 행동 | 1 | 1 | 12 |
| 보상 항 | 5 | 5 | 11 |
| 학습 시간 (창 없이) | 36초 | 7분 54초 | 2분 54초 |
| 과제 코드 | 생성기 | 직접 적음 | Isaac Lab 에 들어 있음 |

- 숫자는 커졌지만 흐름은 같습니다: 학습 → 곡선 → 재생.

## 2. 학습

### 학습 명령

```bash
cd ~/IsaacLab
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py --task Isaac-Velocity-Flat-Anymal-C-v0 --headless
```

- 강의 32 와 같은 학습 라이브러리 rsl_rl 을 씁니다. Isaac Lab 에 들어 있는 학습 스크립트입니다.
- 처음 실행할 때는 로봇 모델을 받느라 시간이 더 걸립니다.

### 시작할 때 나오는 표: 관측과 행동

```text
| Active Observation Terms in Group: 'policy' (shape: (48,)) |
|     0     | base_lin_vel                    |    (3,)   |
|     1     | base_ang_vel                    |    (3,)   |
|     2     | projected_gravity               |    (3,)   |
|     3     | velocity_commands               |    (3,)   |
|     4     | joint_pos                       |   (12,)   |
|     5     | joint_vel                       |   (12,)   |
|     6     | actions                         |   (12,)   |

|  Active Action Terms (shape: 12)   |
|   0    | joint_pos   |          12 |
```

- 관측 48 = 몸통 속도 3 + 몸통 회전 속도 3 + 중력 방향 3 + **속도 명령 3** + 관절 각도 12 + 관절 속도 12 + 직전 행동 12.
- `projected_gravity` 는 몸통에서 본 중력의 방향입니다. 몸이 얼마나 기울었는지를 알려 줍니다.
- 행동 12 = 다리 관절 12개의 목표 각도. Cartpole 과 진자는 힘 · 가속도를 냈고, 여기서는 **목표 각도**를 냅니다.

### 보상 11개

```text
|   0   | track_lin_vel_xy_exp |      1.0 |
|   1   | track_ang_vel_z_exp  |      0.5 |
|   2   | lin_vel_z_l2         |     -2.0 |
|   3   | ang_vel_xy_l2        |    -0.05 |
|   4   | dof_torques_l2       | -2.5e-05 |
|   5   | dof_acc_l2           | -2.5e-07 |
|   6   | action_rate_l2       |    -0.01 |
|   7   | feet_air_time        |      0.5 |
|   8   | undesired_contacts   |     -1.0 |
|   9   | flat_orientation_l2  |     -5.0 |
|   10  | dof_pos_limits       |      0.0 |
```

| 묶음 | 항목 | 뜻 |
|---|---|---|
| 할 일 (+) | `track_lin_vel_xy_exp`, `track_ang_vel_z_exp` | 명령한 속도 · 회전 속도에 가까울수록 더함 |
| 걸음새 (+) | `feet_air_time` | 발을 충분히 오래 들면 더함. 발을 끌지 않게 |
| 자세 (−) | `lin_vel_z_l2`, `ang_vel_xy_l2`, `flat_orientation_l2` | 위아래로 출렁임, 몸이 기울어짐 |
| 닿으면 안 되는 곳 (−) | `undesired_contacts` | 허벅지가 땅에 닿음 |
| 부드럽게 (−) | `dof_torques_l2`, `dof_acc_l2`, `action_rate_l2` | 관절의 힘 · 가속도, 행동의 급한 변화 |

- "잘 걷기"를 한 식으로 쓰지 않습니다. **작은 항 여러 개의 합**으로 씁니다.
- 강의 32 에서 본 것처럼 항 하나를 넣고 빼는 것으로 결과가 달라집니다. 이 11개는 그런 시행착오의 결과물입니다.
- 마지막 `dof_pos_limits` 는 가중치가 0 이라 지금은 쓰이지 않습니다.

### 끝

```text
|   0   | time_out     |   True   |
|   1   | base_contact |  False   |
```

- 20초가 지나거나, 몸통이 땅에 닿으면 끝납니다.

### 학습이 끝났을 때

```text
                       Computation: 173521 steps/s (collection: 0.514s, learning 0.052s)
                       Mean reward: 21.24
               Mean episode length: 982.10
                   Total timesteps: 29491200
Training time: 174.28 seconds
```

- 강의 PC(RTX 5060 Ti 16 GB): 300번에 2분 54초.
- 에피소드 길이 982(최대 1,000): 거의 넘어지지 않고 20초를 버팁니다.
- 2,949만 스텝은 로봇 한 마리가 약 6.8일 걸을 분량입니다.
- 결과는 `logs/rsl_rl/anymal_c_flat/<날짜_시각>/` 에 저장됩니다. `model_0.pt` … `model_299.pt` 가 50번마다 하나씩입니다.

## 3. 학습 곡선

### 네 그래프

| 고친 횟수 | 보상 | 에피소드 길이 | 속도 오차 [m/s] | 넘어져 끝난 비율 |
|---|---|---|---|---|
| 0 | −0.58 | 13 | 0.02 | 0.00 |
| 30 | −3.34 | 224 | 0.33 | 0.74 |
| 60 | 0.40 | 184 | 0.15 | 0.99 |
| 90 | 8.65 | 786 | 0.39 | 0.80 |
| 120 | 14.24 | 956 | 0.36 | 0.24 |
| 200 | 19.35 | 963 | 0.24 | 0.05 |
| 299 | 21.24 | 982 | 0.22 | 0.05 |

- **0 ~ 60번**: 거의 모두 넘어집니다(넘어진 비율 0.99).
- **60 ~ 120번**: 넘어지지 않는 법을 배웁니다. 에피소드 길이가 1,000 가까이로 올라갑니다.
- **120번 이후**: 넘어짐은 5 % 로 줄고, 명령 속도와의 차이가 0.22 m/s 까지 줄어듭니다.
- 맨 처음의 값(에피소드 길이 13, 속도 오차 0.02)은 에피소드가 거의 끝나지 않은 때의 기록이라 뜻이 없습니다.

### TensorBoard 로 직접 보기

```bash
./isaaclab.sh -p -m tensorboard.main --logdir logs/rsl_rl/anymal_c_flat
```

## 4. 학습을 창으로 보기

### 30번만

```bash
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py --task Isaac-Velocity-Flat-Anymal-C-v0 --max_iterations 30 --run_name gui
```

- 처음 몇십 번은 다리를 마구 휘두르다 주저앉습니다.
- 초록 화살표가 명령한 속도, 파란 화살표가 실제 속도입니다.
- 창을 띄우면 계산이 느려집니다. 그래서 학습은 `--headless` 로 하고, 확인은 재생으로 합니다.
- 이 30번짜리 결과는 이름 끝에 `_gui` 가 붙은 폴더에 따로 저장됩니다.

## 5. 재생

### 학습한 정책으로 걷게 하기

```bash
ls logs/rsl_rl/anymal_c_flat/
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/play.py --task Isaac-Velocity-Flat-Anymal-C-Play-v0 --num_envs 32 --load_run 2026-11-20_11-09-35
```

- `--load_run` 에는 `ls` 로 본 **300번 학습한 폴더 이름**을 넣습니다. 내 PC 의 날짜와 시각으로 바꾸세요.
- 빼면 가장 최근 폴더를 씁니다. 4절의 `_gui`(30번)를 나중에 했다면 그쪽을 불러 제대로 걷지 못합니다.
- 과제 이름에 `-Play-` 가 붙은 것은 같은 과제의 재생용입니다. 마리 수가 적고, 잡음과 밀기가 꺼져 있습니다.

- 초록(명령)과 파랑(실제) 화살표가 거의 겹치면 명령대로 걷는 것입니다.

### 내보낸 정책

```bash
ls logs/rsl_rl/anymal_c_flat/2026-11-20_11-09-35/exported
```

```text
policy.onnx  policy.pt
```

- 재생하면 정책이 두 가지 형식으로 저장됩니다. 학습 환경 밖(예: 실물 로봇의 컴퓨터)에서 불러 쓰는 파일입니다.

> **화면 그림**: 4 · 5절의 화면과 움직이는 그림은 2026년 10월에 강의 PC 에서 같은 과제 · 같은 명령으로 찍은 것입니다. 표와 숫자, 학습 곡선은 2026년 11월 20일에 다시 학습한 결과입니다. 그날 재생은 정책을 불러 `exported` 폴더가 만들어지는 것까지 확인했고, 4절의 창을 띄운 학습은 다시 실행하지 않았습니다.

## 6. 해 볼 것

- `--num_envs 1024` 로 학습하면 시간과 마지막 보상이 어떻게 달라지나요?
- `--checkpoint model_50.pt` 를 붙여 50번째 정책을 재생해 봅니다. 학습 곡선의 어느 때와 같은 모습인가요?
- 다른 사족 로봇: `Isaac-Velocity-Flat-Unitree-Go2-v0`.
- 위 세 가지는 이 강의에서 실행해 보지 않았습니다.

## 7. 정리

- 강화학습 세 과제를 지나왔습니다: Cartpole(버티기) → 핑크랩 진자(올리고 버티기) → ANYmal(걷기).
- 과제가 커져도 구성은 같습니다. 관측 · 행동 · 보상 · 끝 · 리셋, 그리고 수천 대의 동시 실행.
- 걷기 같은 복잡한 동작은 **보상 항 여러 개의 합**으로 적습니다.
- 학습은 창 없이, 확인은 재생으로 합니다. 재생하면 밖에서 쓸 수 있는 정책 파일이 나옵니다.
