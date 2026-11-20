# 30. 첫 강화학습: Cartpole 프로젝트를 만들고 학습하기 — 명령 모음

- 영상에서 입력하는 터미널 명령과 기대 출력을 순서대로 모았음. [29. Isaac Lab 시작](../29_isaac_lab/README.md) 다음 내용임.


## 1. Cartpole

### 무엇을 배우나

- 레일 위를 움직이는 **카트**와, 카트에 자유롭게 달린 **막대**임.
- 정책이 할 수 있는 일은 카트를 좌우로 미는 것뿐임. 막대에는 모터가 없음.
- 목표: 막대를 위로 세운 채 버티기.

### 왜 Cartpole 부터인가

- 관측이 숫자 4개, 행동이 숫자 1개임. 학습이 1분 안에 끝나 결과를 바로 볼 수 있음.
- 여기서 익힌 순서(프로젝트 → 학습 → 재생 → 곡선 보기)를 뒤의 진자와 사족보행에서 그대로 씀.

## 2. 프로젝트 만들기

### 외부 프로젝트

- Isaac Lab 폴더 안을 고치지 않고, **밖에 내 폴더**를 만들어 그 안에 과제를 씀. Isaac Lab 을 새 판으로 바꿔도 내 코드는 남음.
- `isaaclab.sh --new` 가 몇 가지를 묻고 폴더를 만들어 줌. Cartpole 과제가 들어 있는 틀이 만들어짐.

### 생성기 실행

```bash
cd ~/IsaacLab
./isaaclab.sh --new
```

- 처음 실행할 때 작은 파이썬 패키지 몇 개를 받음. 인터넷이 필요함.

### 질문 여섯 개

| 질문 | 답 | 누르는 키 |
|---|---|---|
| Task type | External | `Enter` |
| Project path | `/home/내이름` (처음부터 적혀 있는 값) | `Enter` |
| Project name | `Cartpole` | 입력 후 `Enter` |
| Isaac Lab workflow | Manager-based | `↓` `↓` `Space` `Enter` |
| RL library | skrl | `↓` `↓` `Space` `Enter` |
| RL algorithms for skrl | PPO | `↓` `Space` `Enter` |

- 뒤의 세 질문은 **여러 개를 고를 수 있는 목록**임. `↑` `↓` 로 움직이고 `Space` 로 표시(◉)한 뒤 `Enter` 로 확정함. 표시 없이 `Enter` 를 누르면 넘어가지 않음.
- Project name 은 대문자 `C` 로 시작하는 `Cartpole` 로 똑같이 적음. 폴더 이름과 파이썬 패키지 이름이 됨.

```text
? RL library: skrl
? RL algorithms for skrl: PPO

Validating specification...
Generating external project...
  |-- Copying repo files...
  |-- Copying scripts...
  |-- Copying extension files...
  |-- Generating tasks...
  |    |-- Generating 'Template-Cartpole-v0' task...
  |-- Copying vscode files...
Setting up git repo in /home/내이름/Cartpole path...
  |  Initialized empty Git repository in /home/내이름/Cartpole/.git/

--------------------------------------------------------------------------------
Project 'Cartpole' generated successfully in /home/내이름/Cartpole path.
```

- 과제 이름은 `Template-Cartpole-v0` 으로 저절로 정해짐.

> **강의 PC 에서 다르게 한 점**: 강의 PC 는 Isaac Lab 이 다른 폴더에 있고 `~/IsaacLab` 은 그 폴더의 바로 가기임. 그래서 Project path 의 기본값이 달랐고, `/home/내이름` 을 직접 입력했음. 위 명령대로 `~/IsaacLab` 에 받았다면 기본값 그대로 `Enter` 임.

### 만들어진 폴더

```bash
cd ~/Cartpole
ls
ls source/Cartpole/Cartpole/tasks/manager_based/cartpole
```

```text
pyproject.toml  README.md  scripts  source
agents  cartpole_env_cfg.py  __init__.py  mdp
```

| 위치 | 내용 |
|---|---|
| `scripts/skrl/train.py`, `play.py` | 학습 · 재생 스크립트 |
| `scripts/list_envs.py` | 이 프로젝트의 과제 목록 |
| `source/Cartpole/Cartpole/tasks/manager_based/cartpole/` | **과제 코드**. 강의 31 에서 읽음 |

### 프로젝트 설치

```bash
~/IsaacLab/isaaclab.sh -p -m pip install -e source/Cartpole
```

- 반드시 `~/Cartpole` 에서 실행함.
- `-e` 는 파일을 복사하지 않고 이 폴더를 가리키게 설치함. 코드를 고치면 다시 설치하지 않아도 반영됨.
- 설치는 "Isaac Lab 에 내 과제를 등록"하는 일임.

### 과제가 등록되었는지 확인

```bash
~/IsaacLab/isaaclab.sh -p scripts/list_envs.py
```

- 표에 `Template-Cartpole-v0` 이 보이면 됨.
- `ModuleNotFoundError: No module named 'Cartpole'` 이 나오면 설치 명령을 다른 폴더에서 실행한 것임.

## 3. 학습

### 학습 명령

```bash
cd ~/Cartpole
~/IsaacLab/isaaclab.sh -p scripts/skrl/train.py --task=Template-Cartpole-v0
```

- `scripts/skrl/train.py` 는 skrl 로 학습하는 스크립트이고, `--task` 는 방금 만든 과제 이름임.

### 창이 뜨고 학습 시작

- 강의 PC 에서는 명령 뒤 14초에 학습이 시작됨.
- 왼쪽 위의 **Simulation Settings** 창은 X 로 닫음.
- 카메라가 가까이 있어 몇 대만 크게 보임.

### 터미널의 진행 막대

```text
 19%|█▊        | 896/4800 [00:24<01:58, 33.04it/s]
```

- 4,800 스텝 중 896번째, 24초 지남, 약 2분 남음, 1초에 33스텝이라는 뜻임.
- 1스텝은 4,096대가 동시에 한 번씩 행동하는 것임.

### 마우스 휠로 멀리서 보기

- 뷰포트 위에서 휠을 아래로 굴리면 멀어짐. 4,096대가 4 m 간격으로 늘어서 있음.

### 학습 중의 변화

- 처음: 카트가 빠르게 움직이고 막대는 쓰러져 돎.
- 중간: 쓰러지는 막대가 줄고, 카트가 막대 아래로 따라감.
- 끝: 대부분의 막대가 서 있고 카트는 조금씩만 움직임.
- 막대를 세우는 법을 코드로 쓴 사람은 없음. 보상만 주었음.

### 학습 끝

```text
100%|██████████| 4800/4800 [02:12<00:00, 36.36it/s]
Training time: 132.97 seconds
```

- 학습이 끝나면 창이 저절로 닫힘.

### 시간 계산

- 정책은 1초에 60번 결정함. 4,096대 × 4,800스텝 = 약 1,966만 번임.
- 한 대로 실제 시간에 하면 약 91시간 분량임. 그것을 2분 남짓에 했음.

## 4. 결과 폴더

### 무엇이 남았나

```bash
ls logs/skrl/cartpole_direct/
ls logs/skrl/cartpole_direct/*/checkpoints
```

```text
2026-11-20_10-29-01_ppo_torch
agent_1440.pt  agent_1920.pt  agent_2400.pt  agent_2880.pt  agent_3360.pt  agent_3840.pt
agent_4320.pt  agent_4800.pt  agent_480.pt   agent_960.pt   best_agent.pt
```

| 위치 | 내용 |
|---|---|
| `logs/skrl/cartpole_direct/<날짜_시각>_ppo_torch/` | 학습 한 번의 결과. 돌릴 때마다 새로 생김 |
| `checkpoints/agent_480.pt` … `agent_4800.pt` | 480스텝마다 저장한 정책 |
| `checkpoints/best_agent.pt` | 보상이 가장 좋았던 때의 정책 |
| `events.out.tfevents.*` | 학습 곡선 기록 (강의 31 의 TensorBoard) |
| `params/env.yaml`, `agent.yaml` | 이번 학습에 쓴 설정 |

- Manager-Based 로 만들었는데 폴더 이름이 `cartpole_direct` 임. 2.3.2 의 틀에 그렇게 적혀 있을 뿐이고 동작에는 문제가 없음.

## 5. 창 없이 학습

### --headless

```bash
~/IsaacLab/isaaclab.sh -p scripts/skrl/train.py --task=Template-Cartpole-v0 --headless
```

```text
100%|██████████| 4800/4800 [00:34<00:00, 137.22it/s]
Training time: 35.85 seconds
```

| 방식 | 학습 시간 | 1초에 |
|---|---|---|
| 창 띄움 | 133초 | 36스텝 |
| `--headless` | 36초 | 137스텝 |

- 화면을 그리지 않아 3.7배 빠름. 강의 PC(RTX 5060 Ti 16 GB) 에서 잰 값임.
- 두 번의 학습 곡선은 완전히 같았음. 무작위 씨앗이 42 로 고정되어 있기 때문임.
- PC 가 버거우면 `--num_envs 1024` 처럼 대수를 줄임. 대수를 줄이면 같은 4,800스텝에서 모으는 경험도 줄어듦.

> **화면 그림**: 3절의 화면 그림과 움직이는 그림은 2026년 10월에 강의 PC 에서 같은 과제 · 같은 명령으로 찍은 것임. 글자로 적은 출력과 시간은 2026년 11월 20일에 `~/Cartpole` 을 새로 만들어 다시 잰 값임.

## 6. 정리

- `isaaclab.sh --new` 로 Isaac Lab 밖에 내 프로젝트를 만들고, `pip install -e` 로 등록함.
- `train.py --task=과제이름` 으로 학습함. `--headless` 를 붙이면 창 없이 빠르게 함.
- 4,096대가 동시에 4,800스텝. 창 없이 36초면 막대를 세움.
- 결과는 `logs/skrl/.../checkpoints` 에 정책 파일로 남음.

### 다음 강의

- 학습된 정책을 재생해 보고, 학습 곡선을 읽고, 과제 코드(관측 · 행동 · 보상)를 읽음.
