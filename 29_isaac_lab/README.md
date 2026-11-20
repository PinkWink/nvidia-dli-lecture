# 29. Isaac Lab 시작: 강화학습의 생각, 설치, 둘러보기 — 명령 모음

영상에서 입력하는 터미널 명령과 기대 출력을 순서대로 모았습니다. [28. 합성 데이터로 YOLO 학습](../28_palletjack_detect/README.md) 다음 내용입니다.


## 1. 강화학습

### 지금까지와 무엇이 다른가

- 강의 21 의 차선 추종은 사람이 규칙을 썼습니다. "차로 가운데에서 벗어난 만큼 반대로 돌려라."
- 강의 20 · 28 은 **지도학습**입니다. 정답(도로 마스크, 상자)이 붙은 데이터로 배웠습니다.
- **강화학습**(Reinforcement Learning)은 규칙도 정답도 주지 않습니다. 로봇이 직접 해 보고, 잘했는지를 점수(보상)로만 알려 줍니다.

### 한 바퀴

| 말 | 뜻 | 예 (막대 세우기) |
|---|---|---|
| 정책 (policy) | 관측을 받아 행동을 고르는 신경망 | 숫자 4개 → 숫자 1개 |
| 관측 (observation) | 로봇이 지금 알 수 있는 값 | 카트 위치 · 속도, 막대 각도 · 각속도 |
| 행동 (action) | 정책이 내는 값 | 카트를 미는 힘 |
| 보상 (reward) | 방금 행동이 좋았는지의 점수 | 막대가 똑바를수록 높음 |
| 에피소드 (episode) | 시작부터 끝까지의 한 번 | 5초, 또는 카트가 레일 밖으로 나갈 때까지 |

- 처음의 정책은 무작위로 움직입니다. 보상의 합이 커지는 쪽으로 신경망을 조금씩 고칩니다.
- 이 과정에서 쓰는 알고리즘은 **PPO** 입니다. Isaac Lab 에서 가장 널리 쓰입니다.

### 왜 시뮬레이터인가

- 강화학습은 수백만 번 해 봐야 합니다. 실물 로봇 한 대로는 시간이 너무 걸리고 로봇이 망가집니다.
- 시뮬레이터에서는 같은 로봇 수천 대를 GPU 한 장에서 동시에 돌립니다. 다음 강의에서 4,096대를 한꺼번에 돌립니다.

## 2. Isaac Lab

### Isaac Sim 과의 관계

- **Isaac Sim** 은 시뮬레이터입니다. 물리 계산과 화면 그리기를 합니다.
- **Isaac Lab** 은 Isaac Sim 위에서 도는 로봇 학습 도구입니다. 과제(관측 · 행동 · 보상)를 파이썬으로 적고, 학습 라이브러리에 연결해 줍니다.
- Isaac Lab 은 Isaac Sim 의 파이썬을 그대로 씁니다. 따로 파이썬을 설치하지 않습니다.

### 에셋에서 정책까지

- 이 과정에서 만든 USD 로봇이 ① 의 입력이 됩니다.
- 학습이 끝나면 정책 파일(`.pt`)이 남습니다.

## 3. 설치

### 준비

- 강의 1 에서 설치한 Isaac Sim 5.1 (`~/isaacsim`).
- 인터넷 연결. GitHub 와 파이썬 패키지, 그리고 실행할 때 로봇 모델을 받습니다.

### 받기와 Isaac Sim 연결

```bash
cd ~
git clone -b v2.3.2 https://github.com/isaac-sim/IsaacLab.git
cd ~/IsaacLab
ln -s ~/isaacsim _isaac_sim
ls -l _isaac_sim
```

```text
lrwxrwxrwx 1 내이름 내이름 20 ... _isaac_sim -> /home/내이름/isaacsim
```

- `-b v2.3.2` 로 2.3.2 판을 받습니다. Isaac Sim 5.1 과 맞는 판입니다. `detached HEAD` 안내가 나오는데, 판 하나를 그대로 받았다는 뜻입니다.
- `ln -s` 는 바로 가기를 만듭니다. Isaac Lab 은 `_isaac_sim` 이라는 이름으로 Isaac Sim 을 찾습니다.

### Isaac Lab 본체 설치

```bash
sudo apt install -y cmake build-essential
./isaaclab.sh --install none
```

```text
Successfully installed isaaclab_assets-0.2.4
Successfully installed isaaclab_mimic-1.0.16
Successfully installed isaaclab_rl-0.4.7
```

- 패키지를 내려받느라 몇 분 걸립니다.
- `none` 은 강화학습 라이브러리를 아직 설치하지 않는다는 뜻입니다. 다음 단계에서 쓸 것 두 개만 넣습니다.

> **--install 만 쓰지 않는 이유**: `./isaaclab.sh --install` 은 강화학습 라이브러리를 모두 설치합니다. 2026년 10월에 강의 PC 에서 해 보니, 그중 하나가 더 새로운 PyTorch 를 요구해 수 GB 를 받았다가 Isaac Sim 의 PyTorch 2.7.0 으로 되돌리는 일이 생겼습니다. 그래서 필요한 두 개만 설치합니다.

### 강화학습 라이브러리 두 개

```bash
./isaaclab.sh -p -m pip install -e "source/isaaclab_rl[skrl,rsl-rl]"
./isaaclab.sh -p -m pip install "skrl==1.4.3"
./isaaclab.sh -p -c "import torch, skrl; print(torch.__version__, skrl.__version__)"
```

```text
2.7.0+cu128 1.4.3
```

- **`./isaaclab.sh -p`** 는 "Isaac Sim 의 파이썬으로 실행"입니다. 앞으로 Isaac Lab 의 스크립트는 모두 이렇게 실행합니다.
- **skrl** 은 강의 30 ~ 31 의 Cartpole 에서, **rsl_rl** 은 그 뒤의 진자와 사족보행에서 씁니다.
- 둘째 줄에서 skrl 을 1.4.3 으로 맞춥니다. 빠뜨리면 skrl 2.x 가 남습니다. 그러면 학습은 되지만 강의 31 의 재생이 `'PPO' object has no attribute 'set_running_mode'` 로 멈춥니다 (강의 PC 에서 확인).

> **강의 PC 의 설치**: 강의 PC 에는 2026년 10월 25일에 위 명령으로 설치했고, 위 출력은 그때의 기록입니다. 이번에 설치를 처음부터 다시 하지는 않았습니다. 버전 확인 줄과 아래 4 · 5 절은 2026년 11월 20일에 다시 실행했습니다.

## 4. 들어 있는 환경

### 환경 목록

```bash
./isaaclab.sh -p scripts/environments/list_envs.py --keyword Cartpole
```

```text
|   28   | Isaac-Cartpole-v0         | isaaclab.envs:ManagerBasedRLEnv | ...cartpole_env_cfg:CartpoleEnvCfg |
|   29   | Isaac-Cartpole-RGB-v0     | isaaclab.envs:ManagerBasedRLEnv | ...
```

- Isaac Sim 이 창 없이 잠깐 켜졌다 꺼지므로 10 ~ 20초 걸립니다. 위쪽의 노란 경고는 무시합니다.
- `--keyword` 를 빼면 전부 나옵니다. 2.3.2 에는 160개가 넘는 환경이 들어 있습니다.
- 이름은 `Isaac-<과제>-<로봇>-v0` 꼴입니다.

### 과제를 적는 두 방식

| | Manager-Based | Direct |
|---|---|---|
| 적는 법 | 관측 · 행동 · 보상을 **설정 항목**으로 나열 | 클래스 하나에 함수로 직접 |
| 좋은 점 | 항목을 갈아 끼우기 쉽고 읽기 쉽다 | 자유롭고 빠르다 |
| 이름 | `Isaac-Cartpole-v0` | `Isaac-Cartpole-Direct-v0` |
| 이 과정 | 강의 30 ~ 31 (Cartpole), 사족보행 | 핑크랩 진자 |

## 5. 로봇 둘러보기

### 로봇팔: arms.py

```bash
./isaaclab.sh -p scripts/demos/arms.py
```

- Isaac Lab 의 스크립트는 Isaac Sim 창을 그대로 띄웁니다.
- 처음 실행은 로봇 모델을 받느라 1 ~ 2분 걸릴 수 있습니다. 두 번째부터는 15초 안팎입니다.
- 왼쪽 위에 **Simulation Settings** 라는 작은 창이 뜨면 X 로 닫습니다.
- 끝낼 때는 터미널에서 `Ctrl` + `C` 를 누릅니다.

- Franka, UR10, Kinova, Sawyer 가 관절을 무작위로 움직입니다.

### 사족 로봇: quadrupeds.py

```bash
./isaaclab.sh -p scripts/demos/quadrupeds.py
```

- ANYmal B · C · D, Spot, Unitree A1 · Go1 · Go2 입니다. 이 과정의 강화학습 마지막에 ANYmal 을 걷게 합니다.

### 손: hands.py

```bash
./isaaclab.sh -p scripts/demos/hands.py
```

### 이족 · 휴머노이드: bipeds.py

```bash
./isaaclab.sh -p scripts/demos/bipeds.py
```

- Cassie, Unitree H1, Unitree G1 입니다.

> **화면 그림**: 5절의 화면은 2026년 10월에 강의 PC 에서 같은 명령으로 찍은 것입니다. 11월 20일에는 `arms.py` 와 `quadrupeds.py` 가 오류 없이 켜지는 것까지 다시 확인했습니다. `hands.py` 와 `bipeds.py` 는 이번에 다시 실행하지 않았습니다.

## 6. 정리

- **강화학습**: 정책이 관측을 받아 행동하고, 보상의 합이 커지는 쪽으로 고쳐 갑니다.
- **Isaac Lab**: Isaac Sim 위에서 과제를 적고 학습 라이브러리에 연결하는 도구입니다. 수천 대를 동시에 돌립니다.
- 스크립트는 `./isaaclab.sh -p` 로 실행합니다.
- 설치한 것: Isaac Lab 2.3.2, skrl 1.4.3, rsl_rl.

### 다음 강의

- 가장 단순한 과제 Cartpole 을 내 프로젝트로 만들고 학습시킵니다.
