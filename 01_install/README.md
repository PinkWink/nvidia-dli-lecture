# 1. Isaac Sim 5.1 설치 — 명령 모음

영상에서 입력하는 터미널 명령을 순서대로 모았습니다. 한 줄씩 복사해 붙여 넣으세요.

- 터미널 열기: `Ctrl` + `Alt` + `T`
- 터미널에 붙여 넣기: `Ctrl` + `Shift` + `V`
- 기준 환경: Ubuntu 24.04 · NVIDIA 드라이버 580 이상 · RTX 계열 GPU

## 1. GPU 와 드라이버 확인

```bash
nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv
```

```text
name, driver_version, memory.total [MiB]
NVIDIA GeForce RTX 5060 Ti, 580.126.16, 16311 MiB
```

`driver_version` 이 580 이상이면 통과입니다.

### 드라이버 설치 (580 보다 낮거나 `command not found` 일 때만)

```bash
sudo apt update
sudo apt install -y nvidia-driver-580-open
sudo reboot
```

RTX 50 시리즈는 반드시 `-open` 이 붙은 드라이버를 씁니다.

## 2. 예제 코드 받기

```bash
sudo apt install -y git wget unzip
cd ~
git clone https://github.com/PinkWink/nvidia-dli-lecture.git
cd ~/nvidia-dli-lecture
ls
```

## 3. 내 PC 점검

```bash
cd ~/nvidia-dli-lecture
bash 01_install/check_env.sh
```

```text
1. 운영체제
  [통과] Ubuntu 24.04
2. GPU 와 드라이버
  [통과] GPU: NVIDIA GeForce RTX 5060 Ti
  [통과] 드라이버: 580.126.16
  [통과] VRAM: 16 GB
3. 메모리와 디스크
  [통과] RAM: 31 GB
  [통과] 홈 폴더 남은 공간: 106 GB
4. Isaac Sim
  [주의] /home/pw/isaacsim 에 아직 없습니다. 01_install/install_isaacsim.sh 로 설치하세요.

결과: 진행해도 됩니다.
```

`[실패]` 가 없으면 다음으로 넘어갑니다. 설치 전이라 4번의 `[주의]` 는 정상입니다.

## 4. Isaac Sim 내려받기 · 풀기

방법 A 와 B 중 하나만 합니다.

### 방법 A: 스크립트로 한 번에

```bash
cd ~/nvidia-dli-lecture
bash 01_install/install_isaacsim.sh
```

내려받기 8.8 GB, 풀면 약 17 GB 입니다. 중간에 끊기면 같은 명령을 다시 실행합니다 (이어받습니다).

### 방법 B: 한 줄씩 직접

```bash
cd ~/Downloads
wget -c https://downloads.isaacsim.nvidia.com/isaac-sim-standalone-5.1.0-linux-x86_64.zip
ls -l isaac-sim-standalone-5.1.0-linux-x86_64.zip
```

크기가 `8768419777` 이면 끝까지 받은 것입니다. 작으면 `wget -c` 줄을 다시 실행합니다.

```bash
mkdir -p ~/isaacsim
unzip -q ~/Downloads/isaac-sim-standalone-5.1.0-linux-x86_64.zip -d ~/isaacsim
cat ~/isaacsim/VERSION
cd ~/isaacsim
./post_install.sh
```

`VERSION` 이 `5.1.0` 으로 시작하면 맞습니다.

## 5. 설치 위치 확인

```bash
bash ~/nvidia-dli-lecture/01_install/check_env.sh
```

4번이 `[통과] /home/<내 이름>/isaacsim (버전 5.1.0)` 으로 바뀌면 설치가 끝난 것입니다.

## 6. 호환성 검사

```bash
cd ~/isaacsim
./isaac-sim.compatibility_check.sh
```

약 3분 걸립니다. 마지막 줄이 `System checking result: PASSED` 면 통과입니다. 함께 뜨는 창은 `CLOSE` 로 닫습니다.

## 7. 설치 확인 (실제로 켜 보기)

```bash
cd ~/nvidia-dli-lecture
git pull
bash 01_install/verify_install.sh
```

```text
1. 설치된 파일
  [통과] isaac-sim.sh
  [통과] python.sh
  [통과] 버전 5.1.0
  [통과] post_install.sh 실행됨
2. 실제로 켜 보기 (창 없이, 처음에는 오래 걸릴 수 있습니다)
  [통과] Isaac Sim 5.1.0 이 켜졌습니다
  [통과] 물리 시뮬레이션: 큐브가 0.5초에 5.00 m → 3.56 m 로 떨어졌습니다
  [통과] 정상 종료 (6초)

결과: 설치가 정상입니다.
```

마지막 줄이 `결과: 설치가 정상입니다.` 면 끝입니다. `[실패]` 가 있으면 그 줄의 안내를 따릅니다.

## 막힐 때

| 메시지 · 증상 | 해결 |
|---|---|
| `No such file or directory` | `pwd` 로 위치 확인, `ls` 로 이름 확인 |
| `Permission denied` (`./isaac-sim.sh`) | 4절의 `unzip` 으로 다시 풀기 |
| `Could not get lock /var/lib/dpkg/lock` | 1~2분 기다린 뒤 다시 |
| `No space left on device` | `df -h ~` 로 확인. 설치가 끝난 zip 은 `~/Downloads` 에서 지워도 됨 |
| `unzip` 이 `replace ...?` 를 물음 | `A` → `Enter` |

다음: [2. Isaac Sim 실행과 화면 구성](../02_launch/README.md)
