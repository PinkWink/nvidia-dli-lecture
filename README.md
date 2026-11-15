# nvidia-dli-lecture

NVIDIA Isaac Sim 온라인 강의(YouTube)의 예제 코드입니다. 강의가 진행되는 대로 폴더가 추가됩니다.

## 기준 환경

| 항목 | 버전 |
|---|---|
| 운영체제 | Ubuntu 24.04 |
| Isaac Sim | 5.1.0 (`~/isaacsim` 에 설치) |
| NVIDIA 드라이버 | 580 이상 |
| GPU | RTX 계열, VRAM 16 GB 권장 |

모든 코드는 위 환경에서 실제로 실행해 확인했습니다.

## 받기

```bash
cd ~
git clone https://github.com/PinkWink/nvidia-dli-lecture.git
cd ~/nvidia-dli-lecture
```

새 강의의 코드는 `git pull` 로 받습니다.

## 폴더

| 폴더 | 강의 | 내용 |
|---|---|---|
| `01_install/` | 1. Isaac Sim 5.1 설치 | `check_env.sh` PC 점검 · `install_isaacsim.sh` 내려받기와 설치 |
| `02_launch/` | 2. Isaac Sim 실행과 화면 구성 | `hello_isaacsim.py` Python 으로 Isaac Sim 켜기 |
| `03_first_scene/` | 3. 첫 장면: 큐브를 떨어뜨리며 배우는 물리 기초 | `falling_cube.py` 강체 · 충돌체 · 바닥을 코드로 |

## 실행

```bash
# 1. 내 PC 점검, Isaac Sim 설치
bash 01_install/check_env.sh
bash 01_install/install_isaacsim.sh

# 2. Python 으로 Isaac Sim 켜기
~/isaacsim/python.sh 02_launch/hello_isaacsim.py

# 3. 떨어지는 큐브
~/isaacsim/python.sh 03_first_scene/falling_cube.py
~/isaacsim/python.sh 03_first_scene/falling_cube.py --no-ground --seconds 1
~/isaacsim/python.sh 03_first_scene/falling_cube.py --gui
```

- Python 스크립트는 시스템의 `python3` 가 아니라 Isaac Sim 에 들어 있는 `~/isaacsim/python.sh` 로 실행합니다.
- Isaac Sim 을 다른 곳에 설치했다면 `~/isaacsim` 자리에 그 경로를 씁니다. 셸 스크립트는 `ISAACSIM_PATH=/경로 bash ...` 로 지정합니다.
- 내가 만든 장면은 `work/` 에 저장합니다 (git 이 무시하는 폴더).
