# 16. 핑크시티 만들기: Pinky Pro 가 다니는 작은 도시 — 명령 모음

Pinky Pro 가 돌아다닐 3 × 3 m 의 작은 도시 **City of Pink (핑크시티)** 를 코드로 만듭니다. 다음 강의들(지도 만들기 · 자율주행)의 무대입니다. [15. Pinky Pro 의 센서와 위치를 ROS 2 로 내보내기](../15_pinky_ros2_topics/README.md) 다음 내용입니다.

## 1. 준비

강의 11 의 준비(`bash 11_pinky_import/prepare_pinky.sh`)가 되어 있어야 합니다. 로봇 URDF 와 핑크랩 로고 그림([공식 저장소](https://github.com/pinklab-art/pinky_pro)의 `pinklab_logo.png`)을 거기서 가져옵니다.

```bash
cd ~/nvidia-dli-lecture
git pull
```

## 2. 도시 만들기

```bash
~/isaacsim/python.sh 16_pink_city/pink_city.py --save work/pink_city/pink_city.usd
```

```text
핑크시티: 3.0 × 3.0 m, 도로 폭 0.4 m, 블록 0.65 m
장면 저장: /home/사용자/nvidia-dli-lecture/work/pink_city/pink_city.usd
사진 저장: /home/사용자/nvidia-dli-lecture/work/pink_city/pink_city_overview.png
사진 저장: /home/사용자/nvidia-dli-lecture/work/pink_city/pink_city_top.png
...
```

`work/pink_city/` 에 바닥 · 간판 그림(`floor.png`, `sign_*.png` …), 사진 7장, 장면 파일이 생깁니다.

## 3. 창으로 보기 · 키보드로 몰기

```bash
~/isaacsim/python.sh 16_pink_city/pink_city.py --gui            # 도시만
~/isaacsim/python.sh 16_pink_city/pink_city.py --robot --gui    # Pinky Pro 를 올려 W A S D 로 운전
```

뷰포트를 한 번 클릭한 뒤 W(앞) S(뒤) A(왼쪽) D(오른쪽). 창을 닫으면 끝납니다.

## 4. ROS 2 에 연결해서 열기

```bash
source /opt/ros/jazzy/setup.bash
~/isaacsim/python.sh 16_pink_city/pink_city.py --ros2 --gui
```

강의 14 · 15 의 그래프가 붙습니다: `/cmd_vel` 받기, `/scan` · `/odom` · `/tf` · `/joint_states` · `/camera/image_raw` · `/imu` · `/clock` 내보내기. 다음 강의부터 이 명령으로 시작합니다.

## 도시의 치수

| 항목 | 값 |
|---|---|
| 전체 | 3 × 3 m, 가운데가 원점 |
| 바깥 벽 | 높이 0.35 m, 두께 0.04 m |
| 도로 폭 | 0.4 m (한 차로 0.2 m) |
| 순환 도로 | 중심선이 원점에서 1.05 m, 모퉁이는 반지름 0.3 m 의 곡선 |
| 십자 도로 | X 축 · Y 축을 따라 하나씩 |
| 교차로 | 가운데 사거리 1개 + 순환 도로와 만나는 삼거리 4개 |
| 블록 | 0.65 × 0.65 m 네 개 |
| 로봇 시작 위치 | (−1.05, −0.5), +Y 방향 (순환 도로의 왼쪽 변) |

| 블록 | 내용 | 높이 |
|---|---|---|
| 오른쪽 위 | 핑크랩 본사 (옥상에 로고 간판) | 0.55 m |
| 왼쪽 위 | 핑크 아카데미 (옥상 · 입구 간판) + 상가 | 0.26 m · 0.38 m |
| 오른쪽 아래 | ㄱ 자 아파트 | 0.44 m · 0.32 m |
| 왼쪽 아래 | 공원: 나무 5그루 · 연못 · 매점 | 나무 0.20 ~ 0.26 m |

## 옵션

| 옵션 | 뜻 |
|---|---|
| `--gui` | 창을 띄운다 |
| `--robot` | Pinky Pro 를 올린다 (키보드 운전) |
| `--ros2` | Pinky Pro 를 올리고 ROS 2 에 연결한다 |
| `--seconds N` | `--ros2` 로 창 없이 돌릴 때 N 초 뒤 끝냄 |
| `--start X Y YAW` | 로봇의 시작 위치 [m] 와 방향 [도]. 기본 `-1.05 -0.5 90` (강의 21 에서 사용) |
| `--hide-lidar` | 창에서 라이다 빔을 그리지 않는다 (강의 21 에서 사용) |
| `--save 파일.usd` | 만든 장면을 저장한다 |
| `--photo 폴더` | 사진을 저장할 폴더 (기본 `work/pink_city`) |

## 막힐 때

| 증상 | 해결 |
|---|---|
| "핑크랩 로고 그림을 찾지 못해 글씨만 넣었습니다" | `bash 11_pinky_import/prepare_pinky.sh` 를 먼저 실행 |
| `--robot` 에서 URDF 가 없다고 나옴 | 위와 같음 |
| `--ros2` 에서 "ROS 2 환경이 없습니다" | 같은 터미널에서 `source /opt/ros/jazzy/setup.bash` |
| 간판 글씨가 작고 각져 보임 | DejaVu 글꼴이 없는 경우. `sudo apt install fonts-dejavu-core` |
