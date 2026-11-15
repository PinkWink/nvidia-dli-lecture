# 3. 첫 장면: 큐브를 떨어뜨리며 배우는 물리 기초 — 명령 모음

영상에서 입력하는 터미널 명령과 메뉴 순서를 모았습니다. [2. 실행과 화면 구성](../02_launch/README.md) 다음 내용입니다.

## 1. 준비

```bash
mkdir -p ~/nvidia-dli-lecture/work
cd ~/isaacsim
./isaac-sim.sh
```

Isaac Sim 이 뜨면 `File > New` 로 새 장면에서 시작합니다.

## 2. 화면에서 누르는 순서

| 순서 | 동작 | 결과 |
|---|---|---|
| 1 | `Create > Mesh > Cube` | 큐브가 생김 (보이는 모양뿐) |
| 2 | ▶ Play → ■ Stop | 아무 일도 없음 |
| 3 | `Create > Physics > Physics Scene` | 중력이 생김 |
| 4 | ▶ Play → ■ Stop | 여전히 그대로 (큐브가 물리 물체가 아님) |
| 5 | Stage 에서 Cube 클릭 → 우클릭 → `Add > Physics > Rigid Body with Colliders Preset` | 강체 + 충돌체 |
| 6 | 뷰포트 위 눈 아이콘 → `Show By Type > Physics > Colliders > Selected` | 충돌 모양이 초록 선으로 보임 |
| 7 | ▶ Play → ■ Stop | 끝없이 떨어짐 (바닥 없음) |
| 8 | `Create > Physics > Ground Plane` | 바닥이 생김 |
| 9 | Cube 선택 → Property 의 Translate Z 를 `1` 로 | 큐브가 바닥에서 0.5 m 뜸 |
| 10 | ▶ Play → ■ Stop | 바닥 위에 멈춤 (Translate Z = 0.5) |
| 11 | `File > Save As` | 아래 경로에 저장 |

저장 창에 입력할 값:

```text
경로      ~/nvidia-dli-lecture/work/
File name first_scene.usd
```

값을 바꿀 때는 항상 ■ Stop 상태에서 바꿉니다.

## 3. 같은 장면을 코드로

Isaac Sim 창을 닫은 뒤 실행합니다.

```bash
cd ~/nvidia-dli-lecture
~/isaacsim/python.sh 03_first_scene/falling_cube.py
```

```text
   시간[s]     큐브 높이 z[m]
    0.00          1.000
    0.25          0.673
    0.50          0.500
    0.75          0.500
    1.00          0.500
    1.25          0.500
    1.50          0.500
    1.75          0.500
    2.00          0.500
결과: 바닥 위에 멈췄습니다 (한 변 1 m 큐브의 중심 높이 0.5 m).
```

## 4. 하나씩 빼 보기

```bash
~/isaacsim/python.sh 03_first_scene/falling_cube.py --no-rigid-body --seconds 1
~/isaacsim/python.sh 03_first_scene/falling_cube.py --no-ground --seconds 1
```

| 옵션 | 빠진 것 | 1초 뒤 큐브 높이 |
|---|---|---|
| (없음) | 없음 | 0.500 m (바닥 위) |
| `--no-rigid-body` | 강체 · 충돌체 | 1.000 m (그대로) |
| `--no-ground` | 바닥 | −3.987 m (계속 떨어짐) |

## 5. 창으로 보기 · 장면 저장

```bash
~/isaacsim/python.sh 03_first_scene/falling_cube.py --gui
~/isaacsim/python.sh 03_first_scene/falling_cube.py --save work/falling_cube.usd
```

- `--gui` 는 창을 띄웁니다. 창을 닫으면 스크립트도 끝납니다.
- `--save` 로 저장한 파일은 `~/isaacsim/isaac-sim.sh` 로 켠 뒤 `File > Open` 으로 열 수 있습니다.

## 6. 해 볼 것

```bash
~/isaacsim/python.sh 03_first_scene/falling_cube.py --gravity-up --seconds 1
~/isaacsim/python.sh 03_first_scene/falling_cube.py --height 5 --seconds 3
```

- `--gravity-up`: 중력이 위를 향합니다.
- `--height 5`: 5 m 에서 떨어뜨립니다. 바닥에 닿는 시간을 ½ · 9.81 · t² = 4.5 로 계산한 값과 비교해 봅니다.

## 옵션 전체

| 옵션 | 뜻 | 기본값 |
|---|---|---|
| `--no-rigid-body` | 큐브에 강체 · 충돌체를 주지 않음 | |
| `--no-ground` | 바닥을 만들지 않음 | |
| `--gravity-up` | 중력 방향을 +Z 로 | |
| `--height H` | 큐브 중심의 처음 높이 [m] | 1.0 |
| `--seconds S` | 시뮬레이션할 시간 [s] | 2.0 |
| `--gui` | 창을 띄움 | |
| `--save USD` | Play 전 장면을 파일로 저장 | |
