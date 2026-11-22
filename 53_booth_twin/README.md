# 53. 부스의 디지털트윈: 상자를 새로 놓으면 로봇의 길이 막히나 — 명령 모음

- 영상에서 입력하는 터미널 명령과 기대 출력을 순서대로 모았음. [52. 팔이 목표에 손을 뻗게 학습하기](../52_openarm_reach/README.md) 다음 내용임.

| 파일 |
|---|
| [review.py](review.py) |
| [twin.py](twin.py) |
| [paths/pl2m_paths.json](paths/pl2m_paths.json) |

## 1. 옮겨 보기 전에 물어봄

### 이번 강의의 결과

- 부스에 굿즈 상자를 쌓아 둘 자리를 찾는 일임. 네 곳을 놓아 보고, 로봇이 다니던 길을 막는지와 쌓은 것이 무너지는지를 실제로 옮기기 전에 확인함.
- 파란 선이 로봇이 실제로 다닌 길임. 초록 자리(C)만 괜찮고 나머지 셋은 길을 막음.

### 디지털트윈

| 쓰임 | 묻는 것 | 이 강의 |
|---|---|---|
| 설계 · 계획 | 이렇게 바꾸면 들어가나, 길이 막히나 | 여기에 해당함 |
| 운영 | 지금 어디가 멈췄나 | 다루지 않음 |
| 시뮬레이션 | 로봇을 더 넣으면 어떻게 되나 | 강의 44 ~ 48 이 한 일 |

- 디지털트윈은 실제 공간을 컴퓨터 안에 옮겨 놓고 거기서 먼저 물어보는 것임. 무엇을 결정하려는지에 따라 필요한 것이 달라짐.
- 이 강의의 질문은 "상자를 어디에, 몇 단까지 쌓아도 되나" 임. 필요한 것은 부스의 모양, 로봇이 다닌 길, 물리임. 셋 다 앞 강의에서 만들어 둔 것임.

### NVIDIA 과정에서는

| | NVIDIA 과정 | 이 강의 |
|---|---|---|
| 공간 | 창고 | 핑크랩 부스 (강의 38) |
| 로봇의 길 | MobilityGen 으로 기록한 것 | 강의 44 · 46 의 주행 기록 |
| 놓는 것 | 팔레트와 상자 | 상자 더미 |
| 도구 | 따로 만든 USD Explorer 앱 + 익스텐션 | Isaac Sim 에서 돌리는 스크립트 |

- 묻는 것은 같음. 새로 놓는 것이 로봇의 길을 막는가, 쌓으면 무너지는가.

### 만드는 것

| 파일 | 하는 일 |
|---|---|
| `twin.py` | 길 불러오기 · 물체와의 거리 · 상자 쌓기 (계산) |
| `review.py` | 검토 한 번: 지금의 부스 → 검토 안 → 낙하 시험 |
| `paths/pl2m_paths.json` | 로봇이 다닌 길 (강의 44 · 46 의 기록에서 뽑은 것) |

## 2. 재료

### 터미널: 강의 50 의 장면

```bash
cd ~/nvidia-dli-lecture
git pull
~/isaacsim/python.sh 50_usd_layers/make_layers.py
```

- 부스와 컵이 든 강의 50 의 배치(`layout.usda`)를 씀. 강의 50 을 따라 했다면 이미 있음.

### 로봇이 다닌 길

| 기록 | 내용 | 길이 |
|---|---|---|
| `tour` | 강의 44: 스테이션 셋을 세 바퀴 | 25.6 m |
| `mission_pink` · `blue` · `yellow` | 강의 46: 컵을 나르는 임무 세 번 | 8.9 ~ 9.2 m |

- 시뮬레이터가 0.1초마다 적어 둔 로봇의 참값에서, 5 cm 움직일 때마다 한 점씩 뽑았음. 네 기록을 합쳐 900점쯤임.
- 강의 44 의 `pl2m_nav.py --log` 로 자기 기록을 만들 수 있음. 저장소에는 강의 PC 에서 뽑은 것을 넣어 두었음.

## 3. 지금의 부스

### 터미널

```bash
~/isaacsim/python.sh 53_booth_twin/review.py --photo now.png
```

```text
로봇이 다닌 길: tour 25.6 m,  mission_pink 9.0 m,  mission_blue 8.9 m,  mission_yellow 9.2 m

지금의 부스: 충돌체 248개. 길의 가운데 선에서 가장 가까운 것 다섯
  People/staff_9               0.37 m   (tour)
  People/staff_5               0.41 m   (mission_yellow)
  Parts/consult_table_162      0.43 m   (mission_pink)
  Parts/marker_board_167       0.44 m   (mission_pink)
  Parts/drink_stand_top_182    0.45 m   (mission_yellow)
```

- 새로 놓기 전에 지금이 어떤지부터 봄. 로봇은 가구에 0.43 ~ 0.45 m, 앉아 있는 사람 모형에 0.37 m 까지 다가갔음.
- 테이블까지의 0.43 m 는 도킹한 거리임 (강의 45). 로봇이 일부러 다가간 곳임.

### 코드: 길을 선으로

```python
curve = UsdGeom.BasisCurves.Define(stage, f"{where}/{name}")
curve.CreateTypeAttr("linear")
curve.CreatePointsAttr([Gf.Vec3f(float(x), float(y), 0.03) for x, y in points])
curve.CreateCurveVertexCountsAttr([len(points)])
curve.CreateWidthsAttr([0.05])
```

- 점들을 잇는 선(`BasisCurves`)을 바닥에서 3 cm 위에 그림. 좌표를 바꿀 필요가 없음. 기록도 장면도 같은 부스 좌표를 씀.

### 코드: 물체와의 거리

```python
box = cache.ComputeWorldBound(prim).ComputeAlignedRange()
...
dx = np.maximum(np.maximum(low[0] - points[:, 0], 0.0), points[:, 0] - high[0])
dy = np.maximum(np.maximum(low[1] - points[:, 1], 0.0), points[:, 1] - high[1])
distance = float(np.min(np.hypot(dx, dy)))
```

- 충돌체마다 바닥에 드리운 네모를 구하고, 길의 점들에서 그 네모까지의 거리 가운데 가장 작은 것을 냄.
- 로봇의 키(1.3 m)보다 위에 뜬 것은 뺌. 머리 위의 간판은 길을 막지 않음.

## 4. 놓아 보기

### 비워 둘 거리

| | 값 |
|---|---|
| 로봇이 차지하는 폭의 절반 (강의 44 의 Nav2 설정) | 0.32 m |
| 여유 | 0.08 m |
| 길의 가운데 선에서 비워 둘 거리 | 0.40 m |

- 새로 놓는 것은 길에서 0.40 m 넘게 떨어져야 한다고 정했음. `--clear` 로 바꿀 수 있음.
- 지금의 부스에서도 한 곳(0.37 m)은 이 기준보다 좁음. 로봇이 지나갈 수 있었던 가장 좁은 곳임.

### 터미널: 길 한가운데 (A)

```bash
~/isaacsim/python.sh 53_booth_twin/review.py --at 6.5 -3.0 --layers 3 --photo a.png
```

```text
검토 안: (6.50, -3.00) 에 상자 3단 (높이 0.75 m)
  길에서 가장 가까운 거리 0.00 m (tour)  →  막힘 (비워 둘 거리 0.40 m)
  가장 가까운 가구까지 0.36 m (People/staff_9)
```

- 사람 눈에는 넓고 빈 자리임. 로봇에게는 처음 자리와 음료대를 오가는 길 한가운데임.

### 네 자리

| 자리 | (x, y) | 길에서 | 가구에서 | 판정 |
|---|---|---|---|---|
| A | (6.5, -3.0) | 0.00 m | 0.36 m | 막힘 |
| B | (6.1, -3.7) | 0.35 m | 0.28 m | 막힘 |
| C | (8.3, -3.7) | 0.76 m | 0.32 m | 괜찮음 |
| D | (5.9, -2.4) | 0.06 m | 0.05 m | 막힘 |

- B 는 길과 길 사이의 빈 곳임. 길에 걸치지는 않지만 0.35 m 라 기준(0.40 m)에 5 cm 모자람.
- C 는 3단일 때의 값임. 층을 어긋나게 쌓으면 더미가 넓어져 거리가 줄어듦 (5절).

## 5. 무너지는가

### 터미널: 5단을 10 cm 까지 어긋나게

```bash
~/isaacsim/python.sh 53_booth_twin/review.py --at 8.3 -3.7 --layers 5 --jitter 0.10 --drop 3
```

```text
검토 안: (8.30, -3.70) 에 상자 5단 (높이 1.25 m), 층마다 10 cm 까지 어긋남
  길에서 가장 가까운 거리 0.70 m (mission_yellow)  →  괜찮음 (비워 둘 거리 0.40 m)
  가장 가까운 가구까지 0.25 m (Parts/pop_161)
  3초 뒤: 모두 제자리
```

- `--jitter` 는 사람이 대충 쌓은 것을 흉내 냄. 층마다 그만큼까지 옆으로 어긋나고 ±15° 까지 돎.
- `--drop 3` 은 3초 동안 물리를 돌린 뒤 상자가 제자리에 있는지 봄. 5 cm 넘게 내려갔거나 10 cm 넘게 옆으로 간 상자를 셈.

### 터미널: 7단을 20 cm 까지 어긋나게

```bash
~/isaacsim/python.sh 53_booth_twin/review.py --at 8.3 -3.7 --layers 7 --jitter 0.20 --drop 3
```

```text
검토 안: (8.30, -3.70) 에 상자 7단 (높이 1.75 m), 층마다 20 cm 까지 어긋남
  길에서 가장 가까운 거리 0.61 m (mission_yellow)  →  괜찮음 (비워 둘 거리 0.40 m)
  가장 가까운 가구까지 0.18 m (Parts/pop_161)
  3초 뒤: 5개가 벗어남: box_3 높이 0.62 → 0.20 m · 옆으로 59 cm,  box_4 높이 0.87 → 0.15 m · 옆으로 45 cm, ...
```

- 길을 막지 않는 자리여도 무너지면 소용없음. 떨어진 상자가 59 cm 까지 옆으로 갔음. 길까지의 여유(0.61 m)를 거의 다 씀.

### 단 수와 어긋남

- 어긋남마다 씨앗을 바꿔 세 번씩 했음 (딱 맞춘 0 cm 는 한 번).
- 10 cm 까지는 7단도 버팀. 15 cm 부터 무너지는 일이 생기고, 20 cm 면 7단은 세 번 다 무너짐.
- 상자의 짧은 변이 30 cm 임. 어긋남이 그 절반(15 cm)에 가까워지면 위의 상자가 아래 상자 밖으로 나가기 시작함.

### 결론

| 물음 | 답 |
|---|---|
| 어디에 | C (8.3, -3.7). 길에서 0.70 m |
| 몇 단까지 | 7단 (1.75 m) 까지 버팀. 단, 층이 10 cm 넘게 어긋나지 않게 |
| 조심할 것 | 옆의 POP 간판(`pop_161`)과 0.25 m 임 |

- 숫자로 답이 나옴. 이것이 검토의 결과물임.

## 6. 검토 안을 파일로

### 터미널

```bash
~/isaacsim/python.sh 53_booth_twin/review.py --at 8.3 -3.7 --layers 5 --jitter 0.10 --save goods
```

```text
  저장: /home/내이름/nvidia-dli-lecture/work/booth_layers/goods.usda
```

### 파일: goods.usda

```text
over "World"
{
    def "Proposal"
    {
        def Xform "stack"
        {
            def Cube "box_1" (
                prepend apiSchemas = ["PhysicsCollisionAPI", "PhysicsRigidBodyAPI", "PhysicsMassAPI"]
            )
            {
                float physics:mass = 3
                double3 xformOp:translate = (8.3, -3.7, 0.127)
                ...
```

- 검토 안은 원래 장면(`layout.usda`)에 적지 않고 따로 둔 레이어에 적음. 강의 50 의 `rearrange.usda` 와 같은 방식임.
- 이 파일을 `scene.usda` 의 `subLayers` 에 더하면 상자가 놓인 부스가 됨. 빼면 원래대로임.

### 코드: 레이어를 얹고 그 안에 적기

```python
proposal = Sdf.Layer.CreateAnonymous("proposal.usda")
stage.GetRootLayer().subLayerPaths.insert(0, proposal.identifier)
with Usd.EditContext(stage, proposal):
    twin.add_stack(stage, args.at[0], args.at[1], args.layers, args.jitter, args.seed)
```

- 빈 레이어를 하나 만들어 장면 위에 얹고, `Usd.EditContext` 안에서 상자를 만듦. 그 안에서 적는 것은 모두 그 레이어로 감.
- `--save` 를 주지 않으면 이 레이어는 프로그램이 끝날 때 사라짐. 원래 파일은 그대로임.

### 고치는 동안 본 실패

| | 처음 짠 것 | 고친 것 |
|---|---|---|
| 레이어를 얹은 곳 | 세션 레이어 | 루트 레이어 |
| 30 cm 씩 어긋난 7단 | 3초 뒤 "모두 제자리" | 6개가 무너짐 |

- 처음에는 검토 안을 세션 레이어에 얹었음. 단 수와 어긋남을 바꿔 가며 57번을 시험했는데 한 번도 무너지지 않았음.
- 결과가 믿기지 않아 사진을 찍어 보니 상자가 공중에 걸친 채 그대로였음. 세션 레이어에 든 강체에는 물리가 돌지 않았음.
- 2 m 높이에 상자를 띄워 확인했음. 루트 레이어의 것은 바닥(0.15 m)까지 떨어졌고 세션 레이어의 것은 2.0 m 에 그대로 있었음.

## 7. 해 볼 것

### 직접 바꿔 보기

- `--at` 을 바꿔 가며 괜찮은 자리를 하나 더 찾기.
- `--clear 0.5` 로 기준을 올리고 C 가 여전히 괜찮은지 보기.
- `twin.py` 의 `BOX` 를 (0.30, 0.30, 0.30) 으로 바꿔 몇 단까지 버티는지 보기.
- 강의 44 의 `pl2m_nav.py --log` 로 자기 주행을 기록해 `paths` 에 더하기.

## 8. 정리

### 기억할 것

- 디지털트윈은 옮기기 전에 물어보는 곳임. 무엇을 결정할지를 먼저 정함.
- 로봇이 실제로 다닌 길이 가장 좋은 재료임. 사람 눈에 빈 자리가 로봇의 길일 수 있음.
- 판정의 기준은 숫자로 정해 둠. 로봇의 폭에 여유를 더한 거리임.
- 검토 안은 따로 둔 레이어에 적음. 원본은 그대로임.
- 결과가 너무 좋으면 의심함. 사진으로 확인함.

### 한계

- 길은 로봇이 지금까지 다닌 곳임. 새 자리가 생기면 Nav2 가 다른 길을 낼 수 있음. 상자를 놓은 채 다시 주행해 보지는 않았음.
- 거리는 길의 가운데 선과 물체의 네모 사이로 쟀음. 로봇의 모양과 방향은 폭의 절반 하나로 어림했음.
- 상자는 마찰 · 무게가 모두 같은 단단한 육면체임. 실제 상자의 찌그러짐이나 속의 물건은 들어 있지 않음.
- 낙하 시험은 조합마다 세 번임.
- 측정 · 표시 같은 화면 도구와, 버튼이 달린 창(강의 51)은 만들지 않았음.
