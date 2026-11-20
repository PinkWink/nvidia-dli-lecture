# 27. 합성 데이터 만들기: Replicator 로 창고의 팔레트잭 데이터셋 — 명령 모음

영상에서 입력하는 터미널 명령과 기대 출력을 순서대로 모았습니다. [26. HIL 흉내 내기](../26_hil/README.md) 다음 내용입니다.

| 파일 |
|---|
| [make_dataset.py](make_dataset.py) |

## 1. 합성 데이터 (SDG)

### 왜 만드나

- 물체 탐지 모델을 학습하려면 "영상 + 물체를 둘러싼 상자"가 수백 ~ 수천 장 필요합니다. 실제 창고에서 사진을 찍고 사람이 상자를 그리는 일은 오래 걸립니다.
- 시뮬레이터는 어느 픽셀이 어느 물체인지 이미 압니다. 그래서 영상과 정답을 함께 내놓을 수 있습니다. 이것이 합성 데이터 생성(Synthetic Data Generation)입니다.
- 어두운 조명이나 물건에 가려진 장면처럼, 실제로는 모으기 어려운 상황도 원하는 만큼 만들 수 있습니다.

### 네 단계

### 도메인 랜덤화

- 장마다 조건을 바꾸는 것을 **도메인 랜덤화**(domain randomization)라 합니다.
- 조명과 자리가 늘 같으면 모델은 "그 조명의 그 자리"를 외웁니다. 조건을 계속 바꾸면 팔레트잭의 모양 자체를 배웁니다.
- 이 강의에서 바꾸는 것: 카메라 자리, 팔레트잭의 자리 · 방향 · 색, 방해물, 천장 조명.

### 강의 19 와의 차이

| | 강의 19 (도로 마스크) | 강의 27 (팔레트잭 상자) |
|---|---|---|
| 정답 | 픽셀마다 도로 종류 | 팔레트잭마다 상자 하나 |
| 정답을 만든 방법 | 거리 영상과 바닥 지도로 직접 계산 | Replicator 가 이름표를 보고 만듦 |
| 장마다 바꾼 것 | 카메라 자리, 해 | 카메라, 팔레트잭, 방해물, 조명 |
| 저장 | 직접 PNG 로 | Replicator 의 Writer (KITTI 형식) |

### NVIDIA 예제와의 관계

- 이 강의의 구성은 NVIDIA 가 공개한 예제 [synthetic_data_generation_training_workflow](https://github.com/NVIDIA-AI-IOT/synthetic_data_generation_training_workflow) 를 따릅니다. 창고, 팔레트잭 3종, 방해물, KITTI 형식이 같습니다.
- 그 예제의 코드는 Isaac Sim 4.x 의 이름을 씁니다. 이 강의의 `make_dataset.py` 는 같은 구성을 Isaac Sim 5.1 의 이름으로 짧게 다시 쓴 것입니다.
- 예제는 5,000장을 만듭니다. 이 강의는 600장으로 줄였습니다.

## 2. 실행

### 방해물이 있는 창고 500장

```bash
cd ~/nvidia-dli-lecture
git pull
~/isaacsim/python.sh 27_palletjack_sdg/make_dataset.py
```

```text
창고에서 500장을 만듭니다 → /home/내이름/nvidia-dli-lecture/work/palletjack_dataset/warehouse
저장: 영상 500장, 팔레트잭 상자 1411개, 53초
  팔레트잭이 0대 보이는 영상: 54장
  팔레트잭이 1대 보이는 영상: 82장
  팔레트잭이 2대 보이는 영상: 80장
  팔레트잭이 3대 보이는 영상: 94장
  팔레트잭이 4대 보이는 영상: 98장
  팔레트잭이 5대 보이는 영상: 57장
  팔레트잭이 6대 보이는 영상: 35장
```

- 강의 PC(RTX 5060 Ti): Isaac Sim 이 켜지는 시간을 포함해 1분, 폴더 크기 342 MB.
- 창고와 팔레트잭 모델은 NVIDIA 서버에서 받아 옵니다. 처음 실행할 때는 내려받는 시간이 더 걸립니다. 강의 PC 는 이미 받아 둔 상태에서 잰 시간입니다.
- 팔레트잭이 한 대도 없는 영상 54장도 필요합니다. "없을 때는 없다고 답하기"를 배웁니다.

### 방해물이 없는 창고 100장

```bash
~/isaacsim/python.sh 27_palletjack_sdg/make_dataset.py --count 100 --plain --seed 1 --out work/palletjack_dataset/plain
```

```text
창고에서 100장을 만듭니다 → /home/내이름/nvidia-dli-lecture/work/palletjack_dataset/plain
저장: 영상 100장, 팔레트잭 상자 290개, 26초
```

- `--plain` 은 방해물을 놓지 않습니다. `--seed 1` 은 무작위 씨앗을 바꿔 500장과 다른 장면이 나오게 합니다.
- 다음 강의에서 이 100장만으로 학습한 모델과 600장 모두로 학습한 모델을 비교합니다.

### 창을 띄워 보기

```bash
~/isaacsim/python.sh 27_palletjack_sdg/make_dataset.py --count 120 --gui --out work/palletjack_dataset/test
```

- 화면이 빠르게 바뀝니다. 장마다 카메라 · 팔레트잭 · 방해물 · 조명을 새로 뽑는 모습입니다. 다 만들면 창이 저절로 닫힙니다.
- 흰색 · 파란색 수레가 팔레트잭이고, 통 · 상자 · 고깔 · 표지판이 방해물입니다.
- 확인용이므로 끝나면 `work/palletjack_dataset/test` 폴더는 지워도 됩니다.

## 3. 결과 보기

### 폴더

```bash
ls work/palletjack_dataset/warehouse work/palletjack_dataset/warehouse/Camera
```

```text
work/palletjack_dataset/warehouse:
Camera  metadata.txt

work/palletjack_dataset/warehouse/Camera:
depth  instance_segmentation  object_detection  rgb  semantic  semantic_segmentation
```

| 폴더 | 내용 |
|---|---|
| `rgb/N.png` | 학습에 쓸 영상 (960 × 544) |
| `object_detection/N.txt` | 상자 정답. 영상 한 장에 파일 하나 |
| 나머지 | KittiWriter 가 함께 저장하는 것. 이 강의에서는 쓰지 않습니다 |

### 정답 파일 한 개

```bash
grep palletjack work/palletjack_dataset/warehouse/Camera/object_detection/3.txt
```

```text
palletjack 0.00 0 0.00 902 110 959 238 0.00 0.00 0.00 0.00 0.00 0.00 0.00
palletjack 0.00 0 0.00 794 489 905 543 0.00 0.00 0.00 0.00 0.00 0.00 0.00
```

- KITTI 형식입니다. 맨 앞이 이름, 다섯 번째부터 네 숫자가 상자의 왼쪽 위 (x1, y1) 와 오른쪽 아래 (x2, y2) 픽셀입니다. 나머지는 이 강의에서 쓰지 않습니다.
- `grep` 을 빼고 파일을 열면 `pillar`, `wall`, `floor` 같은 줄도 있습니다. 창고 모델에 원래 붙어 있던 이름표입니다. 다음 강의에서 `palletjack` 만 남깁니다.

### 상자를 그려 본 그림

- 위 파일의 상자 두 개를 3번 영상에 그린 것입니다.

### 여덟 장

- 밝은 장면, 어두운 장면, 색이 낀 조명이 섞여 있습니다. 팔레트잭의 색도 장마다 다릅니다.
- 일부만 보이는 팔레트잭에도 상자가 있습니다.

## 4. 코드 읽기

`27_palletjack_sdg/make_dataset.py` 의 중요한 부분입니다.

### ① 장면과 이름표

```python
omni.usd.get_context().open_stage(WAREHOUSE)

jacks = rep.create.group([rep.create.from_usd(path, semantics=[("class", "palletjack")], count=2) for path in PALLETJACKS])
props = None if args.plain else rep.create.group([rep.create.from_usd(path, count=2) for path in DISTRACTORS])
camera = rep.create.camera(clipping_range=(0.1, 1000000))
```

- 창고를 열고, 팔레트잭 3종을 2대씩 놓습니다. `semantics=[("class", "palletjack")]` 이 **이름표**입니다. 정답 파일의 `palletjack` 이 여기서 옵니다.
- 방해물 10종은 이름표 없이 2개씩 놓습니다. 모델은 이것들을 "팔레트잭이 아닌 것"으로 배웁니다.

### ② 장마다 바꿀 것

```python
with rep.trigger.on_frame(num_frames=args.count):
    with camera:
        rep.modify.pose(position=rep.distribution.uniform((-9.0, -11.0, 0.5), (7.0, 15.0, 4.0)), look_at=(0, 0, 0))
    with jacks:
        rep.modify.pose(position=rep.distribution.uniform(AREA_LOW, AREA_HIGH),
                        rotation=rep.distribution.uniform((0, 0, 0), (0, 0, 360)), scale=0.01)
    with rep.get.prims(path_pattern="SteerAxles"):
        rep.randomizer.color(colors=rep.distribution.uniform((0, 0, 0), (1, 1, 1)))
    with rep.get.prims(path_pattern="RectLight"):
        rep.modify.attribute("color", rep.distribution.uniform((0.3, 0.3, 0.3), (1, 1, 1)))
        rep.modify.attribute("intensity", rep.distribution.uniform(3000.0, 25000.0))
        rep.modify.visibility(rep.distribution.choice([True, True, False]))
```

- `rep.trigger.on_frame` 안에 적은 것은 **장마다** 새로 뽑힙니다.
- `rep.distribution.uniform(낮은 값, 높은 값)` 은 그 사이에서 고르게 뽑습니다.
- 카메라는 창고 안 아무 곳에나 놓이고 늘 가운데 `(0, 0, 0)` 를 봅니다. 높이는 0.5 ~ 4 m 입니다.
- 팔레트잭 모델은 cm 단위로 만들어져 있어 `scale=0.01` 로 줄입니다.
- 천장 조명은 색과 밝기를 바꾸고, 셋 중 하나꼴로 끕니다.

### ③ 저장과 실행

```python
writer = rep.WriterRegistry.get("KittiWriter")
writer.initialize(output_dir=str(OUT), omit_semantic_type=True)
writer.attach(rep.create.render_product(camera, (args.width, args.height)))
rep.orchestrator.run_until_complete(num_frames=args.count)
```

- `KittiWriter` 가 영상과 상자 정답을 저장합니다. `render_product` 는 "이 카메라를 이 크기로 그린 결과"입니다.
- `run_until_complete` 가 정한 장수만큼 "바꾸기 → 그리기 → 저장"을 되풀이합니다.

## 5. 정리

- **합성 데이터**: 시뮬레이터가 영상과 정답을 함께 만듭니다. 사람이 상자를 그리지 않습니다.
- **Replicator**: 이름표를 붙이고, 장마다 바꿀 것을 정하고, Writer 를 붙여 실행합니다.
- **도메인 랜덤화**: 카메라 · 물체 · 방해물 · 조명을 장마다 바꿔, 모델이 한 가지 조건만 외우지 않게 합니다.
- 만든 것: 방해물이 있는 500장(상자 1,411개)과 없는 100장(상자 290개).

### 한계

- 600장은 적은 양입니다. NVIDIA 예제는 5,000장을 만들고, 바닥과 벽의 재질도 바꿉니다. 이 강의는 재질을 바꾸지 않습니다.
- 조명 밝기의 범위(3,000 ~ 25,000)는 결과 그림을 보며 정한 값입니다. 너무 높이면 영상이 하얗게 날아갑니다.

### 다음 강의

- 이 데이터로 YOLO 를 학습하고, 실제 창고 사진으로 시험합니다.
