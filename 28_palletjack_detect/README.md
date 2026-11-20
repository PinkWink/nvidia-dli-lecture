# 28. 합성 데이터로 YOLO 학습: 실제 창고 사진에서 팔레트잭 찾기 — 명령 모음

- 영상에서 입력하는 터미널 명령과 기대 출력을 순서대로 모았음. [27. 합성 데이터 만들기](../27_palletjack_sdg/README.md) 다음 내용임.

| 파일 |
|---|
| [to_yolo.py](to_yolo.py) |

## 1. 준비: YOLO 를 설치할 가상환경

### 가상환경 만들기 (한 번)

```bash
python3 -m venv ~/yolo_venv
~/yolo_venv/bin/pip install ultralytics
~/yolo_venv/bin/python -c "import torch, ultralytics; print(torch.__version__, torch.cuda.is_available(), ultralytics.__version__)"
```

```text
2.15.0+cu132 True 8.4.192
```

- **가상환경**은 파이썬 패키지를 따로 담아 두는 폴더임. YOLO(ultralytics)는 자기 버전의 PyTorch 를 설치하므로, Isaac Sim 의 파이썬과 섞이지 않게 따로 둠.
- 가운데의 `True` 가 GPU 를 쓸 수 있다는 뜻임.
- 내려받는 양이 수 GB 임. 강의 PC 에서는 2026년 10월에 설치했고(그때 폴더 크기 5.9 GB), 위 출력은 그 환경의 버전임. 지금 설치하면 더 새 버전이 깔림.

### NVIDIA 예제와의 차이

| | NVIDIA 예제 | 이 강의 |
|---|---|---|
| 모델 | DetectNet_v2 | YOLO11n |
| 도구 | TAO Toolkit (Docker) | 파이썬 패키지 `ultralytics` |
| 데이터 | 5,000장 | 600장 |
| 실제 사진 시험 | LOCO 의 팔레트잭 사진 | 같음 |

- "미리 학습된 모델을 내 데이터로 다시 학습 → 점수 확인 → 실제 사진으로 시험"의 흐름은 같음. 도구만 설치가 쉬운 것으로 바꿨음.

## 2. 데이터를 YOLO 형식으로

### 600장 모두

```bash
cd ~/nvidia-dli-lecture
git pull
~/yolo_venv/bin/python 28_palletjack_detect/to_yolo.py work/palletjack_dataset/warehouse work/palletjack_dataset/plain
```

```text
영상 600장 (학습 540, 검증 60), 팔레트잭 상자 1697개, 팔레트잭이 없는 영상 61장
저장: /home/내이름/nvidia-dli-lecture/work/palletjack_yolo/data.yaml
```

- 10 % 인 60장은 **검증용**으로 떼어 둠. 학습에는 쓰지 않고 점수를 매길 때만 씀.
- 강의 27 에서 센 상자는 1,701개였음. 가로나 세로가 8픽셀보다 작은 상자 4개를 버려 1,697개가 되었음.

### 방해물 없는 100장만 (비교용)

```bash
~/yolo_venv/bin/python 28_palletjack_detect/to_yolo.py work/palletjack_dataset/plain --out work/palletjack_yolo_plain
```

```text
영상 100장 (학습 90, 검증 10), 팔레트잭 상자 290개, 팔레트잭이 없는 영상 7장
저장: /home/내이름/nvidia-dli-lecture/work/palletjack_yolo_plain/data.yaml
```

### 무엇이 바뀌나

| | KITTI (강의 27 의 결과) | YOLO |
|---|---|---|
| 한 줄 | `palletjack 0.00 0 0.00 x1 y1 x2 y2 ...` | `0 x중심 y중심 너비 높이` |
| 단위 | 픽셀 | 영상 크기로 나눈 0 ~ 1 |
| 물체 이름 | 글자 | 번호 (0 = palletjack) |

```text
0 0.885938 0.852941 0.105208 0.290441
```

- `pillar`, `wall` 같은 다른 이름의 줄은 버림.

### data.yaml

```bash
cat work/palletjack_yolo/data.yaml
```

```text
path: /home/내이름/nvidia-dli-lecture/work/palletjack_yolo
train: images/train
val: images/val
names:
  0: palletjack
```

- YOLO 에게 데이터가 어디 있고 물체 종류가 무엇인지 알려 주는 파일임.

## 3. 학습

### 600장 모델

```bash
cd ~/nvidia-dli-lecture/work
~/yolo_venv/bin/yolo detect train model=yolo11n.pt data=$PWD/palletjack_yolo/data.yaml epochs=40 imgsz=640 batch=16 project=$PWD/palletjack_runs name=all
```

```text
YOLO11n summary (fused): 101 layers, 2,597,051 parameters, 0 gradients, 6.4 GFLOPs
                 Class     Images  Instances      Box(P          R      mAP50  mAP50-95)
                   all         60        173       0.88      0.844      0.927      0.786
Speed: 0.3ms preprocess, 2.7ms inference, 0.0ms loss, 0.4ms postprocess per image
Results saved to /home/내이름/nvidia-dli-lecture/work/palletjack_runs/all
```

- `yolo11n.pt` 는 일상 물체 80종으로 미리 학습된 가장 작은 YOLO 임. 처음 실행할 때 저절로 받음. 이것을 팔레트잭 한 종류로 다시 학습시킴(미세 조정).
- `epochs=40` 은 540장을 40번 되풀이해 본다는 뜻임.
- `project` 는 `$PWD` 를 붙여 전체 경로로 줌. 그래야 결과가 `work/palletjack_runs` 에 저장됨.
- 강의 PC: 1분 27초. 결과 모델은 `palletjack_runs/all/weights/best.pt` (5.5 MB).

### 100장 모델

```bash
~/yolo_venv/bin/yolo detect train model=yolo11n.pt data=$PWD/palletjack_yolo_plain/data.yaml epochs=40 imgsz=640 batch=16 project=$PWD/palletjack_runs name=plain
```

- 강의 PC: 30초. 결과 모델은 `palletjack_runs/plain/weights/best.pt`.

### 학습 곡선 (600장 모델)

- `palletjack_runs/all/results.png` 임. 위 줄이 학습용, 아래 줄이 검증용의 오차이고, 오른쪽 네 칸이 점수임. 오차는 내려가고 점수는 올라감.

## 4. 점수: 같은 검증 영상으로 두 모델 비교

### 검증 명령

```bash
~/yolo_venv/bin/yolo detect val model=palletjack_runs/plain/weights/best.pt data=$PWD/palletjack_yolo/data.yaml project=$PWD/palletjack_runs name=val_plain
~/yolo_venv/bin/yolo detect val model=palletjack_runs/all/weights/best.pt data=$PWD/palletjack_yolo/data.yaml project=$PWD/palletjack_runs name=val_all
```

```text
                   all         60        173      0.714      0.514      0.602      0.431
```

```text
                   all         60        173      0.881      0.844      0.927      0.787
```

- 두 모델 모두 **600장 쪽의 검증 영상 60장**(팔레트잭 173대)으로 점수를 매김. 방해물이 있는 장면이 섞여 있음.

### 점수 읽기

| 학습 데이터 | P (정밀도) | R (재현율) | mAP50 | mAP50-95 |
|---|---|---|---|---|
| 방해물 없는 100장 | 0.714 | 0.514 | 0.602 | 0.431 |
| 600장 모두 | 0.881 | 0.844 | 0.927 | 0.787 |

- **P (정밀도)**: 팔레트잭이라고 답한 것 중 진짜 팔레트잭인 비율.
- **R (재현율)**: 진짜 팔레트잭 중 찾아낸 비율.
- **mAP50**: 상자가 정답과 절반 이상 겹치면 맞은 것으로 칠 때의 종합 점수. **mAP50-95** 는 더 정확히 겹쳐야 맞은 것으로 침.
- 100장 모델은 팔레트잭의 절반만 찾음(R 0.514). 두 모델은 장수(100 / 600)와 방해물 유무가 함께 다름. 그래서 이 차이가 방해물 때문인지 장수 때문인지는 이 실험만으로 가를 수 없음.

### 검증 영상에 그린 예측 (600장 모델)

- `palletjack_runs/all/val_batch0_pred.jpg` 임. 상자 옆의 숫자는 믿는 정도(0 ~ 1)임.

## 5. 실제 창고 사진으로 시험

### 사진 받기

```bash
git clone --depth 1 https://github.com/NVIDIA-AI-IOT/synthetic_data_generation_training_workflow.git sdg_workflow
ls sdg_workflow/images/loco_palletjacks | wc -l
```

```text
42
```

- NVIDIA 예제 저장소에 실제 물류 창고 사진 42장이 들어 있음. 공개 데이터셋 LOCO 에서 팔레트잭이 찍힌 사진을 고른 것임. 받는 양은 46 MB 임.

### 두 모델로 찾기

```bash
~/yolo_venv/bin/yolo detect predict model=palletjack_runs/plain/weights/best.pt source=sdg_workflow/images/loco_palletjacks conf=0.4 project=$PWD/palletjack_runs name=loco_plain
~/yolo_venv/bin/yolo detect predict model=palletjack_runs/all/weights/best.pt source=sdg_workflow/images/loco_palletjacks conf=0.4 project=$PWD/palletjack_runs name=loco_all
```

- `conf=0.4` 는 믿는 정도가 0.4 이상인 상자만 그린다는 뜻임.
- 상자가 그려진 사진이 `palletjack_runs/loco_plain`, `palletjack_runs/loco_all` 에 저장됨.

### 결과

| 학습 데이터 | 42장 중 상자가 하나라도 나온 사진 | 상자 수 |
|---|---|---|
| 방해물 없는 100장 | 23장 | 38개 |
| 600장 모두 | 37장 | 74개 |

- 이 사진들에는 정답 상자가 없음. 그래서 점수를 낼 수 없고 눈으로 확인함. **상자 수가 많다고 좋은 것이 아님.** 틀린 상자도 세어지기 때문임.

### 잘 찾은 예

- 위 줄이 100장 모델, 아래 줄이 600장 모델임.
- 600장 모델은 세 사진의 팔레트잭을 0.92 · 0.93 · 0.94 로 찾음. 합성 영상만 보고 학습한 모델이 실제 사진에서도 동작함.
- 100장 모델은 첫 사진에서 놓치고, 둘째 사진에서는 지게차의 로고를 팔레트잭이라고 함. 셋째 사진에서는 팔레트잭을 찾았지만(0.63) 팔레트 더미도 팔레트잭이라고 함(0.50).
- 600장 모델도 첫 사진 왼쪽 끝에 틀린 상자(0.56)가 하나 있음.

### 틀린 예 (600장 모델)

- 선반에 쌓인 상자 더미(0.64), 사무용 의자(0.78), 사람의 다리(0.84)를 팔레트잭이라고 함.
- 합성 장면에는 짐이 가득한 선반도, 의자도, 사람도 없었음. 본 적 없는 물체를 가장 비슷한 것으로 답한 것임. 시뮬레이션과 현실의 이런 차이를 **도메인 차이**(domain gap)라 함.

### 더 좋게 하려면

- 데이터를 늘림. NVIDIA 예제는 5,000장임.
- 틀리게 잡은 물체(짐이 쌓인 선반, 의자, 사람)를 방해물로 넣음.
- 바닥과 벽의 재질도 장마다 바꿈.
- 위 세 가지는 이 강의에서 해 보지 않았음. 다시 만들고 → 학습하고 → 실제 사진으로 보는 일을 되풀이하며 확인해야 함.

## 6. 정리

- **미세 조정**: 미리 학습된 YOLO 를 합성 데이터 540장으로 1분 27초 다시 학습했음.
- **합성 검증 점수**: 600장 모델 mAP50 0.927, 방해물 없는 100장 모델 0.602.
- **실제 사진**: 실제 사진을 한 장도 쓰지 않고 학습한 모델이 실제 팔레트잭을 찾음. 동시에 선반 · 의자 · 사람을 팔레트잭으로 잘못 잡기도 함.
- 합성 데이터의 점수가 높아도 실제 사진으로 따로 확인해야 함.

### 다음 강의

- 로봇 학습 도구 **Isaac Lab** 을 설치하고 강화학습을 시작함.
