# 27. 합성 데이터 만들기: Replicator 로 "다른 Pinky Pro" 데이터셋 — 명령 모음

영상에서 입력하는 터미널 명령과 기대 출력을 순서대로 모았습니다. [26. HIL 흉내 내기](../26_hil/README.md) 다음 내용입니다.

| 파일 | 내용 |
|---|---|
| [make_dataset.py](make_dataset.py) | 핑크시티에 Pinky Pro 3대를 놓고, 로봇 카메라 시점의 영상과 상자 정답(KITTI 형식)을 Replicator 로 만든다 |

## 2. 실행

### 준비

- 강의 11 의 준비(`bash 11_pinky_import/prepare_pinky.sh`)가 되어 있어야 합니다. 로봇 모양과 표정 그림을 씁니다.

### 600장 만들기

```bash
cd ~/nvidia-dli-lecture
git pull
~/isaacsim/python.sh 27_pinky_sdg/make_dataset.py
```

```text
핑크시티에서 600장을 만듭니다 → /home/내이름/nvidia-dli-lecture/work/pinky_dataset
  50 / 600
  100 / 600
  ...
  600 / 600
저장: 영상 600장, 상자 1089개
  Pinky Pro 가 0대 보이는 영상: 29장
  Pinky Pro 가 1대 보이는 영상: 179장
  Pinky Pro 가 2대 보이는 영상: 266장
  Pinky Pro 가 3대 보이는 영상: 126장
```

- 강의 PC(RTX 5060 Ti): Isaac Sim 이 켜지는 시간을 포함해 1분 37초, 폴더 크기 148 MB.
- 로봇이 한 대도 없는 영상 29장도 필요합니다. "없을 때는 없다고 답하기"를 배웁니다.

### 적은 장수로 먼저 해 보기

```bash
~/isaacsim/python.sh 27_pinky_sdg/make_dataset.py --count 24 --out work/pinky_dataset_test
```

- `--gui` 를 붙이면 창에서 장면이 바뀌는 것을 볼 수 있습니다 (느려집니다).

## 3. 결과 보기

### 폴더

```bash
ls ~/nvidia-dli-lecture/work/pinky_dataset ~/nvidia-dli-lecture/work/pinky_dataset/RobotCamera
```

```text
/home/내이름/nvidia-dli-lecture/work/pinky_dataset:
RobotCamera  metadata.txt  preview

/home/내이름/nvidia-dli-lecture/work/pinky_dataset/RobotCamera:
depth  instance_segmentation  object_detection  rgb  semantic  semantic_segmentation
```

| 폴더 | 내용 | 이 과정에서 |
|---|---|---|
| `rgb` | 영상 `0.png` ~ `599.png` (640 × 360) | 학습에 씀 |
| `object_detection` | 상자 정답 `0.txt` ~ `599.txt` | 학습에 씀 |
| `depth`, `semantic` 등 | KittiWriter 가 함께 저장하는 거리 · 분할 그림 | 쓰지 않음 |
| `preview` | 상자를 그려 넣은 확인용 그림 24장 (이 강의 코드가 만듦) | 눈으로 확인 |

### 영상 한 장

### 그 영상의 정답

```bash
cat ~/nvidia-dli-lecture/work/pinky_dataset/RobotCamera/object_detection/1.txt
```

```text
pinky 0.00 2 0.00 325 84 342 138 0.00 0.00 0.00 0.00 0.00 0.00 0.00
pinky 0.00 0 0.00 39 76 104 139 0.00 0.00 0.00 0.00 0.00 0.00 0.00
pinky 0.00 0 0.00 211 37 341 174 0.00 0.00 0.00 0.00 0.00 0.00 0.00
```

- 한 줄이 로봇 한 대입니다. 맨 앞이 이름표, 5 ~ 8번째 숫자가 상자의 왼쪽 · 위 · 오른쪽 · 아래(픽셀)입니다.
- 세 번째 숫자는 가려진 정도입니다. 첫 줄의 `2` 는 많이 가려졌다는 뜻입니다.
- 나머지 0.00 은 3차원 크기와 위치 칸으로, 여기서는 쓰지 않습니다. 자율주행 데이터셋 KITTI 의 형식을 그대로 따른 것입니다.

### 상자를 그려 본 그림

- 앞 로봇 뒤에 가려진 로봇은 보이는 부분만 좁은 상자로 나옵니다 (첫 줄).

### 아홉 장

- 장마다 다른 것: 카메라의 자리와 방향, 로봇의 자리와 방향, 해의 방향과 밝기(그림자), LED 색, 화면 표정.
- 로봇이 아주 가까워 화면을 채우는 장, 멀어서 작은 장, 건물에 가려 조금만 보이는 장이 섞여 있습니다.
