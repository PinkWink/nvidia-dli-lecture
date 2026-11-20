# 26. HIL 흉내 내기: 로봇 컴퓨터의 자원 보기와 줄여 보기 — 명령 모음

- 영상에서 입력하는 터미널 명령과 기대 출력을 순서대로 모았음. [25. Isaac Sim 카메라로 Isaac ROS 사람 분할](../25_isaac_ros_sim/README.md) 다음 내용임.

| 파일 | 내용 |
|---|---|
| [computer_monitor.py](computer_monitor.py) | GPU(사용률 · 메모리 · 온도)와 컨테이너의 CPU 사용량 · 허용량을 1초마다 출력하고 `/diagnostics` 로 보내는 ROS 2 노드. 컨테이너 안에서 실행 |

## 2. 준비: 강의 25 의 상태 만들기

### 터미널 1 (컨테이너): 패키지

```bash
isaac-ros activate
sudo apt-get update
sudo apt-get install -y ros-jazzy-isaac-ros-unet ros-jazzy-isaac-ros-peoplesemseg-models-install \
                        ros-jazzy-isaac-ros-examples ros-jazzy-rqt-image-view ros-jazzy-rqt-runtime-monitor
```

- 강의 25 의 패키지에 `ros-jazzy-rqt-runtime-monitor` 하나를 더했음.

### 터미널 2 (컨테이너 밖): Isaac Sim 과 예제 장면

```bash
source /opt/ros/jazzy/setup.bash
cd ~/isaacsim
./isaac-sim.sh
```

- `Window > Examples > Robotics Examples` → `ROS2` → `ISAAC ROS` → `Sample Scene` → `Load Sample Scene` → `Play` (강의 25 의 2 ~ 3절).

### 터미널 1: 영상 분할

```bash
ros2 launch isaac_ros_unet isaac_ros_unet_tensor_rt_isaac_sim.launch.py \
    engine_file_path:=${ISAAC_ROS_WS}/isaac_ros_assets/models/peoplesemsegnet/deployable_quantized_vanilla_unet_onnx_v2.0/1/model.plan \
    input_binding_names:=['input_1:0']
```

### 이 강의의 노드를 작업 폴더에 복사 (컨테이너 밖)

```bash
cp ~/nvidia-dli-lecture/25_isaac_ros_sim/person_watch.py ~/nvidia-dli-lecture/26_hil/computer_monitor.py ~/isaac_ros-dev/
```

## 3. 자원 사용량 보기

### computer_monitor.py 실행 (터미널 3, 컨테이너 안)

```bash
isaac-ros activate
python3 computer_monitor.py
```

```text
[INFO] [computer_monitor]: GPU  99 %   5708 / 16311 MB  66 C  |  CPU  0.55 / 16 코어  [busy]
[INFO] [computer_monitor]: GPU  97 %   5708 / 16311 MB  64 C  |  CPU  0.53 / 16 코어  [busy]
[INFO] [computer_monitor]: GPU  98 %   5708 / 16311 MB  64 C  |  CPU  0.54 / 16 코어  [busy]
```

- **GPU 98 %**: Isaac Sim 과 영상 분할이 GPU 한 장을 함께 씀. 메모리는 둘을 합쳐 5.7 GB 임.
- **CPU 0.54 / 16 코어**: 컨테이너 전체가 쓰는 CPU 가 코어 반 개쯤임. 영상 분할이 GPU 에서 돌기 때문임.
- GPU 사용률이 90 % 를 넘거나 CPU 가 허용량의 90 % 를 넘으면 `[busy]` 로 표시하게 만들었음.

### 값을 읽는 곳

| 값 | 읽는 곳 |
|---|---|
| GPU 사용률 · 메모리 · 온도 | `nvidia-smi --query-gpu=...` |
| 컨테이너가 쓴 CPU 시간 | `/sys/fs/cgroup/cpu.stat` 의 `usage_usec` |
| 허용된 코어 수 | `/sys/fs/cgroup/cpu.max` |

- 따로 설치하는 것 없이 컨테이너에 있는 것만 씀.

### 표준 진단 토픽으로도 나간다

```bash
ros2 topic echo --once /diagnostics
```

```text
status:
- level: "\x01"
  name: computer_monitor
  message: busy
  hardware_id: NVIDIA GeForce RTX 5060 Ti
  values:
  - key: gpu_util_percent
    value: '98.0'
  - key: gpu_memory_used_mb
    value: '5708.0'
  - key: gpu_memory_total_mb
    value: '16311.0'
  - key: gpu_temperature_c
    value: '65.0'
  - key: cpu_cores_used
    value: '1.05'
  - key: cpu_cores_allowed
    value: '16.0'
```

- `diagnostic_msgs/DiagnosticArray` 는 ROS 2 에서 장치 상태를 알리는 표준 메시지임. `level` 의 `\x01` 은 경고(WARN)임.
- 이때 CPU 가 1.05 로 높게 나온 것은, 바로 앞에서 화면 도구(rqt)를 켜는 중이었기 때문임.

### rqt_runtime_monitor 로 보기 (터미널 4, 컨테이너 안)

```bash
isaac-ros activate
ros2 run rqt_runtime_monitor rqt_runtime_monitor --force-discover
```

- 경고가 한 건 있음.
- `--force-discover` 는 새로 설치한 rqt 플러그인을 다시 찾게 함. 강의 PC 에서는 이것 없이 실행하면 `found no plugin matching` 오류로 끝났음.

### Warnings 왼쪽 세모를 눌러 펼치고 항목 클릭

- 오른쪽에 `computer_monitor.py` 가 보낸 값이 표로 나옴. 1초마다 바뀜.

## 4. 처리 속도 재기

### person_watch.py (터미널 5, 컨테이너 안)

```bash
isaac-ros activate
python3 person_watch.py
```

```text
[INFO] [person_watch]: 사람   1.7 %  (가로 545 ~ 606 화소, 영상 960 x 544)  1초 동안 22장
[INFO] [person_watch]: 사람   1.7 %  (가로 546 ~ 607 화소, 영상 960 x 544)  1초 동안 21장
```

- 강의 25 의 노드임. 맨 끝의 "1초 동안 N장"이 처리 속도임. 제한이 없을 때 21 ~ 22장.

## 5. CPU 를 줄여 보기

### 컨테이너의 CPU 를 2 코어로 (컨테이너 밖 터미널)

```bash
docker update --cpus 2 isaac_ros_dev_container
```

```text
isaac_ros_dev_container
```

- 떠 있는 컨테이너에 바로 적용됨. `computer_monitor.py` 의 출력이 `CPU 0.54 / 2 코어` 로 바뀜.

### 1, 0.5, 0.25 코어로 바꿔 가며 보기

```bash
docker update --cpus 1 isaac_ros_dev_container
docker update --cpus 0.5 isaac_ros_dev_container
docker update --cpus 0.25 isaac_ros_dev_container
```

### 0.5 코어일 때

```text
[INFO] [person_watch]: 사람   1.6 %  (가로 548 ~ 607 화소, 영상 960 x 544)  1초 동안 12장
[INFO] [computer_monitor]: GPU  98 %   5708 / 16311 MB  66 C  |  CPU  0.38 / 0.5 코어  [busy]
```

### 0.25 코어일 때

```text
[INFO] [person_watch]: 사람   1.6 %  (가로 545 ~ 606 화소, 영상 960 x 544)  1초 동안 2장
[INFO] [computer_monitor]: GPU  97 %   5724 / 16311 MB  64 C  |  CPU  0.25 / 0.25 코어  [busy]
```

### 결과 (강의 PC, 설정마다 12초씩)

| 허용한 CPU | 1초에 처리한 영상 | 컨테이너가 쓴 CPU |
|---|---|---|
| 16 코어 (제한 없음) | 21 ~ 22장 | 0.53 ~ 0.56 코어 |
| 2 코어 | 21 ~ 22장 | 0.52 ~ 0.55 코어 |
| 1 코어 | 21 ~ 22장 | 0.53 ~ 0.55 코어 |
| 0.5 코어 | 11 ~ 17장 | 0.36 ~ 0.45 코어 |
| 0.25 코어 | 2 ~ 5장 | 0.25 코어 (다 씀) |
| 다시 16 코어 | 21 ~ 22장 | 0.53 ~ 0.54 코어 |

- **1 코어까지는 그대로**임. 영상 분할 전체가 코어 반 개쯤만 쓰기 때문임.
- **0.5 코어부터 느려짐.** 필요한 양(약 0.54 코어)보다 적게 주었기 때문임.
- 이런 식으로 "이 프로그램에는 CPU 가 얼마나 필요한가"를 실물에 올리기 전에 알 수 있음.
- 재는 노드 두 개도 같은 컨테이너 안에서 돌아 같은 제한을 받음. 표의 숫자에는 그 영향이 들어 있음.

### 되돌리기

```bash
docker update --cpus 16 isaac_ros_dev_container
```

- 숫자는 내 PC 의 코어 수로 씀 (`nproc` 으로 확인). 강의 PC 는 16 임.
