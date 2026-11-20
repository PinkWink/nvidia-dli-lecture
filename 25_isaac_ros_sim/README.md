# 25. Isaac Sim 카메라로 Isaac ROS 사람 분할: SIL — 명령 모음

- 영상에서 입력하는 터미널 명령과 기대 출력을 순서대로 모았음. [24. Isaac ROS 준비](../24_isaac_ros/README.md) 다음 내용임.

| 파일 | 내용 |
|---|---|
| [person_watch.py](person_watch.py) | 분할 결과(`/unet/raw_segmentation_mask`)를 읽어 사람 화소의 비율 · 가로 범위 · 1초당 처리 수를 출력하는 ROS 2 노드. 컨테이너 안에서 실행 |

## 1. 준비

### 컨테이너 (터미널 1)

```bash
isaac-ros activate
```

- 강의 24 뒤에 컨테이너를 닫았다면 패키지를 다시 설치함 (강의 24 의 6절). `model.plan` 은 남아 있음.

```bash
sudo apt-get update
sudo apt-get install -y ros-jazzy-isaac-ros-unet ros-jazzy-isaac-ros-peoplesemseg-models-install \
                        ros-jazzy-isaac-ros-examples ros-jazzy-rqt-image-view
```

### Isaac Sim (터미널 2, 컨테이너 밖)

```bash
source /opt/ros/jazzy/setup.bash
cd ~/isaacsim
./isaac-sim.sh
```

- 강의 7 과 같은 방법임. ROS 2 를 먼저 `source` 하고 켜야 Isaac Sim 이 토픽을 내보냄.

## 2. 예제 장면 열기

### Window > Examples > Robotics Examples

### 아래 창에서 ROS2 왼쪽의 + 를 눌러 펼치기

### ISAAC ROS

### Sample Scene

### Load Sample Scene

- 처음에는 창고 자료를 인터넷에서 받느라 오래 걸림. 아래 상태 줄의 진행이 멈출 때까지 기다림.

### 창고, 지게차, 작업자 두 명, 로봇

- Stage 에 `Worker`, `Worker_2` (사람)와 `Nova_Carter_ROS` (로봇)가 있음.
- 이 로봇에는 카메라 · 라이다 · IMU 를 ROS 2 로 내보내는 그래프가 이미 들어 있음. 강의 15 에서 Pinky Pro 에 손으로 만든 것과 같은 종류임.

## 3. Play 와 토픽 확인

### Play 전 (터미널 1, 컨테이너 안)

```bash
ros2 topic list
```

```text
/parameter_events
/rosout
```

### Play

### Play 후

```bash
ros2 topic list
```

```text
/back_stereo_imu/imu
/chassis/imu
/chassis/odom
/clock
/cmd_vel
/front_3d_lidar/lidar_points
/front_stereo_camera/left/camera_info
/front_stereo_camera/left/image_rect_color
/front_stereo_camera/right/camera_info
/front_stereo_camera/right/image_rect_color
/front_stereo_imu/imu
/left_stereo_imu/imu
/parameter_events
/right_stereo_imu/imu
/rosout
/tf
```

- 컨테이너 안에서 Isaac Sim 의 토픽이 보임. 컨테이너가 내 PC 의 네트워크를 그대로 쓰기 때문임.
- 영상 분할에 넣을 토픽은 `/front_stereo_camera/left/image_rect_color` 임. 앞 카메라의 왼쪽 영상이고 크기는 1920 × 1200 임.

## 4. 영상 분할 실행 (터미널 1)

```bash
ros2 launch isaac_ros_unet isaac_ros_unet_tensor_rt_isaac_sim.launch.py \
    engine_file_path:=${ISAAC_ROS_WS}/isaac_ros_assets/models/peoplesemsegnet/deployable_quantized_vanilla_unet_onnx_v2.0/1/model.plan \
    input_binding_names:=['input_1:0']
```

- 강의 24 와 같은 `model.plan` 임. 이 launch 파일은 위의 카메라 토픽을 받도록 정해져 있음.

### 떠 있는 노드 (새 터미널에서 isaac-ros activate 뒤)

```bash
ros2 node list
```

```text
/dnn_image_encoder/crop_node
/dnn_image_encoder/image_format_converter_node
/dnn_image_encoder/image_to_tensor
/dnn_image_encoder/interleaved_to_planar_node
/dnn_image_encoder/normalize_node
/dnn_image_encoder/resize_node
/launch_ros_2714
/unet_container
/unet_decoder_node
/unet_inference
```

- 강의 24 의 그림 그대로 인코더, 추론, 디코더임.

## 5. 결과 보기 (터미널 3, 컨테이너 안)

```bash
isaac-ros activate
ros2 run rqt_image_view rqt_image_view /front_stereo_camera/left/image_rect_color
```

### 로봇 카메라 영상

- 두 작업자가 겹쳐 서 있음.

### 왼쪽 위 칸을 /unet/colored_segmentation_mask 로

- 빨강이 사람임. 두 사람이 겹쳐 한 덩어리로 나옴. 지게차와 상자는 칠해지지 않았음.

## 6. 결과를 읽는 노드 돌리기

### 노드를 작업 폴더에 복사 (컨테이너 밖)

```bash
cp ~/nvidia-dli-lecture/25_isaac_ros_sim/person_watch.py ~/isaac_ros-dev/
```

### 실행 (컨테이너 안)

```bash
python3 person_watch.py
```

```text
[INFO] [person_watch]: 사람   1.6 %  (가로 547 ~ 607 화소, 영상 960 x 544)  1초 동안 20장
[INFO] [person_watch]: 사람   1.6 %  (가로 547 ~ 607 화소, 영상 960 x 544)  1초 동안 20장
[INFO] [person_watch]: 사람   1.6 %  (가로 545 ~ 607 화소, 영상 960 x 544)  1초 동안 21장
```

- `/unet/raw_segmentation_mask` 를 읽음. 화소마다 종류 번호(0 = 그 밖, 1 = 사람)가 든 영상임.
- 사람이 화면의 1.6 % 이고, 가로 547 ~ 607 화소 사이에 있음. 위 그림의 빨간 덩어리 자리임.
- **1초에 20장**이 처리됨 (강의 PC, RTX 5060 Ti. Isaac Sim 과 같은 GPU 를 나눠 씀).
- 로봇이라면 이 숫자로 판단함. 예: 사람이 화면 가운데에 일정 비율 이상이면 멈춤.

### person_watch.py 의 핵심

```python
PERSON = 1          # PeopleSemSegNet 의 사람 종류 번호

mask = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)[:, :msg.width]
person = mask == PERSON
ratio = 100.0 * person.mean()
columns = np.flatnonzero(person.any(axis=0))
```

- 강의 21 의 `lane_follow.py` 와 같은 방식으로, 영상 메시지를 numpy 배열로 바꿔 셈.

## 7. 사람을 옮겨 보기

### Stage 에서 Worker 클릭, 뷰포트에서 W (이동 도구)

### 초록(Y) 화살표를 끌어 옆으로 약 1.2 m

- Play 중에 옮김.
- 이 그림은 강의 PC 에서 끌지 않고 위치 값을 1.2 m 바꿔 찍었음. 결과는 같음.

### 카메라 영상: 두 사람이 떨어짐

### 분할 결과: 두 덩어리

### person_watch 의 출력도 바뀜

```text
[INFO] [person_watch]: 사람   1.7 %  (가로 426 ~ 608 화소, 영상 960 x 544)  1초 동안 18장
```

| | 옮기기 전 | 옮긴 뒤 |
|---|---|---|
| 사람 비율 | 1.6 % | 1.7 % |
| 가로 범위 | 547 ~ 607 화소 | 426 ~ 608 화소 |

- 가로 범위가 왼쪽으로 넓어졌음. 옮긴 사람이 426 쪽에 있음.

### Stop 을 누르면

- 카메라 토픽이 더 나오지 않아 분할도 멈춤. 강의 PC 에서 Stop 뒤에는 분할 결과가 한 장도 오지 않았음. 다시 Play 하면 이어짐.
