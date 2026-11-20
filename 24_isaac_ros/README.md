# 24. Isaac ROS 준비: 컨테이너에서 사람 분할 돌리기 — 명령 모음

- 영상에서 입력하는 터미널 명령과 기대 출력을 순서대로 모았음. [23. SIL · HIL 과 Script Editor](../23_script_editor/README.md) 다음 내용임.

| 파일 | 내용 |
|---|---|
| [download_assets.sh](download_assets.sh) | isaac_ros_unet 예제 자료(녹화 영상 `quickstart.bag`)를 받음. 컨테이너 안에서 실행 |

- 준비물: 디스크 여유 80 GB 이상 (Isaac ROS 이미지: 받는 양 24.2 GB, 디스크 63.7 GB), `sudo` 권한, NVIDIA GPU.

- 2 · 3절의 설치 명령은 강의 PC 에 2026년 10월 25일에 실행한 것이고, 4절부터는 이 강의를 만들면서 새 작업 폴더로 다시 실행해 확인했음 (Docker 29.9.3, NVIDIA Container Toolkit 1.20.1, Isaac ROS CLI 1.0.2, Isaac ROS 4.0).

## 2. Docker 와 NVIDIA Container Toolkit 설치


### Docker 설치

```bash
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu $(. /etc/os-release && echo "$VERSION_CODENAME") stable" \
    | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update
sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
sudo usermod -aG docker $USER
```

- 마지막 줄은 `sudo` 없이 `docker` 를 쓰도록 내 계정을 `docker` 그룹에 넣음. 재부팅해야 적용됨.

### NVIDIA Container Toolkit 설치

```bash
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --batch --yes --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list \
    | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' \
    | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

- 컨테이너 안에서 GPU 를 쓰게 해 주는 도구임.

### 재부팅 뒤 확인

```bash
sudo reboot
```

```bash
docker run --rm --gpus all ubuntu nvidia-smi
```

- GPU 이름이 든 표가 나오면 통과임.
- 강의 PC 에서는 로그아웃 · 로그인만으로는 `docker` 그룹이 적용되지 않아 재부팅했음.

## 3. Isaac ROS CLI 설치

### 작업 폴더와 환경 변수

```bash
mkdir -p ~/isaac_ros-dev/src
echo '[ -f /.dockerenv ] || export ISAAC_ROS_WS="${HOME}/isaac_ros-dev/"' >> ~/.bashrc
source ~/.bashrc
```

- `ISAAC_ROS_WS` 는 Isaac ROS 가 작업 폴더를 찾는 이름임.
- `[ -f /.dockerenv ] ||` 는 "컨테이너 안에서는 이 줄을 건너뛴다"는 뜻임. 컨테이너가 내 `~/.bashrc` 를 그대로 읽기 때문에, 이것이 없으면 컨테이너 안에서 없는 경로를 가리키게 됨.

### CLI 설치

```bash
k="/usr/share/keyrings/nvidia-isaac-ros.gpg"
curl -fsSL https://isaac.download.nvidia.com/isaac-ros/repos.key | sudo gpg --batch --yes --dearmor -o $k
f="/etc/apt/sources.list.d/nvidia-isaac-ros.list"
s="deb [signed-by=$k] https://isaac.download.nvidia.com/isaac-ros/release-4.0 noble main"
sudo touch $f && (grep -qxF "$s" $f || echo "$s" | sudo tee -a $f)
sudo apt-get update
pip install termcolor --break-system-packages
sudo apt-get install -y isaac-ros-cli
sudo isaac-ros init docker
```

## 4. 컨테이너로 들어가기

### isaac-ros activate

```bash
isaac-ros activate
```

```text
Using image: nvcr.io/nvidia/isaac/ros:noble-ros2_jazzy_...-amd64
Running isaac_ros_dev_container
Creating non-root container 'admin' for host user uid=1000:gid=984
cp: cannot create regular file '/home/admin/.bashrc': Read-only file system
udev: unrecognized service
admin@내PC:/workspaces/isaac_ros-dev$
```

- 처음 실행하면 이미지(24.2 GB)를 받음. 인터넷 속도에 따라 오래 걸림.
- 프롬프트가 `admin@...:/workspaces/isaac_ros-dev$` 로 바뀌면 **컨테이너 안**임.
- `cp: cannot create ...`, `udev: unrecognized service` 줄은 강의 PC 에서도 나오며 무시함.
- 이 터미널을 닫으면 컨테이너가 지워짐. 열어 둠.

### 터미널을 더 열 때

- 새 터미널에서 `isaac-ros activate` 를 또 실행하면, 떠 있는 같은 컨테이너에 붙음.

## 5. 예제 자료 받기 (컨테이너 밖 1번, 안 1번)

### 내려받기 스크립트를 작업 폴더에 복사 (새 터미널, 컨테이너 밖)

```bash
cp ~/nvidia-dli-lecture/24_isaac_ros/download_assets.sh ~/isaac_ros-dev/
```

### 컨테이너 안에서 실행

```bash
bash download_assets.sh
```

```text
자료 버전: 4.0.0
받은 것:
quickstart.bag
quickstart_interface_specs.json
zed2_quickstart_interface_specs.json
zedx_quickstart_interface_specs.json
```

- `quickstart.bag` 은 거리를 걷는 사람들을 찍은 녹화 영상(rosbag)임. 3.5 MB 라 몇 초면 받음.
- 받은 자료는 `~/isaac_ros-dev/isaac_ros_assets` 에 있어 컨테이너를 닫아도 남음.

## 6. 패키지 설치 (컨테이너 안, 들어올 때마다)

```bash
sudo apt-get update
sudo apt-get install -y ros-jazzy-isaac-ros-unet ros-jazzy-isaac-ros-peoplesemseg-models-install \
                        ros-jazzy-isaac-ros-examples ros-jazzy-rqt-image-view
```

```text
Setting up ros-jazzy-isaac-ros-dnn-image-encoder (4.0.0-0noble...) ...
Setting up ros-jazzy-isaac-ros-unet (4.0.0-0noble...) ...
Setting up ros-jazzy-isaac-ros-peoplesemseg-models-install (4.0.0-0noble...) ...
```

- 컨테이너 안에서는 비밀번호 없이 `sudo` 가 됨. 강의 PC 에서 46초.
- 컨테이너를 닫으면 설치한 패키지도 사라짐. 다시 들어오면 이 단계를 다시 함.

## 7. 모델 받기와 TensorRT 엔진 만들기 (컨테이너 안, 처음 한 번)

```bash
ros2 run isaac_ros_peoplesemseg_models_install install_peoplesemsegnet_vanilla.sh --eula
```

```text
Do you accept? [y/n]
...
&&&& PASSED TensorRT.trtexec [TensorRT v101302] ... --saveEngine=.../deployable_quantized_vanilla_unet_onnx_v2.0/1/model.plan --fp16 --skipInference
```

- `Do you accept? [y/n]` 에 `y` 를 입력함 (모델 사용 약관 동의).
- 모델(ONNX)을 받은 뒤, 이 PC 의 GPU 에 맞는 실행 파일 `model.plan` (61 MB) 을 만듦. `&&&& PASSED` 가 나오면 성공임.
- 강의 PC(RTX 5060 Ti 16 GB): 60초.
- `model.plan` 도 `~/isaac_ros-dev` 에 남으므로 한 번만 하면 됨.

## 8. 녹화 영상으로 사람 분할

- 터미널 세 개를 씀. 모두 컨테이너 안임.

### 터미널 1: 영상 분할 노드 실행

```bash
ros2 launch isaac_ros_examples isaac_ros_examples.launch.py launch_fragments:=unet \
    interface_specs_file:=${ISAAC_ROS_WS}/isaac_ros_assets/isaac_ros_unet/quickstart_interface_specs.json \
    engine_file_path:=${ISAAC_ROS_WS}/isaac_ros_assets/models/peoplesemsegnet/deployable_quantized_vanilla_unet_onnx_v2.0/1/model.plan \
    input_binding_names:=['input_1:0']
```

```text
[component_container_mt-1] [INFO] [unet_encoder.resize_node]: [NitrosNode] Node was started
[component_container_mt-1] [INFO] [unet_encoder.image_format_converter_node]: [NitrosNode] Node was started
[component_container_mt-1] [INFO] [unet_decoder]: [NitrosNode] Node was started
[component_container_mt-1] [INFO] [unet_inference]: [NitrosNode] Node was started
```

- `Node was started` 가 여섯 번 나오면 준비된 것임. 아직 들어오는 영상이 없어 아무 일도 하지 않음.
- `interface_specs_file` 은 들어올 영상의 크기(1200 × 632)를 적은 파일이고, `engine_file_path` 는 7절에서 만든 `model.plan` 임.

### 터미널 2: 녹화 영상 재생

```bash
isaac-ros activate
ros2 bag play -l ${ISAAC_ROS_WS}/isaac_ros_assets/isaac_ros_unet/quickstart.bag
```

```text
[INFO] [rosbag2_player]: Set rate to 1
[INFO] [rosbag2_player]: Press SPACE for Pause/Resume
```

- `-l` 은 끝나면 처음부터 다시 재생한다는 뜻임. 이 녹화에는 영상이 3장뿐이라 같은 장면이 되풀이됨.

### 터미널 3: 결과 보기

```bash
isaac-ros activate
ros2 run rqt_image_view rqt_image_view /unet/colored_segmentation_mask
```

- 창 왼쪽 위 칸에서 토픽을 바꿔 가며 봄. 아래 세 그림은 강의 PC 에서 각 토픽의 영상을 한 장씩 저장한 것임.

- 입력: `/image_rect`
- 분할 결과: `/unet/colored_segmentation_mask`
- 겹쳐 보기: `/segmentation_image_overlay`

### 처리 속도 (아무 컨테이너 터미널)

```bash
ros2 topic hz /unet/colored_segmentation_mask
```

```text
average rate: 7.090
	min: 0.087s max: 0.285s std dev: 0.05520s window: 42
```

- 강의 PC 에서 1초에 약 7장이 나왔음. 이 값은 녹화 영상이 재생되는 속도(영상 3장을 되풀이)에 묶여 있어, 모델이 낼 수 있는 가장 빠른 속도는 아님.

### 토픽으로 흐름 확인

```bash
ros2 topic list | grep -E "image_rect|unet|overlay" | grep -v nitros
```

```text
/image_rect
/segmentation_image_overlay
/unet/colored_segmentation_mask
/unet/raw_segmentation_mask
/unet_encoder/converted/image
/unet_encoder/crop/camera_info
/unet_encoder/crop/image
/unet_encoder/image_tensor
/unet_encoder/normalized_tensor
/unet_encoder/resize/camera_info
/unet_encoder/resize/image
```

- 1절 그림의 순서 그대로임: `/image_rect` → 인코더(`/unet_encoder/...`) → 추론 → 디코더(`/unet/...`).

### 끝내기

- 터미널 3, 2, 1 순서로 `Ctrl` + `C`. 터미널 1 의 컨테이너는 다음 강의에서 씀.
