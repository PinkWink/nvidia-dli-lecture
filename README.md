# nvidia-dli-lecture

NVIDIA Isaac Sim 온라인 강의(YouTube)의 예제 코드입니다. 강의가 진행되는 대로 폴더가 추가됩니다.

## 기준 환경

| 항목 | 버전 |
|---|---|
| 운영체제 | Ubuntu 24.04 |
| Isaac Sim | 5.1.0 (`~/isaacsim` 에 설치) |
| NVIDIA 드라이버 | 580 이상 |
| GPU | RTX 계열, VRAM 16 GB 권장 |
| ROS 2 | Jazzy (강의 7 부터) |

모든 코드는 위 환경에서 실제로 실행해 확인했습니다.

## 받기

```bash
cd ~
git clone https://github.com/PinkWink/nvidia-dli-lecture.git
cd ~/nvidia-dli-lecture
```

새 강의의 코드는 `git pull` 로 받습니다.

## 강의별 명령 모음

영상에서 입력하는 터미널 명령은 강의마다 아래 문서에 순서대로 모아 두었습니다. 영상을 보면서 복사해 붙여 넣으세요.

| 강의 | 명령 모음 | 코드 |
|---|---|---|
| 1. Isaac Sim 5.1 설치 | [01_install/README.md](01_install/README.md) | `check_env.sh` PC 점검 · `install_isaacsim.sh` 내려받기와 설치 · `verify_install.sh` 설치 확인 |
| 2. Isaac Sim 실행과 화면 구성 | [02_launch/README.md](02_launch/README.md) | `hello_isaacsim.py` Python 으로 Isaac Sim 켜기 |
| 3. 첫 장면: 큐브를 떨어뜨리며 배우는 물리 기초 | [03_first_scene/README.md](03_first_scene/README.md) | `falling_cube.py` 강체 · 충돌체 · 바닥을 코드로 |
| 4. 로봇 만들기: 몸체 · 바퀴 · 회전 관절 · 구동 | [04_build_robot/README.md](04_build_robot/README.md) | `build_robot.py` 몸체 · 바퀴 · 관절 · 구동을 코드로 |
| 5. 로봇 조종: Articulation Root · Differential Controller · 키보드 | [05_control/README.md](05_control/README.md) | `drive_robot.py` 속도 명령 · 키보드 조종 그래프 |
| 6. 센서 달기: 카메라 · 2D 라이다 | [06_sensors/README.md](06_sensors/README.md) | `add_sensors.py` 카메라 사진 · 라이다 거리 |
| 7. ROS 2 로 데이터 보내기: LaserScan · 카메라 · RViz2 | [07_ros2/README.md](07_ros2/README.md) | `publish_ros2.py` 토픽 내보내기 · `rviz/simple_robot.rviz` |
| 8. URDF 로봇 가져오기: carter.urdf 읽기 · URDF Importer | [08_urdf_import/README.md](08_urdf_import/README.md) | `import_carter.py` URDF 읽기 · 가져오기 |
| 9. 가져온 로봇 몰기: 바닥 · 관절 · 차동 제어기 · 속도 제한 | [09_carter_drive/README.md](09_carter_drive/README.md) | `drive_carter.py` 바퀴 속도 · 속도 제한 · 키보드 |
| 10. 가져온 로봇에 센서 달기 · 준비된 로봇 Nova Carter | [10_carter_sensors/README.md](10_carter_sensors/README.md) | `carter_sensors.py` 카메라 · 라이다 · `nova_carter.py` 센서 목록 |
| 11. 내 로봇 가져오기: Pinky Pro (URDF 만들기 · 가져오기 · 모터) | [11_pinky_import/README.md](11_pinky_import/README.md) | `prepare_pinky.sh` URDF 준비 · `import_pinky.py` 가져오기와 주행 |
| 12. Pinky Pro 센서: 라이다 · 카메라 · 초음파 · IMU | [12_pinky_sensors/README.md](12_pinky_sensors/README.md) | `pinky_sensors.py` 네 센서의 값 |
| 13. Pinky Pro 화면(LCD)과 LED 바 | [13_pinky_lcd_led/README.md](13_pinky_lcd_led/README.md) | `pinky_face_led.py` 표정 · LED 색 |
| 14. Pinky Pro 를 ROS 2 로 움직이기: cmd_vel 구독 · teleop | [14_pinky_ros2/README.md](14_pinky_ros2/README.md) | `pinky_ros2.py` /cmd_vel 받기 · 센서와 위치 내보내기 |
| 15. Pinky Pro 의 센서와 위치를 ROS 2 로 내보내기 · RViz2 | [15_pinky_ros2_topics/README.md](15_pinky_ros2_topics/README.md) | 강의 14 의 `pinky_ros2.py` · `rviz/pinky.rviz` |
| 16. 핑크시티 만들기: Pinky Pro 가 다니는 작은 도시 | [16_pink_city/README.md](16_pink_city/README.md) | `pink_city.py` |
| 17. 핑크시티 지도 만들기: slam_toolbox | [17_pinky_slam/README.md](17_pinky_slam/README.md) | `mapper_params.yaml` · `rviz/slam.rviz` · `drive_city.py` |
| 18. 핑크시티 자율주행: AMCL · Nav2 | [18_pinky_nav2/README.md](18_pinky_nav2/README.md) | `nav2_params.yaml` · `rviz/nav2.rviz` |
| 19. 도로 인식 데이터 모으기: 카메라 영상과 정답 마스크 | [19_road_dataset/README.md](19_road_dataset/README.md) | `collect_dataset.py` |
| 20. 도로를 알아보는 모델 학습: 세그먼테이션 | [20_road_segmentation/README.md](20_road_segmentation/README.md) | `train.py` · `check_onnx.py` |
| 21. 카메라로 차선을 따라 달리기: 세그먼테이션 + 제어기 | [21_lane_follow/README.md](21_lane_follow/README.md) | `lane_follow.py` |
| 22. 장소 이름으로 명령하기: 위치 추정 + 차선 추종 | [22_city_mission/README.md](22_city_mission/README.md) | `city_driver.py` · `rviz/city.rviz` |
| 23. SIL · HIL 과 Script Editor: 코드로 장면 다루기 | [23_script_editor/README.md](23_script_editor/README.md) | `01_usd_api.py` · `02_core_api.py` · `03_drive_pinky.py` Script Editor 에 붙여 넣는 코드 |
| 24. Isaac ROS 준비: 컨테이너에서 사람 분할 돌리기 | [24_isaac_ros/README.md](24_isaac_ros/README.md) | `download_assets.sh` 예제 자료 받기 |
| 25. Isaac Sim 카메라로 Isaac ROS 사람 분할: SIL | [25_isaac_ros_sim/README.md](25_isaac_ros_sim/README.md) | `person_watch.py` 분할 결과를 읽는 노드 |
| 26. HIL 흉내 내기: 로봇 컴퓨터의 자원 보기와 줄여 보기 | [26_hil/README.md](26_hil/README.md) | `computer_monitor.py` GPU · CPU 사용량을 `/diagnostics` 로 |
| 27. 합성 데이터 만들기: Replicator 로 창고의 팔레트잭 데이터셋 | [27_palletjack_sdg/README.md](27_palletjack_sdg/README.md) | `make_dataset.py` 영상 + 상자 정답 600장 |
| 28. 합성 데이터로 YOLO 학습: 실제 창고 사진에서 팔레트잭 찾기 | [28_palletjack_detect/README.md](28_palletjack_detect/README.md) | `to_yolo.py` KITTI → YOLO 형식 |
| 29. Isaac Lab 시작: 강화학습의 생각, 설치, 둘러보기 | [29_isaac_lab/README.md](29_isaac_lab/README.md) | 명령 모음 |
| 30. 첫 강화학습: Cartpole 프로젝트를 만들고 학습하기 | [30_cartpole_train/README.md](30_cartpole_train/README.md) | 명령 모음 |
| 31. Cartpole 들여다보기: 재생, 학습 곡선, 과제 코드 | [31_cartpole_code/README.md](31_cartpole_code/README.md) | 명령 모음 |
| 32. 핑크랩 로터리 역진자: 보상을 직접 설계해 세우기 | [32_pinklab_pendulum/README.md](32_pinklab_pendulum/README.md) | `pendulum_env.py` · `train.py` · `play.py` · `ppo_cfg.py` · `model/` |
| 33. 핑크랩 로터리 역진자 2: 지연 · 마찰 · 센서 잡음을 넣고 다시 학습 | [33_pendulum_real/README.md](33_pendulum_real/README.md) | 명령 모음 (코드는 32 것을 사용) |
| 34. 사족 로봇을 걷게 하기: ANYmal 보행 학습 | [34_quadruped/README.md](34_quadruped/README.md) | 명령 모음 |
| 35. 핑크시티에 Pinky Pro 두 대: 네임스페이스와 동시 자율주행 | [35_two_pinkys/README.md](35_two_pinkys/README.md) | `two_pinkys.py` 두 대 장면 · `two_pinky_nav2.launch.py` Nav2 두 벌 · `start_nav2.py` 준비 확인 후 시작 |
| 36. ROS 2 로 시뮬레이터 조종하기: 자율주행 자동 시험 | [36_auto_test/README.md](36_auto_test/README.md) | `nav_test.py` 자동 반복 시험 · `common/isaac_ros_env.sh` 시뮬레이터 터미널용 환경 |
| 37. PL-2M 가져오기: 이동형 양팔 로봇을 Isaac Sim 으로 | [37_pl2m_import/README.md](37_pl2m_import/README.md) | `prepare_pl2m.sh` URDF 준비 · `import_pl2m.py` 가져오기 · 서 있기 · 팔꿈치 굽히기 |
| 38. 핑크랩 부스 만들기: MuJoCo 장면을 Isaac Sim 으로 옮기기 | [38_pl2m_booth/README.md](38_pl2m_booth/README.md) | `make_booth.py` 부스 옮기기 · 사진 · PL-2M 과 컵 올리기 |
| 39. PL-2M 움직이기: 바퀴 · 팔 · 그리퍼의 구동 값 | [39_pl2m_drive/README.md](39_pl2m_drive/README.md) | `pl2m_drive.py` 구동 값 · 접촉 설정(`tune_pl2m`) · 시험 5가지 · 부스 안 운전 |
| 40. PL-2M 팔의 중력: 영향과 보상 | [40_pl2m_gravity/README.md](40_pl2m_gravity/README.md) | `pl2m_gravity.py` 중력 토크의 수식 · 중력 보상(`GravityCompensation`) · 한쪽 팔만 보상해 비교 · 시험 8가지 |
| 41. PL-2M 을 ROS 2 에 연결하기: 궤적은 ROS 2 에서, 중력 보상은 로봇 안에서 | [41_pl2m_ros2/README.md](41_pl2m_ros2/README.md) | `pl2m_ros2.py` ROS 2 그래프 + 목표 보간기 + 중력 보상 · `trajectory_server.py` 궤적 액션 서버 · `send_pose.py` · `check_ripple.py` |
| 42. MoveIt 으로 팔의 경로 계획하기: 목표만 주면 부딪히지 않는 길을 찾음 | [42_pl2m_moveit/README.md](42_pl2m_moveit/README.md) | `prepare_moveit.sh` · `moveit.launch.py` · `move_arm.py` 관절 · 자세 목표 · `add_obstacle.py` 장애물 · `pl2m_moveit.py` |
| 43. 컵 집기: 그리퍼로 쥐고, 들고, 손에 든 무게를 알리기 | [43_pl2m_pick/README.md](43_pl2m_pick/README.md) | `pl2m_pick.py` 탁자 · 컵 · 손가락 설정 · 든 무게의 보상 · `pick_cup.py` 집는 순서 · `scene.py` |
| 44. 부스에서 Nav2 로 이동하기: 라이다 · 바퀴 오도메트리 · 도착 오차 | [44_pl2m_nav/README.md](44_pl2m_nav/README.md) | `pl2m_nav.py` 부스 + 라이다 + 바퀴 오도메트리 · `make_map.py` · `nav2.launch.py` · `go_to.py` 목적지와 도착 오차 |
| 45. ArUco 마커로 도킹하기: Nav2 모드와 도킹 모드를 오가는 법 | [45_pl2m_dock/README.md](45_pl2m_dock/README.md) | `pl2m_dock.py` 마커 6장 + 앞 카메라 · `aruco_node.py` 마커 찾기 · `dock.py` 모드를 바꿔 가며 도킹 |
| 46. 컵을 집어 나르기: Nav2 · 도킹 · 팔을 모드로 잇기 | [46_pl2m_mission/README.md](46_pl2m_mission/README.md) | `pl2m_mission.py` 장면 합치기 · `layout.py` 스테이션의 배치 · `mission.py` Nav2 → 도킹 → 집기 → 놓기 |
| 47. 머리 카메라로 컵 찾기: YOLO 와 깊이 영상으로 컵의 자리 구하기 | [47_pl2m_vision/README.md](47_pl2m_vision/README.md) | `pl2m_vision.py` 머리 카메라(색 · 깊이) · `make_dataset.py` 학습 데이터 · `cup_detector.py` 컵 찾기 · `locate.py` 상자 + 깊이 → 자리 · `check_cups.py` 채점 |
| 48. 찾은 컵 집기: 좌표표 없이 카메라로 본 자리로 집어 나르기 | [48_pl2m_find_pick/README.md](48_pl2m_find_pick/README.md) | `find_pick.py` 보고(LOOK) → 설 자리 · 팔 정하기 → 도킹 → 다시 보고 집기 · `reach_map.py` 팔이 닿는 범위 재기 |
| 49. TensorRT 로 모델 빠르게 돌리기: 빨라진 만큼이 로봇에서도 빨라지나 | [49_tensorrt/README.md](49_tensorrt/README.md) | `export_compare.py` ONNX · TensorRT FP32 · FP16 으로 바꾸고 정확도 · 속도 견주기 (로봇에서는 강의 47 의 `cup_detector.py --timing`) |
| 50. USD 장면을 파일 여러 개로: 참조 · payload · Variant · 레이어 | [50_usd_layers/README.md](50_usd_layers/README.md) | `make_layers.py` 컵 부품 · 배치 · 덮어쓰는 파일 · 여는 파일 만들기 · `show_layers.py` 값이 온 파일 보기, 레이어 끄기, Variant 바꾸기 |
| 51. 익스텐션 만들기: 버튼으로 컵을 다시 놓는 창 | [51_extension/README.md](51_extension/README.md) | `exts/pinklab.cup_tools/` 익스텐션(이름표 · 창 · 명령으로 하는 버튼) · `open_with_extension.py` 장면을 열고 익스텐션 켜기 · 창 없이 시험 |
| 52. 팔이 목표에 손을 뻗게 학습하기: OpenArm Reach 와 PL-2M 의 왼팔 | [52_openarm_reach/README.md](52_openarm_reach/README.md) | `pl2m_reach_env.py` Isaac Lab 의 Reach 과제에서 로봇을 PL-2M 으로 · `reach_samples.py` 팔이 닿는 범위 재기 · `train.py` · `play.py` 학습과 평가 |
| 53. 부스의 디지털트윈: 상자를 새로 놓으면 로봇의 길이 막히나 | [53_booth_twin/README.md](53_booth_twin/README.md) | `review.py` 지금의 부스 → 검토 안 → 낙하 시험 · `twin.py` 길 · 거리 · 상자 쌓기 · `paths/` 로봇이 다닌 길 |
| 54. 로봇의 눈에 AI 모델 붙이기: 이름으로 찾고, 말로 묻고, 사진 한 장으로 거리 재기 | [54_pl2m_models/README.md](54_pl2m_models/README.md) | `models.py` 받아 쓰는 모델 넷 · `try_models.py` 머리 카메라 영상에 돌리고 참값과 견주기 · `find_go.py` 이름으로 찾아가기 |
| 55. MobilityGen 으로 주행 데이터 만들기: Pinky Pro 를 등록하고, 기록하고, 다시 틀어 영상 그리기 | [55_pinky_mobilitygen/README.md](55_pinky_mobilitygen/README.md) | `pinky_robot.py` MobilityGen 에 Pinky Pro 등록 · `prepare.py` 로봇 파일과 점유 지도 · `record.py` 자동 주행 기록 · `replay.py` 다시 틀며 영상 그리기 · `read_data.py` 데이터 읽기 · `open_ui.py` 창에서 키보드로 몰기 |

강의 4 ~ 7 은 로봇 하나(SimpleRobot)를 단계별로 키우고, 강의 8 ~ 10 은 URDF 로 가져온 Carter 를 같은 순서(가져오기 → 조종 → 센서)로 다룹니다. 강의 11 ~ 13 은 핑크랩의 [Pinky Pro](https://github.com/pinklab-art/pinky_pro) 를 공식 저장소의 URDF 로 가져와 모터 · 센서 · 화면 · LED 까지 올립니다. 뒤 강의의 스크립트는 앞 강의의 스크립트를 불러다 쓰고, 함께 쓰는 도구는 `common/sim.py` 에 있습니다.

## 한눈에 보기

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

# 4 ~ 6. 로봇 만들기 → 조종 → 센서
~/isaacsim/python.sh 04_build_robot/build_robot.py
~/isaacsim/python.sh 05_control/drive_robot.py --keyboard
~/isaacsim/python.sh 06_sensors/add_sensors.py

# 7. ROS 2 (ROS 2 Jazzy 를 먼저 source)
source /opt/ros/jazzy/setup.bash
~/isaacsim/python.sh 07_ros2/publish_ros2.py

# 8 ~ 10. URDF 로봇(Carter) 가져오기 → 몰기 → 센서
mkdir -p work && cp -r ~/isaacsim/exts/isaacsim.asset.importer.urdf/data/urdf/robots/carter work/
~/isaacsim/python.sh 08_urdf_import/import_carter.py
~/isaacsim/python.sh 09_carter_drive/drive_carter.py --keyboard
~/isaacsim/python.sh 10_carter_sensors/carter_sensors.py
~/isaacsim/python.sh 10_carter_sensors/nova_carter.py

# 11 ~ 13. Pinky Pro (ROS 2 Jazzy 필요)
bash 11_pinky_import/prepare_pinky.sh
~/isaacsim/python.sh 11_pinky_import/import_pinky.py --keyboard
~/isaacsim/python.sh 12_pinky_sensors/pinky_sensors.py
~/isaacsim/python.sh 13_pinky_lcd_led/pinky_face_led.py --show

# 14. Pinky Pro 를 ROS 2 로 (ROS 2 Jazzy 를 먼저 source, 다른 터미널에서 teleop)
source /opt/ros/jazzy/setup.bash
~/isaacsim/python.sh 14_pinky_ros2/pinky_ros2.py
```

- Python 스크립트는 시스템의 `python3` 가 아니라 Isaac Sim 에 들어 있는 `~/isaacsim/python.sh` 로 실행합니다.
- Isaac Sim 을 다른 곳에 설치했다면 `~/isaacsim` 자리에 그 경로를 씁니다. 셸 스크립트는 `ISAACSIM_PATH=/경로 bash ...` 로 지정합니다.
- 내가 만든 장면은 `work/` 에 저장합니다 (git 이 무시하는 폴더).
