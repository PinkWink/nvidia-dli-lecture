# 55. MobilityGen 으로 주행 데이터 만들기: Pinky Pro 를 등록하고, 기록하고, 다시 틀어 영상 그리기 — 명령 모음

- 영상에서 입력하는 터미널 명령과 기대 출력을 순서대로 모았음. [54. 로봇의 눈에 AI 모델 붙이기](../54_pl2m_models/README.md) 다음 내용임.

| 파일 |
|---|
| [open_ui.py](open_ui.py) |
| [pinky_robot.py](pinky_robot.py) |
| [prepare.py](prepare.py) |
| [read_data.py](read_data.py) |
| [record.py](record.py) |
| [replay.py](replay.py) |

## 1. 주행을 기록해 두고, 영상은 나중에 그림

### 이번 강의의 결과

- Pinky Pro 가 핑크시티를 스스로 돌아다닌 기록 하나에서 네 순간을 고른 것임. 위는 카메라 영상, 아래는 깊이, 제목은 그때 로봇에 준 속도 명령임.
- 영상과 속도 명령이 짝지어진 데이터임. 주행 모델을 학습시킬 때 쓰는 꼴임.
- 시뮬레이터의 시간 120초를 기록하는 데 37초, 영상 1204장을 그리는 데 84초가 걸렸음.

### MobilityGen 이 하는 일

- MobilityGen 은 Isaac Sim 5.1 에 들어 있는 익스텐션임. 움직이는 로봇의 데이터를 만드는 도구임.
- 기록할 때는 물리만 돌리고 로봇의 상태만 저장함. 카메라 영상은 나중에 기록을 다시 틀면서 그림.
- 그래서 같은 주행을 영상의 크기나 간격을 바꿔 몇 번이든 다시 그릴 수 있음.

### 강의 19 와 견주면

| | 강의 19 | 이 강의 |
|---|---|---|
| 로봇을 놓는 법 | 아무 자리에 옮겨 놓고 한 장 | 이어서 달리는 주행 |
| 영상과 함께 남는 것 | 도로 정답 마스크 | 속도 명령 · 자리 · 관절 |
| 쓰는 곳 | 도로를 알아보는 모델 | 주행을 배우는 모델 |
| 도구 | 직접 짠 수집기 | MobilityGen |

- 강의 19 의 데이터는 한 장 한 장이 따로임. 이 강의의 데이터는 시간 순서가 있고, 장마다 "그때 어떻게 몰았는지" 가 붙어 있음.

### NVIDIA 과정에서는

| | NVIDIA 과정 | 이 강의 |
|---|---|---|
| 장면 | 창고 | 핑크시티 (강의 16) |
| 로봇 | H1 · Carter (들어 있는 것) | Pinky Pro (새로 등록) |
| 로봇을 더하는 법 | Isaac Sim 설치 폴더의 `robots.py` 를 고침 | 내 파일에서 등록. 설치 폴더는 그대로 |
| 기록 | 창에서 버튼 | 스크립트 (창도 됨) |

- 설치 폴더를 고치면 Isaac Sim 을 다시 설치할 때 사라짐. 이 강의는 로봇을 강의 폴더의 파일 하나로 등록함.

### 만드는 것

| 파일 | 하는 일 |
|---|---|
| `prepare.py` | 로봇 파일과 점유 지도를 만듦 |
| `pinky_robot.py` | MobilityGen 에 Pinky Pro 를 등록함 |
| `record.py` | 화면 없이 자동 주행을 기록함 |
| `open_ui.py` | MobilityGen 창을 채워서 엶 (키보드로 몰기) |
| `replay.py` | 기록을 다시 틀며 영상을 그림 |
| `read_data.py` | 만든 데이터를 Python 으로 읽음 |

- 뒤의 것은 Isaac Sim 없이 돎. 데이터를 쓰는 쪽은 시뮬레이터가 필요 없음.

## 2. 재료 준비

### 터미널: 도시를 USD 로

```bash
cd ~/nvidia-dli-lecture
git pull
~/isaacsim/python.sh 16_pink_city/pink_city.py --save work/mobilitygen/pink_city.usd
```

```text
핑크시티: 3.0 × 3.0 m, 도로 폭 0.4 m, 블록 0.65 m
장면 저장: …/work/mobilitygen/pink_city.usd
```

- MobilityGen 은 장면을 USD 파일로 받음. 강의 16 의 스크립트에 있는 `--save` 로 도시를 파일로 만듦.
- 강의 11 의 준비(`bash 11_pinky_import/prepare_pinky.sh`)가 되어 있어야 함.

### 터미널: 로봇과 지도

```bash
~/isaacsim/python.sh 55_pinky_mobilitygen/prepare.py
```

```text
로봇: …/work/mobilitygen/pinky.usda  (관절 로봇의 뿌리: ['base_footprint'])
지도: …/work/mobilitygen/map/map.yaml  (160 × 160 칸, 한 칸 2 cm, 장애물로 친 충돌체 20개, 막힌 칸 12 %)
```

- 로봇 파일은 강의 11 의 가져오기에 강의 12 의 카메라를 붙여 저장한 것임.
- 출력의 "관절 로봇의 뿌리" 는 다음 절에서 로봇을 등록할 때 적는 값임.

### 점유 지도

- 회색이 장애물임. 건물, 나무, 바깥 벽. 로봇의 키 높이(2 ~ 15 cm)에 걸친 충돌체를 위에서 본 것임.
- 강의 17 에서 SLAM 으로 만든 지도와 같은 형식(`map.png` + `map.yaml`)임. 여기서는 장면에서 바로 계산했음.
- 도로와 보도는 지도에 없음. MobilityGen 의 로봇은 도로를 가리지 않고 빈 곳이면 다님.

### 코드: 충돌체를 지도에 칠하기

```python
for prim in stage.Traverse():
    if not prim.HasAPI(UsdPhysics.CollisionAPI) or prim.GetName() == "GroundPlane":
        continue
    box = cache.ComputeWorldBound(prim).ComputeAlignedRange()
    low, high = box.GetMin(), box.GetMax()
    if box.IsEmpty() or low[2] > Z_RANGE[1] or high[2] < Z_RANGE[0]:
        continue
    grid[rows, columns] = 0                      # 그 물체가 덮는 칸을 검게
```

- 충돌체마다 바닥에 드리운 네모를 구해 칸을 칠함. 강의 53 에서 가구의 자리를 구하던 것과 같은 방법임.
- 핑크시티의 건물은 모두 네모라서 이것으로 충분함. 비스듬한 물체는 네모가 실제보다 커짐.

## 3. Pinky Pro 를 등록하기

### 코드: 로봇 클래스

```python
@ROBOTS.register()
class PinkyRobot(WheeledMobilityGenRobot):
    physics_dt: float = 1.0 / 240.0
    wheel_dof_names: List[str] = ["l_wheel_joint", "r_wheel_joint"]
    usd_url: str = os.path.join(WORK, "pinky.usda")
    chassis_subpath: str = "base_footprint"
    wheel_base: float = 0.0961
    wheel_radius: float = 0.028
```

- 들어 있는 Jetbot · Carter 와 같은 틀(`WheeledMobilityGenRobot`)을 물려받고 값만 Pinky Pro 것으로 채움.
- `@ROBOTS.register()` 가 붙은 클래스는 불러오는 순간 로봇 목록에 들어감.
- 바퀴 값은 강의 11 의 것임. 물리는 초당 240 스텝으로 뒀음. 들어 있는 로봇들은 200 스텝임.

### 코드: 카메라

```python
class PinkyCamera(Module):
    resolution: Tuple[int, int] = (640, 360)

    def __init__(self, camera: MobilityGenCamera):
        self.camera = camera

    @classmethod
    def attach(cls, prim_path: str) -> "PinkyCamera":
        return cls(MobilityGenCamera(prim_path, cls.resolution))

    build = attach
```

- Carter 는 두 눈짜리 Hawk 카메라를 새로 붙임. Pinky Pro 는 로봇 파일에 이미 있는 카메라 하나에 붙기만 함.
- `resolution` 이 그려질 영상의 크기임. 재생할 때 바꿀 수 있음.

### 코드: 주행에 쓰는 값

```python
    occupancy_map_radius: float = 0.14
    occupancy_map_collision_radius: float = 4
    keyboard_linear_velocity_gain: float = 0.15
    path_following_speed: float = 0.12
    path_following_stop_distance_threshold: float = 0.08
    path_following_target_point_offset_meters: float = 0.15
```

- Carter 의 값은 바퀴 사이가 41 cm 인 로봇에 맞춘 것임. 바퀴 사이가 10 cm 인 Pinky Pro 에 맞게 모두 줄였음.
- `occupancy_map_radius` 는 길을 찾을 때 장애물을 부풀리는 폭임. 4절에서 이 값을 바꿔 봄.

> **collision_radius 의 단위**: Isaac Sim 5.1 의 `scenarios.py` 는 `occupancy_map_collision_radius` 를 미터가 아니라 지도의 칸 수로 읽음. 코드에서 확인했음. Carter 의 0.5 는 정수로 바뀌며 0칸이 됨. 그러면 로봇의 가운데가 장애물 안에 들어가야 부딪힌 것으로 침. 여기서는 4 로 뒀음. 한 칸이 2 cm 라서 8 cm 임.

## 4. 기록하기

### 터미널: 120초

```bash
~/isaacsim/python.sh 55_pinky_mobilitygen/record.py --seconds 120
```

```text
기록 8개, 시뮬레이터의 시간 120초를 37초에 돌림  → …/work/mobilitygen/recordings
  2026-11-22_20-22-54-246242     5251 스텝 ·  21.9초
  2026-11-22_20-23-00-890933     4816 스텝 ·  20.1초
  2026-11-22_20-23-07-058313     1077 스텝 ·   4.5초
  …
```

- 로봇이 무작위 자리에 놓이고, 무작위 목표까지 길을 찾아 따라감. 도착하면 새 자리에 다시 놓이고 새 기록이 시작됨.
- 화면을 그리지 않으므로 실제 시간보다 빠름. 120초가 37초에 돌았음.

### 코드: Build 버튼이 하는 일

```python
open_stage(city)
world = new_world(physics_dt=robot_type.physics_dt)
world.initialize_physics()
objects.GroundPlane("/World/ground_plane", visible=False)
robot = robot_type.build("/World/robot")
scenario = scenario_type.from_robot_occupancy_map(robot, occupancy_map)
world.reset()
```

- MobilityGen 창의 Build 가 하는 일을 순서대로 적은 것임. 장면을 열고, 로봇을 놓고, 시나리오를 만듦.
- 시나리오가 "어떻게 몰지" 를 정함. 여기서는 `RandomPathFollowingScenario` 임.

### 코드: 한 스텝씩 기록

```python
for count in range(steps):
    world.step(render=False)
    alive = scenario.step(robot_type.physics_dt)
    if not alive:
        scenario.reset()
        writer, step = new_recording(), 0
    writer.write_state_dict_common(scenario.state_dict_common(), step=step)
    step += 1
```

- `scenario.step` 이 속도 명령을 정해 바퀴에 넣음. 도착했거나 부딪혔으면 `False` 를 돌려줌.
- 스텝마다 상태 하나를 `.npy` 파일로 씀. 물리가 초당 240 스텝이라 기록 1초에 파일 240 개임.

### 한 기록에 든 것

```text
2026-11-22_20-22-54-246242/
  config.json            로봇 · 시나리오 · 장면의 이름
  occupancy_map/         그때 쓴 지도
  stage.usd              장면의 사본
  state/common/          00000000.npy  00000001.npy  …
```

- 영상은 아직 없음. 120초의 기록이 114 MB 였음.
- 장면의 사본이 함께 들어감. 재생할 때 이 사본을 엶.

### 처음에는 모퉁이에 걸렸음

- 처음에는 부풀리는 폭을 0.09 m 로 뒀음. 로봇이 건물 모퉁이에서 5 cm 떨어진 채 걸려, 돌라는 명령을 받으면서도 기록이 끝날 때까지 14초를 서 있었음.
- 길이 장애물에서 8 cm 떨어져 지나가는데, 로봇은 모퉁이를 조금 안쪽으로 돎.

### 부풀리는 폭을 바꿔 봄

```bash
~/isaacsim/python.sh 55_pinky_mobilitygen/record.py --seconds 180 --seed 1 --margin 0.09
~/isaacsim/python.sh 55_pinky_mobilitygen/record.py --seconds 180 --seed 1 --margin 0.14
```

- 씨앗을 같게 두고 폭만 바꿨음. 0.09 m 에서는 28개 가운데 7개만 도착했고 20개는 장애물에 8 cm 안으로 다가가 다시 놓였음.
- 0.14 m 에서는 12개가 모두 도착했음. 나머지 하나씩은 시간이 끝나 잘린 것임.
- 같은 씨앗으로 두 번 돌리면 스텝 수까지 같은 기록이 나옴. 확인했음.

### 다른 시나리오: 속도를 무작위로

```bash
~/isaacsim/python.sh 55_pinky_mobilitygen/record.py --seconds 120 --scenario RandomAccelerationScenario --out work/mobilitygen/random
```

- 목표 없이 속도를 조금씩 무작위로 바꾸며 다님. 장애물에 다가가면 다시 놓임.
- 120초 동안 4.9 m 를 갔음. 길을 따라가는 시나리오는 같은 시간에 13.9 m 였음.

### 두 시나리오의 속도 명령

- 길 따라가기는 직진 속도가 거의 0.12 m/s 한 값임. 곧게 가는 때가 67 % 임. 회전 명령은 드물게 ±4 rad/s 를 넘음. 명령에 한계가 없음.
- 무작위 가속은 속도가 고르게 퍼져 있고 뒤로 가는 것도 있음. 대신 멀리 가지 못함.
- 데이터가 어떻게 퍼져 있는지는 학습 전에 봐야 함. 한쪽으로 치우친 데이터로 배운 모델은 그쪽만 잘함.

## 5. 다시 틀어 영상 그리기

### 터미널

```bash
~/isaacsim/python.sh 55_pinky_mobilitygen/replay.py
```

```text
[1/8] 2026-11-22_20-22-54-246242  5251 스텝 → 219장
[2/8] 2026-11-22_20-23-00-890933  4816 스텝 → 201장
…
모두 1204장을 84초에 그림 (초당 14.3장), 86 MB  → …/work/mobilitygen/replays
```

- 기록마다 24 스텝에 한 장, 곧 초당 10장을 그림. 색 영상(`.jpg`)과 깊이 영상(`.png`)이 `replays/` 아래에 생김.
- 원래 기록은 그대로 둠. 영상이 붙은 사본이 따로 생김.

### 코드: 놓고, 그리고, 읽음

```python
for step in range(0, len(reader), args.every):
    scenario.load_state_dict(reader.read_state_dict(index=step))
    scenario.write_replay_data()
    app.update()
    rep.orchestrator.step(rt_subframes=1, delta_time=0.0, pause_timeline=False)
    scenario.update_state()
    writer.write_state_dict_rgb(scenario.state_dict_rgb(), step)
```

- 기록된 상태를 읽어 로봇을 그 자리, 그 관절 각도로 놓고 한 장 그림. 물리는 돌리지 않음.
- 그리기는 강의 47 에서 쓴 Replicator 의 `orchestrator.step` 임.

### 처음 그린 영상: 그림이 모두 사라졌음

- 처음 재생한 영상에는 도로도, 건물의 창문도 없었음. 바닥과 벽이 모두 회색이었음.
- 기록에 든 장면의 사본을 열어 보니 그림 파일의 경로가 `../pink_city/floor.png` 로 바뀌어 있었음. 원래는 전체 경로였음.
- Isaac Sim 의 `save_stage` 가 사본을 만들며 경로를 상대 경로로 바꾼 것임. 그 사본이 기록 폴더로 옮겨지면 그림을 못 찾음.

### 코드: 경로를 그대로 두는 저장

```python
if args.relative_textures:
    save_stage(cached_stage, save_and_reload_in_place=False)
else:
    omni.usd.get_context().get_stage().GetRootLayer().Export(cached_stage)
```

- 레이어의 `Export` 는 적힌 경로를 그대로 둠. 이것으로 사본을 만들게 고쳤음.
- `--relative-textures` 를 주면 고치기 전의 방식으로 기록함. 재생해 보면 회색 영상이 나옴.
- 숫자만 봐서는 알 수 없는 문제였음. 기록 몇 개를 먼저 그려서 눈으로 봐야 함.

### 같은 주행을 다르게 그리기

```bash
~/isaacsim/python.sh 55_pinky_mobilitygen/replay.py --every 8 --out work/mobilitygen/replays_30fps
~/isaacsim/python.sh 55_pinky_mobilitygen/replay.py --size 1280 720 --out work/mobilitygen/replays_hd
~/isaacsim/python.sh 55_pinky_mobilitygen/replay.py --no-depth --out work/mobilitygen/replays_color
```

- 기록 셋(465장)을 기준으로 잰 것임. 세 배 촘촘히 그리면 시간도 크기도 세 배쯤임.
- 영상을 가로 세로 두 배로 키우면 시간은 1.7배, 크기는 2.8배였음.
- 깊이를 빼도 시간은 거의 같음. 크기는 33 MB 에서 7 MB 로 줆. 용량의 대부분이 깊이 영상임.

## 6. 데이터 읽기

### 터미널

```bash
python3 55_pinky_mobilitygen/read_data.py
```

```text
…/work/mobilitygen/replays: 기록 8개.  로봇 PinkyRobot, 시나리오 RandomPathFollowingScenario
상태 하나에 든 것: target_path (5, 2), robot.action (2,), robot.position (3,), …
  [0] 20-22-54   21.8초 · 2.54 m · 영상 219장, 목표까지 남은 거리 0.09 m
  [1] 20-23-00   20.0초 · 2.33 m · 영상 201장, 목표까지 남은 거리 0.09 m
  …
모두 120초 · 13.9 m · 영상 1204장
속도 명령: 직진 평균 0.116 m/s,  회전 |ω| 평균 0.25 rad/s,  왼쪽으로 도는 중 16 % · 곧게 67 % · 오른쪽 17 %
```

- Isaac Sim 없이 도는 스크립트임. `numpy` 와 `Pillow` 만 씀.
- 마지막 기록만 목표까지 0.47 m 가 남았음. 120초가 끝나 잘린 것임.

### 상태 하나에 든 것

| 이름 | 크기 | 뜻 |
|---|---|---|
| `robot.action` | 2 | 속도 명령 (직진 m/s, 회전 rad/s) |
| `robot.position` · `robot.orientation` | 3 · 4 | 자리와 자세 |
| `robot.joint_positions` · `robot.joint_velocities` | 4 · 4 | 바퀴 둘과 캐스터 |
| `robot.linear_velocity` · `robot.angular_velocity` | 3 · 3 | 실제로 움직인 속도 |
| `robot.front_camera.camera.position` · `orientation` | 3 · 4 | 카메라의 자리와 자세 |
| `target_path` | N × 2 | 따라가던 길 |

- 명령한 속도와 실제 속도가 따로 들어 있음.
- 카메라의 자리도 스텝마다 있음. 영상으로 3차원을 복원하는 데 쓸 수 있음.

### 코드: 한 스텝 읽기

```python
state = np.load("state/common/00000120.npy", allow_pickle=True).item()
v, w = state["robot.action"]
x, y, _ = state["robot.position"]

stored = np.array(Image.open("state/depth/…/00000120.png")).astype(np.float32)
depth = 65535.0 / np.maximum(stored, 1.0) - 1.0
```

- 상태는 이름 → 값인 사전이 든 `.npy` 파일임.
- 깊이는 16비트 정수로 저장돼 있음. `65535 / (1 + 거리)` 로 넣은 것이라 거꾸로 풀어야 미터가 됨.

### 그림으로

```bash
python3 55_pinky_mobilitygen/read_data.py --map map.png --commands commands.png --frames frames.png --recording 1
```

- `--map` 은 지도 위에 다닌 길(실선)과 따라가려던 길(점선)을 그림. 2절의 그림임.
- `--commands` 는 속도 명령의 분포, `--frames` 는 한 기록의 영상 넷임.

## 7. 창에서 키보드로 몰기

### 터미널

```bash
~/isaacsim/python.sh 55_pinky_mobilitygen/open_ui.py
```

- MobilityGen 창이 장면 · 지도 · 로봇 · 시나리오가 채워진 채로 열림.
- 사람이 직접 몬 데이터를 섞고 싶을 때 씀. 자동 시나리오와 달리 사람다운 움직임이 들어감.

### 창에서 하는 일

- Build 를 누르면 로봇이 무작위 자리에 놓임.
- 뷰포트를 한 번 누르고 W A S D 로 몲. 직진 0.15 m/s, 회전 1 rad/s 임.
- Start Recording → 몰기 → Stop Recording. Reset 을 누르면 새 자리에 놓이고 새 기록이 시작됨.
- 기록은 `record.py` 와 같은 폴더에 쌓임. `replay.py` 와 `read_data.py` 를 그대로 씀.

### 코드: 창이 뜨기 전에 등록

```python
enable_extension("isaacsim.replicator.mobility_gen.examples")
import pinky_robot                    # 여기서 PinkyRobot 이 목록에 들어감
enable_extension("isaacsim.replicator.mobility_gen.ui")
```

- 창은 뜰 때 로봇 목록을 읽어 Robot Type 의 선택지를 만듦. 그래서 창을 켜기 전에 등록해야 함.
- 창의 Build 도 `save_stage` 로 사본을 만듦. `open_ui.py` 는 그 함수를 경로를 그대로 두는 것으로 갈아 끼움.

### 점검: 버튼을 코드로

```bash
~/isaacsim/python.sh 55_pinky_mobilitygen/open_ui.py --test
```

```text
로봇 목록: JetbotRobot, CarterRobot, H1Robot, SpotRobot, PinkyRobot
기록: …/recordings/2026-11-22_20-34-33-630224  1440 스텝 · 6.0초
```

- Build 와 Start Recording 을 코드로 부르고, W 를 6초 누른 것으로 넣었음. 2 ~ 4초에는 A 도 함께 넣었음.
- 0.88 m 를 갔고, 재생하니 영상 60장이 그림과 함께 나왔음.
- 마우스와 키보드로 직접 눌러 보지는 않았음.
- 강의 PC 의 목록에는 NVIDIA 과정에서 더해 둔 `CarterFixedRobot` 도 있었음. 위 출력에서는 뺐음.

## 8. 해 볼 것

### 직접 바꿔 보기

- `record.py --seed 2` 로 다른 자리 · 다른 목표의 기록 만들기.
- `record.py --margin 0.11` 로 도착하는 비율이 어디서부터 떨어지는지 보기.
- `pinky_robot.py` 의 `path_following_speed` 를 0.2 로 올리고 길에서 얼마나 벗어나는지 `--map` 으로 보기.
- `open_ui.py` 로 직접 몬 기록과 자동 기록의 `--commands` 그림 견주기.

## 9. 정리

### 기억할 것

- MobilityGen 은 기록과 그리기를 나눔. 기록은 빠르고, 영상은 나중에 원하는 크기와 간격으로 그림.
- 새 로봇은 클래스 하나로 등록함. 설치 폴더를 고치지 않아도 됨.
- 들어 있는 값은 큰 로봇의 것임. 작은 로봇은 부풀리는 폭, 속도, 도착 거리를 모두 다시 정해야 함.
- 장애물을 부풀리는 폭 하나로 도착하는 기록이 28개 중 7개에서 12개 중 12개로 바뀌었음.
- 만든 데이터는 몇 개를 먼저 그려서 눈으로 봄. 그림이 사라진 것은 숫자로는 알 수 없었음.

### 한계

- 이 데이터로 주행 모델을 학습시키지는 않았음. 데이터를 만드는 데까지임.
- 지도에 도로가 없어 로봇이 보도와 공원을 가리지 않고 다님. 도로만 다니는 데이터는 만들지 않았음.
- 정답 마스크(semantic segmentation)는 그리지 않았음. 핑크시티에 이름표가 붙어 있지 않음.
- 창의 버튼은 코드로만 불렀음. 게임패드 시나리오는 해 보지 않았음.
- 강의 PC(RTX 5060 Ti) 한 대에서 잰 시간임.
