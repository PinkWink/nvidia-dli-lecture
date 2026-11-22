# 51. 익스텐션 만들기: 버튼으로 컵을 다시 놓는 창 — 명령 모음

- 영상에서 입력하는 터미널 명령과 기대 출력을 순서대로 모았음. [50. USD 장면을 파일 여러 개로](../50_usd_layers/README.md) 다음 내용임.

| 파일 |
|---|
| [open_with_extension.py](open_with_extension.py) |
| [exts/pinklab.cup_tools/config/extension.toml](exts/pinklab.cup_tools/config/extension.toml) |
| [exts/pinklab.cup_tools/pinklab/cup_tools/extension.py](exts/pinklab.cup_tools/pinklab/cup_tools/extension.py) |

## 1. Isaac Sim 에 내 창을 더함

### 이번 강의의 결과

- Isaac Sim 안에 뜬 작은 창이 이 강의에서 만든 것임. 버튼을 누르면 컵이 섞이고, 색이 바뀌고, 음료대 옆 사람이 꺼짐.
- 지금까지는 이런 일을 스크립트의 옵션(`--shuffle 3`)으로 했음. 창으로 만들면 Isaac Sim 을 띄워 둔 채로 여러 번 할 수 있음.

### 익스텐션

| | 스크립트 (지금까지) | 익스텐션 |
|---|---|---|
| 실행 | Isaac Sim 을 띄우면서 한 번 | Isaac Sim 안에서 켜고 끔 |
| 사는 동안 | 스크립트가 끝날 때까지 | 켜져 있는 동안 |
| 쓰는 곳 | 그 스크립트 하나 | 어느 장면에서나 |

- Isaac Sim 자체가 익스텐션의 묶음임. Stage 창, URDF Importer, ROS 2 브리지가 다 익스텐션임. 강의 50 의 Layer 창도 익스텐션을 켜서 띄웠음.
- 익스텐션 하나는 폴더 하나임. 이름표 파일과 파이썬 모듈이 들어 있음.

### NVIDIA 과정에서는

| | NVIDIA 과정 | 이 강의 |
|---|---|---|
| 만드는 법 | Kit App Template 으로 틀을 받아 빌드함 | 파일 넷을 직접 씀 |
| 돌리는 앱 | 따로 만든 Kit 앱 | Isaac Sim 그대로 |
| 만드는 것 | 도형과 에셋을 놓는 버튼 | 강의 50 의 컵을 다루는 버튼 |

- 틀을 받아 빌드하는 길은 앱을 통째로 새로 만들 때 쓰는 방법임. 이미 있는 Isaac Sim 에 창 하나를 더하는 데는 폴더 하나면 됨.

### 만드는 것

```text
51_extension/
├── open_with_extension.py                  강의 50 의 장면을 열고 익스텐션을 켬
└── exts/
    └── pinklab.cup_tools/                  익스텐션 하나 = 폴더 하나
        ├── config/extension.toml           이름표: 이름 · 버전 · 필요한 것 · 파이썬 모듈
        └── pinklab/cup_tools/
            ├── __init__.py
            └── extension.py                켜질 때 · 꺼질 때 하는 일, 버튼이 하는 일
```

- 폴더 이름 `pinklab.cup_tools` 가 익스텐션의 이름임. 파이썬 모듈의 이름과 같게 둠.

## 2. 켜 보기

### 터미널: 준비

```bash
cd ~/nvidia-dli-lecture
git pull
~/isaacsim/python.sh 50_usd_layers/make_layers.py
```

- 강의 50 의 원래 배치(`work/booth_layers/layout.usda`)를 씀. 강의 50 을 따라 했다면 이미 있음.

### 터미널: 장면을 열고 익스텐션 켜기

```bash
~/isaacsim/python.sh 51_extension/open_with_extension.py
```

```text
익스텐션 pinklab.cup_tools: 켜짐
```

- `PinkLAB Cup Tools` 창이 뜸. 맨 윗줄이 방금 한 일, 그 아래가 컵 셋의 자리임. 그림은 창을 왼쪽 위로 옮겨 놓고 찍은 것임.
- Window 메뉴에 `PinkLAB Cup Tools` 항목이 생김. 창을 닫았을 때 다시 띄우는 데 씀.

### 코드: 익스텐션을 켜는 두 줄

```python
manager = omni.kit.app.get_app().get_extension_manager()
manager.add_path(os.path.join(HERE, "exts"))                       # 익스텐션을 찾을 폴더를 하나 더한다
manager.set_extension_enabled_immediate(NAME, True)                # 켠다 → on_startup 이 불린다
```

- Isaac Sim 은 정해진 폴더들에서 익스텐션을 찾음. 내 폴더를 그 목록에 더한 뒤 이름으로 켬.

### Isaac Sim 을 평소처럼 띄울 때

```bash
~/isaacsim/isaac-sim.sh --ext-folder ~/nvidia-dli-lecture/51_extension/exts --enable pinklab.cup_tools
```

```text
[3.177s] [ext: pinklab.cup_tools-0.1.0] startup
```

- 스크립트 없이 Isaac Sim 을 띄울 때는 두 옵션으로 같은 일을 함. 터미널에 위 줄이 나오면 켜진 것임.
- 빈 장면에서는 창에 `no cups at /World/Cups` 가 나옴. 버튼은 컵이 든 장면을 연 뒤에 씀. 이렇게 띄운 앱에서는 창이 뜨는 것까지만 확인했음.

## 3. 이름표: extension.toml

### 파일

```text
[package]
version = "0.1.0"
title = "PinkLAB Cup Tools"

[dependencies]
"omni.ui" = {}                # 창과 버튼
"omni.usd" = {}               # 열려 있는 장면
"omni.kit.commands" = {}      # 되돌릴 수 있는 명령
"omni.kit.menu.utils" = {}    # 위쪽 메뉴에 항목 넣기
"omni.kit.property.usd" = {}  # Variant 를 고르는 명령 (SelectVariantPrim) 이 든 곳

[[python.module]]
name = "pinklab.cup_tools"    # 불러올 파이썬 모듈: 같은 폴더의 pinklab/cup_tools/
```

- `[dependencies]` 는 이 익스텐션이 쓰는 다른 익스텐션임. Isaac Sim 이 그것들을 먼저 켠 뒤에 이것을 켬.
- `[[python.module]]` 이 불러올 파이썬 모듈임. 그 모듈 안에서 `omni.ext.IExt` 를 물려받은 클래스를 찾아 씀.

## 4. 켜질 때와 꺼질 때

### 코드: on_startup

```python
class CupToolsExtension(omni.ext.IExt):
    def on_startup(self, ext_id):
        self._window = ui.Window(TITLE, width=320, height=250)
        with self._window.frame:
            with ui.VStack(spacing=6, height=0):
                self._status = ui.Label("", height=44, word_wrap=True)
                with ui.HStack(spacing=6, height=30):
                    ui.Button("Shuffle", clicked_fn=self.shuffle, tooltip="Put the cups anywhere in the front row")
                    ui.Button("Home", clicked_fn=self.go_home)
        ...
        self._menu = [MenuItemDescription(name=TITLE, onclick_fn=self.show)]
        omni.kit.menu.utils.add_menu_items(self._menu, "Window")
```

- 켜질 때 `on_startup` 이 한 번 불림. 여기서 창을 만들고 메뉴에 항목을 넣음.
- `ui.VStack` 은 위아래로, `ui.HStack` 은 옆으로 늘어놓음. `with` 안에 적은 것이 그 안에 들어감.
- 버튼의 `clicked_fn` 이 눌렸을 때 부를 함수임.

### 코드: on_shutdown

```python
def on_shutdown(self):
    omni.kit.menu.utils.remove_menu_items(self._menu, "Window")
    self._window.destroy()
    self._window = None
```

- 꺼질 때 불림. 켜질 때 만든 것을 빠짐없이 치움.

### 치우지 않으면

| | 끈 뒤 Window 메뉴에 남은 항목 |
|---|---|
| `on_shutdown` 에서 치움 | 0개 |
| 치우지 않음 | 1개 |

- 치우지 않은 익스텐션은 꺼진 뒤에도 메뉴에 항목이 남았음. 이미 없는 익스텐션을 가리키는 항목임.
- 익스텐션은 Isaac Sim 이 떠 있는 동안 여러 번 켜고 꺼짐. 만든 것은 만든 쪽이 치워야 함.

### 창의 글자는 영어로

- 처음에는 버튼에 한글을 썼음. 창에 `??` 로 나왔음. Isaac Sim 의 기본 글꼴에 한글이 없기 때문임.
- 그래서 창에 보이는 글자는 영어로 쓰고, 설명은 코드의 주석에 한글로 달았음.

## 5. 버튼이 하는 일: 명령

### 코드: 컵 옮기기

```python
def move_cup(prim, x, y):
    place = prim.GetAttribute("xformOp:translate")
    before = place.Get()
    omni.kit.commands.execute("ChangeProperty", prop_path=Sdf.Path(place.GetPath()), value=Gf.Vec3d(x, y, before[2]), prev=before)
```

- 값을 바로 적지(`place.Set(...)`) 않고 명령으로 실행함. 명령은 무엇을 무엇으로 바꿨는지를 기억하므로 되돌릴 수 있음.
- Isaac Sim 의 창에서 손으로 하는 일도 거의 다 명령으로 실행됨. 버튼이 같은 명령을 쓰면 손으로 한 것과 똑같이 다뤄짐.

### 쓴 명령 셋

| 버튼 | 명령 | 하는 일 |
|---|---|---|
| Shuffle · Home | `ChangeProperty` | 속성 하나의 값을 바꿈 (컵의 자리) |
| pink · blue · yellow | `SelectVariantPrim` | Variant 를 고름 (강의 50 의 색) |
| Person on / off | `ToggleActivePrims` | Prim 을 켜고 끔 |

- Isaac Sim 5.1 에는 명령이 438개 등록돼 있음 (`omni.kit.commands.get_commands()`).
- `SelectVariantPrim` 은 `omni.kit.property.usd` 익스텐션에 들어 있음. 그래서 이름표의 `[dependencies]` 에 그것을 적었음.

### 터미널: 창 없이 시험하기

```bash
~/isaacsim/python.sh 51_extension/open_with_extension.py --test
```

```text
익스텐션 pinklab.cup_tools: 켜짐
Window 메뉴의 항목: 1개
[Shuffle] shuffled (1)  →  pink (7.46, -1.71),  blue (7.79, -1.74),  yellow (7.65, -1.73)
[blue] 3 cups -> blue  →  pink (7.46, -1.71),  blue (7.79, -1.74),  yellow (7.65, -1.73)
[Person on / off] person off  →  pink (7.46, -1.71),  blue (7.79, -1.74),  yellow (7.65, -1.73)
[Home] home  →  pink (7.50, -1.72),  blue (7.65, -1.72),  yellow (7.80, -1.72)
[Ctrl+Z × 3] undo x 3  →  pink (7.46, -1.71),  blue (7.79, -1.74),  yellow (7.65, -1.73)
끈 뒤 Window 메뉴의 항목: 0개
```

- 버튼이 부르는 함수를 차례로 불러 본 것임. 창을 띄우지 않고도 버튼의 동작을 확인할 수 있음.
- 마지막에서 둘째 줄이 되돌리기임. `Home` 이 컵 셋을 옮겼고, 세 번 되돌리자 섞은 자리로 돌아갔음. 창에서는 Ctrl+Z 를 세 번 누르는 것임.

### 코드: 섞기

```python
rng, spots = random.Random(self._seed), []
while len(spots) < len(found):
    spot = (rng.uniform(FRONT[0], FRONT[1]), rng.uniform(FRONT[2], FRONT[3]))
    if all((spot[0] - x) ** 2 + (spot[1] - y) ** 2 >= GAP ** 2 for x, y in spots):
        spots.append(spot)
for cup, (x, y) in zip(found, spots):
    move_cup(cup, x, y)
```

- 강의 47 의 `--shuffle --front` 와 같은 규칙임. 음료대의 앞줄에서 서로 9 cm 넘게 떨어진 자리를 고름.
- 컵이 든 곳은 `/World/Cups` 임. 강의 38 · 47 · 50 의 장면이 모두 같은 경로를 쓰므로 어느 장면에서나 섞을 수 있음.

## 6. 고치고 다시 읽히기

### 파일을 고친 뒤

| 한 일 | 읽힌 코드 |
|---|---|
| `extension.py` 를 고쳐 저장함 | 예전 것 그대로 |
| 익스텐션을 껐다 켬 | 새 것 |

- `extension.py` 의 값 하나(`GAP`)를 고쳐 저장하고 2초를 기다렸지만 그대로였음. 익스텐션을 껐다 켜자 새 값이 읽혔음.
- 강의 PC 의 Isaac Sim 5.1 에서 본 것임. 스크립트로 띄웠을 때도, `isaac-sim.sh` 로 띄웠을 때도 저장만으로는 다시 읽히지 않았음.

### 껐다 켜는 법

- 코드에서: `set_extension_enabled_immediate(이름, False)` 뒤에 `True`. `open_with_extension.py` 가 끝날 때 끄는 것도 이 함수임. 위의 표는 이렇게 해서 잰 것임.
- 창에서: Window > Extensions 의 목록에서 익스텐션의 스위치를 껐다 켬. 이 강의에서는 코드로만 해 봤음.

## 7. 해 볼 것

### 직접 바꿔 보기

- `FRONT` 를 음료대 전체(`7.39, 7.91, -1.76, -1.44`)로 넓히고 섞어 보기.
- 버튼을 하나 더 만들어, 컵을 한 줄로 5 cm 간격으로 붙여 놓게 하기.
- `on_shutdown` 의 `remove_menu_items` 줄을 지우고, 익스텐션을 끈 뒤 Window 메뉴 보기.
- 강의 47 의 장면(`pl2m_vision.py --gui`)에서 `manager.add_path` 두 줄을 넣어 이 익스텐션을 켜 보기.

## 8. 정리

### 기억할 것

- 익스텐션은 폴더 하나임. 이름표(`extension.toml`)와 파이썬 모듈로 이뤄짐.
- 켜질 때 `on_startup`, 꺼질 때 `on_shutdown` 이 불림. 만든 창과 메뉴는 꺼질 때 치움.
- 버튼이 하는 일은 명령으로 실행함. 그래야 Ctrl+Z 로 되돌릴 수 있음.
- 창의 글자는 영어로 씀. 한글은 `??` 로 나옴.
- 버튼의 함수는 창 없이도 불러 볼 수 있음. 그렇게 시험함.

### 한계

- 버튼을 마우스로 눌러 보지는 않았음. 시험도 화면 캡처도 버튼이 부르는 함수를 직접 불러서 한 것임.
- 한글 글꼴을 넣어 창에 한글을 보이게 하는 것은 하지 않았음.
- 파일을 저장하면 저절로 다시 읽히게 하는 설정은 찾아보지 않았음.
- Kit App Template 으로 앱을 통째로 만드는 것은 다루지 않았음.
