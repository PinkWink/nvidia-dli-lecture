# 50. USD 장면을 파일 여러 개로: 참조 · payload · Variant · 레이어 — 명령 모음

- 영상에서 입력하는 터미널 명령과 기대 출력을 순서대로 모았음. [49. TensorRT 로 모델 빠르게 돌리기](../49_tensorrt/README.md) 다음 내용임.

| 파일 |
|---|
| [make_layers.py](make_layers.py) |
| [show_layers.py](show_layers.py) |

## 1. 장면은 파일 하나가 아님

### 이번 강의의 결과

- 부스, 로봇, 컵이 든 장면임. 여는 파일은 156바이트짜리 `scene.usda` 하나인데, 실제 내용은 다른 파일 다섯 개에 나뉘어 있음.
- 이 강의는 그 파일들을 만들고, 파일 하나를 끄거나 바꾸면 장면이 어떻게 달라지는지를 봄.

### 지금까지의 방식

| | 지금까지 (강의 38 ~ 49) | 이 강의 |
|---|---|---|
| 장면 만들기 | 실행할 때마다 파이썬이 Prim 을 하나씩 만듦 | 저장해 둔 USD 파일들을 불러와 합침 |
| 컵 세 개 | 같은 코드를 세 번 돌림 | 컵 파일 하나를 세 번 참조함 |
| 배치를 바꿔 볼 때 | 코드를 고치거나 옵션을 줌 | 덮어쓸 것만 적은 파일을 위에 쌓음 |
| 되돌리기 | 코드를 되돌림 | 그 파일을 끔 |

- 코드로 만드는 방식은 무엇이 어떻게 생기는지 다 보여서 배우기에 좋았음. 여러 사람이 한 장면을 나눠 고치기에는 파일로 나누는 쪽이 맞음.
- USD 는 처음부터 그렇게 쓰라고 만든 형식임. 파일을 잇는 방법이 넷 있고, 이 강의에서 넷을 다 씀.

### 파일 여섯 개

| 잇는 방법 | 뜻 | 이 강의에서 |
|---|---|---|
| 서브레이어 | 파일을 통째로 겹쳐 쌓음. 위의 것이 이김 | `scene.usda` 가 두 파일을 쌓음 |
| 참조 | 다른 파일의 Prim 을 이 자리에 가져옴 | 컵 |
| payload | 참조와 같지만 불러오기를 미룰 수 있음 | 부스, 로봇 |
| Variant | 한 Prim 안에 든 여러 모습 가운데 하나를 고름 | 컵의 색 |

- 회색 상자 둘은 앞 강의에서 이미 만든 파일임. 나머지 넷을 이 강의에서 만듦.

### 만드는 것

| 파일 | 하는 일 |
|---|---|
| `make_layers.py` | 파일 넷을 만듦 |
| `show_layers.py` | 합쳐진 장면을 열어, 값이 어느 파일에서 왔는지 보여 줌. 레이어를 끄고 Variant 를 바꿈 |

## 2. 파일 만들기

### 터미널: 재료 둘

```bash
cd ~/nvidia-dli-lecture
git pull
~/isaacsim/python.sh 38_pl2m_booth/make_booth.py --no-photo
~/isaacsim/python.sh 37_pl2m_import/import_pl2m.py --save work/pl2m/pl2m_scene.usd --seconds 1
```

- 강의 38 과 37 의 스크립트가 원래 가진 저장 기능임. 부스는 `work/booth/booth.usd`, 로봇은 `work/pl2m/pl2m_scene.usd` 로 저장됨.

### 터미널: 파일 넷

```bash
~/isaacsim/python.sh 50_usd_layers/make_layers.py
```

```text
만든 곳: /home/내이름/nvidia-dli-lecture/work/booth_layers
  cup.usda                  1038 바이트
  layout.usda               1277 바이트
  rearrange.usda             863 바이트
  scene.usda                 156 바이트
  (불러오는 부스)         62225 바이트  ../booth/booth.usd
  (불러오는 로봇)     112418461 바이트  ../pl2m/pl2m_scene.usd
```

- 새로 만든 넷은 다 합쳐 3.3 KB 임. 글자로 된 USD (`.usda`) 라 편집기로 열어 읽을 수 있음.
- `--show cup.usda` 를 붙이면 그 파일의 내용을 화면에 냄.

## 3. 컵: 참조와 Variant

### 파일: cup.usda

```text
def Xform "Cup" (
    prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI"]
    kind = "component"
    variants = {
        string color = "pink"
    }
    prepend variantSets = "color"
)
{
    float physics:mass = 0.2

    def Cylinder "body" (
        prepend apiSchemas = ["PhysicsCollisionAPI"]
    )
    {
        uniform token axis = "Z"
        double height = 0.09
        double radius = 0.025
    }
    variantSet "color" = {
        "blue" {
            over "body"
            {
                color3f[] primvars:displayColor = [(0.06077818, 0.12599176, 0.42807475)]
            }
        }
        ...
```

- 컵 하나의 모든 것이 이 파일에 있음. 모양(원기둥), 물리(강체 · 질량 0.2 kg · 충돌체), 색임.
- 색은 `variantSet "color"` 안에 셋이 들어 있음. 그 가운데 하나만 보임. 고르지 않으면 `pink` 임.
- `over` 는 "이미 있는 Prim 의 값을 덮어씀" 이라는 뜻임. `def` 는 새로 만드는 것이고 `over` 는 고치는 것임.

### 코드: Variant 만들기

```python
colors = cup.GetPrim().GetVariantSets().AddVariantSet("color")
for name, color in COLORS.items():
    colors.AddVariant(name)
    colors.SetVariantSelection(name)
    with colors.GetVariantEditContext():                        # 이 안에서 적는 것은 그 Variant 를 골랐을 때만 보인다
        body.CreateDisplayColorAttr([Gf.Vec3f(*[c ** 2.2 for c in color])])
colors.SetVariantSelection("pink")
```

- Variant 를 하나 고른 뒤 `GetVariantEditContext` 안에서 값을 적으면, 그 값은 그 Variant 안에 들어감.
- 색에 2.2 제곱을 한 것은 강의 38 과 같음. 화면에 보일 색을 렌더러가 쓰는 값으로 바꾼 것임.

### 파일: layout.usda 의 컵

```text
def Xform "cup_blue" (
    prepend references = @./cup.usda@
    variants = {
        string color = "blue"
    }
)
{
    double3 xformOp:translate = (7.65, -1.72, 0.795)
    uniform token[] xformOpOrder = ["xformOp:translate"]
}
```

- 쓰는 쪽은 세 가지만 적음. 어느 파일을 가져올지(`references`), 어느 색을 고를지(`variants`), 어디에 놓을지(`translate`).
- 컵의 반지름이나 질량을 바꾸려면 `cup.usda` 한 곳만 고치면 됨. 세 컵이 함께 바뀜.

### 터미널: 색 바꿔 보기

```bash
~/isaacsim/python.sh 50_usd_layers/show_layers.py --mute rearrange.usda --color cup_blue=pink cup_yellow=pink --photo variant.png
```

```text
Variant 를 바꿈: cup_blue=pink, cup_yellow=pink  (열어 둔 장면에만. 저장하지 않으면 파일은 그대로)
  cup_pink    색 pink    자리 (7.50, -1.72, 0.795)   자리를 적은 파일: layout.usda
  cup_blue    색 pink    자리 (7.65, -1.72, 0.795)   자리를 적은 파일: layout.usda
  cup_yellow  색 pink    자리 (7.80, -1.72, 0.795)   자리를 적은 파일: layout.usda
```

- `--mute rearrange.usda` 는 4절의 덮어쓰는 파일을 끄는 것임. 여기서는 원래 배치에서 색만 보려고 붙였음.
- 색을 바꾸는 데 새 재질도 새 파일도 필요 없음. 컵 안에 이미 든 모습 가운데 다른 것을 고른 것임.

## 4. 덮어쓰는 파일: 서브레이어

### 파일: rearrange.usda

```text
over "World"
{
    over "Cups"
    {
        over "cup_blue" (
            variants = {
                string color = "yellow"
            }
        )
        {
            double3 xformOp:translate = (7.82, -1.72, 0.795)
        }
        ...
    }

    over "Site"
    {
        over "Booth"
        {
            over "People"
            {
                over "staff_5" (
                    active = false
                )
```

- 바꿔 보는 안을 적은 파일임. 컵을 음료대 오른쪽에 모으고, 가운데 컵을 노란색으로 바꾸고, 음료대 옆 사람을 치움. 전부 `over` 라 새로 만드는 것은 없음.
- `active = false` 는 지우지 않고 끄는 것임. 꺼진 Prim 은 장면에 없는 것처럼 다뤄짐.
- 이 사람 모형은 강의 48 에서 로봇이 오른쪽으로 비켜 서지 못하게 막던 것임.

### 파일: scene.usda

```text
#usda 1.0
(
    defaultPrim = "World"
    metersPerUnit = 1
    subLayers = [
        @./rearrange.usda@,
        @./layout.usda@
    ]
    upAxis = "Z"
)
```

- 이 파일에는 Prim 이 하나도 없음. 두 파일을 쌓는다는 말뿐임. 앞에 적은 것이 위임.
- 같은 값을 두 파일이 적었으면 위의 것이 보임.

### 터미널: 열어서 보기

```bash
~/isaacsim/python.sh 50_usd_layers/show_layers.py
```

```text
연 파일: scene.usda  (0.18초)
레이어 (센 것부터): scene.usda > rearrange.usda > layout.usda
Prim 770개
  cup_pink    색 pink    자리 (7.74, -1.72, 0.795)   자리를 적은 파일: rearrange.usda > layout.usda
  cup_blue    색 yellow  자리 (7.82, -1.72, 0.795)   자리를 적은 파일: rearrange.usda > layout.usda
  cup_yellow  색 yellow  자리 (7.90, -1.72, 0.795)   자리를 적은 파일: rearrange.usda > layout.usda
  음료대 옆 사람(staff_5): 꺼져 있음 (active = false)
```

- 컵의 자리를 적은 파일이 둘임. 앞의 것(`rearrange.usda`)이 이겨서 그 값이 보임.
- `layout.usda` 에 적힌 원래 값은 지워지지 않고 아래에 그대로 있음.

### 터미널: 덮어쓰는 파일 끄기

```bash
~/isaacsim/python.sh 50_usd_layers/show_layers.py --mute rearrange.usda
```

```text
레이어를 끔: rearrange.usda
  cup_pink    색 pink    자리 (7.50, -1.72, 0.795)   자리를 적은 파일: layout.usda
  cup_blue    색 blue    자리 (7.65, -1.72, 0.795)   자리를 적은 파일: layout.usda
  cup_yellow  색 yellow  자리 (7.80, -1.72, 0.795)   자리를 적은 파일: layout.usda
  음료대 옆 사람(staff_5): 있음
```

- 끄면 원래 배치가 나옴. 파일은 그대로 두고, 이 장면에서만 없는 것으로 친 것임.
- 바꿔 보는 안을 여러 개 만들어 두고 하나씩 켜서 견줄 수 있음. 원본은 한 글자도 바뀌지 않음.

### 코드: 값이 어디서 왔나

```python
place = cup.GetAttribute("xformOp:translate")
opinions = " > ".join(short(spec.layer) for spec in place.GetPropertyStack(Usd.TimeCode.Default()))
```

- `GetPropertyStack` 은 그 값을 적은 파일들을 센 순서로 돌려줌. 맨 앞이 지금 보이는 값을 준 파일임.
- 값이 예상과 다를 때 가장 먼저 볼 것임. 다른 파일이 위에서 덮어쓰고 있는지가 바로 나옴.

## 5. 부스와 로봇: payload

### 파일: layout.usda 의 부스와 로봇

```text
def Xform "Site" (
    prepend payload = @../booth/booth.usd@
)
{
}

def Xform "pl2m" (
    prepend payload = @../pl2m/pl2m_scene.usd@</World/pl2m>
)
{
    double3 xformOp:translate = (5.3, -3.2, 0.05)
}
```

- 부스 파일은 통째로 `/World/Site` 아래에 옴. 조명, 바닥, 재질이 함께 딸려 옴.
- 로봇 파일에서는 `</World/pl2m>` 만 가져옴. 그 파일에 든 바닥과 조명은 가져오지 않음.
- 로봇의 자리는 여기서 덮어씀. 로봇 파일에 적힌 자리는 무시됨.

### 터미널: 불러오지 않고 열기

```bash
~/isaacsim/python.sh 50_usd_layers/show_layers.py --no-payload
```

```text
연 파일: scene.usda  (0.08초)
Prim 19개 (payload 를 불러오지 않음)
  cup_pink    색 pink    자리 (7.74, -1.72, 0.795)   자리를 적은 파일: rearrange.usda > layout.usda
```

| | payload 를 불러옴 | 불러오지 않음 |
|---|---|---|
| Prim | 770개 | 19개 |
| 여는 데 | 0.18초 | 0.08초 |

- payload 로 건 것은 열 때 불러올지 말지를 고를 수 있음. 참조로 건 컵은 늘 불러옴.
- 불러오지 않아도 컵의 자리와 색은 읽고 고칠 수 있음. 배치만 손볼 때 부스와 로봇을 기다릴 필요가 없음.
- 여는 시간은 둘 다 짧았음. 112 MB 인 로봇 파일도 여는 데는 0.1초임. USD 는 큰 배열을 쓸 때 읽기 때문임. 화면에 그려지기까지의 시간은 재지 않았음.

### 고치는 동안 본 실패

| | 처음 짠 것 | 고친 것 |
|---|---|---|
| 부스의 payload 를 건 곳 | `/World` | `/World/Site` |
| 불러오지 않고 열면 | 컵까지 보이지 않음 | 컵은 보임 |

- 처음에는 부스 파일을 `/World` 에 바로 걸었음. 그랬더니 payload 를 끄자 `/World` 아래의 컵까지 사라졌음.
- payload 를 불러오지 않은 Prim 은 그 아래 전부가 불러오지 않은 것으로 다뤄짐. 미룰 것은 따로 떼어 둔 Prim 에 걸어야 함.

## 6. Isaac Sim 창에서

### 터미널: 창으로 열기

```bash
~/isaacsim/python.sh 50_usd_layers/show_layers.py --gui
```

- Stage 창에는 합쳐진 결과가 보임. 어느 파일에서 왔는지와 상관없이 한 나무로 나옴.
- 다른 파일에서 가져온 Prim 은 아이콘에 주황색 화살표가 붙음. `Site`, `pl2m`, 컵 셋이 그러함.

### Layer 창

- Stage 창 옆의 Layer 탭임. 쌓인 순서대로 `rearrange.usda`, `layout.usda` 가 보임.
- 파일 이름 오른쪽의 눈 모양을 누르면 그 레이어가 꺼짐. 4절의 `--mute` 와 같은 일임.
- 스크립트로 띄운 Isaac Sim 에는 이 창이 기본으로 켜져 있지 않음. `show_layers.py --gui` 가 `omni.kit.widget.layers` 를 켜서 띄움.

## 7. 물리도 그대로

### 터미널: 3초 돌려 보기

```bash
~/isaacsim/python.sh 50_usd_layers/show_layers.py --play 3
```

```text
3초 뒤 컵의 높이: cup_pink 0.795 m,  cup_blue 0.795 m,  cup_yellow 0.795 m
```

- 컵이 음료대 위에 그대로 서 있음. 참조로 가져온 컵의 강체와 충돌체, payload 로 가져온 음료대의 충돌체가 한 장면에서 함께 돎.
- 이 장면의 로봇은 강의 37 에서 가져온 그대로임. 강의 39 · 40 의 구동 값과 마찰은 코드로 넣는 것이라 이 파일에는 없음.

## 8. 해 볼 것

### 직접 바꿔 보기

- `work/booth_layers/rearrange.usda` 를 편집기로 열어 컵의 자리를 고치고 다시 열어 보기.
- `rearrange.usda` 를 복사해 다른 안을 만들고, `scene.usda` 의 `subLayers` 맨 위에 더해 보기.
- `cup.usda` 의 `radius` 를 0.035 로 고쳐 세 컵이 함께 굵어지는 것 보기.
- `make_layers.py` 의 `COLORS` 에 색을 하나 더하고 `--color cup_pink=그 이름` 으로 고르기.

## 9. 정리

### 기억할 것

- USD 장면은 파일 여러 개를 합친 결과임. 잇는 방법은 서브레이어, 참조, payload, Variant 임.
- 같은 것을 여러 번 쓸 때는 참조함. 부품 파일 하나를 고치면 쓰는 곳이 다 바뀜.
- 바꿔 보는 것은 원본을 고치지 않고 위에 쌓음. 끄면 원래대로 돌아옴.
- 미뤄도 되는 큰 것은 payload 로, 따로 떼어 둔 Prim 에 걺.
- 값이 이상하면 그 값을 적은 파일들을 먼저 봄 (`GetPropertyStack`).

### 한계

- 인스턴싱(같은 부품 수백 개를 메모리에서 한 벌만 두는 것)은 다루지 않았음.
- 화면에 그려지기까지의 시간과 메모리는 재지 않았음. payload 의 이득을 숫자로 보인 것은 Prim 의 수뿐임.
- 앞 강의의 스크립트들은 여전히 코드로 장면을 만듦. 이 파일들로 바꿔 돌리는 것은 하지 않았음.
- 로봇의 구동 값 · 마찰 · 손가락 패드는 이 파일에 없음. 그것들을 레이어로 옮기는 것은 해 보지 않았음.
