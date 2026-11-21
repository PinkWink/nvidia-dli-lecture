# 38. 핑크랩 부스 만들기: MuJoCo 장면을 Isaac Sim 으로 옮기기 — 명령 모음

- 영상에서 입력하는 터미널 명령과 기대 출력을 순서대로 모았음. [37. PL-2M 가져오기](../37_pl2m_import/README.md) 다음 내용임.

| 파일 |
|---|
| [make_booth.py](make_booth.py) |

## 1. PL-2M 이 일할 곳

### 로보월드 2026 핑크랩 부스

- 가로 9 m, 세로 6 m 의 전시 부스임. 교실, 상담 테이블, 약국 실증 코너, 인포 데스크가 있음.
- 강의 37 에서 가져온 PL-2M 이 여기서 컵을 나르게 됨. 이번 강의는 그 무대를 만듦.

### 원본은 MuJoCo 장면

- 부스는 저장소 [pinklab_robotworld2026](https://github.com/PinkWink/pinklab_robotworld2026) 의 `src/robotworld` 에 있음. 강의 37 에서 이미 받았음.
- 설계 데이터(`booth_data.json`) → 생성 코드(`world.py`) → MuJoCo 장면 파일(`booth.xml`) 과 벽 그림(PNG) 순서로 만들어짐.
- 이 강의는 마지막 결과인 `booth.xml` 을 읽어 Isaac Sim 장면으로 옮김. 설계 데이터와 생성 코드는 고치지 않음.

### 이번 강의에서 할 일

- MuJoCo 장면 파일에 무엇이 들어 있는지 읽기.
- 도형 · 재질 · 카메라를 하나씩 Isaac Sim 의 Prim 으로 옮기기.
- 전시장답게 조명 넣기.
- 옮긴 부스에 PL-2M 과 컵 세 개를 올리기.

## 2. MuJoCo 장면 파일 읽기

### 터미널

```bash
cd ~/nvidia-dli-lecture
git pull
~/isaacsim/python.sh 38_pl2m_booth/make_booth.py --read-only
```

- 원본의 생성 코드를 불러 `work/booth/booth.xml` 과 벽 그림 31장(`work/booth/textures/`)을 만들고, 내용을 세어 봄.
- 벽의 한글은 나눔 글꼴로 그림. 없으면 `sudo apt install fonts-nanum` (강의 PC 에는 있었음).

### 실행 결과

```text
MuJoCo 장면 파일: /home/pw/nvidia-dli-lecture/work/booth/booth.xml
도형 393개: box 155, capsule 120, sphere 45, cylinder 39, plane 31, mesh 3
그중 라이다 · 카메라에 보이는 것(group 0) 293개, 장식(group 2) 100개
묶음(body) 25개, 재질 61개 (그림 재질 32개), 카메라 7개, 조명 7개
```

- 부스 전체가 단순한 도형 393개로 되어 있음. 벽과 가구는 상자, 사람은 캡슐과 공의 묶음임.
- 벽의 글씨와 사진은 얇은 평면(plane)에 그림을 입힌 것임.

### 도형 하나는 이렇게 적혀 있음

```xml
<geom name="thigh_173" material="col_2b3a55" type="capsule" fromto="-0.05 0 0.52 0.35 0 0.52" size="0.075" group="0" contype="0" conaffinity="0" />
```

- 앉아 있는 손님의 허벅지임. 두 점(`fromto`)을 잇는 반지름 7.5 cm 의 캡슐.
- `group="0"` = 라이다와 카메라에 보이는 것, `material` = 색이나 그림의 이름.

## 3. 옮기는 규칙

### 좌표는 그대로

- MuJoCo 와 Isaac Sim 은 둘 다 위쪽이 Z, 단위가 미터임. 숫자를 바꾸지 않고 그대로 씀.
- 부스는 X 0 ~ 9 m, Y −6 ~ 0 m 에 놓임. 큰 통로가 +X 쪽, 주 출입구가 −Y 쪽임.

### 무엇을 무엇으로

| MuJoCo (MJCF) | Isaac Sim (USD) |
|---|---|
| `geom` box · cylinder · sphere | Cube · Cylinder · Sphere |
| `geom` capsule (`fromto`) | Capsule (두 점을 잇는 방향으로 세움) |
| `geom` plane + 그림 | 네모 Mesh 한 장 + 그림 재질 |
| `geom` mesh | Mesh |
| `body` | Xform |
| `material` (색 · 그림) | OmniPBR 재질 |
| `camera` | Camera |

- 크기 표기가 다른 곳이 있음. MuJoCo 의 상자 `size` 는 **절반 크기**라 두 배를 해서 넣음.

### 충돌체를 붙이는 기준

- `group="0"` 인 도형(291개)에 충돌체(Collider)를 붙임. 벽, 가구, 사람이 여기에 듦.
- 장식(벽 그림, 머리 위 간판)에는 붙이지 않음.
- 원본에서 사람은 "보이지만 부딪히지는 않는" 도형이었음. Isaac Sim 의 PhysX 라이다는 충돌체만 보므로, 사람에게도 충돌체를 붙였음.

### 옮기지 않은 것

- 조명 7개: MuJoCo 는 조명을 8개까지만 쓸 수 있어 원본도 줄여 넣은 것임. 옮기지 않고 5절에서 새로 넣음.
- 전시장 바닥 1개: 기본 바닥이 대신함.

## 4. 옮기기

### 터미널

```bash
~/isaacsim/python.sh 38_pl2m_booth/make_booth.py
```

- 부스를 만들어 `work/booth/booth.usd` 로 저장하고, 설계자가 정해 둔 시점 7개로 사진을 찍음.
- `--plain` 을 붙이면 5절의 조명 없이 기본 조명(강의 3)으로 만듦.

### 실행 결과

```text
옮긴 도형: box 155, capsule 120, sphere 45, cylinder 39, plane 30, mesh 3, 옮기지 않음 1
재질 61개, 카메라 7개, 충돌체 291개
조명 30개 (등기구 12 · 위의 큰 조명 1 · 안쪽 조명 7 · 분홍빛 5 · LED 줄 5)
장면 저장: /home/pw/nvidia-dli-lecture/work/booth/booth.usd
사진 저장: /home/pw/nvidia-dli-lecture/work/booth/photos/booth_bird1.png
...
사진 저장: /home/pw/nvidia-dli-lecture/work/booth/photos/booth_top.png
```

- 도형 393개 중 392개를 옮겼음.

### MuJoCo 와 나란히

- 같은 카메라로 찍은 것임. 벽, 가구, 사람의 자리가 같음.
- 형상만 비교하려고 Isaac Sim 쪽은 기본 조명(`--plain`)으로 찍었음.

### 교실 쪽

- 벽의 글씨와 그림이 뒤집히지 않고 제 방향으로 붙었음.

### 위에서

## 5. 전시장 조명

### 기본 조명과 전시장 조명

- 왼쪽은 강의 3 부터 써 온 해 + 주변광임. 고르게 밝지만 평평해 보임.
- 오른쪽은 홀을 어둡게 하고 부스의 조명을 따로 넣은 것임. 이 강의의 기본값임.

### 넣은 조명

| 조명 | 수 | 종류 | 하는 일 |
|---|---|---|---|
| 등기구 | 12 | Sphere Light + Cone | 설계의 등기구 자리에서 간판과 벽을 비춤 |
| 위의 큰 조명 | 1 | Sphere Light + Cone | 9 m 위에서 부스 전체를 비춤 |
| 안쪽 조명 | 7 | Sphere Light + Cone | 부스 안을 위에서 비춤 |
| 분홍빛 | 5 | Sphere Light + Cone | 물결 간판 아래로 떨어지는 분홍빛 |
| LED 줄 | 5 | Cylinder Light | 정면 띠 아래, 기둥 모서리, 인포 데스크 아래 |

- 등기구 12개의 자리는 설계 데이터에 있는 것임 (정면 3 · 뒷벽 4 · 물결 간판 5).
- 나머지 조명의 자리와 밝기, 색은 이 강의에서 정한 값임.

### 정면에서

- 간판 위의 등기구가 만든 빛의 원, 간판 아래의 LED 띠, 기둥 모서리의 LED 줄이 보임.
- 어둡고 매끈한 전시장 바닥에 부스의 불빛이 비침.

### 한쪽으로만 비추는 조명

```python
light = UsdLux.SphereLight.Define(stage, path)
light.CreateRadiusAttr(radius)
light.CreateNormalizeAttr(True)                 # 밝기가 조명의 크기에 따라 달라지지 않게
light.CreateIntensityAttr(intensity)
light.CreateColorAttr(Gf.Vec3f(*color))
shaping = UsdLux.ShapingAPI.Apply(light.GetPrim())
shaping.CreateShapingConeAngleAttr(cone)
shaping.CreateShapingConeSoftnessAttr(0.4)
```

- Create > Light > Sphere Light 를 만들고 Property 의 Shaping > Cone 에 각도를 넣는 것과 같음.
- Cone 의 각도 밖으로는 빛이 나가지 않아 스포트라이트가 됨. Softness 는 가장자리를 부드럽게 함.

### LED 줄

```python
light = UsdLux.CylinderLight.Define(stage, f"{BOOTH}/Lights/{name}")
light.CreateLengthAttr(length)
light.CreateRadiusAttr(radius)
light.CreateIntensityAttr(intensity)
light.CreateColorAttr(Gf.Vec3f(*color))
```

- Cylinder Light 는 긴 막대 모양의 조명임. 주변을 비추는 일을 맡음 (일부만 보였음).
- 조명은 카메라에 보이지 않아, 같은 자리에 스스로 빛나는 재질의 가는 기둥을 하나 더 둠.

### 바닥과 화면 효과

- 전시장 바닥과 부스 바닥판의 재질을 매끈하게 함 (거칠기 0.22 · 0.25). 조명이 바닥에 비침.
- 밝은 곳이 번져 보이는 효과(Bloom)와 반사를 켬. 스크립트로 띄울 때 적용되는 화면 설정임.

## 6. 코드 읽기

### 상자

```python
if kind == "box":
    prim = UsdGeom.Cube.Define(stage, path).GetPrim()
    UsdGeom.Cube(prim).CreateSizeAttr(1.0)
    place(prim, position, rotation, scale=[2 * v for v in size])          # MJCF 의 size 는 절반 크기
```

- `38_pl2m_booth/make_booth.py` 의 `add_geom` 임 (일부만 보였음).
- 한 변이 1 인 Cube 를 만들고 Scale 로 크기를 줌. Create > Shape > Cube 뒤 Scale 을 넣는 것과 같음.

### 두 점을 잇는 캡슐

```python
if geom.get("fromto"):                            # 두 점을 잇는 기둥 · 캡슐
    ends = np.array(numbers(geom.get("fromto")))
    start, end = ends[:3], ends[3:]
    position, rotation, length = (start + end) / 2, rotation_to(end - start), float(np.linalg.norm(end - start))
```

- 위치 = 두 점의 가운데, 길이 = 두 점 사이 거리, 방향 = Z 축이 두 점을 잇는 쪽을 향하게.
- 사람의 팔 · 다리 120개가 이렇게 놓임.

### 그림을 붙인 평면

```python
mesh = UsdGeom.Mesh.Define(stage, path)
mesh.CreatePointsAttr([(-half_x, -half_y, 0), (half_x, -half_y, 0), (half_x, half_y, 0), (-half_x, half_y, 0)])
mesh.CreateFaceVertexCountsAttr([4])
mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
st = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex)
st.Set([(0, 0), (1, 0), (1, 1), (0, 1)])
```

- 점 네 개로 된 네모 한 장임 (일부만 보였음). `st` 는 "그림의 어느 구석이 어느 점에 오는지" 를 적는 값.
- 평면의 방향은 MJCF 의 `xyaxes`(가로 · 세로 방향)를 회전으로 바꿔 줌.

### 충돌체

```python
if geom.get("group") == "0" and kind not in ("plane", "mesh"):      # 라이다 · 카메라에 보이는 것 → 충돌체
    UsdPhysics.CollisionAPI.Apply(prim)
```

- 3절의 기준을 한 줄로 적은 것임.

## 7. PL-2M 과 컵 올리기

### 터미널

```bash
~/isaacsim/python.sh 38_pl2m_booth/make_booth.py --robot --gui
```

- 강의 37 의 `import_pl2m` 으로 로봇을 가져와 상담 테이블 둘 사이 (5.3, −3.2) 에 놓음.
- 음료대 위에 컵 세 개를 놓음. 지름 5 cm, 높이 9 cm, 200 g 의 원기둥임. 무게는 내용물이 든 컵으로 보고 가정한 값이고, 원본은 100 g 임.
- 로봇 자리와 컵 자리는 원본의 로봇용 설정(`worlds/booth.yaml`)의 값임.

### 창으로 본 모습

- 뷰포트 가운데 상담 테이블 사이에 PL-2M 이 서 있음.
- Stage 에 `Booth`(부스), `pl2m`(로봇), `Cups`(컵)가 따로 들어 있음.

### 로봇 가까이에서

### 음료대의 컵

- 뒤 강의에서 로봇이 집어 상담 테이블로 나를 물체임.

### 실행 결과

```text
   시간[s]     로봇 x        y        z    방향[°]   컵 높이[m]
     0.0    5.300   -3.200    0.107      0.0   0.795 0.795 0.795
     1.0    5.300   -3.200    0.107      0.0   0.795 0.795 0.795
...
     5.0    5.300   -3.200    0.107      0.0   0.795 0.795 0.795
결과: 5초 동안 로봇 이동 0.0 mm, 회전 +0.00°.
```

- 로봇은 부스 바닥판(두께 2 cm) 위에 서서 `base_link` 높이가 0.107 m 임 (강의 37 의 0.087 + 0.02).
- 컵은 음료대(높이 0.75 m) 위에 놓여 가운데 높이가 0.795 m 임. 5초 동안 움직이지 않았음.

## 8. 해 볼 것

### 직접 바꿔 보기

- `--gui` 로 띄워 Stage 의 `Booth > Cameras` 에서 카메라를 골라 설계 시점으로 보기.
- `Booth > People` 의 사람 하나를 골라 Translate 로 옮겨 보기.
- `SPAWN` 값을 바꿔 로봇을 음료대 앞 (7.65, −2.82) 에 놓아 보기.
- `LAMP` · `KEY` · `DOWN` 값을 바꿔 조명의 밝기를 바꿔 보기. `PINK` 를 다른 색으로 바꿔 보기.

## 9. 정리

### 기억할 것

- MuJoCo 와 Isaac Sim 은 좌표계와 단위가 같아, 장면 파일의 숫자를 그대로 옮길 수 있음.
- 도형은 종류별로 대응하는 Prim 이 있음. 크기 표기(절반 크기)와 방향 표기만 바꿔 줌.
- 충돌체는 "로봇이 부딪히거나 라이다로 봐야 하는 것" 에만 붙임.
- 조명은 한 가지로 비추지 않음. 전체는 어둡게, 보여 줄 곳은 스포트라이트로, 선은 LED 줄로 나눠 넣음.
- 부스(장면)와 로봇, 물체는 Stage 에서 따로 둠. 부스만 저장하면 62 KB, 로봇까지 넣으면 112 MB 임.

### 한계

- 조명의 밝기와 색은 사진을 보며 맞춘 값임. 실제 부스의 조명과 비교하지 않았음.
- Bloom 과 반사 설정은 이 스크립트로 띄울 때만 적용됨. 저장한 `booth.usd` 를 File > Open 으로 열었을 때의 모습은 확인하지 않았음.
- 조명이 30개로 늘어난 뒤의 화면 속도는 재지 않았음.
- 로봇은 서 있기만 했음. 부스 안에서 주행하거나 라이다로 벽을 보는 것은 확인하지 않았음.
- 부스 바닥판 가장자리에는 2 cm 의 턱이 있음. 로봇이 넘을 수 있는지는 확인하지 않았음.
- 사람은 움직이지 않는 도형 묶음임.
- Isaac Sim 에 들어 있는 MJCF Importer 는 써 보지 않았음. 이 강의는 장면 파일을 직접 읽어 옮겼음.
- 강의 PC 한 대(Isaac Sim 5.1)에서 확인한 결과임.
