# 13. Pinky Pro 화면(LCD)과 LED 바 — 명령 모음

- 영상에서 입력하는 터미널 명령과 화면에 넣는 값을 순서대로 모았음. [12. Pinky Pro 센서](../12_pinky_sensors/README.md) 다음 내용임.

- 표정 그림은 강의 11 의 1절에서 만든 `~/nvidia-dli-lecture/work/pinky_faces/` 의 PNG 를 씀 (angry · basic · bored · fun · happy · hello · interest · sad).

## 1. 준비

```bash
cd ~/isaacsim
./isaac-sim.sh
```

- `File > Open` → `~/nvidia-dli-lecture/work/pinky_gui.usd`

## 2. 화면에서 누르는 순서

### 화면 (LCD)

| 순서 | 동작 | 값 |
|---|---|---|
| 1 | `pinky > screen_mount` 우클릭 → `Create > Mesh > Plane` | |
| 2 | `Plane` 더블클릭 → 이름 | `LCD` |
| 3 | Translate | `0.0173`, `0`, `-0.0035` |
| 4 | Orient | `90`, `90`, `0` |
| 5 | Scale | `0.052`, `0.039`, `1` |
| 6 | `LCD` 를 선택한 채 `Create > Materials > OmniPBR` | `World > Looks > OmniPBR` 이 생김 |
| 7 | `OmniPBR` 더블클릭 → 이름 | `LCD_Material` |
| 8 | `LCD_Material` 선택 → Property 의 `Albedo` 펼치기 → Albedo Map | 아래 경로 → Enter |
| 9 | `Emissive` 펼치기 → Enable Emission 체크 | |
| 10 | Emissive Color map | 같은 경로 → Enter |
| 11 | Emissive Color 의 R · G · B (숫자 칸을 `Ctrl` + 클릭하고 입력) | `1`, `1`, `1` |

- 그림 경로 (`내이름` 은 내 사용자 이름으로):

```text
/home/내이름/nvidia-dli-lecture/work/pinky_faces/basic.png
```

### LED 바

| 순서 | 동작 | 값 |
|---|---|---|
| 12 | `pinky > robot_lamp > visuals` 클릭 | |
| 13 | `Create > Materials > OmniPBR` → 새 `OmniPBR` 의 이름 | `LED_Material` |
| 14 | 다시 `visuals` 선택 → Property 의 Materials on selected models → Strength | `Stronger than Descendants` |
| 15 | `LED_Material` 선택 → `Emissive` 펼치기 → Enable Emission 체크 | |
| 16 | Emissive Color 의 R · G · B (`Ctrl` + 클릭) | `0`, `1`, `0.2` |
| 17 | Emissive Intensity | `2000` |
| 18 | 색 바꾸기: Emissive Color | `1`, `0`, `0` (빨강) |
| 19 | ▶ Play → ■ Stop → `Ctrl` + `S` | |

## 3. 같은 일을 코드로

- Isaac Sim 창을 닫은 뒤 실행함.

```bash
cd ~/nvidia-dli-lecture
git pull
~/isaacsim/python.sh 13_pinky_lcd_led/pinky_face_led.py
```

```text
화면: /World/pinky/screen_mount/LCD — 가로 52 mm × 세로 39 mm, 표정 basic
LED : /World/pinky/robot_lamp/visuals ← /World/Looks/LED_Material, 색 (0, 1, 0.2)
사진 저장: /home/pw/nvidia-dli-lecture/work/pinky_face.png
결과: 화면에 표정을 띄우고 LED 바를 켰습니다.
```

- 로봇 사진이 `work/pinky_face.png` 로 저장됨.

## 4. 바꿔 보기

```bash
~/isaacsim/python.sh 13_pinky_lcd_led/pinky_face_led.py --face happy --led 1 0 0
~/isaacsim/python.sh 13_pinky_lcd_led/pinky_face_led.py --led 0 0 0
~/isaacsim/python.sh 13_pinky_lcd_led/pinky_face_led.py --show
~/isaacsim/python.sh 13_pinky_lcd_led/pinky_face_led.py --keyboard
```

| 옵션 | 결과 |
|---|---|
| `--face 이름` | 표정 바꾸기 (angry · basic · bored · fun · happy · hello · interest · sad) |
| `--led R G B` | LED 색 (0 ~ 1). `0 0 0` 이면 끔 |
| `--show` | 창을 띄우고 표정 8개와 LED 색을 2초마다 바꿈 |
| `--keyboard` | 창을 띄우고 W A S D 로 조종 |

## 막힐 때

| 증상 | 해결 |
|---|---|
| `표정 그림이 없습니다` | 강의 11 의 1절(준비)을 먼저 실행 |
| 화면이 붉게 보임 | Emissive Color 가 기본값 (1, 0.1, 0.1) → (1, 1, 1) 로 |
| 표정이 옆으로 누움 | `LCD` 의 Orient 가 (`90`, `90`, `0`) 인지 확인 |
| 숫자 칸에 입력이 안 됨 | 색 칸은 슬라이더라 `Ctrl` + 클릭해야 숫자를 직접 넣을 수 있음 |
| LED 색이 안 바뀜 | `visuals` 의 Strength 가 `Stronger than Descendants` 인지 확인 |

## 옵션 전체

| 옵션 | 뜻 |
|---|---|
| `--face 이름` | 표정 (기본 `basic`) |
| `--led R G B` | LED 색 (기본 `0 1 0.2`) |
| `--image 경로` | 로봇 사진 저장 경로 (기본 `work/pinky_face.png`) |
| `--show` | 표정 · LED 색을 차례로 바꿈 (창) |
| `--keyboard` | 창을 띄우고 W A S D 로 조종 |
| `--gui` | 창을 띄움 |
| `--save USD` | Play 전 장면을 저장 |
