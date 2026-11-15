# 2. Isaac Sim 실행과 화면 구성 — 명령 모음

영상에서 입력하는 터미널 명령을 순서대로 모았습니다. [1. 설치](../01_install/README.md) 를 마친 상태에서 시작합니다.

## 1. Isaac Sim 실행

```bash
cd ~/isaacsim
./isaac-sim.sh
```

- 첫 실행은 셰이더 캐시를 만드느라 5~10분 걸립니다. 창이 회색이어도 기다립니다. "응답 없음" 창이 뜨면 `기다리기(Wait)` 를 누릅니다.
- 가운데 화면에 격자와 좌표축이 보이면 성공입니다.
- 실행한 터미널을 닫으면 Isaac Sim 도 꺼집니다.
- 끄기: 창 닫기 또는 터미널에서 `Ctrl` + `C`.

## 2. 실행 파일 세 가지

| 파일 | 하는 일 |
|---|---|
| `~/isaacsim/isaac-sim.sh` | 화면(GUI)을 바로 띄움 |
| `~/isaacsim/isaac-sim.selector.sh` | 실행할 앱과 옵션을 고르는 창을 띄움 |
| `~/isaacsim/python.sh` | Isaac Sim 에 들어 있는 Python 으로 스크립트를 실행 |

## 3. 뷰포트 이동

| 동작 | 방법 |
|---|---|
| 앞뒤 · 좌우 이동 | 마우스 오른쪽 버튼 누른 채 W · A · S · D |
| 위아래 이동 | 마우스 오른쪽 버튼 누른 채 Q · E |
| 둘러보기 | 마우스 오른쪽 버튼 누른 채 끌기 |
| 확대 · 축소 | 마우스 휠 |
| 평행 이동 | 가운데 버튼(휠) 누른 채 끌기 |
| 선택한 물체로 | F 키 |

## 4. Python 으로 켜기

Isaac Sim 창을 닫은 뒤 실행합니다.

```bash
cd ~/nvidia-dli-lecture
~/isaacsim/python.sh 02_launch/hello_isaacsim.py
```

```text
Isaac Sim 버전 : 5.1.0
Kit 버전       : 107.3.3+production.229672.69cbf6ad.gl
위쪽 축        : Z
길이 단위      : 1.0 m
```

앞뒤로 `[Warning]` 등 많은 로그가 함께 나옵니다. 정상입니다.

### 창을 띄워서 켜기

```bash
~/isaacsim/python.sh 02_launch/hello_isaacsim.py --gui
```

창을 닫으면 스크립트도 끝납니다.

## 막힐 때

| 증상 | 해결 |
|---|---|
| 화면 배치 · 설정이 꼬임 | `cd ~/isaacsim && ./isaac-sim.sh --reset-user` |
| 켜지다가 꺼짐 · 화면이 깨짐 | `cd ~/isaacsim && ./clear_caches.sh` 후 다시 실행 |
| `ModuleNotFoundError: No module named 'isaacsim'` | `python3` 가 아니라 `~/isaacsim/python.sh` 로 실행 |

다음: [3. 첫 장면](../03_first_scene/README.md)
