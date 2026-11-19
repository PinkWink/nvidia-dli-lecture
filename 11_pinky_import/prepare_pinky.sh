#!/usr/bin/env bash
# Pinky Pro 의 URDF 와 표정 그림을 준비합니다 (강의 11 ~ 13 에서 사용).
#
#   bash 11_pinky_import/prepare_pinky.sh
#
# 하는 일 (README 1절의 명령과 같습니다)
#   1. 공식 저장소 https://github.com/pinklab-art/pinky_pro 를 ~/pinky_pro/src 에 받는다
#   2. pinky_description 패키지만 빌드한다 (xacro 가 패키지 위치를 찾을 수 있게)
#   3. xacro 파일을 URDF 한 개로 바꿔 ~/nvidia-dli-lecture/work/pinky_description/urdf/pinky.urdf 로 저장한다
#   4. 화면(LCD)에 띄울 표정 GIF 의 첫 장면을 PNG 로 저장한다 (work/pinky_faces/)
set -euo pipefail

ISAACSIM_PATH="${ISAACSIM_PATH:-$HOME/isaacsim}"
WS="$HOME/pinky_pro"
REPO="$WS/src/pinky_pro"
WORK="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/work"

if [ ! -f /opt/ros/jazzy/setup.bash ]; then
  echo "ROS 2 Jazzy 가 없습니다. 강의 7 의 1절(ROS 2 설치)을 먼저 하세요."; exit 1
fi
for tool in git colcon; do
  command -v "$tool" >/dev/null 2>&1 || { echo "$tool 이 없습니다:  sudo apt install -y git python3-colcon-common-extensions"; exit 1; }
done
set +u; source /opt/ros/jazzy/setup.bash; set -u
command -v xacro >/dev/null 2>&1 || { echo "xacro 가 없습니다:  sudo apt install -y ros-jazzy-xacro"; exit 1; }

echo "[1/4] 공식 저장소 받기: $REPO"
mkdir -p "$WS/src"
if [ -d "$REPO/.git" ]; then git -C "$REPO" pull --ff-only; else git clone https://github.com/pinklab-art/pinky_pro.git "$REPO"; fi

echo "[2/4] pinky_description 빌드"
(cd "$WS" && colcon build --packages-select pinky_description)
set +u; source "$WS/install/setup.bash"; set -u

echo "[3/4] xacro → URDF"
mkdir -p "$WORK/pinky_description"
cp -r "$REPO/pinky_description/." "$WORK/pinky_description/"     # 이미 가져온 결과(urdf/pinky/)가 있어도 지우지 않는다
xacro "$REPO/pinky_description/urdf/robot.urdf.xacro" > "$WORK/pinky_description/urdf/pinky.urdf"
echo "  링크 $(grep -c '<link' "$WORK/pinky_description/urdf/pinky.urdf")개, 관절 $(grep -c '<joint' "$WORK/pinky_description/urdf/pinky.urdf")개"

echo "[4/4] 표정 그림 (GIF 첫 장면 → PNG)"
"$ISAACSIM_PATH/python.sh" "$(dirname "${BASH_SOURCE[0]}")/make_faces.py" "$REPO/pinky_emotion/emotion" "$WORK/pinky_faces"

echo
echo "준비 완료: $WORK/pinky_description/urdf/pinky.urdf"
