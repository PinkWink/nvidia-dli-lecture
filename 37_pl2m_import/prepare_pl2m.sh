#!/usr/bin/env bash
# PL-2M (이동형 양팔 로봇) 의 URDF 를 준비합니다 (강의 37 부터 사용).
#
#   bash 37_pl2m_import/prepare_pl2m.sh
#
# 하는 일 (README 2절의 명령과 같습니다)
#   1. 저장소 https://github.com/PinkWink/pinklab_robotworld2026 를 ~/pinklab_robotworld2026 에 받는다
#   2. 로봇 설계 패키지 세 개만 빌드한다 (xacro 가 패키지 위치를 찾을 수 있게)
#        vicpinky_description (바퀴 달린 베이스) · openarm_description (양팔) · mobile_openarm_description (둘을 합친 PL-2M)
#   3. xacro 파일을 URDF 한 개로 바꿔 ~/nvidia-dli-lecture/work/pl2m/pl2m.urdf 로 저장한다
#      (모양 파일의 위치 package://... 는 받은 저장소의 실제 경로로 바꿔 적는다)
set -euo pipefail

WS="$HOME/pinklab_robotworld2026"
WORK="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/work/pl2m"
PACKAGES="vicpinky_description openarm_description mobile_openarm_description"

if [ ! -f /opt/ros/jazzy/setup.bash ]; then
  echo "ROS 2 Jazzy 가 없습니다. 강의 7 의 1절(ROS 2 설치)을 먼저 하세요."; exit 1
fi
for tool in git colcon; do
  command -v "$tool" >/dev/null 2>&1 || { echo "$tool 이 없습니다:  sudo apt install -y git python3-colcon-common-extensions"; exit 1; }
done
set +u; source /opt/ros/jazzy/setup.bash; set -u
command -v xacro >/dev/null 2>&1 || { echo "xacro 가 없습니다:  sudo apt install -y ros-jazzy-xacro"; exit 1; }

echo "[1/3] 저장소 받기: $WS"
if [ -d "$WS/.git" ]; then git -C "$WS" pull --ff-only; else git clone --depth 1 https://github.com/PinkWink/pinklab_robotworld2026.git "$WS"; fi

echo "[2/3] 설계 패키지 빌드: $PACKAGES"
(cd "$WS" && colcon build --packages-select $PACKAGES)
set +u; source "$WS/install/setup.bash"; set -u

echo "[3/3] xacro → URDF"
mkdir -p "$WORK"
xacro "$WS/src/mobile_openarm_description/urdf/mobile_openarm.urdf.xacro" ros2_control:=false \
  | sed "s|package://\([A-Za-z0-9_]*\)/|$WS/src/\1/|g" > "$WORK/pl2m.urdf"
python3 -c "import sys, xml.etree.ElementTree as ET; r = ET.parse(sys.argv[1]).getroot(); print(f'  링크 {len(r.findall(\"link\"))}개, 관절 {len(r.findall(\"joint\"))}개')" "$WORK/pl2m.urdf"

echo
echo "준비 완료: $WORK/pl2m.urdf"
