#!/usr/bin/env bash
# PL-2M 의 MoveIt 설정을 빌드합니다 (강의 42 부터 사용).
#
#   bash 42_pl2m_moveit/prepare_moveit.sh
#
# 하는 일
#   1. MoveIt 이 설치돼 있는지 본다
#   2. 강의 37 에서 받은 저장소 ~/pinklab_robotworld2026 에서 MoveIt 설정 패키지와 그것이 기대는 패키지를 빌드한다
#        mobile_openarm_moveit_config (SRDF · 기구학 · 관절 한계 · OMPL · 제어기 이름)
#        warehouse_interfaces · mobile_openarm_mujoco (설정 패키지가 기대는 것. 빌드만 하고 쓰지는 않는다)
set -euo pipefail

WS="$HOME/pinklab_robotworld2026"
if [ ! -d "$WS/src/mobile_openarm_moveit_config" ]; then
  echo "저장소가 없습니다. 강의 37 의 준비를 먼저 하세요:  bash 37_pl2m_import/prepare_pl2m.sh"; exit 1
fi
set +u; source /opt/ros/jazzy/setup.bash; set -u

echo "[1/2] MoveIt 확인"
for package in moveit_ros_move_group moveit_planners_ompl moveit_kinematics moveit_simple_controller_manager robot_state_publisher; do
  ros2 pkg prefix "$package" >/dev/null 2>&1 || { echo "$package 가 없습니다:  sudo apt install -y ros-jazzy-moveit ros-jazzy-robot-state-publisher"; exit 1; }
done
echo "  있음"

echo "[2/2] MoveIt 설정 빌드"
(cd "$WS" && colcon build --packages-up-to mobile_openarm_moveit_config)
set +u; source "$WS/install/setup.bash"; set -u
python3 - <<'PY'
from mobile_openarm_moveit_config.config import parameters
p = parameters()
names = p["moveit_simple_controller_manager"]["controller_names"]
print(f"  설정을 읽었음: 계획기 {p['default_planning_pipeline']}, 제어기 {len(names)}개 ({', '.join(names)})")
PY

echo
echo "준비됨. MoveIt 을 쓰는 터미널에서는 먼저 다음을 실행하세요."
echo "  source /opt/ros/jazzy/setup.bash"
echo "  source ~/pinklab_robotworld2026/install/setup.bash"
