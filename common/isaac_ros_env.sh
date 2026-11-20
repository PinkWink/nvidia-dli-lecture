# Isaac Sim 을 띄우는 터미널에서 한 번 불러 씁니다 (bash · zsh 공통).
#
#     source ~/nvidia-dli-lecture/common/isaac_ros_env.sh
#     ~/isaacsim/python.sh <스크립트>
#
# 하는 일: 이 터미널의 Isaac Sim 이 "Isaac Sim 안에 들어 있는 ROS 2 Jazzy" 를 쓰게 합니다.
#   - 시스템 ROS 2 (/opt/ros/jazzy) 나 작업 공간을 이미 source 했다면 그 경로를 이 터미널에서만 뺍니다.
#     시스템 ROS 2 는 Python 3.12 용, Isaac Sim 은 Python 3.11 이라 섞이면 rclpy 를 쓰는 기능이 죽습니다.
#   - ROS_DOMAIN_ID 처럼 통신에 쓰는 값은 그대로 둡니다.
# 이 터미널에서는 ros2 명령을 쓰지 않습니다. ros2 명령 · Nav2 · RViz2 는 다른 터미널에서
#     source /opt/ros/jazzy/setup.bash     로 평소처럼 씁니다.

ISAACSIM_PATH="${ISAACSIM_PATH:-$HOME/isaacsim}"
_isaac_ros_lib="$ISAACSIM_PATH/exts/isaacsim.ros2.bridge/jazzy/lib"

if [ ! -d "$_isaac_ros_lib" ]; then
    echo "Isaac Sim 의 ROS 2 라이브러리를 찾지 못했습니다: $_isaac_ros_lib"
    echo "Isaac Sim 을 다른 곳에 설치했다면 먼저  export ISAACSIM_PATH=<설치 폴더>  를 실행하세요."
else
    # 경로 목록(":" 로 이어진 값)에서 ROS 2 설치(/opt/ros) 와 source 해 둔 작업 공간에 속한 항목을 뺀다
    _isaac_ros_drop="/opt/ros/:$_isaac_ros_lib:$AMENT_PREFIX_PATH:$COLCON_PREFIX_PATH"
    _isaac_ros_strip() {
        printf '%s' "$1" | awk -v RS=':' -v ORS=':' -v drop="$_isaac_ros_drop" '
            BEGIN { n = split(drop, d, ":") }
            { sub(/\n$/, ""); keep = ($0 != ""); for (i = 1; i <= n; i++) if (d[i] != "" && index($0, d[i]) == 1) keep = 0; if (keep) print }' | sed 's/:$//'
    }
    export LD_LIBRARY_PATH="$(_isaac_ros_strip "$LD_LIBRARY_PATH")"
    export PYTHONPATH="$(_isaac_ros_strip "$PYTHONPATH")"
    export PATH="$(_isaac_ros_strip "$PATH")"
    unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH ROS_PYTHON_VERSION _isaac_ros_drop
    unset -f _isaac_ros_strip

    export ROS_DISTRO=jazzy
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:+$LD_LIBRARY_PATH:}$_isaac_ros_lib"
    echo "Isaac Sim 내장 ROS 2 (jazzy) 를 씁니다. ROS_DOMAIN_ID = ${ROS_DOMAIN_ID:-0}"
fi
unset _isaac_ros_lib
