"""강의 35: 핑크시티의 Pinky Pro 두 대(pinky1 · pinky2)에 Nav2 를 한 벌씩 띄웁니다.

    source /opt/ros/jazzy/setup.bash
    source ~/pinky_pro/install/setup.bash
    ros2 launch ~/nvidia-dli-lecture/35_two_pinkys/two_pinky_nav2.launch.py
    ros2 launch ~/nvidia-dli-lecture/35_two_pinkys/two_pinky_nav2.launch.py use_rviz:=false

로봇마다 같은 네임스페이스로 띄우는 것
    robot_state_publisher      /pinky1/joint_states → /pinky1/tf (로봇 몸의 링크들)
    Nav2 (공식 pinky_navigation 의 localization_launch.xml · navigation_launch.xml)   지도 서버 · AMCL · 경로 계획 · 제어
    RViz2                      로봇마다 창 하나

준비 시작은 start_nav2.py 가 맡습니다: 노드들이 서로 연결됐는지 확인한 뒤 보내고, 켜졌는지 다시 확인합니다.
설정은 강의 18 의 nav2_params.yaml 을 그대로 읽어, 로봇마다 세 가지만 바꿉니다: 라이다 토픽 이름, 처음 위치, 맨 위에 로봇 이름 한 겹.
"""
import math
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import AnyLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
PARAMS = os.path.join(REPO, "18_pinky_nav2", "nav2_params.yaml")
URDF = os.path.join(REPO, "work", "pinky_description", "urdf", "pinky.urdf")
MAP = os.path.join(REPO, "work", "pink_city", "pink_city_map.yaml")

NAVIGATION = os.path.join(get_package_share_directory("pinky_navigation"), "launch")
# 공식 bringup_launch.xml 이 켜는 노드 목록과 같다
LOCAL_NODES = "['map_server', 'amcl']"
NAV_NODES = "['controller_server', 'planner_server', 'behavior_server', 'bt_navigator', 'waypoint_follower', 'velocity_smoother']"

# two_pinkys.py 와 같은 표: 이름, 도시 좌표의 시작 위치 (x, y) [m], 방향 [도]
ROBOTS = [("pinky1", -1.05, -0.5, 90.0),
          ("pinky2", 1.05, 0.5, -90.0)]


def city_to_map(x, y, yaw):
    """도시 좌표 → 지도 좌표. 지도는 강의 17 에서 pinky1 의 시작 자리를 (0, 0), 그때 보던 방향을 +x 로 만들었다."""
    return y + 0.5, -x - 1.05, math.radians(yaw - 90.0)


def robot_params(name, x, y, yaw):
    """강의 18 의 설정에서 라이다 토픽 · 처음 위치 · 맨 위 이름만 바꿔 임시 파일로 만든다."""
    text = open(PARAMS).read()
    assert "topic: /scan" in text and "initial_pose: [0, 0, 0]" in text
    text = text.replace("topic: /scan", f"topic: /{name}/scan")                  # 비용 지도가 읽는 라이다
    mx, my, myaw = city_to_map(x, y, yaw)
    text = text.replace("initial_pose: [0, 0, 0]", f"initial_pose: {{x: {mx:.3f}, y: {my:.3f}, z: 0.0, yaw: {myaw:.4f}}}")   # AMCL 의 처음 위치
    # 설정 전체를 로봇 이름 아래로 한 칸 들여 넣는다: "amcl:" 은 /amcl 의 설정이고, /pinky1/amcl 은 "pinky1: amcl:" 을 읽는다
    text = f"{name}:\n" + "".join("  " + line for line in text.splitlines(keepends=True))
    with tempfile.NamedTemporaryFile("w", prefix=f"nav2_{name}_", suffix=".yaml", delete=False) as f:
        f.write(text)
    return f.name


def generate_launch_description():
    ld = LaunchDescription([DeclareLaunchArgument("use_rviz", default_value="true"),
                            DeclareLaunchArgument("map", default_value=MAP),
                            DeclareLaunchArgument("rviz_config", default_value=os.path.join(HERE, "rviz", "nav2_namespaced.rviz"))])
    description = open(URDF).read()
    tf_remap = [("/tf", "tf"), ("/tf_static", "tf_static")]          # tf 도 로봇마다 따로: /pinky1/tf
    rviz = []
    for name, x, y, yaw in ROBOTS:
        ld.add_action(Node(package="robot_state_publisher", executable="robot_state_publisher", namespace=name,
                           parameters=[{"use_sim_time": True, "robot_description": description}], remappings=tf_remap))
        # autostart 를 끈다: 노드는 띄우기만 하고, 준비 시작은 아래 start_nav2.py 가 확인한 뒤에 보낸다
        common = {"namespace": name, "use_sim_time": "True", "params_file": robot_params(name, x, y, yaw),
                  "use_composition": "False", "autostart": "False"}
        # GroupAction 으로 감싸야 한 로봇에 준 값(namespace · lifecycle_nodes)이 다음 것에 섞이지 않는다
        ld.add_action(GroupAction([IncludeLaunchDescription(          # 지도 서버 + AMCL
            AnyLaunchDescriptionSource(os.path.join(NAVIGATION, "localization_launch.xml")),
            launch_arguments={**common, "map": LaunchConfiguration("map"), "lifecycle_nodes": LOCAL_NODES}.items())]))
        ld.add_action(GroupAction([IncludeLaunchDescription(          # 경로 계획 + 제어 + 행동 트리
            AnyLaunchDescriptionSource(os.path.join(NAVIGATION, "navigation_launch.xml")),
            launch_arguments={**common, "lifecycle_nodes": NAV_NODES}.items())]))
        rviz.append(GroupAction([IncludeLaunchDescription(            # RViz2 (로봇마다 창 하나)
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("nav2_bringup"), "launch", "rviz_launch.py")),
            launch_arguments={"namespace": name, "use_namespace": "True", "use_sim_time": "True",
                              "rviz_config": LaunchConfiguration("rviz_config")}.items())],
            condition=IfCondition(LaunchConfiguration("use_rviz"))))
    # 받을 준비가 됐는지 확인 → 준비 시작 → 켜졌는지 확인 (로봇 한 대씩 차례로)
    starter = ExecuteProcess(cmd=["python3", os.path.join(HERE, "start_nav2.py")] + [name for name, *_ in ROBOTS], output="screen")
    ld.add_action(starter)
    # RViz2 는 Nav2 가 준비된 뒤에 띄운다. 그래야 Navigation 2 패널이 처음부터 active 로 보인다
    ld.add_action(RegisterEventHandler(OnProcessExit(target_action=starter, on_exit=rviz)))
    return ld
