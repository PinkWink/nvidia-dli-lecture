"""강의 35: 핑크시티의 Pinky Pro 두 대(pinky1 · pinky2)에 Nav2 를 한 벌씩 띄웁니다.

    source /opt/ros/jazzy/setup.bash
    source ~/pinky_pro/install/setup.bash
    ros2 launch ~/nvidia-dli-lecture/35_two_pinkys/two_pinky_nav2.launch.py
    ros2 launch ~/nvidia-dli-lecture/35_two_pinkys/two_pinky_nav2.launch.py use_rviz:=false

로봇마다 같은 네임스페이스로 띄우는 것
    robot_state_publisher      /pinky1/joint_states → /pinky1/tf (로봇 몸의 링크들)
    Nav2 (공식 pinky_navigation 의 localization_launch.xml · navigation_launch.xml)   지도 서버 · AMCL · 경로 계획 · 제어
    RViz2                      로봇마다 창 하나

설정은 강의 18 의 nav2_params.yaml 을 그대로 읽어, 로봇마다 세 가지만 바꿉니다: 라이다 토픽 이름, 처음 위치, 맨 위에 로봇 이름 한 겹.
"""
import math
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
PARAMS = os.path.join(REPO, "18_pinky_nav2", "nav2_params.yaml")
URDF = os.path.join(REPO, "work", "pinky_description", "urdf", "pinky.urdf")
MAP = os.path.join(REPO, "work", "pink_city", "pink_city_map.yaml")

NAVIGATION = os.path.join(get_package_share_directory("pinky_navigation"), "launch")
STAGGER = 15.0        # 로봇 한 대의 Nav2 가 다 뜰 때까지 기다리는 시간 [s]
START_DELAY = 8.0     # 노드를 띄운 뒤 준비 시작을 보낼 때까지 기다리는 시간 [s]
RVIZ_DELAY = 10.0     # 준비 시작을 보낸 뒤 RViz2 를 띄울 때까지 [s]
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
    for k, (name, x, y, yaw) in enumerate(ROBOTS):
        actions = []
        actions.append(Node(package="robot_state_publisher", executable="robot_state_publisher", namespace=name,
                           parameters=[{"use_sim_time": True, "robot_description": description}], remappings=tf_remap))
        params = robot_params(name, x, y, yaw)
        common = {"namespace": name, "use_sim_time": "True", "params_file": params, "use_composition": "False", "autostart": "False"}
        # GroupAction 으로 감싸야 한 로봇에 준 값(namespace · lifecycle_nodes)이 다음 것에 섞이지 않는다
        actions.append(GroupAction([IncludeLaunchDescription(          # 지도 서버 + AMCL
            AnyLaunchDescriptionSource(os.path.join(NAVIGATION, "localization_launch.xml")),
            launch_arguments={**common, "map": LaunchConfiguration("map"), "lifecycle_nodes": LOCAL_NODES}.items())]))
        actions.append(GroupAction([IncludeLaunchDescription(          # 경로 계획 + 제어 + 행동 트리
            AnyLaunchDescriptionSource(os.path.join(NAVIGATION, "navigation_launch.xml")),
            launch_arguments={**common, "lifecycle_nodes": NAV_NODES}.items())]))
        # 노드들이 서로를 다 찾은 뒤에 "준비 시작" 을 보낸다. 뜨자마자 보내면 응답이 유실되어 멈추는 일이 있었다 (강의 PC)
        manage = "ros2 service call /{0}/lifecycle_manager_{1}/manage_nodes nav2_msgs/srv/ManageLifecycleNodes '{{command: 0}}'"
        actions.append(TimerAction(period=START_DELAY, actions=[ExecuteProcess(
            cmd=[manage.format(name, "localization") + " && " + manage.format(name, "navigation")], shell=True, output="log")]))
        # RViz2 (로봇마다 창 하나). Nav2 가 준비된 뒤에 띄워야 Navigation 2 패널이 처음부터 active 로 보인다
        actions.append(TimerAction(period=START_DELAY + RVIZ_DELAY, actions=[GroupAction([IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("nav2_bringup"), "launch", "rviz_launch.py")),
            launch_arguments={"namespace": name, "use_namespace": "True", "use_sim_time": "True",
                              "rviz_config": LaunchConfiguration("rviz_config")}.items())],
            condition=IfCondition(LaunchConfiguration("use_rviz")))]))
        # 노드 수십 개가 한꺼번에 뜨면 준비 신호가 유실되는 일이 있어, 로봇 한 대씩 STAGGER 초 간격으로 띄운다
        ld.add_action(TimerAction(period=float(k * STAGGER), actions=actions))
    return ld
