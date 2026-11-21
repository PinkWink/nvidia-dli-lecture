"""강의 44: Nav2 를 띄웁니다. 띄우는 노드와 설정은 원본 패키지 mobile_openarm_navigation 의 런치를 따릅니다.

    source /opt/ros/jazzy/setup.bash
    ros2 launch 44_pl2m_nav/nav2.launch.py                 # 지도 서버 · AMCL · 경로 계획 · 제어 + robot_state_publisher + RViz
    ros2 launch 44_pl2m_nav/nav2.launch.py rviz:=false     # RViz 없이
    ros2 launch 44_pl2m_nav/nav2.launch.py rsp:=false      # robot_state_publisher 는 다른 런치(강의 42 의 MoveIt)가 띄울 때

띄우는 것
    robot_state_publisher   /joint_states 로 링크 사이의 /tf 를 낸다 (base_footprint → laser_link 가 여기서 나온다)
    map_server · amcl       지도를 내고, 라이다로 지도 위의 위치를 추정한다 (/tf 의 map → odom)
    planner_server          목적지까지의 길을 찾는다
    controller_server       길을 따라가는 /cmd_vel 을 낸다
    behavior_server         막혔을 때의 동작 (제자리 돌기 · 뒤로 가기 · 기다리기)
    bt_navigator            /navigate_to_pose 액션을 받아 위의 것들을 순서대로 부린다
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
REPO = os.path.abspath(os.path.join(HERE, ".."))


def setup(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    params = os.path.join(HERE, "nav2_params.yaml")
    x, y, yaw = (float(v) for v in value("initial_pose").split(","))
    nodes, managed = [], []
    if value("rsp") == "true":
        urdf = open(os.path.join(REPO, "work", "pl2m", "pl2m.urdf")).read()
        nodes.append(Node(package="robot_state_publisher", executable="robot_state_publisher",
                          parameters=[{"robot_description": urdf, "use_sim_time": True}]))
    nodes.append(Node(package="nav2_map_server", executable="map_server", name="map_server", output="screen",
                      parameters=[params, {"yaml_filename": value("map")}]))
    nodes.append(Node(package="nav2_amcl", executable="amcl", name="amcl", output="screen",
                      parameters=[params, {"initial_pose": {"x": x, "y": y, "z": 0.0, "yaw": yaw}}]))
    managed += ["map_server", "amcl"]
    # 길이 막혔을 때만 다시 계획하는 행동 트리 (원본과 같음): 1초마다 다시 계획하면 도착 직전에 로봇이 좌우로 흔들렸다고 함
    tree = os.path.join(get_package_share_directory("nav2_bt_navigator"), "behavior_trees",
                        "navigate_w_recovery_and_replanning_only_if_path_becomes_invalid.xml")
    for package, name in (("nav2_controller", "controller_server"), ("nav2_planner", "planner_server"), ("nav2_smoother", "smoother_server"),
                          ("nav2_behaviors", "behavior_server"), ("nav2_bt_navigator", "bt_navigator")):
        extra = [{"default_nav_to_pose_bt_xml": tree}] if name == "bt_navigator" else []
        nodes.append(Node(package=package, executable=name, name=name, output="screen", parameters=[params] + extra))
        managed.append(name)
    nodes.append(Node(package="nav2_lifecycle_manager", executable="lifecycle_manager", name="lifecycle_manager", output="screen",
                      parameters=[{"use_sim_time": True, "autostart": True, "bond_timeout": 10.0, "node_names": managed}]))
    if value("rviz") == "true":
        nodes.append(Node(package="rviz2", executable="rviz2", arguments=["-d", os.path.join(HERE, "rviz", "nav2.rviz")],
                          parameters=[{"use_sim_time": True}]))
    return nodes


def generate_launch_description():
    from booth import MAP, SPAWN
    return LaunchDescription([
        DeclareLaunchArgument("map", default_value=MAP, description="지도 파일 (make_map.py 가 만든 것)"),
        DeclareLaunchArgument("initial_pose", default_value=",".join(str(v) for v in SPAWN), description="AMCL 의 처음 위치 X,Y,방향[rad]"),
        DeclareLaunchArgument("rsp", default_value="true", description="robot_state_publisher 를 띄울지"),
        DeclareLaunchArgument("rviz", default_value="true"),
        OpaqueFunction(function=setup)])
