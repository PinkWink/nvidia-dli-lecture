"""강의 45: 도킹에 필요한 노드를 띄웁니다.

    source /opt/ros/jazzy/setup.bash
    source ~/pinklab_robotworld2026/install/setup.bash
    ros2 launch 45_pl2m_dock/dock.launch.py                 # robot_state_publisher + 마커 찾는 노드
    ros2 launch 45_pl2m_dock/dock.launch.py rsp:=false      # robot_state_publisher 는 다른 런치(Nav2 · MoveIt)가 띄울 때
    ros2 launch 45_pl2m_dock/dock.launch.py window:=true    # 카메라 영상을 창으로 보기
"""
import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, ".."))


def setup(context):
    actions = []
    if LaunchConfiguration("rsp").perform(context) == "true":
        urdf = open(os.path.join(REPO, "work", "pl2m", "pl2m.urdf")).read()
        actions.append(Node(package="robot_state_publisher", executable="robot_state_publisher",
                            parameters=[{"robot_description": urdf, "use_sim_time": True}]))
    command = [sys.executable, "-u", os.path.join(HERE, "aruco_node.py")]
    if LaunchConfiguration("window").perform(context) == "true":
        command.append("--window")
    actions.append(ExecuteProcess(cmd=command, output="screen"))
    return actions


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument("rsp", default_value="true", description="robot_state_publisher 를 띄울지"),
                              DeclareLaunchArgument("window", default_value="false", description="카메라 영상을 창으로 띄울지"),
                              OpaqueFunction(function=setup)])
