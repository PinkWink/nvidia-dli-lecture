"""강의 42: MoveIt (move_group) 을 띄웁니다. 설정은 원본 패키지 mobile_openarm_moveit_config 의 것을 그대로 씁니다.

    source /opt/ros/jazzy/setup.bash
    source ~/pinklab_robotworld2026/install/setup.bash
    ros2 launch 42_pl2m_moveit/moveit.launch.py                # move_group + robot_state_publisher + RViz
    ros2 launch 42_pl2m_moveit/moveit.launch.py rviz:=false    # RViz 없이

띄우는 것
    robot_state_publisher   /joint_states 로 링크 사이의 /tf 를 낸다 (RViz 가 로봇을 그리는 데 씀)
    move_group              경로를 계획하고, 궤적을 강의 41 의 궤적 서버(액션)로 보낸다
    rviz2                   계획한 경로를 본다

원본의 런치는 MuJoCo 시뮬레이터가 내는 토픽으로 장애물을 넣는 노드까지 띄웁니다. 여기서는 그 노드를 빼고 띄웁니다.
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from mobile_openarm_moveit_config.config import parameters


def setup(context):
    params = parameters()                                  # URDF · SRDF · 기구학 · 관절 한계 · OMPL · 제어기(액션 이름)
    nodes = [
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": params["robot_description"], "use_sim_time": True}]),
        Node(package="moveit_ros_move_group", executable="move_group", output="screen", parameters=[params]),
    ]
    if LaunchConfiguration("rviz").perform(context) == "true":
        config = Path(get_package_share_directory("mobile_openarm_moveit_config")) / "config/moveit.rviz"
        nodes.append(Node(package="rviz2", executable="rviz2", arguments=["-d", str(config)], parameters=[params]))
    return nodes


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument("rviz", default_value="true"), OpaqueFunction(function=setup)])
