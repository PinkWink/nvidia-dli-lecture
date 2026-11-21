#!/usr/bin/env python3
"""강의 42: 장애물을 MoveIt 의 계획 장면(planning scene)에 넣거나 뺍니다.

    python3 42_pl2m_moveit/add_obstacle.py            # 넣기
    python3 42_pl2m_moveit/add_obstacle.py --remove   # 빼기

MoveIt 은 시뮬레이터 안을 보지 못합니다. 무엇이 어디에 있는지 알려 줘야 그것을 피해 계획합니다.
"""
import os
import sys

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from obstacle import BOX_CENTER, BOX_SIZE


def main():
    rclpy.init()
    node = Node("add_obstacle")
    client = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
    if not client.wait_for_service(timeout_sec=20.0):
        sys.exit("MoveIt 이 없음: ros2 launch 42_pl2m_moveit/moveit.launch.py 를 먼저 띄울 것")
    box = CollisionObject(id="post_and_cup")
    box.header.frame_id = "odom"                             # 로봇이 출발한 자리 기준
    if "--remove" in sys.argv:
        box.operation = CollisionObject.REMOVE
    else:
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = BOX_CENTER
        pose.orientation.w = 1.0
        box.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=list(BOX_SIZE))]
        box.primitive_poses = [pose]
        box.operation = CollisionObject.ADD
    scene = PlanningScene(is_diff=True)
    scene.world.collision_objects = [box]
    done = client.call_async(ApplyPlanningScene.Request(scene=scene))
    rclpy.spin_until_future_complete(node, done)
    what = "뺐음" if "--remove" in sys.argv else f"넣었음: 가운데 {BOX_CENTER}, 크기 {BOX_SIZE}"
    print(f"장애물 post_and_cup 을 {what}" if done.result().success else "MoveIt 이 받지 않았음")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
