"""강의 35: Nav2 가 "받을 준비" 가 됐는지 확인한 뒤에 준비 시작을 보내고, 정말 켜졌는지 다시 확인합니다.

    python3 start_nav2.py pinky1 pinky2          # two_pinky_nav2.launch.py 가 대신 실행해 줍니다

로봇 하나, 묶음 하나(localization · navigation)마다
    1. 기다리기   관리자(lifecycle_manager)와 각 노드 사이의 서비스가 연결될 때까지
                  = 노드의 change_state 서비스에 서버 1개(노드)와 클라이언트 1개(관리자)가 보일 때까지
    2. 시작       관리자의 manage_nodes 에 STARTUP 을 보냄
    3. 확인       관리자의 is_active 에 물어봄. 아니면 RESET 뒤 2 부터 다시 (최대 3번)
"""
import sys
import time

import rclpy
from nav2_msgs.srv import ManageLifecycleNodes
from std_srvs.srv import Trigger

GROUPS = {"localization": ["map_server", "amcl"],
          "navigation": ["controller_server", "planner_server", "behavior_server", "bt_navigator", "waypoint_follower",
                         "velocity_smoother"]}
STARTUP, RESET = 0, 3            # ManageLifecycleNodes 의 command 값
WAIT_LIMIT = 60.0                # 연결을 기다리는 최대 시간 [s]
CALL_LIMIT = 60.0                # 서비스 응답을 기다리는 최대 시간 [s]


def connected(node, robot, group):
    """관리자와 이 묶음의 모든 노드가 서로 연결됐는가 (ROS 2 그래프에서 세어 본다)."""
    if node.count_services(f"/{robot}/lifecycle_manager_{group}/manage_nodes") < 1:
        return False
    for name in GROUPS[group]:
        for service in ("change_state", "get_state"):
            path = f"/{robot}/{name}/{service}"
            if node.count_services(path) < 1 or node.count_clients(path) < 1:
                return False
    return True


def wait_connected(node, robot, group):
    """연결이 3번 연속(0.6초)으로 보일 때까지 기다린다. 걸린 시간을 돌려준다 (못 하면 None)."""
    start, streak = time.time(), 0
    while time.time() - start < WAIT_LIMIT:
        rclpy.spin_once(node, timeout_sec=0.2)
        streak = streak + 1 if connected(node, robot, group) else 0
        if streak >= 3:
            return time.time() - start
        time.sleep(0.2)
    return None


def call(node, client, request):
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=CALL_LIMIT)
    return future.result() if future.done() else None


def start(node, robot, group):
    """한 묶음을 켠다. 성공하면 True."""
    waited = wait_connected(node, robot, group)
    if waited is None:
        print(f"[{robot} {group}] 연결을 기다리다 포기 ({WAIT_LIMIT:.0f}초)", flush=True)
        return False
    manage = node.create_client(ManageLifecycleNodes, f"/{robot}/lifecycle_manager_{group}/manage_nodes")
    is_active = node.create_client(Trigger, f"/{robot}/lifecycle_manager_{group}/is_active")
    manage.wait_for_service(timeout_sec=WAIT_LIMIT)
    is_active.wait_for_service(timeout_sec=WAIT_LIMIT)
    for attempt in (1, 2, 3):
        call(node, manage, ManageLifecycleNodes.Request(command=STARTUP))
        answer = call(node, is_active, Trigger.Request())
        if answer is not None and answer.success:
            print(f"[{robot} {group}] 준비됨 (연결 대기 {waited:.1f}초, 시도 {attempt}번)", flush=True)
            return True
        print(f"[{robot} {group}] 시도 {attempt}번 실패 → 초기화 후 다시", flush=True)
        call(node, manage, ManageLifecycleNodes.Request(command=RESET))
        time.sleep(1.0)
    return False


def main():
    robots = sys.argv[1:]
    rclpy.init()
    node = rclpy.create_node("start_nav2")
    ok = all([start(node, robot, group) for robot in robots for group in ("localization", "navigation")])
    print("Nav2 준비 완료" if ok else "Nav2 준비 실패: 실행을 끄고 다시 해 보세요", flush=True)
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
