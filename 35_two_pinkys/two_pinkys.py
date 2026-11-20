"""강의 35: 핑크시티에 Pinky Pro 두 대를 올리고, 로봇마다 ROS 2 네임스페이스를 붙입니다.

    source /opt/ros/jazzy/setup.bash
    ~/isaacsim/python.sh 35_two_pinkys/two_pinkys.py --gui          # 창으로 보기 (Ctrl+C 또는 창 닫기로 끝냄)
    ~/isaacsim/python.sh 35_two_pinkys/two_pinkys.py                # 창 없이
    ~/isaacsim/python.sh 35_two_pinkys/two_pinkys.py --no-namespace # 일부러 이름을 안 붙이기 (두 대가 같은 토픽을 씀)
    ~/isaacsim/python.sh 35_two_pinkys/two_pinkys.py --no-camera    # 카메라 토픽을 빼서 가볍게

로봇마다 생기는 토픽 (pinky1 의 예. pinky2 도 같음)
    받기    /pinky1/cmd_vel
    내보내기 /pinky1/odom · /pinky1/tf · /pinky1/joint_states · /pinky1/scan · /pinky1/imu · /pinky1/camera/image_raw
    공통    /clock  (시계는 하나)

만드는 방법: 강의 11 · 12 · 14 의 함수에 "로봇의 Prim 경로" 와 "네임스페이스" 만 다르게 줘서 두 번 부릅니다.
"""
import argparse
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, ".."))
for folder in ("", "11_pinky_import", "12_pinky_sensors", "14_pinky_ros2", "16_pink_city"):
    sys.path.insert(0, os.path.join(REPO, folder))

# 이름 = 네임스페이스, 도시 좌표의 시작 위치 (x, y) [m] 와 방향 [도]
ROBOTS = [("pinky1", -1.05, -0.5, 90.0),     # 강의 16 ~ 18 과 같은 자리: 왼쪽 도로에서 위(+Y)를 보고
          ("pinky2", 1.05, 0.5, -90.0)]      # 맞은편: 오른쪽 도로에서 아래(-Y)를 보고

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--gui", action="store_true", help="창을 띄운다")
    parser.add_argument("--seconds", type=float, help="이 시간이 지나면 끝냄 (없으면 Ctrl+C 까지)")
    parser.add_argument("--no-namespace", action="store_true", help="네임스페이스를 붙이지 않는다 (토픽이 겹치면 어떻게 되는지 보기)")
    parser.add_argument("--no-camera", action="store_true", help="카메라 토픽을 내보내지 않는다")
    parser.add_argument("--show-lidar", action="store_true", help="창에서 라이다 빔을 그린다")
    args = parser.parse_args()
    if "ROS_DISTRO" not in os.environ:
        print("ROS 2 환경이 없습니다. 같은 터미널에서 먼저 실행하세요:  source /opt/ros/jazzy/setup.bash")
        sys.exit(1)
    from isaacsim import SimulationApp
    app = SimulationApp({"headless": not args.gui, "extra_args": [
        "--/app/runLoops/main/rateLimitEnabled=true", "--/app/runLoops/main/rateLimitFrequency=60"]})

    from isaacsim.core.utils.extensions import enable_extension
    enable_extension("isaacsim.ros2.bridge")
    enable_extension("isaacsim.robot.wheeled_robots")
    app.update()

    import pink_city
    from common.sim import Sim, new_scene, pose_of, set_pose, set_view
    from import_pinky import DEFAULT_URDF, find_urdf, import_pinky
    from pinky_ros2 import add_cmd_vel_subscriber, add_sensor_publishers, add_state_publishers
    from pinky_sensors import add_camera, add_imu, add_lidar

    stage = new_scene()
    pink_city.add_city(stage, pink_city.make_textures())
    urdf = find_urdf(DEFAULT_URDF)
    for k, (name, x, y, yaw) in enumerate(ROBOTS):
        robot = f"/World/{name}"                                  # 로봇마다 다른 Prim 경로
        namespace = "" if args.no_namespace else name            # 로봇마다 다른 네임스페이스
        import_pinky(stage, urdf, height=0.0, path=robot)
        set_pose(stage.GetPrimAtPath(robot), translate=(x, y, 0.0), rotate_xyz=(0.0, 0.0, yaw))
        add_lidar(stage, draw=args.gui and args.show_lidar, robot=robot)
        add_camera(stage, robot=robot)
        add_imu(stage, robot=robot)
        add_cmd_vel_subscriber(robot=robot, namespace=namespace, graph=f"/Graphs/{name}_cmd_vel")
        add_state_publishers(robot=robot, namespace=namespace, graph=f"/Graphs/{name}_state", clock=(k == 0))
        add_sensor_publishers(robot=robot, namespace=namespace, graph=f"/Graphs/{name}_sensors", camera=not args.no_camera)
        print(f"{name}: Prim {robot}, 네임스페이스 '{namespace}', 시작 ({x}, {y}) 방향 {yaw}°", flush=True)
    print(f"ROS_DOMAIN_ID = {os.environ.get('ROS_DOMAIN_ID', '0')}. 끝내려면 Ctrl+C 를 누릅니다.", flush=True)
    if args.gui:
        set_view((-2.6, -3.4, 2.9), (0.0, 0.05, 0.0))

    bodies = [stage.GetPrimAtPath(f"/World/{name}/base_link") for name, *_ in ROBOTS]
    print(f"{'시간[s]':>8} " + " ".join(f"{name + ' x':>9} {'y':>7} {'방향':>7}" for name, *_ in ROBOTS), flush=True)
    sim = Sim(app)
    sim.play()

    def report(t):
        row = []
        for body in bodies:
            x, y, _, yaw = pose_of(body)
            row.append(f"{round(x, 2) + 0.0:9.2f} {round(y, 2) + 0.0:7.2f} {round(yaw) + 0.0:7.0f}")
        print(f"{t:8.0f} " + " ".join(row), flush=True)

    try:
        next_report = 5.0
        while app.is_running() and (args.seconds is None or sim.time < args.seconds):
            app.update()
            if sim.time >= next_report:
                report(sim.time)
                next_report += 5.0
    except KeyboardInterrupt:
        pass
    sim.stop()
    app.close()
