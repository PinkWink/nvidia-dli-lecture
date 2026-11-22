"""강의 55: MobilityGen 에 Pinky Pro 를 등록합니다. Isaac Sim 에 든 Jetbot · Carter 와 같은 틀에 값만 Pinky Pro 것으로 채웁니다.

이 파일을 불러오면(import) 로봇 목록에 "PinkyRobot" 이 생긴다. Isaac Sim 의 설치 폴더는 고치지 않는다.
Isaac Sim 안에서(SimulationApp 을 만든 뒤에) 불러온다.
"""
import math
import os
from typing import List, Tuple

from isaacsim.replicator.mobility_gen.examples.robots import ROBOTS, WheeledMobilityGenRobot
from isaacsim.replicator.mobility_gen.impl.camera import MobilityGenCamera
from isaacsim.replicator.mobility_gen.impl.common import Module
from isaacsim.replicator.mobility_gen.impl.utils.global_utils import join_sdf_paths

HERE = os.path.dirname(os.path.abspath(__file__))
WORK = os.path.abspath(os.path.join(HERE, "..", "work", "mobilitygen"))
CAMERA = "front_camera_mount/front_camera_link/Camera"          # 로봇 안에서 카메라가 있는 곳 (강의 12 에서 단 카메라)


class PinkyCamera(Module):
    """Pinky Pro 의 앞 카메라 하나. (Carter 는 좌우 두 눈짜리 Hawk 카메라를 쓴다.)"""
    resolution: Tuple[int, int] = (640, 360)

    def __init__(self, camera: MobilityGenCamera):
        self.camera = camera

    @classmethod
    def attach(cls, prim_path: str) -> "PinkyCamera":
        return cls(MobilityGenCamera(prim_path, cls.resolution))

    build = attach                                                # 카메라 Prim 은 로봇 파일에 이미 있다. 새로 만들지 않고 붙기만 한다


@ROBOTS.register()
class PinkyRobot(WheeledMobilityGenRobot):
    physics_dt: float = 1.0 / 240.0                              # 물리는 초당 240 스텝. 들어 있는 Jetbot · Carter 는 200 스텝이다
    z_offset: float = 0.02                                       # 다시 놓을 때 바닥에서 띄우는 높이 [m]

    chase_camera_base_path = "base_link"                         # 창에서 로봇을 따라다니는 카메라
    chase_camera_x_offset: float = -0.5
    chase_camera_z_offset: float = 0.45
    chase_camera_tilt_angle: float = 55.0

    occupancy_map_radius: float = 0.14                           # 길을 찾을 때 장애물을 이만큼 부풀린다 [m]. 0.09 로는 모퉁이에 걸렸다 (강의 55 의 4절)
    occupancy_map_z_min: float = 0.02
    occupancy_map_z_max: float = 0.15
    occupancy_map_cell_size: float = 0.02
    occupancy_map_collision_radius: float = 4                    # 가운데가 장애물에 이만큼 다가가면 부딪힌 것으로 본다. 단위가 지도의 칸이다: 4칸 = 8 cm
    #   (Isaac Sim 5.1 의 scenarios.py 는 이 값을 미터가 아니라 칸 수로 읽는다. Carter 의 0.5 는 0칸이 된다)

    front_camera_base_path = CAMERA
    front_camera_rotation = (0.0, 0.0, 0.0)
    front_camera_translation = (0.0, 0.0, 0.0)
    front_camera_type = PinkyCamera

    keyboard_linear_velocity_gain: float = 0.15                  # 키보드로 몰 때의 속도 [m/s] · [rad/s]
    keyboard_angular_velocity_gain: float = 1.0
    gamepad_linear_velocity_gain: float = 0.15
    gamepad_angular_velocity_gain: float = 1.0

    random_action_linear_velocity_range: Tuple[float, float] = (-0.05, 0.15)       # 무작위로 몰 때의 속도 범위
    random_action_angular_velocity_range: Tuple[float, float] = (-1.0, 1.0)
    random_action_linear_acceleration_std: float = 0.5
    random_action_angular_acceleration_std: float = 4.0
    random_action_grid_pose_sampler_grid_size: float = 1.6       # 지도를 이 크기의 칸으로 나눠 그 가운데 한 칸에 놓는다 [m]. 0.5 로는 빈 곳이 없는 칸이 뽑혀 멈췄다

    path_following_speed: float = 0.12                           # 길을 따라갈 때의 속도 [m/s]
    path_following_angular_gain: float = 2.0
    path_following_stop_distance_threshold: float = 0.08         # 끝에서 이 거리 안이면 도착 [m]
    path_following_forward_angle_threshold = math.pi / 4
    path_following_target_point_offset_meters: float = 0.15      # 길 위에서 이만큼 앞의 점을 보고 간다 [m]

    wheel_dof_names: List[str] = ["l_wheel_joint", "r_wheel_joint"]
    usd_url: str = os.path.join(WORK, "pinky.usda")              # prepare.py 가 만드는 로봇 파일
    chassis_subpath: str = "base_footprint"                      # 관절 로봇의 뿌리 (prepare.py 가 출력해 준다)
    wheel_base: float = 0.0961                                   # 좌우 바퀴 사이 거리 [m] (강의 11)
    wheel_radius: float = 0.028                                  # 바퀴 반지름 [m]

    @classmethod
    def build_front_camera(cls, prim_path):
        """Carter 처럼 카메라 장치를 새로 붙이지 않는다. 로봇 파일에 이미 있는 카메라에 붙는다."""
        return cls.front_camera_type.attach(join_sdf_paths(prim_path, cls.front_camera_base_path))
