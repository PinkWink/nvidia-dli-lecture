"""강의 52: PL-2M 의 왼팔로 하는 Reach 과제. Isaac Lab 에 든 OpenArm Reach 과제에서 로봇과 목표의 범위만 바꾼 것입니다.

    바꾼 것                 OpenArm Reach (Isaac Lab)             이 파일
    로봇                    팔 하나 (Isaac Lab 의 USD)             PL-2M (강의 37 의 URDF). 베이스는 바닥에 고정
    움직이는 관절           openarm_joint1 ~ 7                     openarm_left_joint1 ~ 7 (오른팔 · 바퀴 · 손가락은 그대로 둔다)
    손                      openarm_hand                           openarm_left_link7 (손이 합쳐진 마지막 링크)
    시작 자세               팔을 앞으로 든 자세                    손이 앞에 있고 관절이 한계에서 먼 자세 (START)
    목표가 나오는 범위      팔 앞의 탁자 위                        로봇의 앞 · 왼쪽, 컵을 집던 높이
"""
import math
import os

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils import configclass

import isaaclab_tasks.manager_based.manipulation.reach.mdp as mdp
from isaaclab_tasks.manager_based.manipulation.reach.config.openarm.unimanual.reach_openarm_uni_env_cfg import ReachEnvCfg

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
URDF = os.path.join(REPO, "work", "pl2m", "pl2m.urdf")              # 강의 37 의 prepare_pl2m.sh 가 만든 파일
ARM = "openarm_left_joint.*"                                         # 학습하는 관절: 왼팔 일곱 개
HAND = "openarm_left_link7"                                          # 손이 붙은 마지막 링크. Isaac Lab 이 URDF 를 바꿀 때 손(openarm_left_hand)은 이 링크에 합쳐진다
CARRY = [0.93, -2.3726, -0.4355, 2.3729, 0.387, -0.78, 1.3904]       # 운반 자세의 왼팔 관절 [rad] (강의 39). 6번이 한계(±0.785)에 닿아 있다 (비교용)
START = [0.60, -2.20, -0.03, 2.02, 0.59, -0.48, 0.84]                # 시작 자세: 손의 방향은 운반 자세와 16° 다르고, 관절은 한계에서 0.3 rad 넘게 떨어져 있다
                                                                     # reach_samples.py 의 표본 가운데, 쥐는 방향에 가까우면서 관절의 여유가 가장 큰 것을 골랐다

# 목표가 나오는 상자 (로봇 기준의 앞, 왼쪽, 높이) [m]. 시작 자세일 때 끝 링크는 (0.20, 0.25, 0.87) 에 있고, 손끝은 거기서 19 cm 더 앞이다
# 끝 링크는 어떤 자세로도 앞으로 0.44 m 까지만 간다. 쥐는 방향을 지키면 0.37 m 까지다 (강의 52 의 5절에서 잰 것)
TARGET_BOX = ((0.15, 0.30), (0.12, 0.32), (0.80, 0.96))
WIDE_BOX = ((0.20, 0.45), (0.00, 0.30), (0.70, 0.95))                # 처음에 어림으로 잡았던 상자. 닿지 않는 곳이 섞여 있다 (비교용)
TWIST = (math.pi / 12, math.pi / 18)                                 # 목표의 방향을 흔드는 범위: roll ±15° · yaw ±10°. 손목의 6번 관절이 ±45° 밖에 돌지 않아 좁게 잡았다
FULL_TWIST = (math.pi / 6, math.pi / 9)                              # OpenArm Reach 과제의 범위: roll ±30° · yaw ±20° (비교용)


def set_target_box(cfg, box):
    """목표가 나오는 범위를 바꾼다."""
    cfg.commands.ee_pose.ranges.pos_x, cfg.commands.ee_pose.ranges.pos_y, cfg.commands.ee_pose.ranges.pos_z = box


def set_twist(cfg, roll, yaw):
    """목표의 방향을 흔드는 범위 [rad]. 손이 앞을 보는 자세(pitch 90°)에서 roll · yaw 를 ± 이만큼."""
    cfg.commands.ee_pose.ranges.roll, cfg.commands.ee_pose.ranges.yaw = (-roll, roll), (-yaw, yaw)
    cfg.commands.ee_pose.ranges.pitch = (math.pi / 2, math.pi / 2)


def set_start(cfg, pose):
    """왼팔의 시작 자세 (관절 일곱 개 [rad])."""
    cfg.scene.robot.init_state.joint_pos = {f"openarm_left_joint{i + 1}": value for i, value in enumerate(pose)}


PL2M_CFG = ArticulationCfg(
    spawn=sim_utils.UrdfFileCfg(
        asset_path=URDF,
        fix_base=True,                                               # 팔만 배우므로 베이스는 바닥에 붙여 둔다
        merge_fixed_joints=True,                                     # 강의 37 과 같다: 고정 관절로 붙은 빈 링크는 부모에 합친다
        self_collision=False,
        joint_drive=sim_utils.UrdfConverterCfg.JointDriveCfg(
            gains=sim_utils.UrdfConverterCfg.JointDriveCfg.PDGainsCfg(stiffness=None, damping=None)),      # 게인은 아래 actuators 에서 준다
        rigid_props=sim_utils.RigidBodyPropertiesCfg(disable_gravity=False, max_depenetration_velocity=5.0),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False, solver_position_iteration_count=8, solver_velocity_iteration_count=0),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.0),
        joint_pos={f"openarm_left_joint{i + 1}": value for i, value in enumerate(START)},        # 오른팔은 0 (내린 자세). 행동을 주지 않는 관절은 목표가 0 이라 거기에 머문다
    ),
    actuators={
        "arms": ImplicitActuatorCfg(joint_names_expr=["openarm_.*_joint[1-7]"], stiffness=500.0, damping=35.0),         # 강의 39 의 값 (원본 MuJoCo 설정)
        "fingers": ImplicitActuatorCfg(joint_names_expr=["openarm_.*_finger_joint.*"], stiffness=1800.0, damping=15.0),
        "wheels": ImplicitActuatorCfg(joint_names_expr=[".*_wheel_joint"], stiffness=0.0, damping=100.0),
    },
)


@configclass
class Pl2mReachEnvCfg(ReachEnvCfg):
    """PL-2M 의 왼손을 목표 자세로 보내는 과제."""

    def __post_init__(self):
        super().__post_init__()
        # 장면: 로봇을 바꾸고, 탁자를 빼고, 바닥을 로봇의 발밑으로 올린다
        self.scene.robot = PL2M_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
        self.scene.table = None
        self.scene.ground.init_state.pos = (0.0, 0.0, 0.0)
        # 관측 · 행동 · 벌점: 왼팔의 관절만 본다
        self.observations.policy.joint_pos.params["asset_cfg"] = SceneEntityCfg("robot", joint_names=[ARM])
        self.observations.policy.joint_vel.params["asset_cfg"] = SceneEntityCfg("robot", joint_names=[ARM])
        self.rewards.joint_vel.params["asset_cfg"] = SceneEntityCfg("robot", joint_names=[ARM])
        self.actions.arm_action = mdp.JointPositionActionCfg(asset_name="robot", joint_names=[ARM], scale=0.5, use_default_offset=True)
        # 보상: 왼손의 자리와 방향
        for term in (self.rewards.end_effector_position_tracking, self.rewards.end_effector_position_tracking_fine_grained,
                     self.rewards.end_effector_orientation_tracking):
            term.params["asset_cfg"].body_names = [HAND]
        # 목표: 로봇의 앞 · 왼쪽
        self.commands.ee_pose.body_name = HAND
        set_target_box(self, TARGET_BOX)
        set_twist(self, *TWIST)
        # 시작 자세의 0.9 ~ 1.1 배에서 시작한다
        self.events.reset_robot_joints.params["position_range"] = (0.9, 1.1)
        self.viewer.eye = (2.2, 2.2, 1.8)
        self.viewer.lookat = (0.3, 0.1, 0.8)

