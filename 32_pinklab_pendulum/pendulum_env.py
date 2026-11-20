"""강의 32: 핑크랩 로터리 역진자의 강화학습 환경 (Isaac Lab, Direct 방식).

팔(base_to_arm)을 모터가 수평으로 돌리고, 팔 끝의 진자(arm_to_pendulum)는 모터 없이 자유롭게 돕니다.
정책은 팔의 각가속도만 정합니다. 매달린 진자를 흔들어 위로 세우고, 세운 채 버티는 것이 과제입니다.

실물과 맞춘 것
    행동      a ∈ [-1, 1] → 팔 각가속도 150·a rad/s² (실물 펌웨어와 같은 약속). 속도로 적분하고 ±5 rad/s 에서 자름
    팔 한계   ±125° 를 넘으면 정책의 명령 대신 반대쪽으로 최대 가속 (실물 펌웨어의 브레이크)
    진자 마찰 속도에 비례하는 마찰 + 방향만 따르는 마찰. 실물을 흔들어 놓고 잰 값
    제어 주기 물리 700 Hz, 정책 58.33 Hz (물리 12번에 한 번)

실물의 불완전함 (강의 33, 설정으로 켠다)
    delay_range     명령이 팔에 닿기까지의 지연
    friction_range  진자 마찰의 배율
    sensor_noise    각도 센서의 눈금과 잡음, 각도의 차이로 구한 진자 속도, 스텝 수로 센 팔 각도
"""
import math
import os

import torch

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import Articulation, ArticulationCfg
from isaaclab.envs import DirectRLEnv, DirectRLEnvCfg, ViewerCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim import SimulationCfg
from isaaclab.utils import configclass

HERE = os.path.dirname(os.path.abspath(__file__))
MODEL = os.path.join(HERE, "model", "model_v3_nodisc.usd")

MAX_ACCEL = 150.0                 # 행동 1 의 팔 각가속도 [rad/s²]
MAX_ARM_SPEED = 5.0               # 팔 속도 한계 [rad/s]
ARM_RAIL = math.radians(125.0)    # 팔 각도 한계 [rad]
PENDULUM_INERTIA = 3.258e-05      # 축에 대한 진자의 관성 [kg·m²]
VISCOUS = 0.60 * PENDULUM_INERTIA     # 속도에 비례하는 마찰 [N·m·s]. 실물 측정 0.60 /s
ENCODER_STEP = 2.0 * math.pi / 4096       # 진자 각도 센서의 눈금 (12비트) [rad]
MOTOR_STEP = 2.0 * math.pi / 1600         # 스텝모터의 한 스텝 [rad]
ENCODER_NOISE = 0.002                     # 진자 각도 센서의 잡음 [rad]
MAX_DELAY_STEPS = 32                      # 지연을 기억해 둘 물리 스텝 수 (32 / 700 = 46 ms 까지)
COULOMB = 2.50 * PENDULUM_INERTIA     # 방향만 따르는 마찰 [N·m]. 실물 측정 2.50 rad/s²
GRAVITY_TORQUE = 0.00694 * 9.81 * 0.05585     # 진자 무게 x 중력 x 무게중심까지의 거리 [N·m]

ROBOT_CFG = ArticulationCfg(
    prim_path="/World/envs/env_.*/Robot",
    spawn=sim_utils.UsdFileCfg(usd_path=MODEL),
    init_state=ArticulationCfg.InitialStateCfg(joint_pos={"base_to_arm": 0.0, "arm_to_pendulum": 0.0}),
    actuators={
        # 팔: 주어진 속도를 그대로 따라가는 구동기 (스텝모터). 힘의 한계는 모터의 유지 토크
        "motor": ImplicitActuatorCfg(joint_names_expr=["base_to_arm"], stiffness=0.0, damping=16414.0,
                                     effort_limit_sim=0.14, velocity_limit_sim=50.0),
        # 진자: 구동 없음. 마찰은 아래 _apply_action 에서 토크로 넣는다
        "pendulum": ImplicitActuatorCfg(joint_names_expr=["arm_to_pendulum"], stiffness=0.0, damping=0.0),
    },
)


@configclass
class PendulumEnvCfg(DirectRLEnvCfg):
    sim: SimulationCfg = SimulationCfg(dt=1.0 / 700.0, render_interval=12)
    decimation = 12                       # 700 / 12 = 58.33 Hz
    episode_length_s = 8.0
    action_space = 1
    observation_space = 6
    state_space = 0
    scene: InteractiveSceneCfg = InteractiveSceneCfg(num_envs=4096, env_spacing=0.5, replicate_physics=True)
    robot: ArticulationCfg = ROBOT_CFG

    viewer = ViewerCfg(eye=(1.5, 1.5, 0.9), lookat=(0.0, 0.0, 0.1))

    # 실물의 불완전함 (강의 33). 기본값은 모두 꺼짐 = 이상적인 시뮬레이션
    delay_range = (0.0, 0.0)              # 명령이 팔에 닿기까지의 지연 [s]. 환경마다 이 범위에서 하나씩 뽑는다
    friction_range = (1.0, 1.0)           # 진자 마찰에 곱하는 배율. 환경마다 하나씩 뽑는다
    sensor_noise = False                  # True: 실물 센서처럼 눈금 · 잡음을 넣고, 진자 속도를 각도의 차이로 계산한다

    start_hanging = False                 # True: 늘 아래로 매달린 채 시작 (재생용)
    # 보상의 가중치
    energy_weight = 0.5                   # 진자의 에너지가 "서서 멈춘 상태"와 다른 만큼
    arm_center_weight = 0.02              # 팔이 가운데에서 벗어난 만큼 [1/rad²]
    arm_speed_weight = 0.001              # 팔이 빠른 만큼
    action_rate_weight = 0.01             # 행동이 급하게 바뀐 만큼


class PendulumEnv(DirectRLEnv):
    cfg: PendulumEnvCfg

    def __init__(self, cfg: PendulumEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)
        self.arm = self.robot.find_joints("base_to_arm")[0][0]
        self.pendulum = self.robot.find_joints("arm_to_pendulum")[0][0]
        self.action = torch.zeros(self.num_envs, device=self.device)
        self.previous_action = torch.zeros_like(self.action)
        self.arm_speed_command = torch.zeros_like(self.action)
        self.arm_angle_command = torch.zeros_like(self.action)              # 스텝 수로 센 팔 각도 (실물이 보고하는 값)
        self.accel_history = torch.zeros(self.num_envs, MAX_DELAY_STEPS, device=self.device)   # 최근 명령들
        self.delay_steps = torch.zeros(self.num_envs, dtype=torch.long, device=self.device)
        self.friction_scale = torch.ones(self.num_envs, device=self.device)
        self.measured_angle = torch.zeros_like(self.action)                 # 센서가 읽은 진자 각도 (직전 것)
        self.rows = torch.arange(self.num_envs, device=self.device)

    def _setup_scene(self):
        self.robot = Articulation(self.cfg.robot)
        self.scene.clone_environments(copy_from_source=False)
        self.scene.articulations["robot"] = self.robot
        ground = sim_utils.GroundPlaneCfg(color=(0.12, 0.12, 0.14))
        ground.func("/World/Ground", ground)
        light = sim_utils.DomeLightCfg(intensity=700.0)
        light.func("/World/Light", light)

    # ---- 행동 ----
    def _pre_physics_step(self, actions: torch.Tensor):
        self.previous_action = self.action
        self.action = actions.squeeze(-1).clamp(-1.0, 1.0)

    def _apply_action(self):
        """물리 한 번(1/700초)마다 불린다."""
        self.accel_history = torch.roll(self.accel_history, 1, dims=1)      # 명령을 한 칸씩 뒤로 밀고
        self.accel_history[:, 0] = self.action * MAX_ACCEL                  # 맨 앞에 지금 명령을 적는다
        accel = self.accel_history[self.rows, self.delay_steps]             # delay_steps 전의 명령을 꺼내 쓴다
        arm_angle = self.robot.data.joint_pos[:, self.arm]
        accel = torch.where(arm_angle >= ARM_RAIL, torch.full_like(accel, -MAX_ACCEL), accel)     # 한계 밖: 브레이크
        accel = torch.where(arm_angle <= -ARM_RAIL, torch.full_like(accel, MAX_ACCEL), accel)
        self.arm_speed_command = (self.arm_speed_command + accel * self.physics_dt).clamp(-MAX_ARM_SPEED, MAX_ARM_SPEED)
        self.arm_angle_command = self.arm_angle_command + self.arm_speed_command * self.physics_dt
        self.robot.set_joint_velocity_target(self.arm_speed_command.unsqueeze(-1), joint_ids=[self.arm])
        speed = self.robot.data.joint_vel[:, self.pendulum]
        friction = self.friction_scale * (-VISCOUS * speed - COULOMB * torch.tanh(speed / 0.05))
        self.robot.set_joint_effort_target(friction.unsqueeze(-1), joint_ids=[self.pendulum])

    # ---- 관측 ----
    def upright_angle(self) -> torch.Tensor:
        """진자 각도. 0 = 똑바로 선 것, ±π = 아래로 매달린 것."""
        angle = self.robot.data.joint_pos[:, self.pendulum] - math.pi      # 관절 값 0 이 매달린 자세
        return torch.atan2(torch.sin(angle), torch.cos(angle))

    def _get_observations(self) -> dict:
        theta = self.upright_angle()
        arm_angle = self.robot.data.joint_pos[:, self.arm]
        arm_speed = self.robot.data.joint_vel[:, self.arm]
        pendulum_speed = self.robot.data.joint_vel[:, self.pendulum]
        if self.cfg.sensor_noise:
            # 진자: 12비트 눈금 + 잡음. 속도 센서는 없으므로 각도의 차이를 시간으로 나눈다
            theta = torch.round(theta / ENCODER_STEP) * ENCODER_STEP + ENCODER_NOISE * torch.randn_like(theta)
            difference = theta - self.measured_angle
            difference = torch.atan2(torch.sin(difference), torch.cos(difference))
            pendulum_speed = difference / self.step_dt
            self.measured_angle = theta
            # 팔: 실물은 팔을 재지 않는다. 모터에 보낸 스텝 수를 센 값을 쓴다
            arm_angle = torch.floor(self.arm_angle_command / MOTOR_STEP) * MOTOR_STEP
            arm_speed = self.arm_speed_command
        observation = torch.stack([
            arm_angle,                                       # 팔 각도 [rad]
            torch.sin(theta), torch.cos(theta),              # 진자 각도 (한 바퀴 돌아도 값이 이어지게 sin, cos 로)
            arm_speed,                                       # 팔 속도 [rad/s]
            pendulum_speed,                                  # 진자 속도 [rad/s]
            self.action,                                     # 직전 행동
        ], dim=-1)
        return {"policy": observation}

    # ---- 보상 ----
    def _get_rewards(self) -> torch.Tensor:
        theta = self.upright_angle()
        speed = self.robot.data.joint_vel[:, self.pendulum]
        upright = (1.0 + torch.cos(theta)) / 2.0                                # 섰으면 1, 매달렸으면 0
        # 진자의 에너지. 서서 멈춘 상태가 0, 매달려 멈춘 상태가 -1 이 되게 나눈다
        energy = (0.5 * PENDULUM_INERTIA * speed ** 2 + GRAVITY_TORQUE * (torch.cos(theta) - 1.0)) / (2.0 * GRAVITY_TORQUE)
        arm_angle = self.robot.data.joint_pos[:, self.arm]
        arm_speed = self.robot.data.joint_vel[:, self.arm]
        self.extras["log"] = {"upright": upright.mean(), "energy_error": energy.abs().mean()}
        return (upright
                - self.cfg.energy_weight * energy.abs().clamp(max=2.0)
                - self.cfg.arm_center_weight * arm_angle ** 2
                - self.cfg.arm_speed_weight * arm_speed ** 2
                - self.cfg.action_rate_weight * (self.action - self.previous_action) ** 2)

    # ---- 끝 ----
    def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
        time_out = self.episode_length_buf >= self.max_episode_length - 1
        return torch.zeros_like(time_out), time_out        # 시간이 다 될 때만 끝난다. 팔 한계는 브레이크가 막는다

    # ---- 리셋 ----
    def _reset_idx(self, env_ids):
        if env_ids is None:
            env_ids = self.robot._ALL_INDICES
        super()._reset_idx(env_ids)
        count = len(env_ids)
        position = torch.zeros(count, 2, device=self.device)
        velocity = torch.zeros(count, 2, device=self.device)
        if self.cfg.start_hanging:
            position[:, self.pendulum] = torch.empty(count, device=self.device).uniform_(-0.05, 0.05)
        else:                                             # 진자는 한 바퀴 어디서든, 팔은 ±57° 안에서 시작
            position[:, self.pendulum] = torch.empty(count, device=self.device).uniform_(-math.pi, math.pi)
            position[:, self.arm] = torch.empty(count, device=self.device).uniform_(-1.0, 1.0)
            velocity[:, self.pendulum] = torch.empty(count, device=self.device).uniform_(-1.0, 1.0)
        self.robot.write_joint_state_to_sim(position, velocity, env_ids=env_ids)
        self.action[env_ids] = 0.0
        self.previous_action[env_ids] = 0.0
        self.arm_speed_command[env_ids] = 0.0
        self.arm_angle_command[env_ids] = position[:, self.arm]
        self.accel_history[env_ids] = 0.0
        self.measured_angle[env_ids] = torch.atan2(torch.sin(position[:, self.pendulum] - math.pi),
                                                   torch.cos(position[:, self.pendulum] - math.pi))
        low, high = self.cfg.delay_range                 # 지연과 마찰을 환경마다 새로 뽑는다
        delay = torch.empty(count, device=self.device).uniform_(low, high)
        self.delay_steps[env_ids] = (delay / self.physics_dt).round().long().clamp(0, MAX_DELAY_STEPS - 1)
        low, high = self.cfg.friction_range
        self.friction_scale[env_ids] = torch.empty(count, device=self.device).uniform_(low, high)
