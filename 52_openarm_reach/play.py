"""강의 52: 학습한 정책으로 PL-2M 의 왼팔을 움직여 봅니다.

    ~/IsaacLab/isaaclab.sh -p 52_openarm_reach/play.py                                   # 창에서 9대를 계속 재생
    ~/IsaacLab/isaaclab.sh -p 52_openarm_reach/play.py --headless --num_envs 256 --seconds 12     # 숫자로 평가
    옵션: --checkpoint 파일 (없으면 가장 최근 학습의 마지막 정책)  --wide · --carry_start · --full_twist (학습할 때 준 것과 같게)

평가: 목표는 4초마다 바뀝니다. 바뀌기 직전에 손이 목표에서 얼마나 떨어져 있는지(자리 · 방향)를 모아 평균을 냅니다.
"""
import argparse
import glob
import math
import os

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--checkpoint", default=None, help="정책 파일 model_N.pt")
parser.add_argument("--num_envs", type=int, default=9)
parser.add_argument("--seconds", type=float, default=0.0, help="이 시간만 돌리고 결과를 출력한다. 0 이면 창을 닫을 때까지")
parser.add_argument("--wide", action="store_true", help="--wide 로 학습한 정책을 볼 때: 목표의 범위도 그때와 같게")
parser.add_argument("--carry_start", action="store_true", help="--carry_start 로 학습한 정책을 볼 때")
parser.add_argument("--full_twist", action="store_true", help="--full_twist 로 학습한 정책을 볼 때")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
app = AppLauncher(args).app

import torch  # noqa: E402
from isaaclab.envs import ManagerBasedRLEnv  # noqa: E402
from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper  # noqa: E402
from isaaclab_tasks.manager_based.manipulation.reach.config.openarm.unimanual.agents.rsl_rl_ppo_cfg import OpenArmReachPPORunnerCfg  # noqa: E402
from rsl_rl.runners import OnPolicyRunner  # noqa: E402

from pl2m_reach_env import CARRY, FULL_TWIST, WIDE_BOX, Pl2mReachEnvCfg, set_start, set_target_box, set_twist  # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
checkpoint = args.checkpoint
if checkpoint is None:
    runs = sorted(glob.glob(os.path.join(REPO, "work", "reach_logs", "*")))
    assert runs, "학습 결과가 없습니다. train.py 를 먼저 실행하세요."
    checkpoint = max(glob.glob(os.path.join(runs[-1], "model_*.pt")), key=os.path.getmtime)
print(f"정책: {checkpoint}", flush=True)

env_cfg = Pl2mReachEnvCfg()
if args.wide:
    set_target_box(env_cfg, WIDE_BOX)
if args.carry_start:
    set_start(env_cfg, CARRY)
if args.full_twist:
    set_twist(env_cfg, *FULL_TWIST)
env_cfg.scene.num_envs = args.num_envs
env_cfg.observations.policy.enable_corruption = False            # 재생할 때는 관측에 잡음을 넣지 않는다
env_cfg.episode_length_s = 1.0e6 if args.seconds <= 0 else args.seconds + 1.0
env = ManagerBasedRLEnv(env_cfg)
wrapped = RslRlVecEnvWrapper(env)
runner = OnPolicyRunner(wrapped, OpenArmReachPPORunnerCfg().to_dict(), log_dir=None, device=env.device)
runner.load(checkpoint)
policy = runner.get_inference_policy(device=env.device)

command = env.command_manager.get_term("ee_pose")
period = round(env_cfg.commands.ee_pose.resampling_time_range[0] / env.step_dt)        # 목표가 바뀌는 간격 [스텝]
steps = int(args.seconds / env.step_dt) if args.seconds > 0 else None
positions, orientations, count = [], [], 0
observations = wrapped.get_observations()
with torch.inference_mode():
    while app.is_running() and (steps is None or count < steps):
        observations, _, _, _ = wrapped.step(policy(observations))
        count += 1
        if count % period == period - 2:                         # 목표가 바뀌기 직전
            positions.append(command.metrics["position_error"].clone())
            orientations.append(command.metrics["orientation_error"].clone())
if positions:
    position, orientation = torch.cat(positions) * 1000.0, torch.rad2deg(torch.cat(orientations))
    print(f"목표 {len(position)}개.  자리의 오차: 평균 {position.mean():.1f} mm · 가장 큰 것 {position.max():.1f} mm · 2 cm 안에 든 비율 {(position < 20.0).float().mean() * 100:.0f} %", flush=True)
    print(f"            방향의 오차: 평균 {orientation.mean():.1f}° · 가장 큰 것 {orientation.max():.1f}°", flush=True)
env.close()
app.close()
