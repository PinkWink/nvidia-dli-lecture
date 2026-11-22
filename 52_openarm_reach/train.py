"""강의 52: PL-2M 의 왼팔이 목표에 손을 뻗게 PPO (rsl_rl) 로 학습합니다.

    ~/IsaacLab/isaaclab.sh -p 52_openarm_reach/train.py --headless                      # 창 없이 1000번 (약 6분)
    ~/IsaacLab/isaaclab.sh -p 52_openarm_reach/train.py --headless --max_iterations 300
    ~/IsaacLab/isaaclab.sh -p 52_openarm_reach/train.py --headless --no_orientation     # 비교용: 방향 보상을 뺀다
    ~/IsaacLab/isaaclab.sh -p 52_openarm_reach/train.py --headless --wide               # 비교용: 목표의 범위를 닿지 않는 곳까지 넓게
    ~/IsaacLab/isaaclab.sh -p 52_openarm_reach/train.py --headless --carry_start        # 비교용: 운반 자세에서 시작
    ~/IsaacLab/isaaclab.sh -p 52_openarm_reach/train.py --headless --full_twist         # 비교용: 목표의 방향을 넓게 흔듦
    옵션: --num_envs 4096

결과: work/reach_logs/<날짜_시각>/model_N.pt (정책), events.* (TensorBoard 기록)
"""
import argparse
import os
import time
from datetime import datetime

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--num_envs", type=int, default=4096, help="동시에 돌릴 로봇 수")
parser.add_argument("--max_iterations", type=int, default=1000, help="정책을 고치는 횟수")
parser.add_argument("--no_orientation", action="store_true", help="비교용: 손의 방향을 맞추는 보상을 뺀다")
parser.add_argument("--wide", action="store_true", help="비교용: 목표가 나오는 상자를 처음에 어림으로 잡았던 넓은 것으로")
parser.add_argument("--carry_start", action="store_true", help="비교용: 시작 자세를 운반 자세로 (6번 관절이 한계에 닿아 있다)")
parser.add_argument("--full_twist", action="store_true", help="비교용: 목표의 방향을 OpenArm Reach 과제만큼 넓게 흔든다")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
app = AppLauncher(args).app

from isaaclab.envs import ManagerBasedRLEnv  # noqa: E402
from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper  # noqa: E402
from isaaclab_tasks.manager_based.manipulation.reach.config.openarm.unimanual.agents.rsl_rl_ppo_cfg import OpenArmReachPPORunnerCfg  # noqa: E402
from rsl_rl.runners import OnPolicyRunner  # noqa: E402

from pl2m_reach_env import CARRY, FULL_TWIST, WIDE_BOX, Pl2mReachEnvCfg, set_start, set_target_box, set_twist  # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
log_dir = os.path.join(REPO, "work", "reach_logs", datetime.now().strftime("%Y-%m-%d_%H-%M-%S") + ("_no_orientation" if args.no_orientation else "") + ("_wide" if args.wide else "") + ("_carry" if args.carry_start else "")
                       + ("_full_twist" if args.full_twist else ""))

env_cfg = Pl2mReachEnvCfg()
if args.wide:
    set_target_box(env_cfg, WIDE_BOX)
if args.carry_start:
    set_start(env_cfg, CARRY)
if args.full_twist:
    set_twist(env_cfg, *FULL_TWIST)
env_cfg.scene.num_envs = args.num_envs
if args.no_orientation:
    env_cfg.rewards.end_effector_orientation_tracking.weight = 0.0
env = RslRlVecEnvWrapper(ManagerBasedRLEnv(env_cfg))

agent_cfg = OpenArmReachPPORunnerCfg()                 # PPO 의 값은 Isaac Lab 의 OpenArm Reach 과제 것을 그대로 쓴다
agent_cfg.max_iterations = args.max_iterations
runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=log_dir, device=agent_cfg.device)

started = time.time()
runner.learn(num_learning_iterations=agent_cfg.max_iterations, init_at_random_ep_len=True)
print(f"학습 시간: {time.time() - started:.0f}초", flush=True)
print(f"저장: {log_dir}", flush=True)
env.close()
app.close()
