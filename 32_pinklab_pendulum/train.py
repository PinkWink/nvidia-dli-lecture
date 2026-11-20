"""강의 32: 핑크랩 로터리 역진자를 PPO (rsl_rl) 로 학습합니다.

    ~/IsaacLab/isaaclab.sh -p 32_pinklab_pendulum/train.py --headless            # 창 없이 600번 (약 8분)
    ~/IsaacLab/isaaclab.sh -p 32_pinklab_pendulum/train.py                       # 창을 띄우고
    ~/IsaacLab/isaaclab.sh -p 32_pinklab_pendulum/train.py --headless --realistic   # 강의 33: 지연 · 마찰 · 센서 잡음을 넣고
    옵션: --num_envs 4096  --max_iterations 600  --energy_weight 0.5

결과: work/pendulum_logs/<날짜_시각>/model_N.pt (정책), events.* (TensorBoard 기록)
"""
import argparse
import os
import time
from datetime import datetime

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--num_envs", type=int, default=4096, help="동시에 돌릴 진자 수")
parser.add_argument("--max_iterations", type=int, default=600, help="정책을 고치는 횟수")
parser.add_argument("--energy_weight", type=float, default=0.5, help="보상에서 에너지 항의 가중치. 0 이면 뺀다")
parser.add_argument("--arm_center_weight", type=float, default=0.02, help="보상에서 팔이 가운데를 벗어난 벌점의 가중치")
parser.add_argument("--realistic", action="store_true",
                    help="실물의 불완전함을 넣는다: 지연 0 ~ 40 ms, 마찰 0.5 ~ 1.5배, 센서의 눈금과 잡음")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
app = AppLauncher(args).app

from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper  # noqa: E402
from rsl_rl.runners import OnPolicyRunner  # noqa: E402

from pendulum_env import PendulumEnv, PendulumEnvCfg  # noqa: E402
from ppo_cfg import PendulumPpoCfg  # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
log_dir = os.path.join(REPO, "work", "pendulum_logs", datetime.now().strftime("%Y-%m-%d_%H-%M-%S") + ("_realistic" if args.realistic else ""))

env_cfg = PendulumEnvCfg()
env_cfg.scene.num_envs = args.num_envs
env_cfg.energy_weight = args.energy_weight
env_cfg.arm_center_weight = args.arm_center_weight
if args.realistic:
    env_cfg.delay_range = (0.0, 0.040)
    env_cfg.friction_range = (0.5, 1.5)
    env_cfg.sensor_noise = True
env = RslRlVecEnvWrapper(PendulumEnv(env_cfg))

agent_cfg = PendulumPpoCfg()
agent_cfg.max_iterations = args.max_iterations
runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=log_dir, device=agent_cfg.device)

started = time.time()
runner.learn(num_learning_iterations=agent_cfg.max_iterations, init_at_random_ep_len=True)
print(f"학습 시간: {time.time() - started:.0f}초", flush=True)
print(f"저장: {log_dir}", flush=True)
env.close()
app.close()
