"""강의 32: 학습한 정책으로 매달린 진자를 세워 봅니다.

    ~/IsaacLab/isaaclab.sh -p 32_pinklab_pendulum/play.py                 # 창에서 16대를 계속 재생
    ~/IsaacLab/isaaclab.sh -p 32_pinklab_pendulum/play.py --headless --num_envs 256 --seconds 8     # 숫자로 평가
    ~/IsaacLab/isaaclab.sh -p 32_pinklab_pendulum/play.py --headless --num_envs 256 --seconds 8 --delay 20 --sensor_noise   # 강의 33
    옵션: --checkpoint 파일 (없으면 가장 최근 학습의 마지막 정책)  --random_start (학습 때처럼 아무 자세에서 시작)

평가: 모든 진자가 아래로 매달린 채 시작합니다. "섰다" = 진자가 똑바른 자세의 ±10° 안에 1초 동안 머문 것.
"""
import argparse
import glob
import math
import os

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--checkpoint", default=None, help="정책 파일 model_N.pt")
parser.add_argument("--num_envs", type=int, default=16)
parser.add_argument("--seconds", type=float, default=0.0, help="이 시간만 돌리고 결과를 출력한다. 0 이면 창을 닫을 때까지")
parser.add_argument("--delay", type=float, default=0.0, help="명령의 지연 [ms]")
parser.add_argument("--friction", type=float, default=1.0, help="진자 마찰의 배율")
parser.add_argument("--sensor_noise", action="store_true", help="센서의 눈금과 잡음을 넣는다")
parser.add_argument("--trace", default=None, help="0번 진자의 시간 · 각도 · 행동을 이 CSV 파일에 적는다")
parser.add_argument("--random_start", action="store_true", help="매달린 자세가 아니라 아무 자세에서 시작")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
app = AppLauncher(args).app

import torch  # noqa: E402
from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper  # noqa: E402
from rsl_rl.runners import OnPolicyRunner  # noqa: E402

from pendulum_env import PendulumEnv, PendulumEnvCfg  # noqa: E402
from ppo_cfg import PendulumPpoCfg  # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
checkpoint = args.checkpoint
if checkpoint is None:
    runs = sorted(glob.glob(os.path.join(REPO, "work", "pendulum_logs", "*")))
    assert runs, "학습 결과가 없습니다. train.py 를 먼저 실행하세요."
    checkpoint = max(glob.glob(os.path.join(runs[-1], "model_*.pt")), key=os.path.getmtime)
print(f"정책: {checkpoint}", flush=True)

env_cfg = PendulumEnvCfg()
env_cfg.scene.num_envs = args.num_envs
env_cfg.start_hanging = not args.random_start
env_cfg.delay_range = (args.delay / 1000.0, args.delay / 1000.0)
env_cfg.friction_range = (args.friction, args.friction)
env_cfg.sensor_noise = args.sensor_noise
env_cfg.episode_length_s = 8.0 if args.seconds <= 0 else args.seconds + 1.0
env = PendulumEnv(env_cfg)
wrapped = RslRlVecEnvWrapper(env)
agent_cfg = PendulumPpoCfg()
runner = OnPolicyRunner(wrapped, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
runner.load(checkpoint)
policy = runner.get_inference_policy(device=env.device)

step_time = env_cfg.sim.dt * env_cfg.decimation
steps = int(args.seconds / step_time) if args.seconds > 0 else None
inside = torch.zeros(args.num_envs, device=env.device)                 # ±10° 안에 연속으로 머문 스텝 수
stood_at = torch.full((args.num_envs,), float("nan"), device=env.device)
max_arm = torch.zeros(args.num_envs, device=env.device)
observation = wrapped.get_observations()
trace = []
step = 0
with torch.inference_mode():
    while app.is_running() and (steps is None or step < steps):
        observation, _, _, _ = wrapped.step(policy(observation))
        step += 1
        if args.trace:
            trace.append((step * step_time, math.degrees(float(env.upright_angle()[0])),
                          math.degrees(float(env.robot.data.joint_pos[0, env.arm])), float(env.action[0])))
        if steps is None:
            continue
        theta = env.upright_angle().abs()
        inside = torch.where(theta < math.radians(10.0), inside + 1, torch.zeros_like(inside))
        just_stood = (inside * step_time >= 1.0) & torch.isnan(stood_at)
        stood_at[just_stood] = step * step_time - 1.0                  # ±10° 안에 처음 들어온 때
        max_arm = torch.maximum(max_arm, env.robot.data.joint_pos[:, env.arm].abs())

if args.trace:
    with open(args.trace, "w") as file:
        file.write("time,pendulum_deg,arm_deg,action\n")
        file.writelines(f"{t:.4f},{p:.2f},{a:.2f},{u:.3f}\n" for t, p, a, u in trace)
    print(f"기록: {args.trace}", flush=True)
if steps is not None:
    stood = ~torch.isnan(stood_at)
    still_up = env.upright_angle().abs() < math.radians(10.0)
    print(f"{args.num_envs}대를 매달린 자세에서 {args.seconds:.0f}초 동안 돌렸습니다. "
          f"(지연 {args.delay:.0f} ms, 마찰 {args.friction:.1f}배, 센서 잡음 {'있음' if args.sensor_noise else '없음'})", flush=True)
    print(f"  선 진자: {int(stood.sum())}대 ({100.0 * stood.float().mean():.0f} %)", flush=True)
    if stood.any():
        print(f"  서는 데 걸린 시간: 평균 {stood_at[stood].mean():.2f}초, 가장 느린 것 {stood_at[stood].max():.2f}초", flush=True)
    print(f"  끝날 때 서 있는 진자: {int(still_up.sum())}대", flush=True)
    print(f"  팔이 가장 멀리 간 각도: 평균 {math.degrees(max_arm.mean()):.0f}°, 최대 {math.degrees(max_arm.max()):.0f}°", flush=True)
env.close()
app.close()
