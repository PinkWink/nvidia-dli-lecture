"""강의 35: ONNX 정책을 Isaac Sim (Isaac Lab Cartpole) 에서 정해진 시작 자세로 돌려 궤적을 CSV 로 남긴다.

    cd ~/Cartpole
    ~/IsaacLab/isaaclab.sh -p ~/nvidia-dli-lecture/35_sim_to_real/isaac_rollout.py --headless [--onnx <policy.onnx>] [--zero]

시작 자세 4 개 (카트 위치 m, 막대 각도 rad): (0, 0.2) · (0, -0.4) · (1.0, 0.6) · (-0.5, -0.2), 속도 0.
결과: isaac_rollout.csv (env, t, x, theta, x_dot, theta_dot, action). --zero 면 행동 0 (막대가 그냥 쓰러짐).
"""
import argparse
import glob

from isaaclab.app import AppLauncher

ap = argparse.ArgumentParser()
ap.add_argument("--onnx", default=None)
ap.add_argument("--zero", action="store_true")
ap.add_argument("--seconds", type=float, default=5.0)
ap.add_argument("--out", default="isaac_rollout.csv")
AppLauncher.add_app_launcher_args(ap)
args = ap.parse_args()
app = AppLauncher(args).app

import csv  # noqa: E402

import numpy as np  # noqa: E402
import onnxruntime as ort  # noqa: E402
import torch  # noqa: E402

from isaaclab.envs import ManagerBasedRLEnv  # noqa: E402

import Cartpole.tasks  # noqa: E402,F401
from Cartpole.tasks.manager_based.cartpole.cartpole_env_cfg import CartpoleEnvCfg  # noqa: E402

STARTS = [(0.0, 0.2), (0.0, -0.4), (1.0, 0.6), (-0.5, -0.2)]
cfg = CartpoleEnvCfg()
cfg.scene.num_envs = len(STARTS)
cfg.episode_length_s = 1000.0          # 시간 초과로 리셋되지 않게
cfg.terminations.cart_out_of_bounds = None
env = ManagerBasedRLEnv(cfg)
robot = env.scene["robot"]
onnx = None if args.zero else (args.onnx or sorted(glob.glob("logs/skrl/cartpole_direct/*/exported/policy.onnx"))[-1])
sess = ort.InferenceSession(onnx) if onnx else None
print("[ROLLOUT] policy:", onnx or "zero action", flush=True)

env.reset()
pos = torch.tensor(STARTS, device=env.device)
robot.write_joint_state_to_sim(pos, torch.zeros_like(pos))
env.scene.write_data_to_sim(); env.sim.forward(); env.scene.update(0.0)
rows = []
steps = int(args.seconds / env.step_dt)
with torch.inference_mode():
    for k in range(steps):
        q, qd = robot.data.joint_pos, robot.data.joint_vel
        obs = torch.cat([q, qd], dim=1).cpu().numpy().astype(np.float32)   # joint_pos_rel(기본 0) + joint_vel_rel
        act = np.zeros((len(STARTS), 1), np.float32) if sess is None else sess.run(None, {"obs": obs})[0]
        for e in range(len(STARTS)):
            rows.append([e, round(k * env.step_dt, 4), *obs[e].tolist(), float(act[e, 0])])
        env.step(torch.tensor(act, device=env.device))
with open(args.out, "w", newline="") as f:
    w = csv.writer(f); w.writerow(["env", "t", "x", "theta", "x_dot", "theta_dot", "action"]); w.writerows(rows)
print("[ROLLOUT] saved", args.out, len(rows), "rows", flush=True)
env.close()
app.close()
