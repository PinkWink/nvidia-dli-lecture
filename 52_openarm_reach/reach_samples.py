"""강의 52: PL-2M 의 왼팔이 닿는 범위를 잽니다. 관절을 한계 안에서 무작위로 놓고, 끝 링크가 어디에 어느 방향으로 오는지 모읍니다.

    ~/IsaacLab/isaaclab.sh -p 52_openarm_reach/reach_samples.py --headless                    # 2만 4천 가지 자세
    ~/IsaacLab/isaaclab.sh -p 52_openarm_reach/reach_samples.py --headless --save work/reach_samples.npy

목표가 나오는 상자(pl2m_reach_env.py 의 TARGET_BOX)를 정하기 전에 돌려 본다. 닿지 않는 곳에 목표를 내면 아무리 학습해도 오차가 남는다.
"""
import argparse
import math
import os

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--save", metavar="NPY", help="표본을 이 파일에 저장한다 (자리 3 · 방향 4 · 관절 7)")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
app = AppLauncher(args).app

import numpy as np  # noqa: E402
import torch  # noqa: E402
from isaaclab.envs import ManagerBasedRLEnv  # noqa: E402
from isaaclab.utils.math import quat_error_magnitude, quat_from_euler_xyz, subtract_frame_transforms  # noqa: E402

from pl2m_reach_env import HAND, TARGET_BOX, WIDE_BOX, Pl2mReachEnvCfg  # noqa: E402

cfg = Pl2mReachEnvCfg()
cfg.scene.num_envs = 4096
cfg.events.reset_robot_joints = None
env = ManagerBasedRLEnv(cfg)
env.reset()
robot = env.scene["robot"]
joints = [robot.joint_names.index(f"openarm_left_joint{i}") for i in range(1, 8)]
limits = robot.data.soft_joint_pos_limits[0, joints]
hand = robot.body_names.index(HAND)


def hand_pose():
    """끝 링크의 자리와 방향 (로봇 기준)."""
    return subtract_frame_transforms(robot.data.root_pos_w, robot.data.root_quat_w, robot.data.body_pos_w[:, hand], robot.data.body_quat_w[:, hand])


zero = torch.zeros(1, device=robot.device)
grasp = quat_from_euler_xyz(zero, zero + math.pi / 2.0, zero)        # 컵을 쥘 때의 손 방향: 손이 앞을 봄 (pitch 90°). 운반 자세의 방향과 3° 차이다
generator = torch.Generator(device=robot.device).manual_seed(0)
rows = []
for _ in range(6):
    q = robot.data.default_joint_pos.clone()
    q[:, joints] = limits[:, 0] + (limits[:, 1] - limits[:, 0]) * torch.rand(q.shape[0], 7, generator=generator, device=q.device)
    robot.write_joint_state_to_sim(q, torch.zeros_like(q))         # 관절을 그 값으로 바로 놓는다
    for _ in range(3):
        robot.set_joint_position_target(q)
        robot.write_data_to_sim()
        env.sim.step(render=False)
        robot.update(env.sim.get_physics_dt())
    position, orientation = hand_pose()
    turned = quat_error_magnitude(orientation, grasp.expand_as(orientation))               # 쥐는 방향에서 벗어난 각 [rad]
    rows.append(torch.cat([position, orientation, robot.data.joint_pos[:, joints], turned[:, None]], dim=1).cpu().numpy())
samples = np.concatenate(rows)
position, turned = samples[:, :3], np.degrees(samples[:, -1])
print(f"자세 {len(samples)}가지.  끝 링크가 간 범위: 앞 {position[:, 0].min():.2f} ~ {position[:, 0].max():.2f} m,  왼쪽 {position[:, 1].min():.2f} ~ {position[:, 1].max():.2f} m,  "
      f"높이 {position[:, 2].min():.2f} ~ {position[:, 2].max():.2f} m", flush=True)
near = turned < 35.0
print(f"그 가운데 쥐는 방향에서 35° 안인 것 {near.sum()}가지: 앞 {np.percentile(position[near, 0], 5):.2f} ~ {np.percentile(position[near, 0], 95):.2f} m,  "
      f"왼쪽 {np.percentile(position[near, 1], 5):.2f} ~ {np.percentile(position[near, 1], 95):.2f} m,  높이 {np.percentile(position[near, 2], 5):.2f} ~ {np.percentile(position[near, 2], 95):.2f} m  (5 ~ 95 %)", flush=True)
for name, box in (("TARGET_BOX", TARGET_BOX), ("WIDE_BOX", WIDE_BOX)):
    inside = np.all([(position[:, k] >= box[k][0]) & (position[:, k] <= box[k][1]) for k in range(3)], axis=0)
    corner = np.array([box[0][1], box[1][0], box[2][0]])                                   # 가장 먼 모서리: 가장 앞 · 가장 안쪽 · 가장 아래
    print(f"{name}: 상자 안에 든 자세 {inside.sum()}가지 (쥐는 방향 35° 안 {(inside & near).sum()}가지),  "
          f"가장 먼 모서리에서 가장 가까운 표본까지 {np.linalg.norm(position - corner, axis=1).min() * 100:.1f} cm", flush=True)
if args.save:
    np.save(args.save, samples)
    print("저장:", args.save, flush=True)
env.close()
app.close()
