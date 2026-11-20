"""강의 35: skrl 로 학습한 Cartpole 정책을 ONNX 파일로 내보낸다 (관측 정규화까지 포함).

    cd ~/Cartpole
    ~/IsaacLab/isaaclab.sh -p ~/nvidia-dli-lecture/35_sim_to_real/export_onnx.py              # 가장 최근 학습의 best_agent.pt
    ~/IsaacLab/isaaclab.sh -p ~/nvidia-dli-lecture/35_sim_to_real/export_onnx.py <체크포인트.pt> [--no_norm]

결과: 체크포인트 폴더 옆 exported/policy.onnx (--no_norm 이면 policy_no_norm.onnx).
Isaac Sim 을 띄우지 않고 PyTorch 만 씁니다. 입력 obs [N, 4] = (카트 위치, 막대 각도, 카트 속도, 막대 각속도), 출력 action [N, 1].
"""
import argparse
import glob
import os

import torch
import torch.nn as nn

ap = argparse.ArgumentParser()
ap.add_argument("checkpoint", nargs="?", default=None)
ap.add_argument("--no_norm", action="store_true", help="관측 정규화를 빼고 내보내기 (일부러 틀린 정책 만들기)")
a = ap.parse_args()

ckpt = a.checkpoint or sorted(glob.glob("logs/skrl/cartpole_direct/*/checkpoints/best_agent.pt"))[-1]
ck = torch.load(ckpt, map_location="cpu", weights_only=False)
pol, pre = ck["policy"], ck["state_preprocessor"]
print("checkpoint :", ckpt)
print("layers     :", {k: tuple(v.shape) for k, v in pol.items() if "value" not in k})
print("obs mean   :", pre["running_mean"].numpy().round(4))
print("obs std    :", pre["running_variance"].sqrt().numpy().round(4))


class Policy(nn.Module):
    """skrl 의 GaussianMixin 정책에서 평균 행동만 (재생 · 실물 배치 때와 같음)."""

    def __init__(self, norm=True):
        super().__init__()
        self.norm = norm
        self.register_buffer("mean", pre["running_mean"].float())
        self.register_buffer("std", pre["running_variance"].float().sqrt())
        self.net = nn.Sequential(nn.Linear(4, 32), nn.ELU(), nn.Linear(32, 32), nn.ELU(), nn.Linear(32, 1))
        self.net[0].weight.data, self.net[0].bias.data = pol["net_container.0.weight"], pol["net_container.0.bias"]
        self.net[2].weight.data, self.net[2].bias.data = pol["net_container.2.weight"], pol["net_container.2.bias"]
        self.net[4].weight.data, self.net[4].bias.data = pol["policy_layer.weight"], pol["policy_layer.bias"]

    def forward(self, obs):
        if self.norm:
            obs = torch.clamp((obs - self.mean) / (self.std + 1e-8), -5.0, 5.0)
        return self.net(obs)


m = Policy(norm=not a.no_norm).eval()
out_dir = os.path.join(os.path.dirname(os.path.dirname(ckpt)), "exported")
os.makedirs(out_dir, exist_ok=True)
out = os.path.join(out_dir, "policy_no_norm.onnx" if a.no_norm else "policy.onnx")
torch.onnx.export(m, torch.zeros(1, 4), out, input_names=["obs"], output_names=["action"],
                  dynamic_axes={"obs": {0: "batch"}, "action": {0: "batch"}}, opset_version=17, dynamo=False)
x = torch.tensor([[0.0, 0.1, 0.0, 0.0]])
print("test obs   :", x.numpy(), "-> action", m(x).item())
print("saved      :", out, f"({os.path.getsize(out)} bytes)")
