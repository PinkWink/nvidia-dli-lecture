"""강의 35: Isaac Sim 없이 numpy 로 만든 Cartpole 에서 ONNX 정책을 돌린다.

    cd ~/Cartpole
    ~/IsaacLab/isaaclab.sh -p ~/nvidia-dli-lecture/35_sim_to_real/cartpole_numpy.py                  # 기본: 학습과 같은 값
    ~/IsaacLab/isaaclab.sh -p ~/nvidia-dli-lecture/35_sim_to_real/cartpole_numpy.py --pole_mass 3     # 막대를 3 배로
    (옵션: --onnx 파일, --zero, --pole_mass, --cart_mass, --damping, --delay 스텝 수, --obs_noise, --swap_obs, --flip_angle, --compare isaac_rollout.csv)

isaaclab.sh -p 는 Isaac Sim 의 Python 만 빌려 쓰는 것입니다 (시뮬레이터는 띄우지 않음, numpy + onnxruntime 만 사용).
결과: 시작 자세 4 개의 성공(5 초 동안 |막대 각도| < 0.5 rad 유지 · |카트| < 3 m) 여부, numpy_rollout.png 그림.
"""
import argparse
import glob

import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("--onnx", default=None)
ap.add_argument("--zero", action="store_true")
ap.add_argument("--seconds", type=float, default=5.0)
ap.add_argument("--cart_mass", type=float, default=1.0)       # cartpole.usd: cart 1.0 kg
ap.add_argument("--pole_mass", type=float, default=1.0)       # cartpole.usd: pole 1.0 kg
ap.add_argument("--pole_com", type=float, default=0.47)       # 관절 → 막대 무게중심 (m)
ap.add_argument("--pole_inertia", type=float, default=0.0736)  # 무게중심에 대한 관성 (막대 길이 0.94 m 가정: m L^2 / 12)
ap.add_argument("--damping", type=float, default=10.0)        # cart_actuator damping (Isaac Lab CARTPOLE_CFG)
ap.add_argument("--delay", type=int, default=0, help="행동이 몇 스텝 늦게 들어가나 (통신 · 모터 지연 흉내)")
ap.add_argument("--obs_noise", type=float, default=0.0, help="관측에 더할 가우시안 잡음 표준편차")
ap.add_argument("--swap_obs", action="store_true", help="관측 순서 실수: (위치 2, 속도 2) 대신 (카트 2, 막대 2)")
ap.add_argument("--flip_angle", action="store_true", help="각도 부호 실수: 막대 각도 · 각속도 부호를 반대로")
ap.add_argument("--compare", default=None, help="isaac_rollout.py 가 만든 CSV 와 겹쳐 그리기")
ap.add_argument("--random_starts", type=int, default=0, help="시작 자세를 N 개 무작위로 (x ±1 m, theta ±0.6 rad) — 성공률 비교용, 그림 없음")
ap.add_argument("--out", default="numpy_rollout.png")
a = ap.parse_args()

G, DT, DECIMATION, SCALE, LIMIT = 9.81, 1.0 / 120.0, 2, 100.0, 400.0
STARTS = [(0.0, 0.2), (0.0, -0.4), (1.0, 0.6), (-0.5, -0.2)]


def physics_step(s, f_cmd):
    """한 물리 스텝 (dt = 1/120). s = [x, theta, x_dot, theta_dot]. Isaac 규칙: theta 가 + 면 막대는 -x 쪽으로 기움."""
    x, th, xd, thd = s
    phi, phid = -th, -thd                       # phi: +x 쪽으로 기운 각도 (교과서 식)
    f = np.clip(f_cmd, -LIMIT, LIMIT) - a.damping * xd
    M, m, l, I = a.cart_mass, a.pole_mass, a.pole_com, a.pole_inertia
    c, sn = np.cos(phi), np.sin(phi)
    A = np.array([[M + m, m * l * c], [m * l * c, I + m * l * l]])
    b = np.array([f + m * l * sn * phid ** 2, m * G * l * sn])
    xdd, phidd = np.linalg.solve(A, b)
    xd, phid = xd + xdd * DT, phid + phidd * DT  # semi-implicit Euler (PhysX 와 같은 방식)
    return np.array([x + xd * DT, -(phi + phid * DT), xd, -phid])


def policy_fn():
    if a.zero:
        return lambda obs: np.zeros((1, 1), np.float32), "zero action"
    import onnxruntime as ort
    path = a.onnx or sorted(glob.glob("logs/skrl/cartpole_direct/*/exported/policy.onnx"))[-1]
    sess = ort.InferenceSession(path)
    return (lambda obs: sess.run(None, {"obs": obs})[0]), path


pol, name = policy_fn()
rng = np.random.default_rng(0)
if a.random_starts:
    STARTS = [tuple(v) for v in np.random.default_rng(1).uniform([-1.0, -0.6], [1.0, 0.6], (a.random_starts, 2))]
trajs, ok = [], []
for x0, th0 in STARTS:
    s = np.array([x0, th0, 0.0, 0.0])
    queue = [0.0] * a.delay
    tr = []
    for k in range(int(a.seconds / (DT * DECIMATION))):
        o = s.copy()
        if a.flip_angle:
            o[1], o[3] = -o[1], -o[3]
        o = o + rng.normal(0, a.obs_noise, 4) if a.obs_noise else o
        if a.swap_obs:
            o = o[[0, 2, 1, 3]]
        act = float(pol(o.astype(np.float32)[None])[0, 0])
        queue.append(act)
        applied = queue.pop(0)
        tr.append([k * DT * DECIMATION, *s, applied])
        for _ in range(DECIMATION):
            s = physics_step(s, SCALE * applied)
    tr = np.array(tr)
    good = bool(np.all(np.abs(np.arctan2(np.sin(tr[-60:, 2]), np.cos(tr[-60:, 2]))) < 0.5) and np.all(np.abs(tr[:, 1]) < 3.0))
    trajs.append(tr); ok.append(good)
    if not a.random_starts:
        print(f"[NUMPY] start (x={x0:+.1f}, theta={th0:+.1f}) -> {'OK  ' if good else 'FAIL'}  final x={tr[-1, 1]:+.2f} m, theta={tr[-1, 2]:+.2f} rad")
print(f"[NUMPY] policy: {name}")
print(f"[NUMPY] success {sum(ok)}/{len(ok)}  (pole_mass={a.pole_mass}, delay={a.delay}, obs_noise={a.obs_noise}, swap={a.swap_obs}, flip={a.flip_angle})")

if a.random_starts:
    raise SystemExit(0)
import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

for fn in ("NanumGothic", "Noto Sans CJK KR"):
    if any(f.name == fn for f in matplotlib.font_manager.fontManager.ttflist):
        plt.rcParams["font.family"] = fn
        break
plt.rcParams["axes.unicode_minus"] = False
ref = None
if a.compare:
    import csv
    rows = list(csv.DictReader(open(a.compare)))
    ref = [np.array([[float(r[k]) for k in ("t", "x", "theta")] for r in rows if int(r["env"]) == e]) for e in range(len(STARTS))]
fig, axes = plt.subplots(1, 2, figsize=(12, 3.8), dpi=120)
for e, tr in enumerate(trajs):
    col = f"C{e}"
    axes[0].plot(tr[:, 0], tr[:, 2], color=col, lw=1.8, label=f"시작 {STARTS[e]}")
    axes[1].plot(tr[:, 0], tr[:, 1], color=col, lw=1.8)
    if ref is not None:
        axes[0].plot(ref[e][:, 0], ref[e][:, 2], color=col, lw=1.2, ls="--")
        axes[1].plot(ref[e][:, 0], ref[e][:, 1], color=col, lw=1.2, ls="--")
axes[0].set_title("막대 각도 theta (rad)" + ("  — 실선 numpy · 점선 Isaac" if ref is not None else ""))
axes[1].set_title("카트 위치 x (m)")
for ax in axes:
    ax.set_xlabel("시간 (s)"); ax.grid(alpha=0.3)
axes[0].legend(fontsize=8)
fig.tight_layout(); fig.savefig(a.out, facecolor="white")
print("[NUMPY] saved", a.out)
