"""강의 55: MobilityGen 이 만든 데이터를 Python 으로 읽습니다. Isaac Sim 없이 돕니다 (numpy · Pillow · matplotlib).

    python3 55_pinky_mobilitygen/read_data.py                                   # 기록마다 한 줄씩
    python3 55_pinky_mobilitygen/read_data.py --map map.png                     # 지도 위에 다닌 길
    python3 55_pinky_mobilitygen/read_data.py --commands commands.png           # 속도 명령이 어떻게 퍼져 있는지
    python3 55_pinky_mobilitygen/read_data.py --frames frames.png --recording 0  # 한 기록의 영상 · 깊이 · 그때의 명령
    python3 55_pinky_mobilitygen/read_data.py --input work/mobilitygen/recordings      # 영상을 붙이기 전의 기록도 읽힌다

폴더 하나가 주행 하나다:
    config.json                     어떤 로봇 · 어떤 시나리오 · 어떤 장면
    occupancy_map/map.png · .yaml   그때 쓴 지도
    state/common/00000123.npy       스텝마다 하나: 로봇의 자리 · 자세 · 관절 · 속도 · 속도 명령(action) · 따라가던 길
    state/rgb/<카메라>/…jpg          재생하며 그린 색 영상
    state/depth/<카메라>/…png        깊이 영상. 16비트 정수 = 65535 / (1 + 거리[m])
"""
import argparse
import glob
import json
import math
import os

import numpy as np
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
WORK = os.path.abspath(os.path.join(HERE, "..", "work", "mobilitygen"))
PHYSICS_DT = 1.0 / 240.0
COLOR, DEPTH = "state/rgb/robot.front_camera.camera.rgb_image", "state/depth/robot.front_camera.camera.depth_image"


class Recording:
    """기록 폴더 하나."""

    def __init__(self, path):
        self.path, self.name = path, os.path.basename(path)
        self.config = json.load(open(os.path.join(path, "config.json")))
        self.steps = sorted(int(os.path.basename(file)[:-4]) for file in glob.glob(os.path.join(path, "state", "common", "*.npy")))

    def state(self, step):
        """한 스텝의 상태: 이름 → 값 인 사전."""
        return np.load(os.path.join(self.path, "state", "common", f"{step:08d}.npy"), allow_pickle=True).item()

    def color(self, step):
        return np.array(Image.open(os.path.join(self.path, COLOR, f"{step:08d}.jpg")))

    def depth(self, step):
        """깊이 영상을 미터로. 저장된 값 = 65535 / (1 + 거리)."""
        stored = np.array(Image.open(os.path.join(self.path, DEPTH, f"{step:08d}.png"))).astype(np.float32)
        return 65535.0 / np.maximum(stored, 1.0) - 1.0

    def has_images(self):
        return os.path.isdir(os.path.join(self.path, COLOR))

    def table(self, every=1):
        """스텝들을 표로: 시간, x, y, 방향, 명령한 (v, ω), 실제 (v, ω)."""
        rows = []
        for step in self.steps[::every]:
            s = self.state(step)
            (x, y, _), (w, qx, qy, qz) = s["robot.position"], s["robot.orientation"]
            rows.append((step * PHYSICS_DT, x, y, math.atan2(2 * (w * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz)), *s["robot.action"],
                         float(np.hypot(*s["robot.linear_velocity"][:2])), s["robot.angular_velocity"][2]))
        return np.array(rows)

    def plan(self):
        """따라가려던 길 (N, 2). 길을 따라가는 시나리오가 아니면 None."""
        return self.state(self.steps[0]).get("target_path")

    def map(self):
        """지도 그림과, 그림이 덮는 범위 (x 최소, x 최대, y 최소, y 최대) [m]."""
        folder = os.path.join(self.path, "occupancy_map")
        text = dict(line.split(":", 1) for line in open(os.path.join(folder, "map.yaml")) if ":" in line)
        image = np.array(Image.open(os.path.join(folder, text["image"].strip())).convert("L"))
        resolution, origin = float(text["resolution"]), json.loads(text["origin"])
        return image, (origin[0], origin[0] + image.shape[1] * resolution, origin[1], origin[1] + image.shape[0] * resolution)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", default=os.path.join(WORK, "replays"), help="기록들이 든 폴더")
    parser.add_argument("--recording", type=int, default=0, help="--frames 로 볼 기록의 번호 (0부터)")
    parser.add_argument("--map", metavar="PNG", help="지도 위에 다닌 길을 그려 저장한다")
    parser.add_argument("--commands", metavar="PNG", help="속도 명령의 분포를 그려 저장한다")
    parser.add_argument("--frames", metavar="PNG", help="한 기록에서 영상 넷을 골라 깊이 · 명령과 함께 저장한다")
    args = parser.parse_args()
    recordings = [Recording(path) for path in sorted(glob.glob(os.path.join(args.input, "*"))) if os.path.exists(os.path.join(path, "config.json"))]
    if not recordings:
        raise SystemExit(f"{args.input} 에 기록이 없음")
    first = recordings[0]
    print(f"{args.input}: 기록 {len(recordings)}개.  로봇 {first.config['robot_type']}, 시나리오 {first.config['scenario_type']}")
    print("상태 하나에 든 것:", ", ".join(f"{key} {np.shape(value)}" for key, value in first.state(first.steps[0]).items() if value is not None))
    tables, total_time, total_distance, frames = [], 0.0, 0.0, 0
    for number, recording in enumerate(recordings):
        gap = recording.steps[1] - recording.steps[0] if len(recording.steps) > 1 else 1
        table = recording.table(every=max(1, 24 // gap))                      # 0.1초 간격으로 읽는다
        seconds = (recording.steps[-1] + 1) * PHYSICS_DT
        distance = float(np.hypot(np.diff(table[:, 1]), np.diff(table[:, 2])).sum())
        plan = recording.plan()
        left = f", 목표까지 남은 거리 {np.hypot(*(table[-1, 1:3] - plan[-1])):.2f} m" if plan is not None else ""
        images = len(glob.glob(os.path.join(recording.path, COLOR, "*.jpg")))
        print(f"  [{number}] {recording.name[11:19]}  {seconds:5.1f}초 · {distance:.2f} m · 영상 {images}장{left}")
        tables.append(table)
        total_time, total_distance, frames = total_time + seconds, total_distance + distance, frames + images
    everything = np.concatenate(tables)
    print(f"모두 {total_time:.0f}초 · {total_distance:.1f} m · 영상 {frames}장")
    print(f"속도 명령: 직진 평균 {everything[:, 4].mean():.3f} m/s,  회전 |ω| 평균 {np.abs(everything[:, 5]).mean():.2f} rad/s,  "
          f"왼쪽으로 도는 중 {(everything[:, 5] > 0.2).mean() * 100:.0f} % · 곧게 {(np.abs(everything[:, 5]) <= 0.2).mean() * 100:.0f} % · 오른쪽 {(everything[:, 5] < -0.2).mean() * 100:.0f} %")
    if not (args.map or args.commands or args.frames):
        return
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    if args.map:
        image, extent = first.map()
        figure, ax = plt.subplots(figsize=(6.4, 6.4), dpi=120)
        ax.imshow(image, cmap="gray", extent=extent, vmin=-200, vmax=255)
        for number, (recording, table) in enumerate(zip(recordings, tables)):
            line, = ax.plot(table[:, 1], table[:, 2], lw=2, label=f"[{number}] {table[-1, 0]:.0f} s")
            plan = recording.plan()
            if plan is not None:
                ax.plot(plan[:, 0], plan[:, 1], ls=":", lw=1, color=line.get_color())
                ax.plot(*plan[-1], marker="x", ms=7, color=line.get_color())
            ax.plot(table[0, 1], table[0, 2], marker="o", ms=5, color=line.get_color())
        ax.set_xlabel("x [m]"), ax.set_ylabel("y [m]")
        ax.set_title("driven (solid) · planned (dotted) · start (o) · goal (x)", fontsize=10)
        if len(recordings) <= 10:
            ax.legend(fontsize=7, ncol=2, loc="upper right")
        figure.tight_layout(), figure.savefig(args.map)
        print("저장:", args.map)
    if args.commands:
        figure, axes = plt.subplots(1, 2, figsize=(9, 3.6), dpi=120)
        axes[0].hist(everything[:, 4], bins=30, color="#4C78A8")
        axes[0].set_xlabel("commanded linear velocity [m/s]"), axes[0].set_ylabel("samples (every 0.1 s)")
        axes[1].hist(everything[:, 5], bins=40, color="#4C78A8")
        axes[1].set_xlabel("commanded angular velocity [rad/s]  (+ = left)")
        figure.tight_layout(), figure.savefig(args.commands)
        print("저장:", args.commands)
    if args.frames:
        recording = recordings[args.recording]
        if not recording.has_images():
            raise SystemExit("이 기록에는 영상이 없음: replay.py 로 먼저 그릴 것")
        picks = [recording.steps[int(i)] for i in np.linspace(0, len(recording.steps) - 1, 4)]
        figure, axes = plt.subplots(2, 4, figsize=(15, 4.6), dpi=110)
        for column, step in enumerate(picks):
            state, depth = recording.state(step), recording.depth(step)
            axes[0, column].imshow(recording.color(step))
            axes[0, column].set_title(f"t = {step * PHYSICS_DT:.1f} s   v = {state['robot.action'][0]:.2f} m/s   w = {state['robot.action'][1]:+.2f} rad/s", fontsize=9)
            shown = axes[1, column].imshow(np.clip(depth, 0, 3.0), cmap="viridis_r", vmin=0, vmax=3.0)
            axes[1, column].set_title(f"depth: center {depth[depth.shape[0] // 2, depth.shape[1] // 2]:.2f} m", fontsize=9)
            axes[0, column].axis("off"), axes[1, column].axis("off")
        figure.colorbar(shown, ax=axes[1, :], fraction=0.015, label="m")
        figure.savefig(args.frames, bbox_inches="tight")
        print("저장:", args.frames)


if __name__ == "__main__":
    main()
