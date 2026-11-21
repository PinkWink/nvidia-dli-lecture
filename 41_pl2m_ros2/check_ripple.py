#!/usr/bin/env python3
"""강의 41: pl2m_ros2.py --log 로 남긴 기록에서, 첫 궤적 3초 동안 목표와 구동 토크가 얼마나 고르게 바뀌었는지 봅니다.

    python3 41_pl2m_ros2/check_ripple.py work/pl2m/ros2_wall60.json work/pl2m/ros2_sync.json

왼팔 1번 관절을 봅니다. --compensate left 로 실행한 기록이어야 "구동이 낸 토크" 가 0 근처에서 출렁임만 보입니다.
"""
import json
import sys

import numpy as np

print(f"{'기록':<28} {'목표가 안 바뀐 스텝':>14} {'한 스텝 최대[°]':>12} {'구동 토크 범위[N·m]':>20} {'스텝 사이 변화 평균':>14} {'오차 최대[°]':>10}")
for path in sys.argv[1:]:
    data = json.load(open(path))
    time = np.array(data["time"])
    target, angle = np.array(data["left"]["target"]), np.array(data["left"]["angle"])
    drive = np.array(data["left"]["total"])[:, 0] - np.array(data["left"]["added"])[:, 0]      # 전체 토크 − 보상으로 넣은 토크
    moving = np.abs(np.diff(target, axis=0)).max(axis=1) > 1e-4
    starts = [time[i + 1] for i in range(20, len(moving)) if moving[i] and not moving[i - 20:i].any()]
    if not starts:
        print(f"{path:<28} 움직인 구간이 없음")
        continue
    window = (time >= starts[0]) & (time <= starts[0] + 3.0)
    steps = np.abs(np.diff(target[window, 0]))
    print(f"{path.split('/')[-1]:<28} {int((steps < 1e-6).sum()):>10} / {len(steps)} {steps.max():>12.2f} "
          f"{drive[window].min():>10.1f} ~ {drive[window].max():<6.1f} {np.abs(np.diff(drive[window])).mean():>14.2f} "
          f"{np.abs(angle[window, 0] - target[window, 0]).max():>10.2f}")
