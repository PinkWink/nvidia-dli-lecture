"""강의 44: Nav2 가 쓸 부스 지도를 만듭니다. 원본의 지도에 부스의 테두리를 벽으로 그려 넣습니다.

    python3 44_pl2m_nav/make_map.py            # work/booth/booth_map.pgm · booth_map.yaml

원본의 지도: 원본이 자기 MuJoCo 장면에서 바로 만든 것 (SLAM 으로 다 만든 지도를 대신한다고 적혀 있음). 벽 · 가구 · 기둥이 들어 있고 사람은 없다.
더하는 것: 부스(9 × 6 m)의 테두리. 벽이 없는 쪽은 통로와 이어져 있다. 로봇이 통로로 나가는 길을 Nav2 가 내지 않게 벽으로 막는다.
"""
import os

import numpy as np
from PIL import Image

from booth import AREA, MAP, SOURCE_MAP


def read_yaml(path):
    """지도 설정 파일에서 필요한 값만 읽는다."""
    values = dict(line.split(":", 1) for line in open(path).read().splitlines() if ":" in line)
    origin = [float(v) for v in values["origin"].strip(" []").split(",")]
    return values["image"].strip(), float(values["resolution"]), origin


if __name__ == "__main__":
    if not os.path.isfile(SOURCE_MAP):
        raise SystemExit(f"원본 지도가 없습니다: {SOURCE_MAP}\n먼저 강의 37 의 준비를 하세요:  bash ~/nvidia-dli-lecture/37_pl2m_import/prepare_pl2m.sh")
    image_name, resolution, origin = read_yaml(SOURCE_MAP)
    grid = np.array(Image.open(os.path.join(os.path.dirname(SOURCE_MAP), image_name)))      # 0 = 막힌 곳, 254 = 빈 곳. 첫 줄이 y 가 가장 큰 쪽
    height, width = grid.shape
    before = int((grid == 0).sum())

    def cell(x, y):
        """부스 좌표 (x, y) → 그림의 (줄, 칸)."""
        return height - 1 - int((y - origin[1]) / resolution), int((x - origin[0]) / resolution)

    x0, y0, x1, y1 = AREA
    top, left = cell(x0, y1 + resolution)          # 테두리 바로 바깥 칸에 그린다: 부스 안은 지우지 않는다
    bottom, right = cell(x1 + resolution, y0 - resolution)
    grid[top, left:right + 1] = 0
    grid[bottom, left:right + 1] = 0
    grid[top:bottom + 1, left] = 0
    grid[top:bottom + 1, right] = 0
    os.makedirs(os.path.dirname(MAP), exist_ok=True)
    Image.fromarray(grid).save(MAP.replace(".yaml", ".pgm"))
    with open(MAP, "w") as file:
        file.write(f"image: {os.path.basename(MAP).replace('.yaml', '.pgm')}\nmode: trinary\nresolution: {resolution}\n"
                   f"origin: [{origin[0]}, {origin[1]}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")
    print(f"원본 지도: {width} × {height} 칸, 한 칸 {resolution * 100:g} cm, 막힌 칸 {before}개")
    print(f"부스의 테두리를 그려 넣음: 막힌 칸 {int((grid == 0).sum())}개 (+{int((grid == 0).sum()) - before})")
    print(f"저장: {MAP}")
