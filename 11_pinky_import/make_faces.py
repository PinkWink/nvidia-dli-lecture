"""Pinky Pro 의 표정 GIF (pinky_emotion/emotion/*.gif) 에서 표정이 가장 잘 보이는 장면 하나를 PNG 로 저장합니다.

    ~/isaacsim/python.sh 11_pinky_import/make_faces.py ~/pinky_pro/src/pinky_pro/pinky_emotion/emotion work/pinky_faces

Isaac Sim 의 재질은 움직이는 GIF 를 그림으로 쓰지 못해 PNG 한 장으로 바꿉니다. (Isaac Sim 을 켜지 않고 PIL 만 씁니다)
"""
import glob
import os
import sys

from PIL import Image

source, target = (os.path.abspath(os.path.expanduser(p)) for p in sys.argv[1:3])
os.makedirs(target, exist_ok=True)


def drawn_pixels(frame):
    """배경(가장 많은 색)이 아닌 픽셀 수. 표정이 가장 크게 그려진 장면을 고르는 데 쓴다."""
    small = frame.convert("RGB").resize((100, 75))
    colors = small.getcolors(100 * 75)
    background = max(colors)[1]
    return sum(count for count, color in colors if color != background)


for path in sorted(glob.glob(os.path.join(source, "*.gif"))):
    image = Image.open(path)
    best, best_score = 0, -1
    for index in range(0, getattr(image, "n_frames", 1), 4):     # 4장마다 살펴본다 (첫 장면이 빈 화면인 GIF 가 있다)
        image.seek(index)
        score = drawn_pixels(image)
        if score > best_score:
            best, best_score = index, score
    image.seek(best)
    name = os.path.splitext(os.path.basename(path))[0] + ".png"
    image.convert("RGB").save(os.path.join(target, name))
    print(f"  {name}  {image.size[0]} × {image.size[1]}  ({best + 1}번째 장면)")
