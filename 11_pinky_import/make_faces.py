"""Pinky Pro 의 표정 GIF (pinky_emotion/emotion/*.gif) 에서 첫 장면을 PNG 로 저장합니다.

    ~/isaacsim/python.sh 11_pinky_import/make_faces.py ~/pinky_pro/src/pinky_pro/pinky_emotion/emotion work/pinky_faces

Isaac Sim 의 재질은 움직이는 GIF 를 그림으로 쓰지 못해 PNG 한 장으로 바꿉니다. (Isaac Sim 을 켜지 않고 PIL 만 씁니다)
"""
import glob
import os
import sys

from PIL import Image

source, target = (os.path.abspath(os.path.expanduser(p)) for p in sys.argv[1:3])
os.makedirs(target, exist_ok=True)
for path in sorted(glob.glob(os.path.join(source, "*.gif"))):
    image = Image.open(path)
    image.seek(0)                                   # 첫 장면
    name = os.path.splitext(os.path.basename(path))[0] + ".png"
    image.convert("RGB").save(os.path.join(target, name))
    print(f"  {name}  {image.size[0]} × {image.size[1]}")
