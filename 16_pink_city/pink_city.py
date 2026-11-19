"""강의 16: Pinky Pro 가 다니는 작은 도시 "City of Pink (핑크시티)" 를 만듭니다.

    ~/isaacsim/python.sh 16_pink_city/pink_city.py                 # 도시를 만들고 사진을 저장한 뒤 끝냄
    ~/isaacsim/python.sh 16_pink_city/pink_city.py --gui           # 창으로 보기
    ~/isaacsim/python.sh 16_pink_city/pink_city.py --robot --gui   # Pinky Pro 를 올려 키보드(W A S D)로 몰기
    ~/isaacsim/python.sh 16_pink_city/pink_city.py --save work/pink_city/pink_city.usd

도시의 크기 (단위 m, 가운데가 원점)
    전체       3 × 3, 바깥에 높이 0.35 의 벽
    도로       폭 0.4 (한 차로 0.2). 바깥을 도는 순환 도로 + 가운데 십자 도로
    교차로     가운데 사거리 1개, 순환 도로와 만나는 삼거리 4개
    블록       0.65 × 0.65 네 개: 핑크랩 본사 · 핑크 아카데미와 상가 · 공원 · 아파트
    로고       벽 간판 4개 + 본사 · 핑크 아카데미 옥상 간판 (그림은 공식 저장소 https://github.com/pinklab-art/pinky_pro 의 pinklab_logo.png)

도로 · 횡단보도 · 보도는 바닥에 입힌 그림 한 장이고, 벽 · 건물 · 나무에는 충돌체가 있어 라이다에 잡힙니다.
"""
import argparse
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, ".."))
OUT = os.path.join(REPO, "work", "pink_city")
LOGO_SOURCES = [os.path.expanduser("~/pinky_pro/src/pinky_pro/pinky_navigation/scripts/pinklab_logo.png"),
                os.path.join(REPO, "work", "pinky_pro", "pinky_navigation", "scripts", "pinklab_logo.png")]
FONTS = ["/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", "/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf"]

# ---------------------------------------------------------------- 도시의 치수 (m)
SIZE = 3.0                    # 한 변
HALF = SIZE / 2
ROAD = 0.40                   # 도로 폭
RING = 1.05                   # 순환 도로 중심선까지의 거리
RING_CORNER = 0.30            # 순환 도로 모퉁이의 둥근 정도 (중심선 반지름)
BLOCK_MIN, BLOCK_MAX = ROAD / 2, RING - ROAD / 2        # 블록 한 변의 범위: 0.20 ~ 0.85
WALL_HEIGHT = 0.35
WALL_THICKNESS = 0.04
PX_PER_M = 800                # 바닥 그림의 해상도 (1 m = 800 px)

PINK = (236, 112, 170)
DEEP_PINK = (214, 72, 140)
LIGHT_PINK = (250, 214, 228)
BLUE = (86, 110, 184)
COLOR_ASPHALT = (58, 60, 68)
COLOR_SIDEWALK = (226, 196, 210)
COLOR_WHITE = (245, 245, 245)
COLOR_YELLOW = (250, 204, 60)
COLOR_GREEN = (126, 190, 132)

# 정답 지도(강의 19)에 쓰는 종류 번호
LABEL_OTHER, LABEL_SIDEWALK, LABEL_ROAD, LABEL_LANE, LABEL_CROSSWALK = 0, 1, 2, 3, 4
LABEL_NAMES = ["그 밖 (건물 · 벽 · 하늘)", "보도 · 공원", "도로", "중앙선", "횡단보도 · 정지선"]


# ---------------------------------------------------------------- 그림 만들기 (PIL)
def _font(size):
    from PIL import ImageFont
    for path in FONTS:
        if os.path.exists(path):
            return ImageFont.truetype(path, size)
    return ImageFont.load_default()


def _logo():
    from PIL import Image
    for path in LOGO_SOURCES:
        if os.path.exists(path):
            image = Image.open(path).convert("RGBA")
            return image.crop(image.getbbox())
    return None


def make_floor(path, labels=False):
    """바닥 그림: 보도 · 도로 · 차선 · 횡단보도 · 공원 잔디 · 광장 글씨. 그림의 위쪽이 +Y, 오른쪽이 +X 입니다.

    labels=True 면 같은 모양을 색 대신 "종류 번호" 로 그린 정답 지도를 만든다 (강의 19 의 학습 데이터용).
    """
    from PIL import Image, ImageDraw
    n = int(SIZE * PX_PER_M)
    if labels:        # 색 이름은 그대로 두고 값만 종류 번호로 바꾼다
        SIDEWALK = TILE = GREEN = POND = LABEL_SIDEWALK
        ASPHALT, YELLOW, WHITE = LABEL_ROAD, LABEL_LANE, LABEL_CROSSWALK
    else:
        SIDEWALK, ASPHALT, YELLOW, WHITE, GREEN = COLOR_SIDEWALK, COLOR_ASPHALT, COLOR_YELLOW, COLOR_WHITE, COLOR_GREEN
        TILE, POND = (204, 170, 186), (150, 196, 230)
    image = Image.new("L" if labels else "RGB", (n, n), SIDEWALK)
    draw = ImageDraw.Draw(image)

    def px(x, y):
        return (x + HALF) * PX_PER_M, (HALF - y) * PX_PER_M

    def rect(x0, y0, x1, y1, color, radius=0.0):
        (a, b), (c, d) = px(min(x0, x1), max(y0, y1)), px(max(x0, x1), min(y0, y1))
        draw.rounded_rectangle([a, b, c, d], radius=radius * PX_PER_M, fill=color)

    # 보도 블록 무늬 (0.1 m 간격의 옅은 줄)
    for i in range(1, int(SIZE / 0.1)):
        t = i * 0.1 * PX_PER_M
        draw.line([(t, 0), (t, n)], fill=TILE, width=3)
        draw.line([(0, t), (n, t)], fill=TILE, width=3)

    # 순환 도로: 둥근 사각형 고리 = 바깥 모양에서 안쪽 모양을 뺀 것
    outer, inner = RING + ROAD / 2, RING - ROAD / 2
    rect(-outer, -outer, outer, outer, ASPHALT, radius=RING_CORNER + ROAD / 2)
    rect(-inner, -inner, inner, inner, SIDEWALK, radius=max(RING_CORNER - ROAD / 2, 0.02))
    for i in range(1, int(2 * inner / 0.1)):          # 안쪽 보도 무늬를 다시 그린다
        t = -inner + i * 0.1
        draw.line([px(t, -inner + 0.02), px(t, inner - 0.02)], fill=TILE, width=3)
        draw.line([px(-inner + 0.02, t), px(inner - 0.02, t)], fill=TILE, width=3)
    # 십자 도로
    rect(-ROAD / 2, -inner - 0.01, ROAD / 2, inner + 0.01, ASPHALT)
    rect(-inner - 0.01, -ROAD / 2, inner + 0.01, ROAD / 2, ASPHALT)

    # 공원 (왼쪽 아래 블록) 잔디와 광장 (오른쪽 위 블록 앞)
    rect(-BLOCK_MAX + 0.05, -BLOCK_MAX + 0.05, -BLOCK_MIN - 0.05, -BLOCK_MIN - 0.05, GREEN, radius=0.06)
    rect(-0.62, -0.62, -0.43, -0.43, POND, radius=0.09)          # 연못

    # 차선: 가운데 노란 점선, 가장자리 흰 실선
    def dashes(x0, y0, x1, y1, color=YELLOW, dash=0.08, gap=0.06, width=0.012):
        length = ((x1 - x0) ** 2 + (y1 - y0) ** 2) ** 0.5
        ux, uy = (x1 - x0) / length, (y1 - y0) / length
        s = 0.0
        while s < length:
            e = min(s + dash, length)
            draw.line([px(x0 + ux * s, y0 + uy * s), px(x0 + ux * e, y0 + uy * e)], fill=color, width=int(width * PX_PER_M))
            s += dash + gap

    straight = RING - RING_CORNER
    for sign in (-1, 1):
        dashes(-straight, sign * RING, straight, sign * RING)       # 순환 도로의 위 · 아래 변
        dashes(sign * RING, -straight, sign * RING, straight)       # 왼쪽 · 오른쪽 변
        dashes(0, sign * 0.36, 0, sign * (inner - 0.02))            # 세로 도로 (사거리 · 횡단보도 구간은 비움)
        dashes(sign * 0.36, 0, sign * (inner - 0.02), 0)            # 가로 도로
    r = RING_CORNER * PX_PER_M                                       # 모퉁이의 노란 곡선
    for sx, sy, start in ((1, 1, 270), (-1, 1, 180), (-1, -1, 90), (1, -1, 0)):
        cx, cy = px(sx * straight, sy * straight)
        draw.arc([cx - r, cy - r, cx + r, cy + r], start, start + 90, fill=YELLOW, width=int(0.012 * PX_PER_M))

    # 횡단보도: 가운데 사거리의 네 방향
    def zebra(cx, cy, along_x):
        for k in range(5):
            o = -ROAD / 2 + 0.03 + k * 0.075
            if along_x:      # 가로 도로를 건너는 줄무늬 (세로로 길다)
                rect(cx - 0.05, cy + o, cx + 0.05, cy + o + 0.04, WHITE)
            else:
                rect(cx + o, cy - 0.05, cx + o + 0.04, cy + 0.05, WHITE)
    for sign in (-1, 1):
        zebra(sign * 0.28, 0, True)
        zebra(0, sign * 0.28, False)
        # 정지선
        rect(sign * 0.345, -ROAD / 2 + 0.01, sign * 0.36, ROAD / 2 - 0.01, WHITE)
        rect(-ROAD / 2 + 0.01, sign * 0.345, ROAD / 2 - 0.01, sign * 0.36, WHITE)

    # 바닥 글씨: 아래쪽 보도에 도시 이름 (위에서 볼 때 방향을 알 수 있다)
    if labels:
        image.save(path)
        return
    font = _font(int(0.11 * PX_PER_M))
    text = "CITY OF PINK"
    box = draw.textbbox((0, 0), text, font=font)
    x, y = px(0, -HALF + 0.115)
    draw.text((x - (box[2] - box[0]) / 2 - box[0], y - (box[3] - box[1]) / 2 - box[1]), text, font=font, fill=DEEP_PINK)
    image.save(path)


def make_sign(path, text="CITY OF PINK", size=(1600, 400), background=COLOR_WHITE, with_logo=True, color=(40, 40, 48)):
    """간판: 핑크랩 로고 + 글씨."""
    from PIL import Image, ImageDraw
    image = Image.new("RGB", size, background)
    draw = ImageDraw.Draw(image)
    draw.rectangle([0, 0, size[0] - 1, size[1] - 1], outline=PINK, width=14)
    logo = _logo() if with_logo else None
    x = 70
    if logo is not None:
        h = int(size[1] * 0.68)
        logo = logo.resize((int(logo.width * h / logo.height), h))
        image.paste(logo, (x, (size[1] - h) // 2), logo)
        x += logo.width + 60
    font = _font(int(size[1] * 0.36))
    box = draw.textbbox((0, 0), text, font=font)
    while box[2] - box[0] > size[0] - x - 60:
        font = _font(font.size - 6)
        box = draw.textbbox((0, 0), text, font=font)
    draw.text((x + (size[0] - x - 60 - (box[2] - box[0])) / 2 - box[0], (size[1] - (box[3] - box[1])) / 2 - box[1]), text, font=font, fill=color)
    image.save(path)


def make_logo_board(path, size=(800, 800)):
    """옥상 간판: 로고만."""
    from PIL import Image
    image = Image.new("RGB", size, COLOR_WHITE)
    logo = _logo()
    if logo is not None:
        logo.thumbnail((int(size[0] * 0.8), int(size[1] * 0.8)))
        image.paste(logo, ((size[0] - logo.width) // 2, (size[1] - logo.height) // 2), logo)
    image.save(path)


def make_windows(path, wall, glass=(70, 96, 150), columns=4, rows=4, size=512):
    """건물 벽 그림: 창문이 줄지어 있는 한 면."""
    from PIL import Image, ImageDraw
    image = Image.new("RGB", (size, size), wall)
    draw = ImageDraw.Draw(image)
    cw, ch = size / columns, size / rows
    for r in range(rows):
        for c in range(columns):
            lit = (r * 3 + c * 5) % 7 == 0
            draw.rectangle([c * cw + cw * 0.22, r * ch + ch * 0.2, (c + 1) * cw - cw * 0.22, (r + 1) * ch - ch * 0.25],
                           fill=(255, 232, 160) if lit else glass)
    image.save(path)


def make_textures(out=OUT):
    os.makedirs(out, exist_ok=True)
    files = {"floor": os.path.join(out, "floor.png"), "labels": os.path.join(out, "floor_labels.png"), "sign": os.path.join(out, "sign_city.png"),
             "sign_lab": os.path.join(out, "sign_pinklab.png"),
             "sign_hq": os.path.join(out, "sign_headquarters.png"), "sign_academy": os.path.join(out, "sign_academy.png"), "board": os.path.join(out, "logo_board.png"),
             "win_pink": os.path.join(out, "windows_pink.png"), "win_white": os.path.join(out, "windows_white.png"),
             "win_blue": os.path.join(out, "windows_blue.png")}
    make_floor(files["floor"])
    make_floor(files["labels"], labels=True)
    make_sign(files["sign"], "CITY OF PINK")
    make_sign(files["sign_lab"], "PINKLAB")
    make_sign(files["sign_hq"], "PINKLAB", size=(1200, 400))
    make_sign(files["sign_academy"], "PINK ACADEMY", size=(1600, 360), background=(255, 236, 244), with_logo=False, color=DEEP_PINK)
    make_logo_board(files["board"])
    make_windows(files["win_pink"], PINK)
    make_windows(files["win_white"], (244, 240, 242), glass=(120, 150, 200))
    make_windows(files["win_blue"], BLUE, glass=(210, 226, 250), columns=3, rows=3)
    if _logo() is None:
        print("핑크랩 로고 그림을 찾지 못해 글씨만 넣었습니다. 먼저 실행:  bash 11_pinky_import/prepare_pinky.sh", flush=True)
    return files


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--gui", action="store_true", help="창을 띄운다")
    parser.add_argument("--robot", action="store_true", help="Pinky Pro 를 올린다 (창에서는 W A S D 로 운전)")
    parser.add_argument("--ros2", action="store_true", help="Pinky Pro 를 ROS 2 에 연결한다 (/cmd_vel 받기, /scan · /odom · /tf 등 내보내기)")
    parser.add_argument("--seconds", type=float, help="--ros2 일 때 이 시간이 지나면 끝냄 (없으면 Ctrl+C 까지)")
    parser.add_argument("--start", type=float, nargs=3, metavar=("X", "Y", "YAW"), help="로봇의 시작 위치 [m] 와 방향 [도] (기본: -1.05 -0.5 90)")
    parser.add_argument("--parked", type=float, nargs=3, metavar=("X", "Y", "YAW"), action="append", default=[],
                        help="이 자리에 서 있는 Pinky Pro 를 한 대 더 놓는다 (여러 번 쓸 수 있음 — 강의 28)")
    parser.add_argument("--hide-lidar", action="store_true", help="창에서 라이다 빔을 그리지 않는다 (빔이 로봇 카메라 영상에 찍히지 않게 — 강의 21)")
    parser.add_argument("--save", metavar="USD", help="만든 도시를 이 경로에 저장한다")
    parser.add_argument("--photo", metavar="DIR", default=OUT, help="사진을 저장할 폴더")
    args = parser.parse_args()
    TEXTURES = make_textures()
    from isaacsim import SimulationApp
    if args.ros2 and "ROS_DISTRO" not in os.environ:
        print("ROS 2 환경이 없습니다. 같은 터미널에서 먼저 실행하세요:  source /opt/ros/jazzy/setup.bash")
        sys.exit(1)
    # 창 없이 돌리면 시뮬레이션이 실제 시간보다 빨리 갑니다. 1초에 60프레임으로 맞춰 실제 시간과 같게 합니다.
    app = SimulationApp({"headless": not args.gui, "extra_args": [
        "--/app/runLoops/main/rateLimitEnabled=true", "--/app/runLoops/main/rateLimitFrequency=60"]})

import numpy as np
import omni.kit.commands
import omni.usd
from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdPhysics, UsdShade

sys.path.insert(0, REPO)
from common.sim import Sim, new_scene, save_stage, set_pose, set_view

CITY = "/World/PinkCity"
LOOKS = CITY + "/Looks"
ROBOT_START = (-RING, -0.5, 90.0)          # 순환 도로의 왼쪽 변, 오른쪽 차로가 아닌 중심선 위에서 +Y 를 보고 시작


def rgb(color):
    return Gf.Vec3f(*[(c / 255.0) ** 2.2 for c in color])        # 화면의 색(sRGB) → 렌더러가 쓰는 값


def material(stage, name, color=None, texture=None, glow=0.0, roughness=0.6):
    """OmniPBR 재질. texture 가 있으면 그림을 입히고, glow 가 있으면 스스로 빛난다 (간판)."""
    path = f"{LOOKS}/{name}"
    if stage.GetPrimAtPath(path):
        return UsdShade.Material(stage.GetPrimAtPath(path))
    omni.kit.commands.execute("CreateMdlMaterialPrim", mtl_url="OmniPBR.mdl", mtl_name="OmniPBR", mtl_path=path)
    shader = UsdShade.Shader(stage.GetPrimAtPath(path + "/Shader"))
    if color is not None:
        shader.CreateInput("diffuse_color_constant", Sdf.ValueTypeNames.Color3f).Set(rgb(color))
    if texture is not None:
        shader.CreateInput("diffuse_texture", Sdf.ValueTypeNames.Asset).Set(texture)
    shader.CreateInput("reflection_roughness_constant", Sdf.ValueTypeNames.Float).Set(roughness)
    if glow > 0.0:
        shader.CreateInput("enable_emission", Sdf.ValueTypeNames.Bool).Set(True)
        shader.CreateInput("emissive_color", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(1.0))
        if texture is not None:
            shader.CreateInput("emissive_color_texture", Sdf.ValueTypeNames.Asset).Set(texture)
        shader.CreateInput("emissive_intensity", Sdf.ValueTypeNames.Float).Set(glow)
    return UsdShade.Material(stage.GetPrimAtPath(path))


def _mesh(stage, kind, path):
    """Create > Mesh 메뉴와 같은 모양을 만들고, 그 모양의 원래 크기(x, y, z)를 함께 돌려준다."""
    parent = path.rsplit("/", 1)[0]
    if not stage.GetPrimAtPath(parent):
        UsdGeom.Xform.Define(stage, parent)
    omni.kit.commands.execute("CreateMeshPrimWithDefaultXform", prim_type=kind, prim_path=path,
                              select_new_prim=False, prepend_default_prim=False)
    prim = stage.GetPrimAtPath(path)
    points = np.asarray(UsdGeom.Mesh(prim).GetPointsAttr().Get())
    return prim, points.max(axis=0) - points.min(axis=0)


def add_shape(stage, kind, path, center, size, look, yaw=0.0, solid=True):
    """상자(Cube) · 기둥(Cylinder) · 뿔(Cone) · 공(Sphere). size 는 실제 크기 (m), solid 면 충돌체를 붙인다."""
    prim, base = _mesh(stage, kind, path)
    set_pose(prim, translate=center, rotate_xyz=(0.0, 0.0, yaw), scale=[s / b for s, b in zip(size, base)])
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(look)
    if solid:
        UsdPhysics.CollisionAPI.Apply(prim)
        if kind != "Cube":
            UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr("convexHull")
    return prim


def add_board(stage, path, center, width, height, look, facing):
    """세워 둔 그림판. facing = 그림이 보이는 방향의 각도 (0 = +X 쪽에서 보임, 90 = +Y 쪽에서 보임)."""
    prim, base = _mesh(stage, "Plane", path)
    # 평면은 처음에 누워 있다 (법선 +Z). X 로 90° 세운 뒤 Z 로 돌려 원하는 쪽을 보게 한다 (강의 13 의 화면과 같은 방법).
    set_pose(prim, translate=center, rotate_xyz=(90.0, 0.0, 90.0 + facing), scale=(width / base[0], height / base[1], 1.0))
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(look)
    return prim


def add_floor(stage, texture):
    """도로가 그려진 바닥 그림 한 장 (3 × 3 m). 충돌은 new_scene 의 바닥 평면이 맡는다."""
    prim, base = _mesh(stage, "Plane", CITY + "/Floor")
    set_pose(prim, translate=(0.0, 0.0, 0.001), scale=(SIZE / base[0], SIZE / base[1], 1.0))
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(material(stage, "Floor", texture=texture, roughness=0.8))
    return prim


def add_walls(stage, textures):
    """바깥 벽 4개와 벽마다 간판 하나."""
    wall = material(stage, "Wall", color=(246, 240, 243))
    cap = material(stage, "WallCap", color=PINK)
    sign_city = material(stage, "SignCity", texture=textures["sign"], glow=220.0)
    sign_lab = material(stage, "SignPinklab", texture=textures["sign_lab"], glow=220.0)
    e = HALF + WALL_THICKNESS / 2
    long = SIZE + 2 * WALL_THICKNESS
    #        이름      중심        크기 (x, y)                  간판이 보는 쪽 · 간판 재질
    sides = [("North", (0, e), (long, WALL_THICKNESS), -90.0, sign_city),
             ("South", (0, -e), (long, WALL_THICKNESS), 90.0, sign_lab),
             ("East", (e, 0), (WALL_THICKNESS, long), 180.0, sign_lab),
             ("West", (-e, 0), (WALL_THICKNESS, long), 0.0, sign_city)]
    for name, (x, y), (sx, sy), facing, sign in sides:
        add_shape(stage, "Cube", f"{CITY}/Walls/{name}", (x, y, WALL_HEIGHT / 2), (sx, sy, WALL_HEIGHT), wall)
        add_shape(stage, "Cube", f"{CITY}/Walls/{name}Cap", (x, y, WALL_HEIGHT + 0.01), (sx + 0.01, sy + 0.01, 0.02), cap, solid=False)
        inward = np.array([np.cos(np.radians(facing)), np.sin(np.radians(facing))]) * (WALL_THICKNESS / 2 + 0.002)
        add_board(stage, f"{CITY}/Signs/{name}", (x + inward[0], y + inward[1], 0.19), 1.0, 0.25, sign, facing)


def add_building(stage, name, center, size, look, roof):
    """건물 = 창문 그림을 입힌 상자 + 얇은 지붕 판."""
    x, y = center
    sx, sy, h = size
    add_shape(stage, "Cube", f"{CITY}/Buildings/{name}", (x, y, h / 2), (sx, sy, h), look)
    add_shape(stage, "Cube", f"{CITY}/Buildings/{name}Roof", (x, y, h + 0.008), (sx + 0.02, sy + 0.02, 0.016), roof, solid=False)


def add_roof_sign(stage, name, center, roof, width, height, look, along_x):
    """옥상 간판: 얇은 판을 세우고 앞뒤 양쪽에 같은 그림을 붙인다. along_x 면 판이 X 방향으로 길다 (남 · 북에서 보임)."""
    x, y = center
    z = roof + 0.02 + height / 2
    frame = material(stage, "BoardFrame", color=(70, 72, 84))
    size = (width + 0.02, 0.012, height + 0.02) if along_x else (0.012, width + 0.02, height + 0.02)
    add_shape(stage, "Cube", f"{CITY}/Buildings/{name}SignFrame", (x, y, z), size, frame, solid=False)
    for k, post in enumerate((-0.4, 0.4)):
        px, py = (x + post * width, y) if along_x else (x, y + post * width)
        add_shape(stage, "Cube", f"{CITY}/Buildings/{name}SignPost{k}", (px, py, roof + 0.01), (0.012, 0.012, 0.02), frame, solid=False)
    for k, facing in enumerate((-90.0, 90.0) if along_x else (180.0, 0.0)):
        d = np.array([np.cos(np.radians(facing)), np.sin(np.radians(facing))]) * 0.008
        add_board(stage, f"{CITY}/Signs/{name}Roof{k}", (x + d[0], y + d[1], z), width, height, look, facing)


def add_tree(stage, name, x, y, height=0.22):
    trunk = material(stage, "Trunk", color=(128, 92, 70))
    leaf = material(stage, "Leaf", color=(74, 150, 96))
    add_shape(stage, "Cylinder", f"{CITY}/Park/{name}Trunk", (x, y, height * 0.25), (0.03, 0.03, height * 0.5), trunk)
    add_shape(stage, "Cone", f"{CITY}/Park/{name}Leaf", (x, y, height * 0.7), (0.13, 0.13, height * 0.7), leaf)


def add_city(stage, textures):
    """핑크시티 전체: 바닥 그림 · 벽과 간판 · 네 블록의 건물 · 공원."""
    UsdGeom.Xform.Define(stage, CITY)
    add_floor(stage, textures["floor"])
    add_walls(stage, textures)
    pink = material(stage, "WindowsPink", texture=textures["win_pink"])
    white = material(stage, "WindowsWhite", texture=textures["win_white"])
    blue = material(stage, "WindowsBlue", texture=textures["win_blue"])
    roof_dark = material(stage, "RoofDark", color=(84, 86, 98))
    roof_pink = material(stage, "RoofPink", color=DEEP_PINK)
    mid = (BLOCK_MIN + BLOCK_MAX) / 2                       # 블록의 가운데: 0.525

    # 오른쪽 위 블록: 핑크랩 본사 (가장 높은 건물) + 옥상 간판
    add_building(stage, "Headquarters", (mid, mid), (0.42, 0.42, 0.55), pink, roof_dark)
    add_roof_sign(stage, "Headquarters", (mid, mid), 0.55 + 0.016, 0.39, 0.13, material(stage, "SignHeadquarters", texture=textures["sign_hq"], glow=300.0), along_x=True)
    logo = _mesh(stage, "Plane", f"{CITY}/Signs/HeadquartersRoofLogo")       # 옥상 바닥에도 로고 (위에서 내려다볼 때 보인다)
    set_pose(logo[0], translate=(mid, mid + 0.09, 0.55 + 0.018), scale=(0.2 / logo[1][0], 0.2 / logo[1][1], 1.0))
    UsdShade.MaterialBindingAPI.Apply(logo[0]).Bind(material(stage, "LogoBoard", texture=textures["board"]))

    # 왼쪽 위 블록: 상가 두 채 (낮은 흰 건물과 파란 건물)
    add_building(stage, "PinkAcademy", (-mid - 0.13, mid), (0.22, 0.46, 0.26), white, roof_pink)
    academy = material(stage, "SignAcademy", texture=textures["sign_academy"], glow=300.0)
    add_roof_sign(stage, "PinkAcademy", (-mid - 0.13, mid), 0.26 + 0.016, 0.42, 0.095, academy, along_x=False)
    add_board(stage, f"{CITY}/Signs/PinkAcademyDoor", (-mid - 0.13, mid - 0.23 - 0.002, 0.045), 0.20, 0.045, academy, -90.0)   # 도로 쪽 입구 위
    add_building(stage, "Shop", (-mid + 0.15, mid + 0.08), (0.24, 0.30, 0.38), blue, roof_dark)

    # 오른쪽 아래 블록: ㄱ 자 아파트
    add_building(stage, "ApartmentA", (mid, -mid - 0.14), (0.48, 0.20, 0.44), white, roof_pink)
    add_building(stage, "ApartmentB", (mid + 0.14, -mid + 0.10), (0.20, 0.26, 0.32), pink, roof_dark)

    # 왼쪽 아래 블록: 공원 (나무 다섯 그루와 매점)
    for k, (x, y) in enumerate(((-0.74, -0.30), (-0.52, -0.30), (-0.30, -0.33), (-0.74, -0.74), (-0.30, -0.62))):
        add_tree(stage, f"Tree{k}", x, y, height=0.20 + 0.03 * (k % 3))
    add_building(stage, "Kiosk", (-0.33, -0.76), (0.14, 0.12, 0.13), material(stage, "Kiosk", color=LIGHT_PINK), roof_pink)

    # 낮에 건물 그림자가 보이도록 조명을 조금 조정
    light = UsdLux.DistantLight(stage.GetPrimAtPath("/World/Light"))
    light.GetIntensityAttr().Set(1500.0)
    light.CreateAngleAttr(1.5)


def add_parked_robot(stage, source, index, x, y, yaw):
    """서 있는 Pinky Pro 한 대: 로봇을 복사한 뒤 관절과 강체 설정을 지워 "움직이지 않는 물체" 로 만든다.

    충돌체는 남기므로 라이다에 보이고 부딪힌다. 센서와 ROS 2 그래프를 달기 전에 복사해야 깨끗한 사본이 된다.
    """
    from pxr import PhysxSchema, Usd
    path = f"/World/parked_{index}"
    omni.kit.commands.execute("CopyPrim", path_from=source, path_to=path, exclusive_select=False)
    stage.RemovePrim(path + "/joints")
    for prim in Usd.PrimRange(stage.GetPrimAtPath(path)):
        for api in (UsdPhysics.ArticulationRootAPI, UsdPhysics.RigidBodyAPI, UsdPhysics.MassAPI,
                    PhysxSchema.PhysxArticulationAPI, PhysxSchema.PhysxRigidBodyAPI):
            if prim.HasAPI(api):
                prim.RemoveAPI(api)
    set_pose(stage.GetPrimAtPath(path), translate=(x, y, 0.0), rotate_xyz=(0.0, 0.0, yaw))
    return stage.GetPrimAtPath(path)


def save_photos(app, stage, folder):
    """도시를 여러 방향에서 찍어 PNG 로 저장한다. 저장한 파일 목록을 돌려준다."""
    import omni.replicator.core as rep
    from isaacsim.core.utils.viewports import set_camera_view
    from PIL import Image
    os.makedirs(folder, exist_ok=True)
    camera = UsdGeom.Camera.Define(stage, "/World/PhotoCamera")
    camera.CreateFocalLengthAttr(24.0)
    camera.CreateClippingRangeAttr(Gf.Vec2f(0.01, 100.0))
    render_product = rep.create.render_product(str(camera.GetPath()), (1600, 1000))
    annotator = rep.AnnotatorRegistry.get_annotator("rgb")
    annotator.attach([render_product])
    views = {"overview": ((-2.6, -3.4, 2.9), (0.0, 0.05, 0.0)),
             "top": ((0.0, -0.01, 5.3), (0.0, 0.0, 0.0)),
             "street": ((-RING, -1.28, 0.14), (-RING, 0.6, 0.12)),
             "crossroad": ((-0.55, -0.9, 0.55), (0.25, 0.3, 0.12)),
             "north_wall": ((0.3, -0.2, 0.5), (0.0, 1.5, 0.15)),
             "headquarters": ((0.15, -0.55, 0.75), (0.5, 0.5, 0.45)),
             "academy": ((-1.45, -0.25, 0.5), (-0.6, 0.5, 0.2))}
    saved = []
    for name, (eye, target) in views.items():
        set_camera_view(eye=list(eye), target=list(target), camera_prim_path=str(camera.GetPath()))
        for _ in range(60):
            app.update()
        data = np.asarray(annotator.get_data())
        for _ in range(300):                       # 처음에는 그림(텍스처)을 읽느라 화면이 늦게 나온다
            if data.ndim == 3:
                break
            app.update()
            data = np.asarray(annotator.get_data())
        path = os.path.join(folder, f"pink_city_{name}.png")
        Image.fromarray(data[:, :, :3]).save(path)
        saved.append(path)
    annotator.detach()
    stage.RemovePrim("/World/PhotoCamera")
    return saved


if __name__ == "__main__":
    stage = new_scene()
    add_city(stage, TEXTURES)
    if args.robot or args.ros2:
        for folder in ("11_pinky_import", "12_pinky_sensors", "14_pinky_ros2"):
            sys.path.insert(0, os.path.join(REPO, folder))
        from import_pinky import DEFAULT_URDF, ROBOT, add_differential_controller, add_keyboard_control, find_urdf, import_pinky
        import_pinky(stage, find_urdf(DEFAULT_URDF), height=0.0)
        for k, (px, py, pyaw) in enumerate(args.parked):
            add_parked_robot(stage, ROBOT, k, px, py, pyaw)
            print(f"서 있는 Pinky Pro {k}: ({px}, {py}) 방향 {pyaw}°", flush=True)
        x, y, yaw = args.start or ROBOT_START
        set_pose(stage.GetPrimAtPath(ROBOT), translate=(x, y, 0.0), rotate_xyz=(0.0, 0.0, yaw))
        if args.ros2:                              # 강의 14 · 15 의 그래프를 그대로 쓴다
            from isaacsim.core.utils.extensions import enable_extension
            enable_extension("isaacsim.ros2.bridge")
            enable_extension("isaacsim.robot.wheeled_robots")
            app.update()
            from pinky_ros2 import add_cmd_vel_subscriber, add_sensor_publishers, add_state_publishers
            from pinky_sensors import add_camera, add_imu, add_lidar
            add_lidar(stage, draw=args.gui and not args.hide_lidar)
            add_camera(stage)
            add_imu(stage)
            add_cmd_vel_subscriber()
            add_state_publishers()
            add_sensor_publishers()
            print(f"ROS 2 연결: ROS_DOMAIN_ID = {os.environ.get('ROS_DOMAIN_ID', '0')}, 시작 위치 ({x}, {y}) 방향 {yaw}°", flush=True)
        else:
            add_differential_controller()
            add_keyboard_control()
    print(f"핑크시티: {SIZE} × {SIZE} m, 도로 폭 {ROAD} m, 블록 {BLOCK_MAX - BLOCK_MIN:.2f} m", flush=True)
    if args.save:
        save_stage(stage, args.save)
    if args.gui:
        set_view((-2.6, -3.4, 2.9), (0.0, 0.05, 0.0))
    sim = Sim(app)
    sim.play()                                   # 사진은 Play 중에 찍힌다
    sim.run(1.0)
    if args.ros2 and not args.gui:
        try:
            if args.seconds:
                sim.run(args.seconds)
            else:
                while app.is_running():
                    app.update()
        except KeyboardInterrupt:
            pass
    elif args.gui:
        if args.robot:
            print("뷰포트를 한 번 클릭한 뒤 W(앞) S(뒤) A(왼쪽) D(오른쪽) 로 조종합니다. 창을 닫으면 끝납니다.", flush=True)
        sim.wait_until_closed()
    else:
        for path in save_photos(app, stage, args.photo):
            print("사진 저장:", path, flush=True)
    sim.stop()
    app.close()
