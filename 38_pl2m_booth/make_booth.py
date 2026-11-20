"""강의 38: MuJoCo 로 만든 핑크랩 부스(로보월드 2026)를 Isaac Sim 장면으로 옮기고, PL-2M 을 올립니다.

    ~/isaacsim/python.sh 38_pl2m_booth/make_booth.py --read-only        # 부스의 MuJoCo 장면 파일(MJCF)을 만들고 읽기만
    ~/isaacsim/python.sh 38_pl2m_booth/make_booth.py                    # 옮겨서 work/booth/booth.usd 로 저장 + 사진 7장
    ~/isaacsim/python.sh 38_pl2m_booth/make_booth.py --gui              # 창으로 보기
    ~/isaacsim/python.sh 38_pl2m_booth/make_booth.py --robot --gui      # PL-2M 과 컵 3개를 올려서 보기

부스의 원본: https://github.com/PinkWink/pinklab_robotworld2026 의 src/robotworld
    booth/booth_data.json (설계 데이터) → robotworld/world.py 가 MuJoCo 장면 파일 booth.xml 과 벽 그림(PNG)을 만든다.
이 스크립트는 그 booth.xml 을 읽어 도형 하나하나를 Isaac Sim 의 Prim 으로 옮긴다.

    MuJoCo (MJCF)                      Isaac Sim (USD)
    geom box · cylinder · sphere       Cube · Cylinder · Sphere
    geom capsule (fromto)              Capsule (두 점을 잇는 방향으로 세움)
    geom plane + texture               네모 Mesh 한 장 + 그림 재질
    geom mesh                          Mesh
    body                               Xform
    material rgba / texture            OmniPBR 재질 (색 / 그림)
    camera                             Camera
    group 0 (라이다 · 카메라에 보이는 것)   충돌체(Collider)
"""
import argparse
import math
import os
import sys
import xml.etree.ElementTree as ET
from collections import Counter

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, ".."))
SOURCE = os.path.expanduser("~/pinklab_robotworld2026")          # 강의 37 에서 받은 저장소
OUT = os.path.join(REPO, "work", "booth")
BOOTH = "/World/Booth"
LOOKS = "/World/Looks"
SPAWN = (5.3, -3.2, 0.0)          # 로봇 시작 자리: 상담 테이블 둘 사이 (원본 worlds/booth.yaml 의 spawn)
CUPS = [("cup_pink", (7.50, -1.72, 0.795), (0.87, 0.42, 0.65)),       # 음료대 위의 컵 (원본 worlds/booth.yaml 의 objects)
        ("cup_blue", (7.65, -1.72, 0.795), (0.28, 0.39, 0.68)),
        ("cup_yellow", (7.80, -1.72, 0.795), (0.95, 0.77, 0.23))]
CUP_SIZE, CUP_MASS = (0.05, 0.09), 0.1                                # 지름 · 높이 [m], 질량 [kg]
GLOSSY = {"col_e9e8ea": 0.25}                                         # 부스 바닥판: 조명이 비치게 매끈하게 (재질 이름: 거칠기)
PINK, WARM, COOL = (0.87, 0.42, 0.65), (1.0, 0.93, 0.82), (0.72, 0.80, 1.0)      # 핑크랩 분홍 · 전구색 · 차가운 흰색
LAMP, KEY, WASH, LED, DOWN = 45000.0, 170000.0, 12000.0, 2500.0, 15000.0                # 조명 밝기: 등기구 · 위의 큰 조명 · 분홍빛 · LED 줄 · 안쪽 조명
ROBOT_VIEWS = [("robot", (2.9, -4.5, 2.7), (5.4, -3.15, 0.45)),         # 이름, 카메라 위치, 바라보는 점
               ("cups", (7.2, -2.75, 1.35), (7.65, -1.7, 0.78))]


def build_mjcf(out=OUT):
    """원본 저장소의 생성기로 booth.xml 과 벽 그림을 만든다. 만든 파일의 경로를 돌려준다."""
    package = os.path.join(SOURCE, "src", "robotworld")
    if not os.path.isdir(package):
        print(f"부스 원본이 없습니다: {package}")
        print("먼저 강의 37 의 준비를 하세요:  bash ~/nvidia-dli-lecture/37_pl2m_import/prepare_pl2m.sh")
        sys.exit(1)
    sys.path.insert(0, package)
    from robotworld.world import build
    os.makedirs(out, exist_ok=True)
    return str(build(out))


def numbers(text):
    return [float(v) for v in text.split()]


def read_mjcf(path):
    """장면 파일에 무엇이 들어 있는지 세어서 출력한다."""
    root = ET.parse(path).getroot()
    geoms = list(root.iter("geom"))
    kinds = Counter(geom.get("type") for geom in geoms)
    solid = sum(1 for geom in geoms if geom.get("group") == "0")
    print(f"MuJoCo 장면 파일: {path}", flush=True)
    print(f"도형 {len(geoms)}개: " + ", ".join(f"{kind} {count}" for kind, count in kinds.most_common()), flush=True)
    print(f"그중 라이다 · 카메라에 보이는 것(group 0) {solid}개, 장식(group 2) {len(geoms) - solid}개", flush=True)
    print(f"묶음(body) {len(list(root.iter('body')))}개, 재질 {len(root.findall('asset/material'))}개 "
          f"(그림 재질 {sum(1 for m in root.findall('asset/material') if m.get('texture'))}개), "
          f"카메라 {len(list(root.iter('camera')))}개, 조명 {len(list(root.iter('light')))}개", flush=True)
    return root


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--read-only", action="store_true", help="MuJoCo 장면 파일을 만들고 읽기만 한다 (Isaac Sim 을 켜지 않음)")
    parser.add_argument("--robot", action="store_true", help="PL-2M 과 컵 3개를 올리고 5초 동안 지켜본다")
    parser.add_argument("--gui", action="store_true", help="창을 띄운다")
    parser.add_argument("--save", metavar="USD", default=os.path.join(OUT, "booth.usd"), help="만든 장면을 저장할 경로")
    parser.add_argument("--plain", action="store_true", help="전시장 조명 없이 기본 조명(강의 3)만 쓴다")
    parser.add_argument("--no-photo", action="store_true", help="사진을 찍지 않는다")
    parser.add_argument("--seconds", type=float, default=5.0, help="--robot 일 때 지켜보는 시간 [s]")
    args = parser.parse_args()
    args.save = os.path.abspath(os.path.expanduser(args.save))
    mjcf_path = build_mjcf()
    mjcf = read_mjcf(mjcf_path)
    if args.read_only:
        sys.exit(0)

    from isaacsim import SimulationApp
    app = SimulationApp({"headless": not args.gui})

import numpy as np
import omni.kit.commands
import omni.usd
from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdPhysics, UsdShade

sys.path.insert(0, REPO)
from common.sim import Sim, new_scene, pose_of, save_stage, set_pose, set_view


def rotation_from_euler(euler):
    """MJCF 의 euler (x → y → z 순서로 도는 각, 라디안) 를 회전 행렬로."""
    rx, ry, rz = euler
    cx, sx, cy, sy, cz, sz = math.cos(rx), math.sin(rx), math.cos(ry), math.sin(ry), math.cos(rz), math.sin(rz)
    x = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    y = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    z = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    return x @ y @ z


def rotation_from_axes(xyaxes):
    """MJCF 의 xyaxes (x 축 · y 축 방향) 를 회전 행렬로. z 축은 둘에 수직."""
    x = np.array(xyaxes[:3]) / np.linalg.norm(xyaxes[:3])
    y = np.array(xyaxes[3:]) - np.dot(xyaxes[3:], x) * x
    y /= np.linalg.norm(y)
    return np.column_stack([x, y, np.cross(x, y)])


def rotation_to(direction):
    """z 축이 direction 을 향하게 하는 회전 행렬 (기둥 · 캡슐을 두 점 사이에 세울 때)."""
    z = np.array(direction) / np.linalg.norm(direction)
    helper = np.array([1.0, 0.0, 0.0]) if abs(z[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    x = np.cross(helper, z)
    x /= np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z])


def place(prim, position, rotation=None, scale=None):
    """Prim 에 위치 · 회전(행렬) · 크기를 준다."""
    xform = UsdGeom.Xformable(prim)
    xform.AddTranslateOp().Set(Gf.Vec3d(*[float(v) for v in position]))
    if rotation is not None:
        matrix = Gf.Matrix3d(*[float(v) for v in np.asarray(rotation).T.flatten()])     # pxr 은 행 벡터 기준이라 뒤집어 넣는다
        xform.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(matrix.ExtractRotation().GetQuat())
    if scale is not None:
        xform.AddScaleOp().Set(Gf.Vec3f(*[float(v) for v in scale]))


def color_value(rgb):
    return Gf.Vec3f(*[float(c) ** 2.2 for c in rgb])              # 화면의 색(sRGB) → 렌더러가 쓰는 값


def make_material(stage, name, color=None, texture=None, glow=0.0, roughness=0.7):
    """OmniPBR 재질 (강의 13 · 16 과 같은 방법). texture 가 있으면 그림을 입히고, glow 가 있으면 스스로 빛난다 (모니터 · LED)."""
    path = f"{LOOKS}/{name}"
    omni.kit.commands.execute("CreateMdlMaterialPrim", mtl_url="OmniPBR.mdl", mtl_name="OmniPBR", mtl_path=path)
    shader = UsdShade.Shader(stage.GetPrimAtPath(path + "/Shader"))
    if color is not None:
        shader.CreateInput("diffuse_color_constant", Sdf.ValueTypeNames.Color3f).Set(color_value(color))
    if texture is not None:
        shader.CreateInput("diffuse_texture", Sdf.ValueTypeNames.Asset).Set(texture)
    shader.CreateInput("reflection_roughness_constant", Sdf.ValueTypeNames.Float).Set(roughness)
    if glow > 0.0:
        shader.CreateInput("enable_emission", Sdf.ValueTypeNames.Bool).Set(True)
        shader.CreateInput("emissive_color", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(1.0) if texture else color_value(color))
        if texture is not None:
            shader.CreateInput("emissive_color_texture", Sdf.ValueTypeNames.Asset).Set(texture)
        shader.CreateInput("emissive_intensity", Sdf.ValueTypeNames.Float).Set(float(glow) * 1000.0)
    return UsdShade.Material(stage.GetPrimAtPath(path))


def add_materials(stage, mjcf):
    """MJCF 의 재질을 모두 만든다. {이름: 재질}"""
    files = {t.get("name"): t.get("file") for t in mjcf.findall("asset/texture")}
    looks = {}
    for item in mjcf.findall("asset/material"):
        name = item.get("name")
        texture = files.get(item.get("texture"))
        if texture:
            looks[name] = make_material(stage, name, texture=texture, glow=float(item.get("emission", 0)))
        else:
            rgba = numbers(item.get("rgba", "0.5 0.5 0.5 1"))
            looks[name] = make_material(stage, name, color=rgba[:3], glow=float(item.get("emission", 0)) * 5,
                                        roughness=GLOSSY.get(name, 0.7))
    return looks


def add_quad(stage, path, half_x, half_y):
    """그림을 붙일 네모 한 장 (가운데가 원점, 법선 +Z). 그림의 아래 왼쪽이 (−x, −y)."""
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr([(-half_x, -half_y, 0), (half_x, -half_y, 0), (half_x, half_y, 0), (-half_x, half_y, 0)])
    mesh.CreateFaceVertexCountsAttr([4])
    mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    mesh.CreateNormalsAttr([(0, 0, 1)] * 4)
    mesh.CreateDoubleSidedAttr(True)
    st = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex)
    st.Set([(0, 0), (1, 0), (1, 1), (0, 1)])
    return mesh.GetPrim()


def add_geom(stage, parent, geom, looks, meshes):
    """MJCF 의 geom 하나를 Prim 하나로 옮긴다. 옮기지 않은 것은 None."""
    kind, name = geom.get("type"), geom.get("name")
    if name.startswith("hall_floor"):                 # 전시장 바닥: new_scene 의 바닥이 대신한다
        return None
    path = f"{parent}/{name}"
    size = numbers(geom.get("size", "0"))
    position = numbers(geom.get("pos", "0 0 0"))
    rotation = None
    if geom.get("euler"):
        rotation = rotation_from_euler(numbers(geom.get("euler")))
    elif geom.get("xyaxes"):
        rotation = rotation_from_axes(numbers(geom.get("xyaxes")))
    if geom.get("fromto"):                            # 두 점을 잇는 기둥 · 캡슐
        ends = np.array(numbers(geom.get("fromto")))
        start, end = ends[:3], ends[3:]
        position, rotation, length = (start + end) / 2, rotation_to(end - start), float(np.linalg.norm(end - start))
    if kind == "box":
        prim = UsdGeom.Cube.Define(stage, path).GetPrim()
        UsdGeom.Cube(prim).CreateSizeAttr(1.0)
        place(prim, position, rotation, scale=[2 * v for v in size])          # MJCF 의 size 는 절반 크기
    elif kind == "sphere":
        prim = UsdGeom.Sphere.Define(stage, path).GetPrim()
        UsdGeom.Sphere(prim).CreateRadiusAttr(size[0])
        place(prim, position)
    elif kind in ("cylinder", "capsule"):
        shape = (UsdGeom.Cylinder if kind == "cylinder" else UsdGeom.Capsule).Define(stage, path)
        shape.CreateAxisAttr("Z")
        shape.CreateRadiusAttr(size[0])
        shape.CreateHeightAttr(length if geom.get("fromto") else 2 * size[1])
        prim = shape.GetPrim()
        place(prim, position, rotation)
    elif kind == "plane":
        prim = add_quad(stage, path, size[0], size[1])
        place(prim, position, rotation)
    elif kind == "mesh":
        points, faces, uvs = meshes[geom.get("mesh")]
        mesh = UsdGeom.Mesh.Define(stage, path)
        mesh.CreatePointsAttr([tuple(p) for p in points])
        mesh.CreateFaceVertexCountsAttr([3] * (len(faces) // 3))
        mesh.CreateFaceVertexIndicesAttr(faces)
        mesh.CreateDoubleSidedAttr(True)
        mesh.CreateSubdivisionSchemeAttr("none")
        if uvs:
            st = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex)
            st.Set([(u, 1.0 - v) for u, v in uvs])                  # MuJoCo 는 그림의 위가 v = 0
        prim = mesh.GetPrim()
        place(prim, position, rotation)
    else:
        return None
    if geom.get("material") in looks:
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(looks[geom.get("material")])
    if geom.get("group") == "0" and kind not in ("plane", "mesh"):      # 라이다 · 카메라에 보이는 것 → 충돌체
        UsdPhysics.CollisionAPI.Apply(prim)
    return prim


def add_cameras(stage, mjcf):
    """설계자가 정해 둔 시점들을 Camera 로 옮긴다. MuJoCo 와 USD 의 카메라는 둘 다 −z 쪽을 본다."""
    paths = []
    for item in mjcf.iter("camera"):
        camera = UsdGeom.Camera.Define(stage, f"{BOOTH}/Cameras/{item.get('name')}")
        camera.CreateHorizontalApertureAttr(36.0)
        camera.CreateVerticalApertureAttr(20.25)
        camera.CreateFocalLengthAttr(10.125 / math.tan(math.radians(float(item.get("fovy", 45))) / 2))     # 세로 시야각
        camera.CreateClippingRangeAttr(Gf.Vec2f(0.05, 200.0))
        place(camera.GetPrim(), numbers(item.get("pos")), rotation_from_axes(numbers(item.get("xyaxes"))))
        paths.append(str(camera.GetPath()))
    return paths


def add_booth(stage, mjcf):
    """booth.xml 의 내용을 /World/Booth 아래에 만든다. 옮긴 수를 돌려준다."""
    looks = add_materials(stage, mjcf)
    meshes = {}
    for item in mjcf.findall("asset/mesh"):
        points = np.array(numbers(item.get("vertex"))).reshape(-1, 3)
        uvs = np.array(numbers(item.get("texcoord"))).reshape(-1, 2).tolist() if item.get("texcoord") else None
        meshes[item.get("name")] = (points, [int(v) for v in item.get("face").split()], uvs)
    UsdGeom.Xform.Define(stage, BOOTH)
    UsdGeom.Gprim(stage.GetPrimAtPath("/World/GroundVisual")).GetDisplayColorAttr().Set([color_value((0.34, 0.34, 0.36))])   # 전시장 바닥 색
    world = mjcf.find("worldbody")
    count = Counter()
    UsdGeom.Scope.Define(stage, f"{BOOTH}/Parts")
    for geom in world.findall("geom"):                         # 벽 · 가구처럼 따로 놓인 도형
        prim = add_geom(stage, f"{BOOTH}/Parts", geom, looks, meshes)
        count[geom.get("type") if prim else "옮기지 않음"] += 1
    UsdGeom.Scope.Define(stage, f"{BOOTH}/People")
    for body in world.findall("body"):                         # 사람 · 로봇 모형: 도형 여러 개의 묶음
        group = UsdGeom.Xform.Define(stage, f"{BOOTH}/People/{body.get('name')}")
        place(group.GetPrim(), numbers(body.get("pos")), rotation_from_euler(numbers(body.get("euler", "0 0 0"))))
        for geom in body.findall("geom"):
            prim = add_geom(stage, str(group.GetPath()), geom, looks, meshes)
            count[geom.get("type") if prim else "옮기지 않음"] += 1
    cameras = add_cameras(stage, mjcf)
    for name, eye, target in ROBOT_VIEWS:                       # 로봇을 올렸을 때 쓰는 시점 두 개
        camera = UsdGeom.Camera.Define(stage, f"{BOOTH}/Cameras/{name}")
        camera.CreateFocalLengthAttr(24.0)
        camera.CreateClippingRangeAttr(Gf.Vec2f(0.05, 200.0))
    colliders = sum(1 for prim in stage.Traverse() if prim.HasAPI(UsdPhysics.CollisionAPI) and str(prim.GetPath()).startswith(BOOTH))
    print(f"옮긴 도형: " + ", ".join(f"{kind} {n}" for kind, n in count.most_common()), flush=True)
    print(f"재질 {len(looks)}개, 카메라 {len(cameras)}개, 충돌체 {colliders}개", flush=True)
    return cameras


def add_spot(stage, path, position, aim, intensity, color=WARM, cone=50.0, radius=0.04):
    """한쪽으로만 비추는 조명 (스포트라이트): Sphere Light + Shaping 의 Cone. aim = 비추는 방향."""
    light = UsdLux.SphereLight.Define(stage, path)
    light.CreateRadiusAttr(radius)
    light.CreateNormalizeAttr(True)                 # 밝기가 조명의 크기에 따라 달라지지 않게
    light.CreateIntensityAttr(intensity)
    light.CreateColorAttr(Gf.Vec3f(*color))
    shaping = UsdLux.ShapingAPI.Apply(light.GetPrim())
    shaping.CreateShapingConeAngleAttr(cone)
    shaping.CreateShapingConeSoftnessAttr(0.4)
    place(light.GetPrim(), position, rotation_to(-np.array(aim, dtype=float)))       # 조명은 자기 −Z 쪽을 비춘다
    return light


def add_tube(stage, name, start, end, intensity, color=PINK, radius=0.012):
    """LED 줄: 두 점 사이에 빛나는 가는 기둥(눈에 보이는 것) + Cylinder Light(주변을 비추는 것)."""
    start, end = np.array(start, dtype=float), np.array(end, dtype=float)
    length, center = float(np.linalg.norm(end - start)), (start + end) / 2
    along_x = rotation_to(end - start) @ np.array([[0, 0, -1], [0, 1, 0], [1, 0, 0]])     # 자기 X 축이 두 점을 잇게
    light = UsdLux.CylinderLight.Define(stage, f"{BOOTH}/Lights/{name}")
    light.CreateLengthAttr(length)
    light.CreateRadiusAttr(radius)
    light.CreateNormalizeAttr(True)
    light.CreateIntensityAttr(intensity)
    light.CreateColorAttr(Gf.Vec3f(*color))
    place(light.GetPrim(), center, along_x)
    tube = UsdGeom.Cylinder.Define(stage, f"{BOOTH}/Leds/{name}")
    tube.CreateAxisAttr("Z")
    tube.CreateRadiusAttr(radius)
    tube.CreateHeightAttr(length)
    place(tube.GetPrim(), center, rotation_to(end - start))
    UsdShade.MaterialBindingAPI.Apply(tube.GetPrim()).Bind(UsdShade.Material(stage.GetPrimAtPath(f"{LOOKS}/led_{'pink' if color == PINK else 'white'}")))


def add_ribbon(stage, name, points, drop=0.035):
    """구불구불한 간판의 아래 모서리를 따라가는 빛나는 띠 (점들을 잇는 네모들)."""
    mesh = UsdGeom.Mesh.Define(stage, f"{BOOTH}/Leds/{name}")
    vertices, counts, indices = [], [], []
    for x, y, z in points:
        vertices += [(float(x), float(y), float(z)), (float(x), float(y), float(z) - drop)]
    for i in range(len(points) - 1):
        counts.append(4)
        indices += [2 * i, 2 * i + 2, 2 * i + 3, 2 * i + 1]
    mesh.CreatePointsAttr(vertices)
    mesh.CreateFaceVertexCountsAttr(counts)
    mesh.CreateFaceVertexIndicesAttr(indices)
    mesh.CreateDoubleSidedAttr(True)
    mesh.CreateSubdivisionSchemeAttr("none")
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(UsdShade.Material(stage.GetPrimAtPath(f"{LOOKS}/led_white")))


def add_lights(stage, mjcf):
    """전시장 조명: 어두운 홀 + 부스의 등기구 12개 + 위에서 비추는 큰 조명 + LED 줄. 만든 조명의 수를 돌려준다."""
    UsdLux.DistantLight(stage.GetPrimAtPath("/World/Light")).GetIntensityAttr().Set(180.0)        # 홀 전체는 어둡게
    sky = UsdLux.DomeLight(stage.GetPrimAtPath("/World/SkyLight"))
    sky.GetIntensityAttr().Set(220.0)
    sky.CreateColorAttr(Gf.Vec3f(*COOL))
    hall = make_material(stage, "hall_floor", color=(0.20, 0.20, 0.22), roughness=0.22)             # 매끈한 전시장 바닥
    UsdShade.MaterialBindingAPI.Apply(stage.GetPrimAtPath("/World/GroundVisual")).Bind(hall)
    make_material(stage, "led_pink", color=PINK, glow=12.0)
    make_material(stage, "led_white", color=(1.0, 0.92, 0.96), glow=10.0)
    UsdGeom.Scope.Define(stage, f"{BOOTH}/Lights")
    UsdGeom.Scope.Define(stage, f"{BOOTH}/Leds")
    count = 0
    for geom in mjcf.iter("geom"):                              # 설계의 등기구(벽 위 3 · 뒷벽 4 · 물결 간판 5): 빛나는 원판 자리에 조명
        if geom.get("name").startswith("lamp_glow"):
            position, yaw = numbers(geom.get("pos")), numbers(geom.get("euler"))[2]
            out = (math.cos(yaw), math.sin(yaw))                # 등기구의 팔이 벽에서 뻗어 나온 방향
            add_spot(stage, f"{BOOTH}/Lights/{geom.get('name')}", position, (-0.38 * out[0], -0.38 * out[1], -1.0), LAMP)
            count += 1
    add_spot(stage, f"{BOOTH}/Lights/booth_key", (4.5, -3.0, 9.0), (0.0, 0.0, -1.0), KEY, color=(1.0, 0.97, 0.94), cone=38.0, radius=0.3)
    for k, (x, y) in enumerate([(1.3, -4.5), (3.2, -4.6), (3.2, -1.6), (5.6, -1.2), (6.3, -3.2), (6.4, -4.9), (8.2, -4.2)]):
        add_spot(stage, f"{BOOTH}/Lights/down_{k}", (x, y, 4.4), (0.0, 0.0, -1.0), DOWN, cone=58.0)       # 부스 안을 위에서 비추는 조명
        count += 1
    wave = next(item for item in mjcf.findall("asset/mesh") if len(item.get("vertex").split()) > 1500)   # 물결 간판의 옆면
    points = np.array(numbers(wave.get("vertex"))).reshape(-1, 3)
    edge = [p for p in points[:226] if abs(p[2] - points[:, 2].min()) < 1e-6]         # 한쪽 면의 아래 모서리
    add_ribbon(stage, "wave_led", edge)
    for k, i in enumerate(range(12, len(edge), 22)):            # 간판 아래로 떨어지는 분홍빛
        add_spot(stage, f"{BOOTH}/Lights/wave_wash_{k}", (edge[i][0], edge[i][1], edge[i][2] - 0.05), (0.0, 0.0, -1.0), WASH, color=PINK, cone=60.0)
        count += 1
    tubes = [("fascia_led", (2.2, 0.02, 3.69), (7.3, 0.02, 3.69), PINK),            # 정면 띠 아래
             ("pillar_led_a", (7.55, 0.01, 0.05), (7.55, 0.01, 4.45), (1.0, 0.92, 0.96)),   # 기둥 모서리 두 줄
             ("pillar_led_b", (7.55, -0.91, 0.05), (7.55, -0.91, 4.45), (1.0, 0.92, 0.96)),
             ("desk_led_a", (7.52, -6.02, 0.06), (8.98, -6.02, 0.06), PINK),        # 인포 데스크 아래
             ("desk_led_b", (9.02, -4.52, 0.06), (9.02, -5.98, 0.06), PINK)]
    for name, start, end, color in tubes:
        add_tube(stage, name, start, end, LED, color)
    return count + 1 + len(tubes)


def add_cups(stage):
    """음료대 위에 컵 세 개 (지름 5 cm, 높이 9 cm, 100 g 의 원기둥). 뒤 강의에서 집을 물체."""
    UsdGeom.Scope.Define(stage, "/World/Cups")
    for name, center, color in CUPS:
        cup = UsdGeom.Cylinder.Define(stage, f"/World/Cups/{name}")
        cup.CreateAxisAttr("Z")
        cup.CreateRadiusAttr(CUP_SIZE[0] / 2)
        cup.CreateHeightAttr(CUP_SIZE[1])
        cup.CreateDisplayColorAttr([color_value(color)])
        place(cup.GetPrim(), center)
        UsdPhysics.CollisionAPI.Apply(cup.GetPrim())
        UsdPhysics.RigidBodyAPI.Apply(cup.GetPrim())
        UsdPhysics.MassAPI.Apply(cup.GetPrim()).CreateMassAttr(CUP_MASS)


def save_photos(app, stage, cameras, folder):
    """설계 시점마다 한 장씩 찍어 PNG 로 저장한다."""
    import omni.replicator.core as rep
    from PIL import Image
    os.makedirs(folder, exist_ok=True)
    saved = []
    for camera in cameras:
        render_product = rep.create.render_product(camera, (1600, 900))
        annotator = rep.AnnotatorRegistry.get_annotator("rgb")
        annotator.attach([render_product])
        data = np.zeros(0)
        for _ in range(400):                       # 처음에는 그림(텍스처)을 읽느라 화면이 늦게 나온다
            app.update()
            data = np.asarray(annotator.get_data())
            if data.ndim == 3 and _ > 90:
                break
        path = os.path.join(folder, f"booth_{camera.rsplit('/', 1)[1]}.png")
        Image.fromarray(data[:, :, :3]).save(path)
        saved.append(path)
        annotator.detach()
        render_product.destroy()
    return saved


if __name__ == "__main__":
    stage = new_scene()
    cameras = add_booth(stage, mjcf)
    if not args.plain:
        print(f"조명 {add_lights(stage, mjcf)}개 (등기구 12 · 위의 큰 조명 1 · 안쪽 조명 7 · 분홍빛 5 · LED 줄 5)", flush=True)
        import carb
        settings = carb.settings.get_settings()
        settings.set("/rtx/post/lensFlares/enabled", True)           # 밝은 곳이 번져 보이게 (Bloom)
        settings.set("/rtx/post/lensFlares/flareScale", 0.12)
        settings.set("/rtx/reflections/enabled", True)               # 바닥에 조명이 비치게
        settings.set("/rtx/indirectDiffuse/enabled", True)           # 벽에 반사된 빛
    if args.robot:
        sys.path.insert(0, os.path.join(REPO, "37_pl2m_import"))
        from import_pl2m import BODY, find_urdf, import_pl2m
        robot = import_pl2m(stage, find_urdf(os.path.join(REPO, "work", "pl2m", "pl2m.urdf")))
        set_pose(robot, translate=(SPAWN[0], SPAWN[1], 0.07))          # 부스 바닥판(2 cm) 위 5 cm
        add_cups(stage)
    save_stage(stage, args.save)
    if args.gui:
        set_view(eye=(10.6, -11.4, 9.2), target=(4.3, -3.0, 0.8))
    sim = Sim(app)
    if not args.no_photo:
        from isaacsim.core.utils.viewports import set_camera_view
        for name, eye, target in ROBOT_VIEWS:
            set_camera_view(eye=list(eye), target=list(target), camera_prim_path=f"{BOOTH}/Cameras/{name}")
        wanted = [f"{BOOTH}/Cameras/{name}" for name, _, _ in ROBOT_VIEWS] if args.robot else cameras
        if os.environ.get("BOOTH_VIEWS"):          # 조명을 맞출 때: 몇 장만 찍기 (예: BOOTH_VIEWS="bird1 studio")
            wanted = [f"{BOOTH}/Cameras/{name}" for name in os.environ["BOOTH_VIEWS"].split()]
        sim.play()                                 # 사진은 Play 중에 찍힌다
        sim.run(1.0)
        for path in save_photos(app, stage, wanted, os.path.join(os.path.dirname(args.save), "photos")):
            print("사진 저장:", path, flush=True)
    if args.robot:
        body = stage.GetPrimAtPath(BODY)
        cups = [stage.GetPrimAtPath(f"/World/Cups/{name}") for name, _, _ in CUPS]
        sim.play()
        sim.run(1.0)                               # 바닥에 자리 잡을 때까지
        start, (x0, y0, z0, yaw0) = sim.time, pose_of(body)
        print(f"{'시간[s]':>8} {'로봇 x':>8} {'y':>8} {'z':>8} {'방향[°]':>8}   컵 높이[m]", flush=True)

        def report(time):
            x, y, z, yaw = pose_of(body)
            heights = " ".join(f"{pose_of(cup)[2]:.3f}" for cup in cups)
            print(f"{time - start:8.1f} {x:8.3f} {y:8.3f} {z:8.3f} {round(yaw, 1) + 0.0:8.1f}   {heights}", flush=True)

        report(start)
        sim.run(args.seconds, every=1.0, report=report)
        x, y, z, yaw = pose_of(body)
        print(f"결과: {args.seconds:g}초 동안 로봇 이동 {math.hypot(x - x0, y - y0) * 1000:.1f} mm, 회전 {yaw - yaw0:+.2f}°.", flush=True)
    if args.gui:
        sim.play()
        sim.wait_until_closed()
    sim.stop()
    app.close()
