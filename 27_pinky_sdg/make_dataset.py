"""강의 27: 합성 데이터(SDG) — 핑크시티에서 "다른 Pinky Pro 찾기" 학습 데이터를 Replicator 로 만듭니다.

    ~/isaacsim/python.sh 27_pinky_sdg/make_dataset.py                 # 600장 (work/pinky_dataset)
    ~/isaacsim/python.sh 27_pinky_sdg/make_dataset.py --count 40 --out work/pinky_dataset_test
    ~/isaacsim/python.sh 27_pinky_sdg/make_dataset.py --count 40 --gui

만드는 것 (work/pinky_dataset)
    RobotCamera/rgb/0.png …             Pinky Pro 카메라 자리 · 화각에서 본 영상 (640 × 360)
    RobotCamera/object_detection/0.txt … 정답: 영상에 보이는 Pinky Pro 마다 한 줄 (KITTI 형식, 상자의 왼쪽 · 위 · 오른쪽 · 아래)
    preview/000.jpg …                   상자를 그려 넣은 확인용 그림 (처음 24장)

강의 19 와의 차이
    강의 19 는 정답(도로 마스크)을 직접 계산했습니다. 여기서는 Replicator 가 정답을 만듭니다.
      1. 물체에 이름표를 붙인다            add_labels(prim, ["pinky"], "class")
      2. 장마다 무엇을 바꿀지 적는다        rep.trigger.on_frame() 안의 rep.modify / rep.distribution  (해의 방향과 밝기, LED 색)
      3. 기록할 형식을 고른다              rep.WriterRegistry.get("KittiWriter")
      4. 한 장씩 찍는다                    rep.orchestrator.step()
    로봇과 카메라의 자리만은 "도로 위, 카메라 앞" 이 되도록 이 파일에서 직접 뽑습니다.
"""
import argparse
import math
import os
import sys

from isaacsim import SimulationApp

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, ".."))

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--count", type=int, default=600, help="만들 장수")
    parser.add_argument("--out", default=os.path.join(REPO, "work", "pinky_dataset"), help="저장할 폴더")
    parser.add_argument("--seed", type=int, default=0, help="난수 씨앗 (같은 값이면 같은 자리)")
    parser.add_argument("--gui", action="store_true", help="창을 띄운다 (느려짐)")
    args = parser.parse_args()
    app = SimulationApp({"headless": not args.gui})

import numpy as np
import omni.kit.commands
import omni.replicator.core as rep
import omni.usd
from isaacsim.core.utils.semantics import add_labels
from PIL import Image, ImageDraw
from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics

for folder in ("", "11_pinky_import", "12_pinky_sensors", "13_pinky_lcd_led", "16_pink_city"):
    sys.path.insert(0, os.path.join(REPO, folder))
import pink_city
from common.sim import new_scene
from import_pinky import DEFAULT_URDF, JOINTS, ROBOT, find_urdf, import_pinky
from pinky_face_led import FACES, LED_MATERIAL, add_led, add_screen, face_path, set_face
from pinky_sensors import add_camera

SIZE = (640, 360)                      # 영상 크기 (강의 19 와 같다)
ROBOTS = 3                             # 도시에 세워 둘 Pinky Pro 대수
CAMERA = "/World/RobotCamera"       # 기록 폴더 이름이 이 카메라 이름(RobotCamera)이 된다
ROAD, RING, CORNER = pink_city.ROAD, pink_city.RING, pink_city.RING_CORNER


def road_points(step=0.02):
    """도로 중심선 위의 점들 (x, y, 진행 방향[rad]). 순환 도로의 네 변과 십자 도로."""
    points = []
    reach = RING - CORNER
    for d in np.arange(-reach, reach, step):
        points += [(d, RING, 0.0), (d, -RING, 0.0), (RING, d, math.pi / 2), (-RING, d, math.pi / 2)]
    inner = RING - ROAD / 2
    for d in np.arange(-inner, inner, step):
        points += [(d, 0.0, 0.0), (0.0, d, math.pi / 2)]
    return np.array(points)


def on_ground(labels, x, y):
    """그 자리가 로봇이 설 수 있는 바닥(도로 · 보도 · 공원)인가. 건물 터와 도시 밖은 아니다."""
    n = labels.shape[0]
    if abs(x) > pink_city.HALF - 0.12 or abs(y) > pink_city.HALF - 0.12:
        return False
    col = int((x + pink_city.HALF) * pink_city.PX_PER_M)
    row = int((pink_city.HALF - y) * pink_city.PX_PER_M)
    return labels[min(row, n - 1), min(col, n - 1)] != pink_city.LABEL_OTHER


def strip_physics(stage, root):
    """로봇을 "모양만 있는 물체" 로 만든다. 찍는 동안 물리 계산으로 움직이지 않게 관절과 강체 설정을 지운다."""
    stage.RemovePrim(JOINTS)
    for prim in Usd.PrimRange(stage.GetPrimAtPath(root)):
        for api in (UsdPhysics.ArticulationRootAPI, UsdPhysics.RigidBodyAPI, UsdPhysics.MassAPI, UsdPhysics.CollisionAPI,
                    PhysxSchema.PhysxArticulationAPI, PhysxSchema.PhysxRigidBodyAPI, PhysxSchema.PhysxCollisionAPI):
            if prim.HasAPI(api):
                prim.RemoveAPI(api)


def add_robots(stage):
    """Pinky Pro 를 ROBOTS 대 놓고 이름표 "pinky" 를 붙인다. 각 로봇의 (prim, 위치를 바꾸는 손잡이) 를 돌려준다."""
    import_pinky(stage, find_urdf(DEFAULT_URDF), height=0.0)
    robot_camera = UsdGeom.Camera(add_camera(stage))                  # 카메라 자리 · 렌즈 값을 읽으려고 잠깐 단다
    for _ in range(5):
        app.update()
    robot_to_world = omni.usd.get_world_transform_matrix(stage.GetPrimAtPath(ROBOT))
    mount = omni.usd.get_world_transform_matrix(robot_camera.GetPrim()) * robot_to_world.GetInverse()
    camera = UsdGeom.Camera.Define(stage, CAMERA)
    for name in ("HorizontalAperture", "VerticalAperture", "FocalLength", "ClippingRange"):
        getattr(camera, f"Create{name}Attr")(getattr(robot_camera, f"Get{name}Attr")().Get())
    camera_op = camera.AddTransformOp()
    stage.RemovePrim(robot_camera.GetPath())

    add_screen(stage, face_path("basic"))                             # 강의 13 의 화면과 LED
    add_led(stage, (0.0, 1.0, 0.2))
    strip_physics(stage, ROBOT)
    paths = [ROBOT] + [f"/World/pinky_{k}" for k in range(1, ROBOTS)]
    for path in paths[1:]:
        omni.kit.commands.execute("CopyPrim", path_from=ROBOT, path_to=path, exclusive_select=False)
    robots = []
    for path in paths:
        prim = stage.GetPrimAtPath(path)
        add_labels(prim, labels=["pinky"], instance_name="class")     # ← 이름표. Replicator 는 이름표가 있는 물체만 정답에 적는다
        xform = UsdGeom.Xformable(prim)
        xform.ClearXformOpOrder()
        robots.append((prim, xform.AddTransformOp()))
    return robots, camera_op, mount


def pose_matrix(x, y, yaw):
    return Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(0, 0, 1), math.degrees(yaw))) * Gf.Matrix4d().SetTranslate(Gf.Vec3d(x, y, 0.0))


def sample_scene(rng, points, labels):
    """한 장의 배치: 카메라를 든 로봇의 자리와, 다른 Pinky Pro 들의 자리."""
    x, y, heading = points[rng.integers(len(points))]
    if rng.random() < 0.5:
        heading += math.pi
    offset = rng.uniform(-0.15, 0.15)
    cx, cy = x - offset * math.sin(heading), y + offset * math.cos(heading)
    cyaw = heading + math.radians(rng.normal(0.0, 20.0))
    placed = []
    for _ in range(ROBOTS):
        for _ in range(200):
            if rng.random() < 0.7:                           # 카메라 앞 0.18 ~ 1.4 m, 좌우 ±30° 안
                distance, bearing = rng.uniform(0.18, 1.4), cyaw + math.radians(rng.uniform(-30.0, 30.0))
                px, py = cx + distance * math.cos(bearing), cy + distance * math.sin(bearing)
            else:                                            # 도시 어딘가 (화면에 안 보일 수도 있다)
                px, py = rng.uniform(-pink_city.HALF, pink_city.HALF, size=2)
            far_enough = all(math.hypot(px - qx, py - qy) > 0.16 for qx, qy, _ in placed + [(cx, cy, 0.0)])
            if on_ground(labels, px, py) and far_enough:
                placed.append((px, py, rng.uniform(-math.pi, math.pi)))
                break
        else:
            placed.append((5.0 + len(placed), 5.0, 0.0))        # 자리를 못 찾으면 도시 밖에 치워 둔다
    return (cx, cy, cyaw), placed


def read_boxes(path):
    """KITTI 정답 파일 → [(왼쪽, 위, 오른쪽, 아래), …]"""
    boxes = []
    if os.path.exists(path):
        for line in open(path):
            v = line.split()
            if len(v) >= 8:
                boxes.append(tuple(float(t) for t in v[4:8]))
    return boxes


if __name__ == "__main__":
    rng = np.random.default_rng(args.seed)
    rep.set_global_seed(args.seed)
    textures = pink_city.make_textures()
    labels = np.asarray(Image.open(textures["labels"]))
    stage = new_scene()
    pink_city.add_city(stage, textures)
    robots, camera_op, mount = add_robots(stage)
    faces = sorted(os.path.join(FACES, f) for f in os.listdir(FACES) if f.endswith(".png"))
    points = road_points()

    # 장마다 Replicator 가 바꾸는 것: 해의 방향 · 밝기, LED 색
    with rep.trigger.on_frame():
        with rep.get.prims(path_pattern="/World/Light$"):
            rep.modify.attribute("xformOp:rotateXYZ", rep.distribution.uniform((15.0, -40.0, 0.0), (60.0, 40.0, 0.0)))
            rep.modify.attribute("inputs:intensity", rep.distribution.uniform(900.0, 2200.0))
        with rep.get.prims(path_pattern=LED_MATERIAL + "/Shader$"):
            rep.modify.attribute("inputs:emissive_color", rep.distribution.uniform((0.0, 0.0, 0.0), (1.0, 1.0, 1.0)))

    # 기록: 영상 + 상자 정답 (KITTI 형식)
    out = os.path.abspath(os.path.expanduser(args.out))
    writer = rep.WriterRegistry.get("KittiWriter")
    writer.initialize(output_dir=out, omit_semantic_type=True)
    writer.attach(rep.create.render_product(CAMERA, SIZE))

    print(f"핑크시티에서 {args.count}장을 만듭니다 → {out}", flush=True)
    for frame in range(args.count):
        (cx, cy, cyaw), placed = sample_scene(rng, points, labels)
        camera_op.Set(mount * pose_matrix(cx, cy, cyaw))
        for (prim, op), (px, py, pyaw) in zip(robots, placed):
            op.Set(pose_matrix(px, py, pyaw))
        set_face(stage, faces[rng.integers(len(faces))])
        rep.orchestrator.step(rt_subframes=8)                # 한 장 찍기 (자리가 바뀐 뒤 그림이 또렷해지도록 8번 그린다)
        if (frame + 1) % 50 == 0 or frame + 1 == args.count:
            print(f"  {frame + 1} / {args.count}", flush=True)
    rep.orchestrator.wait_until_complete()

    # 확인: 상자 수 세기, 처음 24장에 상자를 그려 저장
    rgb_dir, label_dir = os.path.join(out, "RobotCamera", "rgb"), os.path.join(out, "RobotCamera", "object_detection")
    os.makedirs(os.path.join(out, "preview"), exist_ok=True)
    names = sorted((f[:-4] for f in os.listdir(rgb_dir) if f.endswith(".png")), key=int)
    per_image = [len(read_boxes(os.path.join(label_dir, n + ".txt"))) for n in names]
    for n in names[:24]:
        image = Image.open(os.path.join(rgb_dir, n + ".png")).convert("RGB")
        draw = ImageDraw.Draw(image)
        for box in read_boxes(os.path.join(label_dir, n + ".txt")):
            draw.rectangle(box, outline=(255, 60, 160), width=2)
        image.save(os.path.join(out, "preview", f"{int(n):03d}.jpg"), quality=90)
    print(f"저장: 영상 {len(names)}장, 상자 {sum(per_image)}개", flush=True)
    for k in range(ROBOTS + 1):
        print(f"  Pinky Pro 가 {k}대 보이는 영상: {per_image.count(k)}장", flush=True)
    app.close()
