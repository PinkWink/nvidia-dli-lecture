"""강의 19: 도로 인식(세그먼테이션) 학습 데이터를 핑크시티에서 모읍니다 — 카메라 영상 + 정답 마스크.

    ~/isaacsim/python.sh 19_road_dataset/collect_dataset.py                 # 400장 (work/road_dataset)
    ~/isaacsim/python.sh 19_road_dataset/collect_dataset.py --count 1000    # 장수 바꾸기
    ~/isaacsim/python.sh 19_road_dataset/collect_dataset.py --count 20 --out work/road_dataset_test

만드는 것 (work/road_dataset)
    images/000000.png …    Pinky Pro 카메라와 같은 자리 · 같은 화각의 영상 (640 × 360)
    masks/000000.png …     정답: 픽셀마다 종류 번호 (0 그 밖, 1 보도 · 공원, 2 도로, 3 중앙선, 4 횡단보도 · 정지선)
    preview/000000.jpg …   눈으로 확인하는 그림 (영상 | 정답을 색으로 | 겹친 것) — 처음 24장만
    poses.csv              장마다 카메라를 둔 로봇의 위치 · 방향
    train.txt · val.txt    학습용 · 검증용 목록 (9 : 1)

정답을 만드는 방법
    Isaac Sim 은 영상과 함께 픽셀마다의 거리(depth)를 줍니다. 거리와 카메라 위치로 그 픽셀이 보는 점의 좌표를 계산해서,
    바닥(z ≈ 0)이면 pink_city.py 가 만든 "바닥 정답 지도"(floor_labels.png)에서 그 자리의 종류를 읽습니다.
    바닥이 아니면(건물 · 벽 · 나무 · 하늘) 0 번입니다. 사람이 손으로 칠할 필요가 없습니다.
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
    parser.add_argument("--count", type=int, default=400, help="모을 장수")
    parser.add_argument("--out", default=os.path.join(REPO, "work", "road_dataset"), help="저장할 폴더")
    parser.add_argument("--seed", type=int, default=0, help="난수 씨앗 (같은 값이면 같은 데이터)")
    parser.add_argument("--gui", action="store_true", help="창을 띄운다 (느려짐)")
    args = parser.parse_args()
    app = SimulationApp({"headless": not args.gui})

import numpy as np
import omni.replicator.core as rep
import omni.timeline
import omni.usd
from PIL import Image
from pxr import Gf, UsdGeom, UsdLux

for folder in ("", "11_pinky_import", "12_pinky_sensors", "16_pink_city"):
    sys.path.insert(0, os.path.join(REPO, folder))
import pink_city
from common.sim import new_scene
from import_pinky import DEFAULT_URDF, ROBOT, find_urdf, import_pinky
from pinky_sensors import CAMERA, add_camera

SIZE = (640, 360)                      # 영상 크기 (강의 15 의 /camera/image_raw 와 같다)
FREE_CAMERA = "/World/DatasetCamera"
ROAD, RING, CORNER = pink_city.ROAD, pink_city.RING, pink_city.RING_CORNER
# 정답을 색으로 보여 줄 때의 색 (종류 번호 순서)
PALETTE = np.array([(40, 40, 48), (232, 150, 190), (90, 96, 110), (250, 204, 60), (255, 255, 255)], dtype=np.uint8)


def road_centerlines(step=0.01):
    """도로 중심선 위의 점들: (x, y, 진행 방향[rad]) 의 배열. 순환 도로(둥근 모퉁이) + 십자 도로."""
    points = []
    s = RING - CORNER
    corners = [(s, s, 0.0), (-s, s, 90.0), (-s, -s, 180.0), (s, -s, 270.0)]      # 모퉁이 원의 중심과 시작 각도 (반시계 방향으로 돈다)
    for k, (cx, cy, start) in enumerate(corners):
        for a in np.arange(start, start + 90.0, math.degrees(step / CORNER)):   # 모퉁이의 곡선
            r = math.radians(a)
            points.append((cx + CORNER * math.cos(r), cy + CORNER * math.sin(r), r + math.pi / 2))
        ex, ey = cx + CORNER * math.cos(math.radians(start + 90)), cy + CORNER * math.sin(math.radians(start + 90))
        nx, ny, _ = corners[(k + 1) % 4]
        tx, ty = nx + CORNER * math.cos(math.radians(start + 90)), ny + CORNER * math.sin(math.radians(start + 90))
        heading = math.atan2(ty - ey, tx - ex)
        for d in np.arange(0.0, math.hypot(tx - ex, ty - ey), step):            # 다음 모퉁이까지의 직선
            points.append((ex + d * math.cos(heading), ey + d * math.sin(heading), heading))
    inner = RING - ROAD / 2
    for d in np.arange(-inner, inner, step):                                     # 십자 도로
        points.append((d, 0.0, 0.0))
        points.append((0.0, d, math.pi / 2))
    return np.array(points)


def sample_pose(rng, lines):
    """로봇을 둘 자리 하나를 뽑는다 → (x, y, yaw[rad], 종류)."""
    x, y, heading = lines[rng.integers(len(lines))]
    if rng.random() < 0.5:
        heading += math.pi                                   # 반대 방향으로도 달린다
    kind = rng.random()
    if kind < 0.6:           # 차로 안에서 정상 주행: 중심선에서 좌우 0.15 m 안, 방향 오차 ±15°
        offset, yaw, name = rng.uniform(-0.15, 0.15), heading + math.radians(rng.normal(0.0, 15.0)), "lane"
    elif kind < 0.85:        # 길을 벗어나려는 상황: 좌우 0.26 m 까지(보도 위 포함), 방향 오차 ±45°
        offset, yaw, name = rng.uniform(-0.26, 0.26), heading + math.radians(rng.uniform(-45.0, 45.0)), "edge"
    else:                    # 아무 방향이나 보기
        offset, yaw, name = rng.uniform(-0.26, 0.26), rng.uniform(-math.pi, math.pi), "any"
    return x - offset * math.sin(heading), y + offset * math.cos(heading), yaw, name


def camera_mount(stage):
    """Pinky Pro 를 잠깐 올려 "로봇 기준의 카메라 자리" 와 렌즈 값을 읽고, 같은 값의 자유 카메라를 만든다."""
    import_pinky(stage, find_urdf(DEFAULT_URDF), height=0.0)
    robot_camera = UsdGeom.Camera(add_camera(stage))
    for _ in range(5):
        app.update()
    robot_to_world = omni.usd.get_world_transform_matrix(stage.GetPrimAtPath(ROBOT))
    mount = omni.usd.get_world_transform_matrix(robot_camera.GetPrim()) * robot_to_world.GetInverse()
    camera = UsdGeom.Camera.Define(stage, FREE_CAMERA)
    for name in ("HorizontalAperture", "VerticalAperture", "FocalLength", "ClippingRange"):
        getattr(camera, f"Create{name}Attr")(getattr(robot_camera, f"Get{name}Attr")().Get())
    op = camera.AddTransformOp()
    stage.RemovePrim(ROBOT)                                  # 로봇은 치운다 (카메라만 순간 이동시키며 찍는다)
    return camera, op, mount


def pixel_rays(camera):
    """픽셀마다 "카메라 좌표에서 거리 1 일 때의 (x, y)" — 거리(depth)를 곱하면 그 픽셀이 보는 점이 된다."""
    width, height = SIZE
    fx = width * camera.GetFocalLengthAttr().Get() / camera.GetHorizontalApertureAttr().Get()      # 초점 거리 [픽셀]
    u, v = np.meshgrid(np.arange(width) + 0.5, np.arange(height) + 0.5)
    return (u - width / 2) / fx, -(v - height / 2) / fx                                            # USD 카메라: +X 오른쪽, +Y 위, −Z 앞


def make_mask(depth, rays, camera_to_world, labels):
    """거리 그림 → 정답 마스크. 바닥을 보는 픽셀은 바닥 정답 지도에서 종류를 읽고, 나머지는 0."""
    d = np.where(np.isfinite(depth), depth, 0.0)
    points = np.stack([rays[0] * d, rays[1] * d, -d, np.ones_like(d)], axis=-1)       # 카메라 좌표의 점
    world = points @ np.array(camera_to_world)                                       # 월드 좌표 (USD 는 행 벡터 × 행렬)
    x, y, z = world[..., 0], world[..., 1], world[..., 2]
    n = labels.shape[0]
    col = np.clip(((x + pink_city.HALF) * pink_city.PX_PER_M).astype(int), 0, n - 1)
    row = np.clip(((pink_city.HALF - y) * pink_city.PX_PER_M).astype(int), 0, n - 1)
    ground = np.isfinite(depth) & (np.abs(z) < 0.01) & (np.abs(x) < pink_city.HALF) & (np.abs(y) < pink_city.HALF)
    return np.where(ground, labels[row, col], pink_city.LABEL_OTHER).astype(np.uint8)


def preview(image, mask):
    """영상 | 정답(색) | 겹친 것 을 옆으로 붙인 그림."""
    color = PALETTE[mask]
    blend = (image.astype(np.float32) * 0.45 + color.astype(np.float32) * 0.55).astype(np.uint8)
    return Image.fromarray(np.concatenate([image, color, blend], axis=1))


if __name__ == "__main__":
    rng = np.random.default_rng(args.seed)
    textures = pink_city.make_textures()
    labels = np.asarray(Image.open(textures["labels"]))
    stage = new_scene()
    pink_city.add_city(stage, textures)
    camera, camera_op, mount = camera_mount(stage)
    rays = pixel_rays(camera)
    light = UsdLux.DistantLight(stage.GetPrimAtPath("/World/Light"))
    light_rotate = light.GetPrim().GetAttribute("xformOp:rotateXYZ")

    render_product = rep.create.render_product(FREE_CAMERA, SIZE)
    rgb = rep.AnnotatorRegistry.get_annotator("rgb")
    distance = rep.AnnotatorRegistry.get_annotator("distance_to_image_plane")     # 픽셀마다 카메라 앞쪽으로의 거리 [m]
    rgb.attach([render_product])
    distance.attach([render_product])
    omni.timeline.get_timeline_interface().play()
    for _ in range(60):
        app.update()

    out = os.path.abspath(os.path.expanduser(args.out))
    for name in ("images", "masks", "preview"):
        os.makedirs(os.path.join(out, name), exist_ok=True)
    lines = road_centerlines()
    rows, counts, saved = [], np.zeros(len(PALETTE), dtype=np.int64), 0
    print(f"핑크시티에서 {args.count}장을 모읍니다 → {out}", flush=True)
    while saved < args.count:
        x, y, yaw, kind = sample_pose(rng, lines)
        robot = Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(0, 0, 1), math.degrees(yaw))) * Gf.Matrix4d().SetTranslate(Gf.Vec3d(x, y, 0.0))
        camera_to_world = mount * robot
        camera_op.Set(camera_to_world)
        sun = (rng.uniform(15.0, 60.0), rng.uniform(-40.0, 40.0), rng.uniform(0.0, 360.0))     # 해의 방향 · 밝기를 장마다 바꾼다
        light_rotate.Set(Gf.Vec3f(sun[0], sun[1], 0.0))
        light.GetIntensityAttr().Set(float(rng.uniform(900.0, 2200.0)))
        for _ in range(12):                                  # 새 자리의 그림이 또렷해질 때까지 몇 프레임 기다린다
            app.update()
        image, depth = np.asarray(rgb.get_data()), np.asarray(distance.get_data())
        if image.ndim != 3 or depth.ndim != 2:
            continue
        if (depth < 0.03).mean() > 0.02:                     # 카메라가 물체 속에 들어간 자리는 버린다
            continue
        image = image[:, :, :3]
        mask = make_mask(depth, rays, camera_to_world, labels)
        name = f"{saved:06d}"
        Image.fromarray(image).save(os.path.join(out, "images", name + ".png"))
        Image.fromarray(mask).save(os.path.join(out, "masks", name + ".png"))
        if saved < 24:
            preview(image, mask).save(os.path.join(out, "preview", name + ".jpg"), quality=90)
        rows.append(f"{name},{x:.3f},{y:.3f},{(math.degrees(yaw) + 180.0) % 360.0 - 180.0:.1f},{kind}")
        counts += np.bincount(mask.ravel(), minlength=len(PALETTE))
        saved += 1
        if saved % 50 == 0 or saved == args.count:
            print(f"  {saved} / {args.count}", flush=True)

    with open(os.path.join(out, "poses.csv"), "w") as f:
        f.write("name,x,y,yaw_deg,kind\n" + "\n".join(rows) + "\n")
    order = rng.permutation(saved)
    n_val = max(1, saved // 10)
    for filename, indices in (("val.txt", order[:n_val]), ("train.txt", order[n_val:])):
        with open(os.path.join(out, filename), "w") as f:
            f.write("\n".join(f"{i:06d}" for i in sorted(indices)) + "\n")
    print(f"저장: 영상 {saved}장, 학습용 {saved - n_val} · 검증용 {n_val}", flush=True)
    print("종류별 픽셀 비율:", flush=True)
    for k, share in enumerate(counts / counts.sum()):
        print(f"  {k} {pink_city.LABEL_NAMES[k]:<22} {share * 100:5.1f} %", flush=True)
    omni.timeline.get_timeline_interface().stop()
    app.close()
