"""강의 3 에서 마우스로 만든 "떨어지는 큐브" 장면을 코드로 만듭니다.

    ~/isaacsim/python.sh 03_first_scene/falling_cube.py                    # 전부 갖춘 장면: 바닥 위에 멈춤
    ~/isaacsim/python.sh 03_first_scene/falling_cube.py --no-rigid-body    # 강체가 아님: 안 떨어짐
    ~/isaacsim/python.sh 03_first_scene/falling_cube.py --no-ground        # 바닥 없음: 끝없이 떨어짐
    ~/isaacsim/python.sh 03_first_scene/falling_cube.py --gravity-up       # 중력이 위로: 떠오름
    ~/isaacsim/python.sh 03_first_scene/falling_cube.py --gui              # 창으로 보기
    ~/isaacsim/python.sh 03_first_scene/falling_cube.py --save work/falling_cube.usd

GUI 의 동작 하나가 아래 코드 한 덩어리에 해당합니다.
    Create > Mesh > Cube                               → UsdGeom.Cube
    Create > Physics > Physics Scene                   → UsdPhysics.Scene
    Add > Physics > Rigid Body with Colliders Preset   → UsdPhysics.RigidBodyAPI + CollisionAPI
    Create > Physics > Ground Plane                    → UsdGeom.Plane + CollisionAPI
"""
import argparse
import os
import time

from isaacsim import SimulationApp

parser = argparse.ArgumentParser()
parser.add_argument("--no-rigid-body", action="store_true", help="큐브에 강체 · 충돌체를 주지 않는다")
parser.add_argument("--no-ground", action="store_true", help="바닥을 만들지 않는다")
parser.add_argument("--gravity-up", action="store_true", help="중력 방향을 +Z 로 한다")
parser.add_argument("--height", type=float, default=1.0, help="큐브 중심의 처음 높이 [m]")
parser.add_argument("--seconds", type=float, default=2.0, help="시뮬레이션할 시간 [s]")
parser.add_argument("--gui", action="store_true", help="창을 띄운다 (끝나도 닫을 때까지 유지)")
parser.add_argument("--save", metavar="USD", help="Play 전 장면을 이 경로에 저장한다")
args = parser.parse_args()

app = SimulationApp({"headless": not args.gui})

import omni.physx
import omni.timeline
import omni.usd
from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdPhysics

PRINT_EVERY = 0.25   # 큐브 높이를 출력할 간격 [s]

# ── 새 장면: 단위는 미터, 위쪽은 Z ─────────────────────────────────
omni.usd.get_context().new_stage()
stage = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
world = UsdGeom.Xform.Define(stage, "/World")
stage.SetDefaultPrim(world.GetPrim())
UsdLux.DistantLight.Define(stage, "/World/Light").CreateIntensityAttr(1000.0)

# ── 큐브: 한 변 1 m. 여기까지는 보이는 모양일 뿐 ────────────────────
cube = UsdGeom.Cube.Define(stage, "/World/Cube")
cube.CreateSizeAttr(1.0)
translate = cube.AddTranslateOp()
translate.Set(Gf.Vec3d(0.0, 0.0, args.height))

# ── 물리 장면: 중력 ────────────────────────────────────────────────
scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
scene.CreateGravityDirectionAttr(Gf.Vec3f(0.0, 0.0, 1.0 if args.gravity_up else -1.0))
scene.CreateGravityMagnitudeAttr(9.81)

# ── 강체(힘을 받아 움직임) + 충돌체(부딪힘) ─────────────────────────
if not args.no_rigid_body:
    UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())

# ── 바닥: 충돌면은 무한히 넓은 평면 ─────────────────────────────────
if not args.no_ground:
    ground = UsdGeom.Plane.Define(stage, "/World/GroundPlane")
    ground.CreateAxisAttr(UsdGeom.Tokens.z)
    ground.CreatePurposeAttr(UsdGeom.Tokens.guide)   # 충돌용이라 렌더링하지 않음
    UsdPhysics.CollisionAPI.Apply(ground.GetPrim())
    if args.gui:                                     # 눈에 보이는 판 (충돌과는 무관)
        slab = UsdGeom.Cube.Define(stage, "/World/GroundVisual")
        slab.CreateSizeAttr(1.0)
        slab.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, -0.005))
        slab.AddScaleOp().Set(Gf.Vec3f(25.0, 25.0, 0.01))
        slab.CreateDisplayColorAttr([Gf.Vec3f(0.5, 0.5, 0.5)])

if args.save:
    path = os.path.abspath(os.path.expanduser(args.save))
    os.makedirs(os.path.dirname(path), exist_ok=True)
    stage.GetRootLayer().Export(path)
    print("장면 저장:", path, flush=True)

for _ in range(10):
    app.update()

# ── Play: 물리 스텝(1/60 초)마다 시간을 세고, 0.25 초마다 큐브 높이를 읽는다 ──
sim_time = 0.0


def on_physics_step(dt):
    global sim_time
    sim_time += dt


def cube_z():
    """월드 좌표에서 큐브 중심의 높이."""
    return omni.usd.get_world_transform_matrix(cube.GetPrim()).ExtractTranslation()[2]


physics_sub = omni.physx.get_physx_interface().subscribe_physics_step_events(on_physics_step)
timeline = omni.timeline.get_timeline_interface()
warmup_end = time.time() + float(os.environ.get("SIM_WARMUP", "0"))   # 창이 다 그려질 때까지 기다렸다가 Play (녹화할 때)
while time.time() < warmup_end:
    app.update()
timeline.play()

print(f"{'시간[s]':>8} {'큐브 높이 z[m]':>14}", flush=True)
print(f"{sim_time:8.2f} {cube_z():14.3f}", flush=True)
next_print = PRINT_EVERY
while sim_time < args.seconds - 1e-6:
    app.update()                      # 화면 한 프레임. 그 안에서 물리 스텝이 진행된다
    if sim_time >= next_print - 1e-6:
        print(f"{sim_time:8.2f} {cube_z():14.3f}", flush=True)
        next_print += PRINT_EVERY

z = cube_z()
if abs(z - args.height) < 1e-3:
    print("결과: 움직이지 않았습니다.", flush=True)
elif abs(z - 0.5) < 0.01 and not args.no_ground:
    print("결과: 바닥 위에 멈췄습니다 (한 변 1 m 큐브의 중심 높이 0.5 m).", flush=True)
elif z < 0:
    print("결과: 바닥이 없어 계속 떨어집니다.", flush=True)
elif z > args.height:
    print("결과: 중력이 위를 향해 계속 올라갑니다.", flush=True)
else:
    print("결과: 아직 떨어지는 중입니다. --seconds 를 늘려 보세요.", flush=True)

while args.gui and app.is_running():
    app.update()
physics_sub = None
timeline.stop()
app.close()
