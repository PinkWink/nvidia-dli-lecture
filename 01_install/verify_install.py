"""설치 확인용: Isaac Sim 을 창 없이 켜고, 큐브를 0.5초 떨어뜨려 본 뒤 끕니다.

verify_install.sh 가 실행합니다. 직접 실행하려면:
    ~/isaacsim/python.sh 01_install/verify_install.py
"""
from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

import omni.kit.app
import omni.timeline
import omni.usd
from pxr import UsdGeom, UsdPhysics

print("VERIFY 버전", omni.kit.app.get_app().get_app_version(), flush=True)

# 높이 5 m 에 큐브를 두고 물리를 0.5초 돌립니다. 떨어지면 물리 엔진(GPU 포함)이 동작하는 것입니다.
stage = omni.usd.get_context().get_stage()
UsdPhysics.Scene.Define(stage, "/PhysicsScene")
cube = UsdGeom.Cube.Define(stage, "/Cube")
move = cube.AddTranslateOp()
move.Set((0.0, 0.0, 5.0))
UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())
UsdPhysics.CollisionAPI.Apply(cube.GetPrim())

timeline = omni.timeline.get_timeline_interface()
timeline.play()
while timeline.get_current_time() < 0.5:
    app.update()
z = omni.usd.get_world_transform_matrix(cube.GetPrim()).ExtractTranslation()[2]
timeline.stop()
print(f"VERIFY 높이 {z:.2f}", flush=True)
app.close()
