"""강의 스크립트가 함께 쓰는 작은 도구: 새 장면 만들기, 정해진 시간만큼 시뮬레이션하기, 위치 읽기.

SimulationApp 을 만든 뒤에 import 합니다 (omni · pxr 모듈을 쓰기 때문).
"""
import math
import os

import omni.physx
import omni.timeline
import omni.usd
from pxr import Gf, UsdGeom, UsdLux, UsdPhysics


def new_scene():
    """강의 3 의 장면: 단위 미터 · 위쪽 Z · 조명 · 물리 장면 · 바닥. 만든 stage 를 돌려준다."""
    omni.usd.get_context().new_stage()
    stage = omni.usd.get_context().get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())
    light = UsdLux.DistantLight.Define(stage, "/World/Light")
    light.CreateIntensityAttr(1000.0)
    light.AddRotateXYZOp().Set(Gf.Vec3f(35.0, 20.0, 0.0))     # 비스듬히 비춰 면마다 밝기가 다르게
    UsdLux.DomeLight.Define(stage, "/World/SkyLight").CreateIntensityAttr(500.0)   # 그늘진 면도 보이게 하는 주변광

    scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0.0, 0.0, -1.0))
    scene.CreateGravityMagnitudeAttr(9.81)

    ground = UsdGeom.Plane.Define(stage, "/World/GroundPlane")   # 충돌면: 무한히 넓은 평면
    ground.CreateAxisAttr(UsdGeom.Tokens.z)
    ground.CreatePurposeAttr(UsdGeom.Tokens.guide)
    UsdPhysics.CollisionAPI.Apply(ground.GetPrim())
    slab = UsdGeom.Cube.Define(stage, "/World/GroundVisual")     # 눈에 보이는 판 (충돌과는 무관)
    slab.CreateSizeAttr(1.0)
    slab.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, -0.005))
    slab.AddScaleOp().Set(Gf.Vec3f(50.0, 50.0, 0.01))
    slab.CreateDisplayColorAttr([Gf.Vec3f(0.45, 0.47, 0.5)])
    return stage


def set_pose(prim, translate=(0.0, 0.0, 0.0), rotate_xyz=(0.0, 0.0, 0.0), scale=(1.0, 1.0, 1.0)):
    """Property 창의 Transform 과 같은 세 값: Translate · Orient(각도) · Scale."""
    xform = UsdGeom.Xformable(prim)
    xform.ClearXformOpOrder()
    for name in prim.GetPropertyNames():          # 메뉴 명령이 미리 넣어 둔 값이 있으면 지우고 새로 넣는다
        if name.startswith("xformOp:"):
            prim.RemoveProperty(name)
    xform.AddTranslateOp().Set(Gf.Vec3d(*translate))
    xform.AddRotateXYZOp().Set(Gf.Vec3f(*rotate_xyz))
    xform.AddScaleOp().Set(Gf.Vec3f(*scale))


def pose_of(prim):
    """월드 좌표의 (x, y, z, yaw[도]). yaw 는 위에서 볼 때 반시계 방향(왼쪽으로 돌기)이 + 입니다."""
    m = omni.usd.get_world_transform_matrix(prim)
    p = m.ExtractTranslation()
    yaw = math.degrees(math.atan2(m[0][1], m[0][0]))
    return p[0], p[1], p[2], yaw


def set_view(eye, target):
    """창을 띄웠을 때 뷰포트 시점: eye 에서 target 을 본다."""
    from isaacsim.core.utils.viewports import set_camera_view
    set_camera_view(eye=list(eye), target=list(target))


def save_stage(stage, path):
    path = os.path.abspath(os.path.expanduser(path))
    os.makedirs(os.path.dirname(path), exist_ok=True)
    stage.GetRootLayer().Export(path)
    print("장면 저장:", path, flush=True)


class Sim:
    """Play 를 누르고 물리 시간으로 정해진 만큼 진행한다."""

    def __init__(self, app):
        self.app = app
        self.time = 0.0
        self.timeline = omni.timeline.get_timeline_interface()
        self._sub = omni.physx.get_physx_interface().subscribe_physics_step_events(self._on_step)

    def _on_step(self, dt):
        self.time += dt

    def play(self):
        for _ in range(10):
            self.app.update()
        self.timeline.play()

    def run(self, seconds, every=None, report=None):
        """seconds 만큼 진행. every 초마다 report(시간) 을 부른다."""
        end = self.time + seconds
        next_report = self.time + every if every else None
        while self.time < end - 1e-6:
            self.app.update()
            if next_report is not None and self.time >= next_report - 1e-6:
                report(self.time)
                next_report += every

    def stop(self):
        self.timeline.stop()
        self._sub = None

    def wait_until_closed(self):
        """창을 띄웠을 때: 창을 닫을 때까지 계속 진행한다."""
        while self.app.is_running():
            self.app.update()
