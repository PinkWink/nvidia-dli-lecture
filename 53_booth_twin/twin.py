"""강의 53: 부스의 디지털트윈에서 쓰는 계산들. 로봇이 다닌 길 · 물체와의 거리 · 상자 더미.

Isaac Sim 안에서(pxr 이 있는 곳에서) 불러 쓴다: review.py 가 이 파일의 함수를 차례로 부른다.
"""
import json
import os
import random

import numpy as np
from pxr import Gf, Usd, UsdGeom, UsdPhysics

HERE = os.path.dirname(os.path.abspath(__file__))
PATHS = os.path.join(HERE, "paths", "pl2m_paths.json")            # 로봇이 실제로 다닌 길 (강의 44 · 46 의 기록)
ROBOT_HALF_WIDTH = 0.32                                           # 로봇이 차지하는 폭의 절반 [m] (강의 44 의 Nav2 설정: footprint 의 y)
ROBOT_HEIGHT = 1.30                                               # 이 높이보다 위에 있는 것은 로봇과 닿지 않는다 [m]
BOX = (0.40, 0.30, 0.25)                                          # 쌓을 상자 하나의 크기 (x, y, z) [m]
BOX_MASS = 3.0                                                    # 상자 하나의 무게 [kg]
STACK = "/World/Proposal/stack"                                   # 검토할 상자 더미가 놓이는 곳


def load_paths(file=PATHS):
    """기록한 길들: {이름: (N, 2) 배열}. 부스 좌표 x y [m]."""
    with open(file) as source:
        return {name: np.array(points) for name, points in json.load(source)["paths"].items()}


def draw_paths(stage, paths, color=(0.1, 0.45, 0.95), where="/World/Paths"):
    """길을 바닥 위의 선으로 그린다. 눈으로 보려는 것이고 물리와는 상관없다."""
    stage.RemovePrim(where)
    UsdGeom.Scope.Define(stage, where)
    for name, points in paths.items():
        curve = UsdGeom.BasisCurves.Define(stage, f"{where}/{name}")
        curve.CreateTypeAttr("linear")
        curve.CreatePointsAttr([Gf.Vec3f(float(x), float(y), 0.03) for x, y in points])
        curve.CreateCurveVertexCountsAttr([len(points)])
        curve.CreateWidthsAttr([0.05])
        curve.CreateDisplayColorAttr([Gf.Vec3f(*color)])


def footprints(stage, root, top=ROBOT_HEIGHT):
    """root 아래의 충돌체마다 바닥에 드리운 네모: [(이름, (x 최소, y 최소), (x 최대, y 최대))]. 로봇의 키보다 위에 뜬 것은 뺀다.

    이름은 root 바로 아래의 것으로 묶는다 (가구 하나가 충돌체 여러 개로 돼 있다).
    """
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.guide])
    found = []
    for prim in Usd.PrimRange(stage.GetPrimAtPath(root)):
        if not prim.HasAPI(UsdPhysics.CollisionAPI):
            continue
        box = cache.ComputeWorldBound(prim).ComputeAlignedRange()
        low, high = box.GetMin(), box.GetMax()
        if box.IsEmpty() or low[2] > top or high[2] < 0.02:               # 머리 위에 뜬 것, 바닥에 깔린 것
            continue
        name = str(prim.GetPath())[len(root) + 1:]
        found.append((name, (low[0], low[1]), (high[0], high[1])))
    return found


def clearance(paths, shapes):
    """물체마다, 길에서 가장 가까운 거리 [m]: {이름: (거리, 그 길의 이름)}. 거리는 길의 가운데 선에서 물체의 네모까지다."""
    nearest = {}
    for name, low, high in shapes:
        for path, points in paths.items():
            dx = np.maximum(np.maximum(low[0] - points[:, 0], 0.0), points[:, 0] - high[0])
            dy = np.maximum(np.maximum(low[1] - points[:, 1], 0.0), points[:, 1] - high[1])
            distance = float(np.min(np.hypot(dx, dy)))                    # 점에서 네모까지의 거리
            if name not in nearest or distance < nearest[name][0]:
                nearest[name] = (distance, path)
    return nearest


def nearest_shape(shapes, others):
    """shapes 의 네모들과 others 의 네모들 사이에서 가장 가까운 것: (거리 [m], others 쪽의 이름). 겹치면 0 이다."""
    best = (float("inf"), None)
    for _, low, high in shapes:
        for name, other_low, other_high in others:
            dx = max(other_low[0] - high[0], low[0] - other_high[0], 0.0)
            dy = max(other_low[1] - high[1], low[1] - other_high[1], 0.0)
            best = min(best, (float(np.hypot(dx, dy)), name))
    return best


def add_stack(stage, x, y, layers, jitter=0.0, seed=0, where=STACK):
    """(x, y) 에 상자를 layers 단 쌓는다. jitter [m] 를 주면 층마다 그만큼까지 옆으로 어긋나고 ±15° 까지 돈다 (대충 쌓은 것).

    지금 편집하는 레이어에 적힌다. 검토 안을 따로 둔 레이어에 넣으려면 부르기 전에 편집 대상을 바꿔 둔다 (review.py).
    """
    stage.RemovePrim(where)
    UsdGeom.Xform.Define(stage, where)
    rng = random.Random(seed)
    for layer in range(layers):
        box = UsdGeom.Cube.Define(stage, f"{where}/box_{layer + 1}")
        box.CreateSizeAttr(1.0)
        box.CreateDisplayColorAttr([Gf.Vec3f(0.80, 0.62, 0.42)])
        shift = (rng.uniform(-jitter, jitter), rng.uniform(-jitter, jitter)) if layer else (0.0, 0.0)
        box.AddTranslateOp().Set(Gf.Vec3d(x + shift[0], y + shift[1], BOX[2] * (layer + 0.5) + 0.002 * (layer + 1)))
        box.AddRotateZOp().Set(rng.uniform(-15.0, 15.0) if jitter and layer else 0.0)
        box.AddScaleOp().Set(Gf.Vec3f(*BOX))
        UsdPhysics.CollisionAPI.Apply(box.GetPrim())
        UsdPhysics.RigidBodyAPI.Apply(box.GetPrim())
        UsdPhysics.MassAPI.Apply(box.GetPrim()).CreateMassAttr(BOX_MASS)
    return where


def box_states(stage, where=STACK):
    """상자마다 (이름, x, y, 가운데의 높이)."""
    states = []
    for box in stage.GetPrimAtPath(where).GetChildren():
        x, y, z = UsdGeom.Xformable(box).ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()
        states.append((box.GetName(), x, y, z))
    return states


def fallen(before, after, drop=0.05, slide=0.10):
    """쌓아 둔 자리에서 벗어난 상자들: 5 cm 넘게 내려갔거나 10 cm 넘게 옆으로 간 것. [(이름, 처음 높이, 나중 높이, 옆으로 간 거리)]"""
    moved = []
    for (name, x0, y0, z0), (_, x1, y1, z1) in zip(before, after):
        side = float(np.hypot(x1 - x0, y1 - y0))
        if z0 - z1 > drop or side > slide:
            moved.append((name, z0, z1, side))
    return moved
