"""강의 50: 부스 장면을 USD 파일 여러 개로 나눠 만듭니다. 파일끼리는 참조 · payload · 서브레이어로 잇습니다.

    ~/isaacsim/python.sh 38_pl2m_booth/make_booth.py --no-photo          # 먼저: 부스를 work/booth/booth.usd 로 (강의 38)
    ~/isaacsim/python.sh 37_pl2m_import/import_pl2m.py --save work/pl2m/pl2m_scene.usd --seconds 1     # 먼저: 로봇을 USD 로 (강의 37)
    ~/isaacsim/python.sh 50_usd_layers/make_layers.py                    # work/booth_layers/ 에 파일 넷을 만든다
    ~/isaacsim/python.sh 50_usd_layers/make_layers.py --show cup.usda    # 만든 파일의 내용을 출력 (글자로 된 USD)

만드는 파일 (work/booth_layers/)
    cup.usda          컵 하나. 모양 · 물리 · 색 Variant (pink · blue · yellow) 가 든 부품
    layout.usda       배치: 부스 · 로봇(payload 로 불러옴) + 컵 셋(cup.usda 를 참조, 색과 자리만 정함)
    rearrange.usda    바꿔 보는 안: 컵의 색과 자리, 음료대 옆 사람을 덮어씀. 원본 파일은 건드리지 않음
    scene.usda        여는 파일: 위 둘을 서브레이어로 쌓음 (rearrange 가 위 = 더 셈)
"""
import argparse
import os

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, ".."))
BOOTH = os.path.join(REPO, "work", "booth", "booth.usd")           # 강의 38 이 저장한 부스
ROBOT = os.path.join(REPO, "work", "pl2m", "pl2m_scene.usd")       # 강의 37 이 저장한 장면. 이 가운데 로봇(/World/pl2m)만 가져온다
SPAWN = (5.3, -3.2, 0.05)                                           # 로봇을 놓을 자리 (강의 38)
OUT = os.path.join(REPO, "work", "booth_layers")
COLORS = {"pink": (0.87, 0.42, 0.65), "blue": (0.28, 0.39, 0.68), "yellow": (0.95, 0.77, 0.23)}       # 화면에 보일 색 (강의 38 의 컵과 같다)
CUPS = {"cup_pink": ("pink", (7.50, -1.72, 0.795)), "cup_blue": ("blue", (7.65, -1.72, 0.795)), "cup_yellow": ("yellow", (7.80, -1.72, 0.795))}
# 바꿔 보는 안: 컵을 음료대의 오른쪽에 모으고, 가운데 컵을 노란색으로
MOVED = {"cup_pink": ("pink", (7.74, -1.72, 0.795)), "cup_blue": ("yellow", (7.82, -1.72, 0.795)), "cup_yellow": ("yellow", (7.90, -1.72, 0.795))}
REMOVED = "/World/Site/Booth/People/staff_5"                             # 바꿔 보는 안에서 치울 사람 모형 (음료대 오른쪽에 선 사람)

parser = argparse.ArgumentParser()
parser.add_argument("--show", metavar="FILE", help="만든 뒤 그 파일의 내용을 출력한다")
args = parser.parse_args()

from isaacsim import SimulationApp
app = SimulationApp({"headless": True})

from pxr import Gf, Kind, Sdf, Usd, UsdGeom, UsdPhysics


def new_stage(name):
    """미터 · Z 가 위인 빈 파일을 만든다."""
    path = os.path.join(OUT, name)
    if os.path.exists(path):
        os.remove(path)
    stage = Usd.Stage.CreateNew(path)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    return stage


def make_cup():
    """cup.usda: 컵 하나. 다른 파일이 이것을 참조해 쓴다."""
    stage = new_stage("cup.usda")
    cup = UsdGeom.Xform.Define(stage, "/Cup")
    stage.SetDefaultPrim(cup.GetPrim())                             # 참조할 때 따로 말하지 않으면 이 Prim 을 가져간다
    Usd.ModelAPI(cup.GetPrim()).SetKind(Kind.Tokens.component)      # 더 쪼개지 않는 부품이라는 표시
    UsdPhysics.RigidBodyAPI.Apply(cup.GetPrim())
    UsdPhysics.MassAPI.Apply(cup.GetPrim()).CreateMassAttr(0.2)
    body = UsdGeom.Cylinder.Define(stage, "/Cup/body")
    body.CreateAxisAttr("Z")
    body.CreateRadiusAttr(0.025)
    body.CreateHeightAttr(0.09)
    UsdPhysics.CollisionAPI.Apply(body.GetPrim())
    colors = cup.GetPrim().GetVariantSets().AddVariantSet("color")  # Variant: 한 부품 안에 든 여러 모습. 쓰는 쪽이 하나를 고른다
    for name, color in COLORS.items():
        colors.AddVariant(name)
        colors.SetVariantSelection(name)
        with colors.GetVariantEditContext():                        # 이 안에서 적는 것은 그 Variant 를 골랐을 때만 보인다
            body.CreateDisplayColorAttr([Gf.Vec3f(*[c ** 2.2 for c in color])])              # 화면의 색(sRGB) → 렌더러가 쓰는 값
    colors.SetVariantSelection("pink")                              # 고르지 않았을 때의 기본
    stage.Save()


def make_layout():
    """layout.usda: 부스와 컵을 불러와 놓는다. 여기에는 '무엇을 어디에' 만 적힌다."""
    stage = new_stage("layout.usda")
    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())
    site = UsdGeom.Xform.Define(stage, "/World/Site")                             # 부스 파일의 /World (부스 · 조명 · 바닥 · 재질) 가 통째로 이 아래에 온다
    site.GetPrim().GetPayloads().AddPayload(os.path.relpath(BOOTH, OUT))          # payload: 참조와 같지만, 열 때 불러오지 않을 수도 있다
    robot = UsdGeom.Xform.Define(stage, "/World/pl2m")
    robot.GetPrim().GetPayloads().AddPayload(os.path.relpath(ROBOT, OUT), "/World/pl2m")     # 파일의 한 부분만 가져올 수도 있다: 그 파일 안의 Prim 을 짚는다
    robot.GetPrim().CreateAttribute("xformOp:translate", Sdf.ValueTypeNames.Double3, custom=False).Set(Gf.Vec3d(*SPAWN))   # 로봇 파일에 이미 있는 값을 덮어쓴다
    UsdGeom.Scope.Define(stage, "/World/Cups")
    for name, (color, position) in CUPS.items():
        cup = UsdGeom.Xform.Define(stage, f"/World/Cups/{name}")
        cup.GetPrim().GetReferences().AddReference("./cup.usda")                  # 참조: 다른 파일의 Prim 을 이 자리에 가져온다
        cup.GetPrim().GetVariantSets().GetVariantSet("color").SetVariantSelection(color)
        cup.AddTranslateOp().Set(Gf.Vec3d(*position))
    stage.Save()


def make_rearrange():
    """rearrange.usda: 덮어쓸 것만 적는다 (over). 이 파일을 빼면 원래 배치로 돌아간다."""
    stage = new_stage("rearrange.usda")
    for name, (color, position) in MOVED.items():
        cup = stage.OverridePrim(f"/World/Cups/{name}")                           # over: 새로 만들지 않고, 이미 있는 Prim 의 값을 덮어쓴다
        cup.GetVariantSets().GetVariantSet("color").SetVariantSelection(color)
        cup.CreateAttribute("xformOp:translate", Sdf.ValueTypeNames.Double3, custom=False).Set(Gf.Vec3d(*position))
    stage.OverridePrim(REMOVED).SetActive(False)                                  # 지우지 않고 끈다
    stage.Save()


def make_scene():
    """scene.usda: 서브레이어 둘을 쌓는다. 앞에 적은 것이 위(더 셈)다."""
    stage = new_stage("scene.usda")
    stage.GetRootLayer().subLayerPaths = ["./rearrange.usda", "./layout.usda"]
    stage.GetRootLayer().defaultPrim = "World"
    stage.Save()


for needed, command in ((BOOTH, "38_pl2m_booth/make_booth.py --no-photo"), (ROBOT, "37_pl2m_import/import_pl2m.py --save work/pl2m/pl2m_scene.usd --seconds 1")):
    if not os.path.exists(needed):
        raise SystemExit(f"{needed} 이 없음: 먼저 ~/isaacsim/python.sh {command} 를 실행할 것")
os.makedirs(OUT, exist_ok=True)
for make in (make_cup, make_layout, make_rearrange, make_scene):
    make()
print(f"만든 곳: {OUT}", flush=True)
for name in ("cup.usda", "layout.usda", "rearrange.usda", "scene.usda"):
    print(f"  {name:<20}{os.path.getsize(os.path.join(OUT, name)):>10} 바이트", flush=True)
for label, path in (("(불러오는 부스)", BOOTH), ("(불러오는 로봇)", ROBOT)):
    print(f"  {label:<13}{os.path.getsize(path):>10} 바이트  {os.path.relpath(path, OUT)}", flush=True)
if args.show:
    print(f"\n--- {args.show}\n" + open(os.path.join(OUT, args.show)).read(), flush=True)
app.close()
