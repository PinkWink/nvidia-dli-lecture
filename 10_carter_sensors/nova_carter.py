"""강의 10: 준비된 로봇 Nova Carter 를 불러와 어떤 센서 · 관절이 들어 있는지 살펴봅니다.

    ~/isaacsim/python.sh 10_carter_sensors/nova_carter.py          # 센서 · 관절 목록 출력
    ~/isaacsim/python.sh 10_carter_sensors/nova_carter.py --gui    # 창으로 보기

GUI 의 Create > Robots > Nova Carter with Sensors 와 같은 에셋을 장면에 참조로 넣습니다.
처음 실행할 때는 에셋을 인터넷에서 내려받느라 몇 분 걸릴 수 있습니다.
"""
import argparse
import os
import sys

from isaacsim import SimulationApp

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--gui", action="store_true", help="창을 띄운다 (끝나도 닫을 때까지 유지)")
    args = parser.parse_args()
    app = SimulationApp({"headless": not args.gui})

from isaacsim.storage.native import get_assets_root_path
from pxr import UsdGeom, UsdPhysics

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
from common.sim import Sim, new_scene, set_view

ROBOT = "/World/Nova_Carter"
ASSET = "/Isaac/Robots/NVIDIA/NovaCarter/nova_carter.usd"      # Create > Robots > Nova Carter with Sensors
BODY = ROBOT + "/chassis_link"
SENSORS = BODY + "/sensors"


def add_nova_carter(stage):
    """에셋을 참조로 넣는다. 파일을 복사하지 않고 "이 파일을 여기에 불러와라" 라고 적는 방식."""
    root = get_assets_root_path()
    if root is None:
        print("에셋 서버에 연결하지 못했습니다. 인터넷 연결을 확인하세요.", flush=True)
        sys.exit(1)
    prim = stage.DefinePrim(ROBOT, "Xform")
    prim.GetReferences().AddReference(root + ASSET)
    return prim, root + ASSET


def kind_of(prim):
    """센서 Prim 을 사람이 읽는 종류로."""
    if prim.IsA(UsdGeom.Camera):
        return "카메라"
    name = prim.GetTypeName()
    if name == "OmniLidar":
        return "라이다 (RTX)"
    if name == "IsaacImuSensor":
        return "IMU"
    return None


def report(stage):
    print(f"{'센서 묶음':<16} {'종류':<12} {'개수':>4}  Prim 이름", flush=True)
    total = {}
    for group in stage.GetPrimAtPath(SENSORS).GetChildren():
        found = {}
        for prim in walk(group):
            kind = kind_of(prim)
            if kind:
                found.setdefault(kind, []).append(prim.GetName())
        for kind, names in found.items():
            total[kind] = total.get(kind, 0) + len(names)
            print(f"{group.GetName():<16} {kind:<12} {len(names):>4}  {', '.join(names)}", flush=True)
    print("합계: " + ", ".join(f"{kind} {count}개" for kind, count in total.items()), flush=True)
    return total


def report_joints(stage):
    """회전 관절과 구동 세기. Damping 이 큰 관절이 모터로 돌리는 바퀴입니다."""
    print(f"{'회전 관절':<22} 구동 Damping", flush=True)
    for prim in walk(stage.GetPrimAtPath(ROBOT)):
        if prim.IsA(UsdPhysics.RevoluteJoint):
            has_drive = prim.HasAPI(UsdPhysics.DriveAPI, "angular")
            damping = UsdPhysics.DriveAPI(prim, "angular").GetDampingAttr().Get() if has_drive else 0.0
            print(f"{prim.GetName():<22} {damping:g}", flush=True)


def walk(prim):
    """prim 과 그 아래 모든 Prim (인스턴스 안쪽은 들어가지 않는다)."""
    yield prim
    for child in prim.GetChildren():
        yield from walk(child)


if __name__ == "__main__":
    stage = new_scene()
    prim, url = add_nova_carter(stage)
    print("에셋:", url, flush=True)
    for _ in range(60):                      # 참조한 파일을 다 읽을 때까지
        app.update()
    total = report(stage)
    report_joints(stage)
    if args.gui:
        set_view(eye=(2.0, -2.0, 1.5), target=(0.0, 0.0, 0.3))

    sim = Sim(app)
    sim.play()
    sim.run(2.0)
    print("결과: Nova Carter 에는 " + " · ".join(f"{kind} {count}개" for kind, count in total.items()) + " 가 달려 있습니다.",
          flush=True)
    if args.gui:
        sim.wait_until_closed()
    sim.stop()
    app.close()
