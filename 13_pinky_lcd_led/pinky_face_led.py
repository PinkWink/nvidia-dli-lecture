"""강의 13: Pinky Pro 의 화면(LCD)에 표정을 띄우고, LED 바를 빛나게 합니다.

    ~/isaacsim/python.sh 13_pinky_lcd_led/pinky_face_led.py                           # 기본 표정 + 초록 LED, 사진 저장
    ~/isaacsim/python.sh 13_pinky_lcd_led/pinky_face_led.py --face happy --led 1 0 0  # 표정 happy, 빨간 LED
    ~/isaacsim/python.sh 13_pinky_lcd_led/pinky_face_led.py --led 0 0 0               # LED 끄기
    ~/isaacsim/python.sh 13_pinky_lcd_led/pinky_face_led.py --show                    # 표정 8개와 LED 색을 차례로 바꿔 본다 (창)
    ~/isaacsim/python.sh 13_pinky_lcd_led/pinky_face_led.py --keyboard                # 창을 띄우고 W A S D 로 조종

URDF 의 화면(screen_mount) · LED 바(robot_lamp)는 모양만 있습니다. 빛과 그림은 재질(Material)로 표현합니다.
    화면: screen_mount 아래에 Create > Mesh > Plane → Create > Materials > OmniPBR (Albedo Map · Emissive Color Map = 표정 PNG)
    LED : Create > Materials > OmniPBR (Enable Emission · Emissive Color) → robot_lamp/visuals 에 Stronger than Descendants 로 연결
"""
import argparse
import os
import sys

from isaacsim import SimulationApp

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--face", default="basic", help="표정 이름 (work/pinky_faces/이름.png)")
    parser.add_argument("--led", type=float, nargs=3, default=(0.0, 1.0, 0.2), metavar=("R", "G", "B"),
                        help="LED 색 0 ~ 1 (0 0 0 이면 끔)")
    parser.add_argument("--image", default="work/pinky_face.png", help="로봇 사진을 저장할 경로")
    parser.add_argument("--show", action="store_true", help="창을 띄우고 표정 · LED 색을 차례로 바꾼다")
    parser.add_argument("--keyboard", action="store_true", help="창을 띄우고 W A S D 로 조종한다")
    parser.add_argument("--gui", action="store_true", help="창을 띄운다 (끝나도 닫을 때까지 유지)")
    parser.add_argument("--save", metavar="USD", help="Play 전 장면을 이 경로에 저장한다")
    args = parser.parse_args()
    app = SimulationApp({"headless": not (args.gui or args.keyboard or args.show)})

import numpy as np
import omni.kit.commands
import omni.usd
from pxr import Gf, Sdf, UsdGeom, UsdShade

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
sys.path.insert(0, os.path.join(HERE, "..", "11_pinky_import"))
from common.sim import Sim, new_scene, save_stage, set_pose, set_view
from import_pinky import DEFAULT_URDF, ROBOT, add_differential_controller, add_keyboard_control, find_urdf, import_pinky

FACES = os.path.abspath(os.path.join(HERE, "..", "work", "pinky_faces"))
SCREEN = ROBOT + "/screen_mount"
LCD = SCREEN + "/LCD"
LAMP_VISUAL = ROBOT + "/robot_lamp/visuals"
LCD_MATERIAL = "/World/Looks/LCD_Material"
LED_MATERIAL = "/World/Looks/LED_Material"
# 화면 부품의 앞 유리판은 screen_mount 좌표로 x = 0.017, 가로 57 mm × 세로 41 mm. 그 바로 앞에 4 : 3 평면을 둔다.
LCD_POSITION = (0.0173, 0.0, -0.0035)
LCD_SIZE = (0.052, 0.039)             # 가로 × 세로 [m]
LCD_BRIGHTNESS = 40.0
LED_BRIGHTNESS = 2000.0


def face_path(name):
    path = os.path.join(FACES, name + ".png")
    if not os.path.exists(path):
        names = sorted(f[:-4] for f in os.listdir(FACES)) if os.path.isdir(FACES) else []
        print(f"표정 그림이 없습니다: {path}")
        print("쓸 수 있는 표정:", ", ".join(names) if names else "없음 → bash 11_pinky_import/prepare_pinky.sh 를 먼저 실행")
        sys.exit(1)
    return path


def new_material(stage, path):
    """Create > Materials > OmniPBR 과 같은 재질을 만들고 (재질, 셰이더) 를 돌려준다."""
    omni.kit.commands.execute("CreateMdlMaterialPrim", mtl_url="OmniPBR.mdl", mtl_name="OmniPBR", mtl_path=path)
    return UsdShade.Material(stage.GetPrimAtPath(path)), UsdShade.Shader(stage.GetPrimAtPath(path + "/Shader"))


def add_screen(stage, image):
    """화면: 그림을 입힐 수 있는 평면을 유리판 앞에 붙이고, 스스로 빛나는(Emission) 재질에 표정 그림을 넣는다."""
    omni.kit.commands.execute("CreateMeshPrimWithDefaultXform", prim_type="Plane", prim_path=LCD,
                              select_new_prim=False, prepend_default_prim=False)
    # 평면은 처음에 바닥처럼 누워 있다 (법선 +Z). 세워서 앞(+X)을 보게 돌린다.
    # 여기서는 X 로 90° → Z 로 90° 순서(USD 의 rotateXYZ)로 적었고, GUI 의 Orient 칸에는 같은 회전이 (90, 90, 0) 으로 보인다.
    set_pose(stage.GetPrimAtPath(LCD), translate=LCD_POSITION, rotate_xyz=(90.0, 0.0, 90.0), scale=(*LCD_SIZE, 1.0))
    material, shader = new_material(stage, LCD_MATERIAL)
    shader.CreateInput("enable_emission", Sdf.ValueTypeNames.Bool).Set(True)
    shader.CreateInput("emissive_color", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(1.0))
    shader.CreateInput("emissive_intensity", Sdf.ValueTypeNames.Float).Set(LCD_BRIGHTNESS)
    UsdShade.MaterialBindingAPI.Apply(stage.GetPrimAtPath(LCD)).Bind(material)
    set_face(stage, image)


def set_face(stage, image):
    """표정 바꾸기: 재질의 Albedo Map 과 Emissive Color Map 을 다른 그림으로."""
    shader = UsdShade.Shader(stage.GetPrimAtPath(LCD_MATERIAL + "/Shader"))
    shader.CreateInput("diffuse_texture", Sdf.ValueTypeNames.Asset).Set(image)
    shader.CreateInput("emissive_color_texture", Sdf.ValueTypeNames.Asset).Set(image)


def add_led(stage, color):
    """LED 바: 빛나는 재질을 만들어 robot_lamp 의 보이는 모양 전체에 덮어씌운다."""
    material, _ = new_material(stage, LED_MATERIAL)
    # visuals 는 인스턴스라 안쪽 재질을 직접 고칠 수 없다. 바깥에서 "자식보다 강하게(Stronger than Descendants)" 묶는다.
    UsdShade.MaterialBindingAPI.Apply(stage.GetPrimAtPath(LAMP_VISUAL)).Bind(material, UsdShade.Tokens.strongerThanDescendants)
    set_led(stage, color)


def set_led(stage, color):
    """LED 색 바꾸기. (0, 0, 0) 이면 끈다."""
    shader = UsdShade.Shader(stage.GetPrimAtPath(LED_MATERIAL + "/Shader"))
    on = max(color) > 0.0
    shader.CreateInput("diffuse_color_constant", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color) if on else Gf.Vec3f(0.9))
    shader.CreateInput("enable_emission", Sdf.ValueTypeNames.Bool).Set(on)
    shader.CreateInput("emissive_color", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    shader.CreateInput("emissive_intensity", Sdf.ValueTypeNames.Float).Set(LED_BRIGHTNESS)


def save_photo(app, stage, path, eye=(0.30, 0.16, 0.17), target=(0.0, 0.0, 0.07), size=(960, 720)):
    """로봇을 앞 왼쪽 위에서 본 사진을 PNG 로 저장한다."""
    import omni.replicator.core as rep
    from isaacsim.core.utils.viewports import set_camera_view
    from PIL import Image
    camera = UsdGeom.Camera.Define(stage, "/World/PhotoCamera")
    camera.CreateFocalLengthAttr(35.0)
    camera.CreateClippingRangeAttr(Gf.Vec2f(0.005, 100.0))
    render_product = rep.create.render_product(str(camera.GetPath()), size)
    set_camera_view(eye=list(eye), target=list(target), camera_prim_path=str(camera.GetPath()))
    annotator = rep.AnnotatorRegistry.get_annotator("rgb")
    annotator.attach([render_product])
    for _ in range(80):
        app.update()
    path = os.path.abspath(os.path.expanduser(path))
    os.makedirs(os.path.dirname(path), exist_ok=True)
    Image.fromarray(np.asarray(annotator.get_data())[:, :, :3]).save(path)
    annotator.detach()
    return path


if __name__ == "__main__":
    image = face_path(args.face)
    stage = new_scene()
    import_pinky(stage, find_urdf(DEFAULT_URDF))
    add_differential_controller()
    if args.keyboard:
        add_keyboard_control()
    add_screen(stage, image)
    add_led(stage, args.led)
    if args.save:
        save_stage(stage, args.save)
    if args.gui or args.keyboard or args.show:
        set_view(eye=(0.30, 0.16, 0.17), target=(0.0, 0.0, 0.07))

    led = "끔" if max(args.led) <= 0 else "({:g}, {:g}, {:g})".format(*args.led)
    print(f"화면: {LCD} — 가로 {LCD_SIZE[0] * 1000:.0f} mm × 세로 {LCD_SIZE[1] * 1000:.0f} mm, 표정 {args.face}", flush=True)
    print(f"LED : {LAMP_VISUAL} ← {LED_MATERIAL}, 색 {led}", flush=True)

    sim = Sim(app)
    sim.play()
    sim.run(1.0)
    if args.keyboard:
        print("뷰포트를 한 번 클릭한 뒤 W(앞) S(뒤) A(왼쪽) D(오른쪽) 로 조종합니다. 창을 닫으면 끝납니다.", flush=True)
        sim.wait_until_closed()
    elif args.show:
        faces = sorted(f[:-4] for f in os.listdir(FACES) if f.endswith(".png"))
        colors = [(1, 0, 0), (1, 0.5, 0), (1, 1, 0), (0, 1, 0), (0, 1, 1), (0, 0, 1), (1, 0, 1), (1, 1, 1)]
        print("표정과 LED 색을 2초마다 바꿉니다. 창을 닫으면 끝납니다.", flush=True)
        i = 0
        while app.is_running():
            set_face(stage, os.path.join(FACES, faces[i % len(faces)] + ".png"))
            set_led(stage, colors[i % len(colors)])
            print(f"  표정 {faces[i % len(faces)]:<9} LED {colors[i % len(colors)]}", flush=True)
            end = sim.time + 2.0
            while app.is_running() and sim.time < end:
                app.update()
            i += 1
    else:
        print("사진 저장:", save_photo(app, stage, args.image), flush=True)
        print("결과: 화면에 표정을 띄우고 LED 바를 켰습니다." if max(args.led) > 0 else "결과: 화면에 표정을 띄우고 LED 바는 껐습니다.",
              flush=True)
        if args.gui:
            sim.wait_until_closed()
    sim.stop()
    app.close()
