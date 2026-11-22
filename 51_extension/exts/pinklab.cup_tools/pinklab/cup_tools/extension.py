"""강의 51: 음료대 위의 컵을 버튼으로 다시 놓는 창. Isaac Sim 의 익스텐션이다.

    켜질 때   on_startup   창을 만들고 Window 메뉴에 항목을 넣는다
    꺼질 때   on_shutdown  만든 것을 모두 치운다 (치우지 않으면 다시 켤 때 창과 메뉴가 겹쳐 쌓인다)

버튼이 하는 일은 모두 명령(omni.kit.commands)으로 실행한다. 명령은 Ctrl+Z 로 되돌릴 수 있다.
"""
import random

import omni.ext
import omni.kit.commands
import omni.kit.menu.utils
import omni.ui as ui
import omni.usd
from omni.kit.menu.utils import MenuItemDescription
from pxr import Gf, Sdf

TITLE = "PinkLAB Cup Tools"                             # 창의 글자는 영어로 쓴다. Isaac Sim 의 기본 글꼴에는 한글이 없어 ?? 로 나온다
CUPS = "/World/Cups"                                    # 컵들이 든 곳 (강의 38 · 47 · 50 의 장면이 모두 같다)
HOME = {"cup_pink": (7.50, -1.72), "cup_blue": (7.65, -1.72), "cup_yellow": (7.80, -1.72)}       # 원래 자리 (x, y)
FRONT = (7.39, 7.91, -1.75, -1.70)                      # 섞을 때 쓰는 범위: x 최소 · 최대, y 최소 · 최대. 팔이 닿는 앞줄이다 (강의 48)
GAP = 0.09                                              # 컵끼리 이만큼은 떨어지게 [m]
COLORS = ["pink", "blue", "yellow"]
INSTANCE = None                                         # 켜져 있는 동안의 익스텐션 (시험 스크립트가 버튼의 함수를 부를 때 쓴다)


def cups(stage):
    """장면에 있는 컵 Prim 들."""
    scope = stage.GetPrimAtPath(CUPS) if stage else None
    return list(scope.GetChildren()) if scope else []


def move_cup(prim, x, y):
    """컵을 (x, y) 로 옮긴다. 높이는 그대로 둔다. 명령으로 실행하므로 되돌릴 수 있다."""
    place = prim.GetAttribute("xformOp:translate")
    before = place.Get()
    omni.kit.commands.execute("ChangeProperty", prop_path=Sdf.Path(place.GetPath()), value=Gf.Vec3d(x, y, before[2]), prev=before)


class CupToolsExtension(omni.ext.IExt):
    def on_startup(self, ext_id):
        global INSTANCE
        INSTANCE = self
        self._window = ui.Window(TITLE, width=320, height=250)
        with self._window.frame:
            with ui.VStack(spacing=6, height=0):
                self._status = ui.Label("", height=44, word_wrap=True)
                with ui.HStack(spacing=6, height=30):
                    ui.Button("Shuffle", clicked_fn=self.shuffle, tooltip="Put the cups anywhere in the front row")       # 컵을 음료대의 앞줄 아무 데나
                    ui.Button("Home", clicked_fn=self.go_home)                # 원래 자리로
                with ui.HStack(spacing=6, height=30):
                    ui.Label("All cups:", width=70)                         # 모두 이 색으로
                    for color in COLORS:
                        ui.Button(color, clicked_fn=lambda color=color: self.paint(color))
                ui.Button("Person on / off", height=30, clicked_fn=self.toggle_person)        # 음료대 옆 사람 켜기 · 끄기
        self._menu = [MenuItemDescription(name=TITLE, onclick_fn=self.show)]
        omni.kit.menu.utils.add_menu_items(self._menu, "Window")           # Window 메뉴에서 창을 다시 띄울 수 있게
        self._seed = 0
        self.report("ready")

    def on_shutdown(self):
        global INSTANCE
        INSTANCE = None
        omni.kit.menu.utils.remove_menu_items(self._menu, "Window")
        self._window.destroy()
        self._window = None

    def show(self):
        self._window.visible = True

    # --- 버튼이 하는 일 ---------------------------------------------------
    def report(self, what):
        found = cups(omni.usd.get_context().get_stage())
        places = ",  ".join(f"{cup.GetName().replace('cup_', '')} ({cup.GetAttribute('xformOp:translate').Get()[0]:.2f}, {cup.GetAttribute('xformOp:translate').Get()[1]:.2f})" for cup in found)
        self.status = f"{what}\n{places}" if found else f"no cups at {CUPS}: open a scene with cups"
        self._status.text = self.status

    def shuffle(self):
        found = cups(omni.usd.get_context().get_stage())
        self._seed += 1
        rng, spots = random.Random(self._seed), []
        while len(spots) < len(found):
            spot = (rng.uniform(FRONT[0], FRONT[1]), rng.uniform(FRONT[2], FRONT[3]))
            if all((spot[0] - x) ** 2 + (spot[1] - y) ** 2 >= GAP ** 2 for x, y in spots):
                spots.append(spot)
        for cup, (x, y) in zip(found, spots):
            move_cup(cup, x, y)
        self.report(f"shuffled ({self._seed})")

    def go_home(self):
        for cup in cups(omni.usd.get_context().get_stage()):
            if cup.GetName() in HOME:
                move_cup(cup, *HOME[cup.GetName()])
        self.report("home")

    def paint(self, color):
        changed = 0
        for cup in cups(omni.usd.get_context().get_stage()):
            if cup.GetVariantSets().HasVariantSet("color"):                 # 강의 50 의 컵에만 색 Variant 가 있다
                omni.kit.commands.execute("SelectVariantPrim", prim_path=str(cup.GetPath()), vset_name="color", var_name=color)
                changed += 1
        self.report(f"{changed} cups -> {color}" if changed else "these cups have no color variant")       # 색 Variant 는 강의 50 의 컵에만 있다

    def toggle_person(self):
        stage = omni.usd.get_context().get_stage()
        for path in ("/World/Site/Booth/People/staff_5", "/World/Booth/People/staff_5"):      # 강의 50 의 장면, 그 밖의 장면
            person = stage.GetPrimAtPath(path) if stage else None
            if person:
                omni.kit.commands.execute("ToggleActivePrims", stage_or_context=stage, prim_paths=[path], active=not person.IsActive())
                self.report("person " + ("on" if stage.GetPrimAtPath(path).IsActive() else "off"))
                return
        self.report("no such person in this scene")
