# Lecture 23 (3) Change the wheel speed of the Pinky Pro scene that is open
# Open work/pinky_scene.usd first. You can change the numbers and Run again while playing.
import omni.usd
from pxr import UsdPhysics

LEFT, RIGHT = 360.0, 180.0        # wheel target velocity [deg/s]. left faster -> turns right

stage = omni.usd.get_context().get_stage()
for name, speed in (("l_wheel_joint", LEFT), ("r_wheel_joint", RIGHT)):
    joint = stage.GetPrimAtPath(f"/World/pinky/joints/{name}")
    UsdPhysics.DriveAPI(joint, "angular").GetTargetVelocityAttr().Set(speed)
    print(name, "target", speed, "deg/s")

body = stage.GetPrimAtPath("/World/pinky/base_link")
x, y, z = omni.usd.get_world_transform_matrix(body).ExtractTranslation()
print(f"robot position: x={x:.3f} y={y:.3f} z={z:.3f} m")
