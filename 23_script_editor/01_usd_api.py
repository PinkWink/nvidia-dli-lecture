# Lecture 23 (1) USD API: ground + falling cube
# Paste into Window > Script Editor and press Run (Ctrl + Enter). Run once, in a new scene (File > New).
# (The Script Editor cannot display Korean, so the comments here are in English.)
import omni.usd
from pxr import Gf, PhysicsSchemaTools, UsdGeom, UsdPhysics

stage = omni.usd.get_context().get_stage()          # the scene that is open now

# 1. physics scene: direction and size of gravity
scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
scene.CreateGravityMagnitudeAttr(9.81)

# 2. ground plane with a collider
PhysicsSchemaTools.addGroundPlane(stage, "/World/Ground", "Z", 10, Gf.Vec3f(0, 0, 0), Gf.Vec3f(0.6))

# 3. pink cube, 0.4 m, 1 m above the ground
cube = UsdGeom.Cube.Define(stage, "/World/Cube")
cube.CreateSizeAttr(0.4)
cube.CreateDisplayColorAttr([Gf.Vec3f(1.0, 0.35, 0.65)])
cube.AddTranslateOp().Set(Gf.Vec3d(0, 0, 1.0))

# 4. rigid body + collider (lecture 3: Rigid Body with Colliders Preset)
UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())
UsdPhysics.CollisionAPI.Apply(cube.GetPrim())

print("created:", [str(p.GetPath()) for p in stage.GetPrimAtPath("/World").GetChildren()])
