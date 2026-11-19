# Lecture 23 (2) Core API: the same scene in a few lines
# Run once, in a new scene (File > New).
import numpy as np
from isaacsim.core.api.objects import DynamicCuboid, GroundPlane
from isaacsim.core.api.physics_context import PhysicsContext

PhysicsContext()                                                        # physics scene
GroundPlane("/World/Ground", size=10, color=np.array([0.6, 0.6, 0.6]))  # ground
cube = DynamicCuboid("/World/Cube", position=np.array([0, 0, 1.0]),     # cube with rigid body + collider
                     scale=np.array([0.4, 0.4, 0.4]), color=np.array([1.0, 0.35, 0.65]))

print("cube position:", cube.get_world_pose()[0])
