# Define placement in object coordinates

The generic `place_at(x, y, z)` operation positions the held object's reference point rather than the robot TCP. The pick transaction retains the grasp relationship and derives the required TCP placement pose while preserving the grasped object orientation, so callers describe the physical object outcome without compensating for gripper geometry.
