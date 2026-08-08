# Pick and place

`PickAndPlaceModule` is the single owner of generic pick/place transactions.
Its public workflow is:

1. `scan_objects(object_names)` creates a numbered, immutable scene snapshot.
2. `select_object(number)` pins that object and GraspGenX proposals without moving.
3. `pick_selected()` rechecks feasibility, approaches with collision checking,
   executes straight contact/retreat legs without collision queries, closes the
   gripper, and requires positive closure feedback.
4. `place_at(x, y, z)` places the held object's reference point at the requested
   world-frame position.

`BoxFillingPickAndPlaceModule` derives from the generic module and adds only
destination-container selection, fit checks, and box placement policy.

The two public products are:

```bash
uv run dimos run xarm-box-filling --daemon
uv run dimos run xarm-grasp-sim-agent --daemon
```

Both use GraspGenX as their only grasp provider. The real product uses an xArm6
and wrist RealSense; the simulation product uses xArm7, MuJoCo, and ground-truth
object geometry. Camera-based simulation remains test-only.
