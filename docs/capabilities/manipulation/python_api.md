# Manipulation from Python

Use the client-only `Arm` SDK for ordinary sequential motion. It wraps the
existing typed RPCs without changing robot behavior or owning the connection.
The client uses the same provisioned DimOS environment as the runtime.

## Start and connect

The existing agentic simulation blueprint includes motion planning, the simulated
xArm, camera, perception, grasp generation, and pick/place:

```bash
dimos run xarm-perception-sim-agent
```

For Python-only use without LLM credentials, run `xarm-perception-sim`; it
contains the same classical manipulation stack. On a headless Linux host:

```bash
MUJOCO_GL=egl dimos --viewer none run xarm-perception-sim --headless true
```

Wait for the Modules and sensor streams to start. Use a second terminal for the
client; leave the agent idle while the script commands the arm.

```python skip
from dimos.porcelain.dimos import Dimos
from dimos.sdk.manipulation import Arm

app = Dimos.connect()
arm = Arm.from_app(app)
```

`Arm.from_app()` resolves `ManipulationSpec` and selects the unique pose-capable
planning group. An arm need not have a gripper. Discovery does not start a runtime
or command motion. Missing or ambiguous selections report the available IDs.

For multiple arms or deployed motion modules, select explicitly:

```python skip
arm = Arm.from_app(app, group="left_arm", instance_name="robot0/manipulation")
```

Module resolution checks advertised RPCs and signatures using the same rules
as blueprint Spec injection. Deployed module classes must be importable in the
client. Import `Arm` directly from `dimos.sdk.manipulation`; this is a convenience
module in DimOS, not a separate SDK installation.

## Basic motion

Read state, adjust a joint, and move. These calls command the arm; try them in
simulation with room for the requested movement:

```python skip
print(arm.info)   # Group ID, joint order, and capabilities.
print(arm.state())
q = arm.joints()  # Fresh NumPy array in arm.info.joint_names order.
q[0] += 0.02
result = arm.move_joints(q, speed_scale=0.2)
print(result)

pose = arm.pose()
arm.move_pose([pose.x, pose.y, pose.z + 0.01], speed_scale=0.2)
arm.move_linear(dz=-0.01, check_collision=True)
arm.open_gripper()
```

Joint and pose inputs accept lists, tuples, and NumPy arrays. Positions use
metres; angular joints use radians. `move_pose(position, orientation=None)` takes
world-frame XYZ and an optional XYZW quaternion. Omitting orientation preserves
the current orientation. It targets an endpoint, not necessarily a straight path.
No frame transformations are performed.

`move_linear(dx, dy, dz)` invokes the existing straight-translation primitive
directly, in the world frame. **Collision checking remains off by default**, as
on the RPC. Pass `check_collision=True` to validate against the configured world;
this rejects a colliding segment rather than finding a detour.

| SDK call | Result |
|---|---|
| `move_joints(...)`, `move_pose(...)` | `ExecutionResult` after completed execution |
| `move_linear(...)` | Existing `MoveResult`, including trajectory generation and execution outcomes |
| `set_gripper_position(position)`, `open_gripper()`, `close_gripper()` | `CommandResult` |
| `home()`, `move_to_preset(name)` | `ExecutionResult` after moving to an existing preset |

Gripper positions are normalized travel: `0.0` closed, `1.0` open. Movement calls
accept `speed_scale` and execution `timeout` options. Ordinary calls block and
raise on failure, so scripts do not need success checks between every action.

### Shared presets

```python skip
print(arm.state().joint_presets.keys())
arm.home(speed_scale=0.2)
arm.move_to_preset("init", speed_scale=0.2)
```

These use the same configured Home and captured Init joint values as Viser.
The SDK reads advertised presets on every call, so a server-side Init reset is
reflected immediately. Missing presets raise with the available names; there is
no fallback home pose. Unlike selecting a preset in Viser, these calls execute
the move and wait for completion.

### Failures and connection ownership

```python skip
from dimos.sdk.manipulation import MotionError

try:
    arm.move_linear(dz=0.01, check_collision=True, timeout=30.0)
except MotionError as error:
    print(error.operation, error.result)
    raise
```

`MotionError.result` preserves the original typed failure result. Invalid numeric
inputs raise `ValueError`; unavailable telemetry raises `RuntimeError`. Transport
exceptions propagate unchanged. Failed planning never proceeds to execution.
A timeout does not imply motion stopped, and the SDK neither retries nor cancels
automatically. Use the underlying RPC to inspect execution or request cancellation.

`Arm` borrows the app's connection. Call `app.stop()` when finished; for a
connected app this disconnects without stopping the blueprint or cancelling motion.

## Advanced motion RPCs

Use `arm.rpc` for explicit planning, preview, nonblocking execution, and
cancellation. The original `app.get_module(ManipulationSpec)` API remains available.

```python skip
from dimos.msgs.sensor_msgs.JointState import JointState

motion = arm.rpc
planned = motion.plan_to_joints({
    arm.info.id: JointState(position=arm.joints() + 0.01),
}, speed_scale=0.2)
if planned.succeeded:
    print(motion.preview_plan(planned.plan))
    print(motion.get_visualization_url())
    started = motion.execute(blocking=False)
    if started.succeeded:
        print(motion.wait_for_execution(timeout=30.0))
```

Raw RPC results require explicit checks. `plan_to_poses` accepts stamped poses,
but currently interprets their coordinates as world-frame values; it does not
transform a supplied frame.

Planning stores one pending plan on the Module. A new planning request replaces
it; `execute()` consumes it. Previewing an explicit plan does not change which
plan execution consumes. `clear_planned_path()` discards pending work without
cancelling active execution. Use one motion-commanding client per module;
separate `Arm` objects do not own independent plans or executions.

`execute()` blocks by default. With `blocking=False`, success means dispatch was
accepted. Use `wait_for_execution()` and check `ExecutionStatus.COMPLETED` for
physical completion. A wait timeout leaves execution active; call `cancel()` and
inspect its result to establish whether it stopped. A transport `TimeoutError`
also does not cancel remote work. Do not automatically retry motion after it.

## Objects

Pick/place stays on its typed RPC contract rather than the `Arm` wrapper:

```python skip
from dimos.manipulation.pick_and_place_spec import PickAndPlaceSpec

pick_place = app.get_module(PickAndPlaceSpec)
```

Scan results expose typed objects directly. Choose an exact object ID from the
latest scan; names are not necessarily unique.

```python skip
scan = pick_place.scan_objects(["cup"])
if scan.succeeded:
    for detected in scan.objects:
        print(detected.object_id, detected.name)
```

`pick_object(object_id)` returns `PickResult`. `place_at(x, y, z)` returns
`PlaceResult`; coordinates specify the end-effector release position in the
Module's planning frame, preserving the selected grasp orientation. Both remain
blocking operations. Their statuses preserve perception, planning, execution,
and gripper failure distinctions.

Always inspect `.holding_object` after a failed pick or place. A failed pick
retract can leave the object held; a failed place retract can occur after release.
This is the workflow's tracked state, not continuous object tracking. Stopping
an individual motion is not a whole-pick/place cancellation API.

Agent tools call the same implementations. They receive a formatted version of
these domain results through `agent_encode()`; Python clients receive dataclasses,
not agent text or a metadata dictionary.

## Runnable example

The example demonstrates SDK joint and pose moves, a short linear move, and
gripper control. Each motion completes before the next begins:

```bash
python -m dimos.manipulation.planning.examples.manipulation_client
```

To include pick/place, supply a unique description and a placement position
verified in your simulated scene:

```bash
python -m dimos.manipulation.planning.examples.manipulation_client \
  --object cup --place X Y Z
```

Replace `X Y Z` with numeric planning-frame coordinates. The example rejects
ambiguous scans and stops on a failed action, including a failure while holding
an object. Its motion sequence requires space for a small joint offset and a
1 cm vertical translation. It restores the initial arm pose before scanning.

The example closes the connection in `finally`. For interactive use, call
`app.stop()` when finished; this disconnects without stopping the remote blueprint.
