# Manipulation from Python

Connect a Python application to a running manipulation blueprint and resolve its
Modules by their typed RPC contracts. This client uses the same provisioned DimOS
environment as the runtime.

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
from dimos.manipulation.manipulation_spec import ManipulationSpec
from dimos.manipulation.pick_and_place_spec import PickAndPlaceSpec

app = Dimos.connect()
motion = app.get_module(ManipulationSpec)
pick_place = app.get_module(PickAndPlaceSpec)
```

Use these explicit imports for static typing. Resolution checks advertised RPC
names and the imported module class's signatures using the same signature
compatibility rules as blueprint Spec injection. It requires one match; it does
not choose the first provider or bind by class-name convention. The deployed
module classes must be importable in the client environment.

If several instances implement a Spec, the error lists their names. Select one:

```python skip
motion = app.get_module(ManipulationSpec, instance_name="robot0/manipulation")
```

## Motion

Discover the configured group IDs and current state before selecting targets:

```python skip
print(motion.list_planning_groups())
snapshot = motion.get_state()
print(snapshot)
```

| Operation | Result |
|---|---|
| `plan_to_joints({group_id: JointState(...)})` | `PlanResult` with an inspectable plan |
| `plan_to_poses({group_id: PoseStamped(...)})` | `PlanResult`; poses carry their frame |
| `preview_plan(plan)` / `clear_planned_path()` | `CommandResult` |
| `execute()` / `wait_for_execution(timeout=...)` / `cancel()` | `ExecutionResult` |
| `move_linear(dx=..., planning_group=..., check_collision=True)` | `MoveResult` containing planning and execution outcomes |
| `set_gripper_position(position, planning_group=...)` | `CommandResult` |

Action results expose `.succeeded`; planning, execution, and command results also
carry enum statuses and messages. `MoveResult` contains the component results.
Gripper positions are normalized travel: `0.0` closed, `1.0` open. Translations
are metres and angular joint positions are radians. `move_linear` translates in
the world frame and only checks collisions when requested.

Planning stores one pending plan on the Module. A new planning request replaces
it; `execute()` consumes it. Previewing an explicit plan does not change which
plan execution consumes. `clear_planned_path()` discards pending work without
cancelling active execution.

`execute()` blocks by default. With `blocking=False`, success means dispatch was
accepted. Use `wait_for_execution()` and check `ExecutionStatus.COMPLETED` for
physical completion. A wait timeout leaves execution active; call `cancel()` and
inspect its result to establish whether it stopped. A transport `TimeoutError`
also does not cancel remote work. Do not automatically retry motion after it.

## Objects

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

The example demonstrates joint and pose planning, preview, blocking and
nonblocking execution, cancellation, a short linear move, and gripper control:

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
