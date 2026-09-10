# Manipulation from Python

Use the client-only `Arm` SDK for ordinary sequential motion. It wraps the
existing typed RPCs without changing robot behavior or owning the connection.
The client uses the same provisioned dimOS environment as the runtime.

## Start the runtime and shell

Start the classical manipulation stack in terminal one. It includes motion
planning, the simulated xArm, camera, perception, grasp generation, and pick/place,
without requiring LLM credentials:

```bash skip
env -u MUJOCO_GL dimos --simulation run xarm-perception-sim --headless false
```

`--headless false` opens the native MuJoCo window; clearing `MUJOCO_GL` removes
any earlier `egl` override. The global `--viewer` option controls Rerun, not
MuJoCo. Viser controls are available at the URL printed during startup.

On a headless Linux host:

```bash skip
MUJOCO_GL=egl dimos --simulation --viewer none run xarm-perception-sim --headless true
```

Wait for the Modules and sensor streams to start. In terminal two, using the same
project environment, open the generic [dimOS shell](/docs/usage/cli.md#dimos-shell):

```bash skip
dimos shell
```

The shell provides a connected `app`. Import the SDK and select an arm explicitly;
there are no manipulation-specific preloads or setup helpers:

```python skip
from dimos.manipulation.sdk import Arm

arm = Arm.from_app(app)
arm.info
arm.joints()
arm.pose()
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
client. Import `Arm` directly from `dimos.manipulation.sdk`; this is a convenience
module in dimOS, not a separate SDK installation.

### Explore without moving

Type `arm.` and press Tab to complete SDK methods. Use `arm.move_linear?` to see
its signature and docstring in IPython, or use ordinary Python help:

```python skip
help(arm)
describe(arm.rpc.move_linear)
```

`describe()` inspects raw modules and RPCs; use `help()` or `?` for SDK methods.
Importing, selecting the arm, reading state, and inspecting help do not command
motion. An agentic blueprint such as `xarm-perception-sim-agent` also works, but
keep its agent idle while you command the arm.

## Basic motion

Run these commands one at a time in simulation, observing each result before
continuing. Allow space for a small joint offset and a 1 cm vertical translation.
Save the initial joints before moving:

```python skip
print(arm.info)   # Group ID, joint order, and capabilities.
print(arm.state())
initial = arm.joints()  # Fresh NumPy array in arm.info.joint_names order.
q = initial.copy()
q[0] += 0.02
result = arm.move_joints(q, speed_scale=0.2)
print(result)
```

Move to an endpoint, then translate in a straight line:

```python skip
pose = arm.pose()
arm.move_pose([pose.x, pose.y, pose.z + 0.01], speed_scale=0.2)
arm.move_linear(dz=-0.01, check_collision=True)
```

After successful moves, restore the initial joints and open the gripper. If a
command failed, inspect its result and the current state before issuing another:

```python skip
arm.move_joints(initial, speed_scale=0.2)
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
from dimos.manipulation.sdk import MotionError

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

`Arm` borrows the app's connection. Exit IPython with `exit` or Ctrl-D when
finished; the shell disconnects while the blueprint keeps running. Neither shell
exit nor Ctrl-C during a call establishes that remote motion has stopped.
Use `arm.rpc.cancel()` and inspect its result when cancellation is needed.

## Advanced motion RPCs

Use `arm.rpc` for explicit planning, preview, nonblocking execution, and
cancellation. Use `app.find_module_by_spec(ManipulationSpec)` for direct typed discovery.

```python skip
from dimos.msgs.sensor_msgs.JointState import JointState

motion = arm.rpc
planned = motion.plan_to_joints({
    arm.info.id: JointState(position=arm.joints() + 0.01),
}, speed_scale=0.2)
if planned.succeeded and planned.plan is not None:
    print(motion.preview_plan(planned.plan))
    print(motion.get_visualization_url())
    started = motion.execute(blocking=False, plan_id=planned.plan.plan_id)
    if started.succeeded:
        print(motion.wait_for_execution(timeout=30.0))
```

Raw RPC results require explicit checks. `plan_to_poses` accepts stamped poses,
but currently interprets their coordinates as world-frame values; it does not
transform a supplied frame.

Planning stores one pending plan on the Module. A new planning request replaces
it. Pass `plan_id=planned.plan.plan_id` to execute only the returned plan; a
replacement is rejected without dispatching or consuming it. SDK moves always
pass this ID. Calling `execute()` without an ID explicitly consumes whichever
plan is pending. Previewing a plan does not select it for execution.
`clear_planned_path()` discards pending work without
cancelling active execution. Use one motion-commanding client per module;
separate `Arm` objects do not own independent plans or executions.

`execute()` blocks by default. With `blocking=False`, success means dispatch was
accepted. Use `wait_for_execution()` and check `ExecutionStatus.COMPLETED` for
physical completion. A wait timeout leaves execution active; call `cancel()` and
inspect its result to establish whether it stopped. A transport `TimeoutError`
also does not cancel remote work. Do not automatically retry motion after it.

## Use the same SDK in scripts

The SDK works outside IPython too. Scripts create their own connection and
disconnect in `finally`; inside `dimos shell`, reuse its existing `app` instead:

```python skip
from dimos.porcelain.dimos import Dimos
from dimos.manipulation.sdk import Arm

app = Dimos.connect()
try:
    arm = Arm.from_app(app)
    print(arm.joints())
    print(arm.pose())
finally:
    app.stop()
```

Disconnecting does not stop the remote blueprint or cancel its motion.
