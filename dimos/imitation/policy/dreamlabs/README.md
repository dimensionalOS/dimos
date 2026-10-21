# Dreamlabs observations, previews and manual controls

The protocol-only entry accepts standard dual six-joint YAM input without opening
hardware or executing actions. The separate manual stack below adds acquisition
and explicit rollout controls. A preview is not proof of physical rollout readiness.

## Check the protocol offline

Install the optional lightweight client in the normal DimOS environment:

```bash
uv sync --extra dreamlabs --inexact
uv run --extra dreamlabs python -m dimos.imitation.policy.dreamlabs.probe
uv run --extra dreamlabs pytest dimos/imitation/policy/dreamlabs
```

The default probe prepares synthetic images and YAM state/FK locally. It makes no
API call and needs no credential. Tests replace the SDK network transport, check
three actual colored JPEGs, and compare three representative poses against fixed
canonical-model TCP pose fixtures. The canonical dual-YAM model asset must already
be cached for a fully offline probe. No full robot SDK or `dream_ik` is required at runtime.

Each image follows `Image.resize(320, 176, interpolation=cv2.INTER_AREA)` and
`Image.to_base64(quality=85)`. The result is one JPEG/base64 string with no `data:`
prefix, not raw pixels or a list of historical frames. Color conversion and JPEG
encoding happen once through the existing Image API. The SDK does not decode or
resize the image. It keeps its own nine-frame history at 5 Hz.

State is mapped by joint name into left six joints, right six joints, left grip,
and right grip. Angles remain radians and grips remain 0 closed to 1 open.
Existing Pinocchio FK uses the same canonical `DUAL_OPENYAM_MODEL` as the
coordinator, supplying arm-local XYZ in metres and the first two rotation columns,
plus SDK grip aliases. Each arm's fixed mount is removed from the world pose.
The canonical gripper link has a different coordinate convention from the SDK:
SDK x = -canonical y, SDK y = -canonical x, SDK z = -canonical z. The SDK TCP is
`[0, 0, -0.1347]` metres in that canonical gripper frame. This fixed tool transform
preserves the SDK pose convention; it is not a joint offset or measured physical
calibration. Our `grasp_frame` is only 0.1 m along -z and cannot be substituted
without that extra tool distance and rotation. No vendored URDF remains.

This path does **FK only**, even though its existing helper is named `PinocchioIK`.
It never calls `solve()`. SDK configuration has `robot="yam"` and no `ik`; returned
`q_l_chunk/q_r_chunk` are absolute joint targets, not Cartesian targets solved by
local IK. Server-internal generation is not established by this adapter.

Offline comparison at identical retained left/right q showed canonical TCP
position differences of 9.486/9.560 mm from the former vendor geometry, mainly
height, with orientation differences below 0.000004 rad. Joint screw directions
in the arm base agree despite different local axis labels; no joint-angle sign
or zero-offset remapping was introduced. Both models retain a nonnegative J2
lower bound. Replacing FK geometry does not prove that policy out-of-limit joint
requests or physical calibration are fixed.

## Supply explicit observations

`DreamlabsModule` is a normal module with these RPCs:

- `set_observation(overhead, left_wrist, right_wrist, state, timestamp)` buffers
  one caller-paired tick locally. Supply `Image`, `JointState`, and monotonic
  capture seconds. The SDK checks timestamp order. This prototype does not
  synchronize live sensors or judge their age.
- `get_action()` sends one potentially billed inference request and returns an
  unexecuted `JointTrajectory` preview. It requires a prior observation.
- `reset_episode()` clears the SDK history between trials.
- `stop()` closes the client through the normal module lifecycle.

Calls are serialized. This protocol-only entry has no custom sampler threads, watchdogs, takeover
logic, automatic rollout loops, or added coordinator RPCs. Starting the module
does not create a client or send observations. The `dreamlabs-dual-openyam`
blueprint contains only this protocol module and no hardware stack.

The preview maps `q_l_chunk` and `q_r_chunk` into DimOS order, anchors the first
point on the supplied state, and spaces 48 rows at 20 Hz. Checks cover shape,
finite values, normalized grips, and canonical model position bounds. Arm targets
up to 2 degrees outside a bound are clamped; larger overshoots reject the whole
chunk. The measured anchor is unchanged. Speed, collision, physical calibration,
freshness, tracking, and stop behavior on a real robot remain unverified.

## Keep live inference and execution explicit

Before a real call, confirm the provider credential, model access, image source,
and visible cost limit. Then the bounded, hardware-free probe is available as:

```bash
uv run --extra dreamlabs python -m dimos.imitation.policy.dreamlabs.probe --live
```

It sends one inference with generated black images and a synthetic state. Session
close may make an additional control request. SDK background recording is disabled
with `_record=False`. The hardware-free probe is independent of the station run described below.

## Manual testing stack

`dreamlabs-dual-openyam-manual` composes three existing camera modules, the dual
YAM coordinator and its canonical trajectory task (all 14 joints),
`DreamlabsRolloutModule`, and the existing `ManipulationModule` with Viser. Device paths are required configuration; omitted CAN
ports select the existing mock hardware. This blueprint starts acquisition but
leaves policy inference and policy dispatch idle. With real CAN ports, the
existing driver startup enables motors, calibrates grippers and starts gravity
compensation: idle policy does **not** mean motors are disabled or no low-level
arm commands are written. The earlier enable-and-read fake-rollout script is a
separate diagnostic path.

### Prepare an isolated environment

The manual stack needs the `planning` and `dreamlabs` extras. The existing station
shared environment has RoboPlan 0.6.0; this branch pins the 0.7 series and its lock
resolves `roboplan` and `roboplan-core` to 0.7.0. Version 0.6 lacks
`loadUrdfSceneDescriptionFromXml`, so merely setting `PYTHONPATH` to this checkout
is insufficient. Do not upgrade the shared environment used by other worktrees.

On anvil, run the following setup yourself from the updated PR worktree. It
installs locked dependencies into a new isolated environment; it does not launch
cameras, drivers or inference. The lock records official PyPI RoboPlan packages.
No station installation is performed automatically by this PR.

```bash
cd /home/anvil/worktrees/dreamlabs-fake-rollout-4653-20261021
UV_CACHE_DIR="$PWD/.station_test/uv-cache" \
UV_PROJECT_ENVIRONMENT="$PWD/.station_test/venv" \
  /home/anvil/.local/bin/uv sync --frozen --no-default-groups \
  --extra planning --extra dreamlabs \
  --python /home/anvil/codes/dimos/.venv/bin/python
source .station_test/venv/bin/activate
export PYTHONPATH="$PWD"
```

Use that environment in **both** the launcher and shell terminals. The earlier
`.station_test/client-packages` SDK-only overlay is preserved but is no longer
needed with this complete environment. Check the actual API before hardware launch:

```bash
python -c 'from importlib.metadata import version; import roboplan.core as c; print(version("roboplan-core")); assert callable(getattr(c, "loadUrdfSceneDescriptionFromXml", None))'
dimos run dreamlabs-dual-openyam-manual --help
```

The blueprint also checks that loader before deploying any module, so this
specific mismatch fails before cameras or robot drivers start. This is a
capability check, not a compatibility shim for the old API.

### Launch the stack

For the tested anvil station, the physical USB paths were A overhead, B right
wrist, C left wrist. Recheck ownership and camera roles if cabling changes.
The following command is a **manual hardware launch**, not an offline probe:

```bash
PYTHONPATH=. dimos run dreamlabs-dual-openyam-manual \
  --overheadcamera.device /dev/v4l/by-path/pci-0000:06:00.0-usb-0:1.3:1.0-video-index4 \
  --rightwristcamera.device /dev/v4l/by-path/pci-0000:07:00.0-usb-0:1.3:1.0-video-index4 \
  --leftwristcamera.device /dev/v4l/by-path/pci-0000:08:00.0-usb-0:1.3:1.0-video-index4 \
  --controlcoordinator.left-can-port follower_l \
  --controlcoordinator.right-can-port follower_r \
  --dreamlabsrolloutmodule.timeout-ms 5000
```

In another terminal in the same checkout, activate `.station_test/venv/bin/activate`,
set `PYTHONPATH="$PWD"`, then run `dimos shell`. Use normal SDK credentials
configured locally; do not put API keys in shell commands. These RPCs control
only the policy:

```python
app.DreamlabsRolloutModule.rollout_status()
app.DreamlabsRolloutModule.set_prompt("hold position")
app.DreamlabsRolloutModule.start_rollout()  # billed inference + previews; no policy dispatch
app.DreamlabsRolloutModule.stop_rollout()
```

The explicit actuator opt-in, for a separately cleared physical test, is:

```python
app.DreamlabsRolloutModule.start_rollout(dispatch_actions=True)
app.DreamlabsRolloutModule.stop_rollout()
```

Stop invalidates pending replies before cancelling the existing trajectory task.
An already running API request can finish and be billed, but its result cannot
publish or execute after stop. `busy=True` means that request/history reset is
still draining; wait for `busy=False` before starting again. Cancellation errors
are raised and reported in `last_error`; retry cancellation before restarting.
Stop is not a physical E-stop, does not disable motors or stop acquisition.
Use the existing run shutdown (`Ctrl-C` in the launch terminal) for teardown.

Changing the prompt stops/cancels the current trial and stays idle. A new start
collects a fresh nine-tick episode before inference. No old chunks are queued.
At 5 Hz, the module pairs latest camera/state messages within configurable
age/skew tolerances; this is not hardware synchronization. The SDK owns frame
history and inference chunk blending. One request is active at a time. The
coordinator owns trajectory interpolation, completion, start-state mismatch
checks and cancellation; its existing velocity limiting remains in effect.
No automatic API retries occur. Errors stop the trial and remain visible in
`rollout_status()`. Validated trajectories are published on `policy_preview` in
both modes. Shape and nonfinite arm/gripper values reject the response. Finite
model-output gripper openings are clamped to `[0,1]` (0 closed, 1 open); warning
logs record raw extrema, clipped indices/values and counts. Inputs remain strict.
`get_last_action_response()` returns the last SDK action dict before conversion,
even after conversion rejection or episode reset, so raw diagnostics remain
available in memory. No request images or credentials are retained by that RPC.
Conversion applies the same canonical arm bounds to previews and dispatch: model
output up to 2 degrees beyond a bound is clamped to that bound, with joint name,
indices, raw values, count and maximum correction logged. An overshoot larger than
2 degrees or any nonfinite value rejects the whole chunk. This tolerance is an
explicit prototype choice, not a measured hardware accuracy specification. Raw SDK
output and measured state remain unchanged. Dispatch retains its strict position
bounds check, including the unmodified measured anchor. Preview is not proof of
physical readiness.


### View the robot with Viser

The existing manipulation model uses the same dual-YAM URDF as the coordinator.
Both modules consume `coordinator_joint_state`; no extra driver or motor
initialization is added. Viser displays the 12 arm joints from the received state.
This remains SDK-derived feedback, not independently timestamped motor telemetry.
The default Viser host/port are `127.0.0.1:8095`. From your laptop, forward the port:

```bash
ssh -N -L 8095:127.0.0.1:8095 anvil@10.10.0.229
```

Open `http://localhost:8095`. In the DimOS shell, the configured URL is also
available through `app.ManipulationModule.get_visualization_url()`. An occupied
port can be changed with `--manipulationmodule.visualization.port 8096` (adjust
the SSH forward too). The existing manipulation/Viser extra is required; this
change does not install it on the station.

Viser's existing planning/operator panel is visible by default. Before executing
from the panel or manipulation RPCs, first call `stop_rollout()`, wait for
`busy=False`, and finish/cancel that execution before starting policy again.
Both use the coordinator's same canonical trajectory task: it can replace an
existing trajectory, so it is not a cross-client ownership lock. For a view-only
session, use `--manipulationmodule.visualization.panel-enabled false`. Neither
planner nor policy runs automatically at startup.

The viewer currently shows measured state, not automatic Dreamlabs target ghosts
or measured-commanded error. Existing Viser ghosts are used by manipulation's
`preview_plan()` workflow; there is no `policy_preview` stream adapter, and the
raw 14-joint policy chunk includes grippers outside this 12-joint display model.
No separate visualization framework or ghost adapter is added here. The current
base also has no per-group `gripper_tasks` API from unmerged PR #4747; manipulation
gripper controls remain absent, while policy's normalized grippers continue
through its existing 14-joint trajectory.

### What remains unvalidated

The retained live fake-rollout run02 at `59c3239b1` executed no policy actions.
Its right-gripper output reached `1.0061448679` (21/48 rows above one); left/right
J2 targets reached `-0.0174917514` / `-0.0034554977` rad although measured J2 was
inside the model bounds. The original strict prototype rejected this response.
With the later explicitly requested gripper-output clamp, the right-gripper
outliers become 1.0. The later bounded arm-output correction below also handles
these small J2 overshoots. SDK-reference J2 lower bounds and coordinator-model lower
bounds were both effectively zero, but physical encoder/calibration agreement
has not been independently verified. Equal before/after cached SDK readings did
not prove fresh per-motor CAN feedback; a recent host message timestamp still
does not prove it. No collision, physical tracking, feedback-freshness or real
start/stop validation is claimed. This interface must not be treated as proof
that physical rollout is ready.

The manual interface tests use synthetic streams and mocked inference/execution.
They cover idle/preview/explicit dispatch, in-flight stop/prompt invalidation,
model-bound rejection and blueprint/config wiring. They do not launch devices
or call the live API.


### Retained-response limit check and offline replay

Both retained full-blueprint preview runs started with left J2 at +0.054642 degrees,
inside but almost touching the canonical lower bound (effectively zero). No other
arm joint's SDK-smoothed chunk exceeded the canonical limits in either run.
These data support a small near-boundary overshoot; they do not establish a joint
convention error. The current YAM joint-head path is retained; no local IK is added.

| Retained acquisition commit | Left J2 minimum | Rows corrected | Maximum correction | Maximum TCP displacement from correction |
| --- | ---: | ---: | ---: | ---: |
| `ee042059fa` (previous FK) | -1.015477 degrees | 27/48 | 1.015477 degrees | 4.127 mm |
| `cc10fb4a56` (canonical FK) | -1.326378 degrees | 29/48 | 1.326378 degrees | 5.383 mm |

Replay uses the retained SDK-returned action dictionaries, not new inference.
Those dictionaries are already SDK-smoothed, not untouched HTTP responses.
Both pass the bounded conversion, with all 48 output rows inside the canonical arm
bounds. Original responses and measured anchors are preserved. The TCP figures
compare FK before/after correction using the canonical model and explicit SDK TCP;
they are not observed physical motion. Raw right-gripper outliers are corrected by
the separately approved [0,1] clamp. Neither replay dispatches anything.

Validation: all 37 scoped Dreamlabs tests and the scoped pre-commit checks passed.
Offline regression covers lower/upper near-boundary correction, unchanged in-range
joints and raw responses, preserved anchors, rejection beyond 2 degrees, nonfinite
values, gripper conversion, and the existing stop/stale/prompt behavior. No new
hardware or API test was performed for this change, and fresh motor feedback and
physical calibration remain unverified.
