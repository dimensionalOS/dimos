# Inspect Dreamlabs observations and action previews

This is a protocol/data-flow prototype for standard dual six-joint YAM input.
It does not open cameras, start robot drivers, connect a coordinator, or execute
returned actions. A preview is not a trajectory validated for physical rollout.

## Check the protocol offline

Install the optional lightweight client in the normal DimOS environment:

```bash
uv sync --extra dreamlabs --inexact
uv run --extra dreamlabs python -m dimos.imitation.policy.dreamlabs.probe
uv run --extra dreamlabs pytest dimos/imitation/policy/dreamlabs
```

The default probe prepares synthetic images and YAM state/FK locally. It makes no
API call and needs no credential. Tests replace the SDK network transport, check
three actual colored JPEGs, and compare 38 known and seeded poses against fixed
offline SDK FK references. No full robot SDK or `dream_ik` is required at runtime.

Each image follows `Image.resize(320, 176, interpolation=cv2.INTER_AREA)` and
`Image.to_base64(quality=85)`. The result is one JPEG/base64 string with no `data:`
prefix, not raw pixels or a list of historical frames. Color conversion and JPEG
encoding happen once through the existing Image API. The SDK does not decode or
resize the image. It keeps its own nine-frame history at 5 Hz.

State is mapped by joint name into left six joints, right six joints, left grip,
and right grip. Angles remain radians and grips remain 0 closed to 1 open.
Existing Pinocchio FK supplies arm-local XYZ in metres and the first two rotation
columns, plus the SDK grip aliases. The checked-in kinematic-only reference
retains the SDK's joint geometry and 0.1347 m TCP along flange z. It is not a
physical calibration or a replacement for the coordinator's robot model.

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

Calls are serialized. There are no custom sampler threads, watchdogs, takeover
logic, automatic rollout loops, or added coordinator RPCs. Starting the module
does not create a client or send observations. The `dreamlabs-dual-openyam`
blueprint contains only this protocol module and no hardware stack.

The preview maps `q_l_chunk` and `q_r_chunk` into DimOS order, anchors the first
point on the supplied state, and spaces 48 rows at 20 Hz. Checks cover shape,
finite values, and normalized grips. They do not check physical joint limits,
speed, collision, freshness, tracking, or stop behavior on a real robot.

## Keep live inference and execution explicit

Before a real call, confirm the provider credential, model access, image source,
and visible cost limit. Then the bounded, hardware-free probe is available as:

```bash
uv run --extra dreamlabs python -m dimos.imitation.policy.dreamlabs.probe --live
```

It sends one inference with generated black images and a synthetic state. Session
close may make an additional control request. SDK background recording is disabled
with `_record=False`. No live call has been validated here.

Physical execution is outside this prototype. A later approved station integration
can use the existing `ControlCoordinator.execute_trajectory()` and
`cancel_trajectory()`, and `task_invoke("joint_trajectory", "get_status",
{"t_now": None})` for status. No second executor is needed. This PR leaves those
APIs and all existing lower-level protections unchanged. Real execution still
needs verified robot/calibration, appropriate limits, an operator, workspace and
stop conditions. Do not treat this non-executing prototype as a complete robot
rollout or remove existing protections to attach it to hardware.
