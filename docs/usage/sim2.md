# Replaceable physics

`SimulationModule` owns one shared world and delegates physics to a
`PhysicsBackend`. Robot connections retain their streams and receive a
`SimRobotClient`. The MuJoCo and Genesis adapters implement the same named
body/joint state and command contract.

```text
ConnectionModule -> SimRobotClient -> SimulationModule -> PhysicsBackend
                                           |                 |
                                           +-- WorldState <--+
```

## Description, state and commands

| Type | Meaning |
|---|---|
| `WorldDescription` | Stable body/joint IDs, relationships, motion types, drives and capabilities |
| `WorldState` | One tick's body poses/velocities and joint configuration/velocity/actuator effort |
| `CommandBatch` | Named joint drives, body wrenches or kinematic targets, with an epoch and duration |
| `CommandReport` | Accepted, rejected, applied or expired; includes the relevant physics tick |
| `VisualScene` | Static meshes, materials and body-local visual/camera attachments |

Body poses and velocities use world coordinates at the body origin. Quaternions
are xyzw; distances are metres and angles radians. A joint relates bodies: a
cabinet hinge and a robot elbow have the same physical meaning. A joint need not
have an actuator. Free-root motion is represented by the root body's state.

`JointDrive` targets a declared scalar actuator. `BodyWrench` applies force and
torque to a dynamic body, optionally at an offset point. `KinematicTarget`
prescribes an authored kinematic body's world pose; it never teleports a dynamic
robot. Genesis currently rejects kinematic targets. Unsupported operations fail
explicitly. Initialization/reset is separate from physical actuation.

Acceptance queues a command; the `command_reports` stream reports application
at a physics tick. Zero duration means one step; positive duration uses simulation
time. Reset and pause/resume clear commands and invalidate their epoch. Multiple
body wrenches add.
An explicit scene joint drive overrides the robot's drive for that joint while
active. Actual movement is read from `world_state`; actuator effort excludes
contact/reaction forces and represents the last integration interval. A backend
that cannot report actuator effort returns `None`, including native robosuite tasks.

[`physics.py`](/dimos/sim2/physics.py) defines the contract.
[`SimulationRuntime`](/dimos/sim2/world/runtime.py) owns timing, command expiry,
stock policy execution and publications. Backends own model handles, physics
integration and native command translation. Engine-specific sensor checkpoints
remain private; they are not portable world state or cross-engine save files.

## Attachment and engine selection

After resolving configuration, the coordinator calls
[`attach_simulation()`](/dimos/sim2/composition.py) for `--simulation mujoco` or
`--simulation genesis`. It collects all `describe_simulated_robot()` results
before building one world and injects one client into each retained connection.
Namespaces identify robot instances. A manually supplied empty simulation module
is populated in place. Robot-specific descriptions and policies belong with the
robot package; policies consume neutral `RobotState` without native handles.

## Install and native tasks

```bash
uv sync --extra sim
# Native robosuite task API, including actions, observations, rewards and episodes.
dimos run robosuite-lift

# Genesis requires PyTorch 2.8; repository legacy ML test groups pin older versions.
UV_PROJECT_ENVIRONMENT=.venv-genesis uv sync --no-default-groups --extra sim-genesis
```

Native robosuite tasks use the MuJoCo adapter. Their action vectors do not define
the generic physics interface. Existing camera/IMU Observables and rolling lidar
workers remain available on MuJoCo. Genesis rejects native sensor acquisition,
robosuite task actions and kinematic targets explicitly.

Both adapters assemble MJCF through one importer. Genesis uses MuJoCo for source
asset parsing only; Genesis owns and steps its running physics state. URDF
conversion is not implemented. Authored masses/inertias and placement are
preserved. Explicit MJCF contact-pair eligibility is lowered to Genesis collision
masks; pair-specific solver/friction overrides are not equivalent across engines.
Portable state is not a promise of identical physics or a cross-engine save file.
