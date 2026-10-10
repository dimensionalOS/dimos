# Isolated MuJoCo evaluation

END-263 adds a two-container evaluation path for the exported Robosuite xArm7
lift scene. The normal `xarm-perception-sim` blueprint remains a development
simulator, with its existing local SHM connection. It is **not** an isolation
boundary. Use the dedicated compose file for agent-controlled evaluations.

The robot image contains normal DimOS, Pi, a public xArm description, the
`mujoco_eval` ManipulatorAdapter and `MujocoRobotIO`. It contains no evaluation
suite, simulator engine, scene data, testcases, seeds or recording. The final
image is built from filtered files in a fresh stage: private source is not
hidden in lower image layers. The pinned public xArm description is prepared
at build time; it needs no runtime download. `xarm-eval` composes the adapter
with ControlCoordinator and existing manipulation skills. Its public robot
base placement is 0.912 m, matching this export.

The trusted image runs the existing MujocoEngine directly, without a DimOS
coordinator/bus or exposed module RPC. Only it mounts the private scene,
manifest and results directory. Engine-produced object TF is appended directly
to a new SQLite recording for this episode; it is never received from a bus.
Existing MuJoCo readiness checks and `lifted("cube_main", by_m=0.05)` are reused
without changing their formulas. Missing, non-finite, incomplete or stale
evidence produces `score: null` and an infrastructure error. A pre-existing
output directory is rejected rather than reusing evidence.

## Link and runtime

The dedicated Zenoh TCP session disables multicast, gossip, adminspace and
SHM. Its default-deny [Zenoh ACL](https://zenoh.io/docs/manual/access-control/)
permits incoming puts only on the current episode's command key and outgoing
state/sensor messages. Subscriptions to state/sensor keys are allowed; queries,
queryables, admin, TF/score publication, other episode keys and general RPC
are not. There is no pickle decoding, arbitrary type lookup or bus forwarding.

Commands use strict JSON with a 4 KiB wire limit. Trusted admission checks
schema, finite numbers, array dimensions, joint/actuator bounds, enable state,
run/episode IDs, a monotonically increasing sequence, and a 0.5 s timestamp
window. Timestamp freshness is checked again before application. This path
supports position, stop, enable and disable; velocity, torque, PD and cartesian
commands are unsupported. Stop/disable holds the current pose, without resetting
the episode. Lost commands also cause a hold. The adapter reports stale state
as unavailable; it has no SHM or scene-path fallback.

Allowed sensors are color/depth images, calibration and camera TF relative to
`link7`. Object truth and ideal world-relative simulator TF are private. Robot
FK can still use the public robot model on the normal robot-side bus. Sensor
payloads have an 8 MiB receive limit and fixed message decoders.

Both containers have separate filesystems, IPC and PID namespaces, read-only
roots, no added capabilities and no Docker socket. They share the trusted
container's **network-none** namespace, so loopback is the only network path.
There is no host port or internet route. The trusted process never joins the
robot-side DimOS session. This path intentionally has no model gateway.

`boundary.jsonl` records each command received at the admitted key, including
rejections; `attempts` counts received commands even when invalid. Zenoh ACL
or transport drops happen before this callback and are not counted as decoded
commands. These are basic boundary events, not full process/file/network/model
tracing or a cheating classifier.

## Container acceptance

Build/runtime validation needs Docker and a previously exported Robosuite
scene (including its relative mesh assets). Building downloads public software
and the pinned public robot model. Do not run build/runtime setup where those
actions need separate approval until it is obtained. No host route, firewall,
security setting or privileged container is required.

Create a private input directory and a writable, initially empty result root.
Copy the exported `robosuite/lift` directory into the private input as `lift`.
For each mode `normal`, `blocked`, `forgery`, write a fresh private
`episode.json`, using a distinct episode and output directory:

```json
{
  "run": "acceptance",
  "episode": "normal",
  "endpoint": "tcp/127.0.0.1:7449",
  "scene": "/private/lift/scene.xml",
  "output": "/results/normal",
  "duration_s": 10,
  "seed": 0,
  "case": "robosuite_lift_cube",
  "camera": "wrist_camera",
  "home": [0.0, -0.247, 0.0, 0.909, 0.0, 1.15644, 0.0]
}
```

`seed` records the provenance of the fixed export; this path does not introduce
random scene sampling. The scene/test selection, home pose and duration remain
trusted-side inputs. Set `EVAL_PRIVATE_DIR`, `EVAL_RESULT_DIR`, `EVAL_ENDPOINT`,
`EVAL_RUN`, `EVAL_EPISODE` and `EVAL_PROBE_MODE` on the host. The public IDs and
endpoint must match the private manifest. Result mounts must be writable by
`EVAL_UID:EVAL_GID` (defaults 1000:1000).

```bash
# From the repository root, with the variables above set:
docker compose -f docker/mujoco-eval/compose.yaml build
docker compose -f docker/mujoco-eval/compose.yaml up
# Inspect both container exit codes; both must be zero. Then remove this run:
docker compose -f docker/mujoco-eval/compose.yaml ps -a
docker compose -f docker/mujoco-eval/compose.yaml down
# Repeat with fresh manifests for blocked and forgery, then:
python docker/mujoco-eval/verify.py "$EVAL_RESULT_DIR"
```

The default robot client uses normal ControlCoordinator trajectory APIs and
camera ports for a deterministic hold baseline on the existing lift task.
Its expected trusted score is **0**, since it does not lift the cube. This
checks execution/recording/grading without a paid model; it does not demonstrate
a successful grasp. `blocked` also checks inaccessible private files/SHM/Docker
control and denied outbound connectivity. Both adversarial modes send private
bus/admin/sensor/score, malformed, replayed and cross-episode probes. The host
verifier requires the same private grade and recorded rejections.

For a robot-side agent, replace the robot command with a normal DimOS launch
(`dimos run xarm-eval mcp-server observe-skill`) and Pi configured for the
robot's local MCP endpoint. Model access is intentionally unavailable in this
acceptance topology. Do not mount a scene, testcase, result directory, shared
cache or Docker socket into the robot to make an agent run.

The focused local suite includes actual TCP ACL tests and a real synthetic
MuJoCo scene driven through ControlCoordinator, with camera/calibration
received by MujocoRobotIO and private grading resistant to forged publications.
It does not substitute for running the exported scene in the containers.

```bash
MUJOCO_GL=egl DIMOS_ZENOH_MULTICAST=false DIMOS_ZENOH_GOSSIP=false \
  python -m pytest dimos/evals/isolated_mujoco -o addopts='' -q
```

The score remains a geometric proxy; it does not prove stable physical grasps
or absence of every side channel. Kernel escapes and pretrained benchmark
knowledge are outside scope. Full tracing, cheating classification, Habitat,
a model gateway and global ControlCoordinator migration are deferred.
