# Dual OpenYAM simulation demo

This demo runs two OpenYAM arms, three cameras, bottles, and a bin in MuJoCo.
Perception uses named simulation bodies and sampled geometry. It does not
establish camera-based detection or transfer to physical hardware.

The classical sequence is the primary demonstration. ACT learns one right-arm
bottle task from scripted recordings and is a separate bonus segment.

## Prepare the checkout

Use the feature checkout with the learning stack already included. Install the
project dependencies and retrieve the two planning assets:

```bash
uv sync --extra all
git lfs pull --include="data/.lfs/yam_description.tar.gz,data/.lfs/dual_openyam_abc_box_v2.tar.gz"
uv run python -m dimos.robot.manipulators.dual_openyam.setup_sim_scene
uv run python -m dimos.robot.manipulators.dual_openyam.tool_check_sim_model
```

The scene setup downloads the pinned Amazon ABC asset revision and preserves
its licenses. If `data/dual_openyam_sim` already contains the upstream assets
but no `demo.xml`, add `--existing-assets`. Existing generated scenes are never
overwritten. The generated scene keeps three bottles, separates the unused
third bottle, uses the same target geometry on both sides, and widens the bin
horizontally by 1.5× for the two-bottle task. Robot gains, contact geometry,
friction, and bottle attachment behavior are unchanged. Grasping uses contact
physics; there are no object welds or artificial attachments.

Use `MUJOCO_GL=egl` for headless camera rendering on this machine. The simulation
blueprints select the physics adapter explicitly and reject CAN ports. No
`--simulation` flag is required.

## Classical segment

```bash
MUJOCO_GL=egl dimos --viewer rerun --rerun-open web run dual-openyam-sim-agent --daemon
dimos mcp list-tools
dimos mcp call reset_scene
dimos agent-send "Put bottle_1 and bottle_4 in the bin using the appropriate arm for each. Return both arms home and verify both are inside."
dimos mcp call inspect_sim_scene
dimos stop
```

The model uses `scan_objects` positions to choose the right arm for y ≤ 0 and
the left arm for y > 0. It must pass `planning_group` on every motion, place
with the arm that picked, and check `inside_bin` before claiming completion.
The MCP server uses port 9990 by default. If changing `--mcp-port`, also set
`--mcpclient.mcp-server-url=http://localhost:<port>/mcp` so the client uses the
same endpoint.

For a scripted physical acceptance run with no model service:

```bash
MUJOCO_GL=egl uv run python -m dimos.robot.manipulators.dual_openyam.tool_generate_demos \
  --bimanual --episodes 5 --max-attempts 5 --jitter 0 \
  --report outputs/openyam-classical-check.jsonl
```

Each cycle resets, lifts each bottle at least 5 cm for two seconds, places it,
returns the empty arm home, and checks the entire sampled bottle surface
against the round bin's radius and depth. Both bottles are checked again after
the second placement. Motion completion and object displacement alone do not
count as success. Reports include failures as well as successes.

`reset_scene` stops any policy, cancels the planner, returns both arms home,
resets physics, clears held-object bookkeeping, and waits for measured scene
stability so the next scan sees settled bottles. Use it between stage takes.
The supplied scene is calibrated for these bottles and this bin; it is not a
general object manipulation benchmark.

## Collect and prepare ACT data

```bash
MUJOCO_GL=egl uv run python -m dimos.robot.manipulators.dual_openyam.tool_generate_demos \
  --episodes 100 --max-attempts 160 --seed 0 --jitter 0.015 \
  --recording outputs/openyam-training.db \
  --report outputs/openyam-training-collection.jsonl

dimos imitation inspect outputs/openyam-training.db --workflow dual-openyam-sim
dimos imitation prepare dual-openyam-sim outputs/openyam-training.db \
  --output outputs/openyam-dataset
```

The generator warms up one unrecorded cycle, randomizes the right bottle by up
to 1.5 cm in each horizontal axis before recording, and saves only physically
verified takes. Failed takes and interrupted active takes are discarded. Before saving, the
generator checks the completed motion against the dataset quality gate. It
checks the final saved interval again before counting a take.
It refuses to overwrite a recording or report. Collection gives each module its
own worker so camera rendering does not share a Python process with recording.
Use `--resume --episodes N --recording <same.db> --report <new.jsonl>` to add N
new valid takes; a matching `.scene.json` manifest is required. The scene and
planning-model hashes, full IO contract, task, and joint order must match. A
quality-only manifest migration is required when changing preparation settings;
verify every other field and preserve the previous manifest. Resume preserves
old rows; it does not use the recorder's legacy `append` mode, which replaces
selected streams. `--arm both` alternates arms for
experimentation; the trained task in this run uses the default right arm.

SQLite records all three 320×240 RGB cameras at 30 Hz, measured state, and the
coordinator's applied position commands. Dataset alignment and policy control
run at 15 Hz. This intentionally differs from the original 30 Hz training plan:
rendering at twice the target sample rate provides timing margin. The sim
profile uses a 20 ms normal alignment limit, then holds the previous causal
source value for missing samples. At most 3% of emitted frames may contain
held values; the generator and exporter enforce the same cap. Leading targets
without complete causal data are trimmed. Filled samples can be older than
20 ms, and their count and maximum age are reported. Camera rate and gap
limits remain diagnostic in fill mode. Hardware profiles still use strict
quality checks. Preparation excludes invalid saved episodes and records rejection reasons in
`dimos_meta.json`. Verify at least 100 valid exported episodes before the full
training run. Avoid other GPU workloads during collection.

The action/state order is 14 values: left joints 1–6, right joints 1–6, left
gripper, right gripper. Grippers use meters of half-opening, 0–0.0475. Image
features are `observation.images.top`, `observation.images.left_wrist`, and
`observation.images.right_wrist`; joint state is `observation.state` and the
training target is `action`.

## ACT deployment

LeRobot runs in `dimos/imitation/policy/lerobot/python`, isolated from the host
robotics dependencies. Prewarm before a presentation; first use downloads a
large environment. Its Linux CUDA build is pinned to CUDA 12.8 for this laptop.

```bash
uv sync --project dimos/imitation/policy/lerobot/python --frozen
uv run --project dimos/imitation/policy/lerobot/python --frozen --with-editable . \
  python -c 'import torch; print(torch.__version__, torch.cuda.is_available())'
```

The [validation report](/docs/demos/openyam-validation.md) distinguishes diagnostic checkpoints
from the full training and evaluation work that remains. To launch a checkpoint
without an agent:

```bash
MUJOCO_GL=egl dimos imitation run dual-openyam-lerobot \
  --artifact outputs/dual-openyam-act/checkpoints/last/pretrained_model \
  --task "Put the bottle in the bin" --device cuda
```

Space starts/stops the policy; Q stops the stack. For agent control, launch
`dual-openyam-sim-policy-agent` and override
`--policyrolloutmodule.artifact=<checkpoint-directory>` if needed. `run_policy`
executes only the configured task and takes no natural-language goal.
`policy_status` reports readiness, accepted chunks, activity, and errors.
`stop_policy` cancels its trajectory and releases control.

This demo grants the policy priority 30 over the classical planner/grippers at
20 so all 14 action values can take control together. Stop the policy and
confirm `active=false` before classical motion. This is an explicit change from
the initial priority-10 proposal, which conflicts with latched gripper targets.
Preempted gripper tasks discard their old target rather than closing again
when the policy stops.

Evaluate actual physical outcomes:

```bash
MUJOCO_GL=egl uv run python -m dimos.robot.manipulators.dual_openyam.tool_evaluate_policy \
  --artifact outputs/dual-openyam-act/checkpoints/last/pretrained_model \
  --episodes 10 --seed 1000 --report outputs/openyam-policy-evaluation.jsonl
```

The evaluator checks a real lift, an open gripper, and stable bin containment and reports every
trial. Starting a rollout, accepting action chunks, or low training loss does
not establish pick-and-place success. The stage threshold is 7/10; below that,
keep ACT as an explicitly experimental bonus.

## Implementation boundaries

`plan_pose_sequence` plans every candidate waypoint from the preceding endpoint
before exposing an executable trajectory. Candidate fallback happens only on
planning failure. Measured tool settling is enabled for this sim so fingers do
not close while the arm is still approaching. Tilted transfer/release poses
and elevated waypoints avoid the unreachable upright pose over the bin.

The isolated policy process inherits the host's resolved global configuration
before opening transports. It preserves the declaration's stream types and
uses the newest complete, fresh camera/state set when the latest camera batch
is still arriving. Observation age and alignment limits remain enforced.

Pim's replacement scene has not been supplied. Swapping it requires renewed
FK, reachability, contact, camera, and physical acceptance checks. The current
camera layout and scripted reset flow are available for the composed scene.
