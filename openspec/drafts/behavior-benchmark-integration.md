# Draft Plan: BEHAVIOR Benchmark Integration

Status: exploratory draft, not an active OpenSpec change
Date: 2026-06-24

## Goal

Integrate Stanford BEHAVIOR / OmniGibson as an agentic benchmarking target for DimOS while keeping BEHAVIOR's heavy simulator dependencies isolated from the normal DimOS development environment.

The first useful milestone is not full manipulation-stack integration. It is:

> BEHAVIOR evaluator can run reproducibly, call a DimOS-controlled policy, and produce parseable metrics, JSON logs, and rollout videos.

After that works, the integration can climb toward DimOS agents, skills, the control coordinator, and eventually manipulation planning.

## Core Architecture

Treat BEHAVIOR as an external benchmark runtime connected through its official websocket policy interface.

```text
┌──────────────────────────────┐
│ BEHAVIOR / OmniGibson env     │
│ Isaac Sim + tasks + metrics   │
│                              │
│  eval.py policy=websocket     │
└──────────────┬───────────────┘
               │ obs dict
               ▼
┌──────────────────────────────┐
│ DimOS BEHAVIOR policy bridge │
│ websocket server/client shim │
└──────────────┬───────────────┘
               │ normalized obs / action request
               ▼
┌──────────────────────────────┐
│ DimOS agentic stack           │
│ agent / skills / planner /    │
│ coordinator adapters          │
└──────────────┬───────────────┘
               │ action tensor
               ▼
┌──────────────────────────────┐
│ BEHAVIOR evaluator            │
│ rollout JSON + mp4 + score Q  │
└──────────────────────────────┘
```

## BEHAVIOR Facts That Shape the Plan

- BEHAVIOR uses OmniGibson.
- OmniGibson runs on Isaac Sim / Omniverse / PhysX.
- OmniGibson is a synchronous, MDP-style simulator interface layered over Isaac Sim.
- BEHAVIOR tasks are defined in BDDL with object scope, initial conditions, and goal conditions.
- BEHAVIOR evaluation supports a websocket policy mode:

```bash
python OmniGibson/omnigibson/learning/eval.py \
  policy=websocket \
  log_path=$LOG_PATH \
  task.name=$TASK_NAME \
  env_wrapper._target_=$WRAPPER_MODULE
```

- Default wrapper is `omnigibson.learning.wrappers.RGBLowResWrapper`.
- Other documented wrappers include `DefaultWrapper`, `HeavyRobotWrapper`, and `RichObservationWrapper`.
- Primary metric is task success score `Q`.
- Partial success is satisfied goal predicates divided by total goal predicates.
- Evaluation convention is 1 rollout per task instance, commonly first 10 extra instances per task for self-evaluation.
- Full evaluation can produce up to 500 JSON/video outputs for 50 tasks × 10 instances.

## Dependency Strategy

Do not add OmniGibson / Isaac Sim / BEHAVIOR as normal DimOS dependencies.

Use three layers:

### 1. DimOS Core Environment

Normal `uv` environment. Should stay free of direct Isaac Sim / OmniGibson imports.

### 2. BEHAVIOR Environment

External conda environment, likely named `behavior`, created by the BEHAVIOR setup script.

BEHAVIOR documented requirements:

| Requirement | Value |
|---|---|
| OS | Ubuntu 20.04+ or Windows 10+ |
| RAM | 32GB+ |
| GPU | NVIDIA RTX 2070+ |
| VRAM | 8GB+ |
| Runtime | OmniGibson on Isaac Sim / Omniverse |

Expected setup shape:

```bash
git clone -b v3.7.2 https://github.com/StanfordVL/BEHAVIOR-1K.git
cd BEHAVIOR-1K
./setup.sh --new-env --omnigibson --bddl --dataset --eval \
  --accept-conda-tos --accept-nvidia-eula --accept-dataset-tos
conda activate behavior
```

Known install/runtime gotchas:

- First OmniGibson import can take several minutes.
- CuRobo/primitives can need matching CUDA toolkit/compiler configuration.
- Renderer failure like `HydraEngine rtx failed creating scene renderer` can indicate the wrong GPU; use `OMNIGIBSON_GPU_ID=<id>`.
- Docker installation is documented as temporarily unavailable for full simulator setup.
- BEHAVIOR's challenge Docker policy mode should not launch Isaac Sim inside the policy container; the evaluator runs OmniGibson outside and connects by websocket.

### 3. Bridge Protocol

The stable boundary should be websocket messages plus file-based metrics outputs:

- observation dict from BEHAVIOR evaluator
- action tensor/vector returned to BEHAVIOR
- reset / episode lifecycle events
- timeout and reconnect behavior
- log path for JSON and MP4 outputs

## Current DimOS Integration Seams

Relevant repo seams identified during exploration:

- `dimos/manipulation/manipulation_module.py`
  - `execute_plan()` projects a `GeneratedPlan` into per-robot `JointTrajectory` objects.
  - It dispatches execution through coordinator `task_invoke(task_name, "execute", {"trajectory": trajectory})`.
  - It assumes current joint state, global joint names, local robot joint names, and trajectory generation.

- `dimos/control/coordinator.py`
  - `task_invoke(task_name, method, kwargs)` is the stable control RPC seam.
  - Also exposes task lifecycle/introspection methods like `list_tasks()` and `get_active_tasks()`.

- `dimos/control/tasks/trajectory_task/trajectory_task.py`
  - `execute(trajectory)` validates duration, points, and exact joint-name compatibility.

- `dimos/robot/manipulators/xarm/blueprints/simulation.py`
  - Existing sim pattern composes `MujocoSimModule`, `PickAndPlaceModule`, `ObjectSceneRegistrationModule`, `ControlCoordinator`, and trajectory tasks.

- `dimos/navigation/nav_3d/evaluator/evaluator.py`
  - Useful evaluator-loop pattern: publish scenario inputs, wait for outputs, log results.

## Phased Plan

### Phase 0: BEHAVIOR Dependency Spike

Purpose: prove BEHAVIOR can run independently on target hardware.

Tasks:

- Install BEHAVIOR into an isolated conda environment.
- Verify Isaac Sim / OmniGibson startup.
- Run a minimal OmniGibson robot example.
- Run BEHAVIOR evaluation with a dummy websocket policy.
- Record exact compatibility matrix:
  - OS
  - Python version
  - Isaac Sim version
  - OmniGibson / BEHAVIOR branch or tag
  - CUDA toolkit/compiler version
  - NVIDIA driver and GPU

Success criterion:

```bash
conda activate behavior
python -m omnigibson.examples.robots.robot_control_example --quickstart
python OmniGibson/omnigibson/learning/eval.py \
  policy=websocket \
  log_path=/tmp/behavior_eval \
  task.name=turning_on_radio
```

with a dummy policy producing legal actions and evaluator outputs.

### Phase 1: Black-Box Evaluator Runner

Purpose: make DimOS able to supervise BEHAVIOR runs without importing OmniGibson.

Responsibilities:

- Start a policy bridge process.
- Launch BEHAVIOR `eval.py` as a subprocess in the BEHAVIOR conda env.
- Pass task name, wrapper, log path, websocket host/port, and config overrides.
- Wait for completion.
- Collect stdout/stderr, JSON metrics, and MP4 videos.
- Produce a run summary.

Conceptual run config:

```yaml
benchmark: behavior
behavior_root: /path/to/BEHAVIOR-1K
conda_env: behavior
tasks:
  - turning_on_radio
wrapper: omnigibson.learning.wrappers.RGBLowResWrapper
policy_backend: zero
instances: first_10
log_path: /tmp/dimos_behavior_runs/run_001
port: 8080
max_steps: null
```

### Phase 2: Policy Bridge

Purpose: translate BEHAVIOR observations and action requests into DimOS policy calls.

Observation path:

```text
BEHAVIOR obs dict
  ├── rgb / depth / segmentation depending on wrapper
  ├── proprioception
  ├── need_new_action
  └── optional task or privileged info
        ▼
DimOS normalized observation
        ▼
DimOS policy backend
```

Action path:

```text
DimOS policy decision
        ▼
BehaviorActionAdapter
        ▼
BEHAVIOR action tensor/vector
```

The bridge should preserve BEHAVIOR websocket policy semantics:

- `obs["need_new_action"] == False` can reuse the last action.
- Reset should clear cached action and policy state.
- Host/port should be configurable; avoid default port 80.
- Reconnect/timeout behavior should be explicit.

### Phase 3: Baseline Policies

Purpose: validate benchmark plumbing before involving LLMs or planning.

Suggested baselines:

- zero action policy
- random legal action policy
- scripted smoke policy for one simple task
- trace-recording policy that logs observation schema and action dimensions

These establish action-space correctness, evaluator stability, and metrics ingestion.

### Phase 4: DimOS Agent Backend

Purpose: run an agentic DimOS policy inside the benchmark loop.

Likely shape:

```text
BEHAVIOR evaluator
  ⇄ websocket
BehaviorPolicyBridge
  ⇄ DimOS agent backend
  ⇄ skills / perception summaries / planner calls
```

Important concerns:

- LLM latency versus simulator action frequency.
- Use cached actions or low-level controllers between high-level decisions.
- Record agent traces alongside BEHAVIOR metrics.
- Keep wrapper/track explicit: RGB-only, standard observations, or privileged observations.

### Phase 5: Control Coordinator Adapter

Purpose: optionally reuse DimOS control tasks and coordinator semantics.

Two possible adapter styles:

#### Option A: Direct Action-Space Adapter

DimOS policy emits high-level intent or controller targets, and adapter directly returns BEHAVIOR action vectors.

Pros:

- fastest path to valid benchmark runs
- avoids pretending BEHAVIOR is the existing xArm/MuJoCo stack
- keeps evaluator-compatible action semantics central

Cons:

- less reuse of `ControlCoordinator`
- weaker comparison with existing trajectory execution stack

#### Option B: Virtual Hardware Adapter

Represent BEHAVIOR robot control channels as DimOS hardware/components and route through `ControlCoordinator`.

```text
ControlCoordinator
  ├── base task
  ├── torso task
  ├── left arm task
  ├── right arm task
  └── gripper tasks
        ▼
BehaviorHardwareAdapter
        ▼
BEHAVIOR action tensor
```

Pros:

- reuses coordinator/task architecture
- could support `task_invoke`, `get_joint_positions`, and trajectory tasks
- aligns with existing manipulation abstractions

Cons:

- substantial joint/action mapping work
- BEHAVIOR robot/controller config must be pinned
- exact joint names and dimensions are critical
- synchronous policy stepping differs from normal robot control loops

Recommendation: start with Option A, then evolve toward Option B if benchmarked algorithms need coordinator semantics.

### Phase 6: ManipulationModule / Planner Integration

Purpose: bring current manipulation planning into BEHAVIOR once the benchmark loop and action bridge are proven.

This is the hardest phase. Current `ManipulationModule.execute_plan()` assumes:

```text
GeneratedPlan
  └── path: list[JointState]
        ├── global joint names
        └── positions
```

BEHAVIOR requires additional bridges:

| Needed bridge | Challenge |
|---|---|
| BEHAVIOR objects → DimOS world monitor | IDs, poses, categories, articulations |
| BDDL goals → DimOS skill/planning goals | symbolic predicates vs manipulation primitives |
| BEHAVIOR robot model → DimOS robot config | R1Pro/mobile manipulator differs from xArm |
| GeneratedPlan → BEHAVIOR action tensor | trajectory timing and controller mismatch |
| Base/torso/dual-arm/gripper coordination | action vector must encode all channels |

Do not make this the first integration target.

## Benchmark Output Layout

Suggested DimOS-managed layout:

```text
runs/
  behavior/
    2026-06-24-<run-id>/
      config.yaml
      env.json
      stdout.log
      stderr.log
      tasks/
        turning_on_radio/
          rollout_000.json
          rollout_000.mp4
          ...
      summary.json
      summary.md
```

Summary should include:

- task names
- instance IDs / seeds / config indices
- wrapper
- policy backend
- action-space config
- score `Q`
- partial success predicates
- simulated time
- base distance
- end-effector displacement
- timeout or failure reason
- links to videos

## Risks and Mitigations

| Risk | Concern | Mitigation |
|---|---|---|
| Isaac Sim dependency weight | normal CI/dev env cannot run it | isolate in conda env and self-hosted GPU path |
| Port 80 default | root permission / conflicts | override websocket config to high port |
| Robot mismatch | DimOS xArm stack differs from BEHAVIOR R1Pro | start with action tensor adapter |
| LLM latency | policy may be slower than simulator loop | cache actions, decouple high-level decisions from low-level control |
| Observation mismatch | different wrappers imply different benchmark tracks | pin wrapper and label standard vs privileged track |
| Action-space mismatch | controller config determines action dimensions | pin robot controller config and record it in run metadata |
| Metrics reproducibility | task instances/seeds matter | record task, instance id, config overrides, env version |
| ManipulationModule assumptions | expects joint trajectory plans | defer until object/world/robot adapters exist |

## Things Not To Do First

- Do not add OmniGibson as a normal DimOS dependency.
- Do not treat BEHAVIOR as a drop-in `MujocoSimModule` replacement.
- Do not force BEHAVIOR through `ManipulationModule.execute_plan()` before action and world adapters exist.
- Do not hide adapters outside the benchmarked path.
- Do not mix RGB-only, standard, and privileged wrappers without explicit labels.

## Candidate Future OpenSpec Change

If this becomes implementation work, create a change such as:

```text
add-behavior-benchmark-integration
```

Likely artifact scope:

1. Dependency isolation and runbook
2. Evaluator runner
3. Websocket policy bridge
4. Baseline policies
5. Result ingestion and summaries
6. DimOS agent backend
7. Future coordinator/manipulation adapter design

## Open Questions

- Which BEHAVIOR version/tag should be the initial target: stable release `v3.7.2` or main?
- Which GPU machines will run dependency spike and future benchmarks?
- Should first DimOS policy bridge run inside the BEHAVIOR conda env or outside it with a reimplemented websocket protocol?
- Which wrapper should be the default benchmark mode: `RGBLowResWrapper`, `DefaultWrapper`, or custom?
- Do we care first about challenge-compatible scoring, internal development benchmarking, or both?
- What level of agentic control is the first benchmark target: direct action policy, skill policy, coordinator-backed policy, or full manipulation planner?
- How should BEHAVIOR observations be persisted for debugging without creating huge logs?
- What is the minimum smoke task that can verify a non-zero action policy?
