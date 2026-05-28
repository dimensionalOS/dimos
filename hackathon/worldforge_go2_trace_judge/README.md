# WorldForge Go2 Trace Judge

Hackathon submission from Omar Espejel, Abdel, and Ciro.

## If You Only Have 60 Seconds

1. Watch the main demo:
   https://github.com/omarespejel/worldforge-go2-trace-judge/blob/main/artifacts/showcase/final_hackathon_video.mp4
2. Watch the short decision-to-simulation clip:
   https://github.com/omarespejel/worldforge-go2-trace-judge/blob/main/artifacts/showcase/sim_decision_trace_video.mp4
3. Inspect one evidence trace:
   https://github.com/omarespejel/worldforge-go2-trace-judge/tree/main/artifacts/replay_mpc_arena/decision_traces/decision_001
4. Open the release bundle:
   https://github.com/omarespejel/worldforge-go2-trace-judge/releases/tag/v0.2-replay-mpc-arena-2026-05-28

## One Sentence

WorldForge Go2 Trace Judge makes Go2 autonomy inspectable: it compares candidate
robot actions, scores their predicted futures, selects one action, and writes a
replayable evidence trail.

```text
robot observation + goal + candidate action
-> score
-> selected action
-> evidence trace
-> DimOS execution or simulation handoff
```

## What Matters

- DimOS remains the robot runtime: camera, odometry, replay, simulation, MCP,
  and execution.
- WorldForge-style code acts as the decision and evidence layer around DimOS.
- The project creates one of the first open Go2 replay datasets shaped for
  world-model/action-scoring research.
- Every demo decision has auditable JSON: what was seen, what was scored, what
  was selected, and what happened afterward.

## What We Built

- Real Unitree Go2 venue captures and robot-view traces.
- Label-preserving counterfactual candidate scenes for cube/marker navigation.
- WorldForge-style decision artifacts:
  - `score_info.json`
  - `candidate_scores.json`
  - `selected_action.json`
  - `outcome_after_execution.json`
  - `run_manifest.json`
- A DimOS replay-derived Hugging Face dataset with 2,557 action-conditioned Go2
  current/future frame pairs from six usable public replay DBs.
- A small frozen-DINOv2 latent dynamics head:
  `DINOv2(current_frame) + egomotion_delta -> future DINOv2 latent`.
- Replay-MPC Arena: 12 selected held-out replay decisions with per-decision
  candidate scores and trace files.
- DimOS MCP/MuJoCo proof that a selected movement can be handed to the simulated
  Go2 runtime.

## Reviewer Map

| Question | Open This |
| --- | --- |
| What is the demo? | https://github.com/omarespejel/worldforge-go2-trace-judge/blob/main/artifacts/showcase/final_hackathon_video.mp4 |
| Does the decision loop reach simulation? | https://github.com/omarespejel/worldforge-go2-trace-judge/blob/main/artifacts/showcase/sim_decision_trace_video.mp4 |
| Where are the score traces? | https://github.com/omarespejel/worldforge-go2-trace-judge/tree/main/artifacts/replay_mpc_arena/decision_traces |
| Where is the dataset? | https://huggingface.co/datasets/espejelomar/worldforge-go2-dimos-replay-world-pairs |
| Where is the model? | https://huggingface.co/espejelomar/go2-dimos-replay-latent-dynamics |
| Where is the full source? | https://github.com/omarespejel/worldforge-go2-trace-judge |
| Where is the release bundle? | https://github.com/omarespejel/worldforge-go2-trace-judge/releases/tag/v0.2-replay-mpc-arena-2026-05-28 |

## Current Results

```text
DimOS replay pairs: 2,557
usable replay sources: 6
validation lift vs no-motion baseline: +0.0507 cosine
test lift vs no-motion baseline: +0.0182 cosine
Replay-MPC Arena: 12 selected held-out decisions with JSON evidence
main video: 58 seconds
decision-to-simulation clip: 23 seconds
```

The Replay-MPC Arena number is demo evidence over selected held-out examples,
not a broad autonomy accuracy claim.

## Why This Fits DimOS

DimOS is already the right place for robot runtime concerns:

```text
streams, replay data, blueprints, simulation, camera frames,
odometry, MCP tools, and Go2 control skills
```

This project adds a thin layer above that runtime:

```text
DimOS observes and executes
WorldForge-style scorer compares candidate actions
trace files explain and replay the decision
```

That boundary keeps physical execution host-owned while making decisions easier
to inspect, debug, compare, and train from.

## Reproduce From The External Project

```bash
git clone https://github.com/omarespejel/worldforge-go2-trace-judge.git
cd worldforge-go2-trace-judge

make check
make replay-mpc-arena
make final-video
python3 scripts/build_sim_decision_video.py
```

The replay dataset/model artifacts are also published on Hugging Face:

- Dataset: https://huggingface.co/datasets/espejelomar/worldforge-go2-dimos-replay-world-pairs
- Model: https://huggingface.co/espejelomar/go2-dimos-replay-latent-dynamics

## Scope Boundary

This PR does not vendor the full external project into DimOS and does not modify
DimOS runtime code. It is a hackathon submission pointer with the full source,
artifacts, dataset, model, and videos hosted externally.

Accurate claim:

```text
small action-conditioned latent world model + WorldForge-style decision traces
around DimOS/Go2 replay and simulation
```

Avoided claims:

```text
Go2 foundation model
trained V-JEPA
safety-certified controller
solved autonomous navigation
```
