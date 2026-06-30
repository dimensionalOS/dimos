## Context

DimOS currently has strong runtime pieces for modules, blueprints, ControlCoordinator hardware abstraction, whole-body adapters, MCP skills, and run artifacts. It does not yet have a benchmark runtime boundary that can connect those pieces to simulator backends that run in isolated Python environments or on separate machines.

Robosuite is the first backend because it provides baked manipulation scenes such as `Lift`, `Stack`, and `PickPlace` via `robosuite.make(...)`. A benchmark should reference those baked tasks rather than manually spelling scene geometry, robot placement, or Robosuite action indices in DimOS configs. The integration must also preserve dependency isolation: sidecars cannot require installation of the full `dimos` package because Robosuite, LIBERO-PRO, and BEHAVIOR/OmniGibson may have conflicting dependencies.

The working architecture separates three boundaries:

1. Remote runtime boundary: network protocol between DimOS and simulator sidecar.
2. Local motor bridge: local SHM between the DimOS simulator client module and a ControlCoordinator-facing WholeBodyAdapter.
3. Benchmark orchestration boundary: prelaunch runner owns both sidecar and DimOS blueprint lifetimes.

## Goals / Non-Goals

**Goals:**

- Define a backend-neutral runtime protocol shared by DimOS and sidecars.
- Package that protocol as a lightweight monorepo package that does not import DimOS or simulator SDKs.
- Add a prelaunch orchestration flow that starts the sidecar first, asks it to describe the live runtime, derives DimOS launch material, then starts a blueprint.
- Add DimOS-side runtime client components that translate between protocol frames, local SHM motor frames, and DimOS observation streams.
- Add a Robosuite sidecar package that owns `env.reset()` / `env.step()` and maps Robosuite observations/actions to protocol frames.
- Verify the framework with two script-based demos: fake sidecar smoke test and Robosuite Panda Lift plumbing test.

**Non-Goals:**

- No new `dimos benchmark` CLI command.
- No LLM/agent task-success benchmark in this change.
- No code-as-policy sandbox.
- No broad benchmark taxonomy or leaderboard.
- No requirement that Robosuite sidecar run in the same environment or on the same machine as DimOS.
- No SHM across the remote sidecar boundary.

## Decisions

### Decision: Use first-class monorepo packages for protocol and sidecars

The shared runtime protocol will live in a separate lightweight package, for example `packages/dimos-runtime-protocol`, with a neutral import path such as `dimos_runtime_protocol`. Backend sidecars will also be first-class packages under `packages/`, each with its own `pyproject.toml`, `src/`, and tests.

Rationale: sidecars need protocol types without importing the full DimOS package. Separate package projects preserve monorepo development while allowing isolated virtual environments and backend-specific dependency sets.

Alternatives considered:

- Put protocol under `dimos/simulation/runtime_protocol`: rejected because sidecars would need to install/import DimOS.
- Use loose scripts plus requirements files: rejected because protocol compatibility, packaging, and test isolation become ad hoc.

### Decision: Use Pydantic protocol models plus binary-friendly codec

Protocol messages will be Pydantic models for metadata, envelopes, robot surfaces, actions, states, observations, scores, and errors. Transport payloads may use msgpack or another binary-friendly codec, with Pydantic validating after decode and before encode.

Large arrays and images must not be represented as nested JSON lists in normal operation. Protocol envelopes may carry small numeric arrays directly for motor state/action, and image/depth data should use binary payloads or references with metadata such as shape, dtype, encoding, and stream name.

Rationale: Pydantic gives a shared contract and compatibility checks while avoiding simulator-specific SDK objects at the boundary.

Alternatives considered:

- Raw untyped dicts: rejected because client and sidecar would drift.
- Protobuf first: deferred because Pydantic is already a core dependency and easier to iterate during v1.

### Decision: Keep the remote sidecar boundary network-first

The remote boundary between `SimConnectionModule` and runtime sidecar is a network protocol. The sidecar may run in a separate virtual environment, container, host, or benchmark-managed process. It owns the simulator environment and calls backend-native reset/step APIs.

Rationale: BEHAVIOR/OmniGibson uses a remote websocket policy pattern where evaluator/simulator and policy server communicate over the network. Robosuite and LIBERO-PRO should not be designed around local shared memory assumptions.

Alternatives considered:

- Direct SHM between sidecar and hardware adapter: rejected because it assumes same machine/process environment and confuses remote simulator protocol with local control plumbing.
- In-process simulator import from DimOS: rejected for dependency isolation.

### Decision: Keep SHM local to DimOS control plumbing

Local SHM is used only between DimOS runtime client code and a ControlCoordinator-facing `WholeBodyAdapter`. The adapter exposes `MotorState[]` and accepts `MotorCommand[]`; the runtime client translates those local frames to/from remote protocol frames.

Rationale: ControlCoordinator adapters are synchronous hardware surfaces. A local SHM bridge lets the coordinator interact with simulated motors without making the coordinator depend on network protocol details.

Alternatives considered:

- ControlCoordinator directly calls network protocol: rejected because network behavior should not leak into hardware adapter contracts.
- Sidecar creates SHM directly: rejected because remote sidecar may not be local.

### Decision: Prelaunch orchestrator owns both runtimes

The benchmark prelaunch/runner starts the sidecar, waits for health/metadata, derives a resolved runtime plan, starts the DimOS blueprint, monitors both processes, collects artifacts, and tears both down.

Rationale: benchmark episodes need one outer owner for cleanup, failure attribution, artifact collection, and repeatability. `SimConnectionModule` is a sidecar client, not a sidecar process owner.

Alternatives considered:

- Let `SimConnectionModule` adopt sidecar lifetime: rejected because lifecycle and artifact ownership would be split across environments.
- Treat sidecar as a preexisting daemon only: deferred as a later deployment option.

### Decision: Robosuite configs name baked scenes and robot profiles

User-authored benchmark configs name Robosuite baked tasks such as `env_name: Lift`, `robots: Panda`, controller profile, horizon, control frequency, seed, and desired observation streams. They do not normally enumerate every motor-to-action index. The sidecar constructs the live environment, discovers the robot motor surface and action/observation layout, and reports that description to the resolver.

Rationale: Robosuite builds scenes by instantiating environment classes through `robosuite.make(...)`; scene assets, objects, robot placement, reward, and success logic are part of the task environment.

Alternatives considered:

- Manual `motor_map` in every task config: rejected as too bulky and brittle. Manual mapping may remain an escape hatch for unsupported robots/controllers.

### Decision: Demos are script-based and plumbing-oriented

The change will add plain scripts that orchestrate the fake sidecar and Robosuite Panda Lift demos. Scripts may call or build blueprints directly. No `dimos` CLI command is introduced.

Rationale: the first acceptance target is proving the architecture, not creating a stable public user interface.

Alternatives considered:

- Add `dimos benchmark run`: rejected as premature.
- Require an agent/LLM to solve a task: rejected because it tests agent capability rather than runtime plumbing.

## Risks / Trade-offs

- Backend dependency conflicts → isolate each sidecar in its own package and environment; keep protocol package dependency-light.
- Robosuite action semantics vary by controller → require sidecar runtime description and profile validation; support manual overrides only as escape hatches.
- Network latency can disturb high-rate control → treat v1 demos as plumbing validation; record requested/actual timing and command latency in artifacts.
- Duplicated package version skew between DimOS and sidecar → include protocol version and compatibility checks in handshake.
- Large observations can overload simple JSON payloads → use binary-friendly transport and references for image/depth frames.
- Fake sidecar may give false confidence → require both fake-sidecar smoke demo and real Robosuite Panda Lift plumbing demo.
- Prelaunch orchestration can leave orphan processes → runner must own process groups, health checks, teardown, and failure artifacts.

## Migration Plan

This change is additive. Existing DimOS blueprints, hardware adapters, and CLI commands remain unchanged.

Implementation order:

1. Add shared protocol package and compatibility tests.
2. Add fake sidecar and DimOS runtime client plumbing.
3. Add prelaunch/resolved-plan scaffolding and fake sidecar demo script.
4. Add Robosuite sidecar package and Robosuite mapping/profile support.
5. Add Robosuite Panda Lift demo script and artifact checks.
6. Update the existing roadmap draft to reference this change as the first concrete implementation slice.

Rollback is removing the new packages, demo scripts, and change-specific configs; no existing runtime behavior is replaced.

## Open Questions

- Exact transport library choice for v1 websocket/msgpack implementation.
- Exact blueprint shape used by the scripts during demos.
- Whether the Robosuite sidecar should support remote preexisting endpoints in addition to subprocess launch in the first implementation.
- Final artifact directory naming and trace summarization format.
