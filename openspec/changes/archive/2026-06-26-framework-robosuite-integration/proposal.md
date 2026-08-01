## Why

DimOS can compose robot modules, control hardware through the ControlCoordinator, and expose skills to agents, but it does not yet have a backend-neutral way to run benchmark simulator episodes from isolated environments. Robosuite is the first concrete backend target because it provides baked tabletop manipulation tasks while requiring simulator-specific dependencies and runtime behavior that should not be forced into the main DimOS process.

## What Changes

- Add a backend-neutral runtime sidecar framework for benchmark episodes.
- Add a lightweight shared protocol package that can be installed by both DimOS and simulator sidecars without installing the full DimOS package.
- Add a prelaunch orchestration path that starts the simulator sidecar environment, reads live runtime metadata, derives DimOS hardware/module launch material, and then launches a DimOS blueprint.
- Add a local DimOS control bridge where the ControlCoordinator talks to a WholeBodyAdapter through a local SHM motor data plane, while DimOS talks to the sidecar through a network protocol.
- Add a Robosuite sidecar integration for baked Robosuite tasks such as Panda Lift, deriving the motor surface and observation streams from the live Robosuite environment.
- Add two script-based E2E demos: a fake sidecar smoke demo and a Robosuite Panda Lift plumbing demo.
- Do not add a `dimos` CLI command in this change.
- Do not require an LLM/agent task-success demo in this change.

## Capabilities

### New Capabilities
- `runtime-sidecar-protocol`: Shared remote runtime protocol schemas, codecs, compatibility rules, and network session semantics for DimOS-to-sidecar communication.
- `benchmark-prelaunch-orchestration`: Benchmark episode config resolution, sidecar startup, live metadata discovery, resolved runtime plan generation, DimOS blueprint launch, monitoring, teardown, and artifact ownership.
- `robosuite-runtime-sidecar`: Robosuite sidecar package and mapping layer for baked Robosuite tasks, whole-body motor surfaces, network step/reset/score operations, and observation export.
- `scripted-runtime-demos`: Script-based E2E demos that verify the fake sidecar and Robosuite Panda Lift plumbing without packaging a new DimOS CLI command or requiring LLM task success.

### Modified Capabilities
- None.

## Impact

- Adds monorepo package projects under `packages/` for the shared runtime protocol and backend sidecars.
- Adds DimOS-side runtime client, prelaunch, resolved-plan, local SHM bridge, and WholeBodyAdapter integration code.
- Adds optional Robosuite-side dependencies isolated to the Robosuite sidecar package, not the main DimOS package.
- Adds plain demo scripts and benchmark configs for fake sidecar and Robosuite Panda Lift plumbing demos.
- Produces benchmark artifacts such as episode config, sidecar metadata, resolved runtime plan, protocol trace summary, motor trace, observations, score output, and logs.
