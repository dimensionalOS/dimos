## Context

`keyboard-teleop-a1z` currently composes keyboard Cartesian twist input, the control coordinator, and mock A1Z hardware. Its `EEFTwistTask` owns the six arm joints but has no gripper input. The generic MuJoCo bridge already supports a single scalar gripper immediately after the arm degrees of freedom, but the vendor A1Z description supplies fixed visual fingers rather than an actuated gripper or MJCF scene.

Stage 1 is a standalone simulated operator workflow. It must establish the physical/control interfaces that Stage 2 recording will consume without adding episode state, persistence, or reset automation now.

## Goals / Non-Goals

**Goals:**
- Run the existing A1Z keyboard-teleop blueprint in a deterministic MuJoCo tabletop scene when `--simulation` is enabled.
- Preserve six-joint EEF-twist control and add independently arbitrated, latched gripper endpoints.
- Provide a wrist RGB view, a visible cube, and collision geometry sufficient for manual contact testing.
- Keep the non-simulation path compatible with the existing blueprint behavior.

**Non-Goals:**
- Physical A1Z gripper fidelity or a hardware gripper API.
- Episode keys, automatic reset, recording, SQLite persistence, replay datasets, randomization, or success metrics.
- New skills, MCP tools, ROS dependencies, or a separately named simulation blueprint.

## DimOS Architecture

The existing keyboard module continues to publish `TwistStamped` for EEF motion. It gains a partial `JointState` output for endpoint changes only, naming `arm/gripper`; `[` publishes the open position and `]` the closed position. The keyboard module retains the selected endpoint so the command is latched rather than repeated while a key is held.

`ControlCoordinator` continues to route the twist stream to `EEFTwistTask`, which claims only the six arm joints. A built-in `JointServoTask` configured for `arm/gripper` receives the partial joint command from the coordinator's generic joint-command stream, holds its target indefinitely, and claims only that joint. The claims are disjoint, so the existing task arbitration remains authoritative.

The `keyboard-teleop-a1z` blueprint explicitly selects either the current mock-backed composition or a simulation composition from the resolved `--simulation` configuration. The simulation composition includes `MujocoSimModule` and A1Z hardware configured with the MuJoCo shared-memory adapter at the same address. It uses the same existing CLI blueprint name, so no registry addition is expected unless discovery output changes.

The A1Z scene is an asset-relative MuJoCo XML scene derived from the local A1Z description. Its seven mapped actuators are ordered as the six arm joints followed immediately by the driver prismatic gripper joint; the passive follower finger is coupled by an MJCF equality. The abstract `arm/gripper` value is the driver displacement in meters: `0.0` closed and `0.015` open. The A1Z adapter uses an explicit identity gripper-command mapping so command and reported driver position share those raw units. Each finger has an explicit visible physical A1Z pad insert with matching render and collision geometry; the inserts are simulation fixtures rather than vendor CAD, added so the visible contact surface is the same surface used for manual contact tests. A fixed wrist camera publishes RGB at 640×480 and 30 FPS.

No new DimOS `Spec` Protocol, adapter Protocol, RPC method, skill, or MCP surface is required. The existing generic simulator adapter interface and coordinator joint command contract remain in use.

## Decisions

- **Preserve `EEFTwistTask` for arm control.** Replacing it with the Quest-oriented `TeleopIKTask` would change keyboard Cartesian behavior and requires its full hand/Buttons input model.
- **Use `JointServoTask` for the gripper.** It is the existing scalar joint-control primitive, maintains coordinator arbitration, and supports a held endpoint target. Direct coordinator gripper setters are reserved for one-shot controls because they bypass task state.
- **Model one driver plus one mimic follower.** This matches the generic bridge's one-scalar-gripper assumption. The URDF records the follower's mimic intent, while conversion adds the equivalent MuJoCo equality because the native importer does not preserve it.
- **Use seven actuator mappings but retain `dof=6`.** The generic bridge treats actuator mapping index 6 as the scalar gripper. Every arm joint must therefore have an ordered actuator before the driver, while the follower remains passive.
- **Configure identity gripper mapping for A1Z.** Existing inverted-gripper behavior remains the generic default for compatibility. A1Z opts into identity mapping so its public raw-displacement semantics remain `0.0` closed and `0.015` open.
- **Use a deterministic heuristic scene.** The scene has a table-edge mount, fixed lighting, one 5 cm cube, heuristic collision pads, and an offset wrist camera. Provisional transforms and home joints will be refined by manual simulation runs rather than treated as a vendor-validated setup.
- **Make physical finger-pad inserts explicit.** The visible physical A1Z pad inserts deliberately share their render and collision geometry. This makes the operator-visible contact surface unambiguous while avoiding a claim that the local A1Z description contains vendor-validated pad CAD.
- **Manual reset only.** Restarting the simulator/process is adequate for Stage 1 and avoids prematurely defining the Stage 2 episode lifecycle.
- **Separate simulator and coordinator workers.** MuJoCo simulator startup must not share a serialized worker with the coordinator, whose adapter waits for the simulator. The composition assigns the simulator dedicated worker capacity/placement.

## Safety / Simulation / Replay

The simulated gripper is not a hardware safety model. Its range is limited to 0–0.015 m and the visible finger-pad inserts must be inspected through a full opening sweep before contact tests. The initial scene/home pose, cube pose, camera transform, and insert transforms are implementation hypotheses that require manual validation for self-collision, table collision, reachability, framing, and cube contact.

`--simulation` must be an explicit composition branch: it is not assumed to automatically replace mock hardware. The non-simulation blueprint remains untouched behaviorally, including the absence of a simulation-only gripper servo task. The simulator receives the same arm home vector as the simulated hardware and planning configuration, and is configured directly for 30 FPS. Replay and recording are intentionally absent from this stage.

## Risks / Trade-offs

- Vendor geometry does not define an actuated gripper or finger-pad inserts, so finger axes, travel, insert shape/placement, and servo gains are heuristic. Manual visual and contact checks mitigate this but do not validate hardware equivalence or simulation-to-hardware/data parity.
- The existing simulator bridge only exposes a single gripper scalar. The coupled model preserves that interface but precludes independently controlled fingers.
- MuJoCo XML asset paths and generated keyframes can break after conversion or relocation. Keep the scene self-contained/path-relative and compile it from its final repository location.
- A provisional home pose or scene transform can collide or leave the cube unreachable. Adjust these based on simulation runs and encode the validated values consistently in the scene and A1Z configuration.

## Migration / Rollout

The existing `keyboard-teleop-a1z` command is preserved. Operators opt into the new behavior with `dimos --simulation run keyboard-teleop-a1z`; without that flag it follows the current mock/hardware configuration. Document the simulation invocation and keyboard gripper keys. Run `pytest dimos/robot/test_all_blueprints_generation.py` if blueprint discovery changes the generated registry, and commit generated output only when the test requires it.

## Open Questions

- The validated joint home vector, table/base transform, cube pose, exact camera transform, and collision-pad offsets will be selected through manual Stage 1 simulation iteration.
- Stage 2 must define episode lifecycle, reset semantics, recording schema, and dataset validation separately.
