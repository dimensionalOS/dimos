## Context

The archived LeRobot LIBERO rollout change added a successful policy benchmark path, but the current `RobotPolicyModule` is a plain Python class constructed directly by the rollout script. That helped move quickly, but it conflicts with DimOS naming and composition expectations: a DimOS module is a `dimos.core.module.Module` with blueprint construction, RPC lifecycle, and worker-process deployment semantics.

The current benchmark also passes a benchmark-specific `RuntimeObservationSample` into policy inference and receives a `RuntimeActionFrame` back. That makes the policy component harder to reuse for future simulator and real-robot rollout, where observations will come from DimOS streams or temporal assemblers and actions may go to ControlCoordinator or another action-surface executor rather than a runtime sidecar.

## Goals / Non-Goals

**Goals:**

- Make `RobotPolicyModule` a real DimOS `Module` configured through blueprint kwargs.
- Select policy backends and robot policy contracts through lazy registries, following the control task registry pattern.
- Define `RobotLearningSample` as the reusable public policy input boundary.
- Define `RobotPolicyAction` as the reusable public policy output boundary.
- Keep the LIBERO benchmark lockstep and sidecar-native, but have benchmark evaluation call DimOS module RPCs rather than plain service methods.
- Preserve the existing 50-episode LeRobot LIBERO gate semantics and artifact expectations.

**Non-Goals:**

- Do not add ControlCoordinator action-surface execution in this change.
- Do not introduce streaming policy inference for v1 evaluation.
- Do not change the official `lerobot/VLA-JEPA-LIBERO` checkpoint, suite, episode matrix, or `success_rate > 0.50` gate.
- Do not add a new optional dependency beyond the existing LeRobot/LIBERO rollout dependencies.

## Decisions

### 1. Convert `RobotPolicyModule` in place

`RobotPolicyModule` SHALL become a subclass of `dimos.core.module.Module` rather than being renamed to a separate engine class.

Rationale: the user-facing name should match the DimOS concept. Adding a second engine class would preserve testability, but it would also preserve ambiguity around which object owns policy lifecycle. The module can still keep private helper methods for unit-testable behavior.

### 2. Use lazy registries for backend and contract selection

The module config selects `backend_type`, `backend_params`, `contract_type`, and `contract_params`. Registries map those names to lazy factory import paths.

This follows the control task registry style rather than the eager hardware adapter registry style because policy backends may import heavy or optional libraries such as LeRobot, torch, and checkpoint-specific dependencies.

Conceptual config:

```python
class RobotPolicyModuleConfig(ModuleConfig):
    backend_type: str = "lerobot"
    backend_params: dict[str, object] = {...}
    contract_type: str = "vla_jepa_libero"
    contract_params: dict[str, object] = {}
```

### 3. Use RPC-only policy inference for v1 evaluation

For the benchmark path, `BenchmarkPolicyEvalModule` calls `RobotPolicyModule.infer_action(sample)` by RPC. Streams can be added later for online rollout, but v1 evaluation is lockstep:

```text
observation -> infer one action -> step environment -> next observation
```

RPC keeps timeout/error handling simple and avoids correlation IDs, action queues, and stale-action backpressure semantics.

### 4. Replace benchmark-shaped sample with `RobotLearningSample`

`RobotPolicyModule` consumes a reusable `RobotLearningSample`, not `RuntimeObservationSample`. The sample is role-keyed and runtime-independent. Producers may include benchmark evaluation modules, future temporal sample assemblers, replay loaders, or other simulator adapters.

The LIBERO benchmark module remains responsible for converting runtime sidecar observations and payloads into `RobotLearningSample` for v1.

### 5. Replace runtime frame output with `RobotPolicyAction`

`RobotPolicyModule` returns a runtime-independent `RobotPolicyAction`, not `RuntimeActionFrame`. Benchmark evaluation adapts `RobotPolicyAction` into `RuntimeActionFrame` before calling the sidecar. Future real-robot execution can adapt the same policy action into a ControlCoordinator action-surface command or other execution artifact.

### 6. Add module-backed benchmark evaluation while preserving script convenience

The script remains a developer entrypoint, but it should construct and run a module-backed evaluation composition. The DimOS-shaped flow is:

```text
BenchmarkPolicyEvalModule
  -> LiberoRuntimeModule.reset/step
  -> builds RobotLearningSample
  -> RobotPolicyModule.infer_action(sample)
  -> adapts RobotPolicyAction to RuntimeActionFrame
  -> LiberoRuntimeModule.step(action)
```

This keeps benchmark lifecycle outside the policy module while making policy inference a real module participant.

## Risks / Trade-offs

- **Registry indirection can hide errors until runtime** → fail fast with clear unknown-type messages and expose available registry keys.
- **RPC-only inference is less future-real-time than streams** → explicitly scope streams to later online rollout; keep method names and data models stream-compatible.
- **`RobotLearningSample` can become too generic** → keep it role-keyed but narrow for v1, and let robot policy contracts validate required roles.
- **Script convenience may still look non-blueprint-like** → ensure the script builds the same modules/config that a blueprint would use, even if it runs them locally for benchmark convenience.

## Migration Plan

1. Introduce reusable `RobotLearningSample` and `RobotPolicyAction` models.
2. Convert `RobotPolicyModule` into a DimOS `Module` with config and RPC methods.
3. Add backend and contract registries plus default LeRobot/VLA-JEPA LIBERO registrations.
4. Update benchmark evaluation to build `RobotLearningSample` internally and adapt `RobotPolicyAction` to `RuntimeActionFrame`.
5. Add a module-backed benchmark evaluation module or equivalent blueprint-compatible module composition.
6. Update the rollout script to use the module-backed flow while preserving CLI behavior and artifacts.
7. Update tests and docs, then rerun OpenSpec and targeted validation.

## Open Questions

- Whether the module-backed benchmark runner should run inside a deployed blueprint for every test, or expose a local harness that instantiates modules directly while preserving the same config/RPC boundaries.
- Whether the first stream-based policy interface should be added in this change as disabled/non-used scaffolding, or deferred entirely until online rollout work.
