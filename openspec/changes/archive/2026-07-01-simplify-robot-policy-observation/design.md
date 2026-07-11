## Context

The policy rollout code introduced a reusable robot policy module, backend registry, LeRobot VLA-JEPA backend, and LIBERO benchmark evaluation runner. Review feedback identified that several seams are still shaped by the first benchmark integration rather than by a narrow policy-module contract: policy inputs carry episode/task-index bookkeeping, backend output accepts arbitrary objects, contract descriptions are produced but not consumed, and the LeRobot backend uses dynamic imports and local Protocols instead of the official LeRobot API surface.

The cleanup must be small enough to land as PR review follow-up. It should not redesign high-throughput image transport or stream sample assembly.

## Goals / Non-Goals

**Goals:**

- Make the policy input boundary observation-focused and free of benchmark lifecycle fields.
- Keep policy language prompts available as contract-specific metadata or observation data, not as a generic top-level `task` field.
- Return a constrained flat numeric backend action tuple before contract conversion.
- Remove unused contract description APIs and artifacts.
- Place backend protocol and LeRobot implementation under the backend package layout expected by review.
- Simplify LeRobot VLA-JEPA loading and processor setup around official top-level imports and factory helpers.

**Non-Goals:**

- No redesign of image transport, LCM payload shape, SHM streaming, or asynchronous sample assembly.
- No new policy families beyond the existing VLA-JEPA LIBERO path.
- No change to benchmark episode matrix, success gate, or runtime sidecar protocol.

## Decisions

1. **Rename the input model to `RobotPolicyObservation`.**
   - Rationale: the module consumes one policy observation at an inference step; the previous `RobotLearningSample` name was broad and encouraged carrying unrelated IDs.
   - Alternative considered: keep `RobotLearningSample` and delete fields. Rejected because the name still suggests a training/sample artifact rather than a policy inference observation.

2. **Remove benchmark identifiers from policy observations.**
   - Rationale: `episode_id`, `task_id`, task/init indices, tick counters, and synthetic sample IDs belong to evaluation requests/records. The policy module should only see observation roles, timestamps, and policy-relevant metadata.
   - Alternative considered: keep the fields as optional metadata for debugging. Rejected because it preserves the coupling called out by review.

3. **Carry language prompt through metadata/observations.**
   - Rationale: language is a policy/contract input for VLA-JEPA, but not a universal base-model field. The VLA-JEPA contract can read `metadata["language"]` or an observation role and raise a clear error when missing.
   - Alternative considered: keep top-level `task`. Rejected because it implies every robot policy observation has a generic task string.

4. **Normalize backend output to `tuple[float, ...]`.**
   - Rationale: the only backend output consumed by the contract is a numeric action vector. Normalizing tensors/arrays/lists at the backend boundary removes arbitrary `object` output and simplifies validation.
   - Alternative considered: generic typed envelopes. Rejected as unnecessary complexity for the current single action-vector path.

5. **Remove contract description APIs and artifacts.**
   - Rationale: contract descriptions are currently only written/tested, not used to drive behavior. Action-space validation remains in the contract itself.
   - Alternative considered: keep descriptions for documentation. Rejected because review requested removal until there is a real consumer.

6. **Use official LeRobot imports in the backend module.**
   - Rationale: current LeRobot exports `VLAJEPAPolicy` and processor helpers through top-level package APIs. Direct imports remove local importlib/protocol indirection and surface optional dependency failures at backend module load time, which remains lazy through the registry.
   - Alternative considered: keep dynamic imports for optional dependency isolation. Rejected because the registry already lazy-loads backend modules and review specifically requested top-level imports unless a concrete issue appears.

## Risks / Trade-offs

- **Breaking internal API rename** → Update all policy rollout tests, registry paths, benchmark runner code, docs, and specs in the same change.
- **Optional LeRobot import errors may occur earlier within backend module import** → Backend modules remain lazy-loaded from the registry, so environments that do not instantiate `lerobot` backend are unaffected.
- **Removing contract descriptions reduces artifact metadata** → Keep backend metadata and runtime/episode artifacts; reintroduce contract descriptions later only when there is a consumer.
- **Skipping image transport design leaves a review concern open** → Reply separately that transport/performance is intentionally deferred to a dedicated design.
