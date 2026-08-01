## MODIFIED Requirements

### Requirement: Typed task and answer contracts
Each generated task SHALL have a stable semantic identity, category, versioned controlled utterance, and declared response or completion type. Private records SHALL use discriminated executable contracts and typed expected answers: terminal navigation predicate, finite enum, integer, or entity choice. A terminal navigation contract SHALL encode every condition required by its controlled language, including metric threshold, stopped-state tolerances, and stationary dwell duration. Expected answers SHALL be computed from the oracle view rather than manually supplied.

#### Scenario: Generate a state question
- **WHEN** the compiler selects an entity-state predicate for a television with authoritative power state `OFF`
- **THEN** it emits public controlled language, a private `entity-state` contract bound to the stable television entity, and a typed enum answer `OFF`

#### Scenario: Generate a destination task
- **WHEN** the compiler selects the bathtub destination predicate
- **THEN** it emits a private terminal contract requiring the robot footprint to remain within 1 metre of the bathtub's outer footprint and below the declared linear and angular speed tolerances for the declared stationary dwell duration, and it does not fabricate a textual expected answer

### Requirement: Destination objectivity gates
A destination candidate SHALL resolve to exactly one target entity, use metric robot-footprint-to-target-outer-footprint surface distance, and have at least one collision-free reachable stopping region satisfying the threshold, stopped-state policy, and configured embodiment clearance. Generation-time feasibility and runtime evaluation SHALL use the same versioned distance semantics. Candidates whose feasible stopping region is absent or uncertain SHALL be rejected.

#### Scenario: Bathtub has a reachable stopping region
- **WHEN** the bathtub resolves uniquely and its one-metre robot-footprint stopping band contains a validated reachable collision-free region
- **THEN** the destination candidate is eligible for retention

#### Scenario: Target is geometrically unreachable
- **WHEN** no validated collision-free robot stopping pose can satisfy the destination threshold
- **THEN** the generator rejects the destination candidate and reports the failed reachability gate

### Requirement: Stable identity and provenance
Task identities SHALL be content-derived from semantic intent rather than expected answer or output ordering. Every private task record SHALL identify the source scene, source oracle-view digest, semantic-schema revision, semantic-profile revision, upstream revision, generator revision, predicate-policy version, and language-template version. Every field affecting executable predicate meaning SHALL participate in semantic identity or a declared policy revision.

#### Scenario: Canonical state answer changes
- **WHEN** the same television-state task is compiled from a reset in which the authoritative answer changes from `OFF` to `ON`
- **THEN** the semantic task identity remains stable while the state-bound expected-answer record and source-view digest change

#### Scenario: Predicate meaning changes
- **WHEN** the executable distance metric, stopped-state policy, or count-class policy changes
- **THEN** the task identity or declared predicate-policy version changes so incompatible meanings cannot be conflated

### Requirement: Mandatory generation report
Generation SHALL verify schema validity, required category cardinality, entity-resolution results, answer typing, reachability, distance stability, canonical regeneration, stable-reference integrity, source provenance, record-shape compatibility, and public-oracle leakage before recording each corresponding successful check. A corpus SHALL be marked complete only when every mandatory check has actually executed and passed. Full-release loading SHALL reject incomplete or count-mismatched manifests, duplicate task IDs, non-bijective joins, incompatible public/contract/outcome shapes, identity-payload mismatches, and inconsistent source/outcome digests.

#### Scenario: Complete smoke release
- **WHEN** all four tasks and their private records pass every mandatory validation
- **THEN** the report marks the corpus complete and records exactly four retained tasks

#### Scenario: Public answer leakage is detected
- **WHEN** a public record contains an expected answer or private semantic binding
- **THEN** validation marks the corpus incomplete and identifies the leaking artifact and field

#### Scenario: Structurally inconsistent release is loaded
- **WHEN** a release contains duplicate identities, a manifest count mismatch, or incompatible joined record types
- **THEN** full-release loading rejects it before an evaluator can reset a simulator
