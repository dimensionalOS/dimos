## ADDED Requirements

### Requirement: Offline deterministic task compiler
The system SHALL provide an offline compiler that accepts one validated `SceneOracleView` and produces benchmark task records without querying agent perception, using an LLM to assign truth, or requiring evaluation execution. Given identical input content and generator-policy versions, it SHALL emit byte-equivalent canonical records.

#### Scenario: Regenerate an unchanged corpus
- **WHEN** generation is repeated with the same oracle-view digest, generator revision, predicate policies, language templates, and configuration
- **THEN** the compiler emits the same task identities, public text, private contracts, expected answers, ordering, and canonical bytes

#### Scenario: No candidate passes
- **WHEN** a requested category has no candidate satisfying every mandatory generation gate
- **THEN** generation fails with category-specific diagnostics and does not mark the corpus complete

### Requirement: Typed task and answer contracts
Each generated task SHALL have a stable semantic identity, category, versioned controlled utterance, and declared response or completion type. Private records SHALL use discriminated executable contracts and typed expected answers: terminal navigation predicate, finite enum, integer, or entity choice. Expected answers SHALL be computed from the oracle view rather than manually supplied.

#### Scenario: Generate a state question
- **WHEN** the compiler selects an entity-state predicate for a television with authoritative power state `OFF`
- **THEN** it emits public controlled language, a private `entity-state` contract bound to the stable television entity, and a typed enum answer `OFF`

#### Scenario: Generate a destination task
- **WHEN** the compiler selects the bathtub destination predicate
- **THEN** it emits a private terminal contract requiring the agent to stop within 1 metre of the bathtub's outer footprint and does not fabricate a textual expected answer

### Requirement: Four-category smoke corpus
The smoke compiler SHALL emit exactly one retained task for each of four categories from the canonical apartment oracle view:

1. destination: “Go to the bathtub and stop within 1 meter of its outer edge.”
2. targeted QA: “Is the television ON or OFF?”
3. broad-exploration QA: “How many dining chairs are in the apartment?”
4. multi-hop QA: “Which is closer to the sofa: the bathtub or the television?”

It SHALL reject duplicate categories, missing categories, and extra retained smoke tasks.

#### Scenario: Generate the canonical apartment smoke corpus
- **WHEN** the apartment oracle view contains the uniquely resolvable entities and stable predicates required by all four templates
- **THEN** the compiler emits four tasks in deterministic category order with one task per category

#### Scenario: One category is unsupported
- **WHEN** the apartment oracle view cannot support one of the four objective contracts
- **THEN** smoke generation fails rather than replacing it with a different or guessed question

### Requirement: Destination objectivity gates
A destination candidate SHALL resolve to exactly one target entity, use metric outer-footprint distance, and have at least one collision-free reachable stopping region satisfying the threshold and configured embodiment clearance. Candidates whose feasible stopping region is absent or uncertain SHALL be rejected.

#### Scenario: Bathtub has a reachable stopping region
- **WHEN** the bathtub resolves uniquely and its one-metre stopping band contains a validated reachable collision-free region
- **THEN** the destination candidate is eligible for retention

#### Scenario: Target is geometrically unreachable
- **WHEN** no validated collision-free robot stopping pose can satisfy the destination threshold
- **THEN** the generator rejects the destination candidate and reports the failed reachability gate

### Requirement: Targeted state QA gates
A targeted state question SHALL resolve exactly one entity and one explicit state property whose value belongs to the contract's finite answer vocabulary. Visual appearance or rendered-pixel inference MUST NOT provide the oracle answer.

#### Scenario: Television power state is authoritative
- **WHEN** the unique television exposes canonical power state `ON` or `OFF`
- **THEN** the targeted QA candidate is retained with that exact typed answer

#### Scenario: Television state is only visually inferred
- **WHEN** no authoritative power property exists despite the television's rendered appearance
- **THEN** the generator rejects the candidate rather than classifying pixels

### Requirement: Broad count QA gates
A count question SHALL use an exact authoritative semantic class and declared scope. It SHALL count stable entity identities, not display-title matches, and SHALL exclude entities of other classes even when their names contain a related word.

#### Scenario: Count dining chairs
- **WHEN** the apartment oracle view contains four distinct entities classified as `dining-chair` and a separate entity classified as `work-chair`
- **THEN** the expected answer is integer `4` and the work chair is excluded

#### Scenario: Count class is unavailable
- **WHEN** chair-like assets lack an authoritative class distinguishing dining chairs from other chairs
- **THEN** the generator rejects the count candidate rather than normalizing asset titles

### Requirement: Multi-hop comparison gates
The closer-object question SHALL resolve exactly one anchor and two distinct candidates, compute anchor-to-candidate surface distances under a versioned metric policy, and retain the task only when the winning candidate is unique beyond a configured stability margin. Center-point distance MUST NOT substitute for surface distance.

#### Scenario: One candidate is stably closer to the sofa
- **WHEN** the sofa-to-bathtub and sofa-to-television surface distances differ by more than the configured margin
- **THEN** the generator emits the uniquely closer entity as a typed entity-choice answer

#### Scenario: Comparison lies within the uncertainty margin
- **WHEN** the two surface distances tie or differ by no more than the configured stability margin
- **THEN** the generator rejects the comparison candidate as boundary-sensitive

### Requirement: Stable identity and provenance
Task identities SHALL be content-derived from semantic intent rather than expected answer or output ordering. Every private task record SHALL identify the source scene, source oracle-view digest, semantic-schema revision, generator revision, predicate-policy version, and language-template version.

#### Scenario: Canonical state answer changes
- **WHEN** the same television-state task is compiled from a reset in which the authoritative answer changes from `OFF` to `ON`
- **THEN** the semantic task identity remains stable while the state-bound expected-answer record and source-view digest change

#### Scenario: Predicate meaning changes
- **WHEN** the executable distance metric or count-class policy changes
- **THEN** the task identity or declared predicate-policy version changes so incompatible meanings cannot be conflated

### Requirement: Public and oracle package separation
The generated corpus SHALL use physically separable public and oracle roots joined only by stable opaque task identifiers. Public records SHALL contain controlled task language and the minimum declared response/completion shape. Oracle records SHALL contain semantic entity bindings, executable contracts, expected answers, source provenance, rejection diagnostics, and review material.

#### Scenario: Distribute only public tasks
- **WHEN** the public root is copied without the oracle root
- **THEN** every public task remains readable and contains no expected answer, private entity binding, source oracle digest, semantic provenance, or executable private contract

#### Scenario: Pair private records
- **WHEN** the complete corpus is validated
- **THEN** every retained public task joins to exactly one private contract and exactly one compatible expected-answer or terminal-predicate record

### Requirement: Mandatory generation report
Generation SHALL produce a deterministic report covering schema validity, required category cardinality, entity-resolution results, answer typing, reachability, distance stability, canonical serialization, stable-reference integrity, and public-oracle leakage. A corpus SHALL be marked complete only when every mandatory check passes.

#### Scenario: Complete smoke release
- **WHEN** all four tasks and their private records pass every mandatory validation
- **THEN** the report marks the corpus complete and records exactly four retained tasks

#### Scenario: Public answer leakage is detected
- **WHEN** a public record contains an expected answer or private semantic binding
- **THEN** validation marks the corpus incomplete and identifies the leaking artifact and field

### Requirement: Generation-only scope boundary
This capability SHALL end after producing and validating frozen benchmark artifacts. It SHALL NOT add DimSim episode execution, Pi-baseline integration, agent submission tools, timeouts, trajectory collection, scoring, or result publication.

#### Scenario: Build generation capability
- **WHEN** this change is implemented and its tests run
- **THEN** the implementation can compile and validate the smoke corpus without launching an evaluated agent or modifying a benchmark runner
