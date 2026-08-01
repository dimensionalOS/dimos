## ADDED Requirements

### Requirement: Static map-grounded corpus
The system SHALL generate a versioned corpus for answering grounded spatial questions from immutable indoor maps. Map generation SHALL occur offline and SHALL NOT be part of a later evaluated system.

#### Scenario: Load a fixed benchmark snapshot
- **WHEN** a consumer selects a generated map-question instance
- **THEN** the corpus provides an immutable stored map and all public metadata required to interpret that instance without rerunning mapping

### Requirement: Pilot composition and scene-level splits
The pilot corpus SHALL contain 30 Structured3D scenes, one fixed full-coverage trajectory per scene, 10 development scenes, and 20 held-out evaluation scenes. Splits SHALL be assigned by scene and SHALL NOT divide questions or variants from one scene across splits.

#### Scenario: Validate pilot split membership
- **WHEN** the pilot manifest is validated
- **THEN** every scene belongs to exactly one split, all descendants inherit that split, and the development and held-out scene sets are disjoint

### Requirement: Canonical native map artifacts
Each map variant SHALL store the final `PointCloud2` output emitted by the configured DimOS `VoxelGridMapper` as `global_map.pc2.lcm`. The corpus SHALL record the mapper revision, mapper configuration, artifact hash, frame contract, and variant-specific terminal pose.

#### Scenario: Decode a canonical map
- **WHEN** a generated snapshot is loaded
- **THEN** its `global_map.pc2.lcm` decodes as `PointCloud2`, matches the recorded hash, and has provenance identifying the mapper configuration that produced it

### Requirement: Pre-mapping clean and noisy variants
The generator SHALL produce one clean control and two seeded noisy variants from the same scene and trajectory. Sensor and localization imperfections SHALL be applied before mapping and SHALL include versioned finite sensing, occlusion, range-error, dropout, coverage, and temporally correlated pose-drift policies. The generator SHALL NOT create noisy variants by corrupting a finished map.

#### Scenario: Reproduce a noisy variant
- **WHEN** generation is repeated with the same source revision, trajectory, mapper configuration, noise profile, and seed
- **THEN** it produces a map equivalent under the release's deterministic artifact policy

### Requirement: Physical questions and variant instances
The corpus SHALL store each physical question once, one private oracle answer per physical question, and one public instance for each clean or noisy map variant. Variant instances MAY contain different map-frame marker or pose coordinates, but SHALL retain the same physical question identity and oracle answer.

Variant map-frame query coordinates SHALL be produced by projecting each physical query point or pose through the estimated-pose transform of the nearest true trajectory waypoint, with equal-distance ties resolved stably by waypoint order. This SHALL be treated as a local alignment policy for a map accumulated from drifting scans, not as a single global rigid transform. The generator SHOULD retain collision and marker candidates only when the nearest waypoint is unique outside the configured tie/uncertainty band and SHALL reject ambiguous alignment candidates.

#### Scenario: Validate paired variants
- **WHEN** a physical question is selected
- **THEN** exactly one clean and two noisy instances resolve to that question and exactly one oracle answer is joined by its stable question identifier

#### Scenario: Project drifted coordinates locally
- **WHEN** a query point lies nearest to a true trajectory waypoint for a noisy variant
- **THEN** the public instance coordinate is transformed by that waypoint's true-to-estimated pose, and exact nearest-waypoint ties use waypoint order

### Requirement: Fixed spatial predicate set
The corpus SHALL generate only these seven v1 predicates: square-footprint pose occupancy, fixed-yaw straight translation, in-place rotation, eligible room count, same-room membership, direct room connection, and direct-neighbor count.

#### Scenario: Validate predicate coverage
- **WHEN** a release manifest and its questions are inspected
- **THEN** every question references one of the seven versioned predicates and each retained scene contains generated instances for every predicate

### Requirement: Embodiment-aware continuous collision truth
Pose occupancy, translation, and rotation answers SHALL use a fixed square evaluation footprint plus safety margin against private authoritative geometry. Translation and rotation SHALL evaluate the complete continuous swept footprint, boundary contact SHALL count as collision, and candidates within the configured uncertainty margin SHALL be rejected.

The authoritative collision model SHALL use the union of eligible walkable regions and barriers with validated doorway intervals removed. A room-boundary seam SHALL be traversable only through a validated opening. Rotation SHALL use conservative adaptive interval bounds and SHALL reject, rather than label, any candidate whose collision state cannot be proved within the configured refinement limit.

#### Scenario: Reject endpoint-only validity
- **WHEN** a translation or rotation has collision-free endpoints but its intermediate swept footprint intersects authoritative geometry
- **THEN** the oracle answer is collision and the item is not labeled from endpoint state alone

### Requirement: Benchmark room topology
The private oracle SHALL define Benchmark rooms as eligible navigable regions physically bounded by walls and explicit openings. It SHALL exclude open-plan semantic subdivisions, closets, stairs, and inaccessible regions. A direct connection SHALL be one traversable opening joining exactly two distinct eligible rooms.

Structural surfaces SHALL be selected from Structured3D plane types, while room and opening membership SHALL be selected from semantic groups. The importer SHALL support multiple closed contours on a source plane, validate source index references before reading incidence matrices, and use a versioned room-semantic allowlist. Exterior doors and windows SHALL NOT create room-graph edges. An internal opening SHALL have positive ground-level width, overlap the shared boundary of exactly two eligible rooms, and satisfy the configured footprint clearance.

#### Scenario: Build room graph answers
- **WHEN** room topology is generated for a retained scene
- **THEN** every included marker resolves unambiguously to one eligible room, adjacency is symmetric, and each direct-neighbor answer equals the degree of that room in the validated graph

### Requirement: Neutral public markers
Public spatial query markers SHALL expose only opaque identifiers and map-frame locations needed by the question. They SHALL NOT expose room identifiers, room semantics, room boundaries, topology edges, predicate answers, or oracle provenance.

#### Scenario: Scan a public instance
- **WHEN** a public instance and snapshot are inspected without oracle access
- **THEN** marker records contain no field that directly reveals room membership, connectivity, or an answer

### Requirement: Public and oracle storage separation
The corpus SHALL use human-inspectable hierarchical bundles with physically separable `public` and `oracle` roots. Public bundles SHALL contain scenes, trajectories, questions, variants, snapshots, maps, and instances. Oracle bundles SHALL contain source provenance, authoritative geometry, topology, answers, and review overrides. Stable opaque identifiers SHALL be the only join mechanism.

#### Scenario: Distribute public data independently
- **WHEN** only the public root is copied to a clean environment
- **THEN** every public artifact resolves using relative paths and no oracle file, private path, authoritative geometry, topology, or answer is present

### Requirement: Deterministic question generation
Question labels SHALL be computed from authoritative geometry and topology using versioned executable predicates and controlled templates. Generation SHALL reject ambiguous room assignments, malformed openings, transform failures, and boundary-sensitive collision candidates. An LLM SHALL NOT generate labels or judge answers.

Controlled template variants SHALL produce distinct text when multiple physical questions share a predicate template. The generator SHALL NOT emit exact duplicate physical questions. Eligible-room-count is the explicit cardinality exception: because it has one physical truth per scene, a retained scene SHALL emit one eligible-room-count question, while the other six predicates SHOULD emit two questions when possible, for an expected retained-scene total of 13 questions.

Candidate retention SHALL use deterministic seeded candidate-pool sampling or ranking rather than fixed picks. Boolean predicates SHALL include positive and negative labels when the scene supports both. Collision candidate pools SHALL include multiple inside, outside, and near-obstacle candidates and SHALL use oracle plus uncertainty rejection before selecting opposite labels. Direct-neighbor-count SHALL prefer anchors with distinct count answers when topology supports them; if not, deterministic distinct-room anchors are allowed and candidate statistics SHALL expose the low diversity for corpus-level balancing.

Room markers, opening candidates, and collision motions SHALL be evaluated under their configured geometric tolerance perturbations. The generator SHALL reject any candidate whose room membership, opening validity, or collision result changes or remains indeterminate within the corresponding uncertainty band.

#### Scenario: Regenerate question records
- **WHEN** the same source, policies, templates, and seeds are used
- **THEN** the generator emits the same retained question identities, parameters, and oracle answers

### Requirement: Per-predicate smoke-generation gate
Before full pilot generation, the implementation SHALL run a smoke generation that produces at least one valid retained sample for each of the seven predicates, including all required clean and noisy instances. Full pilot generation SHALL be blocked unless the smoke corpus passes schema, artifact, pairing, oracle, and visualization-load checks.

#### Scenario: Smoke gate passes
- **WHEN** smoke generation produces valid examples for all seven predicates and every required validation succeeds
- **THEN** the implementation permits full 30-scene pilot generation

#### Scenario: Smoke gate blocks incomplete coverage
- **WHEN** any predicate has no retained sample or any required clean/noisy instance, map artifact, oracle answer, or viewer load check fails
- **THEN** the implementation stops before full pilot generation and reports the failed predicate or validation

### Requirement: Automated release validation
The generator SHALL validate frame conventions, units, transform round trips, artifact integrity, deterministic output, collision tolerance stability, room-graph consistency, paired variants, scene-disjoint splits, public-private leakage, and label/nuisance distributions before publishing a release.

#### Scenario: Reject an invalid release
- **WHEN** any mandatory validation fails
- **THEN** the release is not marked complete and the validation report identifies the failing artifact, question, scene, or invariant

### Requirement: Read-only Viser inspection
The system SHALL provide a minimal read-only Viser tool that can select a corpus instance, display its `PointCloud2`, terminal robot pose, square footprint, query markers or motion overlays, and exact question, and optionally display private oracle geometry or topology when available. The tool SHALL NOT edit corpus records or maintain review workflow state.

#### Scenario: Inspect a generated item
- **WHEN** a curator opens a valid public instance with optional oracle access
- **THEN** the tool displays the map and predicate-specific overlays and permits navigation among instances and variants without modifying any artifact

### Requirement: Review overrides and immutable releases
Deterministic generator output SHALL remain unchanged after generation. Isolated manual exclusions or corrections SHALL be stored in `review_overrides.jsonl`; recurring defects SHALL require a generator-policy change and a new release. Published releases SHALL be immutable and versioned.

#### Scenario: Exclude an isolated ambiguous item
- **WHEN** a curator identifies one ambiguous generated question
- **THEN** the curator can record an exclusion in the matching oracle review-overrides file without editing the generated question, answer, instance, or map artifact
