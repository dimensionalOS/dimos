## Context

DimOS currently launches DimSim through `DimSimProcess` and manipulates the browser scene through `SceneClient`. `SceneClient.get_scene_info()` exposes shallow runtime object information, while the DimSim apartment scene contains richer asset transforms and state definitions. Neither boundary currently provides the authoritative semantic classes, aliases, state properties, query geometry, regions, and provenance required to generate objective embodied tasks without title heuristics.

The existing static spatial benchmark already establishes useful corpus conventions: strict frozen Pydantic records, stable content-derived opaque identifiers, canonical JSON/JSONL, deterministic controlled templates, explicit executable predicates, mandatory generation validation, and physically separable public and oracle roots. This change reuses those conventions but creates a distinct DimSim embodied-task corpus and does not modify the existing static map-grounded capability.

The smoke target is intentionally narrow: one canonical apartment scene and exactly four generated tasks. Scene generation, start-pose expansion, agent execution, Pi-baseline integration, submission, and scoring are deferred.

## Goals / Non-Goals

**Goals:**

- Make the pinned DimSim runtime authoritative for stable object identity,
  transforms, geometry, and current state, with a versioned DimOS integration
  profile supplying the apartment taxonomy and state vocabulary absent from the
  external scene format.
- Define a strict private `SceneOracleView` for one coherent DimSim reset.
- Compile exactly one objective smoke task for destination, targeted QA, broad count QA, and multi-hop comparison.
- Reject unsupported or ambiguous candidates instead of guessing labels.
- Write deterministic, immutable public and oracle task artifacts with source digest and provenance.
- Provide a golden apartment fixture and complete automated generation validation.

**Non-Goals:**

- Generating or importing new scenes.
- Providing semantic oracle data to evaluated agents.
- Launching an agent or executing an episode.
- Extending the Pi baseline, adding `submit_answer`, parsing responses, or scoring results.
- Generating paraphrases with an LLM.
- Supporting manipulation, interactive QA, region questions, or canonical left/right questions in the smoke release.
- Replacing or changing the static spatial evaluation corpus.

## Decisions

### 1. A pinned DimSim snapshot and versioned DimOS profile jointly define truth

The unmodified DimSim runtime remains the source of stable asset identity,
current state, transforms, and world-space rendered bounds. A versioned DimOS
apartment profile supplies the semantic classes, approved aliases, state-ID
mapping, navigation sampling bounds, and provenance that the external scene
format does not expose. The live provider derives conservative free space by
querying every enabled non-sensor collider in DimSim's Rapier world. The profile
binds only by exact asset ID at one pinned DimSim commit and never by
display-title substring.

The private provider must assemble:

- scene, reset-state, profile, and semantic-schema revisions;
- a complete frame and unit convention;
- the configured embodiment footprint and canonical spawn used for generation gates;
- navigable and blocked geometry sufficient for stopping-region and reachability checks;
- stable runtime entities joined to profile semantics by exact asset ID, with
  transform, world-space 2-D bounds, semantic properties, and optional region
  membership;
- semantic regions and their geometry;
- field-level authored or policy-derived provenance.

Provenance records will group semantic field paths by source kind and policy version rather than copying a provenance object into every scalar field.

Alternative considered: parse asset titles directly in the generator. Rejected
because display titles are not a semantic taxonomy and can change independently
of stable asset identity.

Alternative considered: fork DimSim to add a semantic scene schema. Deferred to
avoid taking ownership of an external simulator fork. The integration profile
therefore fails closed on revision, ID, state-vocabulary, or geometry mismatch
and is intentionally limited to the apartment smoke corpus.

### 2. `SceneOracleView` is an in-memory generation boundary

The generator will normally receive `SceneOracleView` in memory through a provider interface. It will retain only the canonical content digest and source/policy revisions in the corpus. Tests may serialize the same strict model as a fixture, but fixture storage does not become a production semantic authority.

The provider boundary will allow:

- a live `SceneClient` implementation backed by one synchronous private runtime
  snapshot plus the versioned apartment profile;
- an in-memory fixture provider for unit and golden tests.

Alternative considered: require every corpus release to include the complete oracle view. Rejected because it duplicates scene truth, increases leakage risk, and is unnecessary for normal distribution. Debug tooling may persist a view only in an explicitly private, non-release workspace.

### 3. The compiler is pure after oracle acquisition

Oracle acquisition and corpus compilation are separate phases. Once a validated `SceneOracleView` is available, candidate enumeration, rejection, selection, identity construction, serialization, and validation perform no simulator, perception, network, or LLM calls.

This makes deterministic regeneration and unit testing possible and keeps transient simulator behavior outside the task compiler.

### 4. Use strict discriminated records with public/private separation

The initial package will live under a new DimSim benchmark namespace and define strict, frozen models for:

- release manifest and public task;
- scene-source provenance;
- private task contract;
- private expected outcome;
- generation diagnostics and validation report.

The canonical release layout is:

```text
<release-root>/
├── manifest.json
├── public/
│   └── tasks.jsonl
└── oracle/
    ├── task_contracts.jsonl
    ├── expected_outcomes.jsonl
    └── generation_report.json
```

The public task contains only stable task ID, category, controlled text, template version, and the minimal completion/response shape. The oracle contract contains semantic entity bindings and executable policy parameters. The expected outcome is one of:

- `terminal-predicate` for destination navigation;
- `enum` for television state;
- `integer` for dining-chair count;
- `entity-choice` for the closer-object comparison.

The detailed generation report stays under `oracle/` because rejected candidates and resolution diagnostics may reveal private entity identities or answers. `manifest.json` contains only public-safe release metadata and completion state.

### 5. Task identity describes intent, not answer

A task ID is derived from:

- source scene identity;
- private semantic entity bindings;
- contract kind and parameters;
- predicate-policy version;
- language-template identity.

It excludes expected answer, source-view digest, output order, and serialization path. Therefore an ON and OFF reset can share semantic task identity in a future multi-state corpus while producing distinct state-bound expected-outcome records. The smoke release uses one canonical reset and one expected outcome per task.

All records also carry their own stable IDs where needed, and reference integrity is checked before release completion.

Alternative considered: derive the task ID from public English. Rejected because wording edits would obscure whether executable meaning changed and different contracts can have deceptively similar text.

Alternative considered: include the answer in task identity. Rejected because state variation would incorrectly create a new semantic task.

### 6. Use controlled templates and executable policies

The smoke language is fixed to the four approved sentences. The generator does not ask an LLM to write questions or judge truth. Each category has one versioned executable policy:

- `navigate-within-outer-footprint`: threshold 1.0 metre;
- `entity-state`: property `power`, vocabulary `{ON, OFF}`;
- `count-semantic-class`: exact class `dining-chair`, scene scope;
- `argmin-surface-distance`: anchor sofa and two candidates.

Semantic classes and properties use canonical machine identifiers; aliases are used only to render approved public text, never to select truth-bearing entities.

### 7. Geometry uses surfaces and conservative stability gates

All distance calculations use world-space 2-D footprints under the declared frame contract:

- destination distance is the minimum distance from robot stopping footprint to the bathtub outer footprint;
- comparison distance is minimum polygon-to-polygon surface distance, not center distance.

The live provider samples the profiled apartment bounds at a versioned grid
resolution. For every cell it requires ground support and checks a conservative
square prism enclosing the embodiment footprint, configured clearance, and
half-cell sampling error against every enabled non-sensor Rapier collider.
Four-neighbor flood fill retains only cells connected to the canonical spawn.
The destination generator intersects that collision-cleared reachable geometry
with the one-metre stopping band and requires a non-empty feasible region. The
fixture path exercises the same generator using authored navigable and blocked
polygons.

The comparison policy requires the winning distance to exceed a versioned non-zero stability margin. The margin is generator configuration, recorded with the contract and covered by boundary tests; the smoke fixture must pass it without an override.

Candidates touching uncertainty boundaries, lacking valid polygons, or depending on undeclared canonical orientation are rejected.

### 8. Exact semantic resolution precedes question generation

Generators select entities by exact semantic class and explicit task eligibility, then require the cardinality expected by the contract:

- exactly one eligible bathtub;
- exactly one eligible television with authoritative power state;
- all and only distinct `dining-chair` entities for the count;
- exactly one eligible sofa plus distinct bathtub and television candidates for comparison.

Display-title substring matching is prohibited. A work chair remains excluded because its semantic class is not `dining-chair`.

If multiple otherwise eligible instances exist, the smoke generator fails with diagnostics rather than selecting the first. A later generalized generator may add versioned deterministic disambiguation policies.

### 9. Validation is release-blocking

Generation performs schema validation, exact four-category cardinality, unique entity resolution, answer typing, geometry validity, destination feasibility/reachability, comparison margin stability, stable ID reconstruction, canonical byte regeneration, one-to-one public/private pairing, and recursive public leakage scanning.

Any failure produces a deterministic private report and leaves the manifest incomplete. The golden smoke test asserts the exact four public texts and the expected typed outcomes derived from the canonical apartment fixture.

### 10. Upstream DimSim compatibility is explicit

The current DimOS integration clones an older DimSim branch, while the official
project now publishes releases from `Antim-Labs/DimSim`. DimOS will pin the
existing compatible `run-from-repo` commit rather than silently using whichever
checkout happens to be cached. The provider will require the profile revision,
exact stable IDs, supported state IDs, and valid live bounds and will fail
clearly on incompatibility.

Landing the live apartment smoke path requires the pinned unmodified DimSim
revision and matching DimOS apartment profile. Unit and bundle work can proceed
against a contract fixture, but the change is not complete until the live
provider passes the same generation and release gates.

Updating all DimSim installation and packaging behavior beyond what is necessary to select that compatible revision is outside this change.

## Risks / Trade-offs

- **[Risk] The profile can drift from the external apartment scene.** → Pin the
  DimSim commit, bind by exact stable IDs and state IDs, validate live bounds,
  and fail closed on every mismatch.
- **[Risk] The external DimSim revision and DimOS integration evolve independently.** → Negotiate an explicit oracle schema version and fail fast on incompatibility.
- **[Risk] A full semantic payload could leak answers if routed through normal agent APIs.** → Keep the provider private, store detailed diagnostics under `oracle/`, and recursively scan public output.
- **[Risk] Geometry or reachability policies may reject the only desired smoke candidate.** → Emit gate-specific diagnostics and fix authoritative geometry or the versioned policy instead of adding a manual expected answer.
- **[Risk] Field-level provenance increases schema complexity.** → Use grouped field paths with shared provenance and require coverage only for truth-bearing semantic fields.
- **[Trade-off] Exactly four smoke tasks provide little distributional coverage.** → Treat them as infrastructure contract tests; scene and episode expansion is deliberately deferred.
- **[Trade-off] Controlled language does not measure linguistic robustness.** → Preserve template versioning so paraphrase variants can be added later without weakening v1 objectivity.

## Migration Plan

1. Add strict oracle-view and corpus models plus fixture-only providers without changing the current DimSim runtime path.
2. Add four pure generators, canonical bundle writing, and release validation against a minimal synthetic fixture.
3. Pin the DimSim revision, add the versioned apartment integration profile,
   and assemble the private oracle view from one synchronous `SceneClient`
   snapshot.
4. Run the same contract and golden smoke tests against the compatible live DimSim revision.
5. Expose a generation entrypoint that writes only validated frozen artifacts.

Rollback removes the new benchmark-generation package and private provider method. Existing DimSim runtime behavior and the static spatial corpus remain unchanged because no current agent or evaluator consumes the new artifacts.

## Open Questions

- Whether the DimOS apartment profile should migrate into a future upstream
  DimSim semantic scene schema remains a follow-up decision.
