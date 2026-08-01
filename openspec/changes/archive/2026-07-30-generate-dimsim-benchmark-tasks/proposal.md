## Why

DimSim scenes contain authoritative object identity, transforms, and state, but
the pinned apartment revision does not carry the semantic taxonomy needed to
generate objective embodied tasks. Without a versioned integration profile,
benchmark questions would depend on asset-title heuristics or manually supplied
answers that can drift from the simulated scene.

## What Changes

- Add a private, typed, read-only `SceneOracleView` assembled from one coherent
  DimSim runtime snapshot and a versioned DimOS apartment semantic profile.
- Bind authored semantic classes, aliases, state mappings, navigation policy,
  and provenance to exact stable asset IDs and a pinned DimSim revision; never
  infer them from display titles.
- Add deterministic offline generation of one validated smoke task in each category:
  - destination navigation;
  - targeted state QA;
  - broad-exploration count QA;
  - multi-hop spatial comparison QA.
- Materialize immutable, versioned public task records separately from private executable contracts and expected answers.
- Add stable content-derived identities, canonical serialization, source-view digests, generation diagnostics, and mandatory ambiguity/objectivity gates.
- Add a golden apartment smoke fixture for the agreed bathtub, television, dining-chair, and sofa-relative tasks.
- Exclude runtime DimSim execution, Pi-baseline integration, agent submission, scoring, and scene generation from this change.

## Capabilities

### New Capabilities

- `dimsim-scene-oracle-view`: Private typed export of authoritative DimSim scene semantics and state for offline benchmark generation.
- `dimsim-benchmark-task-generation`: Deterministic generation, validation, and public/private packaging of objective DimSim benchmark tasks.

### Modified Capabilities

None.

## Impact

- Affects the DimSim scene/integration boundary under
  `dimos/simulation/dimsim/` and pins the compatible external apartment
  revision without modifying or forking DimSim.
- Adds a benchmark-generation package under `dimos/benchmark/` with strict models, generators, canonical bundle writing, validation, and smoke fixtures.
- Reuses established static spatial-corpus conventions where applicable: strict immutable records, stable opaque IDs, canonical JSON/JSONL, deterministic templates, and physically separable public/oracle roots.
- Does not change simulator runtime modules, benchmark runners, Pi SDK execution, submission protocols, or score ledgers.
