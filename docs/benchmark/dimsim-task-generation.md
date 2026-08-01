# DimSim benchmark task generation

The DimSim generator compiles four deterministic apartment smoke tasks from one
private `SceneOracleView`. It does not inspect agent observations, infer truth
from rendered pixels, or use an LLM.

## Private oracle contract

`SceneOracleView` represents one coherent runtime snapshot. It records the
scene, reset, semantic-schema, profile, and upstream revisions; the Three.js
frame and units; the embodiment footprint and canonical spawn; conservative
spawn-connected navigation geometry; stable entities, semantic properties,
footprints, and regions; and grouped field provenance.

The live provider joins two versioned inputs:

- the unmodified DimSim runtime supplies exact asset IDs, current state IDs,
  transforms, display titles, world-space bounds, and collision queries over
  every enabled non-sensor Rapier collider in one synchronous browser
  execution;
- `dimos/benchmark/dimsim/apartment_profile.py` supplies semantic classes,
  aliases, the television state-ID vocabulary, navigation bounds, canonical
  spawn, and region membership.

The live command samples the declared apartment bounds at the versioned grid
resolution. A cell is retained only when it has floor support and a conservative
prism enclosing the robot footprint, configured clearance, and half-cell
sampling error does not intersect any live collision source. Four-neighbor
flood fill retains only cells connected to the canonical spawn. The generator
therefore cannot certify a route through an omitted wall or unprofiled asset;
the semantic profile still covers only the eight entities needed by the smoke
questions.

The profile binds only by exact asset ID at pinned DimSim commit
`050aa1ec96e71524cdc9d84575ba5bc46d82993e`; it never selects entities by
display title. Missing IDs, unknown television states, invalid bounds, a moved
spawn, an incompatible embodiment, missing collision geometry, a modified
external checkout, or a different commit fail closed.

The fixture in `dimos/benchmark/dimsim/fixture.py` mirrors the normalized
contract for hermetic compiler tests. It is not used by the live provider.
`SceneClient.get_scene_oracle_snapshot()` is a private browser command, not an
agent skill or normal observation.

## Generate a smoke corpus

Generate from the contract fixture:

```bash
python -m dimos.benchmark.dimsim --fixture --output /tmp/dimsim-smoke
```

Launch the pinned apartment in CPU mode:

```bash
DIMSIM_RENDER=cpu dimos --simulation dimsim --dimsim-scene apt \
  run unitree-go2-basic
```

Then generate from the running instance:

```bash
python -m dimos.benchmark.dimsim --output /tmp/dimsim-smoke \
  --host localhost --port 8090
```

The live command fails if the runtime snapshot is incompatible with the
versioned profile. Generation also rejects missing provenance, ambiguous entity
cardinality, invalid geometry, unreachable stopping regions, unsupported state
values, and surface-distance comparisons within the stability margin.

## Release layout

```text
release/
├── manifest.json
├── public/
│   └── tasks.jsonl
└── oracle/
    ├── task_contracts.jsonl
    ├── expected_outcomes.jsonl
    └── generation_report.json
```

The public directory can be distributed alone. Its records contain controlled
task text and response shapes. Oracle files contain executable bindings,
expected outcomes, source digests, and diagnostics. Opaque task IDs are the only
join key.

Generator, predicate, frame, clearance, comparison-margin, and language-template
policies are versioned in `dimos/benchmark/dimsim/config.py`. Change a policy
version whenever its executable meaning changes. A state-only answer change
keeps the semantic task ID but changes the outcome record and source digest.

Failed generation writes only `oracle/generation_report.json`; it does not write
a manifest or public tasks. A successful report explicitly records schema,
category, entity-resolution, answer-typing, reachability, distance-stability,
stable-reference, leakage, regeneration, and provenance checks.

## Scope

This package ends after corpus generation and validation. Scene generation,
evaluated-agent execution, Pi-baseline integration, submission, scoring, and
result publication belong to later changes.

The apartment profile is deliberately scoped to this four-question smoke
corpus. It is not a general DimSim taxonomy or a claim that the simulator has
high-fidelity robot dynamics.
