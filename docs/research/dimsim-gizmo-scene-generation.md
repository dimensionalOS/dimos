# DimSim variability and Gizmo scene generation

Research date: 2026-07-29

## Summary

DimSim does not require a new simulator implementation for every environment. It
loads versioned scene JSON files and can install either registered scenes or a
local scene file. Its Python scene client can also mutate a running scene by
loading GLB/GLTF maps and adding primitives, walls, colliders, and NPCs.

However, DimSim is not currently a procedural scene generator. A reusable scene
still needs to exist as a DimSim/SimStudio scene JSON artifact, and the current
runtime mutation API does not preserve the benchmark semantic oracle we need.

Gizmo is a credible upstream scene-authoring system. Its documented API generates
structured scenes from text and optional reference images, retains an editable
scene graph, runs placement and physics validation, and exports to USD/USDZ,
MJCF, and SDF. Its editor also exports GLB. Gizmo additionally supports semantic
regions, robot spawns, and sensor anchors.

There is no documented direct Gizmo-to-DimSim export as of the research date.
DimSim consumes SimStudio-style scene JSON, while Gizmo's documented API export
targets are USD/USDZ, MJCF, and SDF. GLB can be loaded into DimSim, but Gizmo
documents GLB as visual-only and lossy: it does not retain physics or joints.
Therefore, benchmark use needs either:

1. an official Gizmo "Export to DimSim" path; or
2. a converter from Gizmo's full scene graph to DimSim scene JSON and oracle
   metadata.

## What DimSim can vary today

### Scene-level variability

The official DimSim CLI has a scene registry and supports:

- `dimsim scene install <name>` for released scenes;
- `dimsim scene install <name> --local <path>` for local JSON or compressed
  scene artifacts;
- `dimsim dev --scene <name>` to select the installed scene.

The repository README also documents adding scenes by placing JSON files in
`public/sims/`. The scene is therefore a data artifact, not a separate codebase.

Sources:

- [DimSim README](https://github.com/Antim-Labs/DimSim/blob/6de93e30ebd82f40ae2c9d62637aff767d33b895/README.md)
- [DimSim scene installer](https://github.com/Antim-Labs/DimSim/blob/6de93e30ebd82f40ae2c9d62637aff767d33b895/dimos-cli/setup.ts)
- [DimSim CLI documentation](https://github.com/Antim-Labs/DimSim/blob/6de93e30ebd82f40ae2c9d62637aff767d33b895/dimos-cli/README.md)

The current public release registry contains only the `apt` scene. The machinery
supports a scene corpus, but a diverse corpus has not yet been published.

### Episode-level/runtime variability

The DimOS `SceneClient` can:

- load GLB/GLTF maps;
- add and remove primitives;
- add walls and physics colliders;
- add animated NPCs;
- clear user-added objects;
- set the embodiment and agent pose;
- execute browser-side scene-editing code.

This is enough to implement seeded episode perturbations such as changing an
initial pose, object state, or selected placements. It is not yet a safe
benchmark-generation interface because the mutations are imperative and the
query surface returns only shallow object information. Generated variants need
to be serialized with stable IDs and authoritative semantics if benchmark goals
refer to them.

Local sources:

- `dimos/simulation/dimsim/scene_client.py`
- `dimos/simulation/dimsim/dimsim_process.py`
- `dimos/core/global_config.py`

### Current integration caveat

DimOS currently clones `paul-nechifor/DimSim` at branch `run-from-repo`. The
official upstream is now `Antim-Labs/DimSim`, with a released CLI and scene
registry. Before building a generation pipeline, the integration should either
track the official package/release or explicitly document why the older fork is
required.

## What Gizmo provides

Gizmo describes itself as a browser-based, AI-powered simulation authoring
platform. It generates a structured and editable scene from text or up to three
reference images. Its documented scene pipeline plans assets, generates
geometry/joints/materials/physics, creates a floorplan, places assets using
constraints, and validates physics.

The public REST API supports asynchronous:

- scene and asset generation;
- natural-language scene edits;
- retrieval of a full scene graph;
- job polling and server-sent progress events;
- exports to MJCF, USD/USDZ, and SDF;
- catalog search over more than 3,000 premade assets.

The editor supports semantic regions, robot spawns, sensor anchors, physics
preview, and manual correction. The docs explicitly warn that Gizmo is in beta,
generation can occasionally fail, browser physics is only a quick inspection,
and outputs should be validated in the target simulator.

Sources:

- [What is Gizmo?](https://docs.gizmo.antimlabs.com/)
- [Gizmo documentation index](https://docs.gizmo.antimlabs.com/llms.txt)
- [API access](https://docs.gizmo.antimlabs.com/api-access)
- [Core generation pipeline](https://docs.gizmo.antimlabs.com/core-idea)
- [Editor and robotics metadata](https://docs.gizmo.antimlabs.com/editor-walkthrough)
- [Export behavior](https://docs.gizmo.antimlabs.com/export-behavior)
- [Practical notes](https://docs.gizmo.antimlabs.com/practical-notes)
- [Quality checklist](https://docs.gizmo.antimlabs.com/quality-checklist)

## Compatibility assessment

| Path | Works now? | What is retained | Benchmark suitability |
|---|---:|---|---|
| Hand-authored DimSim JSON | Yes | DimSim assets, transforms, states, actions | Good after semantic validation |
| Local/registered DimSim scene corpus | Yes | Same as above, versionable | Preferred current path |
| Runtime `SceneClient` mutation | Partly | Runtime geometry and physics | Useful for seeded variants only after serialization and semantic registration |
| Gizmo GLB to `SceneClient.load_map` | Technically yes | Visual meshes | Insufficient: Gizmo documents GLB as losing physics and joints; semantic oracle also needs a sidecar |
| Gizmo MJCF/USD/SDF export | Not in current DimSim | Physics, joints, materials | Rich source, but requires a new DimSim importer or a different simulation backend |
| Gizmo full scene graph to DimSim JSON | Not documented | Potentially the richest mapping | Recommended integration experiment |
| Direct Gizmo-to-DimSim export | Not documented | Unknown | Ask Antim Labs whether this exists or is planned |

## Recommended benchmark workflow

Treat Gizmo as an **offline scene supplier**, not a live dependency of a scored
episode:

1. Generate or select a scene in Gizmo.
2. Retrieve the full scene graph, not only GLB.
3. Convert it to a versioned DimSim scene package.
4. Map or author the benchmark-required oracle fields: stable entity IDs,
   categories, aliases, object states, room/region membership, footprints,
   canonical orientation where needed, and provenance.
5. Run structural, collision, navigation-connectivity, visibility, and
   task-generation gates.
6. Freeze the accepted artifact with its generator prompt, Gizmo scene ID,
   source/export version, converter version, and content hash.
7. Generate benchmark tasks only from the accepted frozen artifact.

This preserves reproducibility and prevents service updates, generation
failures, network access, or account state from changing a scored benchmark
episode.

## Suggested first experiment

Do not begin with thousands of generated scenes. Use:

1. the existing `apt` scene to finish the four-task smoke corpus;
2. one small Gizmo-generated apartment or living room containing a sofa,
   television, dining table, four dining chairs, and bathtub;
3. an adapter spike that proves stable IDs, transforms, colliders, object state,
   and semantic regions survive the Gizmo-to-DimSim path;
4. the same four task templates regenerated against both scenes.

The experiment succeeds only if the task generator can derive and independently
verify objective answers from the imported oracle without manual per-question
patches.

## Open questions

- Does the authenticated Gizmo full scene graph already use the same primitive
  schema as DimSim/SimStudio? Public docs do not specify its response schema.
- Can Antim Labs expose a first-party DimSim JSON export that preserves semantic
  regions, stable IDs, articulation, and state?
- Do Gizmo's generated "semantics" include object taxonomy and state labels, or
  mainly robotics regions and affordances?
- Is generation deterministic or seedable? The public scene-generation request
  schema documents prompt, model, pipeline, reference images, and persistence,
  but no public seed.
- What licensing and redistribution terms apply to generated scenes and premade
  catalog assets in a public benchmark corpus?
