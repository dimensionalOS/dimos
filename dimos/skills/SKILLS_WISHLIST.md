# Skills Wishlist

Backlog of robot skills (see [skills.py](skills.py) `AbstractSkill`/`AbstractRobotSkill`
framework) for reasoning about the environment the robot observes: mapping,
object/scene understanding, and activity over time. Update status as items move
from idea -> prototype -> implemented skill.

Status legend: `idea` | `prototype` | `implemented` | `blocked`

## Skill quick reference

Agent-callable `@skill` entry points per item (details in each item's section;
library-only items have no skill by design or not yet):

| Item | Skill(s) | Container / file |
|---|---|---|
| 1 | `generate_floorplan(...)` | `FloorplanSkillContainer` — [dimos/skills/mapping/floorplan_skill.py](mapping/floorplan_skill.py) (pipeline: [dimos/mapping/floorplan/generator.py](../mapping/floorplan/generator.py)) |
| 2 | `catalog_scene()`, `list_observed_items()`, `detect(*prompts)`, `select(track_id)` | `ObjectSceneRegistrationModule` — [dimos/perception/object_scene_registration.py](../perception/object_scene_registration.py) |
| 3 | — (library only: `LidarPointCloudClient`) | [dimos/mapping/pointclouds/live.py](../mapping/pointclouds/live.py) |
| 4 | — (idea) | will build on `ObjectDB` — [dimos/perception/detection/objectDB.py](../perception/detection/objectDB.py) |
| 5 | `identify_object(object_id)`, `refine_observed_labels()` | `ObjectSceneRegistrationModule` — [dimos/perception/object_scene_registration.py](../perception/object_scene_registration.py) (helpers: [dimos/perception/contextual_labeling.py](../perception/contextual_labeling.py)) |
| 6 | — (idea) | — |
| 7 | — (idea, blocked on 6) | — |
| 8 | — (library only: `SceneModel`, `VoxelOccupancy`) | [dimos/mapping/reconstruction/scene_model.py](../mapping/reconstruction/scene_model.py) |

All skills are exposed as MCP tools by the `unitree-go2-scene-agentic`
blueprint (see Notes). Keep this table in sync when a skill lands or is
renamed — and mirror it in the item's dashboard folder (`skills` /
`skills_note` in `item.json`, see "Publishing to the dashboard").

## 1. Architectural floorplan generation
- **Status:** implemented (skill) — core pipeline promoted to
  [dimos/mapping/floorplan/generator.py](../mapping/floorplan/generator.py)
  (`FloorplanOptions` -> `generate_floorplan()` -> `FloorplanResult.summary()`),
  wrapped as an agent-callable `@skill` in
  [dimos/skills/mapping/floorplan_skill.py](mapping/floorplan_skill.py)
  (`FloorplanSkillContainer.generate_floorplan` — every parameter documented in
  the MCP schema; debug artifacts and 3D-model output OFF by default and the
  temp/session-store cleanup is automatic; a skill never opens the viewer),
  with the CLI kept as a thin wrapper at
  [scripts/demo_lidar_floorplan.py](../../scripts/demo_lidar_floorplan.py).
  Unit-tested end-to-end on a synthetic building
  ([test_generator.py](../mapping/floorplan/test_generator.py),
  [test_floorplan_skill.py](mapping/test_floorplan_skill.py)).
  The pipeline builds a multi-story DXF/JPEG drawing set from live lidar or a
  .rrd recording. Pipeline (built on item 8): the `SceneModel` 3D slices come
  first — every per-level evidence raster (occupancy, structural, ceiling) is
  a cell-aligned `horizontal_section()` plan cut — then the architectural
  DXF/JPEG stages (floor segmentation from trajectory dwell, wall vs furniture
  by height evidence, indoor/outdoor via ceiling evidence + trajectory
  connectivity, glazing detection, doors carved where the trajectory crosses
  walls, stairs from climbs between walkable platforms,
  mezzanine/split-level/sunken-area detection — dwell + slab + walkable
  footprint + overhead-return tests, drawn dashed with their own FFL label on
  the parent sheet (layer A-FLOR-LEVL) — AIA layers/poché/dimensions), and
  finally a cleaning stage that runs **only if necessary** (measured defects —
  dangling endpoints / fragments / duplicates — gate the pro-model critique;
  deterministic `tidy_plan` always runs). Optional OpenAI passes:
  classification review, camera-frame surface confirmation from a session dir,
  and `--ai-render` stylized sheet versions via the image model (gpt-image:
  drafted / cyanotype / presentation, or a custom prompt). The 3D stage is
  voxel-based: item 8's `VoxelOccupancy.closed_region()` seals each connected
  area the robot drove through (progressive closing 0.3-2.5 m, smallest kernel
  that stops the trajectory-seeded flood leaking; the ceiling-evidence indoor
  mask is passed as a barrier so ground floors don't leak through open
  entrances), drawing closed room loops on layer `A-AREA` and bridging
  boundary gaps not explained by walls/doors with wall segments — doors stay
  open (`--no-close-loops` disables; `--voxel` sets the grid). `--viewer`
  writes `<out>.model.rrd` via `save_rerun_sliceable()` and opens dimos-viewer
  where the time slider sweeps a horizontal cross-section through the property
  for debugging.
- **Next step:** compose `floorplan_skill_container` into the agentic Go2
  blueprints so the skill is reachable over MCP on-robot.

## 2. Identify, list, and locate observed items
- **Status:** implemented (replay-validated) — built on the existing
  [ObjectSceneRegistrationModule](../perception/object_scene_registration.py)
  (YOLO-E open-vocab detect -> world-frame `Object` -> dedup/persist in
  `ObjectDB`). Added: `Object.locate_encode()` (name + world position + size +
  confidence), a lidar localizer `Object.from_2d_to_list_lidar()` for robots
  without a depth camera, a `localization="depth"|"lidar"` mode + `lidar` input
  on the module, a prompt-free `list_observed_items()` catalog skill (position
  + size + confidence + description when refined via item 5),
  `get_located_objects()` RPC, and a Go2 blueprint `unitree-go2-scene`. Bundled
  as one agent-facing entry point: `catalog_scene()` — "find and catalog
  everything in the space I've navigated" in a single call — refines any
  ambiguously-labeled item with the item 5 VLM pass, then returns the full
  `list_observed_items()` inventory, so an agent doesn't need to sequence two
  separate tool calls to get a described catalog.
  **Replay-validated end-to-end**:
  [test_object_scene_registration_replay.py](../perception/test_object_scene_registration_replay.py)
  drives the production module over a full recorded Go2 session (87s of
  camera+mid360+odom) and checks accumulation, dedup, world-frame localization,
  and the catalog output; a final pass refines every catalogued object with the
  local VLM. The replay surfaced and fixed two production bugs: `ObjectDB` TTLs
  ran on wall clock instead of stream time (replayed/offset-clock data was
  pruned before promotion), and `Object.to_detection3d_msg()` crashed the
  publish path on coplanar (floor-plane) lidar clusters (Qhull OBB -> AABB
  fallback). Default pointcloud filters proved fine against the Go2's
  accumulated `/lidar` cloud (~27k pts; 156/196 detections survive), so no
  sparsity retuning was needed. Replay note: this recording's odom carries the
  robot's internal clock (~1min off the sensors) — live is unaffected
  (`time_is_now` restamps on receive), and the replay harness restamps odom
  with arrival time the same way. Tuning: lidar localization jitters more than
  depth, so the Go2 blueprint dedups over `distance_threshold=0.35` (vs the
  0.2m depth default), collapsing repeat sightings ~0.1-0.35m apart.
- **Spatial-memory fusion:** `catalog_scene()` now optionally folds in
  [SpatialMemory](../perception/spatial_perception.py)'s named/tagged places —
  semantic locations (e.g. "kitchen", "charging dock") that object detection
  alone can't produce — and tags each detected object with its nearest place
  for context. Wiring is via an optional `SceneMemorySpec` dependency
  ([spatial_memory_spec.py](../perception/spatial_memory_spec.py), backed by a
  new read-only `get_tagged_locations()` on `SpatialMemory`/`SpatialVectorDB`):
  when a `SpatialMemory` module shares the graph it auto-injects, otherwise the
  catalog degrades to objects-only (identical prior output). Combined build:
  `unitree-go2-scene-memory(-agentic)`. Unit-tested in
  [test_object_scene_registration.py](../perception/test_object_scene_registration.py)
  (fusion, nearest-place radius gating, graceful degradation, and a ChromaDB
  round-trip of `get_tagged_locations`).
- **Next step:** validate live on the Go2 via
  `unitree-go2-scene-memory(-agentic)`, then feed item 4 (temporal counts) from
  `ObjectDB`.

## 3. Convert lidar to numpy signals
- **Status:** implemented (client-side utility) —
  [dimos/mapping/pointclouds/live.py](../mapping/pointclouds/live.py)'s
  `LidarPointCloudClient` subscribes to `/lidar` (+ optional `/global_map`)
  over LCM and exposes the accumulated points as a plain `(N, 3)` numpy array,
  with unit tests in `dimos/mapping/pointclouds/test_live.py`. Extracted from
  `demo_lidar_floorplan.py`'s previously ad hoc `LidarCollector`, which now
  uses it. Note this is scoped to *client-side* consumers (scripts/skills
  running outside the module graph) — single-message conversion
  (`PointCloud2.points_f32()`/`.as_numpy()`) and in-graph accumulation +
  grid rasterization (`dimos/mapping/pointclouds/occupancy.py`, wired via the
  live `Map` module) already existed and are unchanged.
- **Next step:** wrap as a callable skill once item 2/4 have a concrete
  need for live client-side points (item 2's `Object.from_2d_to_list_lidar()`
  instead takes an already-accumulated `PointCloud2` as a parameter, supplied
  by its own in-graph module, so it doesn't need this client).

## 4. Temporally count items
- **Status:** idea — partially unblocked by item 2: `ObjectDB` already
  deduplicates observations over time and each `Object.detections_count` is a
  running per-item observation count.
- Track counts of detected items/entities over time (depends on item 2 for
  per-frame detection + item 3 for a shared point-cloud/array representation).

## 5. Contextual labeling / educated guesses / external data sources
- **Status:** implemented (VLM path) —
  [dimos/perception/contextual_labeling.py](../perception/contextual_labeling.py)
  asks a `VlModel` (per `global_config.detection_model`) for an educated guess
  on an ambiguous detection, feeding it the cropped detection image plus
  context the detector doesn't use: the detector's guess + confidence, the
  object's real-world size, and nearby objects from the `ObjectDB` scene.
  `Object` gained `refined_name`/`refined_description` (raw detector `name`
  preserved; encoders and `find_by_name` prefer/accept the refined label, and
  refinement survives `update_object`). Exposed on
  `ObjectSceneRegistrationModule` as skills `identify_object(object_id)`
  (single item) and `refine_observed_labels()` (batch over low-confidence /
  generic-named permanent objects) plus a `refine_object_labels()` RPC.
  Unit-tested with a stub VLM in
  [test_contextual_labeling.py](../perception/test_contextual_labeling.py), and
  exercised for real (moondream) against item 2's replayed Go2 session:
  `test_replay_catalog_with_vlm_descriptions` refines all 16 catalogued objects
  and their descriptions land in the `list_observed_items()` catalog.
- **Next step:** external data sources (e.g. OSM lookups via
  `dimos/agents/skills/osm.py`, object databases) as additional context, and
  live validation on the Go2 alongside item 2's lidar path.

## 6. Identify specific activity (e.g. "human does X")
- **Status:** idea
- Activity/action recognition over a temporal window of observations, distinct
  from static object detection (item 2).

## 7. Count occurrences of activity over time
- **Status:** idea, depends on item 6
- Aggregate activity-recognition events (item 6) into counts/rates over time,
  analogous to item 4 but for activities instead of items.

## 8. Full 3D scene model with cross-sections
- **Status:** prototype —
  [dimos/mapping/reconstruction/scene_model.py](../mapping/reconstruction/scene_model.py)'s
  `SceneModel` fuses everything a mapping session produced into one queryable
  3D model: the lidar cloud + robot trajectory from a Rerun recording
  (`SceneModel.from_rrd` / `load_rrd_points`), optionally colorized by
  projecting the recorded camera stream (`colorize_from_session`: TUM poses +
  `camera_intrinsics.json` + `mem2.db` `color_image`, nearest-view-wins
  z-test). Analysis surface: `horizontal_section(z)` (plan cuts) and
  `vertical_section(p1, p2)` (elevations) return rasterized `Section`s with
  density + mean color; exports: `save_rerun()` (interactive colored model for
  `dimos-viewer`), `to_mesh()` (Poisson) + `save_mesh()` (GLB/glTF/OBJ/PLY/STL
  with vertex colors — drops into Blender/three.js/CAD), `to_voxel_grid()`.
  Demo:
  [scripts/demo_rrd_3d_model.py](../../scripts/demo_rrd_3d_model.py); unit
  tests in
  [test_scene_model.py](../mapping/reconstruction/test_scene_model.py).
- **Next step:** wrap as a callable skill (build model of the space seen so
  far, answer "show me a section at z"). Item 1's migration is done: its
  evidence rasters are `horizontal_section(bounds=...)` cuts (the `bounds`
  parameter keeps multi-band sections cell-aligned).

## Notes
- Items 2 -> 4, 3 -> 2/4, 6 -> 7, and 8 -> 1 have direct dependencies; build
  the foundational piece once and share it rather than duplicating per-skill.
- Interactive test harness: the `unitree-go2-scene-agentic` blueprint wraps
  `unitree-go2-scene` with `McpServer`/`McpClient`, exposing every `@skill`
  here as an MCP tool at http://localhost:9990/mcp (attach Claude Code via
  `claude mcp add --transport http --scope project dimos
  http://localhost:9990/mcp`; see dimos/agents/mcp/README.md).
- Check `dimos/perception` and `dimos/agents/skills` for existing building
  blocks before starting a new skill from scratch.

## Publishing to the dashboard

A local web dashboard renders this wishlist one tab per item:

```sh
./misc/skills_dashboard/serve.sh          # http://127.0.0.1:8642/misc/skills_dashboard/
```

The page is data-driven: it fetches whatever is in
`misc/skills_dashboard/data/<NN-slug>/` at load time, so updating a skill's
status, tests, or figures only means republishing files into its folder — the
dashboard itself never needs editing. Per item folder:

- `item.json` (required) — title, `status` (`idea` | `prototype` |
  `in-progress` | `implemented` | `blocked`), `summary_html`, and optional
  sections: `skills` (agent entry points, rendered as an "Agent skills"
  table: `[{"name": "catalog_scene()", "container": "ModuleClass",
  "desc_html": "...", "file": "dimos/..."}]`) + `skills_note` (one-liner for
  library-only / idea items), `how_it_works` (pipeline steps), `flow`
  (box-arrow diagram), `figures` (see below), `examples` (code blocks),
  `evidence` (free-form results table for items without a pytest suite),
  `test_descriptions` (test name -> "what it verifies" row text),
  `manual_tests` (CI-only tests to list, e.g. `self_hosted`), `cards`,
  `dependencies`, `next_step_html`, `sources`, `concept_file`. Copy an
  existing folder as a template: `02-identify-locate` (fullest) or
  `04-temporal-item-counts` (idea-stage).
- `results.xml` (optional) — pytest junit output. Republish after test runs:

  ```sh
  uv run pytest <the item's test files> \
    --junit-xml misc/skills_dashboard/data/<NN-slug>/results.xml
  ```

  Pass/fail/skip badges, suite timing, and the header KPIs all come from
  these files.
- figures (optional) — PNGs referenced by `item.json`'s `figures` list
  (`light` required, `dark` variant optional). Generator scripts that produce
  the current figures from real skill code live in
  `misc/skills_dashboard/scripts/`.

When a skill's status or behavior changes, update its section above AND
republish its dashboard folder (status in `item.json`, fresh `results.xml`,
new figures if behavior is visual). Adding a new wishlist item: create the
folder, add its slug to `misc/skills_dashboard/data/items.json`, and add a
section to this file.
