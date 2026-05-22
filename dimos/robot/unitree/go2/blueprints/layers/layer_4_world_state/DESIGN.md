# Layer 4 World State Design

This document explains the first Go2 Layer 4 implementation. Layer 4 owns the
robot-facing world-state view. Layer 3 may read and compress this state for an
LLM, but Layer 3 should not be the only place where spatial memory, temporal
memory, odom, navigation state, and runtime mode are normalized.

## Current Scope

Layer 4 currently contains:

- Existing `SpatialMemory`: persistent visual/spatial memory backed by ChromaDB
  and visual-memory files through the existing perception module.
- Existing lazy `TemporalMemory`: imported only by the temporal-memory
  blueprint after CLI config is resolved.
- `_Go2SemanticTemporalMap`: a lightweight RPC view that combines spatial and
  temporal memory evidence for one query.
- `_Go2StructuredWorldState`: a lightweight RPC facade that returns a single
  structured world snapshot.
- Optional `RobotBodyStateSpec`: Layer 6 body/local-policy state read by
  `_Go2StructuredWorldState` when wired.
- V2 memory summaries: normalized `named_objects`, `named_locations`, and
  explicit semantic-temporal evidence entries with entity, location, time,
  evidence source, and confidence fields.
- V3 snapshot policy: Layer 4 snapshots are explicitly ephemeral read-through
  views. Durable writes remain in `SpatialMemory` and `TemporalMemory` until a
  concrete snapshot retention requirement exists.

## Runtime Flow

```text
Layer 6 odom/navigation/perception
  -> SpatialMemory / TemporalMemory
  -> _Go2SemanticTemporalMap.query_semantic_temporal_map(...)
  -> _Go2RobotBodyState.get_robot_body_snapshot(...)
  -> _Go2StructuredWorldState.get_world_snapshot(...)
  -> Layer 3 ContextProvider.get_context(...)
```

`ContextProvider` now prefers `WorldStateSpec.get_world_snapshot(...)` when the
Layer 4 module is wired. Its older direct reads of spatial memory, temporal
memory, navigation, and odom remain as a fallback.

## Implementation Notes

### `_Go2SemanticTemporalMap.query_semantic_temporal_map(...)`

- File: `layer_4_world_state/semantic_temporal_map.py`
- Entry point: RPC-only helper; not an MCP skill.
- Purpose: produce one memory-evidence payload from spatial and temporal
  sources. It does not create a new store, plan actions, or call skills.
- Inputs:
  - Direct arguments: `query`, `spatial_limit`.
  - Optional injected Specs: `SpatialMemorySpec`, `TemporalMemorySpec`.
- Storage:
  - No new database.
  - No persistent state.
  - Reads existing spatial/temporal memory modules only.
- Data read:
  - Spatial memory: `query_by_text(query, limit=spatial_limit)`.
  - Temporal memory: `query(query)`, `get_rolling_summary()`, `get_state()`,
    and `get_entity_roster()`.
- Return shape:
  - `query`
  - `sources`
  - `spatial`
  - `temporal`
  - `fused`
  - optional `errors`
- V2 data shape:
  - `temporal.graph` is read from `TemporalMemorySpec.get_graph_db_stats()`
    when available.
  - `fused.entries` contains compact evidence entries. Each entry includes
    `entity`, `location`, `time`, `evidence_source`, `confidence`, and
    `summary`.
  - Spatial entries use vector-memory metadata such as `pos_x`, `pos_y`,
    `pos_z`, `timestamp`, `frame_id`, and labels when present.
  - Temporal entries use state entities, currently-present entities,
    `entity_roster`, and graph entities when those sections are available.
- Current limits:
  - Fusion is deterministic metadata packaging.
  - It does not resolve contradictions between spatial and temporal evidence.

### `_Go2StructuredWorldState.get_world_snapshot(...)`

- File: `layer_4_world_state/structured_world_state.py`
- Entry point: RPC-only helper; not an MCP skill.
- Purpose: provide one normalized Layer 4 snapshot for Layer 3 and future
  callers.
- Inputs:
  - Direct arguments: `task`, `spatial_limit`.
  - Stream state: latest `odom` message cached by `_on_odom`.
  - Optional injected Specs: `SemanticTemporalMapSpec`,
    `NavigationInterfaceSpec`, `RobotBodyStateSpec`, `SpatialMemorySpec`, and
    `TemporalMemorySpec`.
  - Runtime config: `global_config`.
- Storage:
  - No new database.
  - No persistent files.
  - Keeps only latest odom in memory while the process is running.
- Return shape:
  - `task`
  - `sources`
  - `runtime`
  - `robot_state`
  - `memory_state`
  - `semantic_temporal_map`
  - optional `errors`
- V2 data shape:
  - `memory_state.named_objects` summarizes entities from fused evidence that
    are not typed as locations.
  - `memory_state.named_locations` summarizes entries that include coordinates
    or are typed as locations.
  - `memory_state.summary` reports named object count, named location count,
    evidence entry count, and contributing evidence sources.
- V3 data shape:
  - `snapshot_storage` reports the durability decision for this snapshot API.
    The current policy is `ephemeral_read_through` with no separate durable
    Layer 4 snapshot backend.
- Current limits:
  - Robot safety and connection state are read from Layer 6 when wired, but
    they are still descriptive context rather than physical safety guarantees.
  - The module currently reads navigation state by RPC when available.

### `_Go2StructuredWorldState.get_snapshot_storage_policy(...)`

- File: `layer_4_world_state/structured_world_state.py`
- Entry point: RPC-only helper; not an MCP skill.
- Purpose: expose the V3 storage decision to callers. Layer 4 snapshots are
  normalized read-through views and are not written to a separate durable
  store in this stage.
- Return shape:
  - `durable`: `False`
  - `backend`: `None`
  - `policy`: `ephemeral_read_through`
  - `reason`: short explanation that durable writes stay in existing memory
    modules until snapshot retention requirements are validated.

## Version Boundaries

V1 implemented:

- Keep existing `SpatialMemory` and `TemporalMemory` implementation files in
  place.
- Add Layer 4 RPC facades for semantic-temporal memory and structured world
  snapshots.
- Make Layer 3 `ContextProvider` prefer Layer 4 world snapshots while keeping
  direct-read fallback behavior.

V2 implemented:

- Add named object/location summaries to `memory_state`.
- Make semantic-temporal map entries more explicit: entity, location, time,
  evidence source, and confidence.

V3 implemented:

- Durable snapshot writes are deferred. Layer 4 snapshots remain ephemeral
  read-through state; persistent source data remains in `SpatialMemory` and
  `TemporalMemory`.
- Added a focused full-Go2-agentic-blueprint static test that verifies the
  injected RPC path from Layer 3 `ContextProvider` to Layer 4
  `_Go2StructuredWorldState`, and from Layer 4 to semantic-temporal memory and
  Layer 6 robot-body state.

## Design Rules

- Layer 4 owns normalized state, not LLM prompting or tool choice.
- Layer 4 modules should be RPC helpers unless there is a clear reason to
  expose a method directly as an MCP skill.
- Do not move existing perception or memory implementation files during this
  stage.
- Update this document when Layer 4 storage, snapshot fields, or RPC behavior
  changes.
