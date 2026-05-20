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

## Runtime Flow

```text
Layer 6 odom/navigation/perception
  -> SpatialMemory / TemporalMemory
  -> _Go2SemanticTemporalMap.query_semantic_temporal_map(...)
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
    `NavigationInterfaceSpec`, `SpatialMemorySpec`, and `TemporalMemorySpec`.
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
- Current limits:
  - Robot safety state and connection health are not normalized yet.
  - The module currently reads navigation state by RPC when available.

## Version Boundaries

V1 implemented:

- Keep existing `SpatialMemory` and `TemporalMemory` implementation files in
  place.
- Add Layer 4 RPC facades for semantic-temporal memory and structured world
  snapshots.
- Make Layer 3 `ContextProvider` prefer Layer 4 world snapshots while keeping
  direct-read fallback behavior.

V2 planned:

- Add connection/safety/local-control status to `robot_state`.
- Add named object/location summaries to `memory_state`.
- Make semantic-temporal map entries more explicit: entity, location, time,
  evidence source, and confidence.

V3 planned:

- Decide whether stable world snapshots should be written to a durable store
  such as `memory2` SQLite, JSONL, or TemporalMemory.
- Add tests that build the full Go2 agentic blueprint and verify the injected
  Layer 3 -> Layer 4 RPC path.

## Design Rules

- Layer 4 owns normalized state, not LLM prompting or tool choice.
- Layer 4 modules should be RPC helpers unless there is a clear reason to
  expose a method directly as an MCP skill.
- Do not move existing perception or memory implementation files during this
  stage.
- Update this document when Layer 4 storage, snapshot fields, or RPC behavior
  changes.
