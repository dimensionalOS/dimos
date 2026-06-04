# Folder Structure

Per-producer types: each Module owns its config + emitted message types
in its own file. No shared `config.py`, no umbrella class, no shared YAML.

```
dimos/learning/
│
├── specs/
│   ├── structure.md
│   └── datacollection.md           # Stage 1
│
├── dataprep.py                     # types + pure helpers (no Module)
│                                   #   - Episode, Sample
│                                   #   - StreamField, SyncConfig, OutputConfig, EpisodeExtractor
│                                   #   - resolve_field, compute_stats,
│                                   #     extract_episodes, iter_episode_samples
├── dataprep_module.py              # DataPrepModule(Config) only
│
├── collection/
│   ├── episode_monitor.py          # EpisodeStatus + EpisodeMonitorModule(Config)
│   └── blueprint.py                # learning_collect_quest_<robot>
│
└── formats/                        # dataset writers; each calls DataPrep.compute_stats
    ├── lerobot.py                  # LeRobot v2 (parquet + MP4 + meta/stats.json)
    └── hdf5.py
```

---

## Per-producer typed contracts

| Class | Lives in | Used by |
|---|---|---|
| `EpisodeStatus`, `EpisodeMonitorModuleConfig` | `learning/collection/episode_monitor.py` | `EpisodeMonitorModule`; `DataPrep` |
| `EpisodeExtractor`, `StreamField`, `SyncConfig`, `OutputConfig`, `Episode`, `Sample` | `learning/dataprep.py` | `DataPrepModule`, `ChunkPolicyModule`, format writers |
| `DataPrepModuleConfig` | `learning/dataprep_module.py` | `DataPrepModule` |

---

## Artifact flow

All generated artifacts live under `data/` (gitignored at repo root):

```
data/
├── sessions/<name>.db              ← RecordReplay
└── datasets/<name>/                ← DataPrepModule.build()
    ├── data/        (parquet)
    ├── videos/      (MP4)
    └── meta/
        ├── info.json
        ├── episodes.jsonl
        ├── stats.json              (DataPrep.compute_stats)
        └── dimos_meta.json         (DataPrepModuleConfig.model_dump())
```

`dimos_meta.json` rides with the data: DataPrep writes it alongside the
dataset to record the obs/action schema.

---

## Configuration

All module config is set as kwargs in the blueprint. No CLI flags on
our modules. Framework CLI surface is `GlobalConfig` only (env vars,
`.env`, things like `--record-path`).

---

## Module / non-Module split

A class becomes a **Module** when it has long-lived state with
`start()/stop()` lifecycle **and** typed I/O ports.

| Class | Type | Why |
|---|---|---|
| `EpisodeMonitorModule` | Module | Long-lived; subscribes to inputs; publishes status |
| `DataPrepModule`       | Module | Long-running build job |
| `RecordReplay`         | transport hook | Captures every stream uniformly |
