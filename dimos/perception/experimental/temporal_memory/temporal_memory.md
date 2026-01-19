Dimensional Temporal Memory is a lightweight Video RAG pipeline for building
entity-centric memory over live or replayed video streams. It uses a VLM to
extract evidence in sliding windows, tracks entities across time, maintains a
rolling summary, and persists relations in a compact graph for query-time context.

How It Works
1) Sample frames at a target FPS and analyze them in sliding windows.
2) Extract dense evidence with a VLM (caption + entities + relations).
3) Update a rolling summary for global context.
4) Persist per-window evidence and the entity graph for fast queries.

Setup
- Add your OpenAI key to `.env`:
  `OPENAI_API_KEY=...`
- Install dependencies (recommended set from repo install guide):
  `uv sync --extra dev --extra cpu --extra sim --extra drone`

`uv sync` installs the locked dependency set from `uv.lock` to match the repo's
known-good environment. `uv pip install ...` behaves like pip (ad-hoc installs)
and can drift from the lockfile.

Quickstart
- Run Temporal Memory on a replay:
  `dimos --replay run unitree-go2-temporal-memory`
- In another terminal, open a chat session:
  `humancli`

Artifacts
By default, artifacts are written under `assets/temporal_memory`:
- `evidence.jsonl` (window evidence: captions, entities, relations)
- `state.json` (rolling summary + roster state)
- `entities.json` (current entity roster)
- `frames_index.jsonl` (timestamps for saved frames; written on stop)
- `entity_graph.db` (SQLite graph of relations/distances)

Notes
- Evidence is extracted in sliding windows; queries can reference recent or past entities.
- Distance estimation can run in the background to enrich graph relations.
- Change the output location via `TemporalMemoryConfig(output_dir=...)`.
