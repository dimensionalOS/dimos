Temporal memory runs "Temporal/Spatial RAG" on streamed videos building an continuous entity-based
memory over time. It uses a VLM to extract evidence in sliding windows, tracks
entities across windows, maintains a rolling summary, and stores relations in a graph network.

Methodology
1) Sample frames at a target FPS and analyze them in sliding windows.
2) Extract dense evidence with a VLM (caption + entities + relations).
3) Update rolling summary for global context.
4) Persist per-window evidence + entity graph for query-time context.

Setup
- Put your OpenAI key in `.env`:
  `OPENAI_API_KEY=...`
- Install dimensional dependencies

Quickstart
To run: `dimos --replay run unitree-go2-temporal-memory`

In another terminal: `humancli` to chat with the agent and run memory queries.

Artifacts
By default, artifacts are written under `assets/temporal_memory`:
- `evidence.jsonl` (window evidence: captions, entities, relations)
- `state.json` (rolling summary + roster state)
- `entities.json` (current entity roster)
- `frames_index.jsonl` (timestamps for saved frames; written on stop)
- `entity_graph.db` (SQLite graph of relations/distances)

Notes
- Evidence is extracted in sliding windows, so queries can refer to recent or past entities.
- Distance estimation can run in the background to enrich graph relations.
- If you want a different output directory, set `TemporalMemoryConfig(output_dir=...)`.
