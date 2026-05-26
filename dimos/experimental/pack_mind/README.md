# PACK MIND

A shared semantic memory for a team of Unitree Go2s.
**Alpha sees. Bravo remembers. The pack acts.**

Most multi-robot systems share a *map*. Pack Mind shares *meaning*: one dog finds
something, another dog answers from that shared memory and acts on it — without any
shared map, SLAM, or coordinate exchange. Cross-dog handoff is by **zone name** only.

## What this is

- **`conductor.py`** — the only shared layer. Roster + append-only event blackboard +
  deterministic mission state machine + movement lock. Talks to each dog over MCP
  JSON-RPC (HTTP). Serves the dashboard and a small action API.
- **`dashboard.html` / `static/style.css`** — the projector surface that makes the
  shared memory visible: causal event cards, roster, mission state, operator controls.

Each dog runs its own standard `unitree-go2-agentic` stack. The conductor is net-new.

## Run it (no hardware)

```bash
uv run python dimos/experimental/pack_mind/conductor.py --mock
# open http://localhost:8080
```

Drive the whole Alpha → Bravo story from the dashboard buttons. `--mock` logs MCP
calls instead of making them, so the full demo runs on a laptop.

## Run it (real dogs)

Each dog, on its own compute box:

```bash
dimos run unitree-go2-agentic --robot-ip <dog-ip> --listen-host 0.0.0.0 \
  --mcp-port 9990 --viewer none --daemon
```

Then the conductor, from the control machine:

```bash
uv run python dimos/experimental/pack_mind/conductor.py \
  --dog alpha=<alpha-ip> --dog bravo=<bravo-ip>
```

`--listen-host 0.0.0.0` on each dog is required — the MCP server binds `127.0.0.1`
by default and the conductor would get connection-refused otherwise.

## Demo flow (the 6 controls)

1. **Start Act 1** — Alpha begins scouting.
2. **Inject: Alpha found red backpack @ Zone B** — operator fallback, identical
   blackboard effect to a real VLM detection.
3. **Ask Bravo where backpack is** — Bravo answers from shared memory, not its own.
4. **Send Bravo to Zone B** — Bravo navigates to its *own* `zone_b` (movement lock held).
5. **Verify success** — Bravo self-checks at the zone.
6. **Emergency stop** — release locks, all dogs hold.

Hidden fallback buttons are not cheating. Broken live autonomy is.

> NOTE: MCP tool argument names (`navigate_with_text`, `speak`, `look_out_for`) are
> passed through verbatim. Verify against the skill signatures in `dimos/agents/skills/`
> on the first real-hardware test.
