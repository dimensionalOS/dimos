# Galaxea R1Pro — pick & place in sim

A Galaxea **R1Pro** dual-arm robot doing pick-and-place in a MuJoCo office scene:
objects are detected on the desk, the planner (RoboPlan + Pink IK) reaches for the
nearer-hand object with full dual-arm + torso motion, the gripper grasps, and the
arm lifts. Everything is shown live in a [viser](https://viser.studio) 3D scene.

Two ways to drive it:

- **Scripted** (`drive_pick.py`) — deterministic, no API key. **Start here.**
- **Agentic** — a gpt-4o agent takes natural-language commands over MCP.

---

## What you'll see

- A **MuJoCo viewer window** — the physics sim (needs a desktop/display).
- A **viser scene in your browser** at **http://127.0.0.1:8095** — the robot, a
  translucent target/preview "ghost" of the planned motion, the desk objects, HDRI
  lighting and a ground grid.

## Prerequisites

- Linux with a desktop session (so the MuJoCo window and a browser can open). A GPU
  is recommended for the MuJoCo render.
- [`uv`](https://docs.astral.sh/uv/) and [`direnv`](https://direnv.net/).
- The repo cloned **with its Git LFS assets** — the R1Pro meshes and the office scene
  package are stored in LFS.

## One-time setup

```bash
git lfs install && git lfs pull          # fetch the meshes + scene package
uv sync --extra manipulation             # install sim + planner + viser deps
direnv allow                             # load the project env
```

---

## Run it — scripted (recommended)

Use **two terminals**.

**Terminal 1 — start the sim.** The R1Pro sim defaults to the bundled office scene,
so no extra config is needed:

```bash
source .venv/bin/activate
dimos stop                 # clear stale shared-memory from any prior run
dimos run r1pro-perception-sim
```

Wait until it logs the viser URL, then open **http://127.0.0.1:8095** in your browser.

**Terminal 2 — drive a pick** (home → scan → pick, then holds the object up):

```bash
source .venv/bin/activate
python examples/r1pro_pickplace/drive_pick.py cup
```

Swap `cup` for any object on the desk (e.g. `can`, `bottle`, `box`). Run it again to
pick another object.

> **Always `dimos stop` before each fresh run** — it frees the shared-memory segments
> the sim uses. Skipping it is the #1 cause of a stuck or stale run.

---

## Run it — agentic (natural language)

Needs an **OpenAI API key** (the agent runs gpt-4o):

```bash
export OPENAI_API_KEY=sk-...
source .venv/bin/activate
dimos stop
dimos run r1pro-perception-sim-agent
```

Open the same viser URL. Then, from a second terminal, open the agent chat:

```bash
source .venv/bin/activate
humancli
```

Type a natural-language instruction such as *"scan the desk and pick up the cup"*.
The agent calls the `scan_objects` / `pick` skills over MCP and you watch the plan
execute in viser.

To drive it with **Claude** instead of gpt-4o, set `_R1PRO_AGENT_MODEL` to e.g.
`"anthropic:claude-sonnet-4-5-20250929"` in
[`dimos/manipulation/blueprints.py`](../../dimos/manipulation/blueprints.py) and export
`ANTHROPIC_API_KEY` (plus `uv pip install langchain-anthropic`).

---

## Troubleshooting

| Symptom | Fix |
| --- | --- |
| "address already in use" / sim seems stuck | `dimos stop`, then rerun — stale shared memory. |
| viser page is blank | Confirm Terminal 1 logged `Viser manipulation visualization: http://127.0.0.1:8095`, then reload. |
| `'cup' not found` from the driver | Re-run `drive_pick.py`; the scan repopulates the detections each call. |
| Missing meshes / robot is invisible | You skipped LFS — run `git lfs pull`. |

## How it fits together

- **Blueprints** (`dimos/manipulation/blueprints.py`): `r1pro-perception-sim` (skills only)
  and `r1pro-perception-sim-agent` (adds the gpt-4o MCP agent).
- **Sim**: MuJoCo via the shared-memory engine; the R1Pro asset is `r1pro_dual.xml`.
- **Planning**: RoboPlan world + Pink IK, multi-target so both arms + the torso move.
- **Visualization**: in-process viser server (`dimos/manipulation/visualization/viser/`).
