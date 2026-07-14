# MCP skill verification

A regression/acceptance harness for the perception & scene-understanding skills
in [`../SKILLS_WISHLIST.md`](../SKILLS_WISHLIST.md). Run it after code cleanup —
or any change to perception/mapping code — to confirm each skill still works and
to **eyeball what it produces**.

Each test drives the *real* skill code (not a mock of it), asserts
machine-checkable invariants, and writes the artifacts a human should look at
into `test_outputs/<skill-id>/`. After a run, `test_outputs/REVIEW.md` indexes
every artifact with *what you should see*.

## What's covered

One test module per implemented wishlist skill; see [`manifest.yaml`](manifest.yaml)
for the authoritative per-skill description (input used, how/why tested, expected
outputs). Idea-stage items (4 temporal counts, 6/7 activity) have no skill yet
and are listed under `not_covered` in the manifest.

| id | skill | tier | input |
| --- | --- | --- | --- |
| 01-floorplan | `generate_floorplan` | real-data | a recording (chinaOffice.rrd) |
| 02-identify-locate | `detect`/`select`/`list_observed_items`/`catalog_scene` | headless | synthetic scene |
| 03-lidar-to-numpy | `LidarPointCloudClient` | headless | synthetic scanner |
| 05-contextual-labeling | `identify_object`/`refine_observed_labels` | headless | synthetic + stub VLM |
| 08-scene-3d-model | `SceneModel` sections/mesh | headless | synthetic house |

**Tiers.** `headless` tests use synthetic/in-code inputs — no GPU, API keys, or
network — and run anywhere. The `real-data` floorplan test generates its drawing
from a `.rrd` recording; it's gated on data presence (not a pytest marker), so
it runs wherever a recording + `ezdxf` are available and skips cleanly otherwise.

## Running it

```sh
# The whole suite. The 5 headless tests run in ~10 s; the floorplan test also
# runs (~1.5 min) when a recording is present, else skips:
uv run pytest dimos/skills/skill_verification

# Fast headless-only (skip the slow floorplan generation):
uv run pytest dimos/skills/skill_verification -k "not floorplan"

# Then review:
open dimos/skills/skill_verification/test_outputs/REVIEW.md
```

**Don't clear `addopts` (`-o addopts=""`)** — the repo default config carries
flags this suite relies on; if you must override the marker filter, pass `-m ""`
rather than clearing the whole thing.

**Tiers.** The headless tests (items 2, 3, 5, 8) use synthetic in-code inputs —
no GPU, keys, network, or recordings — and always run. The floorplan test
(item 1) is *data-gated*: it generates the DXF **from a `.rrd` recording** (no
existing drawing needed), so it runs wherever a recording + `ezdxf` are present
and skips cleanly otherwise (CI, fresh checkout). It resolves the recording from
`$SKILL_VERIFY_RRD`, else `chinaOffice.rrd` at the repo root. `ezdxf` ships in
the `mapping` extra (`uv sync --extra mapping`); recordings are large and
gitignored, so supply your own.

## Reviewing the output

Open `test_outputs/REVIEW.md`. For every skill it lists each artifact with a
✅ (produced this run) or ⚪ (its tier was skipped) and a one-line *what to
check*. Compare visual artifacts against the committed reference figures under
[`../../../misc/skills_dashboard/data/`](../../../misc/skills_dashboard/data)
when in doubt — those were generated from the same skill code.

`test_outputs/` is gitignored; it's a scratch area rebuilt on each run.

## Adding a skill

1. Add a `test_NN_<slug>.py` with a module-level `SKILL_ID`, taking the
   `skill_output` fixture; save artifacts via `skill_output.produced(name)`.
2. Add the matching entry to `manifest.yaml` (all fields — `test_00_manifest.py`
   enforces the manifest ↔ test-file correspondence and field completeness).
