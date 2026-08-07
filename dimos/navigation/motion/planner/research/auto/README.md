# planner/research/auto — the local-planner autoresearch lab

The evo lab that wrote the crate in [`planner/rust/`](../../rust/), plus
[`export/`](export/) — the tool that packages this repo's referee into a
standalone lab so the loop can run without write access to its own judge.

| | |
|---|---|
| lab repos | `autoresearch-planner` (current), `motion-autoresearch` (earlier epoch) |
| local checkouts | `~/coding/autoresearch-planner`, `~/coding/motion-autoresearch` |
| run host | frankfurt-research01 |
| referee | this repo: `planner/referee/` |
| candidate seam | `--planner <name>` or any `module:factory` |

## export/

```bash
python -m dimos.navigation.motion.planner.research.auto.export <dest>
```

Copies `planner/referee/` into `<dest>/referee/`, seeds `<dest>/candidate/`
from `planner/rust/`, and writes the bench + gate harness with content hashes
computed from the exported bytes. Those hashes are the boundary: `check_rules.py`
verifies the referee copy is byte-identical to what the lab is pinned to,
`ext_invariants.py` pins the referee's *runtime* (live code-object hashes and
constant values against `.evo/referee.lock`, so a `sitecustomize.py` cannot
monkeypatch the judge), and `bench_guard.py` lives outside the worktrees so a
candidate agent cannot disable the gates that live inside them.

This is why the labs are separate trees rather than folders in this repo. The
referee has to be immutable *from the candidate's side*, and a hash pin against
a vendored copy is what makes that checkable.

## Why the referee is one copyable unit

Every import inside `planner/referee/` is relative (`.types`, `..geometry`,
`.planners.base`) and the package depends on numpy + scipy + pydantic only —
so the directory works under any name, which is exactly what `python -m referee`
in an exported lab relies on. Do not split it, and do not let it grow the
MuJoCo dependency that `control/referee/` needs; that property is the export.

## Provenance

The seeded candidate's baseline, measured on the source battery (56 worlds,
gen 40 seed 0) at kit-authoring time: **106.66/111** referee score, gold 0.9629,
consistency 0.9473, speed 0.89 — see `exporter.py:BASELINE`. Step 6 of standing
up a new lab is reproducing those numbers with its own `./eval`.

Any change under `planner/referee/` obsoletes every lab's `referee.lock`:
re-export, re-baseline, and re-record before comparing across the change.
