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

## export/ — being retired, port in progress

`export/` builds a **vendored** lab: `planner/referee/` copied wholesale into
`<dest>/referee/`, pinned by byte-identity, run as `python -m referee`. That
worked because the referee was a self-contained unit with only relative imports
and a numpy+scipy+pydantic dependency.

It no longer is, deliberately. The domain that production and both referees
share — `motion/{types,geometry,scenarios}.py` — was lifted out of the referee
so there is exactly one copy of it, which means there is no self-contained
directory left to vendor. Two of the three labs never used the vendored model
anyway: `motion-tc-autoresearch` and `autoresearch-mlplanner` both run against
a dimos checkout, because the control referee genuinely needs MuJoCo, the
fitted sim and the planner crate.

**So the labs converge on the checkout model** (`ops/setup-server` +
`dimos-snap` + `ops/update`, as the TC lab does today), and `export/`'s copying
half retires with it. `test_export.py` is skipped at module level until the
port lands.

What must survive the port, because it is the check that actually catches
cheating: **`ext_invariants.py`'s runtime pin** — live code-object hashes and
constant values against `.evo/referee.lock` — retargeted from the copied
`referee.*` package onto the imported `dimos.navigation.motion.*` modules. A
byte-identity check on files cannot see a `sitecustomize.py` or a `.pth` file
monkeypatching the judge inside the interpreter; the runtime pin can. Keep
`bench_guard.py` too: it lives outside the experiment worktrees, which is what
stops a candidate agent from disabling the gates that live inside them.

What is dropped: `frozen.json`'s byte-identity of a copied tree, and
`test_referee_imports_standalone`. Both describe a vendored layout that no
longer exists.

The referee stays immutable *from the candidate's side* — that requirement did
not change. What changed is that it is enforced against an imported module set
and a re-pulled snapshot rather than against a directory the exporter copied.

## Provenance

The seeded candidate's baseline, measured on the source battery (56 worlds,
gen 40 seed 0) at kit-authoring time: **106.66/111** referee score, gold 0.9629,
consistency 0.9473, speed 0.89 — see `exporter.py:BASELINE`. Step 6 of standing
up a new lab is reproducing those numbers with its own `./eval`.

Any change under `planner/referee/` obsoletes every lab's `referee.lock`:
re-export, re-baseline, and re-record before comparing across the change.
