# PGO → jnav Loop-Closure Migration Plan

Goal: land the pose-graph-optimization (PGO) / loop-closure work on
`jeff/feat/jnav_pgo` in the new `jnav` layout, extract the C++ + nix flake into a
standalone `github:jeff-hykin/gsc_pgo` repo, and merge in the offline AprilTag
map-postprocessing tooling.

## Source material

Two branches hold the pieces; neither alone is the target.

| Source | Has | Layout |
|---|---|---|
| `jeff/feat/jnav` | `jnav/{msgs,utils}/`, `loop_closure/{gsc_pgo,ivan_pgo,ivan_pgo_transformer,unrefined_pgo}`, `eval.py`, `eval_all.py`, Scan-Context + Landmark C++ | `dimos/navigation/jnav/modules/...` (inline C++) |
| `jeff/feat/better_pgo` | postprocessing scripts (`add_april.py`, `detect_tags.py`, `post_process.py`, `make_rrd.py`), `map_postprocessing.md` doc | `dimos/navigation/nav_stack/modules/pgo/scripts/...` |
| `jeff/feat/jnav_pgo` (current) | nothing yet — only `nav_stack→cmu_nav` rename on top of `main` | — |

`jeff/feat/jnav` is the more evolved navigation reorg; `better_pgo` is the source
for the AprilTag postprocessing scripts/doc. The migration is the union of both.

## Target layout (on `jeff/feat/jnav_pgo`)

```
dimos/navigation/jnav/
  msgs/                         # Graph3D/GraphDelta3D/Landmark/Marker (.py + .hpp + tests)
  utils/                        # ALL pgo utils (apriltags, trajectory_metrics, recording_db, ...)
  components/                   # renamed from jnav's "modules/"
    loop_closure/
      eval.py
      eval_all.py
      spec.py                   # LoopClosure Protocol
      gsc_pgo/
        module.py               # NativeModule -> builds external gsc_pgo flake
        scripts/
          post_process.py       # ported from better_pgo scripts/post_process.py
          (add_april.py, detect_tags.py, make_rrd.py)  # the other postprocess scripts
experimental/docs/jnav/
  map_postprocessing.md         # adapted from docs/capabilities/navigation/map_postprocessing.md
```

External repo:
```
github:jeff-hykin/gsc_pgo       # all C++ + flake.nix/flake.lock
  flake.nix, flake.lock, CMakeLists.txt
  main.cpp, simple_pgo.{cpp,h}, scan_context.{cpp,h},
  commons.{cpp,h}, point_cloud_utils.hpp, dimos_native_module.hpp,
  pgo_landmark_test.cpp
  msgs/  (Graph3D.hpp, GraphDelta3D.hpp, Landmark.hpp)  # C++ wire helpers
```

## Decisions (confirmed by Jeff, 2026-06-22)

1. **Base branch.** ✅ Base `jnav_pgo` on `jeff/feat/jnav`. BUT `post_process.py`
   must come from `better_pgo` (not jnav, which lacks it).
2. **`components/` scope.** ⏳ pending (whole `jnav/modules/`→`components/` vs only
   `loop_closure`).
3. **`gsc_pgo` repo.** ✅ **public**, created at
   `github.com/jeff-hykin/gsc_pgo`, initial rev `494e7a1d657c3702ec805c9e3d251a2fe8bc9529`.
   Flake input will pin to that rev: `github:jeff-hykin/gsc_pgo/494e7a1...#default`.

## Work breakdown

### Phase 0 — branch setup
- Confirm `jnav_pgo` base (decision 1). If basing on jnav: merge/cherry-pick the
  `dimos/navigation/jnav/` tree onto current `jnav_pgo` (which already has the
  `cmu_nav` rename). Resolve any overlap with cmu_nav's own `pgo` module.

### Phase 1 — external `gsc_pgo` repo
- `gh repo create jeff-hykin/gsc_pgo` (visibility per decision 3). *(outward-facing — confirm first)*
- Move `gsc_pgo/cpp/*` (C++, CMakeLists, flake.nix, flake.lock, `msgs/*.hpp`,
  `pgo_landmark_test.cpp`) into the new repo. Keep the flake's
  `lcm-extended` / `dimos-lcm` / `gtsam-extended` inputs as-is.
- `flake.nix`: `src = ./.;` already self-contained — switching to a tracked git
  repo *fixes* the untracked-files gotcha that forced `path:$PWD#default`
  (see memory: nix untracked-files gotcha). Verify `nix build .#default` from a
  clean checkout.
- Push, tag/record the rev for the flake pin.

### Phase 2 — `gsc_pgo` module.py
- Update `PGOConfig`:
  - `build_command = 'nix build "github:jeff-hykin/gsc_pgo/<rev>#default" --no-write-lock-file'`
  - drop the `path:$PWD` workaround comment; `cwd` no longer needs a local `cpp/`.
  - `executable` resolves from the nix `result/bin/pgo` (confirm NativeModule
    out-of-tree build path handling).
- Keep `In/Out` stream wiring and all loop-closure hyperparameters unchanged.
- Confirm C++ `msgs/*.hpp` wire format still matches `jnav/msgs/*.py` decoders
  (they're hand-synced; the `.hpp` headers travel with the C++ repo).

### Phase 3 — msgs/ + utils/
- `jnav/msgs/`: already present on jnav (Graph3D, GraphDelta3D, Landmark, Marker +
  `.hpp` + tests). Bring over as-is. The `.hpp` files are duplicated into the
  external repo (C++ side); decide whether the canonical `.hpp` lives in
  `gsc_pgo` and `jnav/msgs/*.hpp` is a mirror, or vice-versa. Recommend: canonical
  in `gsc_pgo`, keep a note in `jnav/msgs` pointing at it.
- `jnav/utils/`: bring all utils. `better_pgo`'s `eval_utils/` (apriltags,
  apriltag_agreement, trajectory_metrics, recording_db, voxel_map, module_loading)
  are already absorbed into jnav `utils/` — verify no newer logic in
  `better_pgo` was lost (diff the 6 overlapping files).

### Phase 4 — eval.py / eval_all.py
- Copy from jnav `loop_closure/`. Fix imports to `dimos.navigation.jnav.utils.*`
  and `...jnav.msgs.*` (already correct on jnav).
- `eval_all.py` enumerates the comparison modules (`gsc_pgo`, `ivan_pgo`,
  `ivan_pgo_transformer`, `unrefined_pgo`) → port all four component dirs so eval
  doesn't break. (If baselines are unwanted, prune eval_all's list instead.)

### Phase 5 — postprocessing scripts (from better_pgo)
- Port `scripts/post_process.py` → `components/loop_closure/gsc_pgo/scripts/post_process.py`.
- Port `add_april.py`, `detect_tags.py`, `make_rrd.py` alongside it (the doc's
  3-step flow references all of them).
- Rewrite their internal imports: `eval_utils.apriltags` → `dimos.navigation.jnav.utils.apriltags`, etc.
- Tag quality gates are single-sourced in `utils/apriltags.py` (`DEFAULT_*`); keep
  post_process importing from there (don't duplicate constants).

### Phase 6 — docs
- Adapt `docs/capabilities/navigation/map_postprocessing.md` →
  `experimental/docs/jnav/map_postprocessing.md`.
- Rewrite every script path in the doc:
  `dimos/navigation/nav_stack/modules/pgo/scripts/X.py`
  → `dimos/navigation/jnav/components/loop_closure/gsc_pgo/X.py`.
- Update the `eval_utils/apriltags.py` reference → `jnav/utils/apriltags.py`.
- Add a back-link from the navigation readme if appropriate.

### Phase 7 — wiring + tests
- `spec.py` `LoopClosure` Protocol: bring over, update any module paths.
- Blueprints: if any blueprint references the old `nav_stack`/`pgo` module path,
  repoint to `jnav.components.loop_closure.gsc_pgo`. Regenerate
  `all_blueprints.py` via `pytest dimos/robot/test_all_blueprints_generation.py`.
- Port tests (`test_pgo_synthetic_drift.py`, msgs tests, utils tests). Run
  `uv run pytest` for the loop_closure + msgs + utils dirs.
- `ruff check --fix && ruff format`.

## Risks / cut-corners to flag
- **C++ wire-format drift**: `.hpp` (C++) and `.py` (decode) are hand-synced. Once
  the `.hpp` lives in an external repo, a change there can silently desync the
  Python decoder. Mitigate: keep a `test_Graph3D` round-trip test in `jnav/msgs`
  that builds against the pinned flake rev, or at least a schema-comment check.
- **Private flake in CI**: if `gsc_pgo` is private, dimos CI nix builds will fail
  without a token — do not add a token to CI without asking (secrets rule).
- **eval_results/**: jnav carries committed `eval_results/*/summary.json` snapshots.
  Decide whether to bring those (history/benchmarks) or regenerate.
- **Baseline PGO impls**: `ivan_pgo*` / `unrefined_pgo` carry their own inline C++
  (`unrefined_pgo/cpp`). If only `gsc_pgo` goes external, the layout is asymmetric
  — acceptable (they're experimental baselines) but worth noting.
