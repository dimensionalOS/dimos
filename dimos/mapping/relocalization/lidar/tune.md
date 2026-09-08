# Tuning lidar relocalization

`relocalize()` places a small live cloud into a prior map. `tune.py` measures
whether it finds the right place, how tightly, and how fast. `tune` searches a
rig's `RelocalizeConfig` for a better tradeoff between those.

    uv run python -m dimos.mapping.relocalization.lidar.tune run --samples 12
    uv run python -m dimos.mapping.relocalization.lidar.tune run --samples 12 --view
    uv run --group dev python -m dimos.mapping.relocalization.lidar.tune tune --trials 200
    uv run --group dev python -m dimos.mapping.relocalization.lidar.tune verify --study <name>

### Setup, once

`tune` and `verify` need optuna, and everything here needs the rust
raycaster

    uv sync --group dev
    uv run maturin develop --uv -m dimos/mapping/ray_tracing/rust/py/Cargo.toml

## Tuning Dataset

A recording and a map we relocalize against **in one coordinate frame**

The usual way to get that is a single recording split into two.
One part is used for pulling lidar frames, the other part is used to assemble a premap.

(This is done in order to not use lidar frames used for a map in relocalization, match will be too perfect).

Given that we know coordinates from which lidar frame was pulled, when relocalizing we can measure error.

The shipped pair was built like this.

Both files are available in LFS so this is only for reproducing them:

    dimos map global recording_go2_mid360_2026-05-29_4-45pm-PST_corrected \
      --lidar fastlio_lidar --voxel 0.005 --seek 400 --export --no-gui

`--seek 400` does the split: the premap is the walk after 400 s, and probes
come from before it. `--export` implies `--pgo` and writes
`./<dataset>.pc2.lcm`. The walk is a loop, so the two halves still cover some
of the same ground. Without that they would never match.

Register the pair in `DATASETS`:

```python skip
"go2-sf-area1": Dataset(
    recording="recording_go2_mid360_2026-05-29_4-45pm-PST_corrected.db",
    premap="recording_go2_mid360_2026-05-29_4-45pm-PST_corrected.pc2.lcm",
    window=(0.0, 400.0),   # premap was built from the scans after 400 s so we use lidar frames before then
),
```

## Partial overlap is not optional

Dataset should revisit the part of the route we used for a premap building,
but not all of it, when tuning the relocalizer, we care about catching false positives.

System will auto-pick un-matchable frames and matchable frames when doing the tune.
`window` narrows where it looks.

# Probing

```sh ansi=False
uv run python -m dimos.mapping.relocalization.lidar.tune run --samples 5 --view
```

```results
go2-sf-area1  premap 6,900,428 pts  preset=mid360  cutoff=0.6  frames 3-7
┏━━━━━━━┳━━━━━━┳━━━━━━━━━┳━━━━━━━━━┳━━━━━━━━━┳━━━━━┳━━━━━━━┳━━━━━━┳━━━━━━┳━━━━━┓
┃ start ┃  cov ┃ where   ┃ outcome ┃   off m ┃ f/t ┃   fit ┃  lat ┃ wall ┃ cpu ┃
┡━━━━━━━╇━━━━━━╇━━━━━━━━━╇━━━━━━━━━╇━━━━━━━━━╇━━━━━╇━━━━━━━╇━━━━━━╇━━━━━━╇━━━━━┩
│    0s │ 100% │ in map  │ hit     │   0.157 │ 3/1 │ 0.913 │ 1.02 │ 0.70 │   4 │
│   99s │  99% │ in map  │ hit     │   0.147 │ 3/1 │ 0.854 │ 4.02 │ 3.69 │  42 │
│  198s │  99% │ in map  │ hit     │   0.021 │ 3/1 │ 0.903 │ 1.23 │ 0.90 │   7 │
│  296s │   0% │ outside │ refused │ 116.083 │ 7/2 │ 0.254 │ 3.97 │ 3.65 │  34 │
│  395s │  16% │ outside │ refused │  62.126 │ 7/2 │ 0.243 │ 6.16 │ 5.84 │  70 │
└───────┴──────┴─────────┴─────────┴─────────┴─────┴───────┴──────┴──────┴─────┘
100% of 3 in-map probes hit, 0% false fixes of 5 total (3 hit, 2 refused), 3.97s
latency / 34.1s cpu; 0.147 m off when hit
```

- **where** whether the premap has this place at all. The `outside` rows are
  the negative set, and a fix on one of them is a false fix.
- **off** the median distance the fix moved the cloud's own points.
  Translation and rotation do not add up into one number, because a small
  angle far from the cloud moves it more than a big angle through its middle.
  Displacement is what the robot actually feels.
- **f/t** frames used over attempts made. `7/2` means it gave up after two
  tries at `max_frames`.
- **fit** ICP's own score. Diagnostic only. It never decides correctness,
  because tuning against a score the aligner computes about itself is
  circular.
- **lat** the robot's wait: gathering the first scans, plus every attempt,
  including the failed ones.
- **wall** real seconds spent matching, with the waiting taken out. The gap
  between `lat` and `wall` is how much of the wait was gathering frames.
- **cpu** CPU seconds across every thread. Open3D threads FPFH and RANSAC, so
  this runs several times `wall`, by a ratio that moves with the parameters.
  On a robot the cores are shared, so it is a real price even when the wait
  is short.

## Tuning a new rig

`RelocalizeConfig`'s required fields are scales: voxel sizes, neighbourhood
radii, correspondence distances. They belong to a sensor and an environment,
not to relocalization in general. `PRESETS` names them after the rig they
were measured on. A mid360 walking an outdoor block is the only entry so far.
Do not nudge it to suit a new sensor, add one.

The procedure:

1. **Get a recording and a premap that share a frame.** The cheap way is one
   recording with its premap built from a different stretch of it:
   `dimos map global <recording> --seek <t> --export`. Ground truth is then
   the identity and nothing needs labelling.
2. **Register the pair** in `DATASETS`, with a `window` covering the stretch
   the premap does *not* come from. Include some ground the premap never saw.
   That is the only place a false fix can be caught, and a false fix is the
   failure that matters.
3. **Check the split is real** before trusting anything:
   `... run --samples 12`. The `cover` column should be near 1.0 for probes
   inside the map and near 0.0 outside, with nothing in between. If coverage
   is smeared, the window is wrong.
4. **Search**:
    `python -m dimos.mapping.relocalization.lidar.tune tune --trials 200 --samples 30`
    Samples are how many relocalization attempts we have per trial. we want wide samples like this
    otherwise we can overfit. 200 trials against 8 probes mostly finds lucky draws.

5. **Verify, always**:
    run top studies against previously unseen data to check for overfiting
    `python -m dimos.mapping.relocalization.lidar.tune verify --study <name> --top 8 --repeats 10`.

6. **Add the preset** in `relocalize.py`, named for the rig, with a comment
   saying which study and trial it came from.


## The objective

`tune` optimises five values at once:

- **good** share of probes placed within `TOLERANCE_M` / `TOLERANCE_DEG`,
  *and* accepted by the fitness cutoff.
- **bad** accepted but wrong. Worse than no fix at all: the module publishes
  it as a TF and everything downstream believes it.
- **error** median displacement of the good fixes. Without it a config
  landing at 2 cm and one at 45 cm score the same under a 50 cm tolerance.
- **lat** median wait per probe. It has to be in the objective and not just
  recorded. `ransac_iters` is the speed knob and more iterations never hurt
  quality, so a search that ignores time ends up on the slowest config there
  is.
- **cpu** median CPU per probe, which `lat` does not stand in for: a config
  can be quick on the clock while eating every core, and the ratio moves with
  the parameters.

`fitness_threshold` is tuned alongside the aligner because it is what turns a
fitness number into a decision. One catch: `icp_dist_factor` and `voxel_fine`
change what fitness *means*, so cutoffs cannot be compared across trials with
different values of those.

## Knobs

Every field of `RelocalizeConfig` was a literal in the aligner's body. The
scales are required, so there is no `RelocalizeConfig()` to fall into by
accident.

## Studies

Results live in `optuna.db` at the repo root, keyed
`<dataset>-v<SPACE>-n<frames>s<samples>`.

Running a trial with the same name will resume previous trial, it doesn't overwrite
You can run `uvx optuna-dashboard sqlite:///optuna.db` for a web dashboard at
`http://localhost:8080`
