---
title: "SPACE Benchmark"
---

[SPACE](https://github.com/apple/ml-space-benchmark) (*Spatial Perception And
Cognition Evaluation*, [paper](https://arxiv.org/abs/2410.06468)) measures
large-scale spatial cognition in two stages: a video walkthrough familiarizes
the model with an environment, then five tasks probe the spatial representation
it built. All five run through the [eval framework](/docs/usage/evaluation/evals.md):

| Suite | Kind | Per presentation | Scored by |
|---|---|---|---|
| `space.direction` | multiple-choice QA | 600 questions | benchmark's parser + exact match |
| `space.distance` | multiple-choice QA | 540 questions | benchmark's parser + exact match |
| `space.map_sketching` | multiple-choice QA | 120 questions | benchmark's parser + exact match |
| `space.route_retracing` | interactive episode | 30 environments | benchmark's SPL |
| `space.shortcut_discovery` | interactive episode | 30 environments | benchmark's SPL |

Each suite carries both media presentations, told apart by tags: `bevimage`
(top-down walkthrough video) and `ego` (first-person walkthrough, the setting
a robot actually lives in). The prose presentations are deliberately not wired
up: they have no visual pathway, which is the part a robot stack cares about.

## Setup

Nothing is redistributed with dimOS (code: Apple Sample Code License; data:
CC BY-NC-ND 4.0):

```bash
bash scripts/eval/space.sh           # clones the benchmark, downloads the data (~3.4 GB)
pip install 'dimos[space]'           # the packages its parser imports
```

The script pins the benchmark checkout to the revision the reported scores
were produced against (`DIMOS_SPACE_REV` overrides it). Scoring runs from that
checkout unmodified, so the pin is what keeps runs comparable.

Default locations sit under `~/.cache/dimos/space`; override with
`DIMOS_SPACE_DATA_DIR` / `DIMOS_SPACE_REPO`. Without the data every suite is
empty and imports still work. Frame sampling and episode limits mirror the
benchmark's own defaults and live in `dimos/evals/suites/space/_config.py`.

## Running

```bash
dimos evals run dimos.evals.suites.space.distance        --tags bevimage --limit 20 --system-prompt ""
dimos evals run dimos.evals.suites.space.direction       --tags ego      --limit 20 --system-prompt ""
dimos evals run dimos.evals.suites.space.route_retracing --tags bevimage --limit 5  --system-prompt ""
```

`--system-prompt ""` is required, not cosmetic: SPACE questions carry their own
instructions and ask the model to think step by step, which the default
`EVAL_SYSTEM_PROMPT` ("reply with the answer value only") contradicts.

`--blind` withholds exactly the walkthrough and keeps the question: the
paper's own ablation. A QA case that still passes blind is guessable; chance on
the four-way items is 25%.

### Egocentric navigation needs a habitat-sim interpreter

The `ego` cases of the two navigation suites render observations online, which
requires [habitat-sim](https://github.com/facebookresearch/habitat-sim), a
native simulator with x86_64 Linux conda packages only (no GPU required). Its
builds exist for Python 3.9 alone, while dimOS requires 3.10+, so the
environment runs in a sidecar interpreter (`suites/space/_ego_env.py`) that the
episode drives over stdio; the model loop stays in dimOS:

```bash
micromamba create -n habitat -c conda-forge -c aihabitat     python=3.9 habitat-sim=0.3.0 headless
export DIMOS_SPACE_HABITAT_PYTHON=~/micromamba/envs/habitat/bin/python
```

On machines without it those cases fail preflight with this pointer; everything
else runs anywhere.

## Fidelity

- **Scoring is upstream's, unmodified.** QA replies go through SPACE's
  `parse_answer_from_response` (called unbound; it never touches `self`) and
  are compared exactly as `space/evaluate_qas.py` does. Episodes are scored by
  `space.envs.nav_dm.evaluate_path_efficiency` and the habitat SPL classes.
  `test_scoring_matches_the_official_metric` pins the QA agreement case by
  case; the episode test drives a real environment and checks a ground-truth
  replay scores SPL 1.0.
- **Prompts are upstream's wording**, imported where they are constants and
  copied verbatim (with source pointers) where they live inside agent methods.
- **One disclosed deviation:** SPACE's navigation agents keep a multi-turn
  dialog; dimOS's rig is single-turn, so episodes resend the transcript
  flattened into one message per step, prior replies quoted as text.
- **Ordering:** rows ship in groups of four rotating the correct option through
  positions 1-4. In file order, `--limit 20` would buy five questions asked
  four times each with the answer key 1,2,3,4,1,2,3,4; the suites reorder so
  any prefix stays a spread sample (`_bench.spread`).

## Adding another benchmark

Give it a directory: `dimos/evals/suites/<bench>/` with a `_config.py` exposing
a module-level `config` (environment-variable defaults), `_`-prefixed internals,
and one module per task exporting `SUITE`. Discovery is automatic; there is
nothing to register.
