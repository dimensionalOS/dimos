# SPACE map sketching

This suite integrates SPACE's `MapSketchingBEVText` task into the current DimOS
`EvalRunner`. The original framework proposal, #3378, was superseded by merged
#3411. A question describes a walk through an environment and asks the model to
choose its overhead map. Text is a bounded first integration: it exercises
spatial reasoning without introducing camera, video, or simulator differences.
It does not measure visual perception, navigation, or robot control.

## Setup

Use Linux or macOS, Python 3.12, and the repository's `uv` setup. From a clean
checkout:

```bash skip
uv sync --locked --no-default-groups --extra space
uv run --no-sync python -m dimos.evals.suites.lib.space.commands setup
uv run --no-sync python -m dimos.evals.suites.lib.space.commands run --offline-smoke
```

Setup fetches the pinned official source and one unchanged `qas.json` file into
DimOS's external cache (`CACHE_DIR/evals/space`). It streams the archive until
that member is found, checks its size and SHA-256, and stops; it does not download
the entire video dataset. Limits and source/data pins are in
[`evals/suites/lib/space/constants.py`](/dimos/evals/suites/lib/space/constants.py). Existing mismatched
inputs fail verification. Ordinary suite imports do not download anything.

The source is from [Apple's SPACE repository](https://github.com/apple-aiml-research/ml-space-benchmark),
revision `eec58a24516bfcd4554807b0a2b9d08b04eafd1c`. Upstream code uses the
[Apple Sample Code License](https://github.com/apple-aiml-research/ml-space-benchmark/blob/eec58a24516bfcd4554807b0a2b9d08b04eafd1c/LICENSE);
data uses [CC BY-NC-ND 4.0](https://github.com/apple-aiml-research/ml-space-benchmark/blob/eec58a24516bfcd4554807b0a2b9d08b04eafd1c/LICENSE_DATA).
Benchmark data is not vendored. Keep downloaded data, raw prompts, replies, and
replay inputs local; publish IDs, hashes, configuration, and derived metrics.

`space` installs the dependencies needed to import the unmodified evaluator,
including CPU-capable PyTorch because an upstream helper imports it eagerly.
No Habitat simulator or vLLM model is launched. The offline smoke sends a single
canned reply through DimOS and native SPACE scoring. Its output is explicitly
labelled software-contract evidence, not model performance.

## Run and score

Export the provider key in the shell, for example `OPENAI_API_KEY` for an OpenAI
model. The module command uses exported environment variables, not the main
`dimos` CLI's `.env` loading. Select an explicit model supported by the DimOS
model factory:

```bash skip
uv run --no-sync python -m dimos.evals.suites.lib.space.commands run --model gpt-5.6-luna --smoke
uv run --no-sync python -m dimos.evals.suites.lib.space.commands run --model gpt-5.6-luna
```

The smoke case is outside the frozen evaluation subset. The full run selects
20 of the 120 source rows: two distinct environment variants from each of ten
base layouts, with five correct answers in each answer position. The indices
are fixed in `SELECTED_INDICES`, before model evaluation. This avoids a prefix's
repeated question variants, but twenty related layouts remain a small sample.
Record every attempt; do not select runs or change prompts based on their scores.

`TextQuestion` uses the existing model factory and single-call agent path. It
sends only the original question: no system prompt, observations, tools, labels,
or metadata. Defaults are a 60-second provider timeout, zero retries, and 2,048
output tokens. Other settings come from the existing DimOS model factory; inspect saved requests for the
effective settings. `--timeout` bounds the whole runner process (default 1,800
seconds). The wrapper kills its owned process group on deadline or interruption;
the official scoring worker has a separate 120-second deadline. It logs to files
so a full stderr pipe cannot deadlock a run.

Each attempt gets a unique directory under `STATE_DIR/evals/space`. `job.json`
points to the runner's manifest, per-case results, ATIF trajectories, request and
response files, and a `space-score-*` replay report. Normal DimOS case scores
come from SPACE's `evaluate_on_qa`, normalized from percent to 0..1. The report's
aggregate comes from unmodified `space.evaluate_qas.main`, replaying the saved
responses without any additional model call. The official parser is used as-is,
including its permissive numeric conversion; invalid responses are not repaired.

To score a retained runner directory again:

```bash skip
uv run --no-sync python -m dimos.evals.suites.lib.space.commands score /path/to/run-directory
```

`report.json` lists correct, wrong, parser-invalid, out-of-range choice, timeout,
infrastructure-error, and unreported cases. Official accuracy includes completed
invalid replies. Its denominator excludes cases that did not complete; those
remain listed, and `complete` is false. An interrupted run with no final
`results.jsonl` stays unreported even when some trajectories exist. Incomplete
jobs exit nonzero; wrong answers alone do not make a completed evaluation fail.

The suite is also discoverable as `dimos.evals.suites.space_map_sketching` with
agent `dimos.evals.agents.text_question` through the standard eval CLI. Use the
wrapper above when the whole-job deadline and automatic official replay are
needed. It invokes the same runner, without a second model-execution system.

## Tests

With the repository's test dependencies installed, ordinary focused tests run
without SPACE downloads:

```bash skip
uv run --no-sync pytest dimos/evals/suites/lib/space dimos/evals/agents/test_text_question.py
uv run --no-sync pytest -m self_hosted dimos/evals/suites/lib/space
```

The second command requires the `space` extra and explicitly provisions pinned
inputs. It exercises the native parser, metric, spawned aggregation, and one
complete offline DimOS run. Missing dependencies are failures, not skipped
evidence. Tests use synthetic questions or external cached data; benchmark
questions are not committed as fixtures.
