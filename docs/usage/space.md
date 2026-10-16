# SPACE map sketching

This suite runs SPACE's official `MapSketchingBEVText` task through dimOS's
`EvalRunner` and standard eval CLI.

## Task selection

Map sketching asks the model to choose an overhead map from observations of a
walk. It tests reasoning across viewpoints, with objective scoring and no image
pipeline or simulator. Questions and option order are unchanged. Results cover
this text task, not perception, navigation, or SPACE as a whole.

## Setup

Use Linux or macOS, Python 3.12, and the repository's `uv` setup:

```bash skip
uv sync --locked --no-default-groups --extra space
uv run --no-sync dimos evals setup space
```

The `space` extra installs the pinned `ml-space-benchmark-ky135` package.
Setup downloads one unchanged
`qas.json` into `CACHE_DIR/evals/space`. It streams only as far as that archive member, verifies
its size and SHA-256, and rejects mismatched cached inputs. Pins and limits are in
[`evals/suites/lib/space/constants.py`](/dimos/evals/suites/lib/space/constants.py).
Suite imports do not download anything.

The package preserves source revision `eec58a24516bfcd4554807b0a2b9d08b04eafd1c` from
[Apple's SPACE repository](https://github.com/apple-aiml-research/ml-space-benchmark) under the
[Apple Sample Code License](https://github.com/apple-aiml-research/ml-space-benchmark/blob/eec58a24516bfcd4554807b0a2b9d08b04eafd1c/LICENSE).
Data uses [CC BY-NC-ND 4.0](https://github.com/apple-aiml-research/ml-space-benchmark/blob/eec58a24516bfcd4554807b0a2b9d08b04eafd1c/LICENSE_DATA)
and is not vendored. Keep data, prompts, replies, and replay inputs local;
publish IDs, hashes, configuration, and derived metrics. Check the terms before
commercial use or redistribution.

The `tests-self-hosted` group includes the `space` extra; `all` and default tests
do not select it. The SPACE package includes PyTorch because an upstream helper imports
it eagerly.
No Habitat simulator or vLLM model is launched.

## Run and score

Configure a provider key as for other dimOS evals. This OpenAI example sets an
explicit output budget:

```bash skip
uv run --no-sync dimos evals run dimos.evals.suites.space_map_sketching \
  --agent dimos.evals.agents.text_question \
  --set model=gpt-5.6-luna --set max_output_tokens=16384
uv run --no-sync dimos evals score space /path/to/run-directory
```

Use the directory printed by the run command. Each attempt saves a manifest,
case results, ATIF trajectories, and requests/responses under `STATE_DIR/evals`.
Scoring replays saved replies without provider calls. Each invocation writes a
new `space-score-*` directory inside the run directory, preserving the original
results and trajectories.

`SELECTED_INDICES` freezes 20 of 120 rows before execution: two distinct variants
per layout across ten layouts, with five correct answers per answer position.
This avoids repeated answer-position variants in a file-order prefix. The
related variants are not independent environments. `--limit` can restrict a run
to fewer cases.

The suite uses `TextQuestion` and the existing model factory to send only the
original question, without a system prompt, observations, tools, labels, or
metadata. For OpenAI, the agent defaults to a 60-second provider timeout, zero
retries, and 2,048 output tokens. The example raises that cap because reasoning
can exhaust it before an answer is emitted. The timeout does not bound the whole
run. The scoring subprocess has a 120-second timeout, followed by worker cleanup.

Case scores use SPACE's `evaluate_on_qa`, normalized from percent to 0..1.
The authoritative aggregate uses unmodified `space.evaluate_qas.main`.
Empty and malformed replies remain in the official denominator. Inspect provider
responses to distinguish token-exhausted replies from completed answers.

`report.json` records IDs, question hashes, case scores, the aggregate, and counts
for correct, wrong, invalid, out-of-range, timeout, infrastructure-error, and
unreported cases. `complete` means every selected case was scored without
results-file corruption; `fixed_subset_complete` also requires the full frozen
subset. Neither guarantees correct answers or normal provider completion.
Missing, failed, and timed-out cases remain visible but unscored.

Without `results.jsonl`, a run stays unreported even if trajectories exist.
Malformed results are rejected without changing the saved run. The score command
exits nonzero for incomplete reports or malformed input, not low accuracy or
invalid replies alone.

## Tests

With the repository's test dependencies installed, run the tests that need no
external benchmark data:

```bash skip
uv run --no-sync pytest dimos/evals/suites/lib/space dimos/evals/agents/test_text_question.py
```

Run the official evaluator integration tests:

```bash skip
uv sync --locked --group tests --extra space
uv run --no-sync pytest -m self_hosted dimos/evals/suites/lib/space/test_official.py
```

These tests check the fixed subset, official parser and metric, spawned aggregation,
and dimOS CLI with a canned model. They reuse verified cached inputs or download pinned
task data without provider calls. They verify integration, not model
performance.
