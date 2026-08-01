# Spatial benchmark workflow

This guide covers three tasks: inspect benchmark cases, run a Pi evaluation,
and review a retained agent session. Run commands from the DimOS repository.

The checked-in
[10-case demo](/docs/development/spatial_benchmark_10_case_demo/authoring-spec.json) provides a
complete authoring spec and pinned
[Pi configuration](/docs/development/spatial_benchmark_10_case_demo/pi-config.json). It covers
all seven predicates on clean maps and stores runtime evidence under
`~/spatial-real-pilot/benchmark-runs/spatial-10-case-demo-20260728`.
Its [verified result](/docs/development/spatial_benchmark_10_case_demo/verified-result.md)
records the completed run and a directly viewable, correct session with 11
Python commands.

## Inspect benchmark data

Start the read-only map viewer with agent-visible evidence only:

```sh
uv run --no-sync python -m dimos.benchmark.spatial.cli view \
  --root ~/spatial-real-pilot \
  --public-only
```

Open the printed `http://127.0.0.1:8080` URL. Use the controls to change the
question, predicate, sample, and map variant. Omit `--public-only` only when
private oracle context is appropriate.

Filter the initial case when needed:

```sh
uv run --no-sync python -m dimos.benchmark.spatial.cli view \
  --root ~/spatial-real-pilot \
  --predicate direct-neighbor-count \
  --variant clean \
  --public-only
```

Other selectors include `--instance-id`, `--scene-id`, `--trajectory-id`, and
`--question-id`. Press `Ctrl-C` to stop the viewer.

## Run an evaluation

Keep benchmark evidence outside `/tmp` so it survives cleanup:

```sh
export CORPUS_ROOT="$HOME/spatial-real-pilot"
export RUN_ROOT="$CORPUS_ROOT/benchmark-runs/my-eval"
export EXPERIMENT_ID="my-eval"
mkdir -p "$RUN_ROOT/config" "$RUN_ROOT/private" "$RUN_ROOT/public"
```

Create `"$RUN_ROOT/config/authoring-spec.json"`. It names a Pi configuration,
the exact public cases, and one or more prompt conditions:

```json
{
  "experiment_id": "my-eval",
  "pi_config": "pi-config.json",
  "cases": [
    {
      "case_id": "case-1",
      "selection": {
        "scene_id": "scene_...",
        "trajectory_id": "trajectory_...",
        "question_id": "question_...",
        "variant": "clean",
        "instance_id": "instance_..."
      }
    }
  ],
  "conditions": [
    {
      "name": "visualization-forbidden",
      "prompt_mode": "visualization-forbidden"
    }
  ]
}
```

Place a verified `pi-config.json` beside the spec. It defines the pinned model,
adapter, runner image, resource limits, tool budgets, and implementation
digests. Update its output, corpus, oracle, private, and ledger paths for the
new durable run. Validate it before creating the experiment:

```sh
uv run --no-sync pi-baseline validate "$RUN_ROOT/config/pi-config.json"

uv run --no-sync pi-baseline experiment create \
  "$RUN_ROOT/$EXPERIMENT_ID" \
  --spec "$RUN_ROOT/config/authoring-spec.json" \
  --workers 10
```

To run the checked-in 10-case demo as written:

```sh
export CORPUS_ROOT="$HOME/spatial-real-pilot"
export RUN_ROOT="$CORPUS_ROOT/benchmark-runs/spatial-10-case-demo-20260728"
export EXPERIMENT_ID="spatial-10-case-demo-20260728"
export DEMO_CONFIG="$PWD/docs/development/spatial_benchmark_10_case_demo"

uv run --no-sync pi-baseline validate "$DEMO_CONFIG/pi-config.json"
uv run --no-sync pi-baseline experiment create \
  "$RUN_ROOT/$EXPERIMENT_ID" \
  --spec "$DEMO_CONFIG/authoring-spec.json" \
  --workers 5
```

Run with an OpenAI API key stored as `OPENAI_API_KEY` in `.env`:

```sh
uv run --no-sync pi-baseline experiment run \
  "$RUN_ROOT/$EXPERIMENT_ID" \
  --private-root "$RUN_ROOT/private" \
  --corpus-root "$CORPUS_ROOT" \
  --oracle-root "$CORPUS_ROOT/oracle" \
  --auth-file .env \
  --auth-mode openai-api-key \
  --ledger-path "$RUN_ROOT/private/score-ledger.jsonl" \
  --public-root "$RUN_ROOT/public"
```

Check progress or resume unfinished jobs with the same roots:

```sh
uv run --no-sync pi-baseline experiment status \
  "$RUN_ROOT/$EXPERIMENT_ID" \
  --private-root "$RUN_ROOT/private"

uv run --no-sync pi-baseline experiment resume \
  "$RUN_ROOT/$EXPERIMENT_ID" \
  --private-root "$RUN_ROOT/private" \
  --corpus-root "$CORPUS_ROOT" \
  --oracle-root "$CORPUS_ROOT/oracle" \
  --auth-file .env \
  --auth-mode openai-api-key \
  --ledger-path "$RUN_ROOT/private/score-ledger.jsonl" \
  --public-root "$RUN_ROOT/public"
```

Operational success means a job finished; it does not mean the answer was
correct. Summarize scored QA outcomes separately:

```sh
find "$RUN_ROOT/private/$EXPERIMENT_ID" -name score.v1.json -exec \
  jq -r .outcome {} \; | sort | uniq -c
```

## Review an agent session

Current runs retain a native Pi session inside each terminal private mode
directory. Find admissible session directories with:

```sh
find "$RUN_ROOT/private/$EXPERIMENT_ID" \
  -name native-session-receipt.v1.json -printf '%h\n'
```

Open one result:

```sh
uv run --no-sync pi-baseline session view \
  "$RUN_ROOT/private/$EXPERIMENT_ID/<job-id>/attempt-1/private/<run-id>/<condition>"
```

The command prints a token-scoped loopback URL and opens a read-only browser
view when possible. The viewer shows the native conversation, reasoning,
branches, tool calls, tool results, timestamps, and token usage. It does not
show private scores or permit session continuation.

Press `Ctrl-C` to stop the server and remove its temporary review copy. The
canonical retained session remains unchanged.

For the viewer's privacy and compatibility contract, see
[Pi session history viewer](/docs/development/pi_session_viewer.md).
