# Autoresearch

Ask the DimOS agent to improve an artifact against a measurable objective. Its bundled [autoresearch skill](../agents/harness/skills/autoresearch/SKILL.md) guides it through measuring a baseline, proposing changes, and choosing a candidate from development results. A session freezes that choice before scoring it on held-out cases.

> Autoresearch the alert threshold in this workspace. Use our labeled deliveries to minimize error cost, counting a missed late delivery three times as heavily as a false alarm. Compare against the current threshold and report held-out results.

Send a request like this through `dimos agent run "…"` or an open agent conversation. The generic workflow runs locally and requires a candidate artifact and a repeatable way to measure it.

## A local experiment

This example searches for a threshold for detecting late deliveries. Each candidate is a JSON file. The evaluator measures the cost of one misclassification per trial. A missing late delivery costs three points and a false alarm costs one.

```bash
cat > evaluate.py <<'PY'
import json
from pathlib import Path
import sys

trial = json.load(sys.stdin)
threshold = json.loads(Path(trial["artifact"]).read_text())["threshold"]
cases = {
    "dev": [(3, False), (7, True), (10, True)],
    "heldout": [(4, False), (8, True)],
}
delay, late = cases[trial["split"]][trial["index"]]
prediction = delay >= threshold
cost = (3 if late else 1) if prediction != late else 0
print(json.dumps({"score": cost, "evidence": {"delay": delay, "predicted_late": prediction}}))
PY
dimos autoresearch init research --task "Detect late deliveries" --metric "error cost" \
  --minimize --candidates 2 --dev-trials 3 --heldout-trials 2 --evaluator evaluate.py
echo '{"threshold": 9}' > policy.json
dimos autoresearch submit research policy.json --hypothesis "Baseline threshold of nine"
dimos autoresearch run research 1
echo '{"threshold": 6}' > policy.json
dimos autoresearch submit research policy.json --hypothesis "Catch shorter late deliveries"
dimos autoresearch run research 2
dimos autoresearch status research
dimos autoresearch freeze research 2
```

Use `--evaluator evaluate.py` to run a script with the DimOS Python interpreter. For commands with arguments, quote the whole command, for example `--evaluator "python3 evaluate.py --dataset 'delivery cases.json'"`.

The baseline averages 1 point on development. Candidate 2 averages 0 on development and 0 on held-out cases. `dimos autoresearch results research 2` prints its immutable artifact path and evidence. To publish it as a skill, put directions for using the policy in a Markdown file and pass that file to `dimos autoresearch export research --instructions USAGE.md`.

## Where new experiments fit

- **The agent** owns the hypothesis, interpretation, next proposal and choice of winner. The same conversation can inspect the artifact and report the evidence.
- **Session** owns immutable candidate files, the budget, trial indexes, checkpoints, development ranking and final held-out mean. One evaluation can write at a time, and a retried command skips completed trials. Each attempt has its own evidence directory.
- **The evaluator** owns the candidate format, the meaning of the split and index, execution and scoring. The Python contract is `Callable[[Trial], Evaluation]`. A `Trial` carries the task, artifact path, split, index and output directory. An `Evaluation` carries a numeric score, a note and structured evidence. An evaluator error leaves the trial pending. Represent a measured poor result as a score.
- **CommandEvaluator** runs the configured argv in the original working directory, exchanges these objects as JSON on stdin/stdout, and saves the response and stderr with the trial. The project supplies the evaluator, datasets and dependencies used throughout the experiment. An existing benchmark or a scorer from `dimos.evals.scorers` can provide the measurement.
- **The [MuJoCo arm integration](mujoco/README.md)** implements the same contract in-process. It resets the scene, records an episode, executes the arm program, and turns its judge's verdict into a binary score. Metal supplies the model and scene. Its blueprint, MCP controls and specialized runnable skill export remain available.
