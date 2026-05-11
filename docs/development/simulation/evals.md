# Evals

> **Status: In Progress** — Eval creation workflow and additional rubric types are under active development.

Evals test navigation accuracy by sending the agent to target locations and scoring distance.

## Running Evals

### Sequential

```bash
pytest dimos/e2e_tests/test_dimsim_eval.py -v -s -m slow
```

### Parallel (3 concurrent workflows)

```bash
pytest dimos/e2e_tests/test_dimsim_eval_parallel.py -v -s -m slow
```

### Single eval

```bash
pytest dimos/e2e_tests/test_dimsim_eval.py::TestSimEvalSequential::test_go_to_tv -v -s -m slow
```

### With log capture

```bash
DIMSIM_EVAL_LOG_DIR=./logs pytest dimos/e2e_tests/test_dimsim_eval.py -v -s -m slow
```

## Creating Evals

Evals are JSON workflow files that define a task, starting pose, timeout, and success criteria. They live in `~/.dimsim/evals/<environment>/<workflow>.json`.

### Interactive Wizard

```bash
dimsim eval create
```

Walks you through picking a scene, rubric type, target object, task prompt, threshold, and timeout. Writes the workflow JSON and prints the command to run it.

### Manual Creation

Create a JSON file in `~/.dimsim/evals/<env>/<name>.json`:

```json
{
  "name": "go-to-tv",
  "environment": "apt",
  "task": "Go to the TV",
  "startPose": { "x": 0, "y": 0.5, "z": 3, "yaw": 0 },
  "timeoutSec": 30,
  "successCriteria": {
    "objectDistance": {
      "object": "agent",
      "target": "television",
      "thresholdM": 2.0
    }
  }
}
```

**Fields:**

| Field | Required | Description |
|-------|----------|-------------|
| `name` | Yes | Unique identifier (used in CLI and test selection) |
| `environment` | Yes | Scene name (must be installed via `dimsim scene install`) |
| `task` | Yes | Natural language prompt sent to the agent |
| `startPose` | Yes | Agent spawn position: `x`, `y`, `z`, `yaw` (radians) |
| `timeoutSec` | Yes | Max seconds before the eval is scored |
| `successCriteria` | Yes | One or more rubrics (see below) |

## Rubric Types

### `objectDistance`

Agent must reach a target object within a distance threshold.

```json
"successCriteria": {
  "objectDistance": {
    "object": "agent",
    "target": "refrigerator",
    "thresholdM": 3.0
  }
}
```

- `target` is matched by substring against scene object titles/IDs (case-insensitive)
- `thresholdM` is Euclidean distance in meters from agent to the target's bounding box surface
- Use `dimsim list objects --scene <name>` to see available target objects

### `radiusContains`

Agent must be within a radius of the centroid computed from multiple target objects.

```json
"successCriteria": {
  "radiusContains": {
    "targets": ["couch", "coffee_table"],
    "radiusM": 3.0
  }
}
```

## Registering in the Manifest

To include your eval in automated test runs, add it to `~/.dimsim/evals/manifest.json`:

```json
{
  "version": "1.0",
  "environments": [
    {
      "name": "apt",
      "scene": "apt",
      "workflows": ["go-to-tv", "go-to-couch", "your-new-eval"]
    }
  ]
}
```

## Planned

- Additional rubric types (trajectory metrics, spatial relations, temporal ordering)
- Eval dashboard / results aggregation
- CI integration for automated eval runs on PR
