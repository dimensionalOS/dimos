# Targeted second-pass verification

Scope: DimOS repository only; CPU offline training, isolated local services and a
new disposable PostgreSQL container. No source recordings modified, robot actions,
shared-service restarts, platform database queries, CI, push or deployment.

## Implemented

- Training calls installed/pinned LeRobot 0.6.0's actual
  `lerobot.datasets.factory.make_train_eval_datasets` (factory.py lines 139–214
  inspected). Its rule is last **ceil(N * split) per task**, not global floor.
  Queue-time effective indices remain null until native metadata resolves them.
  Successful results persist indices and per-task membership; empty native training
  splits propagate `empty_split`/422 and become blocked worker outcomes.
- Packages are inference-only, not resumable. The insufficient optimizer/torch-RNG
  file was removed. Actual saved config, feature shapes/mappings/component order,
  units (unknown when capture omits them), fps, installed versions, camera coverage
  and episode lineage are included. Policy plus processors reload without actions.
- Optional `PostgresStore` reuses Store SQL/interface with dedicated schema,
  transaction-scoped advisory serialization, deterministic insertion ordering,
  versioned explicit migrations and immutable-revision triggers. Startup never
  migrates. SQLite remains the isolated fallback. Configuration in README.md.

## Commands and results

From repository root:

```sh
.venv/bin/python -m pytest -o addopts='' --confcutdir=dimos/imitation/learning dimos/imitation/learning -q --junitxml=.learning-evidence/pass2/sqlite.xml
# 48 passed, 1 skipped (explicit PostgreSQL DSN absent)
DIMOS_LEARNING_TEST_POSTGRES_DSN=postgresql://postgres:isolated-test-only@127.0.0.1:32769/learning .venv/bin/python -m pytest -o addopts='' --confcutdir=dimos/imitation/learning dimos/imitation/learning -q --junitxml=.learning-evidence/pass2/postgres.xml
# 49 passed; real PostgreSQL 17, including concurrent CAS/claim and migration tests
.venv/bin/python demo_learning_reference.py
# 2 real episodes / 72 frames; 1 ACT step; package download and load-check succeeded
.venv/bin/python demo_learning_acceptance.py
# 3 actual decoded cameras, 1219 frames each; command dataset correctly blocked
```

From `native/python/lerobot`:

```sh
PYTHONPATH=../../.. CUDA_VISIBLE_DEVICES='' HF_HUB_OFFLINE=1 DIMOS_LEARNING_TEST_DATASET=/home/cc/work/dimos-learning-implementation/dimos/.learning-evidence/reference-f3a3dd8c/work/309b417e-01c8-49ee-b70b-f31e75607470/1/dataset uv run --frozen --group learning --group tests python -m pytest -o addopts='' dimos_lerobot/test_learning_train.py dimos_lerobot/test_learning_readiness.py -q --basetemp=../../../.learning-evidence/pass2/native --junitxml=../../../.learning-evidence/pass2/native.xml
# 12 passed; real CPU ACT + eval + policy/processor reload at splits 0, 0.1, 0.2
```

Ruff on changed Python files and `git diff --check` passed. Backend tests emit
preexisting pytest plugin/config and Starlette HTTPX deprecation warnings.

## Retained local evidence (ignored, not committed)

- `.learning-evidence/reference-f3a3dd8c/acceptance.json`
- `.learning-evidence/three-camera-06b812f4/acceptance.json`
- `.learning-evidence/pass2/{sqlite,postgres,native}.xml`
- `.learning-evidence/pass2/native/test_real_reference_training_0{0,1,2}/policy/training.json`

Reference task `pick up the cup and place it at the goal` has train [0,1]/eval []
when disabled, train [0]/eval [1] at either optional ratio (ceil on two episodes).
The real reference feature order is camera, eight-component state, eight-component
command action; numeric component order is joint1..joint7, arm/gripper. Capture
schema does not declare physical units, so package units are explicitly unknown.
The multicamera source remains blocked for missing gripper commands; this is NOT
three-camera training or rollout acceptance. Multi-task ordering is covered with
synthetic metadata against the actual upstream split factory.

## Remaining boundaries / cleanup

No resume support, physical-unit validation, robot rollout, production PostgreSQL
or real S3/Cognito acceptance is claimed. Prior evidence/packages are untouched.
Parent still owns composed integrated runtime verification.

A new `--rm` PostgreSQL test container was created using:
`docker run --detach --rm --name dimos-learning-pass2-postgres -e POSTGRES_PASSWORD=isolated-test-only -e POSTGRES_DB=learning -p 127.0.0.1::5432 postgres:17-alpine`.
The execution approval layer blocked `docker stop`, so the container is still
running on loopback port 32769. Parent/operator must clean it up with
`docker stop dimos-learning-pass2-postgres`; no shared container was touched.
