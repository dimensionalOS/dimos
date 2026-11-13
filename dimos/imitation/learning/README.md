"""Learning backend local commands and acceptance boundaries."""
# Local acceptance only (explicit fixture auth is not an implicit auth bypass):
uv venv --python /home/cc/codes/dimos/.venv/bin/python .venv
uv pip install --python .venv/bin/python -r dimos/imitation/learning/requirements.txt
uv pip install --python .venv/bin/python -r dimos/imitation/learning/requirements-tests.txt
export DIMOS_LEARNING_STATE_DIR=/home/cc/work/dimos-learning-implementation/runtime/learning
export DIMOS_LEARNING_DELEGATION_SECRET="$(openssl rand -hex 32)"
export DIMOS_LEARNING_SOURCE_ROOTS=/home/cc/work/dimos-bimanual-annotation-artifacts/session_v1
export DIMOS_LEARNING_S3_ENDPOINT=http://127.0.0.1:8770
export DIMOS_LEARNING_S3_BUCKET=learning
.venv/bin/python -m dimos.imitation.learning.store
.venv/bin/python -m uvicorn dimos.imitation.learning.app:app --host 127.0.0.1 --port 8768

# Worker pairing is returned by POST /v1/workers/pairing:
.venv/bin/python -m dimos.imitation.learning.worker --api http://127.0.0.1:8768 --pair CODE --name local-cpu --credentials /isolated/path/worker.json --state "$DIMOS_LEARNING_STATE_DIR"

# Tests:
.venv/bin/python -m pytest -o addopts='' --confcutdir=dimos/imitation/learning dimos/imitation/learning -q
.venv/bin/python demo_learning_acceptance.py
.venv/bin/python demo_learning_reference.py
# Native training/readiness

Evaluation defaults to off. Run submission stores unresolved (`null`) indices;
only training resolves the immutable dataset's task metadata. The pinned LeRobot
0.6.0 `lerobot.datasets.factory.make_train_eval_datasets` is called directly:
last **ceil(N * split) per task**, preserving task encounter/episode order.
10% and 20% are optional, including for a single recording. Native empty training
splits fail closed; no export split or two-recording gate is imposed. Successful
runs/packages contain actual train/eval indices and `split_per_task`.

Packages are explicitly **inference-only** (`resumable_ready=false`): no resume
entrypoint or loader/sampler continuation state is implemented. The previous
optimizer/RNG-only file was insufficient and is no longer emitted. `training.json`
contains actual saved policy config, feature shape/component ordering, source-to-
canonical mapping, fps, installed versions, episode lineage and camera coverage
verification. Units are copied when declared; missing capture units remain null
with an explicit unknown status (never assume radians or gripper units). The
real reference schema has no declared units. Manifest lineage also binds dataset
manifest checksum and run identity. CPU reload is not rollout acceptance.

# Dedicated PostgreSQL store (explicit opt-in)

SQLite is the isolated local acceptance fallback. For composed PostgreSQL use a
separate database/role with access ONLY to the dedicated Learning schema, never
platform org/user tables. Install `requirements-postgres.txt`, then explicitly set:

```sh
export DIMOS_LEARNING_POSTGRES_DSN='postgresql://ROLE:SECRET@HOST/LEARNING_DB'
export DIMOS_LEARNING_POSTGRES_SCHEMA=dimos_learning
export DIMOS_LEARNING_STATE_DIR=/isolated/learning-workspace
.venv/bin/python -m dimos.imitation.learning.store  # operator migration step
```

No constructor or app startup migrates. The schema is mandatory when DSN is set;
only `dimos_learning` or `dimos_learning_*` identifiers are allowed. All SQL uses
that schema plus pg_catalog, with no platform/public fallback. Use a migration
role to create/own the schema; grant the runtime role only USAGE on schema,
SELECT/INSERT/UPDATE/DELETE on documents and USAGE/SELECT on its sequence.
Runtime does not need schema CREATE or migrations-table writes. Do not log DSNs.
A schema-scoped PostgreSQL transaction advisory lock serializes writers just as
SQLite BEGIN IMMEDIATE does, preserving multi-record CAS/claims deterministically;
this conservative implementation is not designed for high-throughput queues.
Migrations are versioned and immutable revisions have database triggers.

Test against an explicitly disposable PostgreSQL instance (never production):

```sh
DIMOS_LEARNING_TEST_POSTGRES_DSN='postgresql://.../disposable_db' \
  .venv/bin/python -m pytest -o addopts='' --confcutdir=dimos/imitation/learning dimos/imitation/learning -q
```

Tests create/drop unique `dimos_learning_test_*` schemas; the role needs permission
to do so. Without that test DSN the real PostgreSQL integration test skips.

# Original native environment installation
cd native/python/lerobot && uv sync --frozen --group learning --group tests
