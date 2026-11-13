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
cd native/python/lerobot && uv sync --frozen --group learning --group tests
