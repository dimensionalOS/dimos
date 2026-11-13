# Backend handoff

## Status
Implemented and locally exercised: durable Learning HTTP API, request-bound JWT delegation, request-scoped source authorization, real three-camera MCAP capture and media preparation, annotation CAS/immutable revisions/source checksums, durable jobs/worker leases/idempotent reports/cancellation, frozen dataset conversion, SHA256 sealed immutable output + S3-compatible publication/readback/restore, native LeRobot 0.6 ACT CPU train/package/load-check, and tests. API/worker import no robot graph or GUI.

## Runnable launch interface
Use an isolated interpreter (already present; do not edit `/home/cc/codes/dimos/.venv`):

```bash
cd /home/cc/work/dimos-learning-implementation/dimos
uv venv --python /home/cc/codes/dimos/.venv/bin/python .venv
uv pip install --python .venv/bin/python -r dimos/imitation/learning/requirements.txt
# tests additionally: uv pip install --python .venv/bin/python -r dimos/imitation/learning/requirements-tests.txt
export DIMOS_LEARNING_STATE_DIR=/home/cc/work/dimos-learning-implementation/runtime/learning
export DIMOS_LEARNING_DELEGATION_SECRET="$(openssl rand -hex 32)" # parent-supplied >=32-byte shared secret
export DIMOS_LEARNING_SOURCE_ROOTS=/home/cc/work/dimos-bimanual-annotation-artifacts/session_v1 # local fixture opt-in only
export DIMOS_LEARNING_S3_ENDPOINT=http://127.0.0.1:8770 # parent emulator
export DIMOS_LEARNING_S3_BUCKET=learning
.venv/bin/python -m dimos.imitation.learning.store
.venv/bin/python -m uvicorn dimos.imitation.learning.app:app --host 127.0.0.1 --port 8768
```

Worker:

```bash
# one-time pairing code from POST /v1/workers/pairing, then:
.venv/bin/python -m dimos.imitation.learning.worker --api http://127.0.0.1:8768 --pair CODE --name local-cpu --credentials /isolated/path/worker.json --state "$DIMOS_LEARNING_STATE_DIR"
# later runs:
.venv/bin/python -m dimos.imitation.learning.worker --api http://127.0.0.1:8768 --credentials /isolated/path/worker.json --state "$DIMOS_LEARNING_STATE_DIR"
```

Native adapter uses `native/python/lerobot/.venv/bin/python` and `uv sync --frozen --group learning --group tests`. Its `learning` group is locked in `native/python/lerobot/uv.lock`.

## Verified evidence
- Unit/integration: `41 passed`; log `.learning-evidence/unit-tests.log`.
- Native adapter: `3 passed`; log `.learning-evidence/native-tests.log`.
- Three-camera real source: `demo_learning_acceptance.py` PASSED. Latest run evidence `.learning-evidence/three-camera-5aac0c67/acceptance.json`, API log same directory; each head/left_wrist/right_wrist has 1219 source frames, MP4 SHA256 in evidence, Range reads returned 206, annotation revision save/reopen succeeded, dataset job is `blocked` with `gripper_command_missing`. Final run command log: `.learning-evidence/three-camera-latest.log`.
- Positive single-arm references: `demo_learning_reference.py` PASSED. Latest evidence `.learning-evidence/reference-9950838b/acceptance.json`; immutable LeRobot dataset has 2 episodes/72 frames and full meta/data/videos/manifest/lineage; every object published and read back via moto HTTP S3-compatible emulator; native ACT one-step loss `112.16423797607422`; policy package SHA256 `f98ea685e391182e7c868ce421d6e9e5ff5f52722dfdb8fc7d0a9920c2fe1eec`; download read-back matched; no-action load check succeeded. Command log `.learning-evidence/reference-latest.log`.
- Source hashes after all runs:
  - session_v1 recording `69d4249c4cb5d52b3d9c4764e38e140ced39dfac33f7ce3ac8c099b6ffb84993`, schema `e5801a058d3d64dc9880725ec0ef2d5566bf546a147666a76ad584d33e54622e`, manifest `821835ab447dd8b51327a89615c3d1497c990a6fc4e21b50c52ab9dc623e3bcf`.
  - session_19ep_v1 recording `0af57c0ae5eaa66c249f8e86b0d48a34c44b678d9b44bd5c6cbe2e3bce4d2dc5`, schema `83cc59cf970a7cc624faaf8b58197f557a28b626bcf3e55378d095d58bfb7487`.
  - session_fix_v1 recording `12762732b5fd08c6a3f5bed62e26f13add97926b11bdf41e3792f6ee3ec54849`, schema `83cc59cf970a7cc624faaf8b58197f557a28b626bcf3e55378d095d58bfb7487`.
- `.venv/bin/python demo_verify_imports.py` PASSED: API imports 808 modules, no `dimos.robot`, `dimos.core.coordination`, `dimos.imitation.policy`, or GUI modules.

## Known boundaries / incomplete
- Positive references are single-camera/single-arm and are NOT three-camera command-action training acceptance. The provided three-camera source remains correctly blocked because no gripper command was recorded.
- PostgreSQL production store not implemented: local acceptance uses explicit isolated SQLite WAL migrations. Frozen contract does not prescribe engine, but this is a deliberate deployment gap.
- Parent-owned S3 emulator on port 8770 and Cognito integration are not verified here; moto HTTP emulator and request-bound local JWT fixtures were used.
- GPU training, live cloud S3, real three-camera training, production deployment, robot actions, CI queries, Kanban changes were not performed.
- Root `CURRENT-PLAN.md` was absent; contract v1 and the current composite `baseHtml` plus all overlays in `/home/cc/work/dimos-learning-service-spec/execution/dimos-learning-implementation.json` were used.
