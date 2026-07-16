#!/usr/bin/env bash
# Reproduce the reference spatiotemporal video-relation evaluation from a checkout.
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: reproduce_reference.sh [--skip-setup] [--skip-tests]

  --skip-setup  Do not run LFS materialization, uv sync, model extraction, or CLIP setup.
  --skip-tests  Do not run pytest, Ruff, or mypy before the real demo.
EOF
}

skip_setup=false
skip_tests=false
while (($#)); do
  case "$1" in
    --skip-setup) skip_setup=true ;;
    --skip-tests) skip_tests=true ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
  shift
done

for command in git uv; do
  command -v "$command" >/dev/null || {
    echo "Missing required command: $command" >&2
    exit 1
  }
done

repo_root="$(git rev-parse --show-toplevel)"
cd "$repo_root"

if ! $skip_setup; then
  command -v git-lfs >/dev/null || {
    echo "Missing required command: git-lfs" >&2
    exit 1
  }

  git lfs install --local
  git lfs pull --include='assets/simple_demo.mp4,data/.lfs/models_yoloe.tar.gz'

  python3 - <<'PY'
from pathlib import Path
for name in ("assets/simple_demo.mp4", "data/.lfs/models_yoloe.tar.gz"):
    size = Path(name).stat().st_size
    print(f"{name}: {size:,} bytes")
    assert size > 1_000_000, f"{name} is still an LFS pointer"
PY

  uv sync --group lint
  uv run python -c "from dimos.utils.data import get_data; print(get_data('models_yoloe'))"

  if ! uv run python -c 'import clip' >/dev/null 2>&1; then
    uv pip install 'git+https://github.com/ultralytics/CLIP.git'
  fi
fi

if ! $skip_tests; then
  uv run pytest dimos/benchmark/spatiotemporal -q
  uv run ruff format --check dimos/benchmark/spatiotemporal
  uv run ruff check dimos/benchmark/spatiotemporal
  uv run --group lint mypy dimos/benchmark/spatiotemporal
fi

uv run python -m dimos.benchmark.spatiotemporal.demo \
  --source-video assets/simple_demo.mp4 \
  --output-root .artifacts/spatiotemporal-video-qa \
  --duration-s 25 \
  --frame-stride 150

uv run python - <<'PY'
import hashlib
import json
from pathlib import Path

root = Path(".artifacts/spatiotemporal-video-qa")
summary_path = root / "summary.json"
summary = json.loads(summary_path.read_text(encoding="utf-8"))
teacher = summary["teacher"]
candidate = summary["candidate"]
review = summary["review"]

assert summary["video"]["duration_s"] == 25
assert summary["video"]["frames"] == 750
assert teacher["detector_repeat_equal"] is True
assert teacher["relation_facts"] > 0
assert teacher["relation_intervals"] > 0
assert teacher["questions"] == teacher["answers"]
assert candidate["candidate_used_oracle"] is False
assert candidate["baseline_kind"] == "public_only_plumbing_smoke_test"
assert candidate["visually_grounded"] is False
assert candidate["questions_answered"] == teacher["questions"]
assert candidate["status_counts"]["missing"] == 0
assert candidate["status_counts"]["invalid"] == 0
assert review["question_count"] == teacher["questions"]
assert (root / review["index_path"]).is_file()

questions = [
    json.loads(line)
    for line in (root / "bundle-a/public/questions.jsonl")
    .read_text(encoding="utf-8")
    .splitlines()
]
answers = [
    json.loads(line)
    for line in (root / "bundle-a/oracle/answers.jsonl")
    .read_text(encoding="utf-8")
    .splitlines()
]
assert len(questions) == len(answers) == teacher["questions"]
for question in questions:
    assert "expected" not in question
    assert "evidence_frame_ids" not in question
    assert "evidence_interval_ids" not in question

answers_by_id = {answer["question_id"]: answer for answer in answers}
for family in ("spatial", "temporal"):
    expected = [
        answers_by_id[question["question_id"]]["expected"]
        for question in questions
        if question["question_kind"] == family
    ]
    assert expected.count(True) == expected.count(False)

print("GENERATED_EXAMPLES")
for family, expected in (("spatial", True), ("spatial", False), ("temporal", True)):
    question = next(
        question
        for question in questions
        if question["question_kind"] == family
        and answers_by_id[question["question_id"]]["expected"] is expected
    )
    answer = answers_by_id[question["question_id"]]
    print(f"  [{family} expected={str(expected).lower()}] {question['text']}")
    print(
        "    evidence_frames="
        f"{answer['evidence_frame_ids']} evidence_intervals="
        f"{answer['evidence_interval_ids']}"
    )

digest = hashlib.sha256(summary_path.read_bytes()).hexdigest()
print(f"SUMMARY_GATES=PASS sha256={digest}")
PY

cat <<'EOF'

Reproduction complete.

Summary:
  .artifacts/spatiotemporal-video-qa/summary.json

Evidence viewer:
  .artifacts/spatiotemporal-video-qa/evidence-viewer/index.html

On macOS, open it with:
  open .artifacts/spatiotemporal-video-qa/evidence-viewer/index.html
EOF
