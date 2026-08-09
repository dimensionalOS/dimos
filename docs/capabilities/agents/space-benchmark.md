---
title: "SPACE benchmark evaluation"
---

`dimos eval space` scores a text task from Apple's SPACE benchmark through the
DimOS agent-evaluation path. SPACE stays in charge: its own `evaluate_qas` loop
draws the prompt, parses the reply and keeps the answer key, and the only call
replaced is the one that would reach a chat completion — that runs one
agent-evaluation case instead. SPACE is not vendored and not a wheel. It is cloned once
at the pinned revision `564e43932adc84543800dd56b99cee37efaeabd8` and imported
unmodified.

## Setup

```bash
uv sync --extra space
npm ci --prefix packages/pi-code-policy-extension
npm run build --prefix packages/pi-code-policy-extension
export OPENAI_API_KEY=...
```

The `space` extra includes the `agents` extra, and the extension built above is
the same one [Frozen recording evaluation](/docs/capabilities/agents/evaluation.md)
needs, along with the Pi and Node versions it pins.

## Run a task

```bash
uv run dimos eval space \
  --task DirectionEstimationBEVText \
  --groups 8 \
  --seed 20260808 \
  --workers 2
```

The first run fetches two things into `~/.cache/dimos/space-benchmark/`, outside
the repository, and reuses them afterwards:

- the SPACE checkout, at the pinned revision;
- Apple's 3.6 GB data release, hashed as it downloads and refused if the digest
  is not the pinned one.

On every later run the cached release is checked against the provenance record
written when it was unpacked; a cache whose record is missing, unreadable, or
names a different digest is refused rather than silently used.

The text tasks that can be named by `--task` are registered in
[`space_qa/tasks.py`](/dimos/benchmark/space_qa/tasks.py); `--groups` cannot
exceed the stimulus count recorded there for the task.

`--output` defaults to a timestamped directory under the same cache, so a run
that is not told otherwise writes outside the repository; with `--output` it
writes wherever that points. Whichever directory it names must be free of an
earlier run: the command refuses one that already holds a `manifest.json` or a
`space/` directory rather than merging into it.

A question whose agent never starts — an unbuilt extension, a missing key — is
recorded with an `infra_error` and reaches SPACE as an empty reply, which SPACE
scores as a miss. So a `0.0%` whose questions all carry one, counted on the
summary's `Infra fails` line, is a failed install rather than a result.

## What a run leaves behind

```text
manifest.json                           what the run selected, written before it starts
subset/qas.json                         the drawn rows, as SPACE itself reads them
cases/qa_00000/case.json                one evaluation case per question
cases.jsonl                             the per-question records, in subset order
space/dimos_qa/<timestamp>/
    results.json                        SPACE's own score
    qa_00000/transcript.md              SPACE's dialog log for that question
    qa_00000/dimos_case.json            this side's record of the same question
    qa_00000/dimos/result.json          the evaluation case result
    qa_00000/dimos/pi-transcript.jsonl  only when Pi wrote one
    qa_00000/dimos/stderr.log           only when stderr was not empty
```

`results.json` is the score: `mean_metrics.accuracy` is the number the run
reports, and nothing on this side recomputes it. `cases.jsonl` is the ledger that
gets a reader from that number back to a question, a transcript and a failure
reason. A run only succeeds if the two agree question by question — a difference
in how many questions were scored, or in any single prediction, fails it.

`manifest.json` records what the score is a score of:

| Field | Meaning |
| --- | --- |
| `benchmark`, `task` | The adapter, and the SPACE task the rows were drawn from. |
| `seed`, `groups` | The sampling arguments, enough to redraw the same subset. |
| `space_revision` | The pinned SPACE commit that scored the run. |
| `data_sha256` | Digest of the archive the cached release was unpacked from, read from the provenance record written beside it. `null` means no such record sits beside the release, so nothing on this side has checked it. |
| `dimos_revision` | The commit the run executed on, or `null` outside a git checkout. |
| `rows` | One entry per question: its index in the subset, its row ordinal upstream, and the SHA-256 of the question text. |

Those digests are checked again when the records are read back, so a question
answered as something other than what the manifest selected fails the run
instead of being scored.

## Reusing local copies

| Variable | Effect |
| --- | --- |
| `DIMOS_SPACE_SOURCE` | Score against an existing SPACE checkout instead of the cached clone. Nothing is fetched, but the revision is still checked: a checkout at another commit is refused, because the pin rather than the path is what makes a score comparable. |
| `DIMOS_SPACE_DATA` | Read an already-extracted release root. Nothing is downloaded and nothing is refused; `data_sha256` reports whatever provenance record the root carries, and is `null` when it carries none — the digest follows the record, not the path the release came in by. This is also the explicit way to keep using a cache the check above refused. |

## Sampling

SPACE asks each stimulus four times with the correct answer in a different slot.
A subset that split a group would grade that stimulus on only some of its
variants and inherit the placement bias, so groups are drawn whole: `--groups 8`
is 8 stimuli and 32 questions. `--seed` fixes which groups are drawn, so the same
task, seed and group count select the same upstream rows anywhere.

## What the score compares to

SPACE's published baselines answer each question in a single chat completion with
no tools, while a question here is answered by a multi-turn agent holding a
`python_exec` kernel. The grader is SPACE's own either way, so runs of this
command are comparable with each other — but not on the same axis as the numbers
in the paper.

## Licensing

SPACE's code is under the Apple Sample Code License and its data release under
CC BY-NC-ND 4.0. Two consequences are visible in this integration:

- Nothing from the release is redistributed here. No question, answer or
  transcript is committed, and every fixture beside the integration is generated.
  A run directory holds the drawn questions verbatim: by default it lands in the
  cache outside the repository, and `--output` writes wherever it points, which
  makes that argument a licensing decision rather than a path preference.
- The release carries a NonCommercial term. What that permits inside this
  organisation is not settled here; check before using a run's output for
  anything beyond research.

## Platform notes

The worker pool is forked rather than spawned: SPACE's agent and config
registries live only in this process's memory, and a spawned worker would come up
with neither. `fork` is already the default on Linux. On macOS the command sets
`OBJC_DISABLE_INITIALIZE_FORK_SAFETY` for the Pi processes the workers exec; if a
fork itself aborts inside Objective-C before that takes effect, export the flag in
the shell.

Each question is one agent-evaluation case, so the output contract and
**unsandboxed** trust boundary described in
[Frozen recording evaluation](/docs/capabilities/agents/evaluation.md) apply to
every question in a run.
