---
title: "SPACE benchmark evaluation"
---

`dimos eval space` scores a text task from Apple's SPACE benchmark through the
DimOS agent-evaluation path. SPACE stays in charge: its own `evaluate_qas` loop
draws the prompt, parses the reply and keeps the answer key, and the only call
replaced is the one that would reach a chat completion — that runs one
agent-evaluation case instead. SPACE is not vendored and not a wheel; it is
cloned once at the pinned revision `564e43932adc84543800dd56b99cee37efaeabd8`
and imported unmodified.

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
the repository, and reuses them afterwards: the SPACE checkout at the pinned
revision, and Apple's 3.6 GB data release, hashed as it downloads and refused if
the digest is not the pinned one. Every later run re-checks the cached release
against the provenance record written when it was unpacked; a cache whose record
is missing, unreadable, or names a different digest is refused rather than
silently used.

The text tasks `--task` can name are registered in
[`space_qa/tasks.py`](/dimos/benchmark/space_qa/tasks.py); `--groups` cannot
exceed the stimulus count recorded there. `--output` defaults to a timestamped
directory under the same cache and must name an absent or empty directory: a run
is read back by path, so the command refuses a used directory rather than
merging into it.

A missing API key, a missing dependency of the `space` extra, or an unbuilt Pi
extension is refused by preflight before anything is fetched. A question that
fails after its agent starts — a crashed agent process, a turn timeout — is
recorded with an `infra_error` and reaches SPACE as an empty reply, which SPACE
scores as a miss; the summary's `Infra fails` line counts those questions, so
the accuracy is only readable next to it. A failure after the reply was read —
an unwritable transcript, a full disk — is recorded beside the answer the run
had already paid for, and the question is still scored on that answer.

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
reports, and nothing on this side recomputes it. `cases.jsonl` is the ledger
that gets a reader from that number back to a question, a transcript and a
failure reason. A run only succeeds if the two agree question by question.

`manifest.json` records what the score is a score of:

| Field | Meaning |
| --- | --- |
| `benchmark`, `task` | The adapter, and the SPACE task the rows were drawn from. |
| `seed`, `groups` | The sampling arguments, enough to redraw the same subset. |
| `space_revision` | The pinned SPACE commit that scored the run. |
| `data_sha256` | Digest of the archive the release was unpacked from, **as its provenance record claims**. `null` means no readable record sits beside the release. |
| `qas_sha256` | Digest of the task's `qas.json` **this run actually read**, computed over the bytes while reading them. |
| `dimos_revision` | The commit the run executed on, or `null` when it did not run from a dimos checkout. Uncommitted changes to that checkout are invisible here — unlike the SPACE checkout, which a run refuses outright when its tree is dirty. |
| `rows` | One entry per question: its index in the subset, its row ordinal upstream, and the SHA-256 of the question text. |

Each row's `question_sha256` is checked again when the records are read back, so
a question answered as something other than what the manifest selected fails the
run instead of being scored.

## Reusing local copies

| Variable | Effect |
| --- | --- |
| `DIMOS_SPACE_SOURCE` | Score against an existing SPACE checkout instead of the cached clone. It must sit at the pin with a clean working tree: it is the code that runs, not the commit it is named after, that grades the replies. |
| `DIMOS_SPACE_DATA` | Read an already-extracted release root. Nothing is downloaded and nothing is refused; `data_sha256` reports whatever provenance record the root carries, `null` when it carries none. This is also the explicit way to keep using a cache the check above refused. |

## Sampling

SPACE asks each stimulus four times with the correct answer in a different slot.
A subset that split a group would grade that stimulus on only some of its
variants and inherit the placement bias, so groups are drawn whole: `--groups 8`
is 8 stimuli and 32 questions. `--seed` fixes which groups are drawn, so the same
task, seed and group count select the same upstream rows anywhere — which is what
makes two runs comparable, and why the flag has no default.

Give `--groups` a task's own stimulus count to run all of it; the seed then only
orders what was drawn and cannot move the score. Below that, a run reports one
draw. A different seed asks about different stimuli, and how far the score moves
between seeds is not something a single run measures.

## What the score compares to

SPACE's published baselines answer each question in a single chat completion with
no tools, while a question here is answered by a multi-turn agent holding a
`python_exec` kernel. The grader is SPACE's own either way, so runs of this
command are comparable with each other — but not on the same axis as the numbers
in the paper.

## Licensing

SPACE's code is under the Apple Sample Code License and its data release under
CC BY-NC-ND 4.0. Nothing from the release is redistributed here: no question,
answer or transcript is committed, and every fixture beside the integration is
generated. A run directory holds the drawn questions verbatim, so where
`--output` points is a licensing decision rather than a path preference. The
release also carries a NonCommercial term; check before using a run's output for
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
