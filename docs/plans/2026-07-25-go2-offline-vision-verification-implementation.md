# Go2 Offline Vision Verification Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Build a motion-free offline pipeline that loads allowlisted Go2 candidate frames from persisted `VisualMemory`, sends only those frames to OpenAI Responses API when explicitly requested, and returns a fail-closed structured target verdict.

**Architecture:** Add provider-neutral evidence and verdict models, a persisted-frame bundle loader, and an OpenAI Responses API verifier. Expose them through an offline CLI whose default mode performs a local dry run; cloud upload requires an explicit `--send` flag. Do not connect this slice to MissionRunner or any movement tool.

**Tech Stack:** Python 3.12, Pydantic 2, OpenAI Python SDK 2.x Responses API, OpenCV, pytest, Ruff.

---

### Task 1: Define the evidence and verdict contracts

**Files:**
- Create: `dimos/perception/target_verification.py`
- Create: `dimos/perception/test_target_verification.py`

**Step 1: Write failing contract tests**

Test that:

- a bundle rejects zero views, more than three views, and repeated frame IDs;
- normalized bounding boxes reject coordinates outside zero to one;
- a cloud `yes` is downgraded to `uncertain` unless two known frames contain
  valid `yes` boxes;
- unknown frame IDs fail closed.

**Step 2: Run the focused tests and verify failure**

Run:

```bash
.venv/bin/python -m pytest dimos/perception/test_target_verification.py -q
```

Expected: import failure because `target_verification.py` does not exist.

**Step 3: Implement the Pydantic models**

Implement:

- `CameraPose`
- `CandidateView`
- `CandidateEvidenceBundle`
- `VerificationView`
- `TargetVerification`
- `apply_verification_policy(bundle, result)`

The policy must never turn `no` or `uncertain` into `yes`.

**Step 4: Run the focused tests**

Expected: all contract tests pass.

### Task 2: Load only selected persisted frames

**Files:**
- Modify: `dimos/perception/target_verification.py`
- Modify: `dimos/perception/test_target_verification.py`

**Step 1: Write failing loader tests**

Create a temporary `VisualMemory` with four synthetic images. Verify that the
loader:

- loads only the requested one-to-three frame IDs;
- rejects missing frame IDs;
- resizes the longest edge to at most 1024 pixels;
- encodes JPEG rather than exposing a file path;
- does not include any non-selected stored image.

**Step 2: Run the loader tests and verify failure**

Run the focused pytest file and confirm the loader is missing.

**Step 3: Implement `PersistedCandidateLoader`**

Use `VisualMemory.load()` and OpenCV. Keep frame order stable, reject duplicate
IDs, and cap JPEG quality and dimensions.

**Step 4: Run the focused tests**

Expected: contract and loader tests pass.

### Task 3: Add the OpenAI Responses verifier

**Files:**
- Modify: `dimos/perception/target_verification.py`
- Modify: `dimos/perception/test_target_verification.py`

**Step 1: Write failing fake-client tests**

Use an injected fake client to verify:

- `responses.parse()` receives `store=False`;
- input contains one text instruction and exactly the selected JPEG data URLs;
- `text_format` is `TargetVerification`;
- model name and timeout are configurable;
- one retry occurs for a timeout;
- terminal provider failure returns `uncertain`, never raises movement approval;
- parsed output is passed through the local multi-view policy.

**Step 2: Run tests and verify failure**

Expected: `OpenAIResponsesVisionVerifier` is missing.

**Step 3: Implement the verifier**

Default to `gpt-5.6-terra`, eight-second timeout, and one retry. Read
`OPENAI_API_KEY` only when creating the real client. Do not log or serialize the
key. Use Pydantic Structured Outputs through `responses.parse()`.

**Step 4: Run focused tests**

Expected: all verifier tests pass without network or credentials.

### Task 4: Add an explicit offline CLI

**Files:**
- Create: `dimos/perception/offline_target_verification.py`
- Create: `dimos/perception/test_offline_target_verification.py`

**Step 1: Write failing CLI tests**

Verify:

- default invocation is a dry run and never constructs an OpenAI client;
- `--send` is the only path that calls the verifier;
- printed and saved JSON excludes base64 image bytes;
- missing memory or frame IDs return a nonzero exit code with a concise error.

**Step 2: Run CLI tests and verify failure**

Run:

```bash
.venv/bin/python -m pytest dimos/perception/test_offline_target_verification.py -q
```

Expected: module import failure.

**Step 3: Implement the CLI**

Supported invocation:

```bash
.venv/bin/python -m dimos.perception.offline_target_verification \
  --memory assets/output/memory/spatial_memory/visual_memory.pkl \
  --candidate-id first-door-candidate \
  --target "a real passable indoor door or doorway" \
  --frame frame_1 --frame frame_2 --frame frame_3
```

The command above is local-only. Add `--send` to authorize the selected images
for one OpenAI request. Add optional `--model` and `--output`.

**Step 4: Run CLI tests**

Expected: all CLI tests pass without a network request.

### Task 5: Verify against the saved Go2 memory and document the boundary

**Files:**
- Modify: `docs/PROJECT_CONTEXT.md`

**Step 1: Run focused quality checks**

```bash
.venv/bin/python -m pytest \
  dimos/perception/test_target_verification.py \
  dimos/perception/test_offline_target_verification.py -q
.venv/bin/ruff check \
  dimos/perception/target_verification.py \
  dimos/perception/offline_target_verification.py \
  dimos/perception/test_target_verification.py \
  dimos/perception/test_offline_target_verification.py
git diff --check
```

Expected: all pass.

**Step 2: Run a local-only dry run on real saved frames**

Use the persisted memory and the known candidate frame IDs. Confirm:

- the file contains 578 images;
- only the explicitly selected candidate frames enter the bundle;
- each resized image is at most 1024 pixels on its longest edge;
- no API request occurs without `--send`.

**Step 3: Check credential readiness without revealing a secret**

Report only whether `OPENAI_API_KEY` is configured. If it is absent, do not make
an API request and state that real model verification remains pending.

**Step 4: Update project context**

Record files, commands, test results, real-memory dry-run results, credential
readiness, unresolved API validation, and the next action. Do not claim the
model recognized a door unless a real structured response was received.

**Step 5: Commit only first-stage files**

Stage the plan, new verifier/CLI/tests, and `docs/PROJECT_CONTEXT.md`. Do not
stage unrelated existing worktree changes.
