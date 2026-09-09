# Handoff: FRANK face identification (`identify.py`)

FRANK is a Go2 robot dog at a demo. Visitors enroll with a name and a selfie through a chat app.
Later, FRANK looks at a camera frame and needs to know who is in it. You are building that matcher.

    dimos/experimental/frank/tools/identify.py

Read `app/API.md` for where selfies live and how to fetch them. Also read `TASKS.md` and tick your
item off when done. The robot is not available; work from photos.

## What to build

`identify.py` with a CLI and an importable API:

```bash
uv run python dimos/experimental/frank/tools/identify.py enroll                 # (re)build embeddings for every selfie in app/data/people/
uv run python dimos/experimental/frank/tools/identify.py who frame.jpg          # one line per face found, best first
uv run python dimos/experimental/frank/tools/identify.py who frame.jpg --json   # machine-readable
```

`who` output, one face per line, largest face first:

```
alice  0.81  bbox=412,88,530,240  size=118px
unknown  0.34  bbox=...  size=61px      (below threshold: best guess name after 'unknown?' if you like)
```

JSON: `[{"person_id", "name", "confidence", "bbox": [x1,y1,x2,y2], "face_px"}]`, `name: null` when
unknown. Exit 0 if any face matched, 4 if faces but no match, 5 if no faces.

Embeddings are cached in `app/data/embeddings.json` (or sqlite alongside), keyed by person_id and
selfie file hash, so `who` is fast and `enroll` only recomputes what changed.

## Approach

- Preferred: a face embedding model that runs on CPU at a few frames per second. Check what the venv
  already has before adding anything: `uv run python -c "import insightface"` / `facenet_pytorch` /
  `onnxruntime`. If none are present, pick the smallest dependable option and state the dependency
  clearly in your hand-back note instead of silently adding it to `pyproject.toml`.
- Cosine similarity against one enrollment embedding per person. Threshold that gives no false
  positives on your test set; report the threshold you chose and why.
- Report face size in pixels. FRANK uses it to decide whether to get closer: the Go2 camera is
  1280×720, wide angle, and faces under about 60 px are unreliable.
- Second signal, optional: if time allows, add `body` embeddings via the repo's torchreid model
  (`dimos/models/embedding/treid.py`) so a person can be re-identified by clothing after their first
  confirmed sighting of the day. Keep it behind a flag; face first.

## Constraints

- Do not touch anything outside `dimos/experimental/frank/`. Do not edit `dimos/`. Do not commit.
- No network calls at inference time. Model weights may download once at enroll.
- Handle: no faces, many faces, sideways faces, a selfie that has no detectable face at enrollment
  (return a clear error the chat server can show the user).

## Test it before you hand back

- Build a small test set from your own webcam or free-licence photos: at least 4 identities,
  frames at roughly 1 m, 2 m, and 3 m equivalents (downscale to simulate distance), one frame with
  two people. Put it in `app/data/test/` (gitignored) and a `test_identify.py` next to the script
  that runs the set and prints a confusion summary. Do not commit photos of real people.
- Timing: print how long `who` takes on a 1280×720 frame.

## Hand back

Accuracy on your set, threshold, timing, the dependency you used, and known failure modes.
Update `TASKS.md`.
