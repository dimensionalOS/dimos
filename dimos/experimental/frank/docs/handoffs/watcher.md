# Handoff: rolling face watcher + world state + wake events

FRANK is a Go2 robot dog driven by an LLM agent (Pi) on a laptop through shell scripts in
`dimos/experimental/frank/`. Today the agent only knows who is in front of it when it
explicitly runs `identify.py who` on one frame. You are making FRANK continuously aware: a rolling
face watcher, a "who is where" world state that the agent sees on every turn without it being stored
in its history, and events that wake the agent when someone it cares about shows up.

Read, in order: `SKILL.md`, `app/API.md`, `identify.py` (the matcher you'll reuse), `loop.py` and
`pi_world.ts` (how the world block reaches the model ephemerally), `inbox.py`, `app/server.py` and
`app/lib/*.py`, `robot.py` (how to attach to the running DimOS instance), `TASKS.md`.

## What to build

### 1. `watch.py`: the rolling watcher

A long-running laptop process. Reads frames, recognizes faces, posts sightings.

```bash
uv run python dimos/experimental/frank/tools/watch.py                 # camera stream from the running DimOS instance
uv run python dimos/experimental/frank/tools/watch.py --source webcam # laptop webcam, for testing without the robot
uv run python dimos/experimental/frank/tools/watch.py --fps 4 --show  # optional preview window with boxes and names
```

- Frame source: DimOS `color_image` stream via `Dimos.connect()` (see `robot.py`; `peek_stream` gives
  a snapshot, look in `dimos/porcelain/dimos.py` for a subscribe path and prefer it), or OpenCV
  webcam. Process at a few frames per second, not every frame. Hold one `identify.Recognizer` for the
  life of the process.
- For each match above threshold, estimate where the person is:
  - bearing from the face's pixel column using the camera intrinsics on the `camera_info` stream
    (fx, cx); fall back to a 120° horizontal FOV assumption for the webcam;
  - range from face width in pixels against a 16 cm head width (state that it's ±30%);
  - world x, y by rotating (range, bearing) by the robot's `odom` pose. With `--source webcam` the
    robot pose is (0, 0, 0).
- Post each sighting to the server (`POST /agent/sightings`, extend the body with `bearing_deg`,
  `range_m`, `face_px`, `in_view: true`). Throttle to one post per person per second. Also post an
  explicit "left view" when a person hasn't been seen for 3 s (`in_view: false`).
- Log one line per state change to stdout ("Alice entered view 1.8 m ahead-left", "Alice left view").
  Never spam per frame.

### 2. Server: world state, watches, events

Add to `app/server.py` / `app/lib/` and document in `app/API.md` (append a "World state" section;
do not change existing endpoints):

- `GET /agent/world` → `{"as_of": ts, "people": [{person_id, name, in_view, last_seen_ts, x, y,
  bearing_deg, range_m, last_chat_ts}]}` sorted by most recently seen. One row per person, no
  duplicates. `in_view` flips false on the "left view" post or after 5 s without a sighting.
- `POST /agent/watch` `{person_id}` and `DELETE /agent/watch/{person_id}`; `GET /agent/watch` lists.
  Watches expire after 10 minutes.
- Events (through the existing queue, same delivery rules):
  - `{"type": "found", person_id, name, x, y, bearing_deg, range_m, ts}` when a sighting arrives for
    a watched person. Highest priority, above chat. Emitting it clears the watch.
  - `{"type": "seen", ...same fields}` when a person enters view after not being seen for 10 minutes
    (configurable in `tasks.yaml` as `seen_event_gap_s`). Below chat in priority.
- Sightings count as agent activity for the idle scheduler only when they are `found`; a busy room
  full of `seen` should not stop wakes.

### 3. `inbox.py`: two commands

- `world` → prints the world state as a short table (name, in view / last seen ago, position,
  bearing and range when in view, last chat ago). Importable `world()` returning the JSON.
- `watch-for <person_id>` / `unwatch <person_id>` → one confirmation line each.

### 4. `loop.py`: use the real world state and surface events

- `world_block()` currently formats `inbox.people()`. Switch it to `inbox.world()` and render the
  same table `inbox.py world` prints, with the `as_of` time. Keep the rest of the mechanism exactly as
  it is: written to `cache/world.txt`, injected by `pi_world.ts`, never in the prompt.
- `prompt_for()`: for `found` and `seen` events, prefix the prompt with a one-line system-style
  header, e.g. `[watcher] Alice seen 2 s ago, 1.8 m ahead-left at (2.1, -0.4). You were looking for
  her.` for `found`, and `[watcher] Bob just came into view, 3.0 m ahead.` for `seen`. Keep the
  JSON below it as today.

### 5. `SKILL.md` and `persona.md`

- `SKILL.md`: a short "Watcher" section: what `watch.py` is, that the world state arrives every turn
  by itself, `inbox.py world` for a fresh read mid-turn, `watch-for` while searching, and that a
  `found` event ends any search: stop, level, greet.
- `persona.md`: one line under the mechanics so Frank knows a `[watcher]` header is his own eyes,
  not a person talking, and he shouldn't answer it in chat.

## Constraints

- No new dependencies. OpenCV, numpy, requests, fastapi are in the venv.
- Do not touch anything outside `dimos/experimental/frank/`. Do not edit `dimos/`. Do not commit.
- Never send motion commands. `watch.py` only reads.
- `loop.py`'s ephemeral world-state mechanism is settled: the system prompt must stay constant across
  turns and the world block must never be stored in the session. Don't rework it.
- Keep `app/API.md` the single source of truth; add, don't change.

## Testing

- Server: unit-level checks with curl for `world`, `watch`, `found` and `seen` events, expiry, the
  no-duplicates guarantee, and priority (`found` beats a queued chat).
- Watcher against the webcam: enroll yourself through the PWA (server running at 127.0.0.1:7790, the
  onboarding page works in Chrome), run `watch.py --source webcam --show`, walk in and out of frame;
  `inbox.py world` should flip `in_view` and positions should move sensibly as you step left/right and
  closer. Then `inbox.py watch-for <you>`, step out and back in, and `inbox.py wait` should return the
  `found` event.
- Full loop once: `loop.py --harness pi --once` with a `found` event queued and confirm Frank's reply
  reads as if he saw the person, and that the world text is absent from the Pi session file
  (`grep -c "WORLD STATE" ~/.pi/agent/sessions/--home-dimos-dimos--/*<session>*`).
- Stop the server and watcher and wipe `app/data/` when done.

## Hand back

What works, how you tested (webcam or robot), sample `inbox.py world` output, position accuracy you
observed at roughly 1 m and 2 m, and anything in the contract you had to add. Update `TASKS.md`.
