# FRANK server contract

One FastAPI process on the laptop. Phones reach it through an ngrok tunnel (HTTPS, required for
the selfie camera). The agent reaches it on localhost. Every part of FRANK builds against this file.

```
uv run python dimos/experimental/frank/app/server.py        # http://127.0.0.1:7790
ngrok http 7790                                          # public https URL for the QR code
```

Port: `FRANK_PORT` env, default `7790`. Data: `dimos/experimental/frank/app/data/` (sqlite `frank.db`
plus `people/<id>.jpg`), gitignored, wiped at the end of a demo day.

## Phone side (public, served through ngrok)

| Method | Path | Body / query | Returns |
|---|---|---|---|
| GET | `/` | | PWA. Onboarding if the browser has no `person_id` in localStorage, chat otherwise |
| POST | `/api/people` | `{name, selfie?}` optional/null selfie = base64 JPEG (data URL ok) | `{person_id, name}` |
| GET | `/api/people/{id}` | | `{person_id, name, created_at}` |
| POST | `/api/people/{id}/messages` | `{text}` | the stored message |
| GET | `/api/people/{id}/messages?after=<msg_id>&wait=<sec>` | long poll, `wait` ≤ 30 | `{messages: [...]}` |
| POST | `/api/people/{id}/forget` | | `{ok: true}` deletes selfie, embedding, chats |

Message shape everywhere:

```json
{"id": 42, "person_id": "p_3f9a", "from": "person" | "frank", "text": "hi", "ts": 1725400000.0}
```

## Agent side (localhost only; refuse non-loopback unless `FRANK_AGENT_TOKEN` matches)

| Method | Path | Body / query | Returns |
|---|---|---|---|
| GET | `/agent/events?wait=<sec>` | long poll, `wait` ≤ 120 | one event, or `{"type": "none"}` on timeout |
| POST | `/agent/send` | `{person_id, text}` | the stored message (also pushed to the phone) |
| GET | `/agent/people` | | `[{person_id, name, created_at, last_chat_ts, last_seen_ts, last_seen_pose, follow_ups_today}]` |
| GET | `/agent/people/{id}/history?since_minutes=<n>` | | `{messages: [...]}` |
| GET | `/agent/people/{id}/selfie.jpg` | | the enrollment photo |
| POST | `/agent/sightings` | `{person_id, confidence, pose: {x, y, yaw}}` | `{ok: true}` |
| POST | `/agent/tasks/{task_id}/done` | `{outcome: "done" | "not_found" | "skipped", note?}` | `{ok: true}` |

Events, in the order the agent should expect them. Chat events always win over wakes.

```json
{"type": "chat", "person_id": "p_3f9a", "name": "Alice", "text": "what's your name?", "ts": 1725400000.0}
{"type": "enrolled", "person_id": "p_3f9a", "name": "Alice", "ts": 1725400000.0}
{"type": "wake", "task_id": "t_17", "task": "follow_up", "person_id": "p_3f9a", "name": "Alice",
 "last_chat_minutes_ago": 31, "last_seen_pose": {"x": 2.1, "y": -0.4, "yaw": 90}, "history": [ ...last 6 messages... ]}
{"type": "wake", "task_id": "t_18", "task": "greet_known", "person_id": "...", "name": "..."}
{"type": "wake", "task_id": "t_19", "task": "explore"}
{"type": "none"}
```

Events are delivered exactly once, in order, to whoever is polling. If nobody polls for 10 minutes the
queue is dropped, since a stale wake is worse than none.

## Idle scheduler (inside the server)

Config `app/tasks.yaml`, reloaded on change:

```yaml
idle_after_s: 300          # no chat or agent activity for this long → consider a wake
cooldown_after_wake_s: 180
tasks:
  follow_up:
    chat_age_min_s: 1200    # talked 20 min .. 3 h ago
    chat_age_max_s: 10800
    max_per_person_per_day: 2
    min_gap_per_person_s: 2700
  greet_known:
    seen_within_s: 120
    no_chat_for_s: 1800
    max_per_person_per_day: 2
  explore: {}               # fallback when nothing else qualifies
```

"Activity" is any phone message, any `/agent/*` write, or any `/agent/events` poll that returned a real
event. A wake is only emitted when `idle_after_s` has passed and `cooldown_after_wake_s` since the last
wake has passed. `task_id` must be closed with `/agent/tasks/{id}/done` before the next wake is emitted;
an unclosed task times out after 5 minutes and counts as `not_found`.

## Agent-side CLI (`inbox.py`)

Thin wrapper over `/agent/*`, so any harness that can run a shell command can drive FRANK:

```bash
uv run python dimos/experimental/frank/tools/inbox.py wait --timeout 90     # prints one event as JSON, exit 0; exit 3 on "none"
uv run python dimos/experimental/frank/tools/inbox.py send p_3f9a "Hi Alice!"
uv run python dimos/experimental/frank/tools/inbox.py people
uv run python dimos/experimental/frank/tools/inbox.py history p_3f9a --since 60
uv run python dimos/experimental/frank/tools/inbox.py selfie p_3f9a out.jpg
uv run python dimos/experimental/frank/tools/inbox.py sighting p_3f9a 0.92 --pose 2.1 -0.4 90
uv run python dimos/experimental/frank/tools/inbox.py done t_17 done
```

Server URL: `FRANK_URL` env, default `http://127.0.0.1:7790`.

## World state (the rolling watcher)

`watch.py` runs on the laptop, recognizes faces a few times a second, and posts what it sees. The
server keeps that in memory — a sighting only matters while it is fresh — and answers "who is where".

`POST /agent/sightings` takes extra optional fields; the old body still works unchanged:

```json
{"person_id": "p_3f9a", "confidence": 0.71, "in_view": true,
 "bearing_deg": 24.5, "range_m": 1.8, "face_px": 88, "x": 2.1, "y": -0.4,
 "pose": {"x": 2.1, "y": -0.4, "yaw": 0}}
```

`bearing_deg` is positive to FRANK's left; `range_m` comes from face width against a 16 cm head, so
it is ±30%; `x, y` are the person in the world frame. `in_view: false` is the watcher saying the
person is gone; it updates the world state but never touches `last_seen_ts` or fires an event.
The reply is `{"ok": true, "event": "found" | "seen" | null}`. Sightings no longer count as agent
activity for the idle scheduler — only a delivered `found` does.

| Method | Path | Body / query | Returns |
|---|---|---|---|
| GET | `/agent/world` | | `{as_of, people: [...]}`, one row per enrolled person, in view first then most recently seen |
| POST | `/agent/watch` | `{person_id}` | `{ok, person_id, expires_at}` — tell me when you see them |
| DELETE | `/agent/watch/{person_id}` | | `{ok}` — `ok: false` if there was no watch |
| GET | `/agent/watch` | | `{watches: [{person_id, expires_in_s}]}` |

```json
{"person_id": "p_3f9a", "name": "Alice", "in_view": true, "last_seen_ts": 1725400000.0,
 "x": 2.1, "y": -0.4, "bearing_deg": 24.5, "range_m": 1.8, "last_chat_ts": 1725399000.0}
```

`in_view` goes false on an `in_view: false` post or 5 s after the last sighting. `bearing_deg` and
`range_m` are null when the person is not in view — they were true of a moment that has passed.
Watches expire after 10 minutes.

Two more events, delivered through the same queue with the same rules:

```json
{"type": "found", "person_id": "p_3f9a", "name": "Alice", "x": 2.1, "y": -0.4,
 "bearing_deg": 24.5, "range_m": 1.8, "ts": 1725400000.0}
{"type": "seen", "person_id": "p_3f9a", "name": "Alice", "x": 2.1, "y": -0.4,
 "bearing_deg": 24.5, "range_m": 1.8, "ts": 1725400000.0}
```

`found` fires when a watched person is recognized, and emitting it clears the watch. `seen` fires
when someone comes into view after `seen_event_gap_s` (`tasks.yaml`, default 600 s) out of sight.
Delivery order is now `found`, then `chat` / `enrolled`, then `seen`, then `wake`.

Two more `inbox.py` commands:

```bash
uv run python dimos/experimental/frank/tools/inbox.py world              # the table above
uv run python dimos/experimental/frank/tools/inbox.py watch-for p_3f9a   # then `wait` returns a `found`
uv run python dimos/experimental/frank/tools/inbox.py unwatch p_3f9a
```

## Phone identity persistence

The phone remembers who it is in `localStorage` (`person_id`, `name`), written on a successful
enrollment. Every read and write is wrapped in try/catch: with storage unavailable the app still
works, it just forgets you on reload.

On load the app validates the stored id against the server before trusting it, because the store is
wiped at the end of a demo day and a phone holding a dead id would chat into a void:

| Method | Path | Body / query | Returns |
|---|---|---|---|
| GET | `/api/people/{id}` | | `{person_id, name, created_at}`, or 404 (existing route) |
| GET | `/api/people/{id}/history?limit=<n>` | last `n` messages, `n` ≤ 200, default 50 | `{messages: [...]}` |

- 200 → skip onboarding, open the chat as that person, redraw the last messages from
  `/history`, then long-poll `/messages?after=<last id>` as usual.
- 404 → clear the stored identity and show onboarding.
- network error → keep the identity and open the chat; the poller retries.

`/api/people/{id}/history` is the phone's mirror of `/agent/people/{id}/history`: newest-last, same
message shape, bounded by count instead of by age. Both are plain `/api` routes with no
`agent_only`, so they work through the funnel.

The chat header area also carries an unobtrusive "Not <name>? Start over" control, which clears the
stored identity and returns to onboarding without forgetting the person on the server ("forget me"
in the menu is still the destructive one).

## World state survives a restart

`lib/world.py` used to be memory-only, so a server restart lost every position. The latest sighting
per person is now written through to sqlite (`sightings_last`: `x, y, bearing_deg, range_m,
face_px, last_seen_ts`) on every `POST /agent/sightings`, and reloaded into `World` at startup, with
`in_view` false until the watcher posts again. `GET /agent/world` therefore shows the same positions
after a restart. Watches stay in memory on purpose — "I am looking for Alice right now" should not
outlive the process. `forget` deletes the row with everything else.

## Push-to-talk (phone)

The chat screen has a hold-to-talk mic under the composer. The transcript lands in the text input
and sends itself ~1 s later, so the person can tap the text to edit or cancel first; what reaches
the agent is an ordinary chat message. Two tiers, in order:

1. `SpeechRecognition` / `webkitSpeechRecognition` in the browser (`continuous: false`,
   `interimResults: true`, page language), so the words appear while talking. No server work.
2. If that API is missing or errors: `MediaRecorder` → the endpoint below → ElevenLabs `scribe_v1`.

| Method | Path | Body | Returns |
|---|---|---|---|
| POST | `/api/people/{id}/transcribe` | multipart, field `audio` (webm or mp4, whatever the browser produced) | `{text}` |

503 when `ELEVENLABS_API_KEY` is not configured (the UI then says "voice unavailable here, type
instead"), 404 for an unknown person, 502 if ElevenLabs fails. The key is read from
`dimos/experimental/frank/.env` then the repo root `.env`, the same way `speak.py` loads it.

## Operator dashboard (`/ops`, laptop only)

A read-only QA window on a live run, served by the same process from `app/lib/ops.py` and
`app/static/ops/`. Every `/ops` route — pages, static files and API — carries the `agent_only`
dependency, so the funnel (which adds `X-Forwarded-For`) gets a 403. It polls every 2 s.

| Method | Path | Returns |
|---|---|---|
| GET | `/ops` | the dashboard |
| GET | `/ops/static/{file}` | its css/js |
| GET | `/ops/api/people` | `{people: [...]}` — `GET /agent/people` rows |
| GET | `/ops/api/selfie/{id}.jpg` | the enrollment photo, for thumbnails |
| DELETE | `/ops/api/people/{id}` | `{ok: true}` — erase a person (chats, tasks, selfie, world row), same as "forget me" |
| GET | `/ops/api/world` | the `GET /agent/world` snapshot |
| GET | `/ops/api/events?limit=<n>` | `{events: [...], tasks: [...]}` — the whole `events` table newest first, plus wake tasks and outcomes |
| GET | `/ops/api/messages?limit=<n>` | `{messages: [...]}` — every chat message, both directions, newest first |
| GET | `/ops/api/context` | what the agent is actually being sent (below) |

`events` rows carry `delivered` (the queue keeps a flag, not a delivery time) and `retired`
(delivered-or-dropped and older than the 10 min queue lifetime).

`/ops/api/context` returns:

```json
{"loop": {"harness": "pi", "session": "frank-20260903", "model": null, "started_ts": 0.0,
          "session_file": "/home/dimos/.pi/agent/sessions/--home-dimos-dimos--/...jsonl"},
 "session_file": "...", "messages": [{"role": "user|assistant|thinking|toolCall|toolResult|bash|meta",
   "ts": "...", "tool": "read", "text": "...", "full": "...", "chars": 9001}],
 "world_block": "WORLD STATE ...", "world_block_note": "injected at call time, not stored",
 "log": ["last 50 lines of cache/loop.log"]}
```

`messages` is the harness session flattened in order — prompts, assistant text, thinking, tool
calls with arguments, tool results. Anything over 1200 characters is truncated, with `full` and
`chars` carried alongside for the UI's "show all" toggle. The world block is shown last and marked
as ephemeral: it is regenerated every turn and appended at LLM-call time, so it never lands in the
session history.

`loop.py` writes `cache/loop_state.json` (`{harness, session, model, started_ts, session_file}`)
when it starts; the endpoint re-globs `~/.pi/agent/sessions/--home-dimos-dimos--/*<session>*.jsonl`
for the newest match, because Pi only creates the file on the first turn.

## Restarting Frank from the dashboard

The `/ops` strip at the top can restart the harness loop, so an edit to `SKILL.md`, `persona.md` or
any script is picked up and no earlier turns come along.

| Method | Path | Body | Returns |
|---|---|---|---|
| GET | `/ops/api/loop` | | `{running, pid, session, harness, model, started_ts}` |
| POST | `/ops/api/loop/restart` | `{harness?: "pi", model?: null}` | `{ok, session, pid, killed: [pids]}` |
| POST | `/ops/api/loop/stop` | | `{ok, killed: [pids]}` |

Restart kills every process matching `frank/loop\.py` and any harness turn it spawned
(`pi -p --session-id frank-`) with SIGTERM, SIGKILL after 2 s, then starts
`python -u loop.py --harness <h> --session frank-YYYYMMDD-HHMMSS [--model m]` from the repo root,
detached (`start_new_session`), `DIMOS_TRANSPORT=lcm`, stdin `/dev/null`, stdout and stderr
appended to `cache/loop.stdout.log` — `loop.py` already writes its own turn log to `cache/loop.log`,
so redirecting there too would double every line. The dashboard tails `loop.log` as the turn log and
shows `loop.stdout.log` only for the lines the turn log does not have (Pi's stderr, tracebacks). Matching is `pgrep` + `os.kill` on purpose: `pkill -f` on those
patterns would also match the server's own command line.

Normally `up.py` runs the loop in the foreground. After a dashboard restart that foreground process
is gone and the new loop is detached, so its output is in `cache/loop.log` (the dashboard's turn log
panel, with anything extra from `cache/loop.stdout.log` under it) rather than on the terminal. The agent-context panel follows the new session automatically:
`loop.py` rewrites `cache/loop_state.json` on start and `/ops/api/context` resolves the session
named there.

## Motion guard and halt (`/ops`)

Motion is on unless `cache/MOTION_OFF` exists; `robot.py` refuses to move while it does.

| Method | Path | Body | Returns |
|---|---|---|---|
| GET | `/ops/api/motion` | | `{on: bool}` — `on` is "no MOTION_OFF file" |
| POST | `/ops/api/motion` | `{on: bool}` | `{on: bool}` — false touches `cache/MOTION_OFF`, true removes it |
| POST | `/ops/api/halt` | | `{ok, exit_code, output, motion_on: false}` |

Halt runs `uv run python robot.py stop` from the repo root first — cancelling the navigation goal in
flight, which works whatever the guard says — and only then touches `MOTION_OFF`, so Frank cannot
start another move until the dashboard's motion toggle is switched back on. The strip polls
`/ops/api/motion` with everything else, showing a green "Motion ON" / red "Motion OFF" button.

Enrollment accepts an omitted or null `selfie`; only `name` is required. Name-only visitors can chat but cannot be matched by the face watcher. Their selfie endpoint returns 404.

`POST /agent/watch` accepts optional `say` (up to 300 characters). On the first confirmed
match, the server posts and speaks that exact response without LLM inference; the found event
includes `automatic_response` so the agent avoids repeating it. CLI: `inbox.py watch-for ID --say "Hello!"`.
`GET /ops/api/runtime` reports listener health, operations, watches, process IDs and recent activity.
`POST /ops/api/runtime/stop` accepts `kind` (`mcp`, `face_watcher`, `operation`, `watch`) and `id`
for an operation/watch. Stopping a sensory listener also turns motion off.
