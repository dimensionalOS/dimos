# FRANK chat server

One FastAPI process. Phones talk to `/api`, the agent talks to `/agent`.
The contract is `API.md` — read that first.

## Run it

```bash
uv run python dimos/experimental/frank/app/server.py     # http://127.0.0.1:7790
```

`FRANK_PORT` changes the port. The server binds `0.0.0.0` so a tunnel or the LAN can reach it.

## Expose it to phones

The selfie camera needs HTTPS, so phones go through a tunnel. The demo uses Tailscale Funnel on a
fixed address; the steps, the URL, and the QR printing are in `../README.md`.

## The agent side

Localhost is trusted. Anything else gets a 403 unless you set a token on the server and send it:

```bash
FRANK_AGENT_TOKEN=sekret uv run python dimos/experimental/frank/app/server.py
curl -H 'Authorization: Bearer sekret' http://<host>:7790/agent/people
```

Use `inbox.py` rather than curl for day-to-day driving.

## Idle scheduler

`tasks.yaml` holds the thresholds; the server re-reads it whenever the file changes, no restart.
When nothing has happened for `idle_after_s` the server emits one `wake` event and waits: no further
wake goes out until that `task_id` is closed with `/agent/tasks/{id}/done` (or times out after 5 min
and counts as `not_found`).

For a quick demo of the loop, drop `idle_after_s` to `10` and `cooldown_after_wake_s` to `5`.

## Data and reset

Everything lives in `app/data/` (gitignored): `frank.db` plus one `people/<person_id>.jpg` per visitor.
End of the day:

```bash
rm -rf dimos/experimental/frank/app/data
```

The server recreates it on the next start. A single visitor can erase themselves from the app's menu
("forget me"), which drops their chats, their selfie and anything parked next to it.

## Layout

```
server.py        routes and startup
lib/store.py     sqlite: people, messages, events, wake tasks
lib/events.py    the agent's event queue, and per-person phone wakeups
lib/scheduler.py idle detection and task choice; reads tasks.yaml
static/          the phone app (plain HTML/CSS/JS, no build step)
```
