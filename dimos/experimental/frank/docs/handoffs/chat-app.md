# Handoff: FRANK chat server + phone app

You are building the chat app for FRANK, a Unitree Go2 robot dog that meets people at a demo.
A visitor scans a QR sticker on the robot, tells FRANK their name, takes a selfie, and then chats
with the agent that drives the robot. Everything you build lives in one directory of the DimOS repo:

    dimos/experimental/frank/app/

Read `app/API.md` first. It is the contract. Do not change endpoint paths or shapes without
updating it; other people are building against it in parallel (an agent-side CLI, a face matcher).
Also read `TASKS.md` in the parent directory and tick your items off when done.

## What to build

1. `app/server.py`: FastAPI + uvicorn, run with `uv run python dimos/experimental/frank/app/server.py`
   from the repo root. Implements every endpoint in `API.md`: the phone side, the agent side, the
   event queue, and the idle scheduler. SQLite in `app/data/frank.db`, selfies in `app/data/people/`.
   Keep it one file if it stays under about 500 lines; otherwise split helpers into `app/lib/`, one
   module per concern (`store.py`, `events.py`, `scheduler.py`). No `__init__.py` files.
2. `app/static/index.html` plus whatever CSS/JS it needs, served at `/`. Phone-first. Two screens:
   - Onboarding: "Introduce yourself to FRANK", a friendly robot-dog placeholder image, name field,
     selfie via `<input type=file accept=image/* capture=user>` with a preview, one consent line
     ("FRANK remembers your face and chats for today. Tap 'forget me' any time."), submit.
     Store `person_id` and `name` in localStorage.
   - Chat: message list, input box, send. Poll `GET /api/people/{id}/messages?after=&wait=25`
     in a loop for new messages. Show a small "FRANK is thinking" indicator while the last message
     is from the person. "Forget me" in a menu. Messages from FRANK should look like a chat bubble
     from a robot, not a system log.
   Plain HTML/JS, no build step, no framework. Must work in iOS Safari and Android Chrome.
3. `app/tasks.yaml` with the defaults in `API.md`, and the scheduler that reads it.
4. `app/README.md`: how to run it, how to expose it (`ngrok http 7790`), how to reset data.

## Constraints

- Python deps: only what the repo venv already has (`fastapi`, `uvicorn`, `requests`, `pyyaml`,
  `pydantic` are all there). Check with `uv run python -c "import x"` before assuming. If you truly
  need something new, say so in your final message instead of adding it.
- Do not touch anything outside `dimos/experimental/frank/`. Do not edit `dimos/`.
- Do not commit. Leave the working tree for the user to review.
- No robot is available. Nothing you build talks to the robot directly.
- Agent endpoints must refuse requests that are not from loopback unless `FRANK_AGENT_TOKEN` is set
  and matches an `Authorization: Bearer` header. The phone side is public through ngrok.
- Long polls must release cleanly on client disconnect and on server shutdown.
- Event delivery is exactly once, in order, one consumer. Use an asyncio queue plus a persisted
  "delivered" marker so a server restart does not replay old chats as new events.
- Write for humans: short functions, plain names, comments only where the reason is not obvious.

## Test it before you hand back

- Start the server. In a browser at `http://127.0.0.1:7790`, enroll a person with a selfie, send a
  message. `curl 'localhost:7790/agent/events?wait=5'` returns the `enrolled` then the `chat` event.
  `curl -X POST localhost:7790/agent/send -d '{"person_id": ..., "text": "hello"}' -H 'content-type: application/json'`
  shows up in the browser without a refresh.
- Set `idle_after_s: 10` and `cooldown_after_wake_s: 5` in a scratch copy of `tasks.yaml`, confirm a
  `wake` arrives after idling, that a second wake waits for `done`, and that per-person limits hold.
- Restart the server mid-conversation; no duplicate events, no lost messages.
- Two phones (two browser profiles) chatting at once, both get only their own messages.

## Hand back

A short note: what works, what you tested by hand, anything in `API.md` you had to change and why,
and anything you left out. Update `TASKS.md`.
