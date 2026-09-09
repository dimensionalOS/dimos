# Handoff: FRANK agent-side CLI (`inbox.py`)

FRANK is a Go2 robot dog driven by an LLM agent running on a laptop. The agent may be Claude Code,
Pi, Codex, or anything else that can run a shell command, so it never gets prompts pushed into it.
Instead it polls a mailbox. You are building that mailbox client.

    dimos/experimental/frank/tools/inbox.py

Read `app/API.md` first. The server (`app/server.py`) is being built in parallel by someone else
against the same contract. Also read `TASKS.md` and tick your item off when done.

## What to build

One script, `inbox.py`, argparse subcommands exactly as listed in the "Agent-side CLI" section of
`API.md`: `wait`, `send`, `people`, `history`, `selfie`, `sighting`, `done`. Server URL from
`FRANK_URL`, default `http://127.0.0.1:7790`. Token from `FRANK_AGENT_TOKEN` if set.

Output rules, because an LLM reads this:

- `wait` prints exactly one JSON object on one line and exits 0. On timeout it prints
  `{"type": "none"}` and exits 3. On a connection error it prints one plain-English line to stderr
  and exits 2. Never print anything else.
- `people` prints a compact table: id, name, minutes since last chat, minutes since last seen,
  follow-ups today. One person per line. Empty list prints `no people yet`.
- `history` prints one message per line: `HH:MM  Alice: text` / `HH:MM  FRANK: text`.
- `send`, `sighting`, `done` print one confirmation line.
- `selfie` writes the JPEG to the given path and prints the path.
- Also expose the same operations as plain functions (`wait_event`, `send`, `people`, ...) so a
  script can `import inbox` instead of shelling out.

Add a "Chat and wake loop" section to `SKILL.md` in the parent directory that tells an agent how
to use this: run `wait`, act on the event, reply with `send`, close wakes with `done`, run `wait`
again. Keep it under 20 lines and match the tone of what's already there.

## Also: `qr.py`

A visitor scans a QR sticker on the robot to open the chat app. Build `dimos/experimental/frank/tools/qr.py`:

```bash
uv run python dimos/experimental/frank/tools/qr.py https://abc123.ngrok.app            # writes qr.png, prints the path
uv run python dimos/experimental/frank/tools/qr.py https://abc123.ngrok.app --print    # also sends it to the default printer with `lp`
uv run python dimos/experimental/frank/tools/qr.py --ngrok --print                     # reads the public URL from ngrok's local API (localhost:4040/api/tunnels)
```

The PNG should be a full page: the QR large in the middle, "Scan to meet FRANK" above it in big
letters, the URL in small text below. Use the `qrcode` package if `uv run python -c "import qrcode"`
works, otherwise the `qrencode` CLI if installed; if neither exists, say so in your hand-back note
rather than adding a dependency. Use Pillow (already in the venv) for the page layout.

## Constraints

- Python deps: `requests` only, plus stdlib. It's already in the venv.
- Do not touch anything outside `dimos/experimental/frank/`. Do not edit `dimos/`. Do not commit.
- If the server isn't ready when you start, write a 60-line fake in
  `dimos/experimental/frank/app/fake_server.py` that serves `API.md` from an in-memory list, just enough
  to test the CLI. Delete it or leave it clearly labelled as a test double.

## Test it before you hand back

- `wait --timeout 3` against nothing running: stderr line, exit 2.
- `wait` against the (real or fake) server with an event queued: JSON line, exit 0.
- `wait` with an empty queue: `{"type": "none"}`, exit 3, returns promptly at the timeout.
- Round trip: `send`, then `history` shows it.

## Hand back

A short note: what works, how you tested it, anything in `API.md` that was ambiguous. Update `TASKS.md`.
