# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""The outer loop that turns FRANK's mailbox into agent turns, for any CLI harness.

    uv run python dimos/experimental/frank/loop.py --harness pi
    uv run python dimos/experimental/frank/loop.py --harness pi --model gpt-5.5 --once
    uv run python dimos/experimental/frank/loop.py --harness claude

Waits on `inbox.py wait`, hands each event to the harness as one non-interactive prompt with the
frank skill loaded and a persistent session id so the agent remembers earlier turns, then waits
again. The harness does the thinking and runs the skill's scripts itself. Swap harnesses by name.
"""

from __future__ import annotations

import argparse
from datetime import date
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from typing import Any
import uuid

from dotenv import load_dotenv
import psutil
import requests

HERE = Path(__file__).parent
REPO = HERE.parents[2]
SKILL_DIR = str(HERE.relative_to(REPO))
PERSONA = HERE / "persona.md"
WORLD = HERE / "cache" / "world.txt"
LOG = HERE / "cache" / "loop.log"
STATE = HERE / "cache" / "loop_state.json"

sys.path.insert(0, str(HERE / "tools"))
import inbox
import robot


def wait_event(timeout: int) -> dict | None:
    """One `inbox.py wait`. None on timeout; raises on a server error."""
    r = subprocess.run(
        [sys.executable, str(HERE / "tools" / "inbox.py"), "wait", "--timeout", str(timeout)],
        capture_output=True,
        text=True,
        cwd=REPO,
    )
    if r.returncode == 3:
        return None
    if r.returncode != 0:
        raise RuntimeError(r.stderr.strip() or f"inbox.py wait exited {r.returncode}")
    return json.loads(r.stdout.strip().splitlines()[-1])


def world_block() -> str:
    """Who is where, right now. Written to cache/world.txt before every turn; the harness's
    ephemeral-context hook (pi_world.ts for Pi) appends it as the last message at LLM-call time,
    so it never lands in the session history and the cached prefix stays intact."""
    head = "WORLD STATE (regenerated every turn; not part of your history):"
    try:
        state = inbox.world()
    except Exception as e:  # server down: say so rather than pretend the world is empty
        return f"{head}\n  unavailable ({e})"
    body = "\n".join("  " + line for line in inbox.format_world(state).splitlines())
    return f"{head}\n{body}"


def watcher_header(event: dict) -> str:
    """One line of FRANK's own eyes, above the raw event, for watcher events."""
    name = event.get("name") or event.get("person_id", "someone")
    where = inbox.where_words({**event, "in_view": True})
    at = ""
    if event.get("x") is not None:
        at = f" at ({float(event['x']):.1f}, {float(event.get('y') or 0.0):.1f})"
    if event.get("type") == "found":
        return f"[watcher] {name} in view, {where}{at}. You were looking for them."
    return f"[watcher] {name} just came into view, {where}{at}."


def prompt_for(event: dict) -> str:
    header = watcher_header(event) + "\n\n" if event.get("type") in ("found", "seen") else ""
    return (
        header + "New FRANK event:\n"
        f"{json.dumps(event, indent=2)}\n\n"
        "Handle it following the frank skill and your persona, using shell commands from the repo root. "
        "Reply to people with inbox.py send. If this is a wake, finish with inbox.py done."
    )


def harness_cmd(name: str, session: str, model: str | None, prompt: str) -> list[str]:
    WORLD.parent.mkdir(exist_ok=True)
    WORLD.write_text(world_block())
    guidance = (HERE / "SKILL.md").read_text() + "\n\n" + PERSONA.read_text()
    if name == "pi":
        catalog = robot.McpAdapter().list_tools()
        if not catalog:
            raise RuntimeError(
                "No live DimOS skills available; refusing to launch an ungrounded agent"
            )
        (HERE / "cache" / "robot-tools.json").write_text(json.dumps(catalog))
        instructions = HERE / "cache" / "instructions.md"
        instructions.write_text(guidance)
        cmd = [
            "pi",
            "-p",
            "--session",
            str(session_file(session)),
            "--no-context-files",
            "--tools",
            ",".join(
                [
                    "bash",
                    "read",
                    "operation_status",
                    "cancel_operation",
                    *[tool["name"] for tool in catalog],
                ]
            ),
            "--append-system-prompt",
            str(instructions),
            "--extension",
            str(HERE / "pi" / "pi_robot.ts"),
            "--extension",
            str(HERE / "pi" / "pi_world.ts"),
            "--extension",
            str(HERE / "pi" / "pi_events.ts"),
            "--extension",
            str(HERE / "pi" / "pi_cerebras.ts"),
        ]
        if model:
            cmd += ["--model", model]
        return [*cmd, "--", prompt]
    if name == "claude":
        # TODO: no ephemeral-message hook in claude -p yet; the world block is not injected here
        cmd = ["claude", "-p", "--append-system-prompt", guidance]
        if model:
            cmd += ["--model", model]
        # first turn creates the session, later turns resume it; the marker file tells us which
        marker = HERE / "cache" / f"claude-{session}.started"
        if marker.exists():
            cmd += ["--resume", session]
        else:
            cmd += ["--session-id", session]
            marker.parent.mkdir(exist_ok=True)
            marker.touch()
        return [*cmd, prompt]
    raise SystemExit(f"unknown harness {name!r}; add it to harness_cmd()")


def run_turn(args: argparse.Namespace, event: dict) -> None:
    cmd = harness_cmd(args.harness, args.session, args.model, prompt_for(event))
    t0 = time.time()
    code = -1
    proc = subprocess.Popen(
        cmd,
        cwd=REPO,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        start_new_session=True,
    )
    try:
        out, err = proc.communicate(timeout=args.turn_timeout)
        code = proc.returncode
    except subprocess.TimeoutExpired as e:
        out, err, code = (
            (e.stdout.decode(errors="replace") if isinstance(e.stdout, bytes) else e.stdout or ""),
            f"turn timed out after {args.turn_timeout}s",
            -1,
        )
        if event.get("type") == "wake":
            subprocess.run(
                [
                    sys.executable,
                    str(HERE / "tools" / "inbox.py"),
                    "done",
                    event["task_id"],
                    "skipped",
                ],
                cwd=REPO,
            )
    finally:
        # Kill the harness and its tools before stopping robot movement. A timed-out shell
        # must not issue another move after cleanup.
        try:
            children = psutil.Process(proc.pid).children(recursive=True)
        except psutil.Error:
            children = []
        for child in children:
            try:
                child.kill()
            except psutil.Error:
                pass
        try:
            os.killpg(proc.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        proc.wait()
        # Successful conversational turns do not own background operation lifetimes.
        # Failed, interrupted, and timed-out harnesses must still stop movement.
        try:
            if code != 0:
                stopped = subprocess.run(
                    [sys.executable, str(HERE / "tools" / "robot.py"), "stop"],
                    cwd=REPO,
                    capture_output=True,
                    text=True,
                    timeout=130,
                )
                if stopped.returncode:
                    log(f"movement cleanup failed: {stopped.stderr.strip()}")
        except subprocess.TimeoutExpired:
            log("movement cleanup timed out")
    log(f"turn {event.get('type')} {event.get('name', '')} {time.time() - t0:.0f}s exit={code}")
    if out.strip():
        log(out.strip())
    if err.strip():
        log(f"[stderr] {err.strip()}")


def session_file(session: str) -> str | None:
    """Exact current session path assigned at startup, independent of the session label."""
    state = json.loads(STATE.read_text())
    return state.get("session_file") if state.get("session") == session else None


def write_state(args: argparse.Namespace) -> None:
    """cache/loop_state.json: what the ops dashboard needs to find this run's transcript."""
    STATE.parent.mkdir(exist_ok=True)
    STATE.write_text(
        json.dumps(
            {
                "harness": args.harness,
                "session": args.session,
                "model": args.model,
                "started_ts": time.time(),
                "session_file": args.session_file if args.harness == "pi" else None,
                "pid": os.getpid(),
            },
            indent=2,
        )
    )


def log(msg: str) -> None:
    line = f"{time.strftime('%H:%M:%S')} {msg}"
    print(line, flush=True)
    LOG.parent.mkdir(exist_ok=True)
    with LOG.open("a") as f:
        f.write(line + "\n")


def terminate(signum: int, frame: Any) -> None:
    raise SystemExit(128 + signum)


def main() -> int:
    signal.signal(signal.SIGTERM, terminate)
    load_dotenv(HERE / ".env")
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    p.add_argument("--harness", default="pi", help="pi | claude")
    p.add_argument("--model", help="model pattern for the harness, else its default")
    p.add_argument(
        "--session",
        default=f"frank-{date.today():%Y%m%d}",
        help="session label; each launch starts a fresh conversation",
    )
    p.add_argument("--wait", type=int, default=90, help="seconds per inbox poll")
    p.add_argument(
        "--turn-timeout", type=int, default=300, help="kill a turn after this many seconds"
    )
    p.add_argument("--once", action="store_true", help="handle one event and exit")
    args = p.parse_args()

    if args.model is None and args.harness == "pi":
        args.model = os.environ.get("FRANK_MODEL")
    headers = {}
    if os.environ.get("FRANK_AGENT_TOKEN"):
        headers["Authorization"] = "Bearer " + os.environ["FRANK_AGENT_TOKEN"]
    url = os.environ.get("FRANK_URL", "http://127.0.0.1:7790").rstrip("/")
    response = requests.post(
        url + "/agent/session/start",
        json={"pid": os.getpid(), "session": args.session},
        headers=headers,
        timeout=150,
    )
    if not response.ok:
        try:
            detail = response.json().get("detail", response.text)
        except ValueError:
            detail = response.text
        log(f"session startup failed ({response.status_code}): {detail}")
        raise SystemExit(f"Frank could not start a fresh session: {detail}")
    sessions = HERE / "cache" / "sessions"
    sessions.mkdir(parents=True, exist_ok=True)
    args.session_file = str(sessions / f"{uuid.uuid4().hex}.jsonl")
    write_state(args)
    log(
        f"frank loop: harness={args.harness} model={args.model or 'default'} session={args.session}"
    )
    while True:
        try:
            event = wait_event(args.wait)
        except RuntimeError as e:
            log(f"inbox unavailable: {e}; retrying in 5s")
            time.sleep(5)
            continue
        if event is None:
            continue
        run_turn(args, event)
        if args.once:
            return 0


if __name__ == "__main__":
    sys.exit(main())
