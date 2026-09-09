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

"""The operator dashboard: everything the laptop needs to QA a live FRANK run.

Read-only. Every route is gated with the server's `agent_only` dependency, so the public funnel
(which adds X-Forwarded-For) gets a 403 — this is a debugging tool for the laptop, not a product.

The interesting panel is `/ops/api/context`: what the agent is *actually* being sent. That is the
harness session transcript (Pi writes JSONL, one message per line) plus the ephemeral world block,
which is regenerated every turn and injected at LLM-call time, so it never appears in the session.
"""

import asyncio
import base64
import binascii
from datetime import datetime
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from typing import Any
import uuid

from fastapi import APIRouter, HTTPException
from fastapi.responses import FileResponse, Response
import psutil
from pydantic import BaseModel

from .store import Store, selfie_path

SKILL_DIR = Path(__file__).resolve().parents[2]
CACHE = SKILL_DIR / "cache"
LOOP_STATE = CACHE / "loop_state.json"
LOOP_LOG = CACHE / "loop.log"
# loop.py writes its own turn log to loop.log; a dashboard-spawned loop is detached, so its raw
# stdout/stderr goes here instead of duplicating every line into the turn log.
LOOP_STDOUT = CACHE / "loop.stdout.log"
WORLD_TXT = CACHE / "world.txt"
STATIC = Path(__file__).resolve().parent.parent / "static" / "ops"

# A tool result can be a whole file; send a slice and let the UI ask for the rest.
PREVIEW_CHARS = 1200

LOOP_PY = SKILL_DIR / "loop.py"
ROBOT_PY = SKILL_DIR / "tools" / "robot.py"
# motion is on unless this file exists; `robot.py` refuses to move while it does
MOTION_OFF = CACHE / "MOTION_OFF"
REPO = SKILL_DIR.parents[2]
# Pi names its session folder after the working directory it was started in (loop.py uses REPO)
PI_SESSIONS = (
    Path.home() / ".pi" / "agent" / "sessions" / f"--{str(REPO).strip('/').replace('/', '-')}--"
)
# what to kill on a restart: the loop itself, and the harness turn it is waiting on
LOOP_PATTERN = r"frank/loop\.py"
HARNESS_PATTERN = r"pi -p --session-id frank-"
SIGKILL_AFTER_S = 2.0


class MotionRequest(BaseModel):
    on: bool


class LoopRequest(BaseModel):
    harness: str = "pi"
    model: str | None = None


def router(
    store: Store, world_snapshot: Any, deps: list, forget: Any, runtime: Any, reset: Any, world: Any
) -> APIRouter:
    """`world_snapshot()` returns the same dict as `GET /agent/world`; `forget(person_id)` erases
    a person the same way the phone's "forget me" does."""
    r = APIRouter(prefix="/ops", dependencies=deps)

    @r.get("")
    @r.get("/")
    def ops_index() -> FileResponse:
        return FileResponse(STATIC / "index.html")

    @r.get("/static/{name}")
    def ops_static(name: str) -> FileResponse:
        # served through the router, not a StaticFiles mount, so `agent_only` covers it too
        path = (STATIC / name).resolve()
        if path.parent != STATIC.resolve() or not path.is_file():
            raise HTTPException(404, "no such file")
        return FileResponse(path)

    @r.get("/api/people")
    def ops_people() -> dict[str, Any]:
        return {"people": store.people_overview()}

    @r.get("/api/selfie/{person_id}.jpg")
    def ops_selfie(person_id: str) -> FileResponse:
        path = selfie_path(person_id)
        if not path.exists():
            raise HTTPException(404, "no selfie on file")
        return FileResponse(path, media_type="image/jpeg")

    @r.delete("/api/people/{person_id}")
    def ops_forget(person_id: str) -> dict[str, bool]:
        """Same erase as the phone's "forget me": chats, tasks, selfie, world row."""
        if store.get_person(person_id) is None:
            raise HTTPException(404, "no such person")
        forget(person_id)
        return {"ok": True}

    @r.get("/api/world")
    def ops_world() -> dict[str, Any]:
        return world_snapshot()

    @r.get("/api/events")
    def ops_events(limit: int = 200) -> dict[str, Any]:
        names = {p["person_id"]: p["name"] for p in store.people_overview()}
        rows = store.query(
            "SELECT * FROM events ORDER BY id DESC LIMIT ?", (max(1, min(limit, 1000)),)
        )
        events = []
        for row in rows:
            payload = json.loads(row["payload"])
            pid = payload.get("person_id")
            events.append(
                {
                    "id": row["id"],
                    "type": row["type"],
                    "person_id": pid,
                    "name": names.get(pid) or payload.get("name"),
                    "ts": row["ts"],
                    # the queue keeps a flag, not a delivery time: 1 means picked up by the agent
                    # or retired as stale (older than the 10 min queue lifetime).
                    "delivered": bool(row["delivered"]),
                    "retired": bool(row["delivered"]) and row["ts"] < time.time() - 600,
                    "payload": payload,
                }
            )
        tasks = [
            {
                "task_id": t["task_id"],
                "task": t["task"],
                "person_id": t["person_id"],
                "name": names.get(t["person_id"]),
                "created_at": t["created_at"],
                "closed_at": t["closed_at"],
                "outcome": t["outcome"],
                "note": t["note"],
            }
            for t in store.query("SELECT * FROM tasks ORDER BY created_at DESC LIMIT 200")
        ]
        return {"events": events, "tasks": tasks}

    @r.get("/api/messages")
    def ops_messages(limit: int = 300) -> dict[str, Any]:
        names = {p["person_id"]: p["name"] for p in store.people_overview()}
        rows = store.query(
            "SELECT * FROM messages ORDER BY id DESC LIMIT ?", (max(1, min(limit, 2000)),)
        )
        return {
            "messages": [
                {
                    "id": m["id"],
                    "person_id": m["person_id"],
                    "name": names.get(m["person_id"]),
                    "from": m["sender"],
                    "text": m["text"],
                    "ts": m["ts"],
                }
                for m in rows
            ]
        }

    @r.get("/api/motion")
    def ops_motion() -> dict[str, bool]:
        return {"on": not MOTION_OFF.exists()}

    @r.post("/api/motion")
    def ops_set_motion(body: MotionRequest) -> dict[str, Any]:
        CACHE.mkdir(exist_ok=True)
        if body.on:
            MOTION_OFF.unlink(missing_ok=True)
            return {"ok": True, "on": True}
        return {**stop_motion(), "on": False}

    @r.post("/api/halt")
    def ops_halt() -> dict[str, Any]:
        """Latch motion off before cancelling all movement owners and direct motion."""
        return stop_motion()

    @r.get("/api/runtime")
    def ops_runtime() -> dict[str, Any]:
        roots = pgrep(LOOP_PATTERN)
        processes = []
        for pid in roots:
            try:
                process = psutil.Process(pid)
                for child in [process, *process.children(recursive=True)]:
                    args = child.cmdline()
                    role = (
                        "Agent loop"
                        if child.pid == pid
                        else "Pi + inbox listener"
                        if any(arg.endswith("pi_events.ts") for arg in args)
                        else "Tool / inbox wait"
                    )
                    processes.append(
                        {"pid": child.pid, "ppid": child.ppid(), "name": child.name(), "role": role}
                    )
            except psutil.Error:
                pass
        watchers = []
        for process in psutil.process_iter(["pid", "cmdline"]):
            if any(arg.endswith("tools/watch.py") for arg in process.info["cmdline"] or []):
                watchers.append(process.pid)
        names = {p["person_id"]: p["name"] for p in store.people_overview()}
        return {
            **runtime.snapshot(),
            "processes": processes,
            "watches": [
                {
                    "id": pid,
                    "name": names.get(pid, pid),
                    "expires_ts": expires,
                    "say": runtime.watch_responses.get(pid),
                }
                for pid, expires in world.watches().items()
            ],
            "face_watcher_pids": watchers,
            "pi_inbox_listener": {
                "active": any(p["role"] == "Pi + inbox listener" for p in processes),
                "loop_running": bool(roots),
            },
        }

    @r.post("/api/runtime/stop")
    async def ops_runtime_stop(body: dict[str, str]) -> dict[str, Any]:
        kind = body.get("kind")
        if kind == "watch":
            runtime.watch_responses.pop(body.get("id", ""), None)
            removed = world.unwatch(body.get("id", ""))
            runtime.record("watch_removed", person_id=body.get("id"))
            return {"ok": True, "removed": removed}
        if kind == "operation":
            token = body.get("id", "")
            op = runtime.operations.get(token)
            if op is None:
                return {"ok": True, "output": "Operation already finished."}
            try:
                result = await runtime.cancel_operation(token)
                return {"ok": True, "result": result}
            except RuntimeError as exc:
                return {"ok": False, "output": str(exc)}

        if kind == "mcp":
            await runtime.stop()
        else:
            processes = [
                p
                for p in psutil.process_iter(["cmdline"])
                if any(arg.endswith("tools/watch.py") for arg in p.info["cmdline"] or [])
            ]
            for process in processes:
                try:
                    process.terminate()
                except psutil.Error:
                    pass
            _, remaining = await asyncio.to_thread(psutil.wait_procs, processes, timeout=3)
            for process in remaining:
                try:
                    process.kill()
                except psutil.Error:
                    pass
            runtime.record("face_watcher_stopped", pids=[p.pid for p in processes])
        return {"ok": True}

    @r.get("/api/loop")
    def ops_loop() -> dict[str, Any]:
        state = read_state()
        pids = pgrep(LOOP_PATTERN)
        return {
            "running": bool(pids),
            "pid": pids[0] if pids else None,
            "session": state.get("session"),
            "harness": state.get("harness"),
            "model": state.get("model"),
            "started_ts": state.get("started_ts"),
        }

    @r.post("/api/loop/stop")
    async def ops_loop_stop() -> dict[str, Any]:
        killed = await asyncio.to_thread(stop_loop)
        stopped = await asyncio.to_thread(stop_motion)
        await runtime.stop()
        runtime.record("frank_off", killed=killed, stopped=stopped["ok"])
        return {**stopped, "killed": killed}

    @r.post("/api/loop/restart")
    async def ops_loop_restart(body: LoopRequest | None = None) -> dict[str, Any]:
        """Kill the loop and its in-flight harness turn, then start a fresh one on a new session,
        so edited skills, persona and scripts are re-read and no earlier turns are carried over."""
        req = body or LoopRequest()
        killed = await asyncio.to_thread(stop_loop)
        stopped = await asyncio.to_thread(stop_motion)
        if not stopped["ok"]:
            return {**stopped, "killed": killed}
        await reset(clear_history=True)
        LOOP_STATE.write_text("{}")
        WORLD_TXT.write_text("")
        session = f"frank-{datetime.now():%Y%m%d-%H%M%S}-{uuid.uuid4().hex[:8]}"
        cmd = [sys.executable, "-u", str(LOOP_PY), "--harness", req.harness, "--session", session]
        if req.model:
            cmd += ["--model", req.model]
        LOOP_STDOUT.parent.mkdir(exist_ok=True)
        with LOOP_STDOUT.open("a") as log, open(os.devnull) as devnull:
            proc = subprocess.Popen(
                cmd,
                cwd=REPO,
                env={**os.environ, "DIMOS_TRANSPORT": "lcm"},
                stdin=devnull,
                stdout=log,
                stderr=subprocess.STDOUT,
                start_new_session=True,  # detached: it outlives this request and this server
            )
        runtime.record("frank_restarted", session=session, pid=proc.pid, killed=killed)
        return {
            "ok": True,
            "session": session,
            "pid": proc.pid,
            "killed": killed,
            "motion_on": False,
        }

    @r.get("/api/image/{index}")
    def ops_image(index: int) -> Response:
        """The nth image in the current session, streamed on demand (browsers cache it)."""
        path = resolve_session_file(read_state())
        if path is None:
            raise HTTPException(404, "no session file")
        found = session_images(path)
        if index >= len(found):
            raise HTTPException(404, "no such image")
        mime, data = found[index]
        try:
            raw = base64.b64decode(data)
        except (ValueError, binascii.Error):
            raise HTTPException(500, "unreadable image data")
        return Response(raw, media_type=mime, headers={"Cache-Control": "no-store"})

    @r.get("/api/context")
    def ops_context() -> dict[str, Any]:
        state = read_state()
        running = bool(pgrep(LOOP_PATTERN))
        path = resolve_session_file(state) if running else None
        return {
            "loop": state,
            "session_file": str(path) if path else None,
            "messages": read_session(path) if path else [],
            "world_block": read_text(WORLD_TXT) if running else "",
            "world_block_note": "injected at call time, not stored in the session",
            "log": tail(LOOP_LOG, 50),
            # only what the turn log does not already have: Pi's stderr, tracebacks
            "stderr": extra_lines(tail(LOOP_STDOUT, 200), read_text(LOOP_LOG))[-50:],
        }

    return r


# --- the loop -----------------------------------------------------------


def pgrep(pattern: str) -> list[int]:
    """Find Frank processes by argument boundaries, not substrings of a shell command."""
    found = []
    for process in psutil.process_iter(["pid", "cmdline"]):
        args = process.info["cmdline"] or []
        if process.pid in {os.getpid(), os.getppid()}:
            continue
        if pattern == LOOP_PATTERN:
            matches = any(arg.endswith("frank/loop.py") for arg in args)
        else:
            matches = "--session-id" in args and any(arg.startswith("frank-") for arg in args)
        if matches:
            found.append(process.pid)
    return found


def stop_motion() -> dict[str, Any]:
    """Fail closed: keep motion disabled even if stopping cannot be confirmed."""
    CACHE.mkdir(exist_ok=True)
    MOTION_OFF.touch()
    try:
        result = subprocess.run(
            [sys.executable, str(ROBOT_PY), "stop"],
            cwd=REPO,
            capture_output=True,
            text=True,
            timeout=130,
        )
        return {
            "ok": result.returncode == 0,
            "exit_code": result.returncode,
            "output": (result.stdout + result.stderr).strip(),
            "motion_on": False,
        }
    except (subprocess.TimeoutExpired, OSError) as exc:
        return {"ok": False, "output": str(exc), "motion_on": False}


def stop_loop(exclude_pid: int | None = None) -> list[int]:
    """SIGTERM the loop and the harness turn it spawned, SIGKILL whatever is left after 2 s."""
    CACHE.mkdir(exist_ok=True)
    MOTION_OFF.touch()
    pids = [pid for pid in pgrep(LOOP_PATTERN) + pgrep(HARNESS_PATTERN) if pid != exclude_pid]
    descendants = []
    for pid in pids:
        try:
            descendants.extend(child.pid for child in psutil.Process(pid).children(recursive=True))
        except psutil.Error:
            pass
    pids = list(dict.fromkeys([*descendants, *pids]))
    for pid in pids:
        try:
            os.kill(pid, signal.SIGTERM)
        except OSError:
            pass
    deadline = time.time() + SIGKILL_AFTER_S
    while time.time() < deadline and any(alive(p) for p in pids):
        time.sleep(0.1)
    for pid in pids:
        if alive(pid):
            try:
                os.kill(pid, signal.SIGKILL)
            except OSError:
                pass
    return pids


def alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
    except OSError:
        return False
    return True


# --- the harness transcript ---------------------------------------------


def read_state() -> dict[str, Any]:
    try:
        return json.loads(LOOP_STATE.read_text())
    except Exception:
        return {}


def resolve_session_file(state: dict[str, Any]) -> Path | None:
    """Only the exact file assigned to the current process; never search older sessions."""
    recorded = state.get("session_file")
    if not recorded:
        return None
    path = Path(recorded)
    return path if path.is_file() else None


def session_images(path: Path) -> list[tuple[str, str]]:
    """Every image in the session, in order: `(mime, base64)`, passed through untouched."""
    out: list[tuple[str, str]] = []
    try:
        lines = path.read_text().splitlines()
    except OSError:
        return out
    for line in lines:
        try:
            msg = (json.loads(line) or {}).get("message") or {}
        except json.JSONDecodeError:
            continue
        for block in as_blocks(msg.get("content")):
            if block.get("type") == "image":
                out.append(image_data(block))
    return out


def read_session(path: Path) -> list[dict[str, Any]]:
    """Flatten a Pi session JSONL into a list the dashboard can render in order."""
    out: list[dict[str, Any]] = []
    try:
        lines = path.read_text().splitlines()
    except OSError:
        return out
    for line in lines:
        line = line.strip()
        if not line:
            continue
        try:
            entry = json.loads(line)
        except json.JSONDecodeError:
            continue
        kind = entry.get("type")
        ts = entry.get("timestamp")
        if kind == "session":
            out.append(
                {
                    "role": "session",
                    "ts": ts,
                    "text": f"session {entry.get('id')} in {entry.get('cwd')}",
                }
            )
        elif kind == "model_change":
            out.append(
                {
                    "role": "meta",
                    "ts": ts,
                    "text": f"model → {entry.get('provider')}/{entry.get('modelId')}",
                }
            )
        elif kind == "thinking_level_change":
            out.append(
                {"role": "meta", "ts": ts, "text": f"thinking → {entry.get('thinkingLevel')}"}
            )
        elif kind == "compaction":
            out.append(
                {"role": "meta", "ts": ts, "text": "compaction: " + str(entry.get("summary"))}
            )
        elif kind == "message":
            out.extend(flatten_message(entry.get("message") or {}, ts))
    n = 0
    for row in out:
        if row.get("role") == "image":
            row["index"] = n
            row["src"] = f"/ops/api/image/{n}?session={path.stem}"
            n += 1
    return out


def flatten_message(msg: dict[str, Any], ts: Any) -> list[dict[str, Any]]:
    """One session message → one row per thing worth seeing (text, thinking, tool call, result)."""
    role = msg.get("role")
    rows: list[dict[str, Any]] = []
    if role in ("user", "assistant"):
        for block in as_blocks(msg.get("content")):
            btype = block.get("type")
            if btype == "text":
                rows.append({"role": role, "ts": ts, "text": block.get("text", "")})
            elif btype == "thinking":
                rows.append({"role": "thinking", "ts": ts, "text": block.get("thinking", "")})
            elif btype == "toolCall":
                rows.append(
                    {
                        "role": "toolCall",
                        "ts": ts,
                        "tool": block.get("name"),
                        "text": json.dumps(block.get("arguments", {}), indent=2),
                    }
                )
            elif btype == "image":
                rows.append(image_row(block, ts))
        if role == "assistant" and msg.get("model"):
            for row in rows:
                row.setdefault("model", msg["model"])
    elif role == "toolResult":
        # a result can be text plus several images (check_leg returns a camera frame and a map
        # crop); keep them in the order the agent saw them
        head = {
            "role": "toolResult",
            "ts": ts,
            "tool": msg.get("toolName"),
            "is_error": bool(msg.get("isError")),
        }
        text: list[str] = []
        for block in as_blocks(msg.get("content")):
            if block.get("type") == "image":
                if text:
                    rows.append({**head, "text": "\n".join(text)})
                    text = []
                rows.append({**image_row(block, ts), "tool": msg.get("toolName")})
            else:
                text.append(block.get("text", ""))
        if text or not rows:
            rows.append({**head, "text": "\n".join(text)})
    elif role == "bashExecution":
        rows.append(
            {
                "role": "bash",
                "ts": ts,
                "tool": msg.get("command"),
                "text": msg.get("output", ""),
            }
        )
    elif role:
        rows.append(
            {"role": role, "ts": ts, "text": str(msg.get("content") or msg.get("summary") or "")}
        )
    return [truncate(r) for r in rows]


def image_data(block: dict[str, Any]) -> tuple[str, str]:
    """`(mime, base64)` from either shape: flat `data`/`mimeType`, or a nested `source`."""
    src = block.get("source") if isinstance(block.get("source"), dict) else {}
    mime = block.get("mimeType") or src.get("media_type") or "image/jpeg"
    return str(mime), str(block.get("data") or src.get("data") or "")


def image_row(block: dict[str, Any], ts: Any) -> dict[str, Any]:
    """Metadata only. The bytes are fetched per image, so the context JSON stays small even when a
    session holds dozens of 240 KB frames."""
    mime, data = image_data(block)
    return {"role": "image", "ts": ts, "mime": mime, "bytes": len(data) * 3 // 4, "text": ""}


def as_blocks(content: Any) -> list[dict[str, Any]]:
    if isinstance(content, str):
        return [{"type": "text", "text": content}]
    if isinstance(content, list):
        return [b for b in content if isinstance(b, dict)]
    return []


def truncate(row: dict[str, Any]) -> dict[str, Any]:
    text = row.get("text") or ""
    if len(text) <= PREVIEW_CHARS:
        return row
    return {**row, "text": text[:PREVIEW_CHARS], "full": text, "chars": len(text)}


def extra_lines(lines: list[str], already: str) -> list[str]:
    """Lines the turn log does not carry. loop.py prints and logs the same text, so the spawned
    process's stdout is mostly a duplicate; what is left is worth showing."""
    seen = set(already.splitlines())
    return [line for line in lines if line.strip() and line not in seen]


def read_text(path: Path) -> str:
    try:
        return path.read_text()
    except OSError:
        return ""


def tail(path: Path, lines: int) -> list[str]:
    return read_text(path).splitlines()[-lines:]
