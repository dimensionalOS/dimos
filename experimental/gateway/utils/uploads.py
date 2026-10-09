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

"""Uploads to Dimensional cloud and the cloud login (`/dimos/uploads*`, `/dimos/cloud/*`).

The work is dimos's own code (dimos.cloud.data.CloudData, dimos.cli.cloud) run by cloud_worker.py in a child process,
one per upload, so cancelling is killing it. This file is the queue (first in, first out, one at a time, kept in
uploads.json so it survives a restart), progress turned into a rate and a time left, readable errors (tracebacks go to
uploads.log), what is already in the cloud (uploaded.json) and the device login's state.
"""

from __future__ import annotations

import asyncio
from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import signal
import sys
import time
from typing import Any

from experimental.gateway.utils import config
from experimental.gateway.utils.events import Bus
from experimental.gateway.utils.introspect import MARKER

FINISHED = ("done", "failed", "cancelled")
# seconds of ticks before a time left is given: the first ones include connecting and are far off
WARMUP = 1.0
# the speed is an exponential moving average over about this many seconds
TAU = 6.0


def now_ms() -> int:
    return int(time.time() * 1000)


class Rate:
    """Bytes per second, smoothed, from progress ticks."""

    def __init__(self) -> None:
        self.last: tuple[float, int] | None = None
        self.speed: float | None = None
        self.since: float | None = None

    def tick(self, t: float, done: int) -> None:
        """A tick at `t` seconds with `done` bytes; the first is only the baseline (a resumed upload's earlier parts
        aren't speed), and going backwards (a restarted part) starts over."""
        if self.last is None or done < self.last[1]:
            self.last, self.speed, self.since = (t, done), None, t
        elif t > self.last[0]:
            dt = t - self.last[0]
            instant = (done - self.last[1]) / dt
            alpha = 1.0 - math.exp(-dt / TAU)
            self.speed = (
                instant if self.speed is None else self.speed + alpha * (instant - self.speed)
            )
            self.last = (t, done)

    def bps(self) -> float | None:
        return self.speed if self.speed and self.speed > 0 else None

    def eta(self, remaining: int, t: float) -> float | None:
        """Seconds left for `remaining` bytes at time `t` (time since the last tick counts as spent)."""
        bps = self.bps()
        if bps is None or self.last is None:
            return None
        if self.since is not None and self.last[0] - self.since < WARMUP:
            return None
        since = max(t - self.last[0], 0.0)
        return max(remaining / bps - since, 1.0 if remaining > 0 else 0.0)


def check_path(text: str) -> tuple[Path, int]:
    """`text` as something to upload: an existing dimos recording (.mcap, or a memory2 .db), not a SQLite sidecar."""
    path = config.expand(text.strip())
    if not path.is_absolute():
        raise ValueError(f"path must be absolute: {path}")
    if not path.exists():
        raise ValueError(f"no such file: {path}")
    if not path.is_file():
        raise ValueError(f"not a file: {path}")
    from dimos.cloud.data import kind_of

    # what dimos's cloud upload calls a recording (an .mcap, or a .db with dimos's streams in it)
    if kind_of(path) != "recording":
        raise ValueError(f"not a dimos recording (.mcap, or a .db dimos recorded): {path.name}")
    return path, path.stat().st_size


def _size_and_mtime(path: str) -> tuple[int, int] | None:
    try:
        stat = os.stat(path)
    except OSError:
        return None
    return stat.st_size, int(stat.st_mtime * 1000)


def uploaded_entry(upload: dict[str, Any]) -> dict[str, Any] | None:
    """What uploaded.json keeps of a done upload (None for one that isn't, or whose file is gone)."""
    found = _size_and_mtime(upload["path"])
    if upload["state"] != "done" or found is None or not upload.get("uploadId"):
        return None
    return {
        "path": upload["path"],
        "uploadId": upload["uploadId"],
        "size": found[0],
        "mtimeMs": found[1],
        "uploadedAt": upload.get("finishedAt") or now_ms(),
        "link": upload.get("link"),
        "changed": False,
    }


def checked(entry: dict[str, Any]) -> dict[str, Any]:
    """With `changed`: the file's size or modification time differs from when it was uploaded."""
    return {**entry, "changed": _size_and_mtime(entry["path"]) != (entry["size"], entry["mtimeMs"])}


class Queue:
    """The queue's state, with no processes: every transition is here (and tested)."""

    def __init__(self, items: list[dict[str, Any]] | None = None) -> None:
        self.items: list[dict[str, Any]] = items or []
        self.waiting_for_login = False
        self.rates: dict[str, Rate] = {}
        # one that was uploading when the gateway stopped is queued again (dimos resumes it)
        for item in self.items:
            if item["state"] == "uploading":
                item.update(state="queued", phase=None, rateBps=None, etaSeconds=None)
        numbers = [int(u["id"][1:]) for u in self.items if u["id"][1:].isdigit()]
        self.next = max(numbers, default=0)

    def get(self, id: str) -> dict[str, Any] | None:
        return next((u for u in self.items if u["id"] == id), None)

    def enqueue(
        self, path: Path, size: int, robot_id: str | None, kind: str | None
    ) -> tuple[dict[str, Any], bool]:
        """Adds one, or returns the one already queued or uploading for that path: (upload, newly added)."""
        text = str(path)
        for upload in self.items:
            if upload["path"] == text and upload["state"] in ("queued", "uploading"):
                return dict(upload), False
        self.next += 1
        upload = {
            "id": f"u{self.next}",
            "path": text,
            "name": path.name or text,
            "size": size,
            "robotId": robot_id or None,
            "kind": kind or None,
            "state": "queued",
            "phase": None,
            "bytesDone": 0,
            "bytesTotal": size,
            "rateBps": None,
            "etaSeconds": None,
            "uploadId": None,
            "skipped": False,
            "notice": None,
            "error": None,
            "errorCode": None,
            "log": None,
            "createdAt": now_ms(),
            "startedAt": None,
            "finishedAt": None,
            "link": None,
        }
        self.items.append(upload)
        return dict(upload), True

    def next_queued(self) -> str | None:
        """The next one to upload; none while waiting for a login or while one is uploading."""
        if self.waiting_for_login or any(u["state"] == "uploading" for u in self.items):
            return None
        return next((u["id"] for u in self.items if u["state"] == "queued"), None)

    def start(self, id: str) -> dict[str, Any] | None:
        upload = self.get(id)
        if upload is None:
            return None
        self.rates[id] = Rate()
        upload.update(
            state="uploading",
            phase="preparing",
            bytesDone=0,
            bytesTotal=upload["size"],
            rateBps=None,
            etaSeconds=None,
            error=None,
            errorCode=None,
            startedAt=now_ms(),
            finishedAt=None,
        )
        return dict(upload)

    def progress(
        self, id: str, phase: str, done: int, total: int, t: float
    ) -> dict[str, Any] | None:
        """A tick at `t` seconds (any clock). Each phase has its own bytes, speed and time left."""
        upload = self.get(id)
        if upload is None or upload["state"] != "uploading":
            return None
        if upload["phase"] != phase:
            self.rates[id] = Rate()
        upload["phase"] = phase
        if total > 0:
            rate = self.rates.setdefault(id, Rate())
            rate.tick(t, done)
            upload.update(
                bytesDone=done,
                bytesTotal=total,
                rateBps=rate.bps(),
                etaSeconds=rate.eta(max(total - done, 0), t),
            )
            if phase == "upload" and done >= total:
                # the cloud puts the parts together
                upload.update(phase="finishing", etaSeconds=None)
        else:
            # a phase with no bytes: an indeterminate bar
            upload.update(bytesDone=0, bytesTotal=0, rateBps=None, etaSeconds=None)
        return dict(upload)

    def succeed(self, id: str, result: dict[str, Any]) -> dict[str, Any] | None:
        upload = self.get(id)
        if upload is None:
            return None
        quota = result.get("quota") or {}
        state = quota.get("state") if isinstance(quota, dict) else None
        notice = None
        if state not in (None, "ok"):
            notice = quota.get("message") or f"quota: {state}"
        upload.update(
            state="done",
            phase=None,
            uploadId=result.get("uploadId"),
            skipped=bool(result.get("skipped")),
            link=result.get("link"),
            bytesDone=upload["bytesTotal"],
            rateBps=None,
            etaSeconds=None,
            notice=notice,
            finishedAt=now_ms(),
        )
        return dict(upload)

    def fail(self, id: str, code: str, message: str, log: Path | None) -> dict[str, Any] | None:
        upload = self.get(id)
        if upload is None:
            return None
        upload.update(
            state="failed",
            phase=None,
            rateBps=None,
            etaSeconds=None,
            error=message,
            errorCode=code,
            log=str(log) if log else None,
            finishedAt=now_ms(),
        )
        return dict(upload)

    def needs_login(self, id: str, message: str) -> dict[str, Any] | None:
        """No login: it goes back to the front of the queue, which waits for one."""
        self.waiting_for_login = True
        upload = self.get(id)
        if upload is None:
            return None
        upload.update(
            state="queued",
            phase=None,
            rateBps=None,
            etaSeconds=None,
            startedAt=None,
            error=message,
            errorCode="not_logged_in",
        )
        return dict(upload)

    def logged_in(self) -> bool:
        """Logged in: the queue goes on. True when it was waiting."""
        was = self.waiting_for_login
        self.waiting_for_login = False
        for upload in self.items:
            if upload["state"] == "queued" and upload["errorCode"] == "not_logged_in":
                upload.update(error=None, errorCode=None)
        return was

    def cancel(self, id: str) -> str | None:
        """ "running" (kill its worker), "cancelled" (it was queued), "removed" (it was finished) or None."""
        upload = self.get(id)
        if upload is None:
            return None
        if upload["state"] == "uploading":
            return "running"
        if upload["state"] == "queued":
            upload.update(state="cancelled", error=None, errorCode=None, finishedAt=now_ms())
            return "cancelled"
        self.items.remove(upload)
        return "removed"

    def cancelled(self, id: str) -> dict[str, Any] | None:
        """The running one's worker was killed on purpose."""
        upload = self.get(id)
        if upload is None:
            return None
        upload.update(
            state="cancelled", phase=None, rateBps=None, etaSeconds=None, finishedAt=now_ms()
        )
        return dict(upload)

    def retry(self, id: str) -> dict[str, Any]:
        """A finished one goes to the back of the queue again (KeyError: no such upload; ValueError: not finished)."""
        upload = self.get(id)
        if upload is None:
            raise KeyError(f"no upload {id}")
        if upload["state"] not in FINISHED:
            raise ValueError(f'{upload["name"]} is already "{upload["state"]}"')
        self.items.remove(upload)
        upload.update(
            state="queued",
            phase=None,
            bytesDone=0,
            bytesTotal=upload["size"],
            rateBps=None,
            etaSeconds=None,
            error=None,
            errorCode=None,
            notice=None,
            skipped=False,
            link=None,
            startedAt=None,
            finishedAt=None,
            createdAt=now_ms(),
        )
        self.waiting_for_login = False
        self.items.append(upload)
        return dict(upload)

    def clear_finished(self) -> None:
        self.items = [u for u in self.items if u["state"] not in FINISHED]


def idle_login() -> dict[str, Any]:
    """state: idle | starting | pending | approved | denied | expired | failed"""
    return {
        "state": "idle",
        "url": None,
        "urlComplete": None,
        "code": None,
        "expiresAt": None,
        "email": None,
        "error": None,
    }


@dataclass
class Running:
    id: str
    pid: int
    cancelled: bool = False


def _kill_group(pid: int) -> None:
    try:
        os.killpg(pid, signal.SIGTERM)
    except (ProcessLookupError, PermissionError):
        pass


def _marked(line: bytes) -> dict[str, Any] | None:
    text = line.decode("utf-8", "replace").strip()
    if not text.startswith(MARKER):
        return None
    try:
        value = json.loads(text[len(MARKER) :])
    except ValueError:
        return None
    return value if isinstance(value, dict) else None


class Uploads:
    def __init__(
        self,
        dimos_dir: Path,
        bus: Bus,
        file: Path | None,
        log: Path,
        worker: list[str] | None = None,
    ) -> None:
        """`file`: where the queue is kept (uploaded.json goes beside it; None in tests). `worker`: the command that
        runs cloud_worker (a fake in tests)."""
        self.dimos_dir = dimos_dir
        self.bus = bus
        self.file = file
        self.log = log
        self.worker = worker or [sys.executable, "-m", "experimental.gateway.utils.cloud_worker"]
        saved: list[dict[str, Any]] = self._read(file) or []
        self.uploaded_by_path: dict[str, dict[str, Any]] = (
            self._read(file.with_name("uploaded.json")) if file else None
        ) or {}
        for upload in saved:
            done = uploaded_entry(upload)
            if done:
                self.uploaded_by_path.setdefault(done["path"], done)
        self.queue = Queue(saved)
        self.wake = asyncio.Event()
        self.running: Running | None = None
        self.login: dict[str, Any] = idle_login()
        self.login_pid: int | None = None
        self.login_task: asyncio.Task[None] | None = None
        self.account_cache: tuple[float, dict[str, Any]] | None = None
        self.stopping = False

    @staticmethod
    def _read(file: Path | None) -> Any:
        try:
            return json.loads(file.read_text()) if file else None
        except (OSError, ValueError):
            return None

    def listing(self) -> dict[str, Any]:
        return {"uploads": self.queue.items, "waitingForLogin": self.queue.waiting_for_login}

    def save(self) -> None:
        if self.file:
            config.write_atomic(self.file, json.dumps(self.queue.items, indent=2))

    def uploaded(self) -> dict[str, Any]:
        return {
            "byPath": {
                path: checked(entry) for path, entry in sorted(self.uploaded_by_path.items())
            }
        }

    def uploaded_one(self, path: str) -> dict[str, Any] | None:
        entry = self.uploaded_by_path.get(str(config.expand(path.strip())))
        return checked(entry) if entry else None

    def remember(self, upload: dict[str, Any]) -> None:
        done = uploaded_entry(upload)
        if done is None:
            return
        self.uploaded_by_path[done["path"]] = done
        if self.file:
            config.write_atomic(
                self.file.with_name("uploaded.json"),
                json.dumps(dict(sorted(self.uploaded_by_path.items())), indent=2),
            )

    def emit(self, upload: dict[str, Any] | None) -> None:
        if upload is not None:
            self.bus.send({"type": "upload", "upload": upload})

    def emit_waiting(self, **extra: Any) -> None:
        self.bus.send({"type": "uploads", "waitingForLogin": self.queue.waiting_for_login, **extra})

    def enqueue(self, path: str, robot_id: str | None, kind: str | None) -> dict[str, Any]:
        """ValueError: not something to upload."""
        checked_path, size = check_path(path)
        upload, added = self.queue.enqueue(checked_path, size, robot_id, kind)
        if added:
            self.save()
            self.emit(upload)
            self.wake.set()
        return upload

    def cancel(self, id: str) -> None:
        """KeyError: no such upload."""
        outcome = self.queue.cancel(id)
        if outcome is None:
            raise KeyError(f"no upload {id}")
        if outcome == "running":
            if self.running and self.running.id == id:
                self.running.cancelled = True
                _kill_group(self.running.pid)
        elif outcome == "cancelled":
            self.emit(self.queue.get(id))
        else:
            self.bus.send({"type": "upload-removed", "id": id})
        self.save()

    def retry(self, id: str) -> dict[str, Any]:
        upload = self.queue.retry(id)
        self.save()
        self.emit(upload)
        self.emit_waiting()
        self.wake.set()
        return upload

    def clear_finished(self) -> dict[str, Any]:
        self.queue.clear_finished()
        self.save()
        self.emit_waiting(cleared=True)
        return self.listing()

    async def spawn(self, args: list[str], header: str) -> asyncio.subprocess.Process:
        """The worker with `args`, in its own process group; its stderr goes to uploads.log under a `header` line."""
        self.log.parent.mkdir(parents=True, exist_ok=True)
        with self.log.open("a") as log:
            log.write(f"\n=== {now_ms()} {header}\n")
            log.flush()
            return await asyncio.create_subprocess_exec(
                *self.worker,
                *args,
                cwd=self.dimos_dir,
                stdin=asyncio.subprocess.DEVNULL,
                stdout=asyncio.subprocess.PIPE,
                stderr=log,
                start_new_session=True,
            )

    async def run_once(self, args: list[str], timeout: float) -> dict[str, Any]:
        """Runs the worker to its end; its last result (an error result raises RuntimeError)."""
        child = await self.spawn(args, " ".join(args))
        assert child.stdout is not None
        last: dict[str, Any] | None = None

        async def read() -> None:
            nonlocal last
            assert child.stdout is not None
            async for line in child.stdout:
                last = _marked(line) or last

        try:
            await asyncio.wait_for(read(), timeout)
        except asyncio.TimeoutError:
            _kill_group(child.pid)
            raise RuntimeError("Dimensional cloud took too long to answer")
        await child.wait()
        if last is None:
            raise RuntimeError(f"the cloud worker gave no answer (details: {self.log})")
        if last.get("event") == "error":
            raise RuntimeError(last.get("message") or "failed")
        return last

    async def account(self, fresh: bool) -> dict[str, Any]:
        """Who is logged in (cached for 20 s; a login or logout clears it)."""
        if not fresh and self.account_cache and time.monotonic() - self.account_cache[0] < 20:
            return self.account_cache[1]
        value = await self.run_once(["account"], 90)
        self.account_cache = (time.monotonic(), value)
        if value.get("loggedIn") is True and self.queue.logged_in():
            self.emit_waiting()
            self.wake.set()
        return value

    async def logout(self) -> dict[str, Any]:
        await self.run_once(["logout"], 60)
        self.login = idle_login()
        return await self.account(True)

    def login_state(self) -> dict[str, Any]:
        if self.login["state"] == "pending" and (self.login["expiresAt"] or math.inf) < now_ms():
            self.login["state"] = "expired"
        return dict(self.login)

    def set_login(self, **change: Any) -> None:
        self.login.update(change)
        self.bus.send({"type": "cloud-login", "login": dict(self.login)})

    async def start_login(self) -> dict[str, Any]:
        """Starts the device login (or returns the one waiting for approval) and answers once the code is known."""
        current = self.login_state()
        if current["state"] in ("starting", "pending"):
            return current
        self.cancel_login()
        child = await self.spawn(["login"], "login")
        self.login_pid = child.pid
        self.set_login(**{**idle_login(), "state": "starting"})
        self.login_task = asyncio.create_task(self._follow_login(child))
        # the code comes back in a second or two (dimos imports, then one request)
        for _ in range(300):
            if self.login_state()["state"] != "starting":
                break
            await asyncio.sleep(0.1)
        return self.login_state()

    async def _follow_login(self, child: asyncio.subprocess.Process) -> None:
        assert child.stdout is not None
        ended = False
        async for line in child.stdout:
            value = _marked(line)
            if value is None:
                continue
            event = value.get("event")
            if event == "code":
                expires_in = value.get("expiresIn")
                self.set_login(
                    state="pending",
                    url=value.get("url"),
                    urlComplete=value.get("urlComplete"),
                    code=value.get("code"),
                    expiresAt=now_ms() + int(expires_in) * 1000 if expires_in is not None else None,
                )
            elif event == "done":
                ended = True
                status = value.get("status") or "failed"
                self.set_login(
                    state="approved" if status == "ok" else status, email=value.get("email")
                )
                if status == "ok":
                    self.account_cache = None
                    if self.queue.logged_in():
                        self.emit_waiting()
                    self.wake.set()
            elif event == "error":
                ended = True
                self.set_login(state="failed", error=value.get("message"))
        await child.wait()
        if self.login_pid == child.pid:
            self.login_pid = None
            if not ended and self.login_state()["state"] in ("starting", "pending"):
                self.set_login(
                    state="failed", error=f"the login worker stopped (details: {self.log})"
                )

    def cancel_login(self) -> dict[str, Any]:
        if self.login_pid is not None:
            _kill_group(self.login_pid)
            self.login_pid = None
        if self.login_state()["state"] in ("starting", "pending"):
            self.set_login(**idle_login())
        return self.login_state()

    def shutdown(self) -> None:
        """The gateway is stopping: kill the workers; the queue keeps the running one, which resumes next time."""
        self.stopping = True
        self.save()
        if self.running:
            _kill_group(self.running.pid)
        if self.login_pid is not None:
            _kill_group(self.login_pid)

    async def work(self) -> None:
        """Uploads the queue, one at a time, for as long as the gateway runs."""
        while not self.stopping:
            next_id = self.queue.next_queued()
            if next_id:
                await self.upload(next_id)
            else:
                await self.wake.wait()
                self.wake.clear()

    async def upload(self, id: str) -> None:
        upload = self.queue.start(id)
        if upload is None:
            return
        self.save()
        self.emit(upload)
        args = ["upload", upload["path"], upload["robotId"] or "", upload["kind"] or ""]
        try:
            child = await self.spawn(args, f"upload {id} {upload['path']}")
        except Exception as error:
            self.finish(
                self.queue.fail(id, "failed", f"couldn't start the cloud worker: {error}", None)
            )
            return
        assert child.stdout is not None
        self.running = Running(id, child.pid)
        started = time.monotonic()
        last_emit = -1.0
        last_phase = ""
        outcome: dict[str, Any] | None = None
        async for line in child.stdout:
            value = _marked(line)
            if value is None:
                continue
            if value.get("event") == "progress":
                phase = value.get("phase") or "upload"
                updated = self.queue.progress(
                    id,
                    phase,
                    int(value.get("done") or 0),
                    int(value.get("total") or 0),
                    time.monotonic() - started,
                )
                # a few events a second at most, and every phase change
                if phase != last_phase or time.monotonic() - last_emit >= 0.4:
                    last_phase, last_emit = phase, time.monotonic()
                    self.emit(updated)
            elif value.get("event") in ("result", "error"):
                outcome = value
        status = await child.wait()
        running, self.running = self.running, None
        if self.stopping:
            return
        if running and running.cancelled:
            updated = self.queue.cancelled(id)
        elif outcome and outcome["event"] == "result":
            updated = self.queue.succeed(id, outcome)
        elif outcome:
            code = outcome.get("code") or "failed"
            message = outcome.get("message") or "the upload failed"
            if code == "not_logged_in":
                updated = self.queue.needs_login(id, message)
                self.account_cache = None
                self.emit_waiting()
            else:
                updated = self.queue.fail(id, code, message, self.log)
        else:
            updated = self.queue.fail(
                id, "failed", f"the cloud worker stopped (exit {status}) without a result", self.log
            )
        self.finish(updated)

    def finish(self, upload: dict[str, Any] | None) -> None:
        if upload is not None:
            self.remember(upload)
        self.save()
        self.emit(upload)
