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

from __future__ import annotations

import asyncio
from collections.abc import Callable, Iterator
import json
import math
import os
from pathlib import Path
import socket
import threading
import time
from typing import Any
import urllib.error

import requests

from dimos.utils.logging_config import setup_logger
from experimental.gateway import store
from experimental.gateway.events import Bus

logger = setup_logger()

FINISHED = ("done", "failed", "cancelled")
WARMUP_S = 1.0
TAU_S = 6.0
ACCOUNT_TTL_S = 20.0


def now_ms() -> int:
    return int(time.time() * 1000)


class CancelledUploadError(Exception):
    pass


class Rate:
    def __init__(self) -> None:
        self.last: tuple[float, int] | None = None
        self.speed: float | None = None
        self.since: float | None = None

    def tick(self, t: float, done: int) -> None:
        if self.last is None or done < self.last[1]:
            self.last, self.speed, self.since = (t, done), None, t
        elif t > self.last[0]:
            dt = t - self.last[0]
            instant = (done - self.last[1]) / dt
            alpha = 1.0 - math.exp(-dt / TAU_S)
            self.speed = (
                instant if self.speed is None else self.speed + alpha * (instant - self.speed)
            )
            self.last = (t, done)

    def bps(self) -> float | None:
        return self.speed if self.speed and self.speed > 0 else None

    def eta(self, remaining: int, t: float) -> float | None:
        bps = self.bps()
        if bps is None or self.last is None or self.since is None:
            return None
        if self.last[0] - self.since < WARMUP_S:
            return None
        return max(remaining / bps - max(t - self.last[0], 0.0), 1.0 if remaining > 0 else 0.0)


def check_path(text: str) -> tuple[Path, int]:
    from dimos.cloud.data import kind_of

    path = Path(text.strip()).expanduser()
    if not path.is_absolute():
        raise ValueError(f"path must be absolute: {path}")
    if not path.is_file():
        raise ValueError(f"no such file: {path}")
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
    return {**entry, "changed": _size_and_mtime(entry["path"]) != (entry["size"], entry["mtimeMs"])}


def causes(error: BaseException | None) -> Iterator[BaseException]:
    while error is not None:
        yield error
        error = error.__cause__ or error.__context__


def classify(error: BaseException) -> tuple[str, str]:
    chain = list(causes(error))
    status = next((e.code for e in chain if isinstance(e, urllib.error.HTTPError)), None)
    text = str(error) or type(error).__name__
    if isinstance(error, FileNotFoundError):
        return "file_missing", f"The file is gone: {error.filename or text}"
    if status == 401:
        return "not_logged_in", "Not logged in to Dimensional cloud (or the login was revoked)."
    if status in (402, 413, 507):
        return "quota", f"Your Dimensional cloud storage quota doesn't allow this upload: {text}"
    if any(isinstance(e, TimeoutError | ConnectionError | urllib.error.URLError) for e in chain):
        return "network", f"Couldn't reach Dimensional cloud: {text}"
    return "failed", text


def console_datasets_url() -> str | None:
    from dimos.core.global_config import global_config

    base = global_config.dimos_cloud_url.rstrip("/")
    return (
        base.replace("://api.", "://console.", 1) + "/console/data" if "://api." in base else None
    )


def idle_login() -> dict[str, Any]:
    return {
        "state": "idle",
        "url": None,
        "urlComplete": None,
        "code": None,
        "expiresAt": None,
        "email": None,
        "error": None,
    }


class Queue:
    def __init__(self, items: list[dict[str, Any]] | None = None) -> None:
        self.items: list[dict[str, Any]] = items or []
        self.waiting_for_login = False
        self.rates: dict[str, Rate] = {}
        for item in self.items:
            if item["state"] == "uploading":
                item.update(state="queued", phase=None, rateBps=None, etaSeconds=None)
        numbers = [int(u["id"][1:]) for u in self.items if u["id"][1:].isdigit()]
        self.next = max(numbers, default=0)

    def get(self, id: str) -> dict[str, Any] | None:
        return next((u for u in self.items if u["id"] == id), None)

    def update(self, id: str, **change: Any) -> dict[str, Any] | None:
        upload = self.get(id)
        if upload is None:
            return None
        upload.update(change)
        return dict(upload)

    def enqueue(
        self, path: Path, size: int, robot_id: str | None, kind: str | None
    ) -> tuple[dict[str, Any], bool]:
        for upload in self.items:
            if upload["path"] == str(path) and upload["state"] in ("queued", "uploading"):
                return dict(upload), False
        self.next += 1
        upload = {
            "id": f"u{self.next}",
            "path": str(path),
            "name": path.name,
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
        if self.waiting_for_login or any(u["state"] == "uploading" for u in self.items):
            return None
        return next((u["id"] for u in self.items if u["state"] == "queued"), None)

    def start(self, id: str) -> dict[str, Any] | None:
        self.rates[id] = Rate()
        upload = self.get(id)
        return self.update(
            id,
            state="uploading",
            phase="preparing",
            bytesDone=0,
            bytesTotal=upload["size"] if upload else 0,
            rateBps=None,
            etaSeconds=None,
            error=None,
            errorCode=None,
            startedAt=now_ms(),
            finishedAt=None,
        )

    def progress(
        self, id: str, phase: str, done: int, total: int, t: float
    ) -> dict[str, Any] | None:
        upload = self.get(id)
        if upload is None or upload["state"] != "uploading":
            return None
        if upload["phase"] != phase:
            self.rates[id] = Rate()
        if total <= 0:
            return self.update(
                id, phase=phase, bytesDone=0, bytesTotal=0, rateBps=None, etaSeconds=None
            )
        rate = self.rates.setdefault(id, Rate())
        rate.tick(t, done)
        eta = rate.eta(max(total - done, 0), t)
        if phase == "upload" and done >= total:
            phase, eta = "finishing", None
        return self.update(
            id, phase=phase, bytesDone=done, bytesTotal=total, rateBps=rate.bps(), etaSeconds=eta
        )

    def succeed(self, id: str, result: dict[str, Any]) -> dict[str, Any] | None:
        upload = self.get(id)
        if upload is None:
            return None
        quota: dict[str, Any] = result.get("quota") or {}
        state = quota.get("state")
        notice = (quota.get("message") or f"quota: {state}") if state not in (None, "ok") else None
        return self.update(
            id,
            state="done",
            phase=None,
            uploadId=result.get("upload_id"),
            skipped=bool(result.get("skipped")),
            link=console_datasets_url() if result.get("upload_id") else None,
            bytesDone=upload["bytesTotal"],
            rateBps=None,
            etaSeconds=None,
            notice=notice,
            finishedAt=now_ms(),
        )

    def fail(self, id: str, code: str, message: str) -> dict[str, Any] | None:
        return self.update(
            id,
            state="failed",
            phase=None,
            rateBps=None,
            etaSeconds=None,
            error=message,
            errorCode=code,
            finishedAt=now_ms(),
        )

    def needs_login(self, id: str, message: str) -> dict[str, Any] | None:
        self.waiting_for_login = True
        return self.update(
            id,
            state="queued",
            phase=None,
            rateBps=None,
            etaSeconds=None,
            startedAt=None,
            error=message,
            errorCode="not_logged_in",
        )

    def logged_in(self) -> bool:
        was, self.waiting_for_login = self.waiting_for_login, False
        for upload in self.items:
            if upload["state"] == "queued" and upload["errorCode"] == "not_logged_in":
                upload.update(error=None, errorCode=None)
        return was

    def cancel(self, id: str) -> str | None:
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
        return self.update(
            id, state="cancelled", phase=None, rateBps=None, etaSeconds=None, finishedAt=now_ms()
        )

    def retry(self, id: str) -> dict[str, Any]:
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


def whoami(key: str) -> requests.Response:
    from dimos.core.global_config import global_config

    return requests.get(
        global_config.dimos_cloud_url.rstrip("/") + "/auth/whoami",
        headers={"Authorization": f"Bearer {key}"},
        timeout=global_config.dimos_http_timeout,
    )


def account() -> dict[str, Any]:
    from dimos.cli import cloud
    from dimos.core.global_config import global_config

    key = cloud.api_key()
    result: dict[str, Any] = {
        "loggedIn": False,
        "email": None,
        "scopes": None,
        "source": None,
        "cloudUrl": global_config.dimos_cloud_url.rstrip("/"),
        "error": None,
    }
    if not key:
        return result
    result["source"] = "env" if global_config.dimos_api_key else "stored"
    try:
        answer = whoami(key)
    except requests.RequestException as error:
        return {**result, "loggedIn": True, "error": f"Couldn't reach Dimensional cloud: {error}"}
    if answer.status_code == 401:
        return {**result, "error": "The saved login was revoked or is invalid: log in again."}
    if not answer.ok:
        return {
            **result,
            "loggedIn": True,
            "error": f"Dimensional cloud answered {answer.status_code}",
        }
    who = answer.json()
    return {**result, "loggedIn": True, "email": who.get("email"), "scopes": who.get("scopes")}


class Uploads:
    def __init__(self, bus: Bus, file: Path | None) -> None:
        self.bus = bus
        self.file = file
        saved = (store.read_json(file) if file else None) or []
        self.uploaded_by_path: dict[str, dict[str, Any]] = (
            store.read_json(file.with_name("uploaded.json")) if file else None
        ) or {}
        for upload in saved:
            done = uploaded_entry(upload)
            if done:
                self.uploaded_by_path.setdefault(done["path"], done)
        self.queue = Queue(saved)
        self.wake = asyncio.Event()
        self.loop: asyncio.AbstractEventLoop | None = None
        self.running: tuple[str, threading.Event] | None = None
        self.login: dict[str, Any] = idle_login()
        self.login_cancel: threading.Event | None = None
        self.account_cache: tuple[float, dict[str, Any]] | None = None
        self.stopping = False

    def listing(self) -> dict[str, Any]:
        return {"uploads": self.queue.items, "waitingForLogin": self.queue.waiting_for_login}

    def save(self) -> None:
        if self.file:
            store.write_atomic(self.file, json.dumps(self.queue.items, indent=2))

    def uploaded(self) -> dict[str, Any]:
        return {"byPath": {p: checked(e) for p, e in sorted(self.uploaded_by_path.items())}}

    def uploaded_one(self, path: str) -> dict[str, Any] | None:
        entry = self.uploaded_by_path.get(str(Path(path.strip()).expanduser()))
        return checked(entry) if entry else None

    def emit(self, upload: dict[str, Any] | None) -> None:
        if upload is not None:
            self.bus.send({"type": "upload", "upload": upload})

    def emit_waiting(self, **extra: Any) -> None:
        self.bus.send({"type": "uploads", "waitingForLogin": self.queue.waiting_for_login, **extra})

    def go_on(self) -> None:
        if self.loop is None:
            self.wake.set()
        else:
            self.loop.call_soon_threadsafe(self.wake.set)

    def enqueue(self, path: str, robot_id: str | None, kind: str | None) -> dict[str, Any]:
        checked_path, size = check_path(path)
        upload, added = self.queue.enqueue(checked_path, size, robot_id, kind)
        if added:
            self.save()
            self.emit(upload)
            self.wake.set()
        return upload

    def cancel(self, id: str) -> None:
        outcome = self.queue.cancel(id)
        if outcome is None:
            raise KeyError(f"no upload {id}")
        if outcome == "running" and self.running and self.running[0] == id:
            self.running[1].set()
        elif outcome == "cancelled":
            self.emit(self.queue.get(id))
        elif outcome == "removed":
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

    async def account(self, fresh: bool) -> dict[str, Any]:
        cached = self.account_cache
        if not fresh and cached and time.monotonic() - cached[0] < ACCOUNT_TTL_S:
            return cached[1]
        value = await asyncio.to_thread(account)
        self.account_cache = (time.monotonic(), value)
        if value["loggedIn"] and self.queue.logged_in():
            self.emit_waiting()
            self.wake.set()
        return value

    async def logout(self) -> dict[str, Any]:
        from dimos.cli import cloud

        await asyncio.to_thread(cloud._forget)
        self.cancel_login()
        self.login = idle_login()
        return await self.account(True)

    def login_state(self) -> dict[str, Any]:
        if self.login["state"] == "pending" and (self.login["expiresAt"] or math.inf) < now_ms():
            self.login["state"] = "expired"
        return dict(self.login)

    def set_login(self, **change: Any) -> None:
        self.login.update(change)
        self.bus.send({"type": "cloud-login", "login": dict(self.login)})

    def _device_login(self, cancel: threading.Event) -> None:
        from dimos.cli import cloud

        try:
            device = cloud._post("/auth/device", label=socket.gethostname())
            self.set_login(
                state="pending",
                url=device["verification_uri"],
                urlComplete=device.get("verification_uri_complete"),
                code=device["user_code"],
                expiresAt=now_ms() + int(device["expires_in"]) * 1000,
            )
            deadline = time.time() + device["expires_in"]
            while time.time() < deadline and not cancel.wait(device["interval"]):
                answer = cloud._post("/auth/token", device_code=device["device_code"])
                if answer["status"] == "ok":
                    cloud._store(answer["api_key"])
                    self.set_login(state="approved", email=answer.get("email"))
                    self.account_cache = None
                    if self.queue.logged_in():
                        self.emit_waiting()
                    self.go_on()
                    return
                if answer["status"] in ("denied", "expired"):
                    self.set_login(state=answer["status"])
                    return
            if not cancel.is_set():
                self.set_login(state="expired")
        except Exception as error:
            if not cancel.is_set():
                self.set_login(state="failed", error=classify(error)[1])

    async def start_login(self) -> dict[str, Any]:
        if self.login_state()["state"] in ("starting", "pending"):
            return self.login_state()
        self.cancel_login()
        self.login_cancel = cancel = threading.Event()
        self.set_login(**{**idle_login(), "state": "starting"})
        threading.Thread(target=self._device_login, args=(cancel,), daemon=True).start()
        for _ in range(300):
            if self.login_state()["state"] != "starting":
                break
            await asyncio.sleep(0.1)
        return self.login_state()

    def cancel_login(self) -> dict[str, Any]:
        if self.login_cancel is not None:
            self.login_cancel.set()
            self.login_cancel = None
        if self.login_state()["state"] in ("starting", "pending"):
            self.set_login(**idle_login())
        return self.login_state()

    def shutdown(self) -> None:
        self.stopping = True
        self.save()
        if self.running:
            self.running[1].set()
        if self.login_cancel is not None:
            self.login_cancel.set()

    async def work(self) -> None:
        self.loop = asyncio.get_running_loop()
        while not self.stopping:
            next_id = self.queue.next_queued()
            if next_id:
                await self.upload(next_id)
            else:
                await self.wake.wait()
                self.wake.clear()

    def _transfer(
        self, path: str, robot_id: str | None, kind: str | None, tick: Callable[..., None]
    ) -> dict[str, Any]:
        from dimos.cli import cloud
        from dimos.cloud.data import CloudData

        if not Path(path).is_file():
            raise FileNotFoundError(2, "No such file", path)
        if not cloud.api_key():
            raise PermissionError("not logged in")
        tick("preparing", 0, 0)
        return CloudData().upload(Path(path), robot_id=robot_id, kind=kind, progress=tick)

    async def upload(self, id: str) -> None:
        upload = self.queue.start(id)
        if upload is None:
            return
        self.save()
        self.emit(upload)
        assert self.loop is not None
        loop, cancel, started = self.loop, threading.Event(), time.monotonic()
        self.running = (id, cancel)
        last: list[Any] = ["", -1.0]

        def progress(phase: str, done: int, total: int) -> None:
            updated = self.queue.progress(id, phase, done, total, time.monotonic() - started)
            if phase != last[0] or time.monotonic() - last[1] >= 0.4:
                last[:] = [phase, time.monotonic()]
                self.emit(updated)

        def tick(phase: str, done: int, total: int) -> None:
            if cancel.is_set():
                raise CancelledUploadError()
            loop.call_soon_threadsafe(progress, phase, done, total)

        try:
            result = await asyncio.to_thread(
                self._transfer, upload["path"], upload["robotId"], upload["kind"], tick
            )
            updated = self.queue.succeed(id, result)
        except CancelledUploadError:
            updated = self.queue.cancelled(id)
        except PermissionError:
            updated = self.queue.needs_login(
                id, "Not logged in to Dimensional cloud. Log in, then retry."
            )
            self.emit_waiting()
        except Exception as error:
            logger.exception("an upload failed", upload=id)
            code, message = classify(error)
            if code == "not_logged_in":
                updated = self.queue.needs_login(id, message)
                self.account_cache = None
                self.emit_waiting()
            else:
                updated = self.queue.fail(id, code, message)
        self.running = None
        if self.stopping:
            return
        if updated is not None:
            done = uploaded_entry(updated)
            if done is not None and self.file:
                self.uploaded_by_path[done["path"]] = done
                store.write_atomic(
                    self.file.with_name("uploaded.json"),
                    json.dumps(dict(sorted(self.uploaded_by_path.items())), indent=2),
                )
        self.save()
        self.emit(updated)
