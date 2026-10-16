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
from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import socket
import threading
import time
from typing import Any
import urllib.error
import urllib.request

from dimos.cli import cloud
from dimos.core.global_config import global_config
from dimos.utils.logging_config import setup_logger
from experimental.gateway import store
from experimental.gateway.events import Bus

logger = setup_logger()

FINISHED = ("done", "failed", "cancelled")
WARMUP_S = 1.0
TAU_S = 6.0
ACCOUNT_TTL_S = 20.0
IDLE_LOGIN: dict[str, Any] = {
    "state": "idle",
    **dict.fromkeys(("url", "urlComplete", "code", "expiresAt", "email", "error")),
}


def now_ms() -> int:
    return int(time.time() * 1000)


class CancelledUploadError(Exception):
    pass


@dataclass
class Rate:
    """Bytes per second, a moving average over about TAU_S seconds."""

    since: float
    t: float
    done: int
    bps: float | None = None

    def tick(self, t: float, done: int) -> None:
        if t > self.t:
            instant = (done - self.done) / (t - self.t)
            alpha = 1.0 - math.exp(-(t - self.t) / TAU_S)
            self.bps = instant if self.bps is None else self.bps + alpha * (instant - self.bps)
            self.t, self.done = t, done

    def eta(self, remaining: int) -> float | None:
        if not self.bps or self.t - self.since < WARMUP_S:
            return None
        return max(remaining / self.bps, 1.0 if remaining > 0 else 0.0)


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
    base = global_config.dimos_cloud_url.rstrip("/")
    return (
        base.replace("://api.", "://console.", 1) + "/console/data" if "://api." in base else None
    )


def new_upload(
    id: str, path: Path, size: int, robot_id: str | None, kind: str | None
) -> dict[str, Any]:
    return {
        "id": id,
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


def account() -> dict[str, Any]:
    key = cloud.api_key()
    base = global_config.dimos_cloud_url.rstrip("/")
    result: dict[str, Any] = {
        "loggedIn": False,
        "email": None,
        "scopes": None,
        "source": None,
        "cloudUrl": base,
        "error": None,
    }
    if not key:
        return result
    result.update(loggedIn=True, source="env" if global_config.dimos_api_key else "stored")
    request = urllib.request.Request(
        base + "/auth/whoami", headers={"Authorization": f"Bearer {key}"}
    )
    try:
        with urllib.request.urlopen(request, timeout=global_config.dimos_http_timeout) as answer:
            who = json.load(answer)
    except urllib.error.HTTPError as error:
        if error.code == 401:
            return {
                **result,
                "loggedIn": False,
                "error": "The saved login was revoked or is invalid: log in again.",
            }
        return {**result, "error": f"Dimensional cloud answered {error.code}"}
    except OSError as error:
        return {**result, "error": f"Couldn't reach Dimensional cloud: {error}"}
    return {**result, "email": who.get("email"), "scopes": who.get("scopes")}


class Uploads:
    """The upload queue, worked one at a time, and the device login it waits on."""

    def __init__(self, bus: Bus, file: Path | None) -> None:
        self.bus = bus
        self.file = file
        self.items: list[dict[str, Any]] = (store.read_json(file) if file else None) or []
        self.uploaded_by_path: dict[str, dict[str, Any]] = (
            store.read_json(file.with_name("uploaded.json")) if file else None
        ) or {}
        for upload in self.items:
            done = uploaded_entry(upload)
            if done:
                self.uploaded_by_path.setdefault(done["path"], done)
            if upload["state"] == "uploading":
                upload.update(state="queued", phase=None, rateBps=None, etaSeconds=None)
        self.next = max((int(u["id"][1:]) for u in self.items if u["id"][1:].isdigit()), default=0)
        self.waiting_for_login = False
        self.rates: dict[str, Rate] = {}
        self.wake = asyncio.Event()
        self.loop: asyncio.AbstractEventLoop | None = None
        self.running: tuple[str, threading.Event] | None = None
        self.login: dict[str, Any] = dict(IDLE_LOGIN)
        self.login_cancel: threading.Event | None = None
        self.account_cache: tuple[float, dict[str, Any]] | None = None
        self.stopping = False

    def get(self, id: str) -> dict[str, Any] | None:
        return next((u for u in self.items if u["id"] == id), None)

    def update(self, id: str, **change: Any) -> dict[str, Any] | None:
        """Change an upload, clearing its progress once it isn't uploading; returns a copy."""
        upload = self.get(id)
        if upload is None:
            return None
        upload.update(change)
        if upload["state"] != "uploading":
            upload.update(phase=None, rateBps=None, etaSeconds=None)
            self.rates.pop(id, None)
        return dict(upload)

    def listing(self) -> dict[str, Any]:
        return {"uploads": self.items, "waitingForLogin": self.waiting_for_login}

    def save(self) -> None:
        if self.file:
            store.write_atomic(self.file, json.dumps(self.items, indent=2))

    def uploaded(self) -> dict[str, Any]:
        return {"byPath": {p: checked(e) for p, e in sorted(self.uploaded_by_path.items())}}

    def uploaded_one(self, path: str) -> dict[str, Any] | None:
        entry = self.uploaded_by_path.get(str(Path(path.strip()).expanduser()))
        return checked(entry) if entry else None

    def emit(self, upload: dict[str, Any] | None) -> None:
        if upload is not None:
            self.bus.send({"type": "upload", "upload": upload})

    def emit_waiting(self, **extra: Any) -> None:
        self.bus.send({"type": "uploads", "waitingForLogin": self.waiting_for_login, **extra})

    def go_on(self) -> None:
        if self.loop is None:
            self.wake.set()
        else:
            self.loop.call_soon_threadsafe(self.wake.set)

    def enqueue(self, path: str, robot_id: str | None, kind: str | None) -> dict[str, Any]:
        checked_path, size = check_path(path)
        for upload in self.items:
            if upload["path"] == str(checked_path) and upload["state"] in ("queued", "uploading"):
                return dict(upload)
        self.next += 1
        upload = new_upload(f"u{self.next}", checked_path, size, robot_id, kind)
        self.items.append(upload)
        self.save()
        self.emit(dict(upload))
        self.wake.set()
        return dict(upload)

    def cancel(self, id: str) -> None:
        upload = self.get(id)
        if upload is None:
            raise KeyError(f"no upload {id}")
        if upload["state"] == "uploading":
            if self.running and self.running[0] == id:
                self.running[1].set()
        elif upload["state"] == "queued":
            self.emit(
                self.update(id, state="cancelled", error=None, errorCode=None, finishedAt=now_ms())
            )
        else:
            self.items.remove(upload)
            self.bus.send({"type": "upload-removed", "id": id})
        self.save()

    def retry(self, id: str) -> dict[str, Any]:
        upload = self.get(id)
        if upload is None:
            raise KeyError(f"no upload {id}")
        if upload["state"] not in FINISHED:
            raise ValueError(f'{upload["name"]} is already "{upload["state"]}"')
        self.items.remove(upload)
        upload.update(
            new_upload(id, Path(upload["path"]), upload["size"], upload["robotId"], upload["kind"]),
            uploadId=upload["uploadId"],
        )
        self.items.append(upload)
        self.waiting_for_login = False
        self.save()
        self.emit(dict(upload))
        self.emit_waiting()
        self.wake.set()
        return dict(upload)

    def clear_finished(self) -> dict[str, Any]:
        self.items = [u for u in self.items if u["state"] not in FINISHED]
        self.save()
        self.emit_waiting(cleared=True)
        return self.listing()

    def logged_in(self) -> None:
        """Stop waiting for a login, and wake the queue."""
        if self.waiting_for_login:
            self.waiting_for_login = False
            self.emit_waiting()
        for upload in self.items:
            if upload["state"] == "queued" and upload["errorCode"] == "not_logged_in":
                upload.update(error=None, errorCode=None)
        self.go_on()

    async def account(self, fresh: bool) -> dict[str, Any]:
        cached = self.account_cache
        if not fresh and cached and time.monotonic() - cached[0] < ACCOUNT_TTL_S:
            return cached[1]
        value = await asyncio.to_thread(account)
        self.account_cache = (time.monotonic(), value)
        if value["loggedIn"]:
            self.logged_in()
        return value

    async def logout(self) -> dict[str, Any]:
        await asyncio.to_thread(cloud._forget)
        self.cancel_login()
        self.login = dict(IDLE_LOGIN)
        return await self.account(True)

    def login_state(self) -> dict[str, Any]:
        if self.login["state"] == "pending" and (self.login["expiresAt"] or math.inf) < now_ms():
            self.login["state"] = "expired"
        return dict(self.login)

    def set_login(self, **change: Any) -> None:
        self.login.update(change)
        self.bus.send({"type": "cloud-login", "login": dict(self.login)})

    def _device_login(self, cancel: threading.Event) -> None:
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
                    self.logged_in()
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
        self.set_login(**{**IDLE_LOGIN, "state": "starting"})
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
            self.set_login(**IDLE_LOGIN)
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
            uploading = any(u["state"] == "uploading" for u in self.items)
            queued = next((u for u in self.items if u["state"] == "queued"), None)
            if queued and not uploading and not self.waiting_for_login:
                await self.upload(queued)
            else:
                await self.wake.wait()
                self.wake.clear()

    def record_progress(
        self, id: str, phase: str, done: int, total: int, t: float
    ) -> dict[str, Any] | None:
        upload = self.get(id)
        if upload is None or upload["state"] != "uploading":
            return None
        if upload["phase"] != phase:
            self.rates.pop(id, None)
        if total <= 0:
            return self.update(
                id, phase=phase, bytesDone=0, bytesTotal=0, rateBps=None, etaSeconds=None
            )
        rate = self.rates.get(id)
        if rate is None or done < rate.done:
            rate = self.rates[id] = Rate(since=t, t=t, done=done)
        else:
            rate.tick(t, done)
        eta = rate.eta(max(total - done, 0))
        if phase == "upload" and done >= total:
            phase, eta = "finishing", None
        return self.update(
            id,
            phase=phase,
            bytesDone=done,
            bytesTotal=total,
            rateBps=rate.bps or None,
            etaSeconds=eta,
        )

    def _transfer(
        self, path: str, robot_id: str | None, kind: str | None, tick: Callable[..., None]
    ) -> dict[str, Any]:
        from dimos.cloud.data import CloudData

        if not Path(path).is_file():
            raise FileNotFoundError(2, "No such file", path)
        if not cloud.api_key():
            raise PermissionError("not logged in")
        tick("preparing", 0, 0)
        return CloudData().upload(Path(path), robot_id=robot_id, kind=kind, progress=tick)

    async def upload(self, upload: dict[str, Any]) -> None:
        id = upload["id"]
        upload.update(
            state="uploading",
            phase="preparing",
            bytesDone=0,
            bytesTotal=upload["size"],
            error=None,
            errorCode=None,
            startedAt=now_ms(),
            finishedAt=None,
        )
        self.save()
        self.emit(dict(upload))
        assert self.loop is not None
        loop, cancel, started = self.loop, threading.Event(), time.monotonic()
        self.running = (id, cancel)
        last_phase, last_emit = "", -1.0

        def progress(phase: str, done: int, total: int) -> None:
            nonlocal last_phase, last_emit
            updated = self.record_progress(id, phase, done, total, time.monotonic() - started)
            if phase != last_phase or time.monotonic() - last_emit >= 0.4:
                last_phase, last_emit = phase, time.monotonic()
                self.emit(updated)

        def tick(phase: str, done: int, total: int) -> None:
            if cancel.is_set():
                raise CancelledUploadError()
            loop.call_soon_threadsafe(progress, phase, done, total)

        try:
            result = await asyncio.to_thread(
                self._transfer, upload["path"], upload["robotId"], upload["kind"], tick
            )
            quota: dict[str, Any] = result.get("quota") or {}
            quota_state = quota.get("state")
            updated = self.update(
                id,
                state="done",
                uploadId=result.get("upload_id"),
                skipped=bool(result.get("skipped")),
                link=console_datasets_url() if result.get("upload_id") else None,
                bytesDone=upload["bytesTotal"],
                notice=(quota.get("message") or f"quota: {quota_state}")
                if quota_state not in (None, "ok")
                else None,
                finishedAt=now_ms(),
            )
        except CancelledUploadError:
            updated = self.update(id, state="cancelled", finishedAt=now_ms())
        except Exception as error:
            if isinstance(error, PermissionError):
                code, message = (
                    "not_logged_in",
                    "Not logged in to Dimensional cloud. Log in, then retry.",
                )
            else:
                logger.exception("an upload failed", upload=id)
                code, message = classify(error)
            if code == "not_logged_in":
                self.waiting_for_login, self.account_cache = True, None
                updated = self.update(
                    id, state="queued", startedAt=None, error=message, errorCode=code
                )
                self.emit_waiting()
            else:
                updated = self.update(
                    id, state="failed", error=message, errorCode=code, finishedAt=now_ms()
                )
        self.running = None
        if self.stopping:
            return
        done = uploaded_entry(updated) if updated else None
        if done is not None and self.file:
            self.uploaded_by_path[done["path"]] = done
            store.write_atomic(
                self.file.with_name("uploaded.json"),
                json.dumps(dict(sorted(self.uploaded_by_path.items())), indent=2),
            )
        self.save()
        self.emit(updated)
