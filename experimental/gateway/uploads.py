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
from dimos.cloud.config import CloudConfig
from dimos.utils.logging_config import setup_logger
from experimental.gateway import constants, store
from experimental.gateway.errors import ApiError
from experimental.gateway.events import Bus

logger = setup_logger()

FINISHED = ("done", "failed", "cancelled")
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
    """Bytes per second, a moving average over about constants.UPLOAD_RATE_SMOOTHING_S."""

    since: float
    at: float
    done: int
    bps: float | None = None

    def tick(self, at: float, done: int) -> None:
        if at > self.at:
            instant = (done - self.done) / (at - self.at)
            alpha = 1.0 - math.exp(-(at - self.at) / constants.UPLOAD_RATE_SMOOTHING_S)
            self.bps = instant if self.bps is None else self.bps + alpha * (instant - self.bps)
            self.at, self.done = at, done

    def eta(self, remaining: int) -> float | None:
        if not self.bps or self.at - self.since < constants.UPLOAD_ETA_WARMUP_S:
            return None
        return max(remaining / self.bps, 1.0 if remaining > 0 else 0.0)


def check_path(text: str) -> tuple[Path, int]:
    from dimos.cloud.data import kind_of

    path = Path(text.strip()).expanduser()
    if not path.is_absolute():
        raise ApiError(400, f"path must be absolute: {path}")
    if not path.is_file():
        raise ApiError(400, f"no such file: {path}")
    if kind_of(path) != "recording":
        raise ApiError(400, f"not a dimos recording (.mcap, or a .db dimos recorded): {path.name}")
    return path, path.stat().st_size


def _size_and_mtime(path: str) -> tuple[int, int] | None:
    try:
        stat = os.stat(path)
    except OSError:
        return None
    return stat.st_size, int(stat.st_mtime * 1000)


def uploaded_entry(
    upload: dict[str, Any], sent_size_and_mtime: tuple[int, int] | None
) -> dict[str, Any] | None:
    """The uploaded.json entry of a done upload, as the file was when its transfer started."""
    if upload["state"] != "done" or sent_size_and_mtime is None or not upload.get("uploadId"):
        return None
    return {
        "path": upload["path"],
        "uploadId": upload["uploadId"],
        "size": sent_size_and_mtime[0],
        "mtimeMs": sent_size_and_mtime[1],
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
    status = next(
        (cause.code for cause in chain if isinstance(cause, urllib.error.HTTPError)), None
    )
    text = str(error) or type(error).__name__
    if isinstance(error, FileNotFoundError):
        return "file_missing", f"The file is gone: {error.filename or text}"
    if status == 401:
        return "not_logged_in", "Not logged in to Dimensional cloud (or the login was revoked)."
    if status in (402, 413, 507):
        return "quota", f"Your Dimensional cloud storage quota doesn't allow this upload: {text}"
    if any(
        isinstance(cause, TimeoutError | ConnectionError | urllib.error.URLError) for cause in chain
    ):
        return "network", f"Couldn't reach Dimensional cloud: {text}"
    return "failed", text


def console_datasets_url() -> str | None:
    # TODO(core): CloudConfig should carry the console URL (neither it nor the upload answer gives one)
    base = CloudConfig().url.rstrip("/")
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
        "createdAt": now_ms(),
        "startedAt": None,
        "finishedAt": None,
        "link": None,
    }


def account() -> dict[str, Any]:
    key = cloud.api_key()
    base = CloudConfig().url.rstrip("/")
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
    result.update(loggedIn=True, source="env" if CloudConfig().api_key else "stored")
    request = urllib.request.Request(
        base + "/auth/whoami", headers={"Authorization": f"Bearer {key}"}
    )
    try:
        with urllib.request.urlopen(request, timeout=CloudConfig().http_timeout) as answer:
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
        self.uploaded_file = file.with_name("uploaded.json") if file else None
        self.items: list[dict[str, Any]] = (store.read_json(file) if file else None) or []
        self.uploaded_by_path: dict[str, dict[str, Any]] = (
            store.read_json(self.uploaded_file) if self.uploaded_file else None
        ) or {}
        for upload in self.items:
            # only when uploaded.json lost it: the file as it is now is the best guess left
            done = uploaded_entry(upload, _size_and_mtime(upload["path"]))
            if done:
                self.uploaded_by_path.setdefault(done["path"], done)
            if upload["state"] == "uploading":
                upload.update(state="queued", phase=None, rateBps=None, etaSeconds=None)
        self.next = max(
            (int(upload["id"][1:]) for upload in self.items if upload["id"][1:].isdigit()),
            default=0,
        )
        self.waiting_for_login = False
        self.rates: dict[str, Rate] = {}
        self.wake = asyncio.Event()
        self.loop: asyncio.AbstractEventLoop | None = None
        self.running: tuple[str, threading.Event] | None = None
        self.login: dict[str, Any] = dict(IDLE_LOGIN)
        self.login_cancel: threading.Event | None = None
        # held while a login attempt checks it is still current and stores its key, and while one is cancelled
        self.login_lock = threading.RLock()
        self.login_changed = threading.Event()
        self.account_cache: tuple[float, dict[str, Any]] | None = None
        self.stopping = False

    def get(self, id: str) -> dict[str, Any] | None:
        return next((upload for upload in self.items if upload["id"] == id), None)

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
        return {
            "byPath": {
                path: checked(entry) for path, entry in sorted(self.uploaded_by_path.items())
            }
        }

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
            raise ApiError(404, f"no upload {id}")
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
            raise ApiError(404, f"no upload {id}")
        if upload["state"] not in FINISHED:
            raise ApiError(409, f'{upload["name"]} is already "{upload["state"]}"')
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
        self.items = [upload for upload in self.items if upload["state"] not in FINISHED]
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
        if not fresh and cached and time.monotonic() - cached[0] < constants.ACCOUNT_CACHE_S:
            return cached[1]
        value = await asyncio.to_thread(account)
        self.account_cache = (time.monotonic(), value)
        if value["loggedIn"]:
            self.logged_in()
        return value

    def announce_account(self, value: dict[str, Any]) -> dict[str, Any]:
        """Cache a just-changed account and tell the clients."""
        self.account_cache = (time.monotonic(), value)
        self.bus.send({"type": "cloud-account", "account": value})
        return value

    async def logout(self) -> dict[str, Any]:
        # TODO(core): dimos.cli.cloud should expose device login / key storage publicly
        self.cancel_login()
        await asyncio.to_thread(cloud._forget)
        self.login = dict(IDLE_LOGIN)
        return self.announce_account(await asyncio.to_thread(account))

    def login_state(self) -> dict[str, Any]:
        if self.login["state"] == "pending" and (self.login["expiresAt"] or math.inf) < now_ms():
            self.login["state"] = "expired"
        return dict(self.login)

    def set_login(self, **change: Any) -> None:
        self.login.update(change)
        self.login_changed.set()
        self.bus.send({"type": "cloud-login", "login": dict(self.login)})

    def _is_current(self, cancel: threading.Event) -> bool:
        return self.login_cancel is cancel and not cancel.is_set()

    def _set_login_if_current(self, cancel: threading.Event, **change: Any) -> bool:
        """Change the login only for the attempt still running; a cancelled one changes nothing."""
        with self.login_lock:
            if not self._is_current(cancel):
                return False
            self.set_login(**change)
            return True

    def _device_login(self, cancel: threading.Event) -> None:
        try:
            # TODO(core): dimos.cli.cloud should expose device login / key storage publicly
            device = cloud._post("/auth/device", label=socket.gethostname())
            self._set_login_if_current(
                cancel,
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
                    with self.login_lock:
                        if not self._is_current(cancel):
                            return  # cancelled while the token request was out: the key is dropped
                        # TODO(core): dimos.cli.cloud should expose device login / key storage publicly
                        cloud._store(answer["api_key"])
                        self.set_login(state="approved", email=answer.get("email"))
                    self.announce_account(account())
                    self.logged_in()
                    return
                if answer["status"] in ("denied", "expired"):
                    self._set_login_if_current(cancel, state=answer["status"])
                    return
            self._set_login_if_current(cancel, state="expired")
        except Exception as error:
            self._set_login_if_current(cancel, state="failed", error=classify(error)[1])

    async def start_login(self) -> dict[str, Any]:
        if self.login_state()["state"] in ("starting", "pending"):
            return self.login_state()
        with self.login_lock:
            self.cancel_login()
            self.login_cancel = cancel = threading.Event()
            self.set_login(**{**IDLE_LOGIN, "state": "starting"})
            self.login_changed.clear()
        threading.Thread(target=self._device_login, args=(cancel,), daemon=True).start()
        # Desktop opens the approval page from this answer, so it waits for the device code (one cloud request)
        await asyncio.to_thread(self.login_changed.wait, CloudConfig().http_timeout)
        return self.login_state()

    def cancel_login(self) -> dict[str, Any]:
        with self.login_lock:
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
            uploading = any(upload["state"] == "uploading" for upload in self.items)
            queued = next((upload for upload in self.items if upload["state"] == "queued"), None)
            if queued and not uploading and not self.waiting_for_login:
                await self.upload(queued)
            else:
                await self.wake.wait()
                self.wake.clear()

    def wait_for_login(self, id: str, message: str) -> dict[str, Any] | None:
        """Requeue an upload that needs a login, and hold the queue until one."""
        self.waiting_for_login, self.account_cache = True, None
        updated = self.update(
            id, state="queued", startedAt=None, error=message, errorCode="not_logged_in"
        )
        self.emit_waiting()
        return updated

    def record_progress(
        self, id: str, phase: str, done: int, total: int, at: float
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
            rate = self.rates[id] = Rate(since=at, at=at, done=done)
        else:
            rate.tick(at, done)
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
        # taken before the transfer reads the file, so one that grows meanwhile reads as changed afterwards
        sent_size_and_mtime = _size_and_mtime(upload["path"])

        def progress(phase: str, done: int, total: int) -> None:
            nonlocal last_phase, last_emit
            updated = self.record_progress(id, phase, done, total, time.monotonic() - started)
            if (
                phase != last_phase
                or time.monotonic() - last_emit >= constants.UPLOAD_PROGRESS_EMIT_S
            ):
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
            # TODO(core): CloudData.upload should take a cancel signal through completion and preview
            if cancel.is_set():
                raise CancelledUploadError(result.get("upload_id"))
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
        except CancelledUploadError as cancelled:
            sent_upload_id = cancelled.args[0] if cancelled.args else None
            updated = self.update(
                id,
                state="cancelled",
                uploadId=sent_upload_id or upload["uploadId"],
                notice="Cancelled after every byte was sent: Dimensional cloud may still keep this copy."
                if sent_upload_id
                else None,
                finishedAt=now_ms(),
            )
        except PermissionError:
            updated = self.wait_for_login(
                id, "Not logged in to Dimensional cloud. Log in, then retry."
            )
        except Exception as error:
            logger.exception("an upload failed", upload=id)
            code, message = classify(error)
            if code == "not_logged_in":
                updated = self.wait_for_login(id, message)
            else:
                updated = self.update(
                    id, state="failed", error=message, errorCode=code, finishedAt=now_ms()
                )
        self.running = None
        if self.stopping:
            return
        done = uploaded_entry(updated, sent_size_and_mtime) if updated else None
        if done is not None and self.uploaded_file:
            self.uploaded_by_path[done["path"]] = done
            store.write_atomic(
                self.uploaded_file,
                json.dumps(dict(sorted(self.uploaded_by_path.items())), indent=2),
            )
        self.save()
        self.emit(updated)
