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

# Copyright 2026 Dimensional Inc.
"""Private, read-only viewer boundary for one retained Pi session."""

from __future__ import annotations

from collections import Counter
from collections.abc import Iterator, Mapping
from contextlib import contextmanager
from dataclasses import dataclass
import hashlib
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import mimetypes
import os
from pathlib import Path
import secrets
import shutil
import stat
import tempfile
import threading
from typing import Any, ClassVar, Final
from urllib.parse import unquote, urlsplit
import webbrowser

from pydantic import ValidationError

from .native_session import (
    CaptureState,
    FailureReason,
    NativeSessionReceipt,
    compare_receipt,
    read_session_bytes,
    validate_native_session,
)
from .topology import PinnedDirectory, TopologyError

RECEIPT_NAME: Final = "native-session-receipt.v1.json"
VIEW_MODEL_NAME: Final = "session.json"
SOURCE_COPY_NAME: Final = "session.jsonl"
VIEWER_ASSET_ROOT: Final = Path(__file__).with_name("viewer_assets")
ASSET_MANIFEST_NAME: Final = "asset-manifest.json"
_SAFE_RAW_OMISSIONS: Final = frozenset(
    {"data", "thinkingSignature", "thoughtSignature", "textSignature"}
)


class SessionViewerError(RuntimeError):
    """Bounded operator-facing viewer failure."""

    def __init__(self, code: str) -> None:
        super().__init__(code)
        self.code = code


@dataclass(frozen=True)
class AdmittedSession:
    """An immutable admitted snapshot of canonical evidence."""

    private: PinnedDirectory
    receipt: NativeSessionReceipt
    source: bytes
    sha256: str

    def close(self) -> None:
        self.private.close()

    def verify_unchanged(self) -> None:
        try:
            current = read_session_bytes(self.private, self.receipt.relative_path or "")
        except (OSError, TopologyError, ValueError) as error:
            raise SessionViewerError("source_mutated") from error
        if hashlib.sha256(current).hexdigest() != self.sha256:
            raise SessionViewerError("source_mutated")


@dataclass(frozen=True)
class StagedSession:
    """Disposable files served to the browser."""

    root: Path
    view_model_path: Path
    source_copy_path: Path
    canonical_sha256: str


def _bounded_error(code: str, error: BaseException) -> SessionViewerError:
    return SessionViewerError(code)


def _read_private_receipt(private: PinnedDirectory) -> bytes:
    try:
        fd = os.open(
            RECEIPT_NAME,
            os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC,
            dir_fd=private.fd,
        )
    except OSError as error:
        raise SessionViewerError("invalid_evidence") from error
    try:
        info = os.fstat(fd)
        if (
            not stat.S_ISREG(info.st_mode)
            or info.st_uid != os.getuid()
            or stat.S_IMODE(info.st_mode) & 0o077
            or info.st_nlink != 1
            or info.st_size > 1024 * 1024
        ):
            raise SessionViewerError("invalid_evidence")
        data = b""
        while len(data) < info.st_size:
            chunk = os.read(fd, info.st_size - len(data))
            if not chunk:
                raise SessionViewerError("invalid_evidence")
            data += chunk
        after = os.fstat(fd)
        if (after.st_dev, after.st_ino, after.st_size) != (
            info.st_dev,
            info.st_ino,
            info.st_size,
        ):
            raise SessionViewerError("invalid_evidence")
        return data
    finally:
        os.close(fd)


def admit_attempt(attempt_directory: Path) -> AdmittedSession:
    """Admit one attempt's receipt-bound complete or valid partial session."""
    attempt: PinnedDirectory | None = None
    private: PinnedDirectory | None = None
    try:
        attempt = PinnedDirectory.open(attempt_directory.expanduser(), create=False)
        try:
            private = attempt.open_relative("private")
        except TopologyError:
            # Scheduler artifacts use the terminal mode directory itself as the
            # private attempt leaf. Accept that shape as well as attempt/private.
            private = attempt
            attempt = None
        raw_receipt = _read_private_receipt(private)
        receipt = NativeSessionReceipt.model_validate_json(raw_receipt)
        if receipt.state is CaptureState.UNAVAILABLE or receipt.relative_path is None:
            raise SessionViewerError("evidence_unavailable")
        if not compare_receipt(private, receipt):
            raise SessionViewerError("receipt_mismatch")
        source = read_session_bytes(private, receipt.relative_path)
        validation = validate_native_session(source, state=receipt.state)
        valid_partial_tail = (
            receipt.state is CaptureState.PARTIAL
            and validation.reason is FailureReason.INVALID_JSON
            and validation.session_id is not None
        )
        if validation.reason is not None and not valid_partial_tail:
            raise SessionViewerError("invalid_evidence")
        return AdmittedSession(
            private=private,
            receipt=receipt,
            source=source,
            sha256=hashlib.sha256(source).hexdigest(),
        )
    except SessionViewerError:
        if private is not None:
            private.close()
        raise
    except (OSError, TopologyError, ValueError, ValidationError) as error:
        if private is not None:
            private.close()
        raise _bounded_error("invalid_evidence", error) from error
    finally:
        if attempt is not None:
            attempt.close()


def _valid_json_lines(source: bytes, state: CaptureState) -> list[dict[str, Any]]:
    values: list[dict[str, Any]] = []
    lines = source.splitlines(keepends=True)
    for index, raw in enumerate(lines):
        try:
            value = json.loads(raw)
        except (UnicodeDecodeError, json.JSONDecodeError):
            if state is CaptureState.PARTIAL and index == len(lines) - 1:
                break
            raise SessionViewerError("invalid_evidence")
        if not isinstance(value, dict):
            raise SessionViewerError("invalid_evidence")
        values.append(value)
    return values


def _safe_raw(value: Any) -> Any:
    if isinstance(value, list):
        return [_safe_raw(item) for item in value]
    if isinstance(value, dict):
        return {
            str(key): "[omitted]" if key in _SAFE_RAW_OMISSIONS else _safe_raw(item)
            for key, item in value.items()
        }
    return value


def _text_parts(content: Any) -> list[dict[str, Any]]:
    if isinstance(content, str):
        return [{"type": "text", "text": content}]
    if not isinstance(content, list):
        return []
    parts: list[dict[str, Any]] = []
    for item in content:
        if not isinstance(item, dict):
            continue
        kind = item.get("type")
        if kind == "text":
            parts.append({"type": "text", "text": str(item.get("text", ""))})
        elif kind == "thinking":
            parts.append(
                {
                    "type": "thinking",
                    "text": str(item.get("thinking", "")),
                    "redacted": bool(item.get("redacted", False)),
                }
            )
        elif kind == "toolCall":
            parts.append(
                {
                    "type": "tool_call",
                    "callId": str(item.get("id", "")),
                    "name": str(item.get("name", "")),
                    "arguments": _safe_raw(item.get("arguments", {})),
                }
            )
        elif kind == "image":
            parts.append(
                {
                    "type": "image",
                    "mimeType": str(item.get("mimeType", "")),
                    "omitted": True,
                }
            )
    return parts


def _usage(value: Any) -> dict[str, Any] | None:
    if not isinstance(value, dict):
        return None
    result: dict[str, Any] = {}
    for key in ("input", "output", "cacheRead", "cacheWrite", "reasoning", "totalTokens"):
        item = value.get(key)
        if isinstance(item, (int, float)) and not isinstance(item, bool):
            result[key] = item
    cost = value.get("cost")
    if isinstance(cost, dict):
        total = cost.get("total")
        if isinstance(total, (int, float)) and not isinstance(total, bool):
            result["cost"] = {"total": total}
    return result or None


def _state_part(entry: Mapping[str, Any]) -> dict[str, Any]:
    kind = str(entry.get("type", "state"))
    values = {
        str(key): _safe_raw(value)
        for key, value in entry.items()
        if key not in {"type", "id", "parentId", "timestamp"}
    }
    value = json.dumps(values, ensure_ascii=False, separators=(",", ":"), sort_keys=True)
    return {"type": "state", "label": kind.replace("_", " "), "value": value}


def _entry_parts(
    entry: Mapping[str, Any],
) -> tuple[str, list[dict[str, Any]], dict[str, Any] | None]:
    if entry.get("type") != "message":
        if entry.get("type") == "custom_message":
            return "custom", _text_parts(entry.get("content")), None
        return "state", [_state_part(entry)], None

    message = entry.get("message")
    if not isinstance(message, dict):
        return "unknown", [], None
    role = str(message.get("role", "unknown"))
    if role == "bashExecution":
        part = {
            "type": "tool_result",
            "callId": "",
            "name": "bash",
            "isError": bool(message.get("cancelled", False))
            or int(message.get("exitCode", 0)) != 0,
            "content": str(message.get("output", "")),
        }
        return "bash", [part], None
    if role == "toolResult":
        rendered = _text_parts(message.get("content"))
        text = "\n".join(
            str(part.get("text", "")) for part in rendered if part.get("type") == "text"
        )
        if any(part.get("type") == "image" for part in rendered):
            text = f"{text}\n[image omitted]".strip()
        part = {
            "type": "tool_result",
            "callId": str(message.get("toolCallId", "")),
            "name": str(message.get("toolName", "")),
            "isError": bool(message.get("isError", False)),
            "content": text,
        }
        if "details" in message:
            part["details"] = _safe_raw(message["details"])
        return "tool", [part], None
    return role, _text_parts(message.get("content")), _usage(message.get("usage"))


def build_view_model(source: bytes, receipt: NativeSessionReceipt) -> dict[str, Any]:
    """Convert admitted Pi v3 JSONL to the viewer's immutable document."""
    lines = _valid_json_lines(source, receipt.state)
    if not lines or lines[0].get("type") != "session":
        raise SessionViewerError("invalid_evidence")
    header, native_entries = lines[0], lines[1:]
    depths: dict[str, int] = {}
    children: Counter[str] = Counter()
    entries: list[dict[str, Any]] = []
    token_totals: dict[str, int | float] = {}
    total_cost = 0.0
    latest_model: str | None = None
    latest_provider: str | None = None

    for native in native_entries:
        entry_id = str(native["id"])
        parent_id = native.get("parentId")
        if isinstance(parent_id, str):
            children[parent_id] += 1
        depth = depths.get(parent_id, -1) + 1 if isinstance(parent_id, str) else 0
        depths[entry_id] = depth
        role, parts, usage = _entry_parts(native)
        message = native.get("message")
        if isinstance(message, dict) and message.get("role") == "assistant":
            latest_model = str(message.get("model", "")) or latest_model
            latest_provider = str(message.get("provider", "")) or latest_provider
        elif native.get("type") == "model_change":
            latest_model = str(native.get("modelId", "")) or latest_model
            latest_provider = str(native.get("provider", "")) or latest_provider
        if usage is not None:
            for key, value in usage.items():
                if key == "cost" and isinstance(value, dict):
                    cost = value.get("total")
                    if isinstance(cost, (int, float)):
                        total_cost += cost
                elif isinstance(value, (int, float)):
                    token_totals[key] = token_totals.get(key, 0) + value
        message_timestamp = (
            message.get("timestamp")
            if isinstance(message, dict) and isinstance(message.get("timestamp"), (int, float))
            else None
        )
        entry_type = str(native.get("type", ""))
        title = entry_type.replace("_", " ")
        if isinstance(message, dict):
            title = {
                "user": "User",
                "assistant": "Assistant",
                "toolResult": f"Tool · {message.get('toolName', '')}".rstrip(" ·"),
                "custom": str(message.get("customType", "Custom")),
                "bashExecution": "Shell execution",
            }.get(str(message.get("role")), title)
        elif entry_type == "custom_message":
            title = str(native.get("customType", "Custom"))
        text_preview = next(
            (
                str(part.get("text", "")).replace("\n", " ")[:100]
                for part in parts
                if part.get("type") in {"text", "thinking"}
            ),
            "",
        )
        entries.append(
            {
                "id": entry_id,
                "parentId": parent_id,
                "depth": depth,
                "timestamp": str(native.get("timestamp", "")),
                "timestampMs": message_timestamp,
                "type": entry_type,
                "role": role,
                "title": title,
                "preview": text_preview,
                "parts": parts,
                "usage": usage,
                "model": message.get("model") if isinstance(message, dict) else None,
                "provider": message.get("provider") if isinstance(message, dict) else None,
                "stopReason": (message.get("stopReason") if isinstance(message, dict) else None),
                "raw": _safe_raw(native),
            }
        )

    entry_ids = {entry["id"] for entry in entries}
    head_ids = [entry["id"] for entry in entries if children[entry["id"]] == 0]
    default_head_id = entries[-1]["id"] if entries else ""
    if default_head_id and default_head_id not in entry_ids:
        raise SessionViewerError("invalid_evidence")
    total_usage: dict[str, Any] = dict(token_totals)
    if total_cost:
        total_usage["cost"] = {"total": total_cost}
    root_ids = [entry["id"] for entry in entries if entry["parentId"] is None]
    return {
        "schemaVersion": "1.0",
        "summary": {
            "id": str(header.get("id", "")),
            "startedAt": str(header.get("timestamp", "")),
            "status": receipt.state.value,
            "entryCount": len(entries),
            "branchCount": sum(1 for count in children.values() if count > 1),
            "model": latest_model,
            "provider": latest_provider,
            "totalUsage": total_usage,
        },
        "entries": entries,
        "rootIds": root_ids,
        "headIds": head_ids,
        "defaultHeadId": default_head_id,
    }


def _write_private_file(path: Path, data: bytes) -> None:
    fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o600)
    try:
        view = memoryview(data)
        while view:
            view = view[os.write(fd, view) :]
    finally:
        os.close(fd)


@contextmanager
def stage_session(admitted: AdmittedSession) -> Iterator[StagedSession]:
    """Stage only the native session copy and derived immutable view model."""
    root = Path(tempfile.mkdtemp(prefix="pi-session-viewer-"))
    os.chmod(root, 0o700)
    try:
        source_path = root / SOURCE_COPY_NAME
        view_model_path = root / VIEW_MODEL_NAME
        _write_private_file(source_path, admitted.source)
        view_model = build_view_model(admitted.source, admitted.receipt)
        encoded = json.dumps(
            view_model, ensure_ascii=False, separators=(",", ":"), sort_keys=True
        ).encode("utf-8")
        _write_private_file(view_model_path, encoded)
        if source_path.read_bytes() != admitted.source:
            raise SessionViewerError("staging_failed")
        yield StagedSession(root, view_model_path, source_path, admitted.sha256)
        if source_path.read_bytes() != admitted.source:
            raise SessionViewerError("staged_copy_mutated")
    except SessionViewerError:
        raise
    except OSError as error:
        raise _bounded_error("staging_failed", error) from error
    finally:
        try:
            shutil.rmtree(root)
        except OSError as error:
            raise _bounded_error("cleanup_failed", error) from error


class _LoopbackHTTPServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = False

    def server_bind(self) -> None:
        super().server_bind()
        host = self.server_address[0]
        if host not in {"127.0.0.1", "::1"}:
            raise SessionViewerError("bind_failed")


def _handler_type(
    token: str, staged: StagedSession, asset_root: Path
) -> type[BaseHTTPRequestHandler]:
    prefix = f"/{token}"
    allowed_assets = frozenset(
        path.relative_to(asset_root).as_posix()
        for path in asset_root.rglob("*")
        if path.is_file() and not path.is_symlink() and path.name != ASSET_MANIFEST_NAME
    )

    class ViewerHandler(BaseHTTPRequestHandler):
        server_version = "PiSessionViewer"
        sys_version = ""
        _headers: ClassVar[dict[str, str]] = {
            "Cache-Control": "no-store",
            "Content-Security-Policy": (
                "default-src 'none'; script-src 'self'; style-src 'self'; "
                "img-src 'self'; font-src 'self'; connect-src 'self'; "
                "base-uri 'none'; form-action 'none'; frame-ancestors 'none'; "
                "object-src 'none'"
            ),
            "Cross-Origin-Resource-Policy": "same-origin",
            "Permissions-Policy": ("camera=(), microphone=(), geolocation=(), payment=(), usb=()"),
            "Referrer-Policy": "no-referrer",
            "X-Content-Type-Options": "nosniff",
            "X-Frame-Options": "DENY",
        }

        def log_message(self, format: str, *args: object) -> None:
            del format, args

        def _send_file(self, path: Path, *, content_type: str) -> None:
            try:
                content = path.read_bytes()
            except OSError:
                self._empty(HTTPStatus.NOT_FOUND)
                return
            self.send_response(HTTPStatus.OK)
            for name, value in self._headers.items():
                self.send_header(name, value)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(content)))
            self.end_headers()
            if self.command == "GET":
                self.wfile.write(content)

        def _empty(self, status: HTTPStatus) -> None:
            self.send_response(status)
            for name, value in self._headers.items():
                self.send_header(name, value)
            self.send_header("Content-Length", "0")
            self.end_headers()

        def _read(self) -> None:
            parsed = urlsplit(self.path)
            if parsed.query:
                self._empty(HTTPStatus.NOT_FOUND)
                return
            target = unquote(parsed.path)
            if target in {prefix, f"{prefix}/"}:
                self._send_file(asset_root / "index.html", content_type="text/html; charset=utf-8")
                return
            if target == f"{prefix}/{VIEW_MODEL_NAME}":
                self._send_file(
                    staged.view_model_path, content_type="application/json; charset=utf-8"
                )
                return
            asset_prefix = f"{prefix}/"
            if target.startswith(asset_prefix):
                relative = target[len(asset_prefix) :]
                if relative in allowed_assets and relative != "index.html":
                    content_type = mimetypes.guess_type(relative)[0] or "application/octet-stream"
                    self._send_file(asset_root / relative, content_type=content_type)
                    return
            self._empty(HTTPStatus.NOT_FOUND)

        def do_GET(self) -> None:
            self._read()

        def do_HEAD(self) -> None:
            self._read()

        def do_POST(self) -> None:
            self._empty(HTTPStatus.METHOD_NOT_ALLOWED)

        def do_PUT(self) -> None:
            self.do_POST()

        def do_PATCH(self) -> None:
            self.do_POST()

        def do_DELETE(self) -> None:
            self.do_POST()

        def do_OPTIONS(self) -> None:
            self.do_POST()

        def do_TRACE(self) -> None:
            self.do_POST()

        def do_CONNECT(self) -> None:
            self.do_POST()

    return ViewerHandler


@dataclass
class ViewerServer:
    """Attached loopback server for a single capability URL."""

    httpd: _LoopbackHTTPServer
    thread: threading.Thread
    url: str
    failures: list[BaseException]

    def wait(self) -> None:
        self.thread.join()
        if self.failures:
            raise SessionViewerError("server_failed")

    def close(self) -> None:
        self.httpd.shutdown()
        self.thread.join(timeout=5)
        self.httpd.server_close()
        if self.thread.is_alive():
            raise SessionViewerError("shutdown_failed")


def _verify_asset_manifest(asset_root: Path) -> bool:
    try:
        if any(path.is_symlink() for path in asset_root.rglob("*")):
            return False
        manifest = json.loads((asset_root / ASSET_MANIFEST_NAME).read_bytes())
        expected = manifest.get("files")
        if not isinstance(expected, dict) or not expected:
            return False
        actual = {
            path.relative_to(asset_root).as_posix()
            for path in asset_root.rglob("*")
            if path.is_file() and path.name != ASSET_MANIFEST_NAME
        }
        if actual != set(expected):
            return False
        return all(
            isinstance(digest, str)
            and hashlib.sha256((asset_root / relative).read_bytes()).hexdigest() == digest
            for relative, digest in expected.items()
        )
    except (OSError, TypeError, ValueError):
        return False


@contextmanager
def serve_staged_session(
    staged: StagedSession, *, asset_root: Path = VIEWER_ASSET_ROOT
) -> Iterator[ViewerServer]:
    """Serve one staged session over a token-scoped loopback origin."""
    if not _verify_asset_manifest(asset_root):
        raise SessionViewerError("assets_unavailable")
    token = secrets.token_urlsafe(24)
    httpd: _LoopbackHTTPServer | None = None
    server: ViewerServer | None = None
    try:
        httpd = _LoopbackHTTPServer(("127.0.0.1", 0), _handler_type(token, staged, asset_root))
        port = int(httpd.server_address[1])
        failures: list[BaseException] = []

        def serve() -> None:
            try:
                httpd.serve_forever()
            except BaseException as error:
                failures.append(error)

        thread = threading.Thread(target=serve, name="pi-session-viewer", daemon=True)
        thread.start()
        server = ViewerServer(httpd, thread, f"http://127.0.0.1:{port}/{token}/", failures)
        yield server
    except SessionViewerError:
        raise
    except OSError as error:
        raise _bounded_error("server_start_failed", error) from error
    finally:
        if server is not None:
            server.close()
        elif httpd is not None:
            httpd.server_close()


def view_attempt(attempt_directory: Path, *, open_browser: bool = True) -> int:
    """Run the attached viewer until interrupted, then verify canonical bytes."""
    admitted = admit_attempt(attempt_directory)
    try:
        with stage_session(admitted) as staged:
            with serve_staged_session(staged) as server:
                print(server.url, flush=True)
                if open_browser:
                    try:
                        webbrowser.open(server.url, new=2)
                    except (OSError, webbrowser.Error):
                        pass
                try:
                    server.wait()
                except KeyboardInterrupt:
                    pass
        admitted.verify_unchanged()
        return 0
    finally:
        admitted.close()
