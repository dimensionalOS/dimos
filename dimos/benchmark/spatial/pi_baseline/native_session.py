"""Small, pinned Pi v3 session and evidence boundary.

This module intentionally does not interpret provider payloads.  It checks the
parts of the Pi file format which are needed to establish provenance and tree
identity, while leaving message contents opaque.
"""

from __future__ import annotations

from enum import StrEnum
import hashlib
import json
import math
import os
from pathlib import Path
import re
import stat
import subprocess
import tempfile
from typing import Literal

from pydantic import BaseModel, ConfigDict, Field, model_validator

from .topology import PinnedDirectory, TopologyError

MAX_SESSION_BYTES = 64 * 1024 * 1024
MAX_SESSION_ENTRIES = 100_000
PRODUCER: Literal["@earendil-works/pi-coding-agent"] = "@earendil-works/pi-coding-agent"
PACKAGE_VERSION: Literal["0.80.10"] = "0.80.10"
SESSION_FORMAT_VERSION: Literal[3] = 3


class CaptureState(StrEnum):
    COMPLETE = "complete"
    PARTIAL = "partial"
    UNAVAILABLE = "unavailable"


class FailureReason(StrEnum):
    MISSING = "missing"
    UNREADABLE = "unreadable"
    UNSAFE_PATH = "unsafe_path"
    UNSAFE_FILE = "unsafe_file"
    TOO_LARGE = "too_large"
    TOO_MANY_ENTRIES = "too_many_entries"
    INVALID_JSON = "invalid_json"
    INVALID_SCHEMA = "invalid_schema"
    DUPLICATE_ID = "duplicate_id"
    UNKNOWN_PARENT = "unknown_parent"
    UNKNOWN_REFERENCE = "unknown_reference"
    RECEIPT_MISMATCH = "receipt_mismatch"
    PROMPT_MISMATCH = "prompt_mismatch"
    EXPORT_FAILED = "export_failed"
    TIMEOUT = "timeout"
    SOURCE_MUTATED = "source_mutated"
    EXECUTABLE_UNSAFE = "executable_unsafe"


class ExportVerificationStatus(StrEnum):
    """The outcome of a content-free native-session export check."""

    SUCCEEDED = "succeeded"
    FAILED = "failed"


class EvidenceModel(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)
    schema_version: Literal["1.0"] = "1.0"


class ExportVerificationRecord(EvidenceModel):
    """Safe, serializable evidence for the pinned native-session export.

    This model deliberately contains no command output, HTML, session data, or
    filesystem paths other than the admitted repository-relative session name.
    """

    record_type: Literal["pi-export-verification"] = "pi-export-verification"
    producer: Literal["@earendil-works/pi-coding-agent"] = PRODUCER
    package_version: Literal["0.80.10"] = PACKAGE_VERSION
    session_format_version: Literal[3] = SESSION_FORMAT_VERSION
    status: ExportVerificationStatus
    relative_path: str | None = None
    source_byte_count_before: int | None = Field(default=None, ge=0)
    source_byte_count_after: int | None = Field(default=None, ge=0)
    source_sha256_before: str | None = Field(default=None, pattern=r"^[0-9a-f]{64}$")
    source_sha256_after: str | None = Field(default=None, pattern=r"^[0-9a-f]{64}$")
    cli_identity_verified: bool = False
    disposable_html_output_produced: bool = False
    disposable_html_output_disposed: bool = False
    failure_code: FailureReason | None = None
    failure_reason: str | None = Field(default=None, max_length=64, pattern=r"^[a-z_ ]+$")

    @model_validator(mode="after")
    def _safe_path_and_status(self) -> ExportVerificationRecord:
        if self.relative_path is not None:
            try:
                _safe_relative(self.relative_path)
            except ValueError as exc:
                raise ValueError("relative_path must be an admitted native session path") from exc
        if self.status is ExportVerificationStatus.SUCCEEDED:
            if (
                self.relative_path is None
                or self.source_byte_count_before is None
                or self.source_byte_count_after is None
                or self.source_sha256_before is None
                or self.source_sha256_after is None
                or not self.cli_identity_verified
                or not self.disposable_html_output_produced
                or not self.disposable_html_output_disposed
                or self.failure_code is not None
                or self.failure_reason is not None
            ):
                raise ValueError("successful export verification is incomplete")
        elif self.failure_code is None or self.failure_reason is None:
            raise ValueError("failed export verification needs bounded failure metadata")
        return self


class PromptContextSidecar(EvidenceModel):
    record_type: Literal["pi-prompt-context"] = "pi-prompt-context"
    byte_count: int = Field(ge=0)
    sha256: str = Field(pattern=r"^[0-9a-f]{64}$")


class PromptContextRecord(EvidenceModel):
    record_type: Literal["pi-prompt-context-record"] = "pi-prompt-context-record"
    kind: Literal["system", "initial"]
    relative_path: str
    byte_count: int = Field(ge=0)
    sha256: str = Field(pattern=r"^[0-9a-f]{64}$")
    session_id: str = Field(min_length=1)

    @model_validator(mode="after")
    def _path_matches_kind(self) -> PromptContextRecord:
        expected = f"pi-prompt/{self.kind}.txt"
        if self.relative_path != expected:
            raise ValueError("prompt sidecar path does not match kind")
        return self


class NativeSessionReceipt(EvidenceModel):
    record_type: Literal["pi-native-session-receipt"] = "pi-native-session-receipt"
    producer: Literal["@earendil-works/pi-coding-agent"] = PRODUCER
    package_version: Literal["0.80.10"] = PACKAGE_VERSION
    session_format_version: Literal[3] = SESSION_FORMAT_VERSION
    state: CaptureState
    session_id: str | None = None
    relative_path: str | None = None
    byte_count: int | None = Field(default=None, ge=0)
    entry_count: int | None = Field(default=None, ge=0)
    sha256: str | None = Field(default=None, pattern=r"^[0-9a-f]{64}$")
    system_prompt: PromptContextRecord | None = None
    initial_prompt: PromptContextRecord | None = None
    reason: FailureReason | None = None

    @model_validator(mode="after")
    def _consistent(self) -> NativeSessionReceipt:
        present = (self.relative_path, self.byte_count, self.entry_count, self.sha256)
        if self.state is CaptureState.UNAVAILABLE:
            if (
                any(item is not None for item in present)
                or self.session_id is not None
                or self.system_prompt is not None
                or self.initial_prompt is not None
                or self.reason is None
            ):
                raise ValueError("unavailable receipt is incomplete")
        elif (
            any(item is None for item in present)
            or self.session_id is None
            or self.system_prompt is None
            or self.initial_prompt is None
        ):
            raise ValueError("captured receipt is incomplete")
        elif (
            self.system_prompt.session_id != self.session_id
            or self.initial_prompt.session_id != self.session_id
        ):
            raise ValueError("prompt sidecar session binding mismatch")
        return self


class VerificationResult(EvidenceModel):
    state: CaptureState
    reason: FailureReason | None = None
    session_id: str | None = None
    entry_count: int = 0


class SessionValidation(VerificationResult):
    pass


def _digest(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def make_prompt_sidecar(context: bytes) -> PromptContextSidecar:
    return PromptContextSidecar(byte_count=len(context), sha256=_digest(context))


def verify_prompt_sidecar(context: bytes, sidecar: PromptContextSidecar) -> bool:
    return len(context) == sidecar.byte_count and _digest(context) == sidecar.sha256


def _root(root: Path | PinnedDirectory) -> tuple[PinnedDirectory, bool]:
    if isinstance(root, PinnedDirectory):
        root.verify()
        return root, False
    return PinnedDirectory.open(Path(root), create=False), True


def _safe_relative(name: str) -> tuple[PinnedDirectory | None, str]:
    parts = tuple(name.split("/"))
    if (
        len(parts) != 2
        or parts[0] != "pi-session"
        or not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9._-]*\.jsonl", parts[1] or "")
    ):
        raise ValueError(FailureReason.UNSAFE_PATH.value)
    return None, parts[1]


def _admit(
    root: PinnedDirectory, name: str, *, max_bytes: int
) -> tuple[int, os.stat_result, PinnedDirectory | None]:
    _, basename = _safe_relative(name)
    try:
        directory = root.open_relative("pi-session")
        fd = os.open(basename, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC, dir_fd=directory.fd)
        info = os.fstat(fd)
        if (
            not stat.S_ISREG(info.st_mode)
            or info.st_uid != os.getuid()
            or stat.S_IMODE(info.st_mode) & 0o077
            or info.st_nlink != 1
        ):
            os.close(fd)
            directory.close()
            raise ValueError(FailureReason.UNSAFE_FILE.value)
        if info.st_size > max_bytes:
            os.close(fd)
            directory.close()
            raise ValueError(FailureReason.TOO_LARGE.value)
        return fd, info, directory
    except FileNotFoundError as exc:
        raise ValueError(FailureReason.MISSING.value) from exc
    except PermissionError as exc:
        raise ValueError(FailureReason.UNREADABLE.value) from exc
    except TopologyError as exc:
        raise ValueError(FailureReason.UNSAFE_PATH.value) from exc
    except OSError as exc:
        raise ValueError(FailureReason.UNSAFE_FILE.value) from exc


def read_session_bytes(
    root: Path | PinnedDirectory, relative_path: str, *, max_bytes: int = MAX_SESSION_BYTES
) -> bytes:
    pinned, owned = _root(root)
    try:
        fd, info, directory = _admit(pinned, relative_path, max_bytes=max_bytes)
        try:
            data = b""
            while len(data) < info.st_size:
                chunk = os.read(fd, min(1024 * 1024, info.st_size - len(data)))
                if not chunk:
                    raise ValueError(FailureReason.UNREADABLE.value)
                data += chunk
            after = os.fstat(fd)
            if (after.st_dev, after.st_ino, after.st_size) != (
                info.st_dev,
                info.st_ino,
                info.st_size,
            ):
                raise ValueError(FailureReason.SOURCE_MUTATED.value)
            return data
        finally:
            os.close(fd)
            if directory is not None:
                directory.close()
    finally:
        if owned:
            pinned.close()


_ENTRY_TYPES = {
    "message",
    "model_change",
    "thinking_level_change",
    "compaction",
    "branch_summary",
    "custom",
    "custom_message",
    "label",
    "session_info",
}
_ROLES = {"user", "assistant", "toolResult", "custom", "bashExecution"}


def _json_number(value: object) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def _text_image_array(value: object) -> bool:
    return isinstance(value, list) and all(
        isinstance(item, dict) and item.get("type") in {"text", "image"} for item in value
    )


def _user_content(value: object) -> bool:
    return isinstance(value, str) or _text_image_array(value)


def _strict_json(raw: bytes) -> object:
    value = json.loads(raw, parse_constant=lambda value: (_ for _ in ()).throw(ValueError(value)))

    def check(item: object) -> object:
        if isinstance(item, float) and not math.isfinite(item):
            raise ValueError("non-finite JSON number")
        if isinstance(item, list):
            for child in item:
                check(child)
        elif isinstance(item, dict):
            for child in item.values():
                check(child)
        return item

    return check(value)


def _bad(
    state: CaptureState, reason: FailureReason, sid: str | None, count: int
) -> SessionValidation:
    return SessionValidation(state=state, reason=reason, session_id=sid, entry_count=count)


def validate_native_session(
    data: bytes,
    *,
    state: CaptureState = CaptureState.COMPLETE,
    max_bytes: int = MAX_SESSION_BYTES,
    max_entries: int = MAX_SESSION_ENTRIES,
) -> SessionValidation:
    if not data:
        return _bad(CaptureState.UNAVAILABLE, FailureReason.MISSING, None, 0)
    if len(data) > max_bytes:
        return _bad(CaptureState.UNAVAILABLE, FailureReason.TOO_LARGE, None, 0)
    ids: set[str] = set()
    sid: str | None = None
    count = 0
    lines = data.splitlines(keepends=True)
    for index, raw_line in enumerate(lines):
        raw = raw_line.rstrip(b"\r\n")
        try:
            value = _strict_json(raw)
        except (UnicodeDecodeError, json.JSONDecodeError):
            if (
                state is CaptureState.PARTIAL
                and index == len(lines) - 1
                and not data.endswith(b"\n")
                and sid is not None
            ):
                return _bad(state, FailureReason.INVALID_JSON, sid, count)
            return _bad(
                state if sid is not None else CaptureState.UNAVAILABLE,
                FailureReason.INVALID_SCHEMA,
                sid,
                count,
            )
        except ValueError:
            return _bad(
                state if count else CaptureState.UNAVAILABLE,
                FailureReason.INVALID_SCHEMA,
                sid,
                count,
            )
        if not isinstance(value, dict):
            return _bad(
                state if count else CaptureState.UNAVAILABLE,
                FailureReason.INVALID_SCHEMA,
                sid,
                count,
            )
        if index == 0:
            if (
                value.get("type") != "session"
                or value.get("version") != 3
                or not isinstance(value.get("id"), str)
                or not value["id"]
                or not isinstance(value.get("timestamp"), str)
                or not value["timestamp"]
                or not isinstance(value.get("cwd"), str)
                or not value["cwd"]
            ):
                return _bad(CaptureState.UNAVAILABLE, FailureReason.INVALID_SCHEMA, None, 0)
            sid = value["id"]
            continue
        typ, entry_id = value.get("type"), value.get("id")
        if (
            "parentId" not in value
            or typ not in _ENTRY_TYPES
            or not isinstance(entry_id, str)
            or not entry_id
            or not isinstance(value.get("timestamp"), str)
            or not value["timestamp"]
        ):
            return _bad(state, FailureReason.INVALID_SCHEMA, sid, count)
        if entry_id in ids:
            return _bad(state, FailureReason.DUPLICATE_ID, sid, count)
        parent = value.get("parentId")
        if parent is not None and (not isinstance(parent, str) or parent not in ids):
            return _bad(state, FailureReason.UNKNOWN_PARENT, sid, count)
        if typ == "message":
            msg = value.get("message")
            if not isinstance(msg, dict) or msg.get("role") not in _ROLES:
                return _bad(state, FailureReason.INVALID_SCHEMA, sid, count)
            role = msg["role"]
            required = {
                "user": ("content", "timestamp"),
                "assistant": (
                    "content",
                    "api",
                    "provider",
                    "model",
                    "usage",
                    "stopReason",
                    "timestamp",
                ),
                "toolResult": ("content", "toolCallId", "toolName", "isError", "timestamp"),
                "custom": ("content", "customType", "display", "timestamp"),
                "bashExecution": ("command", "output", "cancelled", "truncated", "timestamp"),
            }[role]
            if any(key not in msg for key in required):
                return _bad(state, FailureReason.INVALID_SCHEMA, sid, count)
            if role == "custom" and not isinstance(msg["display"], bool):
                return _bad(state, FailureReason.INVALID_SCHEMA, sid, count)
            typed = {
                "user": (("timestamp", _json_number),),
                "assistant": (
                    ("api", str),
                    ("provider", str),
                    ("model", str),
                    ("usage", dict),
                    ("stopReason", str),
                    ("timestamp", _json_number),
                ),
                "toolResult": (
                    ("toolCallId", str),
                    ("toolName", str),
                    ("isError", bool),
                    ("timestamp", _json_number),
                ),
                "custom": (("customType", str), ("display", bool), ("timestamp", _json_number)),
                "bashExecution": (
                    ("command", str),
                    ("output", str),
                    ("cancelled", bool),
                    ("truncated", bool),
                    ("timestamp", _json_number),
                ),
            }.get(role, ())
            if any(
                not isinstance(msg[key], expected)
                if isinstance(expected, type) or isinstance(expected, tuple)
                else not expected(msg[key])
                for key, expected in typed
            ):
                return _bad(state, FailureReason.INVALID_SCHEMA, sid, count)
            if "content" in msg:
                content = msg["content"]
                if role == "user" and not _user_content(content):
                    return _bad(state, FailureReason.INVALID_SCHEMA, sid, count)
                if role in {"assistant", "toolResult"} and not isinstance(content, list):
                    return _bad(state, FailureReason.INVALID_SCHEMA, sid, count)
                if role == "custom" and not _user_content(content):
                    return _bad(state, FailureReason.INVALID_SCHEMA, sid, count)
            if (
                role == "bashExecution"
                and "exitCode" in msg
                and (not _json_number(msg["exitCode"]) or not isinstance(msg["exitCode"], int))
            ):
                return _bad(state, FailureReason.INVALID_SCHEMA, sid, count)
            if role == "assistant" and msg["stopReason"] not in {
                "stop",
                "length",
                "toolUse",
                "error",
                "aborted",
            }:
                return _bad(state, FailureReason.INVALID_SCHEMA, sid, count)
        if typ == "custom_message" and (
            not isinstance(value.get("customType"), str)
            or not value["customType"]
            or "content" not in value
            or not isinstance(value.get("display"), bool)
            or not _user_content(value["content"])
        ):
            return _bad(state, FailureReason.INVALID_SCHEMA, sid, count)
        if typ == "custom" and (
            not isinstance(value.get("customType"), str) or not value["customType"]
        ):
            return _bad(state, FailureReason.INVALID_SCHEMA, sid, count)
        required_top = {
            "model_change": ("provider", "modelId"),
            "thinking_level_change": ("thinkingLevel",),
            "compaction": ("summary", "firstKeptEntryId", "tokensBefore"),
            "branch_summary": ("fromId", "summary"),
            "label": ("targetId",),
            "session_info": (),
        }.get(typ, ())
        if any(key not in value for key in required_top):
            return _bad(state, FailureReason.INVALID_SCHEMA, sid, count)
        top_types = {
            "model_change": (("provider", str), ("modelId", str)),
            "thinking_level_change": (("thinkingLevel", str),),
            "compaction": (("summary", str), ("tokensBefore", (int, float))),
            "branch_summary": (("summary", str),),
            "label": (("targetId", str),),
            "session_info": (),
            "custom_message": (("customType", str), ("display", bool)),
            "custom": (("customType", str),),
        }.get(typ, ())
        if any(not isinstance(value[key], expected) for key, expected in top_types):
            return _bad(state, FailureReason.INVALID_SCHEMA, sid, count)
        if typ == "label" and "label" in value and not isinstance(value["label"], str):
            return _bad(state, FailureReason.INVALID_SCHEMA, sid, count)
        if typ == "session_info" and "name" in value and not isinstance(value["name"], str):
            return _bad(state, FailureReason.INVALID_SCHEMA, sid, count)
        if typ == "compaction" and not _json_number(value["tokensBefore"]):
            return _bad(state, FailureReason.INVALID_SCHEMA, sid, count)
        for ref in ("firstKeptEntryId", "fromId", "targetId"):
            if ref in value and (not isinstance(value[ref], str) or value[ref] not in ids):
                return _bad(state, FailureReason.UNKNOWN_REFERENCE, sid, count)
        ids.add(entry_id)
        count += 1
        if count > max_entries:
            return _bad(state, FailureReason.TOO_MANY_ENTRIES, sid, count - 1)
    if sid is None:
        return _bad(CaptureState.UNAVAILABLE, FailureReason.INVALID_SCHEMA, None, 0)
    if state is CaptureState.COMPLETE:
        if not data.endswith(b"\n"):
            return _bad(state, FailureReason.INVALID_JSON, sid, count)
        # A v3 header is not an entry and therefore cannot be a tree root.
        roots = 0
        for raw in data.splitlines()[1:]:
            value = _strict_json(raw)
            if isinstance(value, dict) and value.get("parentId") is None:
                roots += 1
        if count == 0 or roots != 1:
            return _bad(state, FailureReason.INVALID_SCHEMA, sid, count)
    return SessionValidation(state=state, session_id=sid, entry_count=count)


def _prompt_record(
    root: Path | PinnedDirectory,
    relative_path: str,
    kind: Literal["system", "initial"],
    session_id: str,
) -> PromptContextRecord:
    data = read_prompt_context(root, relative_path)
    return PromptContextRecord(
        kind=kind,
        relative_path=relative_path,
        byte_count=len(data),
        sha256=_digest(data),
        session_id=session_id,
    )


def receipt_for_session(
    root: Path | PinnedDirectory,
    relative_path: str,
    *,
    state: CaptureState = CaptureState.COMPLETE,
    reason: FailureReason | None = None,
    max_bytes: int = MAX_SESSION_BYTES,
    max_entries: int = MAX_SESSION_ENTRIES,
) -> NativeSessionReceipt:
    data = read_session_bytes(root, relative_path, max_bytes=max_bytes)
    validation = validate_native_session(
        data, state=state, max_entries=max_entries, max_bytes=max_bytes
    )
    if validation.reason is not None and not (
        state is CaptureState.PARTIAL and validation.reason is FailureReason.INVALID_JSON
    ):
        raise ValueError(validation.reason.value)
    if validation.session_id is None:
        raise ValueError(FailureReason.INVALID_SCHEMA.value)
    return NativeSessionReceipt(
        state=state,
        session_id=validation.session_id,
        relative_path=relative_path,
        byte_count=len(data),
        entry_count=validation.entry_count,
        sha256=_digest(data),
        system_prompt=_prompt_record(root, "pi-prompt/system.txt", "system", validation.session_id),
        initial_prompt=_prompt_record(
            root, "pi-prompt/initial.txt", "initial", validation.session_id
        ),
        reason=reason or validation.reason,
    )


def compare_receipt(root: Path | PinnedDirectory, receipt: NativeSessionReceipt) -> bool:
    if receipt.state is CaptureState.UNAVAILABLE or receipt.relative_path is None:
        return False
    try:
        return (
            receipt_for_session(
                root, receipt.relative_path, state=receipt.state, reason=receipt.reason
            )
            == receipt
        )
    except (ValueError, TopologyError):
        return False


def write_prompt_context(
    root: Path | PinnedDirectory,
    kind: Literal["system", "initial"],
    content: bytes,
    session_id: str,
) -> PromptContextRecord:
    pinned, owned = _root(root)
    try:
        try:
            pinned.mkdir("pi-prompt")
        except TopologyError:
            raise ValueError(FailureReason.UNSAFE_PATH.value)
        directory = pinned.open_relative("pi-prompt")
        name = f"{kind}.txt"
        fd = os.open(
            name, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o600, dir_fd=directory.fd
        )
        try:
            view = memoryview(content)
            while view:
                view = view[os.write(fd, view) :]
        finally:
            os.close(fd)
        directory.close()
        return PromptContextRecord(
            kind=kind,
            relative_path=f"pi-prompt/{name}",
            byte_count=len(content),
            sha256=_digest(content),
            session_id=session_id,
        )
    finally:
        if owned:
            pinned.close()


def verify_prompt_context(root: Path | PinnedDirectory, record: PromptContextRecord) -> bool:
    try:
        data = read_prompt_context(root, record.relative_path)
        return len(data) == record.byte_count and _digest(data) == record.sha256
    except (ValueError, TopologyError):
        return False


def read_prompt_context(root: Path | PinnedDirectory, relative_path: str) -> bytes:
    pinned, owned = _root(root)
    try:
        parts = relative_path.split("/")
        if len(parts) != 2 or parts[0] != "pi-prompt":
            raise ValueError(FailureReason.UNSAFE_PATH.value)
        try:
            directory = pinned.open_relative(parts[0])
            fd = os.open(parts[1], os.O_RDONLY | os.O_NOFOLLOW, dir_fd=directory.fd)
        except (OSError, TopologyError) as exc:
            raise ValueError(FailureReason.UNSAFE_FILE.value) from exc
        try:
            info = os.fstat(fd)
            if (
                info.st_uid != os.getuid()
                or not stat.S_ISREG(info.st_mode)
                or stat.S_IMODE(info.st_mode) & 0o077
                or info.st_nlink != 1
            ):
                raise ValueError(FailureReason.UNSAFE_FILE.value)
            data = b""
            while len(data) < info.st_size:
                chunk = os.read(fd, min(1024 * 1024, info.st_size - len(data)))
                if not chunk:
                    raise ValueError(FailureReason.UNREADABLE.value)
                data += chunk
            after = os.fstat(fd)
            if (after.st_dev, after.st_ino, after.st_size) != (
                info.st_dev,
                info.st_ino,
                info.st_size,
            ):
                raise ValueError(FailureReason.SOURCE_MUTATED.value)
            return data
        finally:
            os.close(fd)
            directory.close()
    finally:
        if owned:
            pinned.close()


def _validate_package(node: Path, package_root: Path | None) -> tuple[Path, Path]:
    if (
        not node.is_absolute()
        or node.is_symlink()
        or not node.is_file()
        or not os.access(node, os.X_OK)
    ):
        raise ValueError(FailureReason.EXECUTABLE_UNSAFE.value)
    root = package_root or node.parent
    manifest = root / "package.json"
    cli = root / "dist" / "cli.js"
    try:
        package = json.loads(manifest.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError):
        raise ValueError(FailureReason.EXECUTABLE_UNSAFE.value)
    if (
        package.get("version") != PACKAGE_VERSION
        or package.get("bin", {}).get("pi") not in ("dist/cli.js", "./dist/cli.js")
        or cli.is_symlink()
        or not cli.is_file()
    ):
        raise ValueError(FailureReason.EXECUTABLE_UNSAFE.value)
    return node, cli


def _snapshot_input(path: Path) -> tuple[bytes, int, tuple[int, int, int]]:
    try:
        fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC)
    except (FileNotFoundError, PermissionError, OSError) as exc:
        raise ValueError(FailureReason.UNSAFE_FILE.value) from exc
    try:
        info = os.fstat(fd)
        if info.st_uid != os.getuid() or not stat.S_ISREG(info.st_mode) or info.st_nlink != 1:
            raise ValueError(FailureReason.UNSAFE_FILE.value)
        data = b""
        while len(data) < info.st_size:
            chunk = os.read(fd, min(1024 * 1024, info.st_size - len(data)))
            if not chunk:
                raise ValueError(FailureReason.UNREADABLE.value)
            data += chunk
        after = os.fstat(fd)
        if (after.st_dev, after.st_ino, after.st_size) != (info.st_dev, info.st_ino, info.st_size):
            raise ValueError(FailureReason.SOURCE_MUTATED.value)
        return data, fd, (info.st_dev, info.st_ino, info.st_size)
    except BaseException:
        os.close(fd)
        raise


def _snapshot_pinned_input(
    root: PinnedDirectory, relative_path: str
) -> tuple[bytes, int, tuple[int, int, int]]:
    """Snapshot a session file through the already admitted directory fd."""
    _, basename = _safe_relative(relative_path)
    directory = root.open_relative("pi-session")
    try:
        try:
            fd = os.open(basename, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC, dir_fd=directory.fd)
        except OSError as exc:
            raise ValueError(FailureReason.UNSAFE_FILE.value) from exc
        try:
            info = os.fstat(fd)
            if (
                info.st_uid != os.getuid()
                or not stat.S_ISREG(info.st_mode)
                or stat.S_IMODE(info.st_mode) & 0o077
                or info.st_nlink != 1
            ):
                raise ValueError(FailureReason.UNSAFE_FILE.value)
            data = b""
            while len(data) < info.st_size:
                chunk = os.read(fd, min(1024 * 1024, info.st_size - len(data)))
                if not chunk:
                    raise ValueError(FailureReason.UNREADABLE.value)
                data += chunk
            after = os.fstat(fd)
            identity = (info.st_dev, info.st_ino, info.st_size)
            if (after.st_dev, after.st_ino, after.st_size) != identity:
                raise ValueError(FailureReason.SOURCE_MUTATED.value)
            return data, fd, identity
        except BaseException:
            os.close(fd)
            raise
    finally:
        directory.close()


def export_session(
    node: Path,
    input_path: Path | PinnedDirectory,
    *,
    source_relative_path: str | None = None,
    package_root: Path | None = None,
    timeout: float = 30.0,
) -> bytes:
    executable, cli = _validate_package(Path(node), package_root)
    if isinstance(input_path, PinnedDirectory):
        if source_relative_path is None:
            raise ValueError(FailureReason.UNSAFE_PATH.value)
        source, source_fd, source_identity = _snapshot_pinned_input(
            input_path, source_relative_path
        )
    else:
        if source_relative_path is not None:
            raise ValueError(FailureReason.UNSAFE_PATH.value)
        source, source_fd, source_identity = _snapshot_input(Path(input_path))
    try:
        with tempfile.TemporaryDirectory(prefix="pi-export-") as temp:
            snapshot = Path(temp) / "session.jsonl"
            snapshot.write_bytes(source)
            snapshot.chmod(0o400)
            output = Path(temp) / "export.html"
            try:
                result = subprocess.run(
                    [str(executable), str(cli), "--export", str(snapshot), str(output)],
                    shell=False,
                    check=False,
                    timeout=timeout,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )
            except subprocess.TimeoutExpired as exc:
                raise ValueError(FailureReason.TIMEOUT.value) from exc
            if result.returncode != 0 or not output.is_file() or not output.stat().st_size:
                raise ValueError(FailureReason.EXPORT_FAILED.value)
            os.lseek(source_fd, 0, os.SEEK_SET)
            retained = b""
            while len(retained) < len(source):
                chunk = os.read(source_fd, min(1024 * 1024, len(source) - len(retained)))
                if not chunk:
                    raise ValueError(FailureReason.SOURCE_MUTATED.value)
                retained += chunk
            after = os.fstat(source_fd)
            if (after.st_dev, after.st_ino, after.st_size) != source_identity or _digest(
                retained
            ) != _digest(source):
                raise ValueError(FailureReason.SOURCE_MUTATED.value)
            snapshot_bytes = snapshot.read_bytes()
            if len(snapshot_bytes) != len(source) or _digest(snapshot_bytes) != _digest(source):
                raise ValueError(FailureReason.SOURCE_MUTATED.value)
            return output.read_bytes()
    finally:
        os.close(source_fd)


_EXPORT_FAILURE_REASONS: dict[FailureReason, str] = {
    FailureReason.EXPORT_FAILED: "pinned export command failed",
    FailureReason.TIMEOUT: "pinned export command timed out",
    FailureReason.EXECUTABLE_UNSAFE: "pinned executable identity was not verified",
    FailureReason.SOURCE_MUTATED: "source changed during export",
    FailureReason.UNSAFE_PATH: "native session path was not admitted",
    FailureReason.UNSAFE_FILE: "native session file was unsafe",
    FailureReason.MISSING: "native session was missing",
}


def _export_failure_code(error: Exception) -> FailureReason:
    """Translate internal errors to the intentionally bounded public vocabulary."""
    value = str(error)
    for reason in FailureReason:
        if value == reason.value and reason in _EXPORT_FAILURE_REASONS:
            return reason
    return FailureReason.EXPORT_FAILED


def verify_export_session(
    node: Path,
    input_path: Path | PinnedDirectory,
    *,
    source_relative_path: str | None = None,
    package_root: Path | None = None,
    timeout: float = 30.0,
) -> ExportVerificationRecord:
    """Run the pinned export and return a content-free outcome on every failure.

    The verification API is intentionally stricter than :func:`export_session`:
    persistence requires an admitted ``pi-session/*.jsonl`` path, so a direct
    host ``Path`` is rejected rather than copied into the record.  Temporary
    HTML is created and cleaned by ``export_session`` and is never returned.
    """
    relative_path: str | None = None
    before_count: int | None = None
    before_hash: str | None = None
    after_count: int | None = None
    after_hash: str | None = None
    cli_verified = False
    output_produced = False
    output_disposed = False
    export_started = False
    failure: FailureReason | None = None
    pinned_input: PinnedDirectory | None = None
    owns_pinned_input = False

    try:
        if source_relative_path is None:
            raise ValueError(FailureReason.UNSAFE_PATH.value)
        if isinstance(input_path, PinnedDirectory):
            pinned_input = input_path
        else:
            pinned_input = PinnedDirectory.open(Path(input_path), create=False)
            owns_pinned_input = True
        assert pinned_input is not None
        _safe_relative(source_relative_path)
        relative_path = source_relative_path
        _validate_package(Path(node), package_root)
        cli_verified = True
        source, source_fd, _ = _snapshot_pinned_input(pinned_input, relative_path)
        os.close(source_fd)
        before_count = len(source)
        before_hash = _digest(source)
        export_started = True
        exported = export_session(
            Path(node),
            pinned_input,
            source_relative_path=relative_path,
            package_root=package_root,
            timeout=timeout,
        )
        output_produced = bool(exported)
    except Exception as exc:
        failure = _export_failure_code(exc)
    finally:
        if export_started:
            # export_session's TemporaryDirectory has exited on both return and
            # exception, so this is a fact about disposal, not retained output.
            output_disposed = True
        if pinned_input is not None and relative_path is not None:
            try:
                current, current_fd, _ = _snapshot_pinned_input(pinned_input, relative_path)
                os.close(current_fd)
                after_count = len(current)
                after_hash = _digest(current)
            except Exception:
                pass
        if owns_pinned_input and pinned_input is not None:
            pinned_input.close()

    if failure is None and (
        before_hash is None
        or after_hash is None
        or before_count != after_count
        or before_hash != after_hash
    ):
        failure = FailureReason.SOURCE_MUTATED
    if failure is not None:
        return ExportVerificationRecord(
            status=ExportVerificationStatus.FAILED,
            relative_path=relative_path,
            source_byte_count_before=before_count,
            source_byte_count_after=after_count,
            source_sha256_before=before_hash,
            source_sha256_after=after_hash,
            cli_identity_verified=cli_verified,
            disposable_html_output_produced=output_produced,
            disposable_html_output_disposed=output_disposed,
            failure_code=failure,
            failure_reason=_EXPORT_FAILURE_REASONS[failure],
        )
    return ExportVerificationRecord(
        status=ExportVerificationStatus.SUCCEEDED,
        relative_path=relative_path,
        source_byte_count_before=before_count,
        source_byte_count_after=after_count,
        source_sha256_before=before_hash,
        source_sha256_after=after_hash,
        cli_identity_verified=cli_verified,
        disposable_html_output_produced=output_produced,
        disposable_html_output_disposed=output_disposed,
    )


SessionReceipt = NativeSessionReceipt
PromptSidecar = PromptContextSidecar
NativeSessionValidation = SessionValidation
SessionCaptureState = CaptureState
validate_session = validate_native_session
verify_export = export_session
ExportVerification = ExportVerificationRecord
