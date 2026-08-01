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
"""Contract tests for the private Pi session viewer."""

from __future__ import annotations

from collections.abc import Iterator
from contextlib import contextmanager
import hashlib
import json
from pathlib import Path
import stat
from typing import Any

import pytest
import requests

from . import session_viewer
from .native_session import (
    CaptureState,
    FailureReason,
    NativeSessionReceipt,
    PromptContextRecord,
    receipt_for_session,
)
from .session_viewer import (
    VIEWER_ASSET_ROOT,
    SessionViewerError,
    StagedSession,
    ViewerServer,
    admit_attempt,
    build_view_model,
    serve_staged_session,
    stage_session,
    view_attempt,
)


def _line(value: dict[str, Any]) -> bytes:
    return json.dumps(value, separators=(",", ":")).encode() + b"\n"


def _header() -> dict[str, Any]:
    return {
        "type": "session",
        "version": 3,
        "id": "session-1",
        "timestamp": "2026-07-23T12:00:00Z",
        "cwd": "/private/workspace",
    }


def _entry(
    entry_id: str,
    parent_id: str | None,
    entry_type: str,
    **values: Any,
) -> dict[str, Any]:
    return {
        "type": entry_type,
        "id": entry_id,
        "parentId": parent_id,
        "timestamp": f"2026-07-23T12:00:{len(entry_id):02d}Z",
        **values,
    }


def _user_message(text: str) -> dict[str, Any]:
    return {"role": "user", "content": text, "timestamp": 1_753_272_000_000}


def _assistant_message() -> dict[str, Any]:
    return {
        "role": "assistant",
        "content": [
            {"type": "thinking", "thinking": "Inspect the map.", "thinkingSignature": "secret"},
            {"type": "text", "text": "I found the object."},
            {
                "type": "toolCall",
                "id": "call-1",
                "name": "locate",
                "arguments": {"query": "mug"},
            },
            {"type": "image", "data": "base64-secret", "mimeType": "image/png"},
        ],
        "api": "responses",
        "provider": "openai",
        "model": "gpt-test",
        "usage": {
            "input": 10,
            "output": 5,
            "cacheRead": 2,
            "cacheWrite": 1,
            "reasoning": 3,
            "totalTokens": 21,
            "cost": {"total": 0.01},
        },
        "stopReason": "toolUse",
        "timestamp": 1_753_272_001_000,
    }


def _all_entries() -> list[dict[str, Any]]:
    return [
        _entry("e1", None, "message", message=_user_message("Find the mug.")),
        _entry("e2", "e1", "message", message=_assistant_message()),
        _entry(
            "e3",
            "e2",
            "message",
            message={
                "role": "toolResult",
                "content": [
                    {"type": "text", "text": "mug at (1, 2)"},
                    {"type": "image", "data": "hidden", "mimeType": "image/jpeg"},
                ],
                "toolCallId": "call-1",
                "toolName": "locate",
                "isError": False,
                "details": {"location": [1, 2], "thinkingSignature": "secret"},
                "timestamp": 1_753_272_002_000,
            },
        ),
        _entry(
            "e4",
            "e3",
            "message",
            message={
                "role": "custom",
                "content": "custom note",
                "customType": "note",
                "display": True,
                "timestamp": 1_753_272_003_000,
            },
        ),
        _entry(
            "e5",
            "e4",
            "message",
            message={
                "role": "bashExecution",
                "command": "true",
                "output": "done",
                "cancelled": False,
                "truncated": False,
                "exitCode": 0,
                "timestamp": 1_753_272_004_000,
            },
        ),
        _entry("e6", "e5", "model_change", provider="openai", modelId="gpt-next"),
        _entry("e7", "e6", "thinking_level_change", thinkingLevel="high"),
        _entry(
            "e8",
            "e7",
            "compaction",
            summary="Earlier work",
            firstKeptEntryId="e2",
            tokensBefore=100,
        ),
        _entry("e9", "e8", "branch_summary", fromId="e3", summary="Alternate path"),
        _entry("e10", "e9", "custom", customType="metadata", data={"hidden": True}),
        _entry(
            "e11",
            "e10",
            "custom_message",
            customType="notice",
            content="A custom message",
            display=True,
        ),
        _entry("e12", "e11", "label", targetId="e2", label="checkpoint"),
        _entry("e13", "e12", "session_info", name="Review session"),
        _entry("branch", "e2", "message", message=_user_message("Try another route.")),
    ]


def _source(entries: list[dict[str, Any]] | None = None) -> bytes:
    return b"".join(_line(value) for value in [_header(), *(entries or _all_entries())])


def _write_private(path: Path, data: bytes) -> None:
    path.write_bytes(data)
    path.chmod(0o600)


def _asset_root(path: Path) -> Path:
    (path / "assets").mkdir(parents=True)
    files = {
        "index.html": b"<main>viewer</main>",
        "assets/app.js": b"console.log('local')",
        "assets/app.css": b"main{}",
    }
    for relative, content in files.items():
        (path / relative).write_bytes(content)
    manifest = {
        "schema_version": "1.0",
        "files": {
            relative: hashlib.sha256(content).hexdigest() for relative, content in files.items()
        },
    }
    (path / "asset-manifest.json").write_text(json.dumps(manifest))
    return path


@contextmanager
def _attempt(
    tmp_path: Path,
    *,
    state: CaptureState = CaptureState.COMPLETE,
    source: bytes | None = None,
) -> Iterator[tuple[Path, Path, NativeSessionReceipt]]:
    attempt = tmp_path / "attempt"
    private = attempt / "private"
    session_dir = private / "pi-session"
    prompt_dir = private / "pi-prompt"
    session_dir.mkdir(parents=True, mode=0o700)
    prompt_dir.mkdir(mode=0o700)
    session_source = source or _source()
    _write_private(session_dir / "session.jsonl", session_source)
    _write_private(prompt_dir / "system.txt", b"system")
    _write_private(prompt_dir / "initial.txt", b"initial")
    receipt = receipt_for_session(
        private,
        "pi-session/session.jsonl",
        state=state,
        reason=FailureReason.INVALID_JSON if state is CaptureState.PARTIAL else None,
    )
    _write_private(private / "native-session-receipt.v1.json", receipt.model_dump_json().encode())
    yield attempt, private, receipt


def test_view_model_covers_native_entry_types_and_branches() -> None:
    source = _source()
    digest = hashlib.sha256(source).hexdigest()
    prompt = PromptContextRecord(
        kind="system",
        relative_path="pi-prompt/system.txt",
        byte_count=0,
        sha256=hashlib.sha256(b"").hexdigest(),
        session_id="session-1",
    )
    receipt = NativeSessionReceipt(
        state=CaptureState.COMPLETE,
        session_id="session-1",
        relative_path="pi-session/session.jsonl",
        byte_count=len(source),
        entry_count=len(_all_entries()),
        sha256=digest,
        system_prompt=prompt,
        initial_prompt=prompt.model_copy(
            update={"kind": "initial", "relative_path": "pi-prompt/initial.txt"}
        ),
    )

    model = build_view_model(source, receipt)

    assert model["summary"]["status"] == "complete"
    assert model["summary"]["model"] == "gpt-next"
    assert model["summary"]["totalUsage"]["totalTokens"] == 21
    assert model["summary"]["branchCount"] == 1
    assert set(model["headIds"]) == {"e13", "branch"}
    assert {entry["type"] for entry in model["entries"]} == {
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
    encoded = json.dumps(model)
    assert "base64-secret" not in encoded
    assert "thinkingSignature" in encoded
    assert '"[omitted]"' in encoded
    assert "Find the mug." in encoded
    assert "mug at (1, 2)" in encoded
    tool_result = model["entries"][2]["parts"][0]
    assert tool_result["details"] == {
        "location": [1, 2],
        "thinkingSignature": "[omitted]",
    }


def test_valid_partial_tail_is_admitted_and_rendered(tmp_path: Path) -> None:
    partial = _source() + b'{"type":"message","id":"truncated"'
    with _attempt(tmp_path, state=CaptureState.PARTIAL, source=partial) as (attempt, _, _):
        admitted = admit_attempt(attempt)
        try:
            model = build_view_model(admitted.source, admitted.receipt)
            assert model["summary"]["status"] == "partial"
            assert len(model["entries"]) == len(_all_entries())
        finally:
            admitted.close()


def test_terminal_private_mode_directory_is_accepted_directly(tmp_path: Path) -> None:
    with _attempt(tmp_path) as (_, private, _):
        admitted = admit_attempt(private)
        try:
            assert admitted.receipt.state is CaptureState.COMPLETE
        finally:
            admitted.close()


def test_unavailable_and_receipt_mismatch_are_rejected(tmp_path: Path) -> None:
    with _attempt(tmp_path) as (attempt, private, receipt):
        unavailable = NativeSessionReceipt(
            state=CaptureState.UNAVAILABLE,
            reason=FailureReason.MISSING,
        )
        _write_private(
            private / "native-session-receipt.v1.json",
            unavailable.model_dump_json().encode(),
        )
        with pytest.raises(SessionViewerError, match="evidence_unavailable"):
            admit_attempt(attempt)

        changed = receipt.model_copy(update={"sha256": "0" * 64})
        _write_private(
            private / "native-session-receipt.v1.json",
            changed.model_dump_json().encode(),
        )
        with pytest.raises(SessionViewerError, match="receipt_mismatch"):
            admit_attempt(attempt)


def test_unsafe_receipt_permissions_are_rejected(tmp_path: Path) -> None:
    with _attempt(tmp_path) as (attempt, private, _):
        receipt_path = private / "native-session-receipt.v1.json"
        receipt_path.chmod(0o644)
        with pytest.raises(SessionViewerError, match="invalid_evidence"):
            admit_attempt(attempt)
        _write_private(receipt_path, b"{")
        with pytest.raises(SessionViewerError, match="invalid_evidence"):
            admit_attempt(attempt)


def test_path_escape_symlink_and_oversized_receipts_are_rejected(tmp_path: Path) -> None:
    with _attempt(tmp_path) as (attempt, private, receipt):
        escaped = receipt.model_copy(update={"relative_path": "../session.jsonl"})
        _write_private(
            private / "native-session-receipt.v1.json",
            escaped.model_dump_json().encode(),
        )
        with pytest.raises(SessionViewerError, match="receipt_mismatch"):
            admit_attempt(attempt)

    target = tmp_path / "real"
    target.mkdir()
    symlink = tmp_path / "linked-attempt"
    symlink.symlink_to(target, target_is_directory=True)
    with pytest.raises(SessionViewerError, match="invalid_evidence"):
        admit_attempt(symlink)

    with _attempt(tmp_path / "large") as (attempt, private, receipt):
        oversized = private / "pi-session" / "session.jsonl"
        with oversized.open("r+b") as stream:
            stream.truncate(65 * 1024 * 1024)
        too_large = receipt.model_copy(update={"byte_count": 65 * 1024 * 1024, "sha256": "0" * 64})
        _write_private(
            private / "native-session-receipt.v1.json",
            too_large.model_dump_json().encode(),
        )
        with pytest.raises(SessionViewerError, match="receipt_mismatch"):
            admit_attempt(attempt)


def test_staging_is_private_exact_and_excludes_sidecars(tmp_path: Path) -> None:
    with _attempt(tmp_path) as (attempt, _, _):
        admitted = admit_attempt(attempt)
        try:
            with stage_session(admitted) as staged:
                root = staged.root
                assert stat.S_IMODE(root.stat().st_mode) == 0o700
                assert stat.S_IMODE(staged.source_copy_path.stat().st_mode) == 0o600
                assert staged.source_copy_path.read_bytes() == admitted.source
                assert {path.name for path in root.iterdir()} == {
                    "session.jsonl",
                    "session.json",
                }
            assert not root.exists()
        finally:
            admitted.close()


def test_staged_copy_mutation_fails_and_still_cleans_up(tmp_path: Path) -> None:
    with _attempt(tmp_path) as (attempt, _, _):
        admitted = admit_attempt(attempt)
        root: Path | None = None
        try:
            with pytest.raises(SessionViewerError, match="staged_copy_mutated"):
                with stage_session(admitted) as staged:
                    root = staged.root
                    staged.source_copy_path.write_bytes(b"changed")
            assert root is not None and not root.exists()
        finally:
            admitted.close()


def test_cleanup_failure_is_bounded_after_removing_the_stage(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    with _attempt(tmp_path) as (attempt, _, _):
        admitted = admit_attempt(attempt)
        original_rmtree = session_viewer.shutil.rmtree

        def remove_then_fail(path: Path) -> None:
            original_rmtree(path)
            raise OSError

        monkeypatch.setattr(session_viewer.shutil, "rmtree", remove_then_fail)
        try:
            with pytest.raises(SessionViewerError, match="cleanup_failed"):
                with stage_session(admitted):
                    pass
        finally:
            admitted.close()


def test_server_is_token_scoped_loopback_get_head_only_and_hardened(
    tmp_path: Path,
) -> None:
    assets = _asset_root(tmp_path / "assets-root")

    with _attempt(tmp_path / "data") as (attempt, _, _):
        admitted = admit_attempt(attempt)
        try:
            with stage_session(admitted) as staged:
                with serve_staged_session(staged, asset_root=assets) as server:
                    assert server.httpd.server_address[0] == "127.0.0.1"
                    assert server.httpd.server_address[1] != 0
                    response = requests.get(server.url, timeout=2)
                    assert response.status_code == 200
                    assert response.headers["Cache-Control"] == "no-store"
                    assert "default-src 'none'" in response.headers["Content-Security-Policy"]
                    assert requests.head(f"{server.url}session.json", timeout=2).status_code == 200
                    assert (
                        requests.get(
                            server.url.replace(urlsplit_path(server.url), "/wrong/"),
                            timeout=2,
                        ).status_code
                        == 404
                    )
                    assert requests.post(server.url, timeout=2).status_code == 405
                    assert requests.get(f"{server.url}?debug=1", timeout=2).status_code == 404
                    assert (
                        requests.get(f"{server.url}%2e%2e/session.json", timeout=2).status_code
                        == 404
                    )
                    assert (
                        requests.get(f"{server.url}asset-manifest.json", timeout=2).status_code
                        == 404
                    )
                assert not server.thread.is_alive()
        finally:
            admitted.close()


def urlsplit_path(url: str) -> str:
    return "/" + url.split("/", 3)[3]


def test_source_mutation_is_detected(tmp_path: Path) -> None:
    with _attempt(tmp_path) as (attempt, private, _):
        admitted = admit_attempt(attempt)
        try:
            session_path = private / "pi-session" / "session.jsonl"
            _write_private(session_path, _source() + b" ")
            with pytest.raises(SessionViewerError, match="source_mutated"):
                admitted.verify_unchanged()
        finally:
            admitted.close()


def test_missing_assets_are_rejected(tmp_path: Path) -> None:
    staged_root = tmp_path / "stage"
    staged_root.mkdir()
    source = staged_root / "session.jsonl"
    view_model = staged_root / "session.json"
    source.write_text("")
    view_model.write_text("{}")
    staged = StagedSession(staged_root, view_model, source, "0" * 64)
    with pytest.raises(SessionViewerError, match="assets_unavailable"):
        with serve_staged_session(staged, asset_root=tmp_path / "missing"):
            pass


def test_packaged_assets_are_manifest_bound_and_contain_no_outbound_hooks(
    tmp_path: Path,
) -> None:
    staged_root = tmp_path / "stage"
    staged_root.mkdir()
    source = staged_root / "session.jsonl"
    view_model = staged_root / "session.json"
    source.write_text("")
    view_model.write_text("{}")
    staged = StagedSession(staged_root, view_model, source, "0" * 64)
    forbidden = (
        b"api_key",
        b"authorization",
        b"analytics",
        b"telemetry",
        b"api.openai.com",
        b"anthropic.com",
        b"websocket",
        b"xmlhttprequest",
        b"eventsource",
        b"sendbeacon",
    )
    for path in VIEWER_ASSET_ROOT.rglob("*"):
        if path.suffix in {".html", ".js", ".css"}:
            content = path.read_bytes().lower()
            assert all(marker not in content for marker in forbidden)

    copied = _asset_root(tmp_path / "copied-assets")
    (copied / "assets" / "app.js").write_text("changed")
    with pytest.raises(SessionViewerError, match="assets_unavailable"):
        with serve_staged_session(staged, asset_root=copied):
            pass


def test_server_startup_and_runtime_failures_are_bounded(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    assets = _asset_root(tmp_path / "assets-root")
    staged_root = tmp_path / "stage"
    staged_root.mkdir()
    source = staged_root / "session.jsonl"
    view_model = staged_root / "session.json"
    source.write_text("")
    view_model.write_text("{}")
    staged = StagedSession(staged_root, view_model, source, "0" * 64)

    def fail_start(*_args: Any, **_kwargs: Any) -> None:
        raise OSError

    monkeypatch.setattr(session_viewer, "_LoopbackHTTPServer", fail_start)
    with pytest.raises(SessionViewerError, match="server_start_failed"):
        with serve_staged_session(staged, asset_root=assets):
            pass

    monkeypatch.undo()
    with serve_staged_session(staged, asset_root=assets) as server:
        server.failures.append(RuntimeError("content must not escape"))
        server.httpd.shutdown()
        server.thread.join(timeout=2)
        with pytest.raises(SessionViewerError, match="server_failed"):
            server.wait()


@pytest.mark.parametrize(
    ("interrupted", "state"),
    [(False, CaptureState.COMPLETE), (True, CaptureState.PARTIAL)],
)
def test_attached_viewer_normal_and_interrupted_exit_clean_up(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    interrupted: bool,
    state: CaptureState,
    capsys: pytest.CaptureFixture[str],
) -> None:
    roots: list[Path] = []
    original_mkdtemp = session_viewer.tempfile.mkdtemp

    def tracked_mkdtemp(*args: Any, **kwargs: Any) -> str:
        path = Path(original_mkdtemp(*args, **kwargs))
        roots.append(path)
        return str(path)

    def finish(server: ViewerServer) -> None:
        if interrupted:
            raise KeyboardInterrupt
        server.httpd.shutdown()

    monkeypatch.setattr(session_viewer.tempfile, "mkdtemp", tracked_mkdtemp)
    monkeypatch.setattr(ViewerServer, "wait", finish)
    source = _source()
    if state is CaptureState.PARTIAL:
        source += b'{"type":"message","id":"truncated"'
    with _attempt(tmp_path, state=state, source=source) as (attempt, _, _):
        assert view_attempt(attempt, open_browser=False) == 0

    output = capsys.readouterr().out
    assert output.startswith("http://127.0.0.1:")
    assert roots and all(not root.exists() for root in roots)
