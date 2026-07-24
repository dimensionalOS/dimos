from __future__ import annotations

import json
from pathlib import Path

import pytest

from .native_session import (
    CaptureState,
    ExportVerificationStatus,
    FailureReason,
    NativeSessionReceipt,
    export_session,
    make_prompt_sidecar,
    read_prompt_context,
    validate_native_session,
    verify_export_session,
    verify_prompt_context,
    verify_prompt_sidecar,
    write_prompt_context,
)


def session(*entries: dict[str, object]) -> bytes:
    header = {"type": "session", "version": 3, "id": "header", "timestamp": "t", "cwd": "/"}
    return ("\n".join(json.dumps(item) for item in (header, *entries)) + "\n").encode()


def entry(typ: str, ident: str, parent: str | None = None, **extra: object) -> dict[str, object]:
    return {"type": typ, "id": ident, "parentId": parent, "timestamp": "t", **extra}


def test_representative_v3_tree_and_references() -> None:
    records = [
        entry("message", "u", message={"role": "user", "content": "hello", "timestamp": 1}),
        entry(
            "message",
            "a",
            "u",
            message={
                "role": "assistant",
                "content": [],
                "api": "x",
                "provider": "p",
                "model": "m",
                "usage": {},
                "stopReason": "stop",
                "timestamp": 2,
            },
        ),
        entry(
            "message",
            "tr",
            "a",
            message={
                "role": "toolResult",
                "content": [],
                "toolCallId": "x",
                "toolName": "shell",
                "isError": False,
                "timestamp": 1,
            },
        ),
        entry(
            "message",
            "c",
            "tr",
            message={
                "role": "custom",
                "content": "opaque",
                "customType": "note",
                "display": True,
                "timestamp": 1,
            },
        ),
        entry(
            "message",
            "b",
            "c",
            message={
                "role": "bashExecution",
                "command": "pwd",
                "output": "",
                "exitCode": 0,
                "cancelled": False,
                "truncated": False,
                "timestamp": 1,
            },
        ),
        entry("custom_message", "cm", "b", customType="note", content="opaque", display=True),
        entry("model_change", "model", "cm", provider="p", modelId="m"),
        entry("thinking_level_change", "think", "model", thinkingLevel="medium"),
        entry("compaction", "compact", "think", firstKeptEntryId="u", summary="s", tokensBefore=1),
        entry("branch_summary", "branch", "compact", fromId="u", summary="s"),
        entry("label", "label", "branch", targetId="u"),
        entry("custom", "custom", "label", customType="x"),
        entry("session_info", "info", "custom"),
    ]
    result = validate_native_session(session(*records))
    assert result.state is CaptureState.COMPLETE and result.entry_count == len(records)


@pytest.mark.parametrize("role", ["system", "developer", "tool"])
def test_wrong_message_roles_are_rejected(role: str) -> None:
    result = validate_native_session(
        session(entry("message", "m", message={"role": role, "content": []}))
    )
    assert result.reason is FailureReason.INVALID_SCHEMA


@pytest.mark.parametrize(
    "message",
    [
        {"role": "user", "content": "missing timestamp"},
        {
            "role": "assistant",
            "content": [],
            "api": "x",
            "provider": "p",
            "model": "m",
            "usage": {},
            "stopReason": "bad",
            "timestamp": 1,
        },
        {
            "role": "bashExecution",
            "command": "x",
            "output": "",
            "exitCode": True,
            "cancelled": False,
            "truncated": False,
            "timestamp": 1,
        },
    ],
)
def test_message_declared_fields_are_not_relaxed(message: dict[str, object]) -> None:
    assert (
        validate_native_session(session(entry("message", "m", message=message))).reason
        is FailureReason.INVALID_SCHEMA
    )


def test_message_content_containers_and_aborted_stop_reason() -> None:
    assistant = {
        "role": "assistant",
        "content": [],
        "api": "x",
        "provider": "p",
        "model": "m",
        "usage": {},
        "stopReason": "aborted",
        "timestamp": 1,
    }
    custom = {
        "role": "custom",
        "content": [{"type": "text", "text": "x"}, {"type": "image", "data": "opaque"}],
        "customType": "x",
        "display": True,
        "timestamp": 1,
    }
    assert validate_native_session(session(entry("message", "a", message=assistant))).reason is None
    assert validate_native_session(session(entry("message", "c", message=custom))).reason is None
    for role in ("assistant", "toolResult"):
        message = {"role": role, "content": "not an array"}
        assert (
            validate_native_session(session(entry("message", "bad", message=message))).reason
            is FailureReason.INVALID_SCHEMA
        )


def test_custom_message_content_and_numeric_timestamps_are_strict() -> None:
    bad_custom = entry(
        "custom_message", "c", customType="x", content={"unexpected": True}, display=True
    )
    assert validate_native_session(session(bad_custom)).reason is FailureReason.INVALID_SCHEMA
    bad_timestamp = entry(
        "message", "m", message={"role": "user", "content": "x", "timestamp": True}
    )
    assert validate_native_session(session(bad_timestamp)).reason is FailureReason.INVALID_SCHEMA


def test_numeric_entry_fields_reject_bool() -> None:
    record = entry("compaction", "c", summary="s", firstKeptEntryId="m", tokensBefore=True)
    assert (
        validate_native_session(
            session(
                entry("message", "m", message={"role": "user", "content": "x", "timestamp": 1}),
                record,
            )
        ).reason
        is FailureReason.INVALID_SCHEMA
    )


def test_custom_message_is_not_a_message_role() -> None:
    good = entry(
        "custom_message", "c", customType="x", content="bytes", display=False, details=None
    )
    assert validate_native_session(session(good)).reason is None
    bad = entry("custom_message", "c", message={"role": "custom"})
    assert validate_native_session(session(bad)).reason is FailureReason.INVALID_SCHEMA


def test_tree_and_references_must_be_prior() -> None:
    assert (
        validate_native_session(
            session(entry("message", "m", "missing", message={"role": "user"}))
        ).reason
        is FailureReason.UNKNOWN_PARENT
    )
    assert (
        validate_native_session(session(entry("label", "l", targetId="later", label="x"))).reason
        is FailureReason.UNKNOWN_REFERENCE
    )


def test_state_is_caller_supplied_and_receipts_are_explicit(tmp_path: Path) -> None:
    directory = tmp_path / "pi-session"
    directory.mkdir()
    path = directory / "s.jsonl"
    path.write_bytes(
        session(entry("message", "m", message={"role": "user", "content": [], "timestamp": 1}))
    )
    path.chmod(0o600)
    from .native_session import compare_receipt, receipt_for_session

    write_prompt_context(tmp_path, "system", b"system", "s")
    write_prompt_context(tmp_path, "initial", b"initial", "s")
    receipt = receipt_for_session(
        tmp_path,
        "pi-session/s.jsonl",
        state=CaptureState.PARTIAL,
        reason=FailureReason.INVALID_JSON,
    )
    assert receipt.state is CaptureState.PARTIAL and receipt.sha256 is not None
    assert compare_receipt(tmp_path, receipt)
    unavailable = NativeSessionReceipt(state=CaptureState.UNAVAILABLE, reason=FailureReason.MISSING)
    assert unavailable.relative_path is None
    assert (
        validate_native_session(path.read_bytes()[:-1], state=CaptureState.PARTIAL).state
        is CaptureState.PARTIAL
    )


def test_prompt_exact_bytes_and_permissions(tmp_path: Path) -> None:
    system = write_prompt_context(tmp_path, "system", b"\x00exact\xff", "s")
    initial = write_prompt_context(tmp_path, "initial", b"initial", "s")
    assert system.relative_path != initial.relative_path
    assert read_prompt_context(tmp_path, system.relative_path) == b"\x00exact\xff"
    assert verify_prompt_context(tmp_path, system)
    assert (tmp_path / system.relative_path).stat().st_mode & 0o077 == 0
    sidecar = make_prompt_sidecar(b"exact")
    assert verify_prompt_sidecar(b"exact", sidecar) and not verify_prompt_sidecar(
        b"changed", sidecar
    )


def test_session_admission_rejects_unsafe_paths_and_links(tmp_path: Path) -> None:
    (tmp_path / "pi-session").mkdir()
    (tmp_path / "pi-session" / "s.jsonl").write_bytes(session())
    from .native_session import read_session_bytes

    with pytest.raises(ValueError, match="unsafe_path"):
        read_session_bytes(tmp_path, "s.jsonl")
    link = tmp_path / "pi-session" / "link.jsonl"
    link.symlink_to(tmp_path / "pi-session" / "s.jsonl")
    with pytest.raises(ValueError, match="unsafe_file"):
        read_session_bytes(tmp_path, "pi-session/link.jsonl")


def test_export_uses_validated_package_and_does_not_mutate_source(tmp_path: Path) -> None:
    package = tmp_path / "package"
    (package / "dist").mkdir(parents=True)
    (package / "package.json").write_text(
        json.dumps({"version": "0.80.10", "bin": {"pi": "dist/cli.js"}})
    )
    (package / "dist" / "cli.js").write_text("// pinned cli")
    node = tmp_path / "node"
    node.write_text("#!/bin/sh\nprintf '<html>ok' > \"$4\"\n")
    node.chmod(0o700)
    source = tmp_path / "session.jsonl"
    original = session()
    source.write_bytes(original)
    assert export_session(node, source, package_root=package) == b"<html>ok"
    assert source.read_bytes() == original
    with pytest.raises(ValueError, match="executable_unsafe"):
        export_session(Path("node"), source, package_root=package)


def _export_fixture(tmp_path: Path, script: str) -> tuple[Path, Path, Path]:
    package = tmp_path / "package"
    (package / "dist").mkdir(parents=True)
    (package / "package.json").write_text(
        json.dumps({"version": "0.80.10", "bin": {"pi": "dist/cli.js"}})
    )
    (package / "dist" / "cli.js").write_text("// pinned cli")
    node = tmp_path / "node"
    node.write_text(script)
    node.chmod(0o700)
    session_dir = tmp_path / "pi-session"
    session_dir.mkdir()
    (session_dir / "s.jsonl").write_bytes(session())
    (session_dir / "s.jsonl").chmod(0o600)
    return node, package, tmp_path


def test_verified_export_is_content_free_and_disposes_html(tmp_path: Path) -> None:
    node, package, root = _export_fixture(tmp_path, "#!/bin/sh\nprintf '<html>ok' > \"$4\"\n")
    result = verify_export_session(
        node, root, source_relative_path="pi-session/s.jsonl", package_root=package
    )
    assert result.status is ExportVerificationStatus.SUCCEEDED
    assert result.source_sha256_before == result.source_sha256_after
    assert result.disposable_html_output_produced
    assert result.disposable_html_output_disposed
    serialized = json.dumps(result.model_dump(mode="json"))
    assert str(tmp_path) not in serialized and "<html>" not in serialized


def test_verified_export_failure_is_bounded(tmp_path: Path) -> None:
    node, package, root = _export_fixture(tmp_path, "#!/bin/sh\nexit 9\n")
    result = verify_export_session(
        node, root, source_relative_path="pi-session/s.jsonl", package_root=package
    )
    assert result.status is ExportVerificationStatus.FAILED
    assert result.failure_code is FailureReason.EXPORT_FAILED
    assert result.failure_reason == "pinned export command failed"
    assert result.disposable_html_output_disposed
    payload = json.dumps(result.model_dump(mode="json"))
    assert str(tmp_path) not in payload and "stderr" not in payload and "stdout" not in payload


def test_verified_export_timeout_is_bounded_and_disposed(tmp_path: Path) -> None:
    node, package, root = _export_fixture(tmp_path, "#!/bin/sh\nsleep 1\n")
    result = verify_export_session(
        node,
        root,
        source_relative_path="pi-session/s.jsonl",
        package_root=package,
        timeout=0.01,
    )
    assert result.status is ExportVerificationStatus.FAILED
    assert result.failure_code is FailureReason.TIMEOUT
    assert result.disposable_html_output_disposed


def test_verified_export_rejects_host_path_without_leaking_it(tmp_path: Path) -> None:
    result = verify_export_session(Path("node"), tmp_path / "session.jsonl")
    assert result.status is ExportVerificationStatus.FAILED
    assert result.failure_code is FailureReason.UNSAFE_PATH
    assert result.relative_path is None
    assert str(tmp_path) not in json.dumps(result.model_dump(mode="json"))
