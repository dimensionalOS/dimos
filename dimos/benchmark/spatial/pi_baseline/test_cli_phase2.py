from collections.abc import Callable
from pathlib import Path
from types import SimpleNamespace
from typing import cast

import pytest

from dimos.benchmark.spatial.pi_baseline import cli, cli_support
from dimos.benchmark.spatial.pi_baseline.cli_support import retain_private_diagnostic
from dimos.benchmark.spatial.pi_baseline.scheduler_executor import ExecutionInterrupted


def _arguments(root: Path, *, operation: str = "run") -> list[str]:
    args = ["experiment", operation, "experiment"]
    if operation == "retry":
        args += ["--job", "job", "--reason", "because"]
    args += [
        "--private-root",
        str(root),
        "--corpus-root",
        str(root),
        "--oracle-root",
        str(root),
        "--auth-file",
        str(root / "auth"),
        "--ledger-path",
        str(root / "ledger"),
        "--public-root",
        str(root),
        "--json",
    ]
    return args


def test_parser_errors_are_fixed_and_do_not_echo_hostile_input(
    capsys: pytest.CaptureFixture[str],
) -> None:
    with pytest.raises(SystemExit) as raised:
        cli.main(["experiment", "run", "secret\n/path\ttraceback", "--bad"])
    assert raised.value.code == 2
    assert capsys.readouterr().err == "pi-baseline: invalid arguments\n"


def test_rootless_prerequisite_uses_runtime_cancel_event(monkeypatch: pytest.MonkeyPatch) -> None:
    event = SimpleNamespace()
    seen: list[object] = []

    class FakePodman:
        def is_rootless(self, cancel_requested: object) -> bool:
            seen.append(cancel_requested)
            return True

    runtime = SimpleNamespace(_cancel_requested=event)
    monkeypatch.setattr(cli, "RootlessPodman", FakePodman)
    assert cli._host_prerequisite(runtime) is True
    assert seen == [event]


def test_rootless_prerequisite_preserves_cancellation() -> None:
    class FakePodman:
        def is_rootless(self, cancel_requested: object) -> bool:
            raise ExecutionInterrupted

    runtime = SimpleNamespace(_cancel_requested=object())
    original = cli.RootlessPodman
    cli.RootlessPodman = FakePodman
    try:
        with pytest.raises(ExecutionInterrupted):
            cli._host_prerequisite(runtime)
    finally:
        cli.RootlessPodman = original


def test_private_diagnostic_creates_secure_record_and_handles_short_writes(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    writes: list[int] = []
    original_write = cli_support.os.write
    original_fsync = cli_support.os.fsync

    def short_write(fd: int, data: bytes) -> int:
        amount = max(1, min(3, len(data)))
        writes.append(amount)
        return original_write(fd, data[:amount])

    fsyncs: list[int] = []
    monkeypatch.setattr(cli_support.os, "write", short_write)
    monkeypatch.setattr(cli_support.os, "fsync", lambda fd: fsyncs.append(fd) or original_fsync(fd))
    retain_private_diagnostic(tmp_path / "new" / "private", "run", ValueError("secret/path"))
    records = list((tmp_path / "new" / "private").glob("diagnostic-*.json"))
    assert len(records) == 1
    assert len(writes) > 1
    assert len(fsyncs) == 2


@pytest.mark.parametrize("kind", ["ancestor", "leaf"])
def test_private_diagnostic_symlinks_fail_closed_without_external_mutation(
    tmp_path: Path, kind: str
) -> None:
    external = tmp_path / "external"
    external.mkdir()
    marker = external / "marker"
    marker.write_text("unchanged", encoding="utf-8")
    if kind == "ancestor":
        root = tmp_path / "link" / "private"
        (tmp_path / "link").symlink_to(external, target_is_directory=True)
    else:
        root = tmp_path / "private"
        root.symlink_to(external, target_is_directory=True)
    retain_private_diagnostic(root, "run", RuntimeError("secret"))
    assert marker.read_text(encoding="utf-8") == "unchanged"
    assert not list(external.glob("diagnostic-*.json"))


def test_private_diagnostic_exclusive_collision_does_not_overwrite(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    target = tmp_path / "diagnostic-fixed.json"
    target.write_bytes(b"original")

    class FixedUuid:
        hex = "fixed"

    monkeypatch.setattr(cli_support, "uuid4", lambda: FixedUuid())
    retain_private_diagnostic(tmp_path, "run", RuntimeError("replacement"))
    assert target.read_bytes() == b"original"


def test_sigint_is_idempotent_restored_and_cleanup_uses_current_results(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    runtime = SimpleNamespace(store=SimpleNamespace(), cancels=0)

    def cancel() -> None:
        runtime.cancels += 1

    runtime.cancel = cancel
    bindings = object()
    handlers: list[object] = []
    snapshots: list[object] = []
    previous = object()
    fake_snapshot = object()
    summary = SimpleNamespace(outcome=SimpleNamespace(reason="container_cleanup_failed"))

    monkeypatch.setattr(cli, "_build_runtime", lambda args: (runtime, bindings))
    monkeypatch.setattr(cli, "_emit_snapshot", lambda snapshot, as_json: snapshots.append(snapshot))
    monkeypatch.setattr(cli.signal, "getsignal", lambda signum: previous)

    def fake_signal(signum: int, handler: object) -> object:
        handlers.append(handler)
        return previous

    monkeypatch.setattr(cli.signal, "signal", fake_signal)

    def execute(*args: object, **kwargs: object) -> tuple[object, ...]:
        handler = handlers[0]
        assert callable(handler)
        handler(2, None)
        handler(2, None)
        return (summary,)

    monkeypatch.setattr(cli, "execute_pi_operation", execute)
    monkeypatch.setattr(cli, "collect_operational_snapshot", lambda store: fake_snapshot)
    assert cli.main(_arguments(tmp_path)) == 1
    assert runtime.cancels == 1
    assert handlers[-1] is previous
    assert snapshots == [fake_snapshot]


def test_admission_execution_interrupted_restores_handler_snapshots_once_and_returns_130(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    runtime = SimpleNamespace(store=SimpleNamespace(), cancel=lambda: None)
    previous = object()
    handlers: list[object] = []
    snapshots: list[object] = []
    events: list[str] = []
    snapshot = object()

    monkeypatch.setattr(cli, "_build_runtime", lambda args: (runtime, object()))
    monkeypatch.setattr(cli.signal, "getsignal", lambda signum: previous)

    def fake_signal(signum: int, handler: object) -> object:
        handlers.append(handler)
        events.append("restore" if handler is previous else "install")
        return previous

    monkeypatch.setattr(cli.signal, "signal", fake_signal)
    monkeypatch.setattr(cli, "_emit_snapshot", lambda value, as_json: snapshots.append(value))

    def execute(*args: object, **kwargs: object) -> tuple[object, ...]:
        cast("Callable[[int, object], None]", handlers[0])(2, None)
        events.append("admission")
        raise ExecutionInterrupted

    monkeypatch.setattr(cli, "execute_pi_operation", execute)

    def collect(store: object) -> object:
        events.append("snapshot")
        return snapshot

    monkeypatch.setattr(cli, "collect_operational_snapshot", collect)
    assert cli.main(_arguments(tmp_path)) == 130
    assert handlers[-1] is previous
    assert snapshots == [snapshot]
    assert events == ["install", "admission", "restore", "snapshot"]


def test_cancel_failure_is_deferred_until_after_restore_and_snapshot(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    events: list[str] = []
    runtime = SimpleNamespace(store=SimpleNamespace())

    def cancel() -> None:
        events.append("cancel")
        raise RuntimeError("cancel-secret /absolute/path traceback")

    runtime.cancel = cancel
    previous = object()
    handlers: list[object] = []
    monkeypatch.setattr(cli, "_build_runtime", lambda args: (runtime, object()))
    monkeypatch.setattr(cli.signal, "getsignal", lambda signum: previous)

    def fake_signal(signum: int, handler: object) -> object:
        handlers.append(handler)
        events.append("restore" if handler is previous else "install")
        return previous

    monkeypatch.setattr(cli.signal, "signal", fake_signal)
    monkeypatch.setattr(cli, "_emit_snapshot", lambda value, as_json: events.append("render"))
    monkeypatch.setattr(
        cli,
        "execute_pi_operation",
        lambda *args, **kwargs: (cast("Callable[[int, object], None]", handlers[0])(2, None),)
        and (),
    )
    monkeypatch.setattr(
        cli, "collect_operational_snapshot", lambda store: events.append("snapshot") or object()
    )
    retained: list[str] = []
    monkeypatch.setattr(
        cli,
        "_retain_diagnostic",
        lambda root, operation, error: retained.append(type(error).__name__),
    )
    assert cli.main(_arguments(tmp_path)) == 1
    assert events == ["install", "cancel", "restore", "snapshot", "render"]
    assert retained == ["RuntimeError"]


def test_historical_snapshot_cleanup_does_not_override_clean_sigint(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    runtime = SimpleNamespace(store=SimpleNamespace(), cancel=lambda: None)
    handlers: list[object] = []
    previous = object()
    monkeypatch.setattr(cli, "_build_runtime", lambda args: (runtime, object()))
    monkeypatch.setattr(cli, "_emit_snapshot", lambda snapshot, as_json: None)
    monkeypatch.setattr(cli.signal, "getsignal", lambda signum: previous)
    monkeypatch.setattr(
        cli.signal, "signal", lambda signum, handler: handlers.append(handler) or previous
    )

    def execute(*args: object, **kwargs: object) -> tuple[object, ...]:
        cast("Callable[[int, object], None]", handlers[0])(2, None)
        return (SimpleNamespace(outcome=SimpleNamespace(reason="succeeded")),)

    monkeypatch.setattr(cli, "execute_pi_operation", execute)
    monkeypatch.setattr(
        cli,
        "collect_operational_snapshot",
        lambda store: SimpleNamespace(
            failures=(SimpleNamespace(reason="container_cleanup_failed"),)
        ),
    )
    assert cli.main(_arguments(tmp_path)) == 130


@pytest.mark.parametrize("operation", ["run", "resume", "retry"])
def test_each_operation_has_one_build_and_one_dispatch_without_direct_runtime_calls(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, operation: str
) -> None:
    calls: list[tuple[str, object]] = []

    class Runtime:
        store = SimpleNamespace()

        def cancel(self) -> None:
            calls.append(("cancel", None))

        def run(self) -> None:
            raise AssertionError("direct runtime method")

        def resume(self) -> None:
            raise AssertionError("direct runtime method")

        def retry(self, job: str, reason: str) -> None:
            raise AssertionError("direct runtime method")

    runtime = Runtime()
    build_count = 0

    def build(args: object) -> tuple[Runtime, object]:
        nonlocal build_count
        build_count += 1
        return runtime, object()

    monkeypatch.setattr(cli, "_build_runtime", build)
    monkeypatch.setattr(cli, "_emit_snapshot", lambda value, as_json: None)
    monkeypatch.setattr(cli, "collect_operational_snapshot", lambda store: object())

    def dispatch(
        runtime_value: object, bindings: object, op: str, **kwargs: object
    ) -> tuple[object, ...]:
        calls.append(("dispatch", (runtime_value, bindings, op, kwargs)))
        return ()

    monkeypatch.setattr(cli, "execute_pi_operation", dispatch)
    assert cli.main(_arguments(tmp_path, operation=operation)) == 0
    assert build_count == 1
    assert [name for name, _ in calls] == ["dispatch"]
    assert calls[0][1][2] == operation  # type: ignore[index]


@pytest.mark.parametrize("stage", ["build", "admission", "execute", "snapshot", "render"])
def test_post_parse_failures_are_fixed_and_silent(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str], stage: str
) -> None:
    secret = "unique-secret /absolute/host/path traceback"
    runtime = SimpleNamespace(store=SimpleNamespace(), cancel=lambda: None)
    if stage == "build":
        monkeypatch.setattr(
            cli, "_build_runtime", lambda args: (_ for _ in ()).throw(RuntimeError(secret))
        )
    else:
        monkeypatch.setattr(cli, "_build_runtime", lambda args: (runtime, object()))
        if stage == "admission":
            monkeypatch.setattr(
                cli,
                "execute_pi_operation",
                lambda *args, **kwargs: (_ for _ in ()).throw(RuntimeError(secret)),
            )
        elif stage == "execute":
            monkeypatch.setattr(
                cli,
                "execute_pi_operation",
                lambda *args, **kwargs: (_ for _ in ()).throw(ValueError(secret)),
            )
        else:
            monkeypatch.setattr(cli, "execute_pi_operation", lambda *args, **kwargs: ())
        if stage == "snapshot":
            monkeypatch.setattr(
                cli,
                "collect_operational_snapshot",
                lambda store: (_ for _ in ()).throw(RuntimeError(secret)),
            )
        else:
            monkeypatch.setattr(cli, "collect_operational_snapshot", lambda store: object())
        if stage == "render":
            monkeypatch.setattr(
                cli,
                "_emit_snapshot",
                lambda value, as_json: (_ for _ in ()).throw(RuntimeError(secret)),
            )
        else:
            monkeypatch.setattr(cli, "_emit_snapshot", lambda value, as_json: None)
    assert cli.main(_arguments(tmp_path)) == 1
    captured = capsys.readouterr()
    assert captured.out == ""
    assert captured.err == "pi-baseline: operation unavailable\n"
    assert secret not in captured.err


@pytest.mark.parametrize("as_json", [True, False])
def test_status_reconciles_once_and_passes_identical_snapshot_to_projection(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, as_json: bool
) -> None:
    snapshot = object()
    runtime = SimpleNamespace(store=SimpleNamespace())
    monkeypatch.setattr(cli, "_build_runtime", lambda args: (runtime, object()))
    calls = 0

    def collect(store: object) -> object:
        nonlocal calls
        calls += 1
        return snapshot

    monkeypatch.setattr(cli, "collect_operational_snapshot", collect)
    monkeypatch.setattr(
        cli, "execute_pi_operation", lambda *args, **kwargs: pytest.fail("status executed")
    )
    monkeypatch.setattr(
        cli,
        "PiSchedulerExecutor",
        lambda *args, **kwargs: pytest.fail("status constructed Pi executor"),
    )
    projected: list[object] = []
    monkeypatch.setattr(cli, "operational_json", lambda value: projected.append(value) or "{}")
    monkeypatch.setattr(cli, "render_status", lambda value: projected.append(value) or object())
    if not as_json:

        class Console:
            def print(self, value: object) -> None:
                projected.append(value)

        monkeypatch.setattr(cli, "Console", Console)
    args = ["experiment", "status", "experiment", "--private-root", str(tmp_path)]
    if as_json:
        args.append("--json")
    assert cli.main(args) == 0
    assert calls == 1
    assert projected == [snapshot] if as_json else projected[0] is snapshot


def test_keyboard_interrupt_exits_130_without_generic_error(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    runtime = SimpleNamespace(store=SimpleNamespace())
    monkeypatch.setattr(cli, "_build_runtime", lambda args: (runtime, object()))
    monkeypatch.setattr(
        cli,
        "execute_pi_operation",
        lambda *args, **kwargs: (_ for _ in ()).throw(KeyboardInterrupt()),
    )
    monkeypatch.setattr(cli, "collect_operational_snapshot", lambda store: object())
    assert cli.main(_arguments(tmp_path)) == 130
    captured = capsys.readouterr()
    assert captured.out == ""
    assert captured.err == ""


def test_environment_roots_do_not_bypass_required_flags(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("PRIVATE_ROOT", "/secret/private")
    monkeypatch.setenv("CORPUS_ROOT", "/secret/corpus")
    with pytest.raises(SystemExit) as raised:
        cli.main(["experiment", "run", "experiment"])
    assert raised.value.code == 2
    with pytest.raises(SystemExit) as raised:
        cli.main(["run-paired", "experiment"])
    assert raised.value.code == 2
