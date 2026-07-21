# Copyright 2026 Dimensional Inc.
"""Repository-owned experiment CLI for the Pi spatial baseline."""

from __future__ import annotations

import argparse
from pathlib import Path
import signal
import sys
import threading
from typing import Literal, NoReturn, cast

from rich.console import Console

from .cli_support import (
    create_experiment,
    execute_pi_operation,
    load_definition,
    retain_private_diagnostic,
    runtime_bindings,
)
from .config import load_config
from .podman import RootlessPodman
from .scheduler_display import operational_json, render_status
from .scheduler_executor import EventSink, ExecutionInterrupted
from .scheduler_models import (
    AttemptContext,
    ExpandedCase,
    NamedCondition,
    OperationalSnapshot,
    TerminalOutcome,
)
from .scheduler_operational import OperationalObservationError, collect_operational_snapshot
from .scheduler_pi_binding import PiRuntimeBindings
from .scheduler_pi_executor import PiSchedulerExecutor
from .scheduler_runtime import SchedulerRuntime


class _StatusExecutor:
    def run(
        self,
        case: ExpandedCase,
        condition: NamedCondition,
        context: AttemptContext,
        emit: EventSink,
        cancel_requested: threading.Event,
        publication_lock: threading.Lock,
    ) -> TerminalOutcome:
        raise RuntimeError("status runtime cannot execute")


class _SafeArgumentParser(argparse.ArgumentParser):
    def error(self, message: str) -> NoReturn:
        del message
        print("pi-baseline: invalid arguments", file=sys.stderr)
        raise SystemExit(2)


def _runtime_arguments(parser: argparse.ArgumentParser, *, required: bool = True) -> None:
    parser.add_argument("--private-root", type=Path, required=required)
    parser.add_argument("--corpus-root", type=Path, required=required)
    parser.add_argument("--oracle-root", type=Path, required=required)
    parser.add_argument("--auth-file", type=Path, required=required)
    parser.add_argument("--ledger-path", type=Path, required=required)
    parser.add_argument("--public-root", type=Path, required=required)
    parser.add_argument(
        "--json", action="store_true", help="emit machine-readable operational output"
    )


def _parser() -> argparse.ArgumentParser:
    parser = _SafeArgumentParser(prog="pi-baseline")
    subparsers = parser.add_subparsers(
        dest="command", required=True, parser_class=_SafeArgumentParser
    )

    validate = subparsers.add_parser("validate", help="validate an authoring config only")
    validate.add_argument("config", type=Path)

    experiment = subparsers.add_parser("experiment", help="manage an experiment")
    experiment_sub = experiment.add_subparsers(
        dest="experiment_command", required=True, parser_class=_SafeArgumentParser
    )
    create = experiment_sub.add_parser("create")
    create.add_argument("experiment_dir", type=Path)
    create.add_argument("--spec", type=Path, required=True)
    create.add_argument("--workers", type=int)
    create.add_argument("--sample", type=int)
    create.add_argument("--shard", type=int, default=0)
    create.add_argument("--shards", type=int, default=1)

    for name in ("run", "resume"):
        command = experiment_sub.add_parser(name)
        command.add_argument("experiment_dir", type=Path)
        _runtime_arguments(command)

    retry = experiment_sub.add_parser("retry")
    retry.add_argument("experiment_dir", type=Path)
    retry.add_argument("--job", required=True)
    retry.add_argument("--reason", required=True)
    _runtime_arguments(retry)

    status = experiment_sub.add_parser("status")
    status.add_argument("experiment_dir", type=Path)
    status.add_argument("--private-root", type=Path, required=True)
    status.add_argument("--json", action="store_true")

    review = experiment_sub.add_parser("review", help="disabled until Phase 3")
    review.add_argument("experiment_dir", type=Path)
    review.add_argument("--private-root", type=Path, required=True)
    review.add_argument("--reviewer", required=True)
    review.add_argument("--decision", choices=("approved", "rejected"), required=True)

    report = experiment_sub.add_parser("report", help="disabled until Phase 3")
    report.add_argument("experiment_dir", type=Path)
    report.add_argument("--private-root", type=Path, required=True)
    report.add_argument("--review-decision", type=Path, required=True)
    return parser


def _experiment_dir(value: Path) -> Path:
    return value.expanduser().resolve()


def _build_runtime(
    args: argparse.Namespace,
) -> tuple[SchedulerRuntime, PiRuntimeBindings | None]:
    experiment_dir = _experiment_dir(args.experiment_dir)
    manifest, _, snapshot, store = load_definition(experiment_dir)
    if args.experiment_command == "status":
        # Status is deliberately not given execution bindings.  Its private root
        # is only the explicit destination for failure diagnostics.
        return SchedulerRuntime(store, _StatusExecutor()), None
    bindings = runtime_bindings(
        private_root=args.private_root.expanduser(),
        corpus_root=args.corpus_root.expanduser().resolve(),
        oracle_root=args.oracle_root.expanduser().resolve(),
        auth_file=args.auth_file.expanduser().resolve(),
        ledger_path=args.ledger_path.expanduser().resolve(),
        public_root=args.public_root.expanduser().resolve(),
    )
    executor = PiSchedulerExecutor(
        snapshot,
        bindings,
        manifest_executor_fingerprint=manifest.executor_fingerprint,
    )
    return SchedulerRuntime(store, executor), bindings


def _emit_snapshot(snapshot: OperationalSnapshot, as_json: bool) -> None:
    if as_json:
        print(operational_json(snapshot))
    else:
        Console().print(render_status(snapshot))


def _safe_error(error: BaseException, operation: str) -> tuple[str, int]:
    del error, operation
    return "operation unavailable", 1


def _host_prerequisite(runtime: SchedulerRuntime) -> bool:
    """Require rootless Podman while observing the runtime cancellation event."""
    return RootlessPodman().is_rootless(runtime._cancel_requested)


def _retain_diagnostic(private_root: Path, operation: str, error: BaseException) -> None:
    try:
        retain_private_diagnostic(private_root, operation, error)
    except BaseException:
        return


def main(argv: list[str] | None = None) -> int:
    parser = _parser()
    args = parser.parse_args(argv)
    active_runtime: SchedulerRuntime | None = None
    active_private_root: Path | None = None
    try:
        if args.command == "validate":
            load_config(args.config)
            print("configuration is valid")
            return 0
        if args.command == "experiment" and args.experiment_command == "create":
            command = args
            manifest, _ = create_experiment(
                _experiment_dir(command.experiment_dir),
                command.spec,
                workers=command.workers,
                sample=command.sample,
                shard=command.shard,
                shards=command.shards,
            )
            print(
                f"{_experiment_dir(command.experiment_dir) / 'manifest.json'} {manifest.executor_fingerprint}"
            )
            return 0
        if args.command != "experiment":
            print("pi-baseline: operation unavailable", file=sys.stderr)
            return 1
        if args.experiment_command in {"review", "report"}:
            print("pi-baseline: operation unavailable", file=sys.stderr)
            return 1
        if args.experiment_command not in {"run", "resume", "retry", "status"}:
            print("pi-baseline: operation unavailable", file=sys.stderr)
            return 1

        active_private_root = args.private_root.expanduser()
        active_runtime, bindings = _build_runtime(args)
        if args.experiment_command == "status":
            snapshot = collect_operational_snapshot(active_runtime.store)
            _emit_snapshot(snapshot, args.json)
            return 0

        interrupted = False
        cancel_error: BaseException | None = None
        previous_handler = signal.getsignal(signal.SIGINT)
        diagnostic_errors: list[BaseException] = []

        def on_sigint(_signum: int, _frame: object) -> None:
            nonlocal cancel_error, interrupted
            if not interrupted:
                interrupted = True
                try:
                    active_runtime.cancel()
                except BaseException as error:
                    cancel_error = error

        signal.signal(signal.SIGINT, on_sigint)
        operation_error: BaseException | None = None
        snapshot = None
        results = ()

        try:
            try:
                assert bindings is not None
                results = execute_pi_operation(
                    active_runtime,
                    bindings,
                    cast("Literal['run', 'resume', 'retry']", args.experiment_command),
                    host_prerequisite=lambda: _host_prerequisite(active_runtime),
                    job_id_value=getattr(args, "job", None),
                    reason=getattr(args, "reason", None),
                )
            except ExecutionInterrupted as error:
                if not interrupted:
                    operation_error = error
                    diagnostic_errors.append(error)
            except KeyboardInterrupt:
                return 130
            except BaseException as error:
                operation_error = error
                diagnostic_errors.append(error)
        finally:
            try:
                signal.signal(signal.SIGINT, previous_handler)
            except BaseException as error:
                operation_error = operation_error or error
                diagnostic_errors.append(error)
            try:
                snapshot = collect_operational_snapshot(active_runtime.store)
            except BaseException as error:
                operation_error = operation_error or error
                diagnostic_errors.append(error)
        if cancel_error is not None:
            operation_error = operation_error or cancel_error
            diagnostic_errors.append(cancel_error)
        if active_private_root is not None:
            for error in diagnostic_errors:
                _retain_diagnostic(active_private_root, args.experiment_command, error)
        if snapshot is None:
            print("pi-baseline: operation unavailable", file=sys.stderr)
            return 1
        _emit_snapshot(snapshot, args.json)
        cleanup_failed = any(
            summary.outcome is not None and summary.outcome.reason == "container_cleanup_failed"
            for summary in results
        )
        if operation_error is not None or cleanup_failed:
            message, code = _safe_error(
                operation_error or RuntimeError("cleanup"), args.experiment_command
            )
            print(f"pi-baseline: {message}", file=sys.stderr)
            return code
        return 130 if interrupted else 0
    except (
        OSError,
        KeyError,
        PermissionError,
        ValueError,
        RuntimeError,
        OperationalObservationError,
    ) as error:
        if active_private_root is not None:
            _retain_diagnostic(
                active_private_root, getattr(args, "experiment_command", "unknown"), error
            )
        print("pi-baseline: operation unavailable", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        return 130
    except SystemExit:
        raise
    except BaseException as error:
        if active_private_root is not None:
            _retain_diagnostic(
                active_private_root, getattr(args, "experiment_command", "unknown"), error
            )
        print("pi-baseline: operation unavailable", file=sys.stderr)
        return 1
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
