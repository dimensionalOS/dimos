"""Host-local POSIX filesystem state for one scheduler coordinator."""

from __future__ import annotations

from collections.abc import Iterator, Mapping
from contextlib import contextmanager
from dataclasses import dataclass
import fcntl
import hashlib
import json
import os
from pathlib import Path
import re
import stat
import threading
from typing import TypeVar, cast
from uuid import uuid4

from dimos.benchmark.spatial.models import SpatialModel
from dimos.benchmark.spatial.utilities import JsonValue, canonical_json

from .scheduler_models import (
    AttemptContext,
    AttemptManifestSnapshot,
    ExpandedCase,
    ExperimentManifest,
    ExperimentPlan,
    JobSummary,
    NamedCondition,
    OperationalEvent,
    QuarantineMetadata,
    TerminalOutcome,
)
from .scheduler_plan import (
    canonical_manifest_bytes,
    job_id,
    manifest_digest as canonical_manifest_digest,
    validate_manifest,
)

_SAFE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]*$")


class CoordinatorLockError(RuntimeError):
    """The host-local coordinator is already owned by another process."""


class StoreMutationError(RuntimeError):
    """A mutator was called without this store's coordinator lock."""


_LEASE_SECRET = object()


class CoordinatorLeaseCapability:
    """Opaque proof that a worker belongs to this store's live coordinator."""

    __slots__ = ("_store",)

    def __init__(self, store: FilesystemExperimentStore, secret: object) -> None:
        if secret is not _LEASE_SECRET:
            raise TypeError("coordinator lease capabilities are not constructible")
        self._store = store


@dataclass(frozen=True)
class LoadedDefinition:
    manifest: ExperimentManifest
    plan: ExperimentPlan
    manifest_digest: str
    plan_digest: str
    plan_file_digest: str


@dataclass(frozen=True)
class RecoveredAttempt:
    """A record set validated while its descriptors were open."""

    context: AttemptContext
    case: ExpandedCase
    condition: NamedCondition
    outcome: TerminalOutcome | None
    directory: Path


def _safe(value: str) -> str:
    if not _SAFE.fullmatch(value):
        raise ValueError("identifier is not safe for an artifact path")
    return value


class FilesystemExperimentStore:
    """A single-coordinator store; no locking or remote service is implied."""

    def __init__(self, root: Path) -> None:
        self.root = root
        self._lock_state = threading.local()
        self._lease_state = threading.local()
        self._lease_guard = threading.Lock()
        self._lease_capability: CoordinatorLeaseCapability | None = None

    @contextmanager
    def coordinator_lease(self, *, wait: bool = False) -> Iterator[CoordinatorLeaseCapability]:
        """Hold the exclusive coordinator lease for a whole runtime operation."""
        if getattr(self._lease_state, "depth", 0):
            self._lease_state.depth += 1
            try:
                capability = self._lease_state.capability
                assert isinstance(capability, CoordinatorLeaseCapability)
                yield capability
            finally:
                self._lease_state.depth -= 1
            return
        self.root.parent.mkdir(parents=True, exist_ok=True)
        lock_path = self.root.parent / f".{self.root.name}.coordinator.lock"
        handle = lock_path.open("a+")
        acquired = False
        capability: CoordinatorLeaseCapability | None = None
        try:
            flags = fcntl.LOCK_EX if wait else fcntl.LOCK_EX | fcntl.LOCK_NB
            try:
                fcntl.flock(handle.fileno(), flags)
            except OSError as error:
                raise CoordinatorLockError(f"coordinator lease is held: {lock_path}") from error
            acquired = True
            capability = CoordinatorLeaseCapability(self, _LEASE_SECRET)
            with self._lease_guard:
                self._lease_capability = capability
            self._lease_state.depth = 1
            self._lease_state.capability = capability
            self._lock_state.depth = getattr(self._lock_state, "depth", 0) + 1
            assert capability is not None
            yield capability
        finally:
            if acquired:
                self._lock_state.depth = max(0, getattr(self._lock_state, "depth", 1) - 1)
                self._lease_state.depth = 0
                self._lease_state.capability = None
                with self._lease_guard:
                    if self._lease_capability is capability:
                        self._lease_capability = None
                fcntl.flock(handle.fileno(), fcntl.LOCK_UN)
            handle.close()

    @contextmanager
    def lease_mutation(self, capability: CoordinatorLeaseCapability) -> Iterator[None]:
        """Authorize a scheduler worker thread under its owning runtime lease."""
        with self._lease_guard:
            active = self._lease_capability is capability and capability._store is self
        if not active:
            raise StoreMutationError("invalid or inactive scheduler lease capability")
        depth = getattr(self._lock_state, "depth", 0)
        self._lock_state.depth = depth + 1
        try:
            yield
        finally:
            self._lock_state.depth = depth

    @contextmanager
    def coordinator_lock(self, *, wait: bool = False) -> Iterator[None]:
        """Exclusively lock mutating coordinator operations on this host."""
        self.root.parent.mkdir(parents=True, exist_ok=True)
        lock_path = self.root.parent / f".{self.root.name}.coordinator.lock"
        depth = getattr(self._lock_state, "depth", 0)
        if depth:
            self._lock_state.depth = depth + 1
            try:
                yield
            finally:
                self._lock_state.depth = depth
            return
        handle = lock_path.open("a+")
        acquired = False
        try:
            try:
                flags = fcntl.LOCK_EX if wait else fcntl.LOCK_EX | fcntl.LOCK_NB
                fcntl.flock(handle.fileno(), flags)
            except OSError as error:
                raise CoordinatorLockError(f"coordinator lock is held: {lock_path}") from error
            acquired = True
            self._lock_state.depth = 1
            yield
        finally:
            if acquired:
                self._lock_state.depth = 0
            try:
                if acquired:
                    fcntl.flock(handle.fileno(), fcntl.LOCK_UN)
            finally:
                handle.close()

    def create(
        self,
        manifest: ExperimentManifest,
        plan: ExperimentPlan,
        *,
        additional_files: Mapping[str, JsonValue] | None = None,
    ) -> None:
        with self.coordinator_lock():
            self._create_unlocked(manifest, plan, additional_files=additional_files)

    def _create_unlocked(
        self,
        manifest: ExperimentManifest,
        plan: ExperimentPlan,
        *,
        additional_files: Mapping[str, JsonValue] | None = None,
    ) -> None:
        validate_manifest(manifest, plan)
        if self.root.name != manifest.experiment_id:
            raise ValueError("store root name must match manifest experiment_id")
        self.root.parent.mkdir(parents=True, exist_ok=True)
        if self.root.exists():
            raise FileExistsError(self.root)
        temporary = self.root.parent / f".{self.root.name}.{uuid4().hex}.tmp"
        temporary.mkdir()
        try:
            (temporary / "attempts").mkdir()
            self._write_new(temporary / "plan.json", plan.model_dump(mode="json"))
            for name, value in (additional_files or {}).items():
                if Path(name).name != name or not name.endswith(".json"):
                    raise ValueError("additional artifact names must be safe JSON filenames")
                self._write_new(temporary / name, value)
            # The manifest is the final commit marker inside the transaction.
            self._write_new(temporary / "manifest.json", manifest.model_dump(mode="json"))
            _fsync_directory(temporary)
            os.replace(temporary, self.root)
            _fsync_directory(self.root.parent)
        except Exception:
            if temporary.exists():
                _remove_tree(temporary)
            raise

    def load_definition(self) -> LoadedDefinition:
        """Load and validate the canonical on-disk definition for every command."""
        directory = self._experiment_directory()
        plan_bytes = _read_regular_bytes(directory / "plan.json")
        manifest_bytes = _read_regular_bytes(directory / "manifest.json")
        plan = ExperimentPlan.model_validate_json(plan_bytes)
        manifest = ExperimentManifest.model_validate_json(manifest_bytes)
        if plan_bytes != canonical_json(plan.model_dump(mode="json")) + b"\n":
            raise ValueError("plan bytes are not canonical")
        if manifest_bytes != canonical_manifest_bytes(manifest):
            raise ValueError("manifest bytes are not canonical")
        validate_manifest(manifest, plan)
        return LoadedDefinition(
            manifest=manifest,
            plan=plan,
            manifest_digest=canonical_manifest_digest(manifest_bytes),
            plan_digest=plan.plan_digest,
            plan_file_digest=hashlib.sha256(plan_bytes).hexdigest(),
        )

    def _require_lock(self) -> None:
        if not getattr(self._lock_state, "depth", 0):
            raise StoreMutationError("mutating store operation requires coordinator_lock()")

    def create_attempt(
        self, context: AttemptContext, case: ExpandedCase, condition: NamedCondition
    ) -> Path:
        self._require_lock()
        if context.identity.experiment_id != self.root.name:
            raise ValueError("attempt experiment does not match this store")
        definition = self.load_definition()
        if job_id(definition.plan, case, condition) != context.identity.job_id:
            raise ValueError("attempt job identity does not match case and condition")
        attempts_root = self.root / "attempts"
        if _path_is_symlink(attempts_root) or not attempts_root.is_dir():
            raise ValueError("attempts root is not a regular directory")
        base = self.root / "attempts" / _safe(context.identity.job_id)
        directory = base / _safe(context.attempt_id)
        if _path_is_symlink(base) or _path_is_symlink(directory):
            raise ValueError("attempt path contains a symlink")
        if directory.exists():
            raise FileExistsError(directory)
        manifest_path = self.root / "manifest.json"
        manifest_bytes = _read_regular_bytes(manifest_path)
        manifest_value = json.loads(manifest_bytes)
        manifest_digest = canonical_manifest_digest(manifest_bytes)
        if context.manifest_digest is not None and context.manifest_digest != manifest_digest:
            raise ValueError("attempt context manifest digest does not match experiment manifest")
        snapshot = AttemptManifestSnapshot(
            identity=context.identity,
            attempt_id=context.attempt_id,
            manifest_digest=manifest_digest,
            manifest=cast("dict[str, object]", manifest_value),
        )
        base.mkdir(parents=True, exist_ok=True)
        temporary = base / f".{directory.name}.{uuid4().hex}.tmp"
        temporary.mkdir()
        stored_context = context.model_copy(update={"manifest_digest": manifest_digest})
        try:
            self._write_new(temporary / "context.json", stored_context.model_dump(mode="json"))
            self._write_new(
                temporary / "attempt-manifest.v1.json", snapshot.model_dump(mode="json")
            )
            self._write_new(temporary / "case.json", case.model_dump(mode="json"))
            self._write_new(temporary / "condition.json", condition.model_dump(mode="json"))
            (temporary / "events.jsonl").touch()
            _fsync_directory(temporary)
            os.replace(temporary, directory)
            _fsync_directory(base)
        except Exception:
            if temporary.exists():
                _remove_tree(temporary)
            raise
        return directory

    def append_event(self, attempt: AttemptContext, event: OperationalEvent) -> None:
        self._require_lock()
        event = OperationalEvent.model_validate(event.model_dump(mode="python"))
        payload = canonical_json(event.model_dump(mode="json")) + b"\n"
        with self._open_attempt_directory(attempt) as attempt_fd:
            descriptor = os.open(
                "events.jsonl", os.O_WRONLY | os.O_APPEND | os.O_NOFOLLOW, dir_fd=attempt_fd
            )
            try:
                metadata = os.fstat(descriptor)
                if not stat.S_ISREG(metadata.st_mode):
                    raise ValueError("events record is not a regular file")
                written = 0
                while written < len(payload):
                    written += os.write(descriptor, payload[written:])
                os.fsync(descriptor)
            finally:
                os.close(descriptor)

    def write_outcome(self, attempt: AttemptContext, outcome: TerminalOutcome) -> Path:
        """Write the one immutable terminal outcome for an attempt."""
        self._require_lock()
        payload = canonical_json(outcome.model_dump(mode="json")) + b"\n"
        with self._open_attempt_directory(attempt) as attempt_fd:
            temporary_name = f".outcome.v1.json.{uuid4().hex}.tmp"
            temporary_fd: int | None = None
            try:
                temporary_fd = os.open(
                    temporary_name,
                    os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW,
                    0o600,
                    dir_fd=attempt_fd,
                )
                written = 0
                while written < len(payload):
                    written += os.write(temporary_fd, payload[written:])
                os.fsync(temporary_fd)
                os.close(temporary_fd)
                temporary_fd = None

                # Hard-link publication is atomic and refuses to replace an
                # existing outcome.  The temporary name is removed only after
                # the final directory entry is visible.
                os.link(
                    temporary_name,
                    "outcome.v1.json",
                    src_dir_fd=attempt_fd,
                    dst_dir_fd=attempt_fd,
                    follow_symlinks=False,
                )
                os.unlink(temporary_name, dir_fd=attempt_fd)
                _fsync_directory_fd(attempt_fd)
            except Exception:
                if temporary_fd is not None:
                    os.close(temporary_fd)
                try:
                    os.unlink(temporary_name, dir_fd=attempt_fd)
                except FileNotFoundError:
                    pass
                raise
        return self._attempt_path(attempt) / "outcome.v1.json"

    def read_outcome(self, attempt: AttemptContext) -> TerminalOutcome | None:
        with self._open_attempt_directory(attempt) as attempt_fd:
            return _read_optional_model_at(attempt_fd, "outcome.v1.json", TerminalOutcome)

    def events(self, attempt: AttemptContext) -> tuple[OperationalEvent, ...]:
        with self._open_attempt_directory(attempt) as attempt_fd:
            return _read_events_at(attempt_fd)

    def write_summary(self, summary: JobSummary) -> None:
        self._require_lock()
        path = self.root / "jobs" / f"{_safe(summary.identity.job_id)}.json"
        if _path_is_symlink(path.parent):
            raise ValueError("jobs root is not a regular directory")
        if not path.parent.exists():
            path.parent.mkdir(mode=0o700)
        if not path.parent.is_dir():
            raise ValueError("jobs root is not a regular directory")
        if _path_is_symlink(path):
            raise ValueError("summary path is a symlink")
        path.parent.mkdir(exist_ok=True)
        temporary = path.with_name(f".{path.name}.{uuid4().hex}.tmp")
        temporary.write_bytes(canonical_json(summary.model_dump(mode="json")) + b"\n")
        with temporary.open("rb") as stream:
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        _fsync_directory(path.parent)

    def summaries(self, experiment_id: str) -> tuple[JobSummary, ...]:
        if experiment_id != self.root.name:
            raise ValueError("summary experiment does not match this store")
        directory = self.root / "jobs"
        if _path_is_symlink(directory) or not directory.is_dir():
            return ()
        result: list[JobSummary] = []
        with _open_directory(directory) as directory_fd:
            names = sorted(name for name in os.listdir(directory_fd) if name.endswith(".json"))
        for name in names:
            try:
                with _open_directory(directory) as directory_fd:
                    result.append(_read_model_at(directory_fd, name, JobSummary))
            except Exception:
                continue
        return tuple(result)

    def recover_attempts(
        self,
        experiment_id: str,
        job_id_value: str,
        *,
        definition: LoadedDefinition | None = None,
    ) -> tuple[RecoveredAttempt, ...]:
        """Validate records through no-follow descriptors and quarantine corruption."""
        self._require_lock()
        if definition is None:
            definition = self.load_definition()
        if experiment_id != definition.manifest.experiment_id:
            raise ValueError("attempt experiment does not match this store")
        root_name = _safe(job_id_value)
        with _open_directory(self.root) as root_fd:
            attempts_fd = _open_optional_directory(root_fd, "attempts")
            if attempts_fd is None:
                if _entry_exists(root_fd, "attempts"):
                    self._quarantine_at(root_fd, "attempts", "attempts")
                attempts_fd = _mkdir_open_at(root_fd, "attempts")
                os.close(attempts_fd)
                return ()
            with _fd_handle(attempts_fd):
                if _entry_is_symlink(attempts_fd, root_name):
                    self._quarantine_at(attempts_fd, root_name, job_id_value, root_fd)
                    return ()
                job_fd = _open_optional_directory(attempts_fd, root_name)
                if job_fd is None:
                    if _entry_exists(attempts_fd, root_name):
                        self._quarantine_at(attempts_fd, root_name, job_id_value, root_fd)
                    return ()
                with _fd_handle(job_fd):
                    valid: list[RecoveredAttempt] = []
                    for candidate_name in sorted(os.listdir(job_fd)):
                        candidate_path = self.root / "attempts" / root_name / candidate_name
                        if not re.fullmatch(r"attempt-[0-9]+", candidate_name):
                            self._quarantine_at(job_fd, candidate_name, job_id_value, root_fd)
                            continue
                        candidate_fd = _open_optional_directory(job_fd, candidate_name)
                        if candidate_fd is None:
                            self._quarantine_at(job_fd, candidate_name, job_id_value, root_fd)
                            continue
                        with _fd_handle(candidate_fd):
                            try:
                                context = _read_model_at(candidate_fd, "context.json", AttemptContext)
                                snapshot = _read_model_at(
                                    candidate_fd, "attempt-manifest.v1.json", AttemptManifestSnapshot
                                )
                                case = _read_model_at(candidate_fd, "case.json", ExpandedCase)
                                condition = _read_model_at(
                                    candidate_fd, "condition.json", NamedCondition
                                )
                                _read_events_at(candidate_fd)
                                outcome = _read_optional_model_at(
                                    candidate_fd, "outcome.v1.json", TerminalOutcome
                                )
                                expected_job_id = job_id(definition.plan, case, condition)
                                expected_case = next(
                                    item for item in definition.plan.cases if item.case_id == case.case_id
                                )
                                expected_condition = next(
                                    item
                                    for item in definition.plan.conditions
                                    if item.name == condition.name
                                )
                                if (
                                    candidate_name != context.attempt_id
                                    or candidate_name != f"attempt-{context.attempt_number}"
                                    or context.directory_name != candidate_name
                                    or context.identity.experiment_id != definition.manifest.experiment_id
                                    or context.identity.case_id != case.case_id
                                    or context.identity.condition_name != condition.name
                                    or context.identity.job_id != job_id_value
                                    or expected_job_id != job_id_value
                                    or case != expected_case
                                    or condition != expected_condition
                                    or context.manifest_digest != definition.manifest_digest
                                    or snapshot.identity != context.identity
                                    or snapshot.attempt_id != context.attempt_id
                                    or snapshot.manifest_digest != definition.manifest_digest
                                    or snapshot.manifest != definition.manifest.model_dump(mode="json")
                                ):
                                    raise ValueError("attempt identity or manifest mismatch")
                            except Exception:
                                self._quarantine_at(job_fd, candidate_name, job_id_value, root_fd)
                                continue
                        valid.append(
                            RecoveredAttempt(
                                context=context,
                                case=case,
                                condition=condition,
                                outcome=outcome,
                                directory=candidate_path,
                            )
                        )
                    return tuple(valid)

    def observe_attempts_read_only(
        self, experiment_id: str, job_id_value: str
    ) -> tuple[RecoveredAttempt, ...]:
        """Read validated attempt records without mutating or following paths."""
        _, attempts = self.observe_experiment_read_only(experiment_id, (job_id_value,))
        return attempts.get(job_id_value, ())

    def observe_experiment_read_only(
        self, experiment_id: str | None, job_ids: tuple[str, ...] | None
    ) -> tuple[LoadedDefinition, dict[str, tuple[RecoveredAttempt, ...]]]:
        """Observe one experiment through a descriptor-pinned, read-only view."""
        try:
            root_fd = os.open(self.root, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW)
        except OSError as error:
            raise ValueError("experiment is not safely observable") from error
        try:
            definition = _load_definition_at(root_fd)
            if experiment_id is not None and experiment_id != definition.manifest.experiment_id:
                raise ValueError("attempt experiment does not match this store")
            if job_ids is None:
                job_ids = tuple(
                    job_id(definition.plan, case, condition)
                    for item in definition.plan.jobs
                    for case in definition.plan.cases
                    if case.case_id == item.case_id
                    for condition in definition.plan.conditions
                    if condition.name == item.condition_name
                )
            attempts = {
                identifier: _observe_attempts_at(root_fd, definition, identifier)
                for identifier in job_ids
            }
            return definition, attempts
        except (FileNotFoundError, NotADirectoryError, OSError, ValueError, TypeError) as error:
            raise ValueError("experiment is not safely observable") from error
        finally:
            os.close(root_fd)

    def recover_all_attempts(
        self, experiment_id: str, *, definition: LoadedDefinition | None = None
    ) -> None:
        """Quarantine attempts for jobs absent from the current immutable plan too."""
        self._require_lock()
        if definition is None:
            definition = self.load_definition()
        if experiment_id != definition.manifest.experiment_id:
            raise ValueError("attempt experiment does not match this store")
        expected = {
            job_id(definition.plan, case, condition)
            for case in definition.plan.cases
            for condition in definition.plan.conditions
            if any(
                item.case_id == case.case_id and item.condition_name == condition.name
                for item in definition.plan.jobs
            )
        }
        with _open_directory(self.root) as root_fd:
            attempts_fd = _open_optional_directory(root_fd, "attempts")
            if attempts_fd is None:
                if _entry_exists(root_fd, "attempts"):
                    self._quarantine_at(root_fd, "attempts", "attempts")
                attempts_fd = _mkdir_open_at(root_fd, "attempts")
                os.close(attempts_fd)
                return
            with _fd_handle(attempts_fd):
                for job_name in tuple(os.listdir(attempts_fd)):
                    if job_name in expected:
                        continue
                    if _entry_is_symlink(attempts_fd, job_name):
                        self._quarantine_at(attempts_fd, job_name, "unknown-job", root_fd)
                        continue
                    job_fd = _open_optional_directory(attempts_fd, job_name)
                    if job_fd is None:
                        self._quarantine_at(attempts_fd, job_name, "unknown-job", root_fd)
                        continue
                    with _fd_handle(job_fd):
                        for child in tuple(os.listdir(job_fd)):
                            self._quarantine_at(job_fd, child, "unknown-job", root_fd)
                        empty = not os.listdir(job_fd)
                    if empty:
                        os.rmdir(job_name, dir_fd=attempts_fd)
                        _fsync_directory_fd(attempts_fd)

    def used_attempt_numbers(self, job_id_value: str) -> tuple[int, ...]:
        """Include quarantined identities so an attempt number is never reused."""
        self._require_lock()
        names: list[str] = []
        with _open_directory(self.root) as root_fd:
            for allocation_root in ("attempts", "quarantine"):
                allocation_fd = _open_optional_directory(root_fd, allocation_root)
                if allocation_fd is None:
                    if _entry_exists(root_fd, allocation_root):
                        self._quarantine_at(
                            root_fd,
                            allocation_root,
                            allocation_root,
                            quarantine_name="quarantine-recovery"
                            if allocation_root == "quarantine"
                            else "quarantine",
                        )
                    continue
                with _fd_handle(allocation_fd):
                    job_fd = _open_optional_directory(allocation_fd, _safe(job_id_value))
                    if job_fd is None:
                        if _entry_exists(allocation_fd, _safe(job_id_value)):
                            self._quarantine_at(
                                allocation_fd,
                                _safe(job_id_value),
                                job_id_value,
                                root_fd,
                                quarantine_name="quarantine-recovery"
                                if allocation_root == "quarantine"
                                else "quarantine",
                            )
                        continue
                    with _fd_handle(job_fd):
                        if allocation_root == "attempts":
                            names.extend(os.listdir(job_fd))
        numbers = {
            number
            for name in names
            if (number := _canonical_active_attempt_number(name)) is not None
        }
        with _open_directory(self.root) as root_fd:
            quarantine_fd = _open_optional_directory(root_fd, "quarantine")
            if quarantine_fd is not None:
                with _fd_handle(quarantine_fd):
                    job_fd = _open_optional_directory(quarantine_fd, _safe(job_id_value))
                    if job_fd is not None:
                        with _fd_handle(job_fd):
                            for name in os.listdir(job_fd):
                                if not name.endswith(".metadata.json"):
                                    continue
                                try:
                                    metadata = _read_model_at(job_fd, name, QuarantineMetadata)
                                    target_name = name.removesuffix(".metadata.json")
                                    original_number = _attempt_number_from_name(metadata.original_name)
                                    if (
                                        metadata.quarantined_name == target_name
                                        and original_number == metadata.original_attempt_number
                                        and _entry_exists(job_fd, target_name)
                                    ):
                                        numbers.add(metadata.original_attempt_number)
                                except Exception:
                                    continue
        return tuple(sorted(numbers))

    def _quarantine_at(
        self,
        source_fd: int,
        name: str,
        job_id_value: str,
        destination_fd: int | None = None,
        quarantine_name: str = "quarantine",
    ) -> None:
        destination = source_fd if destination_fd is None else destination_fd
        quarantine_fd = _mkdir_open_at(destination, quarantine_name)
        try:
            target_root_fd = _mkdir_open_at(quarantine_fd, _safe(job_id_value))
            try:
                target_name = f"q-{uuid4().hex}"
                attempt_number = _attempt_number_from_name(name)
                if attempt_number is not None:
                    metadata = QuarantineMetadata(
                        quarantined_name=target_name,
                        original_name=name,
                        original_attempt_number=attempt_number,
                    )
                    self._write_new_at(
                        target_root_fd,
                        f"{target_name}.metadata.json",
                        metadata.model_dump(mode="json"),
                    )
                os.rename(name, target_name, src_dir_fd=source_fd, dst_dir_fd=target_root_fd)
                _fsync_directory_fd(source_fd)
                _set_safe_mode_and_fsync(target_root_fd, target_name)
                _fsync_directory_fd(target_root_fd)
                _fsync_directory_fd(quarantine_fd)
            finally:
                os.close(target_root_fd)
        finally:
            os.close(quarantine_fd)

    def _experiment_directory(self) -> Path:
        if (
            _path_is_symlink(self.root)
            or not self.root.is_dir()
            or not _path_is_regular(self.root / "manifest.json")
        ):
            raise ValueError("store root is not a complete experiment")
        return self.root

    def _attempt_path(self, context: AttemptContext) -> Path:
        path = self.root / "attempts" / _safe(context.identity.job_id) / _safe(context.attempt_id)
        if any(_path_is_symlink(item) for item in (path.parent.parent, path.parent, path)):
            raise ValueError("attempt path contains a symlink")
        return path

    @contextmanager
    def _open_attempt_directory(self, context: AttemptContext) -> Iterator[int]:
        with _open_directory(self.root) as root_fd:
            attempts_fd = _open_required_directory(root_fd, "attempts")
            try:
                job_fd = _open_required_directory(attempts_fd, _safe(context.identity.job_id))
                try:
                    attempt_fd = _open_required_directory(job_fd, _safe(context.attempt_id))
                    try:
                        yield attempt_fd
                    finally:
                        os.close(attempt_fd)
                finally:
                    os.close(job_fd)
            finally:
                os.close(attempts_fd)

    @staticmethod
    def _write_new(path: Path, value: JsonValue) -> None:
        if path.exists():
            raise FileExistsError(path)
        with path.open("xb") as stream:
            stream.write(canonical_json(cast("JsonValue", value)) + b"\n")
            stream.flush()
            os.fsync(stream.fileno())
        _fsync_directory(path.parent)

    @staticmethod
    def _write_new_at(parent_fd: int, name: str, value: JsonValue) -> None:
        payload = canonical_json(cast("JsonValue", value)) + b"\n"
        descriptor = os.open(
            name,
            os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW,
            0o600,
            dir_fd=parent_fd,
        )
        try:
            written = 0
            while written < len(payload):
                written += os.write(descriptor, payload[written:])
            os.fsync(descriptor)
        finally:
            os.close(descriptor)
        _fsync_directory_fd(parent_fd)


def _fsync_directory(path: Path) -> None:
    descriptor = os.open(path, os.O_RDONLY)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


ModelT = TypeVar("ModelT", bound=SpatialModel)


@contextmanager
def _open_directory(path: Path) -> Iterator[int]:
    descriptor = os.open(path, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW)
    try:
        yield descriptor
    finally:
        os.close(descriptor)


@contextmanager
def _fd_handle(descriptor: int) -> Iterator[int]:
    try:
        yield descriptor
    finally:
        os.close(descriptor)


def _open_optional_directory(parent_fd: int, name: str) -> int | None:
    try:
        metadata = os.stat(name, dir_fd=parent_fd, follow_symlinks=False)
    except FileNotFoundError:
        return None
    if not stat.S_ISDIR(metadata.st_mode) or stat.S_ISLNK(metadata.st_mode):
        return None
    try:
        return os.open(name, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW, dir_fd=parent_fd)
    except (FileNotFoundError, NotADirectoryError, OSError):
        return None


def _open_required_directory(parent_fd: int, name: str) -> int:
    descriptor = _open_optional_directory(parent_fd, name)
    if descriptor is None:
        raise FileNotFoundError(name)
    return descriptor


def _entry_is_symlink(parent_fd: int, name: str) -> bool:
    try:
        return stat.S_ISLNK(os.stat(name, dir_fd=parent_fd, follow_symlinks=False).st_mode)
    except FileNotFoundError:
        return False


def _entry_exists(parent_fd: int, name: str) -> bool:
    try:
        os.stat(name, dir_fd=parent_fd, follow_symlinks=False)
        return True
    except FileNotFoundError:
        return False


def _attempt_number_from_name(name: str) -> int | None:
    match = re.fullmatch(r"(?:\.?)attempt-([0-9]+)(?:\.[A-Za-z0-9-]+)+", name)
    if match is None:
        match = re.fullmatch(r"attempt-([0-9]+)", name)
    if match is None:
        return None
    number = int(match.group(1))
    return number if 0 < number <= 1_000_000_000 else None


def _canonical_active_attempt_number(name: str) -> int | None:
    """Return a reservation only for a published, canonical attempt name."""
    number = _attempt_number_from_name(name)
    if number is None or name != f"attempt-{number}":
        return None
    return number


def _read_fd_bytes(parent_fd: int, name: str) -> bytes:
    metadata = os.stat(name, dir_fd=parent_fd, follow_symlinks=False)
    if not stat.S_ISREG(metadata.st_mode):
        raise ValueError(f"record is not a regular file: {name}")
    descriptor = os.open(name, os.O_RDONLY | os.O_NOFOLLOW, dir_fd=parent_fd)
    try:
        chunks: list[bytes] = []
        while chunk := os.read(descriptor, 1024 * 1024):
            chunks.append(chunk)
        return b"".join(chunks)
    finally:
        os.close(descriptor)


def _load_definition_at(root_fd: int) -> LoadedDefinition:
    """Load and validate definition bytes relative to an already pinned root."""
    plan_bytes = _read_fd_bytes(root_fd, "plan.json")
    manifest_bytes = _read_fd_bytes(root_fd, "manifest.json")
    plan = ExperimentPlan.model_validate_json(plan_bytes)
    manifest = ExperimentManifest.model_validate_json(manifest_bytes)
    if plan_bytes != canonical_json(plan.model_dump(mode="json")) + b"\n":
        raise ValueError("plan bytes are not canonical")
    if manifest_bytes != canonical_manifest_bytes(manifest):
        raise ValueError("manifest bytes are not canonical")
    validate_manifest(manifest, plan)
    return LoadedDefinition(
        manifest=manifest,
        plan=plan,
        manifest_digest=canonical_manifest_digest(manifest_bytes),
        plan_digest=plan.plan_digest,
        plan_file_digest=hashlib.sha256(plan_bytes).hexdigest(),
    )


def _observe_attempts_at(
    root_fd: int, definition: LoadedDefinition, job_id_value: str
) -> tuple[RecoveredAttempt, ...]:
    experiment_id = definition.manifest.experiment_id
    identifier = _safe(job_id_value)
    if _entry_is_symlink(root_fd, "attempts"):
        raise ValueError("attempt tree contains a symlink")
    attempts_fd = _open_optional_directory(root_fd, "attempts")
    if attempts_fd is None:
        return ()
    try:
        if _entry_is_symlink(attempts_fd, identifier):
            raise ValueError("attempt tree contains a symlink")
        job_fd = _open_optional_directory(attempts_fd, identifier)
        if job_fd is None:
            return ()
        try:
            names = os.listdir(job_fd)
            numeric = sorted(
                (name for name in names if _canonical_active_attempt_number(name) is not None),
                key=lambda name: _canonical_active_attempt_number(name) or 0,
            )
            for name in names:
                if _entry_is_symlink(job_fd, name):
                    raise ValueError("attempt tree contains a symlink")
            result: list[RecoveredAttempt] = []
            for name in numeric:
                attempt_fd = _open_optional_directory(job_fd, name)
                if attempt_fd is None:
                    raise ValueError("attempt tree is not safely observable")
                try:
                    try:
                        context = _read_model_at(attempt_fd, "context.json", AttemptContext)
                        snapshot = _read_model_at(
                            attempt_fd, "attempt-manifest.v1.json", AttemptManifestSnapshot
                        )
                        case = _read_model_at(attempt_fd, "case.json", ExpandedCase)
                        condition = _read_model_at(attempt_fd, "condition.json", NamedCondition)
                        outcome = _read_optional_model_at(
                            attempt_fd, "outcome.v1.json", TerminalOutcome
                        )
                        expected_case = next(
                            item for item in definition.plan.cases if item.case_id == case.case_id
                        )
                        expected_condition = next(
                            item
                            for item in definition.plan.conditions
                            if item.name == condition.name
                        )
                        expected_job_id = job_id(definition.plan, expected_case, expected_condition)
                        if (
                            context.attempt_id != name
                            or context.directory_name != name
                            or context.attempt_number != _canonical_active_attempt_number(name)
                            or context.identity.experiment_id != experiment_id
                            or context.identity.job_id != identifier
                            or context.identity.case_id != case.case_id
                            or context.identity.condition_name != condition.name
                            or expected_job_id != identifier
                            or case != expected_case
                            or condition != expected_condition
                            or context.manifest_digest != definition.manifest_digest
                            or snapshot.identity != context.identity
                            or snapshot.attempt_id != context.attempt_id
                            or snapshot.manifest_digest != definition.manifest_digest
                            or snapshot.manifest != definition.manifest.model_dump(mode="json")
                        ):
                            raise ValueError("attempt identity or manifest mismatch")
                        result.append(
                            RecoveredAttempt(
                                context, case, condition, outcome, Path("attempts") / identifier / name
                            )
                        )
                    except Exception:
                        continue
                finally:
                    os.close(attempt_fd)
            return tuple(result)
        finally:
            os.close(job_fd)
    finally:
        os.close(attempts_fd)


def _read_model_at(parent_fd: int, name: str, model: type[ModelT]) -> ModelT:
    return model.model_validate_json(_read_fd_bytes(parent_fd, name))


def _read_optional_model_at(
    parent_fd: int, name: str, model: type[ModelT]
) -> ModelT | None:
    try:
        return _read_model_at(parent_fd, name, model)
    except FileNotFoundError:
        return None


def _read_events_at(parent_fd: int) -> tuple[OperationalEvent, ...]:
    return tuple(
        OperationalEvent.model_validate_json(line)
        for line in _read_fd_bytes(parent_fd, "events.jsonl").splitlines()
    )


def _mkdir_open_at(parent_fd: int, name: str) -> int:
    try:
        os.mkdir(name, mode=0o700, dir_fd=parent_fd)
    except FileExistsError:
        metadata = os.stat(name, dir_fd=parent_fd, follow_symlinks=False)
        if not stat.S_ISDIR(metadata.st_mode):
            raise ValueError(f"quarantine entry is not a directory: {name}")
    descriptor = os.open(name, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW, dir_fd=parent_fd)
    _fsync_directory_fd(parent_fd)
    return descriptor


def _fsync_directory_fd(descriptor: int) -> None:
    os.fsync(descriptor)


def _set_safe_mode_and_fsync(parent_fd: int, name: str) -> None:
    metadata = os.stat(name, dir_fd=parent_fd, follow_symlinks=False)
    if stat.S_ISLNK(metadata.st_mode):
        return
    if stat.S_ISDIR(metadata.st_mode):
        descriptor = os.open(name, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW, dir_fd=parent_fd)
        mode = 0o700
    elif stat.S_ISREG(metadata.st_mode):
        descriptor = os.open(name, os.O_RDONLY | os.O_NOFOLLOW, dir_fd=parent_fd)
        mode = 0o600
    else:
        return
    try:
        os.fchmod(descriptor, mode)
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def _path_is_symlink(path: Path) -> bool:
    try:
        return stat.S_ISLNK(os.lstat(path).st_mode)
    except FileNotFoundError:
        return False


def _path_exists_nofollow(path: Path) -> bool:
    try:
        os.lstat(path)
        return True
    except FileNotFoundError:
        return False


def _path_is_regular(path: Path) -> bool:
    try:
        metadata = os.lstat(path)
    except FileNotFoundError:
        return False
    return stat.S_ISREG(metadata.st_mode)


def _read_regular_bytes(path: Path) -> bytes:
    metadata = os.lstat(path)
    if not stat.S_ISREG(metadata.st_mode):
        raise ValueError(f"record is not a regular file: {path.name}")
    descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW)
    try:
        with os.fdopen(descriptor, "rb") as stream:
            descriptor = -1
            return stream.read()
    finally:
        if descriptor >= 0:
            os.close(descriptor)


def _fsync_inode(path: Path) -> None:
    flags = os.O_RDONLY | getattr(os, "O_NOFOLLOW", 0)
    try:
        descriptor = os.open(path, flags)
    except OSError:
        descriptor = os.open(path, getattr(os, "O_PATH", os.O_RDONLY) | os.O_NOFOLLOW)
    try:
        try:
            os.fsync(descriptor)
        except OSError:
            # Linux does not fsync symlink inodes opened with O_PATH; the
            # source/destination directory fsyncs still durably publish the rename.
            pass
    finally:
        os.close(descriptor)


def _remove_tree(path: Path) -> None:
    for child in path.iterdir():
        if child.is_dir():
            _remove_tree(child)
        else:
            child.unlink()
    path.rmdir()
