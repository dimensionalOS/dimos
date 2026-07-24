# Copyright 2026 Dimensional Inc.
"""Evidence manifests with strict public/private artifact separation."""

from __future__ import annotations

import ctypes
from dataclasses import dataclass
from enum import Enum
import errno
import hashlib
import os
from pathlib import Path
import stat
from typing import Literal
import uuid

from pydantic import Field

from dimos.benchmark.spatial.models import SpatialModel
from dimos.benchmark.spatial.pi_baseline.records import Prediction
from dimos.benchmark.spatial.pi_baseline.scoring import PrivateScore
from dimos.benchmark.spatial.pi_baseline.topology import PinnedDirectory
from dimos.benchmark.spatial.utilities import canonical_json, validate_relative_path

MAX_EVIDENCE_ARTIFACT_BYTES = 64 * 1024 * 1024
MAX_EVIDENCE_ARTIFACT_COUNT = 128
# Backwards-compatible spelling for non-authoritative callers.
MAX_EVIDENCE_ARTIFACTS = MAX_EVIDENCE_ARTIFACT_COUNT
MAX_EVIDENCE_TOTAL_BYTES = 256 * 1024 * 1024
PUBLIC_PROJECTION_PATHS = frozenset(
    {
        "case.v1.json",
        "map.lcm",
        "provenance.v1.json",
        "result.v1.json",
        "verification.v1.json",
    }
)
AUTHORITATIVE_MANIFEST_NAME = "evidence-manifest.v1.json"
PROVISIONAL_MANIFEST_NAME = "evidence-manifest.provisional.v1.json"
_ORIGINAL_UNLINK = os.unlink


def _rename_noreplace(source: str, destination: str, *, dir_fd: int) -> None:
    """Linux ``renameat2(RENAME_NOREPLACE)`` with one directory anchor."""
    libc = ctypes.CDLL(None, use_errno=True)
    renameat2 = getattr(libc, "renameat2", None)
    if renameat2 is None:
        raise OSError(38, "renameat2 is unavailable")
    renameat2.argtypes = [
        ctypes.c_int,
        ctypes.c_char_p,
        ctypes.c_int,
        ctypes.c_char_p,
        ctypes.c_uint,
    ]
    renameat2.restype = ctypes.c_int
    result = renameat2(dir_fd, os.fsencode(source), dir_fd, os.fsencode(destination), 1)
    if result != 0:
        error = ctypes.get_errno()
        if error == errno.EEXIST:
            raise FileExistsError(error, os.strerror(error), destination)
        raise OSError(error, os.strerror(error), destination)


class EvidenceArtifact(SpatialModel):
    record_type: Literal["evidence-artifact"] = "evidence-artifact"
    path: str = Field(min_length=1)
    sha256: str = Field(pattern=r"^[0-9a-f]{64}$")
    size_bytes: int = Field(ge=0)


class EvidenceManifest(SpatialModel):
    record_type: Literal["evidence-manifest"] = "evidence-manifest"
    schema_version: Literal["1.0"] = "1.0"
    public: tuple[EvidenceArtifact, ...]
    private: tuple[EvidenceArtifact, ...]


class EvidencePublishDisposition(str, Enum):
    """The outcome of attempting to publish the one canonical commit marker."""

    COMMITTED = "committed"
    ALREADY_EXISTING = "already-existing"
    CONFLICT = "conflict"
    DURABILITY_UNKNOWN = "durability-unknown"


@dataclass(frozen=True)
class CommittedEvidenceResult:
    """The validated, committed result admitted by an evidence manifest."""

    manifest: EvidenceManifest
    prediction: Prediction
    score: PrivateScore


@dataclass(frozen=True)
class EvidenceIdentityContext:
    """Execution identity required for admission of a result."""

    run_id: str
    case_id: str
    mode: str
    release_id: str
    scorer_revision: str


def build_evidence_manifest(
    public_root: Path | PinnedDirectory,
    private_root: Path | PinnedDirectory,
    *,
    public_artifacts: tuple[str, ...],
    private_artifacts: tuple[str, ...],
    required_public: tuple[str, ...] = (),
    required_private: tuple[str, ...] = (),
) -> EvidenceManifest:
    public, public_owned = _as_pinned(public_root)
    private, private_owned = _as_pinned(private_root)
    try:
        selected_public = _select_artifacts(public_artifacts, required_public)
        selected_private = _select_artifacts(private_artifacts, required_private)
        _preflight_artifact_plan((*selected_public, *selected_private))
        budget = _ArtifactBudget()
        records_public = _build_artifacts(public, selected_public, budget)
        records_private = _build_artifacts(private, selected_private, budget)
        _assert_public_safe(public, records_public)
        return EvidenceManifest(public=records_public, private=records_private)
    finally:
        if public_owned:
            public.close()
        if private_owned:
            private.close()


def write_evidence_manifest(
    directory: PinnedDirectory | Path,
    manifest: EvidenceManifest,
    name: str = PROVISIONAL_MANIFEST_NAME,
) -> None:
    if name == AUTHORITATIVE_MANIFEST_NAME:
        raise ValueError("authoritative evidence manifest is publish-only")
    payload = canonical_json(manifest.model_dump(mode="json")) + b"\n"
    if isinstance(directory, PinnedDirectory):
        directory.write_bytes(name, payload)
        return
    pinned = PinnedDirectory.open(directory, create=False)
    try:
        pinned.write_bytes(name, payload)
    finally:
        pinned.close()


def publish_evidence_manifest_noreplace(
    directory: PinnedDirectory,
    manifest: EvidenceManifest,
    *,
    public_root: Path | PinnedDirectory | None = None,
    expected_identity: EvidenceIdentityContext,
) -> EvidencePublishDisposition:
    _validate_identity_context(expected_identity)
    if public_root is None:
        return _publish_with_roots(directory, directory, manifest, expected_identity)
    public, owned = _as_pinned(public_root)
    try:
        return _publish_with_roots(directory, public, manifest, expected_identity)
    finally:
        if owned:
            public.close()


def _publish_with_roots(
    directory: PinnedDirectory,
    public: PinnedDirectory,
    manifest: EvidenceManifest,
    expected_identity: EvidenceIdentityContext,
) -> EvidencePublishDisposition:
    """Publish ``manifest`` as an immutable, no-replace commit marker.

    A successful link is never undone.  Consequently a parent-directory fsync
    failure is reported separately: the marker is present, but persistence is
    not known to have reached stable storage.
    """
    final_name = AUTHORITATIVE_MANIFEST_NAME
    directory.verify()
    _admit_manifest(public, directory, manifest, expected_identity)
    try:
        _sync_manifest_artifacts(public, directory, manifest)
    except OSError:
        # No marker has linearized yet; durability is not established and the
        # caller must not mistake this attempt for a committed result.
        return EvidencePublishDisposition.DURABILITY_UNKNOWN
    payload = canonical_json(manifest.model_dump(mode="json")) + b"\n"
    _parse_canonical_manifest(payload)
    temp_name = f".{final_name}.tmp-{os.getpid()}-{uuid.uuid4().hex}"
    descriptor = -1
    linked = False
    try:
        descriptor = os.open(
            temp_name,
            os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW | os.O_CLOEXEC,
            0o600,
            dir_fd=directory.fd,
        )
        view = memoryview(payload)
        while view:
            view = view[os.write(descriptor, view) :]
        os.fsync(descriptor)
        os.close(descriptor)
        descriptor = -1
        try:
            _rename_noreplace(
                temp_name,
                final_name,
                dir_fd=directory.fd,
            )
            linked = True
        except FileExistsError:
            try:
                existing = _read_owned_file(directory, final_name)
                existing_manifest = _parse_canonical_manifest(existing)
                _admit_manifest(public, directory, existing_manifest, expected_identity)
            except Exception:
                return EvidencePublishDisposition.CONFLICT
            if existing != payload:
                return EvidencePublishDisposition.CONFLICT
            try:
                os.fsync(directory.fd)
            except BaseException:
                return EvidencePublishDisposition.DURABILITY_UNKNOWN
            return EvidencePublishDisposition.ALREADY_EXISTING
        # renameat2 consumed the temporary directory entry.  There is no
        # second link and therefore no cleanup window after linearization.
        post_link_failure = False
        try:
            os.fsync(directory.fd)
        except BaseException:
            post_link_failure = True
        if post_link_failure:
            return EvidencePublishDisposition.DURABILITY_UNKNOWN
        return EvidencePublishDisposition.COMMITTED
    finally:
        if descriptor >= 0:
            os.close(descriptor)
        if not linked:
            _cleanup_temp(directory, temp_name)


def load_committed_result(
    public_root: Path | PinnedDirectory,
    private_root: Path | PinnedDirectory,
    *,
    expected_identity: EvidenceIdentityContext,
) -> CommittedEvidenceResult | None:
    """Load only a result bound by a valid canonical evidence commit marker."""
    _validate_identity_context(expected_identity)
    public, own_public = _as_pinned(public_root)
    private, own_private = _as_pinned(private_root)
    try:
        try:
            marker = _read_owned_file(private, "evidence-manifest.v1.json")
        except FileNotFoundError:
            return None
        manifest = _parse_canonical_manifest(marker)
        prediction, score = _admit_manifest(public, private, manifest, expected_identity)
        return CommittedEvidenceResult(manifest, prediction, score)
    finally:
        if own_public:
            public.close()
        if own_private:
            private.close()


def _parse_manifest(data: bytes) -> EvidenceManifest:
    try:
        return EvidenceManifest.model_validate_json(data)
    except Exception as error:
        raise ValueError("invalid evidence manifest") from error


def _parse_canonical_manifest(data: bytes) -> EvidenceManifest:
    manifest = _parse_manifest(data)
    if data != canonical_json(manifest.model_dump(mode="json")) + b"\n":
        raise ValueError("evidence manifest is not canonical")
    return manifest


@dataclass(frozen=True)
class _Snapshot:
    data: bytes
    sha256: str
    size_bytes: int


@dataclass
class _ArtifactBudget:
    count: int = 0
    total_bytes: int = 0

    def add(self, size: int) -> None:
        self.count += 1
        self.total_bytes += size
        if self.count > MAX_EVIDENCE_ARTIFACT_COUNT:
            raise ValueError("evidence artifact count exceeds limit")
        if self.total_bytes > MAX_EVIDENCE_TOTAL_BYTES:
            raise ValueError("evidence aggregate size exceeds limit")


def _select_artifacts(names: tuple[str, ...], required: tuple[str, ...]) -> tuple[str, ...]:
    return tuple(dict.fromkeys((*names, *required)))


def _preflight_artifact_plan(
    paths: tuple[str, ...], *, declared: tuple[int, ...] | None = None
) -> None:
    if len(paths) > MAX_EVIDENCE_ARTIFACT_COUNT:
        raise ValueError("evidence artifact count exceeds limit")
    normalized = tuple(validate_relative_path(path) for path in paths)
    if len(normalized) != len(set(normalized)):
        raise ValueError("manifest contains duplicate artifact paths")
    if declared is not None and sum(declared) > MAX_EVIDENCE_TOTAL_BYTES:
        raise ValueError("evidence aggregate size exceeds limit")


def _open_owned_for_sync(
    root: PinnedDirectory, relative: str
) -> tuple[int, tuple[tuple[tuple[int, int], int], ...]]:
    """Open an admitted file and duplicate every descriptor-pinned ancestor."""
    parts = validate_relative_path(relative).split("/")
    parent = root
    opened: list[PinnedDirectory] = []
    file_fd = -1
    directory_fds: list[tuple[tuple[int, int], int]] = []
    try:
        for part in parts[:-1]:
            parent = PinnedDirectory.open_at(parent, part)
            opened.append(parent)
        file_fd = os.open(parts[-1], os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC, dir_fd=parent.fd)
        info = os.fstat(file_fd)
        if (
            not stat.S_ISREG(info.st_mode)
            or info.st_uid != os.geteuid()
            or stat.S_IMODE(info.st_mode) != 0o600
            or info.st_nlink != 1
        ):
            raise ValueError(f"unsafe evidence artifact: {relative}")
        for ancestor in (root, *opened):
            descriptor = os.dup(ancestor.fd)
            directory_fds.append(((ancestor.device, ancestor.inode), descriptor))
        return file_fd, tuple(directory_fds)
    except BaseException:
        if file_fd >= 0:
            os.close(file_fd)
        for _, descriptor in directory_fds:
            os.close(descriptor)
        raise
    finally:
        for ancestor in reversed(opened):
            ancestor.close()


def _sync_manifest_artifacts(
    public: PinnedDirectory, private: PinnedDirectory, manifest: EvidenceManifest
) -> None:
    directory_fds: dict[tuple[int, int], int] = {}
    try:
        for is_private, artifacts in ((False, manifest.public), (True, manifest.private)):
            root = private if is_private else public
            for artifact in artifacts:
                file_fd, artifact_directories = _open_owned_for_sync(root, artifact.path)
                for key, directory_fd in artifact_directories:
                    old = directory_fds.get(key)
                    if old is None:
                        directory_fds[key] = directory_fd
                    else:
                        os.close(directory_fd)
                try:
                    os.fsync(file_fd)
                finally:
                    os.close(file_fd)
        for directory_fd in directory_fds.values():
            os.fsync(directory_fd)
    finally:
        for directory_fd in directory_fds.values():
            os.close(directory_fd)


def _read_owned_snapshot(root: PinnedDirectory, relative: str, *, retain: bool) -> _Snapshot:
    parts = validate_relative_path(relative).split("/")
    parent = root
    opened: PinnedDirectory | None = None
    try:
        if len(parts) > 1:
            opened = root.open_relative("/".join(parts[:-1]))
            parent = opened
        fd = os.open(parts[-1], os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC, dir_fd=parent.fd)
        try:
            info = os.fstat(fd)
            before = info
            if (
                not stat.S_ISREG(info.st_mode)
                or info.st_uid != os.geteuid()
                or stat.S_IMODE(info.st_mode) != 0o600
                or info.st_nlink != 1
                or info.st_size > MAX_EVIDENCE_ARTIFACT_BYTES
            ):
                raise ValueError(f"unsafe evidence artifact: {relative}")
            chunks: list[bytes] = []
            digest = hashlib.sha256()
            total = 0
            while True:
                chunk = os.read(fd, 1024 * 1024)
                if not chunk:
                    break
                total += len(chunk)
                if total > MAX_EVIDENCE_ARTIFACT_BYTES:
                    raise ValueError(f"evidence artifact exceeds size limit: {relative}")
                digest.update(chunk)
                if retain:
                    chunks.append(chunk)
            after = os.fstat(fd)
            identity = ("st_dev", "st_ino", "st_size", "st_mtime_ns", "st_ctime_ns")
            if (
                any(getattr(before, field) != getattr(after, field) for field in identity)
                or after.st_uid != os.geteuid()
                or stat.S_IMODE(after.st_mode) != 0o600
                or after.st_nlink != 1
            ):
                raise ValueError(f"evidence artifact changed while being read: {relative}")
            return _Snapshot(b"".join(chunks), digest.hexdigest(), total)
        finally:
            os.close(fd)
    finally:
        if opened is not None:
            opened.close()


def _read_owned_file(root: PinnedDirectory, relative: str) -> bytes:
    return _read_owned_snapshot(root, relative, retain=True).data


def _cleanup_temp(directory: PinnedDirectory, name: str) -> None:
    try:
        os.unlink(name, dir_fd=directory.fd)
    except OSError:
        try:
            _ORIGINAL_UNLINK(name, dir_fd=directory.fd)
        except OSError:
            pass


def _admit_manifest(
    public: PinnedDirectory,
    private: PinnedDirectory,
    manifest: EvidenceManifest,
    expected: EvidenceIdentityContext,
) -> tuple[Prediction, PrivateScore]:
    _validate_identity_context(expected)
    artifacts = (*manifest.public, *manifest.private)
    _preflight_artifact_plan(
        tuple(item.path for item in artifacts),
        declared=tuple(item.size_bytes for item in artifacts),
    )
    paths = [validate_relative_path(item.path) for item in artifacts]
    if len(paths) != len(set(paths)):
        raise ValueError("manifest contains duplicate artifact paths")
    if "evidence-manifest.v1.json" in paths:
        raise ValueError("evidence manifest cannot reference itself")
    private_paths = {item.path for item in manifest.private}
    if private_paths & {"prediction.v1.json", "score.v1.json"} != {
        "prediction.v1.json",
        "score.v1.json",
    }:
        raise ValueError("manifest must reference exact prediction.v1.json and score.v1.json")
    if any(
        Path(item.path).name in {"prediction.v1.json", "score.v1.json"}
        and item.path not in {"prediction.v1.json", "score.v1.json"}
        for item in manifest.private
    ):
        raise ValueError("prediction and score must use canonical private paths")
    snapshots: dict[tuple[bool, str], bytes] = {}
    actual_total = 0
    for is_private, group in ((False, manifest.public), (True, manifest.private)):
        root = private if is_private else public
        for artifact in group:
            if not is_private:
                _assert_public_projection(artifact.path)
            retain = is_private and artifact.path in {
                "prediction.v1.json",
                "score.v1.json",
            }
            snapshot = _read_owned_snapshot(root, artifact.path, retain=retain)
            actual_total += snapshot.size_bytes
            if actual_total > MAX_EVIDENCE_TOTAL_BYTES:
                raise ValueError("evidence aggregate size exceeds limit")
            if snapshot.size_bytes != artifact.size_bytes or snapshot.sha256 != artifact.sha256:
                raise ValueError(f"tampered evidence artifact: {artifact.path}")
            if retain:
                snapshots[(is_private, artifact.path)] = snapshot.data
    prediction = Prediction.model_validate_json(snapshots[(True, "prediction.v1.json")])
    score = PrivateScore.model_validate_json(snapshots[(True, "score.v1.json")])
    if (
        prediction.instance_id != score.instance_id
        or prediction.answer_type != score.answer_type
        or prediction.value != score.value
        or score.case_id != prediction.instance_id
    ):
        raise ValueError("prediction and score identities or values do not match")
    if expected is not None and (
        score.run_id != expected.run_id
        or score.case_id != expected.case_id
        or prediction.instance_id != expected.case_id
        or score.mode != expected.mode
        or score.release_id != expected.release_id
        or score.scorer_revision != expected.scorer_revision
    ):
        raise ValueError("evidence result does not match expected identity")
    return prediction, score


def _validate_identity_context(expected: EvidenceIdentityContext | None) -> None:
    if not isinstance(expected, EvidenceIdentityContext) or any(
        not isinstance(value, str) or not value
        for value in (
            expected.run_id,
            expected.case_id,
            expected.mode,
            expected.release_id,
            expected.scorer_revision,
        )
    ):
        raise ValueError("complete evidence identity is required")


def _assert_public_projection(path: str) -> None:
    safe = validate_relative_path(path)
    if safe not in PUBLIC_PROJECTION_PATHS:
        raise ValueError(f"public evidence is not an approved projection: {path}")


def _build_artifacts(
    root: PinnedDirectory, names: tuple[str, ...], budget: _ArtifactBudget
) -> tuple[EvidenceArtifact, ...]:
    result: list[EvidenceArtifact] = []
    for name in names:
        safe = validate_relative_path(name)
        try:
            snapshot = _read_owned_snapshot(root, safe, retain=False)
        except Exception as error:
            raise ValueError(f"required evidence artifact is missing: {name}") from error
        budget.add(snapshot.size_bytes)
        result.append(
            EvidenceArtifact(
                path=safe,
                sha256=snapshot.sha256,
                size_bytes=snapshot.size_bytes,
            )
        )
    return tuple(result)


def _artifacts(
    root: PinnedDirectory, names: tuple[str, ...], budget: _ArtifactBudget
) -> tuple[EvidenceArtifact, ...]:
    """Compatibility alias for the bounded builder primitive."""
    return _build_artifacts(root, names, budget)


def _assert_public_safe(root: PinnedDirectory, artifacts: tuple[EvidenceArtifact, ...]) -> None:
    for artifact in artifacts:
        _assert_public_projection(artifact.path)


def _as_pinned(value: Path | PinnedDirectory) -> tuple[PinnedDirectory, bool]:
    if isinstance(value, PinnedDirectory):
        return value, False
    return PinnedDirectory.open(value, create=False), True
