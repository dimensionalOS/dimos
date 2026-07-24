"""Creation, binding, and private review/report helpers for the Pi CLI."""

from __future__ import annotations

import base64
from collections.abc import Callable
from dataclasses import dataclass
from datetime import UTC, datetime
import hashlib
import json
import os
from pathlib import Path
import tempfile
import traceback
from typing import TYPE_CHECKING, Literal, cast
from uuid import uuid4

from dimos.benchmark.spatial.models import AnswerType, SpatialModel
from dimos.benchmark.spatial.pi_baseline.config import AuthMode, PiBaselineConfig, load_config
from dimos.benchmark.spatial.pi_baseline.evidence import (
    EvidenceIdentityContext,
    load_committed_result,
)
from dimos.benchmark.spatial.pi_baseline.prompts import build_prompt_pair
from dimos.benchmark.spatial.pi_baseline.scheduler_executor import ExecutionInterrupted
from dimos.benchmark.spatial.pi_baseline.scheduler_models import (
    ExpandedCase,
    ExperimentManifest,
    ExperimentPlan,
    JobSummary,
    ReviewDecision,
)
from dimos.benchmark.spatial.pi_baseline.scheduler_pi_binding import (
    PiAdmissionContext,
    PiExecutionSnapshot,
    PiInventoryRecord,
    PiMaterialRecord,
    PiRuntimeBindings,
    PiSelectedInput,
    host_module_paths,
    private_binding_digest,
    runtime_adapter_artifacts,
    verify_public_inputs,
    verify_runtime_artifacts,
    verify_snapshot_artifacts,
)
from dimos.benchmark.spatial.pi_baseline.scheduler_pi_executor import (
    PiCasePayload,
    PiConditionPayload,
    _condition_run_id,
)
from dimos.benchmark.spatial.pi_baseline.scheduler_plan import (
    expand_plan,
    selected_inputs_digest as plan_selected_inputs_digest,
)
from dimos.benchmark.spatial.pi_baseline.scheduler_store import (
    CoordinatorLeaseCapability,
    FilesystemExperimentStore,
)
from dimos.benchmark.spatial.utilities import JsonValue, canonical_json, hash_file_sha256

if TYPE_CHECKING:
    from dimos.benchmark.spatial.pi_baseline.scheduler_runtime import SchedulerRuntime


class ReportRecord(SpatialModel):
    """Private, immutable report envelope; scores never enter scheduler state."""

    record_type: str = "pi-private-report"
    schema_version: str = "1.0"
    experiment_id: str
    manifest_digest: str
    review_decision_digest: str
    scores: tuple[dict[str, object], ...]


@dataclass(frozen=True)
class PiPreflightResult:
    """Frozen, non-mutating result of authoritative Pi admission checks."""

    manifest: ExperimentManifest
    plan: ExperimentPlan
    snapshot: PiExecutionSnapshot
    store: FilesystemExperimentStore
    manifest_digest: str
    private_binding_digest: str
    admission_context: PiAdmissionContext


def _digest(value: JsonValue) -> str:
    return hashlib.sha256(canonical_json(value)).hexdigest()


def _canonical_file_digest(path: Path) -> str:
    return _digest(cast("JsonValue", json.loads(path.read_text(encoding="utf-8"))))


def _load_json(path: Path) -> dict[str, object]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain a JSON object")
    return cast("dict[str, object]", value)


def _fingerprint(label: str, snapshot: PiExecutionSnapshot, plan: ExperimentPlan) -> str:
    return _digest(
        {
            "label": label,
            "snapshot": snapshot.canonical_digest(),
            "plan": plan.plan_digest,
        }
    )


def create_definition(
    experiment_dir: Path,
    spec_path: Path,
    *,
    workers: int | None,
    sample: int | None,
    shard: int,
    shards: int,
) -> tuple[ExperimentManifest, ExperimentPlan, PiExecutionSnapshot, PiBaselineConfig]:
    spec = _load_json(spec_path)
    experiment_id = str(spec.get("experiment_id", experiment_dir.name))
    config_value = spec.get("pi_config", spec.get("config"))
    if not isinstance(config_value, str):
        raise ValueError("authoring spec requires a pi_config path")
    config_path = (spec_path.parent / config_value).resolve()
    config = load_config(config_path)

    raw_cases = spec.get("cases")
    raw_conditions = spec.get("conditions")
    if not isinstance(raw_cases, list) or not isinstance(raw_conditions, list):
        raise ValueError("authoring spec requires cases and conditions arrays")
    cases: list[ExpandedCase] = []
    for raw in raw_cases:
        if not isinstance(raw, dict):
            raise ValueError("each case must be an object")
        case = dict(raw)
        if "payload" not in case:
            case = {"case_id": case.get("case_id"), "payload": {"selection": case.get("selection")}}
        expanded = ExpandedCase.model_validate(case)
        PiCasePayload.model_validate(expanded.payload)
        cases.append(expanded)
    from dimos.benchmark.spatial.pi_baseline.scheduler_models import NamedCondition

    conditions: list[NamedCondition] = []
    for raw in raw_conditions:
        if not isinstance(raw, dict):
            raise ValueError("each condition must be an object")
        condition = dict(raw)
        if "payload" not in condition:
            condition = {
                "name": condition.get("name"),
                "payload": {"prompt_mode": condition.get("prompt_mode")},
            }
        named = NamedCondition.model_validate(condition)
        PiConditionPayload.model_validate(named.payload)
        conditions.append(named)

    plan = expand_plan(
        experiment_id,
        cases,
        conditions,
        workers=workers if workers is not None else 10,
        sample=sample,
        shard=shard,
        shards=shards,
    )
    selected_inputs: list[PiSelectedInput] = []
    with tempfile.TemporaryDirectory(prefix="pi-create-") as directory:
        from dimos.benchmark.spatial.pi_baseline.projection import stage_public_instance

        for case in plan.cases:
            selection = PiCasePayload.model_validate(case.payload).selection
            staged = stage_public_instance(
                Path(config.corpus_root), Path(directory), **selection.model_dump()
            )
            staging_inventory = tuple(
                sorted(
                    (
                        PiInventoryRecord(
                            path=path.relative_to(staged).as_posix(),
                            sha256=hash_file_sha256(path),
                        )
                        for path in staged.rglob("*")
                        if path.is_file()
                    ),
                    key=lambda item: item.path,
                )
            )
            staging_manifest = _load_json(staged / "staging-manifest.v1.json")
            staged_case = _load_json(staged / "cases" / "case.v1.json")
            question = staged_case.get("question")
            if not isinstance(question, dict) or not isinstance(question.get("answer_type"), str):
                raise ValueError("selected public case has no answer type")
            answer_type = question["answer_type"]
            selected_type = AnswerType(answer_type)
            schema_hash = hash_file_sha256(staged / "schema.v1.json")
            release_hash = _digest(cast("JsonValue", staging_manifest["release"]))
            selected_inputs.append(
                PiSelectedInput(
                    case_id=case.case_id,
                    selection=selection.model_dump(),
                    answer_type=selected_type,
                    case_sha256=hash_file_sha256(staged / "cases" / "case.v1.json"),
                    map_sha256=hash_file_sha256(staged / "maps" / "map.lcm"),
                    provenance_sha256=hash_file_sha256(staged / "provenance.v1.json"),
                    staging_manifest_sha256=hash_file_sha256(staged / "staging-manifest.v1.json"),
                    schema_sha256=schema_hash,
                    release_sha256=release_hash,
                    staging_inventory=staging_inventory,
                )
            )
    prompts = build_prompt_pair()
    adapter_file = _adapter_artifact(spec_path.parent, config.node_adapter_command)
    runtime_files = runtime_adapter_artifacts(Path(config.node_adapter_command[1]))
    scorer_file = Path(__file__).with_name("scoring.py")
    source_root = Path(__file__).parents[4]
    node_protocol = (source_root / "packages/pi-spatial-adapter/src/protocol.ts").read_bytes()
    implementation_digests = {
        "adapter": f"adapter@sha256:{_bytes_digest(adapter_file)}",
        "scorer": f"scorer@sha256:{_bytes_digest(scorer_file.read_bytes())}",
        "protocol": f"protocol@sha256:{_bytes_digest(node_protocol)}",
    }
    from dimos.benchmark.spatial.pi_baseline.scheduler_pi_binding import _shared_tool_schemas

    tool_definitions = _shared_tool_schemas()
    prompt_forbidden = prompts.visualization_forbidden.encode()
    prompt_encouraged = prompts.visualization_encouraged.encode()
    scorer_bytes = scorer_file.read_bytes()
    material_files = {
        "prompt.forbidden": prompt_forbidden,
        "prompt.encouraged": prompt_encouraged,
        "tools.schemas": canonical_json(cast("JsonValue", tool_definitions)),
        "node.main": (source_root / "packages/pi-spatial-adapter/src/main.ts").read_bytes(),
        "node.session": (source_root / "packages/pi-spatial-adapter/src/session.ts").read_bytes(),
        "node.tools": (source_root / "packages/pi-spatial-adapter/src/tools.ts").read_bytes(),
        "node.protocol": (source_root / "packages/pi-spatial-adapter/src/protocol.ts").read_bytes(),
        "node.package-lock": (
            source_root / "packages/pi-spatial-adapter/package-lock.json"
        ).read_bytes(),
        "node.package": (source_root / "packages/pi-spatial-adapter/package.json").read_bytes(),
        "node.build": (source_root / "packages/pi-spatial-adapter/tsconfig.json").read_bytes(),
        "node.tool-schema": (
            source_root / "packages/pi-spatial-adapter/src/tool-definitions.v1.json"
        ).read_bytes(),
        "controller": Path(__file__).with_name("controller.py").read_bytes(),
        "runner": Path(__file__).with_name("runner.py").read_bytes(),
        "broker": Path(__file__).with_name("broker.py").read_bytes(),
        "scheduler_pi_executor": Path(__file__).with_name("scheduler_pi_executor.py").read_bytes(),
        **{name: path.read_bytes() for name, path in host_module_paths(source_root).items()},
        "scorer": scorer_bytes,
        "adapter": adapter_file,
        **runtime_files,
        "config.model": canonical_json(cast("JsonValue", config.model.model_dump(mode="json"))),
        "config.budgets": canonical_json(cast("JsonValue", config.budgets.model_dump(mode="json"))),
        "config.limits": canonical_json(
            cast("JsonValue", config.resource_limits.model_dump(mode="json"))
        ),
        "config.network": canonical_json(
            cast("JsonValue", config.audit_network_policy.model_dump(mode="json"))
        ),
        "config.image": canonical_json(cast("JsonValue", config.runner_image)),
    }
    material_inventory = tuple(
        PiMaterialRecord(
            name=name, sha256=_bytes_digest(data), bytes_b64=base64.b64encode(data).decode("ascii")
        )
        for name, data in material_files.items()
    )
    snapshot = PiExecutionSnapshot.from_material(
        policy_revision="pre-image-2-sandbox-plus-1-repair-2-image-post-image-1-submit-v1",
        model=config.model,
        budgets=config.budgets,
        resource_limits=config.resource_limits,
        network_policy=config.audit_network_policy,
        runner_image=config.runner_image,
        node_adapter_command=tuple(config.node_adapter_command),
        scorer_revision=config.scorer_revision,
        implementation_digests=implementation_digests,
        prompt_fingerprint=_bytes_digest(prompt_forbidden),
        tool_fingerprint=_bytes_digest(canonical_json(cast("JsonValue", tool_definitions))),
        executor_fingerprint=_bytes_digest(adapter_file),
        pi_selected_inputs_digest=hashlib.sha256(
            canonical_json(
                cast("JsonValue", [item.model_dump(mode="json") for item in selected_inputs])
            )
        ).hexdigest(),
        prompt_text_forbidden=prompts.visualization_forbidden,
        prompt_text_encouraged=prompts.visualization_encouraged,
        tool_definitions=tool_definitions,
        material_inventory=material_inventory,
        selected_inputs=tuple(selected_inputs),
    )
    snapshot_digest = snapshot.canonical_digest()
    manifest = ExperimentManifest(
        experiment_id=experiment_id,
        plan_digest=plan.plan_digest,
        executor_fingerprint=snapshot_digest,
        executor_snapshot_digest=snapshot_digest,
        selected_inputs_digest=plan_selected_inputs_digest(plan),
        executor_kind="pi",
        model_fingerprint=_fingerprint("model", snapshot, plan),
        prompt_fingerprint=_fingerprint("prompt", snapshot, plan),
        tools_fingerprint=_fingerprint("tools", snapshot, plan),
        corpus_fingerprint=_fingerprint("corpus", snapshot, plan),
        runner_image_fingerprint=_fingerprint("runner-image", snapshot, plan),
        scorer_fingerprint=_fingerprint("scorer", snapshot, plan),
        limits_fingerprint=_fingerprint("limits", snapshot, plan),
        worker_fingerprint=_fingerprint("worker", snapshot, plan),
        workers=plan.workers,
    )
    return manifest, plan, snapshot, config


def create_experiment(
    experiment_dir: Path,
    spec_path: Path,
    *,
    workers: int | None,
    sample: int | None,
    shard: int,
    shards: int,
) -> tuple[ExperimentManifest, Path]:
    manifest, plan, snapshot, _ = create_definition(
        experiment_dir,
        spec_path,
        workers=workers,
        sample=sample,
        shard=shard,
        shards=shards,
    )
    if experiment_dir.name != manifest.experiment_id:
        raise ValueError("experiment directory name must match experiment_id")
    store = FilesystemExperimentStore(experiment_dir)
    store.create(
        manifest,
        plan,
        additional_files={
            "executor.pi.v1.json": cast("JsonValue", snapshot.model_dump(mode="json"))
        },
    )
    return manifest, experiment_dir


def load_definition(
    experiment_dir: Path,
) -> tuple[ExperimentManifest, ExperimentPlan, PiExecutionSnapshot, FilesystemExperimentStore]:
    manifest = ExperimentManifest.model_validate_json(
        (experiment_dir / "manifest.json").read_text(encoding="utf-8")
    )
    plan = ExperimentPlan.model_validate_json(
        (experiment_dir / "plan.json").read_text(encoding="utf-8")
    )
    snapshot = PiExecutionSnapshot.model_validate_json(
        (experiment_dir / "executor.pi.v1.json").read_text(encoding="utf-8")
    )
    verify_snapshot_artifacts(snapshot)
    if snapshot.canonical_digest() != manifest.executor_snapshot_digest:
        raise ValueError("executor snapshot does not match manifest")
    if manifest.executor_fingerprint != manifest.executor_snapshot_digest:
        raise ValueError("Pi executor fingerprint does not match snapshot digest")
    if manifest.selected_inputs_digest != plan_selected_inputs_digest(plan):
        raise ValueError("selected input digest does not match the scheduler plan")
    if tuple(item.case_id for item in snapshot.selected_inputs) != tuple(
        case.case_id for case in plan.cases
    ):
        raise ValueError("Pi selected inputs are not a complete plan-case bijection")
    if any(
        item.selection != PiCasePayload.model_validate(case.payload).selection
        for item, case in zip(snapshot.selected_inputs, plan.cases, strict=True)
    ):
        raise ValueError("Pi selected input selection does not match the expanded plan")
    expected_fingerprints = {
        field: _fingerprint(label, snapshot, plan)
        for field, label in (
            ("model_fingerprint", "model"),
            ("prompt_fingerprint", "prompt"),
            ("tools_fingerprint", "tools"),
            ("corpus_fingerprint", "corpus"),
            ("runner_image_fingerprint", "runner-image"),
            ("scorer_fingerprint", "scorer"),
            ("limits_fingerprint", "limits"),
            ("worker_fingerprint", "worker"),
        )
    }
    if any(getattr(manifest, field) != value for field, value in expected_fingerprints.items()):
        raise ValueError("manifest fingerprints do not match the immutable Pi definition")
    return manifest, plan, snapshot, FilesystemExperimentStore(experiment_dir)


def runtime_bindings(
    *,
    private_root: Path,
    corpus_root: Path,
    oracle_root: Path,
    auth_file: Path,
    auth_mode: AuthMode = "codex-oauth",
    ledger_path: Path,
    public_root: Path,
) -> PiRuntimeBindings:
    if not auth_file.is_file():
        raise ValueError("auth file must be an existing file")
    if not corpus_root.is_dir() or not oracle_root.is_dir():
        raise ValueError("corpus and oracle roots must be existing directories")
    return PiRuntimeBindings(
        auth_file=auth_file,
        auth_mode=auth_mode,
        corpus_root=corpus_root,
        oracle_root=oracle_root,
        private_root=private_root,
        ledger_path=ledger_path,
        public_root=public_root,
    )


def retain_private_diagnostic(private_root: Path, operation: str, error: BaseException) -> None:
    """Best-effort raw diagnostics in an explicitly supplied private directory."""
    directory_flags = os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW | os.O_CLOEXEC
    directory_fd: int | None = None
    try:
        path = private_root.expanduser()
        directory_fd = os.open("/" if path.is_absolute() else ".", directory_flags)
        components = path.parts[1:] if path.is_absolute() else path.parts
        for component in components:
            if component in {"", "."}:
                continue
            try:
                child_fd = os.open(component, directory_flags, dir_fd=directory_fd)
            except FileNotFoundError:
                os.mkdir(component, 0o700, dir_fd=directory_fd)
                child_fd = os.open(component, directory_flags, dir_fd=directory_fd)
            os.close(directory_fd)
            directory_fd = child_fd

        name = f"diagnostic-{uuid4().hex}.json"
        fd = os.open(
            name,
            os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW | os.O_CLOEXEC,
            0o600,
            dir_fd=directory_fd,
        )
        try:
            payload = {
                "record_type": "pi-private-diagnostic",
                "schema_version": "1.0",
                "operation": operation,
                "exception_type": type(error).__name__,
                "exception": repr(error),
                "traceback": "".join(traceback.format_exception(error)),
            }
            encoded = json.dumps(payload, sort_keys=True).encode("utf-8")
            offset = 0
            while offset < len(encoded):
                written = os.write(fd, encoded[offset:])
                if written <= 0:
                    raise OSError("diagnostic write made no progress")
                offset += written
            os.fsync(fd)
        finally:
            os.close(fd)
        os.fsync(directory_fd)
    except Exception:
        # Diagnostics must never affect the public result or cleanup precedence.
        return
    finally:
        if directory_fd is not None:
            try:
                os.close(directory_fd)
            except OSError:
                pass


def validate_runtime_bindings(
    experiment_dir: Path,
    bindings: PiRuntimeBindings,
    snapshot: PiExecutionSnapshot,
    manifest: ExperimentManifest,
) -> None:
    """Validate ephemeral runtime inputs against the immutable definition."""
    result = validate_pi_definition(experiment_dir, bindings)
    if (
        result.manifest.model_dump() != manifest.model_dump()
        or result.snapshot.canonical_digest() != snapshot.canonical_digest()
    ):
        raise ValueError("runtime definition does not match loaded experiment")


def validate_pi_definition(
    experiment_dir: Path,
    bindings: PiRuntimeBindings | None = None,
) -> PiPreflightResult:
    """Authoritative Phase 1C validator for an immutable Pi definition.

    This is deliberately the only CLI-facing definition validator: it loads
    through the Phase 1A store, validates the typed snapshot and selected
    inputs, and verifies that the selected records are exactly the expanded
    plan cases.
    """
    manifest, plan, snapshot, store = load_definition(experiment_dir)
    canonical_manifest_digest = store.load_definition().manifest_digest
    verify_snapshot_artifacts(snapshot)
    plan_ids = tuple(case.case_id for case in plan.cases)
    selected_ids = tuple(item.case_id for item in snapshot.selected_inputs)
    if plan_ids != selected_ids:
        raise ValueError("Pi selected inputs are not exactly the expanded plan cases")
    if manifest.plan_digest != plan.plan_digest:
        raise ValueError("Pi manifest plan digest does not match the stored plan")
    if manifest.selected_inputs_digest != plan_selected_inputs_digest(plan):
        raise ValueError("Pi manifest selected input digest does not match the stored plan")
    admission_context = PiAdmissionContext.from_snapshot(snapshot)
    if bindings is not None:
        verify_runtime_artifacts(snapshot, snapshot.node_adapter_command)
        verify_public_inputs(snapshot, bindings.corpus_root)
        _validate_private_inputs(snapshot, bindings.corpus_root, bindings.oracle_root)
        expected_private_binding = private_binding_digest(
            canonical_manifest_digest, snapshot, bindings.oracle_root
        )
    else:
        expected_private_binding = ""
    return PiPreflightResult(
        manifest=manifest,
        plan=plan,
        snapshot=snapshot,
        store=store,
        manifest_digest=canonical_manifest_digest,
        private_binding_digest=expected_private_binding,
        admission_context=admission_context,
    )


def validate_pi_definition_without_binding(
    experiment_dir: Path, bindings: PiRuntimeBindings
) -> PiPreflightResult:
    """Validate a definition and runtime material without mutating private state."""
    return validate_pi_definition(experiment_dir, bindings)


def _validate_private_inputs(
    snapshot: PiExecutionSnapshot, corpus_root: Path, oracle_root: Path
) -> None:
    """Require exactly one typed answer and at most one compatible override per case."""
    from dimos.benchmark.spatial.pi_baseline.projection import stage_public_instance
    from dimos.benchmark.spatial.pi_baseline.scoring import validate_private_case

    with tempfile.TemporaryDirectory(prefix="pi-private-verify-") as directory:
        parent = Path(directory)
        for selected in snapshot.selected_inputs:
            staged = stage_public_instance(corpus_root, parent, **selected.selection.model_dump())
            case = cast("dict[str, JsonValue]", _load_json(staged / "cases" / "case.v1.json"))
            validate_private_case(case, oracle_root)


def _bytes_digest(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def _adapter_artifact(base: Path, command: list[str]) -> bytes:
    del base
    if len(command) != 2:
        raise ValueError("Pi authoring spec must contain exactly one adapter file argument")
    candidate = Path(command[1])
    if not candidate.is_absolute() or not candidate.is_file() or candidate.is_symlink():
        raise ValueError("Pi authoring spec does not identify its exact adapter artifact")
    return candidate.read_bytes()


def bind_private_tree(
    private_root: Path,
    experiment_id: str,
    manifest_digest: str,
    private_binding_digest: str,
    *,
    store: FilesystemExperimentStore,
    capability: CoordinatorLeaseCapability,
) -> None:
    """Commit or verify one already-frozen canonical private binding digest."""
    with store.lease_mutation(capability):
        definition = store.load_definition()
        if definition.manifest.experiment_id != experiment_id:
            raise ValueError("private binding experiment does not match the leased store")
        if definition.manifest_digest != manifest_digest:
            raise ValueError("private binding manifest digest does not match the leased store")
        manifest_path = private_root / experiment_id / "manifest.digest"
        if manifest_path.exists():
            stored = json.loads(manifest_path.read_text(encoding="utf-8"))
            if (
                not isinstance(stored, dict)
                or stored.get("manifest_digest") != manifest_digest
                or stored.get("private_binding_digest") != private_binding_digest
            ):
                raise ValueError("private runtime tree is bound to a different manifest")
            return
        _write_immutable(
            manifest_path,
            cast(
                "JsonValue",
                {
                    "manifest_digest": manifest_digest,
                    "private_binding_digest": private_binding_digest,
                },
            ),
        )


def execute_pi_operation(
    runtime: SchedulerRuntime,
    bindings: PiRuntimeBindings,
    operation: Literal["run", "resume", "retry"],
    *,
    host_prerequisite: Callable[[], bool | None],
    job_id_value: str | None = None,
    reason: str | None = None,
) -> tuple[JobSummary, ...]:
    """Sole typed Pi execution entry point; admission cannot bypass preflight."""
    from dimos.benchmark.spatial.pi_baseline.scheduler_pi_executor import PiSchedulerExecutor

    with runtime.store.coordinator_lease() as capability:
        result = validate_pi_definition(runtime.store.root, bindings)
        runtime.manifest, runtime.plan = result.manifest, result.plan
        runtime.executor = PiSchedulerExecutor(
            result.snapshot,
            bindings,
            manifest_executor_fingerprint=result.manifest.executor_fingerprint,
            admission_context=result.admission_context,
        )
        try:
            available = host_prerequisite()
        except ExecutionInterrupted:
            raise
        except Exception as error:
            raise RuntimeError("Pi host prerequisite failed") from error
        if available is False:
            raise RuntimeError("Pi host prerequisite is unavailable")
        current = private_binding_digest(
            result.manifest_digest, result.snapshot, bindings.oracle_root
        )
        if current != result.private_binding_digest:
            raise ValueError("private binding changed after Pi preflight")
        bind_private_tree(
            bindings.private_root,
            result.manifest.experiment_id,
            result.manifest_digest,
            current,
            store=runtime.store,
            capability=capability,
        )
        runtime._reload_reconcile_locked()
        if operation == "retry":
            if job_id_value is None or reason is None or not reason.strip():
                raise ValueError("retry requires an explicit reason")
            summary = runtime._summaries.get(job_id_value)
            if summary is None:
                raise KeyError(job_id_value)
            if summary.outcome is None or summary.outcome.status not in {
                "failed",
                "interrupted",
                "cancelled",
            }:
                raise ValueError(
                    "only the latest failed, interrupted, or cancelled outcome may be retried"
                )
        if operation == "run":
            selected = runtime._job_ids_with_state({"pending"})
            return runtime._execute(selected, allowed_states={"pending"}, capability=capability)
        if operation == "resume":
            selected = runtime._job_ids_with_state({"pending", "interrupted"})
            return runtime._execute(
                selected, allowed_states={"pending", "interrupted"}, capability=capability
            )
        if operation == "retry":
            assert job_id_value is not None and reason is not None
            runtime._execute(
                (job_id_value,), retry_reason=reason, allowed_states=set(), capability=capability
            )
            return (runtime._summaries[job_id_value],)
        raise ValueError(f"unknown Pi operation: {operation}")


def execute_pi_precursor(
    runtime: SchedulerRuntime,
    bindings: PiRuntimeBindings,
    *,
    host_prerequisite: Callable[[], bool | None],
    job_id_value: str | None = None,
) -> tuple[JobSummary, ...]:
    """Execute exactly one eligible visualization-forbidden precursor job."""
    from dimos.benchmark.spatial.pi_baseline.scheduler_pi_executor import PiSchedulerExecutor

    with runtime.store.coordinator_lease() as capability:
        result = validate_pi_definition(runtime.store.root, bindings)
        runtime.manifest, runtime.plan = result.manifest, result.plan
        runtime.executor = PiSchedulerExecutor(
            result.snapshot,
            bindings,
            manifest_executor_fingerprint=result.manifest.executor_fingerprint,
            admission_context=result.admission_context,
        )
        try:
            available = host_prerequisite()
        except ExecutionInterrupted:
            raise
        except Exception as error:
            raise RuntimeError("Pi host prerequisite failed") from error
        if available is False:
            raise RuntimeError("Pi host prerequisite is unavailable")
        current = private_binding_digest(
            result.manifest_digest, result.snapshot, bindings.oracle_root
        )
        if current != result.private_binding_digest:
            raise ValueError("private binding changed after Pi preflight")
        bind_private_tree(
            bindings.private_root,
            result.manifest.experiment_id,
            result.manifest_digest,
            current,
            store=runtime.store,
            capability=capability,
        )
        runtime._reload_reconcile_locked()
        candidate_job_ids: tuple[str, ...]
        if job_id_value is not None:
            summary = runtime._summaries.get(job_id_value)
            if summary is None or summary.state != "pending":
                raise ValueError("precursor job is not eligible")
            candidate_job_ids = (job_id_value,)
        else:
            candidate_job_ids = tuple(
                summary.identity.job_id
                for summary in runtime._summaries.values()
                if summary.state == "pending"
                and any(
                    item.case_id == summary.identity.case_id
                    and item.condition_name == summary.identity.condition_name
                    for item in runtime.plan.jobs
                )
                and any(
                    item.name == summary.identity.condition_name
                    and PiConditionPayload.model_validate(item.payload).prompt_mode
                    == "visualization-forbidden"
                    for item in runtime.plan.conditions
                )
            )
            if len(candidate_job_ids) != 1:
                raise ValueError("precursor requires exactly one eligible pending job")

        selected_job = candidate_job_ids[0]
        summary = runtime._summaries[selected_job]
        plan_job = next(
            (item for item in runtime.plan.jobs if item.case_id == summary.identity.case_id),
            None,
        )
        if plan_job is None or plan_job.condition_name != summary.identity.condition_name:
            raise ValueError("precursor job is not in the immutable plan")
        condition = next(
            (item for item in runtime.plan.conditions if item.name == plan_job.condition_name),
            None,
        )
        if (
            condition is None
            or PiConditionPayload.model_validate(condition.payload).prompt_mode
            != "visualization-forbidden"
        ):
            raise ValueError("precursor requires visualization-forbidden")
        return runtime._execute((selected_job,), allowed_states={"pending"}, capability=capability)


def write_review(
    private_root: Path,
    experiment_dir: Path,
    manifest: ExperimentManifest,
    reviewer: str,
    decision: str,
) -> tuple[Path, str]:
    record = ReviewDecision(
        experiment_id=manifest.experiment_id,
        manifest_digest=_canonical_file_digest(experiment_dir / "manifest.json"),
        reviewer=reviewer,
        decision=decision,
        decided_at=datetime.now(UTC),
    )
    path = private_root / manifest.experiment_id / "review-decision.v1.json"
    _write_immutable(path, cast("JsonValue", record.model_dump(mode="json")))
    return path, hash_file_sha256(path)


def write_report(
    experiment_dir: Path,
    private_root: Path,
    review_path: Path,
    summaries: tuple[JobSummary, ...],
    *,
    public_root: Path | None = None,
) -> tuple[Path, str]:
    manifest, plan, snapshot, store = load_definition(experiment_dir)
    review = ReviewDecision.model_validate_json(review_path.read_text(encoding="utf-8"))
    manifest_digest = _canonical_file_digest(experiment_dir / "manifest.json")
    if review.experiment_id != manifest.experiment_id or review.manifest_digest != manifest_digest:
        raise ValueError("review decision is not bound to this manifest")
    if review.decision != "approved":
        raise PermissionError("report requires an approved review decision")
    if len(summaries) != len(plan.jobs) or any(
        summary.state not in {"succeeded", "failed", "interrupted", "cancelled"}
        for summary in summaries
    ):
        raise ValueError("report requires all jobs to be terminal")
    score_records = _committed_report_records(
        manifest,
        plan,
        snapshot,
        store,
        summaries,
        private_root,
        public_root or private_root,
    )
    report = ReportRecord(
        experiment_id=manifest.experiment_id,
        manifest_digest=manifest_digest,
        review_decision_digest=hash_file_sha256(review_path),
        scores=tuple(score_records),
    )
    path = private_root / manifest.experiment_id / "report.v1.json"
    _write_immutable(path, cast("JsonValue", report.model_dump(mode="json")))
    return path, hash_file_sha256(path)


def _committed_report_records(
    manifest: ExperimentManifest,
    plan: ExperimentPlan,
    snapshot: PiExecutionSnapshot,
    store: FilesystemExperimentStore,
    summaries: tuple[JobSummary, ...],
    private_root: Path,
    public_root: Path,
) -> list[dict[str, object]]:
    """Return only results admitted by the immutable evidence commit marker."""
    records: list[dict[str, object]] = []
    for summary in sorted(summaries, key=lambda item: item.identity.job_id):
        if summary.state != "succeeded" or summary.latest_attempt_id is None:
            continue
        attempts = store.observe_attempts_read_only(manifest.experiment_id, summary.identity.job_id)
        attempt = next(
            (
                item
                for item in attempts
                if item.context.attempt_id == summary.latest_attempt_id
                and item.outcome is not None
                and item.outcome.status == "succeeded"
            ),
            None,
        )
        if attempt is None:
            continue
        try:
            # The plan and the scheduler's job identity are authoritative.  In
            # particular, do not let an attempt descriptor choose a different
            # job, condition, or experiment (or consequently a different
            # production evidence namespace).
            if (
                attempt.context.identity != summary.identity
                or attempt.context.identity.experiment_id != manifest.experiment_id
                or attempt.case.case_id != summary.identity.case_id
                or getattr(attempt.condition, "name", summary.identity.condition_name)
                != summary.identity.condition_name
            ):
                continue
            plan_job = next(
                item
                for item in plan.jobs
                if item.case_id == summary.identity.case_id
                and item.condition_name == summary.identity.condition_name
            )
            plan_condition = next(
                item for item in plan.conditions if item.name == plan_job.condition_name
            )
            condition = PiConditionPayload.model_validate(plan_condition.payload)
            # Keep the mode bound to the condition used by the scheduler, not
            # to an arbitrary attempt payload.
            if attempt.condition != plan_condition:
                continue
            selected = next(
                item for item in snapshot.selected_inputs if item.case_id == attempt.case.case_id
            )
            if selected.case_id != summary.identity.case_id:
                continue
            condition_run_id = _condition_run_id(attempt.context)
            namespace = (
                manifest.experiment_id,
                summary.identity.job_id,
                attempt.context.attempt_id,
            )
            public = public_root.joinpath(
                *namespace, "public", condition_run_id, condition.prompt_mode, "evidence"
            )
            private = private_root.joinpath(
                *namespace, "private", condition_run_id, condition.prompt_mode
            )
            if _report_namespace_has_symlink(public_root, public) or _report_namespace_has_symlink(
                private_root, private
            ):
                continue
            for field in ("public_root", "private_root", "public_path", "private_path"):
                authoritative = getattr(attempt.outcome, field, None)
                if authoritative is not None and Path(authoritative) != (
                    public if field.startswith("public") else private
                ):
                    raise ValueError("attempt outcome evidence root is not authoritative")
            provenance = public / "provenance.v1.json"
            if (
                provenance.is_symlink()
                or hash_file_sha256(provenance) != selected.provenance_sha256
            ):
                continue
            provenance_value = _load_json(provenance)
            release_id = provenance_value.get("release_id")
            if not isinstance(release_id, str) or not release_id:
                continue
            expected = EvidenceIdentityContext(
                run_id=condition_run_id,
                case_id=attempt.case.case_id,
                mode=condition.prompt_mode,
                release_id=release_id,
                scorer_revision=snapshot.scorer_revision,
            )
            committed = load_committed_result(
                public,
                private,
                expected_identity=expected,
            )
            if committed is None:
                continue
            marker = private / "evidence-manifest.v1.json"
            records.append(
                {
                    "job_id": summary.identity.job_id,
                    "attempt_id": attempt.context.attempt_id,
                    "committed_manifest_sha256": hash_file_sha256(marker),
                    "prediction": committed.prediction.model_dump(mode="json"),
                    "score": committed.score.model_dump(mode="json"),
                }
            )
        except (OSError, StopIteration, TypeError, ValueError):
            continue
    return records


def _report_namespace_has_symlink(root: Path, path: Path) -> bool:
    """Reject redirection inside a derived evidence namespace."""
    try:
        relative = path.relative_to(root)
    except ValueError:
        return True
    current = root
    for component in relative.parts:
        current /= component
        if current.is_symlink():
            return True
    return False


def _write_immutable(path: Path, value: JsonValue) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists():
        raise FileExistsError(path)
    temporary = path.with_name(f".{path.name}.{uuid4().hex}.tmp")
    with temporary.open("xb") as stream:
        stream.write(canonical_json(value) + b"\n")
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)
    descriptor = os.open(path.parent, os.O_RDONLY)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)
