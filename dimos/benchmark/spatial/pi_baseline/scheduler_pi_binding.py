"""Immutable Pi execution snapshots and runtime-only bindings."""

from __future__ import annotations

import base64
from collections.abc import Mapping
from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
import tempfile
from types import MappingProxyType
from typing import Annotated, Literal, cast

from pydantic import Field, model_validator

from dimos.benchmark.spatial.models import AnswerType, SpatialModel
from dimos.benchmark.spatial.pi_baseline.config import (
    AuditNetworkPolicy,
    Budgets,
    ImplementationDigests,
    ModelConfig,
    PiBaselineConfig,
    PromptMode,
    PublicSelection,
    ResourceLimits,
    validate_node_adapter_command,
)
from dimos.benchmark.spatial.utilities import JsonValue, canonical_json, hash_file_sha256

_RUNTIME_MANIFEST = "runtime-manifest.v1.json"
_RUNTIME_FILES = (
    "main.js",
    "protocol.js",
    "session.js",
    "tools.js",
    "tool-definitions.v1.json",
)
_HOST_MODULES = (
    "config",
    "evidence",
    "gate",
    "podman",
    "projection",
    "prompts",
    "records",
    "scheduler_executor",
    "scheduler_models",
    "topology",
    "transaction",
)

Digest = Annotated[str, Field(pattern=r"^[0-9a-f]{64}$")]
SafeRelativePath = Annotated[
    str,
    Field(pattern=r"^[A-Za-z0-9._/-]+$"),
]


class PiMaterialRecord(SpatialModel):
    """One exact, public, non-secret byte artifact in a Pi snapshot."""

    name: str = Field(min_length=1)
    sha256: Digest
    bytes_b64: str = Field(min_length=1)


class PiInventoryRecord(SpatialModel):
    """One byte in the selected public staging inventory."""

    path: SafeRelativePath
    sha256: Digest


class PiSelectedInput(SpatialModel):
    """A complete typed public selection and its immutable staged inventory."""

    case_id: str = Field(min_length=1)
    selection: PublicSelection
    answer_type: AnswerType
    case_sha256: Digest
    map_sha256: Digest
    provenance_sha256: Digest
    staging_manifest_sha256: Digest
    schema_sha256: Digest
    release_sha256: Digest
    staging_inventory: tuple[PiInventoryRecord, ...] = Field(min_length=5)

    @model_validator(mode="after")
    def validate_inventory(self) -> PiSelectedInput:
        paths = tuple(item.path for item in self.staging_inventory)
        if any(
            path.startswith("/")
            or "//" in path
            or any(part in {"", ".", ".."} for part in path.split("/"))
            for path in paths
        ):
            raise ValueError("Pi staging inventory contains an unsafe path")
        if len(paths) != len(set(paths)):
            raise ValueError("Pi staging inventory contains duplicate paths")
        return self


@dataclass(frozen=True)
class PiAdmissionContext:
    """Immutable, operation-scoped index produced by authoritative preflight."""

    selected_by_case: Mapping[str, PiSelectedInput]
    snapshot_digest: str

    @classmethod
    def from_snapshot(cls, snapshot: PiExecutionSnapshot) -> PiAdmissionContext:
        return cls(
            selected_by_case=MappingProxyType(
                {item.case_id: item for item in snapshot.selected_inputs}
            ),
            snapshot_digest=snapshot.canonical_digest(),
        )

    def selected(self, case_id: str) -> PiSelectedInput:
        try:
            return self.selected_by_case[case_id]
        except KeyError as error:
            raise ValueError("Pi case is not one of the immutable selected inputs") from error


class PiExecutionSnapshot(SpatialModel):
    """Non-secret, immutable execution inputs captured at experiment creation."""

    snapshot_version: Literal["1.1"] = "1.1"
    policy_revision: Literal["pre-image-2-sandbox-plus-1-repair-2-image-post-image-1-submit-v1"] = (
        "pre-image-2-sandbox-plus-1-repair-2-image-post-image-1-submit-v1"
    )
    model: ModelConfig
    budgets: Budgets
    resource_limits: ResourceLimits
    network_policy: AuditNetworkPolicy
    runner_image: str = Field(pattern=r"^.+@sha256:[0-9a-f]{64}$")
    node_adapter_command: tuple[str, ...] = Field(min_length=2, max_length=2)

    @model_validator(mode="after")
    def validate_adapter_command(self) -> PiExecutionSnapshot:
        validate_node_adapter_command(self.node_adapter_command)
        return self

    scorer_revision: str = Field(min_length=1)
    implementation_digests: ImplementationDigests
    prompt_fingerprint: Digest = Field(pattern=r"^[0-9a-f]{64}$")
    tool_fingerprint: Digest = Field(pattern=r"^[0-9a-f]{64}$")
    executor_fingerprint: Digest = Field(pattern=r"^[0-9a-f]{64}$")
    pi_selected_inputs_digest: Digest
    prompt_text_forbidden: str = Field(min_length=1)
    prompt_text_encouraged: str = Field(min_length=1)
    tool_definitions: dict[str, object]
    material_inventory: tuple[PiMaterialRecord, ...] = Field(min_length=1)
    selected_inputs: tuple[PiSelectedInput, ...] = Field(min_length=1)

    @classmethod
    def from_config(
        cls,
        config: PiBaselineConfig,
        *,
        prompt_fingerprint: str,
        tool_fingerprint: str,
        executor_fingerprint: str,
    ) -> PiExecutionSnapshot:
        raise ValueError("construct PiExecutionSnapshot through from_material")

    @classmethod
    def from_material(cls, **values: object) -> PiExecutionSnapshot:
        """Construct and validate a snapshot from actual captured bytes."""
        snapshot = cls.model_validate(values)
        verify_snapshot_artifacts(snapshot)
        return snapshot

    def canonical_digest(self) -> str:
        payload = cast("JsonValue", self.model_dump(mode="json"))
        return hashlib.sha256(canonical_json(payload)).hexdigest()


def artifact_bytes(snapshot_value: str) -> bytes:
    """Decode bytes persisted in the non-secret snapshot."""
    try:
        return base64.b64decode(snapshot_value.encode("ascii"), validate=True)
    except Exception as error:
        raise ValueError("invalid Pi artifact encoding") from error


def runtime_adapter_artifacts(adapter_file: Path) -> dict[str, bytes]:
    """Load the exact immutable JS/JSON files executed by the adapter."""
    runtime_root = adapter_file.parent
    manifest_path = runtime_root / _RUNTIME_MANIFEST
    try:
        manifest = json.loads(manifest_path.read_bytes())
    except Exception as error:
        raise ValueError("Pi adapter runtime manifest is missing or invalid") from error
    if (
        not isinstance(manifest, dict)
        or manifest.get("record_type") != "pi-adapter-runtime-manifest"
        or manifest.get("schema_version") != "1.1"
        or manifest.get("files") != list(_RUNTIME_FILES)
    ):
        raise ValueError("Pi adapter runtime manifest is invalid")
    actual = sorted(
        path.name
        for path in runtime_root.iterdir()
        if path.is_file() and not path.is_symlink() and path.suffix in {".js", ".json"}
    )
    expected = sorted((*_RUNTIME_FILES, _RUNTIME_MANIFEST))
    if actual != expected:
        raise ValueError("Pi adapter runtime files do not match manifest")
    return {
        f"node.runtime.{name}": (runtime_root / name).read_bytes()
        for name in (*_RUNTIME_FILES, _RUNTIME_MANIFEST)
    }


def host_module_paths(source_root: Path) -> dict[str, Path]:
    """Return the bounded Python host-module closure executed by the Pi runner."""
    root = source_root / "dimos/benchmark/spatial/pi_baseline"
    return {name: root / f"{name}.py" for name in _HOST_MODULES}


def verify_snapshot_artifacts(snapshot: PiExecutionSnapshot) -> None:
    """Reject snapshots whose stored bytes no longer match their declared digests."""
    required = {
        "prompt.forbidden",
        "prompt.encouraged",
        "tools.schemas",
        "node.main",
        "node.session",
        "node.tools",
        "node.protocol",
        "node.package-lock",
        "node.package",
        "node.build",
        "node.tool-schema",
        "controller",
        "runner",
        "broker",
        "scheduler_pi_executor",
        *_HOST_MODULES,
        "scorer",
        "adapter",
        "node.runtime.main.js",
        "node.runtime.protocol.js",
        "node.runtime.session.js",
        "node.runtime.tools.js",
        "node.runtime.tool-definitions.v1.json",
        "node.runtime.runtime-manifest.v1.json",
        "config.model",
        "config.budgets",
        "config.limits",
        "config.network",
        "config.image",
    }
    records = {item.name: item for item in snapshot.material_inventory}
    if len(records) != len(snapshot.material_inventory) or set(records) != required:
        raise ValueError("Pi material inventory is incomplete")
    for name, record in records.items():
        data = artifact_bytes(record.bytes_b64)
        if not data or hashlib.sha256(data).hexdigest() != record.sha256:
            raise ValueError(f"Pi material bytes do not match digest: {name}")
    if (
        artifact_bytes(records["prompt.forbidden"].bytes_b64)
        != snapshot.prompt_text_forbidden.encode()
        or artifact_bytes(records["prompt.encouraged"].bytes_b64)
        != snapshot.prompt_text_encouraged.encode()
    ):
        raise ValueError("Pi prompt text does not match snapshotted bytes")
    if artifact_bytes(records["tools.schemas"].bytes_b64) != canonical_json(
        cast("JsonValue", snapshot.tool_definitions)
    ):
        raise ValueError("Pi tool definitions do not match snapshotted schema")
    if (
        hashlib.sha256(artifact_bytes(records["prompt.forbidden"].bytes_b64)).hexdigest()
        != snapshot.prompt_fingerprint
    ):
        raise ValueError("Pi prompt bytes do not match prompt fingerprint")
    if (
        hashlib.sha256(artifact_bytes(records["tools.schemas"].bytes_b64)).hexdigest()
        != snapshot.tool_fingerprint
    ):
        raise ValueError("Pi tool schema bytes do not match tool fingerprint")
    for digest_name, material_name in (
        ("adapter", "adapter"),
        ("scorer", "scorer"),
        ("protocol", "node.protocol"),
    ):
        declared = getattr(snapshot.implementation_digests, digest_name).split("@sha256:")[-1]
        if declared != records[material_name].sha256:
            raise ValueError(f"Pi {digest_name} bytes do not match implementation digest")
    if snapshot.executor_fingerprint != records["adapter"].sha256:
        raise ValueError("Pi executor fingerprint does not match adapter material")
    expected_selection_digest = selected_inputs_digest(snapshot)
    if snapshot.pi_selected_inputs_digest != expected_selection_digest:
        raise ValueError("Pi selected input digest does not match snapshot inputs")
    expected_config = {
        "config.model": snapshot.model.model_dump(mode="json"),
        "config.budgets": snapshot.budgets.model_dump(mode="json"),
        "config.limits": snapshot.resource_limits.model_dump(mode="json"),
        "config.network": snapshot.network_policy.model_dump(mode="json"),
        "config.image": snapshot.runner_image,
    }
    for name, value in expected_config.items():
        if artifact_bytes(records[name].bytes_b64) != canonical_json(cast("JsonValue", value)):
            raise ValueError(f"Pi {name} bytes do not match snapshot configuration")


def verify_runtime_artifacts(
    snapshot: PiExecutionSnapshot, command: tuple[str, ...], *, base_dir: Path | None = None
) -> None:
    """Require the runtime adapter file and scorer source to equal the snapshot bytes."""
    verify_snapshot_artifacts(snapshot)
    root = base_dir or Path.cwd()
    records = {item.name: item for item in snapshot.material_inventory}
    del root
    validate_node_adapter_command(command)
    candidate = Path(command[1])
    if candidate.read_bytes() != artifact_bytes(records["adapter"].bytes_b64):
        raise ValueError("Pi adapter artifact drift detected")
    runtime_files = runtime_adapter_artifacts(candidate)
    for name, data in runtime_files.items():
        if data != artifact_bytes(records[name].bytes_b64):
            raise ValueError(f"Pi {name} artifact drift detected")
    source_root = Path(__file__).parents[4]
    for name, path in {
        "scorer": Path(__file__).with_name("scoring.py"),
        "controller": Path(__file__).with_name("controller.py"),
        "runner": Path(__file__).with_name("runner.py"),
        "node.main": source_root / "packages/pi-spatial-adapter/src/main.ts",
        "node.session": source_root / "packages/pi-spatial-adapter/src/session.ts",
        "node.tools": source_root / "packages/pi-spatial-adapter/src/tools.ts",
        "node.protocol": source_root / "packages/pi-spatial-adapter/src/protocol.ts",
        "node.package-lock": source_root / "packages/pi-spatial-adapter/package-lock.json",
        "node.package": source_root / "packages/pi-spatial-adapter/package.json",
        "node.build": source_root / "packages/pi-spatial-adapter/tsconfig.json",
        "node.tool-schema": source_root
        / "packages/pi-spatial-adapter/src/tool-definitions.v1.json",
        **host_module_paths(source_root),
    }.items():
        if path.read_bytes() != artifact_bytes(records[name].bytes_b64):
            raise ValueError(f"Pi {name} artifact drift detected")
    from dimos.benchmark.spatial.pi_baseline.prompts import build_prompt_pair

    prompts = build_prompt_pair()
    expected_prompt = prompts.visualization_forbidden.encode()
    if expected_prompt != artifact_bytes(
        records["prompt.forbidden"].bytes_b64
    ) or prompts.visualization_encouraged.encode() != artifact_bytes(
        records["prompt.encouraged"].bytes_b64
    ):
        raise ValueError("Pi prompt material drift detected")
    expected_tools = canonical_json(cast("JsonValue", _shared_tool_schemas()))
    if expected_tools != artifact_bytes(records["tools.schemas"].bytes_b64):
        raise ValueError("Pi tool schema drift detected")
    for selected in snapshot.selected_inputs:
        schemas = _tool_schemas(cast("Literal['boolean', 'integer']", selected.answer_type.value))
        if schemas[-1]["parameters"] != _submit_variant(
            snapshot.tool_definitions, selected.answer_type
        ):
            raise ValueError("Pi selected input answer type does not match tool schema")


def selected_inputs_digest(snapshot: PiExecutionSnapshot) -> str:
    payload = cast("JsonValue", [item.model_dump(mode="json") for item in snapshot.selected_inputs])
    return hashlib.sha256(canonical_json(payload)).hexdigest()


def _shared_tool_schemas() -> dict[str, JsonValue]:
    """Return the complete shared schema artifact, including both variants."""
    return cast(
        "dict[str, JsonValue]",
        json.loads(
            (
                Path(__file__).parents[4]
                / "packages/pi-spatial-adapter/src/tool-definitions.v1.json"
            ).read_bytes()
        ),
    )


def _tool_schemas(answer_type: Literal["boolean", "integer"]) -> list[dict[str, JsonValue]]:
    """Return the concrete schemas from the checked-in shared artifact."""
    artifact = _shared_tool_schemas()
    tools = cast("list[dict[str, JsonValue]]", artifact["tools"])
    submit = cast("dict[str, JsonValue]", artifact["submit_answer"])
    variants = cast("dict[str, JsonValue]", submit["variants"])
    tools.append(
        {
            "name": submit["name"],
            "label": submit["label"],
            "description": submit["description"],
            "parameters": cast("dict[str, JsonValue]", variants[answer_type]),
        }
    )
    return tools


def _submit_variant(tool_definitions: dict[str, object], answer_type: AnswerType) -> object:
    """Extract the selected answer variant from the immutable shared schema."""
    submit_value = tool_definitions.get("submit_answer")
    submit = (
        cast("dict[str, object] | None", submit_value) if isinstance(submit_value, dict) else None
    )
    if submit is None or not isinstance(submit.get("variants"), dict):
        raise ValueError("Pi shared tool schema has no submit-answer variants")
    variants = cast("dict[str, object]", submit["variants"])
    return variants.get(answer_type.value)


def verify_public_inputs(
    snapshot: PiExecutionSnapshot,
    corpus_root: Path,
    *,
    case_id: str | None = None,
    selected_input: PiSelectedInput | None = None,
) -> None:
    """Materialize recorded cases and require the authoring corpus to be unchanged."""
    from dimos.benchmark.spatial.pi_baseline.integrity import verify_staging
    from dimos.benchmark.spatial.pi_baseline.projection import stage_public_instance
    from dimos.benchmark.spatial.pi_baseline.records import StagingRecord

    with tempfile.TemporaryDirectory(prefix="pi-verify-") as directory:
        parent = Path(directory)
        if selected_input is not None:
            selected = (selected_input,)
        elif case_id is None:
            selected = snapshot.selected_inputs
        else:
            selected = (
                next((item for item in snapshot.selected_inputs if item.case_id == case_id), None),
            )
            if selected[0] is None:
                raise ValueError("Pi case is not one of the immutable selected inputs")
        for item in selected:
            assert item is not None
            staged = stage_public_instance(corpus_root, parent, **item.selection.model_dump())
            verify_staging(
                staged,
                StagingRecord(
                    case_path="cases/case.v1.json",
                    map_path="maps/map.lcm",
                    map_sha256=item.map_sha256,
                    schema_sha256=hash_file_sha256(staged / "schema.v1.json"),
                ),
            )
            case_hash = hashlib.sha256((staged / "cases" / "case.v1.json").read_bytes()).hexdigest()
            map_hash = hashlib.sha256((staged / "maps" / "map.lcm").read_bytes()).hexdigest()
            if case_hash != item.case_sha256 or map_hash != item.map_sha256:
                raise ValueError(f"selected public input drift for {item.case_id}")
            case_payload = json.loads((staged / "cases" / "case.v1.json").read_bytes())
            answer_type = AnswerType(case_payload["question"]["answer_type"])
            if answer_type is not item.answer_type:
                raise ValueError(f"selected answer type drift for {item.case_id}")
            if hash_file_sha256(staged / "schema.v1.json") != item.schema_sha256:
                raise ValueError(f"selected schema drift for {item.case_id}")
            _submit_variant(snapshot.tool_definitions, answer_type)
            actual = tuple(
                sorted(
                    (
                        PiInventoryRecord(
                            path=path.relative_to(staged).as_posix(),
                            sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                        )
                        for path in staged.rglob("*")
                        if path.is_file()
                    ),
                    key=lambda record: record.path,
                )
            )
            if actual != item.staging_inventory:
                raise ValueError(f"selected staging inventory drift for {item.case_id}")
            if (
                hashlib.sha256((staged / "staging-manifest.v1.json").read_bytes()).hexdigest()
                != item.staging_manifest_sha256
            ):
                raise ValueError(f"selected staging manifest drift for {item.case_id}")
            if (
                hashlib.sha256((staged / "provenance.v1.json").read_bytes()).hexdigest()
                != item.provenance_sha256
            ):
                raise ValueError(f"selected provenance drift for {item.case_id}")
            schema = (
                Path(__file__).parents[4]
                / "packages/pi-spatial-adapter/src/tool-definitions.v1.json"
            ).read_bytes()
            if hashlib.sha256(schema).hexdigest() != item.schema_sha256:
                raise ValueError(f"selected staging schema drift for {item.case_id}")
            release = cast(
                "JsonValue",
                json.loads((staged / "staging-manifest.v1.json").read_bytes())["release"],
            )
            if hashlib.sha256(canonical_json(release)).hexdigest() != item.release_sha256:
                raise ValueError(f"selected release drift for {item.case_id}")


def private_binding_digest(
    manifest_digest: str, snapshot: PiExecutionSnapshot, oracle_root: Path
) -> str:
    """Digest only private bytes and immutable public identities; never persist the path."""
    files = []
    for path in sorted(path for path in oracle_root.rglob("*") if path.is_file()):
        files.append(
            (
                path.relative_to(oracle_root).as_posix(),
                hashlib.sha256(path.read_bytes()).hexdigest(),
            )
        )
    records = {item.name: item for item in snapshot.material_inventory}
    try:
        scorer_bytes = artifact_bytes(records["scorer"].bytes_b64)
    except KeyError as error:
        raise ValueError("Pi material inventory is incomplete") from error
    payload = cast(
        "JsonValue",
        {
            "manifest": manifest_digest,
            "scorer": hashlib.sha256(scorer_bytes).hexdigest(),
            "oracle": files,
        },
    )
    return hashlib.sha256(canonical_json(payload)).hexdigest()


class PiRuntimeBindings(SpatialModel):
    """Execution-time paths and credentials, deliberately absent from snapshots."""

    auth_file: Path
    corpus_root: Path
    oracle_root: Path
    private_root: Path
    ledger_path: Path
    public_root: Path


def reconstruct_config(
    snapshot: PiExecutionSnapshot,
    bindings: PiRuntimeBindings,
    *,
    selection: PublicSelection,
    mode: PromptMode,
    case_id: str,
    run_id: str,
) -> PiBaselineConfig:
    """Rebuild the validated legacy config from immutable inputs and bindings."""
    other_mode: PromptMode = (
        "visualization-encouraged"
        if mode == "visualization-forbidden"
        else "visualization-forbidden"
    )
    return PiBaselineConfig.model_validate(
        {
            "model": snapshot.model.model_dump(mode="json"),
            "node_adapter_command": list(snapshot.node_adapter_command),
            "codex_oauth_auth_path": str(bindings.auth_file),
            "runner_image": snapshot.runner_image,
            "rootless_podman_required": True,
            "resource_limits": snapshot.resource_limits.model_dump(mode="json"),
            "output_root": str(bindings.public_root),
            "audit_network_policy": snapshot.network_policy.model_dump(mode="json"),
            "prompt_modes": [mode, other_mode],
            "corpus_root": str(bindings.corpus_root),
            "oracle_root": str(bindings.oracle_root),
            "private_root": str(bindings.private_root),
            "ledger_path": str(bindings.ledger_path),
            "selection": selection.model_dump(mode="json"),
            "budgets": snapshot.budgets.model_dump(mode="json"),
            "scorer_revision": snapshot.scorer_revision,
            "fixed_smoke_identity": selection.model_dump(mode="json"),
            "implementation_digests": snapshot.implementation_digests.model_dump(mode="json"),
            "case_id": case_id,
            "run_id": run_id,
        }
    )
