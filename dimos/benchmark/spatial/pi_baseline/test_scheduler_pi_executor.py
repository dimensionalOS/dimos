import base64
import hashlib
import json
import os
from pathlib import Path
import subprocess
import tempfile
from threading import Event, Lock
from typing import cast

from pydantic import ValidationError
import pytest

from dimos.benchmark.spatial.models import (
    AnswerType,
    EligibleRoomCountContract,
    EmptyGeometry,
    FrameConventionRecord,
    Instance,
    Manifest,
    ManifestScene,
    MapperConfigurationRecord,
    MapVariant,
    Pose2D,
    Predicate,
    Question,
    Scene,
    Snapshot,
    Split,
    Trajectory,
)
from dimos.benchmark.spatial.pi_baseline.broker import PolicyViolationError
import dimos.benchmark.spatial.pi_baseline.cli_support as cli_support_module
from dimos.benchmark.spatial.pi_baseline.cli_support import (
    create_definition,
    execute_pi_operation,
    validate_pi_definition,
)
from dimos.benchmark.spatial.pi_baseline.config import PiBaselineConfig, PromptMode
from dimos.benchmark.spatial.pi_baseline.controller import AdapterCleanupError, AdapterRunError
from dimos.benchmark.spatial.pi_baseline.gate import ArtifactReference, SmokeRunEvidence
from dimos.benchmark.spatial.pi_baseline.podman import (
    ContainerCleanupError,
    PodmanInfrastructureError,
    PodmanTimeoutError,
)
import dimos.benchmark.spatial.pi_baseline.projection as projection_module
from dimos.benchmark.spatial.pi_baseline.projection import (
    DescriptorMismatchError,
    _verify_staging_descriptor,
)
from dimos.benchmark.spatial.pi_baseline.records import StagingRecord
from dimos.benchmark.spatial.pi_baseline.runner import ConditionRun, _retain_failure_record
from dimos.benchmark.spatial.pi_baseline.scheduler_models import (
    AttemptContext,
    ExecutorArtifactEvent,
    ExecutorProgressEvent,
    ExpandedCase,
    JobIdentity,
    NamedCondition,
    TerminalOutcome,
)
from dimos.benchmark.spatial.pi_baseline.scheduler_pi_binding import (
    PiExecutionSnapshot,
    PiInventoryRecord,
    PiMaterialRecord,
    PiRuntimeBindings,
    PiSelectedInput,
    _shared_tool_schemas,
    _tool_schemas,
    host_module_paths,
    private_binding_digest,
    verify_public_inputs,
    verify_runtime_artifacts,
    verify_snapshot_artifacts,
)
import dimos.benchmark.spatial.pi_baseline.scheduler_pi_executor as scheduler_pi_executor_module
from dimos.benchmark.spatial.pi_baseline.scheduler_pi_executor import (
    ConditionRunner,
    PiSchedulerExecutor,
)
from dimos.benchmark.spatial.pi_baseline.scheduler_plan import manifest_digest
from dimos.benchmark.spatial.pi_baseline.scheduler_runtime import SchedulerRuntime
from dimos.benchmark.spatial.pi_baseline.scheduler_store import FilesystemExperimentStore
from dimos.benchmark.spatial.pi_baseline.topology import PinnedDirectory
from dimos.benchmark.spatial.test_pi_baseline_config import valid_payload
from dimos.benchmark.spatial.utilities import JsonValue, canonical_json


def _operation_pair() -> tuple[Event, Lock]:
    return Event(), Lock()


_SCENE = "synthetic-scene_" + "a" * 64
_TRAJECTORY = "synthetic-trajectory_" + "b" * 64
_QUESTION = "synthetic-question_" + "c" * 64
_INSTANCE = "synthetic-instance_" + "d" * 64
_SELECTION = {
    "scene_id": _SCENE,
    "trajectory_id": _TRAJECTORY,
    "question_id": _QUESTION,
    "variant": "clean",
    "instance_id": _INSTANCE,
}


def _selection() -> dict[str, str]:
    return dict(_SELECTION)


def test_python_binding_uses_checked_in_node_tool_schema_artifact() -> None:
    artifact = json.loads(
        (
            Path(__file__).parents[4] / "packages/pi-spatial-adapter/src/tool-definitions.v1.json"
        ).read_bytes()
    )
    expected = list(artifact["tools"])
    submit = artifact["submit_answer"]
    expected.append(
        {
            "name": submit["name"],
            "label": submit["label"],
            "description": submit["description"],
            "parameters": submit["variants"]["integer"],
        }
    )
    assert _tool_schemas("integer") == expected
    assert _tool_schemas("boolean")[-1]["parameters"] == submit["variants"]["boolean"]
    assert _shared_tool_schemas()["submit_answer"]["variants"].keys() == {"boolean", "integer"}


def _write_json(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(canonical_json(cast("JsonValue", value)) + b"\n")


def _synthetic_corpus(root: Path) -> Path:
    public = root / "public"
    scene_root = public / "scenes" / _SCENE
    trajectory_root = scene_root / "trajectories" / _TRAJECTORY
    variant_root = trajectory_root / "variants" / "clean"
    pose = Pose2D(x_m=0.0, y_m=0.0, yaw_rad=0.0)
    manifest = Manifest(
        release_id="synthetic-release_" + "e" * 64,
        release_version="v1.0.0",
        generator_revision="synthetic-generator",
        mapper_configuration_digest="a" * 64,
        source_dataset_revision="synthetic-source",
        scenes=(
            ManifestScene(
                scene_id=_SCENE, split=Split.DEVELOPMENT, scene_path="public/scenes/synthetic.json"
            ),
        ),
    )
    scene = Scene(scene_id=_SCENE, split=Split.DEVELOPMENT, trajectory_ids=(_TRAJECTORY,))
    trajectory = Trajectory(
        trajectory_id=_TRAJECTORY,
        scene_id=_SCENE,
        policy_version="synthetic-policy",
        frame_id="map",
        waypoints=(pose,),
    )
    question = Question(
        question_id=_QUESTION,
        scene_id=_SCENE,
        trajectory_id=_TRAJECTORY,
        predicate=Predicate.ELIGIBLE_ROOM_COUNT,
        template_version="synthetic-v1",
        text="How many eligible rooms are present?",
        answer_type=AnswerType.INTEGER,
        contract=EligibleRoomCountContract(),
    )
    mapper = MapperConfigurationRecord(
        voxel_size_m=0.1, block_count=1, frame_id="map", emit_every=1
    )
    frame = FrameConventionRecord(frame_id="map")
    snapshot = Snapshot(
        snapshot_id="synthetic-snapshot_" + "f" * 64,
        scene_id=_SCENE,
        trajectory_id=_TRAJECTORY,
        variant=MapVariant.CLEAN,
        terminal_pose=pose,
        map_artifact_path="map.lcm",
        map_artifact_sha256="b" * 64,
        mapper_revision="synthetic-mapper",
        mapper_configuration_digest="a" * 64,
        mapper_configuration=mapper,
        noise_profile_version="none",
        seed=1,
        frame_id="map",
        frame_contract=frame,
    )
    instance = Instance(
        instance_id=_INSTANCE,
        question_id=_QUESTION,
        snapshot_id="synthetic-snapshot_" + "f" * 64,
        scene_id=_SCENE,
        trajectory_id=_TRAJECTORY,
        variant=MapVariant.CLEAN,
        query_geometry=EmptyGeometry(),
    )
    _write_json(root / "manifest.json", manifest.model_dump(mode="json"))
    _write_json(scene_root / "../synthetic.json", scene.model_dump(mode="json"))
    _write_json(trajectory_root / "trajectory.json", trajectory.model_dump(mode="json"))
    (trajectory_root / "questions.jsonl").parent.mkdir(parents=True, exist_ok=True)
    (trajectory_root / "questions.jsonl").write_bytes(question.model_dump_json().encode() + b"\n")
    _write_json(variant_root / "snapshot.json", snapshot.model_dump(mode="json"))
    (variant_root / "instances.jsonl").parent.mkdir(parents=True, exist_ok=True)
    (variant_root / "instances.jsonl").write_bytes(instance.model_dump_json().encode() + b"\n")
    map_path = variant_root / "map.lcm"
    map_path.write_bytes(b"synthetic-map-bytes")
    return root


def _synthetic_oracle(root: Path) -> Path:
    path = root / "scenes" / _SCENE / "trajectories" / _TRAJECTORY
    # The scorer's integer predicate requires an integer answer; use the model's JSON shape.
    answer_value = {"kind": "integer", "value": 0}
    _write_json(
        root / "scenes" / _SCENE / "source.json",
        {
            "record_type": "source-provenance",
            "schema_version": "1.0",
            "scene_id": _SCENE,
            "source_dataset": "synthetic",
            "source_scene_key": "synthetic",
            "source_revision": "v1",
            "source_artifact_sha256": "c" * 64,
            "coordinate_frame_description": "synthetic",
        },
    )
    path.mkdir(parents=True, exist_ok=True)
    _write_json(path / "answers.jsonl", {})
    (path / "answers.jsonl").write_bytes(
        canonical_json(
            cast(
                "JsonValue",
                {
                    "record_type": "answer",
                    "schema_version": "1.0",
                    "question_id": _QUESTION,
                    "predicate": "eligible-room-count",
                    "value": answer_value,
                    "oracle_policy_version": "synthetic-oracle-v1",
                },
            )
        )
        + b"\n"
    )
    return root


def _config(tmp_path: Path) -> PiBaselineConfig:
    auth = tmp_path / "oauth.json"
    auth.write_text("{}", encoding="utf-8")
    payload = valid_payload(auth)
    runtime_root = tmp_path / "runtime"
    runtime_root.mkdir()
    adapter = runtime_root / "main.js"
    adapter.write_bytes(b"synthetic pi adapter")
    runtime_files = {
        "main.js": b"synthetic pi adapter",
        "protocol.js": b"runtime protocol",
        "session.js": b"runtime session",
        "tools.js": b"runtime tools",
        "tool-definitions.v1.json": b"runtime tool schema",
    }
    (runtime_root / "runtime-manifest.v1.json").write_text(
        json.dumps(
            {
                "record_type": "pi-adapter-runtime-manifest",
                "schema_version": "1.1",
                "files": list(runtime_files),
            }
        ),
        encoding="utf-8",
    )
    for name, data in runtime_files.items():
        (runtime_root / name).write_bytes(data)
    corpus = _synthetic_corpus(tmp_path / "corpus")
    oracle = _synthetic_oracle(tmp_path / "oracle")
    selection = _selection()
    payload.update(
        {
            "node_adapter_command": ["/usr/bin/node", str(adapter)],
            "corpus_root": str(corpus),
            "oracle_root": str(oracle),
            "selection": selection,
            "fixed_smoke_identity": selection,
            "output_root": str(tmp_path / "out"),
            "private_root": str(tmp_path / "private"),
            "ledger_path": str(tmp_path / "ledger.jsonl"),
        }
    )
    return PiBaselineConfig.model_validate(payload)


def _case(selection: dict[str, str]) -> ExpandedCase:
    return ExpandedCase(case_id="case-1", payload={"selection": selection})


def _executor(
    tmp_path: Path, condition_runner: ConditionRunner, *, case_count: int = 1
) -> PiSchedulerExecutor:
    config = _config(tmp_path)
    config_path = tmp_path / "definition-config.json"
    config_path.write_text(json.dumps(config.model_dump(mode="json")), encoding="utf-8")
    spec_path = tmp_path / "definition-spec.json"
    spec_path.write_text(
        json.dumps(
            {
                "experiment_id": "experiment-1",
                "pi_config": config_path.name,
                "cases": [
                    {"case_id": f"case-{index}", "selection": _selection()}
                    for index in range(1, case_count + 1)
                ],
                "conditions": [{"name": "pi-condition", "prompt_mode": "visualization-forbidden"}],
            }
        ),
        encoding="utf-8",
    )
    experiment_dir = tmp_path / "experiment-1"
    manifest, plan, created_snapshot, _ = create_definition(
        experiment_dir, spec_path, workers=1, sample=None, shard=0, shards=1
    )
    FilesystemExperimentStore(experiment_dir).create(
        manifest,
        plan,
        additional_files={
            "executor.pi.v1.json": cast("JsonValue", created_snapshot.model_dump(mode="json"))
        },
    )
    preflight_bindings = PiRuntimeBindings(
        auth_file=tmp_path / "oauth.json",
        corpus_root=Path(config.corpus_root),
        oracle_root=Path(config.oracle_root),
        private_root=tmp_path / "preflight-private",
        ledger_path=tmp_path / "preflight-ledger.jsonl",
        public_root=tmp_path / "preflight-public",
    )
    validate_pi_definition(experiment_dir, preflight_bindings)
    from dimos.benchmark.spatial.pi_baseline.projection import stage_public_instance
    from dimos.benchmark.spatial.pi_baseline.prompts import build_prompt_pair

    prompts = build_prompt_pair()
    tool_definitions = _shared_tool_schemas()
    source_root = Path(__file__).parents[4]
    adapter = Path(config.node_adapter_command[-1]).read_bytes()
    runtime_names = (
        "main.js",
        "protocol.js",
        "session.js",
        "tools.js",
        "tool-definitions.v1.json",
        "runtime-manifest.v1.json",
    )
    material_paths = {
        "node.main": source_root / "packages/pi-spatial-adapter/src/main.ts",
        "node.session": source_root / "packages/pi-spatial-adapter/src/session.ts",
        "node.tools": source_root / "packages/pi-spatial-adapter/src/tools.ts",
        "node.protocol": source_root / "packages/pi-spatial-adapter/src/protocol.ts",
        "node.package-lock": source_root / "packages/pi-spatial-adapter/package-lock.json",
        "node.package": source_root / "packages/pi-spatial-adapter/package.json",
        "node.build": source_root / "packages/pi-spatial-adapter/tsconfig.json",
        "node.tool-schema": source_root
        / "packages/pi-spatial-adapter/src/tool-definitions.v1.json",
        "controller": Path(__file__).with_name("controller.py"),
        "runner": Path(__file__).with_name("runner.py"),
        "broker": Path(__file__).with_name("broker.py"),
        "scheduler_pi_executor": Path(__file__).with_name("scheduler_pi_executor.py"),
        "scorer": Path(__file__).with_name("scoring.py"),
        **host_module_paths(source_root),
    }
    material_bytes = {
        "prompt.forbidden": prompts.visualization_forbidden.encode(),
        "prompt.encouraged": prompts.visualization_encouraged.encode(),
        "tools.schemas": canonical_json(cast("JsonValue", tool_definitions)),
        **{name: path.read_bytes() for name, path in material_paths.items()},
        "adapter": adapter,
        **{
            f"node.runtime.{name}": (
                Path(config.node_adapter_command[-1]).parent / name
            ).read_bytes()
            for name in runtime_names
        },
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
    with tempfile.TemporaryDirectory() as directory:
        staged = stage_public_instance(Path(config.corpus_root), Path(directory), **_selection())
        manifest = json.loads((staged / "staging-manifest.v1.json").read_bytes())
        selected = PiSelectedInput(
            case_id="case-1",
            selection=_selection(),
            answer_type=AnswerType.INTEGER,
            case_sha256=_hash(staged / "cases/case.v1.json"),
            map_sha256=_hash(staged / "maps/map.lcm"),
            provenance_sha256=_hash(staged / "provenance.v1.json"),
            staging_manifest_sha256=_hash(staged / "staging-manifest.v1.json"),
            schema_sha256=_hash(staged / "schema.v1.json"),
            release_sha256=hashlib.sha256(canonical_json(manifest["release"])).hexdigest(),
            staging_inventory=tuple(
                sorted(
                    (
                        PiInventoryRecord(path=p.relative_to(staged).as_posix(), sha256=_hash(p))
                        for p in staged.rglob("*")
                        if p.is_file()
                    ),
                    key=lambda record: record.path,
                )
            ),
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
        implementation_digests={
            "adapter": f"adapter@sha256:{_hash_bytes(adapter)}",
            "scorer": f"scorer@sha256:{_hash(material_paths['scorer'])}",
            "protocol": f"protocol@sha256:{_hash(material_paths['node.protocol'])}",
        },
        prompt_fingerprint=_hash_bytes(material_bytes["prompt.forbidden"]),
        tool_fingerprint=_hash_bytes(material_bytes["tools.schemas"]),
        executor_fingerprint=_hash_bytes(adapter),
        prompt_text_forbidden=prompts.visualization_forbidden,
        prompt_text_encouraged=prompts.visualization_encouraged,
        tool_definitions=tool_definitions,
        pi_selected_inputs_digest=_hash_bytes(
            canonical_json(cast("JsonValue", [selected.model_dump(mode="json")]))
        ),
        material_inventory=tuple(
            PiMaterialRecord(name=n, sha256=_hash_bytes(b), bytes_b64=base64.b64encode(b).decode())
            for n, b in material_bytes.items()
        ),
        selected_inputs=(selected,),
    )
    bindings = PiRuntimeBindings(
        auth_file=tmp_path / "oauth.json",
        corpus_root=Path(config.corpus_root),
        oracle_root=tmp_path / "oracle",
        private_root=tmp_path / "runtime-private",
        ledger_path=tmp_path / "runtime-private" / "ledger.jsonl",
        public_root=tmp_path / "runtime-public",
    )
    executor = PiSchedulerExecutor(
        snapshot,
        bindings,
        manifest_executor_fingerprint=snapshot.canonical_digest(),
        condition_runner=condition_runner,  # type: ignore[arg-type]
    )
    # Test-only handles for authoritative pre-admission mutation tests.
    executor._test_experiment_dir = experiment_dir  # type: ignore[attr-defined]
    executor._test_bindings = preflight_bindings  # type: ignore[attr-defined]
    return executor


def _preflight_fixture(
    tmp_path: Path, *, case_count: int = 1
) -> tuple[SchedulerRuntime, Path, PiRuntimeBindings]:
    executor = _executor(
        tmp_path,
        lambda config, mode, cancel_requested, publication_lock: _result(mode),
        case_count=case_count,
    )
    bindings = executor._test_bindings  # type: ignore[attr-defined]
    experiment_dir = executor._test_experiment_dir  # type: ignore[attr-defined]
    return (
        SchedulerRuntime(FilesystemExperimentStore(experiment_dir), executor),
        experiment_dir,
        bindings,
    )


def _fresh_bindings(bindings: PiRuntimeBindings, root: Path) -> PiRuntimeBindings:
    return bindings.model_copy(
        update={
            "private_root": root / "private",
            "public_root": root / "public",
            "ledger_path": root / "ledger.jsonl",
        }
    )


def _assert_rejected_without_attempts(
    experiment_dir: Path, bindings: PiRuntimeBindings, match: str
) -> None:
    with pytest.raises(ValueError, match=match):
        validate_pi_definition(experiment_dir, bindings)
    attempts = experiment_dir / "attempts"
    assert not attempts.exists() or not any(attempts.iterdir())


def _answer_path(bindings: PiRuntimeBindings) -> Path:
    return bindings.oracle_root / "scenes" / _SCENE / "trajectories" / _TRAJECTORY / "answers.jsonl"


def _rewrite_answer(path: Path, **updates: object) -> None:
    answer = json.loads(path.read_text(encoding="utf-8").splitlines()[0])
    answer.update(updates)
    path.write_text(json.dumps(answer) + "\n", encoding="utf-8")


def test_preflight_rejects_staged_schema_drift_without_attempt(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    _, experiment_dir, bindings = _preflight_fixture(tmp_path)
    from dimos.benchmark.spatial.pi_baseline import projection

    original = projection.stage_public_instance

    def drifted_stage(*args: object, **kwargs: object) -> Path:
        staged = original(*args, **kwargs)
        (staged / "schema.v1.json").write_bytes(b"drifted staged schema")
        return staged

    monkeypatch.setattr(projection, "stage_public_instance", drifted_stage)
    _assert_rejected_without_attempts(
        experiment_dir,
        _fresh_bindings(bindings, tmp_path / "schema"),
        "staging schema does not match its record",
    )


def test_preflight_rejects_live_scorer_material_drift_without_attempt(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    _, experiment_dir, bindings = _preflight_fixture(tmp_path)
    scorer_path = Path(__file__).with_name("scoring.py").resolve()
    original = Path.read_bytes

    def drifted_read(path: Path) -> bytes:
        value = original(path)
        return value + b"\ndrift" if path.resolve() == scorer_path else value

    monkeypatch.setattr(Path, "read_bytes", drifted_read)
    _assert_rejected_without_attempts(
        experiment_dir,
        _fresh_bindings(bindings, tmp_path / "scorer"),
        "Pi scorer artifact drift detected",
    )


def test_preflight_rejects_private_answer_predicate_mismatch_without_attempt(
    tmp_path: Path,
) -> None:
    _, experiment_dir, bindings = _preflight_fixture(tmp_path)
    _rewrite_answer(_answer_path(bindings), predicate=Predicate.DIRECT_NEIGHBOR_COUNT.value)
    _assert_rejected_without_attempts(
        experiment_dir,
        _fresh_bindings(bindings, tmp_path / "predicate"),
        "private answer does not match the public question",
    )


def test_preflight_rejects_private_answer_value_type_mismatch_without_attempt(
    tmp_path: Path,
) -> None:
    _, experiment_dir, bindings = _preflight_fixture(tmp_path)
    _rewrite_answer(_answer_path(bindings), value={"kind": "boolean", "value": True})
    _assert_rejected_without_attempts(
        experiment_dir,
        _fresh_bindings(bindings, tmp_path / "answer-type"),
        "oracle answer type does not match the public AnswerType",
    )


def test_preflight_rejects_corrected_override_value_type_mismatch_without_attempt(
    tmp_path: Path,
) -> None:
    _, experiment_dir, bindings = _preflight_fixture(tmp_path)
    override_path = (
        bindings.oracle_root
        / "scenes"
        / _SCENE
        / "trajectories"
        / _TRAJECTORY
        / "review_overrides.jsonl"
    )
    override_path.parent.mkdir(parents=True, exist_ok=True)
    override_path.write_text(
        json.dumps(
            {
                "record_type": "review-override",
                "schema_version": "1.0",
                "override_id": "override_" + "a" * 64,
                "question_id": _QUESTION,
                "action": "correct",
                "reason": "synthetic correction",
                "corrected_value": {"kind": "boolean", "value": True},
            }
        )
        + "\n",
        encoding="utf-8",
    )
    _assert_rejected_without_attempts(
        experiment_dir,
        _fresh_bindings(bindings, tmp_path / "corrected"),
        "oracle answer type does not match the public AnswerType",
    )


def test_preflight_rejects_exclude_with_invalid_underlying_answer_without_attempt(
    tmp_path: Path,
) -> None:
    _, experiment_dir, bindings = _preflight_fixture(tmp_path)
    _rewrite_answer(_answer_path(bindings), value={"kind": "boolean", "value": True})
    override_path = (
        bindings.oracle_root
        / "scenes"
        / _SCENE
        / "trajectories"
        / _TRAJECTORY
        / "review_overrides.jsonl"
    )
    override_path.write_text(
        json.dumps(
            {
                "record_type": "review-override",
                "schema_version": "1.0",
                "override_id": "override_" + "b" * 64,
                "question_id": _QUESTION,
                "action": "exclude",
                "reason": "synthetic exclusion",
            }
        )
        + "\n",
        encoding="utf-8",
    )
    _assert_rejected_without_attempts(
        experiment_dir,
        _fresh_bindings(bindings, tmp_path / "exclude"),
        "oracle answer type does not match the public AnswerType",
    )


def test_preflight_rejects_manifest_bound_private_material_drift_without_attempt(
    tmp_path: Path,
) -> None:
    runtime, experiment_dir, bindings = _preflight_fixture(tmp_path)
    execute_pi_operation(runtime, bindings, "run", host_prerequisite=lambda: True)
    answer_path = _answer_path(bindings)
    answer = json.loads(answer_path.read_text(encoding="utf-8"))
    answer_path.write_text(json.dumps(answer, separators=(", ", ": ")) + "\n", encoding="utf-8")
    with pytest.raises(ValueError, match="private runtime tree is bound"):
        execute_pi_operation(runtime, bindings, "run", host_prerequisite=lambda: True)


def _condition(mode: str = "visualization-forbidden") -> NamedCondition:
    return NamedCondition(name="pi-condition", payload={"prompt_mode": mode})


def _context(job_id: str = "job-1") -> AttemptContext:
    return AttemptContext(
        identity=JobIdentity(
            experiment_id="experiment-1",
            case_id="case-1",
            condition_name="pi-condition",
            job_id=job_id,
        ),
        attempt_id="attempt-1",
        attempt_number=1,
        directory_name="attempt-1",
    )


def _hash(path: Path) -> str:
    return _hash_bytes(path.read_bytes())


def _hash_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _result(mode: PromptMode) -> ConditionRun:
    evidence = SmokeRunEvidence(
        run_id="attempt-1",
        mode=mode,
        case_sha256="a" * 64,
        manifest_sha256="b" * 64,
        review_bundle=ArtifactReference(path="manifest.json", sha256="c" * 64),
    )
    return ConditionRun("attempt-1", mode, Path("/isolated/case"), evidence)


def test_payloads_are_validated_before_runner_invocation(tmp_path: Path) -> None:
    calls: list[tuple[PiBaselineConfig, PromptMode]] = []

    def fake_runner(
        config: PiBaselineConfig, mode: PromptMode, cancel_requested: Event, publication_lock: Lock
    ) -> ConditionRun:
        calls.append((config, mode))
        return _result(mode)

    executor = _executor(tmp_path, fake_runner)
    invalid_case = _case({**_selection(), "scene_id": "not valid"})
    with pytest.raises(ValidationError):
        executor.run(invalid_case, _condition(), _context(), lambda _: None, *_operation_pair())
    with pytest.raises(ValidationError):
        executor.run(
            _case(_selection()),
            _condition("unsupported"),
            _context(),
            lambda _: None,
            *_operation_pair(),
        )
    assert not calls


def test_success_invokes_one_isolated_condition_and_emits_events(tmp_path: Path) -> None:
    calls: list[tuple[PiBaselineConfig, PromptMode]] = []

    def fake_runner(
        config: PiBaselineConfig, mode: PromptMode, cancel_requested: Event, publication_lock: Lock
    ) -> ConditionRun:
        calls.append((config, mode))
        return _result(mode)

    events: list[ExecutorProgressEvent | ExecutorArtifactEvent] = []
    outcome = _executor(tmp_path, fake_runner).run(
        _case(_selection()),
        _condition("visualization-encouraged"),
        _context(),
        events.append,
        *_operation_pair(),
    )

    assert outcome == TerminalOutcome(status="succeeded", reason="Pi condition completed")
    assert len(calls) == 1
    assert calls[0][0].selection.instance_id == _selection()["instance_id"]
    assert calls[0][0].run_id != "attempt-1"
    assert len(calls[0][0].run_id or "") <= 128
    assert calls[0][0].output_root == str(
        tmp_path / "runtime-public" / "experiment-1" / "job-1" / "attempt-1" / "public"
    )
    assert calls[0][0].private_root == str(
        tmp_path / "runtime-private" / "experiment-1" / "job-1" / "attempt-1" / "private"
    )
    assert calls[0][1] == "visualization-encouraged"
    assert [event.kind for event in events] == ["progress", "artifact"]
    assert events[0] == ExecutorProgressEvent(kind="progress", code="executor_progress")
    assert events[1] == ExecutorArtifactEvent(
        kind="artifact",
        code="artifact_recorded",
        artifact_id="evidence-attempt-1",
        artifact_sha256="b" * 64,
    )
    assert set(events[1].model_dump()) == {
        "kind",
        "code",
        "artifact_id",
        "artifact_sha256",
    }
    assert not (tmp_path / "runtime-private" / "ledger.jsonl").exists()


def test_job_validation_uses_only_its_indexed_selected_case(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    calls: list[tuple[str | None, bool]] = []
    original = scheduler_pi_executor_module.verify_public_inputs

    def observe(*args: object, **kwargs: object) -> None:
        selected = kwargs.get("selected_input")
        calls.append((None, selected is not None))
        original(*args, **kwargs)  # type: ignore[arg-type]

    monkeypatch.setattr(scheduler_pi_executor_module, "verify_public_inputs", observe)
    executor = _executor(tmp_path, lambda config, mode, *_: _result(mode))
    executor.run(_case(_selection()), _condition(), _context(), lambda _: None, *_operation_pair())
    assert calls == [(None, True)]


def test_exact_case_selection_mismatch_fails_before_runner(tmp_path: Path) -> None:
    calls: list[object] = []

    def runner(config: PiBaselineConfig, mode: PromptMode, *_: object) -> ConditionRun:
        calls.append(config)
        return _result(mode)

    mismatch = {**_selection(), "instance_id": "different-instance"}
    with pytest.raises(ValueError, match="immutable selected inputs"):
        _executor(tmp_path, runner).run(
            _case(mismatch), _condition(), _context(), lambda _: None, *_operation_pair()
        )
    assert not calls


def test_operation_preflight_scans_full_selection_once_for_many_jobs(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    full_scans = 0
    per_case_scans = 0
    original = scheduler_pi_executor_module.verify_public_inputs

    def observe(*args: object, **kwargs: object) -> None:
        nonlocal full_scans, per_case_scans
        if kwargs.get("selected_input") is None:
            full_scans += 1
        else:
            per_case_scans += 1
        original(*args, **kwargs)  # type: ignore[arg-type]

    runtime, _, bindings = _preflight_fixture(tmp_path, case_count=5)
    monkeypatch.setattr(scheduler_pi_executor_module, "verify_public_inputs", observe)
    monkeypatch.setattr(cli_support_module, "verify_public_inputs", observe)
    execute_pi_operation(runtime, bindings, "run", host_prerequisite=lambda: True)
    assert full_scans == 1
    assert per_case_scans == 5


def test_runtime_owned_operation_pair_reaches_pi_condition_runner(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    seen: dict[str, object] = {}

    def capture_runner(
        config: PiBaselineConfig,
        mode: PromptMode,
        cancel_requested: Event,
        publication_lock: Lock,
    ) -> ConditionRun:
        seen["event"] = cancel_requested
        seen["lock"] = publication_lock
        return _result(mode)

    class CapturingExecutor(PiSchedulerExecutor):
        def __init__(self, *args: object, **kwargs: object) -> None:
            super().__init__(*args, condition_runner=capture_runner, **kwargs)  # type: ignore[arg-type]

    runtime, _, bindings = _preflight_fixture(tmp_path)
    monkeypatch.setattr(scheduler_pi_executor_module, "PiSchedulerExecutor", CapturingExecutor)

    assert (
        execute_pi_operation(runtime, bindings, "run", host_prerequisite=lambda: True)[0].state
        == "succeeded"
    )
    assert seen["event"] is runtime._cancel_requested
    assert seen["lock"] is runtime._publication_lock


def test_attempt_one_ids_are_distinct_for_different_job_identities(tmp_path: Path) -> None:
    run_ids: list[str | None] = []

    def fake_runner(
        config: PiBaselineConfig, mode: PromptMode, cancel_requested: Event, publication_lock: Lock
    ) -> ConditionRun:
        run_ids.append(config.run_id)
        return _result(mode)

    executor = _executor(tmp_path, fake_runner)
    executor.run(
        _case(_selection()), _condition(), _context("job-1"), lambda _: None, *_operation_pair()
    )
    executor.run(
        _case(_selection()), _condition(), _context("job-2"), lambda _: None, *_operation_pair()
    )

    assert run_ids[0] is not None
    assert run_ids[1] is not None
    assert run_ids[0] != run_ids[1]
    assert len(run_ids[0] + "-encouraged") <= 63


def test_policy_failure_becomes_failed_outcome_without_live_runner(tmp_path: Path) -> None:
    def failing_runner(
        config: PiBaselineConfig, mode: PromptMode, cancel_requested: Event, publication_lock: Lock
    ) -> ConditionRun:
        raise PolicyViolationError("visualization_forbidden")

    events: list[ExecutorProgressEvent | ExecutorArtifactEvent] = []
    outcome = _executor(tmp_path, failing_runner).run(
        _case(_selection()), _condition(), _context(), events.append, *_operation_pair()
    )

    assert outcome.status == "failed"
    assert outcome.reason == "policy:visualization_forbidden"
    assert [event.kind for event in events] == ["progress"]


def test_adapter_protocol_failure_has_stable_public_reason_without_raw_error(
    tmp_path: Path,
) -> None:
    raw_message = "adapter_protocol_error: /private/corpus/secret-token"

    def failing_runner(
        config: PiBaselineConfig,
        mode: PromptMode,
        cancel_requested: Event,
        publication_lock: Lock,
    ) -> ConditionRun:
        raise AdapterRunError(raw_message)

    outcome = _executor(tmp_path, failing_runner).run(
        _case(_selection()), _condition(), _context(), lambda _: None, *_operation_pair()
    )

    assert outcome == TerminalOutcome(status="failed", reason="adapter_run_error")
    assert raw_message not in outcome.model_dump_json()
    assert "/private/corpus/secret-token" not in outcome.model_dump_json()


def test_generic_runner_failure_retains_redacted_private_record(
    tmp_path: Path,
) -> None:
    sentinel = "sentinel-secret-prompt-and-oauth"

    def failing_runner(
        config: PiBaselineConfig,
        mode: PromptMode,
        cancel_requested: Event,
        publication_lock: Lock,
    ) -> ConditionRun:
        raise RuntimeError(sentinel)

    events: list[ExecutorProgressEvent | ExecutorArtifactEvent] = []
    outcome = _executor(tmp_path, failing_runner).run(
        _case(_selection()), _condition(), _context(), events.append, *_operation_pair()
    )

    assert outcome == TerminalOutcome(status="failed", reason="executor_failed")
    record_path = (
        tmp_path
        / "runtime-private"
        / "experiment-1"
        / "job-1"
        / "attempt-1"
        / "private"
        / "failure.v1.json"
    )
    record = json.loads(record_path.read_text(encoding="utf-8"))
    assert record["record_type"] == "pi-run-failure"
    assert record["error"] == "RuntimeError"
    assert record["message"] == "scheduler exception"
    assert "subprocess" not in record
    assert all(set(frame) == {"module", "function", "line"} for frame in record["traceback"])
    assert sentinel not in record_path.read_text(encoding="utf-8")
    assert sentinel not in outcome.model_dump_json()
    assert sentinel not in repr(events)


def test_podman_infrastructure_failure_has_container_runtime_reason(tmp_path: Path) -> None:
    def failing_runner(
        config: PiBaselineConfig,
        mode: PromptMode,
        cancel_requested: Event,
        publication_lock: Lock,
    ) -> ConditionRun:
        raise PodmanInfrastructureError("run")

    outcome = _executor(tmp_path, failing_runner).run(
        _case(_selection()), _condition(), _context(), lambda _: None, *_operation_pair()
    )

    assert outcome == TerminalOutcome(status="failed", reason="container_runtime_failed")


@pytest.mark.parametrize(
    "terminal_reason", ["max_turns", "max_tool_calls", "timeout", "session_error", "protocol_error"]
)
def test_adapter_terminal_reason_is_retained_without_other_error_data(
    tmp_path: Path, terminal_reason: str
) -> None:
    error = AdapterRunError("adapter_run_failed", terminal_reason=terminal_reason)
    private = tmp_path / "private"
    _retain_failure_record(private, None, None, error)
    record = json.loads((private / "failure.v1.json").read_text(encoding="utf-8"))
    assert record["adapter_reason"] == terminal_reason
    assert "adapter_run_failed" not in record


def test_scheduler_distinguishes_adapter_terminal_reason_without_leaking_details(
    tmp_path: Path,
) -> None:
    def failing_runner(
        config: PiBaselineConfig,
        mode: PromptMode,
        cancel_requested: Event,
        publication_lock: Lock,
    ) -> ConditionRun:
        raise AdapterRunError("adapter_run_failed", terminal_reason="timeout")

    outcome = _executor(tmp_path, failing_runner).run(
        _case(_selection()), _condition(), _context(), lambda _: None, *_operation_pair()
    )
    record_path = (
        tmp_path
        / "runtime-private"
        / "experiment-1"
        / "job-1"
        / "attempt-1"
        / "private"
        / "failure.v1.json"
    )
    record = json.loads(record_path.read_text(encoding="utf-8"))
    assert outcome == TerminalOutcome(status="failed", reason="adapter_run_failed:timeout")
    assert record["adapter_reason"] == "timeout"
    assert "adapter_run_failed" not in record


def test_descriptor_mismatch_retains_only_safe_private_metadata(tmp_path: Path) -> None:
    mismatch = DescriptorMismatchError(6, 7, "a" * 64)

    def failing_runner(
        config: PiBaselineConfig,
        mode: PromptMode,
        cancel_requested: Event,
        publication_lock: Lock,
    ) -> ConditionRun:
        raise mismatch

    outcome = _executor(tmp_path, failing_runner).run(
        _case(_selection()), _condition(), _context(), lambda _: None, *_operation_pair()
    )
    record_path = (
        tmp_path
        / "runtime-private"
        / "experiment-1"
        / "job-1"
        / "attempt-1"
        / "private"
        / "failure.v1.json"
    )
    record = json.loads(record_path.read_text(encoding="utf-8"))

    assert outcome == TerminalOutcome(status="failed", reason="executor_failed")
    assert record["descriptor_mismatch"] == {
        "expected_entry_count": 6,
        "actual_entry_count": 7,
        "actual_entries_sha256": "a" * 64,
    }
    rendered = record_path.read_text(encoding="utf-8")
    assert "unexpected-entry-secret" not in rendered
    assert set(record["descriptor_mismatch"]) == {
        "expected_entry_count",
        "actual_entry_count",
        "actual_entries_sha256",
    }


def test_projection_descriptor_mismatch_reports_set_metadata(tmp_path: Path) -> None:
    corpus = _synthetic_corpus(tmp_path / "corpus")
    staging_parent = tmp_path / "staging"
    staging_parent.mkdir()
    staged = cast(
        "Path",
        projection_module.stage_public_instance(
            corpus,
            staging_parent,
            **_selection(),
        ),
    )
    (staged / "unexpected-entry-secret").write_bytes(b"private")
    root = PinnedDirectory.open(staged, create=False)
    try:
        with pytest.raises(DescriptorMismatchError) as raised:
            _verify_staging_descriptor(
                root,
                StagingRecord(
                    case_path="cases/case.v1.json",
                    map_path="maps/map.lcm",
                    map_sha256=_hash(staged / "maps/map.lcm"),
                    schema_sha256=_hash(staged / "schema.v1.json"),
                ),
            )
    finally:
        root.close()

    error = raised.value
    assert error.expected_entry_count == 6
    assert error.actual_entry_count == 7
    assert error.actual_entries_sha256 == _hash_bytes(
        canonical_json(
            cast(
                "JsonValue",
                sorted(
                    {
                        "cases/case.v1.json",
                        "maps/map.lcm",
                        "schema.v1.json",
                        "staging-manifest.v1.json",
                        "inventory.v1.json",
                        "provenance.v1.json",
                        "unexpected-entry-secret",
                    }
                ),
            )
        )
    )


def test_projection_uses_fresh_cursor_after_root_cursor_is_advanced(tmp_path: Path) -> None:
    corpus = _synthetic_corpus(tmp_path / "corpus")
    staging_parent = tmp_path / "staging"
    staging_parent.mkdir()
    staged = cast(
        "Path",
        projection_module.stage_public_instance(
            corpus,
            staging_parent,
            **_selection(),
        ),
    )
    root = PinnedDirectory.open(staged, create=False)
    try:
        with os.scandir(root.fd) as entries:
            tuple(entries)
        _verify_staging_descriptor(
            root,
            StagingRecord(
                case_path="cases/case.v1.json",
                map_path="maps/map.lcm",
                map_sha256=_hash(staged / "maps/map.lcm"),
                schema_sha256=_hash(staged / "schema.v1.json"),
            ),
        )
        root.verify()
    finally:
        root.close()
    assert root.fd == -1


def test_failure_record_persistence_failure_preserves_public_outcome(
    tmp_path: Path,
) -> None:
    def failing_runner(
        config: PiBaselineConfig,
        mode: PromptMode,
        cancel_requested: Event,
        publication_lock: Lock,
    ) -> ConditionRun:
        raise RuntimeError("private persistence must not affect outcome")

    executor = _executor(tmp_path, failing_runner)
    external = tmp_path / "external-private"
    external.mkdir()
    private_root = tmp_path / "symlink-private"
    private_root.symlink_to(external, target_is_directory=True)
    executor._runtime_bindings = executor._runtime_bindings.model_copy(  # type: ignore[attr-defined]
        update={"private_root": private_root}
    )
    outcome = executor.run(
        _case(_selection()), _condition(), _context(), lambda _: None, *_operation_pair()
    )

    assert outcome == TerminalOutcome(status="failed", reason="executor_failed")


@pytest.mark.parametrize(
    ("command", "operation"),
    [
        (["podman", "create", "--name", "secret-container"], "podman-create"),
        (["podman", "run", "--detach", "secret-image"], "podman-create"),
        (["/usr/bin/podman", "start", "secret-container"], "podman-start"),
        (["podman", "logs", "secret-container"], "podman-logs"),
        (["podman", "run", "--volume", "/secret/path:/work"], "subprocess"),
        (["/usr/bin/node", "secret-script.js"], "subprocess"),
    ],
)
def test_runner_called_process_failure_record_is_allowlisted_and_redacted(
    tmp_path: Path, command: list[str], operation: str
) -> None:
    error = subprocess.CalledProcessError(
        17, command, output="secret stdout", stderr="secret stderr"
    )
    private = tmp_path / "private"
    _retain_failure_record(private, None, None, error)

    record_path = private / "failure.v1.json"
    record = json.loads(record_path.read_text(encoding="utf-8"))
    assert record["subprocess"] == {"operation": operation, "returncode": 17}
    rendered = record_path.read_text(encoding="utf-8")
    assert "secret-container" not in rendered
    assert "/secret/path" not in rendered
    assert "secret stdout" not in rendered
    assert "secret stderr" not in rendered


def test_podman_timeout_failure_record_contains_only_safe_operation(
    tmp_path: Path,
) -> None:
    private = tmp_path / "private"
    error = PodmanTimeoutError("exec", "command_cap")
    _retain_failure_record(private, "visualization-encouraged", None, error)

    record_path = private / "failure.v1.json"
    record = json.loads(record_path.read_text(encoding="utf-8"))
    assert record["subprocess"] == {
        "timeout_operation": "exec",
        "deadline_source": "command_cap",
        "stdout_truncated": False,
        "stderr_truncated": False,
        "stdout_bucket": "empty",
        "stderr_bucket": "empty",
    }
    rendered = record_path.read_text(encoding="utf-8")
    for secret in ("podman", "/private/credential", "secret-output", str(error)):
        assert secret not in rendered


def test_scheduler_called_process_failure_preserves_public_outcome_and_redaction(
    tmp_path: Path,
) -> None:
    command = ["podman", "create", "--volume", "/secret/path:/work"]

    def failing_runner(
        config: PiBaselineConfig,
        mode: PromptMode,
        cancel_requested: Event,
        publication_lock: Lock,
    ) -> ConditionRun:
        raise subprocess.CalledProcessError(23, command, stderr="secret stderr")

    outcome = _executor(tmp_path, failing_runner).run(
        _case(_selection()), _condition(), _context(), lambda _: None, *_operation_pair()
    )
    record_path = (
        tmp_path
        / "runtime-private"
        / "experiment-1"
        / "job-1"
        / "attempt-1"
        / "private"
        / "failure.v1.json"
    )
    record = json.loads(record_path.read_text(encoding="utf-8"))
    assert outcome == TerminalOutcome(status="failed", reason="executor_failed")
    assert record["subprocess"] == {"operation": "podman-create", "returncode": 23}
    rendered = record_path.read_text(encoding="utf-8")
    assert "secret/path" not in rendered
    assert "secret stderr" not in rendered


@pytest.mark.parametrize(
    "cleanup_error",
    [
        ContainerCleanupError("container cleanup leaked details"),
        AdapterCleanupError("adapter cleanup leaked details"),
    ],
    ids=["container", "adapter"],
)
def test_cleanup_failures_become_safe_failed_outcomes_without_raw_error_leakage(
    tmp_path: Path, cleanup_error: Exception
) -> None:
    def failing_runner(
        config: PiBaselineConfig,
        mode: PromptMode,
        cancel_requested: Event,
        publication_lock: Lock,
    ) -> ConditionRun:
        raise cleanup_error

    outcome = _executor(tmp_path, failing_runner).run(
        _case(_selection()), _condition(), _context(), lambda _: None, *_operation_pair()
    )

    assert outcome == TerminalOutcome(status="failed", reason="container_cleanup_failed")
    assert str(cleanup_error) not in outcome.reason
    assert "leaked details" not in outcome.model_dump_json()


def test_snapshot_excludes_runtime_bindings_and_digest_mismatch_stops_execution(
    tmp_path: Path,
) -> None:
    snapshot = _executor(
        tmp_path, lambda config, mode, cancel_requested, publication_lock: _result(mode)
    )._snapshot
    serialized = snapshot.model_dump(mode="json")
    assert "codex_oauth_auth_path" not in serialized
    assert "corpus_root" not in serialized
    assert "oracle_root" not in serialized
    assert "output_root" not in serialized
    assert "private_root" not in serialized
    assert "ledger_path" not in serialized

    runner_called = False

    def fake_runner(
        config: PiBaselineConfig, mode: PromptMode, cancel_requested: Event, publication_lock: Lock
    ) -> ConditionRun:
        nonlocal runner_called
        runner_called = True
        return _result(mode)

    with pytest.raises(ValueError, match="does not match"):
        PiSchedulerExecutor(
            snapshot,
            PiRuntimeBindings(
                auth_file=tmp_path / "oauth.json",
                corpus_root=tmp_path / "corpus",
                oracle_root=tmp_path / "oracle",
                private_root=tmp_path / "private",
                ledger_path=tmp_path / "ledger.jsonl",
                public_root=tmp_path / "public",
            ),
            manifest_executor_fingerprint="0" * 64,
            condition_runner=fake_runner,
        )
    assert not runner_called


def test_snapshot_rejects_actual_material_byte_drift(tmp_path: Path) -> None:
    snapshot = _executor(
        tmp_path, lambda config, mode, cancel_requested, publication_lock: _result(mode)
    )._snapshot
    records = list(snapshot.material_inventory)
    original = records[0]
    records[0] = original.model_copy(
        update={"bytes_b64": base64.b64encode(b"drifted bytes").decode("ascii")}
    )
    drifted = snapshot.model_copy(update={"material_inventory": tuple(records)})

    with pytest.raises(ValueError, match="bytes do not match digest"):
        verify_snapshot_artifacts(drifted)


@pytest.mark.parametrize(
    "material_name",
    [
        "broker",
        "scheduler_pi_executor",
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
    ],
)
def test_snapshot_rejects_live_host_module_material_drift(
    tmp_path: Path, material_name: str
) -> None:
    snapshot = _executor(
        tmp_path, lambda config, mode, cancel_requested, publication_lock: _result(mode)
    )._snapshot
    records = list(snapshot.material_inventory)
    index = next(index for index, record in enumerate(records) if record.name == material_name)
    records[index] = records[index].model_copy(
        update={"bytes_b64": base64.b64encode(b"live host module drift").decode("ascii")}
    )
    with pytest.raises(ValueError, match="bytes do not match digest"):
        verify_snapshot_artifacts(
            snapshot.model_copy(update={"material_inventory": tuple(records)})
        )


def test_runtime_adapter_rejects_stale_runtime_file_content(tmp_path: Path) -> None:
    executor = _executor(
        tmp_path, lambda config, mode, cancel_requested, publication_lock: _result(mode)
    )
    runtime_root = Path(executor._snapshot.node_adapter_command[1]).parent
    (runtime_root / "tools.js").write_bytes(b"stale runtime tools")
    with pytest.raises(ValueError, match="node.runtime.tools.js artifact drift detected"):
        verify_runtime_artifacts(executor._snapshot, executor._snapshot.node_adapter_command)


def test_runtime_adapter_rejects_missing_and_additional_runtime_files(tmp_path: Path) -> None:
    executor = _executor(
        tmp_path, lambda config, mode, cancel_requested, publication_lock: _result(mode)
    )
    runtime_root = Path(executor._snapshot.node_adapter_command[1]).parent
    (runtime_root / "tools.js").unlink()
    with pytest.raises(ValueError, match="runtime files do not match manifest"):
        verify_runtime_artifacts(executor._snapshot, executor._snapshot.node_adapter_command)

    additional_root = tmp_path / "additional"
    additional_root.mkdir()
    executor = _executor(
        additional_root, lambda config, mode, cancel_requested, publication_lock: _result(mode)
    )
    runtime_root = Path(executor._snapshot.node_adapter_command[1]).parent
    (runtime_root / "stale.js").write_bytes(b"stale")
    with pytest.raises(ValueError, match="runtime files do not match manifest"):
        verify_runtime_artifacts(executor._snapshot, executor._snapshot.node_adapter_command)


def test_snapshot_rejects_runtime_material_drift(tmp_path: Path) -> None:
    snapshot = _executor(
        tmp_path, lambda config, mode, cancel_requested, publication_lock: _result(mode)
    )._snapshot
    records = list(snapshot.material_inventory)
    index = next(
        index for index, record in enumerate(records) if record.name == "node.runtime.tools.js"
    )
    records[index] = records[index].model_copy(
        update={"bytes_b64": base64.b64encode(b"drift").decode()}
    )
    with pytest.raises(ValueError, match="material bytes do not match digest"):
        verify_snapshot_artifacts(
            snapshot.model_copy(update={"material_inventory": tuple(records)})
        )


def test_runtime_public_corpus_drift_is_rejected(tmp_path: Path) -> None:
    snapshot = _executor(
        tmp_path, lambda config, mode, cancel_requested, publication_lock: _result(mode)
    )._snapshot
    corpus_copy = tmp_path / "corpus"
    _synthetic_corpus(corpus_copy)
    manifest_path = corpus_copy / "manifest.json"
    manifest = json.loads(manifest_path.read_bytes())
    manifest["release_version"] = "v9.9.9"
    manifest_path.write_text(json.dumps(manifest), encoding="utf-8")

    with pytest.raises(
        ValueError, match="selected public input drift|staging inventory drift|release"
    ):
        verify_public_inputs(snapshot, corpus_copy)


def test_frozen_binding_commit_is_idempotent(tmp_path: Path) -> None:
    executor = _executor(
        tmp_path, lambda config, mode, cancel_requested, publication_lock: _result(mode)
    )
    store = FilesystemExperimentStore(executor._test_experiment_dir)  # type: ignore[attr-defined]
    manifest = store.load_definition().manifest
    snapshot = executor._snapshot
    oracle_root = tmp_path / "oracle-drift"
    oracle_root.mkdir()
    (oracle_root / "answers.jsonl").write_bytes(b"synthetic private oracle\n")
    private_root = tmp_path / "private"
    frozen_manifest_digest = manifest_digest(manifest)
    frozen_binding_digest = private_binding_digest(frozen_manifest_digest, snapshot, oracle_root)
    from dimos.benchmark.spatial.pi_baseline.cli_support import bind_private_tree

    with store.coordinator_lease() as capability:
        bind_private_tree(
            private_root,
            manifest.experiment_id,
            frozen_manifest_digest,
            frozen_binding_digest,
            store=store,
            capability=capability,
        )
        binding_path = private_root / manifest.experiment_id / "manifest.digest"
        before = binding_path.read_bytes()
        bind_private_tree(
            private_root,
            manifest.experiment_id,
            frozen_manifest_digest,
            frozen_binding_digest,
            store=store,
            capability=capability,
        )
        assert binding_path.read_bytes() == before


def test_lifecycle_events_contain_only_public_identifiers_and_digests(tmp_path: Path) -> None:
    events: list[ExecutorProgressEvent | ExecutorArtifactEvent] = []

    def fake_runner(
        config: PiBaselineConfig, mode: PromptMode, cancel_requested: Event, publication_lock: Lock
    ) -> ConditionRun:
        return _result(mode)

    _executor(tmp_path, fake_runner).run(
        _case(_selection()), _condition(), _context(), events.append, *_operation_pair()
    )
    rendered = repr(events)
    assert "manifest.json" not in rendered
    assert "runtime-private" not in rendered
    assert "runtime-public" not in rendered
    assert "score" not in rendered
    assert "attempt-1" in rendered
    assert "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb" in rendered
    assert "b" * 64 in rendered
