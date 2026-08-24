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

"""Deterministic eval-row and simulator-grounded VQA dataset generation.

Ground truth is computed analytically from a *privileged* modality; the emitted
case quizzes a different (or lossily-encoded) surface. Rows are pure data —
a suite module maps them onto typed :class:`PassiveEval` cases.
"""

from __future__ import annotations

from collections import Counter
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
import json
from pathlib import Path
from tempfile import TemporaryDirectory
from typing import TYPE_CHECKING, Any

import yaml

from dimos.evals.robocasa import export_robocasa_snapshots
from dimos.evals.scorers import choice, exact, yes_no
from dimos.evals.types import PassiveEval, Select, Suite
from dimos.evals.vqa import (
    SUPPORTED_QUESTION_FAMILIES,
    GeneratedQuestion,
    SceneSnapshot,
    generate_questions,
    normalize_category,
)
from dimos.memory.cli.dataset import open_dataset
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat

if TYPE_CHECKING:
    from dimos.memory.store.base import Store
    from dimos.memory.stream import Stream

Row = dict[str, object]
VQA_SCHEMA_VERSION = 1
_SPEC_KEYS = frozenset(
    {
        "source",
        "use_case",
        "task",
        "episodes",
        "seed",
        "camera",
        "image_size",
        "question_families",
        "targets",
        "output",
        "split",
        "robot",
        "min_visible_pixels",
        "min_horizontal_separation",
        "max_bbox_overlap",
        "max_spatial_pairs",
    }
)


@dataclass(frozen=True, kw_only=True)
class VqaGenerationSpec:
    source: str
    use_case: str
    task: str
    episodes: int
    seed: int
    camera: str
    image_size: tuple[int, int]
    question_families: tuple[str, ...]
    targets: tuple[str, ...]
    output: Path
    split: str | None = "target"
    robot: str = "PandaOmron"
    min_visible_pixels: int = 64
    min_horizontal_separation: float = 0.1
    max_bbox_overlap: float = 0.5
    max_spatial_pairs: int = 8

    def __post_init__(self) -> None:
        if self.source != "robocasa":
            raise ValueError(f"unsupported VQA source: {self.source!r}")
        if not self.use_case.strip() or not self.task.strip() or not self.camera.strip():
            raise ValueError("use_case, task, and camera must not be empty")
        object.__setattr__(self, "use_case", normalize_category(self.use_case))
        if self.episodes <= 0:
            raise ValueError("episodes must be positive")
        if len(self.image_size) != 2 or any(size <= 0 for size in self.image_size):
            raise ValueError("image_size must contain two positive integers")
        if not self.question_families:
            raise ValueError("question_families must not be empty")
        unknown = set(self.question_families) - SUPPORTED_QUESTION_FAMILIES
        if unknown:
            raise ValueError(f"unsupported question families: {sorted(unknown)}")
        if not self.targets:
            raise ValueError("targets must not be empty")
        object.__setattr__(
            self,
            "targets",
            tuple(dict.fromkeys(normalize_category(target) for target in self.targets)),
        )
        if self.split not in {None, "all", "pretrain", "target"}:
            raise ValueError("split must be one of: all, pretrain, target, or null")
        if self.min_visible_pixels <= 0:
            raise ValueError("min_visible_pixels must be positive")
        if not 0 < self.min_horizontal_separation <= 1:
            raise ValueError("min_horizontal_separation must be in (0, 1]")
        if not 0 <= self.max_bbox_overlap <= 1:
            raise ValueError("max_bbox_overlap must be in [0, 1]")
        if self.max_spatial_pairs <= 0:
            raise ValueError("max_spatial_pairs must be positive")


@dataclass(frozen=True, kw_only=True)
class GenerationOutput:
    output_dir: Path
    manifest: Path
    dataset: Path
    summary: Path
    accepted: int
    accepted_by_family: Mapping[str, int]
    rejected: Mapping[str, int]


@dataclass(frozen=True, kw_only=True)
class _PendingCase:
    snapshot_index: int
    question: GeneratedQuestion


def load_generation_spec(path: str | Path) -> VqaGenerationSpec:
    spec_path = Path(path).expanduser().resolve()
    raw = yaml.safe_load(spec_path.read_text())
    if not isinstance(raw, dict):
        raise TypeError("VQA generation spec must be a YAML/JSON object")
    unknown = set(raw) - _SPEC_KEYS
    if unknown:
        raise ValueError(f"unknown VQA generation spec fields: {sorted(unknown)}")

    required = {
        "source",
        "use_case",
        "task",
        "episodes",
        "seed",
        "camera",
        "image_size",
        "question_families",
        "targets",
        "output",
    }
    missing = required - set(raw)
    if missing:
        raise ValueError(f"missing VQA generation spec fields: {sorted(missing)}")

    image_size = raw["image_size"]
    families = raw["question_families"]
    targets = raw["targets"]
    if not isinstance(image_size, list) or len(image_size) != 2:
        raise ValueError("image_size must be a two-item list")
    if not isinstance(families, list) or not all(isinstance(v, str) for v in families):
        raise ValueError("question_families must be a list of strings")
    if not isinstance(targets, list) or not all(isinstance(v, str) for v in targets):
        raise ValueError("targets must be a list of strings")

    output = Path(str(raw["output"])).expanduser()
    if not output.is_absolute():
        output = spec_path.parent / output
    return VqaGenerationSpec(
        source=str(raw["source"]),
        use_case=str(raw["use_case"]),
        task=str(raw["task"]),
        episodes=int(raw["episodes"]),
        seed=int(raw["seed"]),
        camera=str(raw["camera"]),
        image_size=(int(image_size[0]), int(image_size[1])),
        question_families=tuple(families),
        targets=tuple(targets),
        output=output.resolve(),
        split=(str(raw.get("split", "target")) if raw.get("split", "target") is not None else None),
        robot=str(raw.get("robot", "PandaOmron")),
        min_visible_pixels=int(raw.get("min_visible_pixels", 64)),
        min_horizontal_separation=float(raw.get("min_horizontal_separation", 0.1)),
        max_bbox_overlap=float(raw.get("max_bbox_overlap", 0.5)),
        max_spatial_pairs=int(raw.get("max_spatial_pairs", 8)),
    )


def generate_from_spec(path: str | Path, *, source_python: str) -> GenerationOutput:
    spec = load_generation_spec(path)
    snapshots = export_robocasa_snapshots(spec, source_python=source_python)
    return materialize_vqa_dataset(spec, snapshots)


def materialize_vqa_dataset(
    spec: VqaGenerationSpec, snapshots: Sequence[SceneSnapshot]
) -> GenerationOutput:
    """Generate cases and atomically materialize a Memory dataset + manifest."""
    if not snapshots:
        raise ValueError("VQA source returned no scene snapshots")
    if spec.output.exists():
        raise FileExistsError(f"VQA output already exists: {spec.output}")

    observed_categories = sorted(
        {entity.category for snapshot in snapshots for entity in snapshot.entities}
    )
    pending: list[_PendingCase] = []
    rejected: Counter[str] = Counter()
    for snapshot_index, snapshot in enumerate(snapshots):
        batch = generate_questions(
            snapshot,
            targets=spec.targets,
            families=spec.question_families,
            min_visible_pixels=spec.min_visible_pixels,
            min_horizontal_separation=spec.min_horizontal_separation,
            max_bbox_overlap=spec.max_bbox_overlap,
            max_spatial_pairs=spec.max_spatial_pairs,
        )
        pending.extend(
            _PendingCase(snapshot_index=snapshot_index, question=question)
            for question in batch.questions
        )
        rejected.update(batch.rejected)

    pending, balance_dropped = _balance_presence(pending)
    if balance_dropped:
        rejected["presence_balance"] += balance_dropped
    accepted_by_family: Counter[str] = Counter(case.question.family for case in pending)
    for family in spec.question_families:
        if accepted_by_family[family] == 0:
            detail = (
                f"accepted={dict(sorted(accepted_by_family.items()))}, "
                f"rejected={dict(sorted(rejected.items()))}, "
                f"observed_categories={observed_categories}"
            )
            if family == "semantic_presence":
                raise ValueError(
                    "semantic_presence requires at least one accepted yes and one accepted no case; "
                    + detail
                )
            raise ValueError(f"no answerable {family} cases were generated; {detail}")

    spec.output.parent.mkdir(parents=True, exist_ok=True)
    with TemporaryDirectory(prefix=f".{spec.output.name}-", dir=spec.output.parent) as temp:
        temp_dir = Path(temp)
        dataset = temp_dir / "observations.db"
        manifest = temp_dir / "cases.json"
        summary = temp_dir / "generation_summary.json"
        rows = _write_dataset_and_rows(spec, snapshots, pending, dataset)
        manifest.write_text(json.dumps(rows, indent=2) + "\n")
        summary_payload = {
            "schema_version": VQA_SCHEMA_VERSION,
            "accepted": len(rows),
            "accepted_by_family": dict(sorted(accepted_by_family.items())),
            "accepted_by_answer": dict(
                sorted(Counter(case.question.reference_outputs for case in pending).items())
            ),
            "rejected": dict(sorted(rejected.items())),
            "observed_categories": observed_categories,
            "episodes": len(snapshots),
        }
        summary.write_text(json.dumps(summary_payload, indent=2) + "\n")
        temp_dir.rename(spec.output)

    return GenerationOutput(
        output_dir=spec.output,
        manifest=spec.output / "cases.json",
        dataset=spec.output / "observations.db",
        summary=spec.output / "generation_summary.json",
        accepted=len(pending),
        accepted_by_family=dict(sorted(accepted_by_family.items())),
        rejected=dict(sorted(rejected.items())),
    )


def load_vqa_manifest(path: str | Path) -> Suite:
    """Load generated rows as ordinary ``PassiveEval`` cases."""
    manifest = Path(path).expanduser().resolve()
    raw = json.loads(manifest.read_text())
    if not isinstance(raw, list) or not raw:
        raise ValueError("VQA manifest must be a non-empty list")

    cases: list[PassiveEval[str]] = []
    for index, row in enumerate(raw):
        if not isinstance(row, dict):
            raise TypeError(f"VQA manifest row {index} must be an object")
        if row.get("schema_version") != VQA_SCHEMA_VERSION:
            raise ValueError(f"VQA manifest row {index} has unsupported schema_version")
        answer_type = row.get("answer_type")
        if answer_type == "yes_no":
            parse = yes_no
        elif answer_type == "choice":
            parse = choice
        else:
            raise ValueError(f"VQA manifest row {index} has unsupported answer_type")
        dataset = Path(str(row["dataset"])).expanduser()
        if not dataset.is_absolute():
            dataset = manifest.parent / dataset
        stream = str(row["stream"])
        frame_ts = float(row["frame_ts"])
        tags = row.get("tags", [])
        if not isinstance(tags, list) or not all(isinstance(tag, str) for tag in tags):
            raise ValueError(f"VQA manifest row {index} tags must be strings")
        cases.append(
            PassiveEval(
                id=str(row["id"]),
                inputs=str(row["inputs"]),
                expected=str(row["reference_outputs"]),
                parse=parse,
                score=exact,
                context=(_select_frame(stream, frame_ts),),
                dataset=str(dataset.resolve()),
                tags=frozenset(tags),
            )
        )
    return cases


def _select_frame(name: str, ts: float) -> Select:
    def select(store: Store) -> Stream[Any, Any]:
        return store.streams[name].at(ts, tolerance=1e-6).limit(1)

    return select


def _balance_presence(cases: Sequence[_PendingCase]) -> tuple[list[_PendingCase], int]:
    yes = [
        case
        for case in cases
        if case.question.family == "semantic_presence" and case.question.reference_outputs == "yes"
    ]
    no = [
        case
        for case in cases
        if case.question.family == "semantic_presence" and case.question.reference_outputs == "no"
    ]
    other = [case for case in cases if case.question.family != "semantic_presence"]
    keep = min(len(yes), len(no))
    balanced_presence = [item for pair in zip(yes[:keep], no[:keep], strict=True) for item in pair]
    return [*balanced_presence, *other], len(yes) + len(no) - 2 * keep


def _write_dataset_and_rows(
    spec: VqaGenerationSpec,
    snapshots: Sequence[SceneSnapshot],
    cases: Sequence[_PendingCase],
    dataset: Path,
) -> list[dict[str, Any]]:
    store = SqliteStore(path=str(dataset))
    images = store.stream("color_image", Image)
    rows: list[dict[str, Any]] = []
    written_snapshots: set[int] = set()
    family_ordinals: Counter[str] = Counter()
    try:
        for case in cases:
            snapshot = snapshots[case.snapshot_index]
            frame_ts = float(case.snapshot_index)
            if case.snapshot_index not in written_snapshots:
                images.append(
                    Image.from_numpy(
                        snapshot.image,
                        format=ImageFormat.RGB,
                        frame_id=spec.camera,
                        ts=frame_ts,
                    ),
                    ts=frame_ts,
                )
                written_snapshots.add(case.snapshot_index)

            question = case.question
            ordinal = family_ordinals[question.family]
            family_ordinals[question.family] += 1
            provenance = dict(snapshot.provenance)
            seed = int(provenance.get("seed", spec.seed + case.snapshot_index))
            rows.append(
                {
                    "schema_version": VQA_SCHEMA_VERSION,
                    "id": (f"{spec.use_case}-seed{seed:04d}-{question.family}-{ordinal:04d}"),
                    "inputs": question.inputs,
                    "reference_outputs": question.reference_outputs,
                    "answer_type": question.answer_type,
                    "dataset": "observations.db",
                    "stream": "color_image",
                    "frame_ts": frame_ts,
                    "tags": ["generated", "vqa", *question.family.split("_", 1)],
                    "provenance": provenance,
                    "oracle": dict(question.oracle),
                }
            )
    finally:
        store.stop()
    return rows


def displacement_rows(dataset: str, windows: Sequence[tuple[float, float]]) -> list[Row]:
    """Straight-line displacement over each window (odom is the privileged truth;
    the case quizzes the encoded odom summary). Sampling-invariant ground truth."""
    store = open_dataset(dataset)
    try:
        rows: list[Row] = []
        for t1, t2 in windows:
            obs = store.streams.odom.range_time(t1, t2).to_list()
            if len(obs) < 2:
                continue
            d = (obs[-1].data.position - obs[0].data.position).length()
            rows.append(
                {
                    "id": f"{dataset}_disp_{t1:g}_{t2:g}",
                    "q": "How far in a straight line is your final position from your "
                    "position at the first shown observation, in meters?",
                    "a": round(d, 1),
                    "band": max(1.0, d * 0.4),
                    "stream": "odom",
                    "window": [t1, t2],
                    "dataset": dataset,
                }
            )
        return rows
    finally:
        store.stop()


def path_length_rows(dataset: str, windows: Sequence[tuple[float, float]]) -> list[Row]:
    """Integrated path length per window. Deliberately hard on a downsampled
    encoding — expect partial credit; that gap is the finding."""
    store = open_dataset(dataset)
    try:
        rows: list[Row] = []
        for t1, t2 in windows:
            path, prev = 0.0, None
            for obs in store.streams.odom.range_time(t1, t2):
                p = obs.data.position
                if prev is not None:
                    path += (p - prev).length()
                prev = p
            if prev is None:
                continue
            rows.append(
                {
                    "id": f"{dataset}_path_{t1:g}_{t2:g}",
                    "q": "Roughly how many meters did you travel in total over these "
                    "observations (path length, not displacement)?",
                    "a": round(path, 1),
                    "band": max(2.0, path * 0.5),
                    "stream": "odom",
                    "window": [t1, t2],
                    "dataset": dataset,
                }
            )
        return rows
    finally:
        store.stop()
