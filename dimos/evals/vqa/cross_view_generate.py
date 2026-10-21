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

"""Generate cross-view same-object VQA cases from Dan localize traces."""

from __future__ import annotations

from dataclasses import replace
import json
from pathlib import Path
import shutil
import tempfile
from typing import Any

import jsonlines
from pydantic import BaseModel, ConfigDict, Field

from dimos.agents.cli.util.file_system_utils import STATE_DIR
from dimos.evals.vqa.contracts import InsufficientEvidenceError
from dimos.evals.vqa.generate import PrivateLabel, PublicCase
from dimos.evals.vqa.primitives.cross_view_identity import (
    IdentityPair,
    answer_cross_view_same_object,
    pair_case_id,
    propose_identity_pairs,
    stack_pair_image,
    views_from_localize,
)
from dimos.memory.cli.dataset import open_dataset
from dimos.perception.localize.dandetect import DanDetector
from dimos.perception.localize.localize import LocalizeTrace, embed_index
from dimos.perception.localize.rig import Rig, window_tf
from dimos.perception.localize.tools.tool_localize import GO2_PARAMS, PRESETS, parse_params
from dimos.perception.localize.types import Localization, LocalizeParams
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class CrossViewGenerationRequest(BaseModel):
    """Offline cross-view identity dataset request."""

    model_config = ConfigDict(extra="forbid", frozen=True)

    dataset: str
    queries: tuple[str, ...] = Field(min_length=1)
    output: Path | None = None
    start_s: float = Field(default=0.0, ge=0.0)
    duration_s: float | None = Field(default=None, gt=0.0)
    params_preset: str = "go2"
    # Stricter than go2 default for identity pool quality.
    min_views: int = Field(default=3, ge=1)

    def output_directory(self) -> Path:
        if self.output is not None:
            return self.output.expanduser()
        return STATE_DIR / "datasets" / "vqa" / f"{Path(self.dataset).stem}-cross-view"


class CrossViewGenerationResult(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)

    output: Path
    cases: tuple[PublicCase, ...]
    n_instances_considered: int
    n_pairs_proposed: int
    n_rejected: int


def _resolve_params(request: CrossViewGenerationRequest) -> LocalizeParams:
    base, fields = parse_params(request.params_preset, PRESETS.get("go2", GO2_PARAMS))
    params = replace(base, **fields) if fields else base
    return replace(params, min_views=request.min_views)


def _window_bounds(store: Any, start_s: float, duration_s: float | None) -> tuple[float, float]:
    rig = Rig.from_store(store)
    lo, hi = rig.time_range()
    after = lo + start_s
    before = lo + start_s + duration_s if duration_s is not None else hi
    if before <= after:
        raise ValueError("localize window before must be greater than after")
    return after, before


def collect_query_instances(
    query: str,
    hits: list[Localization],
    trace: LocalizeTrace,
) -> dict[str, tuple[Any, ...]]:
    """instance_id → SightingView tuple for one query."""
    if len(hits) != len(trace.answers):
        raise ValueError(
            f"localize trace length mismatch for {query!r}: "
            f"{len(hits)} hits vs {len(trace.answers)} answer groups"
        )
    by_id: dict[str, tuple[Any, ...]] = {}
    for hit, sightings in zip(hits, trace.answers, strict=True):
        views = views_from_localize(query, hit, list(sightings))
        if views:
            by_id[hit.instance_id] = views
    return by_id


def generate_cross_view_dataset(request: CrossViewGenerationRequest) -> CrossViewGenerationResult:
    """Localize with traces → filter → pair → write a standalone VQA dataset."""
    output = request.output_directory()
    if output.exists() and any(output.iterdir()):
        raise ValueError(f"output directory must be absent or empty: {output}")

    params = _resolve_params(request)
    store = open_dataset(request.dataset)
    store.start()
    try:
        after, before = _window_bounds(store, request.start_s, request.duration_s)
        rig = Rig.from_store(store)
        rig.tf = window_tf(store, after, before)
        if rig.lidar is not None:
            rig.lidar = rig.lidar.after(after).before(before)

        queries = [q.strip() for q in request.queries if q.strip()]
        if not queries:
            raise ValueError("at least one non-empty query is required")

        traces = [LocalizeTrace() for _ in queries]
        with DanDetector("siglip", "owlv2", "edgetam") as dan:
            index = embed_index(rig, dan.embedder, after, before)
            results = dan.localize(
                queries, index=index, rig=rig, params=params, traces=traces
            )

        all_pairs: list[IdentityPair] = []
        n_instances = 0
        for query, hits, trace in zip(queries, results, traces, strict=True):
            logger.info(f"cross-view {query!r}: {len(hits)} verified hit(s)")
            by_id = collect_query_instances(query, hits, trace)
            n_instances += len(by_id)
            pairs = propose_identity_pairs(by_id)
            logger.info(f"cross-view {query!r}: {len(pairs)} pair proposal(s)")
            all_pairs.extend(pairs)
    finally:
        store.stop()

    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix=f".{output.name}-", dir=output.parent) as temporary:
        staging = Path(temporary)
        assets = staging / "assets"
        assets.mkdir(parents=True, exist_ok=True)
        cases: list[PublicCase] = []
        labels: list[PrivateLabel] = []
        audit_rows: list[dict[str, object]] = []
        rejected = 0

        for index, pair in enumerate(all_pairs):
            try:
                answered = answer_cross_view_same_object(pair)
            except InsufficientEvidenceError as exc:
                rejected += 1
                audit_rows.append(
                    {
                        "status": "rejected",
                        "reason": str(exc),
                        "query": pair.query,
                        "same_object": pair.same_object,
                    }
                )
                continue

            case_id = pair_case_id(pair, index)
            image = stack_pair_image(pair.left.image, pair.right.image)
            rel = f"assets/{case_id}.jpg"
            image.save(str(staging / rel))
            cases.append(
                PublicCase(
                    id=case_id,
                    image=rel,
                    question=answered.question,
                    choices=answered.choices,
                )
            )
            labels.append(PrivateLabel(id=case_id, answer=answered.answer))
            audit_rows.append(
                {
                    "status": "answered",
                    "case_id": case_id,
                    "answer": answered.model_dump(),
                }
            )

        with jsonlines.open(staging / "cases.jsonl", mode="w") as writer:
            for case in cases:
                writer.write(case.model_dump())
        with jsonlines.open(staging / "labels.jsonl", mode="w") as writer:
            for label in labels:
                writer.write(label.model_dump())
        audit_dir = staging / "audit"
        audit_dir.mkdir(parents=True, exist_ok=True)
        (audit_dir / "run.json").write_text(
            json.dumps(
                {
                    "family": "cross_view_same_object",
                    "dataset": request.dataset,
                    "queries": list(request.queries),
                    "params_preset": request.params_preset,
                    "min_views": request.min_views,
                    "start_s": request.start_s,
                    "duration_s": request.duration_s,
                    "n_instances_considered": n_instances,
                    "n_pairs_proposed": len(all_pairs),
                    "n_cases": len(cases),
                    "n_rejected": rejected,
                },
                indent=2,
            )
            + "\n"
        )
        (audit_dir / "pairs.json").write_text(json.dumps(audit_rows, indent=2) + "\n")
        if output.exists():
            shutil.rmtree(output)
        shutil.move(str(staging), str(output))

    return CrossViewGenerationResult(
        output=output,
        cases=tuple(cases),
        n_instances_considered=n_instances,
        n_pairs_proposed=len(all_pairs),
        n_rejected=rejected,
    )
