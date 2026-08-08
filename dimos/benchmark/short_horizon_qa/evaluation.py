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

"""Frozen integer QA as one complete built-in Evaluation."""

from __future__ import annotations

import errno
from pathlib import Path
import re
import time

from openevals.exact import exact_match
from pydantic import BaseModel

from dimos.agents.code_policy_core import FrozenMemoryEnvironment
from dimos.benchmark.evaluation.models import (
    EvaluationReport,
    InlineNativeResult,
    SummaryItem,
)
from dimos.benchmark.evaluation.progress import (
    CaseHeaderProgress,
    StatusProgress,
    emit_progress,
)
from dimos.benchmark.evaluation.protocol import EvaluationContext
from dimos.benchmark.short_horizon_qa.integer_answer import (
    load_exact_integer_oracle,
    parse_integer_prediction,
)
from dimos.benchmark.short_horizon_qa.models import (
    FrozenIntegerQaCase,
    FrozenIntegerQaConfig,
    MapperSettings,
)
from dimos.benchmark.short_horizon_qa.prepare import prepare_bundle
from dimos.benchmark.short_horizon_qa.service import load_bundle
from dimos.constants import CACHE_DIR
from dimos.memory2.cli.dataset import resolve_dataset

EVALUATION_PROTOCOL = """Use `python_exec` to inspect the read-only `memory` object
for the frozen robot recording. Compute the requested integer from the recording;
do not guess. End with exactly one line in this form:

ANSWER: <integer>
"""


class FrozenIntegerQaEvaluation:
    name = "frozen-integer-qa"
    config_model: type[BaseModel] = FrozenIntegerQaConfig

    def run(self, config: BaseModel, context: EvaluationContext) -> EvaluationReport:
        if not isinstance(config, FrozenIntegerQaConfig):
            raise TypeError("frozen-integer-qa received the wrong configuration type")
        case_path = Path(config.case).expanduser()
        if not case_path.is_absolute():
            case_path = context.spec_dir / case_path
        case_path = case_path.resolve()
        case = FrozenIntegerQaCase.model_validate_json(case_path.read_bytes())
        oracle = load_exact_integer_oracle(case, case_path.parent)
        emit_progress(
            context.progress,
            CaseHeaderProgress(
                case_id=case.case_id,
                source=case.source.recording,
                progress=case.source.progress,
                question=case.task.prompt,
            ),
        )
        bundle = _materialize_frozen_memory(case, context)
        _, cutoff, source_path, derived_path = load_bundle(
            bundle,
            progress=case.source.progress,
        )
        emit_progress(
            context.progress,
            StatusProgress(channel="eval", message="memory ready"),
        )
        started = time.monotonic()
        with context.agent.open_session(
            FrozenMemoryEnvironment(
                recording_path=str(source_path),
                derived_recording_path=str(derived_path),
                memory_cutoff_timestamp=cutoff.cutoff_timestamp,
            )
        ) as session:
            outcome = session.run(
                evaluation_protocol=EVALUATION_PROTOCOL,
                task_input=case.task.prompt,
            )
        prediction = parse_integer_prediction(outcome.final_text)
        native_result = exact_match(
            outputs={
                "status": prediction.status,
                "integer_answer": prediction.integer_answer,
            },
            reference_outputs={
                "status": "parsed",
                "integer_answer": oracle.expected_count,
            },
        )
        return EvaluationReport(
            summary=(
                SummaryItem(key="case", label="Case", value=case.case_id),
                SummaryItem(
                    key="recording",
                    label="Recording",
                    value=f"{case.source.recording} @ {case.source.progress * 100:g}%",
                ),
                SummaryItem(
                    key="answer",
                    label="Answer",
                    value=prediction.integer_answer,
                ),
                SummaryItem(
                    key="exact_match",
                    label="Exact match",
                    value=bool(native_result["score"]),
                ),
                SummaryItem(
                    key="tool_calls",
                    label="Tool calls",
                    value=outcome.tool_call_count,
                ),
                SummaryItem(
                    key="duration",
                    label="Duration",
                    value=f"{time.monotonic() - started:.1f}s",
                ),
            ),
            native_result=InlineNativeResult(value=native_result),
        )


def _materialize_frozen_memory(
    case: FrozenIntegerQaCase,
    context: EvaluationContext,
) -> Path:
    source_path = resolve_dataset(case.source.recording).resolve()
    stat = source_path.stat()
    stem = re.sub(r"[^A-Za-z0-9_.-]+", "-", source_path.stem)[:64]
    mapper = MapperSettings()
    raw_key = (
        f"{stem}-{stat.st_size}-{stat.st_mtime_ns}-p{case.source.progress:.9f}-"
        f"v{mapper.voxel_size_m}-b{mapper.block_count}-d{mapper.device}-"
        f"c{int(mapper.carve_columns)}-f{mapper.frame_id}-e{mapper.emit_every}"
    )
    key = re.sub(r"[^A-Za-z0-9_.-]+", "-", raw_key)
    bundle = CACHE_DIR / "evaluation" / "frozen_memory" / key
    manifest = bundle / "manifest.v1.json"
    if not manifest.is_file():
        emit_progress(
            context.progress,
            StatusProgress(channel="eval", message="preparing memory"),
        )
        bundle.parent.mkdir(parents=True, exist_ok=True)
        try:
            prepare_bundle(
                case.source.recording,
                [],
                bundle,
                progress=[case.source.progress],
                mapper=mapper,
                map_progress=lambda current, total: emit_progress(
                    context.progress,
                    StatusProgress(
                        channel="eval",
                        message=f"mapping {current}/{total} frames",
                    ),
                ),
            )
        except OSError as exc:
            concurrent_publish = isinstance(exc, FileExistsError) or exc.errno in {
                errno.EEXIST,
                errno.ENOTEMPTY,
            }
            if not concurrent_publish or not manifest.is_file():
                raise
    return bundle


frozen_integer_qa = FrozenIntegerQaEvaluation()
