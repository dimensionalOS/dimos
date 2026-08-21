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

"""Lazy built-in Evaluation discovery."""

from __future__ import annotations

from dataclasses import dataclass
import importlib
import importlib.metadata as importlib_metadata
from typing import Any

from pydantic import BaseModel

from dimos.benchmark.evaluation.protocol import Evaluation

BUILTIN_EVALUATIONS: dict[str, str] = {
    "libero-pro": "dimos.benchmark.libero_pro.evaluation:libero_pro",
    "vlnce-r2r": "dimos.benchmark.vlnce_r2r.evaluation:vlnce_r2r",
}


class EvaluationRegistryError(ValueError):
    """A built-in Evaluation name or target could not be resolved."""


@dataclass(frozen=True)
class ResolvedEvaluation:
    name: str
    provider: str
    version: str
    evaluation: Evaluation


def available_evaluations() -> list[str]:
    return sorted(BUILTIN_EVALUATIONS)


def resolve_evaluation(name: str) -> ResolvedEvaluation:
    path = BUILTIN_EVALUATIONS.get(name)
    if path is None:
        available = ", ".join(available_evaluations())
        raise EvaluationRegistryError(
            f"Unknown evaluation {name!r}. Available evaluations: {available}."
        )
    target = _load_target(path, name)
    return ResolvedEvaluation(
        name=name,
        provider="dimos",
        version=_distribution_version("dimos"),
        evaluation=_validate_target(name, target),
    )


def _load_target(path: str, name: str) -> Any:
    module_name, separator, attribute = path.partition(":")
    if not separator:
        raise EvaluationRegistryError(f"Invalid built-in evaluation target for {name!r}: {path}")
    try:
        return getattr(importlib.import_module(module_name), attribute)
    except Exception as exc:
        raise EvaluationRegistryError(
            f"Failed to load evaluation {name!r} from {path!r}: {type(exc).__name__}: {exc}"
        ) from exc


def _validate_target(name: str, target: Any) -> Evaluation:
    if not isinstance(target, Evaluation):
        raise EvaluationRegistryError(
            f"Evaluation {name!r} must expose name, runtime_profile, config_model, and run()"
        )
    config_model = target.config_model
    if not isinstance(config_model, type) or not issubclass(config_model, BaseModel):
        raise EvaluationRegistryError(
            f"Evaluation {name!r} config_model must be a Pydantic BaseModel type"
        )
    if target.name != name.rpartition(".")[2]:
        raise EvaluationRegistryError(f"Evaluation {name!r} loaded a target named {target.name!r}")
    return target


def _distribution_version(name: str) -> str:
    try:
        return importlib_metadata.version(name)
    except importlib_metadata.PackageNotFoundError:
        return "source"
