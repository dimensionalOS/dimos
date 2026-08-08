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

"""Lazy built-in and installed-package Evaluation discovery."""

from __future__ import annotations

from dataclasses import dataclass
import importlib
import importlib.metadata as importlib_metadata
import re
from typing import Any

from packaging.utils import canonicalize_name
from pydantic import BaseModel

from dimos.benchmark.evaluation.protocol import Evaluation

ENTRY_POINT_GROUP = "dimos.evaluations"
LOCAL_NAME_PATTERN = re.compile(r"^[a-z0-9]+(?:-[a-z0-9]+)*$")
BUILTIN_EVALUATIONS = {
    "frozen-integer-qa": ("dimos.benchmark.short_horizon_qa.evaluation:frozen_integer_qa"),
}


class EvaluationRegistryError(ValueError):
    """An Evaluation name or plugin could not be resolved."""


@dataclass(frozen=True)
class ResolvedEvaluation:
    name: str
    provider: str
    version: str
    evaluation: Evaluation


def available_evaluations() -> list[str]:
    return sorted([*BUILTIN_EVALUATIONS, *_external_entries()])


def resolve_evaluation(name: str) -> ResolvedEvaluation:
    if name in BUILTIN_EVALUATIONS:
        target = _load_target(BUILTIN_EVALUATIONS[name], name)
        return ResolvedEvaluation(
            name=name,
            provider="dimos",
            version=_distribution_version("dimos"),
            evaluation=_validate_target(name, target),
        )

    entries = _external_entries()
    entry = entries.get(name)
    if entry is None:
        available = available_evaluations()
        suffix = f" Available evaluations: {', '.join(available)}." if available else ""
        raise EvaluationRegistryError(f"Unknown evaluation {name!r}.{suffix}")
    try:
        target = entry.load()
    except Exception as exc:
        raise EvaluationRegistryError(
            f"Failed to load evaluation {name!r} from {entry.value!r}: {type(exc).__name__}: {exc}"
        ) from exc
    distribution = entry.dist
    assert distribution is not None
    distribution_name = distribution.metadata["Name"]
    return ResolvedEvaluation(
        name=name,
        provider=distribution_name,
        version=distribution.version,
        evaluation=_validate_target(name, target),
    )


def _external_entries() -> dict[str, importlib_metadata.EntryPoint]:
    result: dict[str, importlib_metadata.EntryPoint] = {}
    for entry in importlib_metadata.entry_points(group=ENTRY_POINT_GROUP):
        distribution = entry.dist
        if distribution is None:
            continue
        distribution_name = distribution.metadata.get("Name")
        if not distribution_name or LOCAL_NAME_PATTERN.fullmatch(entry.name) is None:
            continue
        namespace = str(canonicalize_name(distribution_name))
        qualified_name = f"{namespace}.{entry.name}"
        if qualified_name in result:
            raise EvaluationRegistryError(
                f"Multiple installed entry points provide evaluation {qualified_name!r}"
            )
        result[qualified_name] = entry
    return result


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
            f"Evaluation {name!r} must expose name, config_model, and run()"
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
