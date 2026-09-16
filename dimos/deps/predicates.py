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

"""JSON-serializable predicates over global configuration values.

A predicate is a list whose first element is the operator::

    ["eq", "simulation", "mujoco"]
    ["ne", "viewer", "none"]
    ["in", "simulation", ["mujoco", "true"]]
    ["truthy", "local_relay"]
    ["not", <predicate>]
    ["any", <predicate>, ...]
    ["all", <predicate>, ...]

The catalog generator derives predicates from module-level ``if`` tests that
read ``global_config`` and the runtime reader evaluates them against the
preparsed configuration. Both sides share this module so the encoding is
never duplicated.
"""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any

Predicate = list[Any]


def evaluate(predicate: Predicate, config: Mapping[str, object]) -> bool | None:
    """Evaluate a predicate; ``None`` means "unknown" (missing field or operator).

    Callers treat ``None`` as true so an unknown condition yields a superset.
    """
    if not predicate:
        return None
    operator = predicate[0]
    if operator == "not":
        inner = evaluate(predicate[1], config)
        return None if inner is None else not inner
    if operator in ("any", "all"):
        results = [evaluate(inner, config) for inner in predicate[1:]]
        if operator == "any":
            if any(result is True for result in results):
                return True
            return False if all(result is False for result in results) else None
        if any(result is False for result in results):
            return False
        return True if all(result is True for result in results) else None
    if operator in ("eq", "ne", "in", "truthy"):
        field = predicate[1]
        if field not in config:
            return None
        value = config[field]
        if operator == "eq":
            return bool(value == predicate[2])
        if operator == "ne":
            return bool(value != predicate[2])
        if operator == "in":
            return bool(value in predicate[2])
        return bool(value)
    return None


def holds(predicate: Predicate | None, config: Mapping[str, object]) -> bool:
    """True unless the predicate evaluates to False (unknown counts as true)."""
    if predicate is None:
        return True
    return evaluate(predicate, config) is not False


def simplify(predicate: Predicate) -> Predicate:
    """Flatten nested any/all, drop double negations, collapse single-child lists."""
    operator = predicate[0]
    if operator == "not":
        inner = simplify(predicate[1])
        if inner[0] == "not":
            return list(inner[1])
        return ["not", inner]
    if operator in ("any", "all"):
        children: list[Predicate] = []
        for child in predicate[1:]:
            child = simplify(child)
            if child[0] == operator:
                children.extend(child[1:])
            else:
                children.append(child)
        if len(children) == 1:
            return children[0]
        return [operator, *children]
    return list(predicate)


def fields(predicate: Predicate) -> set[str]:
    """Configuration fields a predicate reads."""
    operator = predicate[0]
    if operator == "not":
        return fields(predicate[1])
    if operator in ("any", "all"):
        result: set[str] = set()
        for child in predicate[1:]:
            result |= fields(child)
        return result
    return {predicate[1]}
