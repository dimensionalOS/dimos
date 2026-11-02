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

from __future__ import annotations

from collections.abc import Collection
import copy
import enum
import json
from pathlib import Path
import re
import types
import typing
from typing import Any

import yaml

SOURCE_FILE = Path(__file__).parent / "annotations.yaml"
# a blueprint with no `hidden:` of its own is hidden when its name says it's a coordinator, test or benchmark
HIDDEN_NAME = re.compile(r"^coordinator-|test|benchmark")


def load(path: Path = SOURCE_FILE) -> dict[str, Any]:
    """The `robots:` section: its `types` and `robots` (an older checkout has them at the top)."""
    source: dict[str, Any] = yaml.safe_load(path.read_text())
    section = source.get("robots") or {}
    return section if "types" in section else source


def owner_of(doc: dict[str, Any], path: str) -> str | None:
    owners = [
        (len(directory), str(robot_id))
        for robot_id, robot in doc["robots"].items()
        for directory in robot.get("dirs", [])
        if path == directory or path.startswith(directory.rstrip("/") + "/")
    ]
    return max(owners, key=lambda owner: owner[0])[1] if owners else None


def robot_of(doc: dict[str, Any], name: str, ref: str | None) -> str | None:
    for robot_id, robot in doc["robots"].items():
        if name in robot.get("blueprints", {}):
            return str(robot_id)
    return owner_of(doc, ref.split(":")[0].replace(".", "/") + ".py") if ref else None


def setting_key(spec: dict[str, Any]) -> str | None:
    if "global" in spec:
        return str(spec["global"])
    if "module" in spec:
        return f"{spec['module']}.{spec['field']}"
    return None


def settings_keys(settings: list[dict[str, Any]]) -> set[str]:
    keys = {key for setting in settings if (key := setting_key(setting)) is not None}
    for setting in settings:
        for choice in setting.get("choices", []):
            keys.update(choice.get("set", {}))
    return keys


def _jsonable(value: Any) -> Any:
    if isinstance(value, enum.Enum):
        value = value.value
    if isinstance(value, Path):
        return str(value)
    try:
        json.dumps(value)
    except TypeError:
        return None
    return value


def reflected(field_name: str) -> dict[str, Any]:
    from dimos.core.global_config import GlobalConfig

    field = GlobalConfig.model_fields[field_name]
    annotation = field.annotation
    union = typing.get_origin(annotation) in (typing.Union, types.UnionType)
    parts = list(typing.get_args(annotation)) if union else [annotation]
    main = [part for part in parts if part is not type(None)]
    one = main[0] if len(main) == 1 else None
    options: list[Any] | None = None
    if one is not None and typing.get_origin(one) is typing.Literal:
        options = list(typing.get_args(one))
        one = type(options[0]) if options else str
    elif isinstance(one, type) and issubclass(one, enum.Enum):
        options = [member.value for member in one]
        one = type(options[0]) if options else str
    kinds = {bool: "boolean", int: "integer", float: "number", str: "string", Path: "string"}
    kind = kinds.get(one, "json") if isinstance(one, type) else "json"
    out: dict[str, Any] = {
        "type": kind,
        "nullable": type(None) in parts,
        "default": _jsonable(field.default) if not field.is_required() else None,
        "kind": {"boolean": "bool", "integer": "number", "number": "number"}.get(kind, "text"),
    }
    if options is not None:
        out["choices"] = [{"value": value, "label": str(value)} for value in options]
    if field.description:
        out["description"] = field.description
    return out


def _setting(
    index: int, entry: dict[str, Any], inherited: dict[str, Any] | None, keys: set[str]
) -> dict[str, Any]:
    from dimos.core.global_config import GlobalConfig

    key = setting_key(entry)
    if key is None:
        return {
            "id": f"pick-{index}",
            "key": None,
            "scope": None,
            "kind": "pick",
            "label": entry["label"],
            "required": False,
            "docs": entry.get("docs"),
            "default": entry.get("default", entry["choices"][0]["label"]),
            "choices": entry["choices"],
            "when": entry.get("when"),
        }
    inherited = dict(inherited or {})
    if "when" in inherited and not set(inherited["when"]) <= keys:
        del inherited["when"]
    base = reflected(entry["global"]) if entry.get("global") in GlobalConfig.model_fields else {}
    merged = {**base, **inherited, **entry}
    if "label" not in merged:
        words = key.rsplit(".", 1)[-1].replace("_", " ")
        merged["label"] = words[:1].upper() + words[1:]
    ordered: dict[str, Any] = {
        ordered_key: merged.pop(ordered_key)
        for ordered_key in ("global", "module", "field", "label")
        if ordered_key in merged
    }
    return {
        "id": key,
        "key": key,
        "scope": "global" if "global" in ordered else "module",
        "kind": "text",
        "required": False,
        "docs": None,
        **ordered,
        **merged,
        "choices": merged.get("choices"),
        "when": merged.get("when"),
    }


def resolved(source: dict[str, Any], registry: Collection[str]) -> dict[str, Any]:
    out = copy.deepcopy(source)
    out["about"] = None
    for robot_id, robot in out["robots"].items():
        defaults = robot.pop("defaults", {}).get("recommended_config", [])
        by_key = {setting_key(entry): entry for entry in defaults if setting_key(entry)}
        for name, blueprint in robot["blueprints"].items():
            own = blueprint.get("recommended_config")
            settings = defaults if own is None else own
            keys = settings_keys(settings)
            blueprint["robot"] = robot_id
            blueprint["recommended_config"] = [
                _setting(
                    index,
                    setting,
                    None if own is None else by_key.get(setting_key(setting) or ""),
                    keys,
                )
                for index, setting in enumerate(settings)
            ]
            blueprint.setdefault("starter", None)
            blueprint.setdefault("hidden", HIDDEN_NAME.search(name) is not None)
            blueprint["registered"] = name in registry
        robot.setdefault("recommended", [])
    return out
