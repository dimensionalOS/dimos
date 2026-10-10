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
import types
import typing
from typing import Any

import yaml

SOURCE_FILE = Path(__file__).parent / "annotations.yaml"


def load(path: Path = SOURCE_FILE) -> dict[str, Any]:
    source: dict[str, Any] = yaml.safe_load(path.read_text())
    return source


def blueprint_file(ref: str) -> str:
    return ref.split(":")[0].replace(".", "/") + ".py"


def _inside(path: str, directory: str) -> bool:
    return path == directory or path.startswith(directory.rstrip("/") + "/")


def owner_of(doc: dict[str, Any], path: str) -> str | None:
    best: tuple[int, str] | None = None
    for robot_id, robot in doc["robots"].items():
        for directory in robot.get("dirs", []):
            if _inside(path, directory) and (best is None or len(directory) > best[0]):
                best = (len(directory), robot_id)
    return best[1] if best else None


def robot_of(doc: dict[str, Any], name: str, ref: str | None) -> str | None:
    for robot_id, robot in doc["robots"].items():
        if name in robot.get("blueprints", {}):
            return str(robot_id)
    return owner_of(doc, blueprint_file(ref)) if ref else None


def setting_key(spec: dict[str, Any]) -> str | None:
    if "global" in spec:
        return str(spec["global"])
    if "module" in spec:
        return f"{spec['module']}.{spec['field']}"
    return None


def settings_keys(settings: list[dict[str, Any]]) -> set[str]:
    keys: set[str] = set()
    for setting in settings:
        key = setting_key(setting)
        if key is not None:
            keys.add(key)
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


def _generated_setting(
    entry: dict[str, Any], inherited: dict[str, Any] | None, keys: set[str]
) -> dict[str, Any]:
    from dimos.core.global_config import GlobalConfig

    key = setting_key(entry)
    if key is None:
        return copy.deepcopy(entry)
    inherited = dict(inherited or {})
    if "when" in inherited and not set(inherited["when"]) <= keys:
        del inherited["when"]
    base = reflected(entry["global"]) if entry.get("global") in GlobalConfig.model_fields else {}
    merged = {**base, **copy.deepcopy(inherited), **copy.deepcopy(entry)}
    if "label" not in merged:
        words = key.rsplit(".", 1)[-1].replace("_", " ")
        merged["label"] = words[:1].upper() + words[1:]
    ordered = {k: merged.pop(k) for k in ("global", "module", "field", "label") if k in merged}
    return {**ordered, **merged}


def generate(source: dict[str, Any]) -> dict[str, Any]:
    out = copy.deepcopy(source)
    for robot in out["robots"].values():
        defaults = robot.get("defaults", {}).get("recommended_config", [])
        by_key = {setting_key(entry): entry for entry in defaults if setting_key(entry)}
        if defaults:
            keys = settings_keys(defaults)
            robot["defaults"]["recommended_config"] = [
                _generated_setting(e, None, keys) for e in defaults
            ]
        for blueprint in robot["blueprints"].values():
            if "recommended_config" in blueprint:
                own = blueprint["recommended_config"]
                keys = settings_keys(own)
                blueprint["recommended_config"] = [
                    _generated_setting(e, by_key.get(setting_key(e) or ""), keys) for e in own
                ]
    return out


def _resolved_setting(index: int, spec: dict[str, Any]) -> dict[str, Any]:
    key = setting_key(spec)
    if key is None:
        options = copy.deepcopy(spec["choices"])
        return {
            "id": f"pick-{index}",
            "key": None,
            "scope": None,
            "kind": "pick",
            "label": spec["label"],
            "required": False,
            "docs": spec.get("docs"),
            "default": spec.get("default", options[0]["label"]),
            "choices": options,
            "when": copy.deepcopy(spec.get("when")),
        }
    return {
        "id": key,
        "key": key,
        "scope": "global" if "global" in spec else "module",
        "kind": "text",
        "required": False,
        "docs": None,
        **copy.deepcopy(spec),
        "choices": copy.deepcopy(spec.get("choices")),
        "when": copy.deepcopy(spec.get("when")),
    }


def resolved(source: dict[str, Any], registry: Collection[str]) -> dict[str, Any]:
    doc = generate(source)
    out = copy.deepcopy(doc)
    out["about"] = None
    listed = set()
    for robot_id, robot in out["robots"].items():
        original = doc["robots"][robot_id]
        for name, blueprint in robot["blueprints"].items():
            listed.add(name)
            own = original["blueprints"][name]
            settings = own.get("recommended_config")
            if settings is None:
                settings = original.get("defaults", {}).get("recommended_config", [])
            blueprint["robot"] = robot_id
            blueprint["recommended_config"] = [
                _resolved_setting(index, setting) for index, setting in enumerate(settings)
            ]
            blueprint.setdefault("starter", None)
            blueprint.setdefault("hidden", False)
            blueprint["registered"] = name in registry
        robot.pop("defaults", None)
        robot.setdefault("recommended", [])
    out["unlisted"] = sorted(set(registry) - listed)
    return out
