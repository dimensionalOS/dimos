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

from dataclasses import dataclass, field
import inspect
import json
from typing import Any

from pydantic import ValidationError

ModuleValues = dict[str, dict[str, Any]]

HIDDEN = "•••"
SECRET_WORDS = ("secret", "token", "password", "passwd")
STRUCTURED = ("global", "modules", "secrets")
LAUNCH_GLOBAL_DEFAULTS: dict[str, Any] = {"rerun_open": "none", "rerun_web": True}


@dataclass
class LaunchOverrides:
    global_: dict[str, Any] = field(default_factory=dict)
    modules: ModuleValues = field(default_factory=dict)
    secrets: list[str] = field(default_factory=list)

    def to_json(self) -> dict[str, Any]:
        value: dict[str, Any] = {"global": self.global_, "modules": self.modules}
        if self.secrets:
            value["secrets"] = self.secrets
        return value

    @staticmethod
    def from_json(value: Any) -> LaunchOverrides:
        value = value if isinstance(value, dict) else {}
        return LaunchOverrides(
            dict(value.get("global") or {}),
            {m: dict(f) for m, f in (value.get("modules") or {}).items()},
            [p for p in value.get("secrets") or [] if isinstance(p, str)],
        )


def parse(raw: Any) -> LaunchOverrides:
    if raw is None:
        return LaunchOverrides()
    if not isinstance(raw, dict):
        raise ValueError("overrides must be an object")
    if not any(key in raw for key in STRUCTURED):
        return LaunchOverrides(dict(raw))
    other = next((key for key in raw if key not in STRUCTURED), None)
    if other is not None:
        raise ValueError(f"overrides has `{other}` next to `global`/`modules`")
    global_, modules, secrets = (
        raw.get("global") or {},
        raw.get("modules") or {},
        raw.get("secrets") or [],
    )
    if not isinstance(global_, dict):
        raise ValueError("overrides.global must be an object")
    if not isinstance(modules, dict) or not all(isinstance(f, dict) for f in modules.values()):
        raise ValueError("overrides.modules must be an object of objects")
    if not isinstance(secrets, list) or not all(isinstance(p, str) for p in secrets):
        raise ValueError("overrides.secrets must be a list of paths")
    return LaunchOverrides(dict(global_), {m: dict(f) for m, f in modules.items()}, secrets)


def is_secret_name(path: str) -> bool:
    name = path.rsplit(".", 1)[-1].lower()
    return name == "key" or name.endswith("_key") or any(word in name for word in SECRET_WORDS)


def secret_paths(
    global_: dict[str, Any], modules: ModuleValues, listed: list[str] | tuple[str, ...] = ()
) -> list[str]:
    paths = [key for key in sorted(global_) if is_secret_name(key) or key in listed]
    for module, fields in sorted(modules.items()):
        paths += [
            f"{module}.{name}"
            for name in sorted(fields)
            if is_secret_name(name) or f"{module}.{name}" in listed
        ]
    return paths


def as_text(value: Any) -> str:
    return value if isinstance(value, str) else json.dumps(value, separators=(",", ":"))


def secret_env(global_: dict[str, Any], modules: ModuleValues, paths: list[str]) -> dict[str, str]:
    env = {key.upper(): as_text(v) for key, v in global_.items() if key in paths and v is not None}
    for module, fields in modules.items():
        for name, value in fields.items():
            if f"{module}.{name}" in paths and value is not None:
                env[f"{module.replace('/', '_')}__{name}".upper()] = as_text(value)
    return env


def without_secrets(
    global_: dict[str, Any], modules: ModuleValues, paths: list[str]
) -> tuple[dict[str, Any], ModuleValues]:
    flat = {k: v for k, v in global_.items() if k not in paths}
    nested = {
        m: {n: v for n, v in f.items() if f"{m}.{n}" not in paths} for m, f in modules.items()
    }
    return flat, {m: f for m, f in nested.items() if f}


def redact(
    global_: dict[str, Any], modules: ModuleValues, paths: list[str]
) -> tuple[dict[str, Any], ModuleValues]:
    flat = {k: HIDDEN if k in paths and v is not None else v for k, v in global_.items()}
    nested = {
        m: {n: HIDDEN if f"{m}.{n}" in paths and v is not None else v for n, v in f.items()}
        for m, f in modules.items()
    }
    return flat, nested


def keep_hidden(sent: dict[str, Any], saved: dict[str, Any]) -> dict[str, Any]:
    kept = {}
    for key, value in sent.items():
        if value == HIDDEN and is_secret_name(key):
            if saved.get(key) is not None:
                kept[key] = saved[key]
        else:
            kept[key] = value
    return kept


def without_hidden(values: dict[str, Any]) -> dict[str, Any]:
    return {k: v for k, v in values.items() if v != HIDDEN}


def merge(base: dict[str, Any], over: dict[str, Any]) -> dict[str, Any]:
    merged = dict(base)
    for key, value in over.items():
        if value is None:
            merged.pop(key, None)
        else:
            merged[key] = value
    return dict(sorted(merged.items()))


def merge_modules(base: ModuleValues, over: ModuleValues) -> ModuleValues:
    merged = {module: dict(fields) for module, fields in base.items()}
    for module, fields in over.items():
        entry = merge(merged.get(module, {}), fields)
        if entry:
            merged[module] = entry
        else:
            merged.pop(module, None)
    return dict(sorted(merged.items()))


def flag_fields() -> set[str]:
    from dimos.cli.commands.global_options import create_dynamic_callback

    callback = create_dynamic_callback()  # type: ignore[no-untyped-call]
    return set(inspect.signature(callback).parameters) - {"ctx"}


def check_global(values: dict[str, Any]) -> None:
    from dimos.core.global_config import GlobalConfig

    unknown = sorted(set(values) - flag_fields())
    if unknown:
        raise ValueError(f"not a dimos GlobalConfig flag: {', '.join(unknown)}")
    try:
        GlobalConfig.model_validate({k: v for k, v in values.items() if v is not None})
    except ValidationError as error:
        problems = "; ".join(f"{'.'.join(map(str, e['loc']))}: {e['msg']}" for e in error.errors())
        raise ValueError(f"bad GlobalConfig value: {problems}") from error


def global_flags(values: dict[str, Any], explicit_bools: bool = False) -> list[str]:
    flags: list[str] = []
    for key, value in sorted(values.items()):
        flag = "--" + key.replace("_", "-")
        if value is True:
            flags.append(f"{flag}=true" if explicit_bools else flag)
        elif value is False:
            flags.append("--no-" + key.replace("_", "-"))
        elif value is not None:
            flags.append(f"{flag}={as_text(value)}")
    return flags


def module_flags(modules: ModuleValues) -> list[str]:
    flags = []
    for module, fields in sorted(modules.items()):
        root = module.replace("/", "_").replace("_", "-")
        for name, value in sorted(fields.items()):
            if value is not None:
                flags.append(f"--{root}.{name.replace('_', '-')}={as_text(value)}")
    return flags


def shown_args(args: list[str]) -> list[str]:
    shown: list[str] = []
    hide_next = False
    for arg in args:
        if hide_next and not arg.startswith("-"):
            shown.append(HIDDEN)
            hide_next = False
            continue
        option, separator, _ = arg.partition("=")
        secret = arg.startswith("--") and is_secret_name(option[2:].replace("-", "_"))
        shown.append(f"{option}={HIDDEN}" if secret and separator else arg)
        hide_next = secret and not separator
    return shown
