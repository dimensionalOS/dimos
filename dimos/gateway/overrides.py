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

"""Launch overrides: what `POST /dimos/runs` may set on top of Desktop's saved config, checked against dimos's own
schemas (GlobalConfig's JSON Schema, each module field's) and turned into `dimos run` flags: GlobalConfig keys before
`run`, module fields after it as `--<module>.<field>=<value>` (dimos's BlueprintConfigParser reads those).

The same rules and messages as Desktop's Rust (src/dimos/overrides.rs), so a client sees the same 400 from either.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import json
from typing import Any

# {module: {field: value}}
ModuleValues = dict[str, dict[str, Any]]

# what a secret's value is shown as, everywhere; sent back, it means "the saved value"
HIDDEN = "•••"
SECRET_WORDS = ("secret", "token", "password", "passwd")


@dataclass
class LaunchOverrides:
    """A launch's own config. None (null) drops Desktop's saved value for that key (dimos's default applies)."""

    global_: dict[str, Any] = field(default_factory=dict)
    modules: ModuleValues = field(default_factory=dict)
    # paths (`robot_ip`, `<module>.<field>`) to treat as secrets besides the ones named like one
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


def text(value: Any) -> str:
    """`value` as compact JSON, as serde_json writes it."""
    return json.dumps(value, separators=(",", ":"), ensure_ascii=False)


def kind_of(value: Any) -> str:
    if value is None:
        return "null"
    if isinstance(value, bool):
        return "boolean"
    if isinstance(value, int | float):
        return "number"
    if isinstance(value, str):
        return "string"
    if isinstance(value, list):
        return "list"
    return "object"


def parse(raw: Any) -> LaunchOverrides:
    """The request's `overrides`: `{global?: {...}, modules?: {...}}`, or (the older form) a flat `{key: value}` of
    GlobalConfig keys. Mixing the two, or anything that isn't an object, is a ValueError."""
    if raw is None:
        return LaunchOverrides()
    if not isinstance(raw, dict):
        raise ValueError(f"overrides must be an object, not {kind_of(raw)}")
    if not any(key in raw for key in STRUCTURED):
        return LaunchOverrides(dict(raw), {})
    other = next((key for key in raw if key not in STRUCTURED), None)
    if other is not None:
        raise ValueError(
            f"overrides has `{other}` next to `global`/`modules`: put GlobalConfig keys under overrides.global"
        )
    secrets = raw.get("secrets")
    if secrets is not None and (
        not isinstance(secrets, list) or not all(isinstance(path, str) for path in secrets)
    ):
        raise ValueError(
            "overrides.secrets must be a list of paths (`robot_ip`, `<module>.<field>`), not "
            + kind_of(secrets)
        )
    global_ = raw.get("global")
    if global_ is not None and not isinstance(global_, dict):
        raise ValueError(f"overrides.global must be an object, not {kind_of(global_)}")
    modules: ModuleValues = {}
    given = raw.get("modules")
    if given is not None and not isinstance(given, dict):
        raise ValueError(f"overrides.modules must be an object, not {kind_of(given)}")
    for module, fields in sorted((given or {}).items()):
        if not isinstance(fields, dict):
            raise ValueError(
                f"overrides.modules.{module} must be an object of fields, not {kind_of(fields)}"
            )
        modules[module] = dict(fields)
    return LaunchOverrides(dict(global_ or {}), modules, list(secrets or []))


STRUCTURED = ("global", "modules", "secrets")


# secrets: by name (dimos has no secret marker), or listed by the request


def is_secret_name(path: str) -> bool:
    """`key`, `*_key`, or a name with secret/token/password/passwd in it (the path's last part, any case)."""
    name = path.rsplit(".", 1)[-1].lower()
    return name == "key" or name.endswith("_key") or any(word in name for word in SECRET_WORDS)


def secret_paths(
    global_: dict[str, Any], modules: ModuleValues, listed: list[str] | tuple[str, ...] = ()
) -> list[str]:
    """The secret paths among these values: GlobalConfig keys, then `<module>.<field>`."""
    paths = [key for key in sorted(global_) if is_secret_name(key) or key in listed]
    for module, fields in sorted(modules.items()):
        paths += [
            f"{module}.{name}"
            for name in sorted(fields)
            if is_secret_name(name) or f"{module}.{name}" in listed
        ]
    return paths


def secret_env(global_: dict[str, Any], modules: ModuleValues, paths: list[str]) -> dict[str, str]:
    """Secret values as the environment dimos reads them from (not flags, so never in argv): a GlobalConfig key as
    `KEY` (GlobalConfig is a BaseSettings), a module field as `<MODULE>__<FIELD>` (merging.merge_environment). Global
    first, then modules; strings as they are, else JSON."""
    env: dict[str, str] = {}
    for key in sorted(global_):
        if key in paths and global_[key] is not None:
            env[key.upper()] = as_text(global_[key])
    for module, fields in sorted(modules.items()):
        for name in sorted(fields):
            if f"{module}.{name}" in paths and fields[name] is not None:
                env[f"{module.replace('/', '_')}__{name}".upper()] = as_text(fields[name])
    return env


def as_text(value: Any) -> str:
    return value if isinstance(value, str) else text(value)


def without(
    global_: dict[str, Any], modules: ModuleValues, paths: list[str]
) -> tuple[dict[str, Any], ModuleValues]:
    """These values without the secret ones (what goes on the command line)."""
    flat = {key: value for key, value in global_.items() if key not in paths}
    nested = {
        module: {name: value for name, value in fields.items() if f"{module}.{name}" not in paths}
        for module, fields in modules.items()
    }
    return flat, {module: fields for module, fields in nested.items() if fields}


def redact(
    global_: dict[str, Any], modules: ModuleValues, paths: list[str]
) -> tuple[dict[str, Any], ModuleValues]:
    """These values with every non-null secret shown as HIDDEN."""
    flat = {k: HIDDEN if k in paths and v is not None else v for k, v in global_.items()}
    nested = {
        module: {
            name: HIDDEN if f"{module}.{name}" in paths and value is not None else value
            for name, value in fields.items()
        }
        for module, fields in modules.items()
    }
    return flat, nested


def hide_value(message: str, secret: bool) -> str:
    """A validation message for a secret path, without its value."""
    if not secret:
        return message
    return message.split(", got ", 1)[0] + " (value hidden: it's secret)"


def keep_hidden(sent: dict[str, Any], saved: dict[str, Any], secret: Any) -> dict[str, Any]:
    """`sent` with each HIDDEN secret put back to its saved value (dropped when none is saved)."""
    kept = {}
    for key, value in sent.items():
        if value == HIDDEN and secret(key):
            if saved.get(key) is not None:
                kept[key] = saved[key]
        else:
            kept[key] = value
    return kept


def validate_global(
    values: dict[str, Any],
    schema: dict[str, Any],
    at: str,
    listed: list[str] | tuple[str, ...] = (),
) -> None:
    """Checks GlobalConfig values against its JSON Schema. Lists and objects aren't `dimos` flags (its CLI skips
    them), so they're refused too. None always passes (back to dimos's default). ValueError: the first problem."""
    properties = schema.get("properties") or {}
    for key, value in sorted(values.items()):
        if key not in properties:
            raise ValueError(unknown(f"{at}.{key}", "GlobalConfig field", key, list(properties)))
        if value is None:
            continue
        if main_type(resolve(properties[key], schema), schema) in ("array", "object"):
            raise ValueError(
                f"{at}.{key}: a list or object can't be passed as a `dimos` flag; set it in dimos's config file"
            )
        why = check(value, properties[key], schema)
        if why:
            secret = is_secret_name(key) or key in listed
            raise ValueError(hide_value(f"{at}.{key}: {why}", secret))


def validate_modules(
    values: ModuleValues, config: dict[str, Any], at: str, listed: list[str] | tuple[str, ...] = ()
) -> None:
    """Checks module values against a blueprint's config (`GET /dimos/blueprints/{name}/config`: `modules: [{module,
    args: [{name, schema?, choices?, json_compatible?}]}]`). None always passes (it drops a saved value)."""
    blueprint = config.get("name") or "the blueprint"
    modules = config.get("modules") or []
    names = [entry.get("module") for entry in modules if isinstance(entry.get("module"), str)]
    for module, fields in sorted(values.items()):
        entry = next((m for m in modules if m.get("module") == module), None)
        if entry is None:
            raise ValueError(unknown(f"{at}.{module}", f"module in {blueprint}", module, names))
        args = entry.get("args") or []
        settable = [a["name"] for a in args if a.get("json_compatible") is not False]
        for name, value in sorted(fields.items()):
            path = f"{at}.{module}.{name}"
            arg = next((a for a in args if a.get("name") == name), None)
            if arg is None:
                raise ValueError(unknown(path, f"field of {module}", name, settable))
            if arg.get("json_compatible") is False:
                raise ValueError(
                    f"{path}: its type ({arg.get('type') or '?'}) can't be written as JSON, so it can't be set "
                    "from a launch"
                )
            if value is None:
                continue
            secret = is_secret_name(name) or f"{module}.{name}" in listed
            if isinstance(arg.get("schema"), dict):
                why = check(value, arg["schema"], arg["schema"])
                if why:
                    raise ValueError(hide_value(f"{path}: {why}", secret))
            elif isinstance(arg.get("choices"), list):
                if not any(same(value, option) for option in arg["choices"]):
                    raise ValueError(
                        hide_value(
                            f"{path}: needs one of {options(arg['choices'])}, got {text(value)}",
                            secret,
                        )
                    )


def module_flags(modules: ModuleValues) -> list[str]:
    """`dimos run <bp>` flags for module values: `--<module>.<field>=<value>`, sorted. The module part is dimos's
    config key (`/` -> `_`), both parts kebab-cased as dimos's parser spells them; strings go as they are, the rest as
    JSON (dimos parses `[`/`{` as JSON and lets pydantic coerce the rest). Nones are left out."""
    flags = []
    for module, fields in sorted(modules.items()):
        root = module.replace("/", "_").replace("_", "-")
        for name, value in sorted(fields.items()):
            if value is None:
                continue
            flags.append(
                f"--{root}.{name.replace('_', '-')}={value if isinstance(value, str) else text(value)}"
            )
    return flags


def merge(base: dict[str, Any], over: dict[str, Any]) -> dict[str, Any]:
    """`base` with `over` on top; a None in `over` removes the key."""
    merged = dict(base)
    for key, value in over.items():
        if value is None:
            merged.pop(key, None)
        else:
            merged[key] = value
    return dict(sorted(merged.items()))


def merge_modules(base: ModuleValues, over: ModuleValues) -> ModuleValues:
    """merge() per module; a module left with no fields is dropped."""
    merged = {module: dict(fields) for module, fields in base.items()}
    for module, fields in over.items():
        entry = merge(merged.get(module, {}), fields)
        if entry:
            merged[module] = entry
        else:
            merged.pop(module, None)
    return dict(sorted(merged.items()))


# a small JSON Schema check: type, enum/const, anyOf/oneOf, $ref, minimum/maximum, items, additionalProperties


def resolve(schema: dict[str, Any], root: dict[str, Any]) -> dict[str, Any]:
    reference = schema.get("$ref")
    if isinstance(reference, str):
        defs = root.get("$defs") or root.get("definitions") or {}
        target = defs.get(reference.rsplit("/", 1)[-1])
        if isinstance(target, dict):
            return resolve(target, root)
    return schema


def main_type(schema: dict[str, Any], root: dict[str, Any]) -> str | None:
    variants = schema.get("anyOf") or schema.get("oneOf")
    if isinstance(variants, list):
        types = [main_type(resolve(v, root), root) for v in variants]
        found = [t for t in types if t is not None and t != "null"]
        return found[0] if len(found) == 1 else None
    kind = schema.get("type")
    if isinstance(kind, str):
        return kind
    if isinstance(kind, list):
        return next((k for k in kind if isinstance(k, str) and k != "null"), None)
    return None


def same(a: Any, b: Any) -> bool:
    """JSON equality (true isn't 1, 1 isn't 1.0), as serde_json compares."""
    return text(a) == text(b)


def options(values: list[Any]) -> str:
    return ", ".join(text(value) for value in values)


def number(value: float) -> str:
    """A schema bound as Rust prints an f64 (0, not 0.0; 0.5)."""
    return str(int(value)) if float(value).is_integer() else str(value)


ARTICLES = {
    "string": "a string",
    "boolean": "true or false",
    "integer": "a whole number",
    "number": "a number",
    "array": "a list",
    "object": "an object",
    "null": "null",
}


def fits(value: Any, kind: str) -> bool:
    if kind == "string":
        return isinstance(value, str)
    if kind == "boolean":
        return isinstance(value, bool)
    if kind == "integer":
        return (isinstance(value, int) and not isinstance(value, bool)) or (
            isinstance(value, float) and value.is_integer()
        )
    if kind == "number":
        return isinstance(value, int | float) and not isinstance(value, bool)
    if kind == "array":
        return isinstance(value, list)
    if kind == "object":
        return isinstance(value, dict)
    if kind == "null":
        return value is None
    return True


def check(value: Any, schema: dict[str, Any], root: dict[str, Any]) -> str | None:
    """None, or why `value` doesn't fit `schema` (`needs a number, got string "fast"`)."""
    schema = resolve(schema, root)
    variants = schema.get("anyOf") or schema.get("oneOf")
    if isinstance(variants, list):
        reasons: list[str] = []
        for variant in variants:
            why = check(value, variant, root)
            if why is None:
                return None
            if not reasons or reasons[-1] != why:
                reasons.append(why)
        return "; or ".join(reasons)
    if isinstance(schema.get("enum"), list) and not any(same(value, o) for o in schema["enum"]):
        return f"needs one of {options(schema['enum'])}, got {text(value)}"
    if "const" in schema and not same(schema["const"], value):
        return f"needs {text(schema['const'])}, got {text(value)}"
    kind = schema.get("type")
    types = [kind] if isinstance(kind, str) else [k for k in kind or [] if isinstance(k, str)]
    if types and not any(fits(value, k) for k in types):
        wanted = " or ".join(ARTICLES.get(k, "a value") for k in types)
        return f"needs {wanted}, got {kind_of(value)} {text(value)}"
    if isinstance(value, int | float) and not isinstance(value, bool):
        minimum, maximum = schema.get("minimum"), schema.get("maximum")
        if isinstance(minimum, int | float) and value < minimum:
            return f"needs at least {number(minimum)}, got {text(value)}"
        if isinstance(maximum, int | float) and value > maximum:
            return f"needs at most {number(maximum)}, got {text(value)}"
    if isinstance(schema.get("items"), dict) and isinstance(value, list):
        for index, item in enumerate(value):
            why = check(item, schema["items"], root)
            if why:
                return f"item {index}: {why}"
    if isinstance(schema.get("additionalProperties"), dict) and isinstance(value, dict):
        for key, item in sorted(value.items()):
            why = check(item, schema["additionalProperties"], root)
            if why:
                return f"`{key}`: {why}"
    return None


def distance(a: str, b: str) -> int:
    row = list(range(len(b) + 1))
    for i, ca in enumerate(a):
        previous, row[0] = row[0], i + 1
        for j, cb in enumerate(b):
            current = row[j + 1]
            row[j + 1] = min(previous + (ca != cb), row[j] + 1, current + 1)
            previous = current
    return row[len(b)]


def unknown(path: str, what: str, name: str, known: list[str]) -> str:
    """`overrides.global.robot_iq: no such GlobalConfig field (did you mean robot_ip?)`"""
    close = sorted((distance(name, k), k) for k in known)
    if close and close[0][0] <= max(3, len(name) // 3):
        return f"{path}: no such {what} (did you mean {close[0][1]}?)"
    if len(known) <= 12:
        return f"{path}: no such {what} (it has: {', '.join(known)})"
    return f"{path}: no such {what}"
