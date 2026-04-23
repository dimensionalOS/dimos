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

import json
from typing import TYPE_CHECKING, Any

from dimos.porcelain.module_source import ModuleSource

if TYPE_CHECKING:
    from dimos.core.module import SkillInfo


class _SkillCallable:
    """Callable wrapper around a remote skill method."""

    def __init__(self, module_proxy: Any, name: str, info: SkillInfo) -> None:
        self._module_proxy = module_proxy
        self._name = name
        self._info = info

    def __call__(self, *args: Any, **kwargs: Any) -> Any:
        method = getattr(self._module_proxy, self._name)
        return method(*args, **kwargs)

    def __repr__(self) -> str:
        schema = json.loads(self._info.args_schema)
        params = _format_params(schema)
        return f"Skill: {self._info.class_name}.{self._name}({params})"


class SkillsProxy:
    """Attribute-access proxy that discovers and exposes skills from all deployed modules."""

    def __init__(self, source: ModuleSource) -> None:
        self._source = source
        self._cache: dict[str, list[tuple[str, Any, SkillInfo]]] | None = None
        self._cache_key: frozenset[str] | None = None

    def _build_cache(self) -> None:
        names = self._source.list_module_names()
        modules_key = frozenset(names)
        if self._cache_key == modules_key and self._cache is not None:
            return

        skill_map: dict[str, list[tuple[str, Any, SkillInfo]]] = {}
        for name in names:
            try:
                module_proxy = self._source.get_rpyc_module(name)
                skills = list(module_proxy.get_skills())
            except Exception:
                continue
            for info in skills:
                skill_map.setdefault(info.func_name, []).append((name, module_proxy, info))

        self._cache = skill_map
        self._cache_key = modules_key

    def __getattr__(self, name: str) -> _SkillCallable:
        if name.startswith("_"):
            raise AttributeError(name)
        self._build_cache()
        assert self._cache is not None

        if name not in self._cache:
            raise AttributeError(f"No skill named {name!r}")

        entries = self._cache[name]
        if len(entries) > 1:
            modules = [cls_name for cls_name, _, _ in entries]
            raise AttributeError(
                f"Ambiguous skill {name!r} found in modules: {modules}. "
                f"Call via module directly: app.{modules[0]}.{name}()"
            )
        _cls_name, module_proxy, info = entries[0]
        return _SkillCallable(module_proxy, name, info)

    def __repr__(self) -> str:
        self._build_cache()
        assert self._cache is not None

        if not self._cache:
            return "Skills: (none)"

        lines = ["Skills:"]
        for name in sorted(self._cache):
            for cls_name, _, info in self._cache[name]:
                schema = json.loads(info.args_schema)
                params = _format_params(schema)
                lines.append(f"  {name}({params})")
                desc = schema.get("description", "")
                if desc:
                    lines.append(f"    {desc}")
                lines.append(f"    [{cls_name}]")
        return "\n".join(lines)

    def __dir__(self) -> list[str]:
        self._build_cache()
        assert self._cache is not None
        return list(self._cache.keys())


def _format_params(schema: dict[str, Any]) -> str:
    """Format JSON Schema properties into a Python-style parameter string."""
    props = schema.get("properties", {})
    required = set(schema.get("required", []))
    parts: list[str] = []
    for pname, pdef in props.items():
        ptype = pdef.get("type", "Any")
        type_map = {"number": "float", "integer": "int", "string": "str", "boolean": "bool"}
        ptype = type_map.get(ptype, ptype)
        if "default" in pdef:
            parts.append(f"{pname}: {ptype} = {pdef['default']!r}")
        elif pname not in required:
            parts.append(f"{pname}: {ptype} = None")
        else:
            parts.append(f"{pname}: {ptype}")
    return ", ".join(parts)
