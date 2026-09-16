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

"""Build ``blueprint_catalog.json`` from the registry, the sources and the lock.

Run ``python -m dimos.deps.build_catalog`` to print findings and warnings;
``pytest dimos/deps/test_catalog_generation.py`` regenerates the committed file.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
import sys

from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.deps.analysis import Analyzer, Finding
from dimos.deps.catalog import CATALOG_VERSION
from dimos.deps.import_map import BACKENDS
from dimos.deps.lock import LockIndex
from dimos.deps.rules import DEFAULTS, GLOBAL_RULES, SCENARIOS
from dimos.robot.all_blueprints import all_blueprints, all_modules

ENTRY_SECTIONS = ("blueprints", "modules", "rules", "registries")


@dataclass(frozen=True)
class CatalogBuild:
    catalog: dict[str, object]
    findings: tuple[Finding, ...]
    warnings: tuple[str, ...]


def build_catalog(project_root: Path = DIMOS_PROJECT_ROOT) -> CatalogBuild:
    analyzer = Analyzer(project_root, LockIndex.load(project_root))
    findings: list[Finding] = []
    warnings: list[str] = []
    sections: dict[str, dict[str, object]] = {"blueprints": {}, "modules": {}}
    for section, registry in (("blueprints", all_blueprints), ("modules", all_modules)):
        for name, target in sorted(registry.items()):
            result = analyzer.analyze_root(name, target)
            sections[section][name] = result.to_json()
            findings.extend(result.findings)
            warnings.extend(result.warnings)
    rules: dict[str, object] = {}
    for rule in GLOBAL_RULES:
        requirements, rule_findings, rule_warnings = analyzer.analyze_rule(rule)
        rules[rule.name] = {"when": rule.when, **requirements.to_json()}
        findings.extend(rule_findings)
        warnings.extend(rule_warnings)
    registries: dict[str, dict[str, object]] = {}
    for (family, name), file in sorted(analyzer.manifests().items()):
        requirements, entry_findings, entry_warnings = analyzer.analyze_files([file])
        registries.setdefault(family, {})[name] = requirements.to_json()
        findings.extend(entry_findings)
        warnings.extend(entry_warnings)
    for family, name, first, second in analyzer.manifest_conflicts():
        if analyzer.analyze_files([first])[0] != analyzer.analyze_files([second])[0]:
            findings.append(
                Finding(
                    analyzer.relative(second),
                    0,
                    name,
                    f"{family} {name!r} is defined by two manifests with different requirements",
                    f"the other is {analyzer.relative(first)}; give them one implementation",
                )
            )
    catalog: dict[str, object] = {
        "version": CATALOG_VERSION,
        "defaults": dict(DEFAULTS),
        "scenarios": {name: scenario.when for name, scenario in SCENARIOS.items()},
        "rules": rules,
        "registries": registries,
        "backends": {
            name: {accelerator: list(extras) for accelerator, extras in table.items()}
            for name, table in BACKENDS.items()
        },
        "blueprints": sections["blueprints"],
        "modules": sections["modules"],
    }
    return CatalogBuild(catalog, tuple(dict.fromkeys(findings)), tuple(dict.fromkeys(warnings)))


def render(catalog: dict[str, object]) -> str:
    """JSON with one line per registry entry: small, and diffs stay readable."""

    def line(value: object) -> str:
        return json.dumps(value, sort_keys=True, separators=(", ", ": "))

    lines = ["{"]
    keys = sorted(catalog)
    for index, key in enumerate(keys):
        value = catalog[key]
        comma = "," if index < len(keys) - 1 else ""
        if key in ENTRY_SECTIONS and isinstance(value, dict):
            lines.append(f"  {json.dumps(key)}: {{")
            names = sorted(value)
            for position, name in enumerate(names):
                entry_comma = "," if position < len(names) - 1 else ""
                lines.append(f"    {json.dumps(name)}: {line(value[name])}{entry_comma}")
            lines.append(f"  }}{comma}")
        else:
            lines.append(f"  {json.dumps(key)}: {line(value)}{comma}")
    lines.append("}")
    return "\n".join(lines) + "\n"


def main() -> int:
    build = build_catalog()
    for warning in build.warnings:
        print(f"warning: {warning}")
    for finding in build.findings:
        print(f"error: {finding}")
    size = len(render(build.catalog))
    print(f"{len(build.findings)} findings, {len(build.warnings)} warnings, {size} bytes")
    return 1 if build.findings else 0


if __name__ == "__main__":
    sys.exit(main())
