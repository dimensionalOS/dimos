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

"""Run environment checks inside another interpreter.

The launcher cannot import the packages of a managed environment, so it runs
``<env python> -m dimos.deps.probe`` with a request on stdin and reads the
report from stdout (stderr passes through). Heavy checks (backend probes,
blueprint imports) only ever run here, never in the launcher process.

Exit codes: 0 report produced (findings live inside it), 1 the probe itself
crashed, 2 the request was invalid.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, field
import importlib
import json
from pathlib import Path
import subprocess
import sys
from typing import Any

from dimos.deps.catalog import Plan
from dimos.deps.environment import EnvironmentReport, Outcome, Status, check_environment

REQUEST_SCHEMA = 2


class ProbeError(RuntimeError):
    """The probe process failed or returned something that is not a report."""


@dataclass(frozen=True)
class ProbeRequest:
    extras: tuple[str, ...]
    backends: Mapping[str, str] = field(default_factory=dict)
    native: tuple[str, ...] = ()
    system: tuple[str, ...] = ()
    tools: tuple[str, ...] = ()
    checks: tuple[str, ...] = ()
    blueprints: tuple[str, ...] = ()
    global_config: Mapping[str, Any] = field(default_factory=dict)
    """Preparsed global configuration without secrets, applied before importing blueprints."""

    @classmethod
    def from_plan(
        cls,
        plan: Plan,
        checks: tuple[str, ...],
        *,
        blueprints: tuple[str, ...] = (),
        global_config: Mapping[str, Any] | None = None,
    ) -> ProbeRequest:
        return cls(
            extras=tuple(sorted(plan.extras)),
            backends=dict(plan.backends),
            native=tuple(sorted(plan.native)),
            system=tuple(sorted(plan.system)),
            tools=tuple(sorted(plan.tools)),
            checks=checks,
            blueprints=blueprints,
            global_config=_jsonable(global_config or {}),
        )

    def plan(self) -> Plan:
        return Plan(
            extras=frozenset(self.extras),
            backends=dict(self.backends),
            native=frozenset(self.native),
            system=frozenset(self.system),
            tools=frozenset(self.tools),
        )

    def to_json(self) -> dict[str, Any]:
        return {
            "schema": REQUEST_SCHEMA,
            "extras": list(self.extras),
            "backends": dict(self.backends),
            "native": list(self.native),
            "system": list(self.system),
            "tools": list(self.tools),
            "checks": list(self.checks),
            "blueprints": list(self.blueprints),
            "global_config": dict(self.global_config),
        }

    @classmethod
    def from_json(cls, data: Mapping[str, Any]) -> ProbeRequest:
        if data.get("schema") != REQUEST_SCHEMA:
            raise ValueError(f"unsupported probe request schema {data.get('schema')!r}")
        return cls(
            extras=tuple(data["extras"]),
            backends=dict(data.get("backends", {})),
            native=tuple(data.get("native", [])),
            system=tuple(data.get("system", [])),
            tools=tuple(data.get("tools", [])),
            checks=tuple(data.get("checks", [])),
            blueprints=tuple(data.get("blueprints", [])),
            global_config=dict(data.get("global_config", {})),
        )


def _jsonable(values: Mapping[str, Any]) -> dict[str, Any]:
    return {key: str(value) if isinstance(value, Path) else value for key, value in values.items()}


def run_probe(python: Path, request: ProbeRequest, *, timeout: float = 900.0) -> EnvironmentReport:
    """Run the probe with ``python`` and return its report."""
    try:
        completed = subprocess.run(
            [str(python), "-m", "dimos.deps.probe"],
            input=json.dumps(request.to_json()),
            stdout=subprocess.PIPE,
            text=True,
            timeout=timeout,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired) as error:
        raise ProbeError(f"could not run the environment probe with {python}: {error}") from error
    if completed.returncode == 2:
        raise ProbeError(
            f"environment probe {python} rejected the request: the environment's dimos is "
            "older than this one"
        )
    if completed.returncode != 0:
        raise ProbeError(f"environment probe {python} exited with {completed.returncode}")
    try:
        return EnvironmentReport.from_json(json.loads(completed.stdout))
    except (ValueError, KeyError, TypeError) as error:
        raise ProbeError(f"environment probe {python} returned no report: {error}") from error


def check_backends(backends: Mapping[str, str]) -> dict[str, Outcome]:
    """Import each backend and confirm the accelerator it was resolved with works."""
    results: dict[str, Outcome] = {}
    try:
        # Loading torch first mirrors the runtime order: its bundled CUDA libraries
        # are what onnxruntime-gpu picks up afterwards.
        torch: Any = importlib.import_module("torch")
    except Exception:
        torch = None
    for backend, accelerator in backends.items():
        if backend != "onnxruntime":
            results[backend] = Outcome("missing", "unknown backend")
            continue
        try:
            onnxruntime = importlib.import_module("onnxruntime")
            providers = list(onnxruntime.get_available_providers())
        except Exception as error:
            results[backend] = Outcome("missing", f"import failed: {type(error).__name__}: {error}")
            continue
        wanted = "CUDAExecutionProvider" if accelerator == "cuda" else "CPUExecutionProvider"
        status: Status = "satisfied" if wanted in providers else "missing"
        results[backend] = Outcome(status, f"providers={providers}")
    if any(accelerator == "cuda" for accelerator in backends.values()) and torch is not None:
        available = bool(torch.cuda.is_available())
        status = "satisfied" if available else "missing"
        results["torch"] = Outcome(status, f"torch.cuda.is_available()={available}")
    return results


def check_blueprints(
    names: tuple[str, ...], global_config: Mapping[str, Any]
) -> dict[str, Outcome]:
    """Import blueprint targets exactly as ``dimos run`` would."""
    from dimos.core.global_config import global_config as config

    results: dict[str, Outcome] = {}
    if global_config:
        try:
            config.update(**global_config)
        except Exception as error:
            return dict.fromkeys(names, Outcome("missing", f"invalid global config: {error}"))
    from dimos.robot.get_all_blueprints import get_by_name

    for name in names:
        try:
            get_by_name(name)
        except Exception as error:
            results[name] = Outcome("missing", f"{type(error).__name__}: {error}")
        else:
            results[name] = Outcome("satisfied")
    return results


def main() -> int:
    try:
        request = ProbeRequest.from_json(json.loads(sys.stdin.read()))
    except (ValueError, KeyError, TypeError) as error:
        print(f"invalid probe request: {error}", file=sys.stderr)
        return 2
    report = check_environment(request.plan(), checks=request.checks)
    if "backends" in request.checks:
        report.backends = check_backends(request.backends)
    if "blueprints" in request.checks:
        report.blueprints = check_blueprints(request.blueprints, request.global_config)
    json.dump(report.to_json(), sys.stdout)
    sys.stdout.write("\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
