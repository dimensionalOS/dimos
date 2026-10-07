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

"""Host health checks. A doctor is a module in doctors/ with ``description``, ``check()``,
optionally ``fix()``, and ``warning = True`` when failing should not fail the run."""

from __future__ import annotations

from collections.abc import Iterable
from dataclasses import dataclass
from types import ModuleType

from dimos.hosted.plugins import load


@dataclass(frozen=True, slots=True)
class Result:
    description: str
    ok: bool
    error: str | None = None
    # None: not attempted; otherwise what the fix did.
    fix_note: str | None = None
    # A failing warning is reported but does not fail the doctor run.
    warning: bool = False


def doctors() -> list[ModuleType]:
    """By file name, so service_installed fixes before service_running starts anything."""
    return load("dimos.hosted.doctors")


def _check(doctor: ModuleType) -> tuple[bool, str | None]:
    try:
        return bool(doctor.check()), None
    except Exception as exc:
        return False, f"{type(exc).__name__}: {exc}"


def run(fix: bool = False, modules: Iterable[ModuleType] | None = None) -> list[Result]:
    """Check every doctor; with ``fix``, fix the failing ones that can and check again."""
    results = []
    for doctor in doctors() if modules is None else modules:
        ok, error = _check(doctor)
        note = None
        if not ok and fix:
            if getattr(doctor, "fix", None) is None:
                note = "no automatic fix"
            else:
                try:
                    doctor.fix()
                    note = "fixed"
                except Exception as exc:
                    note = f"fix failed: {type(exc).__name__}: {exc}"
                ok, error = _check(doctor)
        results.append(
            Result(doctor.description, ok, error, note, bool(getattr(doctor, "warning", False)))
        )
    return results
