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

"""Plan one run: catalog presets under a configuration, resolved for a profile."""

from __future__ import annotations

from collections.abc import Iterable, Mapping
from dataclasses import dataclass

from dimos.deps.catalog import Plan, default_catalog
from dimos.deps.profiles import Profile, UnsupportedProfileError, resolve_profile
from dimos.deps.selectors import SelectorInput


@dataclass(frozen=True)
class RunPlan:
    names: tuple[str, ...]
    plan: Plan
    profile: Profile | None
    """``None`` when the host is unsupported and no override was given."""
    overridden: bool
    unsupported: UnsupportedProfileError | None

    @property
    def builtin_names(self) -> tuple[str, ...]:
        return tuple(name for name in self.names if name not in self.plan.external)


def plan_run(
    names: Iterable[str],
    config: Mapping[str, object],
    profile_override: str | None,
    inputs: Iterable[SelectorInput] = (),
) -> RunPlan:
    """Plan the requested names; an unsupported host leaves backends unresolved.

    ``config`` holds the planning values (global configuration plus the derived
    properties) and ``inputs`` the module-level selections read from the request.
    Raises :class:`dimos.deps.catalog.CatalogError` for unknown built-in names and
    ``ValueError`` for an override that does not fit the host.
    """
    names = tuple(names)
    profile: Profile | None
    unsupported: UnsupportedProfileError | None = None
    overridden = False
    try:
        profile, overridden = resolve_profile(profile_override)
    except UnsupportedProfileError as error:
        profile, unsupported = None, error
    accelerator = profile.accelerator if profile is not None else None
    plan = default_catalog().plan_for(names, config, accelerator=accelerator, inputs=inputs)
    return RunPlan(names, plan, profile, overridden, unsupported)


def install_recipes(plan: Plan) -> dict[str, str]:
    """Shell recipes that install the plan's extras into an existing environment."""
    extras = sorted(plan.extras)
    if not extras:
        return {}
    flags = " ".join(f"--extra {extra}" for extra in extras)
    return {
        "checkout": f"uv sync {flags} --inexact",
        "release": f"pip install 'dimos[{','.join(extras)}]'",
    }
