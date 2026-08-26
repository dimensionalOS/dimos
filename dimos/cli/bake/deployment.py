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


"""A bake target: which modules, their tuned config blocks, and the zenoh session."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, field
import importlib

from dimos.cli.bake.errors import BakeError


@dataclass(frozen=True)
class Deployment:
    """What `dimos bake --deployment` embeds: the module list plus this site's overrides."""

    modules: tuple[str, ...]
    # Full config blocks replacing a module's class defaults, keyed by module id.
    configs: Mapping[str, Mapping[str, object]] = field(default_factory=dict)
    # The `session` block, `ZenohConfig(...).to_wire()`; None opens zenoh's defaults.
    session: Mapping[str, object] | None = None


def load_deployment(ref: str) -> Deployment:
    """Import `pkg.mod:NAME` and check it is a Deployment."""
    module_name, _, attr = ref.partition(":")
    try:
        target = getattr(importlib.import_module(module_name), attr)
    except (ImportError, AttributeError) as exc:
        raise BakeError(f"cannot import deployment `{ref}`: {exc}") from exc
    if not isinstance(target, Deployment):
        raise BakeError(f"`{ref}` is a {type(target).__name__}, not a Deployment")
    return target
