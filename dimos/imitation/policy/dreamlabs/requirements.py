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

"""Dependency preflight for the manual stack, before any modules start."""

from importlib import import_module


def check_planning_dependencies() -> str | None:
    """Reject old/incomplete RoboPlan installs instead of starting a partial stack."""
    hint = (
        "Dreamlabs manual stack needs the locked RoboPlan 0.7 planning environment. "
        "Follow dimos/imitation/policy/dreamlabs/README.md#prepare-an-isolated-environment; "
        "do not reuse an unsynchronized shared virtualenv."
    )
    try:
        core = import_module("roboplan.core")
    except (ImportError, OSError) as exc:
        return f"{hint} RoboPlan import failed: {type(exc).__name__}"
    if not callable(getattr(core, "loadUrdfSceneDescriptionFromXml", None)):
        return f"{hint} Missing roboplan.core.loadUrdfSceneDescriptionFromXml."
    return None
