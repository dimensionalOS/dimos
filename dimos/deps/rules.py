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

"""Configuration scenarios and dynamic rules for the dependency catalog.

Scenarios re-run the import analysis under a configuration (for example
``simulation="mujoco"``) so module-level composition switches on
``global_config`` are evaluated; only the delta against the defaults is
stored per blueprint.

Dynamic rules describe imports the scanner cannot follow: function-local
imports chosen by configuration, subprocess entry points and native tools.
A rule with a trigger file contributes only when that file is in the
closure; a rule without one applies to every run (CLI-level composition).

``DEFAULTS`` mirrors ``GlobalConfig`` defaults by hand so generation never
instantiates the settings model (which reads the environment and ``.env``).
A test keeps the two in sync.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass

from dimos.deps.predicates import Predicate

DEFAULTS: Mapping[str, object] = {
    "simulation": "",
    "replay": False,
    "viewer": "rerun",
    "robot_ip": None,
    "local_relay": False,
    "relay_url": None,
    "record": "",
    "record_engine": "python",
    "unitree_connection_type": "webrtc",
}
"""Values catalog predicates and selectors read when a run does not set them.

``unitree_connection_type`` is a ``GlobalConfig`` property, not a field; the
launcher adds it through ``GlobalConfig.planning_values``.
"""

MUJOCO_PREDICATE: Predicate = [
    "any",
    ["in", "simulation", ["mujoco", "true"]],
    ["eq", "robot_ip", "mujoco"],
]
DIMSIM_PREDICATE: Predicate = ["eq", "simulation", "dimsim"]


@dataclass(frozen=True)
class Scenario:
    name: str
    overrides: Mapping[str, object]
    when: Predicate


@dataclass(frozen=True)
class GlobalRule:
    """Files whose requirements every run adds when ``when`` holds, whatever it runs."""

    name: str
    when: Predicate
    roots: tuple[str, ...]
    """Analysis roots relative to ``dimos/``; their declarations are the requirements."""


SCENARIOS: Mapping[str, Scenario] = {
    "mujoco": Scenario("mujoco", {"simulation": "mujoco"}, MUJOCO_PREDICATE),
    "dimsim": Scenario("dimsim", {"simulation": "dimsim"}, DIMSIM_PREDICATE),
}

GLOBAL_RULES: tuple[GlobalRule, ...] = (
    GlobalRule(
        "relay",
        when=["any", ["truthy", "local_relay"], ["truthy", "relay_url"]],
        roots=("web/relay_bridge/relay_bridge_module.py",),
    ),
    GlobalRule(
        "rust-recorder",
        when=["all", ["truthy", "record"], ["eq", "record_engine", "rust"]],
        roots=("experimental/memory/rust_cli_recorder.py",),
    ),
)
