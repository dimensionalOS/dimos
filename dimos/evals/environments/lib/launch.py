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

"""Finding and validating the dimos a live eval case drives."""

from __future__ import annotations


def default_mcp_url() -> str:
    from dimos.core.global_config import global_config

    return f"http://localhost:{global_config.mcp_port}/mcp"


def blueprint_modules(blueprint: str) -> tuple[type, ...]:
    """Module classes a ``dimos run <blueprint>`` composition would deploy.
    Raises on an unknown name."""
    from dimos.core.coordination.blueprints import autoconnect
    from dimos.robot.get_all_blueprints import get_by_name

    composed = autoconnect(*(get_by_name(name) for name in blueprint.split()))
    return tuple(atom.module for atom in composed.blueprints)
