#!/usr/bin/env python3
# Copyright 2025-2026 Dimensional Inc.
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

from typing import Any

__all__ = ["_go2_robot_body"]

_LAYER_BLUEPRINTS: dict[str, Any] = {}


def __getattr__(name: str) -> Any:
    if name not in __all__:
        raise AttributeError(name)
    _build_layer_blueprints()
    return _LAYER_BLUEPRINTS[name]


def _build_layer_blueprints() -> None:
    if _LAYER_BLUEPRINTS:
        return

    from dimos.core.coordination.blueprints import autoconnect
    from dimos.robot.unitree.go2.blueprints.layers.layer_6_robot_body.robot_body_state import (
        _Go2RobotBodyState,
    )
    from dimos.robot.unitree.go2.blueprints.smart.unitree_go2 import unitree_go2

    robot_body = autoconnect(
        unitree_go2,
        _Go2RobotBodyState.blueprint(),
    )

    _LAYER_BLUEPRINTS["_go2_robot_body"] = robot_body
    globals().update(_LAYER_BLUEPRINTS)
