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

"""Resolve connection descriptions without opening hardware.

Robot factories are imported on demand, just like manipulation backends.
"""

from dataclasses import replace
from importlib import import_module
from typing import Any

from dimos.control.components import HardwareComponent
from dimos.control.connection import (
    A1ZConnectionConfig,
    A750ConnectionConfig,
    DualOpenYamConnectionConfig,
    DualXArmConnectionConfig,
    HardwareConnectionConfig,
    MixedArmConnectionConfig,
    OpenArmConnectionConfig,
    OpenYamConnectionConfig,
    PiperConnectionConfig,
    XArmConnectionConfig,
)
from dimos.hardware.whole_body.damiao.config import DamiaoRuntimeConfig


def _hardware(robot: str, factory: str, **kwargs: Any) -> HardwareComponent:
    module = import_module(f"dimos.robot.manipulators.{robot}.config")
    result: HardwareComponent = getattr(module, factory)(**kwargs)
    return result


def _resolved_component(
    component: HardwareComponent, resolved: HardwareComponent
) -> HardwareComponent:
    adapter_kwargs = {**resolved.adapter_kwargs, **component.adapter_kwargs}
    runtime = resolved.adapter_kwargs.get("runtime_config")
    if isinstance(runtime, DamiaoRuntimeConfig):
        configured = component.adapter_kwargs.get("runtime_config", runtime)
        adapter_kwargs["runtime_config"] = replace(configured, bus_devices=runtime.bus_devices)
    return replace(
        component,
        adapter_type=resolved.adapter_type,
        address=resolved.address,
        limits=component.limits if resolved.adapter_type in ("mock", "mock_whole_body") else None,
        adapter_kwargs=adapter_kwargs,
    )


def _xarm(
    component: HardwareComponent, dof: int, address: str | None, simulation: str
) -> HardwareComponent:
    resolved = _hardware("xarm", f"xarm{dof}_hardware", address=address, simulation=simulation)
    # Keep the blueprint's gripper and other adapter options.
    return replace(
        _resolved_component(component, resolved),
        adapter_kwargs={**component.adapter_kwargs, "arm_dof": dof},
    )


def resolve_connection(
    config: HardwareConnectionConfig,
    hardware: list[HardwareComponent],
    simulation: str,
) -> list[HardwareComponent]:
    """Select concrete hardware while preserving blueprint-owned descriptions."""
    if isinstance(config, (OpenArmConnectionConfig, DualOpenYamConnectionConfig)):
        if len(hardware) > 1:
            raise ValueError("Coupled connection requires at most one hardware description")
        robot = config.backend
        resolved = _hardware(
            robot,
            f"{robot}_hardware",
            left_can_port=config.left_can_port,
            right_can_port=config.right_can_port,
        )
        return [_resolved_component(hardware[0], resolved)] if hardware else [resolved]

    if isinstance(config, (DualXArmConnectionConfig, MixedArmConnectionConfig)):
        if len(hardware) != 2:
            raise ValueError("Dual-arm connection requires exactly two hardware descriptions")
        left, right = hardware
        if isinstance(config, DualXArmConnectionConfig):
            return [
                _xarm(left, 7, config.left_address, simulation),
                _xarm(right, 6, config.right_address, simulation),
            ]
        return [
            _xarm(left, 6, config.xarm_address, ""),
            _resolved_component(
                right, _hardware("piper", "piper_hardware", address=config.piper_address)
            ),
        ]

    if len(hardware) != 1:
        raise ValueError("Single-arm connection requires exactly one hardware description")
    component = hardware[0]
    if isinstance(config, XArmConnectionConfig):
        return [
            _xarm(
                component,
                config.dof,
                config.address,
                simulation if config.backend == "xarm" else "",
            )
        ]
    if isinstance(config, PiperConnectionConfig):
        resolved = _hardware(
            "piper",
            "piper_hardware",
            address=config.address,
            simulation=simulation if config.backend == "piper" else "",
        )
    elif isinstance(config, A1ZConnectionConfig):
        # A1Z's blueprint already owns the gripper/dynamics configuration.
        return [
            replace(
                component,
                adapter_type="galaxea_a1z"
                if config.address is not None and not simulation
                else "mock",
                address=config.address if not simulation else None,
                limits=component.limits if simulation or config.address is None else None,
            )
        ]
    elif isinstance(config, A750ConnectionConfig):
        resolved = _hardware("a750", "a750_hardware", address=config.address)
    elif isinstance(config, OpenYamConnectionConfig):
        resolved = _hardware(
            "openyam", "openyam_hardware", address=config.address, simulation=simulation
        )
    else:
        raise TypeError(f"Unsupported connection config: {type(config).__name__}")
    return [_resolved_component(component, resolved)]
