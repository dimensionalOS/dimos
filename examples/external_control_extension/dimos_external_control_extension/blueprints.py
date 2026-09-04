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

"""Blueprint-style entry points for the external control extension example."""

from __future__ import annotations

import logging
import time

from dimos.control.components import HardwareComponent, HardwareType, make_twist_base_joints
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.control.extensions import register_control_task, register_hardware_adapter
from dimos_external_control_extension.adapters import ExternalTestBaseAdapter

LOGGER = logging.getLogger("dimos_external_control_extension")

ADAPTER_TYPE = "external_test_base"
TASK_TYPE = "external_test_drive"
TASK_FACTORY_PATH = "dimos_external_control_extension.tasks:create_task"
HARDWARE_ID = "external_test_base"
TASK_NAME = "drive_demo"


def register_extensions() -> None:
    """Register this package's external ControlCoordinator extensions."""
    register_hardware_adapter(HardwareType.BASE, ADAPTER_TYPE, ExternalTestBaseAdapter)
    LOGGER.info(
        "[external_test_robot] registered hardware adapter: BASE/%s",
        ADAPTER_TYPE,
    )
    register_control_task(TASK_TYPE, TASK_FACTORY_PATH)
    LOGGER.info("[external_test_robot] registered control task: %s", TASK_TYPE)


def build_demo_coordinator() -> ControlCoordinator:
    """Build a coordinator using only externally registered names."""
    register_extensions()
    return ControlCoordinator(
        tick_rate=20.0,
        publish_joint_state=False,
        log_ticks=True,
        hardware=[
            HardwareComponent(
                hardware_id=HARDWARE_ID,
                hardware_type=HardwareType.BASE,
                joints=make_twist_base_joints(HARDWARE_ID),
                adapter_type=ADAPTER_TYPE,
            )
        ],
        tasks=[
            TaskConfig(
                name=TASK_NAME,
                type=TASK_TYPE,
                joint_names=make_twist_base_joints(HARDWARE_ID),
                priority=10,
                params={"velocity": 0.1},
            )
        ],
    )


def run_demo(target_writes: int = 3, timeout: float = 2.0) -> ControlCoordinator:
    """Start the demo coordinator until the external adapter receives commands."""
    coordinator = build_demo_coordinator()
    coordinator.start()
    try:
        adapter = coordinator._hardware[HARDWARE_ID].adapter
        if not isinstance(adapter, ExternalTestBaseAdapter):
            raise TypeError(f"Expected ExternalTestBaseAdapter, got {type(adapter).__name__}")

        deadline = time.monotonic() + timeout
        while adapter.write_count < target_writes:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError(
                    f"External adapter received {adapter.write_count} writes; "
                    f"expected {target_writes}"
                )
            adapter.command_event.wait(timeout=remaining)
            adapter.command_event.clear()
        LOGGER.info(
            "[external_test_robot] demo observed %s external adapter writes",
            adapter.write_count,
        )
        return coordinator
    finally:
        coordinator.stop()


register_extensions()

__all__ = [
    "ADAPTER_TYPE",
    "HARDWARE_ID",
    "TASK_NAME",
    "TASK_TYPE",
    "build_demo_coordinator",
    "register_extensions",
    "run_demo",
]
