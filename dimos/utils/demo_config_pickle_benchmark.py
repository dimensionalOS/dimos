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

"""Measure construction and pickle costs for representative validated configs."""

from collections.abc import Callable
from functools import partial
import pickle
from statistics import median
import timeit
from typing import Any

from dimos.control.tasks.trajectory_task.trajectory_task import JointTrajectoryTaskConfig
from dimos.hardware.manipulators.galaxea_a1z.config import A1ZConfig, A1ZGripperConfig
from dimos.manipulation.execution_manager import ExecutionTarget

PICKLE_PROTOCOL = 5
ITERATIONS = 100_000
REPEATS = 7


def _trajectory_config() -> JointTrajectoryTaskConfig:
    return JointTrajectoryTaskConfig(joint_names=("arm/j1", "arm/j2"))


def _execution_target() -> ExecutionTarget:
    return ExecutionTarget.from_coordinator_mapping(
        robot_name="arm",
        model_joint_names=("j1", "j2"),
        coordinator_to_model={"arm/j1": "j1", "arm/j2": "j2"},
    )


def _a1z_config() -> A1ZConfig:
    return A1ZConfig(gravity_comp_factor=0.5, gripper=A1ZGripperConfig())


def _median_microseconds(operation: Callable[[], Any]) -> float:
    samples = timeit.repeat(operation, number=ITERATIONS, repeat=REPEATS)
    return median(samples) * 1_000_000 / ITERATIONS


def main() -> None:
    """Print median costs using a fixed pickle protocol."""
    factories = {
        "JointTrajectoryTaskConfig": _trajectory_config,
        "ExecutionTarget": _execution_target,
        "A1ZConfig": _a1z_config,
    }
    print(f"pickle protocol={PICKLE_PROTOCOL}, iterations={ITERATIONS}, repeats={REPEATS}")
    print("model, bytes, construct_us, dump_us, load_us")
    for name, factory in factories.items():
        value = factory()
        payload = pickle.dumps(value, protocol=PICKLE_PROTOCOL)
        construct_us = _median_microseconds(factory)
        dump_us = _median_microseconds(partial(pickle.dumps, value, protocol=PICKLE_PROTOCOL))
        load_us = _median_microseconds(partial(pickle.loads, payload))
        print(f"{name}, {len(payload)}, {construct_us:.3f}, {dump_us:.3f}, {load_us:.3f}")


if __name__ == "__main__":
    main()
