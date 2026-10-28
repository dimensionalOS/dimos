# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0

"""Exercise the trajectory conversion without loading the optional policy runtime."""

import ast
from pathlib import Path
from types import SimpleNamespace

from dimos_cdrgen.registry import decode, encode
import numpy as np

from dimos.message_utils import time as message_time
from dimos.messages.std_msgs.msg import Header
from dimos.messages.trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def test_policy_chunk_preserves_joint_order_values_and_duration_on_wire():
    tree = ast.parse(Path(__file__).with_name("runtime.py").read_text())
    runtime = next(
        node
        for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == "LeRobotPolicyRuntime"
    )
    method = next(
        node
        for node in runtime.body
        if isinstance(node, ast.FunctionDef) and node.name == "_trajectory"
    )
    namespace = {
        "np": np,
        "message_time": message_time,
        "Header": Header,
        "JointTrajectory": JointTrajectory,
        "JointTrajectoryPoint": JointTrajectoryPoint,
    }
    module = ast.Module(
        body=[
            ast.ImportFrom(module="__future__", names=[ast.alias(name="annotations")], level=0),
            method,
        ],
        type_ignores=[],
    )
    exec(compile(ast.fix_missing_locations(module), "runtime.py:_trajectory", "exec"), namespace)
    policy = SimpleNamespace(config=SimpleNamespace(joint_names=["arm/b", "arm/a"], fps=4))
    trajectory = namespace["_trajectory"](
        policy, np.array([1, 2], dtype=np.float32), np.array([[3, 4], [5, 6]], dtype=np.float32)
    )
    assert trajectory.joint_names == ["arm/b", "arm/a"]
    for little_endian in (True, False):
        restored = decode(encode(trajectory, little_endian=little_endian), JointTrajectory)
        assert [
            message_time.to_nanoseconds(point.time_from_start) for point in restored.points
        ] == [0, 250000000, 500000000]
        np.testing.assert_array_equal(
            [point.positions for point in restored.points], [[1, 2], [3, 4], [5, 6]]
        )
        np.testing.assert_array_equal(
            [point.velocities for point in restored.points], np.zeros((3, 2))
        )
        assert all(len(point.accelerations) == len(point.effort) == 0 for point in restored.points)
