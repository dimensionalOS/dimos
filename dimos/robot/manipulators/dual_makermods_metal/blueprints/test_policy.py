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

"""Composition and plumbing for the opt-in policy blueprint."""

from __future__ import annotations

import os
import time
from typing import cast

import numpy as np
import pytest

from dimos.control.connection.mock_joint_connection import MockJointConnection
from dimos.control.coordinator import ControlCoordinator
from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.coordination.worker_manager_python import WorkerManagerPython
from dimos.robot.manipulators.damiao._damiao_test_helpers import mock_connection_rig
from dimos.robot.manipulators.damiao.damiao_blueprint import DamiaoTeleopCoordinator
from dimos.robot.manipulators.dual_makermods_metal.blueprints.policy import (
    dual_metal_policy_blueprint,
)
from dimos.robot.manipulators.dual_makermods_metal.config import ALL_ARM_JOINTS, FINGER_JOINTS
from dimos.robot.manipulators.dual_makermods_metal.connection import DualMetalConnection
from dimos.robot.manipulators.dual_makermods_metal.policy.contract import (
    HOME,
    PolicyManifest,
)
from dimos.robot.manipulators.dual_makermods_metal.policy.station import MetalStationKinematics
from dimos.utils.testing.waiting import wait_until

MODEL_DIR = os.environ.get("DIMOS_METAL_POLICY_MODEL_DIR")
EVIDENCE_DIR = os.environ.get("DIMOS_METAL_POLICY_EVIDENCE_DIR")


def _coordinator_atom(blueprint):
    return next(a for a in blueprint.blueprints if issubclass(a.module, ControlCoordinator))


def python_workers(coordinator: ModuleCoordinator) -> WorkerManagerPython:
    """The python worker manager, so tests can assert real workers existed and stopped."""
    return cast("WorkerManagerPython", coordinator._managers["python"])


def test_blueprint_is_fake_only_and_claims_the_12_arm_joints():
    blueprint = dual_metal_policy_blueprint(
        model_dir="/tmp/model", manifest_path="/tmp/model/manifest.json"
    )
    modules = {atom.module for atom in blueprint.blueprints}
    assert DamiaoTeleopCoordinator in modules
    assert MockJointConnection in modules
    # The fake path cannot construct the CAN-backed connection at all.
    assert DualMetalConnection not in modules

    coordinator = _coordinator_atom(blueprint)
    connection = next(a for a in blueprint.blueprints if a.module is MockJointConnection)
    initial = connection.kwargs["initial_positions"]
    # The connection model keeps the URDF joint names; the module prefixes its
    # own source ("arm") when it publishes readings.
    assert set(initial) == {
        *ALL_ARM_JOINTS,
        *(FINGER_JOINTS[side][0] for side in ("left", "right")),
    }

    tasks = {task.name: task for task in coordinator.kwargs["tasks"]}
    policy = tasks["metal_arm_policy"]
    assert policy.type == "metal_arm_policy"
    assert policy.joint_names == [f"arm/{joint}" for joint in ALL_ARM_JOINTS]
    assert policy.params["model_dir"] == "/tmp/model"
    assert policy.params["manifest_path"] == "/tmp/model/manifest.json"
    assert policy.params["teleop"] is False

    for side in ("left", "right"):
        gripper = tasks[f"{side}_arm_gripper"]
        assert gripper.joint_names == [f"arm/{FINGER_JOINTS[side][0]}"]


def test_blueprint_teleop_variant_enables_deadman_and_goal_timeout():
    blueprint = dual_metal_policy_blueprint(
        model_dir="/tmp/model", manifest_path="/tmp/model/manifest.json", teleop=True
    )
    tasks = {task.name: task for task in _coordinator_atom(blueprint).kwargs["tasks"]}
    policy = tasks["metal_arm_policy"]
    assert policy.params["teleop"] is True
    assert policy.params["goal_timeout_s"] == 0.25


@pytest.mark.skipif(
    not MODEL_DIR or not EVIDENCE_DIR,
    reason="set DIMOS_METAL_POLICY_MODEL_DIR and DIMOS_METAL_POLICY_EVIDENCE_DIR",
)
def test_registry_task_drives_the_fake_connection_to_the_goal(tmp_path):
    """Real registry task + real ONNX actor + mock joint feedback, no hardware."""
    manifest = os.path.join(MODEL_DIR or "", "metal_arm_policy.manifest.json")
    blueprint = dual_metal_policy_blueprint(model_dir=MODEL_DIR, manifest_path=manifest)
    atom = _coordinator_atom(blueprint)
    coordinator = atom.module(**atom.kwargs)
    with mock_connection_rig(DualMetalConnection, coordinator, tmp_path=tmp_path) as rig:
        assert "metal_arm_policy" in coordinator._tasks
        task = coordinator._tasks["metal_arm_policy"]
        assert task.claim().joints == frozenset(f"arm/{joint}" for joint in ALL_ARM_JOINTS)

        initial = dict(rig.positions)
        q0 = np.asarray([initial[f"arm/{joint}"] for joint in ALL_ARM_JOINTS], dtype=np.float32)
        current = task._kinematics.pose24(q0).tolist()
        moved = list(current)
        moved[0] += 0.05  # left TCP +x
        moved[3] -= 0.05  # right TCP -x
        assert coordinator.task_invoke("metal_arm_policy", "set_goal_pose24", {"pose24": moved})
        assert coordinator.task_invoke("metal_arm_policy", "engage", {})

        for _ in range(12):
            rig.tick()
            time.sleep(0.055)
            rig.tick()

        after = dict(rig.positions)
        deltas = np.asarray(
            [after[f"arm/{joint}"] - initial[f"arm/{joint}"] for joint in ALL_ARM_JOINTS]
        )
        assert np.abs(deltas).max() > 1e-3, "policy did not move the fake arms"
        assert task.state_snapshot()["session"] == "active"

        # Jaws belong to their own tasks and must not have moved.
        for side in ("left", "right"):
            finger = f"arm/{FINGER_JOINTS[side][0]}"
            assert after[finger] == pytest.approx(initial[finger], abs=1e-9)

        before_release = dict(rig.positions)
        assert coordinator.task_invoke("metal_arm_policy", "release", {})
        for _ in range(3):
            rig.tick()
            time.sleep(0.02)
        held = dict(rig.positions)
        drift = max(
            abs(held[f"arm/{joint}"] - before_release[f"arm/{joint}"]) for joint in ALL_ARM_JOINTS
        )
        assert drift < 1e-6, f"arms drifted after release: {drift}"
        assert task.state_snapshot()["fault"] == "released"


@pytest.mark.skipif(
    not MODEL_DIR or not EVIDENCE_DIR,
    reason="set DIMOS_METAL_POLICY_MODEL_DIR and DIMOS_METAL_POLICY_EVIDENCE_DIR",
)
def test_shipped_fake_blueprint_workers_close_the_loop():
    """The shipped blueprint, built as real workers: input -> policy -> feedback.

    Nothing here is a stand-in: the real ``dual_makermods_metal_policy_fake``
    composition is deployed through ``ModuleCoordinator.build`` (forkserver
    workers, the real coordinator tick loop and RPC), its own model, mount
    geometry and initial positions are used, the ONNX actor comes from the
    manifest, and the only robot interface is the shipped ``MockJointConnection``
    (no CAN). The test drives goals over RPC and reads joint feedback over RPC.
    """
    manifest = os.path.join(MODEL_DIR or "", "metal_arm_policy.manifest.json")
    blueprint = dual_metal_policy_blueprint(model_dir=MODEL_DIR, manifest_path=manifest)
    # Namespaced RPC topics and streams: other DimOS processes may be live on
    # the same host (a real robot run, another test session).
    blueprint = blueprint.namespace(f"metalpolicy_worker_{os.getpid()}")
    parsed = BlueprintConfigParser(blueprint).parse(
        environ={}, overrides={"g": {"viewer": "none", "n_workers": 2}}
    )
    coordinator = ModuleCoordinator.build(blueprint, parsed)
    try:
        control = coordinator.get_instance(ControlCoordinator)
        connection = coordinator.get_instance(MockJointConnection)
        assert control is not None
        assert connection is not None
        # Nothing in this composition can construct the CAN-backed connection.
        assert DualMetalConnection not in {a.module for a in blueprint.blueprints}
        workers = python_workers(coordinator).workers
        assert workers, "expected real worker processes"

        arm_names = [f"arm/{joint}" for joint in ALL_ARM_JOINTS]
        jaw_names = {side: f"arm/{FINGER_JOINTS[side][0]}" for side in ("left", "right")}
        initial = control.get_joint_positions()
        assert set(arm_names) <= set(initial)
        np.testing.assert_allclose(
            np.asarray([initial[name] for name in arm_names]),
            np.asarray(HOME),
            atol=1e-9,
        )
        for name in jaw_names.values():
            assert initial[name] == pytest.approx(0.0, abs=1e-9)

        # The goal is the ship-time FK of the measured pose, offset in x.
        policy_manifest = PolicyManifest.load(manifest)
        goal = MetalStationKinematics(policy_manifest.station).pose24(
            np.asarray([initial[name] for name in arm_names], dtype=np.float32)
        )
        goal[0] += 0.05
        goal[3] -= 0.05

        ticks_before = control.get_tick_count()
        assert control.task_invoke("metal_arm_policy", "set_goal_pose24", {"pose24": goal.tolist()})
        assert control.task_invoke("metal_arm_policy", "engage", {})

        def moved() -> bool:
            now = control.get_joint_positions()
            return max(abs(now[name] - initial[name]) for name in arm_names) > 1e-3

        wait_until(moved, timeout=15, message="real workers never moved the arms")
        assert control.get_tick_count() > ticks_before
        state = control.task_invoke("metal_arm_policy", "state_snapshot", {})
        assert state["session"] == "active"
        assert state["policy_ticks"] >= 1

        # Jaws belong to their own tasks: the policy never claims them.
        assert control.task_invoke("left_arm_gripper", "set_normalized", {"values": [1.0]})
        wait_until(
            lambda: control.get_joint_positions()[jaw_names["left"]] < -1e-3,
            timeout=10,
            message="left jaw did not move",
        )
        after_jaw = control.get_joint_positions()
        assert after_jaw[jaw_names["right"]] == pytest.approx(0.0, abs=1e-9)

        # Release stops emission; the connection holds the last arm command.
        assert control.task_invoke("metal_arm_policy", "release", {})
        deadline = time.monotonic() + 5.0
        previous = control.get_joint_positions()
        while time.monotonic() < deadline:
            time.sleep(0.1)
            current = control.get_joint_positions()
            if all(abs(current[name] - previous[name]) < 1e-9 for name in arm_names):
                break
            previous = current
        else:
            raise AssertionError("arms did not settle after release")
        held = control.get_joint_positions()
        time.sleep(0.3)
        later = control.get_joint_positions()
        assert max(abs(later[name] - held[name]) for name in arm_names) < 1e-9
        assert control.task_invoke("metal_arm_policy", "state_snapshot", {})["session"] == (
            "disengaged"
        )
    finally:
        coordinator.stop()
    # Every worker process exited; none is left to leak into the next test.
    assert not python_workers(coordinator).workers
