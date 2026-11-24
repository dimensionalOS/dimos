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

"""Shipped Unitree assemblies use concrete canonical connections, not adapters."""

import importlib
import math

import pytest

from dimos.control.connection_source import ConnectionSource
from dimos.control.coordinator import ControlCoordinator, ControlCoordinatorConfig, TaskConfig
from dimos.control.task import CoordinatorState, JointStateSnapshot
from dimos.control.tasks.registry import control_task_registry
from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.nav_msgs.Path import Path
from dimos.robot.unitree.go2.connection import GO2_BASE_JOINTS


@pytest.mark.parametrize("name", ["coordinator", "rpp_controller", "holonomic_controller"])
def test_go2_shipped_connections_and_tasks(name):
    module = importlib.import_module(f"dimos.robot.unitree.go2.blueprints.basic.unitree_go2_{name}")
    blueprint = getattr(module, f"unitree_go2_{name}").solidify()
    # A documentation-range IP: parsing only validates config, it never connects.
    parsed = BlueprintConfigParser(blueprint).parse(
        [], environ={}, global_overrides={"robot_ip": "192.0.2.1"}
    )
    atom = next(a for a in blueprint.blueprints if issubclass(a.module, ControlCoordinator))
    config = ControlCoordinatorConfig(**parsed.module_kwargs(atom.name))
    assert not config.hardware
    (declaration,) = config.connections
    source = ConnectionSource(declaration, ["velocity_command"], ["joint_state"], 0.2)
    assert source.joint_names == list(GO2_BASE_JOINTS)
    pairs = {(old, new) for (_, old), new in blueprint.remapping_map.items()}
    assert ("velocity_command", "go2_velocity_command") in pairs
    assert ("go2_joint_state", "joint_state") in pairs
    for cfg in config.tasks:
        assert "yaw_joint" not in cfg.params
        task = control_task_registry.create(cfg.type, cfg, connections={"go2": source})
        assert task.claim().joints == frozenset(GO2_BASE_JOINTS)


@pytest.mark.parametrize("simulation", ["", "mujoco"])
def test_g1_shipped_groot_backends(simulation, monkeypatch):
    from dimos.core.global_config import global_config
    from dimos.robot.unitree.g1.effectors.low_level.groot_wholebody import G1WholeBodyConnection
    from dimos.simulation.engines.mujoco_g1_connection import MujocoG1Connection
    from dimos.utils.data import LfsPath

    def no_download(self):
        raise AssertionError("blueprint configuration must not load policy or mesh artifacts")

    monkeypatch.setattr(LfsPath, "_ensure_downloaded", no_download)
    with monkeypatch.context() as scoped:
        scoped.setattr(global_config, "simulation", simulation)
        for name in ["groot_wbc"]:
            module = importlib.import_module(
                f"dimos.robot.unitree.g1.blueprints.primitive.unitree_g1_{name}"
            )
            module = importlib.reload(module)
            bp = getattr(module, f"unitree_g1_{name}").solidify()
            parsed = BlueprintConfigParser(bp).parse(
                [], environ={}, global_overrides={"simulation": simulation}
            )
            atom = next(a for a in bp.blueprints if issubclass(a.module, ControlCoordinator))
            config = ControlCoordinatorConfig(**parsed.module_kwargs(atom.name))
            assert not config.hardware
            source = ConnectionSource(
                config.connections[0], ["position_command"], ["joint_state", "imu"], 0.2
            )
            assert len(source.joint_names) == 29
            # Sim and hardware present the same connection; only the module behind it differs.
            connection = MujocoG1Connection if simulation else G1WholeBodyConnection
            assert any(a.module is connection for a in bp.blueprints)
            assert config.tasks[0].params["hardware_id"] == "g1"
            assert config.tasks[0].params["auto_dry_run"] is (not simulation)
    # Leave import-time backend selection consistent with the restored globals.
    importlib.reload(module)


@pytest.mark.parametrize("kind", ["rpp_path_follower", "holonomic_pose_follower"])
def test_calibrated_followers_use_independent_pose_and_emit_body_rates(kind):
    from dimos.msgs.geometry_msgs.Pose import Pose
    from dimos.msgs.nav_msgs.Odometry import Odometry

    names = list(GO2_BASE_JOINTS)
    task = control_task_registry.create(
        kind,
        TaskConfig(
            name="follow",
            type=kind,
            joint_names=names,
        ),
    )
    q = Quaternion(0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))
    path = Path(poses=[PoseStamped(position=[0.0, y, 0.0], orientation=q) for y in [0.0, 1.0, 2.0]])
    task.on_path(path, 1.0)
    for t in [1.0, 1.1, 1.2]:
        task.on_odometry(Odometry(ts=t, pose=Pose(orientation=q)), t)
        state = CoordinatorState(joints=JointStateSnapshot(), t_now=t, dt=0.1)
        command = task.compute(state)
        assert command is not None
        vx, vy, wz = command.velocities
        assert vx >= 0.0  # First holonomic tick has dt=0.
        assert abs(vy) < 1e-6
    assert vx > 0.0  # world +y at yaw pi/2 is body forward, not lateral.
    assert "odometry" in {b.stream for b in control_task_registry.bindings_for(kind).consumes}
