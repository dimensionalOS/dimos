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


from __future__ import annotations

from pathlib import Path
import subprocess
import sys
import textwrap
from typing import Any

import numpy as np
import pytest

from dimos.robot.assets.model import UsdModel
from dimos.robot.assets.usd_assets import hash_of_file

pytest.importorskip("pxr")
mujoco = pytest.importorskip("mujoco")

_ROBOTS = ("g1", "go2")


def _module(robot: str) -> Any:
    if robot == "g1":
        from dimos.robot.unitree.g1.sim2 import G1Sim

        return G1Sim
    from dimos.robot.unitree.go2.sim2 import Go2Sim

    return Go2Sim


def _mount_frames(module: Any) -> list[str]:
    return [
        getattr(module, name)
        for name in ("lidar_mount_frame", "l1_mount_frame", "camera_mount_frame")
        if hasattr(module, name)
    ]


@pytest.mark.parametrize("robot", _ROBOTS)
def test_the_layer_names_every_frame_a_sensor_mounts_at(robot: str) -> None:
    module = _module(robot)
    frames = module.usd.frame_names()

    mounts = _mount_frames(module)
    assert mounts, f"{robot} advertises no mount frames, so this test proves nothing"
    assert set(mounts) <= frames, f"{robot}: {sorted(set(mounts) - frames)} is not in its layer"


@pytest.mark.parametrize("robot", _ROBOTS)
def test_every_file_the_layer_names_is_beside_it_and_hashes(robot: str) -> None:
    module = _module(robot)
    source = Path(module.usd.source_path)
    layer = UsdModel.from_file(source)

    named = layer.referenced_files()
    assert named, f"{robot} names no files at all, which a mesh-built robot should not"
    missing = sorted(name for name in named if not (source.parent / name).is_file())
    assert not missing, f"{robot}: {missing}"
    unhashable = sorted(name for name in named if not hash_of_file(str(source.parent / name)))
    assert not unhashable, f"{robot}: {unhashable} (a git-lfs pointer, most likely)"


@pytest.mark.parametrize("robot", _ROBOTS)
def test_mujoco_simulates_the_robot_off_the_layer_alone(robot: str) -> None:
    import mujoco._usd

    from dimos.sim2.usd_layer import usd_layer

    module = _module(robot)
    model = mujoco.MjSpec.from_file(str(usd_layer(module.usd))).compile()
    data = mujoco.MjData(model)
    for _ in range(100):
        mujoco.mj_step(model, data)

    assert model.nu > 0, f"{robot} has no actuators, so no servo of its can be driven"
    assert model.nmesh > 0, f"{robot} drew nothing: its meshes did not compose"
    assert bool(np.isfinite(data.qpos).all()), f"{robot} left the real numbers within 100 steps"


def test_mujoco_keeps_its_actuators_when_usd_core_went_first() -> None:
    source = textwrap.dedent("""
        from dimos.robot.unitree.g1.sim2 import G1Sim
        from dimos.sim2.usd_layer import usd_layer
        # usd-core first: composing the mounted layer is what a blueprint does before the
        # world is built.
        path = usd_layer(G1Sim.usd)
        import mujoco
        print(len(list(mujoco.MjSpec.from_file(str(path)).actuators)))
    """)

    finished = subprocess.run(
        [sys.executable, "-c", source], capture_output=True, text=True, timeout=600
    )

    assert finished.returncode == 0, finished.stderr[-2000:]
    assert int(finished.stdout.strip().splitlines()[-1]) == 29, "the G1 lost its actuators"


def _mujoco_frames(model: Any, data: Any) -> dict[str, tuple[Any, Any]]:
    """Every body's world pose, keyed by the frame name its prim path ends in."""
    frames = {}
    for body in range(model.nbody):
        path = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body)
        if path:
            frames[path.rsplit("/", 1)[-1]] = (data.xpos[body], data.xmat[body].reshape(3, 3))
    return frames


@pytest.mark.parametrize("robot", _ROBOTS)
def test_the_physics_and_the_tf_chain_place_a_frame_the_same(robot: str) -> None:
    """One layer, two readers, and they have to agree.

    MuJoCo builds the world from the prims' own transforms; tf is rebuilt by ``usd_to_urdf``
    from the joints -- their local frames and their axis. A field one reads and the other
    ignores diverges in silence, which is how a lidar ended up pointing down in physics and up
    on tf, and how every joint came to turn about z. Posing both at the same angles and
    comparing is the check that catches it.
    """
    import mujoco._usd

    from dimos.sim2.usd_layer import usd_layer

    module = _module(robot)
    model = mujoco.MjSpec.from_file(str(usd_layer(module.usd))).compile()
    data = mujoco.MjData(model)
    angles = {}
    for joint in range(model.njnt):
        if int(model.jnt_type[joint]) != mujoco.mjtJoint.mjJNT_HINGE:
            continue
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint)
        angle = 0.1 + 0.01 * joint  # distinct per joint, so a swapped pair cannot pass
        data.qpos[model.jnt_qposadr[joint]] = angle
        angles[(name or "").rsplit("/", 1)[-1]] = angle
    mujoco.mj_forward(model, data)
    physics = _mujoco_frames(model, data)

    placed: dict[str, Any] = {module.usd.root_link: np.eye(4)}
    joints = list(module.usd.joints)
    while True:  # the joints come in no particular order, so place what can be placed
        grew = False
        for joint in joints:
            if joint.parent_link in placed and joint.child_link not in placed:
                placed[joint.child_link] = placed[joint.parent_link] @ joint.parent_from_child(
                    angles.get(joint.name, 0.0)
                )
                grew = True
        if not grew:
            break
    compared = 0
    for frame, pose in placed.items():
        if frame not in physics:
            continue
        position, rotation = physics[frame]
        compared += 1
        assert np.allclose(pose[:3, 3], position - physics[module.usd.root_link][0], atol=2e-3), (
            f"{robot}: {frame} sits somewhere else in physics than on tf"
        )
        assert np.allclose(pose[:3, :3], rotation, atol=2e-3), (
            f"{robot}: {frame} faces somewhere else in physics than on tf"
        )
    assert compared > 10, f"{robot}: only {compared} frames compared, so this proves little"
