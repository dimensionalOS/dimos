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

"""The committed layer is the robot: what both ends read off it, with nothing else on disk.

These robots used to be described by an MJCF converted at import time, and what was worth
checking then was that the converter's URDF matched that MJCF (see
`dimos/sim2/utils/test_mjcf_to_urdf.py`). The description is now one committed `.usda` and these
are the properties that replace it: the layer loads, it carries the frames the module advertises
mounts at, and every file it names is there to be served.

Reading the layer rather than running the sim, so a broken vendoring is caught in a second
instead of as a robot missing from a scene thirteen minutes in.
"""

from __future__ import annotations

from pathlib import Path
import subprocess
import sys
import textwrap
from typing import Any

import numpy as np
import pytest

from dimos.robot.assets.model import UsdModel
from dimos.robot.assets.usd import hash_of_file

pytest.importorskip("pxr")
mujoco = pytest.importorskip("mujoco")

# The robots, each with the mounts its module adds, and the root the description names.
_ROBOTS = ("g1", "go2")


def _module(robot: str) -> Any:
    if robot == "g1":
        from dimos.robot.unitree.g1.sim2 import G1Sim

        return G1Sim
    from dimos.robot.unitree.go2.sim2 import Go2Sim

    return Go2Sim


def _mount_frames(module: Any) -> list[str]:
    """Every frame the module says a sensor may be mounted at."""
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
    """A mesh that is a git-lfs pointer reads as a mesh with no faces, much later and elsewhere."""
    module = _module(robot)
    # The committed layer, not the mounted one: the mounts add frames, never files.
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
    """Loads, has the actuators its servos drive, and steps without flying apart."""
    # Importing it is what registers MuJoCo's USD reader; without it the layer goes to the MJCF
    # parser and comes back as "syntax error: line 1, column 0".
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
    """Writing a layer with usd-core must not cost MuJoCo the physics prims when it reads one.

    Two OpenUSD runtimes end up in one process and whichever initialises first owns the plugin
    registry, so with usd-core first MuJoCo's ``mjcPhysics`` schemas go unregistered and every
    ``MjcActuator`` is dropped. Geometry loads either way, so the G1 came up as a complete robot
    with no actuators at all and the only symptom was a servo failing to find its motor.

    In a subprocess, because what is being tested is the order in which a *process* first
    touches each runtime, and this one has already touched both.
    """
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
