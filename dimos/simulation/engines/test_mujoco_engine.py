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

import time

import pytest

from dimos.simulation.engines import mujoco_engine

_SCENE = """
<mujoco>
  <option timestep="0.002"/>
  <worldbody>
    <light pos="0 0 1"/>
    <geom type="plane" size="1 1 0.1"/>
    <body name="ball" pos="0 0 0.5">
      <freejoint/>
      <geom type="sphere" size="0.05" mass="0.1"/>
    </body>
  </worldbody>
</mujoco>
"""


@pytest.mark.mujoco
def test_viewer_failure_falls_back_to_headless_stepping(tmp_path, monkeypatch):
    """macOS without mjpython: the sim must keep stepping, not die silently."""
    scene = tmp_path / "scene.xml"
    scene.write_text(_SCENE)

    def refuse(*args, **kwargs):
        raise RuntimeError(
            "`launch_passive` requires that the Python script be run under `mjpython`"
        )

    monkeypatch.setattr(mujoco_engine.viewer, "launch_passive", refuse)
    engine = mujoco_engine.MujocoEngine(config_path=scene, headless=False)
    assert engine.connect()
    try:
        deadline = time.monotonic() + 5.0
        while engine.data.time == 0.0 and time.monotonic() < deadline:
            time.sleep(0.05)
        assert engine.data.time > 0.0
        assert engine.headless is True
        position, _ = engine.get_body_pose("ball")
        assert position[2] < 0.5  # gravity acted: physics is really running
        assert engine.get_body_pose("nothing") is None
    finally:
        engine.disconnect()
