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

"""State and contact invariants at a collision-window handoff."""

from pathlib import Path

import mujoco
import numpy as np
import pytest

from dimos.experimental.gta_sim2.streaming import CollisionCatalog, compile_window, transfer_state


def test_loaded_contact_matches_unstreamed_step(tmp_path: Path) -> None:
    # A moving sphere crosses an exact triangle boundary while unrelated terrain
    # is evicted. Controls, external forces and warm start are nonzero.
    path = tmp_path / "surfaces.npz"
    np.savez(
        path,
        coverage_bounds=np.array([[-50, -50, -5], [50, 50, 5]]),
        triangles=np.array(
            [
                [[-2, -2, 0], [2, -2, 0], [2, 2, 0]],
                [[-2, -2, 0], [2, 2, 0], [-2, 2, 0]],
                [[20, -2, 0], [24, -2, 0], [24, 2, 0]],
            ],
            dtype=np.float64,
        ),
    )
    catalog = CollisionCatalog(path)
    spec = mujoco.MjSpec.from_string("""<mujoco><option timestep="0.002"/>
      <worldbody><body name="robot" pos="0 0 0.19"><freejoint name="root"/>
      <geom name="wheel" type="sphere" size="0.2" mass="2"/></body></worldbody>
      <actuator><motor name="motor" joint="root" gear="1 0 0 0 0 0"/></actuator>
      </mujoco>""")
    old = compile_window(spec, catalog, np.array([10.0, 0.0]), 15)
    new = compile_window(spec, catalog, np.array([0.0, 0.0]), 4)
    old.data.qvel[0] = 0.4
    old.data.ctrl[0] = 0.3
    old.data.qfrc_applied[1] = 0.1
    old.data.xfrc_applied[1, 2] = 0.2
    old.model.actuator_gainprm[0, 0] = 0.8
    mujoco.mj_step(old.model, old.data)
    mujoco.mj_forward(old.model, old.data)
    assert old.data.ncon > 0
    assert np.linalg.norm(old.data.qacc_warmstart) > 0
    assert transfer_state(old.model, old.data, new.model, new.data) == 0
    assert new.data.ncon == old.data.ncon
    np.testing.assert_allclose(new.data.qacc, old.data.qacc, atol=1e-12)
    mujoco.mj_step(old.model, old.data)
    mujoco.mj_step(new.model, new.data)
    np.testing.assert_allclose(new.data.qpos, old.data.qpos, atol=1e-12)
    np.testing.assert_allclose(new.data.qvel, old.data.qvel, atol=1e-12)
    assert len(new.indices) == 2 < len(old.indices)
    missing = compile_window(spec, catalog, np.array([22.0, 0.0]), 3)
    with pytest.raises(ValueError, match="contacting collider"):
        transfer_state(new.model, new.data, missing.model, missing.data)


def test_rejects_different_dynamics() -> None:
    old = mujoco.MjModel.from_xml_string(
        '<mujoco><worldbody><body name="robot"><freejoint/><geom type="sphere" size=".2" mass="2"/></body></worldbody></mujoco>'
    )
    new = mujoco.MjModel.from_xml_string(
        '<mujoco><worldbody><body name="robot"><freejoint/><geom type="sphere" size=".2" mass="3"/></body></worldbody></mujoco>'
    )
    with pytest.raises(ValueError, match="body_mass"):
        transfer_state(old, mujoco.MjData(old), new, mujoco.MjData(new))
