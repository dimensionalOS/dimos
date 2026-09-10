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

"""Physics scheduling must remain independent of camera/viewer frame time."""

import mujoco
import numpy as np
import pytest

from dimos.simulation.engines.mujoco_engine import (
    CameraConfig,
    MujocoEngine,
    _CameraRendererState,
    _physics_steps_due,
)

pytestmark = pytest.mark.mujoco


def test_render_delay_is_recovered_with_fixed_physics_steps():
    first_count, deadline = _physics_steps_due(0.0, 0.0, 0.002)
    catchup_count, deadline = _physics_steps_due(0.05, deadline, 0.002)

    assert first_count + catchup_count == 26
    assert deadline == pytest.approx(0.052)


def test_early_wakeup_does_not_advance_physics():
    assert _physics_steps_due(0.001, 0.002, 0.002) == (0, 0.002)


def test_long_pause_caps_recovery_work_and_restarts_the_clock():
    steps, deadline = _physics_steps_due(10.0, 0.0, 0.002)

    assert steps == 64
    assert deadline == pytest.approx(10.002)


@pytest.fixture
def camera_engine(tmp_path):
    path = tmp_path / "camera.xml"
    path.write_text("""<mujoco><worldbody><camera name="test" pos="0 0 2"/>
    <body name="arm"><joint name="joint"/><geom type="sphere" size=".1"/></body>
    </worldbody><actuator><position name="joint" joint="joint"/></actuator></mujoco>""")
    engine = MujocoEngine(config_path=path, headless=True)
    try:
        yield engine
    finally:
        engine.disconnect()


def test_camera_rendering_can_pause_and_resume_without_stopping_physics(camera_engine, mocker):
    renderer = mocker.Mock(spec=mujoco.Renderer)
    renderer.render.return_value = np.zeros((8, 8, 3), dtype=np.uint8)
    state = _CameraRendererState(
        CameraConfig(name="test", width=8, height=8), 0, renderer, None, None, 0.05
    )
    camera_engine._render_cameras(1.0, {"test": state})
    camera_engine.set_camera_streaming_enabled(False)
    camera_engine._render_cameras(2.0, {"test": state})
    renderer.render.assert_called_once_with()
    before = camera_engine.data.time
    mujoco.mj_step(camera_engine.model, camera_engine.data)
    assert camera_engine.data.time > before
    camera_engine.set_camera_streaming_enabled(True)
    camera_engine._render_cameras(3.0, {"test": state})
    assert renderer.render.call_count == 2
    frame = camera_engine.read_camera("test")
    assert frame is not None and frame.timestamp == 3.0


def test_position_setpoint_jump_is_limited_by_physics_time(tmp_path):
    path = tmp_path / "motors.xml"
    path.write_text("""<mujoco><option timestep=".002"/><worldbody>
    <body><joint name="base" type="slide"/><geom type="sphere" size=".1"/></body>
    <body><joint name="arm"/><geom type="sphere" size=".1"/></body>
    </worldbody><actuator><position name="base" joint="base"/>
    <position name="arm" joint="arm"/></actuator></mujoco>""")
    engine = MujocoEngine(
        config_path=path, headless=True, position_target_velocity_limits={"base": 0.1}
    )
    try:
        engine.set_position_target(engine.joint_names.index("base"), 5.0)
        engine.set_position_target(engine.joint_names.index("arm"), 1.0)
        for _ in range(50):
            engine._apply_control()
            mujoco.mj_step(engine.model, engine.data)
        assert engine.data.ctrl[engine.model.actuator("base").id] == pytest.approx(0.01)
        assert engine.data.ctrl[engine.model.actuator("arm").id] == 1.0
        engine.set_position_target(engine.joint_names.index("base"), -5.0)
        engine._apply_control()
        assert engine.data.ctrl[engine.model.actuator("base").id] == pytest.approx(0.0098)
    finally:
        engine.disconnect()


def test_limited_position_servo_starts_and_resets_at_configured_pose(tmp_path):
    path = tmp_path / "servo.xml"
    path.write_text("""<mujoco><worldbody><body><joint name="base" type="slide"/>
    <geom type="sphere" size=".1"/></body></worldbody>
    <actuator><position name="base" joint="base"/></actuator></mujoco>""")
    engine = MujocoEngine(
        config_path=path,
        headless=True,
        reset_joint_positions=[1.5],
        position_target_velocity_limits={"base": 0.1},
    )
    try:
        engine._apply_control()
        assert engine.data.ctrl[0] == 1.5
        engine.set_position_target(0, 2.0)
        engine._apply_control()
        engine.reset()
        engine._apply_control()
        assert engine.data.ctrl[0] == 1.5
    finally:
        engine.disconnect()
