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

from collections.abc import Iterator
from types import SimpleNamespace
from typing import Any

import mujoco
import numpy as np
from numpy.typing import NDArray
import pytest
from pytest_mock import MockerFixture
from structlog.testing import capture_logs

from dimos.simulation.mujoco import mujoco_process, policy
from dimos.simulation.mujoco.config import MujocoConfig
from dimos.simulation.mujoco.logging_config import configure_mujoco_logging


@pytest.mark.parametrize(
    ("frame_seconds", "steps_per_frame", "frames", "warnings"),
    [(0.14, 7, 108, 3), (5.0, 7, 3, 3), (0.07, 35, 216, 0), (0.07, 70, 216, 0), (0.14, 7, 10, 0)],
    ids=["slow", "very-slow", "real-time", "fast", "short-sample"],
)
def test_rendering_reports_sustained_slow_simulation_pace(
    mocker: MockerFixture, frame_seconds: float, steps_per_frame: int, frames: int, warnings: int
) -> None:
    model = mujoco.MjModel.from_xml_string(
        """<mujoco><worldbody>
          <camera name="head_camera"/><camera name="lidar_front_camera"/>
          <camera name="lidar_left_camera"/><camera name="lidar_right_camera"/>
          <body><freejoint/><geom type="sphere" size=".1"/></body>
        </worldbody><keyframe><key name="home"/></keyframe></mujoco>"""
    )
    data = mujoco.MjData(model)
    mocker.patch.object(mujoco_process, "load_scene_xml", return_value="unused")
    mocker.patch.object(mujoco_process, "load_model", return_value=(model, data, mocker.Mock()))
    mocker.patch.object(mujoco_process, "PersonPositionController")
    clock = SimpleNamespace(now=100.0)

    def advance(seconds: float) -> None:
        clock.now += seconds

    def render() -> NDArray[np.uint8]:
        advance(frame_seconds)
        return np.zeros((1, 1, 3), dtype=np.uint8)

    mock_time = mocker.patch.object(mujoco_process, "time")
    mock_time.monotonic.side_effect = lambda: clock.now
    mock_time.time.side_effect = lambda: clock.now
    mock_time.sleep.side_effect = advance
    renderer = mocker.patch.object(mujoco_process.mujoco, "Renderer")
    rgb = mocker.Mock()
    rgb.render.side_effect = render
    depth = mocker.Mock()
    depth.render.return_value = np.zeros((1, 1), dtype=np.float32)
    renderer.side_effect = [rgb, depth, depth, depth]
    shm = mocker.Mock()
    shm.read_command.return_value = None

    # Exclude deployment time from the pace calculation.
    def should_run() -> bool:
        advance(60.0)
        return True

    shm.should_run.side_effect = should_run
    shm.should_stop.side_effect = lambda: rgb.render.call_count >= frames

    with capture_logs() as records:
        mujoco_process._run_simulation(
            MujocoConfig(headless=True, shadows="on", steps_per_frame=steps_per_frame),
            shm,
            robot_model="unitree_g1",
        )

    assert len(records) == warnings
    for record in records:
        assert record["log_level"] == "warning"
        assert "below real time" in record["event"]
        assert "Wall-time motion commands may travel less than expected" in record["event"]
        assert record["wall_seconds"] >= 5.0
        assert record["real_time_factor"] == pytest.approx(0.014 / frame_seconds)
        assert record["simulation_seconds"] / record["wall_seconds"] == pytest.approx(
            record["real_time_factor"]
        )


@pytest.mark.parametrize("robot", ["unitree_go2", "unitree_go1", "unitree_g1", None])
def test_simulation_preserves_requested_robot_for_labels(
    robot: str | None, mocker: MockerFixture
) -> None:
    mocker.patch.object(mujoco_process, "load_scene_xml", return_value="<mujoco/>")
    load_model = mocker.patch.object(
        mujoco_process, "load_model", side_effect=RuntimeError("stop before opening viewer")
    )

    with pytest.raises(RuntimeError, match="stop before opening viewer"):
        mujoco_process._run_simulation(MujocoConfig(), mocker.Mock(), robot_model=robot)

    load_model.assert_called_once_with(
        mocker.ANY, robot=robot or "unitree_go1", scene_xml="<mujoco/>"
    )


@pytest.fixture
def simulation(mocker: MockerFixture) -> Iterator[Any]:
    control_callback = mujoco.get_mjcb_control()
    warning_callback = mujoco.get_mju_user_warning()
    mujoco.set_mjcb_control(None)
    configure_mujoco_logging()
    try:
        model = mujoco.MjModel.from_xml_string(
            """<mujoco><worldbody><body>
              <freejoint/><geom type="sphere" size=".1"/><site name="imu_in_pelvis"/>
              <body pos="0 0 -.2"><joint name="leg"/>
                <geom type="sphere" size=".1"/>
              </body>
            </body></worldbody><actuator><position joint="leg"/></actuator>
            <sensor>
              <velocimeter name="local_linvel_pelvis" site="imu_in_pelvis"/>
              <gyro name="gyro_pelvis" site="imu_in_pelvis"/>
            </sensor><keyframe>
              <key name="home" qpos="0 0 .8 1 0 0 0 .2" ctrl=".2"/>
            </keyframe></mujoco>"""
        )
        data = mujoco.MjData(model)
        session = mocker.patch.object(policy.ort, "InferenceSession")
        session.return_value.run.return_value = [np.array([[0.4]], dtype=np.float32)]
        shm = mocker.Mock()
        shm.read_command.return_value = None
        controller = mujoco_process.MockController(shm)
        locomotion = policy.G1OnnxController(
            "unused.onnx", np.array([0.2]), 0.02, 2, 0.5, controller
        )
        mujoco.set_mjcb_control(locomotion.get_control)
        yield model, data, locomotion, controller, shm
    finally:
        mujoco.set_mjcb_control(control_callback)
        mujoco.set_mju_user_warning(warning_callback)


@pytest.mark.parametrize(
    ("field", "warning"),
    [("qpos", "BADQPOS"), ("qvel", "BADQVEL"), ("qfrc_applied", "BADQACC"), ("ctrl", "BADCTRL")],
)
@pytest.mark.parametrize("invalid", [float("nan"), float("inf"), 1e12])
def test_invalid_physics_restores_start_pose_policy_and_reports_reset(
    simulation: Any, field: str, warning: str, invalid: float
) -> None:
    model, data, locomotion, controller, shm = simulation
    start_pos = (-2.0, 3.0, 0.8)
    mujoco_process._reset_simulation(model, data, locomotion, controller, start_pos)
    initial_obs = locomotion.get_obs(model, data).copy()
    initial_ctrl = data.ctrl.copy()
    # Exercise the policy history and command cache before corrupting physics.
    shm.read_command.return_value = (np.array([1.0, 0.0, 0.0]), np.zeros(3))
    for _ in range(3):
        mujoco.mj_step(model, data)
    assert not np.array_equal(locomotion.get_obs(model, data), initial_obs)
    getattr(data, field)[0] = invalid
    previous_time = data.time

    with capture_logs() as records:
        recovered = mujoco_process._step_simulation(model, data, locomotion, controller, start_pos)

    assert recovered
    assert records[-1] == {
        "event": "MuJoCo simulation reset after invalid physics state",
        "simulation_time": previous_time,
        "warnings": [f"mjWARN_{warning}"],
        "start_pos": start_pos,
        "odometry_discontinuity": True,
        "log_level": "error",
    }
    shm.read_command.return_value = None
    np.testing.assert_array_equal(data.qpos, [*start_pos, 1, 0, 0, 0, 0.2])
    np.testing.assert_array_equal(data.qvel, np.zeros(model.nv))
    np.testing.assert_array_equal(data.ctrl, initial_ctrl)
    np.testing.assert_array_equal(locomotion.get_obs(model, data), initial_obs)
    np.testing.assert_array_equal(controller.get_command(), np.zeros(3))
    assert data.time == 0
    assert data.warning.number.sum() == 0
    assert np.isfinite(data.qacc).all()
    # Reset warnings do not keep triggering recovery on subsequent valid steps.
    with capture_logs() as records:
        for _ in range(10):
            assert not mujoco_process._step_simulation(
                model, data, locomotion, controller, start_pos
            )
    assert records == []
    assert data.time == pytest.approx(10 * model.opt.timestep)
