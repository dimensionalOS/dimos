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

"""Native windows consume snapshots without advancing or owning the simulation."""

from contextlib import nullcontext
import json
from pathlib import Path
import subprocess

import mujoco
import mujoco.viewer
import numpy as np
import pytest

from dimos.core.stream import Transport
from dimos.core.transport import pLCMTransport
from dimos.sim2.messages import FrameMetadata, StateFrame
from dimos.sim2.world import viewer as viewer_module
from dimos.sim2.world.viewer import NativeMujocoViewer


@pytest.fixture
def checkpoint_model(tmp_path):
    def create(name, radius=0.1):
        model = mujoco.MjModel.from_xml_string(
            f'<mujoco><worldbody><body name="robot_pelvis"><freejoint/>'
            f'<geom type="sphere" size="{radius}"/></body></worldbody></mujoco>'
        )
        path = tmp_path / f"{name}.mjb"
        mujoco.mj_saveModel(model, str(path), None)
        description = {
            "model": str(path),
            "physics_dt": 0.01,
            "snapshot_stride": 1,
            "snapshot_slots": 2,
            "model_prefixes": {"robot": "robot_"},
            "root_bodies": {"robot": "pelvis"},
        }

        def frame(sequence, x, *, episode=0):
            data = mujoco.MjData(model)
            data.qpos[0] = x
            data.time = sequence * 0.01
            state = np.empty(mujoco.mj_stateSize(model, mujoco.mjtState.mjSTATE_INTEGRATION))
            mujoco.mj_getState(model, data, state, mujoco.mjtState.mjSTATE_INTEGRATION)
            return StateFrame(
                FrameMetadata(sequence, episode, sequence, data.time),
                {"state": state, "wall_time": np.array([100.0 + data.time])},
                str(path),
            )

        return description, frame

    return create


@pytest.fixture
def window_boundary(mocker):
    handles = []
    displayed = []

    def launch(model, data, **kwargs):
        handle = mocker.MagicMock(spec=mujoco.viewer.Handle)
        handle.__enter__.return_value = handle
        handle.lock.side_effect = nullcontext
        handle.cam = mujoco.MjvCamera()
        handle.opt = mujoco.MjvOption()
        handle.perturb = mujoco.MjvPerturb()
        handle.is_running.side_effect = [True, False]
        handle.sync.side_effect = lambda **kwargs: displayed.append(
            (data.time, float(data.qpos[0]), float(model.geom_size[0, 0]))
        )
        handles.append(handle)
        return handle

    launch_mock = mocker.patch.object(mujoco.viewer, "launch_passive", side_effect=launch)
    mocker.patch.object(viewer_module, "_parent_closed", return_value=False)
    return launch_mock, handles, displayed


def test_window_restores_live_snapshots_and_discards_local_physics_edits(
    checkpoint_model, window_boundary, tmp_path, mocker
):
    description, frame = checkpoint_model("world")
    initial, latest = frame(1, 0.25), frame(2, 0.75)
    transport = mocker.Mock(spec=Transport)
    unsubscribe = mocker.Mock()
    sink = []

    def subscribe(callback):
        sink.append(callback)
        callback(initial)
        return unsubscribe

    transport.subscribe.side_effect = subscribe
    launch, handles, displayed = window_boundary
    ordinary_launch = launch.side_effect

    def launch_with_updates(model, data, **kwargs):
        handle = ordinary_launch(model, data, **kwargs)
        handle.is_running.side_effect = [True, True, False]
        record = handle.sync.side_effect

        def sync(**kwargs):
            record(**kwargs)
            np.testing.assert_array_equal(model.opt.gravity, [0, 0, -9.81])
            model.opt.gravity[:] = 123.0
            data.qpos[0] = 999.0
            handle.perturb.active = 1
            sink[0](latest)

        handle.sync.side_effect = sync
        return handle

    launch.side_effect = launch_with_updates
    step = mocker.spy(mujoco, "mj_step")

    viewer_module._view(description, transport, tmp_path)

    assert displayed == [(0.01, 0.25, 0.1), (0.02, 0.75, 0.1)]
    assert handles[0].cam.trackbodyid == 1
    step.assert_not_called()
    transport.publish.assert_not_called()
    unsubscribe.assert_called_once_with()
    transport.stop.assert_called_once_with()


def test_new_snapshot_model_reopens_window_with_reset_state(
    checkpoint_model, window_boundary, tmp_path, mocker
):
    description, before = checkpoint_model("before", radius=0.1)
    _, after = checkpoint_model("after", radius=0.2)
    transport = mocker.Mock(spec=Transport)
    unsubscribe = mocker.Mock()
    sink = []

    def subscribe(callback):
        sink.append(callback)
        callback(before(1, 0.25))
        return unsubscribe

    transport.subscribe.side_effect = subscribe
    launch, handles, displayed = window_boundary
    ordinary_launch = launch.side_effect

    def launch_with_reset(model, data, **kwargs):
        handle = ordinary_launch(model, data, **kwargs)
        record = handle.sync.side_effect
        handle.is_running.side_effect = [True, True, False]

        def sync(**kwargs):
            record(**kwargs)
            sink[0](after(2, 1.5, episode=1))

        handle.sync.side_effect = sync
        return handle

    launch.side_effect = launch_with_reset

    viewer_module._view(description, transport, tmp_path)

    assert launch.call_count == 2
    assert displayed == [(0.01, 0.25, 0.1), (0.02, 1.5, 0.2), (0.02, 1.5, 0.2)]
    assert json.loads((tmp_path / "ready.json").read_text()) == {
        "initial_sequence": 2,
        "initial_episode": 1,
    }
    for handle in handles:
        handle.__exit__.assert_called_once_with(None, None, None)
    unsubscribe.assert_called_once_with()
    transport.stop.assert_called_once_with()


@pytest.fixture
def launcher(mocker, monkeypatch, tmp_path):
    monkeypatch.setattr(viewer_module.sys, "platform", "linux")
    monkeypatch.setattr(viewer_module.tempfile, "tempdir", str(tmp_path))
    process = mocker.Mock(spec=subprocess.Popen)
    process.stdin = mocker.Mock()
    process.poll.return_value = None
    process.returncode = 0
    popen = mocker.patch.object(viewer_module.subprocess, "Popen", return_value=process)
    transport = pLCMTransport("test/viewer/snapshots")
    viewer = NativeMujocoViewer({"model": "unused-by-parent.mjb"}, transport)
    try:
        yield viewer, process, popen
    finally:
        viewer.stop()
        transport.stop()


def test_startup_timeout_closes_child_pipe_reaps_process_and_removes_bootstrap(
    launcher, tmp_path, mocker
):
    viewer, process, popen = launcher
    mocker.patch.object(viewer_module.time, "monotonic", side_effect=[0.0, 1.0])

    with pytest.raises(TimeoutError, match="did not receive a snapshot"):
        viewer.start(timeout=0.5)

    process.stdin.close.assert_called_once_with()
    process.wait.assert_called_once_with(timeout=2.0)
    assert not Path(popen.call_args.args[0][-1]).exists()
    assert list(tmp_path.iterdir()) == []
    assert viewer.status()["running"] is False


def test_child_startup_error_is_reported_and_cleaned(launcher, tmp_path):
    viewer, process, popen = launcher

    def failed_process(command, **kwargs):
        (Path(command[-1]).parent / "error.txt").write_text("cannot open display")
        return process

    popen.side_effect = failed_process
    process.poll.return_value = 1
    process.returncode = 1

    with pytest.raises(RuntimeError, match="cannot open display"):
        viewer.start()

    assert viewer.status()["exitcode"] == 1
    assert "cannot open display" in viewer.status()["error"]
    assert list(tmp_path.iterdir()) == []
    process.stdin.close.assert_called_once_with()
    process.wait.assert_called_once_with(timeout=2.0)
