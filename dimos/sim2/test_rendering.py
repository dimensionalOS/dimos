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

"""Exercise upstream camera acquisition on the actual simulation owner thread."""

from concurrent.futures import ThreadPoolExecutor
import subprocess
import sys
import threading
import weakref

import numpy as np
import pytest
from robosuite.environments.manipulation.lift import Lift

from dimos.sim2.rendering import make_environment


def _render_lift() -> None:
    assert threading.current_thread() is not threading.main_thread()
    environment = make_environment(
        "Lift",
        robots="Panda",
        has_renderer=False,
        has_offscreen_renderer=True,
        use_camera_obs=True,
        camera_names="agentview",
        camera_heights=32,
        camera_widths=48,
        camera_depths=True,
        hard_reset=True,
        control_freq=20,
    )
    try:
        assert isinstance(environment, Lift)
        first_context = weakref.ref(environment.sim._render_context_offscreen)
        observations = environment.reset()
        assert first_context() is None
        assert observations["agentview_image"].shape == (32, 48, 3)
        assert np.ptp(observations["agentview_image"]) > 0
        observations, _, _, _ = environment.step(np.zeros(environment.action_dim))
        assert observations["agentview_depth"].shape == (32, 48, 1)
        assert np.isfinite(observations["agentview_depth"]).all()
        assert np.ptp(observations["agentview_depth"]) > 0
        last_context = weakref.ref(environment.sim._render_context_offscreen)
    finally:
        environment.close()
    assert environment.sim is None
    assert last_context() is None


def _render_lift_in_thread() -> None:
    with ThreadPoolExecutor(max_workers=1) as worker:
        worker.submit(_render_lift).result()


@pytest.mark.parametrize("worker_process", [False, True])
@pytest.mark.self_hosted
def test_stock_task_renders_on_owner_thread_and_releases_contexts_on_reset_and_close(
    worker_process,
):
    # A process timeout contains driver crashes / Cocoa main-thread deadlocks.
    # Construction, rendering, hard reset and destruction all occur on one worker.
    script = f"""
from dimos.core.coordination.python_worker import get_forkserver_context
from dimos.sim2.test_rendering import _render_lift_in_thread
if not {worker_process!r}:
    _render_lift_in_thread()
else:
    process = get_forkserver_context().Process(target=_render_lift_in_thread, daemon=True)
    process.start()
    try:
        process.join(timeout=45)
        assert process.exitcode == 0, f"rendering child exited with {{process.exitcode}}"
    finally:
        if process.is_alive():
            process.terminate()
            process.join(timeout=5)
"""
    completed = subprocess.run(
        [sys.executable, "-c", script], capture_output=True, text=True, timeout=60
    )
    assert completed.returncode == 0, completed.stdout + completed.stderr
