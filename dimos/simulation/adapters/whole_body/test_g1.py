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

from pathlib import Path
import threading
import time
import uuid

from dimos.hardware.whole_body.spec import WholeBodyAdapter
import dimos.simulation.adapters.whole_body.g1 as g1_mod
from dimos.simulation.adapters.whole_body.g1 import SimMujocoG1WholeBodyAdapter
from dimos.simulation.engines import mujoco_shm
from dimos.simulation.engines.mujoco_shm import ManipShmWriter


def test_sim_g1_adapter_satisfies_whole_body_protocol() -> None:
    adapter = SimMujocoG1WholeBodyAdapter(address=Path("unused.xml"))

    assert isinstance(adapter, WholeBodyAdapter)
    assert adapter.get_limits() is None


def test_sim_g1_adapter_rejects_stale_state(monkeypatch) -> None:
    key = uuid.uuid4().hex[:10]
    monkeypatch.setattr(g1_mod, "shm_key_from_path", lambda _: key)
    monkeypatch.setattr(mujoco_shm, "STATE_STALE_TIMEOUT_S", 0.05)
    writer = ManipShmWriter(key)
    writer.signal_ready(num_joints=29, arm_joints=29)
    stop = threading.Event()

    def publish() -> None:
        while not stop.wait(0.01):
            writer._mark_joint_state()

    heartbeat = threading.Thread(target=publish, daemon=True)
    heartbeat.start()
    adapter = SimMujocoG1WholeBodyAdapter(address=Path("unused.xml"))
    assert adapter.connect() is True
    stop.set()
    heartbeat.join()
    time.sleep(0.06)
    try:
        assert adapter.is_connected() is False
        assert adapter.write_motor_commands([]) is False
    finally:
        adapter.disconnect()
        writer.cleanup()
