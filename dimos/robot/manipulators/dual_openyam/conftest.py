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

"""Opt-in fixtures for the simulated Dual OpenYAM tests."""

import mujoco
import pytest
from pytest_mock import MockerFixture


@pytest.fixture(scope="session")
def mujoco_gl() -> None:
    """Skip a test that renders where MuJoCo cannot open an OpenGL context."""
    try:
        mujoco.Renderer(mujoco.MjModel.from_xml_string("<mujoco/>"), height=8, width=8).close()
    except mujoco.FatalError:
        pytest.skip("no GL backend renders here")


@pytest.fixture
def no_rpc(mocker: MockerFixture) -> None:
    """Modules built in the test serve no RPCs: their streams get in-memory transports."""
    mocker.patch("dimos.protocol.rpc.zenohrpc.ZenohRPC.start")
    mocker.patch("dimos.protocol.rpc.zenohrpc.ZenohRPC.serve_module_rpc")
