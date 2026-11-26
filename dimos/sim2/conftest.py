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

import sys

import pytest


@pytest.fixture(scope="session")
def renders() -> None:
    """Skip a test that renders where no GL backend renders, as GitHub-hosted runners and a headless Mac."""
    mujoco = pytest.importorskip("mujoco")
    gl_errors: tuple[type[Exception], ...] = (mujoco.FatalError,)
    if sys.platform == "darwin":
        from mujoco.cgl.cgl import CGLError

        gl_errors = (*gl_errors, CGLError)
    try:
        mujoco.Renderer(mujoco.MjModel.from_xml_string("<mujoco/>"), 8, 8).close()
    except gl_errors as error:
        pytest.skip(f"no GL backend renders here: {error}")
