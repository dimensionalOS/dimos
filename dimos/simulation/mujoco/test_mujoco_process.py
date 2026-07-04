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

from dimos.simulation.mujoco.mujoco_process import _mujoco_headless_enabled


def test_mujoco_headless_enabled_from_env(monkeypatch):
    monkeypatch.delenv("DIMOS_MUJOCO_HEADLESS", raising=False)
    assert _mujoco_headless_enabled() is False

    monkeypatch.setenv("DIMOS_MUJOCO_HEADLESS", "1")
    assert _mujoco_headless_enabled() is True

    monkeypatch.setenv("DIMOS_MUJOCO_HEADLESS", "true")
    assert _mujoco_headless_enabled() is True

    monkeypatch.setenv("DIMOS_MUJOCO_HEADLESS", "0")
    assert _mujoco_headless_enabled() is False
