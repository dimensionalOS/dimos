# Copyright 2025-2026 Dimensional Inc.
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

from dimos.core.global_config import GlobalConfig


class TestGlobalConfigSecurityDefaults:
    """Network services must bind to localhost by default (not 0.0.0.0)."""

    def test_listen_host_defaults_to_localhost(self) -> None:
        config = GlobalConfig()
        assert config.listen_host == "127.0.0.1", (
            f"listen_host must default to 127.0.0.1, got {config.listen_host}"
        )


class TestSimulatorBackendResolution:
    """`--simulation <name>` selects the connection backend; `--replay` wins."""

    def test_simulation_selects_named_backend(self) -> None:
        config = GlobalConfig(simulation="dimsim")
        assert config.effective_simulator == "dimsim"
        assert config.unitree_connection_type == "dimsim"

    def test_simulation_mujoco(self) -> None:
        config = GlobalConfig(simulation="mujoco")
        assert config.effective_simulator == "mujoco"
        assert config.unitree_connection_type == "mujoco"

    def test_unset_returns_none_and_webrtc(self) -> None:
        config = GlobalConfig(simulation="")
        assert config.effective_simulator is None
        assert config.unitree_connection_type == "webrtc"

    def test_replay_overrides_simulation(self) -> None:
        config = GlobalConfig(replay=True, simulation="mujoco")
        assert config.effective_simulator == "replay"
        assert config.unitree_connection_type == "replay"
