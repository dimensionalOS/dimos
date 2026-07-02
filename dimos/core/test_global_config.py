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
from dimos.protocol.pubsub.impl.zenohqos import DEFAULT_ZENOH_QOS, ZenohQoS


class TestGlobalConfigSecurityDefaults:
    """Network services must bind to localhost by default (not 0.0.0.0)."""

    def test_listen_host_defaults_to_localhost(self) -> None:
        config = GlobalConfig()
        assert config.listen_host == "127.0.0.1", (
            f"listen_host must default to 127.0.0.1, got {config.listen_host}"
        )


class TestZenohQoSField:
    def test_default_is_rule_table(self) -> None:
        assert GlobalConfig().zenoh_qos == DEFAULT_ZENOH_QOS

    def test_env_json_override(self, monkeypatch) -> None:
        monkeypatch.setenv("DIMOS_ZENOH_QOS", '[{"key": "dimos/x", "reliability": "best_effort"}]')
        config = GlobalConfig()
        assert config.zenoh_qos == (ZenohQoS(key="dimos/x", reliability="best_effort"),)

    def test_update_coerces_dicts(self) -> None:
        # Blueprint overrides arrive as plain dicts via global_config.update().
        config = GlobalConfig()
        config.update(zenoh_qos=({"key": "dimos/x", "congestion_control": "block"},))
        assert config.zenoh_qos == (ZenohQoS(key="dimos/x", congestion_control="block"),)
