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

"""Tests for Zenoh transport scaffold — Phase 1.

Tests the conditional logic added to support Zenoh alongside LCM:
- GlobalConfig transport field
- _get_transport_for() branching
- LCM configurator gating
"""

from __future__ import annotations

import pytest

from dimos.core.blueprints import autoconnect
from dimos.core.global_config import GlobalConfig
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.core.transport import ZENOH_AVAILABLE, LCMTransport, pLCMTransport
from dimos.msgs.sensor_msgs.Image import Image


class TypedMsg:
    """A fake typed message with lcm_encode for testing."""

    @staticmethod
    def lcm_encode() -> bytes:
        return b""


class UntypedMsg:
    """A message without lcm_encode — triggers pickle transport."""

    pass


class ProducerModule(Module):
    typed_out: Out[Image]
    untyped_out: Out[UntypedMsg]


class ConsumerModule(Module):
    typed_out: In[Image]
    untyped_out: In[UntypedMsg]


class TestGlobalConfigTransportField:
    def test_default_transport_is_lcm(self) -> None:
        config = GlobalConfig()
        assert config.transport == "lcm"

    def test_transport_can_be_set_to_zenoh(self) -> None:
        config = GlobalConfig()
        config.update(transport="zenoh")
        assert config.transport == "zenoh"


class TestZenohAvailableGuard:
    def test_zenoh_available_is_bool(self) -> None:
        assert isinstance(ZENOH_AVAILABLE, bool)

    @pytest.mark.skipif(not ZENOH_AVAILABLE, reason="zenoh not installed")
    def test_zenoh_transport_classes_exist_when_available(self) -> None:
        from dimos.core.transport import ZenohTransport, pZenohTransport

        assert ZenohTransport is not None
        assert pZenohTransport is not None


class TestGetTransportForBranching:
    """Test that _get_transport_for() returns the right transport type based on config."""

    def _make_blueprint(self):  # type: ignore[no-untyped-def]
        return autoconnect(ProducerModule.blueprint(), ConsumerModule.blueprint())

    def test_lcm_transport_returned_when_transport_is_lcm(self, mocker) -> None:
        mocker.patch("dimos.core.blueprints.global_config.transport", "lcm")
        bp = self._make_blueprint()
        transport = bp._get_transport_for("typed_out", Image)
        assert isinstance(transport, LCMTransport)

    def test_lcm_pickle_transport_returned_for_untyped_when_lcm(self, mocker) -> None:
        mocker.patch("dimos.core.blueprints.global_config.transport", "lcm")
        bp = self._make_blueprint()
        transport = bp._get_transport_for("untyped_out", UntypedMsg)
        assert isinstance(transport, pLCMTransport)

    @pytest.mark.skipif(not ZENOH_AVAILABLE, reason="zenoh not installed")
    @pytest.mark.xfail(reason="Zenoh pubsub not yet implemented (Phase 2)", raises=NotImplementedError)
    def test_zenoh_transport_returned_when_transport_is_zenoh(self, mocker) -> None:
        from dimos.core.transport import ZenohTransport

        mocker.patch("dimos.core.blueprints.global_config.transport", "zenoh")
        bp = self._make_blueprint()
        transport = bp._get_transport_for("typed_out", Image)
        assert isinstance(transport, ZenohTransport)

    @pytest.mark.skipif(not ZENOH_AVAILABLE, reason="zenoh not installed")
    @pytest.mark.xfail(reason="Zenoh pubsub not yet implemented (Phase 2)", raises=NotImplementedError)
    def test_zenoh_pickle_transport_returned_for_untyped_when_zenoh(self, mocker) -> None:
        from dimos.core.transport import pZenohTransport

        mocker.patch("dimos.core.blueprints.global_config.transport", "zenoh")
        bp = self._make_blueprint()
        transport = bp._get_transport_for("untyped_out", UntypedMsg)
        assert isinstance(transport, pZenohTransport)

    @pytest.mark.skipif(not ZENOH_AVAILABLE, reason="zenoh not installed")
    @pytest.mark.xfail(reason="Zenoh pubsub not yet implemented (Phase 2)", raises=NotImplementedError)
    def test_zenoh_topic_uses_dimos_prefix(self, mocker) -> None:
        from dimos.core.transport import pZenohTransport

        mocker.patch("dimos.core.blueprints.global_config.transport", "zenoh")
        bp = self._make_blueprint()
        transport = bp._get_transport_for("untyped_out", UntypedMsg)
        assert isinstance(transport, pZenohTransport)
        assert "dimos/" in transport.topic

    def test_zenoh_raises_when_not_available(self, mocker) -> None:
        mocker.patch("dimos.core.blueprints.global_config.transport", "zenoh")
        mocker.patch("dimos.core.transport.ZENOH_AVAILABLE", False)

        bp = self._make_blueprint()
        with pytest.raises(RuntimeError, match="eclipse-zenoh is not installed"):
            bp._get_transport_for("typed_out", Image)


class TestConfiguratorGating:
    def test_lcm_configurators_run_when_transport_is_lcm(self, mocker) -> None:
        mocker.patch("dimos.core.blueprints.global_config.transport", "lcm")
        mock_lcm_configs = mocker.patch(
            "dimos.protocol.service.system_configurator.lcm_config.lcm_configurators",
            return_value=[],
        )
        mocker.patch("dimos.protocol.service.system_configurator.base.configure_system")

        bp = autoconnect(ProducerModule.blueprint(), ConsumerModule.blueprint())
        bp._run_configurators()

        mock_lcm_configs.assert_called_once()

    def test_lcm_configurators_skipped_when_transport_is_zenoh(self, mocker) -> None:
        mocker.patch("dimos.core.blueprints.global_config.transport", "zenoh")
        mock_lcm_configs = mocker.patch(
            "dimos.protocol.service.system_configurator.lcm_config.lcm_configurators",
            return_value=[],
        )
        mocker.patch("dimos.protocol.service.system_configurator.base.configure_system")

        bp = autoconnect(ProducerModule.blueprint(), ConsumerModule.blueprint())
        bp._run_configurators()

        mock_lcm_configs.assert_not_called()
