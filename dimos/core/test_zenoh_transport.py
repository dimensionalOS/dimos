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

import threading
from types import SimpleNamespace
from typing import cast

import numpy as np
from pydantic import ValidationError
import pytest

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import (
    _coerce_transport_to_backend,
    _get_transport_for,
    _run_configurators,
)
from dimos.core.global_config import GlobalConfig, global_config
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.core.transport import (
    JpegLcmTransport,
    LCMTransport,
    ZenohTransport,
    pLCMTransport,
    pZenohTransport,
)
from dimos.msgs.sensor_msgs.Image import Image
from dimos.protocol.service.zenohservice import ZenohSessionPool


class TypedMsg:
    """A fake typed message with lcm_encode for testing."""

    @staticmethod
    def lcm_encode() -> bytes:
        return b""


class UntypedMsg:
    """A message without lcm_encode. Triggers pickle transport."""

    pass


class ProducerModule(Module):
    typed_data: Out[TypedMsg]
    untyped_data: Out[UntypedMsg]


class ConsumerModule(Module):
    typed_data: In[TypedMsg]
    untyped_data: In[UntypedMsg]


@pytest.fixture()
def use_lcm(mocker):
    mocker.patch.object(global_config, "transport", "lcm")


@pytest.fixture()
def use_zenoh(mocker):
    mocker.patch.object(global_config, "transport", "zenoh")


@pytest.fixture()
def blueprint():
    return autoconnect(ProducerModule.blueprint(), ConsumerModule.blueprint())


@pytest.fixture()
def lcm_config_mock(mocker):
    """Stub out LCM system configuration; return the lcm_configurators mock for assertions."""
    mock = mocker.patch(
        "dimos.protocol.service.system_configurator.lcm_config.lcm_configurators",
        return_value=[],
    )
    mocker.patch("dimos.protocol.service.system_configurator.base.configure_system")
    return mock


@pytest.fixture()
def session_pool():
    pool = ZenohSessionPool()
    yield pool
    pool.close_all()


@pytest.fixture()
def collector():
    """Collect subscription messages; `event` fires when the first one arrives."""
    received = []
    event = threading.Event()

    def callback(msg):
        received.append(msg)
        event.set()

    return SimpleNamespace(received=received, event=event, callback=callback)


def test_default_transport_is_lcm_on_linux(mocker) -> None:
    mocker.patch("dimos.core.global_config.platform.system", return_value="Linux")

    config = GlobalConfig()
    assert config.transport == "lcm"


def test_default_transport_is_zenoh_on_macos(mocker) -> None:
    mocker.patch("dimos.core.global_config.platform.system", return_value="Darwin")

    config = GlobalConfig()
    assert config.transport == "zenoh"


def test_transport_can_be_set_to_zenoh() -> None:
    config = GlobalConfig()
    config.update(transport="zenoh")
    assert config.transport == "zenoh"


def test_invalid_transport_is_rejected_at_init() -> None:
    with pytest.raises(ValidationError, match="transport"):
        GlobalConfig(transport=cast("object", "invalid"))


def test_invalid_transport_is_rejected_on_update() -> None:
    config = GlobalConfig()
    with pytest.raises(ValidationError, match="transport"):
        config.update(transport=cast("object", "invalid"))


def test_lcm_transport_returned_when_transport_is_lcm(use_lcm, blueprint) -> None:
    transport = _get_transport_for(blueprint, "typed_data", TypedMsg)
    assert isinstance(transport, LCMTransport)


def test_lcm_pickle_transport_returned_for_untyped_when_lcm(use_lcm, blueprint) -> None:
    transport = _get_transport_for(blueprint, "untyped_data", UntypedMsg)
    assert isinstance(transport, pLCMTransport)


def test_zenoh_transport_returned_when_transport_is_zenoh(use_zenoh, blueprint) -> None:
    transport = _get_transport_for(blueprint, "typed_data", TypedMsg)
    assert isinstance(transport, ZenohTransport)


def test_zenoh_pickle_transport_returned_for_untyped_when_zenoh(use_zenoh, blueprint) -> None:
    transport = _get_transport_for(blueprint, "untyped_data", UntypedMsg)
    assert isinstance(transport, pZenohTransport)


def test_zenoh_topic_uses_dimos_prefix(use_zenoh, blueprint) -> None:
    transport = _get_transport_for(blueprint, "untyped_data", UntypedMsg)
    assert isinstance(transport, pZenohTransport)
    assert "dimos/" in transport.topic


def test_lcm_configurators_run_when_transport_is_lcm(use_lcm, blueprint, lcm_config_mock) -> None:
    _run_configurators(blueprint)
    lcm_config_mock.assert_called_once()


def test_lcm_configurators_skipped_when_transport_is_zenoh(
    use_zenoh, blueprint, lcm_config_mock
) -> None:
    _run_configurators(blueprint)
    lcm_config_mock.assert_not_called()


def test_zenoh_transport_broadcast_and_subscribe(retry_until, session_pool, collector) -> None:
    t = ZenohTransport("dimos/test/transport", Image, session_pool=session_pool)
    t.start()
    t.subscribe(collector.callback)

    test_img = Image(np.zeros((2, 2, 3), dtype=np.uint8))
    retry_until(collector.event, lambda: t.broadcast(None, test_img))
    assert isinstance(collector.received[0], Image)
    t.stop()


def test_pzenoh_transport_broadcast_and_subscribe(retry_until, session_pool, collector) -> None:
    t = pZenohTransport("dimos/test/pickle_transport", session_pool=session_pool)
    t.start()
    t.subscribe(collector.callback)

    retry_until(collector.event, lambda: t.broadcast(None, {"key": "value"}))
    assert collector.received[0] == {"key": "value"}
    t.stop()


def test_auto_start_on_broadcast(session_pool) -> None:
    t = pZenohTransport("dimos/test/autostart", session_pool=session_pool)
    # Don't call start(); broadcast should auto-start
    t.broadcast(None, "test")
    assert t._started
    t.stop()


def test_stop_and_restart(session_pool) -> None:
    t = pZenohTransport("dimos/test/restart", session_pool=session_pool)
    t.start()
    assert t._started
    t.stop()
    assert not t._started
    t.start()
    assert t._started
    t.stop()


def test_coerce_lcm_to_zenoh_typed(use_zenoh) -> None:
    t = _coerce_transport_to_backend(LCMTransport("/cmd_vel", Image))
    assert type(t) is ZenohTransport
    assert t.topic.topic == "dimos/cmd_vel"


def test_coerce_pickled_lcm_to_zenoh(use_zenoh) -> None:
    t = _coerce_transport_to_backend(pLCMTransport("/human_input"))
    assert type(t) is pZenohTransport
    assert t.topic == "dimos/human_input"


def test_coerce_zenoh_to_lcm_typed(use_lcm) -> None:
    t = _coerce_transport_to_backend(ZenohTransport("dimos/cmd_vel", Image))
    assert type(t) is LCMTransport
    assert t.topic.topic == "/cmd_vel"


def test_coerce_identity_when_backend_matches(use_lcm) -> None:
    orig = LCMTransport("/cmd_vel", Image)
    assert _coerce_transport_to_backend(orig) is orig


def test_coerce_leaves_deliberate_jpeg_untouched(use_zenoh) -> None:
    jpeg = JpegLcmTransport("/color_image", Image)
    assert _coerce_transport_to_backend(jpeg) is jpeg
